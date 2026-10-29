"""Asynchronous, append-only JSONL recording for teleoperation sessions.

Typical use::

    recorder = SessionRecorder(
        "data/recordings/20261023_120000",
        task="Move the cube into the tray",
        session_metadata={"robot_type": "mycobot280_nova5"},
    )
    recorder.start_episode(metadata={"operator_id": "operator-1"})
    recorder.record_event({
        "event_type": "teleop_sample",
        "mycobot_receive_time_ns": 123456789,
        "mycobot_joint_position_deg": [0.0] * 6,
    })  # Nonblocking; False means the bounded queue rejected the event.
    recorder.end_episode("completed")
    recorder.close()

``output_root`` is the unique session directory, not its parent. The caller is
responsible for creating a unique directory name. Events are serialized before
being put on a bounded queue, then written by a background thread to
``episodes/<episode-id>/telemetry.jsonl``. Timestamps in events must be supplied
by the caller; this module never invents sensor acquisition timestamps.
"""

from __future__ import annotations

import dataclasses
import json
import os
import queue
import re
import threading
import time
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Mapping


SCHEMA_VERSION = 1
RAW_FORMAT = "mycobot_leader_raw"
DEFAULT_QUEUE_SIZE = 4096
_STOP = object()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace(
        "+00:00", "Z"
    )


def _json_value(value: Any) -> Any:
    """Convert common metadata values to strict JSON-compatible values."""
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    if isinstance(value, Enum):
        return _json_value(value.value)
    if isinstance(value, Path):
        return str(value)
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return _json_value(dataclasses.asdict(value))
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]

    # Support numpy scalars/arrays without making NumPy a runtime dependency.
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        return _json_value(tolist())
    item = getattr(value, "item", None)
    if callable(item):
        return _json_value(item())
    raise TypeError("Value of type {} is not JSON serializable".format(type(value).__name__))


def _encode_json(value: Any) -> str:
    return json.dumps(
        _json_value(value),
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
    )


def _write_json_atomic(path: Path, value: Mapping[str, Any]) -> None:
    """Replace a JSON metadata file atomically within its directory."""
    temp_path = path.with_name(path.name + ".tmp")
    payload = _encode_json(value) + "\n"
    with temp_path.open("w", encoding="utf-8") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp_path, path)


def _episode_dirname(episode_id: int | str) -> str:
    if isinstance(episode_id, bool):
        raise ValueError("episode_id must be an integer or a non-empty string")
    if isinstance(episode_id, int):
        if episode_id < 0:
            raise ValueError("episode_id must be non-negative")
        return "episode-{:04d}".format(episode_id)
    raw = str(episode_id).strip()
    slug = re.sub(r"[^A-Za-z0-9_-]+", "_", raw).strip("_-")[:80]
    if not slug:
        raise ValueError("episode_id must contain letters or digits")
    return "episode-{}".format(slug)


class SessionRecorder:
    """Write session/episode metadata and timestamped events asynchronously.

    Parameters:
        output_root: Unique session directory, e.g. ``data/recordings/<id>``.
        task: Natural-language description of the demonstration task.
        session_metadata: Optional equipment/operator/configuration metadata.
        queue_size: Maximum pending events. ``record_event`` never waits for
            free queue space; it returns ``False`` and counts the dropped event.

    Lifecycle:
        Call ``start_episode`` before recording. Call ``record_event`` from
        sensor/control callbacks. Call ``end_episode`` after motion stops, then
        ``close`` once the session is finished. An episode still open at close
        is finalized as incomplete.
    """

    def __init__(
        self,
        output_root: str | Path,
        task: str,
        session_metadata: Mapping[str, Any] | None = None,
        queue_size: int = DEFAULT_QUEUE_SIZE,
    ) -> None:
        if not isinstance(task, str) or not task.strip():
            raise ValueError("task must be a non-empty natural-language description")
        if queue_size < 1:
            raise ValueError("queue_size must be at least 1")

        self.session_dir = Path(output_root).expanduser()
        self.episodes_dir = self.session_dir / "episodes"
        self.session_dir.mkdir(parents=True, exist_ok=True)
        if (self.session_dir / "session.json").exists():
            raise FileExistsError(
                "A session already exists at {}; choose a new output directory".format(
                    self.session_dir
                )
            )
        self.episodes_dir.mkdir(parents=True, exist_ok=True)

        self._lock = threading.RLock()
        # Serializes episode boundaries without blocking the writer's counter
        # updates while end_episode waits for its queue barrier.
        self._lifecycle_lock = threading.RLock()
        self._queue: queue.Queue[Any] = queue.Queue(maxsize=queue_size)
        self._closed = False
        self._active_episode: dict[str, Any] | None = None
        self._next_episode_number = 1
        self._event_index = 0
        self._dropped_events = 0
        self._written_events = 0
        self._written_by_episode: dict[Path, int] = {}
        self._writer_error: str | None = None
        self._writer = threading.Thread(
            target=self._writer_loop,
            name="mycobot-raw-writer",
            daemon=True,
        )

        now_utc = _utc_now()
        self._session_metadata: dict[str, Any] = {
            **dict(session_metadata or {}),
            "schema_version": SCHEMA_VERSION,
            "format": RAW_FORMAT,
            "session_id": self.session_dir.name,
            "task": task.strip(),
            "status": "recording",
            "started_at_utc": now_utc,
            "finished_at_utc": None,
            "episodes": [],
            "dropped_events": 0,
            "written_events": 0,
            "writer_error": None,
        }
        self._task = task.strip()
        _write_json_atomic(self.session_dir / "session.json", self._session_metadata)
        self._writer.start()

    @property
    def active_episode_dir(self) -> Path | None:
        with self._lock:
            active = self._active_episode
            return None if active is None else active["directory"]

    @property
    def active_episode_id(self) -> str | None:
        with self._lock:
            active = self._active_episode
            return None if active is None else active["episode_id"]

    @property
    def dropped_events(self) -> int:
        with self._lock:
            return self._dropped_events

    @property
    def written_events(self) -> int:
        with self._lock:
            return self._written_events

    @property
    def writer_error(self) -> str | None:
        with self._lock:
            return self._writer_error

    @property
    def queue_depth(self) -> int:
        return self._queue.qsize()

    def start_episode(
        self,
        episode_id: int | str | None = None,
        metadata: Mapping[str, Any] | None = None,
    ) -> Path:
        """Start and return an episode directory.

        ``episode_id`` defaults to a sequential integer beginning at 1.
        ``metadata`` may include operator, environment, object and other labels.
        """
        with self._lifecycle_lock:
            with self._lock:
                self._ensure_open()
                if self._active_episode is not None:
                    raise RuntimeError("An episode is already active")
                if episode_id is None:
                    episode_id = self._next_episode_number
                episode_dir = self.episodes_dir / _episode_dirname(episode_id)
                if episode_dir.exists():
                    raise FileExistsError("Episode already exists: {}".format(episode_dir))
                episode_dir.mkdir(parents=True)
                start_utc = _utc_now()
                start_monotonic_ns = time.monotonic_ns()
                episode = {
                    **dict(metadata or {}),
                    "schema_version": SCHEMA_VERSION,
                    "episode_id": str(episode_id),
                    "episode_directory": episode_dir.name,
                    "task": self._task,
                    "status": "recording",
                    "reason": None,
                    "started_at_utc": start_utc,
                    "started_monotonic_ns": start_monotonic_ns,
                    "finished_at_utc": None,
                    "finished_monotonic_ns": None,
                    "event_count": 0,
                    "dropped_events": 0,
                }
                _write_json_atomic(episode_dir / "episode.json", episode)
                self._active_episode = {
                    "episode_id": str(episode_id),
                    "directory": episode_dir,
                    "metadata": episode,
                    "accepted_events": 0,
                    "dropped_events": 0,
                }
                self._event_index = 0
                if isinstance(episode_id, int):
                    self._next_episode_number = max(
                        self._next_episode_number, episode_id + 1
                    )
                else:
                    self._next_episode_number += 1
                return episode_dir

    def record_event(self, event: Mapping[str, Any]) -> bool:
        """Queue one JSON-compatible telemetry event without blocking.

        Returns ``False`` when no episode is active, the queue is full, the
        recorder is closed, or the writer has failed. False is also reflected
        in the episode/session drop or error metadata where possible.
        """
        if not isinstance(event, Mapping):
            raise TypeError("event must be a mapping")
        with self._lock:
            if self._closed or self._active_episode is None:
                return False
            active = self._active_episode
            if self._writer_error:
                self._dropped_events += 1
                active["dropped_events"] += 1
                self._sync_counts()
                return False
            event_index = self._event_index
            self._event_index += 1
            row = {
                **dict(event),
                "schema_version": SCHEMA_VERSION,
                "episode_id": active["episode_id"],
                "event_index": event_index,
            }
            try:
                encoded = _encode_json(row) + "\n"
            except (TypeError, ValueError):
                self._dropped_events += 1
                active["dropped_events"] += 1
                self._sync_counts()
                return False
            item = (active["directory"], encoded)
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                self._dropped_events += 1
                active["dropped_events"] += 1
                self._sync_counts()
                return False
            active["accepted_events"] += 1
            return True

    def end_episode(
        self,
        status: str,
        reason: str | None = None,
        metadata: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Drain queued events, close the JSONL file, and finalize episode.json."""
        if not isinstance(status, str) or not status.strip():
            raise ValueError("status must be a non-empty string")
        with self._lifecycle_lock:
            return self._end_episode_locked(status, reason)

    def _end_episode_locked(self, status: str, reason: str | None) -> dict[str, Any]:
        with self._lock:
            self._ensure_open()
            if self._active_episode is None:
                raise RuntimeError("No episode is active")
            active = self._active_episode
            # Gate producers before placing the barrier. The lifecycle lock
            # prevents a new episode until this barrier has completed.
            self._active_episode = None
            barrier = _Barrier(active["directory"])
        self._queue.put(barrier)
        barrier.done.wait()
        with self._lock:
            episode = active["metadata"]
            if metadata:
                episode.update(dict(metadata))
            if barrier.error:
                self._set_writer_error(barrier.error)
                episode_status = "error"
                reason = reason or barrier.error
            elif self._writer_error:
                episode_status = "error"
                reason = reason or self._writer_error
            else:
                episode_status = status.strip()
            episode.update(
                {
                    "status": episode_status,
                    "reason": reason,
                    "finished_at_utc": _utc_now(),
                    "finished_monotonic_ns": time.monotonic_ns(),
                    "event_count": active["accepted_events"],
                    "written_event_count": self._written_by_episode.get(
                        active["directory"], 0
                    ),
                    "dropped_events": active["dropped_events"],
                }
            )
            _write_json_atomic(active["directory"] / "episode.json", episode)
            self._session_metadata["episodes"].append(
                {
                    "episode_id": episode["episode_id"],
                    "directory": episode["episode_directory"],
                    "status": episode_status,
                    "event_count": episode["event_count"],
                    "written_event_count": episode["written_event_count"],
                    "dropped_events": episode["dropped_events"],
                }
            )
            self._sync_counts()
            _write_json_atomic(self.session_dir / "session.json", self._session_metadata)
            return dict(episode)

    def close(self, status: str = "completed") -> dict[str, Any]:
        """Stop the worker and finalize session metadata; safe to call once."""
        if not isinstance(status, str) or not status.strip():
            raise ValueError("status must be a non-empty string")
        with self._lifecycle_lock:
            with self._lock:
                if self._closed:
                    return dict(self._session_metadata)
                active = self._active_episode is not None
            if active:
                # An episode not explicitly ended is never presented as complete.
                self._end_episode_locked(
                    "incomplete", "session_closed_before_episode_end"
                )

            with self._lock:
                self._closed = True
            self._queue.put(_STOP)
            self._writer.join()
            with self._lock:
                final_status = status.strip()
                if self._writer_error:
                    final_status = "error"
                elif active and final_status in ("complete", "completed", "success"):
                    final_status = "incomplete"
                self._session_metadata.update(
                    {
                        "status": final_status,
                        "finished_at_utc": _utc_now(),
                        "dropped_events": self._dropped_events,
                        "written_events": self._written_events,
                        "writer_error": self._writer_error,
                    }
                )
                _write_json_atomic(self.session_dir / "session.json", self._session_metadata)
                return dict(self._session_metadata)

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("Session recorder is closed")

    def _sync_counts(self) -> None:
        self._session_metadata["dropped_events"] = self._dropped_events
        self._session_metadata["written_events"] = self._written_events
        self._session_metadata["writer_error"] = self._writer_error

    def _set_writer_error(self, error: str) -> None:
        with self._lock:
            if self._writer_error is None:
                self._writer_error = error
                self._sync_counts()

    def _writer_loop(self) -> None:
        open_files: dict[Path, Any] = {}
        while True:
            item = self._queue.get()
            try:
                if item is _STOP:
                    for path, stream in list(open_files.items()):
                        error = self._flush_close(path, stream)
                        if error:
                            self._set_writer_error(error)
                    return
                if isinstance(item, _Barrier):
                    stream = open_files.pop(item.episode_dir, None)
                    try:
                        if stream is not None:
                            item.error = self._flush_close(item.episode_dir, stream)
                        if item.error:
                            self._set_writer_error(item.error)
                    finally:
                        item.done.set()
                    continue
                episode_dir, encoded = item
                if self.writer_error:
                    continue
                try:
                    stream = open_files.get(episode_dir)
                    if stream is None:
                        stream = (episode_dir / "telemetry.jsonl").open(
                            "a", encoding="utf-8", buffering=1
                        )
                        open_files[episode_dir] = stream
                    stream.write(encoded)
                    # Flush Python buffers per row. fsync is reserved for episode
                    # boundaries so the control loop is unaffected by disk sync.
                    stream.flush()
                    with self._lock:
                        self._written_events += 1
                        self._written_by_episode[episode_dir] = (
                            self._written_by_episode.get(episode_dir, 0) + 1
                        )
                        self._sync_counts()
                except Exception as exc:
                    self._set_writer_error("{}: {}".format(type(exc).__name__, exc))
                    stream = open_files.pop(episode_dir, None)
                    if stream is not None:
                        try:
                            stream.close()
                        except OSError:
                            pass
            finally:
                self._queue.task_done()

    def _flush_close(self, episode_dir: Path, stream: Any) -> str | None:
        try:
            stream.flush()
            os.fsync(stream.fileno())
            stream.close()
            return None
        except Exception as exc:
            try:
                stream.close()
            except Exception:
                pass
            return "{}: {}".format(type(exc).__name__, exc)


class _Barrier:
    def __init__(self, episode_dir: Path) -> None:
        self.episode_dir = episode_dir
        self.done = threading.Event()
        self.error: str | None = None
