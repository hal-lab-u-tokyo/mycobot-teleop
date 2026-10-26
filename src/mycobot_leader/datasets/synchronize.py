"""Build a camera-clock synchronized, loss-explicit intermediate dataset.

Synchronization uses the camera worker's host receive monotonic clock, which
shares a clock domain with the host request/send timestamps in telemetry. The
camera bag timestamps are retained as source metadata and are never treated as
host times unless the caller separately inspects their timestamp domain.

The intermediate stores frame references (bag path + frame number), not image
pixels. It is deterministic for a given raw session and configuration, and it
does not mutate the raw session.
"""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import asdict, dataclass, field
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any, Iterable

from ._io import DatasetInputError, camera_timestamp_path, read_json, read_jsonl, session_episodes


INTERMEDIATE_FORMAT = "mycobot_leader_synchronized"
INTERMEDIATE_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class SynchronizationConfig:
    """Explicit timing policy for a synchronization run.

    ``action_delay_ns`` means pair the observation at camera time ``t`` with
    the nearest command event at ``t + action_delay_ns``. A positive delay
    therefore associates a later command with each observation.
    """

    reference_camera: str
    state_mode: str = "interpolate"
    state_source: str | None = None
    action_source: str | None = None
    state_slop_ns: int = 40_000_000
    max_state_bracket_ns: int = 150_000_000
    action_slop_ns: int = 40_000_000
    camera_slop_ns: int = 20_000_000
    action_delay_ns: int = 0

    def __post_init__(self) -> None:
        if not self.reference_camera:
            raise ValueError("reference_camera must be a camera name")
        if self.state_mode not in {"interpolate", "nearest"}:
            raise ValueError("state_mode must be 'interpolate' or 'nearest'")
        for name in ("state_slop_ns", "max_state_bracket_ns", "action_slop_ns", "camera_slop_ns"):
            if getattr(self, name) < 0:
                raise ValueError("{} must be non-negative".format(name))


@dataclass
class SynchronizationResult:
    """Synchronized samples and explicit exclusion diagnostics."""

    episode_dir: str
    config: SynchronizationConfig
    samples: list[dict[str, Any]] = field(default_factory=list)
    excluded: list[dict[str, Any]] = field(default_factory=list)
    diagnostics: dict[str, Any] = field(default_factory=dict)

    @property
    def candidate_count(self) -> int:
        return len(self.samples) + len(self.excluded)

    def to_dict(self) -> dict[str, Any]:
        return {
            "episode_dir": self.episode_dir,
            "config": asdict(self.config),
            "candidate_count": self.candidate_count,
            "accepted_count": len(self.samples),
            "excluded_count": len(self.excluded),
            "samples": self.samples,
            "excluded": self.excluded,
            "diagnostics": self.diagnostics,
        }


@dataclass(frozen=True)
class _TimedVector:
    time_ns: int
    vector: tuple[float, ...]
    source: str | None
    event_index: Any


@dataclass(frozen=True)
class _Frame:
    frame_number: int
    host_ns: int
    timestamp_ms: float | None
    timestamp_domain: str | None


def _finite_vector(value: Any, *, where: str) -> tuple[float, ...]:
    if not isinstance(value, list) or not value:
        raise DatasetInputError("{} must be a non-empty numeric array".format(where))
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise DatasetInputError("{} contains a non-numeric value".format(where)) from exc
    if not all(math.isfinite(item) for item in result):
        raise DatasetInputError("{} contains a non-finite value".format(where))
    return result


def _load_vectors(episode_dir: Path, config: SynchronizationConfig) -> tuple[list[_TimedVector], list[_TimedVector], dict[str, Any]]:
    path = episode_dir / "telemetry.jsonl"
    states: list[_TimedVector] = []
    actions: list[_TimedVector] = []
    state_sources: set[str | None] = set()
    action_sources: set[str | None] = set()
    malformed_events = 0
    for line, event in read_jsonl(path):
        kind = event.get("event_type")
        if kind == "robot_state":
            try:
                start, end = int(event["host_request_start_ns"]), int(event["host_response_ns"])
                if end < start:
                    raise ValueError("response precedes request")
                if event.get("source") is None:
                    raise ValueError("source is required")
                source = str(event["source"])
                vector = _finite_vector(event.get("joint_position_deg"), where="{}:{} joint_position_deg".format(path, line))
                states.append(_TimedVector((start + end) // 2, vector, source, event.get("event_index")))
                state_sources.add(source)
            except (KeyError, TypeError, ValueError) as exc:
                malformed_events += 1
                raise DatasetInputError("Invalid robot_state at {}:{}: {}".format(path, line, exc)) from exc
        elif kind == "action":
            try:
                start, end = int(event["host_send_start_ns"]), int(event["host_response_ns"])
                if end < start:
                    raise ValueError("response precedes send")
                source = str(event["source"]) if event.get("source") is not None else None
                vector = _finite_vector(event.get("joint_target_deg"), where="{}:{} joint_target_deg".format(path, line))
                actions.append(_TimedVector((start + end) // 2, vector, source, event.get("event_index")))
                action_sources.add(source)
            except (KeyError, TypeError, ValueError) as exc:
                malformed_events += 1
                raise DatasetInputError("Invalid action at {}:{}: {}".format(path, line, exc)) from exc

    if config.state_source is not None:
        states = [row for row in states if row.source == config.state_source]
    elif len(state_sources) > 1:
        raise DatasetInputError(
            "Multiple robot_state sources {} found; set state_source explicitly".format(sorted(str(v) for v in state_sources))
        )
    if config.action_source is not None:
        actions = [row for row in actions if row.source == config.action_source]
    elif len(action_sources) > 1:
        raise DatasetInputError(
            "Multiple action sources {} found; set action_source explicitly".format(sorted(str(v) for v in action_sources))
        )
    states.sort(key=lambda row: row.time_ns)
    actions.sort(key=lambda row: row.time_ns)
    for label, rows in (("robot_state", states), ("action", actions)):
        if any(right.time_ns == left.time_ns for left, right in zip(rows, rows[1:])):
            raise DatasetInputError("Duplicate {} timestamps make nearest matching ambiguous in {}".format(label, path))
    return states, actions, {
        "robot_state_count": len(states),
        "action_count": len(actions),
        "robot_state_source": config.state_source or (next(iter(state_sources)) if state_sources else None),
        "action_source": config.action_source or (next(iter(action_sources)) if action_sources else None),
        "malformed_events": malformed_events,
        "robot_state_reordered_count": _count_reorders(states),
        "action_reordered_count": _count_reorders(actions),
    }


def _count_reorders(rows: list[_TimedVector]) -> int:
    # The source JSONL event order is retained by event_index. Count clock
    # inversions without changing the deterministic timestamp-sorted matching.
    indexed = [row for row in rows if isinstance(row.event_index, int)]
    indexed.sort(key=lambda row: row.event_index)
    return sum(right.time_ns < left.time_ns for left, right in zip(indexed, indexed[1:]))


def _load_frames(episode_dir: Path, camera_name: str) -> tuple[list[_Frame], Path, Path]:
    timestamp_path = camera_timestamp_path(episode_dir, camera_name)
    bag_path = episode_dir / "cameras" / (camera_name + ".bag")
    if not timestamp_path.is_file():
        raise DatasetInputError("Missing timestamp file for camera {!r}: {}".format(camera_name, timestamp_path))
    if not bag_path.is_file():
        raise DatasetInputError("Missing camera bag for {!r}: {}".format(camera_name, bag_path))
    frames: list[_Frame] = []
    for line, row in read_jsonl(timestamp_path):
        try:
            frame_number = int(row["frame_number"])
            host_ns = int(row["host_receive_monotonic_ns"])
            timestamp = row.get("timestamp_ms")
            timestamp_ms = None if timestamp is None else float(timestamp)
            if timestamp_ms is not None and not math.isfinite(timestamp_ms):
                raise ValueError("timestamp_ms is not finite")
            frames.append(_Frame(frame_number, host_ns, timestamp_ms, str(row.get("timestamp_domain")) if row.get("timestamp_domain") is not None else None))
        except (KeyError, TypeError, ValueError) as exc:
            raise DatasetInputError("Invalid camera timestamp at {}:{}: {}".format(timestamp_path, line, exc)) from exc
    frames.sort(key=lambda frame: frame.host_ns)
    if not frames:
        raise DatasetInputError("Camera timestamp file is empty: {}".format(timestamp_path))
    times = [frame.host_ns for frame in frames]
    if len(times) != len(set(times)):
        raise DatasetInputError("Camera host receive timestamps must be unique: {}".format(timestamp_path))
    return frames, bag_path, timestamp_path


def _nearest(rows: list[Any], times: list[int], target_ns: int, slop_ns: int) -> tuple[Any | None, int | None]:
    index = bisect_left(times, target_ns)
    candidates = []
    if index < len(rows):
        candidates.append(rows[index])
    if index:
        candidates.append(rows[index - 1])
    if not candidates:
        return None, None
    row = min(
        candidates,
        key=lambda item: (
            abs((item.time_ns if hasattr(item, "time_ns") else item.host_ns) - target_ns),
            item.time_ns if hasattr(item, "time_ns") else item.host_ns,
        ),
    )
    row_time = row.time_ns if hasattr(row, "time_ns") else row.host_ns
    delta = row_time - target_ns
    if abs(delta) > slop_ns:
        return None, delta
    return row, delta


def _state_at(states: list[_TimedVector], times: list[int], target_ns: int, config: SynchronizationConfig) -> tuple[tuple[float, ...] | None, dict[str, Any]]:
    if not states:
        return None, {"reason": "no_robot_state_samples"}
    index = bisect_left(times, target_ns)
    if index < len(states) and times[index] == target_ns:
        row = states[index]
        return row.vector, {"mode": "exact", "source_event_index": row.event_index, "delta_ns": 0}
    if config.state_mode == "nearest":
        row, delta = _nearest(states, times, target_ns, config.state_slop_ns)
        if row is None:
            return None, {"reason": "state_outside_slop", "nearest_delta_ns": delta}
        return row.vector, {"mode": "nearest", "source_event_index": row.event_index, "delta_ns": delta}
    if index == 0 or index >= len(states):
        return None, {"reason": "state_outside_sample_coverage"}
    before, after = states[index - 1], states[index]
    gap = after.time_ns - before.time_ns
    if gap <= 0:
        return None, {"reason": "invalid_state_bracket"}
    if gap > config.max_state_bracket_ns:
        return None, {"reason": "state_bracket_too_wide", "bracket_gap_ns": gap}
    if target_ns - before.time_ns > config.state_slop_ns or after.time_ns - target_ns > config.state_slop_ns:
        return None, {
            "reason": "state_outside_slop",
            "before_delta_ns": target_ns - before.time_ns,
            "after_delta_ns": after.time_ns - target_ns,
        }
    if len(before.vector) != len(after.vector):
        return None, {"reason": "state_shape_changed"}
    alpha = (target_ns - before.time_ns) / gap
    vector = tuple(a + alpha * (b - a) for a, b in zip(before.vector, after.vector))
    return vector, {
        "mode": "linear_interpolation",
        "before_event_index": before.event_index,
        "after_event_index": after.event_index,
        "bracket_gap_ns": gap,
    }


def synchronize_episode(
    episode_dir: str | Path,
    config: SynchronizationConfig,
    *,
    session_dir: str | Path | None = None,
) -> SynchronizationResult:
    """Synchronize one episode to each frame from the selected reference camera.

    Missing state/action/camera matches become explicit exclusions. State
    interpolation is allowed only between two observed samples and within both
    the configured bracket width and per-side slop.
    """
    episode_path = Path(episode_dir).expanduser().resolve()
    episode_meta = read_json(episode_path / "episode.json")
    states, actions, diagnostics = _load_vectors(episode_path, config)
    reference_frames, reference_bag, reference_timestamps = _load_frames(episode_path, config.reference_camera)
    other_cameras: dict[str, tuple[list[_Frame], Path, Path]] = {}
    missing_timestamp_cameras: list[str] = []
    camera_dir = episode_path / "cameras"
    for bag_path in sorted(camera_dir.glob("*.bag")):
        name = bag_path.stem
        if name != config.reference_camera:
            try:
                other_cameras[name] = _load_frames(episode_path, name)
            except DatasetInputError as exc:
                diagnostics.setdefault("camera_issues", {})[name] = str(exc)
                missing_timestamp_cameras.append(name)

    state_times = [row.time_ns for row in states]
    action_times = [row.time_ns for row in actions]
    camera_times = {name: [frame.host_ns for frame in bundle[0]] for name, bundle in other_cameras.items()}
    accepted: list[dict[str, Any]] = []
    excluded: list[dict[str, Any]] = []
    source_root = Path(session_dir).expanduser().resolve() if session_dir is not None else episode_path.parent.parent.resolve()

    for frame in reference_frames:
        reasons: list[str] = []
        reasons.extend("camera_{}_timestamps_unavailable".format(name) for name in missing_timestamp_cameras)
        state, state_match = _state_at(states, state_times, frame.host_ns, config)
        if state is None:
            reasons.append(str(state_match["reason"]))
        action, action_delta = _nearest(actions, action_times, frame.host_ns + config.action_delay_ns, config.action_slop_ns)
        if action is None:
            reasons.append("action_outside_slop_or_missing")
        paired_cameras: dict[str, Any] = {
            config.reference_camera: {
                "bag_path": str(reference_bag.relative_to(source_root)),
                "timestamp_path": str(reference_timestamps.relative_to(source_root)),
                "frame_number": frame.frame_number,
                "host_receive_monotonic_ns": frame.host_ns,
                "timestamp_ms": frame.timestamp_ms,
                "timestamp_domain": frame.timestamp_domain,
                "delta_ns": 0,
            }
        }
        for name, (frames, bag_path, timestamp_path) in other_cameras.items():
            paired, delta = _nearest(frames, camera_times[name], frame.host_ns, config.camera_slop_ns)
            if paired is None:
                reasons.append("camera_{}_outside_slop_or_missing".format(name))
            else:
                paired_cameras[name] = {
                    "bag_path": str(bag_path.relative_to(source_root)),
                    "timestamp_path": str(timestamp_path.relative_to(source_root)),
                    "frame_number": paired.frame_number,
                    "host_receive_monotonic_ns": paired.host_ns,
                    "timestamp_ms": paired.timestamp_ms,
                    "timestamp_domain": paired.timestamp_domain,
                    "delta_ns": delta,
                }
        base = {
            "reference_time_monotonic_ns": frame.host_ns,
            "reference_frame_number": frame.frame_number,
            "cameras": paired_cameras,
        }
        if reasons:
            excluded.append({**base, "reasons": reasons, "state_match": state_match, "action_delta_ns": action_delta})
            continue
        assert state is not None and action is not None
        if len(state) != len(action.vector):
            excluded.append({**base, "reasons": ["state_action_shape_mismatch"], "state_shape": len(state), "action_shape": len(action.vector)})
            continue
        accepted.append({
            **base,
            "observation": {
                "joint_position_deg": list(state),
                "state_match": state_match,
                "state_match_delta_ns": state_match.get("delta_ns", 0),
            },
            "action": {
                "joint_target_deg": list(action.vector),
                "source_event_index": action.event_index,
                "source_time_monotonic_ns": action.time_ns,
                "delta_from_delayed_observation_ns": action_delta,
                "action_delay_ns": config.action_delay_ns,
                "match_delta_ns": action_delta,
            },
            "task": episode_meta.get("task"),
        })

    diagnostics.update({
        "reference_camera": config.reference_camera,
        "reference_frame_count": len(reference_frames),
        "accepted_count": len(accepted),
        "excluded_count": len(excluded),
        "exclusion_reasons": _reason_counts(excluded),
        "source_episode_status": episode_meta.get("status"),
    })
    return SynchronizationResult(str(episode_path), config, accepted, excluded, diagnostics)


def _reason_counts(excluded: list[dict[str, Any]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for row in excluded:
        for reason in row.get("reasons", []):
            counts[reason] = counts.get(reason, 0) + 1
    return counts


def synchronize_session(session_dir: str | Path, config: SynchronizationConfig) -> dict[str, Any]:
    """Synchronize every finalized episode and return a serializable result."""
    root = Path(session_dir).expanduser().resolve()
    session = read_json(root / "session.json")
    results = [synchronize_episode(path, config, session_dir=root) for path in session_episodes(root)]
    return {
        "format": INTERMEDIATE_FORMAT,
        "schema_version": INTERMEDIATE_SCHEMA_VERSION,
        "source_session": str(root),
        "session_id": session.get("session_id", root.name),
        "task": session.get("task"),
        "config": asdict(config),
        "episodes": [result.to_dict() for result in results],
        "summary": {
            "episode_count": len(results),
            "candidate_count": sum(result.candidate_count for result in results),
            "accepted_count": sum(len(result.samples) for result in results),
            "excluded_count": sum(len(result.excluded) for result in results),
        },
        "limitations": [
            "This is a synchronized intermediate, not a LeRobot dataset.",
            "Camera fields reference frames in preserved raw bag files; RGB pixels are not decoded or copied.",
        ],
    }


def write_synchronized_intermediate(
    session_dir: str | Path,
    output_dir: str | Path,
    config: SynchronizationConfig,
) -> dict[str, Any]:
    """Write deterministic manifest + per-episode JSONL, preserving raw input.

    Output layout is ``manifest.json`` and ``episodes/<episode>/samples.jsonl``
    plus ``excluded.jsonl``. Every record includes source-relative bag and
    timestamp paths. The output tree must not already exist.
    """
    root = Path(session_dir).expanduser().resolve()
    output = Path(output_dir).expanduser().resolve()
    if output == root or root in output.parents:
        raise ValueError("Derived output must be outside the raw session directory")
    if output.exists():
        raise FileExistsError("Refusing to overwrite derived output: {}".format(output))
    result = synchronize_session(root, config)
    output.mkdir(parents=True)
    episode_root = output / "episodes"
    episode_root.mkdir()
    for episode in result["episodes"]:
        name = Path(episode["episode_dir"]).name
        target = episode_root / name
        target.mkdir()
        _write_jsonl(target / "samples.jsonl", episode["samples"])
        _write_jsonl(target / "excluded.jsonl", episode["excluded"])
        # Keep the compact report beside the sample stream for auditability.
        (target / "diagnostics.json").write_text(
            json.dumps({key: value for key, value in episode.items() if key not in {"samples", "excluded"}}, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    payload_files = _file_inventory(output)
    manifest = {
        **{key: value for key, value in result.items() if key != "episodes"},
        "episodes": [
            {key: value for key, value in episode.items() if key not in {"samples", "excluded"}}
            for episode in result["episodes"]
        ],
        "source_inputs": _file_inventory(root),
        "converter_revision": os.environ.get("MYCOBOT_LEADER_GIT_COMMIT"),
        "derived_payload_sha256": _inventory_digest(payload_files),
        "derived_payload_files": payload_files,
        "output_schema": "episodes/<episode>/samples.jsonl + excluded.jsonl + diagnostics.json",
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest


def _write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    with path.open("x", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _file_inventory(root: Path) -> list[dict[str, Any]]:
    return [
        {
            "path": path.relative_to(root).as_posix(),
            "size_bytes": path.stat().st_size,
            "sha256": _sha256(path),
        }
        for path in sorted(root.rglob("*"))
        if path.is_file()
    ]


def _inventory_digest(files: list[dict[str, Any]]) -> str:
    digest = hashlib.sha256()
    for item in files:
        digest.update(item["path"].encode("utf-8"))
        digest.update(b"\0")
        digest.update(item["sha256"].encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


__all__ = [
    "INTERMEDIATE_FORMAT",
    "INTERMEDIATE_SCHEMA_VERSION",
    "SynchronizationConfig",
    "SynchronizationResult",
    "synchronize_episode",
    "synchronize_session",
    "write_synchronized_intermediate",
]
