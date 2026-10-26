"""Offline quality summaries for immutable raw recording sessions.

The report intentionally uses only the Python standard library. It reads
``session.json``, per-episode metadata/telemetry JSONL and RealSense timestamp
JSONL files; it never changes input files.

Public API::

    report = inspect_quality("data/recordings/<session-id>")
    write_quality_report(report, "quality-report.json")
    report = generate_quality_report("data/recordings/<session-id>", "report.json")
"""

from __future__ import annotations

from collections import Counter, defaultdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
from statistics import mean, median
from typing import Any, Iterable, Mapping


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError("Expected a JSON object in {}".format(path))
    return value


def _read_jsonl(path: Path) -> tuple[list[dict[str, Any]], int]:
    rows: list[dict[str, Any]] = []
    malformed = 0
    if not path.is_file():
        return rows, malformed
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                if isinstance(row, dict):
                    rows.append(row)
                else:
                    malformed += 1
            except (json.JSONDecodeError, UnicodeDecodeError):
                malformed += 1
    return rows, malformed


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _event_time_ns(event: Mapping[str, Any]) -> int | None:
    """Pick a host monotonic instant without pretending it is sensor time."""
    if event.get("event_type") == "action":
        keys = ("host_send_start_ns", "host_response_ns", "host_monotonic_ns")
    elif event.get("event_type") == "robot_state":
        keys = ("host_response_ns", "host_request_start_ns", "host_monotonic_ns")
    else:
        keys = ("host_receive_time_ns", "host_monotonic_ns", "host_response_ns", "host_send_start_ns", "host_request_start_ns")
    for key in keys:
        value = event.get(key)
        if isinstance(value, int) and not isinstance(value, bool):
            return value
    return None


def _timing_summary(events: Iterable[Mapping[str, Any]], gap_factor: float) -> dict[str, Any]:
    # Preserve JSONL order so clock rollbacks and duplicate times remain
    # observable instead of disappearing through a sort.
    timestamps = [
        value for event in events
        if (value := _event_time_ns(event)) is not None
    ]
    gaps = [b - a for a, b in zip(timestamps, timestamps[1:])]
    positive = [gap for gap in gaps if gap > 0]
    typical = median(positive) if positive else None
    gap_limit = typical * gap_factor if typical is not None else None
    long_gaps = [gap for gap in gaps if gap_limit is not None and gap > gap_limit]
    return {
        "sample_count": len(timestamps),
        "timestamp_source": "host_monotonic_ns (request/response/send time; sensor time not substituted)",
        "duplicate_or_non_increasing_count": sum(1 for gap in gaps if gap <= 0),
        "delta_ms": _distribution([gap / 1_000_000 for gap in gaps]),
        "long_gap_threshold_ms": gap_limit / 1_000_000 if gap_limit is not None else None,
        "long_gap_count": len(long_gaps),
        "long_gap_examples_ms": [round(gap / 1_000_000, 3) for gap in long_gaps[:20]],
    }


def _distribution(values: Iterable[float]) -> dict[str, float | int | None]:
    items = [float(value) for value in values if math.isfinite(float(value))]
    if not items:
        return {"count": 0, "min": None, "median": None, "mean": None, "max": None}
    return {
        "count": len(items),
        "min": min(items),
        "median": median(items),
        "mean": mean(items),
        "max": max(items),
    }


def _vector_metrics(
    events: list[dict[str, Any]],
    value_key: str,
    *,
    spike_threshold_deg: float,
    stationary_epsilon_deg: float,
    stationary_min_duration_s: float,
) -> dict[str, Any] | None:
    samples: list[tuple[int, list[float], Mapping[str, Any]]] = []
    for event in events:
        raw = event.get(value_key)
        if not isinstance(raw, list) or not raw:
            continue
        vector = [_finite_number(item) for item in raw]
        if any(item is None for item in vector):
            continue
        timestamp = _event_time_ns(event)
        if timestamp is None:
            continue
        samples.append((timestamp, [float(item) for item in vector], event))
    if not samples:
        return None

    width = min(len(row[1]) for row in samples)
    samples = [(stamp, vector[:width], event) for stamp, vector, event in samples]
    ranges: list[dict[str, float | int]] = []
    for index in range(width):
        values = [vector[index] for _, vector, _ in samples]
        ranges.append({
            "joint": index + 1,
            "min_deg": min(values),
            "max_deg": max(values),
            "range_deg": max(values) - min(values),
            "first_deg": values[0],
            "last_deg": values[-1],
        })

    deltas: list[tuple[int, int, float]] = []
    for (before_ns, before, _), (after_ns, after, _) in zip(samples, samples[1:]):
        largest = max(abs(b - a) for a, b in zip(before, after))
        deltas.append((before_ns, after_ns, largest))
    spikes = [
        {"start_monotonic_ns": start, "end_monotonic_ns": end, "max_joint_delta_deg": delta}
        for start, end, delta in deltas if delta >= spike_threshold_deg
    ]

    # Stationary runs are adjacent intervals where every joint changes by at
    # most epsilon. A run's duration is measured from its first to last sample.
    stationary: list[dict[str, Any]] = []
    run_start: int | None = None
    run_end: int | None = None
    for start, end, delta in deltas:
        if delta <= stationary_epsilon_deg:
            if run_start is None:
                run_start = start
            run_end = end
        else:
            _append_stationary(stationary, run_start, run_end, stationary_min_duration_s)
            run_start = run_end = None
    _append_stationary(stationary, run_start, run_end, stationary_min_duration_s)

    return {
        "value_key": value_key,
        "units": "degrees",
        "sample_count": len(samples),
        "joint_count": width,
        "joint_ranges": ranges,
        "max_per_sample_delta_deg": _distribution(delta for _, _, delta in deltas),
        "spike_threshold_deg_per_sample": spike_threshold_deg,
        "spike_count": len(spikes),
        "spike_examples": spikes[:20],
        "stationary_epsilon_deg_per_sample": stationary_epsilon_deg,
        "stationary_min_duration_s": stationary_min_duration_s,
        "stationary_interval_count": len(stationary),
        "stationary_intervals": stationary[:20],
    }


def _append_stationary(output: list[dict[str, Any]], start: int | None, end: int | None, min_duration_s: float) -> None:
    if start is None or end is None:
        return
    duration_s = max(0.0, (end - start) / 1_000_000_000)
    if duration_s >= min_duration_s:
        output.append({
            "start_monotonic_ns": start,
            "end_monotonic_ns": end,
            "duration_s": duration_s,
        })


def _camera_timestamp_files(episode_dir: Path) -> list[tuple[str, Path]]:
    cameras_dir = episode_dir / "cameras"
    if not cameras_dir.is_dir():
        return []
    found: list[tuple[str, Path]] = []
    # Current recorder supports cameras/<name>/timestamps.jsonl. Also accept
    # flat <name>_timestamps.jsonl/<name>.timestamps.jsonl for older experiments.
    for path in sorted(cameras_dir.glob("*/timestamps.jsonl")):
        found.append((path.parent.name, path))
    for path in sorted(cameras_dir.glob("*_timestamps.jsonl")):
        found.append((path.name.removesuffix("_timestamps.jsonl"), path))
    for path in sorted(cameras_dir.glob("*.timestamps.jsonl")):
        found.append((path.name.removesuffix(".timestamps.jsonl"), path))
    return found


def _camera_metrics(camera_name: str, path: Path, start_event: Mapping[str, Any] | None, stop_event: Mapping[str, Any] | None, gap_factor: float) -> dict[str, Any]:
    rows, malformed = _read_jsonl(path)
    frames: list[tuple[int, float | None, int | None, Mapping[str, Any]]] = []
    for row in rows:
        frame_number = row.get("frame_number")
        if not isinstance(frame_number, int) or isinstance(frame_number, bool):
            frame_number = None
        host_ns = row.get("host_receive_monotonic_ns")
        if not isinstance(host_ns, int) or isinstance(host_ns, bool):
            host_ns = None
        stamp = _finite_number(row.get("timestamp_ms"))
        frames.append((frame_number if frame_number is not None else len(frames), stamp, host_ns, row))
    # JSONL row order is the receive order. Keep it for clock monotonicity and
    # separately order counters when estimating missing frame numbers.
    frame_numbers = sorted(number for number, _, _, _ in frames if number is not None)
    frame_gaps = [b - a for a, b in zip(frame_numbers, frame_numbers[1:])]
    frame_number_order = [number for number, _, _, _ in frames if number is not None]
    frame_number_deltas = [b - a for a, b in zip(frame_number_order, frame_number_order[1:])]
    timestamp_keys = ("host_receive_monotonic_ns", "timestamp_ms")
    host_clock = sum(1 for _, _, host_ns, _ in frames if host_ns is not None) >= 2
    if host_clock:
        time_values = [host_ns for _, _, host_ns, _ in frames if host_ns is not None]
        deltas_ms = [(b - a) / 1_000_000 for a, b in zip(time_values, time_values[1:])]
        source = timestamp_keys[0]
    else:
        time_values = [stamp for _, stamp, _, _ in frames if stamp is not None]
        deltas_ms = [b - a for a, b in zip(time_values, time_values[1:])]
        source = timestamp_keys[1]
    positive = [delta for delta in deltas_ms if delta > 0]
    typical = median(positive) if positive else None
    long_gap_count = sum(1 for delta in deltas_ms if typical is not None and delta > typical * gap_factor)
    observed_fps = (1000.0 / typical) if typical and typical > 0 else None
    return {
        "camera_name": camera_name,
        "timestamps_path": str(path),
        "frame_count": len(rows),
        "malformed_timestamp_rows": malformed,
        "timestamp_source_for_rate": source,
        "timestamp_delta_ms": _distribution(deltas_ms),
        "observed_fps": observed_fps,
        "frame_number_gap_count": sum(1 for delta in frame_gaps if delta > 1),
        "estimated_missing_frame_count": sum(max(0, delta - 1) for delta in frame_gaps),
        "frame_number_non_increasing_count": sum(1 for delta in frame_number_deltas if delta <= 0),
        "timestamp_non_increasing_count": sum(1 for delta in deltas_ms if delta <= 0),
        "long_gap_threshold_ms": typical * gap_factor if typical is not None else None,
        "long_gap_count": long_gap_count,
        "camera_started_event": dict(start_event) if start_event else None,
        "camera_stopped_event": dict(stop_event) if stop_event else None,
        "reported_frame_count": (stop_event or {}).get("frame_count"),
        "reported_dropped_frames": (stop_event or {}).get("dropped_frames"),
        "timestamp_domain": (stop_event or {}).get("timestamp_domain") or (frames[0][3].get("timestamp_domain") if frames else None),
    }


def inspect_quality(
    session_dir: str | Path,
    *,
    timing_gap_factor: float = 3.0,
    spike_threshold_deg: float = 15.0,
    stationary_epsilon_deg: float = 0.05,
    stationary_min_duration_s: float = 1.0,
) -> dict[str, Any]:
    """Return a JSON-compatible quality report for one raw session.

    Thresholds are screening defaults, not automatic exclusion criteria. All
    detected intervals are reported with their units and threshold settings.
    """
    root = Path(session_dir).expanduser()
    if timing_gap_factor <= 1:
        raise ValueError("timing_gap_factor must be greater than 1")
    if spike_threshold_deg <= 0 or stationary_epsilon_deg < 0 or stationary_min_duration_s < 0:
        raise ValueError("motion thresholds must be non-negative; spike threshold must be positive")
    session_path = root / "session.json"
    if not session_path.is_file():
        raise FileNotFoundError("No session.json found in {}".format(root))
    session = _read_json(session_path)

    episodes_dir = root / "episodes"
    episode_dirs = sorted(path for path in episodes_dir.iterdir() if path.is_dir()) if episodes_dir.is_dir() else []
    episode_reports: list[dict[str, Any]] = []
    total_event_counts: Counter[str] = Counter()
    task_counts: Counter[str] = Counter()
    session_malformed_jsonl = 0

    for episode_dir in episode_dirs:
        metadata_path = episode_dir / "episode.json"
        metadata = _read_json(metadata_path) if metadata_path.is_file() else {}
        events, malformed = _read_jsonl(episode_dir / "telemetry.jsonl")
        session_malformed_jsonl += malformed
        counts = Counter(str(event.get("event_type", "unknown")) for event in events)
        total_event_counts.update(counts)
        task = str(metadata.get("task", session.get("task", "unknown")))
        task_counts[task] += 1

        streams: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for event in events:
            event_type = str(event.get("event_type", "unknown"))
            stream_name = event_type
            if event_type == "robot_state":
                stream_name = "robot_state:" + str(event.get("source", "unknown"))
            elif event_type == "action":
                stream_name = "action:" + str(event.get("action_name", "unknown"))
            if _event_time_ns(event) is not None:
                streams[stream_name].append(event)
        timings = {name: _timing_summary(rows, timing_gap_factor) for name, rows in sorted(streams.items())}

        state_reports: dict[str, Any] = {}
        for source in sorted({str(event.get("source", "unknown")) for event in events if event.get("event_type") == "robot_state"}):
            source_events = [event for event in events if event.get("event_type") == "robot_state" and str(event.get("source", "unknown")) == source]
            metric = _vector_metrics(
                source_events, "joint_position_deg", spike_threshold_deg=spike_threshold_deg,
                stationary_epsilon_deg=stationary_epsilon_deg,
                stationary_min_duration_s=stationary_min_duration_s,
            )
            if metric:
                state_reports[source] = metric
        action_report = _vector_metrics(
            [event for event in events if event.get("event_type") == "action"],
            "joint_target_deg", spike_threshold_deg=spike_threshold_deg,
            stationary_epsilon_deg=stationary_epsilon_deg,
            stationary_min_duration_s=stationary_min_duration_s,
        )
        if action_report is None:
            # Compatibility with the earliest design's field name.
            action_report = _vector_metrics(
                [event for event in events if event.get("event_type") == "action"],
                "nova_action_servoj_deg", spike_threshold_deg=spike_threshold_deg,
                stationary_epsilon_deg=stationary_epsilon_deg,
                stationary_min_duration_s=stationary_min_duration_s,
            )

        start_events: dict[str, Mapping[str, Any]] = {}
        stop_events: dict[str, Mapping[str, Any]] = {}
        for event in events:
            if event.get("event_type") == "camera_started":
                start_events[str(event.get("camera_name", "unknown"))] = event
            elif event.get("event_type") == "camera_stopped":
                stop_events[str(event.get("camera_name", "unknown"))] = event
        cameras = {
            name: _camera_metrics(name, path, start_events.get(name), stop_events.get(name), timing_gap_factor)
            for name, path in _camera_timestamp_files(episode_dir)
        }
        # Keep camera status events visible even if timestamp JSONL was not
        # created, e.g. partial startup or an SDK failure.
        for name in sorted((set(start_events) | set(stop_events)) - set(cameras)):
            cameras[name] = {
                "camera_name": name,
                "timestamps_path": None,
                "frame_count": None,
                "malformed_timestamp_rows": 0,
                "observed_fps": None,
                "frame_number_gap_count": None,
                "estimated_missing_frame_count": None,
                "camera_started_event": dict(start_events[name]) if name in start_events else None,
                "camera_stopped_event": dict(stop_events[name]) if name in stop_events else None,
                "reported_frame_count": (stop_events.get(name) or {}).get("frame_count"),
                "reported_dropped_frames": (stop_events.get(name) or {}).get("dropped_frames"),
            }

        episode_reports.append({
            "episode_id": str(metadata.get("episode_id", episode_dir.name.removeprefix("episode-"))),
            "episode_directory": episode_dir.name,
            "status": metadata.get("status", "unknown"),
            "outcome": _last_value(events, "episode_outcome", "outcome"),
            "reason": metadata.get("reason"),
            "task": task,
            "started_at_utc": metadata.get("started_at_utc"),
            "finished_at_utc": metadata.get("finished_at_utc"),
            "duration_s": _duration_s(metadata.get("started_monotonic_ns"), metadata.get("finished_monotonic_ns")),
            "metadata_event_count": metadata.get("event_count"),
            "telemetry_event_count": len(events),
            "malformed_telemetry_lines": malformed,
            "event_counts": dict(sorted(counts.items())),
            "timing": timings,
            "robot_state": state_reports,
            "action": action_report,
            "cameras": cameras,
        })

    return {
        "report_schema_version": 1,
        "generated_at_utc": _utc_now(),
        "session_path": str(root),
        "session_id": session.get("session_id", root.name),
        "session_status": session.get("status", "unknown"),
        "session_started_at_utc": session.get("started_at_utc"),
        "session_finished_at_utc": session.get("finished_at_utc"),
        "thresholds": {
            "timing_gap_factor": timing_gap_factor,
            "spike_threshold_deg_per_sample": spike_threshold_deg,
            "stationary_epsilon_deg_per_sample": stationary_epsilon_deg,
            "stationary_min_duration_s": stationary_min_duration_s,
        },
        "episode_count": len(episode_reports),
        "episodes_by_status": dict(sorted(Counter(str(item["status"]) for item in episode_reports).items())),
        "episodes_by_task": dict(sorted(task_counts.items())),
        "event_counts": dict(sorted(total_event_counts.items())),
        "malformed_telemetry_lines": session_malformed_jsonl,
        "episodes": episode_reports,
    }


def _last_value(events: Iterable[Mapping[str, Any]], event_type: str, field: str) -> Any:
    result = None
    for event in events:
        if event.get("event_type") == event_type and field in event:
            result = event[field]
    return result


def _duration_s(start: Any, finish: Any) -> float | None:
    if isinstance(start, int) and isinstance(finish, int) and finish >= start:
        return (finish - start) / 1_000_000_000
    return None


def write_quality_report(report: Mapping[str, Any], output_path: str | Path) -> Path:
    """Write a report JSON atomically to an output path."""
    path = Path(output_path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_name(path.name + ".tmp")
    with temp_path.open("w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp_path, path)
    return path


def generate_quality_report(session_dir: str | Path, output_path: str | Path | None = None, **kwargs: Any) -> dict[str, Any]:
    """Inspect a session and optionally persist the resulting report."""
    report = inspect_quality(session_dir, **kwargs)
    if output_path is not None:
        write_quality_report(report, output_path)
    return report


__all__ = ["inspect_quality", "generate_quality_report", "write_quality_report"]
