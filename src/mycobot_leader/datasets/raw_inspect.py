"""Read-only diagnostics for the raw session and episode files.

The report deliberately describes timestamp quality and coverage; it does not
repair data or infer missing sensor samples.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import statistics
from pathlib import Path
from typing import Any

from ._io import DatasetInputError, camera_timestamp_path, read_json, read_jsonl, session_episodes


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = int(index)
    upper = min(lower + 1, len(ordered) - 1)
    blend = index - lower
    return ordered[lower] * (1.0 - blend) + ordered[upper] * blend


def _timing_summary(times_ns: list[int]) -> dict[str, Any]:
    if len(times_ns) < 2:
        return {
            "sample_count": len(times_ns),
            "duration_s": 0.0 if times_ns else None,
            "effective_hz": None,
            "delta_ms": None,
            "duplicate_count": 0,
            "backward_count": 0,
        }
    deltas = [(right - left) / 1_000_000 for left, right in zip(times_ns, times_ns[1:])]
    positive = [delta for delta in deltas if delta > 0]
    duration_s = (times_ns[-1] - times_ns[0]) / 1_000_000_000
    return {
        "sample_count": len(times_ns),
        "duration_s": duration_s,
        "effective_hz": (len(times_ns) - 1) / duration_s if duration_s > 0 else None,
        "delta_ms": {
            "min": min(deltas),
            "median": statistics.median(deltas),
            "p95": _percentile(deltas, 0.95),
            "max": max(deltas),
        },
        "duplicate_count": sum(delta == 0 for delta in deltas),
        "backward_count": sum(delta < 0 for delta in deltas),
        "positive_delta_count": len(positive),
    }


@dataclass(frozen=True)
class InspectionReport:
    """Serializable summary returned by :func:`inspect_session`."""

    session_dir: str
    session: dict[str, Any]
    episodes: list[dict[str, Any]]
    warnings: list[str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def inspect_session(session_dir: str | Path) -> InspectionReport:
    """Inspect a raw session's event clocks, counts, and camera timestamp files."""
    root = Path(session_dir).expanduser().resolve()
    session_path = root / "session.json"
    session = read_json(session_path)
    episode_reports: list[dict[str, Any]] = []
    warnings: list[str] = []

    for episode_dir in session_episodes(root):
        episode = read_json(episode_dir / "episode.json")
        telemetry_path = episode_dir / "telemetry.jsonl"
        event_counts: dict[str, int] = {}
        state_times: list[int] = []
        action_times: list[int] = []
        event_count = 0
        for line, event in read_jsonl(telemetry_path):
            event_count += 1
            kind = str(event.get("event_type", "unknown"))
            event_counts[kind] = event_counts.get(kind, 0) + 1
            try:
                if kind == "robot_state":
                    start = int(event["host_request_start_ns"])
                    end = int(event["host_response_ns"])
                    if end < start:
                        warnings.append("{}:{} robot_state response precedes request".format(telemetry_path, line))
                    state_times.append((start + end) // 2)
                elif kind == "action":
                    start = int(event["host_send_start_ns"])
                    end = int(event["host_response_ns"])
                    if end < start:
                        warnings.append("{}:{} action response precedes send".format(telemetry_path, line))
                    action_times.append((start + end) // 2)
            except (KeyError, TypeError, ValueError):
                warnings.append("{}:{} missing or invalid timestamp fields".format(telemetry_path, line))

        cameras: dict[str, Any] = {}
        cameras_dir = episode_dir / "cameras"
        if cameras_dir.is_dir():
            bag_paths = sorted(cameras_dir.glob("*.bag"))
            for bag_path in bag_paths:
                camera_name = bag_path.stem
                timestamp_path = camera_timestamp_path(episode_dir, camera_name)
                if not timestamp_path.is_file():
                    cameras[camera_name] = {
                        "bag_path": str(bag_path.relative_to(root)),
                        "timestamp_path": None,
                        "timestamp_summary": None,
                        "warning": "camera timestamp JSONL is missing",
                    }
                    warnings.append("{} has no timestamp JSONL".format(bag_path))
                    continue
                host_times: list[int] = []
                frame_numbers: list[int] = []
                timestamp_domains: dict[str, int] = {}
                for line, row in read_jsonl(timestamp_path):
                    try:
                        host_times.append(int(row["host_receive_monotonic_ns"]))
                        frame_numbers.append(int(row["frame_number"]))
                        domain = str(row.get("timestamp_domain", "unknown"))
                        timestamp_domains[domain] = timestamp_domains.get(domain, 0) + 1
                    except (KeyError, TypeError, ValueError):
                        warnings.append("{}:{} missing or invalid camera timestamp".format(timestamp_path, line))
                cameras[camera_name] = {
                    "bag_path": str(bag_path.relative_to(root)),
                    "timestamp_path": str(timestamp_path.relative_to(root)),
                    "timestamp_summary": _timing_summary(host_times),
                    "frame_number_gap_count": sum(
                        right > left + 1 for left, right in zip(frame_numbers, frame_numbers[1:])
                    ),
                    "timestamp_domains": timestamp_domains,
                }

        episode_reports.append({
            "episode_dir": episode_dir.name,
            "episode": episode,
            "event_count": event_count,
            "event_counts": event_counts,
            "robot_state_timing": _timing_summary(state_times),
            "action_timing": _timing_summary(action_times),
            "cameras": cameras,
        })

    if not episode_reports:
        warnings.append("session contains no finalized episodes")
    return InspectionReport(str(root), session, episode_reports, warnings)


__all__ = ["DatasetInputError", "InspectionReport", "inspect_session"]
