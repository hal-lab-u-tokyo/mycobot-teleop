"""Shared strict JSON and raw-session file helpers."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterator


class DatasetInputError(ValueError):
    """Raw session input is missing, malformed, or internally inconsistent."""


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise DatasetInputError("Could not read {}: {}".format(path, exc)) from exc
    except json.JSONDecodeError as exc:
        raise DatasetInputError("Invalid JSON in {}: {}".format(path, exc)) from exc
    if not isinstance(value, dict):
        raise DatasetInputError("Expected a JSON object in {}".format(path))
    return value


def read_jsonl(path: Path) -> Iterator[tuple[int, dict[str, Any]]]:
    try:
        stream = path.open("r", encoding="utf-8")
    except OSError as exc:
        raise DatasetInputError("Could not read {}: {}".format(path, exc)) from exc
    with stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError as exc:
                raise DatasetInputError(
                    "Invalid JSON at {}:{}: {}".format(path, line_number, exc)
                ) from exc
            if not isinstance(value, dict):
                raise DatasetInputError(
                    "Expected a JSON object at {}:{}".format(path, line_number)
                )
            yield line_number, value


def session_episodes(session_dir: Path) -> list[Path]:
    episodes = session_dir / "episodes"
    if not episodes.is_dir():
        raise DatasetInputError("Missing episodes directory: {}".format(episodes))
    return sorted(
        (path for path in episodes.iterdir() if path.is_dir() and (path / "episode.json").is_file()),
        key=lambda path: path.name,
    )


def camera_timestamp_path(episode_dir: Path, camera_name: str) -> Path:
    """Resolve currently supported timestamp layouts without guessing contents."""
    candidates = (
        episode_dir / "cameras" / camera_name / "timestamps.jsonl",
        episode_dir / "cameras" / (camera_name + ".timestamps.jsonl"),
        episode_dir / "cameras" / (camera_name + "_timestamps.jsonl"),
        episode_dir / "timestamps" / (camera_name + ".jsonl"),
    )
    for path in candidates:
        if path.is_file():
            return path
    return candidates[0]
