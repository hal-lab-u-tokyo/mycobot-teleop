"""Append-only episode-level curation decisions kept outside raw recordings.

Public API::

    path = write_curation_decisions(
        session_dir,
        [{"episode_id": "1", "decision": "keep", "reason": "...", "method": "manual_review"}],
    )
    rows = load_curation_decisions(path)

Each row references an episode ID from ``session.json``'s episode directories
and records a decision revision. The raw session directory is read-only from
this module's perspective; output defaults to a sibling ``curation/`` folder.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
from typing import Any, Iterable, Mapping


DECISIONS_SCHEMA_VERSION = 1
VALID_DECISIONS = {"keep", "filter", "relabel"}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _episode_ids(session_dir: Path) -> set[str]:
    episodes_dir = session_dir / "episodes"
    if not episodes_dir.is_dir():
        raise FileNotFoundError("No episodes directory found in {}".format(session_dir))
    result: set[str] = set()
    for episode_dir in episodes_dir.iterdir():
        if not episode_dir.is_dir():
            continue
        metadata_path = episode_dir / "episode.json"
        if metadata_path.is_file():
            with metadata_path.open("r", encoding="utf-8") as stream:
                metadata = json.load(stream)
            if isinstance(metadata, dict) and metadata.get("episode_id") is not None:
                result.add(str(metadata["episode_id"]))
                continue
        # Preserve compatibility with partially written episodes whose
        # metadata is missing, using the recorder's directory convention.
        result.add(episode_dir.name.removeprefix("episode-"))
    return result


def _normalize_decision(row: Mapping[str, Any], valid_ids: set[str]) -> dict[str, Any]:
    episode_id = row.get("episode_id")
    if episode_id is None or not str(episode_id).strip():
        raise ValueError("Every curation row must include a non-empty episode_id")
    episode_id = str(episode_id)
    if episode_id not in valid_ids:
        raise ValueError("Unknown episode_id {!r}; session contains {}".format(episode_id, sorted(valid_ids)))

    decision = row.get("decision")
    if decision not in VALID_DECISIONS:
        raise ValueError("decision must be one of {}".format(", ".join(sorted(VALID_DECISIONS))))
    reason = row.get("reason")
    method = row.get("method")
    if not isinstance(reason, str) or not reason.strip():
        raise ValueError("Every curation decision must include a non-empty reason")
    if not isinstance(method, str) or not method.strip():
        raise ValueError("Every curation decision must include a non-empty method")

    normalized: dict[str, Any] = {
        "curation_schema_version": DECISIONS_SCHEMA_VERSION,
        "episode_id": episode_id,
        "decision": decision,
        "reason": reason.strip(),
        "method": method.strip(),
        "decided_at_utc": str(row.get("decided_at_utc") or _utc_now()),
    }
    # Relabel instructions are explicit metadata overrides; no raw metadata is
    # edited. They are copied into the separate decision row for traceability.
    labels = row.get("labels")
    if labels is not None:
        if not isinstance(labels, Mapping) or not labels:
            raise ValueError("labels must be a non-empty object when provided")
        normalized["labels"] = dict(labels)
    if decision == "relabel" and "labels" not in normalized:
        raise ValueError("relabel decisions must include a non-empty labels object")
    if row.get("reviewer") is not None:
        normalized["reviewer"] = str(row["reviewer"])
    if row.get("evidence") is not None:
        normalized["evidence"] = row["evidence"]
    return normalized


def default_decisions_path(session_dir: str | Path) -> Path:
    """Return the recommended curation output path outside the raw session."""
    root = Path(session_dir).expanduser().resolve()
    return root.parent / "curation" / (root.name + "-decisions.jsonl")


def load_curation_decisions(path: str | Path) -> list[dict[str, Any]]:
    """Read all append-only decision revisions from a JSONL file."""
    source = Path(path).expanduser()
    rows: list[dict[str, Any]] = []
    if not source.exists():
        return rows
    with source.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError("Invalid JSON at {}:{}".format(source, line_number)) from exc
            if not isinstance(value, dict):
                raise ValueError("Expected a JSON object at {}:{}".format(source, line_number))
            rows.append(value)
    return rows


def write_curation_decisions(
    session_dir: str | Path,
    decisions: Iterable[Mapping[str, Any]],
    output_path: str | Path | None = None,
) -> Path:
    """Append episode decisions to a separate JSONL file and return its path.

    Decisions are validated against raw episode IDs. Multiple revisions for an
    episode are allowed across calls and remain in append order. Within one
    call, each episode may appear at most once to avoid accidental ambiguity.
    The destination is rejected if it resolves inside the raw session tree.
    """
    root = Path(session_dir).expanduser().resolve()
    if not (root / "session.json").is_file():
        raise FileNotFoundError("No session.json found in {}".format(root))
    target = (Path(output_path).expanduser() if output_path is not None else default_decisions_path(root)).resolve()
    try:
        target.relative_to(root)
    except ValueError:
        pass
    else:
        raise ValueError("Curation decisions must be stored outside the immutable raw session directory")

    valid_ids = _episode_ids(root)
    normalized: list[dict[str, Any]] = []
    seen: set[str] = set()
    for row in decisions:
        if not isinstance(row, Mapping):
            raise TypeError("decisions must contain mappings")
        item = _normalize_decision(row, valid_ids)
        if item["episode_id"] in seen:
            raise ValueError("At most one decision per episode is allowed in a single write")
        seen.add(item["episode_id"])
        item["session_id"] = root.name
        normalized.append(item)
    if not normalized:
        raise ValueError("At least one curation decision is required")

    encoded_rows = [
        json.dumps(item, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n"
        for item in normalized
    ]
    target.parent.mkdir(parents=True, exist_ok=True)
    # Append and fsync so an earlier curation history remains available. This
    # file is deliberately independent from telemetry, episode and session JSON.
    with target.open("a", encoding="utf-8") as stream:
        stream.writelines(encoded_rows)
        stream.flush()
        os.fsync(stream.fileno())
    return target


__all__ = [
    "DECISIONS_SCHEMA_VERSION",
    "VALID_DECISIONS",
    "default_decisions_path",
    "load_curation_decisions",
    "write_curation_decisions",
]
