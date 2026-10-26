"""Convert synchronized raw RealSense/robot data to a local LeRobot v3 dataset.

The exporter is deliberately offline. It reads recorded bag files and never
connects to a camera or robot. It uses each accepted sample's explicit frame
references, writes the RGB arrays through LeRobot's public dataset writer, and
stores the original host-monotonic time in an auxiliary feature so the nominal
LeRobot FPS clock is not mistaken for the acquisition clock.
"""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
from importlib.metadata import PackageNotFoundError, version as package_version
import json
from pathlib import Path
import re
from typing import Any

from ._io import DatasetInputError, read_json, read_jsonl, session_episodes
from .synchronize import SynchronizationConfig, synchronize_session


LEROBOT_VERSION = "0.6.1"


class LeRobotExportError(RuntimeError):
    """The synchronized raw session cannot be converted safely."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_optional_dependencies() -> tuple[Any, Any, Any]:
    try:
        import numpy as np
    except ImportError as exc:
        raise LeRobotExportError(
            "LeRobot export requires NumPy; install the dataset export dependencies."
        ) from exc
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise LeRobotExportError(
            "LeRobot export needs pyrealsense2 to decode the recorded RGB bag files. "
            "Install a binding compatible with this Python and macOS environment."
        ) from exc
    try:
        import lerobot
        from lerobot.datasets import LeRobotDataset
    except ImportError as exc:
        raise LeRobotExportError(
            "Install the dataset extra for lerobot=={} to write a LeRobot v3 dataset.".format(
                LEROBOT_VERSION
            )
        ) from exc
    version_value = getattr(lerobot, "__version__", None)
    if version_value is None:
        try:
            version_value = package_version("lerobot")
        except PackageNotFoundError:
            version_value = None
    if version_value is not None and version_value != LEROBOT_VERSION:
        raise LeRobotExportError(
            "This converter is pinned to lerobot=={}; found {}.".format(LEROBOT_VERSION, version_value)
        )
    return np, rs, LeRobotDataset


def _read_events(path: Path) -> list[dict[str, Any]]:
    return [event for _, event in read_jsonl(path)]


def _camera_profiles(episode_dir: Path) -> dict[str, tuple[int, int, int]]:
    events_path = episode_dir / "telemetry.jsonl"
    profiles: dict[str, tuple[int, int, int]] = {}
    for event in _read_events(events_path):
        if event.get("event_type") != "camera_started":
            continue
        name = event.get("camera_name")
        width, height, fps = event.get("width"), event.get("height"), event.get("fps")
        if not isinstance(name, str) or not all(isinstance(v, int) and v > 0 for v in (width, height, fps)):
            continue
        profiles[name] = (width, height, fps)
    if not profiles:
        raise DatasetInputError("No camera_started profile found in {}".format(events_path))
    return profiles


class _BagFrameReader:
    """Sequentially decode only requested RGB frames from one RealSense bag."""

    def __init__(self, rs: Any, np: Any, bag_path: Path, profile: tuple[int, int, int]) -> None:
        self._rs = rs
        self._np = np
        self._bag_path = bag_path
        self._width, self._height, self._fps = profile
        self._pipeline: Any = None
        self._last_number: int | None = None
        self._last_image: Any = None

    def __enter__(self) -> "_BagFrameReader":
        config = self._rs.config()
        config.enable_device_from_file(str(self._bag_path), repeat_playback=False)
        config.enable_stream(
            self._rs.stream.color,
            self._width,
            self._height,
            self._rs.format.rgb8,
            self._fps,
        )
        self._pipeline = self._rs.pipeline()
        try:
            profile = self._pipeline.start(config)
            profile.get_device().as_playback().set_real_time(False)
        except Exception as exc:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            raise LeRobotExportError(
                "Could not open recorded RGB bag {}: {}".format(self._bag_path, exc)
            ) from exc
        return self

    def frame(self, wanted_number: int) -> Any:
        if self._last_number == wanted_number and self._last_image is not None:
            return self._last_image.copy()
        if self._last_number is not None and wanted_number < self._last_number:
            raise LeRobotExportError(
                "Requested camera frames are not monotonic in {} ({} after {}).".format(
                    self._bag_path, wanted_number, self._last_number
                )
            )
        while self._last_number is None or self._last_number < wanted_number:
            try:
                frames = self._pipeline.wait_for_frames(5000)
            except Exception as exc:
                raise LeRobotExportError(
                    "Bag ended or failed before frame {} in {}: {}".format(
                        wanted_number, self._bag_path, exc
                    )
                ) from exc
            color = frames.get_color_frame()
            if not color:
                continue
            frame_number = int(color.get_frame_number())
            if frame_number < wanted_number:
                self._last_number = frame_number
                continue
            if frame_number != wanted_number:
                raise LeRobotExportError(
                    "Bag {} skipped requested frame {} (next frame is {}).".format(
                        self._bag_path, wanted_number, frame_number
                    )
                )
            image = self._np.asanyarray(color.get_data()).copy()
            if image.shape != (self._height, self._width, 3):
                raise LeRobotExportError(
                    "Unexpected RGB shape {} in {} (expected {}).".format(
                        image.shape, self._bag_path, (self._height, self._width, 3)
                    )
                )
            self._last_number = frame_number
            self._last_image = image
        return self._last_image.copy()

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                if exc_type is None:
                    raise


def _episode_dirs_by_name(session_dir: Path) -> dict[str, Path]:
    return {path.name: path for path in session_episodes(session_dir)}


def _latest_curation(session_dir: Path, curation_path: str | Path | None) -> dict[str, dict[str, Any]]:
    if curation_path is None:
        default = session_dir.parent / "curation" / (session_dir.name + "-decisions.jsonl")
        path = default if default.is_file() else None
    else:
        path = Path(curation_path).expanduser()
    if path is None:
        return {}
    latest: dict[str, dict[str, Any]] = {}
    for _, row in read_jsonl(path):
        if row.get("session_id") == session_dir.name and row.get("episode_id") is not None:
            latest[str(row["episode_id"])] = row
    return latest


def _task_for_sample(sample: dict[str, Any], curation: dict[str, Any] | None) -> str:
    task = sample.get("task")
    if curation and isinstance(curation.get("labels"), dict):
        task = curation["labels"].get("task", task)
    if not isinstance(task, str) or not task.strip():
        raise DatasetInputError("Synchronized samples require a non-empty task description")
    return task.strip()


def export_lerobot(
    session_dir: str | Path,
    output_dir: str | Path,
    config: SynchronizationConfig,
    *,
    fps: int = 30,
    curation_path: str | Path | None = None,
    repo_id: str | None = None,
) -> dict[str, Any]:
    """Export a synchronized raw session and verify it through the public reader.

    ``output_dir`` must not exist. Episodes marked ``filter`` are skipped when
    a curation file is present; completed failures remain eligible. Interrupted
    and error episodes are excluded and listed in the conversion manifest.
    Samples that don't have all configured camera/state/action matches are
    already explicit in the synchronized output and are not exported.
    """
    if fps <= 0:
        raise ValueError("fps must be positive")
    root = Path(session_dir).expanduser().resolve()
    output = Path(output_dir).expanduser().resolve()
    if output.exists():
        raise FileExistsError("Refusing to overwrite derived output: {}".format(output))
    if not (root / "session.json").is_file():
        raise DatasetInputError("No session.json found in {}".format(root))

    np, rs, LeRobotDataset = _load_optional_dependencies()
    sync = synchronize_session(root, config)
    raw_episodes = _episode_dirs_by_name(root)
    curation = _latest_curation(root, curation_path)

    export_episodes: list[tuple[dict[str, Any], Path, dict[str, Any] | None]] = []
    skipped: list[dict[str, Any]] = []
    profiles_by_camera: dict[str, tuple[int, int, int]] = {}
    action_width: int | None = None
    state_width: int | None = None
    for synchronized_episode in sync["episodes"]:
        episode_path = Path(synchronized_episode["episode_dir"]).resolve()
        episode_name = episode_path.name
        raw_path = raw_episodes.get(episode_name)
        if raw_path is None:
            raise DatasetInputError("Synchronized episode not found in raw session: {}".format(episode_name))
        metadata = read_json(raw_path / "episode.json")
        episode_id = str(metadata.get("episode_id", episode_name.removeprefix("episode-")))
        decision = curation.get(episode_id)
        if decision and decision.get("decision") == "filter":
            skipped.append({"episode_id": episode_id, "reason": "curation_filter", "curation": decision})
            continue
        if metadata.get("status") not in {"completed", "complete"}:
            skipped.append({
                "episode_id": episode_id,
                "reason": "episode_status_{}".format(metadata.get("status", "unknown")),
            })
            continue
        samples = synchronized_episode.get("samples", [])
        if not samples:
            skipped.append({"episode_id": episode_id, "reason": "no_synchronized_samples"})
            continue

        profiles = _camera_profiles(raw_path)
        for name, profile in profiles.items():
            if name in profiles_by_camera and profiles_by_camera[name] != profile:
                raise DatasetInputError("Camera profile changed within the session for {!r}".format(name))
            profiles_by_camera[name] = profile
        if set(profiles) != set(profiles_by_camera):
            raise DatasetInputError("Camera names differ between episodes in {}".format(root))
        for sample in samples:
            if set(sample.get("cameras", {})) != set(profiles):
                raise DatasetInputError(
                    "Synchronized sample camera set differs from camera profile in {}".format(episode_name)
                )
            observation = sample.get("observation", {}).get("joint_position_deg")
            action = sample.get("action", {}).get("joint_target_deg")
            if not isinstance(observation, list) or not isinstance(action, list):
                raise DatasetInputError("Synchronized sample is missing state or action in {}".format(episode_name))
            if state_width is None:
                state_width, action_width = len(observation), len(action)
            if len(observation) != state_width or len(action) != action_width:
                raise DatasetInputError("State/action shape changed within the session")
        export_episodes.append((synchronized_episode, raw_path, decision))

    if not export_episodes:
        raise LeRobotExportError("No completed episodes contain synchronized samples after filtering.")
    assert state_width is not None and action_width is not None
    camera_names = sorted(profiles_by_camera)
    feature_names = ["joint_{}".format(index + 1) for index in range(max(state_width, action_width))]
    features: dict[str, dict[str, Any]] = {
        "observation.state": {
            "dtype": "float32", "shape": [state_width],
            "names": feature_names[:state_width],
        },
        "action": {
            "dtype": "float32", "shape": [action_width],
            "names": feature_names[:action_width],
        },
        "source_time_ns": {
            "dtype": "int64", "shape": [1], "names": ["host_monotonic_ns"],
        },
        "observation_state_match_delta_ms": {
            "dtype": "float32", "shape": [1], "names": ["delta_ms"],
        },
        "action_match_delta_ms": {
            "dtype": "float32", "shape": [1], "names": ["delta_ms"],
        },
    }
    for name in camera_names:
        width, height, _ = profiles_by_camera[name]
        features["observation.images." + name] = {
            "dtype": "video", "shape": [height, width, 3],
            "names": ["height", "width", "channels"],
        }

    dataset_id = repo_id or "local/{}".format(re.sub(r"[^A-Za-z0-9_-]+", "-", output.name).strip("-") or "dataset")
    output.parent.mkdir(parents=True, exist_ok=True)
    dataset = LeRobotDataset.create(
        repo_id=dataset_id,
        root=output,
        fps=fps,
        features=features,
        robot_type="mycobot280_dobot_nova5",
        use_videos=True,
        image_writer_processes=0,
        image_writer_threads=2,
        streaming_encoding=False,
    )

    exported: list[dict[str, Any]] = []
    total_frames = 0
    try:
        for synchronized_episode, raw_path, decision in export_episodes:
            episode_path = Path(synchronized_episode["episode_dir"]).resolve()
            episode_meta = read_json(raw_path / "episode.json")
            episode_id = str(episode_meta.get("episode_id", episode_path.name.removeprefix("episode-")))
            samples = synchronized_episode["samples"]
            readers: dict[str, _BagFrameReader] = {}
            try:
                for name in camera_names:
                    bag_path = raw_path / "cameras" / (name + ".bag")
                    if not bag_path.is_file():
                        raise DatasetInputError("Missing RGB bag: {}".format(bag_path))
                    reader = _BagFrameReader(rs, np, bag_path, profiles_by_camera[name])
                    reader.__enter__()
                    readers[name] = reader
                for sample in samples:
                    frame: dict[str, Any] = {
                        "observation.state": np.asarray(
                            sample["observation"]["joint_position_deg"], dtype=np.float32
                        ),
                        "action": np.asarray(
                            sample["action"]["joint_target_deg"], dtype=np.float32
                        ),
                        "source_time_ns": np.asarray(
                            [sample["reference_time_monotonic_ns"]], dtype=np.int64
                        ),
                        "observation_state_match_delta_ms": np.asarray(
                            [sample["observation"].get("state_match_delta_ns", 0) / 1_000_000],
                            dtype=np.float32,
                        ),
                        "action_match_delta_ms": np.asarray(
                            [sample["action"].get("match_delta_ns", 0) / 1_000_000],
                            dtype=np.float32,
                        ),
                        "task": _task_for_sample(sample, decision),
                    }
                    for name in camera_names:
                        camera = sample["cameras"][name]
                        frame["observation.images." + name] = readers[name].frame(
                            int(camera["frame_number"])
                        )
                    dataset.add_frame(frame)
                dataset.save_episode()
            finally:
                for reader in readers.values():
                    reader.__exit__(None, None, None)
            total_frames += len(samples)
            telemetry_events = _read_events(raw_path / "telemetry.jsonl")
            outcome = next(
                (
                    event.get("outcome")
                    for event in reversed(telemetry_events)
                    if event.get("event_type") == "episode_outcome"
                ),
                episode_meta.get("outcome"),
            )
            exported.append({
                "source_episode_id": episode_id,
                "source_episode_directory": episode_path.name,
                "dataset_episode_index": len(exported),
                "frames": len(samples),
                "outcome": outcome,
                "curation_decision": decision.get("decision") if decision else "unreviewed",
                "synchronization_excluded_count": synchronized_episode.get("excluded_count", 0),
            })
    finally:
        dataset.finalize()

    # A reader round-trip checks that the final files can be opened through
    # LeRobot's public API before the conversion manifest claims success.
    reader = LeRobotDataset(repo_id=dataset_id, root=output)
    roundtrip_frames = len(reader)
    if roundtrip_frames != total_frames or roundtrip_frames == 0:
        raise LeRobotExportError(
            "LeRobot reader round-trip returned {} frames; expected {}.".format(
                roundtrip_frames, total_frames
            )
        )
    first = reader[0]
    required_keys = {"observation.state", "action"} | {
        "observation.images." + name for name in camera_names
    }
    missing = sorted(required_keys - set(first))
    if missing:
        raise LeRobotExportError("LeRobot reader round-trip is missing features: {}".format(missing))

    manifest = {
        "format": "lerobot_v3",
        "converter_schema_version": 1,
        "generated_at_utc": _utc_now(),
        "source_session": str(root),
        "source_session_id": root.name,
        "source_session_json_sha256": _sha256(root / "session.json"),
        "repo_id": dataset_id,
        "lerobot_version": LEROBOT_VERSION,
        "fps": fps,
        "features": features,
        "state_semantics": "{} measured joint position in degrees".format(
            config.state_source or next((entry["diagnostics"].get("robot_state_source") for entry in sync["episodes"] if entry.get("diagnostics", {}).get("robot_state_source")), "selected robot source")
        ),
        "action_semantics": "absolute Nova ServoJ joint target in degrees",
        "synchronization_config": sync["config"],
        "time_semantics": "LeRobot timestamps use the requested nominal FPS; source_time_ns preserves each raw host-monotonic reference instant.",
        "match_delta_semantics": {
            "observation_state_match_delta_ms": "signed source state time minus reference image time; zero for exact/interpolated state",
            "action_match_delta_ms": "signed matched action time minus (reference image time + action_delay)",
        },
        "camera_bags": {
            name: [
                {
                    "episode": raw_path.name,
                    "path": str((raw_path / "cameras" / (name + ".bag")).relative_to(root)),
                    "sha256": _sha256(raw_path / "cameras" / (name + ".bag")),
                }
                for _, raw_path, _ in export_episodes
            ]
            for name in camera_names
        },
        "episode_map": exported,
        "skipped_episodes": skipped,
        "summary": {
            "source_episode_count": len(sync["episodes"]),
            "exported_episode_count": len(exported),
            "exported_frame_count": total_frames,
            "synchronized_excluded_count": sum(int(item.get("excluded_count", 0)) for item, _, _ in export_episodes),
            "skipped_episode_count": len(skipped),
            "roundtrip_frame_count": roundtrip_frames,
        },
    }
    manifest_path = output / "conversion_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, allow_nan=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest


def prepare_synchronized_intermediate(
    session_dir: str | Path,
    output_dir: str | Path,
    config: SynchronizationConfig,
) -> dict[str, Any]:
    """Backward-compatible alias for the image-reference staging export."""
    from .synchronize import write_synchronized_intermediate

    return write_synchronized_intermediate(session_dir, output_dir, config)


__all__ = ["LEROBOT_VERSION", "LeRobotExportError", "export_lerobot", "prepare_synchronized_intermediate"]
