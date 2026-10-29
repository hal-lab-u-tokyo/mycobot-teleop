"""Native RealSense bag capture for one or more RGB cameras.

The RealSense SDK dependency is imported lazily so applications can use the
rest of the package without the SDK installed. ``pyrealsense2`` must be
available in the Python interpreter running the recorder; the Homebrew SDK
command-line tools alone do not provide that Python binding.

Typical use::

    cameras = MultiRealSenseCapture(
        [CameraSpec("front", "123456789"), CameraSpec("wrist", "987654321")],
        output_dir="data/recordings/session/episodes/episode-0001/cameras",
    )
    cameras.start()
    try:
        ...  # record the teleoperation episode
        cameras.raise_if_failed()
    finally:
        final_status = cameras.stop()

Each camera is selected by its USB serial number and recorded into a separate
``<name>.bag`` file. A sibling ``<name>.timestamps.jsonl`` records each RGB
frame's SDK frame number, timestamp and timestamp domain, plus the PC monotonic
time immediately after ``wait_for_frames`` returns. The bag is written by
librealsense, preserving supported device metadata. The capture worker consumes
framesets to monitor progress and detect counter gaps; it does not re-time or
synchronize cameras. The default stream is RGB8 at 640x480 and 30 fps; depth is
not enabled by this adapter.

On macOS, USB access for librealsense may require launching the Python process
with administrator privileges. If enumeration or startup reports no device,
check USB permission and confirm the serial with ``rs-enumerate-devices``.
This class never invokes sudo itself.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import threading
import time
from typing import Any, Iterable, Mapping


class RealSenseCaptureError(RuntimeError):
    """Base error for RealSense camera setup and capture failures."""


class RealSenseUnavailableError(RealSenseCaptureError):
    """Raised when the optional Python SDK or a configured device is absent."""


@dataclass(frozen=True)
class CameraSpec:
    """Stable role-to-device mapping for a camera in a capture session."""

    name: str
    serial: str

    def __post_init__(self) -> None:
        if not self.name or self.name in {".", ".."}:
            raise ValueError("Camera name must be a non-empty simple path name")
        if Path(self.name).name != self.name or "/" in self.name or "\\" in self.name:
            raise ValueError("Camera name must not contain path separators")
        if not self.serial:
            raise ValueError("Camera serial must be provided explicitly")


@dataclass
class CameraStatus:
    """Runtime health and basic frame-clock diagnostics for one camera."""

    name: str
    serial: str
    bag_path: str
    state: str = "configured"
    model: str | None = None
    width: int | None = None
    height: int | None = None
    fps: int | None = None
    observed_fps: float | None = None
    frame_count: int = 0
    timestamp_rows_written: int = 0
    dropped_frames: int = 0
    last_frame_number: int | None = None
    last_timestamp_ms: float | None = None
    timestamp_domain: str | None = None
    started_monotonic_ns: int | None = None
    stopped_monotonic_ns: int | None = None
    error: str | None = None
    timestamps_path: str = ""
    timestamps_error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable status snapshot."""
        return asdict(self)


@dataclass
class _CameraRuntime:
    spec: CameraSpec
    status: CameraStatus
    pipeline: Any = None
    worker: threading.Thread | None = None
    timestamps_file: Any = None


def _load_sdk() -> Any:
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise RealSenseUnavailableError(
            "pyrealsense2 is not installed in this Python environment. Install a "
            "librealsense Python binding compatible with this interpreter and OS; "
            "the `rs-enumerate-devices` binary by itself is not sufficient."
        ) from exc
    return rs


def _normalize_specs(camera_specs: Iterable[CameraSpec | Mapping[str, Any]]) -> list[CameraSpec]:
    specs = [
        spec if isinstance(spec, CameraSpec) else CameraSpec(**spec)
        for spec in camera_specs
    ]
    if not specs:
        raise ValueError("At least one RealSense camera must be configured")
    names = [spec.name for spec in specs]
    serials = [spec.serial for spec in specs]
    if len(names) != len(set(names)):
        raise ValueError("Camera names must be unique")
    if len(serials) != len(set(serials)):
        raise ValueError("Camera serials must be unique")
    return specs


def _enum_name(value: Any) -> str:
    name = getattr(value, "name", None)
    if name:
        return str(name)
    return str(value)


class MultiRealSenseCapture:
    """Capture native ``.bag`` files from explicitly selected RealSense RGB streams.

    Args:
        camera_specs: ``CameraSpec`` items or mappings with ``name`` and
            ``serial`` keys. Explicit serial selection prevents camera roles
            from silently swapping between sessions.
        output_dir: Directory that receives one ``<name>.bag`` per camera.
        width: Requested RGB width, default 640.
        height: Requested RGB height, default 480.
        fps: Requested RGB frame rate, default 30.
        wait_timeout_ms: Maximum wait for a frameset before capture is marked
            failed. This is deliberately longer than one frame interval so
            ordinary frame-rate variation is not considered an error.

    Methods:
        start(): Start all configured device pipelines and monitoring workers.
            If any camera fails to start, already-started pipelines are stopped
            and an exception is raised.
        stop(): Stop pipelines so bag footers are flushed, join workers, and
            return final per-camera status snapshots. Safe to call repeatedly.
        close(): Alias for ``stop()``.
        raise_if_failed(): Raise when a capture worker has encountered an error.

    Properties:
        status: A fresh mapping of camera name to ``CameraStatus`` snapshots.
        is_running: True while at least one configured capture is running.

    The caller owns episode timing and any robot/camera synchronization. Call
    ``start`` only for the episode window to be captured and always call
    ``stop`` in a ``finally`` block.
    """

    def __init__(
        self,
        camera_specs: Iterable[CameraSpec | Mapping[str, Any]],
        output_dir: str | Path,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        wait_timeout_ms: int = 1000,
    ) -> None:
        if width <= 0 or height <= 0 or fps <= 0:
            raise ValueError("width, height, and fps must be positive")
        if wait_timeout_ms <= 0:
            raise ValueError("wait_timeout_ms must be positive")
        self._specs = _normalize_specs(camera_specs)
        self.output_dir = Path(output_dir)
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.wait_timeout_ms = int(wait_timeout_ms)
        self._rs: Any = None
        self._runtimes: dict[str, _CameraRuntime] = {}
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._started = False
        self._stopping = False

    @staticmethod
    def enumerate_devices() -> list[dict[str, str]]:
        """List connected devices without starting streams or writing files."""
        rs = _load_sdk()
        try:
            devices = rs.context().query_devices()
            return [
                {
                    "serial": device.get_info(rs.camera_info.serial_number),
                    "name": device.get_info(rs.camera_info.name),
                    "firmware": (
                        device.get_info(rs.camera_info.firmware_version)
                        if device.supports(rs.camera_info.firmware_version)
                        else "unknown"
                    ),
                }
                for device in devices
            ]
        except Exception as exc:
            raise RealSenseUnavailableError(
                "Could not enumerate RealSense devices. Check the USB connection, "
                "SDK installation, and macOS USB permissions (the capture process "
                "may need to be launched with sudo)."
            ) from exc

    @property
    def is_running(self) -> bool:
        with self._lock:
            return any(runtime.status.state == "running" for runtime in self._runtimes.values())

    @property
    def status(self) -> dict[str, CameraStatus]:
        """Return status snapshots, safe to serialize via ``to_dict``."""
        with self._lock:
            return {
                name: CameraStatus(**runtime.status.to_dict())
                for name, runtime in self._runtimes.items()
            }

    def start(self) -> dict[str, CameraStatus]:
        """Start one bag recorder and frame-monitor worker per configured camera."""
        with self._lock:
            if self._started:
                raise RuntimeError("RealSense capture can only be started once")
            self._started = True
        rs = _load_sdk()
        self._rs = rs
        self.output_dir.mkdir(parents=True, exist_ok=True)

        try:
            devices = rs.context().query_devices()
            available = {
                device.get_info(rs.camera_info.serial_number): device
                for device in devices
            }
            missing = [spec.serial for spec in self._specs if spec.serial not in available]
            if missing:
                raise RealSenseUnavailableError(
                    "Configured RealSense serial(s) not found: {}. Detected: {}. "
                    "On macOS, run the recording process with USB access (often sudo).".format(
                        ", ".join(missing), ", ".join(available) or "none"
                    )
                )

            for spec in self._specs:
                runtime = self._make_runtime(spec)
                self._runtimes[spec.name] = runtime
                if (Path(runtime.status.bag_path).exists()
                        or Path(runtime.status.timestamps_path).exists()):
                    raise RealSenseCaptureError(
                        "Refusing to overwrite existing RealSense recording: {} or {}".format(
                            runtime.status.bag_path,
                            runtime.status.timestamps_path,
                        )
                    )
                runtime.timestamps_file = open(
                    runtime.status.timestamps_path,
                    "x",
                    encoding="utf-8",
                    buffering=1,
                )
                config = rs.config()
                config.enable_device(spec.serial)
                config.enable_stream(
                    rs.stream.color,
                    self.width,
                    self.height,
                    rs.format.rgb8,
                    self.fps,
                )
                config.enable_record_to_file(runtime.status.bag_path)
                try:
                    profile = runtime.pipeline.start(config)
                    device = profile.get_device()
                    if device.supports(rs.camera_info.name):
                        runtime.status.model = device.get_info(rs.camera_info.name)
                    color = profile.get_stream(rs.stream.color).as_video_stream_profile()
                    runtime.status.width = color.width()
                    runtime.status.height = color.height()
                    runtime.status.fps = color.fps()
                    runtime.status.state = "running"
                    runtime.status.started_monotonic_ns = time.monotonic_ns()
                except Exception as exc:
                    raise self._start_error(spec, exc) from exc

                runtime.worker = threading.Thread(
                    target=self._consume_frames,
                    args=(runtime,),
                    name="realsense-{}".format(spec.name),
                    daemon=True,
                )
                runtime.worker.start()
        except Exception:
            self.stop()
            raise
        return self.status

    def _make_runtime(self, spec: CameraSpec) -> _CameraRuntime:
        status = CameraStatus(
            name=spec.name,
            serial=spec.serial,
            bag_path=str(self.output_dir / (spec.name + ".bag")),
            timestamps_path=str(self.output_dir / (spec.name + ".timestamps.jsonl")),
            width=self.width,
            height=self.height,
            fps=self.fps,
        )
        return _CameraRuntime(spec=spec, status=status, pipeline=self._rs.pipeline())

    @staticmethod
    def _start_error(spec: CameraSpec, exc: Exception) -> RealSenseCaptureError:
        return RealSenseCaptureError(
            "Could not start RealSense camera {!r} (serial {}). Check its supported "
            "RGB profile, USB bandwidth, SDK permissions, and whether another "
            "application is using it: {}".format(spec.name, spec.serial, exc)
        )

    def _consume_frames(self, runtime: _CameraRuntime) -> None:
        last_number: int | None = None
        while not self._stop_event.is_set():
            try:
                frames = runtime.pipeline.wait_for_frames(self.wait_timeout_ms)
                # Capture this before retrieving frame metadata: it is the host
                # monotonic time at which the SDK handed the frameset to us.
                host_receive_monotonic_ns = time.monotonic_ns()
                color = frames.get_color_frame()
                if not color:
                    continue
                frame_number = int(color.get_frame_number())
                timestamp_ms = float(color.get_timestamp())
                domain = _enum_name(color.get_frame_timestamp_domain())
                self._write_timestamp(
                    runtime,
                    frame_number=frame_number,
                    timestamp_ms=timestamp_ms,
                    timestamp_domain=domain,
                    host_receive_monotonic_ns=host_receive_monotonic_ns,
                )
                with self._lock:
                    status = runtime.status
                    if last_number is not None and frame_number > last_number + 1:
                        status.dropped_frames += frame_number - last_number - 1
                    status.frame_count += 1
                    status.last_frame_number = frame_number
                    status.last_timestamp_ms = timestamp_ms
                    status.timestamp_domain = domain
                last_number = frame_number
            except Exception as exc:
                if self._stop_event.is_set() or self._stopping:
                    break
                with self._lock:
                    runtime.status.state = "error"
                    runtime.status.error = str(exc)
                self._stop_event.set()
                break

        with self._lock:
            if runtime.status.state == "running" and self._stopping:
                runtime.status.state = "stopped"

    def _write_timestamp(
        self,
        runtime: _CameraRuntime,
        *,
        frame_number: int,
        timestamp_ms: float,
        timestamp_domain: str,
        host_receive_monotonic_ns: int,
    ) -> None:
        record = {
            "frame_number": frame_number,
            "timestamp_ms": timestamp_ms,
            "timestamp_domain": timestamp_domain,
            "host_receive_monotonic_ns": host_receive_monotonic_ns,
        }
        try:
            if runtime.timestamps_file is None:
                raise OSError("timestamp sidecar is not open")
            runtime.timestamps_file.write(
                json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n"
            )
            with self._lock:
                runtime.status.timestamp_rows_written += 1
        except Exception as exc:
            detail = "timestamp sidecar write failed: {}".format(exc)
            with self._lock:
                runtime.status.timestamps_error = detail
                runtime.status.error = detail
                runtime.status.state = "error"
            raise RealSenseCaptureError(detail) from exc

    def raise_if_failed(self) -> None:
        """Raise a concise error if any camera worker stopped unexpectedly."""
        with self._lock:
            failures = [
                "{} ({}): {}".format(name, status.serial, status.error or status.state)
                for name, status in ((n, r.status) for n, r in self._runtimes.items())
                if status.state == "error"
            ]
        if failures:
            raise RealSenseCaptureError(
                "RealSense capture failed: " + "; ".join(failures)
            )

    def stop(self) -> dict[str, CameraStatus]:
        """Stop all pipelines, flush native bag files, and return final status."""
        with self._lock:
            if not self._runtimes:
                return self.status
            self._stopping = True
            self._stop_event.set()
            runtimes = list(self._runtimes.values())

        stop_errors: list[str] = []
        for runtime in runtimes:
            if runtime.pipeline is None:
                continue
            try:
                runtime.pipeline.stop()
            except Exception as exc:
                if runtime.status.state not in {"error", "stopped"}:
                    stop_errors.append("{}: {}".format(runtime.spec.name, exc))

        # Mark the capture boundary once every pipeline has received stop,
        # before worker joins and timestamp-sidecar fsync add shutdown latency.
        stopped_at = time.monotonic_ns()

        for runtime in runtimes:
            worker = runtime.worker
            if worker is not None and worker.is_alive():
                worker.join(timeout=max(2.0, self.wait_timeout_ms / 1000.0 + 1.0))
                if worker.is_alive():
                    stop_errors.append("{} capture worker did not stop".format(runtime.spec.name))

        for runtime in runtimes:
            stream = runtime.timestamps_file
            if stream is None:
                continue
            try:
                stream.flush()
                os.fsync(stream.fileno())
            except Exception as exc:
                detail = "timestamp sidecar flush/fsync failed: {}".format(exc)
                runtime.status.timestamps_error = detail
                runtime.status.error = detail
                runtime.status.state = "error"
                stop_errors.append("{}: {}".format(runtime.spec.name, detail))
            finally:
                try:
                    stream.close()
                except Exception as exc:
                    detail = "timestamp sidecar close failed: {}".format(exc)
                    runtime.status.timestamps_error = detail
                    runtime.status.error = detail
                    runtime.status.state = "error"
                    stop_errors.append("{}: {}".format(runtime.spec.name, detail))
                runtime.timestamps_file = None

        with self._lock:
            for runtime in runtimes:
                runtime.status.stopped_monotonic_ns = stopped_at
                if runtime.status.started_monotonic_ns is not None:
                    elapsed_s = (
                        stopped_at - runtime.status.started_monotonic_ns
                    ) / 1_000_000_000
                    if elapsed_s > 0:
                        runtime.status.observed_fps = (
                            runtime.status.frame_count / elapsed_s
                        )
                if runtime.status.state == "running":
                    runtime.status.state = "stopped"
                if runtime.status.state == "configured":
                    runtime.status.state = "not_started"
                if stop_errors and runtime.status.state != "error":
                    runtime.status.error = "; ".join(stop_errors)
        return self.status

    def close(self) -> dict[str, CameraStatus]:
        """Alias for :meth:`stop`."""
        return self.stop()

    def __enter__(self) -> "MultiRealSenseCapture":
        self.start()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.stop()
