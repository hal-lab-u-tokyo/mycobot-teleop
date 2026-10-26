"""Optional camera capture adapters for data collection."""

from .realsense import (
    CameraStatus,
    CameraSpec,
    MultiRealSenseCapture,
    RealSenseCaptureError,
    RealSenseUnavailableError,
)

__all__ = [
    "CameraSpec",
    "CameraStatus",
    "MultiRealSenseCapture",
    "RealSenseCaptureError",
    "RealSenseUnavailableError",
]
