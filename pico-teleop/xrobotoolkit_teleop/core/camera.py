"""Camera streaming abstraction.

Robots expose cameras via ``RobotAdapter.cameras() -> dict[str, CameraStream]``.
Consumers (e.g. ``FrameSender`` for headset video, future loggers) pull frames
through this Protocol and never touch hardware-specific camera classes.

A ``CameraStream`` owns its own resolution and frame rate — consumers read
``width/height/fps`` off the camera rather than carrying their own copy. This
is the single source of truth for the stream's format.
"""

from __future__ import annotations

from typing import Optional, Protocol

import numpy as np


class CameraStream(Protocol):
    """Single-stream camera handle.

    Implementations return frames as ``np.ndarray`` in BGR channel order
    (matching pyrealsense's default and OpenCV's convention). Resolution and
    frame rate are fixed for the lifetime of the stream and exposed as
    attributes so downstream consumers (encoders, loggers) can configure
    themselves from the camera.
    """

    name: str
    width: int
    height: int
    fps: int

    def start(self) -> None:
        """Open the camera and begin streaming. Idempotent."""

    def stop(self) -> None:
        """Stop streaming and release resources. Idempotent."""

    def get_frame(self) -> Optional[np.ndarray]:
        """Return the latest BGR frame, or None if not yet available."""
