"""RealSense D435 capture wrapper.

Owns the pyrealsense2 pipeline for a single color stream (cam_0, the head
camera). Implements the ``CameraStream`` Protocol directly so it can be
handed to any consumer (``FrameSender``, loggers) without an adapter.
"""

from __future__ import annotations

import logging
import threading
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


class RealmanCamera:
    """Single-color-stream RealSense capture; implements ``CameraStream``.

    Picks the first non-"Platform Camera" RealSense device found, mirroring
    the discovery pattern used by the legacy realman_robot_controller.
    """

    def __init__(
        self,
        name: str = "head",
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        serial: Optional[str] = None,
    ) -> None:
        self.name = name
        self.width = width
        self.height = height
        self.fps = fps
        self.serial = serial

        self._pipeline = None
        self._lock = threading.Lock()
        self._started = False

    def start(self) -> None:
        """Open the camera and start streaming. Idempotent."""
        with self._lock:
            if self._started:
                return

            import pyrealsense2 as rs

            ctx = rs.context()
            chosen_serial = self.serial
            if chosen_serial is None:
                for d in ctx.devices:
                    name = d.get_info(rs.camera_info.name)
                    sn = d.get_info(rs.camera_info.serial_number)
                    logger.info("RealSense device found: %s (%s)", name, sn)
                    if name.lower() != "platform camera":
                        chosen_serial = sn
                        break

            if chosen_serial is None:
                raise RuntimeError("No RealSense color camera detected")

            cfg = rs.config()
            cfg.enable_device(chosen_serial)
            cfg.enable_stream(
                rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps
            )

            pipeline = rs.pipeline()
            try:
                pipeline.start(cfg)
            except RuntimeError as exc:
                if "power state" in str(exc).lower():
                    raise RuntimeError(
                        "RealSense failed to set power state (often another process "
                        "still holds the camera — e.g. teleop stopped with Ctrl+Z). "
                        "Try: kill those Python jobs, or unplug/replug the USB cable."
                    ) from exc
                raise
            self._pipeline = pipeline
            self.serial = chosen_serial
            self._started = True
            logger.info(
                "RealSense cam_0 started: %s @ %dx%d %d fps",
                chosen_serial, self.width, self.height, self.fps,
            )

    def get_frame(self, timeout_ms: int = 1000) -> Optional[np.ndarray]:
        """Block briefly for the next color frame. Returns BGR ndarray or None."""
        if not self._started or self._pipeline is None:
            return None
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=timeout_ms)
        except Exception as exc:
            logger.warning("RealSense wait_for_frames failed: %s", exc)
            return None
        color = frames.get_color_frame()
        if not color:
            return None
        return np.asanyarray(color.get_data()).copy()

    def stop(self) -> None:
        with self._lock:
            if self._pipeline is not None:
                try:
                    self._pipeline.stop()
                except Exception as exc:
                    logger.warning("RealSense pipeline.stop failed: %s", exc)
                self._pipeline = None
            self._started = False
