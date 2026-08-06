"""Standalone RealSense -> Unity headset streaming test.

This is a Python re-implementation of the OrinVideoSender ``--listen``
flow for a single RealSense D435 color stream. Use it to verify the
end-to-end pipeline without the robot or XR teleop:

    python scripts/hardware/test_realsense_stream.py --listen-port 13579

Then in the Unity client on the headset:

  1. Camera panel -> select **ZEDMINI** (or any preset; we ignore it).
  2. Click **Listen**.
  3. In the dialog, enter ``<this PC's LAN IP>`` and the port above.
  4. Click Confirm. The camera window should populate within a couple
     of seconds.

Note: the ``XRoboToolkit-PC-Service`` is **not** in this video path. You
do not need it running for video — only for tracking/control.
"""

from __future__ import annotations

import logging
import signal
import time

import tyro

from xrobotoolkit_teleop.hardware.frame_sender import (
    DEFAULT_LISTEN_PORT,
    FrameSender,
)
from xrobotoolkit_teleop.hardware.realman_camera import RealmanCamera


def main(
    listen_host: str = "0.0.0.0",
    listen_port: int = DEFAULT_LISTEN_PORT,
    width: int = 1280,
    height: int = 720,
    fps: int = 30,
    bitrate_mbps: float = 10.0,
) -> None:
    """Stream RealSense cam_0 H264 frames to the headset."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    camera = RealmanCamera(name="head", width=width, height=height, fps=fps)
    sender = FrameSender(
        camera=camera,
        listen_host=listen_host,
        listen_port=listen_port,
        bitrate_bps=int(bitrate_mbps * 1024 * 1024),
    )

    print("=" * 60)
    print("RealSense cam_0 H264 sender (OrinVideoSender-compatible)")
    print("=" * 60)
    print(f"Listen:    {listen_host}:{listen_port}")
    print(f"Frame:     {width}x{height} @ {fps} fps")
    print(f"Bitrate:   {bitrate_mbps} Mbps")
    print("=" * 60)
    print("In Unity: Camera panel -> Listen -> enter <PC IP>:%d" % listen_port)

    stop = False

    def _on_signal(*_: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        sender.start()
        while not stop:
            time.sleep(0.5)
    finally:
        print("\nShutting down sender...")
        sender.stop()
        camera.stop()


if __name__ == "__main__":
    tyro.cli(main)
