#!/usr/bin/env python3
"""Mock the PICO video receiver and Remote Vision control client."""

from __future__ import annotations

import argparse
import logging
import socket
import threading
import time
from pathlib import Path
from typing import Optional

try:
    from .remote_vision_protocol import (
        CameraRequest,
        frame_control_message,
        read_video_buffer,
        serialize_camera_request,
    )
except ImportError:  # Permit direct execution.
    from remote_vision_protocol import (
        CameraRequest,
        frame_control_message,
        read_video_buffer,
        serialize_camera_request,
    )


LOG = logging.getLogger("mock-pico")


class VideoReceiver:
    """Receive and optionally save a finite number of framed H.264 AUs."""

    def __init__(
        self,
        listen_host: str,
        listen_port: int,
        frame_limit: int,
        output: Optional[Path] = None,
        slow_delay_s: float = 0.0,
        timeout_s: float = 10.0,
    ) -> None:
        self.listen_host = listen_host
        self.listen_port = listen_port
        self.frame_limit = frame_limit
        self.output = output
        self.slow_delay_s = slow_delay_s
        self.timeout_s = timeout_s
        self.ready_event = threading.Event()
        self.stop_event = threading.Event()
        self.bound_port: Optional[int] = None
        self.frames_received = 0
        self.bytes_received = 0
        self.error: Optional[Exception] = None
        self._server: Optional[socket.socket] = None
        self._connection: Optional[socket.socket] = None

    def run(self) -> None:
        output_file = None
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.listen_host, self.listen_port))
            server.listen(1)
            server.settimeout(self.timeout_s)
            self._server = server
            self.bound_port = int(server.getsockname()[1])
            self.ready_event.set()
            LOG.info("video receiver listening on %s:%d", self.listen_host, self.bound_port)

            connection, address = server.accept()
            self._connection = connection
            connection.settimeout(self.timeout_s)
            LOG.info("video sender connected from %s:%d", *address)
            if self.output is not None:
                output_file = self.output.open("wb")

            started = time.monotonic()
            while not self.stop_event.is_set() and (
                self.frame_limit <= 0 or self.frames_received < self.frame_limit
            ):
                payload = read_video_buffer(connection)
                if not payload.startswith((b"\x00\x00\x00\x01", b"\x00\x00\x01")):
                    raise ValueError("H.264 payload is not Annex-B byte stream data")
                if output_file is not None:
                    output_file.write(payload)
                self.frames_received += 1
                self.bytes_received += len(payload)
                if self.slow_delay_s > 0:
                    self.stop_event.wait(self.slow_delay_s)

            elapsed = max(time.monotonic() - started, 1e-9)
            LOG.info(
                "received %d AUs, %.2f MiB, %.1f AU/s",
                self.frames_received,
                self.bytes_received / (1024 * 1024),
                self.frames_received / elapsed,
            )
        except Exception as exc:
            if not self.stop_event.is_set():
                self.error = exc
                LOG.exception("mock video receiver failed")
        finally:
            self.ready_event.set()
            if output_file is not None:
                output_file.close()
            self.stop()

    def stop(self) -> None:
        self.stop_event.set()
        for sock in (self._connection, self._server):
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    sock.close()
                except OSError:
                    pass
        self._connection = None
        self._server = None


def discover_advertise_ip(control_host: str, control_port: int) -> str:
    """Return the local IPv4 address selected for the control-host route."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((control_host, control_port))
        return str(probe.getsockname()[0])
    finally:
        probe.close()


def run_control_client(args: argparse.Namespace) -> int:
    receiver = VideoReceiver(
        listen_host=args.video_listen_host,
        listen_port=args.video_port,
        frame_limit=args.frames,
        output=Path(args.output) if args.output else None,
        slow_delay_s=args.slow_ms / 1000.0,
        timeout_s=args.timeout,
    )
    receiver_thread = threading.Thread(target=receiver.run, name="MockVideoReceiver")
    receiver_thread.start()
    if not receiver.ready_event.wait(args.timeout) or receiver.bound_port is None:
        receiver.stop()
        receiver_thread.join()
        raise RuntimeError("mock video receiver did not become ready")

    advertise_ip = args.video_advertise_ip or discover_advertise_ip(
        args.control_host, args.control_port
    )
    request = CameraRequest(
        width=args.width,
        height=args.height,
        fps=args.fps,
        bitrate=int(args.bitrate_mbps * 1_000_000),
        enable_mv_hevc=0,
        render_mode=args.render_mode,
        port=receiver.bound_port,
        camera=args.camera,
        ip=advertise_ip,
    )

    try:
        control = socket.create_connection(
            (args.control_host, args.control_port), timeout=args.timeout
        )
    except Exception:
        receiver.stop()
        receiver_thread.join(timeout=2.0)
        raise
    control.settimeout(args.timeout)
    LOG.info(
        "sending OPEN_CAMERA to %s:%d with video target %s:%d",
        args.control_host,
        args.control_port,
        advertise_ip,
        receiver.bound_port,
    )
    try:
        control.sendall(
            frame_control_message("OPEN_CAMERA", serialize_camera_request(request))
        )
        receiver_thread.join(timeout=args.timeout + max(5.0, args.frames / 5.0))
        control.sendall(frame_control_message("CLOSE_CAMERA"))
    finally:
        receiver.stop()
        receiver_thread.join(timeout=2.0)
        control.close()

    if receiver_thread.is_alive():
        raise RuntimeError("mock video receiver did not stop")
    if receiver.error is not None:
        raise RuntimeError("video receiver failed") from receiver.error
    if args.frames > 0 and receiver.frames_received != args.frames:
        raise RuntimeError(
            "received %d of %d requested AUs"
            % (receiver.frames_received, args.frames)
        )
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", choices=("receive", "control"), default="receive"
    )
    parser.add_argument("--video-listen-host", default="0.0.0.0")
    parser.add_argument("--video-port", type=int, default=12345)
    parser.add_argument("--video-advertise-ip", default="")
    parser.add_argument("--control-host", default="127.0.0.1")
    parser.add_argument("--control-port", type=int, default=13579)
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument("--output", default="")
    parser.add_argument("--slow-ms", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--width", type=int, default=2560)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--bitrate-mbps", type=float, default=10.0)
    parser.add_argument("--render-mode", type=int, default=1)
    parser.add_argument("--camera", default="ZEDMINI")
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.frames <= 0:
        raise ValueError("--frames must be positive")
    if args.slow_ms < 0 or args.timeout <= 0:
        raise ValueError("slow delay must be non-negative and timeout positive")
    for value, name in (
        (args.video_port, "--video-port"),
        (args.control_port, "--control-port"),
    ):
        if not 0 <= value <= 65535:
            raise ValueError("%s must be between 0 and 65535" % name)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    if args.mode == "control":
        return run_control_client(args)

    receiver = VideoReceiver(
        listen_host=args.video_listen_host,
        listen_port=args.video_port,
        frame_limit=args.frames,
        output=Path(args.output) if args.output else None,
        slow_delay_s=args.slow_ms / 1000.0,
        timeout_s=args.timeout,
    )
    try:
        receiver.run()
    except KeyboardInterrupt:
        receiver.stop()
    if receiver.error is not None:
        raise RuntimeError("video receiver failed") from receiver.error
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
