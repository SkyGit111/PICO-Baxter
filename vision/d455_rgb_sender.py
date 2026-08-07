#!/usr/bin/env python3
"""Send one D455 RGB stream to PICO Remote Vision over low-latency TCP."""

from __future__ import annotations

import argparse
import logging
import signal
import socket
import threading
import time
from typing import Optional

try:
    from .gst_pipeline import GstPipeline, PipelineConfig
    from .remote_vision_protocol import frame_video_buffer
except ImportError:  # Permit direct execution: python vision/d455_rgb_sender.py
    from gst_pipeline import GstPipeline, PipelineConfig
    from remote_vision_protocol import frame_video_buffer


LOG = logging.getLogger("d455-vision")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="D455 RGB to low-latency SBS H.264 PICO sender"
    )
    parser.add_argument(
        "--device",
        default="",
        help="Stable D455 RGB V4L2 path, preferably /dev/v4l/by-id/...",
    )
    parser.add_argument("--pico-ip", required=True)
    parser.add_argument("--pico-port", type=int, default=12345)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--bitrate-mbps", type=float, default=10.0)
    parser.add_argument("--key-interval", type=int, default=30)
    parser.add_argument(
        "--source-format",
        default=None,
        help="Optional V4L2 raw format such as YUY2; normally auto-negotiated",
    )
    parser.add_argument("--connect-timeout", type=float, default=3.0)
    parser.add_argument("--send-timeout", type=float, default=1.0)
    parser.add_argument("--reconnect-delay", type=float, default=1.0)
    parser.add_argument("--send-buffer-kib", type=int, default=64)
    parser.add_argument(
        "--test-source",
        action="store_true",
        help="Use a live SMPTE test pattern instead of opening a camera",
    )
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


class DirectVideoSender:
    def __init__(
        self,
        pipeline: GstPipeline,
        host: str,
        port: int,
        connect_timeout: float,
        send_timeout: float,
        reconnect_delay: float,
        send_buffer_bytes: int,
    ) -> None:
        self.pipeline = pipeline
        self.host = host
        self.port = port
        self.connect_timeout = connect_timeout
        self.send_timeout = send_timeout
        self.reconnect_delay = reconnect_delay
        self.send_buffer_bytes = send_buffer_bytes
        self.stop_event = threading.Event()
        self._socket: Optional[socket.socket] = None
        self.frames_sent = 0
        self.bytes_sent = 0
        self.pre_keyframe_drops = 0
        self.discontinuities = 0

    def stop(self) -> None:
        self.stop_event.set()
        self._close_socket()

    def run(self) -> None:
        self.pipeline.start()
        LOG.info("GStreamer capture and encoder started")
        try:
            while not self.stop_event.is_set():
                sock = self._connect()
                if sock is None:
                    self.stop_event.wait(self.reconnect_delay)
                    continue
                try:
                    self._stream_connection(sock)
                except (OSError, TimeoutError, EOFError) as exc:
                    if not self.stop_event.is_set():
                        LOG.warning("video connection lost: %s", exc)
                finally:
                    self._close_socket()
        finally:
            self.pipeline.stop()

    def _connect(self) -> Optional[socket.socket]:
        LOG.info("connecting to PICO video receiver %s:%d", self.host, self.port)
        try:
            sock = socket.create_connection(
                (self.host, self.port), timeout=self.connect_timeout
            )
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.send_buffer_bytes)
            sock.settimeout(self.send_timeout)
        except OSError as exc:
            LOG.warning("PICO connection failed: %s", exc)
            return None
        self._socket = sock
        LOG.info("connected; waiting for the next H.264 keyframe")
        return sock

    def _stream_connection(self, sock: socket.socket) -> None:
        waiting_for_keyframe = True
        previous_pts: Optional[int] = None
        expected_period_ns = int(1_000_000_000 / self.pipeline.config.fps)
        last_report = time.monotonic()
        report_frames = self.frames_sent
        while not self.stop_event.is_set():
            item = self.pipeline.pull_access_unit(timeout_ms=250)
            if item is None:
                continue
            payload, is_keyframe, pts = item
            if not payload:
                continue
            if (
                previous_pts is not None
                and pts is not None
                and pts - previous_pts > expected_period_ns * 3 // 2
            ):
                self.discontinuities += 1
                waiting_for_keyframe = True
                LOG.warning(
                    "encoded AU gap detected; dropping dependent frames until IDR"
                )
            previous_pts = pts
            if waiting_for_keyframe:
                if not is_keyframe:
                    self.pre_keyframe_drops += 1
                    continue
                waiting_for_keyframe = False
                LOG.info("sending from H.264 keyframe")

            framed = frame_video_buffer(payload)
            sock.sendall(framed)
            self.frames_sent += 1
            self.bytes_sent += len(framed)

            now = time.monotonic()
            if now - last_report >= 5.0:
                fps = (self.frames_sent - report_frames) / (now - last_report)
                LOG.info(
                    "video active: %.1f sent AU/s, total=%d, pre-IDR drops=%d, gaps=%d",
                    fps,
                    self.frames_sent,
                    self.pre_keyframe_drops,
                    self.discontinuities,
                )
                last_report = now
                report_frames = self.frames_sent

    def _close_socket(self) -> None:
        sock = self._socket
        self._socket = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass


def main() -> int:
    args = parse_args()
    if not 1 <= args.pico_port <= 65535:
        raise ValueError("--pico-port must be between 1 and 65535")
    if args.bitrate_mbps <= 0:
        raise ValueError("--bitrate-mbps must be positive")
    if args.connect_timeout <= 0 or args.send_timeout <= 0:
        raise ValueError("socket timeouts must be positive")
    if args.reconnect_delay < 0 or args.send_buffer_kib <= 0:
        raise ValueError("reconnect delay and send buffer must be valid")

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    config = PipelineConfig(
        device=args.device,
        width=args.width,
        height=args.height,
        fps=args.fps,
        bitrate_kbps=int(args.bitrate_mbps * 1024),
        key_interval=args.key_interval,
        source_format=args.source_format,
        test_source=args.test_source,
    )
    config.validate()
    sender = DirectVideoSender(
        pipeline=GstPipeline(config),
        host=args.pico_ip,
        port=args.pico_port,
        connect_timeout=args.connect_timeout,
        send_timeout=args.send_timeout,
        reconnect_delay=args.reconnect_delay,
        send_buffer_bytes=args.send_buffer_kib * 1024,
    )

    def request_stop(_signum: int, _frame: object) -> None:
        LOG.info("shutdown requested")
        sender.stop()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    LOG.info(
        "input=%s %dx%d@%d; output=%dx%d SBS; target=%s:%d",
        "videotestsrc" if args.test_source else args.device,
        args.width,
        args.height,
        args.fps,
        args.width * 2,
        args.height,
        args.pico_ip,
        args.pico_port,
    )
    try:
        sender.run()
    except KeyboardInterrupt:
        sender.stop()
    LOG.info("video sender stopped; sent %d access units", sender.frames_sent)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
