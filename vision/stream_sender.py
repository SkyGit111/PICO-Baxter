"""Low-latency framed H.264 TCP sender."""

from __future__ import annotations

import logging
import socket
import threading
import time
from typing import Optional

try:
    from .gst_pipeline import GstPipeline
    from .remote_vision_protocol import frame_video_buffer
except ImportError:  # Permit imports from a directly executed sibling module.
    from gst_pipeline import GstPipeline
    from remote_vision_protocol import frame_video_buffer


LOG = logging.getLogger("d455-vision")


class DirectVideoSender:
    """Pull encoded access units and send them to one TCP receiver."""

    def __init__(
        self,
        pipeline: GstPipeline,
        host: str,
        port: int,
        connect_timeout: float,
        send_timeout: float,
        reconnect_delay: float,
        send_buffer_bytes: int,
        manage_pipeline: bool = True,
    ) -> None:
        self.pipeline = pipeline
        self.host = host
        self.port = port
        self.connect_timeout = connect_timeout
        self.send_timeout = send_timeout
        self.reconnect_delay = reconnect_delay
        self.send_buffer_bytes = send_buffer_bytes
        self.manage_pipeline = manage_pipeline
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
        if self.manage_pipeline:
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
            if self.manage_pipeline:
                self.pipeline.stop()

    def _connect(self) -> Optional[socket.socket]:
        LOG.info("connecting to PICO video receiver %s:%d", self.host, self.port)
        sock: Optional[socket.socket] = None
        try:
            sock = socket.create_connection(
                (self.host, self.port), timeout=self.connect_timeout
            )
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.send_buffer_bytes)
            sock.settimeout(self.send_timeout)
        except OSError as exc:
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
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
