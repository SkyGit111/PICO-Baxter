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


class StreamBacklogError(TimeoutError):
    """The reliable TCP stream is retaining video that is no longer fresh."""


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
        max_send_ms: float = 250.0,
        tcp_notsent_lowat_bytes: int = 4096,
        dscp: int = 34,
    ) -> None:
        self.pipeline = pipeline
        self.host = host
        self.port = port
        self.connect_timeout = connect_timeout
        self.send_timeout = send_timeout
        self.reconnect_delay = reconnect_delay
        self.send_buffer_bytes = send_buffer_bytes
        self.manage_pipeline = manage_pipeline
        self.max_send_ms = max_send_ms
        self.tcp_notsent_lowat_bytes = tcp_notsent_lowat_bytes
        self.dscp = dscp
        self.stop_event = threading.Event()
        self._socket: Optional[socket.socket] = None
        self.frames_sent = 0
        self.bytes_sent = 0
        self.pre_keyframe_drops = 0
        self.discontinuities = 0
        self.slow_sends = 0
        self.reconnects = 0
        self.keyframe_requests = 0
        self.last_send_ms = 0.0
        self.maximum_send_ms = 0.0

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
                    if not self.stop_event.is_set():
                        self.reconnects += 1
                        self.stop_event.wait(self.reconnect_delay)
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
            if hasattr(socket, "TCP_NOTSENT_LOWAT"):
                try:
                    sock.setsockopt(
                        socket.IPPROTO_TCP,
                        socket.TCP_NOTSENT_LOWAT,
                        self.tcp_notsent_lowat_bytes,
                    )
                except OSError as exc:
                    LOG.warning("TCP_NOTSENT_LOWAT unavailable: %s", exc)
            if self.dscp > 0:
                try:
                    sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, self.dscp << 2)
                except OSError as exc:
                    LOG.warning("could not set video DSCP=%d: %s", self.dscp, exc)
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
        actual_send_buffer = sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
        LOG.info(
            "connected; kernel send buffer=%d bytes, TCP_NOTSENT_LOWAT=%d bytes, "
            "DSCP=%d; waiting for a fresh H.264 keyframe",
            actual_send_buffer,
            self.tcp_notsent_lowat_bytes,
            self.dscp,
        )
        return sock

    def _stream_connection(self, sock: socket.socket) -> None:
        waiting_for_keyframe = True
        previous_pts: Optional[int] = None
        expected_period_ns = int(1_000_000_000 / self.pipeline.config.fps)
        last_report = time.monotonic()
        report_frames = self.frames_sent
        self._request_keyframe("new video connection")
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
                self._request_keyframe("encoded AU gap")
            previous_pts = pts
            if waiting_for_keyframe:
                if not is_keyframe:
                    self.pre_keyframe_drops += 1
                    continue
                waiting_for_keyframe = False
                LOG.info("sending from H.264 keyframe")

            framed = frame_video_buffer(payload)
            send_started = time.monotonic()
            try:
                sock.sendall(framed)
            except TimeoutError as exc:
                self.last_send_ms = (time.monotonic() - send_started) * 1000.0
                self.maximum_send_ms = max(self.maximum_send_ms, self.last_send_ms)
                self.slow_sends += 1
                raise StreamBacklogError(
                    "H.264 AU send timed out after %.1f ms; closing the TCP "
                    "stream to discard any partially queued historical video"
                    % self.last_send_ms
                ) from exc
            self.last_send_ms = (time.monotonic() - send_started) * 1000.0
            self.maximum_send_ms = max(self.maximum_send_ms, self.last_send_ms)
            self.frames_sent += 1
            self.bytes_sent += len(framed)
            if self.max_send_ms > 0.0 and self.last_send_ms > self.max_send_ms:
                self.slow_sends += 1
                raise StreamBacklogError(
                    "one H.264 AU took %.1f ms to enqueue (limit %.1f ms); "
                    "closing the TCP stream instead of delivering old frames"
                    % (self.last_send_ms, self.max_send_ms)
                )

            now = time.monotonic()
            if now - last_report >= 5.0:
                fps = (self.frames_sent - report_frames) / (now - last_report)
                LOG.info(
                    "video active: %.1f sent AU/s, total=%d, pipeline-age=%s, "
                    "send=%.1fms max=%.1fms, pre-IDR drops=%d, gaps=%d, "
                    "slow-sends=%d, reconnects=%d",
                    fps,
                    self.frames_sent,
                    self._pipeline_age_text(),
                    self.last_send_ms,
                    self.maximum_send_ms,
                    self.pre_keyframe_drops,
                    self.discontinuities,
                    self.slow_sends,
                    self.reconnects,
                )
                last_report = now
                report_frames = self.frames_sent

    def _request_keyframe(self, reason: str) -> None:
        request = getattr(self.pipeline, "request_keyframe", None)
        if callable(request) and request():
            self.keyframe_requests += 1
            LOG.info("requested immediate H.264 keyframe: %s", reason)

    def _pipeline_age_text(self) -> str:
        age = getattr(self.pipeline, "last_access_unit_age_ms", None)
        return "unknown" if age is None else "%.1fms" % age

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
