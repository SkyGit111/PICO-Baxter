"""PICO Remote Vision OPEN_CAMERA/CLOSE_CAMERA control server."""

from __future__ import annotations

import logging
import socket
import threading
from typing import Optional

try:
    from .gst_pipeline import GstPipeline
    from .remote_vision_protocol import (
        CameraRequest,
        parse_camera_request,
        read_control_message,
    )
    from .stream_sender import DirectVideoSender
except ImportError:  # Permit imports from a directly executed sibling module.
    from gst_pipeline import GstPipeline
    from remote_vision_protocol import (
        CameraRequest,
        parse_camera_request,
        read_control_message,
    )
    from stream_sender import DirectVideoSender


LOG = logging.getLogger("d455-vision")


def resolve_video_target(
    request: CameraRequest,
    control_peer_ip: str,
    allow_target_ip_mismatch: bool,
) -> tuple[str, int]:
    """Choose a safe video target for an OPEN_CAMERA request."""
    requested_ip = request.ip.strip()
    target_ip = requested_ip or control_peer_ip
    if target_ip != control_peer_ip and not allow_target_ip_mismatch:
        raise ValueError(
            "OPEN_CAMERA target IP %s differs from control peer %s"
            % (target_ip, control_peer_ip)
        )
    return target_ip, request.port


class RemoteVisionListener:
    """Handle PICO Remote Vision control requests on TCP port 13579."""

    def __init__(
        self,
        pipeline: GstPipeline,
        listen_host: str,
        listen_port: int,
        connect_timeout: float,
        send_timeout: float,
        reconnect_delay: float,
        send_buffer_bytes: int,
        allow_target_ip_mismatch: bool = False,
        sender_factory=DirectVideoSender,
        max_send_ms: float = 250.0,
        tcp_notsent_lowat_bytes: int = 4096,
        dscp: int = 34,
    ) -> None:
        self.pipeline = pipeline
        self.listen_host = listen_host
        self.listen_port = listen_port
        self.connect_timeout = connect_timeout
        self.send_timeout = send_timeout
        self.reconnect_delay = reconnect_delay
        self.send_buffer_bytes = send_buffer_bytes
        self.allow_target_ip_mismatch = allow_target_ip_mismatch
        self.sender_factory = sender_factory
        self.max_send_ms = max_send_ms
        self.tcp_notsent_lowat_bytes = tcp_notsent_lowat_bytes
        self.dscp = dscp

        self.stop_event = threading.Event()
        self.ready_event = threading.Event()
        self.bound_port: Optional[int] = None
        self._listen_socket: Optional[socket.socket] = None
        self._control_socket: Optional[socket.socket] = None
        self._video_sender: Optional[DirectVideoSender] = None
        self._video_thread: Optional[threading.Thread] = None
        self._video_error: Optional[Exception] = None
        self._stream_lock = threading.RLock()

    def run(self) -> None:
        self.pipeline.start()
        LOG.info("GStreamer capture and encoder started")
        try:
            self._open_listener()
            while not self.stop_event.is_set():
                try:
                    control, address = self._listen_socket.accept()
                except socket.timeout:
                    self.pipeline.raise_on_bus_error()
                    continue
                except OSError:
                    if self.stop_event.is_set():
                        break
                    raise

                LOG.info("Remote Vision control connection from %s:%d", *address)
                self._control_socket = control
                control.settimeout(0.5)
                try:
                    self._handle_control_connection(control, address[0])
                finally:
                    self._stop_video_stream()
                    self._close_control_socket()
        finally:
            self.ready_event.set()
            self._stop_video_stream()
            self._close_control_socket()
            self._close_listen_socket()
            self.pipeline.stop()

    def stop(self) -> None:
        self.stop_event.set()
        self._close_control_socket()
        self._close_listen_socket()
        self._stop_video_stream()

    def _open_listener(self) -> None:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.listen_host, self.listen_port))
        listener.listen(1)
        listener.settimeout(0.5)
        self._listen_socket = listener
        self.bound_port = int(listener.getsockname()[1])
        self.ready_event.set()
        LOG.info(
            "Remote Vision control listener ready on %s:%d",
            self.listen_host,
            self.bound_port,
        )

    def _handle_control_connection(
        self, control: socket.socket, control_peer_ip: str
    ) -> None:
        while not self.stop_event.is_set():
            self._raise_video_worker_error()
            try:
                message = read_control_message(control)
            except socket.timeout:
                self.pipeline.raise_on_bus_error()
                self._raise_video_worker_error()
                continue
            except EOFError:
                LOG.info("Remote Vision control peer disconnected")
                return
            except OSError:
                if not self.stop_event.is_set():
                    LOG.info("Remote Vision control socket closed")
                return
            except ValueError as exc:
                LOG.warning("invalid Remote Vision control message: %s", exc)
                return

            LOG.info("Remote Vision command: %s", message.command)
            if message.command == "OPEN_CAMERA":
                try:
                    request = parse_camera_request(message.data)
                    target = resolve_video_target(
                        request,
                        control_peer_ip,
                        self.allow_target_ip_mismatch,
                    )
                except ValueError as exc:
                    LOG.warning("OPEN_CAMERA rejected: %s", exc)
                    continue
                self._log_request_compatibility(request)
                self._start_video_stream(*target)
            elif message.command == "CLOSE_CAMERA":
                self._stop_video_stream()
            else:
                LOG.warning("ignoring unknown Remote Vision command %r", message.command)

    def _log_request_compatibility(self, request: CameraRequest) -> None:
        output_width = self.pipeline.config.width * 2
        output_height = self.pipeline.config.height
        output_fps = self.pipeline.config.fps
        LOG.info(
            "OPEN_CAMERA requested camera=%r %dx%d@%d bitrate=%d render=%d "
            "target=%s:%d; configured output=%dx%d@%d",
            request.camera,
            request.width,
            request.height,
            request.fps,
            request.bitrate,
            request.render_mode,
            request.ip or "<control-peer>",
            request.port,
            output_width,
            output_height,
            output_fps,
        )
        if (request.width, request.height, request.fps) != (
            output_width,
            output_height,
            output_fps,
        ):
            LOG.warning(
                "PICO request differs from the fixed SBS output; "
                "the configured %dx%d@%d stream will be sent",
                output_width,
                output_height,
                output_fps,
            )

    def _start_video_stream(self, host: str, port: int) -> None:
        with self._stream_lock:
            self._stop_video_stream()
            self._raise_video_worker_error()
            sender = self.sender_factory(
                pipeline=self.pipeline,
                host=host,
                port=port,
                connect_timeout=self.connect_timeout,
                send_timeout=self.send_timeout,
                reconnect_delay=self.reconnect_delay,
                send_buffer_bytes=self.send_buffer_bytes,
                manage_pipeline=False,
                max_send_ms=self.max_send_ms,
                tcp_notsent_lowat_bytes=self.tcp_notsent_lowat_bytes,
                dscp=self.dscp,
            )
            thread = threading.Thread(
                target=self._run_video_sender,
                args=(sender,),
                name="RemoteVisionVideo",
                daemon=True,
            )
            self._video_sender = sender
            self._video_thread = thread
            thread.start()

    def _run_video_sender(self, sender: DirectVideoSender) -> None:
        try:
            sender.run()
        except Exception as exc:
            self._video_error = exc
            LOG.exception("Remote Vision video worker failed")

    def _raise_video_worker_error(self) -> None:
        error = self._video_error
        if error is not None:
            raise RuntimeError("Remote Vision video worker failed") from error

    def _stop_video_stream(self) -> None:
        with self._stream_lock:
            sender = self._video_sender
            thread = self._video_thread
            self._video_sender = None
            self._video_thread = None
            if sender is not None:
                sender.stop()
            if thread is not None and thread is not threading.current_thread():
                join_timeout = max(3.0, self.connect_timeout + 1.0)
                thread.join(timeout=join_timeout)
                if thread.is_alive():
                    raise RuntimeError(
                        "video worker did not stop within %.1f seconds" % join_timeout
                    )
            if sender is not None:
                LOG.info(
                    "video stream summary: AUs=%d bytes=%d slow-sends=%d "
                    "reconnects=%d gaps=%d keyframe-requests=%d max-send=%.1fms",
                    getattr(sender, "frames_sent", 0),
                    getattr(sender, "bytes_sent", 0),
                    getattr(sender, "slow_sends", 0),
                    getattr(sender, "reconnects", 0),
                    getattr(sender, "discontinuities", 0),
                    getattr(sender, "keyframe_requests", 0),
                    getattr(sender, "maximum_send_ms", 0.0),
                )

    def _close_control_socket(self) -> None:
        control = self._control_socket
        self._control_socket = None
        if control is not None:
            try:
                control.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                control.close()
            except OSError:
                pass

    def _close_listen_socket(self) -> None:
        listener = self._listen_socket
        self._listen_socket = None
        if listener is not None:
            try:
                listener.close()
            except OSError:
                pass
