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
    from .remote_vision_protocol import (
        CameraRequest,
        frame_video_buffer,
        parse_camera_request,
        read_control_message,
    )
except ImportError:  # Permit direct execution: python vision/d455_rgb_sender.py
    from gst_pipeline import GstPipeline, PipelineConfig
    from remote_vision_protocol import (
        CameraRequest,
        frame_video_buffer,
        parse_camera_request,
        read_control_message,
    )


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
    parser.add_argument(
        "--mode",
        choices=("direct", "listen"),
        default="direct",
        help="Directly connect to PICO, or accept Remote Vision control requests",
    )
    parser.add_argument("--pico-ip", default="")
    parser.add_argument("--pico-port", type=int, default=12345)
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=13579)
    parser.add_argument(
        "--allow-target-ip-mismatch",
        action="store_true",
        help="Allow OPEN_CAMERA to send video to an IP other than the control peer",
    )
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

        self.stop_event = threading.Event()
        self.ready_event = threading.Event()
        self.bound_port: Optional[int] = None
        self._listen_socket: Optional[socket.socket] = None
        self._control_socket: Optional[socket.socket] = None
        self._video_sender: Optional[DirectVideoSender] = None
        self._video_thread: Optional[threading.Thread] = None
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
            try:
                message = read_control_message(control)
            except socket.timeout:
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
            sender = self.sender_factory(
                pipeline=self.pipeline,
                host=host,
                port=port,
                connect_timeout=self.connect_timeout,
                send_timeout=self.send_timeout,
                reconnect_delay=self.reconnect_delay,
                send_buffer_bytes=self.send_buffer_bytes,
                manage_pipeline=False,
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

    @staticmethod
    def _run_video_sender(sender: DirectVideoSender) -> None:
        try:
            sender.run()
        except Exception:
            LOG.exception("Remote Vision video worker failed")

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


def main() -> int:
    args = parse_args()
    if args.mode == "direct" and not args.pico_ip:
        raise ValueError("--pico-ip is required in direct mode")
    if not 1 <= args.pico_port <= 65535:
        raise ValueError("--pico-port must be between 1 and 65535")
    if not 1 <= args.listen_port <= 65535:
        raise ValueError("--listen-port must be between 1 and 65535")
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
    pipeline = GstPipeline(config)
    if args.mode == "direct":
        service = DirectVideoSender(
            pipeline=pipeline,
            host=args.pico_ip,
            port=args.pico_port,
            connect_timeout=args.connect_timeout,
            send_timeout=args.send_timeout,
            reconnect_delay=args.reconnect_delay,
            send_buffer_bytes=args.send_buffer_kib * 1024,
        )
    else:
        service = RemoteVisionListener(
            pipeline=pipeline,
            listen_host=args.listen_host,
            listen_port=args.listen_port,
            connect_timeout=args.connect_timeout,
            send_timeout=args.send_timeout,
            reconnect_delay=args.reconnect_delay,
            send_buffer_bytes=args.send_buffer_kib * 1024,
            allow_target_ip_mismatch=args.allow_target_ip_mismatch,
        )

    def request_stop(_signum: int, _frame: object) -> None:
        LOG.info("shutdown requested")
        service.stop()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    LOG.info(
        "input=%s %dx%d@%d; output=%dx%d SBS; mode=%s",
        "videotestsrc" if args.test_source else args.device,
        args.width,
        args.height,
        args.fps,
        args.width * 2,
        args.height,
        args.mode,
    )
    if args.mode == "direct":
        LOG.info("direct video target=%s:%d", args.pico_ip, args.pico_port)
    else:
        LOG.info("control listener=%s:%d", args.listen_host, args.listen_port)
    try:
        service.run()
    except KeyboardInterrupt:
        service.stop()
    if isinstance(service, DirectVideoSender):
        LOG.info("video sender stopped; sent %d access units", service.frames_sent)
    else:
        LOG.info("Remote Vision listener stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
