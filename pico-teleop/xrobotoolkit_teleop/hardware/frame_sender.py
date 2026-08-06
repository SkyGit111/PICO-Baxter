"""Camera -> Unity headset H264 streamer.

Implements the same wire protocol as the upstream
`XRoboToolkit-Orin-Video-Sender` so that the existing Unity
``RemoteCameraWindow`` / ``UICameraCtrl`` "Listen" flow works without any
Unity-side changes.

Camera-agnostic: takes any ``CameraStream`` (see ``core/camera.py``). The
camera owns its width/height/fps; ``FrameSender`` reads those off it and
configures the H264 encoder to match — no resize, no duplicated config.

Wire protocol (we are the SERVER the headset connects to):

1. Headset connects to ``<our_ip>:<our_port>`` (the IP/port the user types
   into the Unity Listen dialog).
2. Headset sends one wrapped packet:

       [4-byte BE outer-length]
       [int32 LE command_length]
       [command bytes]                 # b"OPEN_CAMERA" or b"CLOSE_CAMERA"
       [int32 LE data_length]
       [data bytes]                    # CameraRequestData (only for OPEN_CAMERA)

   ``CameraRequestData`` layout (from
   ``XRoboToolkit-Unity-Client/.../CameraRequestSerializer.cs``):

       [0xCA][0xFE]                    # magic
       [u8 version=1]
       [int32 LE width]
       [int32 LE height]
       [int32 LE fps]
       [int32 LE bitrate]
       [int32 LE enableMvHevc]
       [int32 LE renderMode]
       [int32 LE port]                 # headset's listening port for video
       [u8 camera_len][camera bytes]   # e.g. "ZED" / "VR"
       [u8 ip_len][ip bytes]           # headset's local IP

3. We open a SECOND TCP socket out to ``<headset_ip>:<headset_port>``
   (extracted from CameraRequestData) and stream H264. Each access unit:

       [4-byte BE length][N bytes Annex-B H264]

4. ``CLOSE_CAMERA`` (or socket close) tears down the egress and we go
   back to listening.

This module is intentionally hardware-agnostic at its boundaries: pass
in any ``CameraStream``.
"""

from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from fractions import Fraction
from typing import Optional, Tuple

from xrobotoolkit_teleop.core.camera import CameraStream

logger = logging.getLogger(__name__)

DEFAULT_LISTEN_PORT = 13579

CAMERA_REQUEST_MAGIC = b"\xCA\xFE"
CAMERA_REQUEST_VERSION = 1


class _DialogClosed(Exception):
    """Headset closed the control socket."""


class FrameSender:
    """TCP server that H264-encodes a ``CameraStream`` and sends it to a headset."""

    def __init__(
        self,
        camera: CameraStream,
        listen_host: str = "0.0.0.0",
        listen_port: int = DEFAULT_LISTEN_PORT,
        bitrate_bps: int = 10 * 1024 * 1024,
        gop_size: int = 30,
    ) -> None:
        self.camera = camera
        self.listen_host = listen_host
        self.listen_port = listen_port
        self.width = camera.width
        self.height = camera.height
        self.fps = camera.fps
        self.bitrate_bps = bitrate_bps
        self.gop_size = gop_size

        self._stop_event = threading.Event()
        self._accept_thread: Optional[threading.Thread] = None
        self._stream_thread: Optional[threading.Thread] = None
        self._listen_sock: Optional[socket.socket] = None
        self._control_sock: Optional[socket.socket] = None
        self._egress_sock: Optional[socket.socket] = None
        self._egress_lock = threading.Lock()
        self._streaming = threading.Event()
        self._encoder = None
        self._av = None
        self._frame_index = 0

    # ----------------------------- lifecycle -----------------------------

    def start(self) -> None:
        if self._accept_thread is not None and self._accept_thread.is_alive():
            return
        # camera.start() is idempotent; safe to call even if owner already started it.
        self.camera.start()
        self._init_encoder()
        self._stop_event.clear()
        self._open_listen_socket()
        self._accept_thread = threading.Thread(
            target=self._accept_loop, name="FrameSenderAccept", daemon=True
        )
        self._accept_thread.start()
        logger.info(
            "FrameSender listening on %s:%d (waiting for headset)",
            self.listen_host, self.listen_port,
        )

    def stop(self) -> None:
        # We do not stop the camera — the caller owns its lifecycle (the camera
        # may be shared with other consumers, e.g. data loggers).
        self._stop_event.set()
        self._streaming.clear()
        self._close_control_sock()
        self._close_egress_sock()
        if self._listen_sock is not None:
            try:
                self._listen_sock.close()
            except OSError:
                pass
            self._listen_sock = None
        if self._stream_thread is not None:
            self._stream_thread.join(timeout=2.0)
            self._stream_thread = None
        if self._accept_thread is not None:
            self._accept_thread.join(timeout=2.0)
            self._accept_thread = None

    # ----------------------------- encoding ------------------------------

    def _init_encoder(self) -> None:
        import av

        self._av = av
        codec = av.CodecContext.create("libx264", "w")
        codec.width = self.width
        codec.height = self.height
        codec.pix_fmt = "yuv420p"
        # PyAV 17+ removed av.Rational; fractions.Fraction is accepted here.
        codec.time_base = Fraction(1, self.fps)
        codec.framerate = Fraction(self.fps, 1)
        codec.bit_rate = self.bitrate_bps
        codec.gop_size = self.gop_size
        codec.max_b_frames = 0
        codec.options = {
            "preset": "ultrafast",
            "tune": "zerolatency",
            "profile": "baseline",
            "x264-params": "annexb=1:repeat-headers=1:keyint=%d:min-keyint=%d"
            % (self.gop_size, self.gop_size),
        }
        self._encoder = codec
        self._frame_index = 0
        logger.info(
            "H264 encoder initialised: %dx%d @ %d fps, %d bps",
            self.width, self.height, self.fps, self.bitrate_bps,
        )

    # ----------------------------- networking ----------------------------

    def _open_listen_socket(self) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.listen_host, self.listen_port))
        s.listen(1)
        self._listen_sock = s

    def _accept_loop(self) -> None:
        assert self._listen_sock is not None
        self._listen_sock.settimeout(1.0)
        while not self._stop_event.is_set():
            try:
                conn, addr = self._listen_sock.accept()
            except socket.timeout:
                continue
            except OSError as exc:
                if self._stop_event.is_set():
                    return
                logger.warning("accept failed: %s", exc)
                time.sleep(0.5)
                continue
            logger.info("headset control connection from %s", addr)
            self._control_sock = conn
            try:
                self._handle_control(conn)
            except _DialogClosed:
                logger.info("headset closed control connection")
            except Exception:
                logger.exception("control handler crashed")
            finally:
                self._streaming.clear()
                self._close_egress_sock()
                self._close_control_sock()
                # Wait for any in-flight stream thread to wind down before
                # we accept again.
                if self._stream_thread is not None:
                    self._stream_thread.join(timeout=2.0)
                    self._stream_thread = None

    def _handle_control(self, conn: socket.socket) -> None:
        conn.settimeout(None)
        while not self._stop_event.is_set():
            outer = _read_exact(conn, 4)
            (outer_len,) = struct.unpack(">I", outer)
            if outer_len == 0 or outer_len > 64 * 1024:
                logger.warning("bogus control frame length %d, dropping peer", outer_len)
                return
            body = _read_exact(conn, outer_len)
            try:
                command, data = _parse_network_data_protocol(body)
            except ValueError as exc:
                logger.warning("malformed NetworkDataProtocol: %s", exc)
                continue

            logger.info("received command %r (%d bytes)", command, len(data))

            if command == "OPEN_CAMERA":
                self._handle_open_camera(data)
            elif command == "CLOSE_CAMERA":
                self._handle_close_camera()
            else:
                logger.info("ignoring unknown command %r", command)

    def _handle_open_camera(self, data: bytes) -> None:
        try:
            req = _parse_camera_request(data)
        except ValueError as exc:
            logger.warning("malformed CameraRequestData: %s", exc)
            return
        logger.info(
            "OPEN_CAMERA from headset: %dx%d @ %d fps, bitrate=%d, "
            "camera=%r, target=%s:%d",
            req["width"], req["height"], req["fps"], req["bitrate"],
            req["camera"], req["ip"], req["port"],
        )

        self._streaming.clear()
        self._close_egress_sock()
        if self._stream_thread is not None:
            self._stream_thread.join(timeout=2.0)
            self._stream_thread = None

        if not _connect_egress(req["ip"], req["port"], egress_holder=self):
            return

        self._streaming.set()
        self._stream_thread = threading.Thread(
            target=self._stream_loop, name="FrameSenderStream", daemon=True
        )
        self._stream_thread.start()

    def _handle_close_camera(self) -> None:
        logger.info("CLOSE_CAMERA: tearing down egress")
        self._streaming.clear()
        self._close_egress_sock()
        if self._stream_thread is not None:
            self._stream_thread.join(timeout=2.0)
            self._stream_thread = None

    def _close_control_sock(self) -> None:
        if self._control_sock is not None:
            try:
                self._control_sock.close()
            except OSError:
                pass
            self._control_sock = None

    def _close_egress_sock(self) -> None:
        with self._egress_lock:
            if self._egress_sock is not None:
                try:
                    self._egress_sock.close()
                except OSError:
                    pass
                self._egress_sock = None

    # ----------------------------- streaming -----------------------------

    def _stream_loop(self) -> None:
        assert self._encoder is not None
        # Reset encoder state so the first sent NAL is an IDR.
        self._init_encoder()
        backoff = 0.05
        while self._streaming.is_set() and not self._stop_event.is_set():
            bgr = self.camera.get_frame()
            if bgr is None:
                continue
            try:
                packets = self._encode(bgr)
            except Exception as exc:
                logger.exception("H264 encode failed: %s", exc)
                continue
            for pkt_bytes in packets:
                if not pkt_bytes:
                    continue
                if not self._send_packet(pkt_bytes):
                    self._streaming.clear()
                    break
            else:
                continue
            time.sleep(backoff)
            break
        # Flush encoder
        try:
            for pkt in self._encoder.encode(None):
                self._send_packet(bytes(pkt))
        except Exception:
            pass
        logger.info("stream loop exiting")

    def _encode(self, bgr) -> list[bytes]:
        assert self._av is not None and self._encoder is not None
        frame = self._av.VideoFrame.from_ndarray(bgr, format="bgr24")
        frame = frame.reformat(format="yuv420p")
        frame.pts = self._frame_index
        self._frame_index += 1
        return [bytes(p) for p in self._encoder.encode(frame)]

    def _send_packet(self, data: bytes) -> bool:
        with self._egress_lock:
            sock = self._egress_sock
            if sock is None:
                return False
            try:
                header = struct.pack(">I", len(data))
                sock.sendall(header + data)
                return True
            except OSError as exc:
                logger.warning("egress send failed: %s — closing", exc)
                try:
                    sock.close()
                except OSError:
                    pass
                self._egress_sock = None
                return False


# ----------------------- module-level helpers -------------------------

def _read_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise _DialogClosed()
        buf.extend(chunk)
    return bytes(buf)


def _parse_network_data_protocol(body: bytes) -> Tuple[str, bytes]:
    if len(body) < 8:
        raise ValueError("body too small for NetworkDataProtocol header")
    cmd_len = int.from_bytes(body[0:4], "little", signed=True)
    if cmd_len < 0 or 4 + cmd_len + 4 > len(body):
        raise ValueError("invalid command length %d" % cmd_len)
    command = body[4 : 4 + cmd_len].split(b"\x00", 1)[0].decode("utf-8", "replace")
    off = 4 + cmd_len
    data_len = int.from_bytes(body[off : off + 4], "little", signed=True)
    off += 4
    if data_len < 0 or off + data_len > len(body):
        raise ValueError("invalid data length %d" % data_len)
    return command, bytes(body[off : off + data_len])


def _parse_camera_request(data: bytes) -> dict:
    if len(data) < 3 + 7 * 4 + 1 + 1:
        raise ValueError("CameraRequestData too small (%d bytes)" % len(data))
    if data[0:2] != CAMERA_REQUEST_MAGIC:
        raise ValueError("bad magic %r" % data[0:2])
    if data[2] != CAMERA_REQUEST_VERSION:
        raise ValueError("unsupported version %d" % data[2])
    off = 3
    ints = struct.unpack_from("<7i", data, off)
    off += 7 * 4
    width, height, fps, bitrate, enable_mv_hevc, render_mode, port = ints

    camera, off = _read_compact_string(data, off)
    ip, off = _read_compact_string(data, off)
    return {
        "width": width,
        "height": height,
        "fps": fps,
        "bitrate": bitrate,
        "enableMvHevc": enable_mv_hevc,
        "renderMode": render_mode,
        "port": port,
        "camera": camera,
        "ip": ip,
    }


def _read_compact_string(data: bytes, off: int) -> tuple[str, int]:
    if off >= len(data):
        raise ValueError("string length out of range")
    length = data[off]
    off += 1
    if length == 0:
        return "", off
    if off + length > len(data):
        raise ValueError("string content out of range")
    return data[off : off + length].decode("utf-8", "replace"), off + length


def _connect_egress(ip: str, port: int, *, egress_holder: "FrameSender") -> bool:
    try:
        s = socket.create_connection((ip, port), timeout=5.0)
        s.settimeout(None)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except OSError as exc:
        logger.warning("egress connect to %s:%d failed: %s", ip, port, exc)
        return False
    with egress_holder._egress_lock:
        if egress_holder._egress_sock is not None:
            try:
                egress_holder._egress_sock.close()
            except OSError:
                pass
        egress_holder._egress_sock = s
    logger.info("egress connected to headset %s:%d", ip, port)
    return True
