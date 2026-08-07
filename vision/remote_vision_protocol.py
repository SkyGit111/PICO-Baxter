"""Wire helpers for PICO Remote Vision.

Video is transported as a TCP byte stream containing a four-byte unsigned
big-endian length followed immediately by one complete Annex-B H.264 access
unit.  The control-plane decoder mirrors the OPEN_CAMERA/CLOSE_CAMERA format
used by the upstream PICO Remote Vision implementation.
"""

from __future__ import annotations

import socket
import struct
from dataclasses import dataclass
from typing import Tuple


MAX_VIDEO_BUFFER_BYTES = 32 * 1024 * 1024
MAX_CONTROL_MESSAGE_BYTES = 64 * 1024
CAMERA_REQUEST_MAGIC = b"\xCA\xFE"
CAMERA_REQUEST_VERSION = 1


@dataclass(frozen=True)
class ControlMessage:
    command: str
    data: bytes


@dataclass(frozen=True)
class CameraRequest:
    width: int
    height: int
    fps: int
    bitrate: int
    enable_mv_hevc: int
    render_mode: int
    port: int
    camera: str
    ip: str


def frame_video_buffer(data: bytes) -> bytes:
    """Prefix one encoded H.264 buffer with its four-byte BE length."""
    size = len(data)
    if size <= 0:
        raise ValueError("video buffer must not be empty")
    if size > MAX_VIDEO_BUFFER_BYTES:
        raise ValueError("video buffer exceeds safety limit")
    return struct.pack(">I", size) + data


def recv_exact(sock: socket.socket, size: int) -> bytes:
    """Read exactly *size* bytes or raise EOFError on a closed peer."""
    if size < 0:
        raise ValueError("size must not be negative")
    result = bytearray()
    while len(result) < size:
        chunk = sock.recv(size - len(result))
        if not chunk:
            raise EOFError("peer closed the TCP connection")
        result.extend(chunk)
    return bytes(result)


def read_control_message(sock: socket.socket) -> ControlMessage:
    """Read one outer-length-framed Remote Vision control message."""
    outer_length = struct.unpack(">I", recv_exact(sock, 4))[0]
    if outer_length <= 0 or outer_length > MAX_CONTROL_MESSAGE_BYTES:
        raise ValueError("invalid control message length: %d" % outer_length)
    return parse_control_body(recv_exact(sock, outer_length))


def parse_control_body(body: bytes) -> ControlMessage:
    """Decode a NetworkDataProtocol body."""
    if len(body) < 8:
        raise ValueError("control body is too small")

    command_length = int.from_bytes(body[0:4], "little", signed=True)
    if command_length < 0 or 4 + command_length + 4 > len(body):
        raise ValueError("invalid command length: %d" % command_length)

    command_start = 4
    command_end = command_start + command_length
    command = body[command_start:command_end].split(b"\x00", 1)[0].decode(
        "utf-8", "replace"
    )

    data_length = int.from_bytes(
        body[command_end : command_end + 4], "little", signed=True
    )
    data_start = command_end + 4
    if data_length < 0 or data_start + data_length != len(body):
        raise ValueError("invalid data length: %d" % data_length)

    return ControlMessage(command=command, data=bytes(body[data_start:]))


def parse_camera_request(data: bytes) -> CameraRequest:
    """Decode the payload carried by an OPEN_CAMERA command."""
    minimum_size = 3 + 7 * 4 + 1 + 1
    if len(data) < minimum_size:
        raise ValueError("camera request is too small")
    if data[0:2] != CAMERA_REQUEST_MAGIC:
        raise ValueError("invalid camera request magic")
    if data[2] != CAMERA_REQUEST_VERSION:
        raise ValueError("unsupported camera request version: %d" % data[2])

    values = struct.unpack_from("<7i", data, 3)
    width, height, fps, bitrate, enable_mv_hevc, render_mode, port = values
    offset = 3 + 7 * 4
    camera, offset = _read_compact_string(data, offset)
    ip, offset = _read_compact_string(data, offset)
    if offset != len(data):
        raise ValueError("camera request contains trailing data")
    if width <= 0 or height <= 0 or fps <= 0:
        raise ValueError("camera dimensions and fps must be positive")
    if not 1 <= port <= 65535:
        raise ValueError("camera target port is out of range")

    return CameraRequest(
        width=width,
        height=height,
        fps=fps,
        bitrate=bitrate,
        enable_mv_hevc=enable_mv_hevc,
        render_mode=render_mode,
        port=port,
        camera=camera,
        ip=ip,
    )


def _read_compact_string(data: bytes, offset: int) -> Tuple[str, int]:
    if offset >= len(data):
        raise ValueError("string length is missing")
    length = data[offset]
    offset += 1
    end = offset + length
    if end > len(data):
        raise ValueError("string content is truncated")
    return data[offset:end].decode("utf-8", "replace"), end
