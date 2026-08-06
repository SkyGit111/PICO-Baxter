"""Wire-level frame codec for the PICO/Unity <-> PC Service TCP link.

Frame layout (both directions, little-endian):

    +------+------+------------+----------------+---------------+------+
    | HEAD |  CMD | LEN (u32)  | PARAMS (LEN B) | TS_MS (i64)   | END  |
    +------+------+------------+----------------+---------------+------+
       1B     1B        4B            LEN B            8B          1B

Directions have *different* head bytes but share the tail byte:

    PICO -> PC  : HEAD = 0x3F, END = 0xA5   (``SEND_PACKET_HEAD`` in Unity)
    PC   -> PICO: HEAD = 0xCF, END = 0xA5   (``RECEIVE_PACKET_HEAD`` in Unity)

See ``XRoboToolkit-Unity-Client/Assets/Scripts/Network/PackageHandle.cs`` for
the canonical definition, and ``Business_global.h`` in the C++ service for
the PC-side constants.

This module exposes:

- ``encode(cmd, params, *, head=HEAD_PC_TO_PICO, ts_ms=None) -> bytes``
- ``FrameDecoder`` — stateful streaming decoder that consumes raw TCP bytes
  and yields ``(cmd, params, ts_ms)`` tuples. It tolerates garbage / re-syncs
  on the HEAD byte, just like the C++ service's ``dealClientPack`` loop.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Iterator, Optional

HEAD_PICO_TO_PC: int = 0x3F
HEAD_PC_TO_PICO: int = 0xCF
FRAME_END: int = 0xA5

FRAME_OVERHEAD: int = 1 + 1 + 4 + 8 + 1


def _now_ms() -> int:
    return int(time.time() * 1000)


def encode(
    cmd: int,
    params: bytes | bytearray | memoryview,
    *,
    head: int = HEAD_PC_TO_PICO,
    ts_ms: Optional[int] = None,
) -> bytes:
    """Pack ``params`` into a single framed packet.

    ``head`` defaults to the PC->PICO direction because this function is
    primarily used by ``downstream.py`` / ``upstream.py`` when writing
    control messages out to the headset. Unit tests use both directions.
    """
    if not 0 <= cmd <= 0xFF:
        raise ValueError(f"cmd must fit in a byte, got {cmd}")
    if not 0 <= head <= 0xFF:
        raise ValueError(f"head must fit in a byte, got {head}")

    body = bytes(params) if not isinstance(params, bytes) else params
    if ts_ms is None:
        ts_ms = _now_ms()

    return (
        bytes((head, cmd))
        + struct.pack("<I", len(body))
        + body
        + struct.pack("<q", ts_ms)
        + bytes((FRAME_END,))
    )


@dataclass
class Frame:
    cmd: int
    params: bytes
    ts_ms: int


class FrameDecoder:
    """Streaming decoder. Feed arbitrary chunks with :meth:`feed`, then
    iterate :meth:`frames` to drain decoded packets.

    Resyncs on bad HEAD bytes by discarding one byte at a time until a
    plausible HEAD is seen, mirroring the C++ ``NoPadding`` logic.
    """

    def __init__(self, *, expected_head: int = HEAD_PICO_TO_PC) -> None:
        self._expected_head = expected_head
        self._buf = bytearray()

    @property
    def buffer_size(self) -> int:
        return len(self._buf)

    def feed(self, chunk: bytes | bytearray | memoryview) -> None:
        if chunk:
            self._buf.extend(chunk)

    def frames(self) -> Iterator[Frame]:
        while True:
            frame = self._try_one()
            if frame is None:
                return
            yield frame

    def _try_one(self) -> Optional[Frame]:
        buf = self._buf
        while buf and buf[0] != self._expected_head:
            del buf[0]

        if len(buf) < 1 + 1 + 4:
            return None

        (length,) = struct.unpack_from("<I", buf, 2)

        total = 1 + 1 + 4 + length + 8 + 1
        if len(buf) < total:
            return None

        end_byte = buf[total - 1]
        if end_byte != FRAME_END:
            del buf[0]
            return self._try_one()

        cmd = buf[1]
        params = bytes(buf[6 : 6 + length])
        (ts_ms,) = struct.unpack_from("<q", buf, 6 + length)

        del buf[:total]
        return Frame(cmd=cmd, params=params, ts_ms=ts_ms)
