"""Unit tests for the wire-level frame codec."""

from __future__ import annotations

import struct

import pytest

from xrbt_service.frame_codec import (
    FRAME_END,
    HEAD_PC_TO_PICO,
    HEAD_PICO_TO_PC,
    FrameDecoder,
    encode,
)


def _hand_build(
    cmd: int, params: bytes, ts_ms: int, *, head: int = HEAD_PICO_TO_PC
) -> bytes:
    return (
        bytes((head, cmd))
        + struct.pack("<I", len(params))
        + params
        + struct.pack("<q", ts_ms)
        + bytes((FRAME_END,))
    )


class TestEncode:
    def test_connect_frame_layout(self) -> None:
        payload = b"SN12345|-1"
        packet = encode(0x19, payload, head=HEAD_PICO_TO_PC, ts_ms=1_700_000_000_000)

        assert packet[0] == HEAD_PICO_TO_PC
        assert packet[1] == 0x19
        assert struct.unpack_from("<I", packet, 2)[0] == len(payload)
        assert packet[6 : 6 + len(payload)] == payload
        ts_slice = packet[6 + len(payload) : 6 + len(payload) + 8]
        assert struct.unpack("<q", ts_slice)[0] == 1_700_000_000_000
        assert packet[-1] == FRAME_END

    def test_pc_to_pico_default_head(self) -> None:
        packet = encode(0x5F, b"{}", ts_ms=0)
        assert packet[0] == HEAD_PC_TO_PICO

    def test_empty_payload_still_has_ts_and_tail(self) -> None:
        packet = encode(0x23, b"", ts_ms=42)
        assert len(packet) == 1 + 1 + 4 + 0 + 8 + 1
        assert packet[0] == HEAD_PC_TO_PICO
        assert packet[1] == 0x23
        assert packet[-1] == FRAME_END

    def test_rejects_bad_cmd(self) -> None:
        with pytest.raises(ValueError):
            encode(0x1FF, b"", ts_ms=0)


class TestRoundTrip:
    def test_single_frame(self) -> None:
        frame_bytes = encode(0x6D, b"hello", head=HEAD_PICO_TO_PC, ts_ms=123)
        dec = FrameDecoder()
        dec.feed(frame_bytes)
        frames = list(dec.frames())
        assert len(frames) == 1
        assert frames[0].cmd == 0x6D
        assert frames[0].params == b"hello"
        assert frames[0].ts_ms == 123

    def test_multiple_frames_in_one_chunk(self) -> None:
        a = encode(0x19, b"SN-A|-1", head=HEAD_PICO_TO_PC, ts_ms=1)
        b = encode(0x6C, b"SN-A|1.0|v1.2.3", head=HEAD_PICO_TO_PC, ts_ms=2)
        c = encode(0x23, b"SN-A", head=HEAD_PICO_TO_PC, ts_ms=3)

        dec = FrameDecoder()
        dec.feed(a + b + c)
        frames = list(dec.frames())

        assert [f.cmd for f in frames] == [0x19, 0x6C, 0x23]
        assert [f.ts_ms for f in frames] == [1, 2, 3]
        assert [f.params for f in frames] == [
            b"SN-A|-1", b"SN-A|1.0|v1.2.3", b"SN-A",
        ]

    def test_split_across_arbitrary_boundaries(self) -> None:
        packet = encode(
            0x6D,
            b"{\"functionName\":\"Tracking\"}",
            head=HEAD_PICO_TO_PC,
            ts_ms=77,
        )
        dec = FrameDecoder()
        for i in range(len(packet)):
            dec.feed(packet[i : i + 1])
            if i < len(packet) - 1:
                assert list(dec.frames()) == []
        frames = list(dec.frames())
        assert len(frames) == 1
        assert frames[0].cmd == 0x6D

    def test_large_tracking_payload(self) -> None:
        body = b"x" * 50_000
        packet = encode(0x6D, body, head=HEAD_PICO_TO_PC, ts_ms=99)
        dec = FrameDecoder()
        dec.feed(packet)
        (f,) = list(dec.frames())
        assert f.cmd == 0x6D
        assert len(f.params) == 50_000


class TestResync:
    def test_skips_garbage_prefix(self) -> None:
        good = encode(0x23, b"SN", head=HEAD_PICO_TO_PC, ts_ms=0)
        dec = FrameDecoder()
        dec.feed(b"\x00\x01\x02" + good)
        (f,) = list(dec.frames())
        assert f.cmd == 0x23

    def test_bad_tail_byte_drops_frame(self) -> None:
        good = bytearray(
            encode(0x23, b"SN", head=HEAD_PICO_TO_PC, ts_ms=0)
        )
        good[-1] = 0x00
        dec = FrameDecoder()
        dec.feed(bytes(good))
        # Should resync past the bogus frame without blowing up.
        assert list(dec.frames()) == []

    def test_bad_frame_then_good_frame(self) -> None:
        bad = bytearray(encode(0x23, b"SN", head=HEAD_PICO_TO_PC, ts_ms=0))
        bad[-1] = 0x00
        good = encode(0x6D, b"ok", head=HEAD_PICO_TO_PC, ts_ms=1)
        dec = FrameDecoder()
        dec.feed(bytes(bad) + good)
        frames = list(dec.frames())
        assert len(frames) == 1
        assert frames[0].cmd == 0x6D
        assert frames[0].params == b"ok"


class TestDirectionalDecoders:
    def test_pc_to_pico_decoder_ignores_pico_to_pc_head(self) -> None:
        pc_msg = encode(0x5F, b"{}", head=HEAD_PC_TO_PICO, ts_ms=0)
        dec = FrameDecoder(expected_head=HEAD_PICO_TO_PC)
        dec.feed(pc_msg)
        assert list(dec.frames()) == []
