"""End-to-end upstream TCP test.

Boot ``UpstreamServer`` on a loopback port, speak the framed protocol
like the Unity client does, and confirm the shared :class:`DeviceRegistry`
reflects the resulting state.
"""

from __future__ import annotations

import asyncio
import json
import socket

import pytest

from xrbt_service.device_model import DeviceRegistry
from xrbt_service.frame_codec import HEAD_PICO_TO_PC, encode
from xrbt_service.upstream import (
    CMD_CONNECT,
    CMD_HEARTBEAT,
    CMD_SEND_VERSION,
    CMD_TO_CONTROLLER_FUNCTION,
    UpstreamServer,
)


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


@pytest.fixture
async def server():
    registry = DeviceRegistry()
    port = _free_port()
    srv = UpstreamServer(
        registry,
        host="127.0.0.1",
        port=port,
        heartbeat_timeout_s=30.0,
    )
    task = asyncio.create_task(srv.serve())
    for _ in range(50):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(0.2)
                s.connect(("127.0.0.1", port))
            break
        except OSError:
            await asyncio.sleep(0.02)
    else:
        task.cancel()
        raise RuntimeError("upstream server never came up")

    yield registry, srv, port

    task.cancel()
    try:
        await task
    except (asyncio.CancelledError, Exception):
        pass


def _pack(cmd: int, params: bytes, ts_ms: int = 0) -> bytes:
    return encode(cmd, params, head=HEAD_PICO_TO_PC, ts_ms=ts_ms)


async def _wait_for(pred, timeout: float = 2.0, interval: float = 0.02):
    """Poll the async ``pred`` (a coroutine function) until truthy."""
    async def _poll():
        while True:
            result = await pred()
            if result:
                return result
            await asyncio.sleep(interval)

    return await asyncio.wait_for(_poll(), timeout=timeout)


async def test_connect_register_and_version(server) -> None:
    registry, _srv, port = server
    reader, writer = await asyncio.open_connection("127.0.0.1", port)
    try:
        device_sn = "SN-UNIT-001"
        writer.write(_pack(CMD_CONNECT, f"{device_sn}|-1".encode()))
        writer.write(_pack(CMD_SEND_VERSION, f"{device_sn}|1.0|0.9.1".encode()))
        await writer.drain()

        async def _has_version():
            r = await registry.get(device_sn)
            if r is None:
                return None
            return r if r.app_version else None

        rec = await _wait_for(_has_version)
        assert rec.device_sn == device_sn
        assert rec.app_version == "0.9.1"
        assert rec.peer_addr[0] == "127.0.0.1"
    finally:
        writer.close()
        await writer.wait_closed()


async def test_tracking_payload_stored_verbatim(server) -> None:
    registry, _srv, port = server
    reader, writer = await asyncio.open_connection("127.0.0.1", port)
    try:
        device_sn = "SN-UNIT-002"
        writer.write(_pack(CMD_CONNECT, f"{device_sn}|-1".encode()))
        raw = json.dumps({
            "functionName": "Tracking",
            "value": json.dumps({
                "Head": {"pose": "0.1,0.2,0.3,0,0,0,1", "status": 3},
                "Controller": {
                    "left": {
                        "pose": "1,2,3,0,0,0,1",
                        "trigger": 0.5, "grip": 0.25,
                        "axisX": 0.1, "axisY": -0.2,
                        "axisClick": False,
                        "primaryButton": True, "secondaryButton": False,
                        "menuButton": False,
                    },
                },
            }),
        })
        writer.write(_pack(CMD_TO_CONTROLLER_FUNCTION, raw.encode("utf-8")))
        await writer.drain()

        async def _ready() -> object:
            r = await registry.get(device_sn)
            return r if (r is not None and r.raw_state_json) else None

        rec = await _wait_for(_ready)
        assert rec is not None
        assert rec.raw_state_json == raw
        assert rec.head.pose == (0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0)
        assert rec.head.status == 3
        assert rec.left_controller.pose == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
        assert rec.left_controller.trigger == 0.5
        assert rec.left_controller.primary_button is True
    finally:
        writer.close()
        await writer.wait_closed()


async def test_heartbeat_bumps_timestamp(server) -> None:
    registry, _srv, port = server
    reader, writer = await asyncio.open_connection("127.0.0.1", port)
    try:
        device_sn = "SN-UNIT-003"
        writer.write(_pack(CMD_CONNECT, f"{device_sn}|-1".encode()))
        await writer.drain()

        async def _present():
            return await registry.get(device_sn)

        rec = await _wait_for(_present)
        first_hb = rec.last_heartbeat

        await asyncio.sleep(0.05)
        writer.write(_pack(CMD_HEARTBEAT, device_sn.encode()))
        await writer.drain()

        async def _hb_moved() -> object:
            r = await registry.get(device_sn)
            if r is None:
                return None
            return r if r.last_heartbeat > first_hb else None

        rec2 = await _wait_for(_hb_moved)
        assert rec2.last_heartbeat > first_hb
    finally:
        writer.close()
        await writer.wait_closed()


async def test_disconnect_unregisters_device(server) -> None:
    registry, _srv, port = server
    reader, writer = await asyncio.open_connection("127.0.0.1", port)
    device_sn = "SN-UNIT-004"
    writer.write(_pack(CMD_CONNECT, f"{device_sn}|-1".encode()))
    await writer.drain()

    async def _present() -> object:
        return await registry.get(device_sn)

    assert await _wait_for(_present)

    writer.close()
    await writer.wait_closed()

    async def _absent() -> object:
        return (await registry.get(device_sn)) is None

    assert await _wait_for(_absent)
