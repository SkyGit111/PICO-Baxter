"""End-to-end exercises of the ``downstream.DownstreamServer`` RPC surface.

These spin up a real ``grpc.aio`` server on a loopback port, use the
generated stub to subscribe, and assert the ``ServerFeedback`` messages
match what ``xrobotoolkit_sdk`` expects to see:

- ``name == "deviceStateJson"``
- ``devicestatejson.devid`` == our SN
- ``devicestatejson.statejson`` is the *raw* JSON payload, unchanged.

There's no real upstream TCP in this test; the ``UpstreamServer`` is
constructed but never bound. We drive state directly through the shared
:class:`DeviceRegistry`.
"""

from __future__ import annotations

import asyncio
import socket

import grpc
import pytest

from xrbt_service.device_model import DeviceRegistry
from xrbt_service.downstream import (
    FEEDBACK_NAME_DEVICE_STATE_JSON,
    DownstreamServer,
)
from xrbt_service.generated import PXREAService_pb2 as pb
from xrbt_service.generated import PXREAService_pb2_grpc as pb_grpc
from xrbt_service.upstream import UpstreamServer


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


@pytest.fixture
async def wired_server():
    registry = DeviceRegistry()
    # UpstreamServer constructed only to let DownstreamServer look up TCP
    # sockets by SN; we never call `serve()` on it in this test.
    upstream = UpstreamServer(registry, host="127.0.0.1", port=0)
    port = _free_port()
    server = DownstreamServer(registry, upstream, host="127.0.0.1", port=port)
    serve_task = asyncio.create_task(server.serve())

    # Wait briefly for the gRPC server to actually bind.
    for _ in range(50):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect(("127.0.0.1", port))
            break
        except OSError:
            await asyncio.sleep(0.02)
    else:
        serve_task.cancel()
        raise RuntimeError("downstream server never came up")

    yield registry, upstream, port

    await server.stop(grace=0.1)
    serve_task.cancel()
    try:
        await serve_task
    except (asyncio.CancelledError, Exception):
        pass


async def test_watch_receives_state_update(wired_server) -> None:
    registry, _, port = wired_server

    device_sn = "PA1234567890"
    raw_json = '{"functionName":"Tracking","value":"{\\"Head\\":{\\"pose\\":\\"0,0,0,0,0,0,1\\"}}"}'

    await registry.register(device_sn, ("10.0.0.5", 49152))

    async with grpc.aio.insecure_channel(f"127.0.0.1:{port}") as channel:
        stub = pb_grpc.EAServiceStub(channel)
        call = stub.WatchServerFeedback(pb.VRPid(pid=4242))

        # Give the server a tick to register the subscriber before we push.
        await asyncio.sleep(0.05)
        await registry.update_state_json(device_sn, raw_json)

        got: pb.ServerFeedback = await asyncio.wait_for(call.read(), timeout=2.0)
        # The first message may be the initial "deviceFind" snapshot; drain
        # until we see the statejson update.
        while got.name != FEEDBACK_NAME_DEVICE_STATE_JSON:
            got = await asyncio.wait_for(call.read(), timeout=2.0)

        assert got.name == FEEDBACK_NAME_DEVICE_STATE_JSON
        assert got.devicestatejson.devid == device_sn
        assert got.devicestatejson.statejson == raw_json

        call.cancel()


async def test_watch_gets_initial_snapshot(wired_server) -> None:
    registry, _, port = wired_server

    device_sn = "SN-SNAPSHOT"
    raw_json = '{"functionName":"Tracking","value":"{\\"Head\\":{\\"pose\\":\\"1,1,1,0,0,0,1\\"}}"}'

    # Populate state BEFORE any subscriber connects: a fresh subscriber
    # should still see the latest snapshot on connect, matching the C++
    # service's reconnect behaviour.
    await registry.register(device_sn, ("127.0.0.1", 1))
    await registry.update_state_json(device_sn, raw_json)

    async with grpc.aio.insecure_channel(f"127.0.0.1:{port}") as channel:
        stub = pb_grpc.EAServiceStub(channel)
        call = stub.WatchServerFeedback(pb.VRPid(pid=1))

        got = await asyncio.wait_for(call.read(), timeout=2.0)
        assert got.name == FEEDBACK_NAME_DEVICE_STATE_JSON
        assert got.devicestatejson.devid == device_sn
        assert got.devicestatejson.statejson == raw_json

        call.cancel()


async def test_device_control_json_on_unknown_device_returns_not_found(
    wired_server,
) -> None:
    _, _, port = wired_server

    async with grpc.aio.insecure_channel(f"127.0.0.1:{port}") as channel:
        stub = pb_grpc.EAServiceStub(channel)
        with pytest.raises(grpc.aio.AioRpcError) as excinfo:
            await stub.DeviceControlJson(
                pb.DeviceControlParameterJson(
                    devid="MISSING", parameter="{}"
                )
            )
        assert excinfo.value.code() == grpc.StatusCode.NOT_FOUND


async def test_send_beat_and_send_bytes_to_room_are_noops(wired_server) -> None:
    _, _, port = wired_server
    from google.protobuf import empty_pb2

    async with grpc.aio.insecure_channel(f"127.0.0.1:{port}") as channel:
        stub = pb_grpc.EAServiceStub(channel)
        await stub.SendBeat(empty_pb2.Empty())
        await stub.SendBytesToRoom(pb.RoomBytesInfo(content=b"ignored"))


async def test_cancel_server_feedback_is_accepted(wired_server) -> None:
    _, _, port = wired_server

    async with grpc.aio.insecure_channel(f"127.0.0.1:{port}") as channel:
        stub = pb_grpc.EAServiceStub(channel)
        await stub.CancelServerFeedback(pb.VRPid(pid=99))
