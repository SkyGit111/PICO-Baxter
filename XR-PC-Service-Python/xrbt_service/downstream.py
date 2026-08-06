"""gRPC server implementing :class:`EAService`.

This is the plane the Python consumer (``xrobotoolkit_sdk``) attaches to
via ``WatchServerFeedback``. Every pose/button update the upstream TCP
thread commits to the :class:`DeviceRegistry` wakes the per-subscriber
coroutines here, which then emit a fresh ``ServerFeedback`` to that
client.

Parity notes vs. the C++ service:

- ``ServerFeedback.name`` for a pose update is ``"deviceStateJson"``
  (verified against ``FeedbackController.cpp`` line 200).
- The ``statejson`` field carries the raw ``0x6D`` UTF-8 payload verbatim,
  i.e.  ``{"functionName":"Tracking","value":"..."}``. No reshaping.
- ``DeviceControlJson`` and ``SendBytesToDevice`` both translate to a
  ``0x5F CMD_COMMON_FUNCTION`` frame sent down the headset's upstream
  TCP socket. This is the same path v2 video signaling will use — no
  special case for JSON content.
"""

from __future__ import annotations

import asyncio
import logging
from typing import AsyncIterator, Optional

import grpc
from google.protobuf import empty_pb2

from .device_model import DeviceRegistry
from .generated import PXREAService_pb2 as pb
from .generated import PXREAService_pb2_grpc as pb_grpc
from .upstream import UpstreamServer

logger = logging.getLogger(__name__)

FEEDBACK_NAME_DEVICE_STATE_JSON = "deviceStateJson"
FEEDBACK_NAME_DEVICE_CONNECT = "deviceFind"
FEEDBACK_NAME_DEVICE_DISCONNECT = "deviceMissing"


def _build_state_feedback(device_sn: str, raw_json: str) -> pb.ServerFeedback:
    fb = pb.ServerFeedback()
    fb.name = FEEDBACK_NAME_DEVICE_STATE_JSON
    fb.devicestatejson.devid = device_sn
    fb.devicestatejson.statejson = raw_json
    return fb


def _build_device_find(device_sn: str) -> pb.ServerFeedback:
    fb = pb.ServerFeedback()
    fb.name = FEEDBACK_NAME_DEVICE_CONNECT
    fb.devid = device_sn
    return fb


class EAServiceImpl(pb_grpc.EAServiceServicer):
    """``asyncio`` implementation of ``EAService``.

    Holds a reference to both the shared :class:`DeviceRegistry` (to read
    state) and the :class:`UpstreamServer` (to resolve a SN -> TCP socket
    for ``DeviceControlJson`` / ``SendBytesToDevice``).
    """

    def __init__(
        self,
        registry: DeviceRegistry,
        upstream: UpstreamServer,
    ) -> None:
        self._registry = registry
        self._upstream = upstream

    # ------------------------- streaming RPC -----------------------------

    async def WatchServerFeedback(  # noqa: N802 — proto-defined name
        self,
        request: pb.VRPid,
        context: grpc.aio.ServicerContext,
    ) -> AsyncIterator[pb.ServerFeedback]:
        pid = request.pid
        logger.info("WatchServerFeedback subscribed (pid=%d)", pid)

        last_sent_ns: dict[str, int] = {}

        try:
            # Initial snapshot so fresh subscribers see *something* right
            # away, mirroring what happens when a client reconnects to the
            # C++ service with devices already registered.
            for rec in await self._registry.snapshot():
                if rec.raw_state_json:
                    yield _build_state_feedback(rec.device_sn, rec.raw_state_json)
                    last_sent_ns[rec.device_sn] = rec.last_update_ns
                else:
                    yield _build_device_find(rec.device_sn)

            while True:
                # ``changed`` is a Set+Clear broadcast event. Wait for the
                # next update, then flush any SNs that advanced since we
                # last forwarded them. When the peer cancels the RPC, the
                # aio runtime cancels this coroutine — we catch that below.
                await self._registry.changed.wait()

                for rec in await self._registry.snapshot():
                    if not rec.raw_state_json:
                        continue
                    last = last_sent_ns.get(rec.device_sn, 0)
                    if rec.last_update_ns <= last:
                        continue
                    yield _build_state_feedback(
                        rec.device_sn, rec.raw_state_json
                    )
                    last_sent_ns[rec.device_sn] = rec.last_update_ns
        except asyncio.CancelledError:
            raise
        except grpc.aio.AioRpcError:
            pass
        finally:
            logger.info("WatchServerFeedback unsubscribed (pid=%d)", pid)

    # ------------------------- unary RPCs --------------------------------

    async def DeviceControlJson(  # noqa: N802
        self,
        request: pb.DeviceControlParameterJson,
        context: grpc.aio.ServicerContext,
    ) -> empty_pb2.Empty:
        await self._forward_control_blob(
            request.devid, request.parameter.encode("utf-8"), context
        )
        return empty_pb2.Empty()

    async def SendBytesToDevice(  # noqa: N802
        self,
        request: pb.DeviceBytesInfo,
        context: grpc.aio.ServicerContext,
    ) -> empty_pb2.Empty:
        await self._forward_control_blob(request.devid, request.content, context)
        return empty_pb2.Empty()

    async def SendBytesToRoom(  # noqa: N802
        self,
        request: pb.RoomBytesInfo,
        context: grpc.aio.ServicerContext,
    ) -> empty_pb2.Empty:
        # v1 stub: fan-out-to-all is not a consumer-facing feature for the
        # teleop use case. Returning OK keeps the wire contract intact.
        logger.debug("SendBytesToRoom (%d bytes): no-op in v1", len(request.content))
        return empty_pb2.Empty()

    async def SendBeat(  # noqa: N802
        self,
        request: empty_pb2.Empty,
        context: grpc.aio.ServicerContext,
    ) -> empty_pb2.Empty:
        return empty_pb2.Empty()

    async def CancelServerFeedback(  # noqa: N802
        self,
        request: pb.VRPid,
        context: grpc.aio.ServicerContext,
    ) -> empty_pb2.Empty:
        # ``WatchServerFeedback`` already terminates when the client closes
        # its side of the stream; this explicit RPC is largely ceremonial.
        logger.info("CancelServerFeedback (pid=%d)", request.pid)
        return empty_pb2.Empty()

    # ---------------------------- helpers --------------------------------

    async def _forward_control_blob(
        self,
        devid: str,
        content: bytes,
        context: grpc.aio.ServicerContext,
    ) -> None:
        conn = self._upstream.get_connection(devid)
        if conn is None:
            logger.warning(
                "control blob targeted at unknown device %r (%d bytes)",
                devid, len(content),
            )
            await context.abort(
                grpc.StatusCode.NOT_FOUND, f"device {devid!r} not connected"
            )
            return
        await conn.send_common_function(content)


class DownstreamServer:
    def __init__(
        self,
        registry: DeviceRegistry,
        upstream: UpstreamServer,
        *,
        host: str = "127.0.0.1",
        port: int = 60061,
    ) -> None:
        self._registry = registry
        self._upstream = upstream
        self._host = host
        self._port = port
        self._server: Optional[grpc.aio.Server] = None

    async def serve(self) -> None:
        self._server = grpc.aio.server()
        pb_grpc.add_EAServiceServicer_to_server(
            EAServiceImpl(self._registry, self._upstream),
            self._server,
        )
        bind_spec = f"{self._host}:{self._port}"
        self._server.add_insecure_port(bind_spec)
        await self._server.start()
        logger.info("downstream gRPC listening on %s", bind_spec)

        try:
            await self._server.wait_for_termination()
        except asyncio.CancelledError:
            await self._server.stop(grace=1.0)
            raise

    async def stop(self, grace: float = 1.0) -> None:
        if self._server is not None:
            await self._server.stop(grace=grace)
