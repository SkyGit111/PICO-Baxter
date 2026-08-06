"""Upstream TCP server.

Accepts framed packets from PICO headsets (Unity client) on
``0.0.0.0:63901``, dispatches them by CMD byte, and updates a shared
:class:`DeviceRegistry`.

Command dispatch is driven by a mutable ``handlers`` dict on the
:class:`UpstreamServer`, so v2 can register additional handlers (e.g. for
``OPEN_CAMERA`` / ``CLOSE_CAMERA`` / ``0x72``) at startup without editing
this module. Unknown CMDs are logged and dropped.

Per the C++ service, we:

- accept any new TCP connection;
- wait for a ``0x19`` CCMD_CONNECT (``deviceSN|-1``) to learn the device SN;
- only after SN is known do we route subsequent messages under that SN;
- track ``0x23`` CCMD_CLIENT_HEARTBEAT (once every 10 s per Unity) and
  evict devices that miss it for longer than ``heartbeat_timeout_s``.

The C++ service does NOT reply to ``0x6C`` CCMD_SEND_VERSION; Unity treats
the successful ``send`` as confirmation on its own side. We match that.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Awaitable, Callable, Dict, Optional

from .device_model import DeviceRegistry
from .frame_codec import (
    HEAD_PC_TO_PICO,
    HEAD_PICO_TO_PC,
    Frame,
    FrameDecoder,
    encode,
)
from .tracking_parser import apply_tracking

logger = logging.getLogger(__name__)

# --- command byte constants (PICO -> PC) ---------------------------------
CMD_CONNECT = 0x19            # CCMD_CONNECT
CMD_SEND_VERSION = 0x6C       # CCMD_SEND_VERSION
CMD_TO_CONTROLLER_FUNCTION = 0x6D  # the ~71 Hz tracking stream
CMD_HEARTBEAT = 0x23          # CCMD_CLIENT_HEARTBEAT
CMD_CUSTOM_TO_PC = 0x72       # passthrough / v2-reserved
# --- command byte constants (PC -> PICO) ---------------------------------
CMD_COMMON_FUNCTION = 0x5F    # CMD_FROM_CONTROLLER_COMMON_FUNCTION

HEARTBEAT_CHECK_INTERVAL_S = 5.0


Handler = Callable[["Connection", Frame], Awaitable[None]]


class Connection:
    """Per-socket state + send helpers exposed to handlers.

    ``device_sn`` is populated when we see the first ``CMD_CONNECT`` frame.
    Before that we buffer nothing and simply ignore non-connect frames, the
    same behaviour the C++ ``TCPConnectionModel`` shows.
    """

    def __init__(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
        registry: DeviceRegistry,
    ) -> None:
        self._reader = reader
        self._writer = writer
        self.registry = registry
        self.decoder = FrameDecoder(expected_head=HEAD_PICO_TO_PC)

        peer = writer.get_extra_info("peername") or ("0.0.0.0", 0)
        self.peer_addr = (str(peer[0]), int(peer[1]))

        self.device_sn: Optional[str] = None

    async def send(self, cmd: int, params: bytes) -> None:
        try:
            self._writer.write(encode(cmd, params, head=HEAD_PC_TO_PICO))
            await self._writer.drain()
        except (ConnectionError, OSError) as e:
            logger.warning("send to %s failed: %s", self.peer_addr, e)
            self._writer.close()

    async def send_common_function(self, payload: str | bytes) -> None:
        """Send a ``0x5F`` message to the headset. Used by
        ``DeviceControlJson`` and ``SendBytesToDevice`` on the gRPC side.
        """
        body = payload.encode("utf-8") if isinstance(payload, str) else payload
        await self.send(CMD_COMMON_FUNCTION, body)

    def close(self) -> None:
        if not self._writer.is_closing():
            self._writer.close()


class UpstreamServer:
    def __init__(
        self,
        registry: DeviceRegistry,
        *,
        host: str = "0.0.0.0",
        port: int = 63901,
        heartbeat_timeout_s: float = 30.0,
    ) -> None:
        self.registry = registry
        self.host = host
        self.port = port
        self.heartbeat_timeout_s = heartbeat_timeout_s

        self._connections_by_sn: Dict[str, Connection] = {}

        # Exposed + mutable on purpose: v2 code registers OPEN_CAMERA,
        # CLOSE_CAMERA etc. here.
        self.handlers: Dict[int, Handler] = {
            CMD_CONNECT: _handle_connect,
            CMD_SEND_VERSION: _handle_send_version,
            CMD_TO_CONTROLLER_FUNCTION: _handle_tracking,
            CMD_HEARTBEAT: _handle_heartbeat,
            CMD_CUSTOM_TO_PC: _handle_custom_to_pc,
        }

        self._server: Optional[asyncio.base_events.Server] = None
        self._reaper_task: Optional[asyncio.Task[None]] = None

    def get_connection(self, device_sn: str) -> Optional[Connection]:
        """Used by the gRPC server to find which TCP socket to send a
        ``0x5F`` control message down.
        """
        return self._connections_by_sn.get(device_sn)

    async def serve(self) -> None:
        self._server = await asyncio.start_server(
            self._on_client, host=self.host, port=self.port
        )
        sockets = self._server.sockets or []
        logger.info(
            "upstream TCP listening on %s",
            ", ".join(str(s.getsockname()) for s in sockets),
        )
        self._reaper_task = asyncio.create_task(self._reap_stale())
        try:
            async with self._server:
                await self._server.serve_forever()
        except asyncio.CancelledError:
            pass
        finally:
            if self._reaper_task is not None:
                self._reaper_task.cancel()

    async def _on_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        conn = Connection(reader, writer, self.registry)
        logger.info("upstream connection opened from %s", conn.peer_addr)
        try:
            while True:
                chunk = await reader.read(65536)
                if not chunk:
                    break
                conn.decoder.feed(chunk)
                for frame in conn.decoder.frames():
                    await self._dispatch(conn, frame)
        except (ConnectionError, OSError) as e:
            logger.info("upstream connection %s error: %s", conn.peer_addr, e)
        except Exception:
            logger.exception("upstream connection %s crashed", conn.peer_addr)
        finally:
            if conn.device_sn is not None:
                self._connections_by_sn.pop(conn.device_sn, None)
                await self.registry.unregister(conn.device_sn)
            conn.close()
            logger.info("upstream connection closed: %s", conn.peer_addr)

    async def _dispatch(self, conn: Connection, frame: Frame) -> None:
        handler = self.handlers.get(frame.cmd)
        if handler is None:
            logger.debug(
                "unknown upstream cmd 0x%02X from %s (len=%d)",
                frame.cmd, conn.peer_addr, len(frame.params),
            )
            return

        await handler(conn, frame)

        # Track which SN -> which connection, so downstream gRPC can route
        # control messages down the right socket.
        if conn.device_sn is not None:
            self._connections_by_sn[conn.device_sn] = conn

    async def _reap_stale(self) -> None:
        try:
            while True:
                await asyncio.sleep(HEARTBEAT_CHECK_INTERVAL_S)
                expired = await self.registry.expire_stale(self.heartbeat_timeout_s)
                for rec in expired:
                    conn = self._connections_by_sn.pop(rec.device_sn, None)
                    if conn is not None:
                        logger.info("reaping stale device %s (no heartbeat)", rec.device_sn)
                        conn.close()
        except asyncio.CancelledError:
            pass


# ------------------------- individual handlers --------------------------

async def _handle_connect(conn: Connection, frame: Frame) -> None:
    payload = frame.params.decode("utf-8", errors="replace")
    # Unity sends ``<deviceSN>|-1``; we only care about the SN.
    device_sn = payload.split("|", 1)[0].strip()
    if not device_sn:
        logger.warning("empty device SN in CONNECT from %s", conn.peer_addr)
        return
    conn.device_sn = device_sn
    await conn.registry.register(device_sn, conn.peer_addr)
    logger.info(
        "device %s registered (peer=%s)", device_sn, conn.peer_addr
    )


async def _handle_send_version(conn: Connection, frame: Frame) -> None:
    payload = frame.params.decode("utf-8", errors="replace")
    # Unity sends ``<deviceSN>|1.0|<appVersion>``.
    parts = payload.split("|")
    if len(parts) >= 3:
        device_sn, _proto_ver, app_version = parts[0], parts[1], parts[2]
    elif len(parts) >= 1:
        device_sn, app_version = parts[0], ""
    else:
        return

    device_sn = device_sn.strip()
    if conn.device_sn is None and device_sn:
        # Some Unity builds send VERSION before CONNECT; register lazily.
        conn.device_sn = device_sn
        await conn.registry.register(device_sn, conn.peer_addr)

    if conn.device_sn:
        await conn.registry.set_app_version(conn.device_sn, app_version)
        logger.info(
            "device %s reports app_version=%r",
            conn.device_sn, app_version,
        )


async def _handle_tracking(conn: Connection, frame: Frame) -> None:
    if conn.device_sn is None:
        return
    payload = frame.params.decode("utf-8", errors="replace")

    rec = await conn.registry.get(conn.device_sn)
    if rec is None:
        return

    apply_tracking(rec, payload)
    await conn.registry.update_state_json(conn.device_sn, payload)


async def _handle_heartbeat(conn: Connection, frame: Frame) -> None:
    if conn.device_sn is None:
        return
    await conn.registry.touch_heartbeat(conn.device_sn)


async def _handle_custom_to_pc(conn: Connection, frame: Frame) -> None:
    # v1: passthrough, logged and dropped. v2 media plane overrides this
    # handler via ``UpstreamServer.handlers[CMD_CUSTOM_TO_PC] = ...``.
    logger.debug(
        "0x72 CUSTOM_TO_PC from %s (sn=%s, %d bytes): unhandled in v1",
        conn.peer_addr, conn.device_sn, len(frame.params),
    )
