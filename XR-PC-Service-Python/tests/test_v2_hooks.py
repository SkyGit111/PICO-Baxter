"""Pin down the v2 design hooks promised in ``serialized-strolling-fairy.md``.

These tests fail if someone accidentally breaks an API the future media
plane is supposed to reuse. They are cheap insurance against the kind of
refactor that tends to drop peer-addr tracking or event-bus callbacks.
"""

from __future__ import annotations

import asyncio

from xrbt_service.device_model import DeviceRegistry
from xrbt_service.upstream import (
    CMD_CUSTOM_TO_PC,
    UpstreamServer,
)


async def test_peer_addr_is_captured_on_register() -> None:
    reg = DeviceRegistry()
    await reg.register("SN-A", ("192.168.1.42", 49152))
    assert await reg.peer_addr_of("SN-A") == ("192.168.1.42", 49152)
    assert await reg.peer_addr_of("missing") is None


async def test_event_bus_connect_and_disconnect_callbacks_fire() -> None:
    reg = DeviceRegistry()
    seen_connect: list[str] = []
    seen_disconnect: list[str] = []

    async def on_connect(rec) -> None:
        seen_connect.append(rec.device_sn)

    def on_disconnect(rec) -> None:
        seen_disconnect.append(rec.device_sn)

    reg.on_device_connected(on_connect)
    reg.on_device_disconnected(on_disconnect)

    await reg.register("SN-X", ("10.0.0.1", 1))
    await reg.register("SN-X", ("10.0.0.1", 1))  # re-register is not a new connect
    await reg.unregister("SN-X")

    assert seen_connect == ["SN-X"]
    assert seen_disconnect == ["SN-X"]


async def test_upstream_handlers_dict_is_mutable_for_v2() -> None:
    """Ensure v2 can override CMD_CUSTOM_TO_PC without monkeypatching
    private attributes.
    """
    reg = DeviceRegistry()
    srv = UpstreamServer(reg, host="127.0.0.1", port=0)

    assert CMD_CUSTOM_TO_PC in srv.handlers

    called = asyncio.Event()

    async def v2_handler(conn, frame) -> None:
        called.set()

    srv.handlers[CMD_CUSTOM_TO_PC] = v2_handler
    # Invoke directly — we don't need a real socket for this assertion.
    await srv.handlers[CMD_CUSTOM_TO_PC](None, None)
    assert called.is_set()


async def test_registry_changed_event_wakes_waiters() -> None:
    """WatchServerFeedback relies on this. A v2 media subscriber can use
    the same primitive for its own state-change observers.
    """
    reg = DeviceRegistry()

    wake = asyncio.create_task(reg.changed.wait())
    await asyncio.sleep(0)

    await reg.register("SN-W", ("127.0.0.1", 1))
    await asyncio.wait_for(wake, timeout=1.0)
