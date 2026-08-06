"""In-memory state for all connected XR devices.

Mirrors (loosely) the C++ service's per-device record, but the important
invariants for the Python port are:

1. The raw ``statejson`` blob received on command ``0x6D`` is the source of
   truth that the downstream gRPC server fans out to subscribers. The C++
   service stuffs it verbatim into ``ServerFeedback.devicestatejson.statejson``
   and so do we. Parsed pose/button fields are derived + cached for logging
   or future use, but are *not* what the consumer SDK actually reads.

2. :attr:`DeviceRecord.peer_addr` is the ``(ip, port)`` of the headset's
   upstream TCP socket. A v2 media publisher uses it to know where to send
   video frames without reaching into ``upstream.py`` internals.

3. The event bus (:meth:`DeviceRegistry.on_device_connected`,
   :meth:`on_device_disconnected`, :meth:`on_state_changed`) lets a future
   media subsystem react to device lifecycle without editing this module or
   ``upstream.py``.
"""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass, field
from typing import Awaitable, Callable, Dict, List, Optional, Tuple

PeerAddr = Tuple[str, int]

OnConnect = Callable[["DeviceRecord"], Awaitable[None] | None]
OnDisconnect = Callable[["DeviceRecord"], Awaitable[None] | None]
OnStateChanged = Callable[["DeviceRecord"], Awaitable[None] | None]


@dataclass
class ControllerState:
    pose: Tuple[float, ...] = ()
    trigger: float = 0.0
    grip: float = 0.0
    axis_x: float = 0.0
    axis_y: float = 0.0
    axis_click: bool = False
    primary_button: bool = False
    secondary_button: bool = False
    menu_button: bool = False


@dataclass
class HeadState:
    pose: Tuple[float, ...] = ()
    status: int = 0


@dataclass
class DeviceRecord:
    """Per-device runtime state.

    ``raw_state_json`` is the unmodified UTF-8 payload from the last
    ``0x6D`` frame — this is what ``downstream.py`` forwards to
    ``xrobotoolkit_sdk`` as ``statejson``.
    """

    device_sn: str
    peer_addr: PeerAddr
    app_version: str = ""
    last_heartbeat: float = field(default_factory=time.monotonic)
    last_update_ns: int = 0

    raw_state_json: str = ""

    head: HeadState = field(default_factory=HeadState)
    left_controller: ControllerState = field(default_factory=ControllerState)
    right_controller: ControllerState = field(default_factory=ControllerState)

    # --- reserved for v2, intentionally untouched by v1 ---
    left_hand: object = None
    right_hand: object = None
    body_tracking: object = None
    motion_tracking: object = None


class DeviceRegistry:
    """Thread-/task-safe registry keyed by ``device_sn``.

    All public methods are coroutine-safe via a single ``asyncio.Lock``.
    The ``changed`` :class:`asyncio.Event` fires every time *any* device's
    state updates, so ``WatchServerFeedback`` subscribers can ``await`` it.
    """

    def __init__(self) -> None:
        self._devices: Dict[str, DeviceRecord] = {}
        self._lock = asyncio.Lock()

        self._on_connect: List[OnConnect] = []
        self._on_disconnect: List[OnDisconnect] = []
        self._on_state_changed: List[OnStateChanged] = []

        self.changed: asyncio.Event = asyncio.Event()

    # ---------------------------- lifecycle ------------------------------

    async def register(self, device_sn: str, peer_addr: PeerAddr) -> DeviceRecord:
        is_new = False
        async with self._lock:
            rec = self._devices.get(device_sn)
            if rec is None:
                rec = DeviceRecord(device_sn=device_sn, peer_addr=peer_addr)
                self._devices[device_sn] = rec
                is_new = True
            else:
                rec.peer_addr = peer_addr
                rec.last_heartbeat = time.monotonic()

        if is_new:
            await self._fire(self._on_connect, rec)
        self._wake_watchers()
        return rec

    async def unregister(self, device_sn: str) -> None:
        async with self._lock:
            rec = self._devices.pop(device_sn, None)
        if rec is not None:
            await self._fire(self._on_disconnect, rec)
            self._wake_watchers()

    async def touch_heartbeat(self, device_sn: str) -> None:
        async with self._lock:
            rec = self._devices.get(device_sn)
            if rec is not None:
                rec.last_heartbeat = time.monotonic()

    async def set_app_version(self, device_sn: str, version: str) -> None:
        async with self._lock:
            rec = self._devices.get(device_sn)
            if rec is not None:
                rec.app_version = version

    async def update_state_json(self, device_sn: str, raw_json: str) -> None:
        """Store the raw ``0x6D`` payload. Called on every tracking frame
        (~71 Hz in normal operation). Wakes all ``WatchServerFeedback``
        subscribers.
        """
        rec: Optional[DeviceRecord]
        async with self._lock:
            rec = self._devices.get(device_sn)
            if rec is None:
                return
            rec.raw_state_json = raw_json
            rec.last_update_ns = time.monotonic_ns()

        await self._fire(self._on_state_changed, rec)
        self._wake_watchers()

    async def expire_stale(self, max_silence_s: float) -> List[DeviceRecord]:
        now = time.monotonic()
        expired: List[DeviceRecord] = []
        async with self._lock:
            for sn, rec in list(self._devices.items()):
                if now - rec.last_heartbeat > max_silence_s:
                    del self._devices[sn]
                    expired.append(rec)
        for rec in expired:
            await self._fire(self._on_disconnect, rec)
        if expired:
            self._wake_watchers()
        return expired

    # ---------------------------- lookups --------------------------------

    async def snapshot(self) -> List[DeviceRecord]:
        async with self._lock:
            return list(self._devices.values())

    async def get(self, device_sn: str) -> Optional[DeviceRecord]:
        async with self._lock:
            return self._devices.get(device_sn)

    async def peer_addr_of(self, device_sn: str) -> Optional[PeerAddr]:
        """Reserved for v2 media publisher use."""
        async with self._lock:
            rec = self._devices.get(device_sn)
            return rec.peer_addr if rec else None

    # ---------------------------- event bus ------------------------------

    def on_device_connected(self, cb: OnConnect) -> None:
        self._on_connect.append(cb)

    def on_device_disconnected(self, cb: OnDisconnect) -> None:
        self._on_disconnect.append(cb)

    def on_state_changed(self, cb: OnStateChanged) -> None:
        self._on_state_changed.append(cb)

    # --------------------------- internals -------------------------------

    def _wake_watchers(self) -> None:
        # Set+clear is a simple "broadcast once" pattern. Subscribers that
        # did ``await changed.wait()`` will unblock; any that are mid-work
        # and loop back will re-await cleanly.
        self.changed.set()
        self.changed.clear()

    @staticmethod
    async def _fire(
        callbacks: List[Callable[[DeviceRecord], Awaitable[None] | None]],
        rec: DeviceRecord,
    ) -> None:
        for cb in list(callbacks):
            try:
                result = cb(rec)
                if asyncio.iscoroutine(result):
                    await result
            except Exception:
                # Event-bus consumers must not be able to break the core
                # service. Log-and-continue is intentional.
                import logging
                logging.getLogger(__name__).exception(
                    "device-registry callback %r failed", cb
                )
