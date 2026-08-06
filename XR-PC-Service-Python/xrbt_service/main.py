"""Entrypoint: wire the upstream + downstream servers together.

Shape intentionally kept as::

    await asyncio.gather(upstream.serve(), downstream.serve())

v2 appends ``media.publisher.serve()`` (and friends) to the same
``gather`` call — no restructuring of the event loop.
"""

from __future__ import annotations

import asyncio
import logging
import signal
import sys
from typing import Iterable

from .config import Config, load as load_config
from .device_model import DeviceRegistry
from .downstream import DownstreamServer
from .upstream import UpstreamServer

logger = logging.getLogger("xrbt_service")


async def serve(cfg: Config | None = None) -> None:
    cfg = cfg or load_config()
    registry = DeviceRegistry()

    upstream = UpstreamServer(
        registry,
        host=cfg.upstream_host,
        port=cfg.upstream_port,
        heartbeat_timeout_s=cfg.heartbeat_timeout_s,
    )
    downstream = DownstreamServer(
        registry,
        upstream,
        host=cfg.downstream_host,
        port=cfg.downstream_port,
    )

    stop_event = asyncio.Event()

    def _request_stop(*_: object) -> None:
        logger.info("shutdown signal received")
        stop_event.set()

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _request_stop)
        except NotImplementedError:
            # Windows fallback — signals are handled via default KeyboardInterrupt.
            pass

    tasks: Iterable[asyncio.Task[None]] = [
        asyncio.create_task(upstream.serve(), name="upstream"),
        asyncio.create_task(downstream.serve(), name="downstream"),
        # v2: asyncio.create_task(media_publisher.serve(), name="media"),
    ]

    stop_task = asyncio.create_task(stop_event.wait(), name="stop")
    done, _ = await asyncio.wait(
        {*tasks, stop_task}, return_when=asyncio.FIRST_COMPLETED
    )

    # Propagate any server-crash exception so we don't silently exit 0 when
    # a listener died. (stop_task completing normally is fine.)
    for t in done:
        if t is stop_task:
            continue
        exc = t.exception()
        if exc is not None:
            for other in tasks:
                other.cancel()
            raise exc

    for t in tasks:
        t.cancel()

    # Give cancellation a moment to settle cleanly (gRPC grace, TCP close).
    await asyncio.gather(*tasks, return_exceptions=True)


def run() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    try:
        asyncio.run(serve())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    run()
    sys.exit(0)
