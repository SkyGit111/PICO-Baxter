"""Cross-platform Python replacement for the XRoboToolkit PC Service.

Public surface:

- :class:`DeviceRegistry` — shared in-memory state.
- :class:`UpstreamServer` — PICO/Unity TCP termination (port 63901).
- :class:`DownstreamServer` — gRPC ``EAService`` (port 60061).
- :func:`config.load` — environment / INI-based settings.

See ``main.py`` for the wired-up entry point.
"""

from .config import Config, load as load_config
from .device_model import DeviceRegistry
from .downstream import DownstreamServer
from .upstream import UpstreamServer

__all__ = [
    "Config",
    "DeviceRegistry",
    "DownstreamServer",
    "UpstreamServer",
    "load_config",
]
