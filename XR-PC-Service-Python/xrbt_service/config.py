"""Runtime configuration.

Precedence (highest wins):

1. Environment variables (``XRBT_*``).
2. ``setting.ini`` in the process working directory, using the same section /
   key names as the C++ service (``[TCP] TcpBindPort``, etc.). Not all keys
   are honoured yet; only ports are read for v1.
3. Compiled-in defaults.
"""

from __future__ import annotations

import configparser
import os
from dataclasses import dataclass
from pathlib import Path

DEFAULT_UPSTREAM_HOST = "0.0.0.0"
DEFAULT_UPSTREAM_PORT = 63901

DEFAULT_DOWNSTREAM_HOST = "127.0.0.1"
DEFAULT_DOWNSTREAM_PORT = 60061

DEFAULT_HEARTBEAT_TIMEOUT_S = 30.0

# --- v2 media plane (reserved, NOT bound in v1) --------------------------
# Documented here so nothing else in the project collides with these ports.
# Do not add listeners on these until the media plane lands.
DEFAULT_MEDIA_UDP_PORT = 63910       # RTP/UDP video sink
DEFAULT_MEDIA_SIGNALING_PORT = 63911 # WebRTC signaling, if/when used
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Config:
    upstream_host: str = DEFAULT_UPSTREAM_HOST
    upstream_port: int = DEFAULT_UPSTREAM_PORT
    downstream_host: str = DEFAULT_DOWNSTREAM_HOST
    downstream_port: int = DEFAULT_DOWNSTREAM_PORT
    heartbeat_timeout_s: float = DEFAULT_HEARTBEAT_TIMEOUT_S


def _read_ini(path: Path) -> dict[str, str]:
    if not path.is_file():
        return {}
    parser = configparser.ConfigParser()
    try:
        parser.read(path)
    except configparser.Error:
        return {}
    out: dict[str, str] = {}
    if parser.has_option("TCP", "TcpBindPort"):
        out["upstream_port"] = parser.get("TCP", "TcpBindPort")
    if parser.has_option("GRPC", "BindHost"):
        out["downstream_host"] = parser.get("GRPC", "BindHost")
    if parser.has_option("GRPC", "BindPort"):
        out["downstream_port"] = parser.get("GRPC", "BindPort")
    return out


def load(ini_path: str | os.PathLike[str] | None = "setting.ini") -> Config:
    ini: dict[str, str] = {}
    if ini_path is not None:
        ini = _read_ini(Path(ini_path))

    def _get(key: str, env: str, default: str | int | float) -> str:
        v = os.environ.get(env)
        if v is not None:
            return v
        if key in ini:
            return ini[key]
        return str(default)

    return Config(
        upstream_host=_get("upstream_host", "XRBT_UPSTREAM_HOST", DEFAULT_UPSTREAM_HOST),
        upstream_port=int(_get("upstream_port", "XRBT_UPSTREAM_PORT", DEFAULT_UPSTREAM_PORT)),
        downstream_host=_get("downstream_host", "XRBT_DOWNSTREAM_HOST", DEFAULT_DOWNSTREAM_HOST),
        downstream_port=int(_get("downstream_port", "XRBT_DOWNSTREAM_PORT", DEFAULT_DOWNSTREAM_PORT)),
        heartbeat_timeout_s=float(
            _get("heartbeat_timeout_s", "XRBT_HEARTBEAT_TIMEOUT_S", DEFAULT_HEARTBEAT_TIMEOUT_S)
        ),
    )
