"""Parse the ``0x6D`` CCMD_TO_CONTROLLER_FUNCTION JSON payload.

Shape (from ``TrackingData.cs`` + verified against xrobotoolkit_sdk's
``py_bindings.cpp``)::

    {
      "functionName": "Tracking",
      "value": "<JSON-encoded string with Head/Controller/Hand/...>"
    }

The inner ``value`` is a *stringified* JSON blob, e.g.::

    {
      "Head": {"pose": "x,y,z,qx,qy,qz,qw", "status": N},
      "Controller": {
        "left":  {"pose": "...", "trigger": ..., "grip": ..., ...},
        "right": {"pose": "...", ...}
      },
      "Hand": {...},
      "Body": {...},
      "Motion": {...},
      "timeStampNs": 12345,
      "Input": N
    }

v1 extracts Head + Controller into a :class:`DeviceRecord`. Hand / Body /
Motion are left as TODOs — the raw ``statejson`` still flows through to the
SDK downstream, so nothing about the consumer breaks; v2 just fills in the
cached Python-side fields.

This module is pure-functional (no I/O) so it is trivially unit-testable
against captured fixture JSON.
"""

from __future__ import annotations

import json
import logging
from typing import Any, Dict, Optional, Tuple

from .device_model import ControllerState, DeviceRecord, HeadState

logger = logging.getLogger(__name__)

FUNCTION_TRACKING = "Tracking"


def _parse_pose(pose_str: Any) -> Tuple[float, ...]:
    if not isinstance(pose_str, str):
        return ()
    try:
        return tuple(float(x) for x in pose_str.split(","))
    except ValueError:
        logger.debug("bad pose string: %r", pose_str)
        return ()


def _as_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def _as_bool(v: Any, default: bool = False) -> bool:
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    if isinstance(v, str):
        return v.lower() in ("true", "1")
    return default


def _fill_controller(dst: ControllerState, src: Dict[str, Any]) -> None:
    dst.pose = _parse_pose(src.get("pose"))
    dst.trigger = _as_float(src.get("trigger"))
    dst.grip = _as_float(src.get("grip"))
    dst.axis_x = _as_float(src.get("axisX"))
    dst.axis_y = _as_float(src.get("axisY"))
    dst.axis_click = _as_bool(src.get("axisClick"))
    dst.primary_button = _as_bool(src.get("primaryButton"))
    dst.secondary_button = _as_bool(src.get("secondaryButton"))
    dst.menu_button = _as_bool(src.get("menuButton"))


def parse_function_envelope(payload: str) -> Optional[Tuple[str, Any]]:
    """Parse the outer ``{functionName, value}`` envelope.

    Returns ``(function_name, decoded_value)`` or ``None`` on malformed JSON.
    ``decoded_value`` may be a string (leave it to the caller) or a
    pre-decoded dict if the sender inlined the object (some variants do).
    """
    try:
        outer = json.loads(payload)
    except (json.JSONDecodeError, TypeError):
        logger.debug("invalid outer JSON: %.80r...", payload)
        return None

    if not isinstance(outer, dict):
        return None

    name = outer.get("functionName")
    value = outer.get("value")
    if not isinstance(name, str):
        return None

    if isinstance(value, str):
        try:
            value = json.loads(value)
        except json.JSONDecodeError:
            pass

    return name, value


def apply_tracking(record: DeviceRecord, payload: str) -> bool:
    """Merge a raw ``0x6D`` JSON payload into the given :class:`DeviceRecord`.

    Returns ``True`` if the payload was a recognised Tracking frame and at
    least one top-level field was applied. Always stores the raw payload on
    the record so the gRPC fan-out can forward it verbatim.
    """
    record.raw_state_json = payload

    env = parse_function_envelope(payload)
    if env is None:
        return False
    name, value = env
    if name != FUNCTION_TRACKING or not isinstance(value, dict):
        return False

    applied = False

    head = value.get("Head")
    if isinstance(head, dict):
        record.head = HeadState(
            pose=_parse_pose(head.get("pose")),
            status=int(_as_float(head.get("status"))),
        )
        applied = True

    ctrl = value.get("Controller")
    if isinstance(ctrl, dict):
        left = ctrl.get("left")
        if isinstance(left, dict):
            _fill_controller(record.left_controller, left)
            applied = True
        right = ctrl.get("right")
        if isinstance(right, dict):
            _fill_controller(record.right_controller, right)
            applied = True

    # Hand / Body / Motion: v2 territory. Deliberately not populated;
    # downstream consumers that want these today must re-parse the
    # ``raw_state_json`` blob themselves, exactly as they do against the
    # C++ service.

    return applied
