"""Parse ``DeviceStateJson.statejson`` payloads into the shared State.

The wire shape (see py_bindings.cpp lines 116-267 for the C++ equivalent)::

    {
      "functionName": "Tracking",
      "value": "<stringified JSON>"
    }

Inner value::

    {
      "Head": {"pose": "x,y,z,qx,qy,qz,qw", ...},
      "Controller": {
        "left":  {"pose": "...", "trigger": ..., "grip": ..., "axisX": ...,
                  "axisY": ..., "axisClick": bool, "menuButton": bool,
                  "primaryButton": bool, "secondaryButton": bool},
        "right": {...}
      },
      "Hand": {
        "leftHand":  {"scale": ..., "isActive": int,
                      "HandJointLocations": [{"p": "x,y,z,qx,qy,qz,qw"}, ...26]},
        "rightHand": {...}
      },
      "Body": {
        "timeStampNs": int,
        "joints": [{"p": "pose", "va": "vel", "wva": "acc", "t": int}, ...24]
      },
      "Motion": {
        "timeStampNs": int,
        "joints": [{"p": "...", "va": "...", "wva": "...", "sn": "..."}, ...]
      },
      "timeStampNs": int
    }
"""

from __future__ import annotations

import json
import logging
from typing import Tuple

from ._state import State, Pose7, Vel6

logger = logging.getLogger(__name__)


def _parse_pose7(s) -> Pose7:
    if not isinstance(s, str):
        return (0.0,) * 7  # type: ignore[return-value]
    try:
        parts = [float(x) for x in s.split(",")]
    except ValueError:
        return (0.0,) * 7  # type: ignore[return-value]
    if len(parts) < 7:
        parts = parts + [0.0] * (7 - len(parts))
    return tuple(parts[:7])  # type: ignore[return-value]


def _parse_vel6(s) -> Vel6:
    if not isinstance(s, str):
        return (0.0,) * 6  # type: ignore[return-value]
    try:
        parts = [float(x) for x in s.split(",")]
    except ValueError:
        return (0.0,) * 6  # type: ignore[return-value]
    if len(parts) < 6:
        parts = parts + [0.0] * (6 - len(parts))
    return tuple(parts[:6])  # type: ignore[return-value]


def _as_float(v, default: float = 0.0) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def _as_int(v, default: int = 0) -> int:
    try:
        return int(v)
    except (TypeError, ValueError):
        return default


def _as_bool(v) -> bool:
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return v != 0
    if isinstance(v, str):
        return v.lower() in ("true", "1")
    return False


def apply_state_json(state: State, raw_state_json: str) -> None:
    """Parse ``raw_state_json`` (the DeviceStateJson.statejson field) and commit
    into ``state`` under ``state.lock``."""
    try:
        outer = json.loads(raw_state_json)
    except (ValueError, TypeError) as exc:
        logger.debug("bad outer JSON: %s", exc)
        return

    value_str = outer.get("value")
    if not isinstance(value_str, str):
        return
    try:
        value = json.loads(value_str)
    except (ValueError, TypeError) as exc:
        logger.debug("bad inner JSON: %s", exc)
        return

    with state.lock:
        controller = value.get("Controller") or {}
        left = controller.get("left")
        if isinstance(left, dict):
            state.left_controller_pose = _parse_pose7(left.get("pose"))
            state.left_trigger = _as_float(left.get("trigger"))
            state.left_grip = _as_float(left.get("grip"))
            state.left_menu_button = _as_bool(left.get("menuButton"))
            state.left_axis = (
                _as_float(left.get("axisX")),
                _as_float(left.get("axisY")),
            )
            state.left_axis_click = _as_bool(left.get("axisClick"))
            state.left_primary_button = _as_bool(left.get("primaryButton"))
            state.left_secondary_button = _as_bool(left.get("secondaryButton"))

        right = controller.get("right")
        if isinstance(right, dict):
            state.right_controller_pose = _parse_pose7(right.get("pose"))
            state.right_trigger = _as_float(right.get("trigger"))
            state.right_grip = _as_float(right.get("grip"))
            state.right_menu_button = _as_bool(right.get("menuButton"))
            state.right_axis = (
                _as_float(right.get("axisX")),
                _as_float(right.get("axisY")),
            )
            state.right_axis_click = _as_bool(right.get("axisClick"))
            state.right_primary_button = _as_bool(right.get("primaryButton"))
            state.right_secondary_button = _as_bool(right.get("secondaryButton"))

        head = value.get("Head")
        if isinstance(head, dict):
            state.headset_pose = _parse_pose7(head.get("pose"))

        if "timeStampNs" in value:
            state.timestamp_ns = _as_int(value.get("timeStampNs"))

        hand = value.get("Hand") or {}
        left_hand = hand.get("leftHand")
        if isinstance(left_hand, dict):
            state.left_hand_scale = _as_float(left_hand.get("scale"), 1.0)
            state.left_hand_is_active = _as_int(left_hand.get("isActive"))
            joints = left_hand.get("HandJointLocations") or []
            for i in range(min(26, len(joints))):
                entry = joints[i]
                if isinstance(entry, dict):
                    state.left_hand_tracking[i] = _parse_pose7(entry.get("p"))

        right_hand = hand.get("rightHand")
        if isinstance(right_hand, dict):
            state.right_hand_scale = _as_float(right_hand.get("scale"), 1.0)
            state.right_hand_is_active = _as_int(right_hand.get("isActive"))
            joints = right_hand.get("HandJointLocations") or []
            for i in range(min(26, len(joints))):
                entry = joints[i]
                if isinstance(entry, dict):
                    state.right_hand_tracking[i] = _parse_pose7(entry.get("p"))

        body = value.get("Body")
        if isinstance(body, dict):
            if "timeStampNs" in body:
                state.body_timestamp_ns = _as_int(body.get("timeStampNs"))
            joints = body.get("joints")
            if isinstance(joints, list):
                n = min(24, len(joints))
                for i in range(n):
                    joint = joints[i]
                    if not isinstance(joint, dict):
                        continue
                    if "p" in joint:
                        state.body_joints_pose[i] = _parse_pose7(joint.get("p"))
                    if "va" in joint:
                        state.body_joints_velocity[i] = _parse_vel6(joint.get("va"))
                    if "wva" in joint:
                        state.body_joints_acceleration[i] = _parse_vel6(joint.get("wva"))
                    if "t" in joint:
                        state.body_joints_timestamp[i] = _as_int(joint.get("t"))
                state.body_data_available = True

        motion = value.get("Motion")
        if isinstance(motion, dict):
            if "timeStampNs" in motion:
                state.motion_timestamp_ns = _as_int(motion.get("timeStampNs"))
            joints = motion.get("joints")
            if isinstance(joints, list):
                n = min(3, len(joints))
                poses = []
                vels = []
                accs = []
                sns = []
                for i in range(n):
                    joint = joints[i]
                    if not isinstance(joint, dict):
                        continue
                    poses.append(_parse_pose7(joint.get("p")))
                    vels.append(_parse_vel6(joint.get("va")))
                    accs.append(_parse_vel6(joint.get("wva")))
                    sns.append(str(joint.get("sn", "")))
                state.motion_tracker_pose = poses
                state.motion_tracker_velocity = vels
                state.motion_tracker_acceleration = accs
                state.motion_tracker_serial_numbers = sns
                state.num_motion_data_available = len(poses)
