"""Shared in-memory state updated by the gRPC subscriber thread.

Mirrors the globals in ``bindings/py_bindings.cpp``. A single ``threading.Lock``
guards everything; the getter functions read under that lock and return copies,
matching the Pybind behavior (which took per-field mutexes and returned values).
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import List, Tuple

Pose7 = Tuple[float, float, float, float, float, float, float]  # x,y,z,qx,qy,qz,qw
Vel6 = Tuple[float, float, float, float, float, float]

_ZERO_POSE: Pose7 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
_ZERO_VEL: Vel6 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass
class State:
    lock: threading.Lock = field(default_factory=threading.Lock)

    # Poses
    left_controller_pose: Pose7 = _ZERO_POSE
    right_controller_pose: Pose7 = _ZERO_POSE
    headset_pose: Pose7 = _ZERO_POSE

    # Controller inputs
    left_trigger: float = 0.0
    left_grip: float = 0.0
    left_menu_button: bool = False
    left_axis: Tuple[float, float] = (0.0, 0.0)
    left_axis_click: bool = False
    left_primary_button: bool = False  # X
    left_secondary_button: bool = False  # Y

    right_trigger: float = 0.0
    right_grip: float = 0.0
    right_menu_button: bool = False
    right_axis: Tuple[float, float] = (0.0, 0.0)
    right_axis_click: bool = False
    right_primary_button: bool = False  # A
    right_secondary_button: bool = False  # B

    timestamp_ns: int = 0

    # Hand tracking (26 joints each)
    left_hand_tracking: List[Pose7] = field(
        default_factory=lambda: [_ZERO_POSE for _ in range(26)]
    )
    left_hand_scale: float = 1.0
    left_hand_is_active: int = 0
    right_hand_tracking: List[Pose7] = field(
        default_factory=lambda: [_ZERO_POSE for _ in range(26)]
    )
    right_hand_scale: float = 1.0
    right_hand_is_active: int = 0

    # Body tracking (24 joints)
    body_joints_pose: List[Pose7] = field(
        default_factory=lambda: [_ZERO_POSE for _ in range(24)]
    )
    body_joints_velocity: List[Vel6] = field(
        default_factory=lambda: [_ZERO_VEL for _ in range(24)]
    )
    body_joints_acceleration: List[Vel6] = field(
        default_factory=lambda: [_ZERO_VEL for _ in range(24)]
    )
    body_joints_timestamp: List[int] = field(default_factory=lambda: [0] * 24)
    body_timestamp_ns: int = 0
    body_data_available: bool = False

    # Motion trackers (up to 3)
    motion_tracker_pose: List[Pose7] = field(default_factory=list)
    motion_tracker_velocity: List[Vel6] = field(default_factory=list)
    motion_tracker_acceleration: List[Vel6] = field(default_factory=list)
    motion_tracker_serial_numbers: List[str] = field(default_factory=list)
    motion_timestamp_ns: int = 0
    num_motion_data_available: int = 0


# Global singleton. Reset on close().
_STATE = State()


def get_state() -> State:
    return _STATE


def reset_state() -> None:
    global _STATE
    _STATE = State()
