from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

import numpy as np

ArmName = Literal["left_arm", "right_arm"]


def _array(values, shape: tuple[int, ...], name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {arr.shape}")
    return arr.copy()


@dataclass(frozen=True)
class Pose:
    """Cartesian pose with quaternion orientation.

    Attributes:
        position: XYZ position, shape ``(3,)``.
        orientation: Quaternion in ``[w, x, y, z]`` order, shape ``(4,)``.
    """

    position: np.ndarray
    orientation: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", _array(self.position, (3,), "position"))
        object.__setattr__(self, "orientation", _array(self.orientation, (4,), "orientation"))


@dataclass(frozen=True)
class ControllerInput:
    pose: Pose
    grip: float = 0.0
    trigger: float = 0.0
    buttons: dict[str, bool] = field(default_factory=dict)
    joystick: tuple[float, float] = (0.0, 0.0)


@dataclass(frozen=True)
class XRState:
    timestamp_ns: int
    headset: Pose | None
    left_controller: ControllerInput
    right_controller: ControllerInput
    buttons: dict[str, bool] = field(default_factory=dict)


@dataclass(frozen=True)
class RobotState:
    timestamp: float
    joint_positions: dict[ArmName, np.ndarray] = field(default_factory=dict)
    gripper_positions: dict[ArmName, float] = field(default_factory=dict)


@dataclass(frozen=True)
class HeadAngles:
    """2DoF head pan/tilt target in degrees, relative to the neutral headset pose."""

    yaw_deg: float
    pitch_deg: float


@dataclass(frozen=True)
class BaseTarget:
    """Mobile-base velocity command (body frame). Robots without a base ignore this."""

    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0


@dataclass(frozen=True)
class HeightTarget:
    """Lift/torso height command in meters. Robots without a lift ignore this."""

    height_m: float = 0.0


@dataclass(frozen=True)
class LiftVelocity:
    """Open-loop lift speed command as a signed percent in ``[-100, 100]``.

    Positive = up, negative = down, ``0`` = stop. Robots without a lift ignore this.
    """

    speed_pct: int = 0
