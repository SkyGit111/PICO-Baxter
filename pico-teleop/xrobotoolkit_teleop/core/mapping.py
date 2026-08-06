"""Default XR-to-robot mappers and shared geometry helpers.

All XR delta-pose math (controllers -> arm Cartesian targets) and headset-to-yaw/pitch
math (head -> 2DoF pan/tilt angles) lives here. Per-robot files configure these
mappers via MappingConfig and consume their outputs.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

import numpy as np

from xrobotoolkit_teleop.core.types import ControllerInput, HeadAngles, Pose

ControllerName = Literal["left_controller", "right_controller"]


# Default headset -> world rotation (XR Y-up -> robot Z-up).
R_HEADSET_TO_WORLD = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)

# Raw XR headset axes (before applying headset_to_world).
_HEAD_UP_RAW = np.array([0.0, 1.0, 0.0], dtype=np.float64)
_HEAD_RIGHT_RAW = np.array([1.0, 0.0, 0.0], dtype=np.float64)
_HEAD_FORWARD_RAW = np.array([0.0, 0.0, -1.0], dtype=np.float64)


# ---------------------------------------------------------------------------
# Quaternion / rotation helpers
# ---------------------------------------------------------------------------


def normalize_quat(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    if q.shape != (4,):
        raise ValueError(f"quaternion must have shape (4,), got {q.shape}")
    norm = np.linalg.norm(q)
    if norm == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    q = q / norm
    if q[0] < 0.0:
        q = -q
    return q


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def mat_to_quat_wxyz(m: np.ndarray) -> np.ndarray:
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return normalize_quat(np.array([w, x, y, z], dtype=np.float64))


def quat_wxyz_to_rot_mat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = normalize_quat(q)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def quat_to_angle_axis(q: np.ndarray, eps: float = 1.0e-6) -> np.ndarray:
    q = normalize_quat(q)
    angle = 2.0 * np.arccos(np.clip(q[0], -1.0, 1.0))
    if angle < eps:
        return np.zeros(3, dtype=np.float64)
    sin_half = np.sin(angle / 2.0)
    if abs(sin_half) < eps:
        return np.zeros(3, dtype=np.float64)
    return q[1:] / sin_half * angle


def quat_about_axis(angle: float, axis: np.ndarray) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    norm = np.linalg.norm(axis)
    if norm == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    axis = axis / norm
    half = angle / 2.0
    return normalize_quat(
        np.array(
            [np.cos(half), axis[0] * np.sin(half), axis[1] * np.sin(half), axis[2] * np.sin(half)],
            dtype=np.float64,
        )
    )


def delta_rotation(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    delta_q = quat_mul(target, quat_conjugate(source))
    return quat_to_angle_axis(delta_q)


def apply_delta_pose(source: Pose, delta_position: np.ndarray, delta_rot: np.ndarray) -> Pose:
    angle = np.linalg.norm(delta_rot)
    if angle > 1.0e-6:
        rot_delta = quat_about_axis(angle, delta_rot / angle)
    else:
        rot_delta = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return Pose(
        position=source.position + delta_position,
        orientation=normalize_quat(quat_mul(rot_delta, source.orientation)),
    )


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.zeros(3, dtype=np.float64)
    return np.asarray(v, dtype=np.float64) / n


def _proj_perp(v: np.ndarray, axis_unit: np.ndarray) -> np.ndarray:
    return np.asarray(v, dtype=np.float64) - np.dot(v, axis_unit) * axis_unit


def _signed_angle_about_axis(v_from: np.ndarray, v_to: np.ndarray, axis: np.ndarray) -> float:
    n = _unit(axis)
    if np.linalg.norm(n) < 1e-12:
        return 0.0
    a = _proj_perp(v_from, n)
    b = _proj_perp(v_to, n)
    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)
    if na < 1e-10 or nb < 1e-10:
        return 0.0
    a = a / na
    b = b / nb
    return float(np.arctan2(np.dot(np.cross(a, b), n), np.dot(a, b)))


def relative_yaw_pitch_deg(
    neutral: Pose,
    current: Pose,
    headset_to_world: np.ndarray = R_HEADSET_TO_WORLD,
) -> tuple[float, float]:
    """Yaw/pitch of current headset pose relative to neutral, for pan/tilt servos."""
    h = np.asarray(headset_to_world, dtype=np.float64)
    if h.shape != (3, 3):
        raise ValueError("headset_to_world must have shape (3, 3)")

    rn = h @ quat_wxyz_to_rot_mat(neutral.orientation) @ h.T
    rc = h @ quat_wxyz_to_rot_mat(current.orientation) @ h.T
    world_up = _unit(h @ _HEAD_UP_RAW)
    head_forward = _unit(h @ _HEAD_FORWARD_RAW)
    head_right = _unit(h @ _HEAD_RIGHT_RAW)

    f_n = _unit(rn @ head_forward)
    f_c = _unit(rc @ head_forward)
    ear_axis_n = _unit(rn @ head_right)

    pitch_rad = _signed_angle_about_axis(f_n, f_c, ear_axis_n)
    yaw_rad = _signed_angle_about_axis(f_n, f_c, world_up)
    return float(np.degrees(yaw_rad)), float(np.degrees(pitch_rad))


def apply_deadband(value: float, deadband: float) -> float:
    return 0.0 if abs(value) < deadband else value


# ---------------------------------------------------------------------------
# Mapper config
# ---------------------------------------------------------------------------


@dataclass
class MappingConfig:
    """Default mapping parameters shared across robots.

    Per-robot files override fields in their own config dataclass before
    constructing mappers.
    """

    scale_factor: float = 1.0
    grip_threshold: float = 0.9
    headset_to_world: np.ndarray = field(default_factory=lambda: R_HEADSET_TO_WORLD.copy())

    # Head pan/tilt smoothing params.
    head_smoothing_alpha: float = 0.25
    head_max_step_deg: float = 20.0
    head_deadband_deg: float = 1.0

    def __post_init__(self) -> None:
        h = np.asarray(self.headset_to_world, dtype=np.float64)
        if h.shape != (3, 3):
            raise ValueError("headset_to_world must have shape (3, 3)")
        self.headset_to_world = h
        if not 0.0 < self.head_smoothing_alpha <= 1.0:
            raise ValueError("head_smoothing_alpha must be in (0, 1]")
        if self.head_max_step_deg <= 0.0:
            raise ValueError("head_max_step_deg must be positive")


# ---------------------------------------------------------------------------
# Arm mapper: 6DoF controller -> Cartesian EE target with grip-latch reference.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _ArmReference:
    controller_pose: Pose
    robot_pose: Pose


class ArmPoseMapper:
    """Map a 6DoF controller pose to a robot end-effector target.

    Uses a grip-latch reference: when grip first crosses ``grip_threshold``,
    the current controller pose and current robot EE pose are stored as
    references, and subsequent controller motion is applied as a delta.
    Releasing the grip clears the reference.
    """

    def __init__(self, controller: ControllerName, config: MappingConfig, arm_label: str = ""):
        self.controller = controller
        self.config = config
        self.arm_label = arm_label or controller
        self._reference: _ArmReference | None = None
        self._headset_to_world_quat = mat_to_quat_wxyz(config.headset_to_world)

    def reset(self) -> None:
        self._reference = None

    def _to_world(self, pose: Pose) -> Pose:
        q_world = quat_mul(
            quat_mul(self._headset_to_world_quat, pose.orientation),
            quat_conjugate(self._headset_to_world_quat),
        )
        return Pose(
            position=self.config.headset_to_world @ pose.position,
            orientation=normalize_quat(q_world),
        )

    def update(self, controller_input: ControllerInput, current_robot_pose: Pose) -> Pose | None:
        """Return target EE pose, or None if grip is not active.

        Side effect: latches a reference on first grip activation, clears on release.
        """
        grip_active = controller_input.grip > self.config.grip_threshold
        if not grip_active:
            if self._reference is not None:
                print(f"{self.arm_label} grip released; pose command stopped.")
                self._reference = None
            return None

        controller_pose = self._to_world(controller_input.pose)
        if self._reference is None:
            self._reference = _ArmReference(
                controller_pose=controller_pose,
                robot_pose=current_robot_pose,
            )
            print(f"{self.arm_label} grip latched.")

        delta_pos = (controller_pose.position - self._reference.controller_pose.position) * self.config.scale_factor
        delta_rot = delta_rotation(self._reference.controller_pose.orientation, controller_pose.orientation)
        return apply_delta_pose(self._reference.robot_pose, delta_pos, delta_rot)


# ---------------------------------------------------------------------------
# Head mapper: headset pose -> yaw/pitch angles with smoothing/deadband.
# ---------------------------------------------------------------------------


class HeadAngleMapper:
    """Map headset pose to smoothed (yaw, pitch) degrees relative to a neutral pose.

    Servo unit conversion (gain, center, range) is the adapter's responsibility;
    this class returns angles only.
    """

    def __init__(self, config: MappingConfig):
        self.config = config
        self._last_yaw = 0.0
        self._last_pitch = 0.0

    def reset(self) -> None:
        self._last_yaw = 0.0
        self._last_pitch = 0.0

    def map_pose(self, neutral: Pose, current: Pose) -> HeadAngles:
        yaw_deg, pitch_deg = relative_yaw_pitch_deg(neutral, current, self.config.headset_to_world)
        yaw_deg = apply_deadband(yaw_deg, self.config.head_deadband_deg)
        pitch_deg = apply_deadband(pitch_deg, self.config.head_deadband_deg)
        yaw = self._smooth(self._last_yaw, yaw_deg)
        pitch = self._smooth(self._last_pitch, pitch_deg)
        self._last_yaw = yaw
        self._last_pitch = pitch
        return HeadAngles(yaw_deg=yaw, pitch_deg=pitch)

    def center(self) -> HeadAngles:
        self.reset()
        return HeadAngles(yaw_deg=0.0, pitch_deg=0.0)

    def _smooth(self, previous: float, target: float) -> float:
        smoothed = previous + self.config.head_smoothing_alpha * (target - previous)
        delta = float(np.clip(smoothed - previous, -self.config.head_max_step_deg, self.config.head_max_step_deg))
        return float(previous + delta)
