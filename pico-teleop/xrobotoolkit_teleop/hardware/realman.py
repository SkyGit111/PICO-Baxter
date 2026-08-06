"""RealMan dual-arm hardware adapter (arms + head servo + cameras in one file).

This is the full per-robot file. To add a new robot, copy the shape: a Config
dataclass, an Adapter implementing the abstract ``command_*`` hooks, and a
``build_*`` factory.
"""

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np

from xrobotoolkit_teleop.core.camera import CameraStream
from xrobotoolkit_teleop.core.mapping import (
    ArmPoseMapper,
    HeadAngleMapper,
    MappingConfig,
)
from xrobotoolkit_teleop.core.robot_adapter import RobotAdapter
from xrobotoolkit_teleop.core.types import ArmName, BaseTarget, HeadAngles, LiftVelocity, Pose, RobotState


# Make ``control/`` importable for ``realman_robot_controller`` and ``head_servo``.
_CONTROL_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "control")
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


DEFAULT_LEFT_ARM_IP = "169.254.128.18"
DEFAULT_RIGHT_ARM_IP = "169.254.128.19"
DEFAULT_ARM_PORT = 8080
DEFAULT_LOCAL_IP = "169.254.128.20"


@dataclass
class RealmanConfig:
    """All RealMan-specific tuning, hardware addresses, and mapping params."""

    # Arm hardware addresses.
    left_arm_ip: str = DEFAULT_LEFT_ARM_IP
    right_arm_ip: str = DEFAULT_RIGHT_ARM_IP
    arm_port: int = DEFAULT_ARM_PORT
    local_ip: str = DEFAULT_LOCAL_IP
    arms: tuple[ArmName, ...] = ("left_arm", "right_arm")

    # Per-subsystem control rates (each runs in its own thread).
    arm_rate_hz: float = 50.0
    head_rate_hz: float = 25.0
    dry_run: bool = False
    dry_run_log_period_s: float = 0.5

    # Mapping (arm + head).
    mapping: MappingConfig = field(default_factory=MappingConfig)

    # Head servo: enable + serial + servo unit conversion.
    head_servo_enabled: bool = True
    head_servo_port: str = "/dev/ttyUSB0"
    head_servo_baudrate: int = 9600
    head_toggle_button: str = "X"
    # Servo center positions (raw units), allowed range, gain (units per deg), and sign.
    servo1_center: float = 385.0
    servo2_center: float = 500.0
    servo1_min: float = 270.0
    servo1_max: float = 500.0
    servo2_min: float = 430.0
    servo2_max: float = 570.0
    pitch_gain: float = 1.0 / 0.19  # ~deg per unit -> units per deg
    yaw_gain: float = 1.0 / 0.19
    pitch_sign: float = 1.0
    yaw_sign: float = 1.0
    servo_command_deadband: float = 3.0
    servo_move_time_ms: int = 60

    # Mobile base (chassis) — publishes geometry_msgs/Twist on a ROS topic.
    base_enabled: bool = True
    base_cmd_topic: str = "/base_cmd_vel"
    base_max_linear_mps: float = 0.12
    base_max_angular_radps: float = 0.35
    base_deadzone: float = 0.15
    base_rate_hz: float = 20.0

    # Lift — ``rm_set_lift_speed`` on left arm; right thumbstick Y: deadzone = stop,
    # outside deadzone = constant ±``lift_speed_pct`` (no stick-deflection ramp).
    lift_enabled: bool = True
    lift_joystick_deadzone: float = 0.15
    lift_joystick_invert: bool = False
    lift_speed_pct: int = 50
    lift_rate_hz: float = 50.0

    # Cameras.
    head_camera_enabled: bool = True
    head_camera_serial: Optional[str] = None
    head_camera_width: int = 1280
    head_camera_height: int = 720
    head_camera_fps: int = 30


# ---------------------------------------------------------------------------
# Adapter
# ---------------------------------------------------------------------------


class RealmanAdapter(RobotAdapter):
    """Single-file RealMan adapter: arms (IK), head servo, cameras."""

    def __init__(self, config: RealmanConfig | None = None) -> None:
        super().__init__()
        self.config = config or RealmanConfig()
        self.arm_rate_hz = self.config.arm_rate_hz
        self.head_rate_hz = self.config.head_rate_hz
        self.base_rate_hz = self.config.base_rate_hz
        self.lift_rate_hz = self.config.lift_rate_hz
        # ``dt`` here is the arm-rate dt, used by the RealMan SDK as a control period.
        self.dt = 1.0 / self.config.arm_rate_hz

        # Chassis + lift wiring (flipped on via config).
        self.base_enabled = self.config.base_enabled
        self.lift_enabled = self.config.lift_enabled
        self.base_max_linear_mps = self.config.base_max_linear_mps
        self.base_max_angular_radps = self.config.base_max_angular_radps
        self.base_deadzone = self.config.base_deadzone
        self.lift_joystick_deadzone = self.config.lift_joystick_deadzone
        self.lift_joystick_invert = self.config.lift_joystick_invert
        self.lift_speed_pct = self.config.lift_speed_pct
        self._base_pub = None
        self._twist_cls = None

        # Arm mappers (one per arm) — controller<->arm pairing is hard-coded for
        # dual-arm RealMan: left controller drives left arm, right drives right.
        self.arm_mappers = {}
        for arm in self.config.arms:
            controller = "left_controller" if arm == "left_arm" else "right_controller"
            self.arm_mappers[arm] = ArmPoseMapper(
                controller=controller,
                config=self.config.mapping,
                arm_label=arm,
            )

        # Head servo mapper (angles only; servo unit conversion happens here).
        if self.config.head_servo_enabled:
            self.head_mapper = HeadAngleMapper(self.config.mapping)
            self.head_toggle_button = self.config.head_toggle_button

        # Hardware handles populated in connect().
        self.arm_controllers: dict[ArmName, Any] = {}
        self.q_measured_deg: dict[ArmName, np.ndarray] = {}
        self.q_cmd_deg: dict[ArmName, np.ndarray] = {}
        self.gripper_targets: dict[ArmName, float] = {}
        self._ik_params_cls = None
        self._last_dry_log_at: dict[tuple[ArmName, str], float] = {}

        self._servo_client = None
        self._last_sent_servo1: float | None = None
        self._last_sent_servo2: float | None = None

        self._cameras: dict[str, CameraStream] = {}

    # ------------------------------------------------------------------ lifecycle

    def connect(self) -> None:
        self._connect_arms()
        if self.config.head_servo_enabled:
            self._connect_head_servo()
        if self.config.head_camera_enabled:
            self._connect_cameras()
        if self.config.lift_enabled and "left_arm" not in self.arm_controllers:
            print("Warning: lift_enabled but no left_arm connected; disabling lift.")
            self.lift_enabled = False
        if self.config.base_enabled:
            self._connect_base()

    def shutdown(self) -> None:
        print("Shutting down RealMan adapter...")
        # Stop chassis + lift before arms so the robot decelerates predictably.
        if self._base_pub is not None and self._twist_cls is not None:
            try:
                self._base_pub.publish(self._twist_cls())
            except Exception as exc:
                print(f"Warning: failed to publish zero base Twist: {exc}")
        if self.lift_enabled and not self.config.dry_run and "left_arm" in self.arm_controllers:
            try:
                self.arm_controllers["left_arm"].arm.rm_set_lift_speed(0)
            except Exception as exc:
                print(f"Warning: failed to stop lift: {exc}")
        if self.config.dry_run:
            print("Dry-run enabled; no stop commands sent to RealMan hardware.")
        else:
            for arm_name, arm in self.arm_controllers.items():
                try:
                    print(f"Stopping {arm_name}...")
                    arm.stop()
                except Exception as exc:
                    print(f"Warning: error stopping {arm_name}: {exc}")
        try:
            from realman_robot_controller import RealmanRobotController

            if RealmanRobotController.is_initialized():
                RealmanRobotController.reset_instance()
        except Exception as exc:
            print(f"Warning: error during RealMan cleanup: {exc}")

        if self._servo_client is not None:
            try:
                self._servo_client.close()
            except Exception as exc:
                print(f"Warning: error closing servo client: {exc}")

        for cam in self._cameras.values():
            try:
                cam.stop()
            except Exception as exc:
                print(f"Warning: error stopping camera {cam.name}: {exc}")

    # ------------------------------------------------------------------ arms

    def read_state(self) -> RobotState:
        joint_positions: dict[ArmName, np.ndarray] = {}
        gripper_positions: dict[ArmName, float] = {}
        for arm_name, arm in self.arm_controllers.items():
            try:
                q_rad = arm.get_joint_positions()
                q_deg = np.degrees(np.asarray(q_rad, dtype=np.float64))
                self.q_measured_deg[arm_name] = q_deg.copy()
                joint_positions[arm_name] = np.asarray(q_rad, dtype=np.float64).copy()
            except Exception as exc:
                print(f"Warning: failed to read state for {arm_name}: {exc}")
                if arm_name in self.q_measured_deg:
                    joint_positions[arm_name] = np.radians(self.q_measured_deg[arm_name])
            try:
                gripper_positions[arm_name] = float(arm.get_gripper_position())
            except Exception as exc:
                print(f"Warning: failed to read gripper for {arm_name}: {exc}")
                gripper_positions[arm_name] = self.gripper_targets.get(arm_name, 0.0)

        return RobotState(
            timestamp=time.time(),
            joint_positions=joint_positions,
            gripper_positions=gripper_positions,
        )

    def get_end_effector_pose(self, arm: ArmName) -> Pose:
        if arm not in self.q_measured_deg:
            self.read_state()
        position, orientation = self._fk(arm, self.q_measured_deg[arm])
        return Pose(position=position, orientation=orientation)

    def command_end_effector_pose(self, arm: ArmName, pose: Pose) -> None:
        if arm not in self.arm_controllers:
            raise ValueError(f"{arm} is not connected")
        seed = self.q_cmd_deg.get(arm, self.q_measured_deg[arm]).copy()
        q_sol = self._ik(arm, pose.position, pose.orientation, seed)
        if q_sol is None:
            self._log_throttled(
                arm,
                "ik_failed",
                f"RealMan IK failed for {arm}: target={self._pose_str(pose)}",
                force=not self.config.dry_run,
            )
            return

        self.q_cmd_deg[arm] = q_sol.copy()

        if self.config.dry_run:
            self._log_throttled(
                arm,
                "pose",
                f"[dry-run] {arm} IK q_deg={np.round(q_sol, 3).tolist()} target={self._pose_str(pose)}",
            )
            return

        try:
            self.arm_controllers[arm].set_joint_positions(np.radians(q_sol))
        except Exception as exc:
            print(f"Warning: failed to send arm command for {arm}: {exc}")

    def command_gripper(self, arm: ArmName, value: float) -> None:
        value = float(np.clip(value, 0.0, 1.0))
        self.gripper_targets[arm] = value
        if self.config.dry_run:
            self._log_throttled(arm, "gripper", f"[dry-run] {arm} gripper={value:.3f}")
            return
        if arm not in self.arm_controllers:
            return
        try:
            self.arm_controllers[arm].set_gripper_position(value)
        except Exception as exc:
            print(f"Warning: failed to send gripper command for {arm}: {exc}")

    # ------------------------------------------------------------------ head servo

    def command_head_servo(self, target: HeadAngles) -> None:
        """Convert (yaw, pitch) deg -> servo units, clamp, and send."""
        if self._servo_client is None:
            return

        servo1_raw = self.config.servo1_center + (self.config.pitch_sign * self.config.pitch_gain * target.pitch_deg)
        servo2_raw = self.config.servo2_center + (self.config.yaw_sign * self.config.yaw_gain * target.yaw_deg)
        servo1 = float(np.clip(servo1_raw, self.config.servo1_min, self.config.servo1_max))
        servo2 = float(np.clip(servo2_raw, self.config.servo2_min, self.config.servo2_max))

        # Skip tiny commands (deadband) unless this is the first send.
        if self._last_sent_servo1 is not None and self._last_sent_servo2 is not None:
            d1 = abs(servo1 - self._last_sent_servo1)
            d2 = abs(servo2 - self._last_sent_servo2)
            if d1 <= self.config.servo_command_deadband and d2 <= self.config.servo_command_deadband:
                return

        try:
            self._servo_client.servo_move(
                servo1_angle=servo1,
                servo2_angle=servo2,
                wait_time=0.0,
                move_time_ms=self.config.servo_move_time_ms,
            )
            self._last_sent_servo1 = servo1
            self._last_sent_servo2 = servo2
        except Exception as exc:
            print(f"Warning: failed to send head servo command: {exc}")

    # ------------------------------------------------------------------ base (chassis)

    def command_base(self, target: BaseTarget) -> None:
        if self._base_pub is None or self._twist_cls is None:
            if self.config.dry_run:
                self._log_throttled(
                    "left_arm",  # reuse the throttle keyspace; the topic label distinguishes
                    "base",
                    f"[dry-run] base vx={target.vx:.3f} vyaw={target.vyaw:.3f}",
                )
            return
        msg = self._twist_cls()
        msg.linear.x = float(target.vx)
        msg.angular.z = float(target.vyaw)
        try:
            self._base_pub.publish(msg)
        except Exception as exc:
            print(f"Warning: failed to publish base Twist: {exc}")

    # ------------------------------------------------------------------ lift

    def command_lift_velocity(self, target: LiftVelocity) -> None:
        speed = int(max(-100, min(100, target.speed_pct)))
        if self.config.dry_run:
            self._log_throttled("left_arm", "lift", f"[dry-run] lift speed_pct={speed}")
            return
        arm = self.arm_controllers.get("left_arm")
        if arm is None:
            return
        try:
            arm.arm.rm_set_lift_speed(speed)
        except Exception as exc:
            print(f"Warning: failed to send lift speed command: {exc}")

    # ------------------------------------------------------------------ cameras

    def cameras(self) -> dict[str, CameraStream]:
        return dict(self._cameras)

    # ------------------------------------------------------------------ internals

    def _connect_arms(self) -> None:
        from xrobotoolkit_teleop.hardware.interface.realman import RealmanArmInterface
        from Robotic_Arm.rm_robot_interface import rm_inverse_kinematics_params_t
        from realman_robot_controller import RealmanRobotController

        self._ik_params_cls = rm_inverse_kinematics_params_t

        if not RealmanRobotController.is_initialized():
            RealmanRobotController.get_instance(
                left_arm_ip=self.config.left_arm_ip,
                right_arm_ip=self.config.right_arm_ip,
                arm_port=self.config.arm_port,
                local_ip=self.config.local_ip,
                init_node=True,
            )

        for arm_name in self.config.arms:
            arm_ip = self.config.left_arm_ip if arm_name == "left_arm" else self.config.right_arm_ip
            mode = "dry-run" if self.config.dry_run else "hardware"
            print(f"Connecting RealMan {arm_name} at {arm_ip}:{self.config.arm_port} ({mode})")
            arm = RealmanArmInterface(
                arm_name=arm_name,
                robot_ip=arm_ip,
                robot_port=self.config.arm_port,
                local_ip=self.config.local_ip,
                dt=self.dt,
            )
            self.arm_controllers[arm_name] = arm

        if not self.arm_controllers:
            raise RuntimeError("No RealMan arm controllers were initialized")

        self.read_state()
        for arm_name, q_deg in self.q_measured_deg.items():
            self.q_cmd_deg[arm_name] = q_deg.copy()
            self.gripper_targets[arm_name] = 0.0

    def _connect_head_servo(self) -> None:
        from head_servo import SerialServoClient

        try:
            self._servo_client = SerialServoClient(
                port=self.config.head_servo_port,
                baudrate=self.config.head_servo_baudrate,
            )
        except Exception as exc:
            print(f"Warning: head servo unavailable ({exc}); head following disabled.")
            self._servo_client = None
            self.head_mapper = None

    def _connect_cameras(self) -> None:
        try:
            from xrobotoolkit_teleop.hardware.realman_camera import RealmanCamera
        except Exception as exc:
            print(f"Warning: head camera unavailable ({exc}); skipping.")
            return
        try:
            cam = RealmanCamera(
                name="head",
                width=self.config.head_camera_width,
                height=self.config.head_camera_height,
                fps=self.config.head_camera_fps,
                serial=self.config.head_camera_serial,
            )
            cam.start()
            self._cameras["head"] = cam
        except Exception as exc:
            print(f"Warning: failed to start head camera ({exc}); skipping.")

    def _connect_base(self) -> None:
        """Set up the ROS publisher for the chassis Twist topic.

        We rely on the ROS node already being initialized by
        ``RealmanRobotController`` (it calls ``rospy.init_node`` when constructed
        with ``init_node=True``, which we do in ``_connect_arms``). If we ever
        run without arms, this would need its own ``rospy.init_node`` guard.
        """
        try:
            import rospy
            from geometry_msgs.msg import Twist
        except Exception as exc:
            print(f"Warning: base_enabled but ROS is unavailable ({exc}); disabling base.")
            self.base_enabled = False
            return
        try:
            self._twist_cls = Twist
            self._base_pub = rospy.Publisher(self.config.base_cmd_topic, Twist, queue_size=10)
            print(f"Chassis: publishing Twist on {self.config.base_cmd_topic}")
        except Exception as exc:
            print(f"Warning: failed to create base publisher ({exc}); disabling base.")
            self.base_enabled = False
            self._base_pub = None
            self._twist_cls = None

    def _fk(self, arm_name: ArmName, q_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        arm = self.arm_controllers[arm_name].arm
        pose = arm.rm_algo_forward_kinematics(q_deg.tolist(), 0)
        pose = np.asarray(pose, dtype=np.float64)
        return pose[:3].copy(), pose[3:7].copy()

    def _ik(
        self,
        arm_name: ArmName,
        target_xyz: np.ndarray,
        target_quat_wxyz: np.ndarray,
        seed_deg: np.ndarray,
    ) -> np.ndarray | None:
        arm = self.arm_controllers[arm_name].arm
        q_pose = [
            float(target_xyz[0]),
            float(target_xyz[1]),
            float(target_xyz[2]),
            float(target_quat_wxyz[0]),
            float(target_quat_wxyz[1]),
            float(target_quat_wxyz[2]),
            float(target_quat_wxyz[3]),
        ]
        params = self._ik_params_cls(q_in=seed_deg.tolist(), q_pose=q_pose, flag=0)
        ret, q_out = arm.rm_algo_inverse_kinematics(params)
        if ret != 0:
            return None
        return np.asarray(q_out, dtype=np.float64)

    def _log_throttled(self, arm: ArmName, topic: str, message: str, force: bool = False) -> None:
        if force:
            print(message)
            return
        now = time.time()
        key = (arm, topic)
        if now - self._last_dry_log_at.get(key, 0.0) >= self.config.dry_run_log_period_s:
            print(message)
            self._last_dry_log_at[key] = now

    @staticmethod
    def _pose_str(pose: Pose) -> str:
        pos = np.round(pose.position, 4).tolist()
        quat = np.round(pose.orientation, 4).tolist()
        return f"pos={pos}, quat_wxyz={quat}"


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------


def build_realman(config: RealmanConfig | None = None) -> RealmanAdapter:
    """Construct a ready-to-``connect()`` RealMan adapter with default config."""
    return RealmanAdapter(config or RealmanConfig())
