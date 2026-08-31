#!/usr/bin/env python3
"""
Precision PICO -> Baxter teleoperation for one arm.

This is the stable follow-up to baxter_live_microtest.py.  It keeps the
verified transport, coordinate mapping and Baxter safety rules, then adds:

- independent XR receive and 50 Hz robot-control loops;
- first-order low-pass filtering of controller position;
- a moving millimetre-scale hand deadband that remains active everywhere;
- a continuous braking-aware Cartesian target servo with speed and acceleration caps;
- the last successful joint command as the next IK user seed;
- asynchronous latest-only IK, backtracking and skip-with-hold behaviour;
- Grip hysteresis, release-to-rearm and packet/robot safety stops;
- zero-motion arming, fixed lock orientation, and strict first-IK continuity.

Deliberate scope:
- one arm only;
- translation only (controller rotation is ignored);
- Grip is the dead-man switch;
- no gripper command is sent (Trigger is reserved for a later, separately
  verified gripper stage);
- no automatic robot enable;
- normal POSITION_MODE only, never RAW_POSITION_MODE.

Expected UDP packet fields are the same as baxter_live_microtest.py:

    {
        "version": 1,
        "mode": "real",
        "seq": 123,
        "sent_monotonic_ns": 123456789,
        "valid": true,
        "left":  {"pose": {"position": [x, y, z]}, "grip": 0.0},
        "right": {"pose": {"position": [x, y, z]}, "grip": 0.0}
    }

Run this script with the system Python that can import rospy and
baxter_interface after sourcing baxter.sh.  It intentionally refuses to send
commands unless --execute is present.
"""

import argparse
import csv
import json
import math
import os
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import baxter_interface
import rospy

from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


PROTOCOL_VERSION = 1
DEFAULT_BAXTER_SPEED_RATIO = 0.30
Vector3 = List[float]
JointDict = Dict[str, float]


@dataclass
class QuaternionValue:
    x: float
    y: float
    z: float
    w: float


def copy_quaternion(value: Any) -> QuaternionValue:
    """Copy a Baxter/ROS quaternion into a small mutable-independent value."""
    return QuaternionValue(
        x=float(value.x),
        y=float(value.y),
        z=float(value.z),
        w=float(value.w),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stable, precision PICO translation teleoperation for Baxter."
    )

    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=15000)

    # Near 1:1 mapping by default.  Calibrate --scale only after checking the
    # logged hand_delta and measured endpoint displacement.
    parser.add_argument("--scale", type=float, default=1.00)
    parser.add_argument(
        "--max-translation",
        type=float,
        default=0.0,
        help="Maximum endpoint displacement from the Grip-lock pose in metres; 0 disables.",
    )
    parser.add_argument(
        "--hand-filter-tau",
        type=float,
        default=0.03,
        help="Controller-position low-pass time constant in seconds; 0 disables.",
    )
    parser.add_argument(
        "--hand-deadzone",
        type=float,
        default=0.0007,
        help="Moving hand deadband radius in controller metres; 0 disables.",
    )
    parser.add_argument(
        "--motion-start-threshold",
        type=float,
        default=0.003,
        help=(
            "Hand displacement required after Grip lock before IK motion starts. "
            "Grip alone therefore cannot move the arm."
        ),
    )

    # Independent Cartesian servo target.  These limits apply before IK.
    parser.add_argument("--control-rate-hz", type=float, default=50.0)
    parser.add_argument(
        "--command-rate-hz",
        type=float,
        default=50.0,
        help="Independent rate that continuously republishes the latest accepted joint command.",
    )
    parser.add_argument("--target-time-constant", type=float, default=0.04)
    parser.add_argument("--max-ee-speed", type=float, default=0.45)
    parser.add_argument("--max-ee-accel", type=float, default=2.00)
    parser.add_argument(
        "--max-command-lead",
        type=float,
        default=0.08,
        help=(
            "Maximum Cartesian command lead ahead of the measured endpoint in metres; "
            "0 disables. This does not limit total travel."
        ),
    )
    parser.add_argument(
        "--max-control-dt",
        type=float,
        default=0.05,
        help="Largest dt used by the target generator after a scheduling pause.",
    )

    # Baxter position-control and IK continuity limits.
    parser.add_argument("--speed-ratio", type=float, default=0.80)
    parser.add_argument(
        "--max-joint-step",
        type=float,
        default=0.12,
        help=(
            "Maximum commanded joint lead from the measured angle in radians per IK update; "
            "0 disables. This does not limit total joint travel."
        ),
    )
    parser.add_argument(
        "--max-ik-difference",
        type=float,
        default=0.25,
        help=(
            "Reject consecutive IK branches farther than this; 0 disables. "
            "This is a branch-continuity test, not a total joint-motion limit."
        ),
    )
    parser.add_argument(
        "--max-first-ik-difference",
        type=float,
        default=0.10,
        help=(
            "Maximum joint difference between the Grip-lock joint pose and the "
            "first accepted IK solution; 0 disables."
        ),
    )
    parser.add_argument("--ik-backtrack-attempts", type=int, default=3)
    parser.add_argument(
        "--ik-backend",
        choices=("ros-service", "pykdl"),
        default="ros-service",
        help=(
            "IK implementation. ros-service preserves the existing Baxter "
            "path; pykdl is an opt-in local backend that requires baxter_pykdl."
        ),
    )
    parser.add_argument(
        "--ik-budget-ms",
        type=float,
        default=90.0,
        help="Stop launching additional IK backtracking calls after this elapsed budget.",
    )
    parser.add_argument(
        "--max-ik-result-age",
        type=float,
        default=0.25,
        help=(
            "Reject an asynchronous IK result this many seconds after submission. "
            "This bounds target staleness but cannot cancel a running ROS call."
        ),
    )
    parser.add_argument(
        "--orientation-relax-after",
        type=int,
        default=0,
        help=(
            "After this many consecutive strict-orientation IK misses, retry "
            "toward the measured tool orientation; 0 disables relaxation."
        ),
    )
    parser.add_argument(
        "--max-orientation-relax-deg",
        type=float,
        default=3.0,
        help="Maximum temporary tool-orientation relaxation in degrees.",
    )

    # Input freshness and dead-man hysteresis.
    parser.add_argument("--max-age", type=float, default=0.50)
    parser.add_argument("--connection-timeout", type=float, default=0.75)
    parser.add_argument("--grip-press", type=float, default=0.60)
    parser.add_argument("--grip-release", type=float, default=0.40)
    parser.add_argument("--status-rate-hz", type=float, default=5.0)
    parser.add_argument(
        "--csv-log",
        default="",
        help=(
            "Optional CSV diagnostics path. Parent directories are created. "
            "Example: ~/pico_baxter_ws/logs/precision.csv"
        ),
    )

    parser.add_argument(
        "--execute",
        action="store_true",
        help="Required before this script may send joint position commands.",
    )

    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not args.execute:
        raise SystemExit("Refusing to command the robot without --execute.")

    if not (0.0 < args.scale <= 10.0):
        raise ValueError("--scale must be in (0, 10.0].")
    if args.max_translation < 0.0:
        raise ValueError("--max-translation must be non-negative.")
    if args.hand_filter_tau < 0.0:
        raise ValueError("--hand-filter-tau must be non-negative.")
    if args.hand_deadzone < 0.0:
        raise ValueError("--hand-deadzone must be non-negative.")
    if args.motion_start_threshold < 0.0:
        raise ValueError("--motion-start-threshold must be non-negative.")
    if args.control_rate_hz <= 0.0:
        raise ValueError("--control-rate-hz must be positive.")
    if args.command_rate_hz <= 0.0:
        raise ValueError("--command-rate-hz must be positive.")
    if args.target_time_constant <= 0.0:
        raise ValueError("--target-time-constant must be positive.")
    if args.max_ee_speed <= 0.0:
        raise ValueError("--max-ee-speed must be positive.")
    if args.max_ee_accel <= 0.0:
        raise ValueError("--max-ee-accel must be positive.")
    if args.max_command_lead < 0.0:
        raise ValueError("--max-command-lead must be non-negative.")
    if args.max_control_dt <= 0.0:
        raise ValueError("--max-control-dt must be positive.")
    if not (0.0 < args.speed_ratio <= 1.0):
        raise ValueError("--speed-ratio must be in (0, 1.0].")
    if args.max_joint_step < 0.0:
        raise ValueError("--max-joint-step must be non-negative.")
    if args.max_ik_difference < 0.0:
        raise ValueError("--max-ik-difference must be non-negative.")
    if args.max_first_ik_difference < 0.0:
        raise ValueError("--max-first-ik-difference must be non-negative.")
    if args.ik_backtrack_attempts < 1:
        raise ValueError("--ik-backtrack-attempts must be at least 1.")
    if args.ik_budget_ms <= 0.0:
        raise ValueError("--ik-budget-ms must be positive.")
    if args.max_ik_result_age <= 0.0:
        raise ValueError("--max-ik-result-age must be positive.")
    if args.orientation_relax_after < 0:
        raise ValueError("--orientation-relax-after must be non-negative.")
    if not (0.0 <= args.max_orientation_relax_deg <= 15.0):
        raise ValueError("--max-orientation-relax-deg must be in [0, 15].")
    if args.max_age <= 0.0:
        raise ValueError("--max-age must be positive.")
    if args.connection_timeout <= 0.0:
        raise ValueError("--connection-timeout must be positive.")
    if not (0.0 <= args.grip_release < args.grip_press <= 1.0):
        raise ValueError(
            "Grip thresholds must satisfy 0 <= release < press <= 1."
        )
    if args.status_rate_hz <= 0.0:
        raise ValueError("--status-rate-hz must be positive.")
    if not (0 < args.port <= 65535):
        raise ValueError("--port must be in [1, 65535].")


def finite_vector3(value: Any) -> Optional[Vector3]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(item) for item in result):
        return None
    return result


def controller_position(packet: Dict[str, Any], side: str) -> Optional[Vector3]:
    controller = packet.get(side)
    if not isinstance(controller, dict):
        return None
    pose = controller.get("pose")
    if not isinstance(pose, dict):
        return None
    return finite_vector3(pose.get("position"))


def controller_grip(packet: Dict[str, Any], side: str) -> float:
    controller = packet.get(side)
    if not isinstance(controller, dict):
        return 0.0
    try:
        value = float(controller.get("grip", 0.0))
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(value):
        return 0.0
    return max(0.0, min(1.0, value))


def packet_age(packet: Dict[str, Any]) -> float:
    sent_ns = packet.get("sent_monotonic_ns")
    if not isinstance(sent_ns, int):
        return float("inf")
    return (time.monotonic_ns() - sent_ns) / 1_000_000_000.0


def add(a: Vector3, b: Vector3) -> Vector3:
    return [a[index] + b[index] for index in range(3)]


def subtract(a: Vector3, b: Vector3) -> Vector3:
    return [a[index] - b[index] for index in range(3)]


def multiply(vector: Vector3, scalar: float) -> Vector3:
    return [value * scalar for value in vector]


def dot(a: Vector3, b: Vector3) -> float:
    return sum(a[index] * b[index] for index in range(3))


def vector_norm(vector: Vector3) -> float:
    return math.sqrt(dot(vector, vector))


def clamp_vector_norm(vector: Vector3, maximum: float) -> Vector3:
    length = vector_norm(vector)
    if length <= maximum or length <= 1e-12:
        return list(vector)
    return multiply(vector, maximum / length)


class MovingDeadbandVector3:
    """
    Stateful deadband that follows the hand.

    The previous implementation applied a deadzone only around the Grip-lock
    origin, so it stopped suppressing jitter after the hand had moved away.
    This filter keeps its output fixed while the sample remains inside a small
    sphere around the current output.  Once the sample leaves that sphere, the
    output follows while retaining only the configured deadband distance.
    """

    def __init__(self, radius: float) -> None:
        self.radius = radius
        self.value: Optional[Vector3] = None

    def reset(self, value: Optional[Vector3] = None) -> None:
        self.value = None if value is None else list(value)

    def update(self, sample: Vector3) -> Vector3:
        if self.value is None or self.radius <= 0.0:
            self.value = list(sample)
            return list(self.value)

        displacement = subtract(sample, self.value)
        distance = vector_norm(displacement)
        if distance <= self.radius or distance <= 1e-12:
            return list(self.value)

        # Move only the amount outside the deadband. This preserves fine
        # cumulative motion while rejecting local tracking noise everywhere.
        self.value = subtract(
            sample,
            multiply(displacement, self.radius / distance),
        )
        return list(self.value)


def map_pico_to_baxter(pico_delta: Vector3, scale: float) -> Vector3:
    """
    Preserve the signs used by the latest successful Baxter live test.

    Exact implementation:
        baxter_dx = -scale * pico_dz
        baxter_dy = -scale * pico_dx
        baxter_dz = +scale * pico_dy

    Do not change these signs while tuning precision parameters; axis changes
    should be verified in a separate small-displacement test.
    """
    pico_x, pico_y, pico_z = pico_delta
    return [
        -scale * pico_z,
        -scale * pico_x,
        scale * pico_y,
    ]


class LowPassVector3:
    def __init__(self, time_constant: float) -> None:
        self.time_constant = time_constant
        self.value: Optional[Vector3] = None

    def reset(self, value: Optional[Vector3] = None) -> None:
        self.value = None if value is None else list(value)

    def update(self, sample: Vector3, dt: float) -> Vector3:
        if self.value is None or self.time_constant <= 0.0:
            self.value = list(sample)
            return list(self.value)

        safe_dt = max(0.0, dt)
        alpha = 1.0 - math.exp(-safe_dt / self.time_constant)
        self.value = [
            self.value[index]
            + alpha * (sample[index] - self.value[index])
            for index in range(3)
        ]
        return list(self.value)


class CartesianTargetLimiter:
    """
    Continuous Cartesian target servo with speed and acceleration caps.

    The previous limiter hard-reset velocity to zero whenever the desired
    direction reversed. That produced stop/restart discontinuities. This
    version generates a desired velocity toward the target, includes a
    braking-speed bound, and acceleration-limits the transition without a
    hard zero on reversal.
    """

    def __init__(
        self,
        maximum_speed: float,
        maximum_acceleration: float,
        target_time_constant: float,
    ) -> None:
        self.maximum_speed = maximum_speed
        self.maximum_acceleration = maximum_acceleration
        self.target_time_constant = target_time_constant
        self.position: Optional[Vector3] = None
        self.velocity: Vector3 = [0.0, 0.0, 0.0]
        self.speed_saturated = False
        self.acceleration_saturated = False

    def reset(self, position: Optional[Vector3] = None) -> None:
        self.position = None if position is None else list(position)
        self.velocity = [0.0, 0.0, 0.0]
        self.speed_saturated = False
        self.acceleration_saturated = False

    def update(self, desired_position: Vector3, dt: float) -> Vector3:
        if self.position is None:
            self.reset(desired_position)
            return list(desired_position)

        dt = max(dt, 1e-6)
        error = subtract(desired_position, self.position)
        distance = vector_norm(error)

        if distance <= 1e-7 and vector_norm(self.velocity) <= 1e-5:
            self.position = list(desired_position)
            self.velocity = [0.0, 0.0, 0.0]
            self.speed_saturated = False
            self.acceleration_saturated = False
            return list(self.position)

        if distance <= 1e-12:
            desired_velocity = [0.0, 0.0, 0.0]
            requested_speed = 0.0
        else:
            direction = multiply(error, 1.0 / distance)
            proportional_speed = distance / self.target_time_constant
            braking_speed = math.sqrt(
                max(0.0, 2.0 * self.maximum_acceleration * distance)
            )
            requested_speed = proportional_speed
            target_speed = min(
                proportional_speed,
                braking_speed,
                self.maximum_speed,
            )
            desired_velocity = multiply(direction, target_speed)

        self.speed_saturated = requested_speed > self.maximum_speed + 1e-9

        raw_velocity_change = subtract(desired_velocity, self.velocity)
        maximum_velocity_change = self.maximum_acceleration * dt
        self.acceleration_saturated = (
            vector_norm(raw_velocity_change)
            > maximum_velocity_change + 1e-9
        )
        velocity_change = clamp_vector_norm(
            raw_velocity_change,
            maximum_velocity_change,
        )
        next_velocity = add(self.velocity, velocity_change)
        next_velocity = clamp_vector_norm(next_velocity, self.maximum_speed)

        step = multiply(next_velocity, dt)

        # Snap only when the step would cross the target in the target
        # direction. During a reversal, the velocity decelerates continuously
        # instead of being reset to zero.
        if (
            distance > 0.0
            and dot(step, error) > 0.0
            and vector_norm(step) >= distance
        ):
            self.position = list(desired_position)
            self.velocity = [0.0, 0.0, 0.0]
        else:
            self.position = add(self.position, step)
            self.velocity = next_velocity

        return list(self.position)

    def enforce_position(self, accepted: Vector3) -> None:
        """Apply anti-windup without discarding the current motion direction."""
        self.position = list(accepted)
        self.velocity = clamp_vector_norm(self.velocity, self.maximum_speed)

    def accept_backtracked_position(
        self,
        accepted: Vector3,
        alpha: float,
    ) -> None:
        """Align with a reachable IK target while preserving partial continuity."""
        self.position = list(accepted)
        safe_alpha = max(0.0, min(1.0, alpha))
        self.velocity = multiply(self.velocity, safe_alpha)
        self.speed_saturated = False
        self.acceleration_saturated = False


class CsvDiagnostics:
    """Buffered 50 Hz diagnostics for separating input, servo and robot lag."""

    FIELDNAMES = [
        "wall_time",
        "event",
        "seq",
        "packet_age_ms",
        "grip",
        "loop_dt_ms",
        "raw_hand_dx",
        "raw_hand_dy",
        "raw_hand_dz",
        "hand_dx",
        "hand_dy",
        "hand_dz",
        "robot_dx",
        "robot_dy",
        "robot_dz",
        "desired_lag_m",
        "tracking_error_m",
        "joint_tracking_error_rad",
        "command_speed_mps",
        "ik_backend",
        "ik_time_ms",
        "ik_result_age_ms",
        "ik_request_id",
        "ik_request_seq",
        "ik_replaced_pending",
        "ik_alpha",
        "ik_diff_rad",
        "ik_code",
        "translation_saturated",
        "command_lead_saturated",
        "speed_saturated",
        "acceleration_saturated",
    ]

    def __init__(self, path: str) -> None:
        self._file = None
        self._writer = None
        self._last_flush = time.monotonic()
        if not path:
            return

        expanded = os.path.abspath(os.path.expanduser(path))
        parent = os.path.dirname(expanded)
        if parent:
            os.makedirs(parent, exist_ok=True)
        self._file = open(expanded, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()
        print("CSV diagnostics:       {}".format(expanded))

    @property
    def enabled(self) -> bool:
        return self._writer is not None

    def write(self, row: Dict[str, Any]) -> None:
        if self._writer is None:
            return
        complete = {name: row.get(name, "") for name in self.FIELDNAMES}
        self._writer.writerow(complete)
        now = time.monotonic()
        if now - self._last_flush >= 1.0:
            assert self._file is not None
            self._file.flush()
            self._last_flush = now

    def close(self) -> None:
        if self._file is not None:
            self._file.flush()
            self._file.close()
            self._file = None
            self._writer = None


@dataclass
class ReceiverSnapshot:
    serial: int
    packet: Optional[Dict[str, Any]]
    received_monotonic: Optional[float]
    error: Optional[str]


class LatestPacketReceiver:
    """Continuously drain UDP and expose only the newest datagram."""

    def __init__(self, bind_host: str, port: int) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((bind_host, port))
        self._socket.settimeout(0.05)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="pico-udp-receiver",
            daemon=True,
        )
        self._snapshot = ReceiverSnapshot(0, None, None, None)

    def start(self) -> None:
        self._thread.start()

    def snapshot(self) -> ReceiverSnapshot:
        with self._lock:
            return ReceiverSnapshot(
                serial=self._snapshot.serial,
                packet=self._snapshot.packet,
                received_monotonic=self._snapshot.received_monotonic,
                error=self._snapshot.error,
            )

    def close(self) -> None:
        self._stop_event.set()
        try:
            self._socket.close()
        except OSError:
            pass
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _publish(
        self,
        packet: Optional[Dict[str, Any]],
        received_monotonic: float,
        error: Optional[str],
    ) -> None:
        with self._lock:
            self._snapshot = ReceiverSnapshot(
                serial=self._snapshot.serial + 1,
                packet=packet,
                received_monotonic=received_monotonic,
                error=error,
            )

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                encoded, _source = self._socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break

            received = time.monotonic()

            # Drain queued packets so the control loop never chases old poses.
            self._socket.setblocking(False)
            try:
                while True:
                    encoded, _source = self._socket.recvfrom(65535)
                    received = time.monotonic()
            except (BlockingIOError, socket.timeout):
                pass
            except OSError as exc:
                if self._stop_event.is_set():
                    break
                self._publish(
                    None,
                    time.monotonic(),
                    "UDP receive error: {}".format(exc),
                )
                continue
            finally:
                try:
                    self._socket.settimeout(0.05)
                except OSError:
                    pass

            try:
                decoded = json.loads(encoded.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                self._publish(None, received, "invalid UDP payload: {}".format(exc))
                continue

            if not isinstance(decoded, dict):
                self._publish(None, received, "UDP JSON root is not an object")
                continue

            self._publish(decoded, received, None)
class ContinuousJointCommandPublisher:
    """Republish the latest accepted joint command independently of slow IK calls."""

    def __init__(self, limb: Any, rate_hz: float) -> None:
        self._limb = limb
        self._period = 1.0 / rate_hz
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="baxter-joint-command-publisher",
            daemon=True,
        )
        self._command: Optional[JointDict] = None
        self._error: Optional[str] = None

    def start(self) -> None:
        self._thread.start()

    def set_command(self, command: JointDict) -> None:
        with self._lock:
            self._command = dict(command)

    def error(self) -> Optional[str]:
        with self._lock:
            return self._error

    def close(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        next_time = time.monotonic()
        while not self._stop_event.is_set() and not rospy.is_shutdown():
            with self._lock:
                command = None if self._command is None else dict(self._command)
            if command:
                try:
                    self._limb.set_joint_positions(command, raw=False)
                except Exception as exc:
                    with self._lock:
                        self._error = "joint publisher failed: {}".format(exc)
                    return

            next_time += self._period
            delay = next_time - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                next_time = time.monotonic()


def robot_is_ready(robot_enable: Any) -> Tuple[bool, str]:
    state = robot_enable.state()
    if state is None:
        return False, "robot state unavailable"
    if not state.enabled:
        return False, "robot is not enabled"
    if state.stopped:
        return False, "robot is stopped"
    if state.error:
        return False, "robot reports an error"
    return True, "ready"


def endpoint_position(endpoint: Dict[str, Any]) -> Vector3:
    return [
        float(endpoint["position"].x),
        float(endpoint["position"].y),
        float(endpoint["position"].z),
    ]


def create_target_pose(position: Vector3, orientation: Any) -> PoseStamped:
    target = PoseStamped()
    target.header.stamp = rospy.Time.now()
    target.header.frame_id = "base"
    target.pose.position.x = position[0]
    target.pose.position.y = position[1]
    target.pose.position.z = position[2]
    target.pose.orientation.x = orientation.x
    target.pose.orientation.y = orientation.y
    target.pose.orientation.z = orientation.z
    target.pose.orientation.w = orientation.w
    return target


def decode_result_code(response: Any) -> int:
    if not response.result_type:
        return int(response.RESULT_INVALID)
    code = response.result_type[0]
    if isinstance(code, str):
        return ord(code)
    if isinstance(code, (bytes, bytearray)):
        return int(code[0])
    return int(code)


def quaternion_distance(a: Any, b: Any) -> float:
    """Return the shortest angular distance between two quaternions."""
    dot = abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def relax_orientation(reference: Any, measured: Any, max_angle: float) -> Any:
    """Move reference toward measured by at most max_angle using quaternion SLERP."""
    values_a = [reference.x, reference.y, reference.z, reference.w]
    values_b = [measured.x, measured.y, measured.z, measured.w]
    dot = sum(a * b for a, b in zip(values_a, values_b))
    if dot < 0.0:
        values_b = [-value for value in values_b]
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    distance = 2.0 * math.acos(dot)
    if distance <= 1e-9 or max_angle <= 0.0:
        return QuaternionValue(*values_a)
    fraction = min(1.0, max_angle / distance)
    if dot > 0.9995:
        values = [
            a + fraction * (b - a) for a, b in zip(values_a, values_b)
        ]
        norm = math.sqrt(sum(value * value for value in values))
        values = [value / norm for value in values]
    else:
        theta = math.acos(dot)
        scale_a = math.sin((1.0 - fraction) * theta) / math.sin(theta)
        scale_b = math.sin(fraction * theta) / math.sin(theta)
        values = [
            scale_a * a + scale_b * b for a, b in zip(values_a, values_b)
        ]
    return QuaternionValue(*values)


def solve_ik(
    service: Any,
    target: PoseStamped,
    seed_angles: Optional[JointDict],
) -> Tuple[bool, JointDict, int]:
    request = SolvePositionIKRequest()
    request.pose_stamp.append(target)

    if seed_angles:
        request.seed_mode = request.SEED_USER
        seed = JointState()
        seed.name = sorted(seed_angles)
        seed.position = [seed_angles[name] for name in seed.name]
        request.seed_angles.append(seed)
    else:
        request.seed_mode = request.SEED_CURRENT

    response = service(request)
    code = decode_result_code(response)
    valid = (
        code != int(response.RESULT_INVALID)
        and bool(response.joints)
        and bool(response.joints[0].name)
    )
    if not valid:
        return False, {}, code

    solution = {
        name: float(position)
        for name, position in zip(
            response.joints[0].name,
            response.joints[0].position,
        )
    }
    if not solution or not all(math.isfinite(value) for value in solution.values()):
        return False, {}, code
    return True, solution, code


def solve_ik_with_backtracking(
    service: Any,
    current_position: Vector3,
    current_orientation: Any,
    desired_position: Vector3,
    seed_angles: Optional[JointDict],
    attempts: int,
    budget_ms: float,
) -> Tuple[bool, JointDict, int, Optional[PoseStamped], float]:
    """Try a small number of local Cartesian targets within a time budget."""
    direction = subtract(desired_position, current_position)
    last_code = 0
    started = time.monotonic()

    for attempt in range(attempts):
        if attempt > 0 and (time.monotonic() - started) * 1000.0 >= budget_ms:
            break

        alpha = 0.5 ** attempt
        partial_position = add(current_position, multiply(direction, alpha))
        target = create_target_pose(partial_position, current_orientation)
        valid, solution, result_code = solve_ik(
            service,
            target,
            seed_angles,
        )
        last_code = result_code
        if valid:
            return True, solution, result_code, target, alpha

    # Do not fall back to SEED_CURRENT during an active Grip session.
    # Baxter is kinematically redundant; a current-seed fallback can select a
    # different elbow/wrist configuration for the same tool pose.  We prefer
    # holding the previous command over changing IK branch.

    return False, {}, last_code, None, 0.0


@dataclass
class IkWorkItem:
    request_id: int
    session_id: int
    sequence: int
    submitted_monotonic: float
    current_position: Vector3
    current_orientation: QuaternionValue
    desired_position: Vector3
    seed_angles: Optional[JointDict]
    attempts: int
    budget_ms: float


@dataclass
class IkWorkResult:
    request: IkWorkItem
    started_monotonic: float
    completed_monotonic: float
    valid: bool
    solution: JointDict
    result_code: int
    accepted_target: Optional[PoseStamped]
    accepted_alpha: float
    error: Optional[str]

    @property
    def solve_time_ms(self) -> float:
        return (self.completed_monotonic - self.started_monotonic) * 1000.0

    @property
    def total_age_ms(self) -> float:
        return (
            self.completed_monotonic - self.request.submitted_monotonic
        ) * 1000.0


@dataclass
class IkWorkerSnapshot:
    in_flight: bool
    pending_request_id: Optional[int]
    completed_request_id: Optional[int]
    replaced_pending: int


class RosServiceIkBackend:
    """Compatibility backend for Baxter's blocking PositionKinematicsNode."""

    name = "ros-service"

    def __init__(self, service: Any) -> None:
        self._service = service

    def solve(
        self,
        request: IkWorkItem,
    ) -> Tuple[bool, JointDict, int, Optional[PoseStamped], float]:
        return solve_ik_with_backtracking(
            self._service,
            request.current_position,
            request.current_orientation,
            request.desired_position,
            request.seed_angles,
            request.attempts,
            request.budget_ms,
        )


class PyKdlIkBackend:
    """Opt-in local IK backend using ``baxter_pykdl``.

    The backend intentionally shares the same latest-only worker, user seed,
    Cartesian backtracking, age rejection and main-thread continuity checks as
    the ROS service backend.  It does not add collision checking; callers must
    treat it as experimental until it has passed the read-only benchmark and
    low-speed robot validation.
    """

    name = "pykdl"

    def __init__(self, kinematics: Any, joint_names: List[str]) -> None:
        if not joint_names:
            raise ValueError("joint_names must not be empty")
        self._kinematics = kinematics
        self._joint_names = list(joint_names)

    def solve(
        self,
        request: IkWorkItem,
    ) -> Tuple[bool, JointDict, int, Optional[PoseStamped], float]:
        direction = subtract(
            request.desired_position,
            request.current_position,
        )
        orientation = [
            request.current_orientation.x,
            request.current_orientation.y,
            request.current_orientation.z,
            request.current_orientation.w,
        ]
        seed_vector = None
        if request.seed_angles:
            if set(request.seed_angles) != set(self._joint_names):
                raise KeyError("IK seed joint set does not match limb joints")
            seed_vector = [
                request.seed_angles[name] for name in self._joint_names
            ]

        started = time.monotonic()
        for attempt in range(request.attempts):
            if (
                attempt > 0
                and (time.monotonic() - started) * 1000.0
                >= request.budget_ms
            ):
                break

            alpha = 0.5 ** attempt
            position = add(
                request.current_position,
                multiply(direction, alpha),
            )
            values = self._kinematics.inverse_kinematics(
                position,
                orientation,
                seed_vector,
            )
            if values is None:
                continue
            values = list(values)
            if len(values) != len(self._joint_names):
                continue
            solution = {
                name: float(value)
                for name, value in zip(self._joint_names, values)
            }
            if not all(math.isfinite(value) for value in solution.values()):
                continue
            target = create_target_pose(position, request.current_orientation)
            return True, solution, 1, target, alpha

        return False, {}, 0, None, 0.0


class LatestOnlyIkWorker:
    """Run at most one IK call and retain only the newest pending request.

    A completed result blocks the next solve until the control thread consumes
    it.  Consuming a result also drops the queued request, because that request
    was built with the pre-result IK seed.  The control thread then submits a
    fresh latest target with the newly accepted seed.
    """

    def __init__(self, backend_or_service: Any) -> None:
        if callable(getattr(backend_or_service, "solve", None)):
            self._backend = backend_or_service
        else:
            # Keep the small callable-service constructor useful for tests and
            # for existing code while exposing a backend boundary for local IK.
            self._backend = RosServiceIkBackend(backend_or_service)
        self._condition = threading.Condition()
        self._stop = False
        self._in_flight = False
        self._pending: Optional[IkWorkItem] = None
        self._completed: Optional[IkWorkResult] = None
        self._next_request_id = 1
        self._replaced_pending = 0
        self._thread = threading.Thread(
            target=self._run,
            name="baxter-ik-worker",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def submit(
        self,
        session_id: int,
        sequence: int,
        current_position: Vector3,
        current_orientation: Any,
        desired_position: Vector3,
        seed_angles: Optional[JointDict],
        attempts: int,
        budget_ms: float,
        submitted_monotonic: Optional[float] = None,
    ) -> int:
        with self._condition:
            request_id = self._next_request_id
            self._next_request_id += 1
            request = IkWorkItem(
                request_id=request_id,
                session_id=session_id,
                sequence=sequence,
                submitted_monotonic=(
                    time.monotonic()
                    if submitted_monotonic is None
                    else submitted_monotonic
                ),
                current_position=list(current_position),
                current_orientation=copy_quaternion(current_orientation),
                desired_position=list(desired_position),
                seed_angles=(
                    None if seed_angles is None else dict(seed_angles)
                ),
                attempts=attempts,
                budget_ms=budget_ms,
            )
            if self._pending is not None:
                self._replaced_pending += 1
            self._pending = request
            self._condition.notify_all()
            return request_id

    def poll_result(self) -> Optional[IkWorkResult]:
        with self._condition:
            result = self._completed
            if result is None:
                return None
            self._completed = None
            # Refresh the queued target and seed from the control thread after
            # it accepts or rejects this result.
            self._pending = None
            self._condition.notify_all()
            return result

    def cancel_pending(self) -> None:
        """Discard work that has not started and any unconsumed result."""
        with self._condition:
            self._pending = None
            self._completed = None
            self._condition.notify_all()

    def snapshot(self) -> IkWorkerSnapshot:
        with self._condition:
            return IkWorkerSnapshot(
                in_flight=self._in_flight,
                pending_request_id=(
                    None if self._pending is None else self._pending.request_id
                ),
                completed_request_id=(
                    None
                    if self._completed is None
                    else self._completed.request.request_id
                ),
                replaced_pending=self._replaced_pending,
            )

    def close(self) -> None:
        with self._condition:
            self._stop = True
            self._pending = None
            self._completed = None
            self._condition.notify_all()
        if self._thread.is_alive():
            # A rospy ServiceProxy call cannot be cancelled.  The daemon may
            # therefore outlive shutdown if the remote service is wedged.
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        while True:
            with self._condition:
                while (
                    not self._stop
                    and (self._pending is None or self._completed is not None)
                ):
                    self._condition.wait(timeout=0.1)
                if self._stop:
                    return
                request = self._pending
                self._pending = None
                self._in_flight = True

            assert request is not None
            started = time.monotonic()
            try:
                (
                    valid,
                    solution,
                    result_code,
                    accepted_target,
                    accepted_alpha,
                ) = self._backend.solve(request)
                error = None
            except Exception as exc:
                valid = False
                solution = {}
                result_code = 0
                accepted_target = None
                accepted_alpha = 0.0
                error = "{}: {}".format(type(exc).__name__, exc)
            completed = time.monotonic()
            result = IkWorkResult(
                request=request,
                started_monotonic=started,
                completed_monotonic=completed,
                valid=valid,
                solution=solution,
                result_code=result_code,
                accepted_target=accepted_target,
                accepted_alpha=accepted_alpha,
                error=error,
            )

            with self._condition:
                self._in_flight = False
                if self._stop:
                    return
                self._completed = result
                self._condition.notify_all()


def ik_result_rejection_reason(
    result: IkWorkResult,
    active_session_id: int,
    now_monotonic: float,
    maximum_age: float,
) -> Optional[str]:
    """Return why an asynchronous result is unsafe to apply, if any."""
    if result.request.session_id != active_session_id:
        return "IK result belongs to an inactive Grip session"
    age = now_monotonic - result.request.submitted_monotonic
    if age < 0.0:
        return "IK result timestamp is in the future"
    if age > maximum_age:
        return "stale IK result: {:.1f} ms".format(age * 1000.0)
    return None


def maximum_joint_difference(a: JointDict, b: JointDict) -> float:
    if set(a) != set(b):
        raise KeyError("joint sets do not match")
    return max(abs(a[name] - b[name]) for name in a)


def limited_joint_command(
    current: JointDict,
    solution: JointDict,
    maximum_step: float,
) -> JointDict:
    if set(current) != set(solution):
        raise KeyError("IK joint set does not match limb joints")
    if maximum_step <= 0.0:
        return dict(solution)

    command: JointDict = {}
    for name, current_value in current.items():
        difference = solution[name] - current_value
        limited_difference = max(-maximum_step, min(maximum_step, difference))
        command[name] = current_value + limited_difference
    return command


def hold_current_position(
    limb: Any,
    repeat: int = 3,
    interval: float = 0.025,
) -> None:
    """Replace the prior target with the measured joint position."""
    try:
        current = limb.joint_angles()
        if not current:
            return
        for _ in range(repeat):
            if rospy.is_shutdown():
                break
            limb.set_joint_positions(current, raw=False)
            rospy.sleep(interval)
    except Exception as exc:  # Baxter/ROS exceptions vary across SDK releases.
        rospy.logerr("Failed to issue hold command: %s", exc)


def target_pose_position(target: PoseStamped) -> Vector3:
    return [
        float(target.pose.position.x),
        float(target.pose.position.y),
        float(target.pose.position.z),
    ]


def print_configuration(args: argparse.Namespace) -> None:
    print("")
    print("BAXTER PRECISION TELEOP - REAL ROBOT")
    print("Arm:                    {}".format(args.side))
    print("Control rate:           {:.1f} Hz".format(args.control_rate_hz))
    print("Command publish rate:   {:.1f} Hz".format(args.command_rate_hz))
    print("Translation scale:      {:.3f}".format(args.scale))
    print("Maximum displacement:   {:.3f} m".format(args.max_translation))
    print("Hand filter tau:        {:.3f} s".format(args.hand_filter_tau))
    print("Moving hand deadband:   {:.4f} m".format(args.hand_deadzone))
    print("Motion start threshold: {:.4f} m".format(args.motion_start_threshold))
    print("Maximum EE speed:       {:.3f} m/s".format(args.max_ee_speed))
    print("Maximum EE acceleration:{:.3f} m/s^2".format(args.max_ee_accel))
    print("Maximum command lead:   {:.3f} m".format(args.max_command_lead))
    print("Baxter speed ratio:     {:.3f}".format(args.speed_ratio))
    print("Maximum joint lead:     {:.4f} rad".format(args.max_joint_step))
    print("IK branch threshold:    {:.3f} rad".format(args.max_ik_difference))
    print("First IK threshold:     {:.3f} rad".format(args.max_first_ik_difference))
    print("IK backtrack attempts:  {}".format(args.ik_backtrack_attempts))
    print("IK backend:             {} (async latest-only)".format(
        args.ik_backend
    ))
    print("IK time budget:         {:.1f} ms".format(args.ik_budget_ms))
    print("Maximum IK result age:  {:.1f} ms".format(
        args.max_ik_result_age * 1000.0
    ))
    print(
        "Orientation relaxation: after {} misses, up to {:.1f} deg".format(
            args.orientation_relax_after,
            args.max_orientation_relax_deg,
        )
    )
    print("Mapping:                [-PICO z, -PICO x, +PICO y]")
    print("Rotation:               DISABLED")
    print("Gripper:                NOT COMMANDED")
    print("Other arm:              NOT COMMANDED")
    print("Automatic enable:       DISABLED")
    print("")


def main() -> None:
    args = parse_args()
    validate_args(args)
    print_configuration(args)

    rospy.init_node(
        "{}_baxter_precision_teleop".format(args.side),
        anonymous=True,
        disable_signals=True,
    )

    robot_enable = baxter_interface.RobotEnable(
        baxter_interface.CHECK_VERSION
    )
    ready, reason = robot_is_ready(robot_enable)
    if not ready:
        raise SystemExit(
            "Robot is not ready: {}.\n"
            "This script will not enable it automatically.".format(reason)
        )

    limb = baxter_interface.Limb(args.side)
    if args.ik_backend == "ros-service":
        service_name = (
            "/ExternalTools/{}/PositionKinematicsNode/IKService"
        ).format(args.side)
        print("Waiting for IK service:", service_name)
        rospy.wait_for_service(service_name, timeout=10.0)
        ik_service = rospy.ServiceProxy(service_name, SolvePositionIK)
        ik_backend = RosServiceIkBackend(ik_service)
    else:
        try:
            from baxter_pykdl import baxter_kinematics
        except ImportError as exc:
            raise SystemExit(
                "--ik-backend pykdl requires the baxter_pykdl package"
            ) from exc
        ik_backend = PyKdlIkBackend(
            baxter_kinematics(args.side),
            list(limb.joint_names()),
        )
    ik_worker = LatestOnlyIkWorker(ik_backend)

    receiver = LatestPacketReceiver(args.bind_host, args.port)

    controller_filter = LowPassVector3(args.hand_filter_tau)
    moving_deadband = MovingDeadbandVector3(args.hand_deadzone)
    target_limiter = CartesianTargetLimiter(
        args.max_ee_speed,
        args.max_ee_accel,
        args.target_time_constant,
    )
    diagnostics = CsvDiagnostics(args.csv_log)
    command_publisher = ContinuousJointCommandPublisher(
        limb,
        args.command_rate_hz,
    )

    locked = False
    rearm_blocked = False
    input_available = False
    connection_lost = False

    controller_reference: Optional[Vector3] = None
    raw_controller_reference: Optional[Vector3] = None
    robot_position_reference: Optional[Vector3] = None
    robot_orientation_reference: Optional[Any] = None
    motion_active = False
    first_ik_pending = False
    latest_raw_controller: Optional[Vector3] = None
    latest_controller: Optional[Vector3] = None
    latest_grip = 0.0
    latest_sequence = -1
    latest_packet_age = float("inf")

    last_sequence = -1
    last_snapshot_serial = 0
    last_receive_time: Optional[float] = None
    last_controller_sample_time: Optional[float] = None
    last_control_time: Optional[float] = None
    last_status_time = 0.0
    last_safety_report_time = 0.0
    consecutive_ik_misses = 0
    grip_session_id = 0
    # 上一个完整且已接受的 IK 解，用作下一请求的 SEED_USER 和分支基准。
    last_successful_joint_command: Optional[JointDict] = None

    # 最近一次实际发送给 Baxter 的关节位置命令。
    last_sent_joint_command: Optional[JointDict] = None

    # 最近一次通过全部安全检查的笛卡尔目标。
    last_accepted_cartesian_position: Optional[Vector3] = None

    def report_safety(message: str) -> None:
        nonlocal last_safety_report_time
        now_local = time.monotonic()
        if now_local - last_safety_report_time >= 0.5:
            print("[SAFE STOP] {}".format(message))
            last_safety_report_time = now_local

    def clear_reference(reason_text: str, safety_stop: bool) -> None:
        nonlocal locked
        nonlocal rearm_blocked
        nonlocal controller_reference
        nonlocal raw_controller_reference
        nonlocal robot_position_reference
        nonlocal robot_orientation_reference
        nonlocal motion_active
        nonlocal first_ik_pending
        nonlocal last_control_time
        nonlocal last_successful_joint_command
        nonlocal last_sent_joint_command
        nonlocal last_accepted_cartesian_position
        nonlocal consecutive_ik_misses

        was_locked = locked
        if was_locked:
            try:
                current_hold = {
                    name: float(value)
                    for name, value in limb.joint_angles().items()
                }
                if current_hold:
                    command_publisher.set_command(current_hold)
            except Exception as exc:
                rospy.logerr("Failed to update hold command: %s", exc)

        locked = False
        controller_reference = None
        raw_controller_reference = None
        robot_position_reference = None
        robot_orientation_reference = None
        motion_active = False
        first_ik_pending = False
        last_control_time = None
        last_successful_joint_command = None
        last_sent_joint_command = None
        last_accepted_cartesian_position = None
        consecutive_ik_misses = 0
        ik_worker.cancel_pending()
        target_limiter.reset()
        moving_deadband.reset()

        if safety_stop:
            rearm_blocked = True
            report_safety(reason_text)
        elif was_locked:
            print("[UNLOCK] {}".format(reason_text))

    def keep_previous_command() -> None:
        if last_sent_joint_command:
            try:
                command_publisher.set_command(last_sent_joint_command)
            except Exception as exc:
                clear_reference(
                    "failed to hold previous command: {}".format(exc),
                    safety_stop=True,
                )

    def freeze_cartesian_target() -> None:
        """
        IK 失败或分支被拒绝时，把轨迹生成器退回到最近一次
        真正被接受的位置，避免机器人停止后目标仍继续远离。
        """
        if last_accepted_cartesian_position is not None:
            target_limiter.accept_backtracked_position(
                last_accepted_cartesian_position,
                0.0,
            )

    print("Precision teleop ready on UDP {}:{}.".format(args.bind_host, args.port))
    print("Keep the controller still, then press and HOLD Grip to move.")
    print("Release Grip to stop.  After a safety stop, release fully before rearming.")
    print("")

    rate = rospy.Rate(args.control_rate_hz)

    try:
        limb.set_joint_position_speed(args.speed_ratio)
        receiver.start()
        command_publisher.start()
        ik_worker.start()

        while not rospy.is_shutdown():
            now = time.monotonic()
            snapshot = receiver.snapshot()

            publisher_error = command_publisher.error()
            if publisher_error is not None:
                clear_reference(publisher_error, safety_stop=True)
                break

            if snapshot.serial != last_snapshot_serial:
                last_snapshot_serial = snapshot.serial
                last_receive_time = snapshot.received_monotonic
                connection_lost = False

                if snapshot.error is not None:
                    input_available = False
                    clear_reference(snapshot.error, safety_stop=True)
                else:
                    packet = snapshot.packet
                    assert packet is not None
                    error_text: Optional[str] = None

                    if packet.get("version") != PROTOCOL_VERSION:
                        error_text = "protocol version mismatch"
                    elif packet.get("mode") != "real":
                        error_text = "non-real XR packet rejected"
                    elif not bool(packet.get("valid", False)):
                        error_text = "XR source marked invalid"

                    sequence = packet.get("seq")
                    if error_text is None and not isinstance(sequence, int):
                        error_text = "invalid sequence number"

                    age = packet_age(packet)
                    if error_text is None and (age < 0.0 or age > args.max_age):
                        error_text = "stale XR packet: {:.1f} ms".format(
                            age * 1000.0
                        )

                    current_controller = controller_position(packet, args.side)
                    if error_text is None and current_controller is None:
                        error_text = "controller pose unavailable"

                    if error_text is not None:
                        input_available = False
                        clear_reference(error_text, safety_stop=True)
                    else:
                        assert isinstance(sequence, int)
                        assert current_controller is not None

                        # A sender restart is safe only after the operator has
                        # released Grip.  This avoids silently reusing a stale
                        # lock reference after sequence-number regression.
                        if last_sequence >= 0 and sequence < last_sequence:
                            clear_reference(
                                "XR sequence regressed (sender restart)",
                                safety_stop=True,
                            )
                        last_sequence = sequence

                        # Ignore exact sequence duplicates for filtering, but
                        # their packet timestamp still undergoes the age check.
                        is_new_sample = sequence != latest_sequence
                        latest_grip = controller_grip(packet, args.side)
                        latest_packet_age = age

                        if is_new_sample:
                            latest_raw_controller = list(current_controller)
                            sample_time = snapshot.received_monotonic
                            assert sample_time is not None
                            if last_controller_sample_time is None:
                                sample_dt = 1.0 / args.control_rate_hz
                            else:
                                sample_dt = max(
                                    0.0,
                                    sample_time - last_controller_sample_time,
                                )
                            last_controller_sample_time = sample_time
                            latest_controller = controller_filter.update(
                                current_controller,
                                sample_dt,
                            )
                            latest_sequence = sequence

                        input_available = latest_controller is not None

            # Wall-clock receive timeout is separate from sender packet age.
            if (
                last_receive_time is None
                or now - last_receive_time > args.connection_timeout
            ):
                input_available = False
                if not connection_lost:
                    clear_reference(
                        "no XR packet for {:.2f} seconds".format(
                            args.connection_timeout
                        ),
                        safety_stop=True,
                    )
                    connection_lost = True
                    last_sequence = -1
                    latest_sequence = -1

            if not input_available or latest_controller is None:
                rate.sleep()
                continue

            # Every safety stop requires observing a complete release.
            if not locked and latest_grip <= args.grip_release:
                rearm_blocked = False

            if not locked:
                # Keep the filter centred on current hand position while idle;
                # old motion must never leak into the next Grip session.
                controller_filter.reset(latest_controller)
                moving_deadband.reset(latest_controller)

                if rearm_blocked or latest_grip < args.grip_press:
                    rate.sleep()
                    continue

                ready, reason = robot_is_ready(robot_enable)
                if not ready:
                    clear_reference(reason, safety_stop=True)
                    rate.sleep()
                    continue

                endpoint = limb.endpoint_pose()
                current_angles = {
                    name: float(value)
                    for name, value in limb.joint_angles().items()
                }
                if not current_angles:
                    clear_reference(
                        "joint angles unavailable",
                        safety_stop=True,
                    )
                    rate.sleep()
                    continue

                stable_controller = moving_deadband.update(latest_controller)
                controller_reference = list(stable_controller)
                raw_controller_reference = (
                    list(latest_raw_controller)
                    if latest_raw_controller is not None
                    else list(latest_controller)
                )
                robot_position_reference = endpoint_position(endpoint)
                # Translation-only teleoperation must preserve the tool
                # orientation captured at Grip lock.  Using the measured
                # orientation every frame turns accidental drift into the next
                # target and allows a self-reinforcing wrist/elbow twist.
                robot_orientation_reference = copy_quaternion(
                    endpoint["orientation"]
                )
                target_limiter.reset(robot_position_reference)

                # Seed the very first IK call with the exact Grip-lock joint
                # state.  A redundant 7-DoF arm can otherwise choose another
                # joint configuration for the same Cartesian pose.
                last_successful_joint_command = dict(current_angles)
                last_sent_joint_command = dict(current_angles)
                command_publisher.set_command(current_angles)
                motion_active = False
                first_ik_pending = True
                consecutive_ik_misses = 0
                last_accepted_cartesian_position = list(
                    robot_position_reference
                )
                last_control_time = now
                grip_session_id += 1
                locked = True

                print("[LOCK] Grip reference established")
                print(
                    "       endpoint:",
                    [round(value, 4) for value in robot_position_reference],
                )
                rate.sleep()
                continue

            if latest_grip <= args.grip_release:
                clear_reference(
                    "Grip released; holding current position",
                    safety_stop=False,
                )
                rate.sleep()
                continue

            ready, reason = robot_is_ready(robot_enable)
            if not ready:
                clear_reference(reason, safety_stop=True)
                rate.sleep()
                continue

            assert controller_reference is not None
            assert raw_controller_reference is not None
            assert robot_position_reference is not None
            assert robot_orientation_reference is not None

            if last_control_time is None:
                control_dt = 1.0 / args.control_rate_hz
            else:
                control_dt = now - last_control_time
            last_control_time = now
            control_dt = min(
                max(control_dt, 1e-6),
                args.max_control_dt,
            )

            stable_controller = moving_deadband.update(latest_controller)
            raw_hand_delta = subtract(
                latest_raw_controller or latest_controller,
                raw_controller_reference,
            )
            hand_delta = subtract(stable_controller, controller_reference)

            # Grip is a clutch, not a motion command.  Until the controller has
            # moved beyond a small arming threshold, hold the exact Grip-lock
            # joint pose and do not call IK at all.
            if not motion_active:
                if vector_norm(hand_delta) < args.motion_start_threshold:
                    command_publisher.set_command(last_sent_joint_command or {})
                    target_limiter.reset(robot_position_reference)
                    rate.sleep()
                    continue
                motion_active = True
                print(
                    "[MOTION] hand displacement {:.4f} m; IK motion enabled"
                    .format(vector_norm(hand_delta))
                )

            unclamped_robot_delta = map_pico_to_baxter(
                hand_delta,
                args.scale,
            )
            translation_saturated = (
                args.max_translation > 0.0
                and vector_norm(unclamped_robot_delta)
                > args.max_translation + 1e-9
            )
            robot_delta = list(unclamped_robot_delta)
            if translation_saturated:
                robot_delta = clamp_vector_norm(
                    robot_delta,
                    args.max_translation,
                )

            desired_position = add(robot_position_reference, robot_delta)
            smooth_position = target_limiter.update(
                desired_position,
                control_dt,
            )

            endpoint_now = limb.endpoint_pose()
            current_endpoint_position = endpoint_position(endpoint_now)
            # Keep the Grip-lock orientation normally.  Near a workspace
            # boundary, repeated misses may temporarily soften that constraint
            # by a few degrees toward the measured pose.  This never changes
            # the stored reference and never enables a different IK seed.
            target_endpoint_orientation = robot_orientation_reference
            orientation_relaxation = 0.0
            if (
                args.orientation_relax_after > 0
                and consecutive_ik_misses >= args.orientation_relax_after
                and args.max_orientation_relax_deg > 0.0
            ):
                orientation_relaxation = math.radians(
                    args.max_orientation_relax_deg
                )
                target_endpoint_orientation = relax_orientation(
                    robot_orientation_reference,
                    endpoint_now["orientation"],
                    orientation_relaxation,
                )
                orientation_relaxation = min(
                    orientation_relaxation,
                    quaternion_distance(
                        robot_orientation_reference,
                        endpoint_now["orientation"],
                    ),
                )

            # Anti-windup: the command may move rapidly over unlimited total
            # distance, but it may not run arbitrarily far ahead of the robot.
            # This is a local servo lead, not a workspace limit.
            command_lead_saturated = False
            command_lead = subtract(
                smooth_position,
                current_endpoint_position,
            )
            if (
                args.max_command_lead > 0.0
                and vector_norm(command_lead) > args.max_command_lead
            ):
                smooth_position = add(
                    current_endpoint_position,
                    clamp_vector_norm(command_lead, args.max_command_lead),
                )
                target_limiter.enforce_position(smooth_position)
                command_lead_saturated = True

            desired_lag = vector_norm(
                subtract(desired_position, smooth_position)
            )
            tracking_error = vector_norm(
                subtract(smooth_position, current_endpoint_position)
            )

            # Only the main control thread may accept an IK result and update
            # command/seed state.  The worker only performs the blocking ROS
            # call and returns an immutable request/result pair.
            ik_result = ik_worker.poll_result()
            accepted_result = False
            if ik_result is not None:
                rejection = ik_result_rejection_reason(
                    ik_result,
                    grip_session_id,
                    now,
                    args.max_ik_result_age,
                )
                if rejection is not None:
                    if ik_result.request.session_id == grip_session_id:
                        consecutive_ik_misses += 1
                        freeze_cartesian_target()
                        keep_previous_command()
                    if now - last_status_time >= 1.0 / args.status_rate_hz:
                        print(
                            "[IK STALE] request={} seq={}: {}".format(
                                ik_result.request.request_id,
                                ik_result.request.sequence,
                                rejection,
                            )
                        )
                        last_status_time = now
                elif ik_result.error is not None:
                    consecutive_ik_misses += 1
                    freeze_cartesian_target()
                    keep_previous_command()
                    if now - last_status_time >= 1.0 / args.status_rate_hz:
                        print("[IK SKIP] service error: {}".format(
                            ik_result.error
                        ))
                        last_status_time = now
                elif not ik_result.valid or ik_result.accepted_target is None:
                    consecutive_ik_misses += 1
                    freeze_cartesian_target()
                    keep_previous_command()
                    if now - last_status_time >= 1.0 / args.status_rate_hz:
                        print(
                            "[IK SKIP] no reachable target; "
                            "holding previous command; code={}".format(
                                ik_result.result_code
                            )
                        )
                        last_status_time = now
                else:
                    solution = ik_result.solution
                    current_angles = {
                        name: float(value)
                        for name, value in limb.joint_angles().items()
                    }
                    joint_tracking_error = 0.0
                    if last_sent_joint_command is not None and current_angles:
                        try:
                            joint_tracking_error = maximum_joint_difference(
                                last_sent_joint_command,
                                current_angles,
                            )
                        except KeyError:
                            joint_tracking_error = float("nan")

                    if not current_angles or set(solution) != set(current_angles):
                        clear_reference(
                            "IK joint set does not match limb joints",
                            safety_stop=True,
                        )
                        rate.sleep()
                        continue

                    try:
                        assert last_successful_joint_command is not None
                        branch_difference = maximum_joint_difference(
                            solution,
                            last_successful_joint_command,
                        )
                    except KeyError as exc:
                        clear_reference(str(exc), safety_stop=True)
                        rate.sleep()
                        continue

                    active_branch_limit = args.max_ik_difference
                    if (
                        first_ik_pending
                        and args.max_first_ik_difference > 0.0
                    ):
                        if active_branch_limit <= 0.0:
                            active_branch_limit = args.max_first_ik_difference
                        else:
                            active_branch_limit = min(
                                active_branch_limit,
                                args.max_first_ik_difference,
                            )

                    if (
                        active_branch_limit > 0.0
                        and branch_difference > active_branch_limit
                    ):
                        consecutive_ik_misses += 1
                        freeze_cartesian_target()
                        keep_previous_command()
                        if now - last_status_time >= 1.0 / args.status_rate_hz:
                            print(
                                "[IK SKIP] IK jump {:.4f} rad exceeds {:.4f}; "
                                "holding lock branch".format(
                                    branch_difference,
                                    active_branch_limit,
                                )
                            )
                            last_status_time = now
                    else:
                        try:
                            command = limited_joint_command(
                                current_angles,
                                solution,
                                args.max_joint_step,
                            )
                            command_publisher.set_command(command)
                        except Exception as exc:
                            clear_reference(
                                "joint command failed: {}".format(exc),
                                safety_stop=True,
                            )
                            rate.sleep()
                            continue

                        consecutive_ik_misses = 0
                        last_sent_joint_command = dict(command)
                        last_successful_joint_command = dict(solution)
                        first_ik_pending = False
                        accepted_result = True

                        accepted_position = target_pose_position(
                            ik_result.accepted_target
                        )
                        last_accepted_cartesian_position = list(
                            accepted_position
                        )
                        if ik_result.accepted_alpha < 1.0:
                            target_limiter.accept_backtracked_position(
                                accepted_position,
                                ik_result.accepted_alpha,
                            )

                        result_age_ms = (
                            now - ik_result.request.submitted_monotonic
                        ) * 1000.0
                        worker_snapshot = ik_worker.snapshot()
                        diagnostics.write(
                            {
                                "wall_time": time.time(),
                                "event": "MOVE",
                                "seq": latest_sequence,
                                "packet_age_ms": latest_packet_age * 1000.0,
                                "grip": latest_grip,
                                "loop_dt_ms": control_dt * 1000.0,
                                "raw_hand_dx": raw_hand_delta[0],
                                "raw_hand_dy": raw_hand_delta[1],
                                "raw_hand_dz": raw_hand_delta[2],
                                "hand_dx": hand_delta[0],
                                "hand_dy": hand_delta[1],
                                "hand_dz": hand_delta[2],
                                "robot_dx": robot_delta[0],
                                "robot_dy": robot_delta[1],
                                "robot_dz": robot_delta[2],
                                "desired_lag_m": desired_lag,
                                "tracking_error_m": tracking_error,
                                "joint_tracking_error_rad": joint_tracking_error,
                                "command_speed_mps": vector_norm(
                                    target_limiter.velocity
                                ),
                                "ik_backend": args.ik_backend,
                                "ik_time_ms": ik_result.solve_time_ms,
                                "ik_result_age_ms": result_age_ms,
                                "ik_request_id": ik_result.request.request_id,
                                "ik_request_seq": ik_result.request.sequence,
                                "ik_replaced_pending": (
                                    worker_snapshot.replaced_pending
                                ),
                                "ik_alpha": ik_result.accepted_alpha,
                                "ik_diff_rad": branch_difference,
                                "ik_code": ik_result.result_code,
                                "translation_saturated": int(
                                    translation_saturated
                                ),
                                "command_lead_saturated": int(
                                    command_lead_saturated
                                ),
                                "speed_saturated": int(
                                    target_limiter.speed_saturated
                                ),
                                "acceleration_saturated": int(
                                    target_limiter.acceleration_saturated
                                ),
                            }
                        )

                        if now - last_status_time >= 1.0 / args.status_rate_hz:
                            saturation_flags = "{}{}{}{}".format(
                                "T" if translation_saturated else "-",
                                "L" if command_lead_saturated else "-",
                                "V" if target_limiter.speed_saturated else "-",
                                "A" if target_limiter.acceleration_saturated else "-",
                            )
                            print(
                                "[MOVE] seq={}/{} age={:.1f}ms grip={:.2f} "
                                "hand={:.3f}m cmd_delta={:.3f}m v={:.3f}m/s "
                                "lag={:.3f}m track={:.3f}m joint_err={:.3f}rad "
                                "ik={:.1f}ms result_age={:.1f}ms alpha={:.3f} "
                                "diff={:.4f} relax={:.1f}deg sat={}".format(
                                    ik_result.request.sequence,
                                    latest_sequence,
                                    latest_packet_age * 1000.0,
                                    latest_grip,
                                    vector_norm(hand_delta),
                                    vector_norm(robot_delta),
                                    vector_norm(target_limiter.velocity),
                                    desired_lag,
                                    tracking_error,
                                    joint_tracking_error,
                                    ik_result.solve_time_ms,
                                    result_age_ms,
                                    ik_result.accepted_alpha,
                                    branch_difference,
                                    math.degrees(orientation_relaxation),
                                    saturation_flags,
                                )
                            )
                            last_status_time = now

            # While one request is in flight this replaces only the pending
            # target.  No FIFO is created and the robot continues receiving the
            # last accepted command from ContinuousJointCommandPublisher.
            submission_position = (
                list(target_limiter.position)
                if target_limiter.position is not None
                else list(smooth_position)
            )
            ik_worker.submit(
                session_id=grip_session_id,
                sequence=latest_sequence,
                current_position=current_endpoint_position,
                current_orientation=target_endpoint_orientation,
                desired_position=submission_position,
                seed_angles=last_successful_joint_command,
                attempts=args.ik_backtrack_attempts,
                budget_ms=args.ik_budget_ms,
                submitted_monotonic=now,
            )

            if (
                not accepted_result
                and ik_result is None
                and now - last_status_time >= 1.0 / args.status_rate_hz
            ):
                worker_snapshot = ik_worker.snapshot()
                print(
                    "[IK WAIT] seq={} in_flight={} pending={} replaced={}".format(
                        latest_sequence,
                        int(worker_snapshot.in_flight),
                        worker_snapshot.pending_request_id,
                        worker_snapshot.replaced_pending,
                    )
                )
                last_status_time = now

            rate.sleep()

    except (KeyboardInterrupt, rospy.ROSInterruptException):
        print("\nCtrl+C received.")

    finally:
        print("Issuing hold command...")
        try:
            final_hold = {
                name: float(value)
                for name, value in limb.joint_angles().items()
            }
            if final_hold:
                command_publisher.set_command(final_hold)
                time.sleep(0.10)
        except Exception as exc:
            rospy.logerr("Failed to issue final hold command: %s", exc)
        ik_worker.close()
        command_publisher.close()
        try:
            limb.set_joint_position_speed(DEFAULT_BAXTER_SPEED_RATIO)
        except Exception as exc:
            rospy.logerr("Failed to restore Baxter speed ratio: %s", exc)
        receiver.close()
        diagnostics.close()
        print("Baxter precision teleop stopped.")


if __name__ == "__main__":
    main()
