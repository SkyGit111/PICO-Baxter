#!/usr/bin/env python3
"""
First-stage PICO -> Baxter live motion test.

Safety design:
- One arm only.
- Translation only; orientation is fixed.
- Does not enable the robot automatically.
- Requires real XR packets and explicit terminal confirmation.
- Grip is the dead-man switch.
- Packet timeout, stale data, invalid IK, excessive IK jump, robot error,
  or Ctrl+C causes the script to command a hold at the current position.
- Uses normal POSITION_MODE, never RAW_POSITION_MODE.
"""

import argparse
import json
import math
import socket
import time
from typing import Any, Dict, List, Optional, Tuple

import rospy
import baxter_interface

from geometry_msgs.msg import PoseStamped
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest


PROTOCOL_VERSION = 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=15000)

    # Deliberately conservative first-test defaults.
    parser.add_argument("--scale", type=float, default=0.10)
    parser.add_argument("--max-translation", type=float, default=0.005)
    parser.add_argument("--control-rate-hz", type=float, default=10.0)
    parser.add_argument("--speed-ratio", type=float, default=0.03)

    # Maximum command change relative to the currently measured joint angle
    # in one control iteration.
    parser.add_argument("--max-joint-step", type=float, default=0.002)

    # Reject an IK solution that is unexpectedly far from current joints.
    parser.add_argument("--max-ik-difference", type=float, default=0.08)

    parser.add_argument("--max-age", type=float, default=0.50)
    parser.add_argument("--connection-timeout", type=float, default=0.75)
    parser.add_argument("--grip-press", type=float, default=0.60)
    parser.add_argument("--grip-release", type=float, default=0.40)

    parser.add_argument(
        "--execute",
        action="store_true",
        help="Required before this script may send joint position commands.",
    )

    return parser.parse_args()


def finite_vector3(value: Any) -> Optional[List[float]]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None

    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError):
        return None

    if not all(math.isfinite(item) for item in result):
        return None

    return result


def controller_position(
    packet: Dict[str, Any],
    side: str,
) -> Optional[List[float]]:
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


def subtract(a: List[float], b: List[float]) -> List[float]:
    return [a[index] - b[index] for index in range(3)]


def vector_norm(vector: List[float]) -> float:
    return math.sqrt(sum(value * value for value in vector))


def clamp_vector_norm(
    vector: List[float],
    maximum: float,
) -> List[float]:
    length = vector_norm(vector)

    if length <= maximum or length <= 1e-12:
        return vector

    factor = maximum / length
    return [value * factor for value in vector]


def map_pico_to_baxter(
    pico_delta: List[float],
    scale: float,
) -> List[float]:
    """
    Same provisional mapping used by the successful dry-run:

        Baxter +X = PICO +Z
        Baxter +Y = PICO -X
        Baxter +Z = PICO +Y
    """
    pico_x, pico_y, pico_z = pico_delta

    return [
        -scale * pico_z,
        -scale * pico_x,
        scale * pico_y,
    ]


def create_target_pose(
    initial_position: List[float],
    initial_orientation: Any,
    robot_delta: List[float],
) -> PoseStamped:
    target = PoseStamped()
    target.header.stamp = rospy.Time.now()
    target.header.frame_id = "base"

    target.pose.position.x = initial_position[0] + robot_delta[0]
    target.pose.position.y = initial_position[1] + robot_delta[1]
    target.pose.position.z = initial_position[2] + robot_delta[2]

    # First live test: preserve the lock-time endpoint orientation.
    target.pose.orientation.x = initial_orientation.x
    target.pose.orientation.y = initial_orientation.y
    target.pose.orientation.z = initial_orientation.z
    target.pose.orientation.w = initial_orientation.w

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


def solve_ik(
    service: Any,
    target: PoseStamped,
) -> Tuple[bool, Dict[str, float], int]:
    request = SolvePositionIKRequest()
    request.pose_stamp.append(target)
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

    return True, solution, code



def solve_ik_with_backtracking(
    service,
    current_position,
    current_orientation,
    desired_position,
    attempts=9,
):
    """
    Try the desired Cartesian target first.

    If it is unreachable, repeatedly shorten the motion from the current
    measured endpoint toward the desired target:

        1.0, 0.5, 0.25, 0.125, ...

    The current measured endpoint orientation is used rather than the
    orientation captured when Grip was initially pressed. This approximates
    position-only teleoperation with Baxter's full-pose IK service.
    """
    direction = [
        desired_position[index] - current_position[index]
        for index in range(3)
    ]

    last_code = 0

    for attempt in range(attempts):
        alpha = 0.5 ** attempt

        partial_delta = [
            alpha * value
            for value in direction
        ]

        target = create_target_pose(
            current_position,
            current_orientation,
            partial_delta,
        )

        valid, solution, result_code = solve_ik(
            service,
            target,
        )

        last_code = result_code

        if valid:
            return (
                True,
                solution,
                result_code,
                target,
                alpha,
            )

    return False, {}, last_code, None, 0.0

def packet_age(packet: Dict[str, Any]) -> float:
    sent_ns = packet.get("sent_monotonic_ns")

    if not isinstance(sent_ns, int):
        return float("inf")

    return (
        time.monotonic_ns() - sent_ns
    ) / 1_000_000_000.0



def recv_latest_packet(sock):
    """Receive one packet, then discard backlog and retain only the newest."""
    encoded, source = sock.recvfrom(65535)

    # Temporarily enter non-blocking mode and drain the receive queue.
    sock.setblocking(False)

    try:
        while True:
            encoded, source = sock.recvfrom(65535)
    except BlockingIOError:
        pass
    finally:
        # Restore the timeout used by the main control loop.
        sock.settimeout(0.05)

    return encoded, source

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


def limited_joint_command(
    current: Dict[str, float],
    solution: Dict[str, float],
    maximum_step: float,
) -> Dict[str, float]:
    command = {}

    for name, current_value in current.items():
        if name not in solution:
            raise KeyError("IK solution missing joint: {}".format(name))

        difference = solution[name] - current_value
        limited_difference = max(
            -maximum_step,
            min(maximum_step, difference),
        )

        command[name] = current_value + limited_difference

    return command


def hold_current_position(
    limb: Any,
    repeat: int = 1,
    interval: float = 0.025,
) -> None:
    """
    Replace the previous target with the currently measured joint position.
    """
    try:
        current = limb.joint_angles()

        if not current:
            return

        for _ in range(repeat):
            if rospy.is_shutdown():
                break

            limb.set_joint_positions(current, raw=False)
            rospy.sleep(interval)

    except Exception as exc:
        rospy.logerr("Failed to issue hold command: %s", exc)


def main() -> None:
    args = parse_args()

    if not args.execute:
        raise SystemExit(
            "Refusing to command the robot without --execute."
        )

    if args.max_translation < 0:
        raise ValueError(
            "--max-translation must be non-negative; "
            "use 0 to disable the limit."
        )

    if args.scale <= 0 or args.scale > 10.0:
        raise ValueError(
            "--scale must be in (0, 10.0]."
        )

    if args.speed_ratio <= 0 or args.speed_ratio > 1.0:
        raise ValueError(
            "--speed-ratio must be in (0, 1.0]."
        )

    if args.max_joint_step < 0:
        raise ValueError(
            "--max-joint-step must be non-negative; "
            "use 0 to send the complete IK solution."
        )

    if args.grip_release >= args.grip_press:
        raise ValueError(
            "--grip-release must be below --grip-press."
        )

    print("")
    print("REAL ROBOT MOTION TEST")
    print("Arm:                 {}".format(args.side))
    print("Translation scale:   {:.3f}".format(args.scale))
    print("Maximum displacement:{:.3f} m".format(args.max_translation))
    print("Speed ratio:         {:.3f}".format(args.speed_ratio))
    print("Maximum joint step:  {:.4f} rad".format(args.max_joint_step))
    print("Rotation:             DISABLED")
    print("Other arm:            NOT COMMANDED")
    print("Automatic enable:     DISABLED")
    print("")

    rospy.init_node(
        "{}_pico_live_microtest".format(args.side),
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

    service_name = (
        "/ExternalTools/{}/PositionKinematicsNode/IKService"
    ).format(args.side)

    print("Waiting for IK service:", service_name)
    rospy.wait_for_service(service_name, timeout=10.0)

    ik_service = rospy.ServiceProxy(service_name, SolvePositionIK)

    # This value persists in the robot interface until changed again.
    limb.set_joint_position_speed(args.speed_ratio)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind_host, args.port))
    sock.settimeout(0.05)

    locked = False
    rearm_blocked = False

    controller_reference = None
    robot_position_reference = None
    robot_orientation_reference = None

    last_sequence = -1
    last_receive_time = None
    last_control_time = 0.0
    last_print_time = 0.0
    timeout_reported = False

    def clear_reference(
        reason_text: str,
        safety_stop: bool,
    ) -> None:
        nonlocal locked
        nonlocal rearm_blocked
        nonlocal controller_reference
        nonlocal robot_position_reference
        nonlocal robot_orientation_reference

        was_locked = locked

        if was_locked:
            hold_current_position(limb)

        locked = False
        controller_reference = None
        robot_position_reference = None
        robot_orientation_reference = None

        if safety_stop:
            rearm_blocked = True

        if was_locked or safety_stop:
            print(
                "[{}] {}".format(
                    "SAFE STOP" if safety_stop else "UNLOCK",
                    reason_text,
                )
            )

    print("")
    print("Live microtest ready.")
    print("Do not press Grip yet.")
    print("Keep the controller still, then press and HOLD Grip while moving.")
    print("Release Grip to stop and clear the reference.")
    print("")

    try:
        while not rospy.is_shutdown():
            now = time.monotonic()

            try:
                encoded, _source = recv_latest_packet(sock)
            except socket.timeout:
                if (
                    last_receive_time is not None
                    and now - last_receive_time
                    > args.connection_timeout
                ):
                    if locked:
                        clear_reference(
                            "XR packet stream timed out",
                            safety_stop=True,
                        )

                    if not timeout_reported:
                        print(
                            "[SAFE] No XR packet for {:.2f} seconds."
                            .format(args.connection_timeout)
                        )
                        timeout_reported = True

                    last_sequence = -1

                continue

            last_receive_time = time.monotonic()
            timeout_reported = False

            try:
                packet = json.loads(encoded.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                clear_reference(
                    "invalid UDP payload",
                    safety_stop=True,
                )
                continue

            if packet.get("version") != PROTOCOL_VERSION:
                clear_reference(
                    "protocol version mismatch",
                    safety_stop=True,
                )
                continue

            if packet.get("mode") != "real":
                clear_reference(
                    "non-real XR packet rejected",
                    safety_stop=True,
                )
                continue

            sequence = packet.get("seq")

            if not isinstance(sequence, int):
                clear_reference(
                    "invalid sequence number",
                    safety_stop=True,
                )
                continue

            if sequence <= last_sequence:
                continue

            last_sequence = sequence

            age = packet_age(packet)

            if age < 0 or age > args.max_age:
                clear_reference(
                    "stale XR packet: {:.1f} ms".format(age * 1000),
                    safety_stop=True,
                )
                continue

            if not bool(packet.get("valid", False)):
                clear_reference(
                    "XR source marked invalid",
                    safety_stop=True,
                )
                continue

            current_controller = controller_position(packet, args.side)
            grip = controller_grip(packet, args.side)

            if current_controller is None:
                clear_reference(
                    "controller pose unavailable",
                    safety_stop=True,
                )
                continue

            # A safety stop requires a full Grip release before rearming.
            if not locked and grip <= args.grip_release:
                rearm_blocked = False

            if not locked:
                if rearm_blocked or grip < args.grip_press:
                    continue

                ready, reason = robot_is_ready(robot_enable)

                if not ready:
                    clear_reference(reason, safety_stop=True)
                    continue

                endpoint = limb.endpoint_pose()

                controller_reference = current_controller
                robot_position_reference = [
                    float(endpoint["position"].x),
                    float(endpoint["position"].y),
                    float(endpoint["position"].z),
                ]
                robot_orientation_reference = endpoint["orientation"]

                locked = True
                last_control_time = 0.0

                print("")
                print("[LOCK] Grip reference established")
                print(
                    "       endpoint:",
                    [
                        round(value, 4)
                        for value in robot_position_reference
                    ],
                )
                continue

            if grip <= args.grip_release:
                clear_reference(
                    "Grip released; holding current position",
                    safety_stop=False,
                )
                continue

            ready, reason = robot_is_ready(robot_enable)

            if not ready:
                clear_reference(reason, safety_stop=True)
                continue

            if now - last_control_time < 1.0 / args.control_rate_hz:
                continue

            last_control_time = now

            pico_delta = subtract(
                current_controller,
                controller_reference,
            )

            robot_delta = map_pico_to_baxter(
                pico_delta,
                args.scale,
            )

            if args.max_translation > 0:
                robot_delta = clamp_vector_norm(
                    robot_delta,
                    args.max_translation,
                )

            # Absolute target requested by the controller displacement.
            desired_position = [
                robot_position_reference[index] + robot_delta[index]
                for index in range(3)
            ]

            # Use the robot's current measured pose as the local IK starting
            # pose. In particular, do not force the orientation captured at
            # Grip lock for the entire motion.
            endpoint_now = limb.endpoint_pose()

            current_endpoint_position = [
                float(endpoint_now["position"].x),
                float(endpoint_now["position"].y),
                float(endpoint_now["position"].z),
            ]

            current_endpoint_orientation = endpoint_now["orientation"]

            try:
                (
                    valid,
                    solution,
                    result_code,
                    target,
                    accepted_alpha,
                ) = solve_ik_with_backtracking(
                    ik_service,
                    current_endpoint_position,
                    current_endpoint_orientation,
                    desired_position,
                    attempts=9,
                )
            except rospy.ServiceException as exc:
                if now - last_print_time >= 0.5:
                    print(
                        "[IK SKIP] service error: {}".format(exc)
                    )
                    last_print_time = now
                continue

            if not valid:
                # One unreachable sample must not terminate the Grip session.
                # Keep the previous robot command and try again next frame.
                if now - last_print_time >= 0.5:
                    print(
                        "[IK SKIP] no reachable target along requested "
                        "direction; Grip remains active"
                    )
                    print(
                        "          requested_delta:",
                        [round(value, 4) for value in robot_delta],
                    )
                    last_print_time = now
                continue

            current_angles = {
                name: float(value)
                for name, value in limb.joint_angles().items()
            }

            if set(solution) != set(current_angles):
                clear_reference(
                    "IK joint set does not match limb joints",
                    safety_stop=True,
                )
                continue

            max_ik_difference = max(
                abs(solution[name] - current_angles[name])
                for name in current_angles
            )

            if (
                args.max_ik_difference > 0
                and max_ik_difference > args.max_ik_difference
            ):
                if now - last_print_time >= 0.5:
                    print(
                        "[IK SKIP] branch jump rejected: "
                        "{:.4f} rad; Grip remains active".format(
                            max_ik_difference
                        )
                    )
                    last_print_time = now
                continue

            if args.max_joint_step > 0:
                try:
                    command = limited_joint_command(
                        current_angles,
                        solution,
                        args.max_joint_step,
                    )
                except KeyError as exc:
                    clear_reference(str(exc), safety_stop=True)
                    continue
            else:
                # No custom per-cycle step limit:
                # send the complete current IK solution.
                command = dict(solution)

            # Normal safety-modified position mode. Never raw=True.
            limb.set_joint_positions(command, raw=False)

            if now - last_print_time >= 0.5:
                endpoint = limb.endpoint_pose()

                print(
                    "[MOVE] seq={} age={:.1f} ms grip={:.2f} "
                    "target_delta={} ik_alpha={:.3f} "
                    "max_ik_diff={:.4f}"
                    .format(
                        sequence,
                        age * 1000.0,
                        grip,
                        [round(value, 4) for value in robot_delta],
                        accepted_alpha,
                        max_ik_difference,
                    )
                )
                print(
                    "       measured endpoint:",
                    [
                        round(float(endpoint["position"].x), 4),
                        round(float(endpoint["position"].y), 4),
                        round(float(endpoint["position"].z), 4),
                    ],
                    "IK code:",
                    result_code,
                )

                last_print_time = now

    except KeyboardInterrupt:
        print("\nCtrl+C received.")

    finally:
        print("Issuing hold command...")
        hold_current_position(limb)

        # Restore Baxter's documented default speed ratio.
        limb.set_joint_position_speed(0.30)

        sock.close()
        print("Live microtest stopped.")


if __name__ == "__main__":
    main()
