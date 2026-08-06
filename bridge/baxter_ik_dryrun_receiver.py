#!/usr/bin/env python3
"""
Receive real PICO controller packets and test Baxter IK.

SAFETY:
- This script NEVER enables Baxter.
- This script NEVER sends joint commands.
- This script only reads the current endpoint pose and calls the IK service.
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
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


EXPECTED_PROTOCOL_VERSION = 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Baxter IK dry-run receiver for PICO teleoperation."
    )
    parser.add_argument(
        "--side",
        choices=("left", "right"),
        default="left",
        help="Baxter arm and PICO controller to test.",
    )
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=15000)

    parser.add_argument(
        "--scale",
        type=float,
        default=0.30,
        help="Robot translation / controller translation.",
    )
    parser.add_argument(
        "--max-translation",
        type=float,
        default=0.10,
        help="Maximum target displacement from the lock point, in metres.",
    )
    parser.add_argument(
        "--solve-rate-hz",
        type=float,
        default=8.0,
        help="Maximum IK query rate.",
    )
    parser.add_argument(
        "--max-age",
        type=float,
        default=0.25,
        help="Reject packets older than this many seconds.",
    )
    parser.add_argument(
        "--connection-timeout",
        type=float,
        default=0.50,
        help="Clear the control reference after this period without packets.",
    )
    parser.add_argument(
        "--grip-press",
        type=float,
        default=0.60,
        help="Grip threshold used to establish a reference.",
    )
    parser.add_argument(
        "--grip-release",
        type=float,
        default=0.40,
        help="Grip threshold used to release a reference.",
    )
    parser.add_argument(
        "--allow-mock",
        action="store_true",
        help="Allow mock packets. Disabled by default.",
    )
    return parser.parse_args()


def as_vector3(value: Any) -> Optional[List[float]]:
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

    return as_vector3(pose.get("position"))


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


def subtract(
    current: List[float],
    reference: List[float],
) -> List[float]:
    return [
        current[0] - reference[0],
        current[1] - reference[1],
        current[2] - reference[2],
    ]


def norm(vector: List[float]) -> float:
    return math.sqrt(sum(value * value for value in vector))


def clamp_norm(vector: List[float], maximum: float) -> List[float]:
    length = norm(vector)

    if length <= maximum or length <= 1e-9:
        return vector

    factor = maximum / length
    return [value * factor for value in vector]


def map_pico_delta_to_baxter(
    raw_delta: List[float],
    scale: float,
) -> List[float]:
    """
    Provisional Unity/PICO -> Baxter base-frame translation mapping.

    Assumed PICO/Unity axes:
        +X: controller moves to the user's right
        +Y: controller moves upward
        +Z: controller moves forward

    Baxter base axes:
        +X: forward from Baxter
        +Y: Baxter's left
        +Z: upward

    Therefore:
        Baxter X =  PICO Z
        Baxter Y = -PICO X
        Baxter Z =  PICO Y

    This must be verified from dry-run logs before commanding hardware.
    """
    pico_x, pico_y, pico_z = raw_delta

    return [
        scale * pico_z,
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

    # Rotation control is deliberately disabled in this first test.
    target.pose.orientation.x = initial_orientation.x
    target.pose.orientation.y = initial_orientation.y
    target.pose.orientation.z = initial_orientation.z
    target.pose.orientation.w = initial_orientation.w

    return target


def result_code(response: Any) -> int:
    if not response.result_type:
        return int(response.RESULT_INVALID)

    code = response.result_type[0]

    # Compatibility with different Python/ROS message representations.
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
    code = result_code(response)

    valid = (
        code != int(response.RESULT_INVALID)
        and bool(response.joints)
        and bool(response.joints[0].name)
    )

    if not valid:
        return False, {}, code

    solution = dict(
        zip(
            response.joints[0].name,
            response.joints[0].position,
        )
    )

    return True, solution, code


def packet_is_fresh(
    packet: Dict[str, Any],
    max_age: float,
) -> Tuple[bool, float]:
    sent_ns = packet.get("sent_monotonic_ns")

    if not isinstance(sent_ns, int):
        return False, float("inf")

    age = (
        time.monotonic_ns() - sent_ns
    ) / 1_000_000_000.0

    return 0.0 <= age <= max_age, age


def main() -> None:
    args = parse_args()

    if args.scale <= 0:
        raise ValueError("--scale must be positive")

    if args.max_translation <= 0:
        raise ValueError("--max-translation must be positive")

    if args.solve_rate_hz <= 0:
        raise ValueError("--solve-rate-hz must be positive")

    if args.grip_release >= args.grip_press:
        raise ValueError(
            "--grip-release must be lower than --grip-press"
        )

    rospy.init_node(
        "{}_pico_ik_dryrun".format(args.side),
        anonymous=True,
        disable_signals=True,
    )

    service_name = (
        "/ExternalTools/{}/"
        "PositionKinematicsNode/IKService"
    ).format(args.side)

    print("Waiting for IK service:", service_name)
    rospy.wait_for_service(service_name, timeout=10.0)

    ik_service = rospy.ServiceProxy(
        service_name,
        SolvePositionIK,
    )

    limb = baxter_interface.Limb(args.side)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind_host, args.port))
    sock.settimeout(0.05)

    print("")
    print("==================================================")
    print("Baxter IK dry-run is ready")
    print("Arm:              {}".format(args.side))
    print(
        "UDP:              {}:{}".format(
            args.bind_host,
            args.port,
        )
    )
    print("Scale:            {:.2f}".format(args.scale))
    print(
        "Maximum offset:   {:.3f} m".format(
            args.max_translation
        )
    )
    print("Rotation control: DISABLED")
    print("Joint commands:   DISABLED")
    print("Robot enabling:   DISABLED")
    print("==================================================")
    print("")
    print(
        "Press and hold the {} Grip to establish a reference."
        .format(args.side)
    )

    locked = False
    controller_reference = None
    robot_position_reference = None
    robot_orientation_reference = None

    last_receive_time = None
    last_sequence = -1
    last_solve_time = 0.0
    last_print_time = 0.0
    timeout_reported = False

    def clear_reference(reason: str) -> None:
        nonlocal locked
        nonlocal controller_reference
        nonlocal robot_position_reference
        nonlocal robot_orientation_reference

        if locked:
            print("[UNLOCK] {}".format(reason))

        locked = False
        controller_reference = None
        robot_position_reference = None
        robot_orientation_reference = None

    try:
        while not rospy.is_shutdown():
            now = time.monotonic()

            try:
                encoded, source = sock.recvfrom(65535)
            except socket.timeout:
                if (
                    last_receive_time is not None
                    and now - last_receive_time
                    > args.connection_timeout
                ):
                    clear_reference("packet stream timed out")

                    if not timeout_reported:
                        print(
                            "[SAFE] No packets for {:.2f} s; "
                            "reference cleared."
                            .format(args.connection_timeout)
                        )
                        timeout_reported = True

                    # Permit sender sequence numbers to restart from zero.
                    last_sequence = -1

                continue

            last_receive_time = time.monotonic()
            timeout_reported = False

            try:
                packet = json.loads(encoded.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                clear_reference("invalid UDP payload")
                continue

            if packet.get("version") != EXPECTED_PROTOCOL_VERSION:
                clear_reference("protocol version mismatch")
                continue

            mode = packet.get("mode")
            if mode != "real" and not args.allow_mock:
                clear_reference(
                    "non-real packet rejected: {}".format(mode)
                )
                continue

            sequence = packet.get("seq")
            if not isinstance(sequence, int):
                clear_reference("invalid sequence number")
                continue

            if sequence <= last_sequence:
                continue

            last_sequence = sequence

            fresh, age = packet_is_fresh(packet, args.max_age)
            if not fresh:
                clear_reference(
                    "stale packet: {:.1f} ms".format(
                        age * 1000.0
                    )
                )
                continue

            if not bool(packet.get("valid", False)):
                clear_reference("XR source marked invalid")
                continue

            current_controller = controller_position(
                packet,
                args.side,
            )
            grip = controller_grip(packet, args.side)

            if current_controller is None:
                clear_reference("controller pose unavailable")
                continue

            if not locked:
                if grip < args.grip_press:
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
                last_solve_time = 0.0

                print("")
                print("[LOCK] Grip reference established")
                print(
                    "       controller reference:",
                    [
                        round(value, 4)
                        for value in controller_reference
                    ],
                )
                print(
                    "       Baxter endpoint:",
                    [
                        round(value, 4)
                        for value in robot_position_reference
                    ],
                )
                continue

            if grip <= args.grip_release:
                clear_reference("Grip released")
                continue

            if now - last_solve_time < 1.0 / args.solve_rate_hz:
                continue

            last_solve_time = now

            raw_delta = subtract(
                current_controller,
                controller_reference,
            )

            robot_delta_unclamped = map_pico_delta_to_baxter(
                raw_delta,
                args.scale,
            )

            robot_delta = clamp_norm(
                robot_delta_unclamped,
                args.max_translation,
            )

            target = create_target_pose(
                robot_position_reference,
                robot_orientation_reference,
                robot_delta,
            )

            try:
                valid, solution, seed_code = solve_ik(
                    ik_service,
                    target,
                )
            except rospy.ServiceException as exc:
                clear_reference(
                    "IK service error: {}".format(exc)
                )
                continue

            if now - last_print_time < 0.50:
                continue

            last_print_time = now

            target_xyz = [
                target.pose.position.x,
                target.pose.position.y,
                target.pose.position.z,
            ]

            print("")
            print(
                "[{}] seq={} age={:.1f} ms grip={:.2f}"
                .format(
                    "IK OK" if valid else "IK FAIL",
                    sequence,
                    age * 1000.0,
                    grip,
                )
            )
            print(
                "       raw PICO Δxyz:",
                [round(value, 4) for value in raw_delta],
            )
            print(
                "       mapped Baxter Δxyz:",
                [round(value, 4) for value in robot_delta],
            )
            print(
                "       target base xyz:",
                [round(value, 4) for value in target_xyz],
            )
            print("       IK result code:", seed_code)

            if valid:
                current_angles = limb.joint_angles()

                maximum_joint_difference = max(
                    abs(
                        float(solution[name])
                        - float(current_angles[name])
                    )
                    for name in solution
                    if name in current_angles
                )

                print(
                    "       max joint difference: "
                    "{:.4f} rad".format(
                        maximum_joint_difference
                    )
                )
                print(
                    "       solution:",
                    {
                        name: round(float(value), 4)
                        for name, value in sorted(
                            solution.items()
                        )
                    },
                )

    except KeyboardInterrupt:
        print("\nDry-run stopped.")
    finally:
        sock.close()

    print("No robot motion command was issued.")


if __name__ == "__main__":
    main()
