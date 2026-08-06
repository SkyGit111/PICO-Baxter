#!/usr/bin/env python3
"""Receive PICO bridge packets without controlling any robot."""

import argparse
import json
import socket
import time
from typing import Any, Dict, Optional


EXPECTED_VERSION = 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=15000)
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
        help="Enter safe-stop state after receiving no packet for this long.",
    )
    return parser.parse_args()


def controller_summary(controller: Optional[Dict[str, Any]]) -> str:
    if not controller:
        return "missing"

    pose = controller.get("pose")
    grip = float(controller.get("grip", 0.0))
    trigger = float(controller.get("trigger", 0.0))

    if pose is None:
        position_text = "pose=None"
    else:
        position = pose.get("position", [])
        position_text = "pos=" + str(
            [round(float(value), 3) for value in position]
        )

    return (
        f"{position_text} "
        f"grip={grip:.2f} trigger={trigger:.2f}"
    )


def main() -> None:
    args = parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind_host, args.port))
    sock.settimeout(0.05)

    print(
        "Dry receiver listening on "
        f"{args.bind_host}:{args.port}"
    )
    print("No robot commands will be issued.")

    last_seq = -1
    last_receive_time = None
    last_print_time = 0.0
    timeout_reported = False

    try:
        while True:
            now = time.monotonic()

            try:
                encoded, source = sock.recvfrom(65535)
            except socket.timeout:
                if (
                    last_receive_time is not None
                    and now - last_receive_time > args.connection_timeout
                    and not timeout_reported
                ):
                    print(
                        "[SAFE STOP] Packet stream timed out; "
                        "robot output would be disabled."
                    )
                    timeout_reported = True
                continue

            receive_time = time.monotonic()
            last_receive_time = receive_time
            timeout_reported = False

            try:
                packet = json.loads(encoded.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                print(f"[DROP] Invalid JSON from {source}: {exc}")
                continue

            if packet.get("version") != EXPECTED_VERSION:
                print(
                    "[DROP] Protocol version mismatch:",
                    packet.get("version"),
                )
                continue

            seq = packet.get("seq")
            if not isinstance(seq, int):
                print("[DROP] Missing or invalid sequence number")
                continue

            if seq <= last_seq:
                print(
                    f"[DROP] Out-of-order packet: seq={seq}, "
                    f"last_seq={last_seq}"
                )
                continue

            last_seq = seq

            sent_ns = packet.get("sent_monotonic_ns")
            if not isinstance(sent_ns, int):
                print("[DROP] Missing monotonic timestamp")
                continue

            age_seconds = (
                time.monotonic_ns() - sent_ns
            ) / 1_000_000_000.0

            stale = age_seconds > args.max_age
            source_valid = bool(packet.get("valid", False))
            safe_to_consume = source_valid and not stale

            # Limit normal logging to approximately 2 Hz.
            if receive_time - last_print_time >= 0.5:
                status = "OK" if safe_to_consume else "SAFE"
                print(
                    f"[{status}] seq={seq} "
                    f"mode={packet.get('mode')} "
                    f"age={age_seconds * 1000:.1f} ms "
                    f"source_valid={source_valid}"
                )
                print(
                    "       left: ",
                    controller_summary(packet.get("left")),
                )
                print(
                    "       right:",
                    controller_summary(packet.get("right")),
                )

                left = packet.get("left") or {}
                right = packet.get("right") or {}

                left_deadman = float(left.get("grip", 0.0)) >= 0.5
                right_deadman = float(right.get("grip", 0.0)) >= 0.5

                print(
                    "       deadman:",
                    f"left={left_deadman}",
                    f"right={right_deadman}",
                )

                last_print_time = receive_time

    except KeyboardInterrupt:
        print("\nReceiver stopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
