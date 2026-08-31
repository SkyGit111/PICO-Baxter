#!/usr/bin/env python3
"""Read XR input and forward it to a local UDP receiver.

Use --mock to generate synthetic data without wearing or connecting the PICO.
Without --mock, data is read from SdkXRInputSource.
"""

import argparse
import dataclasses
import json
import math
import socket
import time
from typing import Any, Dict, List, Optional, Tuple


PROTOCOL_VERSION = 1


def to_jsonable(value: Any) -> Any:
    """Convert SDK objects into JSON-compatible values."""
    if value is None:
        return None

    if isinstance(value, (str, int, float, bool)):
        return value

    if dataclasses.is_dataclass(value):
        return to_jsonable(dataclasses.asdict(value))

    if isinstance(value, dict):
        return {str(key): to_jsonable(item) for key, item in value.items()}

    if isinstance(value, (list, tuple)):
        return [to_jsonable(item) for item in value]

    if hasattr(value, "_asdict"):
        return to_jsonable(value._asdict())

    if hasattr(value, "__dict__"):
        public_values = {
            key: item
            for key, item in vars(value).items()
            if not key.startswith("_")
        }
        return to_jsonable(public_values)

    return str(value)


def pose_to_dict(pose: Any) -> Optional[Dict[str, Any]]:
    if pose is None:
        return None

    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)

    if position is None or orientation is None:
        return None

    return {
        "position": [float(value) for value in position],
        "orientation_wxyz": [float(value) for value in orientation],
    }


def controller_to_dict(controller: Any) -> Dict[str, Any]:
    if controller is None:
        return {
            "pose": None,
            "grip": 0.0,
            "trigger": 0.0,
            "buttons": {},
        }

    return {
        "pose": pose_to_dict(getattr(controller, "pose", None)),
        "grip": float(getattr(controller, "grip", 0.0)),
        "trigger": float(getattr(controller, "trigger", 0.0)),
        "buttons": to_jsonable(getattr(controller, "buttons", {})),
    }


def build_real_packet(state: Any, seq: int) -> Dict[str, Any]:
    head = pose_to_dict(getattr(state, "headset", None))
    left = controller_to_dict(getattr(state, "left_controller", None))
    right = controller_to_dict(getattr(state, "right_controller", None))

    valid = any(
        (
            head is not None,
            left["pose"] is not None,
            right["pose"] is not None,
        )
    )

    return {
        "version": PROTOCOL_VERSION,
        "seq": seq,
        "mode": "real",
        "sent_monotonic_ns": time.monotonic_ns(),
        "sent_wall_time_ns": time.time_ns(),
        "xr_timestamp_ns": int(getattr(state, "timestamp_ns", 0)),
        "valid": valid,
        "head": head,
        "left": left,
        "right": right,
    }


def build_mock_packet(seq: int, start_time: float) -> Dict[str, Any]:
    elapsed = time.monotonic() - start_time

    # Small, predictable motion for testing the transport layer.
    x_offset = 0.03 * math.sin(elapsed)
    y_offset = 0.02 * math.cos(elapsed * 0.7)

    # Alternate the mock dead-man grip every two seconds.
    grip_active = 1.0 if int(elapsed / 2.0) % 2 == 0 else 0.0
    trigger = 0.5 + 0.5 * math.sin(elapsed * 0.8)

    identity_quaternion = [1.0, 0.0, 0.0, 0.0]

    return {
        "version": PROTOCOL_VERSION,
        "seq": seq,
        "mode": "mock",
        "sent_monotonic_ns": time.monotonic_ns(),
        "sent_wall_time_ns": time.time_ns(),
        "xr_timestamp_ns": 0,
        "valid": True,
        "head": {
            "position": [0.0, 1.60, 0.0],
            "orientation_wxyz": identity_quaternion,
        },
        "left": {
            "pose": {
                "position": [-0.30 + x_offset, 1.15 + y_offset, 0.35],
                "orientation_wxyz": identity_quaternion,
            },
            "grip": grip_active,
            "trigger": trigger,
            "buttons": {"mock": True},
        },
        "right": {
            "pose": {
                "position": [0.30 - x_offset, 1.15 + y_offset, 0.35],
                "orientation_wxyz": identity_quaternion,
            },
            "grip": grip_active,
            "trigger": 1.0 - trigger,
            "buttons": {"mock": True},
        },
    }


def build_destinations(
    host: str,
    primary_port: int,
    additional_ports: List[int],
) -> List[Tuple[str, int]]:
    """Return a stable de-duplicated local fan-out destination list."""
    ports = [primary_port] + list(additional_ports)
    for port in ports:
        if not 0 < port <= 65535:
            raise ValueError("UDP ports must be in [1, 65535]")
    return [(host, port) for port in dict.fromkeys(ports)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=15000)
    parser.add_argument(
        "--additional-port",
        type=int,
        action="append",
        default=[],
        help=(
            "Send the same latest XR packet to another UDP port. Repeat this "
            "option to fan out to independently running arm processes."
        ),
    )
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Generate synthetic XR data instead of reading the PICO.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.rate_hz <= 0:
        raise ValueError("--rate-hz must be greater than zero")

    destinations = build_destinations(
        args.host,
        args.port,
        args.additional_port,
    )
    period = 1.0 / args.rate_hz
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    xr_input = None
    if not args.mock:
        # Import only in real mode, so mock mode can be tested independently.
        from xrobotoolkit_teleop.core.xr_input import SdkXRInputSource

        xr_input = SdkXRInputSource()

    print(
        "PICO UDP sender started:",
        f"mode={'mock' if args.mock else 'real'}",
        "destinations={}".format(
            ",".join("{}:{}".format(*item) for item in destinations)
        ),
        f"rate={args.rate_hz:.1f} Hz",
    )

    seq = 0
    start_time = time.monotonic()
    next_send_time = time.monotonic()

    try:
        while True:
            if args.mock:
                packet = build_mock_packet(seq, start_time)
            else:
                state = xr_input.poll()
                packet = build_real_packet(state, seq)

            encoded = json.dumps(
                packet,
                ensure_ascii=False,
                separators=(",", ":"),
            ).encode("utf-8")

            if len(encoded) > 60000:
                raise RuntimeError(
                    f"UDP packet is unexpectedly large: {len(encoded)} bytes"
                )

            for destination in destinations:
                sock.sendto(encoded, destination)

            if seq % max(1, int(args.rate_hz)) == 0:
                print(
                    f"sent seq={seq} mode={packet['mode']} "
                    f"valid={packet['valid']} bytes={len(encoded)}"
                )

            seq += 1
            next_send_time += period
            sleep_time = next_send_time - time.monotonic()

            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Reset scheduling after a prolonged delay.
                next_send_time = time.monotonic()

    except KeyboardInterrupt:
        print("\nSender stopped.")
    finally:
        if xr_input is not None:
            xr_input.close()
        sock.close()


if __name__ == "__main__":
    main()
