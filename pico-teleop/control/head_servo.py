from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Protocol


# Servo mechanics: one serial command unit moves the horn by this much (deg).
SERVO_DEG_PER_UNIT = 0.19
# Head angle (deg) to command units so ~1 deg head ~= ~1 deg mechanical rotation.
DEFAULT_HEAD_SERVO_GAIN = 1.0 / SERVO_DEG_PER_UNIT


class ServoOutput(Protocol):
    def servo_move(
        self,
        servo1_angle: float | None = None,
        servo2_angle: float | None = None,
        wait_time: float = 0.0,
        move_time_ms: int = 1000,
    ) -> None:
        """Send pan/tilt servo targets."""

    def close(self) -> None:
        """Release hardware resources."""


@dataclass(frozen=True)
class ServoRange:
    min_value: float
    max_value: float

    def __post_init__(self) -> None:
        if self.min_value > self.max_value:
            raise ValueError("servo range min must be <= max")

    def clamp(self, value: float) -> float:
        return float(min(max(value, self.min_value), self.max_value))


class SerialServoClient:
    """Minimal serial wrapper for the two-servo protocol used by the robot."""

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 9600):
        import serial  # type: ignore[import-not-found]

        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baudrate
        self.ser.timeout = 1.0
        self.servo_angle1: int | None = None
        self.servo_angle2: int | None = None
        self.ser.open()

    def _send_move_command(self, targets: list[tuple[int, float]], move_time_ms: int) -> None:
        move_time_ms = max(0, min(65535, int(move_time_ms)))
        cmd = bytearray([0x55, 0x55, 5 + 3 * len(targets), 0x03, len(targets)])
        cmd.extend((move_time_ms & 0xFF, (move_time_ms >> 8) & 0xFF))
        for servo_id, angle in targets:
            position = int(angle)
            cmd.extend((servo_id, position & 0xFF, (position >> 8) & 0xFF))
        self.ser.write(cmd)

    def servo_move(
        self,
        servo1_angle: float | None = None,
        servo2_angle: float | None = None,
        wait_time: float = 0.0,
        move_time_ms: int = 1000,
    ) -> None:
        targets: list[tuple[int, float]] = []
        if servo1_angle is not None:
            angle1 = int(servo1_angle)
            targets.append((1, angle1))
            self.servo_angle1 = angle1
        if servo2_angle is not None:
            angle2 = int(servo2_angle)
            targets.append((2, angle2))
            self.servo_angle2 = angle2
        if targets:
            self._send_move_command(targets, move_time_ms)
        if wait_time > 0.0:
            time.sleep(wait_time)

    def close(self) -> None:
        if self.ser.is_open:
            self.ser.close()


def command_head_servo_target(
    servo_client: ServoOutput,
    servo1: float,
    servo2: float,
    wait_time: float = 0.0,
    move_time_ms: int = 1000,
) -> None:
    """Send a complete pan/tilt target through any compatible servo client."""
    servo_client.servo_move(
        servo1_angle=servo1,
        servo2_angle=servo2,
        wait_time=wait_time,
        move_time_ms=move_time_ms,
    )
