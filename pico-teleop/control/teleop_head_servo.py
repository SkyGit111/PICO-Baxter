"""PICO headset pan/tilt servo teleoperation (head-only standalone).

Run XRoboToolkit-PC-Service-Python separately before starting this script.
"""

from __future__ import annotations

import os
import sys
import time

import numpy as np
import tyro  # type: ignore[import-not-found]

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from control.head_servo import DEFAULT_HEAD_SERVO_GAIN, SerialServoClient, command_head_servo_target
from xrobotoolkit_teleop.core.mapping import HeadAngleMapper, MappingConfig
from xrobotoolkit_teleop.core.types import Pose
from xrobotoolkit_teleop.core.xr_input import SdkXRInputSource


def _angles_to_servo_units(
    yaw_deg: float,
    pitch_deg: float,
    servo1_center: float,
    servo2_center: float,
    pitch_gain: float,
    yaw_gain: float,
    pitch_sign: float,
    yaw_sign: float,
    servo1_min: float,
    servo1_max: float,
    servo2_min: float,
    servo2_max: float,
) -> tuple[float, float]:
    servo1 = servo1_center + pitch_sign * pitch_gain * pitch_deg
    servo2 = servo2_center + yaw_sign * yaw_gain * yaw_deg
    return (
        float(np.clip(servo1, servo1_min, servo1_max)),
        float(np.clip(servo2, servo2_min, servo2_max)),
    )


def main(
    servo_port: str = "/dev/ttyUSB0",
    baudrate: int = 9600,
    rate_hz: float = 30.0,
    toggle_button: str = "X",
    servo1_center: float = 385.0,
    servo2_center: float = 500.0,
    servo1_min: float = 270.0,
    servo1_max: float = 500.0,
    servo2_min: float = 430.0,
    servo2_max: float = 570.0,
    yaw_gain: float = DEFAULT_HEAD_SERVO_GAIN,
    pitch_gain: float = DEFAULT_HEAD_SERVO_GAIN,
    yaw_sign: float = 1.0,
    pitch_sign: float = 1.0,
    deadband_deg: float = 1.0,
    smoothing_alpha: float = 0.25,
    max_step_deg: float = 20.0,
    command_deadband_units: float = 3.0,
    move_time_ms: int = 0,
):
    """Launch headset yaw/pitch control for the pan/tilt servos."""
    if rate_hz <= 0.0:
        raise ValueError("rate_hz must be positive")
    effective_move_time_ms = move_time_ms or max(25, int(round((1000.0 / rate_hz) * 1.5)))

    config = MappingConfig(
        head_smoothing_alpha=smoothing_alpha,
        head_max_step_deg=max_step_deg,
        head_deadband_deg=deadband_deg,
    )
    mapper = HeadAngleMapper(config)

    print("=" * 60)
    print("PICO Headset Servo Teleoperation")
    print("=" * 60)
    print(f"Toggle button:  {toggle_button}")
    print(f"Servo port:     {servo_port}  baud={baudrate}")
    print(f"Rate:           {rate_hz} Hz")
    print(f"Servo1 range:   {servo1_min}..{servo1_max} center={servo1_center}")
    print(f"Servo2 range:   {servo2_min}..{servo2_max} center={servo2_center}")
    print("=" * 60)
    print()

    xr_input = SdkXRInputSource()
    servo_client = SerialServoClient(servo_port, baudrate)
    period = 1.0 / rate_hz
    enabled = False
    prev_toggle = False
    neutral: Pose | None = None
    last_s1: float | None = None
    last_s2: float | None = None

    try:
        print("Head servo teleoperation running. Press Ctrl+C to exit.")
        print(f"Press VR {toggle_button} to enable/disable head following.")
        while True:
            t0 = time.time()
            state = xr_input.poll()
            toggle = bool(state.buttons.get(toggle_button, False))

            if toggle and not prev_toggle:
                enabled = not enabled
                print(f"Head servo following {'enabled' if enabled else 'disabled'}.")
                if enabled:
                    neutral = state.headset
                    mapper.reset()
                    s1 = float(np.clip(servo1_center, servo1_min, servo1_max))
                    s2 = float(np.clip(servo2_center, servo2_min, servo2_max))
                    command_head_servo_target(servo_client, servo1=s1, servo2=s2, move_time_ms=effective_move_time_ms)
                    last_s1, last_s2 = s1, s2
            prev_toggle = toggle

            if enabled and state.headset is not None and neutral is not None:
                angles = mapper.map_pose(neutral, state.headset)
                s1, s2 = _angles_to_servo_units(
                    angles.yaw_deg,
                    angles.pitch_deg,
                    servo1_center,
                    servo2_center,
                    pitch_gain,
                    yaw_gain,
                    pitch_sign,
                    yaw_sign,
                    servo1_min,
                    servo1_max,
                    servo2_min,
                    servo2_max,
                )
                if (
                    last_s1 is None
                    or abs(s1 - last_s1) > command_deadband_units
                    or abs(s2 - last_s2) > command_deadband_units
                ):
                    command_head_servo_target(servo_client, servo1=s1, servo2=s2, move_time_ms=effective_move_time_ms)
                    last_s1, last_s2 = s1, s2

            sleep = period - (time.time() - t0)
            if sleep > 0.0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        print("\nHead servo teleoperation interrupted by user.")
    finally:
        xr_input.close()
        servo_client.close()


if __name__ == "__main__":
    tyro.cli(main)
