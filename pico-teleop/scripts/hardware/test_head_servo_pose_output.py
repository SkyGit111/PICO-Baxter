"""Print live headset pose (XYZ + Euler RPY deg) and pan/tilt servo targets.

Does not open the serial servo port or move hardware. Start
XRoboToolkit-PC-Service-Python first, then run from the repo root.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from control.head_servo import DEFAULT_HEAD_SERVO_GAIN
from xrobotoolkit_teleop.core.mapping import HeadAngleMapper, MappingConfig
from xrobotoolkit_teleop.core.types import Pose
from xrobotoolkit_teleop.core.xr_input import SdkXRInputSource


def _quat_wxyz_to_rpy_deg(q: np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(np.array([w, x, y, z]))
    if n > 0.0:
        w, x, y, z = w / n, x / n, y / n, z / n
    sinp = 2.0 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return tuple(float(np.degrees(v)) for v in (roll, pitch, yaw))


def _pose_str(pose: Pose) -> str:
    pos = ", ".join(f"{v:.4f}" for v in pose.position)
    roll, pitch, yaw = _quat_wxyz_to_rpy_deg(pose.orientation)
    rpy = ", ".join(f"{v:.4f}" for v in (roll, pitch, yaw))
    return f"pos=[{pos}] euler_rpy_deg=[{rpy}] (roll,pitch,yaw)"


def main(
    rate_hz: float = 30.0,
    samples: int = 0,
    recenter_button: str = "X",
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
):
    """Poll live headset poses and print mapped servo targets."""
    if rate_hz <= 0.0:
        raise ValueError("rate_hz must be positive")
    if samples < 0:
        raise ValueError("samples must be non-negative")

    config = MappingConfig(
        head_smoothing_alpha=smoothing_alpha,
        head_max_step_deg=max_step_deg,
        head_deadband_deg=deadband_deg,
    )
    mapper = HeadAngleMapper(config)
    xr_input = SdkXRInputSource()
    period = 1.0 / rate_hz
    neutral_pose: Pose | None = None
    prev_recenter_pressed = False
    count = 0

    print("=" * 60)
    print("Head Pose To Servo Target Test")
    print("=" * 60)
    print("This only prints targets; it does not move servos.")
    print(f"Rate:           {rate_hz} Hz")
    print(f"Samples:        {'unlimited' if samples == 0 else samples}")
    print(f"Recenter:       VR {recenter_button} button")
    print("=" * 60)
    print()

    try:
        while samples == 0 or count < samples:
            t0 = time.time()
            state = xr_input.poll()
            if state.headset is None:
                print("headset=None")
                time.sleep(period)
                continue

            recenter_pressed = bool(state.buttons.get(recenter_button, False))
            if neutral_pose is None or (recenter_pressed and not prev_recenter_pressed):
                neutral_pose = state.headset
                mapper.reset()
                print(f"Neutral headset pose latched: {_pose_str(neutral_pose)}")

            prev_recenter_pressed = recenter_pressed
            angles = mapper.map_pose(neutral_pose, state.headset)
            servo1 = float(
                np.clip(
                    servo1_center + pitch_sign * pitch_gain * angles.pitch_deg,
                    servo1_min,
                    servo1_max,
                )
            )
            servo2 = float(
                np.clip(
                    servo2_center + yaw_sign * yaw_gain * angles.yaw_deg,
                    servo2_min,
                    servo2_max,
                )
            )
            print(
                f"timestamp_ns={state.timestamp_ns} "
                f"yaw={angles.yaw_deg:7.2f} pitch={angles.pitch_deg:7.2f} "
                f"servo1={servo1:7.2f} servo2={servo2:7.2f} "
                f"{_pose_str(state.headset)}"
            )
            count += 1

            sleep = period - (time.time() - t0)
            if sleep > 0.0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        print("\nHead pose servo target test interrupted by user.")
    finally:
        xr_input.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--samples", type=int, default=0)
    parser.add_argument("--recenter-button", default="X")
    parser.add_argument("--servo1-center", type=float, default=385.0)
    parser.add_argument("--servo2-center", type=float, default=500.0)
    parser.add_argument("--servo1-min", type=float, default=270.0)
    parser.add_argument("--servo1-max", type=float, default=500.0)
    parser.add_argument("--servo2-min", type=float, default=430.0)
    parser.add_argument("--servo2-max", type=float, default=570.0)
    parser.add_argument("--yaw-gain", type=float, default=DEFAULT_HEAD_SERVO_GAIN)
    parser.add_argument("--pitch-gain", type=float, default=DEFAULT_HEAD_SERVO_GAIN)
    parser.add_argument("--yaw-sign", type=float, default=1.0)
    parser.add_argument("--pitch-sign", type=float, default=1.0)
    parser.add_argument("--deadband-deg", type=float, default=1.0)
    parser.add_argument("--smoothing-alpha", type=float, default=0.25)
    parser.add_argument("--max-step-deg", type=float, default=20.0)
    args = parser.parse_args()
    main(**vars(args))
