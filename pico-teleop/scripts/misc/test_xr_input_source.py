"""Print live XR input from the SDK-backed input source."""

from __future__ import annotations

import time

import tyro

from xrobotoolkit_teleop.core.types import ControllerInput, Pose
from xrobotoolkit_teleop.core.xr_input import SdkXRInputSource


def _pose_str(pose: Pose | None) -> str:
    if pose is None:
        return "None"
    pos = ", ".join(f"{v:.3f}" for v in pose.position)
    quat = ", ".join(f"{v:.3f}" for v in pose.orientation)
    return f"pos=[{pos}] quat_wxyz=[{quat}]"


def _controller_str(controller: ControllerInput) -> str:
    return (
        f"{_pose_str(controller.pose)} "
        f"grip={controller.grip:.3f} trigger={controller.trigger:.3f} "
        f"buttons={controller.buttons}"
    )


def main(rate_hz: float = 10.0):
    xr_input = SdkXRInputSource()
    period = 1.0 / rate_hz
    try:
        while True:
            state = xr_input.poll()
            print(f"timestamp_ns={state.timestamp_ns}")
            print(f"  headset: {_pose_str(state.headset)}")
            print(f"  left:    {_controller_str(state.left_controller)}")
            print(f"  right:   {_controller_str(state.right_controller)}")
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nXR input test interrupted.")
    finally:
        xr_input.close()


if __name__ == "__main__":
    tyro.cli(main)
