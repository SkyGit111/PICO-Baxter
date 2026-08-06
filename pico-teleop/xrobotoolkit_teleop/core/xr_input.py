from __future__ import annotations

from typing import Protocol

import numpy as np

from xrobotoolkit_teleop.core.types import ControllerInput, Pose, XRState


class XRInputSource(Protocol):
    """Source of live XR tracking and controller state."""

    def poll(self) -> XRState:
        """Return the latest XR state snapshot."""

    def close(self) -> None:
        """Release any SDK resources."""


def _pose_from_sdk(pose_xyzw: np.ndarray | list[float] | tuple[float, ...]) -> Pose:
    pose = np.asarray(pose_xyzw, dtype=np.float64)
    if pose.shape != (7,):
        raise ValueError(f"XR pose must have shape (7,), got {pose.shape}")
    return Pose(
        position=pose[:3],
        orientation=np.array([pose[6], pose[3], pose[4], pose[5]], dtype=np.float64),
    )


class SdkXRInputSource:
    """XR input source backed by ``xrobotoolkit_sdk`` through ``XrClient``."""

    def __init__(self, client=None):
        if client is None:
            from xrobotoolkit_teleop.common.xr_client import XrClient

            client = XrClient()
        self.client = client

    def poll(self) -> XRState:
        headset = _pose_from_sdk(self.client.get_pose_by_name("headset"))
        left_axis = self.client.get_joystick_state("left")
        right_axis = self.client.get_joystick_state("right")
        left_joystick = (float(left_axis[0]), float(left_axis[1]))
        right_joystick = (float(right_axis[0]), float(right_axis[1]))
        left_buttons = {
            "X": self.client.get_button_state_by_name("X"),
            "Y": self.client.get_button_state_by_name("Y"),
            "left_menu_button": self.client.get_button_state_by_name(
                "left_menu_button"
            ),
            "left_axis_click": self.client.get_button_state_by_name(
                "left_axis_click"
            ),
        }
        right_buttons = {
            "A": self.client.get_button_state_by_name("A"),
            "B": self.client.get_button_state_by_name("B"),
            "right_menu_button": self.client.get_button_state_by_name(
                "right_menu_button"
            ),
            "right_axis_click": self.client.get_button_state_by_name(
                "right_axis_click"
            ),
        }
        buttons = {**left_buttons, **right_buttons}
        return XRState(
            timestamp_ns=int(self.client.get_timestamp_ns()),
            headset=headset,
            left_controller=ControllerInput(
                pose=_pose_from_sdk(self.client.get_pose_by_name("left_controller")),
                grip=float(self.client.get_key_value_by_name("left_grip")),
                trigger=float(self.client.get_key_value_by_name("left_trigger")),
                buttons=left_buttons,
                joystick=left_joystick,
            ),
            right_controller=ControllerInput(
                pose=_pose_from_sdk(self.client.get_pose_by_name("right_controller")),
                grip=float(self.client.get_key_value_by_name("right_grip")),
                trigger=float(self.client.get_key_value_by_name("right_trigger")),
                buttons=right_buttons,
                joystick=right_joystick,
            ),
            buttons=buttons,
        )

    def close(self) -> None:
        close = getattr(self.client, "close", None)
        if close is not None:
            close()
