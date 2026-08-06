"""Pure-Python drop-in replacement for the xrobotoolkit_sdk Pybind module.

Subscribes to the EAService gRPC stream on 127.0.0.1:60061 (served by either
the C++ XRoboToolkit-PC-Service or the sibling xrbt_service Python port) and
exposes the exact same getter surface that ``bindings/py_bindings.cpp`` did.

Consumers (e.g. pico-teleop's ``xr_client.py``) should not need any changes:

    import xrobotoolkit_sdk as xrt
    xrt.init()
    xrt.get_left_controller_pose()
    xrt.close()
"""

from ._client import (
    close,
    device_control_json,
    get_A_button,
    get_B_button,
    get_X_button,
    get_Y_button,
    get_body_joints_acceleration,
    get_body_joints_pose,
    get_body_joints_timestamp,
    get_body_joints_velocity,
    get_body_timestamp_ns,
    get_headset_pose,
    get_left_axis,
    get_left_axis_click,
    get_left_controller_pose,
    get_left_grip,
    get_left_hand_is_active,
    get_left_hand_tracking_state,
    get_left_menu_button,
    get_left_trigger,
    get_motion_timestamp_ns,
    get_motion_tracker_acceleration,
    get_motion_tracker_pose,
    get_motion_tracker_serial_numbers,
    get_motion_tracker_velocity,
    get_right_axis,
    get_right_axis_click,
    get_right_controller_pose,
    get_right_grip,
    get_right_hand_is_active,
    get_right_hand_tracking_state,
    get_right_menu_button,
    get_right_trigger,
    get_time_stamp_ns,
    init,
    is_body_data_available,
    num_motion_data_available,
    send_bytes_to_device,
)

__all__ = [
    "init",
    "close",
    "get_left_controller_pose",
    "get_right_controller_pose",
    "get_headset_pose",
    "get_left_trigger",
    "get_left_grip",
    "get_right_trigger",
    "get_right_grip",
    "get_left_menu_button",
    "get_right_menu_button",
    "get_left_axis_click",
    "get_right_axis_click",
    "get_left_axis",
    "get_right_axis",
    "get_X_button",
    "get_A_button",
    "get_Y_button",
    "get_B_button",
    "get_time_stamp_ns",
    "get_left_hand_tracking_state",
    "get_right_hand_tracking_state",
    "get_left_hand_is_active",
    "get_right_hand_is_active",
    "is_body_data_available",
    "get_body_joints_pose",
    "get_body_joints_velocity",
    "get_body_joints_acceleration",
    "get_body_joints_timestamp",
    "get_body_timestamp_ns",
    "num_motion_data_available",
    "get_motion_tracker_pose",
    "get_motion_tracker_velocity",
    "get_motion_tracker_acceleration",
    "get_motion_tracker_serial_numbers",
    "get_motion_timestamp_ns",
    "device_control_json",
    "send_bytes_to_device",
]
