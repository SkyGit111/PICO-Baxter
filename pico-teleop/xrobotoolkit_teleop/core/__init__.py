"""Core teleoperation abstractions.

The core package is intentionally robot-agnostic: XR input is mapped into
per-subsystem targets via ``mapping`` and dispatched through ``RobotAdapter``.
"""

from xrobotoolkit_teleop.core.camera import CameraStream
from xrobotoolkit_teleop.core.mapping import ArmPoseMapper, HeadAngleMapper, MappingConfig
from xrobotoolkit_teleop.core.robot_adapter import RobotAdapter
from xrobotoolkit_teleop.core.teleop_engine import TeleopEngine
from xrobotoolkit_teleop.core.types import (
    ArmName,
    BaseTarget,
    ControllerInput,
    HeadAngles,
    HeightTarget,
    Pose,
    RobotState,
    XRState,
)
from xrobotoolkit_teleop.core.xr_input import SdkXRInputSource, XRInputSource

__all__ = [
    "ArmName",
    "ArmPoseMapper",
    "BaseTarget",
    "CameraStream",
    "ControllerInput",
    "HeadAngles",
    "HeadAngleMapper",
    "HeightTarget",
    "MappingConfig",
    "Pose",
    "RobotAdapter",
    "RobotState",
    "SdkXRInputSource",
    "TeleopEngine",
    "XRInputSource",
    "XRState",
]
