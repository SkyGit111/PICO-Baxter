from __future__ import annotations

import time

import numpy as np

from xrobotoolkit_teleop.core.mapping import ArmPoseMapper, MappingConfig
from xrobotoolkit_teleop.core.robot_adapter import RobotAdapter
from xrobotoolkit_teleop.core.types import ControllerInput, Pose, RobotState, XRState


class FakeRobot(RobotAdapter):
    def __init__(self, mapping: MappingConfig):
        super().__init__()
        self.ee_pose = Pose(np.array([1.0, 2.0, 3.0]), np.array([1.0, 0.0, 0.0, 0.0]))
        self.pose_commands: list[tuple[str, Pose]] = []
        self.gripper_commands: list[tuple[str, float]] = []
        self.arm_mappers = {
            "left_arm": ArmPoseMapper("left_controller", mapping, "left_arm"),
            "right_arm": ArmPoseMapper("right_controller", mapping, "right_arm"),
        }

    def connect(self) -> None:
        pass

    def shutdown(self) -> None:
        pass

    def read_state(self) -> RobotState:
        return RobotState(timestamp=time.time())

    def get_end_effector_pose(self, arm):
        return self.ee_pose

    def command_end_effector_pose(self, arm, pose: Pose) -> None:
        self.pose_commands.append((arm, pose))

    def command_gripper(self, arm, value: float) -> None:
        self.gripper_commands.append((arm, value))


def _state(left_pos, left_grip: float, left_trigger: float = 0.0) -> XRState:
    identity = np.array([1.0, 0.0, 0.0, 0.0])
    left = ControllerInput(
        pose=Pose(np.asarray(left_pos, dtype=np.float64), identity),
        grip=left_grip,
        trigger=left_trigger,
    )
    right = ControllerInput(pose=Pose(np.zeros(3), identity), grip=0.0, trigger=0.0)
    return XRState(timestamp_ns=1, headset=None, left_controller=left, right_controller=right)


def test_grip_latches_reference_and_commands_scaled_delta():
    # Identity headset_to_world so the controller delta is in robot frame directly.
    mapping = MappingConfig(scale_factor=2.0, headset_to_world=np.eye(3))
    robot = FakeRobot(mapping)

    robot.update_arms(_state([0.0, 0.0, 0.0], left_grip=1.0, left_trigger=0.25))
    robot.update_arms(_state([0.1, 0.0, 0.0], left_grip=1.0, left_trigger=0.75))

    left_commands = [cmd for cmd in robot.pose_commands if cmd[0] == "left_arm"]
    assert len(left_commands) == 2
    np.testing.assert_allclose(left_commands[0][1].position, np.array([1.0, 2.0, 3.0]))
    np.testing.assert_allclose(left_commands[1][1].position, np.array([1.2, 2.0, 3.0]))
    assert ("left_arm", 0.25) in robot.gripper_commands
    assert ("left_arm", 0.75) in robot.gripper_commands


def test_release_clears_reference_and_stops_pose_commands():
    mapping = MappingConfig(headset_to_world=np.eye(3))
    robot = FakeRobot(mapping)

    robot.update_arms(_state([0.0, 0.0, 0.0], left_grip=1.0))
    robot.update_arms(_state([0.2, 0.0, 0.0], left_grip=0.0))

    left_commands = [cmd for cmd in robot.pose_commands if cmd[0] == "left_arm"]
    assert len(left_commands) == 1


def test_xr_snapshot_push_and_read():
    mapping = MappingConfig()
    robot = FakeRobot(mapping)
    assert robot._read_xr() is None
    s = _state([0.0, 0.0, 0.0], left_grip=0.0)
    robot.push_xr_state(s)
    assert robot._read_xr() is s
