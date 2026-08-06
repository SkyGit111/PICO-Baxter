"""
Realman Robot Hardware Interface

Wraps the RealmanRobotController to provide a unified interface
matching the XRoboToolkit hardware interface pattern.
"""

import sys
import os
from typing import List, Optional, Union
import numpy as np

# Add control directory to path to import RealmanRobotController
control_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), 'control')
if control_dir not in sys.path:
    sys.path.insert(0, control_dir)

from realman_robot_controller import RealmanRobotController


class RealmanArmInterface:
    """
    Interface for a single Realman robot arm.
    
    This class wraps the RealmanRobotController singleton and provides
    a unified interface for controlling one arm (left or right).
    
    Args:
        arm_name: Name of the arm ('left_arm' or 'right_arm')
        robot_ip: IP address of the robot
        robot_port: Port for robot connection
        local_ip: Local IP for UDP communication
        dt: Control timestep
    """
    
    # Number of joints for Realman arm (7-DOF)
    NUM_JOINTS = 7
    
    # Gripper range
    GRIPPER_MIN = 0
    GRIPPER_MAX = 12000
    
    def __init__(
        self,
        arm_name: str = "left_arm",
        robot_ip: str = "169.254.128.18",
        robot_port: int = 8080,
        local_ip: str = "169.254.128.20",
        dt: float = 0.01,
    ):
        self.arm_name = arm_name
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.local_ip = local_ip
        self.dt = dt
        
        # Initialize the singleton controller if not already done
        if not RealmanRobotController.is_initialized():
            self.controller = RealmanRobotController.get_instance(
                left_arm_ip=robot_ip if arm_name == "left_arm" else "169.254.128.18",
                right_arm_ip=robot_ip if arm_name == "right_arm" else "169.254.128.19",
                arm_port=robot_port,
                local_ip=local_ip,
                init_node=True,
            )
        else:
            self.controller = RealmanRobotController._instance
        
        # Select arm interface based on arm_name
        if arm_name == "left_arm":
            self.arm = self.controller.left_arm
            self.gripper_tcp = self.controller.left_gripper_tcp
            self.gripper_udp = self.controller.left_gripper_udp
        elif arm_name == "right_arm":
            self.arm = self.controller.right_arm
            self.gripper_tcp = self.controller.right_gripper_tcp
            self.gripper_udp = self.controller.right_gripper_udp
        else:
            raise ValueError(f"Invalid arm_name: {arm_name}. Must be 'left_arm' or 'right_arm'")
        
        # State tracking
        self._joint_positions = np.zeros(self.NUM_JOINTS)
        self._joint_velocities = np.zeros(self.NUM_JOINTS)
        self._gripper_position = 0.0  # Normalized 0-1
        
        # Desired state for control
        self.q_des = None
        self.q_des_gripper = 0.0
        
    def get_joint_positions(self, joint_names: Optional[Union[str, List[str]]] = None) -> Union[float, np.ndarray]:
        """
        Get the current joint positions of the arm.
        
        Args:
            joint_names: Name(s) of joints to get positions for. If None, returns all.
            
        Returns:
            Joint positions in radians. Shape: (NUM_JOINTS,) or single float.
        """
        # Get current state from controller
        status = self.arm.rm_get_current_arm_state()
        
        if status[0] != 0:
            print(f"Warning: Failed to get arm state for {self.arm_name}, status: {status[0]}")
            return self._joint_positions.copy()
        
        # Extract joint angles (degrees to radians)
        joints_deg = status[1]['joint']
        self._joint_positions = np.radians(joints_deg[:self.NUM_JOINTS])
        
        return self._joint_positions.copy()
    
    def get_joint_velocities(self, joint_names: Optional[Union[str, List[str]]] = None) -> Union[float, np.ndarray]:
        """
        Get the current joint velocities of the arm.
        
        Args:
            joint_names: Name(s) of joints to get velocities for. If None, returns all.
            
        Returns:
            Joint velocities in rad/s. Shape: (NUM_JOINTS,) or single float.
        """
        # Realman API doesn't directly provide velocities, return zeros
        # Could be estimated from position differences if needed
        return self._joint_velocities.copy()
    
    def set_joint_positions(
        self,
        positions: Union[float, List[float], np.ndarray],
        **kwargs
    ) -> bool:
        """
        Move the arm to the given joint positions using CANFD.
        
        Args:
            positions: Desired joint positions in radians. Shape: (NUM_JOINTS,)
            **kwargs: Additional arguments (velocity, acceleration, etc.)
            
        Returns:
            bool: True if command was sent successfully
        """
        # Convert to numpy array if needed
        if isinstance(positions, (list, tuple)):
            positions = np.array(positions)
        elif isinstance(positions, float):
            positions = np.array([positions])
        
        # Ensure correct length
        if len(positions) != self.NUM_JOINTS:
            print(f"Warning: Expected {self.NUM_JOINTS} joint positions, got {len(positions)}")
            positions = positions[:self.NUM_JOINTS]
        
        # Store desired position
        self.q_des = positions.copy()
        
        # Convert radians to degrees for Realman API
        joints_deg = np.degrees(positions).tolist()
        
        # Send command via CANFD (non-blocking, continuous control)
        # rm_movej_canfd(joints, follow_mode, trajectory_mode, speed, acceleration)
        self.arm.rm_movej_canfd(joints_deg, False, 1, 0, 70)
        
        return True
    
    def get_gripper_position(self) -> float:
        """
        Get the current gripper position (normalized 0-1).
        
        Returns:
            float: Gripper position (0=open, 1=closed)
        """
        if self.gripper_udp and self.gripper_udp.is_connected():
            raw_pos = self.gripper_udp.get_gripper_position()
            if raw_pos is not None:
                # Clamp and normalize 0-12000 to 0-1
                raw_pos = max(self.GRIPPER_MIN, min(self.GRIPPER_MAX, raw_pos))
                self._gripper_position = raw_pos / self.GRIPPER_MAX
        
        return self._gripper_position
    
    def set_gripper_position(self, position: float) -> bool:
        """
        Set the gripper position.
        
        Args:
            position: Target position (0=open, 1=closed, normalized)
            
        Returns:
            bool: True if command was sent successfully
        """
        # Clamp to valid range
        position = max(0.0, min(1.0, position))
        self.q_des_gripper = position
        
        # Convert normalized 0-1 to raw 0-12000
        raw_position = int(position * self.GRIPPER_MAX)
        
        # Send via TCP client
        if self.gripper_tcp and self.gripper_tcp.is_connected():
            return self.gripper_tcp.set_gripper_position(raw_position, use_rate_limit=True)
        
        return False
    
    def set_catch_pos(self, pos: float) -> bool:
        """
        Set the gripper position (alias for set_gripper_position).
        
        Args:
            pos: Target position (0=open, 1=closed, normalized)
            
        Returns:
            bool: True if command was sent successfully
        """
        return self.set_gripper_position(pos)
    
    def go_home(self) -> bool:
        """
        Move the arm to a pre-defined home pose.
        
        Returns:
            bool: True if the action was successful
        """
        # Default home position in degrees for Realman arm
        # These values may need to be adjusted based on the specific robot
        home_position_deg = [0, 0, 0, 0, 0, 0, 0]
        
        # Use blocking move to home position
        self.arm.rm_movej(
            home_position_deg,
            20,  # velocity
            0,   # radius
            0,   # trajectory_connect (RM_TRAJECTORY_DISCONNECT_E)
            1    # block (RM_MOVE_MULTI_BLOCK)
        )
        
        # Also reset gripper to open
        self.set_gripper_position(0.0)
        
        return True
    
    def get_ee_pose(self) -> np.ndarray:
        """
        Get the current end effector pose.
        
        Returns:
            np.ndarray: End effector pose [x, y, z, qw, qx, qy, qz]
        """
        status, pose_data = self.arm.rm_get_current_arm_state()
        
        if status != 0:
            print(f"Warning: Failed to get EE pose for {self.arm_name}, status: {status}")
            return np.zeros(7)
        
        # pose_data contains pose information
        # Format may vary based on Realman API
        if 'pose' in pose_data:
            return np.array(pose_data['pose'])
        
        return np.zeros(7)
    
    def stop(self) -> None:
        """
        Stop the arm and gripper motion.
        """
        # Stop arm motion
        self.arm.rm_stop()
        
        # Stop gripper
        if self.gripper_tcp and self.gripper_tcp.is_connected():
            # Send stop command (position hold)
            current_pos = self.get_gripper_position()
            self.set_gripper_position(current_pos)
    
    def __del__(self):
        """Cleanup when object is deleted."""
        try:
            self.stop()
        except:
            pass
