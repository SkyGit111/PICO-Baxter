# Ignore lint errors because this file is mostly copied from ACT (https://github.com/tonyzhaozh/act).
# ruff: noqa
"""
Realman Robot Controller
统一的真机机器人控制器，包含所有与真机相关的函数，包括机器人连接建立、机械臂控制、夹爪控制、相机控制等。
"""
import threading
from typing import Optional, List
import numpy as np
try:
    import rospy
except ImportError:
    class _RospyFallback:
        @staticmethod
        def init_node(*args, **kwargs):
            print("Warning: rospy is not installed; skipping ROS node initialization.")

        @staticmethod
        def loginfo(message):
            print(message)

        @staticmethod
        def logwarn(message):
            print(f"Warning: {message}")

        @staticmethod
        def logerr(message):
            print(f"Error: {message}")

    rospy = _RospyFallback()


import sys
import os

# 添加当前 control 目录到 sys.path，以便导入子目录中的模块
control_dir = os.path.dirname(os.path.abspath(__file__))
if control_dir not in sys.path:
    sys.path.insert(0, control_dir)

# 从 control 目录下的子目录导入
from task_planning.apis.hardware_control.gripper_tcp_client import GripperTCPClient
from task_planning.apis.hardware_control.gripper_udp_client import GripperUDPClient
from Robotic_Arm.rm_robot_interface import *

GRIPPER_POSITION_OPEN = 0


class RealmanRobotController:
    """
    统一的真机机器人控制器（单例模式）
    
    负责：
    - 机器人连接建立和管理
    - 机械臂控制
    - 夹爪控制
    - 相机控制
    - 状态获取
    """
    _instance = None
    _lock = threading.Lock()
    _initialized = False
    
    @classmethod
    def get_instance(
        cls,
        left_arm_ip: str = "169.254.128.18",
        right_arm_ip: str = "169.254.128.19",
        arm_port: int = 8080,
        local_ip: str = "169.254.128.20",
        init_node: bool = False,
    ):
        """
        获取单例实例（推荐使用此方法）
        
        Args:
            left_arm_ip: 左臂IP地址
            right_arm_ip: 右臂IP地址
            arm_port: 机械臂端口
            local_ip: 本机IP（用于UDP接收）
            init_node: 是否初始化ROS节点
            
        Returns:
            RealmanRobotController 实例
        """
        return cls(left_arm_ip=left_arm_ip, right_arm_ip=right_arm_ip, 
                   arm_port=arm_port, local_ip=local_ip, init_node=init_node)
    
    def __new__(
        cls,
        left_arm_ip: str = "169.254.128.18",
        right_arm_ip: str = "169.254.128.19",
        arm_port: int = 8080,
        local_ip: str = "169.254.128.20",
        init_node: bool = False,
    ):
        """
        单例模式实现
        
        Args:
            left_arm_ip: 左臂IP地址
            right_arm_ip: 右臂IP地址
            arm_port: 机械臂端口
            local_ip: 本机IP（用于UDP接收）
            init_node: 是否初始化ROS节点
        """
        if cls._instance is None:
            with cls._lock:
                # 双重检查锁定
                if cls._instance is None:
                    cls._instance = super(RealmanRobotController, cls).__new__(cls)
        return cls._instance
    
    def __init__(
        self,
        left_arm_ip: str = "169.254.128.18",
        right_arm_ip: str = "169.254.128.19",
        arm_port: int = 8080,
        local_ip: str = "169.254.128.20",
        init_node: bool = False,
    ):
        """
        初始化机器人控制器（仅在首次调用时执行）
        
        Args:
            left_arm_ip: 左臂IP地址
            right_arm_ip: 右臂IP地址
            arm_port: 机械臂端口
            local_ip: 本机IP（用于UDP接收）
            init_node: 是否初始化ROS节点
        """
        # 单例模式：只在第一次初始化时执行
        if RealmanRobotController._initialized:
            return
        
        with RealmanRobotController._lock:
            # 双重检查锁定
            if RealmanRobotController._initialized:
                return
            
            if init_node:
                rospy.init_node("realman_robot_controller", anonymous=True)
            
            # 保存配置
            self.left_arm_ip = left_arm_ip
            self.right_arm_ip = right_arm_ip
            self.arm_port = arm_port
            self.local_ip = local_ip
            
            # 初始化左右机械臂（使用 RoboticArm）
            self.left_arm = RoboticArm(mode=rm_thread_mode_e(2))
            self.right_arm = RoboticArm(mode=rm_thread_mode_e(2))
            
            # 创建机械臂连接
            self.left_handle = self.left_arm.rm_create_robot_arm(self.left_arm_ip, self.arm_port, 3)
            self.right_handle = self.right_arm.rm_create_robot_arm(self.right_arm_ip, self.arm_port, 3)
            
            if self.left_handle.id == -1 or self.right_handle.id == -1:
                rospy.logerr("机械臂连接失败！")
                raise RuntimeError("机械臂连接失败")
            
            rospy.loginfo(f"左臂连接成功，句柄ID: {self.left_handle.id}")
            rospy.loginfo(f"右臂连接成功，句柄ID: {self.right_handle.id}")
            
            # 初始化夹爪TCP控制客户端（用于写入位置）
            self.left_gripper_tcp = GripperTCPClient(
                robot_ip=self.left_arm_ip,
                robot_port=self.arm_port
            )
            self.right_gripper_tcp = GripperTCPClient(
                robot_ip=self.right_arm_ip,
                robot_port=self.arm_port
            )
            
            # 初始化夹爪UDP读取客户端（用于读取位置）
            self.left_gripper_udp = GripperUDPClient(
                local_ip=self.local_ip,
                udp_port=8085,
                robot_ip=self.left_arm_ip,
                robot_port=self.arm_port
            )
            self.right_gripper_udp = GripperUDPClient(
                local_ip=self.local_ip,
                udp_port=8086,
                robot_ip=self.right_arm_ip,
                robot_port=self.arm_port
            )
            
            # 连接夹爪客户端
            if self.left_gripper_tcp.connect():
                rospy.loginfo("左夹爪TCP客户端连接成功")
            else:
                rospy.logwarn("左夹爪TCP客户端连接失败")
            
            if self.right_gripper_tcp.connect():
                rospy.loginfo("右夹爪TCP客户端连接成功")
            else:
                rospy.logwarn("右夹爪TCP客户端连接失败")
            
            if self.left_gripper_udp.connect():
                rospy.loginfo(f"左夹爪UDP客户端连接成功 (端口: {self.left_gripper_udp.udp_port})")
            else:
                rospy.logwarn("左夹爪UDP客户端连接失败")
            
            if self.right_gripper_udp.connect():
                rospy.loginfo(f"右夹爪UDP客户端连接成功 (端口: {self.right_gripper_udp.udp_port})")
            else:
                rospy.logwarn("右夹爪UDP客户端连接失败")
            
            # 夹爪控制线程锁和状态跟踪
            self._gripper_lock = threading.Lock()
            self._left_gripper_busy = False
            self._right_gripper_busy = False
            self._gripper_busy_lock = threading.Lock()  # 保护busy标志的锁
            
            # 缓存状态
            self.left_arm_qpos = None
            self.left_gripper_qpos = None
            self.right_arm_qpos = None
            self.right_gripper_qpos = None
            
            # 标记已初始化（使用类变量）
            RealmanRobotController._initialized = True
    
    def get_qpos(self):
        """
        获取关节位置（归一化）
        qpos: [right_arm_qpos (7), right_gripper_qpos (1)]
        api return deg, pi0 need rad
        """
        # 使用 rm_get_current_arm_state() 获取机械臂状态
        left_arm_status = self.left_arm.rm_get_current_arm_state()
        right_arm_status = self.right_arm.rm_get_current_arm_state()

        # 检查状态码
        if left_arm_status[0] != 0:
            rospy.logwarn(f"获取左臂状态失败，状态码: {left_arm_status[0]}")
        else:
            # 提取关节角度（度转弧度）
            left_joints_deg = left_arm_status[1]['joint'].copy()
            self.left_arm_qpos = np.radians(left_joints_deg[:7])
 
        if right_arm_status[0] != 0:
            rospy.logwarn(f"获取右臂状态失败，状态码: {right_arm_status[0]}")
        else:
            # 提取关节角度（度转弧度）
            right_joints_deg = right_arm_status[1]['joint'].copy()
            self.right_arm_qpos = np.radians(right_joints_deg[:7])
        
        # 获取夹爪位置（通过UDP读取）
        self.left_gripper_qpos = [0.5] 
        self.right_gripper_qpos = [0.5]  # 默认值

        # 通过UDP读取左夹爪位置
        if self.left_gripper_udp and self.left_gripper_udp.is_connected():
            left_gripper_raw = self.left_gripper_udp.get_gripper_position()
            if left_gripper_raw is not None:
                # UDP返回0-12000范围，归一化到0-1
                left_gripper_raw = max(0, min(12000, left_gripper_raw))
                self.left_gripper_qpos = [left_gripper_raw / 12000.0]
        
        # 通过UDP读取右夹爪位置
        if self.right_gripper_udp and self.right_gripper_udp.is_connected():
            right_gripper_raw = self.right_gripper_udp.get_gripper_position()
            if right_gripper_raw is not None:
                # UDP返回0-12000范围，归一化到0-1
                right_gripper_raw = max(0, min(12000, right_gripper_raw))
                self.right_gripper_qpos = [right_gripper_raw / 12000.0]
        
        return np.concatenate([self.left_arm_qpos, self.left_gripper_qpos, self.right_arm_qpos, self.right_gripper_qpos])
    
    def _set_gripper_worker(self, side, position):
        """夹爪控制工作线程（使用TCP客户端发送命令）"""
        try:
            # 执行夹爪控制（TCP客户端，非阻塞队列方式）
            if side == "left":
                if self.left_gripper_tcp.is_connected():
                    self.left_gripper_tcp.set_gripper_position(position, use_rate_limit=True)
                else:
                    rospy.logwarn("左夹爪TCP客户端未连接")
            elif side == "right":
                if self.right_gripper_tcp.is_connected():
                    self.right_gripper_tcp.set_gripper_position(position, use_rate_limit=True)
                else:
                    rospy.logwarn("右夹爪TCP客户端未连接")
        except Exception as e:
            rospy.logwarn(f"设置{side}夹爪位置失败: {e}")
        finally:
            # 清除busy标志
            with self._gripper_busy_lock:
                if side == "left":
                    self._left_gripper_busy = False
                elif side == "right":
                    self._right_gripper_busy = False
    
    def set_left_gripper_pose(self, left_gripper_desired_pos_normalized):
        """设置夹爪位置（归一化输入：0-1，阻塞模式：如果上一条指令未完成则跳过）"""
        # 反归一化到实际位置范围(写的范围是12000)
        left_gripper_desired_pos = int(left_gripper_desired_pos_normalized * 12000)
        
        # 限制范围
        left_gripper_desired_pos = max(0, min(12000, left_gripper_desired_pos))
        
        # 检查是否有正在执行的指令，如果有则跳过本次调用（原子操作）
        with self._gripper_busy_lock:
            if self._left_gripper_busy:
                rospy.logwarn(f"夹爪正在执行指令，跳过本次调用（左:{self._left_gripper_busy}）")
                return
            # 立即设置busy标志，防止并发调用
            self._left_gripper_busy = True
        
        # 使用线程实现调用（线程内部会阻塞等待完成）
        with self._gripper_lock:
            threading.Thread(
                target=self._set_gripper_worker,
                args=("left", left_gripper_desired_pos),
                daemon=True
            ).start()
    
    def set_right_gripper_pose(self, right_gripper_desired_pos_normalized):
        """
        设置右夹爪位置（归一化输入：0-1，阻塞模式：如果上一条指令未完成则跳过）
        
        Args:
            right_gripper_desired_pos_normalized: 右夹爪归一化位置 (0-1)
        """
        # 反归一化到实际位置范围(写的范围是12000)
        right_gripper_desired_pos = int(right_gripper_desired_pos_normalized * 12000)
        
        # 限制范围
        right_gripper_desired_pos = max(0, min(12000, right_gripper_desired_pos))
        
        # 检查是否有正在执行的指令，如果有则跳过本次调用（原子操作）
        with self._gripper_busy_lock:
            if self._right_gripper_busy:
                rospy.logwarn(f"夹爪正在执行指令，跳过本次调用（右:{self._right_gripper_busy}）")
                return
            # 立即设置busy标志，防止并发调用
            self._right_gripper_busy = True
        
        # 使用线程实现调用（线程内部会阻塞等待完成）
        with self._gripper_lock:
            threading.Thread(
                target=self._set_gripper_worker,
                args=("right", right_gripper_desired_pos),
                daemon=True
            ).start()
    
    def move_right_arm_joints_canfd(self, joints_deg: List[float]):
        """
        移动右臂到指定关节角度
        
        Args:
            joints_deg: 关节角度列表（度）
        """
        self.right_arm.rm_movej_canfd(joints_deg, False, 1, 0, 70)
    
    def move_right_arm_joints_blocking(self, joints_deg: List[float], velocity: int = 20):
        """
        移动右臂到指定关节角度（阻塞模式）
        
        Args:
            joints_deg: 关节角度列表（度）
            velocity: 速度
        """
        self.right_arm.rm_movej(
            joints_deg,
            velocity,
            0,
            rm_trajectory_connect_config_e.RM_TRAJECTORY_DISCONNECT_E,
            RM_MOVE_MULTI_BLOCK
        )

    def move_right_arm_pose_canfd(self, pose: List[float]):
        """
        移动右臂到指定位姿
        
        Args:
            pose: 目标位姿列表（x, y, z, rx, ry, rz）
        """
        self.right_arm.rm_movep_canfd(pose, False, 0, 70)

    def move_left_arm_joints_canfd(self, joints_deg: List[float]):
        """
        移动左臂到指定关节角度
        
        Args:
            joints_deg: 关节角度列表（度）
        """
        self.left_arm.rm_movej_canfd(joints_deg, False, 1, 0, 70)
    
    def move_left_arm_joints_blocking(self, joints_deg: List[float], velocity: int = 20):
        """
        移动左臂到指定关节角度（阻塞模式）
        
        Args:
            joints_deg: 关节角度列表（度）
            velocity: 速度
        """
        self.left_arm.rm_movej(
            joints_deg,
            velocity,
            0,
            rm_trajectory_connect_config_e.RM_TRAJECTORY_DISCONNECT_E,
            RM_MOVE_MULTI_BLOCK
        )

    def move_left_arm_pose_canfd(self, pose: List[float]):
        """
        移动左臂到指定位姿
        
        Args:
            pose: 目标位姿列表（x, y, z, rx, ry, rz）
        """
        self.left_arm.rm_movep_canfd(pose, False, 0, 70)
    
    def reset_left_gripper(self):
        """复位夹爪：打开（使用TCP客户端）"""
        open_pos = GRIPPER_POSITION_OPEN    # 0 (全开)
        
        # 打开夹爪
        if self.left_gripper_tcp and self.left_gripper_tcp.is_connected():
            self.left_gripper_tcp.set_gripper_position(open_pos, use_rate_limit=False)
        
        rospy.sleep(2.0)  # 等待夹爪打开完成

    def reset_right_gripper(self):
        """复位夹爪：打开（使用TCP客户端）"""
        open_pos = GRIPPER_POSITION_OPEN    # 0 (全开)
        
        # 打开夹爪
        if self.right_gripper_tcp and self.right_gripper_tcp.is_connected():
            self.right_gripper_tcp.set_gripper_position(open_pos, use_rate_limit=False)
        
        rospy.sleep(2.0)  # 等待夹爪打开完成

    def reset_joints(self, reset_position_left, reset_position_right):
        """复位关节位置"""
        # 移动到复位位置（度）
        # 使用 rm_movej 控制机械臂（阻塞模式）
        # 参数：joint (度), v=20 (速度), r=0, connect=RM_TRAJECTORY_DISCONNECT_E, block=RM_MOVE_MULTI_BLOCK
        self.left_arm.rm_movej(
            reset_position_left, 
            20, 
            0, 
            rm_trajectory_connect_config_e.RM_TRAJECTORY_DISCONNECT_E, 
            RM_MOVE_MULTI_BLOCK
        )
        self.right_arm.rm_movej(
            reset_position_right, 
            20, 
            0, 
            rm_trajectory_connect_config_e.RM_TRAJECTORY_DISCONNECT_E, 
            RM_MOVE_MULTI_BLOCK
        )

    def set_lift_speed(self, speed: int):
        """
        设置升降台速度（开环控制）
        
        Args:
            speed (int): 速度百分比，-100~100
                - speed < 0: 升降台向下运动
                - speed > 0: 升降台向上运动
                - speed = 0: 升降台停止运动
        
        Returns:
            int: 函数执行的状态码
                - 0: 成功
                - 1: 控制器返回false，参数错误或机械臂状态发生错误
                - -1: 数据发送失败
                - -2: 数据接收失败或超时
                - -3: 返回值解析失败
        """
        if not hasattr(self, 'left_arm') or self.left_arm is None:
            rospy.logerr("左臂未初始化，无法控制升降台")
            return -1
        
        # 限制速度范围
        speed = max(-100, min(100, speed))
        
        status = self.left_arm.rm_set_lift_speed(speed)
        if status != 0:
            rospy.logwarn(f"设置升降台速度失败，状态码: {status}")
        else:
            rospy.loginfo(f"升降台速度设置为: {speed}%")
        
        return status
    
    def set_lift_height(self, height: int, speed: int = 50, block: bool = True):
        """
        设置升降台高度（位置闭环控制）
        
        Args:
            height (int): 目标高度，单位 mm，范围：0~2600
            speed (int): 速度百分比，1~100，默认50
            block (bool): 是否阻塞等待完成，默认True
        
        Returns:
            int: 函数执行的状态码
                - 0: 成功
                - 1: 控制器返回false，参数错误或机械臂状态发生错误
                - -1: 数据发送失败
                - -2: 数据接收失败或超时
                - -3: 返回值解析失败
        """
        if not hasattr(self, 'left_arm') or self.left_arm is None:
            rospy.logerr("左臂未初始化，无法控制升降台")
            return -1
        
        # 限制参数范围
        height = max(0, min(2600, height))
        speed = max(1, min(100, speed))
        block_int = 1 if block else 0
        
        status = self.left_arm.rm_set_lift_height(speed, height, block_int)
        if status != 0:
            rospy.logwarn(f"设置升降台高度失败，状态码: {status}")
        else:
            rospy.loginfo(f"升降台高度设置为: {height}mm (速度: {speed}%, 阻塞: {block})")
        
        return status
    
    def get_lift_state(self):
        """
        获取升降台状态
        
        Returns:
            tuple: (status_code, lift_state_dict)
                - status_code (int): 函数执行的状态码
                    - 0: 成功
                    - 1: 控制器返回false
                    - -1: 数据发送失败
                    - -2: 数据接收失败或超时
                    - -3: 返回值解析失败
                - lift_state_dict (dict): 升降台状态字典，包含以下键：
                    - 'pos': 当前位置 (mm)
                    - 'current': 当前电流 (mA)
                    - 'mode': 运行模式
                        - 0: Idle (空闲)
                        - 1: UpSpeed (向上速度控制)
                        - 2: UpPos (向上位置控制)
                        - 3: DownSpeed (向下速度控制)
                        - 4: DownPos (向下位置控制)
                    - 'err_flag': 错误标志
        """
        if not hasattr(self, 'left_arm') or self.left_arm is None:
            rospy.logerr("左臂未初始化，无法获取升降台状态")
            return -1, {}
        
        status, lift_state = self.left_arm.rm_get_lift_state()
        if status != 0:
            rospy.logwarn(f"获取升降台状态失败，状态码: {status}")
        return status, lift_state
    
    def stop_lift(self):
        """
        停止升降台运动（设置速度为0）
        
        Returns:
            int: 函数执行的状态码
        """
        return self.set_lift_speed(0)
    
    
    @classmethod
    def reset_instance(cls):
        """
        重置单例实例（用于测试或重新初始化）
        注意：这会清理现有连接，请谨慎使用
        """
        with cls._lock:
            if cls._instance is not None:
                try:
                    cls._instance.cleanup()
                except Exception as e:
                    rospy.logwarn(f"清理实例时出错: {e}")
                cls._instance = None
                cls._initialized = False
    
    @classmethod
    def is_initialized(cls) -> bool:
        """
        检查单例是否已初始化
        
        Returns:
            bool: 如果已初始化返回 True，否则返回 False
        """
        return cls._initialized
    
    def cleanup(self):
        """清理资源"""
        # 清理夹爪TCP客户端
        if hasattr(self, 'left_gripper_tcp') and self.left_gripper_tcp:
            try:
                self.left_gripper_tcp.disconnect()
                rospy.loginfo("左夹爪TCP客户端已断开")
            except Exception as e:
                rospy.logwarn(f"清理左夹爪TCP客户端失败: {e}")
        
        if hasattr(self, 'right_gripper_tcp') and self.right_gripper_tcp:
            try:
                self.right_gripper_tcp.disconnect()
                rospy.loginfo("右夹爪TCP客户端已断开")
            except Exception as e:
                rospy.logwarn(f"清理右夹爪TCP客户端失败: {e}")
        
        # 清理夹爪UDP客户端
        if hasattr(self, 'left_gripper_udp') and self.left_gripper_udp:
            try:
                self.left_gripper_udp.disconnect()
                rospy.loginfo("左夹爪UDP客户端已断开")
            except Exception as e:
                rospy.logwarn(f"清理左夹爪UDP客户端失败: {e}")
        
        if hasattr(self, 'right_gripper_udp') and self.right_gripper_udp:
            try:
                self.right_gripper_udp.disconnect()
                rospy.loginfo("右夹爪UDP客户端已断开")
            except Exception as e:
                rospy.logwarn(f"清理右夹爪UDP客户端失败: {e}")
        
        # 清理机械臂连接
        try:
            if hasattr(self, 'left_arm') and self.left_arm is not None:
                self.left_arm.rm_delete_robot_arm()
            if hasattr(self, 'right_arm') and self.right_arm is not None:
                self.right_arm.rm_delete_robot_arm()
            rospy.loginfo("机械臂连接已断开")
        except Exception as e:
            rospy.logwarn(f"清理机械臂连接失败: {e}")
