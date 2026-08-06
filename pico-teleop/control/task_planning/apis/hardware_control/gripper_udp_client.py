#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
夹爪UDP读取客户端 - UDP实时推送版本
用于通过UDP实时读取机械臂夹爪位置，无需ROS依赖

功能：
1. UDP方式接收夹爪位置数据
2. 支持配置机器人实时推送
3. 线程安全的数据访问
"""

import sys
import socket
import threading
import json
import copy
import time

# 添加 Robotic_Arm 模块路径
robotic_arm_path = '/home/rm/realman-teleop/src'
if robotic_arm_path not in sys.path:
    sys.path.insert(0, robotic_arm_path)

from Robotic_Arm.rm_robot_interface import *


class GripperUDPClient:
    """
    UDP方式读取夹爪位置的客户端
    
    通过机器人实时推送获取夹爪位置数据 (rm_plus_state)
    
    位置范围: 0-12000
        - 0: 夹爪全开
        - 12000: 夹爪全闭
    """
    
    # 默认配置
    DEFAULT_LOCAL_IP = "169.254.128.20"
    DEFAULT_UDP_PORT = 8085
    DEFAULT_ROBOT_IP = "169.254.128.18"
    DEFAULT_ROBOT_PORT = 8080
    DEFAULT_PUSH_INTERVAL = 5  # 推送周期(ms)
    
    def __init__(self, local_ip=None, udp_port=None, robot_ip=None, robot_port=None, max_port_retries=10):
        """
        初始化夹爪UDP客户端
        
        Args:
            local_ip: 本机IP地址（接收UDP数据）
            udp_port: UDP端口 (默认8085)
            robot_ip: 机器人IP地址（用于配置实时推送）
            robot_port: 机器人API端口 (默认8080)
            max_port_retries: 端口被占用时最大重试次数
        """
        self.local_ip = local_ip or self.DEFAULT_LOCAL_IP
        self.udp_port = udp_port or self.DEFAULT_UDP_PORT
        self.robot_ip = robot_ip or self.DEFAULT_ROBOT_IP
        self.robot_port = robot_port or self.DEFAULT_ROBOT_PORT
        self.max_port_retries = max_port_retries
        
        self.socket = None
        self.robot = None
        self.handle = None
        self.is_running = False
        
        # 数据存储
        self.atom_data = None
        self.data_lock = threading.Lock()
        self._recv_thread = None
    
    def connect(self, push_interval=None):
        """
        建立UDP连接并启动数据接收
        
        Args:
            push_interval: 推送周期(ms)，默认5ms
        
        Returns:
            bool: 连接是否成功
        """
        push_interval = push_interval or self.DEFAULT_PUSH_INTERVAL
        
        # 1. 绑定UDP端口
        if not self._bind_udp_socket():
            return False
        
        # 2. 连接机器人并配置实时推送
        if not self._setup_robot_push(push_interval):
            self._cleanup_socket()
            return False
        
        # 3. 启动接收线程
        self.is_running = True
        self._recv_thread = threading.Thread(target=self._receive_data, daemon=True)
        self._recv_thread.start()
        
        return True
    
    def _bind_udp_socket(self):
        """绑定UDP端口，端口被占用时自动尝试下一个"""
        retry_count = 0
        original_port = self.udp_port
        
        while retry_count < self.max_port_retries:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket.settimeout(1.0)
                self.socket.bind((self.local_ip, self.udp_port))
                return True
            except OSError:
                retry_count += 1
                self.udp_port += 1
                if retry_count >= self.max_port_retries:
                    print(f"❌ 无法绑定UDP端口，已尝试 {self.max_port_retries} 次 (原始端口: {original_port})")
                    return False
        return False
    
    def _setup_robot_push(self, push_interval):
        """连接机器人并配置实时推送"""
        try:
            self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
            self.handle = self.robot.rm_create_robot_arm(self.robot_ip, self.robot_port)
            
            if self.handle.id == -1:
                print(f"❌ 机器人连接失败: {self.robot_ip}:{self.robot_port}")
                return False
            
            # 配置UDP推送 (plus_state=1 启用夹爪数据)
            config = rm_udp_custom_config_t()
            config.plus_state = 1
            
            config_setting = rm_realtime_push_config_t(
                push_interval,  # 推送周期(ms)
                True,           # 启用推送
                self.udp_port,  # 目标端口
                0,              # 强制刷新
                self.local_ip,  # 目标IP
                config          # 自定义配置
            )
            
            result = self.robot.rm_set_realtime_push(config_setting)
            if result != 0:
                print(f"⚠️  UDP实时推送配置返回码: {result}")
            
            return True
            
        except Exception as e:
            print(f"❌ 机器人连接异常: {e}")
            return False
    
    def _receive_data(self):
        """数据接收线程"""
        while self.is_running:
            try:
                data, addr = self.socket.recvfrom(4096)
                try:
                    text = data.decode("utf-8")
                    json_data = json.loads(text)
                    with self.data_lock:
                        self.atom_data = json_data
                except (UnicodeDecodeError, json.JSONDecodeError):
                    pass
            except socket.timeout:
                continue
            except Exception as e:
                if self.is_running:
                    print(f"❌ UDP接收异常: {e}")
    
    def get_raw_data(self):
        """
        获取原始UDP数据
        
        Returns:
            dict: 原始JSON数据，无数据时返回None
        """
        with self.data_lock:
            if self.atom_data is not None:
                return copy.deepcopy(self.atom_data)
            return None
    
    def get_gripper_position(self):
        """
        获取夹爪位置
        
        Returns:
            float: 夹爪位置 (0-12000)，无数据时返回None
        """
        data = self.get_raw_data()
        if data is None:
            return None
        
        try:
            if 'rm_plus_state' in data and 'pos' in data['rm_plus_state']:
                pos_raw = data['rm_plus_state']['pos']
                if isinstance(pos_raw, list):
                    return float(pos_raw[0]) if len(pos_raw) > 0 else None
                else:
                    return float(pos_raw)
            return None
        except (KeyError, TypeError, IndexError, ValueError):
            return None
    
    def get_gripper_state(self):
        """
        获取完整夹爪状态
        
        Returns:
            dict: 夹爪状态字典，包含position等字段，无数据时返回None
        """
        data = self.get_raw_data()
        if data is None:
            return None
        
        try:
            if 'rm_plus_state' in data:
                state = data['rm_plus_state']
                return {
                    'position': self.get_gripper_position(),
                    'raw_state': state
                }
            return None
        except (KeyError, TypeError):
            return None
    
    def is_connected(self):
        """返回连接状态"""
        return self.is_running and self.socket is not None
    
    def _cleanup_socket(self):
        """清理Socket资源"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
    
    def _stop_robot_push(self):
        """停止机器人UDP推送"""
        if self.robot:
            try:
                config = rm_udp_custom_config_t()
                config_setting = rm_realtime_push_config_t(
                    5, False, self.udp_port, 0, self.local_ip, config
                )
                self.robot.rm_set_realtime_push(config_setting)
            except:
                pass
    
    def _cleanup_robot(self):
        """清理机器人连接"""
        if self.robot:
            try:
                self.robot.rm_delete_robot_arm()
            except:
                pass
            self.robot = None
            self.handle = None
    
    def disconnect(self):
        """断开连接并清理资源"""
        self.is_running = False
        
        # 等待接收线程结束
        if self._recv_thread and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=2.0)
        
        self._stop_robot_push()
        self._cleanup_socket()
        self._cleanup_robot()
    
    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.disconnect()
        return False


if __name__ == "__main__":
    print("=" * 50)
    print("夹爪UDP客户端测试 - 读取夹爪位置")
    print("=" * 50)
    
    # 配置
    LOCAL_IP = "169.254.128.20"
    ROBOT_IP = "169.254.128.18"
    
    with GripperUDPClient(local_ip=LOCAL_IP, robot_ip=ROBOT_IP) as client:
        print(f"\n等待UDP数据 (端口: {client.udp_port})...")
        time.sleep(2)
        
        print("\n开始持续读取夹爪位置 (5Hz)，按 Ctrl+C 退出...\n")
        
        try:
            while True:
                position = client.get_gripper_position()
                if position is not None:
                    print(f"📍 夹爪位置: {position:.1f}")
                else:
                    print("⏳ 等待数据...")
                time.sleep(0.2)  # 5Hz
        except KeyboardInterrupt:
            print("\n\n用户中断")
    
    print("测试完成!")


