#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

"""
夹爪TCP控制客户端 - 直接TCP通信版本
用于通过TCP直接控制机械臂夹爪，无需ROS依赖

功能：
1. TCP方式发送夹爪控制命令
2. 支持频率限制和异步命令队列
"""

import socket
import threading
import time
import queue
import numpy as np


class GripperTCPClient:
    """
    TCP方式控制夹爪位置的客户端
    
    位置范围: 0-12000
        - 0: 夹爪全开
        - 12000: 夹爪全闭
    """
    
    # 默认配置
    DEFAULT_ROBOT_IP = "169.254.128.18"
    DEFAULT_ROBOT_PORT = 8080
    DEFAULT_FREQUENCY = 10  # 10Hz
    
    def __init__(self, robot_ip=None, robot_port=None, frequency=None):
        """
        初始化夹爪TCP客户端
        
        Args:
            robot_ip: 机器人IP地址
            robot_port: 机器人端口 (默认8080)
            frequency: 命令发送频率限制 (默认10Hz)
        """
        self.robot_ip = robot_ip or self.DEFAULT_ROBOT_IP
        self.robot_port = robot_port or self.DEFAULT_ROBOT_PORT
        self.frequency = frequency or self.DEFAULT_FREQUENCY
        
        self.socket = None
        self.connected = False
        self._cmd_queue = queue.Queue(maxsize=100)
        self._sender_thread = None
        
        # 频率限制
        self.min_interval = 1.0 / self.frequency
        self.last_command_time = 0
        
    def connect(self):
        """
        建立TCP连接并初始化夹爪模式
        
        Returns:
            bool: 连接是否成功
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
            self.socket.settimeout(5)
            self.socket.connect((self.robot_ip, self.robot_port))
            
            # 关闭modbus模式
            close_modbus_cmd = '{"command":"close_modbus_mode","port":1}\r\n'
            self.socket.send(close_modbus_cmd.encode('utf-8'))
            self.socket.recv(1024)
            
            time.sleep(0.01)
            
            # 设置plus模式
            set_plus_cmd = '{"command":"set_rm_plus_mode","mode":115200}\r\n'
            self.socket.send(set_plus_cmd.encode('utf-8'))
            self.socket.recv(1024)
            
            self.connected = True
            
            # 启动发送线程
            self._sender_thread = threading.Thread(target=self._send_worker, daemon=True)
            self._sender_thread.start()
            
            return True
            
        except Exception as e:
            print(f"❌ GripperTCPClient 连接失败: {e}")
            self.connected = False
            return False
    
    def _send_worker(self):
        """专用发送线程 - 从队列获取命令并发送"""
        while self.connected:
            try:
                cmd = self._cmd_queue.get(timeout=1)
                self.socket.sendall(cmd)
            except queue.Empty:
                continue
            except Exception as e:
                print(f"❌ 发送错误: {e}")
                self.connected = False
    
    def set_gripper_position(self, position, use_rate_limit=True):
        """
        设置夹爪位置
        
        Args:
            position: 目标位置 (0-12000, 0=全开, 12000=全闭)
            use_rate_limit: 是否使用频率限制 (默认True, 10Hz)
        
        Returns:
            bool: 命令是否成功发送
        """
        if not self.connected:
            print("❌ 夹爪客户端未连接")
            return False
            
        current_time = time.time()
        if use_rate_limit and (current_time - self.last_command_time < self.min_interval):
            return False
        
        try:
            position = int(np.clip(position, 0, 12000))
            cmd = f'{{"command":"hand_follow_pos","hand_pos":[{position}]}}\r\n'.encode()
            self._cmd_queue.put_nowait(cmd)
            self.last_command_time = current_time
            return True
        except queue.Full:
            print("⚠️  命令队列已满，丢弃数据包")
            return False
        except Exception as e:
            print(f"❌ 设置夹爪位置失败: {e}")
            return False
    
    def is_connected(self):
        """返回连接状态"""
        return self.connected
    
    def disconnect(self):
        """断开连接"""
        self.connected = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
    
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
    print("夹爪TCP客户端测试")
    print("=" * 50)
    
    with GripperTCPClient() as client:
        print("\n等待连接稳定...")
        time.sleep(2)
        
        print("\n开始测试序列...")
        
        print("1. 全开 (0)")
        client.set_gripper_position(0)
        time.sleep(2)
        
        print("2. 半闭合 (6000)")
        client.set_gripper_position(6000)
        time.sleep(2)
        
        print("3. 全闭 (12000)")
        client.set_gripper_position(12000)
        time.sleep(2)
        
        print("4. 全开 (0)")
        client.set_gripper_position(0)
        time.sleep(2)
        
        print("\n测试完成!")
