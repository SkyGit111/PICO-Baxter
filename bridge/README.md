# Baxter 精细遥操作控制桥

`baxter_precision_teleop.py` 是当前单臂 PICO → Baxter 实机控制入口。它保留已验证
的 UDP 协议、坐标映射和 Baxter 安全检查，并且只控制指定机械臂的平移；控制器
旋转和夹爪命令仍然禁用。

## 当前控制链

```text
PICO 4 Ultra
→ XR PC Service
→ SdkXRInputSource
→ TeleopEngine
→ UDP 最新状态接收器（默认 127.0.0.1:15000）
→ Baxter 精细遥操作桥
→ Baxter IK Service
→ Baxter SDK POSITION_MODE
```

视频服务位于独立进程中，不会调用本控制桥。

## 本次控制改进

- Grip 锁定后必须检测到明确手部位移，才允许首次 IK 运动；
- 固定 Grip 锁定时的工具姿态，避免测量漂移不断改变下一帧目标；
- 首次 IK 使用锁定时的完整关节状态作为用户种子，并设置更严格的分支连续性阈值；
- IK 回退调用受次数和时间预算共同限制，失败时保持上一条安全命令；
- 笛卡尔目标不能无限领先于实测末端位置；
- 独立线程以固定频率持续发布最近一次接受的关节命令，避免慢 IK 调用造成命令断续；
- 保留 Grip 松开重置、输入超时、序列倒退、机器人状态和关节集合检查；
- CSV 诊断增加命令超前限幅状态。

## 重要安全边界

- 程序没有 `--execute` 时拒绝发送机器人命令；
- 程序不会自动启用 Baxter；
- 只使用普通 `POSITION_MODE`，不使用 `RAW_POSITION_MODE`；
- Grip 是持续按住才运动的 dead-man switch；
- 发生安全停止后，必须完整松开 Grip 才能重新解锁；
- 本地单元测试不等于 Baxter 真机验证。

在机器人主机上运行前，必须正确 source Baxter/ROS 环境。先查看参数：

```bash
/usr/bin/python3 bridge/baxter_precision_teleop.py --help
```

首次实机测试应保持视频关闭、降低机械臂速度、清空工作空间，并由操作员随时准备
松开 Grip 或使用急停。不要在没有完成小位移方向检查和 IK dry-run 的情况下直接
扩大运动范围。

## 无硬件测试

以下测试使用 ROS、Baxter 消息和 Limb 的内存替身，不连接机器人，也不会发送
任何真实关节命令：

```bash
/usr/bin/python3 -m unittest discover -s bridge/tests -v
```

测试覆盖四元数距离与姿态松弛、笛卡尔 anti-windup、关节命令限幅、IK 回退预算、
用户种子连续性以及独立命令发布线程的成功和失败路径。
