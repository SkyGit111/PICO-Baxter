# Baxter 精细遥操作控制桥

`baxter_precision_teleop.py` 是当前单臂 PICO → Baxter 实机控制入口。它保留已验证
的 UDP 协议、坐标映射和 Baxter 安全检查，并且只控制指定机械臂的平移；控制器
旋转和夹爪命令仍然禁用。

## 当前控制链

```text
PICO 4 Ultra
→ XR PC Service
→ SdkXRInputSource
→ pico_udp_sender.py
→ UDP 最新状态接收器（默认 127.0.0.1:15000）
→ Baxter 精细遥操作桥
→ latest-only 异步 Baxter IK worker
→ Baxter SDK POSITION_MODE
```

视频服务位于独立进程中，不会调用本控制桥。

## 本次控制改进

- Grip 锁定后必须检测到明确手部位移，才允许首次 IK 运动；
- 固定 Grip 锁定时的工具姿态，避免测量漂移不断改变下一帧目标；
- 首次 IK 使用锁定时的完整关节状态作为用户种子，并设置更严格的分支连续性阈值；
- IK 回退调用受次数和时间预算共同限制，失败时保持上一条安全命令；
- 阻塞的 ROS IK Service 已移入 single-flight worker，控制状态机不再同步等待；
- worker 同时最多保留一个在途请求和一个最新 pending 目标，不建立 FIFO；
- 主控制线程按 Grip session 和结果年龄验收 IK，默认拒绝超过 `250 ms` 的旧结果；
- 主线程取走一个结果时会丢弃用旧 seed 构造的 pending 请求，再以最新目标和 seed 提交；
- 笛卡尔目标不能无限领先于实测末端位置；
- 独立线程以固定频率持续发布最近一次接受的关节命令，避免慢 IK 调用造成命令断续；
- 保留 Grip 松开重置、输入超时、序列倒退、机器人状态和关节集合检查；
- CSV 诊断增加 IK request ID、请求序列、求解时间、结果年龄和 pending 覆盖计数。

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

测试覆盖四元数复制/距离/姿态松弛、笛卡尔 anti-windup、关节命令限幅、IK 回退预算、
用户种子连续性、异步 IK 的 single-flight/latest-only/session/超时逻辑，以及独立命令
发布线程的成功和失败路径。

异步化只能避免慢 IK 阻塞 Grip 和安全状态机，不能让 Baxter IK Service 本身变快。
若真机日志持续出现超过 `--max-ik-result-age` 的求解，下一阶段应按交接方案比较
PyKDL、TRAC-IK 和本地微分 IK，而不是简单放宽旧结果时限。

主程序现在提供两个显式后端：

- `--ik-backend ros-service`：默认值，保留 Baxter 原有 IK Service；
- `--ik-backend pykdl`：可选的本地 `baxter_pykdl` 后端，不经过 ROS IK Service。

两者共用同一个 single-flight/latest-only worker、Grip session、结果年龄、用户 seed、
回退和关节分支连续性检查。`pykdl` 不提供碰撞检查，尚未经过本项目 Baxter 真机验证，
因此不能因为本地求解更快就直接作为默认值。应先运行下面的只读基准，再在低速、小位移、
清空工作空间的条件下显式加入 `--ik-backend pykdl`。

## 只读 IK 后端基准

`baxter_ik_benchmark.py` 围绕当前末端位置生成中心点和六个轴向小偏移，对同一目标
比较 Baxter ROS IK Service 与 `baxter_pykdl`。它只读取状态并计算 IK，代码中没有
任何 Limb 命令调用，但仍应在清空工作空间、正确 source Baxter/ROS 环境后运行：

```bash
cd ~/pico_baxter_ws/PICO-Baxter
/usr/bin/python3 bridge/baxter_ik_benchmark.py \
  --side left \
  --offset 0.02 \
  --repeats 10 \
  --json-output /tmp/baxter-left-ik-benchmark.json
```

输出分别统计成功率、`median/P95/P99/max` 求解时间，以及解相对当前 seed 的最大
关节差。该基准不能证明本地 IK 具备碰撞安全，也不能替代 Baxter 真机运动测试；
它只用于决定下一阶段后端路线。
