# PICO 视频流与 Baxter 控制流联合优化交接摘要

> 更新时间：2026-08-31（Asia/Shanghai）
> 仓库：`PICO-Baxter`
> 当前分支：`feat/d455-remote-vision`
> 文档基线：`03794a3 优化 PICO 视频低延迟与积压保护`
> 重要状态：本文主体保留该基线时的真机日志和问题分析；当前实现状态以紧随其后的“实施更新”和 `git log` 为准。

## 0. 2026-08-31 实施更新

本文后续章节是实施前的交接快照，其中“异步 IK 尚未实现”“QuaternionValue 修复未提交”
等措辞用于保留当时证据，不再代表当前代码状态。现已完成：

- 修复不可变 Baxter orientation 导致的姿态松弛崩溃，并默认关闭未经真机验证的姿态松弛；
- 将阻塞 IK 移入 single-flight/latest-only worker，主状态机不再同步等待 ROS IK Service；
- Grip session 隔离、旧结果年龄拒绝、pending 覆盖、旧 seed pending 丢弃和 worker 异常恢复；
- 保留独立关节命令发布线程，并扩充 IK request/延迟/积压 CSV 诊断；
- 新增只读 `baxter_ik_benchmark.py`，比较 ROS Service 与 `baxter_pykdl`；
- 新增显式可选的 `--ik-backend pykdl`，默认仍为 `ros-service`；
- 控制侧 23 项、视频侧 49 项本地自动化测试通过。

上述结果均不是 Baxter/PICO/D455 真机验收。每个 Baxter 入口进程仍只负责单臂平移，
控制器旋转尚未接入。`pykdl` 不提供碰撞检查，必须先做只读基准，再进行低速、小位移、
清空工作空间的显式试验，不能直接设为默认后端。

后续控制阶段又加入了默认关闭的双臂/夹爪试验路径：XR sender 可向两个本地 UDP 端口
扇出同一份双手状态，左右臂各运行独立进程；每侧可显式加入 `--enable-gripper`，由
独立 latest-only 线程非阻塞控制已人工校准的电动夹爪或吸盘。该路径同样只有自动化测试，
没有 Baxter 真机验收，也没有双臂互相碰撞检测；详细启动方式见 `bridge/README.md`。

## 1. 给接手者的最短结论

本项目包含两条逻辑独立、但需要联合验收的实时链路：

```text
控制链：PICO tracking → XR PC Service → UDP → Baxter 控制桥 → IK → Baxter

视频链：D455 RGB → GStreamer SBS/H.264 → TCP → PICO Remote Vision
```

目前最重要的事实如下：

1. 视频流曾在真实 D455/PICO 上成功显示，但历史真机主观延迟约为 `2～3 s` 且不稳定。随后完成了发送端积压保护和低延迟参数优化，但优化后的真机延迟尚未重新测量。
2. 控制流的 PICO 捕获、UDP 通信和基本坐标映射不是当前主要矛盾；核心问题集中在 Baxter 7DoF IK 的延迟、分支连续性和目标跟随之间的取舍。
3. 最近一次 Baxter 真机日志证明同步 IK 会把标称 `50 Hz` 的控制循环阻塞到数百毫秒甚至超过一秒，已观察到 `623.9 ms`、`1771.2 ms` 和 `1068.3 ms` 的单次 IK 延迟。
4. 当前代码只有“关节命令发布线程”独立于 IK，目标计算与 IK 仍在主循环同步执行。因此它只能在慢 IK 时重复发送旧命令，不能保持目标更新和控制状态机的实时性。
5. 实验性的软姿态松弛曾因 Baxter orientation 对象不可变而触发 `AttributeError: can't set attribute`。工作区中的未提交修改已经修复该崩溃，并把姿态松弛默认关闭，但该修复尚未提交，也尚未再次真机验证。
6. 下一步不要继续盲目调 `max-command-lead`、姿态角或 IK 阈值。优先把 IK 请求/响应与固定频率控制状态机真正解耦，并设计 latest-only、session-safe、stale-result rejection。
7. 视频和控制仍应保持独立进程，避免一个链路故障直接杀死另一个；但应在同一场联合测试中统一采集 CPU、网络、视频积压、XR packet age、IK latency 和控制跟踪误差。

## 2. 仓库协作约束

根目录 `AGENTS.md` 要求：

- 仓库自行维护的 README、架构说明、部署说明和测试手册使用简体中文；
- 命令、标识符、协议字段、路径和命令行参数保持原文；
- `pico-teleop/`、`XR-PC-Service-Python/`、`pico-PC-Service-pybind/` 是上游快照，默认不要修改其文档或进行无关重构；
- 自动化测试、合成媒体测试和真机验证必须明确区分。

接手时必须先运行：

```bash
git status --short
git diff -- bridge/baxter_precision_teleop.py
git log --oneline --decorate -12
```

不要覆盖用户或前一对话留下的未提交修改。

## 3. 当前仓库版本与工作区状态

主要提交：

| 提交 | 内容 |
| --- | --- |
| `096e152` | 导入最初 PICO-Baxter 工作区 |
| `72241a9` | 添加低延迟 D455 Remote Vision sender |
| `7f6d786` | 添加 PICO 控制的 Remote Vision listener |
| `789dd17` | 完成 D455 Remote Vision 服务 |
| `c35d3ea` | GStreamer GI 统一使用系统 Python |
| `088c41a` | 仓库自维护文档改为简体中文 |
| `e4c52ee` | 增强 Baxter 精细遥操作安全与连续性 |
| `03794a3` | 视频低延迟与积压保护，当前 `HEAD` |

截至本文生成时，工作区只有：

```text
 M bridge/baxter_precision_teleop.py
```

该未提交 diff 只包含以下控制侧修复：

- 删除 `copy.copy(reference)` 后再写四元数属性的做法；
- 新增 `QuaternionValue` dataclass 作为可写/独立的四元数返回值；
- `relax_orientation()` 总是返回新的 `QuaternionValue`；
- `--orientation-relax-after` 默认值从 `3` 改为 `0`，即默认关闭未完成真机验证的软姿态松弛。

不要把这份 dirty state 描述成“已完成优化”。它只是修复一个已复现崩溃，并采取更保守的默认值。

## 4. 总体架构与联合运行原则

### 4.1 控制链

```text
PICO 4 Ultra
→ XR PC Service
→ SdkXRInputSource
→ bridge/pico_udp_sender.py
→ UDP JSON，默认 127.0.0.1:15000
→ bridge/baxter_precision_teleop.py
→ Baxter PositionKinematicsNode IK Service
→ baxter_interface.Limb.set_joint_positions(raw=False)
```

### 4.2 视频链

```text
Intel RealSense D455 RGB
→ V4L2/GStreamer
→ 1280×720 RGB 左右复制成 2560×720 SBS
→ x264 低延迟 H.264 Annex-B AU
→ 4 字节大端长度 + 完整 AU
→ TCP
→ PICO Remote Vision
```

支持两种视频网络模式：

- `direct`：机器人主机主动连接 PICO TCP `12345`；
- `listen`：机器人主机监听 TCP `13579`，处理 PICO 的 `OPEN_CAMERA` / `CLOSE_CAMERA`，再建立视频连接。

### 4.3 为什么仍然要分进程

联合优化不等于合并成同一个 Python 进程。保持独立进程有以下必要性：

- 视频服务不依赖 ROS、Baxter SDK 或 XR PC Service；
- 视频编码、TCP 重连或 D455 故障不应让 Baxter 控制桥退出；
- Baxter IK 卡顿或 ROS 故障不应关闭 PICO 视频；
- 两条链路可分别降级和重启；
- 联合测试时可以通过同一时间轴关联资源争用和网络拥塞。

合理的最终形态是“独立进程 + 统一启动/停止脚本 + 统一日志目录 + 联合健康监控”，不是把所有逻辑塞进一个事件循环。

## 5. 控制数据协议

`bridge/pico_udp_sender.py` 产生协议版本 `1` 的 JSON UDP 包，核心字段如下：

```json
{
  "version": 1,
  "seq": 123,
  "mode": "real",
  "sent_monotonic_ns": 123456789,
  "sent_wall_time_ns": 123456789,
  "xr_timestamp_ns": 123456789,
  "valid": true,
  "head": {
    "position": [0.0, 1.6, 0.0],
    "orientation_wxyz": [1.0, 0.0, 0.0, 0.0]
  },
  "left": {
    "pose": {
      "position": [0.0, 0.0, 0.0],
      "orientation_wxyz": [1.0, 0.0, 0.0, 0.0]
    },
    "grip": 1.0,
    "trigger": 0.0,
    "buttons": {}
  },
  "right": {}
}
```

当前 Baxter 控制桥只使用所选手柄的：

- `pose.position`；
- `grip`。

当前明确禁用：

- PICO 手柄旋转到 Baxter 末端旋转的映射；
- `trigger` 到夹爪的控制；
- 双臂同时控制。

当前坐标映射是：

```text
Baxter Δx = -PICO Δz
Baxter Δy = -PICO Δx
Baxter Δz = +PICO Δy
```

脚本启动时也会打印：

```text
Mapping: [-PICO z, -PICO x, +PICO y]
```

## 6. 当前控制实现的完整流程

入口：`bridge/baxter_precision_teleop.py`

### 6.1 输入接收

`LatestPacketReceiver` 使用独立 UDP 线程：

- 接收一个数据包后继续 drain socket，只保留队列中最新包；
- JSON 解码错误不会直接驱动机器人；
- 主循环检查协议版本、`valid`、序列号、发送时间和本机接收超时；
- `--max-age` 默认 `0.50 s`；
- `--connection-timeout` 默认 `0.75 s`；
- 安全停止后必须完整松开 Grip 才能重新解锁。

注意：跨主机时 `sent_monotonic_ns` 不能直接比较，因为 monotonic clock 不共享时基。必须确认当前 packet age 实现是否只在同一主机部署时成立；如果发送端和控制端拆到不同主机，应优先审计这一点。

### 6.2 Grip 状态机

Grip 使用 hysteresis：

- `--grip-press 0.60`；
- `--grip-release 0.40`。

未锁定时：

1. 滤波器和 moving deadband 持续以当前手位置为中心，避免旧运动泄漏到下一次 Grip；
2. 确认 Baxter 已启用、未 stopped、未 error；程序本身不会自动 enable；
3. 记录手柄位置参考、Baxter 末端位置、末端姿态和当前 7 个关节角；
4. 把 Grip 时的当前关节角作为首个 `SEED_USER`；
5. 在手位移超过 `--motion-start-threshold` 前不调用 IK。

默认启动门槛：

```text
--motion-start-threshold 0.003
```

即 Grip 后手柄必须移动约 `3 mm` 才启用 IK。这解决了“只按 Grip，Baxter 就因冗余 IK 自己扭动”的真机问题，但会引入轻微起步门槛。

### 6.3 手柄信号处理

默认：

- 一阶低通：`--hand-filter-tau 0.03`；
- moving deadband：`--hand-deadzone 0.0007`；
- 比例：`--scale 1.0`；
- 总位移限制：`--max-translation 0`，`0` 表示关闭。

### 6.4 笛卡尔目标生成

`CartesianTargetLimiter` 提供连续目标轨迹：

- 控制标称频率：`--control-rate-hz 50`；
- 一阶目标时间常数：`--target-time-constant 0.04`；
- 最大末端速度：`--max-ee-speed 0.45 m/s`；
- 最大末端加速度：`--max-ee-accel 2.0 m/s²`；
- 调度暂停后用于轨迹计算的最大 `dt`：`--max-control-dt 0.05 s`。

anti-windup 通过：

```text
--max-command-lead 0.08 m
```

把平滑命令限制在真实 Baxter 末端前方 `8 cm` 内。这不是总工作空间限制，而是局部 command lead 限制。对比文档认为 `8 cm` 可能偏保守，未来可测试 `0.12～0.20 m` 或动态窗口；但在同步 IK 卡顿未解决之前，不应先放宽它，因为可能放大卡顿后的追赶动作。

### 6.5 IK 与分支连续性

当前每次主循环同步调用 `solve_ik_with_backtracking()`：

- 尝试 alpha：`1.0`、`0.5`、`0.25`，默认最多 3 次；
- `--ik-budget-ms 90` 只在一次调用返回后决定是否继续下一次；
- 它不能取消或超时中断已经发出的 ROS service call；
- active Grip 期间只使用完整的上一帧成功 IK 解作为 `SEED_USER`；
- 不回退到 `SEED_CURRENT`，以避免 7DoF 冗余解突然切换 elbow/wrist branch；
- 首个 IK 解相对 Grip lock 关节角默认不能超过 `0.10 rad`；
- 后续连续 IK 解默认不能超过 `0.25 rad`。

对应参数：

```text
--max-first-ik-difference 0.10
--max-ik-difference 0.25
--ik-backtrack-attempts 3
--ik-budget-ms 90
```

### 6.6 关节命令

当前已实现 `ContinuousJointCommandPublisher`：

- 独立线程默认 `50 Hz`；
- 不断重发最近一次接受的完整关节命令；
- 调用 `set_joint_positions(command, raw=False)`；
- 不使用 `RAW_POSITION_MODE`；
- 发布线程异常会回报主循环并触发安全停止。

关节 command lead 默认：

```text
--max-joint-step 0.12 rad
```

实际含义不是“每帧固定步长”，而是每次更新时命令相对实测关节角最多领先 `0.12 rad`。

### 6.7 当前姿态策略

当前控制是 translation-only。Grip 时锁定工具 orientation，整个 Grip session 默认保持该姿态，以防止把每帧测得的微小姿态漂移继续喂回 IK。

提交 `e4c52ee` 曾加入实验性软姿态松弛：连续若干次 strict-orientation IK miss 后，最多向当前实测姿态松弛若干度。其参数是：

```text
--orientation-relax-after
--max-orientation-relax-deg
```

当前未提交修复把 `--orientation-relax-after` 默认改为 `0`，因此默认禁用。原因：

- 真机日志中绝大多数时间 `relax=0.0deg`，没有证据证明它改善手感；
- 第一次真正进入松弛路径时，旧实现因不可变 orientation 崩溃；
- 崩溃虽已在本地修复，但新实现只经过自动化测试，没有再次真机验证；
- 同步 IK 阻塞是比姿态可达性更优先的问题。

## 7. V2、V3、V4 的演进与取舍

用户此前提供过：

- `baxter_precision_teleop_v2.py`；
- `baxter_precision_teleop_v3.py`；
- `baxter_precision_teleop_v4.py`；
- `控制板本V2-V3-V4对比.md`。

这些附件原先位于用户 Windows 的 Desktop/Downloads，不应假设另一对话一定能读取；关键结论已整理如下。

### 7.1 V2：跟手优先

保留了比较理想的手柄侧参数：

```text
scale = 1
max_translation = 0
hand_filter_tau = 0.03 s
hand_deadzone = 0.7 mm
max_ee_speed = 0.45 m/s
max_ee_accel = 2.0 m/s²
speed_ratio = 0.8
```

优点：直接、自由、大动作跟手感最好。

缺点：缺少闭环 command lead 限制，软件目标可远远甩开真实机器人。历史诊断出现过：

```text
tracking_error ≈ 0.533 m
joint_error ≈ 1.497 rad
```

这会把 IK 推向远离真实机器人的区域，产生慢 IK、大量回退和后半程失控式掉队。

### 7.2 V3：执行闭环优先

核心增加：

- 独立 `50 Hz` 关节命令持续发布；
- `max-command-lead = 0.08 m`；
- `max-joint-step = 0.12 rad`；
- IK 回退次数减少并增加约 `90 ms` budget。

优点：慢 IK 不会让 Baxter SDK 的命令流完全断掉，目标也不再无限甩开机器人。

缺点：错误 IK 一旦接受，就会被独立发布线程稳定地重复执行；同时常规 IK branch threshold 曾放宽到 `0.40 rad`，首帧仍可能由冗余 IK 选择另一套构型，并允许 `SEED_CURRENT` fallback。真机出现过“只按 Grip 还没动手，机器人就扭动/反弯”的现象。

### 7.3 V4：IK 构型稳定优先

核心增加：

- Grip 后手柄移动超过 `3 mm` 才开始 IK；
- Grip lock 时当前关节角直接作为首个 `SEED_USER`；
- 整个 Grip session 锁定工具 orientation；
- 删除 active session 中的 `SEED_CURRENT` fallback；
- 首个 IK 阈值 `0.10 rad`，常规连续阈值 `0.25 rad`。

优点：明显降低 Grip 自发运动、翻肘、翻腕和姿态漂移风险。

代价：固定 orientation 降低工作空间边缘的可达性；完全禁止 branch recovery 降低恢复能力；`3 mm` gate 和较保守阈值降低部分跟手感。

### 7.4 不能把版本号理解为单调升级

准确的理解是：

```text
V2：追随自由度最好，但闭环不足
V3：命令连续和 anti-windup 更好，但 IK branch 管理不足
V4：IK branch 更稳定，但可达性和响应性更保守
```

合理后续方向不是简单等于 V4，也不是直接回滚 V2，而是保留：

- V2 的手柄侧低延迟与连续目标生成；
- V3 的独立命令发布和 anti-windup；
- V4 的 Grip gate、首帧用户 seed 和严格 branch continuity；
- 再解决“同步 IK 阻塞”和“受控恢复”两个尚未真正解决的问题。

## 8. 最近一次 Baxter 真机测试：必须保留的证据

最近一次用户运行了全幅参数：

```bash
/usr/bin/python3 bridge/baxter_precision_teleop.py \
  --side left \
  --bind-host 127.0.0.1 \
  --port 15000 \
  --scale 1.0 \
  --max-translation 0 \
  --speed-ratio 0.80 \
  --max-ee-speed 0.45 \
  --max-ee-accel 2.0 \
  --max-command-lead 0.08 \
  --max-joint-step 0.12 \
  --max-ik-difference 0.25 \
  --max-first-ik-difference 0.10 \
  --motion-start-threshold 0.003 \
  --orientation-relax-after 3 \
  --max-orientation-relax-deg 3 \
  --csv-log logs/left_v5_fullscale.csv \
  --execute
```

用户明确反馈：效果比之前更差。

### 8.1 日志中的正常部分

- UDP packet age 多数约 `0.6～11 ms`，说明本次测试中的 PICO→UDP 输入并非主要瓶颈；
- 小位移时 `tracking_error` 可低至 `0.001～0.02 m`；
- 大多数正常接受的连续 IK `diff` 在约 `0.00～0.10 rad`；
- `SEED_USER` 连续性检查确实拦截了多个约 `0.7 rad` 的构型跳变。

### 8.2 明确异常

单次同步 IK 延迟出现：

```text
ik=201.1 ms
ik=623.9 ms
ik=1771.2 ms
ik=1068.3 ms
```

这远大于 `50 Hz` 循环的 `20 ms` 周期。`--ik-budget-ms 90` 没有阻止这些延迟，因为它不能取消正在进行的 ROS service call。

大幅移动时还出现：

```text
tracking_error = 0.080 m
desired_lag = 0.360 m
command_speed = 0.450 m/s
sat = -LV- / --V- / --VA
```

解释：command lead 被卡在 `8 cm`，手柄目标仍在远处，速度/加速度限幅持续工作；如果 IK 中间卡住，返回后目标生成器又继续追赶，主观感受很容易变成卡顿、拖拽和突然追赶。

分支拒绝例子：

```text
[IK SKIP] IK jump 0.6956 rad exceeds 0.2500
[IK SKIP] IK jump 0.7028 rad exceeds 0.2500
[IK SKIP] IK jump 0.4233 rad exceeds 0.1000
```

这些拒绝本身是安全机制正确工作，但也说明 Baxter IK 在某些目标上给出了另一套构型。不能简单放宽阈值，否则可能重新引入翻肘/翻腕。

### 8.3 已复现崩溃

日志最后是：

```text
Traceback (most recent call last):
  ...
  target_endpoint_orientation = relax_orientation(...)
  ...
  result.x, result.y, result.z, result.w = values
AttributeError: can't set attribute
```

根因：`endpoint["orientation"]` 在 Baxter SDK 中是不可变的 tuple-like 对象；`copy.copy()` 仍返回相同不可变类型，不能写 `.x/.y/.z/.w`。

当前未提交 diff 已改为返回 `QuaternionValue`，并默认禁用姿态松弛。自动化测试覆盖了数学边界和不修改 reference，但这不等于 Baxter 真机修复已验收。

## 9. 控制侧当前最高优先级设计任务

### P0：把 IK 从固定频率控制状态机中真正解耦

目标不是简单“把 `solve_ik()` 丢进线程”。必须满足以下语义：

1. 主控制循环持续以固定频率处理最新 XR、Grip、机器人状态和目标轨迹，不因 IK service call 阻塞；
2. IK worker 同一时间最多只有一个在途请求，避免并发调用 Baxter IK service；
3. worker 忙时只保留最新待求目标，覆盖旧 pending target，不建立 FIFO backlog；
4. 每个请求携带 `session_id`、递增 `request_id`、目标生成时间和 seed snapshot；
5. Grip release、安全停止、序列倒退或连接丢失时增加/失效 `session_id`；旧 session 的 IK 结果即使稍后返回也必须丢弃；
6. 返回结果必须检查 result age、目标版本、joint set、first/consecutive branch difference 和当前安全状态；
7. 只允许主控制线程决定是否接受结果、更新 `last_successful_joint_command` 和发布新 command；worker 不直接驱动机器人；
8. 日志分别记录 `ik_service_ms`、`ik_result_age_ms`、`request_replaced_count`、`stale_result_count`、`worker_busy`；
9. 如果单个 service call 无限挂起，仅用一个 daemon thread 仍无法恢复。需要评估 ROS service persistent/reconnect、独立 subprocess 或可重建 worker 的边界；不要通过无限创建线程绕过挂起，否则会积累不可控请求；
10. 异步化后要重新定义 backtracking：一个请求内部做 3 次同步回退仍可能长期占据 worker。更合理的是一次只求一个 alpha，并由状态机按最新目标决定是否继续下一 alpha。

### P1：重新设计目标追赶语义

异步 IK 后要避免对旧手柄位置求解：

- pending target 始终是最新 hand intent 经当前 anti-windup 后的目标；
- worker 返回期间，主循环可以更新 desired target，但不能在没有新 IK 的情况下不断推进 accepted Cartesian state；
- 明确区分 `desired_position`、`limited_position`、`submitted_ik_position`、`accepted_ik_position` 和 measured endpoint；
- IK miss/branch reject 后应冻结或回退“已提交目标”，而不是破坏手柄 reference；
- 大动作时可考虑基于 tracking error 动态调整 command lead，但必须在异步架构稳定后做消融测试。

### P2：受控姿态/branch recovery

在 P0/P1 完成前，保持 Grip-lock orientation 和 `SEED_USER`，不要默认打开松弛。

未来若恢复软约束，应满足：

- 只在连续严格姿态不可达且 IK worker 未积压时触发；
- 松弛只针对单次目标，不改变 Grip orientation reference；
- 默认最大 `±3°`，实测不足再考虑 `±5°`；
- 有独立日志字段；
- branch recovery 不能等于无条件 `SEED_CURRENT`；候选解必须按与当前关节的代价、关节极限余量和 elbow/wrist 偏好排序；
- 任何新 branch 都应需要更严格的速度/位移过渡或显式重新 clutch，而不是直接接受。

### P3：参数消融，而不是同时改全部参数

后续应保持同一主代码，用开关逐项测试：

- command lead on/off/不同数值；
- strict orientation 与小角度 relaxation；
- first IK threshold；
- branch recovery；
- motion gate；
- IK worker 与同步基线。

每次只改一个主要变量，使用相同动作轨迹和 CSV 指标比较。主观手感必须与日志同时记录。

## 10. 控制侧安全边界

不可退让的规则：

- 没有 `--execute` 时拒绝向机器人发命令；
- 程序不自动 enable Baxter；
- 只使用普通 `POSITION_MODE`，不使用 `RAW_POSITION_MODE`；
- Grip 是必须持续按住的 dead-man switch；
- 输入失效、安全停止或 Grip release 时保持当前实测关节位置；
- 安全停止后必须先完整松开 Grip 才能重新 arm；
- 不允许旧 session 的异步 IK 结果在重新 Grip 后生效；
- 第一次真机测试必须低速、小比例、有限总位移、清空工作区，并准备松 Grip/急停；
- 视频首次联合测试应先在 Baxter 禁止运动或 `--execute` 关闭时完成资源与故障隔离验证。

## 11. 控制侧自动化测试现状

命令：

```bash
/usr/bin/python3 -m unittest discover -s bridge/tests -v
```

2026-08-31 在当前 Windows 工作区执行结果：

```text
Ran 10 tests
OK
```

覆盖：

- 四元数最短距离；
- 姿态松弛限幅且不修改 reference；
- Cartesian limiter 的 enforce/backtracking；
- joint command lead；
- joint set mismatch；
- IK backtracking 保持 `SEED_USER`；
- elapsed budget 阻止开始下一次 IK；
- 独立关节发布线程正常与失败路径。

没有覆盖：

- ROS service call 长时间阻塞；
- 异步 IK latest-only/session invalidation（尚未实现）；
- 真实 Baxter 动力学、控制器内部插值和关节极限；
- 真实 PICO 手柄噪声；
- 视频同时运行时的资源争用。

因此 `10 tests OK` 只是无硬件自动化测试，不是真机验收。

## 12. 视频实现现状

入口：`vision/d455_rgb_sender.py`

### 12.1 默认媒体配置

```text
输入：1280×720 @ 30 FPS
SBS 输出：2560×720
码率：6 Mbps
GOP：15 帧
VBV：50 ms
H.264：Baseline、Annex-B、完整 AU、无 B 帧
x264：zerolatency、ultrafast、1 reference frame
```

GStreamer 各关键阶段使用最多一个 buffer 的 downstream-leaky queue，appsink 也只保留最新 buffer，目的是丢旧帧而不是积累历史画面。

### 12.2 TCP 积压保护

默认：

```text
send timeout：250 ms
慢写上限：250 ms
SO_SNDBUF 请求值：16 KiB
TCP_NOTSENT_LOWAT 请求值：4 KiB
DSCP：AF41
```

保护行为：

- AU 发送超时或部分写后不再复用该连接，直接重连；
- PTS 断层时丢弃依赖帧直到下一个 IDR；
- 新连接和 gap 后主动请求 IDR；
- 固定 GOP 是 Force Key Unit 不受支持时的后备；
- 每 5 秒打印 AU/s、`pipeline-age`、当前/最大发送耗时、`slow-sends`、`reconnects`、`gaps`。

### 12.3 已知真机结论

已经确认过：

- 真实 PICO 能显示 D455 视频；
- 优化前主观延迟约 `2～3 s` 且不稳定。

尚未确认：

- 提交 `03794a3` 后真实 D455→PICO 的端到端延迟；
- 降到 6 Mbps、GOP 15、VBV 50 ms 后的中位数/P95/最大延迟；
- 与 Baxter 控制同时运行时是否产生 CPU 或网络干扰；
- PICO 侧是否仍存在解码/渲染队列积压。

### 12.4 激进降载候选

如果优化后默认配置仍明显延迟，可测试：

```text
输入：640×360 @ 30 FPS
SBS 输出：1280×360
码率：3 Mbps
GOP：10
VBV：30 ms
send timeout：150 ms
慢写上限：120 ms
SO_SNDBUF：8 KiB
TCP_NOTSENT_LOWAT：2 KiB
```

完整命令见 `vision/LATENCY_TUNING.md`。先确认 D455 节点实际支持所选分辨率，不要随机猜 `/dev/videoN`。

## 13. 视频侧自动化测试现状

命令：

```bash
/usr/bin/python3 -m unittest discover -s vision/tests -v
```

2026-08-31 在当前 Windows 工作区执行结果：

```text
Ran 49 tests
OK
```

覆盖协议、listener 状态机、SBS 管线描述、keyframe/gap、慢接收方 backpressure、连接恢复、CLI 默认值等。

Linux 上还应运行：

```bash
bash scripts/test_vision_local.sh
```

它会使用真实 GStreamer 编码器和合成图像验证 2560×720、30 FPS、Baseline、无 B 帧和 SBS 两半一致。

这些仍然不等于 D455/PICO 真机验证。

## 14. 关键文件导航

### 控制

| 文件 | 作用 |
| --- | --- |
| `bridge/pico_udp_sender.py` | 从 `SdkXRInputSource` 读取 PICO 状态并发 UDP；支持 `--mock` |
| `bridge/udp_dry_receiver.py` | 只验证 UDP 协议与新鲜度，不碰 Baxter |
| `bridge/baxter_ik_dryrun_receiver.py` | IK dry-run，验证映射/可达性但不执行机器人动作 |
| `bridge/baxter_live_microtest.py` | 早期小位移真机基线 |
| `bridge/baxter_precision_teleop.py` | 当前单臂实机控制入口 |
| `bridge/tests/test_baxter_precision_teleop.py` | 当前控制侧无硬件自动化测试 |
| `bridge/README.md` | 控制桥说明，但后续实现异步 IK 后必须同步更新 |

### 视频

| 文件 | 作用 |
| --- | --- |
| `vision/d455_rgb_sender.py` | 视频 CLI 入口 |
| `vision/gst_pipeline.py` | D455/test source、SBS、x264 和 appsink 管线 |
| `vision/stream_sender.py` | AU TCP sender、积压保护、重连、关键帧恢复 |
| `vision/listener.py` | `OPEN_CAMERA` / `CLOSE_CAMERA` 控制 listener |
| `vision/remote_vision_protocol.py` | 控制和视频 framing 协议 |
| `vision/diagnose.py` | D455 节点枚举和 pipeline preflight |
| `vision/mock_pico.py` | 模拟 PICO 控制和视频接收 |
| `vision/README.md` | 依赖与运行方式 |
| `vision/TESTING.md` | 分阶段测试手册 |
| `vision/LATENCY_TUNING.md` | 延迟诊断与激进降载参数 |

### 上游快照

| 目录 | 作用 |
| --- | --- |
| `pico-teleop/` | `SdkXRInputSource`、通用 teleop engine 等上游代码快照 |
| `XR-PC-Service-Python/` | Python XR PC Service 快照 |
| `pico-PC-Service-pybind/` | pybind PC Service 快照 |

除非定位到明确的上游缺陷，不要为了当前 Baxter/video 优化先大规模修改上游快照。

## 15. 建议的联合测试顺序

### 阶段 A：纯自动化

```bash
/usr/bin/python3 -m unittest discover -s bridge/tests -v
/usr/bin/python3 -m unittest discover -s vision/tests -v
bash scripts/test_vision_local.sh
```

### 阶段 B：控制传输但不碰机器人

终端 1：

```bash
/usr/bin/python3 bridge/udp_dry_receiver.py \
  --bind-host 127.0.0.1 \
  --port 15000
```

终端 2：

```bash
/usr/bin/python3 bridge/pico_udp_sender.py \
  --mock \
  --host 127.0.0.1 \
  --port 15000 \
  --rate-hz 50
```

随后用真实 PICO 替换 `--mock`，确认 `seq`、packet age、Grip 和坐标轴。

### 阶段 C：真实 D455 到模拟 PICO

严格按 `vision/TESTING.md`：

1. `vision.diagnose --list-only`；
2. 确认 by-id RGB 节点；
3. preflight；
4. `listen` 模式发送；
5. `vision.mock_pico` 收取并用 `ffprobe`/`ffmpeg` 检查。

### 阶段 D：真实视频，Baxter 禁止运动

运行真实 PICO tracking 与 Remote Vision，但不要启动实机控制入口（该入口没有
`--execute` 会直接拒绝运行）；只运行 UDP receiver 或 IK dry-run。记录：

- 视频 AU/s、pipeline age、send time、reconnect/gap；
- XR packet age、丢包/序列；
- `pidstat` CPU/内存；
- `ping` 与 `ss -tinp`；
- PICO 端显示延迟。

### 阶段 E：低速、小范围 Baxter 真机

异步 IK P0 完成并通过无硬件测试后，先使用：

```text
scale = 0.25
max_translation = 0.05 m
speed_ratio = 0.15
max_ee_speed = 0.10 m/s
max_ee_accel = 0.50 m/s²
max_command_lead = 0.04 m
max_joint_step = 0.06 rad
orientation_relax_after = 0
```

每个轴分别做小位移方向测试，再做圆/往返动作。必须准备松 Grip 和急停。

### 阶段 F：视频与控制联合真机

只有视频单链和低速控制单链都稳定后才联合。至少记录统一时间戳的：

- `packet_age_ms`；
- control loop jitter；
- IK service duration/result age；
- `tracking_error_m`、`joint_tracking_error_rad`；
- command lead/speed/accel saturation；
- 视频 AU/s、pipeline age、send duration、gap/reconnect；
- CPU、内存、Wi-Fi RTT/RSSI、TCP send queue；
- 端到端视频延迟的 30 个样本中位数/P95/最大值；
- 操作者对卡顿、拖拽、突跳、精度的主观记录。

## 16. 联合优化时需要新增的可观测性

建议两个进程都支持统一的 wall-clock ISO 时间和 monotonic 时间，并写入同一个测试 run 目录，例如：

```text
logs/2026-08-31_run-01/
  control.csv
  control.log
  video.log
  system.csv
  metadata.md
```

控制侧建议新增：

- `control_loop_dt_ms` 与最大 jitter；
- `ik_request_id`、`ik_session_id`；
- `ik_service_ms`；
- `ik_result_age_ms`；
- `ik_worker_busy`；
- `ik_pending_replaced`；
- `ik_stale_result_dropped`；
- `submitted_target_xyz` 与 `accepted_target_xyz`；
- Grip lock/unlock/safety stop 事件行。

视频侧已有大部分运行日志，但可以补充可机器解析的 CSV/JSONL：

- `pipeline_age_ms`；
- `send_ms` 与 max；
- AU/s；
- `slow_sends`、`reconnects`、`gaps`；
- 当前连接状态和 PICO peer；
- encoder bitrate/resolution/profile。

## 17. 不要重复的错误

1. 不要把“增加了更多限制或线程”直接称为优化；必须由真机指标和手感证明。
2. 不要认为 `ik_budget_ms` 能终止一次已经阻塞的 ROS service call。
3. 不要在同步 IK 卡顿未解决时放宽 command lead 来掩盖问题。
4. 不要因为 branch reject 影响手感就直接放宽到接受 `0.7 rad` 跳变。
5. 不要无条件恢复 `SEED_CURRENT`；它可能重新引入另一套 elbow/wrist 构型。
6. 不要把自动化测试或 synthetic GStreamer 测试写成 PICO、D455 或 Baxter 真机验证。
7. 不要用 `appsink drop=true` 推断 TCP/PICO 端不会积压；已经交给 TCP 的旧字节无法被 appsink 撤回。
8. 不要随机切换 `/dev/videoN`；使用稳定 by-id 路径并先看格式。
9. 不要让视频异常传播到控制进程，反之亦然。
10. 不要忽略当前未提交的 quaternion 修复，也不要在未审查 diff 的情况下覆盖它。

## 18. 推荐的下一次实现步骤

1. 先为异步 IK 设计纯 Python 状态机和 fake slow service 测试；测试应覆盖 1.7 秒旧结果、Grip release、重新 Grip、新请求覆盖旧 pending、branch reject 和 worker error。
2. 实现 single-flight latest-only IK worker；主循环保持固定频率，不直接调用 ROS IK service。
3. 增加 request/session/result age 诊断字段。
4. 更新 `bridge/README.md`，明确异步语义和限制。
5. 跑控制侧自动化测试；在 Linux/Baxter 环境跑 dry-run，但不要立即全幅执行。
6. 使用保守参数完成小位移真机验证，并把同步旧版作为对照组。
7. 单独复测视频优化后的真机延迟，保存默认配置和激进降载配置两组数据。
8. 最后进行视频+tracking+IK dry-run 联合资源测试，再进入视频+低速 Baxter 联合真机。

## 19. 可直接粘贴给新 Codex 的接手提示词

```text
请先完整阅读仓库根目录的 AGENTS.md 和 PICO_VIDEO_CONTROL_HANDOFF.md，随后检查
git status、当前分支、bridge/baxter_precision_teleop.py 的未提交 diff，以及
bridge/README.md、vision/README.md、vision/TESTING.md、vision/LATENCY_TUNING.md。

我要在同一个对话中联合优化 PICO→Baxter 控制流和 D455→PICO 视频流，但两条链路
仍应保持独立进程和故障隔离。当前控制侧 P0 是同步 ROS IK service call 曾阻塞
623.9～1771.2 ms；请先设计并实现 single-flight、latest-only、session-safe 的异步
IK worker，确保 Grip release 或重新 Grip 后绝不接受旧结果。不要先放宽 IK branch
阈值，不要默认启用 SEED_CURRENT，不要把 ik_budget_ms 当作单次 service timeout。

当前工作区还有未提交的 QuaternionValue 崩溃修复和默认关闭 orientation relaxation
的修改，必须保留并审查。每次实现后运行 bridge/tests，并明确自动化测试不等于
Baxter 真机验证。

视频侧提交 03794a3 已实现 6 Mbps/GOP15/VBV50ms、单 buffer 泄漏队列、小 TCP
缓冲、send deadline、gap 后等待 IDR 和重连保护；49 项自动化测试通过，但优化后
真实 D455/PICO 延迟仍未复测。请按 vision/TESTING.md 和 LATENCY_TUNING.md 组织
默认/激进配置真机对比，并设计与控制侧统一时间轴的联合日志和资源监控。

开始前先汇报你从代码和文档确认的真实当前状态，再提出最小、安全、可验证的修改；
不要把代码复杂度增加直接表述为优化。
```

## 20. 当前测试结论汇总

| 项目 | 状态 | 能证明什么 | 不能证明什么 |
| --- | --- | --- | --- |
| 控制单元测试 | 10 项通过 | 数学 helper、限幅、同步 IK 回退、关节发布替身逻辑 | Baxter 真机实时性和手感 |
| 视频 Python 测试 | 49 项通过 | 协议、状态机、积压保护、管线描述 | D455/PICO 真机延迟 |
| 合成 GStreamer E2E | 仓库提供脚本，需在 Linux/GI 环境执行 | 编码、framing、SBS 和解码链 | D455 采集与 PICO 显示延迟 |
| 历史真实视频 | 可显示，优化前约 2～3 s 主观延迟 | 链路基本兼容 | 当前低延迟提交已达标 |
| 最近 Baxter 真机 | 明确暴露慢 IK、branch reject 和姿态松弛崩溃 | 当前同步控制架构存在实时性缺陷 | 当前未提交修复已真机通过 |

交接时最重要的态度是：以日志和真机行为为准。当前项目有清晰的安全基础和较完整的测试框架，但控制实时性与视频真机延迟都仍处于需要继续验证和迭代的阶段。
