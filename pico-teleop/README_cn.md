# XRoboToolkit RealMan 遥操作

面向 RealMan 硬件的 PICO 遥操作运行时（pico-teleop）。

> English version: see [README.md](README.md)

## 运行链路

```text
PICO 头显
  -> XRoboToolkit-PC-Service-Python      （跑在机器人 PC 上）
  -> xrobotoolkit_sdk / XrClient         （来自 Pybind 绑定库）
  -> SdkXRInputSource
  -> TeleopEngine
  -> RobotAdapter
  -> RealManRobotAdapter
  -> RealMan 硬件（手臂 / 夹爪 / 头部舵机 / 底盘 / 升降）
```

本仓库已精简为 RealMan 生产链路。仿真、UR、ARX、Galaxea、Placo、MuJoCo、Meshcat、可视化脚本及对应机器人资源均已从本包中移除。

## 保留内容

- `scripts/hardware/teleop_realman_hardware.py`：RealMan 启动入口。
- `scripts/misc/test_xr_input_source.py`：仅检查 XR 实时输入。
- `xrobotoolkit_teleop/core/`：共享的遥操作类型、XR 输入、引擎与适配器 API。
- `xrobotoolkit_teleop/hardware/realman.py`：RealMan 正/逆运动学、指令下发、夹爪控制、空跑模式。
- `xrobotoolkit_teleop/hardware/interface/realman.py`：RealMan 机械臂底层封装。
- `control/`：RealMan 控制器、RealMan Python SDK 封装、夹爪 TCP/UDP 客户端。

## 安装（xr-robotics conda 环境）

所有依赖打包在名为 `xr-robotics` 的 conda 环境中。安装脚本会自动 clone 并编译
`XRoboToolkit-PC-Service-Pybind`（提供 `xrobotoolkit_sdk`），再装上本包。

```bash
# 1) 创建 conda 环境（按系统 Python 版本创建；已在 Ubuntu 22.04 / 24.04 测试）
bash setup_conda.sh --conda xr-robotics

# 2) 激活环境
conda activate xr-robotics

# 3) 安装：clone + 编译 Pybind，并以可编辑方式安装 xrobotoolkit_teleop
bash setup_conda.sh --install
```

RealMan SDK 动态库仍需在目标平台单独准备好。

## 运行

先单独启动 `XRoboToolkit-PC-Service-Python` 并连接 PICO 头显，再运行下面的脚本。

```bash
conda activate xr-robotics

# 检查 XR 实时输入
python scripts/misc/test_xr_input_source.py

# RealMan 空跑（不下发真实硬件，建议首次先空跑确认映射）
python scripts/hardware/teleop_realman_hardware.py --dry-run

# 连接真实硬件运行
python scripts/hardware/teleop_realman_hardware.py
```

自定义网络配置：

```bash
python scripts/hardware/teleop_realman_hardware.py \
  --left-arm-ip 169.254.128.18 \
  --right-arm-ip 169.254.128.19 \
  --arm-port 8080 \
  --local-ip 169.254.128.20 \
  --scale-factor 2.0 \
  --arm-rate-hz 50
```

## 手柄按键说明

遥操作时 **左手柄控制左臂，右手柄控制右臂**。各子系统在独立线程内以各自频率运行。

### 手臂与夹爪

| 输入 | 动作 |
| --- | --- |
| 左 握把 Grip（按住） | 控制**左臂**末端位姿；首次按下锁定参考点，之后以增量叠加；松开即停止 |
| 右 握把 Grip（按住） | 控制**右臂**末端位姿，逻辑同上 |
| 左 扳机 Trigger | 控制**左夹爪**，模拟量 `0=张开 → 1=闭合` |
| 右 扳机 Trigger | 控制**右夹爪**，模拟量 `0=张开 → 1=闭合` |

### 底盘（移动底座）— 左摇杆

| 输入 | 动作 |
| --- | --- |
| 左摇杆 Y 轴 | 底盘**前进 / 后退**（linear.x） |
| 左摇杆 X 轴 | 底盘**原地转向**（angular.z，摇杆向左 = 左转） |

死区默认 `0.15`，死区内不动；超出后线性映射到最大速度（默认 v ≤ 0.12 m/s，ω ≤ 0.35 rad/s）。

### 升降柱 — 右摇杆 Y 轴

| 输入 | 动作 |
| --- | --- |
| 右摇杆 Y 轴 | 死区内 = 停止；超出死区 = 固定速度（默认 ±50%）匀速上/下，不随摇杆幅度变化 |

可用 `--lift-joystick-invert` 反向，`--lift-speed-pct` 调整速度。

### 头部舵机（云台跟随）

| 输入 | 动作 |
| --- | --- |
| X 键（左手柄，切换） | **开关头部跟随**；开启瞬间将当前头显朝向设为中立位，之后头部舵机跟随头显做 yaw/pitch（带平滑与死区）；再按一次关闭 |
| 头显朝向 | 跟随开启后，转头即驱动机器人头部云台 |

> **重要安全约定：** 只有**按住对应握把**时机械臂才跟随手柄运动。**松开握把**即停止接收位姿指令并保持当前位置——最直接的「松手即停」机制。

## 标准启动顺序

1. 在机器人 PC 启动 `XRoboToolkit-PC-Service-Python`。
2. 在 Pico 头显打开 XRoboToolkit App，点选机器人 PC 的 IP 连接，勾选 Controller 跟踪并打开 **Send**。
3. 运行遥操作脚本（首次先 `--dry-run`）。

## 许可证

MIT，详见 [LICENSE](LICENSE)。
