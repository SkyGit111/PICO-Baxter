# PICO-Baxter

本仓库用于实现基于 PICO 4 Ultra 的 Baxter 机器人遥操作与视觉反馈。

## 仓库结构

- `bridge/`：Baxter 端控制、测试和 XR 桥接脚本。
- `pico-teleop/`：上游遥操作框架的源码快照。
- `XR-PC-Service-Python/`：Python 版 XR PC Service 的源码快照。
- `pico-PC-Service-pybind/`：pybind 版 PC Service 的源码快照。
- `vision/`：独立的 D455 RGB 视觉反馈服务。
- `config/`：共享配置。
- `scripts/`：视觉服务的依赖安装与自动化测试脚本。

## 机器人控制链

```text
PICO 4 Ultra
→ XR PC Service
→ SdkXRInputSource
→ bridge/pico_udp_sender.py
→ UDP latest-only 输入
→ bridge/baxter_precision_teleop.py
→ single-flight 异步 Baxter IK
→ Baxter SDK
```

当前 Baxter 入口仍是单臂平移控制；双臂、手柄旋转和夹爪尚未接入实机控制链。
`pico-teleop` 内含通用 `TeleopEngine` 抽象，但当前 Baxter 运行入口没有经过它。

## 视觉反馈链

```text
Intel RealSense D455 RGB
→ 独立的 GStreamer 低延迟 H.264 发送服务
→ PICO Remote Vision
```

机器人控制链与视频链运行在不同进程中。视频服务不依赖 ROS，也不会导入或运行
XR PC Service、Baxter SDK 或 Baxter 遥操作桥。

视频服务支持两种网络模式：

- 直接连接 PICO 的 TCP `12345` 端口；
- 监听 TCP `0.0.0.0:13579`，处理 PICO Remote Vision 的
  `OPEN_CAMERA` / `CLOSE_CAMERA` 请求。

D455 初始输入为 1280×720、30 FPS，发送前将同一 RGB 图像横向复制为
2560×720 SBS。依赖、运行参数和测试流程参见
[`vision/README.md`](vision/README.md) 与
[`vision/TESTING.md`](vision/TESTING.md)。延迟日志判读和分级降载配置参见
[`vision/LATENCY_TUNING.md`](vision/LATENCY_TUNING.md)。

导入的上游版本记录参见 [`UPSTREAM_VERSIONS.md`](UPSTREAM_VERSIONS.md)。

控制流与视频流的当前状态、真机证据、已知问题和后续联合优化计划参见
[`PICO_VIDEO_CONTROL_HANDOFF.md`](PICO_VIDEO_CONTROL_HANDOFF.md)。
