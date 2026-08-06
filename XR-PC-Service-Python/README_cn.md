# XRoboToolkit-PC-Service-Python

纯 Python、跨平台的 PC 服务，用来替代仅支持 Ubuntu 的 C++/Qt6 版
`XRoboToolkit-PC-Service`。

> English version: see [README.md](README.md)

它在两个方向上都说同样的协议，因此现有的 `xrobotoolkit_sdk` pip 包
（被 `pico-teleop` / `XRoboToolkit-Teleop-Sample-Python` 使用）无需改动即可工作：

- **上行（Upstream）** — TCP 服务监听 `0.0.0.0:63901`，PICO/Unity 帧协议。
- **下行（Downstream）** — gRPC（`EAService`）监听 `127.0.0.1:60061`，与 C++ 服务
  使用相同的 `.proto`、相同的 `ServerFeedback{name="deviceStateJson", ...}` 结构。

v1 范围：头显位姿 + 手柄位姿 / 按键 / 扳机 / 握把 / 摇杆轴。手部追踪、身体追踪、
动作追踪器、机器人相机 → VR 视频流均已预留设计但尚未实现。

## 在整套遥操作中的位置

本服务跑在**机器人 PC** 上，是头显与机器人之间的桥梁：

```
Pico 头显（Unity 客户端）
   │  WiFi · TCP 63901
   ▼
XRoboToolkit-PC-Service-Python   ← 本仓库
   │  gRPC · 127.0.0.1:60061
   ▼
xrobotoolkit_sdk  →  pico-teleop  →  RealMan 机器人
```

建议与 `pico-teleop` 装进**同一个 `xr-robotics` conda 环境**，一个环境即可同时
启动 PC 服务和遥操作脚本。

## 安装

```bash
pip install -e .[dev]
```

`xrbt_service/generated/` 下的 gRPC 桩代码已提交。如需重新生成：

```bash
python -m grpc_tools.protoc -Iproto \
    --python_out=xrbt_service/generated \
    --grpc_python_out=xrbt_service/generated \
    proto/PXREAService.proto
# 把同级 import 改回相对导入：
sed -i 's/^import PXREAService_pb2 as/from . import PXREAService_pb2 as/' \
    xrbt_service/generated/PXREAService_pb2_grpc.py
```

## 运行

```bash
python -m xrbt_service.main
```

默认绑定：

- 上行：`0.0.0.0:63901`
- 下行：`127.0.0.1:60061`

可用环境变量覆盖（`XRBT_UPSTREAM_HOST`、`XRBT_UPSTREAM_PORT`、
`XRBT_DOWNSTREAM_HOST`、`XRBT_DOWNSTREAM_PORT`），或在工作目录放一个
`setting.ini`（键名与 C++ 服务一致）。

### 连接成功标志

服务跑起来后，在 Pico 头显打开 XRoboToolkit App，会弹出服务器连接窗口；用手柄
**扳机键**点选本机器人 PC 的 IP。连接成功后头显主面板显示 `WORKING`。在面板勾选
要同步的位姿并打开 **Send**，位姿数据即开始同步到 PC。

## 测试

```bash
pytest
```

## 架构

```
   Unity TCP 63901  <─┐                ┌─> gRPC 60061 <─>  xrobotoolkit_sdk
                     upstream.py      downstream.py
                     frame_codec.py      │
                           │             │
                           ▼             │
                       device_model.py  <┘
                     (共享状态 + 变更事件 + 事件总线钩子)
```

控制平面（本服务）与媒体平面（`xrbt_service/media/`，v1 预留为空）刻意分离：视频
不应走 gRPC 通道。
