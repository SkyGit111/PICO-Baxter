# D455 RGB 视觉反馈服务

本目录实现面向 PICO Remote Vision 的独立视频链。它不会导入或运行 XR PC
Service、ROS、Baxter SDK 或 Baxter 遥操作桥。一个 GStreamer 管线独占 D455
RGB 设备，将 1280×720 图像横向复制为 2560×720 SBS，再进行低延迟 H.264
编码和 TCP 发送。

## 运行依赖

目标环境为机器人主机上的 x86 Ubuntu。需要安装 GStreamer、Python GI、V4L2
工具、x264/H.264 插件以及 FFmpeg：

```bash
sudo apt install \
  python3-gi gir1.2-gstreamer-1.0 \
  gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav v4l-utils ffmpeg
```

也可以在仓库根目录运行：

```bash
bash scripts/install_vision_dependencies.sh
```

安装脚本会先运行 Python 测试，再短暂启动真实的 GStreamer 编码与解析管线，
但使用的是合成图像，因此不需要连接 D455 或 PICO。

脚本有意默认使用 `/usr/bin/python3`。Ubuntu 通过 APT 安装的 `python3-gi`
通常无法被 Conda、pyenv 或 `/usr/local` 下的自定义 Python 导入。只有在其他
解释器已经能够正常加载 GI 时才应覆盖默认值：

```bash
VISION_PYTHON=/path/to/python bash scripts/test_vision_local.sh
```

## 查找 D455 RGB 节点

不要硬编码 `/dev/video0`。先列出稳定的 by-id 路径和每个节点支持的格式：

```bash
/usr/bin/python3 -m vision.diagnose --list-only

for device in /dev/v4l/by-id/*video-index*; do
  printf '\n===== %s =====\n' "$device"
  v4l2-ctl --device "$device" --list-formats-ext
done
```

选择提供 RGB/彩色图像、并支持 1280×720@30 的节点。优先使用 raw YUY2；如果
只有 MJPEG 支持目标分辨率和帧率，则使用 MJPEG。不要同时运行 RealSense Viewer、
其他 V4L2 程序或第二个视频服务来打开同一个 D455 节点。

后续示例统一使用环境变量保存实际路径：

```bash
D455_DEVICE='/dev/v4l/by-id/替换为实际的D455-RGB节点'
```

## 编码和延迟策略

默认参数如下：

- 输入：1280×720、30 FPS；
- 输出：2560×720 SBS；
- 码率：10 Mbps；
- H.264 Baseline、Annex-B、按完整 AU 输出；
- `zerolatency`、`ultrafast`、`bframes=0`；
- 1 个参考帧，无前向预测，100 ms VBV；
- 关键帧间隔 30 帧；
- 分支队列和 appsink 最多保留 1 个 buffer，旧帧及时丢弃。

发送端会检测丢帧造成的时间戳断层，并暂停发送依赖帧，直到下一个 IDR。TCP
写入设置了超时；发生超时或部分发送后会丢弃当前连接并重新建立，不会继续复用
已经失去帧边界的字节流。

每个 H.264 AU 的 TCP 格式为：

```text
[4 字节无符号大端长度][完整 Annex-B H.264 AU]
```

## 直接连接 PICO

直接模式连接 PICO Remote Vision 的 TCP 视频监听端口，默认是 `12345`：

```bash
/usr/bin/python3 -m vision.d455_rgb_sender \
  --mode direct \
  --device "$D455_DEVICE" \
  --input-mode raw \
  --source-format YUY2 \
  --pico-ip 192.168.1.50 \
  --pico-port 12345 \
  --verbose
```

如果 D455 节点只在 MJPEG 下支持目标模式，则去掉 `--source-format`，改用：

```bash
--input-mode mjpeg
```

没有摄像头时可以使用合成图像测试直接模式：

```bash
/usr/bin/python3 -m vision.d455_rgb_sender \
  --test-source \
  --pico-ip 127.0.0.1
```

## PICO 控制的监听模式

监听模式实现 PICO Remote Vision 的 `OPEN_CAMERA` / `CLOSE_CAMERA` 控制流。
进程监听 TCP `0.0.0.0:13579`；收到 `OPEN_CAMERA` 后，向请求中指定的 PICO
IP 和端口建立第二条视频 TCP 连接；收到 `CLOSE_CAMERA` 后关闭视频连接。

D455 管线只在进程启动时打开一次。重复打开、关闭 Remote Vision 不会重复打开
摄像头。

```bash
/usr/bin/python3 -m vision.d455_rgb_sender \
  --mode listen \
  --device "$D455_DEVICE" \
  --input-mode raw \
  --source-format YUY2 \
  --listen-host 0.0.0.0 \
  --listen-port 13579 \
  --verbose
```

在 PICO Remote Vision 中填写机器人主机的局域网 IP 和端口 `13579`。默认情况
下，`OPEN_CAMERA` 声明的视频目标 IP 必须与控制连接的对端 IP 一致，防止局域网
中的其他客户端借助本服务连接无关主机。如果实际 PICO 软件确实声明了不同 IP，
应先保留并检查日志，再显式添加 `--allow-target-ip-mismatch`。

PICO 请求中的宽度、高度、FPS、码率、相机预设和渲染模式会写入日志。当前版本
始终保持配置的 2560×720@30 SBS 输出，不会根据每次请求重复构建摄像头管线。

## 自动化测试

协议、状态机和管线描述测试不需要 GStreamer 或硬件：

```bash
/usr/bin/python3 -m unittest discover -s vision/tests -v
```

完整的合成视频端到端测试会启动真实 GStreamer 编码器和模拟 PICO：

```bash
bash scripts/test_vision_local.sh
```

真实 PICO、D455、CPU 负载、Wi-Fi 和端到端显示延迟必须在机器人主机与实验室
网络中验证。完整步骤参见 [`TESTING.md`](TESTING.md)。
