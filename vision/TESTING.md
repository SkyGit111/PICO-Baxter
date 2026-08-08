# 视频链验证手册

请在 x86 Ubuntu 机器人主机上按顺序完成各阶段。摄像头、网络、重连和故障注入
测试期间，应保持 Baxter 运动功能禁用。当前没有任何测试结果可以替代 D455 与
PICO 真机验收。

## 1. 更新代码和安装依赖

```bash
cd ~/pico_baxter_ws/PICO-Baxter
git fetch origin
git switch feat/d455-remote-vision
git pull --ff-only
bash scripts/install_vision_dependencies.sh
```

脚本默认使用 `/usr/bin/python3`，会运行 Python 测试并短暂启动完整的合成
GStreamer 编码管线。两部分都成功后才能继续。

## 2. 完整合成视频端到端测试

```bash
bash scripts/test_vision_local.sh
```

该脚本执行以下流程：

1. 使用 `videotestsrc` 启动真实 GStreamer 发送端；
2. 启动模拟 PICO 控制端和视频接收端；
3. 发送 `OPEN_CAMERA`；
4. 接收 120 个带长度前缀的 H.264 AU；
5. 发送 `CLOSE_CAMERA`；
6. 使用 `ffprobe` 检查 2560×720、30 FPS、Baseline、无 B 帧；
7. 解码一帧并检查 SBS 左右两半的像素 MD5 完全相同。

预期最后出现：

```text
Local Remote Vision end-to-end test passed.
```

## 3. 确认 D455 RGB 节点

列出稳定设备路径：

```bash
/usr/bin/python3 -m vision.diagnose --list-only
```

逐一查看所有节点的格式，并保存输出：

```bash
for device in /dev/v4l/by-id/*video-index*; do
  printf '\n===== %s =====\n' "$device"
  v4l2-ctl --device "$device" --list-formats-ext
done | tee /tmp/d455-formats.txt
```

选择满足以下条件的节点：

- 是彩色/RGB 节点，不是 Depth、Infrared 或 Metadata；
- 支持 1280×720、30 FPS；
- 优先选择 raw `YUYV`（GStreamer 参数写作 `YUY2`）；
- 如果 raw 不支持目标模式，则选择 `MJPG`/MJPEG。

确定后设置环境变量，例如：

```bash
D455_DEVICE='/dev/v4l/by-id/替换为实际的D455-RGB节点'
printf '%s\n' "$D455_DEVICE"
```

检查是否有其他进程正在占用设备：

```bash
fuser -v "$D455_DEVICE"
```

如果有 RealSense Viewer、其他 V4L2 程序或旧的视频发送进程，应先正常停止它，
避免同时打开 D455。若提示权限不足，确认当前用户属于 `video` 组：

```bash
groups
sudo usermod -aG video "$USER"
```

修改用户组后需要注销并重新登录。

## 4. 预检真实摄像头管线

预检会短暂打开所选摄像头，协商完整 SBS/H.264 管线，然后释放设备。运行前确保
没有其他程序打开 D455。

如果节点支持 raw YUYV 1280×720@30：

```bash
/usr/bin/python3 -m vision.diagnose \
  --device "$D455_DEVICE" \
  --input-mode raw \
  --source-format YUY2
```

如果只有 MJPEG 支持目标模式：

```bash
/usr/bin/python3 -m vision.diagnose \
  --device "$D455_DEVICE" \
  --input-mode mjpeg
```

预期输出包含 `Preflight OK`。若协商失败，请保留完整错误和
`/tmp/d455-formats.txt`，不要通过随机更换 `/dev/videoN` 猜测设备。

## 5. D455 到模拟 PICO

先验证真实摄像头、编码、TCP 协议和解码，不连接 PICO。

终端 A 启动视频服务。以下为 raw 示例：

```bash
cd ~/pico_baxter_ws/PICO-Baxter
D455_DEVICE='/dev/v4l/by-id/替换为实际的D455-RGB节点'

/usr/bin/python3 -m vision.d455_rgb_sender \
  --mode listen \
  --device "$D455_DEVICE" \
  --input-mode raw \
  --source-format YUY2 \
  --listen-host 127.0.0.1 \
  --listen-port 13579 \
  --verbose
```

如果选择的是 MJPEG 节点，改成 `--input-mode mjpeg`，并删除
`--source-format YUY2`。

终端 B 启动模拟 PICO：

```bash
cd ~/pico_baxter_ws/PICO-Baxter

/usr/bin/python3 -m vision.mock_pico \
  --mode control \
  --control-host 127.0.0.1 \
  --video-listen-host 127.0.0.1 \
  --video-advertise-ip 127.0.0.1 \
  --video-port 0 \
  --frames 300 \
  --output /tmp/d455-sbs.h264 \
  --timeout 20 \
  --verbose
```

检查录制的码流：

```bash
ffprobe -v error -f h264 -show_streams /tmp/d455-sbs.h264
ffmpeg -v error -f h264 -i /tmp/d455-sbs.h264 \
  -frames:v 1 -y /tmp/d455-sbs.png
```

检查重点：

- 模拟 PICO 正好收到 300 个 AU；
- 分辨率是 2560×720；
- `has_b_frames=0`；
- 终端 A 收到并处理 `OPEN_CAMERA` 和 `CLOSE_CAMERA`；
- `/tmp/d455-sbs.png` 左右两半显示同一幅彩色图像。

## 6. 真实 PICO 监听模式

在机器人主机运行：

```bash
cd ~/pico_baxter_ws/PICO-Baxter
D455_DEVICE='/dev/v4l/by-id/替换为实际的D455-RGB节点'

/usr/bin/python3 -m vision.d455_rgb_sender \
  --mode listen \
  --device "$D455_DEVICE" \
  --input-mode raw \
  --source-format YUY2 \
  --listen-host 0.0.0.0 \
  --listen-port 13579 \
  --verbose
```

MJPEG 节点仍需改成 `--input-mode mjpeg` 并删除 `--source-format YUY2`。

确认端口和防火墙：

```bash
ss -ltnp | grep ':13579'
sudo ufw status
```

如果 UFW 已启用但实验室网段尚未放行，应按照实验室网络策略，仅为可信网段添加
TCP `13579` 规则。

在 PICO Remote Vision 中填写机器人主机的局域网 IP 和端口 `13579`，依次测试：

1. Open；
2. 持续观看至少 2 分钟；
3. Close；
4. 再次 Open；
5. 关闭或重启 PICO 应用后再次连接。

保留完整发送端日志，尤其是 `OPEN_CAMERA requested`、AU/s、gap 和重连信息。
如果请求声明的目标 IP 与控制连接对端不同，先保存日志并确认网络结构，再考虑
使用 `--allow-target-ip-mismatch`。

## 7. 直接模式备用测试

仅当 PICO 确实提供 TCP `12345` 普通视频监听端口时使用：

```bash
/usr/bin/python3 -m vision.d455_rgb_sender \
  --mode direct \
  --device "$D455_DEVICE" \
  --input-mode raw \
  --source-format YUY2 \
  --pico-ip <PICO-IP> \
  --pico-port 12345 \
  --verbose
```

## 8. 稳定性与控制链共存测试

1. 视频单独运行 30 分钟；
2. 与 XR tracking、Baxter IK dry-run 同时运行 30 分钟；
3. 在 Baxter 禁止运动时进行 PICO 应用关闭/重开、D455 拔插和视频进程终止测试；
4. 确认视频故障不会导致 XR PC Service 或 Baxter 控制进程退出。

建议记录：

- `pidstat -p <video-pid> 1` 的 CPU 和内存数据；
- 发送 AU/s、gap 数和重连数；
- XR 数据包年龄和控制超时；
- Wi-Fi 信号及丢包；
- 从摄像头到 PICO 显示的端到端延迟。

## 9. 可选的用户级 systemd 服务

```bash
mkdir -p ~/.config/pico-baxter ~/.config/systemd/user
cp vision/systemd/pico-baxter-vision.env.example \
  ~/.config/pico-baxter/vision.env
cp vision/systemd/pico-baxter-vision.service \
  ~/.config/systemd/user/
```

编辑 `~/.config/pico-baxter/vision.env`，填写实际设备路径和输入模式，然后运行：

```bash
systemctl --user daemon-reload
systemctl --user enable --now pico-baxter-vision.service
journalctl --user -u pico-baxter-vision.service -f
```
