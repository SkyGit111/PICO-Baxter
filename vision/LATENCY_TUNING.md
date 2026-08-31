# D455 到 PICO 视频延迟诊断与调优

## 当前结论

在本轮优化之前，实验室真机已经确认 PICO 能显示 D455 视频，但主观延迟约为
2～3 秒且不稳定。此前 D455 到模拟 PICO 的测试为 1280×720@30 输入、2560×720
SBS 输出，约 30 AU/s、无时间戳断层；这只能证明采集、编码和协议链可运行，不能
证明真实 PICO 的端到端延迟合格。

本轮优化降低了默认码率、GOP 和 VBV，给全部 GStreamer 阶段增加单 buffer 的
下游泄漏队列，并为 TCP 增加小发送缓冲、`TCP_NOTSENT_LOWAT`、发送截止时间、
慢写重连和即时 IDR 请求。相关自动化测试已覆盖慢接收端、连接恢复、时间戳断层
和关键帧恢复，但优化后的真实 D455/PICO 延迟仍必须在实验室重新验证。

## 为什么 TCP 会产生秒级延迟

TCP 保证可靠、有序传输。如果无线链路或 PICO 消费速度短时低于编码输出速度，
旧视频会先进入发送端内核、网络、PICO 内核和应用接收队列；画面可能仍然连续，
但显示的是数秒前的历史。GStreamer 的 `appsink max-buffers=1 drop=true` 只能限制
应用尚未取出的编码帧，不能撤回已经交给 TCP 的字节。

因此本实现采用以下边界：

1. 所有 GStreamer 排队点只保留最新 buffer；
2. 默认码率从 10 Mbps 降到 6 Mbps，给 Wi-Fi 抖动留出余量；
3. 默认 GOP 从 30 帧降到 15 帧，VBV 从 100 ms 降到 50 ms；
4. 请求 16 KiB socket 发送缓冲和 4 KiB `TCP_NOTSENT_LOWAT`；
5. 单个完整 AU 写入超过 250 ms，或 socket 写入超时，就关闭该视频连接；
6. 新连接和时间戳断层主动请求 IDR，从新关键帧恢复。

发送端无法在没有 PICO 回执的协议中直接测量 PICO 应用内部的排队深度。关闭并
重建 TCP 连接是发现发送背压后丢弃未完成旧字节流的安全方法，但 PICO 是否能在
控制连接保持期间稳定接受重复视频连接仍属于真机测试项。

## 第一轮：完整分辨率的默认优化配置

机器人主机使用已经确认的 RGB 节点运行：

```bash
cd ~/pico_baxter_ws/PICO-Baxter

/usr/bin/python3 -m vision.d455_rgb_sender \
  --mode listen \
  --device /dev/v4l/by-path/pci-0000:00:14.0-usb-0:4:1.3-video-index0 \
  --input-mode raw \
  --source-format YUY2 \
  --listen-host 0.0.0.0 \
  --listen-port 13579 \
  --verbose
```

这些默认值保持 D455 1280×720@30 输入和 2560×720 SBS 输出：

```text
bitrate=6.0 Mbps, GOP=15, VBV=50 ms
send-timeout=250 ms, max-send=250 ms
send-buffer=16 KiB, TCP_NOTSENT_LOWAT=4 KiB, DSCP=34
```

在 PICO 中 Open 后至少运行 5 分钟，同时保存完整终端日志。近距离快速挥手或在
D455 前运行手机秒表，用可同时观察真实场景和 PICO 画面的方式测量延迟。

## 如何判读日志

每 5 秒会出现类似日志：

```text
video active: 30.0 sent AU/s, total=900, pipeline-age=42.0ms,
send=1.8ms max=18.4ms, pre-IDR drops=0, gaps=0, slow-sends=0, reconnects=0
```

- `pipeline-age` 持续低于约 100 ms：主机采集和编码通常没有明显积压。
- `pipeline-age` 经常高于 250 ms：主机 CPU、USB、颜色转换或 x264 编码路径过载。
- `send`/`max` 接近 250 ms，或 `slow-sends`、`reconnects` 增加：TCP 接收速度不足，
  优先检查 Wi-Fi、PICO 性能和输出码率。
- `gaps` 增加：编码端为了保持最新画面丢过旧 AU；随后应从新 IDR 恢复。
- `pipeline-age` 和发送耗时都低，但 PICO 仍有秒级延迟：根据现有可观测数据推断，
  积压更可能位于 PICO 的 TCP 接收、H.264 解码或渲染路径。应立即测试下一节的
  降载配置，并保留两组日志对比。

上述数值是诊断起点，不是真机验收结论。最终阈值要以实验室测量为准。

## 第二轮：激进降载配置

该配置将 D455 输入降为 640×360@30，并复制成 1280×360 SBS，同时把码率降到
3 Mbps、GOP 降到 10 帧。先确认 D455 节点支持该模式：

```bash
/usr/bin/python3 -m vision.d455_rgb_sender \
  --preflight-only \
  --device /dev/v4l/by-path/pci-0000:00:14.0-usb-0:4:1.3-video-index0 \
  --input-mode raw \
  --source-format YUY2 \
  --width 640 \
  --height 360 \
  --bitrate-mbps 3 \
  --key-interval 10 \
  --vbv-buffer-ms 30
```

预检成功后运行：

```bash
/usr/bin/python3 -m vision.d455_rgb_sender \
  --mode listen \
  --device /dev/v4l/by-path/pci-0000:00:14.0-usb-0:4:1.3-video-index0 \
  --input-mode raw \
  --source-format YUY2 \
  --width 640 \
  --height 360 \
  --fps 30 \
  --bitrate-mbps 3 \
  --key-interval 10 \
  --vbv-buffer-ms 30 \
  --send-timeout 0.15 \
  --max-send-ms 120 \
  --send-buffer-kib 8 \
  --tcp-notsent-lowat-kib 2 \
  --listen-host 0.0.0.0 \
  --listen-port 13579 \
  --verbose
```

如果 640×360 不受该 D455 节点支持，先用 `v4l2-ctl --list-formats-ext` 选择实际
支持的较低 RGB 模式；不要随机切换 `/dev/videoN`。PICO 请求日志可能仍显示
2560×720，这是控制请求中的期望值；发送端会明确警告实际固定输出为 1280×360。
是否正确显示要由真机确认。

如果日志只有偶发的单个大 IDR 写入略超 250 ms，而画面延迟已明显改善，可以将
`--max-send-ms` 谨慎提高到 `400`。提高该值也提高了允许历史画面停留的上限，不能
用它掩盖持续的 `slow-sends`。

## 网络与资源检查

测试期间另开终端记录：

```bash
PICO_IP=<PICO-IP>
ping -i 0.2 "$PICO_IP"
iw dev
ss -tinp
pidstat -p <视频进程PID> 1
```

重点检查 Wi-Fi 是否为 5 GHz、信号是否稳定、往返时间是否出现数百毫秒尖峰、
视频 TCP 的发送队列是否持续增长，以及视频进程是否长期占满 CPU。机器人控制链
同时运行时，还要记录 XR 数据包年龄和控制超时，确认视频降载没有以控制质量为
代价。

## 测试记录模板

每个配置至少记录以下信息：

```text
日期/提交：
PICO Remote Vision 版本：
机器人主机 CPU/GPU：
Wi-Fi 频段、RSSI、平均/P95 ping：
输入和 SBS 输出分辨率：
码率/GOP/VBV/socket 参数：
30 次延迟测量的中位数/P95/最大值：
平均 AU/s：
pipeline-age 常态/最大值：
send 常态/最大值：
slow-sends/reconnects/gaps：
是否正确 Close/Open：
是否影响 XR/Baxter 控制：
```

## 参考资料

- [GStreamer x264enc 参数说明](https://gstreamer.freedesktop.org/documentation/x264/)
- [GStreamer Force Key Unit 事件](https://gstreamer.freedesktop.org/documentation/video/video-event.html)
- [Linux TCP_NOTSENT_LOWAT 说明](https://kernel.org/doc/html/v5.8/networking/ip-sysctl.html)
- [Python socket 常量与超时接口](https://docs.python.org/3/library/socket.html)
