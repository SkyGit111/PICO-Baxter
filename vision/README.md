# D455 RGB sender

This directory contains the video-feedback plane for PICO Remote Vision. It
does not import or run the XR PC Service, ROS, the Baxter SDK, or the Baxter
teleoperation bridge. One GStreamer pipeline owns the D455 RGB device and
duplicates its 1280x720 image horizontally into a 2560x720 SBS frame.

## Target runtime dependencies

Install the Ubuntu GStreamer runtime, Python GI bindings, V4L2 tools, and the
x264/parser plugins. Package names vary slightly by Ubuntu release; the
typical packages are:

```bash
sudo apt install \
  python3-gi gir1.2-gstreamer-1.0 \
  gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav v4l-utils ffmpeg
```

Or run `bash scripts/install_vision_dependencies.sh` from the repository root.
The installation check briefly runs a synthetic pipeline through the real
GStreamer encoder and parser; it does not require a camera or PICO.

Confirm the stable RGB node before running:

```bash
v4l2-ctl --list-devices
v4l2-ctl --device /dev/v4l/by-id/<D455-RGB-node> --list-formats-ext
```

Do not run another RealSense or V4L2 capture process against the same D455.

## Direct PICO mode

The initial mode connects directly to a PICO Remote Vision TCP listener. Each
Annex-B H.264 access unit is prefixed by a four-byte unsigned big-endian
length.

```bash
python3 -m vision.d455_rgb_sender \
  --device /dev/v4l/by-id/<D455-RGB-node> \
  --pico-ip 192.168.1.50 \
  --pico-port 12345
```

The defaults are 1280x720 at 30 FPS, 2560x720 SBS output, 10 Mbps baseline
H.264, `zerolatency`, `ultrafast`, no B frames, a 30-frame key interval, one
reference frame, no lookahead, and a 100 ms VBV buffer.

Raw V4L2 input is the default. Use `--input-mode mjpeg` when the selected D455
RGB node only provides 1280x720@30 as MJPEG. `--source-format` applies only to
raw input, for example `--source-format YUY2`.

Use a synthetic source for development without a camera:

```bash
python3 -m vision.d455_rgb_sender --test-source --pico-ip 127.0.0.1
```

The pipeline and appsink queues retain at most one buffer and drop older
buffers. The sender detects timestamp gaps caused by such drops and waits for
the next IDR instead of sending P-frames whose references were discarded.
TCP writes have a deadline; a timed-out or partially written stream is
discarded and reconnected rather than reused.

## PICO-controlled listener mode

Listener mode implements the Remote Vision `OPEN_CAMERA` / `CLOSE_CAMERA`
control flow. It binds TCP `0.0.0.0:13579`; an `OPEN_CAMERA` request starts a
video connection to the IP and port requested by the PICO, while
`CLOSE_CAMERA` stops that connection. The D455 pipeline is opened exactly once
when the process starts and remains the sole camera owner across repeated
open/close requests.

```bash
python3 -m vision.d455_rgb_sender \
  --mode listen \
  --device /dev/v4l/by-id/<D455-RGB-node> \
  --listen-host 0.0.0.0 \
  --listen-port 13579
```

In PICO Remote Vision, enter the robot PC's LAN IP and port `13579`, then use
the camera open/close controls. By default, an `OPEN_CAMERA` target IP must
match the PICO control connection's peer IP. This prevents another LAN client
from using the sender to connect to an unrelated host. If the deployed PICO
software legitimately reports a different target IP, inspect the log first
and then explicitly opt in with `--allow-target-ip-mismatch`.

The requested width, height, FPS, bitrate, camera preset, and render mode are
logged. This version deliberately keeps the configured 2560x720@30 SBS output
instead of rebuilding the camera pipeline for every request.

## Local tests

The protocol and pipeline-description tests do not require GStreamer or
hardware:

```bash
python3 -m unittest discover -s vision/tests -v
```

PICO, D455, CPU-load, Wi-Fi, and glass-to-glass latency validation must be
performed on the x86 Ubuntu robot host.

Run `bash scripts/test_vision_local.sh` for the full synthetic GStreamer plus
mock-PICO test. See [`TESTING.md`](TESTING.md) for the complete lab runbook.
