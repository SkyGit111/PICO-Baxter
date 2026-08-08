# Video validation runbook

Run every phase in order on the x86 Ubuntu robot host. Keep Baxter motion
disabled during camera, network, reconnect, and failure-injection tests.

## 1. Update and install

```bash
cd ~/pico_baxter_ws/PICO-Baxter
git fetch origin
git switch feat/d455-remote-vision
git pull --ff-only
bash scripts/install_vision_dependencies.sh
```

The installer runs the Python suite and briefly starts a complete synthetic
GStreamer encode pipeline. Do not proceed until both pass.

## 2. Full synthetic end-to-end test

```bash
bash scripts/test_vision_local.sh
```

This starts the real GStreamer sender with `videotestsrc`, starts the mock
PICO control/video client, sends `OPEN_CAMERA`, receives 120 framed H.264
access units, sends `CLOSE_CAMERA`, checks 2560x720 baseline/30 FPS/no B
frames with `ffprobe`, decodes one frame, and verifies that both SBS halves
have identical pixel MD5 values.

## 3. Find the D455 RGB node

```bash
/usr/bin/python3 -m vision.diagnose --list-only
v4l2-ctl --device /dev/v4l/by-id/<candidate> --list-formats-ext
```

Select the stable color/RGB node, not a depth or metadata node. Confirm that
it advertises 1280x720 at 30 FPS. Ensure no other process owns it:

```bash
fuser -v /dev/v4l/by-id/<D455-RGB-node>
```

If camera access is denied, confirm the login user belongs to the `video`
group. After `sudo usermod -aG video "$USER"`, log out and back in before
retrying.

## 4. Preflight the real camera pipeline

Preflight opens the selected camera briefly, negotiates the complete SBS/H.264
pipeline, and then releases it. Stop any other D455 users first.

Prefer raw YUY2 if the node supports 1280x720@30:

```bash
/usr/bin/python3 -m vision.diagnose \
  --device /dev/v4l/by-id/<D455-RGB-node> \
  --input-mode raw \
  --source-format YUY2
```

If only MJPEG provides the required mode:

```bash
/usr/bin/python3 -m vision.diagnose \
  --device /dev/v4l/by-id/<D455-RGB-node> \
  --input-mode mjpeg
```

## 5. D455 to the mock PICO

Terminal A, raw example:

```bash
/usr/bin/python3 -m vision.d455_rgb_sender \
  --mode listen \
  --device /dev/v4l/by-id/<D455-RGB-node> \
  --input-mode raw \
  --source-format YUY2 \
  --listen-host 127.0.0.1 \
  --listen-port 13579 \
  --verbose
```

Terminal B:

```bash
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

Inspect the captured stream:

```bash
ffprobe -v error -f h264 -show_streams /tmp/d455-sbs.h264
ffmpeg -v error -f h264 -i /tmp/d455-sbs.h264 -frames:v 1 -y /tmp/d455-sbs.png
```

## 6. Real PICO listener mode

```bash
/usr/bin/python3 -m vision.d455_rgb_sender \
  --mode listen \
  --device /dev/v4l/by-id/<D455-RGB-node> \
  --input-mode raw \
  --source-format YUY2 \
  --listen-host 0.0.0.0 \
  --listen-port 13579 \
  --verbose
```

In PICO Remote Vision, enter the robot PC LAN IP and port 13579. Exercise
Open, Close, and Open again. Preserve the complete sender log, especially the
`OPEN_CAMERA requested` line. If the request target differs from the control
peer, report that line before using `--allow-target-ip-mismatch`.

Confirm the listener and host firewall before opening the PICO client:

```bash
ss -ltnp | grep ':13579'
sudo ufw status
```

If UFW is active and does not already allow the lab subnet, add a narrowly
scoped TCP rule for port 13579 according to the lab network policy.

## 7. Direct fallback mode

If the PICO exposes a plain video listener on 12345:

```bash
/usr/bin/python3 -m vision.d455_rgb_sender \
  --mode direct \
  --device /dev/v4l/by-id/<D455-RGB-node> \
  --input-mode raw \
  --source-format YUY2 \
  --pico-ip <PICO-IP> \
  --pico-port 12345 \
  --verbose
```

## 8. Stability and coexistence

Run video alone for 30 minutes, then alongside XR tracking and Baxter IK
dry-run for 30 minutes. Record `pidstat -p <video-pid> 1`, packet age/control
timeouts, AU/s, gap count, reconnect count, and glass-to-glass latency. With
Baxter disabled, also close/reopen the PICO app, unplug/replug D455, terminate
the video process, and verify that the control processes remain alive.

## 9. Optional user service

```bash
mkdir -p ~/.config/pico-baxter ~/.config/systemd/user
cp vision/systemd/pico-baxter-vision.env.example ~/.config/pico-baxter/vision.env
cp vision/systemd/pico-baxter-vision.service ~/.config/systemd/user/
```

Edit the environment file, then:

```bash
systemctl --user daemon-reload
systemctl --user enable --now pico-baxter-vision.service
journalctl --user -u pico-baxter-vision.service -f
```
