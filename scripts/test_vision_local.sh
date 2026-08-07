#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
test_dir="$(mktemp -d)"
sender_pid=""

cleanup() {
  if [[ -n "$sender_pid" ]] && kill -0 "$sender_pid" 2>/dev/null; then
    kill -TERM "$sender_pid" 2>/dev/null || true
    wait "$sender_pid" 2>/dev/null || true
  fi
  rm -r -- "$test_dir"
}
trap cleanup EXIT INT TERM

cd "$repo_root"
python3 -m unittest discover -s vision/tests -v
python3 -m vision.diagnose --test-source

python3 -m vision.d455_rgb_sender \
  --mode listen \
  --test-source \
  --listen-host 127.0.0.1 \
  --listen-port 13579 \
  --verbose >"$test_dir/sender.log" 2>&1 &
sender_pid="$!"

listener_ready=0
for _attempt in $(seq 1 50); do
  if ! kill -0 "$sender_pid" 2>/dev/null; then
    echo "Video sender exited before becoming ready:" >&2
    cat "$test_dir/sender.log" >&2
    exit 1
  fi
  if python3 -c 'import socket; s=socket.create_connection(("127.0.0.1", 13579), 0.2); s.close()' 2>/dev/null; then
    listener_ready=1
    break
  fi
  sleep 0.1
done

if [[ "$listener_ready" -ne 1 ]]; then
  echo "Remote Vision listener did not become ready:" >&2
  cat "$test_dir/sender.log" >&2
  exit 1
fi

python3 -m vision.mock_pico \
  --mode control \
  --control-host 127.0.0.1 \
  --control-port 13579 \
  --video-listen-host 127.0.0.1 \
  --video-advertise-ip 127.0.0.1 \
  --video-port 0 \
  --frames 120 \
  --output "$test_dir/sbs.h264" \
  --timeout 15 \
  --verbose

probe="$test_dir/probe.txt"
ffprobe -v error -f h264 -select_streams v:0 \
  -show_entries stream=codec_name,profile,width,height,r_frame_rate,has_b_frames \
  -of default=noprint_wrappers=1 "$test_dir/sbs.h264" >"$probe"

grep -qx 'codec_name=h264' "$probe"
grep -Eq '^profile=(Constrained )?Baseline$' "$probe"
grep -qx 'width=2560' "$probe"
grep -qx 'height=720' "$probe"
grep -qx 'has_b_frames=0' "$probe"
grep -qx 'r_frame_rate=30/1' "$probe"

left_md5="$(ffmpeg -v error -f h264 -i "$test_dir/sbs.h264" -frames:v 1 \
  -vf 'crop=1280:720:0:0' -f md5 -)"
right_md5="$(ffmpeg -v error -f h264 -i "$test_dir/sbs.h264" -frames:v 1 \
  -vf 'crop=1280:720:1280:0' -f md5 -)"

if [[ "$left_md5" != "$right_md5" ]]; then
  echo "SBS halves differ: left=$left_md5 right=$right_md5" >&2
  exit 1
fi

kill -TERM "$sender_pid"
wait "$sender_pid"
sender_pid=""

echo "Local Remote Vision end-to-end test passed."
cat "$probe"
echo "$left_md5"
