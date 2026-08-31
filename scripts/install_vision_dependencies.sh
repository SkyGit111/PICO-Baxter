#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
vision_python="${VISION_PYTHON:-/usr/bin/python3}"

sudo apt-get update
sudo apt-get install -y \
  python3-gi \
  gir1.2-gstreamer-1.0 \
  gir1.2-gst-plugins-base-1.0 \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  v4l-utils \
  ffmpeg

if [[ ! -x "$vision_python" ]]; then
  echo "Vision Python is not executable: $vision_python" >&2
  echo "Override it with VISION_PYTHON=/path/to/python if required." >&2
  exit 1
fi
if ! "$vision_python" -c 'import gi; gi.require_version("Gst", "1.0"); gi.require_version("GstVideo", "1.0"); from gi.repository import Gst, GstVideo' 2>/dev/null; then
  echo "GStreamer GI is unavailable in $vision_python." >&2
  echo "Ubuntu's python3-gi normally requires VISION_PYTHON=/usr/bin/python3." >&2
  exit 1
fi

cd "$repo_root"
echo "Using vision Python: $vision_python"
"$vision_python" -m unittest discover -s vision/tests -v
"$vision_python" -m vision.diagnose --test-source

echo "Vision dependencies and the synthetic GStreamer pipeline are ready."
