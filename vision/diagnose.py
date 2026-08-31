#!/usr/bin/env python3
"""Inspect D455 V4L2 nodes and validate the configured GStreamer pipeline."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

try:
    from .gst_pipeline import GstPipeline, PipelineConfig, required_elements
except ImportError:  # Permit direct execution.
    from gst_pipeline import GstPipeline, PipelineConfig, required_elements


DEFAULT_BY_ID_DIR = Path("/dev/v4l/by-id")


def discover_video_devices(by_id_dir: Path = DEFAULT_BY_ID_DIR) -> list[Path]:
    if not by_id_dir.is_dir():
        return []
    return sorted(path for path in by_id_dir.iterdir() if path.name != ".")


def describe_device(path: Path) -> str:
    try:
        resolved = path.resolve(strict=True)
    except (FileNotFoundError, OSError):
        return "%s -> <broken>" % path
    return "%s -> %s" % (path, resolved)


def query_v4l2_formats(device: str) -> str:
    executable = shutil.which("v4l2-ctl")
    if executable is None:
        raise RuntimeError("v4l2-ctl is unavailable; install v4l-utils")
    result = subprocess.run(
        [executable, "--device", device, "--list-formats-ext"],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=10.0,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "v4l2-ctl failed for %s (exit %d):\n%s"
            % (device, result.returncode, result.stdout.strip())
        )
    return result.stdout


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="")
    parser.add_argument("--input-mode", choices=("raw", "mjpeg"), default="raw")
    parser.add_argument("--source-format", default=None)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--bitrate-mbps", type=float, default=6.0)
    parser.add_argument("--test-source", action="store_true")
    parser.add_argument(
        "--list-only",
        action="store_true",
        help="Only list stable V4L2 paths; do not validate a pipeline",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print("Python interpreter:", sys.executable)
    print("Stable V4L2 device paths:")
    devices = discover_video_devices()
    if devices:
        for path in devices:
            print("  ", describe_device(path))
    else:
        print("   <none found under %s>" % DEFAULT_BY_ID_DIR)

    if args.list_only:
        return 0
    if not args.test_source and not args.device:
        raise ValueError("select a D455 RGB node with --device")
    if args.device:
        if not os.path.exists(args.device):
            raise FileNotFoundError("camera device does not exist: %s" % args.device)
        print("\nV4L2 formats for %s:" % args.device)
        print(query_v4l2_formats(args.device).rstrip())

    config = PipelineConfig(
        device=args.device,
        width=args.width,
        height=args.height,
        fps=args.fps,
        bitrate_kbps=int(args.bitrate_mbps * 1024),
        input_mode=args.input_mode,
        source_format=args.source_format,
        test_source=args.test_source,
    )
    pipeline = GstPipeline(config)
    print("\nRequired GStreamer elements:")
    print("  " + " ".join(required_elements(config)))
    version = pipeline.preflight()
    print("\nPreflight OK:", version)
    print("Pipeline:", pipeline.description)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
