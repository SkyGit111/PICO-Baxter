#!/usr/bin/env python3
"""D455 RGB to PICO Remote Vision command-line entry point."""

from __future__ import annotations

import argparse
import logging
import signal

try:
    from .gst_pipeline import GstPipeline, PipelineConfig
    from .listener import RemoteVisionListener
    from .stream_sender import DirectVideoSender
except ImportError:  # Permit direct execution: python vision/d455_rgb_sender.py
    from gst_pipeline import GstPipeline, PipelineConfig
    from listener import RemoteVisionListener
    from stream_sender import DirectVideoSender


LOG = logging.getLogger("d455-vision")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="D455 RGB to low-latency SBS H.264 PICO sender"
    )
    parser.add_argument(
        "--device",
        default="",
        help="Stable D455 RGB V4L2 path, preferably /dev/v4l/by-id/...",
    )
    parser.add_argument(
        "--mode",
        choices=("direct", "listen"),
        default="direct",
        help="Directly connect to PICO, or accept Remote Vision control requests",
    )
    parser.add_argument("--pico-ip", default="")
    parser.add_argument("--pico-port", type=int, default=12345)
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=13579)
    parser.add_argument(
        "--allow-target-ip-mismatch",
        action="store_true",
        help="Allow OPEN_CAMERA to send video to an IP other than the control peer",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--bitrate-mbps", type=float, default=6.0)
    parser.add_argument("--key-interval", type=int, default=15)
    parser.add_argument("--vbv-buffer-ms", type=int, default=50)
    parser.add_argument(
        "--input-mode",
        choices=("raw", "mjpeg"),
        default="raw",
        help="D455 V4L2 transport format; use mjpeg when raw 720p30 is unavailable",
    )
    parser.add_argument(
        "--source-format",
        default=None,
        help="Optional V4L2 raw format such as YUY2; normally auto-negotiated",
    )
    parser.add_argument("--connect-timeout", type=float, default=3.0)
    parser.add_argument("--send-timeout", type=float, default=0.25)
    parser.add_argument("--reconnect-delay", type=float, default=0.20)
    parser.add_argument("--send-buffer-kib", type=int, default=16)
    parser.add_argument(
        "--tcp-notsent-lowat-kib",
        type=int,
        default=4,
        help="Linux TCP unsent-data limit; bounds sender-side queue growth",
    )
    parser.add_argument(
        "--max-send-ms",
        type=float,
        default=250.0,
        help="Reconnect if one complete H.264 AU takes longer; 0 disables",
    )
    parser.add_argument(
        "--dscp",
        type=int,
        default=34,
        help="IPv4 DSCP value for video traffic; 34 is AF41 and 0 disables",
    )
    parser.add_argument(
        "--test-source",
        action="store_true",
        help="Use a live SMPTE test pattern instead of opening a camera",
    )
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="Briefly start the pipeline to validate plugins, device, and caps",
    )
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.mode == "direct" and not args.pico_ip and not args.preflight_only:
        raise ValueError("--pico-ip is required in direct mode")
    if not 1 <= args.pico_port <= 65535:
        raise ValueError("--pico-port must be between 1 and 65535")
    if not 1 <= args.listen_port <= 65535:
        raise ValueError("--listen-port must be between 1 and 65535")
    if args.bitrate_mbps <= 0:
        raise ValueError("--bitrate-mbps must be positive")
    if args.connect_timeout <= 0 or args.send_timeout <= 0:
        raise ValueError("socket timeouts must be positive")
    if args.reconnect_delay < 0 or args.send_buffer_kib <= 0:
        raise ValueError("reconnect delay and send buffer must be valid")
    if args.tcp_notsent_lowat_kib <= 0:
        raise ValueError("--tcp-notsent-lowat-kib must be positive")
    if args.max_send_ms < 0:
        raise ValueError("--max-send-ms must be non-negative")
    if not 0 <= args.dscp <= 63:
        raise ValueError("--dscp must be between 0 and 63")

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    config = PipelineConfig(
        device=args.device,
        width=args.width,
        height=args.height,
        fps=args.fps,
        bitrate_kbps=int(args.bitrate_mbps * 1024),
        key_interval=args.key_interval,
        vbv_buffer_ms=args.vbv_buffer_ms,
        input_mode=args.input_mode,
        source_format=args.source_format,
        test_source=args.test_source,
    )
    config.validate()
    pipeline = GstPipeline(config)
    if args.preflight_only:
        version = pipeline.preflight()
        print("Preflight OK:", version)
        print("Pipeline:", pipeline.description)
        return 0
    if args.mode == "direct":
        service = DirectVideoSender(
            pipeline=pipeline,
            host=args.pico_ip,
            port=args.pico_port,
            connect_timeout=args.connect_timeout,
            send_timeout=args.send_timeout,
            reconnect_delay=args.reconnect_delay,
            send_buffer_bytes=args.send_buffer_kib * 1024,
            max_send_ms=args.max_send_ms,
            tcp_notsent_lowat_bytes=args.tcp_notsent_lowat_kib * 1024,
            dscp=args.dscp,
        )
    else:
        service = RemoteVisionListener(
            pipeline=pipeline,
            listen_host=args.listen_host,
            listen_port=args.listen_port,
            connect_timeout=args.connect_timeout,
            send_timeout=args.send_timeout,
            reconnect_delay=args.reconnect_delay,
            send_buffer_bytes=args.send_buffer_kib * 1024,
            allow_target_ip_mismatch=args.allow_target_ip_mismatch,
            max_send_ms=args.max_send_ms,
            tcp_notsent_lowat_bytes=args.tcp_notsent_lowat_kib * 1024,
            dscp=args.dscp,
        )

    def request_stop(_signum: int, _frame: object) -> None:
        LOG.info("shutdown requested")
        service.stop()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    LOG.info(
        "input=%s %dx%d@%d; output=%dx%d SBS; mode=%s",
        "videotestsrc" if args.test_source else args.device,
        args.width,
        args.height,
        args.fps,
        args.width * 2,
        args.height,
        args.mode,
    )
    if args.mode == "direct":
        LOG.info("direct video target=%s:%d", args.pico_ip, args.pico_port)
    else:
        LOG.info("control listener=%s:%d", args.listen_host, args.listen_port)
    LOG.info(
        "latency guard: bitrate=%.1fMbps GOP=%d VBV=%dms send-timeout=%.0fms "
        "max-send=%.0fms send-buffer=%dKiB TCP_NOTSENT_LOWAT=%dKiB DSCP=%d",
        args.bitrate_mbps,
        args.key_interval,
        args.vbv_buffer_ms,
        args.send_timeout * 1000.0,
        args.max_send_ms,
        args.send_buffer_kib,
        args.tcp_notsent_lowat_kib,
        args.dscp,
    )
    try:
        service.run()
    except KeyboardInterrupt:
        service.stop()
    if isinstance(service, DirectVideoSender):
        LOG.info("video sender stopped; sent %d access units", service.frames_sent)
    else:
        LOG.info("Remote Vision listener stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
