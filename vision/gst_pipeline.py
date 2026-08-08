"""GStreamer pipeline construction and lazy runtime binding."""

from __future__ import annotations

import os
import shlex
import sys
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class PipelineConfig:
    device: str
    width: int = 1280
    height: int = 720
    fps: int = 30
    bitrate_kbps: int = 10 * 1024
    key_interval: int = 30
    vbv_buffer_ms: int = 100
    input_mode: str = "raw"
    source_format: Optional[str] = None
    test_source: bool = False

    def validate(self) -> None:
        if not self.test_source and not self.device:
            raise ValueError("a V4L2 --device path is required")
        if self.width <= 0 or self.height <= 0 or self.fps <= 0:
            raise ValueError("width, height and fps must be positive")
        if self.bitrate_kbps <= 0 or self.key_interval <= 0:
            raise ValueError("bitrate and key interval must be positive")
        if self.vbv_buffer_ms <= 0:
            raise ValueError("VBV buffer duration must be positive")
        if self.input_mode not in ("raw", "mjpeg"):
            raise ValueError("input mode must be raw or mjpeg")
        if self.input_mode == "mjpeg" and self.source_format:
            raise ValueError("source format applies only to raw input mode")


def build_pipeline_description(config: PipelineConfig) -> str:
    """Build a single-capture RGB-to-SBS low-latency H.264 pipeline."""
    config.validate()
    output_width = config.width * 2
    if config.test_source:
        source = "videotestsrc is-live=true pattern=smpte do-timestamp=true"
        input_caps = [
            "video/x-raw",
            "width=%d" % config.width,
            "height=%d" % config.height,
            "framerate=%d/1" % config.fps,
        ]
        decoder = []
    else:
        # shlex.quote also produces a valid quoted token for Gst.parse_launch.
        source = "v4l2src device=%s do-timestamp=true" % shlex.quote(config.device)
        input_caps = [
            "image/jpeg" if config.input_mode == "mjpeg" else "video/x-raw",
            "width=%d" % config.width,
            "height=%d" % config.height,
            "framerate=%d/1" % config.fps,
        ]
        if config.source_format:
            input_caps.append("format=%s" % config.source_format)
        decoder = ["! jpegdec"] if config.input_mode == "mjpeg" else []

    return " ".join(
        [
            "compositor name=sbs background=black latency=0",
            "sink_0::xpos=0 sink_0::ypos=0",
            "sink_1::xpos=%d sink_1::ypos=0" % config.width,
            "! video/x-raw,width=%d,height=%d,framerate=%d/1"
            % (output_width, config.height, config.fps),
            "! videoconvert",
            "! video/x-raw,format=I420",
            "! x264enc name=encoder tune=zerolatency speed-preset=ultrafast",
            "bframes=0 byte-stream=true aud=true sliced-threads=true",
            "ref=1 rc-lookahead=0 sync-lookahead=0 vbv-buf-capacity=%d"
            % config.vbv_buffer_ms,
            "key-int-max=%d bitrate=%d" % (config.key_interval, config.bitrate_kbps),
            "! h264parse config-interval=-1",
            "! video/x-h264,profile=baseline,stream-format=byte-stream,alignment=au",
            "! appsink name=encoded_sink max-buffers=1 drop=true sync=false",
            "emit-signals=false enable-last-sample=false",
            source,
            "! %s" % ",".join(input_caps),
            *decoder,
            "! videoconvert",
            "! tee name=split",
            "split. ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0",
            "leaky=downstream ! sbs.sink_0",
            "split. ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0",
            "leaky=downstream ! sbs.sink_1",
        ]
    )


class GstPipeline:
    """Own one GStreamer pipeline and therefore one D455 open handle."""

    def __init__(self, config: PipelineConfig) -> None:
        self.config = config
        self.description = build_pipeline_description(config)
        self._gst = None
        self._pipeline = None
        self._sink = None

    def start(self) -> None:
        if self._pipeline is not None:
            return
        if not self.config.test_source and not os.path.exists(self.config.device):
            raise FileNotFoundError("camera device does not exist: %s" % self.config.device)

        Gst = _load_gst()
        _check_element_factories(Gst, self.config)
        pipeline = Gst.parse_launch(self.description)
        sink = pipeline.get_by_name("encoded_sink")
        if sink is None:
            pipeline.set_state(Gst.State.NULL)
            raise RuntimeError("GStreamer pipeline has no encoded_sink")

        try:
            _set_playing_or_raise(Gst, pipeline)
        except Exception:
            pipeline.set_state(Gst.State.NULL)
            raise

        self._gst = Gst
        self._pipeline = pipeline
        self._sink = sink

    def preflight(self) -> str:
        """Briefly run the pipeline to validate plugins and negotiated caps."""
        if not self.config.test_source and not os.path.exists(self.config.device):
            raise FileNotFoundError("camera device does not exist: %s" % self.config.device)
        Gst = _load_gst()
        _check_element_factories(Gst, self.config)
        pipeline = Gst.parse_launch(self.description)
        try:
            if pipeline.get_by_name("encoded_sink") is None:
                raise RuntimeError("GStreamer pipeline has no encoded_sink")
            _set_playing_or_raise(Gst, pipeline)
        finally:
            pipeline.set_state(Gst.State.NULL)
        return Gst.version_string()

    def pull_access_unit(
        self, timeout_ms: int = 250
    ) -> Optional[tuple[bytes, bool, Optional[int]]]:
        """Return ``(Annex-B bytes, is_keyframe, pts_ns)`` or None."""
        if self._sink is None or self._gst is None:
            raise RuntimeError("pipeline is not running")
        sample = self._sink.emit("try-pull-sample", timeout_ms * 1_000_000)
        if sample is None:
            self.raise_on_bus_error()
            return None

        buffer = sample.get_buffer()
        success, mapping = buffer.map(self._gst.MapFlags.READ)
        if not success:
            raise RuntimeError("failed to map encoded GStreamer buffer")
        try:
            payload = bytes(mapping.data)
        finally:
            buffer.unmap(mapping)
        is_keyframe = not buffer.has_flags(self._gst.BufferFlags.DELTA_UNIT)
        pts = None if buffer.pts == self._gst.CLOCK_TIME_NONE else int(buffer.pts)
        return payload, is_keyframe, pts

    def raise_on_bus_error(self) -> None:
        if self._pipeline is None or self._gst is None:
            return
        message = self._pipeline.get_bus().pop_filtered(
            self._gst.MessageType.ERROR | self._gst.MessageType.EOS
        )
        if message is None:
            return
        if message.type == self._gst.MessageType.ERROR:
            error, debug = message.parse_error()
            raise RuntimeError("GStreamer error: %s (%s)" % (error, debug or "no details"))
        raise RuntimeError("GStreamer pipeline reached end of stream")

    def stop(self) -> None:
        if self._pipeline is not None and self._gst is not None:
            self._pipeline.set_state(self._gst.State.NULL)
        self._sink = None
        self._pipeline = None
        self._gst = None


def _load_gst():
    try:
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
    except (ImportError, ValueError) as exc:
        raise RuntimeError(
            "GStreamer Python bindings are unavailable for %s; install "
            "python3-gi and run the vision service with Ubuntu's system "
            "interpreter (normally /usr/bin/python3)" % sys.executable
        ) from exc
    Gst.init(None)
    return Gst


def required_elements(config: PipelineConfig) -> tuple[str, ...]:
    elements = [
        "videotestsrc" if config.test_source else "v4l2src",
        "compositor",
        "videoconvert",
        "tee",
        "queue",
        "x264enc",
        "h264parse",
        "appsink",
    ]
    if not config.test_source and config.input_mode == "mjpeg":
        elements.append("jpegdec")
    return tuple(elements)


def _check_element_factories(Gst, config: PipelineConfig) -> None:
    missing = [
        name
        for name in required_elements(config)
        if Gst.ElementFactory.find(name) is None
    ]
    if missing:
        raise RuntimeError("missing GStreamer elements: %s" % ", ".join(missing))


def _set_playing_or_raise(Gst, pipeline, timeout_seconds: int = 5) -> None:
    result = pipeline.set_state(Gst.State.PLAYING)
    if result == Gst.StateChangeReturn.FAILURE:
        _raise_startup_error(Gst, pipeline, "failed to enter PLAYING")
    if result == Gst.StateChangeReturn.ASYNC:
        result, _current, _pending = pipeline.get_state(
            timeout_seconds * Gst.SECOND
        )
        if result == Gst.StateChangeReturn.FAILURE:
            _raise_startup_error(Gst, pipeline, "failed during startup")
        if result == Gst.StateChangeReturn.ASYNC:
            raise RuntimeError(
                "GStreamer pipeline startup timed out after %d seconds"
                % timeout_seconds
            )


def _raise_startup_error(Gst, pipeline, summary: str) -> None:
    message = pipeline.get_bus().timed_pop_filtered(
        100 * Gst.MSECOND, Gst.MessageType.ERROR
    )
    if message is not None:
        error, debug = message.parse_error()
        raise RuntimeError(
            "GStreamer pipeline %s: %s (%s)"
            % (summary, error, debug or "no details")
        )
    raise RuntimeError("GStreamer pipeline %s" % summary)
