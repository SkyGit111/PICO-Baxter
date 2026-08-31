import unittest
import sys
import types
from unittest import mock

from vision.gst_pipeline import (
    PipelineConfig,
    _check_element_factories,
    _set_playing_or_raise,
    build_pipeline_description,
    required_elements,
    GstPipeline,
)


class PipelineDescriptionTests(unittest.TestCase):
    def test_default_pipeline_is_sbs_and_bounded(self):
        description = build_pipeline_description(
            PipelineConfig(device="/dev/v4l/by-id/d455-rgb")
        )
        self.assertIn("v4l2src device=/dev/v4l/by-id/d455-rgb", description)
        self.assertIn("sink_1::xpos=1280", description)
        self.assertIn("latency=0", description)
        self.assertIn("width=2560,height=720,framerate=30/1", description)
        self.assertIn("tune=zerolatency", description)
        self.assertIn("speed-preset=ultrafast", description)
        self.assertIn("bframes=0", description)
        self.assertIn("rc-lookahead=0", description)
        self.assertIn("sync-lookahead=0", description)
        self.assertIn("vbv-buf-capacity=50", description)
        self.assertIn("key-int-max=15", description)
        self.assertIn("bitrate=6144", description)
        self.assertIn("ref=1", description)
        self.assertIn("alignment=au", description)
        self.assertIn("profile=baseline", description)
        self.assertIn("max-buffers=1 drop=true", description)
        self.assertGreaterEqual(description.count("leaky=downstream"), 4)
        self.assertEqual(description.count("v4l2src"), 1)

    def test_test_source_does_not_open_v4l2(self):
        description = build_pipeline_description(
            PipelineConfig(device="", test_source=True)
        )
        self.assertIn("videotestsrc", description)
        self.assertNotIn("v4l2src", description)

    def test_mjpeg_pipeline_decodes_before_sbs(self):
        description = build_pipeline_description(
            PipelineConfig(device="/dev/d455", input_mode="mjpeg")
        )
        self.assertIn("image/jpeg,width=1280,height=720", description)
        self.assertIn("! jpegdec ! queue", description)
        self.assertIn("leaky=downstream ! videoconvert", description)

    def test_mjpeg_rejects_raw_source_format(self):
        with self.assertRaises(ValueError):
            build_pipeline_description(
                PipelineConfig(
                    device="/dev/d455", input_mode="mjpeg", source_format="YUY2"
                )
            )

    def test_missing_device_is_rejected(self):
        with self.assertRaises(ValueError):
            build_pipeline_description(PipelineConfig(device=""))

    def test_mjpeg_requires_decoder_but_test_source_does_not(self):
        mjpeg = required_elements(PipelineConfig(device="/dev/d455", input_mode="mjpeg"))
        synthetic = required_elements(PipelineConfig(device="", test_source=True))
        self.assertIn("jpegdec", mjpeg)
        self.assertNotIn("jpegdec", synthetic)
        self.assertIn("videotestsrc", synthetic)
        self.assertNotIn("v4l2src", synthetic)

    def test_missing_plugins_are_reported_together(self):
        class ElementFactory:
            @staticmethod
            def find(name):
                return None if name in ("x264enc", "h264parse") else object()

        class FakeGst:
            pass

        FakeGst.ElementFactory = ElementFactory
        with self.assertRaisesRegex(
            RuntimeError, "missing GStreamer elements: x264enc, h264parse"
        ):
            _check_element_factories(
                FakeGst, PipelineConfig(device="", test_source=True)
            )


class PipelineStartupTests(unittest.TestCase):
    class Return:
        FAILURE = "failure"
        ASYNC = "async"
        SUCCESS = "success"

    class Gst:
        class State:
            PLAYING = "playing"

        class MessageType:
            ERROR = "error"

        SECOND = 1_000
        MSECOND = 1

    Gst.StateChangeReturn = Return

    class Bus:
        @staticmethod
        def timed_pop_filtered(_timeout, _message_type):
            return None

    class Pipeline:
        def __init__(self, initial, settled):
            self.initial = initial
            self.settled = settled
            self.timeout = None

        def set_state(self, state):
            if state != "playing":
                raise AssertionError("unexpected state")
            return self.initial

        def get_state(self, timeout):
            self.timeout = timeout
            return self.settled, "playing", None

        def get_bus(self):
            return PipelineStartupTests.Bus()

    def test_async_start_waits_until_success(self):
        pipeline = self.Pipeline(self.Return.ASYNC, self.Return.SUCCESS)
        _set_playing_or_raise(self.Gst, pipeline, timeout_seconds=7)
        self.assertEqual(pipeline.timeout, 7_000)

    def test_async_start_timeout_is_rejected(self):
        pipeline = self.Pipeline(self.Return.ASYNC, self.Return.ASYNC)
        with self.assertRaisesRegex(RuntimeError, "timed out after 5 seconds"):
            _set_playing_or_raise(self.Gst, pipeline)

    def test_immediate_start_failure_is_rejected(self):
        pipeline = self.Pipeline(self.Return.FAILURE, self.Return.SUCCESS)
        with self.assertRaisesRegex(RuntimeError, "failed to enter PLAYING"):
            _set_playing_or_raise(self.Gst, pipeline)


class PipelineFreshnessTests(unittest.TestCase):
    def test_buffer_age_uses_pipeline_running_time(self):
        pipeline = GstPipeline(PipelineConfig(device="", test_source=True))

        class Gst:
            CLOCK_TIME_NONE = -1

            class Format:
                TIME = "time"

        class Clock:
            @staticmethod
            def get_time():
                return 2_000_000_000

        class RunningPipeline:
            @staticmethod
            def get_clock():
                return Clock()

            @staticmethod
            def get_base_time():
                return 1_000_000_000

        class Segment:
            @staticmethod
            def to_running_time(_format, pts):
                return pts

        class Sample:
            @staticmethod
            def get_segment():
                return Segment()

        pipeline._gst = Gst
        pipeline._pipeline = RunningPipeline()
        age = pipeline._calculate_buffer_age_ms(
            Sample(), types.SimpleNamespace(pts=900_000_000)
        )
        self.assertAlmostEqual(age, 100.0)

    def test_keyframe_request_is_sent_upstream_from_encoder_source_pad(self):
        pipeline = GstPipeline(PipelineConfig(device="", test_source=True))
        events = []

        class EncoderSourcePad:
            @staticmethod
            def send_event(event):
                events.append(event)
                return True

        class GstVideo:
            @staticmethod
            def video_event_new_upstream_force_key_unit(running_time, headers, count):
                return running_time, headers, count

        fake_gi = types.ModuleType("gi")
        fake_gi.require_version = lambda *_args: None
        fake_repository = types.ModuleType("gi.repository")
        fake_repository.GstVideo = GstVideo
        pipeline._gst = types.SimpleNamespace(CLOCK_TIME_NONE=-1)
        pipeline._keyframe_event_pad = EncoderSourcePad()
        with mock.patch.dict(
            sys.modules,
            {"gi": fake_gi, "gi.repository": fake_repository},
        ):
            self.assertTrue(pipeline.request_keyframe())
        self.assertEqual(events, [(-1, True, 1)])


if __name__ == "__main__":
    unittest.main()
