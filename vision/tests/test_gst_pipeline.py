import unittest

from vision.gst_pipeline import PipelineConfig, build_pipeline_description


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
        self.assertIn("alignment=au", description)
        self.assertIn("profile=baseline", description)
        self.assertIn("max-buffers=1 drop=true", description)
        self.assertEqual(description.count("v4l2src"), 1)

    def test_test_source_does_not_open_v4l2(self):
        description = build_pipeline_description(
            PipelineConfig(device="", test_source=True)
        )
        self.assertIn("videotestsrc", description)
        self.assertNotIn("v4l2src", description)

    def test_missing_device_is_rejected(self):
        with self.assertRaises(ValueError):
            build_pipeline_description(PipelineConfig(device=""))


if __name__ == "__main__":
    unittest.main()
