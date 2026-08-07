import tempfile
import unittest
from pathlib import Path

from vision.diagnose import describe_device, discover_video_devices


class DeviceDiscoveryTests(unittest.TestCase):
    def test_missing_by_id_directory_is_empty(self):
        with tempfile.TemporaryDirectory() as temporary:
            missing = Path(temporary) / "missing"
            self.assertEqual(discover_video_devices(missing), [])

    def test_device_paths_are_sorted(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "z-camera").touch()
            (root / "a-camera").touch()
            self.assertEqual(
                [path.name for path in discover_video_devices(root)],
                ["a-camera", "z-camera"],
            )

    def test_broken_device_description_is_explicit(self):
        with tempfile.TemporaryDirectory() as temporary:
            missing = Path(temporary) / "missing"
            self.assertIn("<broken>", describe_device(missing))


if __name__ == "__main__":
    unittest.main()
