import sys
import unittest
from unittest import mock

from vision.d455_rgb_sender import parse_args
from vision.diagnose import parse_args as parse_diagnose_args


class SenderCliTests(unittest.TestCase):
    def test_default_profile_is_bounded_and_low_latency(self):
        with mock.patch.object(sys, "argv", ["d455_rgb_sender", "--test-source"]):
            args = parse_args()

        self.assertEqual(args.bitrate_mbps, 6.0)
        self.assertEqual(args.key_interval, 15)
        self.assertEqual(args.vbv_buffer_ms, 50)
        self.assertEqual(args.send_timeout, 0.25)
        self.assertEqual(args.reconnect_delay, 0.20)
        self.assertEqual(args.send_buffer_kib, 16)
        self.assertEqual(args.tcp_notsent_lowat_kib, 4)
        self.assertEqual(args.max_send_ms, 250.0)
        self.assertEqual(args.dscp, 34)

    def test_aggressive_profile_can_be_selected_explicitly(self):
        argv = [
            "d455_rgb_sender",
            "--test-source",
            "--width",
            "640",
            "--height",
            "360",
            "--bitrate-mbps",
            "3",
            "--key-interval",
            "10",
            "--vbv-buffer-ms",
            "30",
            "--send-timeout",
            "0.15",
            "--max-send-ms",
            "120",
            "--send-buffer-kib",
            "8",
            "--tcp-notsent-lowat-kib",
            "2",
        ]
        with mock.patch.object(sys, "argv", argv):
            args = parse_args()

        self.assertEqual((args.width, args.height), (640, 360))
        self.assertEqual(args.bitrate_mbps, 3.0)
        self.assertEqual(args.key_interval, 10)
        self.assertEqual(args.vbv_buffer_ms, 30)
        self.assertEqual(args.send_timeout, 0.15)
        self.assertEqual(args.max_send_ms, 120.0)
        self.assertEqual(args.send_buffer_kib, 8)
        self.assertEqual(args.tcp_notsent_lowat_kib, 2)

    def test_diagnose_uses_the_same_default_bitrate(self):
        with mock.patch.object(sys, "argv", ["diagnose", "--test-source"]):
            args = parse_diagnose_args()
        self.assertEqual(args.bitrate_mbps, 6.0)


if __name__ == "__main__":
    unittest.main()
