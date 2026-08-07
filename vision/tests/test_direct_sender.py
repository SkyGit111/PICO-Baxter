import struct
import unittest

from vision.d455_rgb_sender import DirectVideoSender


class FakePipeline:
    class Config:
        fps = 30

    config = Config()

    def __init__(self, frames):
        self.frames = iter(frames)

    def pull_access_unit(self, timeout_ms=250):
        return next(self.frames)


class CapturingSocket:
    def __init__(self, sender, stop_after):
        self.sender = sender
        self.stop_after = stop_after
        self.writes = []

    def sendall(self, data):
        self.writes.append(data)
        if len(self.writes) >= self.stop_after:
            self.sender.stop_event.set()


class DirectSenderTests(unittest.TestCase):
    @staticmethod
    def make_sender(frames):
        return DirectVideoSender(
            pipeline=FakePipeline(frames),
            host="127.0.0.1",
            port=12345,
            connect_timeout=1.0,
            send_timeout=1.0,
            reconnect_delay=0.0,
            send_buffer_bytes=65536,
        )

    def test_connection_starts_at_keyframe(self):
        frames = [
            (b"delta", False, 0),
            (b"key", True, 33_333_333),
        ]
        sender = self.make_sender(frames)
        sock = CapturingSocket(sender, stop_after=1)
        sender._stream_connection(sock)
        self.assertEqual(sock.writes, [struct.pack(">I", 3) + b"key"])
        self.assertEqual(sender.pre_keyframe_drops, 1)

    def test_pts_gap_waits_for_next_keyframe(self):
        period = 33_333_333
        frames = [
            (b"key1", True, 0),
            (b"delta1", False, period),
            (b"broken", False, period * 4),
            (b"key2", True, period * 5),
        ]
        sender = self.make_sender(frames)
        sock = CapturingSocket(sender, stop_after=3)
        sender._stream_connection(sock)
        payloads = [entry[4:] for entry in sock.writes]
        self.assertEqual(payloads, [b"key1", b"delta1", b"key2"])
        self.assertEqual(sender.discontinuities, 1)


if __name__ == "__main__":
    unittest.main()
