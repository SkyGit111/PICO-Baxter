import socket
import time
import unittest

from vision.stream_sender import DirectVideoSender, StreamBacklogError


class RepeatingLargeFramePipeline:
    class Config:
        fps = 30

    config = Config()
    last_access_unit_age_ms = 0.0

    def __init__(self):
        self.pts = 0

    def pull_access_unit(self, timeout_ms=250):
        self.pts += 33_333_333
        return b"\x00\x00\x00\x01" + (b"x" * (4 * 1024 * 1024)), True, self.pts

    @staticmethod
    def request_keyframe():
        return True


class BackpressureIntegrationTests(unittest.TestCase):
    def test_non_reading_receiver_triggers_send_deadline(self):
        sender_socket, receiver_socket = socket.socketpair()
        try:
            sender_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
            sender_socket.settimeout(0.05)
            sender = DirectVideoSender(
                pipeline=RepeatingLargeFramePipeline(),
                host="unused",
                port=0,
                connect_timeout=0.1,
                send_timeout=0.05,
                reconnect_delay=0.0,
                send_buffer_bytes=4096,
                max_send_ms=100.0,
            )

            started = time.monotonic()
            with self.assertRaises(StreamBacklogError):
                sender._stream_connection(sender_socket)
            elapsed = time.monotonic() - started

            self.assertLess(elapsed, 0.5)
            # A finite OS receive buffer may accept a few complete AUs before
            # it fills. The deadline must still stop the producer promptly.
            self.assertLess(sender.frames_sent, 10)
            self.assertEqual(sender.slow_sends, 1)
        finally:
            sender_socket.close()
            receiver_socket.close()


if __name__ == "__main__":
    unittest.main()
