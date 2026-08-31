import socket
import struct
import unittest
from unittest import mock

from vision.stream_sender import DirectVideoSender, StreamBacklogError


class FakePipeline:
    class Config:
        fps = 30

    config = Config()

    def __init__(self, frames):
        self.frames = iter(frames)
        self.keyframe_requests = 0
        self.last_access_unit_age_ms = 12.5

    def pull_access_unit(self, timeout_ms=250):
        return next(self.frames)

    def request_keyframe(self):
        self.keyframe_requests += 1
        return True


class CapturingSocket:
    def __init__(self, sender, stop_after):
        self.sender = sender
        self.stop_after = stop_after
        self.writes = []

    def sendall(self, data):
        self.writes.append(data)
        if len(self.writes) >= self.stop_after:
            self.sender.stop_event.set()


class FailingSocket:
    def sendall(self, _data):
        raise OSError("connection reset")


class TimingOutSocket:
    def sendall(self, _data):
        raise TimeoutError("send timed out")


class PassiveSocket:
    def __init__(self):
        self.writes = []

    def sendall(self, data):
        self.writes.append(data)


class ConnectedSocket:
    def __init__(self):
        self.options = []
        self.timeout = None

    def setsockopt(self, level, option, value):
        self.options.append((level, option, value))

    def getsockopt(self, level, option):
        self.options.append((level, option, "get"))
        return 32768

    def settimeout(self, value):
        self.timeout = value


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
        self.assertEqual(sender.pipeline.keyframe_requests, 1)

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
        self.assertEqual(sender.pipeline.keyframe_requests, 2)

    def test_connection_loss_retries_and_starts_again_at_keyframe(self):
        frames = [
            (b"key1", True, 0),
            (b"delta-after-reconnect", False, 33_333_333),
            (b"key2", True, 66_666_666),
        ]
        sender = self.make_sender(frames)
        sender.manage_pipeline = False
        second = CapturingSocket(sender, stop_after=1)
        sockets = iter((FailingSocket(), second))
        sender._connect = lambda: next(sockets)

        sender.run()

        self.assertEqual(second.writes, [struct.pack(">I", 4) + b"key2"])
        self.assertEqual(sender.pre_keyframe_drops, 1)

    def test_slow_complete_write_closes_stream_instead_of_building_history(self):
        sender = self.make_sender([(b"key", True, 0)])
        sender.max_send_ms = 250.0
        sock = PassiveSocket()
        with mock.patch(
            "vision.stream_sender.time.monotonic",
            side_effect=(0.0, 0.0, 0.300),
        ):
            with self.assertRaises(StreamBacklogError):
                sender._stream_connection(sock)
        self.assertEqual(sender.frames_sent, 1)
        self.assertEqual(sender.slow_sends, 1)
        self.assertAlmostEqual(sender.last_send_ms, 300.0)

    def test_pipeline_age_is_exposed_in_diagnostics(self):
        sender = self.make_sender([])
        self.assertEqual(sender._pipeline_age_text(), "12.5ms")

    def test_send_timeout_is_counted_as_backlog(self):
        sender = self.make_sender([(b"key", True, 0)])
        with mock.patch(
            "vision.stream_sender.time.monotonic",
            side_effect=(0.0, 0.0, 0.251),
        ):
            with self.assertRaises(StreamBacklogError):
                sender._stream_connection(TimingOutSocket())
        self.assertEqual(sender.frames_sent, 0)
        self.assertEqual(sender.slow_sends, 1)
        self.assertAlmostEqual(sender.last_send_ms, 251.0)

    def test_connect_applies_low_latency_socket_options(self):
        sender = self.make_sender([])
        sock = ConnectedSocket()
        with mock.patch(
            "vision.stream_sender.socket.create_connection", return_value=sock
        ) as create_connection:
            self.assertIs(sender._connect(), sock)

        create_connection.assert_called_once_with(
            ("127.0.0.1", 12345), timeout=sender.connect_timeout
        )
        self.assertEqual(sock.timeout, sender.send_timeout)
        self.assertIn(
            (socket.IPPROTO_TCP, socket.TCP_NODELAY, 1), sock.options
        )
        self.assertIn(
            (socket.SOL_SOCKET, socket.SO_SNDBUF, sender.send_buffer_bytes),
            sock.options,
        )
        self.assertIn(
            (socket.IPPROTO_IP, socket.IP_TOS, sender.dscp << 2), sock.options
        )
        if hasattr(socket, "TCP_NOTSENT_LOWAT"):
            self.assertIn(
                (
                    socket.IPPROTO_TCP,
                    socket.TCP_NOTSENT_LOWAT,
                    sender.tcp_notsent_lowat_bytes,
                ),
                sock.options,
            )


if __name__ == "__main__":
    unittest.main()
