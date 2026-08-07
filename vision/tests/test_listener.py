import socket
import struct
import threading
import time
import unittest

from vision.d455_rgb_sender import RemoteVisionListener, resolve_video_target
from vision.remote_vision_protocol import CAMERA_REQUEST_MAGIC, CameraRequest


class FakePipeline:
    class Config:
        width = 1280
        height = 720
        fps = 30

    config = Config()

    def __init__(self):
        self.start_count = 0
        self.stop_count = 0

    def start(self):
        self.start_count += 1

    def stop(self):
        self.stop_count += 1


class FakeSender:
    instances = []

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.stop_event = threading.Event()
        self.started = threading.Event()
        type(self).instances.append(self)

    def run(self):
        self.started.set()
        self.stop_event.wait(2.0)

    def stop(self):
        self.stop_event.set()


def compact_string(value):
    encoded = value.encode("utf-8")
    return bytes([len(encoded)]) + encoded


def camera_request(ip="127.0.0.1", port=12345):
    return (
        CAMERA_REQUEST_MAGIC
        + b"\x01"
        + struct.pack("<7i", 2560, 720, 30, 10_000_000, 0, 1, port)
        + compact_string("ZEDMINI")
        + compact_string(ip)
    )


def control_frame(command, data=b""):
    encoded_command = command.encode("utf-8")
    body = (
        len(encoded_command).to_bytes(4, "little", signed=True)
        + encoded_command
        + len(data).to_bytes(4, "little", signed=True)
        + data
    )
    return struct.pack(">I", len(body)) + body


def wait_until(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class TargetSelectionTests(unittest.TestCase):
    @staticmethod
    def make_request(ip):
        return CameraRequest(2560, 720, 30, 10_000_000, 0, 1, 12345, "cam", ip)

    def test_empty_request_ip_uses_control_peer(self):
        self.assertEqual(
            resolve_video_target(self.make_request(""), "192.168.1.20", False),
            ("192.168.1.20", 12345),
        )

    def test_different_request_ip_is_rejected_by_default(self):
        with self.assertRaises(ValueError):
            resolve_video_target(
                self.make_request("192.168.1.30"), "192.168.1.20", False
            )

    def test_different_request_ip_can_be_explicitly_allowed(self):
        self.assertEqual(
            resolve_video_target(
                self.make_request("192.168.1.30"), "192.168.1.20", True
            ),
            ("192.168.1.30", 12345),
        )


class ListenerStateMachineTests(unittest.TestCase):
    def setUp(self):
        FakeSender.instances = []
        self.pipeline = FakePipeline()
        self.listener = RemoteVisionListener(
            pipeline=self.pipeline,
            listen_host="127.0.0.1",
            listen_port=0,
            connect_timeout=1.0,
            send_timeout=1.0,
            reconnect_delay=0.1,
            send_buffer_bytes=65536,
            sender_factory=FakeSender,
        )
        self.thread = threading.Thread(target=self.listener.run, daemon=True)
        self.thread.start()
        self.assertTrue(self.listener.ready_event.wait(2.0))
        self.assertIsNotNone(self.listener.bound_port)
        self.control = socket.create_connection(
            ("127.0.0.1", self.listener.bound_port), timeout=1.0
        )

    def tearDown(self):
        try:
            self.control.close()
        except OSError:
            pass
        self.listener.stop()
        self.thread.join(timeout=2.0)
        self.assertFalse(self.thread.is_alive())
        self.assertEqual(self.pipeline.start_count, 1)
        self.assertEqual(self.pipeline.stop_count, 1)

    def test_open_close_and_reopen_keep_one_pipeline(self):
        self.control.sendall(control_frame("OPEN_CAMERA", camera_request()))
        self.assertTrue(wait_until(lambda: len(FakeSender.instances) == 1))
        first = FakeSender.instances[0]
        self.assertTrue(first.started.wait(1.0))
        self.assertFalse(first.kwargs["manage_pipeline"])
        self.assertIs(first.kwargs["pipeline"], self.pipeline)

        self.control.sendall(control_frame("OPEN_CAMERA", camera_request(port=12346)))
        self.assertTrue(wait_until(lambda: len(FakeSender.instances) == 2))
        second = FakeSender.instances[1]
        self.assertTrue(second.started.wait(1.0))
        self.assertTrue(first.stop_event.is_set())
        self.assertEqual(second.kwargs["port"], 12346)
        self.assertEqual(self.pipeline.start_count, 1)
        self.assertEqual(self.pipeline.stop_count, 0)

        self.control.sendall(control_frame("CLOSE_CAMERA"))
        self.assertTrue(wait_until(second.stop_event.is_set))
        self.assertEqual(self.pipeline.start_count, 1)

    def test_control_disconnect_stops_video_stream(self):
        self.control.sendall(control_frame("OPEN_CAMERA", camera_request()))
        self.assertTrue(wait_until(lambda: len(FakeSender.instances) == 1))
        sender = FakeSender.instances[0]
        self.assertTrue(sender.started.wait(1.0))
        self.control.close()
        self.assertTrue(wait_until(sender.stop_event.is_set))

    def test_mismatched_target_is_rejected(self):
        self.control.sendall(
            control_frame("OPEN_CAMERA", camera_request(ip="127.0.0.2"))
        )
        time.sleep(0.05)
        self.assertEqual(FakeSender.instances, [])


if __name__ == "__main__":
    unittest.main()
