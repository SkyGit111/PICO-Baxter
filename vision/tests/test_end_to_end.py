import socket
import threading
import time
import unittest

from vision.listener import RemoteVisionListener
from vision.mock_pico import VideoReceiver
from vision.remote_vision_protocol import (
    CameraRequest,
    frame_control_message,
    serialize_camera_request,
)


class EncodedTestPipeline:
    class Config:
        width = 1280
        height = 720
        fps = 30

    config = Config()

    def __init__(self):
        self.started = 0
        self.stopped = 0
        self.frame_index = 0
        self.lock = threading.Lock()

    def start(self):
        self.started += 1

    def stop(self):
        self.stopped += 1

    def raise_on_bus_error(self):
        pass

    def pull_access_unit(self, timeout_ms=250):
        time.sleep(0.001)
        with self.lock:
            index = self.frame_index
            self.frame_index += 1
        payload = b"\x00\x00\x00\x01\x65" + index.to_bytes(4, "big")
        return payload, True, index * 33_333_333


class EndToEndTransportTests(unittest.TestCase):
    @staticmethod
    def open_request(port):
        return frame_control_message(
            "OPEN_CAMERA",
            serialize_camera_request(
                CameraRequest(
                    width=2560,
                    height=720,
                    fps=30,
                    bitrate=10_000_000,
                    enable_mv_hevc=0,
                    render_mode=1,
                    port=port,
                    camera="ZEDMINI",
                    ip="127.0.0.1",
                )
            ),
        )

    def test_control_open_stream_receive_and_close(self):
        pipeline = EncodedTestPipeline()
        listener = RemoteVisionListener(
            pipeline=pipeline,
            listen_host="127.0.0.1",
            listen_port=0,
            connect_timeout=0.5,
            send_timeout=0.5,
            reconnect_delay=0.05,
            send_buffer_bytes=65536,
        )
        listener_thread = threading.Thread(target=listener.run)
        listener_thread.start()
        self.assertTrue(listener.ready_event.wait(2.0))

        receiver = VideoReceiver(
            listen_host="127.0.0.1",
            listen_port=0,
            frame_limit=20,
            timeout_s=2.0,
        )
        receiver_thread = threading.Thread(target=receiver.run)
        receiver_thread.start()
        self.assertTrue(receiver.ready_event.wait(2.0))

        control = socket.create_connection(
            ("127.0.0.1", listener.bound_port), timeout=1.0
        )
        try:
            control.sendall(self.open_request(receiver.bound_port))
            receiver_thread.join(timeout=3.0)
            self.assertFalse(receiver_thread.is_alive())
            self.assertIsNone(receiver.error)
            self.assertEqual(receiver.frames_received, 20)
            control.sendall(frame_control_message("CLOSE_CAMERA"))
        finally:
            control.close()
            receiver.stop()
            listener.stop()
            receiver_thread.join(timeout=1.0)
            listener_thread.join(timeout=2.0)

        self.assertFalse(listener_thread.is_alive())
        self.assertEqual(pipeline.started, 1)
        self.assertEqual(pipeline.stopped, 1)

    def test_repeated_open_close_uses_one_pipeline(self):
        pipeline = EncodedTestPipeline()
        listener = RemoteVisionListener(
            pipeline=pipeline,
            listen_host="127.0.0.1",
            listen_port=0,
            connect_timeout=0.5,
            send_timeout=0.5,
            reconnect_delay=0.01,
            send_buffer_bytes=65536,
        )
        listener_thread = threading.Thread(target=listener.run)
        listener_thread.start()
        self.assertTrue(listener.ready_event.wait(2.0))
        control = socket.create_connection(
            ("127.0.0.1", listener.bound_port), timeout=1.0
        )

        try:
            for _cycle in range(3):
                receiver = VideoReceiver(
                    listen_host="127.0.0.1",
                    listen_port=0,
                    frame_limit=10,
                    timeout_s=2.0,
                )
                receiver_thread = threading.Thread(target=receiver.run)
                receiver_thread.start()
                self.assertTrue(receiver.ready_event.wait(1.0))
                control.sendall(self.open_request(receiver.bound_port))
                receiver_thread.join(timeout=2.0)
                self.assertFalse(receiver_thread.is_alive())
                self.assertIsNone(receiver.error)
                self.assertEqual(receiver.frames_received, 10)
                control.sendall(frame_control_message("CLOSE_CAMERA"))
        finally:
            control.close()
            listener.stop()
            listener_thread.join(timeout=2.0)

        self.assertFalse(listener_thread.is_alive())
        self.assertEqual(pipeline.started, 1)
        self.assertEqual(pipeline.stopped, 1)


if __name__ == "__main__":
    unittest.main()
