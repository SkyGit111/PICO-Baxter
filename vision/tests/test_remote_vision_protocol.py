import socket
import struct
import unittest

from vision.remote_vision_protocol import (
    CAMERA_REQUEST_MAGIC,
    frame_video_buffer,
    parse_camera_request,
    parse_control_body,
    read_control_message,
    recv_exact,
)


class VideoFramingTests(unittest.TestCase):
    def test_video_buffer_has_big_endian_length(self):
        payload = b"\x00\x00\x00\x01\x65abc"
        self.assertEqual(frame_video_buffer(payload), struct.pack(">I", len(payload)) + payload)

    def test_empty_video_buffer_is_rejected(self):
        with self.assertRaises(ValueError):
            frame_video_buffer(b"")

    def test_recv_exact_handles_fragmented_reads(self):
        left, right = socket.socketpair()
        try:
            right.sendall(b"ab")
            right.sendall(b"cdef")
            self.assertEqual(recv_exact(left, 6), b"abcdef")
        finally:
            left.close()
            right.close()


class ControlProtocolTests(unittest.TestCase):
    @staticmethod
    def make_body(command, data):
        encoded = command.encode("utf-8")
        return (
            len(encoded).to_bytes(4, "little", signed=True)
            + encoded
            + len(data).to_bytes(4, "little", signed=True)
            + data
        )

    def test_parse_open_camera_control_body(self):
        body = self.make_body("OPEN_CAMERA", b"request")
        message = parse_control_body(body)
        self.assertEqual(message.command, "OPEN_CAMERA")
        self.assertEqual(message.data, b"request")

    def test_read_outer_big_endian_frame(self):
        body = self.make_body("CLOSE_CAMERA", b"")
        left, right = socket.socketpair()
        try:
            right.sendall(struct.pack(">I", len(body)) + body)
            message = read_control_message(left)
            self.assertEqual(message.command, "CLOSE_CAMERA")
        finally:
            left.close()
            right.close()

    def test_parse_camera_request(self):
        camera = b"ZEDMINI"
        ip = b"192.168.1.42"
        data = (
            CAMERA_REQUEST_MAGIC
            + b"\x01"
            + struct.pack("<7i", 2560, 720, 30, 10_000_000, 0, 1, 12345)
            + bytes([len(camera)])
            + camera
            + bytes([len(ip)])
            + ip
        )
        request = parse_camera_request(data)
        self.assertEqual(request.width, 2560)
        self.assertEqual(request.height, 720)
        self.assertEqual(request.ip, "192.168.1.42")
        self.assertEqual(request.port, 12345)

    def test_trailing_control_bytes_are_rejected(self):
        with self.assertRaises(ValueError):
            parse_control_body(self.make_body("CLOSE_CAMERA", b"") + b"junk")


if __name__ == "__main__":
    unittest.main()
