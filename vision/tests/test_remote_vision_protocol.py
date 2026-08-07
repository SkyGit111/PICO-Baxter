import socket
import struct
import unittest

from vision.remote_vision_protocol import (
    CAMERA_REQUEST_MAGIC,
    MAX_CONTROL_MESSAGE_BYTES,
    MAX_VIDEO_BUFFER_BYTES,
    frame_video_buffer,
    frame_control_message,
    parse_camera_request,
    parse_control_body,
    read_video_buffer,
    read_control_message,
    recv_exact,
    serialize_camera_request,
)


class VideoFramingTests(unittest.TestCase):
    def test_video_buffer_has_big_endian_length(self):
        payload = b"\x00\x00\x00\x01\x65abc"
        self.assertEqual(frame_video_buffer(payload), struct.pack(">I", len(payload)) + payload)

    def test_empty_video_buffer_is_rejected(self):
        with self.assertRaises(ValueError):
            frame_video_buffer(b"")

    def test_video_buffer_round_trip(self):
        left, right = socket.socketpair()
        try:
            right.sendall(frame_video_buffer(b"\x00\x00\x00\x01\x65frame"))
            self.assertEqual(
                read_video_buffer(left), b"\x00\x00\x00\x01\x65frame"
            )
        finally:
            left.close()
            right.close()

    def test_oversized_video_length_is_rejected_before_payload_read(self):
        left, right = socket.socketpair()
        try:
            right.sendall(struct.pack(">I", MAX_VIDEO_BUFFER_BYTES + 1))
            with self.assertRaises(ValueError):
                read_video_buffer(left)
        finally:
            left.close()
            right.close()

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

    def test_oversized_control_length_is_rejected_before_body_read(self):
        left, right = socket.socketpair()
        try:
            right.sendall(struct.pack(">I", MAX_CONTROL_MESSAGE_BYTES + 1))
            with self.assertRaises(ValueError):
                read_control_message(left)
        finally:
            left.close()
            right.close()

    def test_control_message_serializer_round_trip(self):
        framed = frame_control_message("OPEN_CAMERA", b"request")
        body_length = struct.unpack(">I", framed[:4])[0]
        self.assertEqual(body_length, len(framed) - 4)
        self.assertEqual(
            parse_control_body(framed[4:]),
            parse_control_body(self.make_body("OPEN_CAMERA", b"request")),
        )

    def test_empty_control_command_is_rejected(self):
        with self.assertRaises(ValueError):
            frame_control_message("")

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

    def test_camera_request_serializer_round_trip(self):
        original = parse_camera_request(
            CAMERA_REQUEST_MAGIC
            + b"\x01"
            + struct.pack("<7i", 2560, 720, 30, 10_000_000, 0, 1, 12345)
            + b"\x03cam"
            + b"\x09127.0.0.1"
        )
        self.assertEqual(parse_camera_request(serialize_camera_request(original)), original)

    def test_trailing_control_bytes_are_rejected(self):
        with self.assertRaises(ValueError):
            parse_control_body(self.make_body("CLOSE_CAMERA", b"") + b"junk")


if __name__ == "__main__":
    unittest.main()
