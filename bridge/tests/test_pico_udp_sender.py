import importlib.util
import unittest
from pathlib import Path


def load_sender_module():
    path = Path(__file__).resolve().parents[1] / "pico_udp_sender.py"
    spec = importlib.util.spec_from_file_location(
        "pico_udp_sender_under_test",
        path,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


sender = load_sender_module()


class DestinationTests(unittest.TestCase):
    def test_primary_and_additional_ports_preserve_order(self):
        self.assertEqual(
            sender.build_destinations("127.0.0.1", 15000, [15001, 15002]),
            [
                ("127.0.0.1", 15000),
                ("127.0.0.1", 15001),
                ("127.0.0.1", 15002),
            ],
        )

    def test_duplicate_ports_are_removed(self):
        self.assertEqual(
            sender.build_destinations("localhost", 15000, [15000, 15001, 15001]),
            [("localhost", 15000), ("localhost", 15001)],
        )

    def test_invalid_port_is_rejected(self):
        for port in (0, 65536, -1):
            with self.subTest(port=port), self.assertRaises(ValueError):
                sender.build_destinations("127.0.0.1", 15000, [port])


if __name__ == "__main__":
    unittest.main()
