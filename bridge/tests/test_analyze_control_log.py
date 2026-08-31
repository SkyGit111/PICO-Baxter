import importlib.util
import unittest
from pathlib import Path


def load_analyzer_module():
    path = Path(__file__).resolve().parents[1] / "analyze_control_log.py"
    spec = importlib.util.spec_from_file_location(
        "analyze_control_log_under_test",
        path,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


analyzer = load_analyzer_module()


class ControlLogAnalyzerTests(unittest.TestCase):
    def test_mixed_results_produce_acceptance_and_latency_summary(self):
        rows = [
            {
                "event": "MOVE",
                "seq": "12",
                "ik_request_seq": "10",
                "ik_backend": "ros-service",
                "packet_age_ms": "10",
                "loop_dt_ms": "20",
                "ik_time_ms": "100",
                "ik_result_age_ms": "110",
                "tracking_error_m": "0.01",
                "joint_tracking_error_rad": "0.02",
            },
            {
                "event": "IK_REJECT",
                "seq": "20",
                "ik_request_seq": "11",
                "ik_backend": "ros-service",
                "packet_age_ms": "15",
                "loop_dt_ms": "60",
                "ik_time_ms": "400",
                "ik_result_age_ms": "420",
            },
        ]
        report = analyzer.analyze_rows(rows, 50.0, 250.0, 0.05)
        self.assertEqual(report["ik_results"], 2)
        self.assertEqual(report["accepted_moves"], 1)
        self.assertEqual(report["ik_acceptance_rate"], 0.5)
        self.assertEqual(report["metrics"]["ik_sequence_lag"]["max"], 9.0)
        self.assertEqual(report["metrics"]["ik_result_age_ms"]["p95"], 420.0)
        self.assertTrue(any("IK 结果年龄" in item for item in report["warnings"]))
        self.assertTrue(any("Grip session" in item for item in report["warnings"]))

    def test_invalid_and_non_finite_values_are_ignored(self):
        rows = [{
            "event": "MOVE",
            "ik_time_ms": "nan",
            "packet_age_ms": "bad",
            "loop_dt_ms": "",
        }]
        report = analyzer.analyze_rows(rows, 50.0, 250.0, 0.05)
        self.assertEqual(report["metrics"]["ik_time_ms"]["count"], 0)
        self.assertEqual(report["metrics"]["packet_age_ms"]["count"], 0)

    def test_empty_log_is_reported_without_division_by_zero(self):
        report = analyzer.analyze_rows([], 50.0, 250.0, 0.05)
        self.assertEqual(report["ik_results"], 0)
        self.assertEqual(report["ik_acceptance_rate"], 0.0)
        self.assertEqual(report["warnings"], [])


if __name__ == "__main__":
    unittest.main()
