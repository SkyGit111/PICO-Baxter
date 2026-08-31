import unittest

from bridge import baxter_ik_benchmark as benchmark


class BenchmarkHelperTests(unittest.TestCase):
    def test_target_positions_cover_origin_and_both_axis_directions(self):
        targets = benchmark.target_positions([1.0, 2.0, 3.0], 0.02)
        self.assertEqual(len(targets), 7)
        self.assertEqual(targets[0], [1.0, 2.0, 3.0])
        self.assertIn([0.98, 2.0, 3.0], targets)
        self.assertIn([1.0, 2.0, 3.02], targets)

    def test_percentile_uses_nearest_rank(self):
        values = list(range(1, 101))
        self.assertEqual(benchmark.percentile(values, 0.95), 95)
        self.assertEqual(benchmark.percentile(values, 0.99), 99)
        self.assertIsNone(benchmark.percentile([], 0.95))

    def test_summary_separates_failures_from_branch_statistics(self):
        records = [
            {
                "elapsed_ms": 1.0,
                "success": True,
                "seed_difference_rad": 0.1,
            },
            {
                "elapsed_ms": 5.0,
                "success": False,
                "seed_difference_rad": None,
            },
            {
                "elapsed_ms": 2.0,
                "success": True,
                "seed_difference_rad": 0.3,
            },
        ]
        summary = benchmark.summarize(records)
        self.assertEqual(summary["calls"], 3)
        self.assertEqual(summary["successes"], 2)
        self.assertAlmostEqual(summary["success_rate"], 2.0 / 3.0)
        self.assertEqual(summary["time_ms"]["median"], 2.0)
        self.assertEqual(summary["seed_difference_rad"]["median"], 0.2)

    def test_joint_difference_requires_matching_joint_sets(self):
        self.assertAlmostEqual(
            benchmark.maximum_joint_difference(
                {"j0": 0.0, "j1": 1.0},
                {"j0": 0.2, "j1": 0.7},
            ),
            0.3,
        )
        self.assertIsNone(
            benchmark.maximum_joint_difference({"j0": 0.0}, {"j1": 0.0})
        )


if __name__ == "__main__":
    unittest.main()
