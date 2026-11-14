"""Unit tests for benchmark logic; use synthetic terrain, do NOT require Gale files."""
from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

try:
    from .benchmark_navigation import describe_samples, run_benchmark, write_outputs
    from .planner_v4 import RouteEngine
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from benchmark_navigation import describe_samples, run_benchmark, write_outputs
    from planner_v4 import RouteEngine


class BenchmarkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        data = np.zeros((14, 14), dtype=np.float64)
        cls.engine = RouteEngine(data, {"dx_m": 5.0, "dy_m": 5.0})
        cls.cases = (("flat", (3, 2), (3, 10)),)

    def test_median_and_range(self):
        stats = describe_samples([10, 20, 30])
        self.assertEqual(stats["median_ms"], 20)
        self.assertEqual(stats["min_ms"], 10)
        self.assertEqual(stats["max_ms"], 30)

    def test_bad_samples_rejected(self):
        for values in ([], [-1], [float("nan")], [float("inf")]):
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    describe_samples(values)

    def test_profile_routes_and_counts(self):
        report = run_benchmark(self.engine, repeats=2, warmups=1, cases=self.cases)
        self.assertEqual(len(report["results"]), 4)
        for row in report["results"]:
            self.assertEqual(row["status"], "found")
            self.assertEqual(row["repeats"], 2)
            self.assertEqual(len(row["runtime_samples_ms"]), 2)
            self.assertGreater(row["distance_m"], 0)
        self.assertEqual(len(report["combined_call_times"]), 2)

    def test_outputs_exist(self):
        report = run_benchmark(self.engine, repeats=1, warmups=0, cases=self.cases)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_outputs(report, root)
            self.assertTrue((root / "navigation_benchmark.csv").is_file())
            self.assertTrue((root / "navigation_benchmark.json").is_file())
            self.assertTrue((root / "navigation_benchmark_report.md").is_file())

    def test_invalid_repeat_count(self):
        with self.assertRaises(ValueError):
            run_benchmark(self.engine, repeats=0, cases=self.cases)


if __name__ == "__main__":
    unittest.main()
