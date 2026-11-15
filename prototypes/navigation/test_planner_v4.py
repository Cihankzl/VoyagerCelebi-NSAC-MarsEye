"""No pytest dependency: python prototypes/navigation/test_planner_v4.py"""
import math
import unittest
import numpy as np
from planner_v4 import RouteEngine, PROFILES


class TestMarsEyeV4(unittest.TestCase):
    def setUp(self):
        self.flat = np.zeros((30, 30), dtype=np.float64)
        self.meta = {"dx_m": 5.0, "dy_m": 5.0}

    def test_flat_two_routes(self):
        engine = RouteEngine(self.flat, self.meta)
        r = engine.compute([5, 5], [20, 20], "rover")
        a, b = r["routes"]["shortest_feasible"], r["routes"]["risk_aware"]
        self.assertEqual(a["status"], "found")
        self.assertEqual(b["status"], "found")
        self.assertAlmostEqual(a["distance_m"], 15 * math.sqrt(2) * 5, places=2)
        self.assertAlmostEqual(a["distance_m"], b["distance_m"])
        self.assertIsNone(a["mean_risk_score_0_1"])

    def test_invalid_and_blocked(self):
        engine = RouteEngine(self.flat, self.meta)
        with self.assertRaises(ValueError):
            engine.compute([-1, 0], [1, 1], "rover")
        with self.assertRaises(ValueError):
            engine.compute([1.2, 0], [1, 1], "rover")
        data = self.flat.copy()
        data[9:12, 5:25] = 100.0
        engine = RouteEngine(data, self.meta)
        item = engine.compute([9, 10], [20, 20], "rover")
        self.assertEqual(item["routes"]["shortest_feasible"]["status"], "blocked_endpoint")

    def test_unknown_external_risk_consistent(self):
        risk = np.zeros_like(self.flat)
        risk[:, 15] = np.nan
        engine = RouteEngine(self.flat, self.meta, risk_maps={"rover": risk})
        r = engine.compute([5, 5], [5, 25], "rover")
        self.assertEqual(r["routes"]["shortest_feasible"]["status"], "no_path")
        self.assertEqual(r["routes"]["risk_aware"]["status"], "no_path")

    def test_external_risk_detour(self):
        risk = np.zeros_like(self.flat)
        risk[12:18, 6:24] = 1.0
        engine = RouteEngine(self.flat, self.meta, risk_maps={"rover": risk})
        r = engine.compute([15, 3], [15, 26], "rover")
        short, safe = r["routes"]["shortest_feasible"], r["routes"]["risk_aware"]
        self.assertEqual(short["status"], "found")
        self.assertEqual(safe["status"], "found")
        self.assertGreaterEqual(safe["distance_m"], short["distance_m"] - .001)
        self.assertLess(safe["weighted_cost_m_equiv"], short["weighted_cost_m_equiv"])
        self.assertEqual(safe["risk_source"], "external_risk_map")

    def test_reference_is_analysis_only(self):
        engine = RouteEngine(self.flat, self.meta)
        r = engine.compute([4, 4], [8, 8], "astronaut", include_reference=True)
        self.assertTrue(r["routes"]["unconstrained_reference"]["analysis_only"])


if __name__ == "__main__":
    unittest.main()
