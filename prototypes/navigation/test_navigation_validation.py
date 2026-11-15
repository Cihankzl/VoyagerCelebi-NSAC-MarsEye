"""Independent synthetic-unit tests for MarsEye V4 and validator.

Run: python prototypes/navigation/test_navigation_validation.py
"""
from __future__ import annotations

import math
import unittest

import numpy as np

try:
    from .planner_v4 import RouteEngine, PROFILES
    from .validate_navigation import validate_path, validate_comparison
except ImportError:
    from planner_v4 import RouteEngine, PROFILES
    from validate_navigation import validate_path, validate_comparison


class NavigationContractTests(unittest.TestCase):
    @staticmethod
    def engine(z=None, risk=None):
        if z is None:
            z = np.zeros((20, 20), dtype=float)
        risks = {"rover": risk} if risk is not None else None
        return RouteEngine(np.asarray(z, dtype=float), {"dx_m": 5.0, "dy_m": 5.0}, risk_maps=risks)

    def test_flat_route_and_metrics(self):
        engine = self.engine()
        result = engine.compute([2, 2], [7, 10], "rover")
        self.assertEqual(result["routes"]["shortest_feasible"]["status"], "found")
        self.assertEqual(validate_comparison(result), [])
        self.assertEqual(validate_path(engine, result, "shortest_feasible"), [])
        self.assertEqual(validate_path(engine, result, "risk_aware"), [])
        expected_m = 5 * math.hypot(5, 5) + 3 * 5.0
        self.assertAlmostEqual(result["routes"]["shortest_feasible"]["distance_m"], expected_m, delta=0.01)

    def test_endpoints_equal(self):
        engine = self.engine()
        result = engine.compute([3, 3], [3, 3], "astronaut")
        for kind in ("shortest_feasible", "risk_aware"):
            self.assertEqual(result["routes"][kind]["points_rc"], [[3, 3]])
            self.assertEqual(result["routes"][kind]["distance_m"], 0.0)
            self.assertEqual(validate_path(engine, result, kind), [])

    def test_invalid_coordinate_outside(self):
        engine = self.engine()
        with self.assertRaises(ValueError):
            engine.compute([-1, 1], [4, 5], "rover")

    def test_nan_barrier_no_route(self):
        z = np.zeros((21, 21), dtype=float)
        z[10, :] = np.nan
        engine = self.engine(z)
        result = engine.compute([4, 4], [17, 17], "rover")
        self.assertEqual(result["routes"]["shortest_feasible"]["status"], "no_path")
        self.assertEqual(result["routes"]["risk_aware"]["status"], "no_path")

    def test_no_diagonal_corner_cut(self):
        engine = self.engine()
        allowed = np.ones((20, 20), dtype=bool)
        allowed[5, 6] = False
        self.assertFalse(engine._legal_edge((5, 5), (6, 6), allowed, PROFILES["rover"].max_edge_grade_deg))

    def test_risk_aware_avoids_external_high_risk_corridor(self):
        risk = np.zeros((20, 20), dtype=float)
        risk[10, 4:16] = 1.0
        engine = self.engine(risk=risk)
        result = engine.compute([10, 3], [10, 16], "rover")
        short = result["routes"]["shortest_feasible"]
        safer = result["routes"]["risk_aware"]
        self.assertEqual(short["status"], "found")
        self.assertEqual(safer["status"], "found")
        self.assertGreater(short["weighted_cost_m_equiv"], safer["weighted_cost_m_equiv"])
        self.assertGreater(safer["distance_m"], short["distance_m"])
        self.assertEqual(validate_comparison(result), [])
        self.assertEqual(validate_path(engine, result, "risk_aware"), [])


if __name__ == "__main__":
    unittest.main()
