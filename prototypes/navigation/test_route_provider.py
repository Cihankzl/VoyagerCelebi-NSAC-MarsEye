"""Run from repo root: python prototypes/navigation/test_route_provider.py"""
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from planner_v4 import RouteEngine
from route_provider import RouteProvider


class CountingEngine(RouteEngine):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.calls = 0

    def compute(self, *args, **kwargs):
        self.calls += 1
        return super().compute(*args, **kwargs)


class TestRouteProvider(unittest.TestCase):
    def setUp(self):
        self.meta = {"dx_m": 5.0, "dy_m": 5.0}
        self.engine = CountingEngine(np.zeros((30, 30), dtype=float), self.meta)
        self.provider = RouteProvider(self.engine)

    def test_both_objectives_and_cache_same_direction(self):
        shortest = self.provider.get_route([5, 5], [5, 20], "rover", "shortest_feasible")
        risk = self.provider.get_route([5, 5], [5, 20], "rover", "risk_aware")
        self.assertEqual(shortest["status"], "found")
        self.assertEqual(risk["status"], "found")
        self.assertAlmostEqual(shortest["distance_m"], 75.0)
        self.assertAlmostEqual(risk["distance_m"], 75.0)
        self.assertEqual(shortest["objective_unit"], "m")
        self.assertEqual(risk["objective_unit"], "model_weighted_m_equiv")
        self.assertEqual(self.engine.calls, 1)
        self.assertEqual(self.provider.engine_compute_calls, 1)
        # Caller edits must not corrupt cached paths.
        shortest["points_rc"][0][0] = 777
        self.assertEqual(self.provider.get_route([5, 5], [5, 20], "rover", "shortest_feasible")["points_rc"][0], [5, 5])
        self.assertEqual(self.engine.calls, 1)

    def test_direction_is_not_assumed_symmetric(self):
        z = np.broadcast_to(np.arange(30, dtype=float) * 0.25, (30, 30)).copy()
        provider = RouteProvider(RouteEngine(z, self.meta))
        ascending = provider.get_route([10, 3], [10, 20], "rover", "risk_aware")
        descending = provider.get_route([10, 20], [10, 3], "rover", "risk_aware")
        self.assertEqual(ascending["status"], "found")
        self.assertEqual(descending["status"], "found")
        self.assertGreater(ascending["objective_cost"], descending["objective_cost"])

    def test_matrix_schema_and_json_safety(self):
        nodes = [
            {"id": "A", "rc": [5, 4]},
            {"id": "S1", "rc": [5, 12]},
            {"id": "B", "rc": [5, 26]},
        ]
        out = self.provider.build_cost_matrix(nodes, "rover", "shortest_feasible")
        self.assertEqual(len(out["cost_matrix"]), 3)
        self.assertEqual(out["cost_matrix"][0][0], 0)
        self.assertAlmostEqual(out["cost_matrix"][0][1], 40.0)
        self.assertTrue(out["directed"])
        json.dumps(out, allow_nan=False)

    def test_unreachable_is_none_not_infinite(self):
        risk_map = np.zeros((30, 30), dtype=float)
        risk_map[:, 15] = np.nan
        provider = RouteProvider(RouteEngine(np.zeros((30, 30)), self.meta, risk_maps={"rover": risk_map}))
        start, goal = [5, 4], [5, 26]
        leg = provider.get_route(start, goal, "rover", "risk_aware")
        self.assertEqual(leg["status"], "no_path")
        self.assertEqual(leg["objective_cost"], None)
        self.assertEqual(leg["points_rc"], [])
        matrix = provider.build_cost_matrix([{"id":"A","rc":start},{"id":"B","rc":goal}])
        self.assertIsNone(matrix["cost_matrix"][0][1])
        json.dumps(matrix, allow_nan=False)

    def test_invalid_indices_and_duplicate_ids(self):
        with self.assertRaises(ValueError):
            self.provider.get_route([5.5, 3], [10, 10])
        with self.assertRaises(ValueError):
            self.provider.get_route([5, 3], [10, 10], route_type="unconstrained_reference")
        with self.assertRaises(ValueError):
            self.provider.get_route([-1, 3], [10, 10])
        with self.assertRaises(ValueError):
            self.provider.build_cost_matrix([{"id":"A","rc":[5,2]},{"id":"A","rc":[5,3]}])

    def test_from_repo_reads_once(self):
        with tempfile.TemporaryDirectory() as tmp:
            data = Path(tmp) / "data" / "processed"
            data.mkdir(parents=True)
            np.save(data / "elevation.npy", np.zeros((30, 30), dtype=np.float32))
            np.save(data / "valid_mask.npy", np.ones((30, 30), dtype=bool))
            (data / "terrain_meta.json").write_text(json.dumps(self.meta), encoding="utf-8")
            provider = RouteProvider.from_repo(tmp)
            self.assertEqual(provider.get_route([3, 3], [3, 12])["status"], "found")


if __name__ == "__main__":
    unittest.main()
