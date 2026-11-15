"""Run from repo root: python prototypes/navigation/test_mission_api.py
Needs: pip install fastapi uvicorn httpx numpy
Covers thermal cost, the 3-route mission service and the FastAPI endpoints."""
from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
for p in (str(ROOT), str(HERE)):
    if p not in sys.path:
        sys.path.insert(0, p)

from planner_v4 import RouteEngine  # noqa: E402
from route_provider import RouteProvider  # noqa: E402
from mission_service import MissionService  # noqa: E402


class TestThermalCost(unittest.TestCase):
    def test_no_thermal_is_unchanged_and_cold_band_is_avoided(self):
        flat = np.zeros((30, 30))
        meta = {"dx_m": 5.0, "dy_m": 5.0}
        base = RouteEngine(flat, meta).compute([15, 2], [15, 27], "rover")["routes"]["risk_aware"]
        self.assertIsNone(base["mean_temp_k"])
        temp = np.full((30, 30), 196.0)
        temp[10:20, 8:22] = 189.0          # cold trap on the direct line
        temp[0, 0] = -9999.0               # NoData must never become a temperature
        eng = RouteEngine(flat, meta, thermal_k=temp)
        r = eng.compute([15, 2], [15, 27], "rover")["routes"]
        self.assertGreater(r["risk_aware"]["min_temp_k"], 189.0)   # detoured around the cold band
        self.assertEqual(r["shortest_feasible"]["min_temp_k"], 189.0)
        self.assertGreater(r["risk_aware"]["distance_m"], r["shortest_feasible"]["distance_m"])
        self.assertTrue(np.isnan(eng.thermal_k[0, 0]))


class TestMissionService(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.svc = MissionService.from_repo(ROOT)

    def test_three_routes_and_order(self):
        wps = [{"id": "S1", "rc": [75, 150]}, {"id": "S2", "rc": [45, 65]}, {"id": "S3", "rc": [150, 100]}]
        res = self.svc.plan_mission([99, 20], [99, 180], wps, "optimized")
        self.assertEqual(set(res["routes"]), {"shortest", "astronaut", "rover"})
        for key, r in res["routes"].items():
            self.assertEqual(r["status"], "found", key)
            self.assertEqual(r["visit_order"][0], "A")
            self.assertEqual(r["visit_order"][-1], "B")
            self.assertEqual(sorted(r["visit_order"][1:-1]), ["S1", "S2", "S3"])
            self.assertEqual(r["points_rc"][0], [99, 20])
            self.assertEqual(r["points_rc"][-1], [99, 180])
            for w in wps:
                self.assertIn(w["rc"], r["points_rc"])
            self.assertGreaterEqual(r["savings_vs_manual"]["objective_saved"], -1e-6)
            self.assertAlmostEqual(sum(l["distance_m"] for l in r["legs"]), r["distance_m"], places=1)
            self.assertIsNotNone(r["mean_temp_k"])
            st = json.loads((ROOT / "data/processed/thermal/thermal_grid.json").read_text())["stats_k"]
            self.assertTrue(st["min"] - 0.01 <= r["min_temp_k"] <= r["mean_temp_k"] <= r["max_temp_k"] <= st["max"] + 0.01)
        json.dumps(res, allow_nan=False)
        # Shortest really is the shortest of the 3 for a direct A->B
        direct = self.svc.plan_mission([99, 20], [99, 180], [], "optimized")["routes"]
        self.assertLessEqual(direct["shortest"]["distance_m"], direct["astronaut"]["distance_m"] + 1e-6)

    def test_manual_keeps_click_order(self):
        wps = [{"id": "S1", "rc": [75, 150]}, {"id": "S2", "rc": [45, 65]}]
        r = self.svc.plan_mission([99, 20], [99, 180], wps, "manual")["routes"]["rover"]
        self.assertEqual(r["visit_order"], ["A", "S1", "S2", "B"])

    def test_bad_input(self):
        with self.assertRaises(ValueError):
            self.svc.plan_mission([999, 0], [1, 1], [])
        with self.assertRaises(ValueError):
            self.svc.plan_mission([1, 1], [2, 2], [{"id": "A", "rc": [3, 3]}])


try:
    from fastapi.testclient import TestClient
    import route_api
except Exception:  # fastapi/httpx not installed
    TestClient = None


@unittest.skipIf(TestClient is None, "fastapi/httpx not installed")
class TestApi(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.client = TestClient(route_api.app)

    def test_endpoints(self):
        c = self.client
        h = c.get("/api/health").json()
        self.assertEqual(h["status"], "ok")
        self.assertTrue(h["thermal_in_cost"])
        self.assertEqual(c.get("/api/terrain/metadata").json()["grid_shape"], [198, 200])
        r = c.post("/api/route", json={"start_rc": [99, 20], "goal_rc": [99, 180], "profile": "rover"})
        self.assertEqual(r.status_code, 200)
        self.assertEqual(r.json()["routes"]["risk_aware"]["status"], "found")
        m = c.post("/api/mission", json={"start_rc": [99, 20], "goal_rc": [99, 180],
                                         "waypoints": [{"id": "S1", "rc": [45, 65]}], "order_mode": "optimized"})
        self.assertEqual(m.status_code, 200)
        self.assertEqual(m.json()["routes"]["astronaut"]["visit_order"], ["A", "S1", "B"])
        bad = c.post("/api/mission", json={"start_rc": [500, 20], "goal_rc": [99, 180]})
        self.assertEqual(bad.status_code, 422)
        cors = c.options("/api/mission", headers={"Origin": "http://127.0.0.1:8000",
                                                  "Access-Control-Request-Method": "POST"})
        self.assertEqual(cors.headers.get("access-control-allow-origin"), "http://127.0.0.1:8000")


if __name__ == "__main__":
    unittest.main()
