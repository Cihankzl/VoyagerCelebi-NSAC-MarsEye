import unittest
from pathlib import Path
from mission_planner import (
    DemoTerrainRouter,
    LegResult,
    MultiStopMissionPlanner,
    save_mission_output,
)


class CustomMatrixRouter:
    """Yönlü maliyet ve özel engel senaryolarını deterministik test etmek için yapay maliyet matrisi."""
    def __init__(self, edge_table: dict[tuple[str, str], tuple[float, float]], blocked: set[str] | None = None):
        self.edge_table = edge_table
        self.blocked = blocked or set()
        self.call_count = 0

    def compute_leg(
        self,
        from_id: str,
        to_id: str,
        start_rc: list[int],
        goal_rc: list[int],
        profile: str,
        route_type: str,
    ) -> LegResult:
        self.call_count += 1
        if from_id in self.blocked or to_id in self.blocked:
            return LegResult(from_id, to_id, "blocked_endpoint", [], 0.0, 0.0, reason="Hedef engelli.")
        if (from_id, to_id) not in self.edge_table:
            return LegResult(from_id, to_id, "no_path", [], 0.0, 0.0, reason="Bağlantı yok.")

        dist_m, w_cost = self.edge_table[(from_id, to_id)]
        mid_rc = [(start_rc[0] + goal_rc[0]) // 2, (start_rc[1] + goal_rc[1]) // 2]
        pts = [list(start_rc), mid_rc, list(goal_rc)]
        return LegResult(from_id, to_id, "found", pts, dist_m, w_cost)


class TestMultiStopMissionPlanner(unittest.TestCase):
    def test_1_single_science_target(self):
        """Senaryo 1: Tek Science Target (A -> S1 -> B) ve ortak hücre birleştirme kontrolü."""
        planner = MultiStopMissionPlanner(DemoTerrainRouter())
        req = {
            "start_rc": [99, 20],
            "goal_rc": [99, 180],
            "science_targets": [{"id": "S1", "rc": [80, 100]}],
            "profile": "rover",
            "route_type": "risk_aware",
            "mode": "optimized",
        }
        res = planner.plan_mission(req)
        self.assertEqual(res["status"], "found")
        self.assertEqual(res["visit_order"], ["A", "S1", "B"])
        self.assertEqual(len(res["legs"]), 2)
        # Ortak düğüm S1 ([80, 100]) birleşik listede yalnızca 1 kez bulunmalı
        self.assertEqual(res["points_rc"].count([80, 100]), 1)

    def test_2_three_targets_permutation_optimization(self):
        """Senaryo 2: Üç hedef için tüm permütasyonlar taranıp en düşük maliyetli sıra bulunmalı."""
        edges = {
            ("A", "S1"): (300, 900), ("A", "S2"): (200, 400), ("A", "S3"): (500, 1500),
            ("S1", "S2"): (400, 1200), ("S1", "S3"): (250, 500), ("S1", "B"): (350, 900),
            ("S2", "S1"): (220, 450), ("S2", "S3"): (600, 1800), ("S2", "B"): (500, 1400),
            ("S3", "S1"): (300, 800), ("S3", "S2"): (450, 1100), ("S3", "B"): (200, 420),
        }
        # En iyi yol: A -> S2 (400) -> S1 (450) -> S3 (500) -> B (420) = 1770
        router = CustomMatrixRouter(edges)
        planner = MultiStopMissionPlanner(router)
        req = {
            "start_rc": [99, 20],
            "goal_rc": [99, 180],
            "science_targets": [
                {"id": "S1", "rc": [45, 65]},
                {"id": "S2", "rc": [110, 100]},
                {"id": "S3", "rc": [75, 150]},
            ],
            "profile": "rover",
            "route_type": "risk_aware",
            "mode": "optimized",
        }
        res = planner.plan_mission(req)
        self.assertEqual(res["status"], "found")
        self.assertEqual(res["visit_order"], ["A", "S2", "S1", "S3", "B"])
        self.assertEqual(res["total_weighted_cost_m_equiv"], 1770.0)
        # Önbellek kontrolü: 3 hedef için en fazla 3*(3+1) = 12 yönlü A* çağrısı yapılmalı
        self.assertEqual(router.call_count, 12)

    def test_3_rover_vs_astronaut_profiles(self):
        """Senaryo 3: Rover ve Astronaut profillerinde farklı maliyetler üretilmeli."""
        planner = MultiStopMissionPlanner(DemoTerrainRouter())
        base_req = {
            "start_rc": [120, 20],
            "goal_rc": [50, 180],
            "science_targets": [
                {"id": "S1", "rc": [40, 70]},
                {"id": "S2", "rc": [130, 110]},
                {"id": "S3", "rc": [80, 140]},
            ],
            "route_type": "risk_aware",
            "mode": "optimized",
        }
        res_rover = planner.plan_mission({**base_req, "profile": "rover"})
        res_astro = planner.plan_mission({**base_req, "profile": "astronaut"})

        self.assertEqual(res_rover["status"], "found")
        self.assertEqual(res_astro["status"], "found")
        self.assertNotEqual(res_rover["total_weighted_cost_m_equiv"], res_astro["total_weighted_cost_m_equiv"])

    def test_4_shortest_feasible_vs_risk_aware(self):
        """Senaryo 4: Seçilen maliyet türü (mesafe vs riskli maliyet) sıralamayı değiştirmeli."""
        # A -> S1 -> S2 -> B: Kısa mesafe (100+100+100=300m) ama çok dik/riskli (3000)
        # A -> S2 -> S1 -> B: Uzun mesafe (200+200+200=600m) ama güvenli/düz (900)
        edges = {
            ("A", "S1"): (100, 1000), ("S1", "S2"): (100, 1000), ("S2", "B"): (100, 1000),
            ("A", "S2"): (200, 300), ("S2", "S1"): (200, 300), ("S1", "B"): (200, 300),
        }
        planner = MultiStopMissionPlanner(CustomMatrixRouter(edges))
        req = {
            "start_rc": [99, 20],
            "goal_rc": [99, 180],
            "science_targets": [{"id": "S1", "rc": [50, 80]}, {"id": "S2", "rc": [120, 120]}],
            "profile": "rover",
            "mode": "optimized",
        }
        res_short = planner.plan_mission({**req, "route_type": "shortest_feasible"})
        res_risk = planner.plan_mission({**req, "route_type": "risk_aware"})

        self.assertEqual(res_short["visit_order"], ["A", "S1", "S2", "B"])
        self.assertEqual(res_short["total_distance_m"], 300.0)

        self.assertEqual(res_risk["visit_order"], ["A", "S2", "S1", "B"])
        self.assertEqual(res_risk["total_weighted_cost_m_equiv"], 900.0)

    def test_5_impassable_connection_and_blocked_endpoint(self):
        """Senaryo 5: Geçilemeyen bağlantı veya engelli hedefte uygun hata durumu dönmeli."""
        router_blocked = DemoTerrainRouter(blocked_cells={(45, 65)})
        planner = MultiStopMissionPlanner(router_blocked)
        req = {
            "start_rc": [99, 20],
            "goal_rc": [99, 180],
            "science_targets": [{"id": "S1", "rc": [45, 65]}],
            "profile": "rover",
            "route_type": "risk_aware",
            "mode": "optimized",
        }
        res = planner.plan_mission(req)
        self.assertEqual(res["status"], "blocked_endpoint")

    def test_6_duplicate_target_prevention(self):
        """Senaryo 6: Aynı hedef yanlışlıkla iki kez verilmişse reddedilmeli."""
        planner = MultiStopMissionPlanner(DemoTerrainRouter())
        req = {
            "start_rc": [99, 20],
            "goal_rc": [99, 180],
            "science_targets": [
                {"id": "S1", "rc": [45, 65]},
                {"id": "S1", "rc": [75, 150]},
            ],
            "profile": "rover",
            "route_type": "risk_aware",
            "mode": "optimized",
        }
        res = planner.plan_mission(req)
        self.assertEqual(res["status"], "invalid_input")

    def test_7_directed_asymmetric_costs(self):
        """Senaryo 7: S1 -> S2 ve S2 -> S1 yönlü maliyetleri farklıysa düşük yön seçilmeli."""
        edges = {
            ("A", "S1"): (100, 100), ("A", "S2"): (100, 100),
            ("S1", "S2"): (100, 120),   # S1 -> S2 ucuz (iniş)
            ("S2", "S1"): (100, 950),   # S2 -> S1 pahalı (dik tırmanış)
            ("S1", "B"): (100, 100), ("S2", "B"): (100, 100),
        }
        planner = MultiStopMissionPlanner(CustomMatrixRouter(edges))
        req = {
            "start_rc": [99, 20],
            "goal_rc": [99, 180],
            "science_targets": [{"id": "S1", "rc": [60, 80]}, {"id": "S2", "rc": [110, 120]}],
            "profile": "astronaut",
            "route_type": "risk_aware",
            "mode": "optimized",
        }
        res = planner.plan_mission(req)
        self.assertEqual(res["visit_order"], ["A", "S1", "S2", "B"])
        self.assertEqual(res["total_weighted_cost_m_equiv"], 320.0)


if __name__ == "__main__":
    # Örnek JSON çıktısını da diske yazdırıp testleri çalıştır
    demo_planner = MultiStopMissionPlanner(DemoTerrainRouter())
    demo_out = demo_planner.plan_mission(
        {
            "start_rc": [99, 20],
            "goal_rc": [99, 180],
            "science_targets": [
                {"id": "S1", "rc": [45, 65]},
                {"id": "S2", "rc": [110, 100]},
                {"id": "S3", "rc": [75, 150]},
            ],
            "profile": "rover",
            "route_type": "risk_aware",
            "mode": "optimized",
        }
    )
    save_mission_output(demo_out, Path(__file__).resolve().parent / "output" / "mission_route.json")
    unittest.main(verbosity=2)