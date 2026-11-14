from __future__ import annotations

import itertools
import json
import math
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Callable, Dict, List, Optional, Protocol, Tuple


GRID_ROWS = 198
GRID_COLS = 200
CELL_SIZE_M = 5.0
MAX_SCIENCE_TARGETS = 5


@dataclass
class LegResult:
    """Navigation V4 (veya test router) tarafından döndürülen tekil rota parçası."""
    from_id: str
    to_id: str
    status: str  # "found", "no_path", "blocked_endpoint"
    points_rc: List[List[int]]
    distance_m: float
    weighted_cost_m_equiv: float
    reason: Optional[str] = None

    def to_dict(self) -> dict:
        data = {
            "from": self.from_id,
            "to": self.to_id,
            "status": self.status,
            "points_rc": self.points_rc,
            "distance_m": round(self.distance_m, 2),
            "weighted_cost_m_equiv": round(self.weighted_cost_m_equiv, 2),
        }
        if self.reason:
            data["reason"] = self.reason
        return data


class PairwiseRouteProvider(Protocol):
    """Navigation V4 ile mission_planner arasındaki standart arayüz sözleşmesi."""
    def compute_leg(
        self,
        from_id: str,
        to_id: str,
        start_rc: List[int],
        goal_rc: List[int],
        profile: str,
        route_type: str,
    ) -> LegResult:
        ...


class NavigationV4Adapter:
    """
    Aşama 2 Entegrasyon Katmanı:
    Mevcut `planner_v4.py` içindeki A* fonksiyonunu sarmalar.
    V4 fonksiyonu paylaşıldığında `v4_plan_fn` parametresiyle doğrudan bağlanır.
    """
    def __init__(self, v4_plan_fn: Callable[..., dict]):
        self.v4_plan_fn = v4_plan_fn

    def compute_leg(
        self,
        from_id: str,
        to_id: str,
        start_rc: List[int],
        goal_rc: List[int],
        profile: str,
        route_type: str,
    ) -> LegResult:
        raw = self.v4_plan_fn(
            start_rc=tuple(start_rc),
            goal_rc=tuple(goal_rc),
            profile=profile,
            route_type=route_type,
        )
        status = raw.get("status", "found" if raw.get("found", False) else "no_path")
        if status != "found":
            return LegResult(
                from_id=from_id,
                to_id=to_id,
                status=status,
                points_rc=[],
                distance_m=0.0,
                weighted_cost_m_equiv=0.0,
                reason=raw.get("reason", f"Navigation V4 rota bulamadı ({status})."),
            )

        points = [list(pt) for pt in raw["points_rc"]]
        return LegResult(
            from_id=from_id,
            to_id=to_id,
            status="found",
            points_rc=points,
            distance_m=float(raw["distance_m"]),
            weighted_cost_m_equiv=float(raw["weighted_cost_m_equiv"]),
        )


class DemoTerrainRouter:
    """
    Aşama 1 Bağımsız Test Yönlendiricisi (198x200 grid, 5m hücre aralığı):
    - Yönlü (asimetrik) tırmanış maliyeti üretir (doğu/kuzey yönü yokuş yukarı).
    - Rover ve Astronaut profilleri için farklı risk çarpanları uygular.
    - Tanımlı engel bölgeleri için 'blocked_endpoint' veya 'no_path' döndürür.
    """
    def __init__(
        self,
        blocked_cells: Optional[set[Tuple[int, int]]] = None,
        unreachable_pairs: Optional[set[Tuple[str, str]]] = None,
    ):
        self.blocked_cells = blocked_cells or set()
        self.unreachable_pairs = unreachable_pairs or set()

    @staticmethod
    def _elevation_m(rc: List[int]) -> float:
        # Örnek sentetik kot modeli: satır küçüldükçe ve sütun büyüdükçe rakım artar
        r, c = rc
        return (GRID_ROWS - r) * 0.4 + c * 0.6

    def compute_leg(
        self,
        from_id: str,
        to_id: str,
        start_rc: List[int],
        goal_rc: List[int],
        profile: str,
        route_type: str,
    ) -> LegResult:
        if tuple(start_rc) in self.blocked_cells or tuple(goal_rc) in self.blocked_cells:
            return LegResult(
                from_id=from_id,
                to_id=to_id,
                status="blocked_endpoint",
                points_rc=[],
                distance_m=0.0,
                weighted_cost_m_equiv=0.0,
                reason="Başlangıç veya hedef hücresi geçilmez bölgede.",
            )

        if (from_id, to_id) in self.unreachable_pairs:
            return LegResult(
                from_id=from_id,
                to_id=to_id,
                status="no_path",
                points_rc=[],
                distance_m=0.0,
                weighted_cost_m_equiv=0.0,
                reason=f"{from_id} -> {to_id} arasında geçilebilir arazi koridoru yok.",
            )

        # Basit 8-yönlü ızgara adımlama ile ara hücreleri oluştur
        points: List[List[int]] = [list(start_rc)]
        curr_r, curr_c = start_rc
        goal_r, goal_c = goal_rc

        while (curr_r, curr_c) != (goal_r, goal_c):
            step_r = 0 if curr_r == goal_r else (1 if goal_r > curr_r else -1)
            step_c = 0 if curr_c == goal_c else (1 if goal_c > curr_c else -1)
            curr_r += step_r
            curr_c += step_c
            if (curr_r, curr_c) in self.blocked_cells:
                return LegResult(
                    from_id=from_id,
                    to_id=to_id,
                    status="no_path",
                    points_rc=[],
                    distance_m=0.0,
                    weighted_cost_m_equiv=0.0,
                    reason="Arazi üzerinde geçilmez engele çarpıldı.",
                )
            points.append([curr_r, curr_c])

        # Geometrik mesafe (m)
        dist_m = 0.0
        for i in range(1, len(points)):
            dr = points[i][0] - points[i - 1][0]
            dc = points[i][1] - points[i - 1][1]
            dist_m += math.hypot(dr, dc) * CELL_SIZE_M

        # Yönlü tırmanış ve profil ağırlıklı maliyet
        dz = self._elevation_m(goal_rc) - self._elevation_m(start_rc)
        climb_m = max(0.0, dz)
        descent_m = max(0.0, -dz)

        if profile == "astronaut":
            # Astronot dik tırmanışa daha duyarlı, orta pürüzlülüğe daha esnek
            risk_penalty = climb_m * 14.0 + descent_m * 3.0 + dist_m * 0.35
        else:
            # Rover pürüzlülük ve enerji tüketimine duyarlı
            risk_penalty = climb_m * 9.0 + descent_m * 2.0 + dist_m * 0.55

        weighted_cost = dist_m + risk_penalty
        return LegResult(
            from_id=from_id,
            to_id=to_id,
            status="found",
            points_rc=points,
            distance_m=dist_m,
            weighted_cost_m_equiv=weighted_cost,
        )


class MultiStopMissionPlanner:
    """
    Birden fazla bilimsel hedef (Science Target) için yönlü maliyet matrisi
    ve permütasyon tabanlı görev sırası optimizasyonu gerçekleştirir.
    """
    def __init__(self, router: PairwiseRouteProvider):
        self.router = router
        # Önbellek anahtarı: (from_id, to_id, profile, route_type)
        self._leg_cache: Dict[Tuple[str, str, str, str], LegResult] = {}

    def clear_cache(self) -> None:
        self._leg_cache.clear()

    @staticmethod
    def _validate_cell(rc: List[int], label: str) -> None:
        if not isinstance(rc, (list, tuple)) or len(rc) != 2:
            raise ValueError(f"Geçersiz koordinat formatı ({label}): [row, col] bekleniyor.")
        r, c = rc
        if not (0 <= r < GRID_ROWS and 0 <= c < GRID_COLS):
            raise ValueError(f"Harita sınırı dışında koordinat ({label}): {rc}")

    def _validate_request(self, request: dict) -> Tuple[List[int], List[int], List[dict], str, str, str]:
        start_rc = request.get("start_rc")
        goal_rc = request.get("goal_rc")
        targets = request.get("science_targets", [])
        profile = request.get("profile", "rover")
        route_type = request.get("route_type", "risk_aware")
        mode = request.get("mode", "optimized")

        self._validate_cell(start_rc, "A (start_rc)")
        self._validate_cell(goal_rc, "B (goal_rc)")

        if profile not in ("rover", "astronaut"):
            raise ValueError(f"Geçersiz profil: '{profile}'. 'rover' veya 'astronaut' olmalı.")
        if route_type not in ("shortest_feasible", "risk_aware"):
            raise ValueError(f"Geçersiz route_type: '{route_type}'. 'shortest_feasible' veya 'risk_aware' olmalı.")
        if mode not in ("manual", "optimized"):
            raise ValueError(f"Geçersiz mode: '{mode}'. 'manual' veya 'optimized' olmalı.")

        if not (1 <= len(targets) <= MAX_SCIENCE_TARGETS):
            raise ValueError(f"Science Target sayısı 1 ile {MAX_SCIENCE_TARGETS} arasında olmalıdır (verilen: {len(targets)}).")

        seen_ids = {"A", "B"}
        seen_coords = {tuple(start_rc), tuple(goal_rc)}
        for t in targets:
            tid = t.get("id")
            trc = t.get("rc")
            if not tid or tid in seen_ids:
                raise ValueError(f"Tekrarlanan veya geçersiz hedef kimliği tespit edildi: '{tid}'")
            self._validate_cell(trc, tid)
            if tuple(trc) in seen_coords:
                raise ValueError(f"Tekrarlanan hedef koordinatı tespit edildi ({tid}: {trc}).")
            seen_ids.add(tid)
            seen_coords.add(tuple(trc))

        return list(start_rc), list(goal_rc), targets, profile, route_type, mode

    def _get_leg(
        self,
        from_id: str,
        to_id: str,
        node_coords: Dict[str, List[int]],
        profile: str,
        route_type: str,
    ) -> LegResult:
        cache_key = (from_id, to_id, profile, route_type)
        if cache_key not in self._leg_cache:
            self._leg_cache[cache_key] = self.router.compute_leg(
                from_id=from_id,
                to_id=to_id,
                start_rc=node_coords[from_id],
                goal_rc=node_coords[to_id],
                profile=profile,
                route_type=route_type,
            )
        return self._leg_cache[cache_key]

    @staticmethod
    def _merge_points_rc(legs: List[LegResult]) -> List[List[int]]:
        """Ardışık rota parçalarını birleştirirken ortak bağlantı hücresini iki kez eklemez."""
        merged: List[List[int]] = []
        for idx, leg in enumerate(legs):
            if idx == 0:
                merged.extend(leg.points_rc)
            else:
                if leg.points_rc and merged and leg.points_rc[0] == merged[-1]:
                    merged.extend(leg.points_rc[1:])
                else:
                    merged.extend(leg.points_rc)
        return merged

    def plan_mission(self, request: dict) -> dict:
        try:
            start_rc, goal_rc, targets, profile, route_type, mode = self._validate_request(request)
        except ValueError as exc:
            return {
                "status": "invalid_input",
                "reason": str(exc),
                "profile": request.get("profile"),
                "route_type": request.get("route_type"),
                "mode": request.get("mode"),
                "visit_order": [],
                "total_distance_m": None,
                "total_weighted_cost_m_equiv": None,
                "points_rc": [],
                "legs": [],
            }

        node_coords: Dict[str, List[int]] = {"A": start_rc, "B": goal_rc}
        target_ids: List[str] = []
        for t in targets:
            node_coords[t["id"]] = list(t["rc"])
            target_ids.append(t["id"])

        # Ziyaret edilecek aday hedef sıralamalarını üret
        if mode == "manual":
            custom_order = request.get("manual_order")
            if custom_order is not None:
                if set(custom_order) != set(target_ids) or len(custom_order) != len(target_ids):
                    return {
                        "status": "invalid_input",
                        "reason": "manual_order listedeki tüm Science Target ID'lerini tam bir kez içermelidir.",
                        "profile": profile,
                        "route_type": route_type,
                        "mode": mode,
                        "visit_order": [],
                        "total_distance_m": None,
                        "total_weighted_cost_m_equiv": None,
                        "points_rc": [],
                        "legs": [],
                    }
                candidate_orders = [tuple(custom_order)]
            else:
                candidate_orders = [tuple(target_ids)]
        else:
            # Optimized Mode: Tüm permütasyonları değerlendir (Brute Force)
            candidate_orders = list(itertools.permutations(target_ids))

        best_sequence: Optional[List[str]] = None
        best_legs: Optional[List[LegResult]] = None
        best_objective_val = float("inf")
        failure_legs: List[LegResult] = []

        for order in candidate_orders:
            full_seq = ["A", *order, "B"]
            seq_legs: List[LegResult] = []
            seq_valid = True
            total_dist = 0.0
            total_weighted = 0.0

            for u, v in zip(full_seq[:-1], full_seq[1:]):
                leg = self._get_leg(u, v, node_coords, profile, route_type)
                seq_legs.append(leg)
                if leg.status != "found":
                    seq_valid = False
                    failure_legs.append(leg)
                    break
                total_dist += leg.distance_m
                total_weighted += leg.weighted_cost_m_equiv

            if not seq_valid:
                continue

            objective_val = total_dist if route_type == "shortest_feasible" else total_weighted
            if objective_val < best_objective_val:
                best_objective_val = objective_val
                best_sequence = full_seq
                best_legs = seq_legs

        if best_sequence is None or best_legs is None:
            has_blocked = any(lg.status == "blocked_endpoint" for lg in failure_legs)
            fail_status = "blocked_endpoint" if has_blocked else "no_path"
            last_fail = failure_legs[-1] if failure_legs else None
            return {
                "status": fail_status,
                "reason": last_fail.reason if last_fail else "Geçerli çoklu hedef rotası oluşturulamadı.",
                "profile": profile,
                "route_type": route_type,
                "mode": mode,
                "visit_order": [],
                "total_distance_m": None,
                "total_weighted_cost_m_equiv": None,
                "points_rc": [],
                "legs": [lg.to_dict() for lg in failure_legs],
            }

        total_distance_m = round(sum(lg.distance_m for lg in best_legs), 2)
        total_weighted_cost = round(sum(lg.weighted_cost_m_equiv for lg in best_legs), 2)
        merged_points_rc = self._merge_points_rc(best_legs)

        return {
            "status": "found",
            "profile": profile,
            "route_type": route_type,
            "mode": mode,
            "visit_order": best_sequence,
            "total_distance_m": total_distance_m,
            "total_weighted_cost_m_equiv": total_weighted_cost,
            "points_rc": merged_points_rc,
            "legs": [lg.to_dict() for lg in best_legs],
        }


def save_mission_output(result: dict, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    sample_request = {
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

    planner = MultiStopMissionPlanner(router=DemoTerrainRouter())
    mission_result = planner.plan_mission(sample_request)

    out_file = Path(__file__).resolve().parent / "output" / "mission_route.json"
    save_mission_output(mission_result, out_file)

    print(f"Durum           : {mission_result['status']}")
    print(f"Ziyaret Sırası  : {' -> '.join(mission_result['visit_order'])}")
    print(f"Toplam Mesafe   : {mission_result['total_distance_m']} m")
    print(f"Ağırlıklı Maliyet: {mission_result['total_weighted_cost_m_equiv']}")
    print(f"Birleşik Nokta  : {len(mission_result['points_rc'])} hücre")
    print(f"Çıktı Kaydedildi: {out_file}")