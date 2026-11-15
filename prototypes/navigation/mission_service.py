"""MarsEye — 3 primary routes for single or multi-stop missions.

Glue between the existing modules (nothing here reimplements A*):
  * prototypes/navigation/planner_v4.py      RouteEngine (A*, slope/roughness/ascent/thermal cost)
  * prototypes/navigation/route_provider.py  RouteProvider (directed, cached legs)
  * v4/mission_planning/mission_planner.py   MultiStopMissionPlanner (visit-order optimisation)

The application compares three routes side by side:
  shortest   -> profile "astronaut", route_type "shortest_feasible" (pure distance; loosest feasible limits)
  astronaut  -> profile "astronaut", route_type "risk_aware"
  rover      -> profile "rover",     route_type "risk_aware"

Usage (repo root):
  python -m prototypes.navigation.mission_service
"""
from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path
from typing import Any, Sequence

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:  # python -m prototypes.navigation.mission_service / package import
    from .planner_v4 import PROFILES
    from .route_provider import RouteProvider
except ImportError:  # python prototypes/navigation/route_api.py (script directory on sys.path)
    from planner_v4 import PROFILES  # type: ignore
    from route_provider import RouteProvider  # type: ignore

from v4.mission_planning.mission_planner import (  # noqa: E402
    MAX_SCIENCE_TARGETS,
    MultiStopMissionPlanner,
    NavigationV4Adapter,
)

PRIMARY_ROUTES: dict[str, dict[str, str]] = {
    "shortest": {"label": "Shortest Route", "profile": "astronaut",
                 "route_type": "shortest_feasible", "color": "#61c8ff"},
    "astronaut": {"label": "Astronaut Safe Route", "profile": "astronaut",
                  "route_type": "risk_aware", "color": "#ad86ff"},
    "rover": {"label": "Rover Optimal Route", "profile": "rover",
              "route_type": "risk_aware", "color": "#42d5bd"},
}

PRESET_MISSION = {
    "start_rc": [99, 20],
    "goal_rc": [99, 180],
    "waypoints": [
        # Deliberately clicked out of order so "optimized" shows a real saving.
        {"id": "S1", "rc": [75, 150]},
        {"id": "S2", "rc": [45, 65]},
        {"id": "S3", "rc": [150, 100]},
    ],
}

METRIC_KEYS = ("distance_m", "mean_slope_deg", "max_slope_deg", "max_edge_grade_deg",
               "mean_roughness_m", "elevation_gain_m", "elevation_loss_m", "weighted_cost_m_equiv",
               "mean_temp_k", "min_temp_k", "max_temp_k", "mean_temp_c", "min_temp_c", "max_temp_c",
               "mean_cold_index_0_1")


def _rc(value: object, label: str) -> list[int]:
    if not isinstance(value, (list, tuple)) or len(value) != 2 or any(type(v) is not int for v in value):
        raise ValueError(f"{label} must be two integers [row, col]")
    return [int(value[0]), int(value[1])]


class MissionService:
    def __init__(self, provider: RouteProvider):
        self.provider = provider
        self.engine = provider.engine
        self._lock = threading.Lock()

    @classmethod
    def from_repo(cls, repo_root: str | Path | None = None, **kwargs) -> "MissionService":
        return cls(RouteProvider.from_repo(repo_root, **kwargs))

    # ------------------------------------------------------------------ helpers
    def _allowed(self, profile_name: str) -> np.ndarray:
        p = PROFILES[profile_name]
        allowed = self.engine.valid & (self.engine.slope < p.max_cell_slope_deg)
        risk = self.engine.risk_maps.get(profile_name)
        if risk is not None:
            allowed &= np.isfinite(risk)
        return allowed

    def _summary(self, points: list[list[int]], profile_name: str) -> dict[str, Any]:
        e = self.engine
        s = e._summarize([tuple(p) for p in points], PROFILES[profile_name],
                         e.risk_maps.get(profile_name), self._allowed(profile_name), 0.0)
        return {k: s.get(k) for k in METRIC_KEYS}

    def _leg_fn(self, start_rc, goal_rc, profile, route_type):
        return self.provider.get_route(list(start_rc), list(goal_rc), profile, route_type)

    def _check(self, start_rc, goal_rc, waypoints):
        rows, cols = self.engine.elevation.shape
        start, goal = _rc(start_rc, "start_rc"), _rc(goal_rc, "goal_rc")
        clean = []
        seen = {"A", "B"}
        for i, w in enumerate(waypoints or []):
            wid = w.get("id") if isinstance(w, dict) else None
            if not isinstance(wid, str) or not wid.strip() or wid in seen:
                raise ValueError(f"waypoints[{i}].id must be unique and not A/B")
            seen.add(wid)
            clean.append({"id": wid, "rc": _rc(w.get("rc"), f"waypoints[{i}].rc")})
        if len(clean) > MAX_SCIENCE_TARGETS:
            raise ValueError(f"At most {MAX_SCIENCE_TARGETS} science waypoints are supported")
        for label, (r, c) in [("start_rc", start), ("goal_rc", goal)] + [(w["id"], w["rc"]) for w in clean]:
            if not (0 <= r < rows and 0 <= c < cols):
                raise ValueError(f"{label} {[r, c]} outside grid rows 0..{rows-1}, cols 0..{cols-1}")
        return start, goal, clean

    # ------------------------------------------------------------------ main API
    def plan_mission(self, start_rc: Sequence[int], goal_rc: Sequence[int],
                     waypoints: Sequence[dict] | None = None, order_mode: str = "optimized") -> dict[str, Any]:
        if order_mode not in ("optimized", "manual"):
            raise ValueError("order_mode must be 'optimized' or 'manual'")
        start, goal, wps = self._check(start_rc, goal_rc, waypoints)
        t0 = time.perf_counter()
        with self._lock:
            routes = {key: self._plan_one(key, start, goal, wps, order_mode) for key in PRIMARY_ROUTES}
        e = self.engine
        return {
            "schema_version": "1.1",
            "terrain_id": "gale_5m_v1" if e.elevation.shape == (198, 200) else "custom_grid",
            "grid_shape": list(e.elevation.shape),
            "dx_m": e.dx, "dy_m": e.dy,
            "coordinate_system": "row_col_0_based_origin_upper_left",
            "start_rc": start, "goal_rc": goal, "waypoints": wps, "order_mode": order_mode,
            "thermal_in_cost": e.cold_index is not None,
            "risk_source": "DEM_slope_roughness_proxy" + ("+THEMIS_thermal" if e.cold_index is not None else ""),
            "experimental": True,
            "route_definitions": PRIMARY_ROUTES,
            "routes": routes,
            "runtime_ms": round(1000 * (time.perf_counter() - t0), 1),
            "engine": "python_fastapi",
        }

    def _plan_one(self, key: str, start, goal, wps, order_mode) -> dict[str, Any]:
        spec = PRIMARY_ROUTES[key]
        profile, route_type = spec["profile"], spec["route_type"]
        base = {"key": key, "label": spec["label"], "color": spec["color"], "profile": profile,
                "route_type": route_type,
                "objective_unit": "m" if route_type == "shortest_feasible" else "model_weighted_m_equiv"}

        if not wps:
            leg = self.provider.get_route(start, goal, profile, route_type)
            if leg["status"] != "found":
                return {**base, "status": leg["status"], "visit_order": [], "points_rc": [], "legs": []}
            pts = leg["points_rc"]
            metrics = self._summary(pts, profile)
            return {**base, "status": "found", "visit_order": ["A", "B"], "points_rc": pts,
                    **metrics, "objective_cost": leg["objective_cost"],
                    "legs": [{"from": "A", "to": "B", "status": "found", "point_index_range": [0, len(pts) - 1],
                              **metrics}],
                    "manual": None, "optimized": None, "savings_vs_manual": None}

        planner = MultiStopMissionPlanner(NavigationV4Adapter(self._leg_fn))
        req = {"start_rc": start, "goal_rc": goal, "science_targets": wps,
               "profile": profile, "route_type": route_type}
        opt = planner.plan_mission({**req, "mode": "optimized"})
        man = planner.plan_mission({**req, "mode": "manual"})  # click order, reuses cached legs
        chosen = opt if order_mode == "optimized" else man

        def objective(res):
            if res.get("status") != "found":
                return None
            return res["total_distance_m"] if route_type == "shortest_feasible" else res["total_weighted_cost_m_equiv"]

        def brief(res):
            return {"status": res.get("status"), "visit_order": res.get("visit_order", []),
                    "objective_cost": objective(res), "distance_m": res.get("total_distance_m")}

        savings = None
        if objective(opt) is not None and objective(man) is not None:
            saved = objective(man) - objective(opt)
            savings = {
                "objective_saved": round(saved, 2),
                "objective_saved_pct": round(100 * saved / objective(man), 2) if objective(man) else 0.0,
                "distance_saved_m": round(man["total_distance_m"] - opt["total_distance_m"], 2),
                "manual_visit_order": man["visit_order"],
                "optimized_visit_order": opt["visit_order"],
            }

        if chosen.get("status") != "found":
            return {**base, "status": chosen.get("status"), "reason": chosen.get("reason"),
                    "visit_order": [], "points_rc": [], "legs": chosen.get("legs", []),
                    "manual": brief(man), "optimized": brief(opt), "savings_vs_manual": savings}

        merged = chosen["points_rc"]
        legs, idx = [], 0
        for leg in chosen["legs"]:
            n = len(leg["points_rc"])
            legs.append({"from": leg["from"], "to": leg["to"], "status": leg["status"],
                         "point_index_range": [idx, idx + n - 1], **self._summary(leg["points_rc"], profile)})
            idx += n - 1
        return {**base, "status": "found", "visit_order": chosen["visit_order"], "points_rc": merged,
                **self._summary(merged, profile), "objective_cost": round(objective(chosen), 3),
                "legs": legs, "manual": brief(man), "optimized": brief(opt), "savings_vs_manual": savings}


def main() -> None:
    svc = MissionService.from_repo()
    res = svc.plan_mission(PRESET_MISSION["start_rc"], PRESET_MISSION["goal_rc"], PRESET_MISSION["waypoints"])
    for k, r in res["routes"].items():
        print(f"{k:10s} {r['status']:10s} {' -> '.join(r['visit_order']):28s} "
              f"{r.get('distance_m')} m  T={r.get('mean_temp_k')} K  saved={r.get('savings_vs_manual')}")
    print("runtime", res["runtime_ms"], "ms")


if __name__ == "__main__":
    main()
