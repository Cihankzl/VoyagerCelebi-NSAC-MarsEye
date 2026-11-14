"""MarsEye: real Gale DEM end-to-end Mission Planner + Route Provider test.

Run from repository root:
  python prototypes/navigation/test_real_mission_integration.py
  python prototypes/navigation/test_real_mission_integration.py --profile astronaut

Independent integration smoke test: does NOT edit teammate's mission_planner.py.
Selects targets directly on a known valid V4 route; not geological science claims.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
MISSION_PATH = ROOT / "v4" / "mission_planning"
if not MISSION_PATH.is_dir():
    raise SystemExit(f"Missing teammate's module: {MISSION_PATH}")
sys.path.insert(0, str(MISSION_PATH))

from route_provider import RouteProvider
from mission_planner import MultiStopMissionPlanner, NavigationV4Adapter


def assert_valid_mission(result: dict, start: list[int], goal: list[int], target_ids: list[str]) -> None:
    if result.get("status") != "found":
        raise AssertionError(f"Mission failed: {result.get('status')}: {result.get('reason')}")
    order = result["visit_order"]
    if order[0] != "A" or order[-1] != "B" or sorted(order[1:-1]) != sorted(target_ids):
        raise AssertionError(f"Wrong visit order: {order}")
    legs = result["legs"]
    if len(legs) != len(target_ids) + 1:
        raise AssertionError("Wrong number of A* legs")
    if result["points_rc"][0] != start or result["points_rc"][-1] != goal:
        raise AssertionError("Combined route endpoints do not match")
    n_points = 0
    for i, leg in enumerate(legs):
        if leg["status"] != "found" or len(leg["points_rc"]) == 0:
            raise AssertionError(f"Invalid leg: {leg['from']} -> {leg['to']}")
        if leg["from"] != order[i] or leg["to"] != order[i + 1]:
            raise AssertionError("Leg order mismatch")
        if i and legs[i-1]["points_rc"][-1] != leg["points_rc"][0]:
            raise AssertionError("Legs are discontinuous")
        n_points += len(leg["points_rc"]) - (1 if i else 0)
    if len(result["points_rc"]) != n_points:
        raise AssertionError("Duplicate or missing junction in combined route")
    if result["total_distance_m"] is None or result["total_weighted_cost_m_equiv"] is None:
        raise AssertionError("Mission total missing")


def main() -> None:
    parser = argparse.ArgumentParser(description="Real Gale multi-stop mission integration smoke test")
    parser.add_argument("--profile", choices=["rover", "astronaut"], default="rover")
    parser.add_argument("--targets", type=int, choices=[1, 2, 3, 4, 5], default=3)
    parser.add_argument("--output", type=Path, default=ROOT / "prototypes/navigation/output_mission_integration/mission_v4_gale.json")
    args = parser.parse_args()

    start, goal = [99, 20], [99, 180]
    provider = RouteProvider.from_repo(ROOT)
    reference = provider.get_route(start, goal, profile=args.profile, route_type="risk_aware")
    if reference["status"] != "found":
        raise SystemExit(f"Reference V4 route not found: {reference['status']}")
    pts = reference["points_rc"]
    indexes = [((i + 1) * (len(pts) - 1)) // (args.targets + 1) for i in range(args.targets)]
    if len(set(indexes)) != len(indexes) or any(i in (0, len(pts) - 1) for i in indexes):
        raise SystemExit("Reference route too short for distinct science target cells")
    targets = [{"id": f"S{i+1}", "rc": list(pts[idx])} for i, idx in enumerate(indexes)]
    ids = [t["id"] for t in targets]
    print(f"Source: actual Gale DEM; risk_source={reference['risk_source']}")
    print(f"Profile: {args.profile}; A={start}; B={goal}")
    print(f"Integration TEST waypoints (NOT validated science targets): {targets}")

    adapter = NavigationV4Adapter(provider.get_route)
    planner = MultiStopMissionPlanner(adapter)
    report = {
        "note": "Integration waypoints chosen from an actual feasible V4 route; not geological science targets.",
        "profile": args.profile, "risk_source": reference["risk_source"],
        "start_rc": start, "goal_rc": goal, "science_targets": targets,
        "results": {},
    }
    for route_type in ("shortest_feasible", "risk_aware"):
        base_request = {
            "start_rc": start, "goal_rc": goal, "science_targets": targets,
            "profile": args.profile, "route_type": route_type,
        }
        optimized = planner.plan_mission({**base_request, "mode": "optimized"})
        manual = planner.plan_mission({**base_request, "mode": "manual"})
        assert_valid_mission(optimized, start, goal, ids)
        assert_valid_mission(manual, start, goal, ids)
        cost_key = "total_distance_m" if route_type == "shortest_feasible" else "total_weighted_cost_m_equiv"
        if optimized[cost_key] > manual[cost_key] + 0.02:
            raise AssertionError("Optimized objective is greater than manual order")
        print(f"{route_type:18} PASS  optimized={' -> '.join(optimized['visit_order'])} "
              f"distance={optimized['total_distance_m']}m "
              f"weighted={optimized['total_weighted_cost_m_equiv']} "
              f"manual_objective={manual[cost_key]} optimized_objective={optimized[cost_key]}")
        report["results"][route_type] = {"optimized": optimized, "manual": manual}
    report["route_provider_engine_compute_calls"] = provider.engine_compute_calls
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    print(f"Integration PASS; V4 engine compute calls={provider.engine_compute_calls}")
    print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
