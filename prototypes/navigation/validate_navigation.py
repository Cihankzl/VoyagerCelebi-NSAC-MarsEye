"""MarsEye Navigation V4 independent A/B quality checks, no frontend needed.

From repo root:
  python prototypes/navigation/validate_navigation.py

Does not alter planner_v4.py or pre-existing routes.json.
Writes diagnostics under prototypes/navigation/output_validation/.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict
from pathlib import Path
import sys
import time

import numpy as np

try:
    from .planner_v4 import PROFILES, RouteEngine, risk_cost_of_step
except ImportError:
    from planner_v4 import PROFILES, RouteEngine, risk_cost_of_step

REPO_ROOT = Path(__file__).resolve().parents[2]


def _close(actual: float, expected: float, *, tolerance: float = 0.012) -> bool:
    return math.isclose(float(actual), float(expected), rel_tol=0.0, abs_tol=tolerance)


def validate_path(engine: RouteEngine, response: dict, kind: str) -> list[str]:
    """Independent checks of each returned found route, using loaded DEM layers."""
    errors: list[str] = []
    route = response["routes"][kind]
    if route.get("status") != "found":
        if route.get("points_rc", []) != []:
            errors.append(f"{kind}: failure response must have empty points_rc")
        return errors

    pts = route.get("points_rc")
    if not isinstance(pts, list) or not pts:
        return [f"{kind}: found but no points_rc"]
    if pts[0] != response["start_rc"] or pts[-1] != response["goal_rc"]:
        errors.append(f"{kind}: first/last coordinate does not match A/B")

    profile_name = response["profile"]
    profile = PROFILES[profile_name]
    allowed = engine.valid & (engine.slope < profile.max_cell_slope_deg)
    risk = engine.risk_maps[profile_name]
    if risk is not None:
        allowed &= np.isfinite(risk)

    length_m = 0.0
    cost = 0.0
    ascent = 0.0
    for i, rc in enumerate(pts):
        if not (isinstance(rc, list) and len(rc) == 2 and all(type(v) is int for v in rc)):
            errors.append(f"{kind}: malformed point at index {i}")
            break
        r, c = rc
        if not engine._legal_node((r, c), allowed):
            errors.append(f"{kind}: blocked/outside cell {rc}")
            break
        if i == 0:
            continue
        prev = tuple(pts[i - 1])
        cur = (r, c)
        if not engine._legal_edge(prev, cur, allowed, profile.max_edge_grade_deg):
            errors.append(f"{kind}: illegal edge {prev} -> {cur}")
            break
        dr, dc = cur[0] - prev[0], cur[1] - prev[1]
        step_m = math.hypot(dr * engine.dy, dc * engine.dx)
        length_m += step_m
        z0, z1 = float(engine.elevation[prev]), float(engine.elevation[cur])
        ascent += max(0.0, z1 - z0)
        cold = getattr(engine, "cold_index", None)
        cost += risk_cost_of_step(
            profile, float(engine.slope[prev]), float(engine.slope[cur]),
            float(engine.roughness[prev]), float(engine.roughness[cur]),
            z0, z1, step_m,
            float(risk[prev]) if risk is not None else None,
            float(risk[cur]) if risk is not None else None,
            float(cold[prev]) if cold is not None else None,
            float(cold[cur]) if cold is not None else None,
        )

    if not _close(route.get("distance_m", math.nan), length_m):
        errors.append(f"{kind}: distance mismatch: JSON={route.get('distance_m')}, recomputed={length_m:.3f}")
    if not _close(route.get("weighted_cost_m_equiv", math.nan), cost):
        errors.append(f"{kind}: weighted cost mismatch")
    if not _close(route.get("elevation_gain_m", math.nan), ascent):
        errors.append(f"{kind}: ascent mismatch")
    return errors


def validate_comparison(response: dict) -> list[str]:
    errors: list[str] = []
    short = response["routes"]["shortest_feasible"]
    risk = response["routes"]["risk_aware"]
    if short.get("status") == "found" and risk.get("status") == "found":
        if float(short["distance_m"]) > float(risk["distance_m"]) + 0.012:
            errors.append("Shortest Feasible is longer than Risk-Aware under same constraints")
        if float(risk["weighted_cost_m_equiv"]) > float(short["weighted_cost_m_equiv"]) + 0.012:
            errors.append("Risk-Aware has larger weighted model cost than Shortest Feasible")
    elif short.get("status") == "found" and risk.get("status") != "found":
        errors.append("Risk-Aware cannot find path although Shortest Feasible can")
    elif short.get("status") != "found" and risk.get("status") == "found":
        errors.append("Shortest Feasible cannot find path although Risk-Aware can")
    return errors


def _derive_scenarios(engine: RouteEngine) -> list[tuple[str, list[int], list[int]]]:
    rows, cols = engine.elevation.shape
    preferred_a = [rows // 2, cols // 10]
    preferred_b = [rows // 2, cols * 9 // 10]
    scenarios = [
        ("default_A_to_B", preferred_a, preferred_b),
        ("reverse_B_to_A", preferred_b, preferred_a),
        ("same_point", preferred_a, preferred_a),
    ]
    # Add an independently useful sub-leg of a known valid path, when available.
    for profile in ("rover", "astronaut"):
        baseline = engine.compute(preferred_a, preferred_b, profile)
        points = baseline["routes"]["shortest_feasible"].get("points_rc", [])
        if len(points) >= 8:
            i, j = len(points) // 4, (3 * len(points)) // 4
            scenarios.append(("route_subsegment", list(points[i]), list(points[j])))
            break
    # Blocked endpoint test: a cell that is valid but fails Rover cell slope.
    rover = PROFILES["rover"]
    blocked = np.argwhere(engine.valid & (engine.slope >= rover.max_cell_slope_deg))
    if blocked.size:
        rc = [int(v) for v in blocked[len(blocked) // 2]]
        scenarios.append(("blocked_rover_endpoint", rc, preferred_b))
    return scenarios


def run_real(output: Path) -> tuple[dict, int]:
    engine = RouteEngine.from_files(
        REPO_ROOT / "data/processed/elevation.npy",
        REPO_ROOT / "data/processed/terrain_meta.json",
        REPO_ROOT / "data/processed/valid_mask.npy",
    )
    scenarios = _derive_scenarios(engine)
    rows_out: list[dict] = []
    errors: list[str] = []
    print(f"Grid: {list(engine.elevation.shape)}; dx={engine.dx:.6f} m; dy={engine.dy:.6f} m")
    for name, start, goal in scenarios:
        for profile in ("rover", "astronaut"):
            t0 = time.perf_counter()
            response = engine.compute(start, goal, profile)
            validation = validate_comparison(response)
            for kind in ("shortest_feasible", "risk_aware"):
                validation.extend(validate_path(engine, response, kind))
            if name == "blocked_rover_endpoint" and profile == "rover":
                if any(response["routes"][k]["status"] != "blocked_endpoint" for k in ("shortest_feasible", "risk_aware")):
                    validation.append("Rover blocked endpoint was not rejected")
            errors.extend(f"{name}/{profile}: {err}" for err in validation)
            for kind in ("shortest_feasible", "risk_aware"):
                r = response["routes"][kind]
                rows_out.append({
                        "scenario": name,
                     "profile": profile,
                        "route_type": kind,
                        "start_rc": json.dumps(start),
                        "goal_rc": json.dumps(goal),
                        "status": r["status"],
                        "distance_m": r.get("distance_m"),
                        "mean_slope_deg": r.get("mean_slope_deg"),
                        "max_slope_deg": r.get("max_slope_deg"),
                        "max_edge_grade_deg": r.get("max_edge_grade_deg"),
                        "mean_roughness_m": r.get("mean_roughness_m"),
                        "elevation_gain_m": r.get("elevation_gain_m"),
                        "elevation_loss_m": r.get("elevation_loss_m"),
                        "weighted_cost_m_equiv": r.get("weighted_cost_m_equiv"),
                        "runtime_ms": r.get("runtime_ms"),
                        "validation": "FAIL" if validation else "PASS",
                    })
            elapsed = time.perf_counter() - t0
            short = response["routes"]["shortest_feasible"]
            risk = response["routes"]["risk_aware"]
            print(f"{name:23s} {profile:9s} short={short['status']:17s} risk={risk['status']:17s} "
                  f"QA={'FAIL' if validation else 'PASS'}  wall={elapsed:.2f}s")
    output.mkdir(parents=True, exist_ok=True)
    summary = {
        "terrain_shape": list(engine.elevation.shape),
        "dx_m": engine.dx, "dy_m": engine.dy,
        "validation_passed": not errors,
        "scenario_count": len(scenarios),
        "profile_case_count": len(scenarios) * 2,
        "route_case_count": len(rows_out),
        "errors": errors,
        "results": rows_out,
        "note": "DEM proxy and profile settings are experimental; QA checks internal consistency, not operational Mars safety.",
    }
    (output / "navigation_validation.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8"
    )
    with (output / "navigation_validation.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows_out[0]))
        writer.writeheader()
        writer.writerows(rows_out)
    print("Saved:", output.resolve())
    print("Validation:", "PASS" if not errors else "FAIL")
    for error in errors:
        print("ERROR:", error)
    return summary, 0 if not errors else 1


def main() -> None:
    parser = argparse.ArgumentParser(description="MarsEye V4 multiple-A/B QA and performance report")
    parser.add_argument("--output", type=Path, default=Path("prototypes/navigation/output_validation"))
    args = parser.parse_args()
    try:
        _, exit_code = run_real(args.output)
    except (OSError, ValueError, KeyError) as error:
        print(f"Could not validate processed Gale DEM: {error}", file=sys.stderr)
        sys.exit(2)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
