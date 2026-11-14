"""MarsEye Navigation V4 performance benchmark (independent from frontend).

Run from repository root:
  python prototypes/navigation/benchmark_navigation.py
  python prototypes/navigation/benchmark_navigation.py --repeats 9 --warmups 2

No modification to planner_v4.py or route outputs is performed.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import sys
import time
from typing import Any

try:
    from .planner_v4 import RouteEngine
except ImportError:
    from planner_v4 import RouteEngine

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CASES = (
    ("default_A_to_B", (99, 20), (99, 180)),
    ("reverse_B_to_A", (99, 180), (99, 20)),
    ("route_subsegment", (103, 60), (108, 140)),
)
ROUTES = ("shortest_feasible", "risk_aware")
PROFILES = ("rover", "astronaut")


def describe_samples(values: list[float]) -> dict[str, float]:
    """Median and range. Avoid a misleading p95 with only a few samples."""
    if not values or any(not math.isfinite(v) or v < 0 for v in values):
        raise ValueError("Samples must be nonempty, finite nonnegative numbers")
    return {
        "count": len(values),
        "median_ms": round(statistics.median(values), 3),
        "mean_ms": round(statistics.fmean(values), 3),
        "min_ms": round(min(values), 3),
        "max_ms": round(max(values), 3),
    }


def stable_route_signature(route: dict) -> tuple[Any, ...]:
    """Ignore runtime noise but catch non-reproducible route/path/metrics."""
    return (
        route.get("status"),
        tuple(map(tuple, route.get("points_rc", []))),
        route.get("distance_m"),
        route.get("weighted_cost_m_equiv"),
        route.get("elevation_gain_m"),
        route.get("elevation_loss_m"),
    )


def run_case(engine: RouteEngine, scenario: str, start: tuple[int, int],
             goal: tuple[int, int], profile: str, warmups: int, repeats: int):
    if repeats < 1 or warmups < 0:
        raise ValueError("repeats must be >=1 and warmups >=0")
    # Warmups deliberately excluded from measurements. A and B are unchanged.
    for _ in range(warmups):
        engine.compute(list(start), list(goal), profile, False)

    route_samples: dict[str, list[float]] = {route: [] for route in ROUTES}
    wall_samples: list[float] = []
    baseline_signatures = None
    canonical: dict[str, dict] = {}
    for _ in range(repeats):
        t0 = time.perf_counter()
        result = engine.compute(list(start), list(goal), profile, False)
        wall_ms = (time.perf_counter() - t0) * 1000.0
        wall_samples.append(wall_ms)
        signatures = tuple(stable_route_signature(result["routes"][name]) for name in ROUTES)
        if baseline_signatures is not None and signatures != baseline_signatures:
            raise RuntimeError(f"Non-reproducible route detected for {scenario}/{profile}")
        baseline_signatures = signatures
        for kind in ROUTES:
            route = result["routes"][kind]
            elapsed = float(route["runtime_ms"])
            if not math.isfinite(elapsed) or elapsed < 0:
                raise RuntimeError("Invalid planner runtime_ms")
            route_samples[kind].append(elapsed)
            canonical[kind] = route

    rows = []
    for kind in ROUTES:
        route = canonical[kind]
        metrics = describe_samples(route_samples[kind])
        rows.append({
            "scenario": scenario,
            "profile": profile,
            "route_type": kind,
            "start_rc": json.dumps(start),
            "goal_rc": json.dumps(goal),
            "status": route["status"],
            "distance_m": route.get("distance_m"),
            "weighted_cost_m_equiv": route.get("weighted_cost_m_equiv"),
            "mean_slope_deg": route.get("mean_slope_deg"),
            "max_slope_deg": route.get("max_slope_deg"),
            "mean_roughness_m": route.get("mean_roughness_m"),
            "elevation_gain_m": route.get("elevation_gain_m"),
            "repeats": repeats,
            "warmups": warmups,
            "median_runtime_ms": metrics["median_ms"],
            "mean_runtime_ms": metrics["mean_ms"],
            "min_runtime_ms": metrics["min_ms"],
            "max_runtime_ms": metrics["max_ms"],
            "runtime_samples_ms": route_samples[kind],
        })
    return rows, describe_samples(wall_samples), wall_samples


def run_benchmark(engine: RouteEngine, *, repeats: int = 5, warmups: int = 1,
                  cases=DEFAULT_CASES) -> dict[str, Any]:
    if repeats < 1 or warmups < 0:
        raise ValueError("repeats must be >=1 and warmups >=0")
    rows = []
    walls = []
    for scenario, start, goal in cases:
        for profile in PROFILES:
            metrics, wall_summary, samples = run_case(engine, scenario, start, goal, profile, warmups, repeats)
            rows.extend(metrics)
            walls.append({"scenario": scenario, "profile": profile,
                          **wall_summary, "wall_samples_ms": samples})
            short, risk = metrics
            short_time, risk_time = short["median_runtime_ms"], risk["median_runtime_ms"]
            ratio = risk_time / short_time if short_time > 0 else None
            ratio_str = f"{ratio:.1f}x" if ratio is not None else "n/a"
            print(f"{scenario:21s} {profile:9s} "
                  f"short={short_time:9.2f}ms  risk={risk_time:9.2f}ms  "
                  f"risk/short={ratio_str}  both={wall_summary['median_ms']:.2f}ms")
    return {
        "schema_version": "1.0",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "terrain_shape": list(engine.elevation.shape),
        "dx_m": engine.dx,
        "dy_m": engine.dy,
        "warmups_per_case": warmups,
        "measured_runs_per_case": repeats,
        "profiles": list(PROFILES),
        "route_types": list(ROUTES),
        "planner_time_note": "runtime_ms is V4's per-route planning timer; excludes route summarization",
        "wall_time_note": "Wall time is entire compute() for BOTH routes, excludes engine/DEM load and warmups",
        "method_note": "Experimental model, local machine, repeated runs, NOT operational Mars performance",
        "results": rows,
        "combined_call_times": walls,
    }


def write_outputs(report: dict, output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    (output / "navigation_benchmark.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    csv_fields = [key for key in report["results"][0] if key != "runtime_samples_ms"]
    with (output / "navigation_benchmark.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=csv_fields)
        writer.writeheader()
        writer.writerows({k: row[k] for k in csv_fields} for row in report["results"])

    lines = [
        "# MarsEye V4 — Navigation performance benchmark",
        "",
        f"- Repeats per scenario/profile: {report['measured_runs_per_case']} (warmups: {report['warmups_per_case']})",
        "- Values are median planning runtimes (milliseconds) measured by planner_v4.py.",
        "- Data and machine-specific results, not Mars mission performance or guaranteed UI latency.",
        "",
        "| Scenario | Profile | Shortest Feasible (ms) | Risk-Aware (ms) | Ratio |",
        "|---|---|---:|---:|---:|",
    ]
    for index in range(0, len(report["results"]), 2):
        short, risk = report["results"][index:index + 2]
        t_short, t_risk = short["median_runtime_ms"], risk["median_runtime_ms"]
        ratio = f"{t_risk/t_short:.2f}x" if t_short > 0 else "n/a"
        lines.append(f"| {short['scenario']} | {short['profile']} | {t_short:.2f} | {t_risk:.2f} | {ratio} |")
    lines.extend([
        "", "## Interpretation", "",
        "- Compare Shortest and Risk-Aware **within the same profile**; their cost weights differ by profile.",
        "- Time differences depend on the machine, process load and chosen endpoints.",
        "- At small sample sizes, report median and range; do not claim statistically robust p95.",
        "- `risk_aware` costs are experimental generalized model costs, not energy consumption or safety probabilities.",
        "- `planner_v4.py` computes shortest and risk-aware routes in each `compute` call.",
        "- The measured wall-clock time for both is in `navigation_benchmark.json`.",
        "", "## Next steps", "",
        "Review results on a quiet machine, repeat the benchmark, and check agreement across runs.",
    ])
    (output / "navigation_benchmark_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="MarsEye V4 repeated route runtime benchmark")
    parser.add_argument("--repeats", type=int, default=5, help="Measured runs for every scenario/profile; default 5")
    parser.add_argument("--warmups", type=int, default=1, help="Excluded warmups for every scenario/profile; default 1")
    parser.add_argument("--output", type=Path,
                        default=Path("prototypes/navigation/output_benchmark"))
    args = parser.parse_args()
    if args.repeats < 1 or args.warmups < 0:
        parser.error("--repeats must be >= 1 and --warmups must be >= 0")
    try:
        engine = RouteEngine.from_files(
            REPO_ROOT / "data/processed/elevation.npy",
            REPO_ROOT / "data/processed/terrain_meta.json",
            REPO_ROOT / "data/processed/valid_mask.npy",
        )
        print(f"Grid: {engine.elevation.shape}, dx={engine.dx:.6f}m, dy={engine.dy:.6f}m")
        print(f"Warmups: {args.warmups}; measured repetitions: {args.repeats}")
        report = run_benchmark(engine, repeats=args.repeats, warmups=args.warmups)
        write_outputs(report, args.output)
        print("Saved:", args.output.resolve())
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print("Benchmark failed:", error, file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
