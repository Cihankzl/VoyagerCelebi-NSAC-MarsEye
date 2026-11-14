"""MarsEye V3: real DEM, roughness + ascent-aware A*, separate rover/EVA profiles.

Place next to planner_v2.py and run from the repo root, for example:

python prototypes/navigation/planner_v3.py --mode real \
  --elevation data/processed/elevation.npy \
  --meta data/processed/terrain_meta.json \
  --mask data/processed/valid_mask.npy \
  --output prototypes/navigation/output_v3

No certified Mars rover or EVA safety/energy limits are implied by the defaults.
The 3x3 detrended DEM roughness measures terrain at the DEM scale only.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import heapq
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from planner_v2 import load_surface, make_risk, strict_grid, astar

MOVES = [(dr, dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]


@dataclass(frozen=True)
class Profile:
    max_cell_slope_deg: float
    max_edge_grade_deg: float
    slope_weight: float
    roughness_weight: float
    ascent_weight: float
    # These are illustrative, unitless weights except ascent_weight [m-equivalent/m].


PROFILES = {
    "rover": Profile(
        max_cell_slope_deg=20.0,
        max_edge_grade_deg=20.0,
        slope_weight=2.0,
        roughness_weight=4.0,
        ascent_weight=0.8,
    ),
    "astronaut": Profile(
        max_cell_slope_deg=25.0,
        max_edge_grade_deg=25.0,
        slope_weight=2.7,
        roughness_weight=1.0,
        ascent_weight=4.0,
    ),
}


def detrended_roughness(elevation: np.ndarray, valid: np.ndarray, dx: float, dy: float):
    """3x3 RMS of elevations remaining after subtracting a local tangent plane.

    As opposed to raw standard deviation, an ideal planar incline has ~zero
    roughness even if the plane is steep. This is NOT a rock-detection method.
    """
    h, w = elevation.shape
    z = elevation.astype(np.float64).copy()
    z[~valid] = np.nan
    dz_dr, dz_dc = np.gradient(z, dy, dx)
    z_pad = np.pad(z, 1, mode="constant", constant_values=np.nan)
    squared_sum = np.zeros((h, w), dtype=float)
    count = np.zeros((h, w), dtype=np.int16)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            neighbor = z_pad[1 + dr:h + 1 + dr, 1 + dc:w + 1 + dc]
            residual = neighbor - z - dz_dr * (dr * dy) - dz_dc * (dc * dx)
            good = np.isfinite(residual) & valid
            squared_sum[good] += residual[good] ** 2
            count[good] += 1
    rough = np.full((h, w), np.nan, dtype=float)
    np.sqrt(np.divide(squared_sum, count, out=rough, where=count >= 4), out=rough,
            where=count >= 4)
    rough[~valid] = np.nan
    return rough


def edge_grade_deg(z_from: float, z_to: float, distance_m: float) -> float:
    return float(np.degrees(np.arctan2(abs(z_to - z_from), distance_m)))


def plan_route(
    elevation: np.ndarray,
    slope: np.ndarray,
    roughness: np.ndarray,
    valid: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    dx: float,
    dy: float,
    profile: Profile,
):
    rows, cols = elevation.shape
    allowed = valid & np.isfinite(roughness) & (slope < profile.max_cell_slope_deg)

    def is_allowed(node):
        r, c = node
        return 0 <= r < rows and 0 <= c < cols and bool(allowed[r, c])

    if not is_allowed(start) or not is_allowed(goal):
        return None, "blocked_endpoint"

    def heuristic(node):
        # Admissible: every edge cost >= horizontal Euclidean distance.
        return float(np.hypot((goal[0] - node[0]) * dy, (goal[1] - node[1]) * dx))

    q = [(heuristic(start), 0.0, start)]
    dist = {start: 0.0}
    parent = {}
    while q:
        _, g, node = heapq.heappop(q)
        if g > dist[node] + 1e-9:
            continue
        if node == goal:
            path = [node]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return path[::-1], "found"

        r, c = node
        for dr, dc in MOVES:
            nxt = (r + dr, c + dc)
            if not is_allowed(nxt):
                continue
            # Prevent diagonal traversals through corners of blocked cells.
            if dr and dc and (not is_allowed((r + dr, c)) or not is_allowed((r, c + dc))):
                continue
            step_m = float(np.hypot(dr * dy, dc * dx))
            h0, h1 = float(elevation[node]), float(elevation[nxt])
            if edge_grade_deg(h0, h1, step_m) >= profile.max_edge_grade_deg:
                continue

            # Continuous, nonnegative penalties. The ascent term penalizes uphill
            # travel rather than net elevation change between endpoints.
            local_slope = 0.5 * float(slope[node] + slope[nxt])
            local_rough = 0.5 * float(roughness[node] + roughness[nxt])
            slope_pen = (local_slope / 15.0) ** 2
            rough_pen = local_rough / 1.0  # normalized by reference 1 metre RMS
            ascent = max(0.0, h1 - h0)
            step_cost = step_m * (
                1.0 + profile.slope_weight * slope_pen
                + profile.roughness_weight * rough_pen
            ) + profile.ascent_weight * ascent
            new_g = g + step_cost
            if new_g < dist.get(nxt, float("inf")):
                dist[nxt] = new_g
                parent[nxt] = node
                heapq.heappush(q, (new_g + heuristic(nxt), new_g, nxt))
    return None, "no_path"


def summarize(path, elevation, slope, roughness, dx, dy, profile=None):
    pts = np.asarray(path, dtype=int)
    rr, cc = pts[:, 0], pts[:, 1]
    heights = elevation[rr, cc]
    step_rc = np.diff(pts, axis=0)
    step_m = np.hypot(step_rc[:, 0] * dy, step_rc[:, 1] * dx)
    rise = np.diff(heights)
    grades = np.degrees(np.arctan2(np.abs(rise), step_m)) if len(step_m) else np.array([0.0])
    slopes = slope[rr, cc]
    rough = roughness[rr, cc]
    stats = {
        "status": "found",
        "points_rc": pts.tolist(),
        "distance_m": round(float(step_m.sum()), 2),
        "max_slope_deg": round(float(np.max(slopes)), 2),
        "mean_slope_deg": round(float(np.mean(slopes)), 2),
        "max_edge_grade_deg": round(float(np.max(grades)), 2),
        "mean_roughness_m": round(float(np.mean(rough)), 3),
        "max_roughness_m": round(float(np.max(rough)), 3),
        "elevation_gain_m": round(float(np.maximum(rise, 0.0).sum()), 2),
        "elevation_loss_m": round(float(np.maximum(-rise, 0.0).sum()), 2),
        "cells_ge_20deg": int(np.sum(slopes >= 20.0)),
    }
    if profile is not None:
        stats["profile"] = asdict(profile)
    return stats


def main():
    p = argparse.ArgumentParser(description="MarsEye V3: separate rover vs astronaut A* routes")
    p.add_argument("--mode", choices=("real", "synthetic"), default="real")
    p.add_argument("--elevation", type=Path, default=Path("data/processed/elevation.npy"))
    p.add_argument("--meta", type=Path, default=Path("data/processed/terrain_meta.json"))
    p.add_argument("--mask", type=Path, default=Path("data/processed/valid_mask.npy"))
    p.add_argument("--start", nargs=2, type=int, metavar=("ROW", "COL"))
    p.add_argument("--goal", nargs=2, type=int, metavar=("ROW", "COL"))
    p.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "output_v3")
    p.add_argument("--rover-max-slope", type=float, default=20.0,
                   help="Illustrative hard limit, degrees, applied to local and edge slope")
    p.add_argument("--astronaut-max-slope", type=float, default=25.0,
                   help="Illustrative hard limit, degrees, applied to local and edge slope")
    p.add_argument("--skip-terrain-json", action="store_true")
    args = p.parse_args()

    elevation, meta, data_type, default_start, default_goal = load_surface(args)
    start = tuple(args.start) if args.start is not None else default_start
    goal = tuple(args.goal) if args.goal is not None else default_goal
    dx, dy = float(meta["dx_m"]), float(meta["dy_m"])
    if args.mode == "real":
        if args.mask and not args.mask.exists():
            raise FileNotFoundError(f"Valid-mask file not found: {args.mask}")
        mask = args.mask
    else:
        mask = None
    slope, _, valid = make_risk(elevation, meta, mask)
    rough = detrended_roughness(elevation, valid, dx, dy)
    valid &= np.isfinite(rough)
    profiles = {
        name: Profile(**{
            **asdict(profile),
            "max_cell_slope_deg": float(getattr(args, f"{name}_max_slope")),
            "max_edge_grade_deg": float(getattr(args, f"{name}_max_slope")),
        })
        for name, profile in PROFILES.items()
    }
    for profile in profiles.values():
        if not 0.0 < profile.max_cell_slope_deg <= 90.0:
            raise ValueError("Max slope must be greater than 0 and at most 90 degrees")
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    result = {
        "data_type": data_type,
        "grid_shape": list(elevation.shape),
        "dx_m": dx,
        "dy_m": dy,
        "start_rc": list(start),
        "goal_rc": list(goal),
        "notes": [
            "Rover and astronaut parameter values are demonstration assumptions, not calibrated vehicles or human EVA limits.",
            "DEM-scale roughness is detrended 3x3 RMS; small rocks and soil bearing capacity are not resolved.",
            "Uphill elevation gain is a proxy for effort, not a validated energy/metabolic rate.",
            "Unconstrained shortest route is reference only and may violate profile limits.",
        ],
        "profiles": {key: asdict(value) for key, value in profiles.items()},
        "routes": {},
    }

    # Preserve an easy-to-compare distance-only reference, clearly unconstrainted.
    shortest = astar(np.where(valid, 1.0, np.inf), start, goal, dx, dy)
    result["routes"]["shortest_reference"] = summarize(shortest, elevation, slope, rough, dx, dy)
    result["routes"]["shortest_reference"]["note"] = "Distance-only, without rover/astronaut slope limits"

    for name, profile in profiles.items():
        path, status = plan_route(elevation, slope, rough, valid, start, goal, dx, dy, profile)
        if path is None:
            result["routes"][name] = {"status": status, "profile": asdict(profile)}
            print(f"{name}: {status}")
        else:
            stats = summarize(path, elevation, slope, rough, dx, dy, profile)
            result["routes"][name] = stats
            assert stats["max_slope_deg"] < profile.max_cell_slope_deg + 0.01
            assert stats["max_edge_grade_deg"] < profile.max_edge_grade_deg + 0.01
            print(f"{name}: {stats['distance_m']:.2f} m, mean slope={stats['mean_slope_deg']:.2f}°, "
                  f"max slope={stats['max_slope_deg']:.2f}°, roughness={stats['mean_roughness_m']:.3f}m, "
                  f"ascent={stats['elevation_gain_m']:.2f}m, max edge grade={stats['max_edge_grade_deg']:.2f}°")

    (output / "routes.json").write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    np.save(output / "slope.npy", slope)
    np.save(output / "roughness.npy", rough)
    np.save(output / "valid_mask.npy", valid)
    if not args.skip_terrain_json:
        terrain = {
            "data_type": data_type,
            "grid_shape": list(elevation.shape),
            "dx_m": dx,
            "dy_m": dy,
            "elevation_m": strict_grid(elevation, valid),
            "slope_deg": strict_grid(slope, valid),
            "roughness_m": strict_grid(rough, valid, decimals=3),
            "source_metadata": meta,
        }
        (output / "terrain.json").write_text(json.dumps(terrain, separators=(",", ":"), allow_nan=False), encoding="utf-8")
    fig, ax = plt.subplots(figsize=(10, 7))
    im = ax.imshow(np.ma.masked_invalid(slope), origin="upper", cmap="YlOrRd")
    fig.colorbar(im, ax=ax, label="DEM slope (degrees)")
    colors = {"shortest_reference": "#2665D8", "rover": "#14B8A6", "astronaut": "#7C3AED"}
    labels = {"shortest_reference": "Shortest (reference)", "rover": "Rover", "astronaut": "Astronaut"}
    for name in ("shortest_reference", "rover", "astronaut"):
        data = result["routes"][name]
        if data.get("status") != "found":
            continue
        coords = np.array(data["points_rc"], dtype=int)
        ax.plot(coords[:, 1], coords[:, 0], color=colors[name], linewidth=2.0, label=labels[name])
    ax.scatter([start[1], goal[1]], [start[0], goal[0]], c="black", s=45, zorder=5)
    ax.set(title="MarsEye V3 — Rover vs Astronaut (experimental profiles)", xlabel="Column", ylabel="Row")
    ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(output / "routes_preview.png", dpi=150)
    plt.close(fig)
    print("Output directory:", output.resolve())
    print("NOTE: Profile limits/cost weights are illustrative; not certified mission requirements.")


if __name__ == "__main__":
    main()
