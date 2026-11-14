"""MarsEye V4 — dynamic A/B, feasible shortest vs risk-aware A*.

Independent from V3: existing planner_v3.py and JSON outputs remain unchanged.
All profile limits and cost weights are EXPERIMENTAL, not mission safety ratings.

Run from repository root:
 python prototypes/navigation/planner_v4.py --start 99 20 --goal 99 180 --profile rover
 python prototypes/navigation/planner_v4.py --start 99 20 --goal 99 180 --profile both --include-reference

Optional, when 2nd-year EEE's validated risk layers are ready:
 python prototypes/navigation/planner_v4.py --risk-rover data/processed/surface_risk_rover.npy --risk-astronaut data/processed/surface_risk_astronaut.npy
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, asdict
import heapq
import json
import math
from pathlib import Path
import time
from typing import Any

import numpy as np

MOVES = [(dr, dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]


@dataclass(frozen=True)
class Profile:
    max_cell_slope_deg: float
    max_edge_grade_deg: float
    slope_weight: float
    roughness_weight: float
    ascent_weight: float
    external_risk_weight: float = 6.0


PROFILES = {
    "rover": Profile(20.0, 20.0, 2.0, 4.0, 0.8),
    "astronaut": Profile(25.0, 25.0, 2.7, 1.0, 4.0),
}


def make_layers(elevation: np.ndarray, meta: dict, supplied_mask: np.ndarray | None = None):
    """Match V3's DEM-scale slope and detrended 3x3 roughness concepts."""
    dx, dy = float(meta["dx_m"]), float(meta["dy_m"])
    if not (math.isfinite(dx) and math.isfinite(dy) and dx > 0 and dy > 0):
        raise ValueError("dx_m/dy_m must be positive finite metres")
    if elevation.ndim != 2 or min(elevation.shape) < 3:
        raise ValueError("elevation must be a 2D array at least 3x3")
    z = elevation.astype(np.float64, copy=True)
    valid = np.isfinite(z)
    nodata = meta.get("nodata")
    if nodata is not None:
        valid &= z != float(nodata)
    if supplied_mask is not None:
        if supplied_mask.shape != z.shape:
            raise ValueError("valid_mask shape must match elevation")
        valid &= supplied_mask.astype(bool)
    z[~valid] = np.nan
    dz_dr, dz_dc = np.gradient(z, dy, dx)
    slope = np.degrees(np.arctan(np.hypot(dz_dc, dz_dr)))
    valid &= np.isfinite(slope)

    # Detrend a local tangent plane before RMS: a perfect slope has near-zero roughness.
    h, w = z.shape
    padded = np.pad(z, 1, constant_values=np.nan)
    residual2 = np.zeros((h, w), dtype=np.float64)
    count = np.zeros((h, w), dtype=np.int16)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            neighbor = padded[1 + dr:h + 1 + dr, 1 + dc:w + 1 + dc]
            residual = neighbor - z - dz_dr * (dr * dy) - dz_dc * (dc * dx)
            good = valid & np.isfinite(residual)
            residual2[good] += residual[good] ** 2
            count[good] += 1
    rough = np.full(z.shape, np.nan, dtype=np.float64)
    np.sqrt(np.divide(residual2, count, out=rough, where=count >= 4), out=rough, where=count >= 4)
    valid &= np.isfinite(rough)
    slope[~valid] = np.nan
    rough[~valid] = np.nan
    return slope, rough, valid


def load_risk(path: Path | None, shape: tuple[int, int]) -> np.ndarray | None:
    if path is None:
        return None
    arr = np.load(path, allow_pickle=False)
    if arr.shape != shape:
        raise ValueError(f"Risk map {path} shape {arr.shape} != terrain {shape}")
    if not np.issubdtype(arr.dtype, np.number):
        raise ValueError("Risk map must be numeric")
    arr = arr.astype(np.float64)
    # NaN = unknown, a blocked cell for BOTH route types (common feasible space).
    if np.any(np.isinf(arr)) or np.any((np.isfinite(arr)) & ((arr < 0) | (arr > 1))):
        raise ValueError("Risk map must use values 0..1 or NaN for unknown")
    return arr


def risk_cost_of_step(profile: Profile, slope_a: float, slope_b: float,
                      rough_a: float, rough_b: float, z_a: float, z_b: float,
                      step_m: float, risk_a: float | None, risk_b: float | None) -> float:
    ascent = max(0.0, z_b - z_a)
    if risk_a is not None:
        # External risk already contains the terrain components; do not double count them.
        penalty = profile.external_risk_weight * 0.5 * (risk_a + risk_b)
    else:
        avg_slope = 0.5 * (slope_a + slope_b)
        avg_rough = 0.5 * (rough_a + rough_b)
        penalty = profile.slope_weight * (avg_slope / 15.0) ** 2 + profile.roughness_weight * (avg_rough / 1.0)
    return step_m * (1.0 + penalty) + profile.ascent_weight * ascent


class RouteEngine:
    def __init__(self, elevation: np.ndarray, meta: dict,
                 mask: np.ndarray | None = None,
                 risk_maps: dict[str, np.ndarray | None] | None = None):
        self.elevation = np.asarray(elevation, dtype=np.float64)
        self.meta = dict(meta)
        self.dx = float(self.meta["dx_m"])
        self.dy = float(self.meta["dy_m"])
        self.slope, self.roughness, self.valid = make_layers(self.elevation, meta, mask)
        self.risk_maps: dict[str, np.ndarray | None] = {}
        for name in PROFILES:
            risk = (risk_maps or {}).get(name)
            if risk is not None:
                risk = np.asarray(risk, dtype=np.float64)
                if risk.shape != self.elevation.shape:
                    raise ValueError(f"Risk map {name} shape differs from elevation")
                if np.any(np.isinf(risk)) or np.any(np.isfinite(risk) & ((risk < 0) | (risk > 1))):
                    raise ValueError(f"Risk map {name} must have 0..1 values or NaNs")
            self.risk_maps[name] = risk

    @classmethod
    def from_files(cls, elevation: Path, meta: Path, mask: Path | None,
                   risk_rover: Path | None = None, risk_astronaut: Path | None = None):
        data = np.load(elevation, allow_pickle=False)
        m = json.loads(meta.read_text(encoding="utf-8"))
        if "cell_size_m" in m:
            m.setdefault("dx_m", m["cell_size_m"])
            m.setdefault("dy_m", m["cell_size_m"])
        vm = np.load(mask, allow_pickle=False) if mask is not None else None
        risks = {"rover": load_risk(risk_rover, data.shape),
                 "astronaut": load_risk(risk_astronaut, data.shape)}
        return cls(data, m, vm, risks)

    def _legal_node(self, node, allowed):
        r, c = node
        rows, cols = self.elevation.shape
        return 0 <= r < rows and 0 <= c < cols and bool(allowed[r, c])

    def _grade(self, a, b, dist_m):
        return math.degrees(math.atan2(abs(float(self.elevation[b]) - float(self.elevation[a])), dist_m))

    def _legal_edge(self, a, b, allowed, grade_limit: float | None):
        if not self._legal_node(b, allowed):
            return False
        dr, dc = b[0] - a[0], b[1] - a[1]
        if abs(dr) > 1 or abs(dc) > 1 or (dr == dc == 0):
            return False
        # Avoid crossing the corner of a forbidden cell.
        if dr and dc:
            if not self._legal_node((a[0] + dr, a[1]), allowed) or not self._legal_node((a[0], a[1] + dc), allowed):
                return False
        if grade_limit is not None:
            step_m = math.hypot(dr * self.dy, dc * self.dx)
            if self._grade(a, b, step_m) >= grade_limit:
                return False
        return True

    def _astar(self, allowed, start, goal, grade_limit, profile, risk, objective):
        if not self._legal_node(start, allowed) or not self._legal_node(goal, allowed):
            return None, "blocked_endpoint"
        if start == goal:
            return [start], "found"

        def heuristic(node):
            return math.hypot((goal[0] - node[0]) * self.dy, (goal[1] - node[1]) * self.dx)

        q = [(heuristic(start), 0.0, start)]
        best = {start: 0.0}
        parent = {}
        while q:
            _, g, node = heapq.heappop(q)
            if g > best[node] + 1e-9:
                continue
            if node == goal:
                path = [node]
                while path[-1] != start:
                    path.append(parent[path[-1]])
                return path[::-1], "found"
            for dr, dc in MOVES:
                nxt = (node[0] + dr, node[1] + dc)
                if not self._legal_edge(node, nxt, allowed, grade_limit):
                    continue
                step_m = math.hypot(dr * self.dy, dc * self.dx)
                if objective == "distance":
                    cost = step_m
                else:
                    ra = float(risk[node]) if risk is not None else None
                    rb = float(risk[nxt]) if risk is not None else None
                    cost = risk_cost_of_step(
                        profile, float(self.slope[node]), float(self.slope[nxt]),
                        float(self.roughness[node]), float(self.roughness[nxt]),
                        float(self.elevation[node]), float(self.elevation[nxt]), step_m, ra, rb)
                new_g = g + cost
                if new_g + 1e-9 < best.get(nxt, math.inf):
                    best[nxt] = new_g
                    parent[nxt] = node
                    heapq.heappush(q, (new_g + heuristic(nxt), new_g, nxt))
        return None, "no_path"

    def _summarize(self, path, profile, risk, allowed, elapsed_ms, *, analysis_only=False):
        pts = np.asarray(path, dtype=int)
        rr, cc = pts[:, 0], pts[:, 1]
        z = self.elevation[rr, cc]
        diff = np.diff(pts, axis=0)
        step_m = np.hypot(diff[:, 0] * self.dy, diff[:, 1] * self.dx)
        dz = np.diff(z)
        grades = np.degrees(np.arctan2(abs(dz), step_m)) if len(step_m) else np.array([0.0])
        s = self.slope[rr, cc]
        rough = self.roughness[rr, cc]
        weighted_cost = 0.0
        for a, b, d in zip(path, path[1:], step_m):
            weighted_cost += risk_cost_of_step(profile, float(self.slope[a]), float(self.slope[b]),
                                               float(self.roughness[a]), float(self.roughness[b]),
                                               float(self.elevation[a]), float(self.elevation[b]), float(d),
                                               float(risk[a]) if risk is not None and np.isfinite(risk[a]) else None,
                                               float(risk[b]) if risk is not None and np.isfinite(risk[b]) else None)
        metrics = {
            "status": "found",
            "points_rc": pts.tolist(),
            "distance_m": round(float(step_m.sum()), 3),
            "mean_slope_deg": round(float(np.mean(s)), 3),
            "max_slope_deg": round(float(np.max(s)), 3),
            "max_edge_grade_deg": round(float(np.max(grades)), 3),
            "mean_roughness_m": round(float(np.mean(rough)), 4),
            "elevation_gain_m": round(float(np.maximum(dz, 0).sum()), 3),
            "elevation_loss_m": round(float(np.maximum(-dz, 0).sum()), 3),
            "weighted_cost_m_equiv": round(weighted_cost, 3),
            "runtime_ms": round(float(elapsed_ms), 2),
            "mean_risk_score_0_1": round(float(np.nanmean(risk[rr, cc])), 4) if risk is not None else None,
            "risk_source": "external_risk_map" if risk is not None else "DEM_slope_roughness_proxy",
        }
        if analysis_only:
            violations = sum(not bool(allowed[p]) for p in path)
            violations += sum(g >= profile.max_edge_grade_deg for g in grades)
            metrics.update({"analysis_only": True, "profile_constraint_violations": int(violations)})
        return metrics

    @staticmethod
    def _parse_rc(value, label):
        if not isinstance(value, (list, tuple)) or len(value) != 2 or any(type(v) is not int for v in value):
            raise ValueError(f"{label} must be two integers: [row, col]")
        return (value[0], value[1])

    def compute(self, start_rc: list[int] | tuple[int, int],
                goal_rc: list[int] | tuple[int, int], profile_name: str,
                include_reference: bool = False) -> dict[str, Any]:
        if profile_name not in PROFILES:
            raise ValueError(f"Unknown profile {profile_name!r}; expected rover or astronaut")
        start = self._parse_rc(start_rc, "start_rc")
        goal = self._parse_rc(goal_rc, "goal_rc")
        rows, cols = self.elevation.shape
        if not all(0 <= r < rows and 0 <= c < cols for r, c in (start, goal)):
            raise ValueError(f"Endpoints out of grid bounds: rows 0..{rows-1}, cols 0..{cols-1}")
        profile = PROFILES[profile_name]
        risk = self.risk_maps[profile_name]
        # Both objectives use precisely the same feasible cells and edge-grade limits.
        allowed = self.valid & (self.slope < profile.max_cell_slope_deg)
        if risk is not None:
            allowed &= np.isfinite(risk)  # unknown external-risk region is excluded

        result = {
            "schema_version": "1.0",
            "terrain_id": "gale_5m_v1" if self.elevation.shape == (198, 200) else "custom_grid",
            "grid_shape": [rows, cols],
            "dx_m": self.dx,
            "dy_m": self.dy,
            "coordinate_system": "row_col_0_based_origin_upper_left",
            "start_rc": list(start),
            "goal_rc": list(goal),
            "profile": profile_name,
            "profile_parameters": asdict(profile),
            "experimental": True,
            "risk_source": "external_risk_map" if risk is not None else "DEM_slope_roughness_proxy",
            "routes": {},
        }
        for route_name, objective in (("shortest_feasible", "distance"), ("risk_aware", "risk")):
            t0 = time.perf_counter()
            path, status = self._astar(allowed, start, goal, profile.max_edge_grade_deg, profile, risk, objective)
            elapsed = 1000 * (time.perf_counter() - t0)
            result["routes"][route_name] = (
                self._summarize(path, profile, risk, allowed, elapsed) if path is not None
                else {"status": status, "points_rc": [], "runtime_ms": round(elapsed, 2)}
            )
        if include_reference:
            t0 = time.perf_counter()
            # Reference disregards profile limits and missing external risk; but not DEM NoData.
            path, status = self._astar(self.valid, start, goal, None, profile, None, "distance")
            elapsed = 1000 * (time.perf_counter() - t0)
            result["routes"]["unconstrained_reference"] = (
                self._summarize(path, profile, None, allowed, elapsed, analysis_only=True) if path is not None
                else {"status": status, "points_rc": [], "analysis_only": True, "runtime_ms": round(elapsed, 2)}
            )
        return result


def main():
    p = argparse.ArgumentParser(description="MarsEye V4 dynamic feasible vs risk-aware A*")
    p.add_argument("--elevation", type=Path, default=Path("data/processed/elevation.npy"))
    p.add_argument("--meta", type=Path, default=Path("data/processed/terrain_meta.json"))
    p.add_argument("--mask", type=Path, default=Path("data/processed/valid_mask.npy"))
    p.add_argument("--risk-rover", type=Path)
    p.add_argument("--risk-astronaut", type=Path)
    p.add_argument("--start", type=int, nargs=2, metavar=("ROW", "COL"))
    p.add_argument("--goal", type=int, nargs=2, metavar=("ROW", "COL"))
    p.add_argument("--profile", choices=("rover", "astronaut", "both"), default="both")
    p.add_argument("--include-reference", action="store_true")
    p.add_argument("--output", type=Path, default=Path("prototypes/navigation/output_v4/routes.json"))
    args = p.parse_args()

    engine = RouteEngine.from_files(args.elevation, args.meta, args.mask, args.risk_rover, args.risk_astronaut)
    rows, cols = engine.elevation.shape
    start = args.start or [rows // 2, cols // 10]
    goal = args.goal or [rows // 2, cols * 9 // 10]
    names = list(PROFILES) if args.profile == "both" else [args.profile]
    result = {"schema_version": "1.0", "results": {n: engine.compute(start, goal, n, args.include_reference) for n in names}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    print("Saved:", args.output)
    for name, item in result["results"].items():
        print(name, "risk source:", item["risk_source"])
        for key, route in item["routes"].items():
            print(f"  {key}: {route['status']}; distance={route.get('distance_m', '—')} m; "+
                  f"cost={route.get('weighted_cost_m_equiv', '—')}; time={route['runtime_ms']} ms")


if __name__ == "__main__":
    main()
