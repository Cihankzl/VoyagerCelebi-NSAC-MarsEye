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
    # Thermal (THEMIS night-time brightness temperature) penalty weight.
    # Applied only when the engine is given a thermal layer; colder cells
    # ("cold traps", low thermal inertia proxy) cost more. EXPERIMENTAL.
    thermal_weight: float = 1.0


PROFILES = {
    "rover": Profile(20.0, 20.0, 2.0, 4.0, 0.8, thermal_weight=1.2),
    "astronaut": Profile(25.0, 25.0, 2.7, 1.0, 4.0, thermal_weight=1.0),
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
                      step_m: float, risk_a: float | None, risk_b: float | None,
                      cold_a: float | None = None, cold_b: float | None = None) -> float:
    """Generalized step cost. cold_a/cold_b are optional 0..1 thermal cold indices
    (0 = warmest cell of the pilot area, 1 = coldest). Without a thermal layer the
    cost is identical to the original V4 model."""
    ascent = max(0.0, z_b - z_a)
    if risk_a is not None:
        # External risk already contains the terrain components; do not double count them.
        penalty = profile.external_risk_weight * 0.5 * (risk_a + risk_b)
    else:
        avg_slope = 0.5 * (slope_a + slope_b)
        avg_rough = 0.5 * (rough_a + rough_b)
        penalty = profile.slope_weight * (avg_slope / 15.0) ** 2 + profile.roughness_weight * (avg_rough / 1.0)
    if cold_a is not None and cold_b is not None:
        # Thermal is a separate physical input, so it is added in both risk modes.
        penalty = penalty + profile.thermal_weight * 0.5 * (cold_a + cold_b)
    return step_m * (1.0 + penalty) + profile.ascent_weight * ascent


def load_thermal(path: Path | None, shape: tuple[int, int]) -> np.ndarray | None:
    """Load a (rows, cols) brightness-temperature grid in Kelvin aligned to the DEM."""
    if path is None or not Path(path).is_file():
        return None
    arr = np.load(path, allow_pickle=False).astype(np.float64)
    if arr.shape != shape:
        raise ValueError(f"Thermal grid {path} shape {arr.shape} != terrain {shape}")
    arr[(arr <= -9000) | ~np.isfinite(arr)] = np.nan  # never treat -9999 as a temperature
    return arr


class RouteEngine:
    def __init__(self, elevation: np.ndarray, meta: dict,
                 mask: np.ndarray | None = None,
                 risk_maps: dict[str, np.ndarray | None] | None = None,
                 thermal_k: np.ndarray | None = None):
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
        # Optional thermal layer (Kelvin). cold_index: 0 warmest .. 1 coldest of the area.
        self.thermal_k: np.ndarray | None = None
        self.cold_index: np.ndarray | None = None
        if thermal_k is not None:
            t = np.asarray(thermal_k, dtype=np.float64).copy()
            if t.shape != self.elevation.shape:
                raise ValueError("Thermal grid shape differs from elevation")
            t[(t <= -9000) | ~np.isfinite(t)] = np.nan
            if np.isfinite(t).any():
                lo, hi = float(np.nanmin(t)), float(np.nanmax(t))
                cold = (hi - t) / (hi - lo) if hi > lo else np.zeros_like(t)
                # Unknown temperature: neutral mid value, not a hidden block.
                cold = np.where(np.isfinite(cold), np.clip(cold, 0.0, 1.0), 0.5)
                self.thermal_k, self.cold_index = t, cold
        self._table_cache: dict[str, Any] = {}

    def step_cost(self, profile: Profile, a, b, step_m: float, risk: np.ndarray | None) -> float:
        """Exact cost of one move a->b (used by A*, summaries and validation)."""
        ra = float(risk[a]) if risk is not None and np.isfinite(risk[a]) else None
        rb = float(risk[b]) if risk is not None and np.isfinite(risk[b]) else None
        ca = float(self.cold_index[a]) if self.cold_index is not None else None
        cb = float(self.cold_index[b]) if self.cold_index is not None else None
        return risk_cost_of_step(profile, float(self.slope[a]), float(self.slope[b]),
                                 float(self.roughness[a]), float(self.roughness[b]),
                                 float(self.elevation[a]), float(self.elevation[b]), float(step_m),
                                 ra, rb, ca, cb)

    @classmethod
    def from_files(cls, elevation: Path, meta: Path, mask: Path | None,
                   risk_rover: Path | None = None, risk_astronaut: Path | None = None,
                   thermal: Path | None = None):
        data = np.load(elevation, allow_pickle=False)
        m = json.loads(meta.read_text(encoding="utf-8"))
        if "cell_size_m" in m:
            m.setdefault("dx_m", m["cell_size_m"])
            m.setdefault("dy_m", m["cell_size_m"])
        vm = np.load(mask, allow_pickle=False) if mask is not None else None
        risks = {"rover": load_risk(risk_rover, data.shape),
                 "astronaut": load_risk(risk_astronaut, data.shape)}
        return cls(data, m, vm, risks, load_thermal(thermal, data.shape))

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
                    cost = self.step_cost(profile, node, nxt, step_m, risk)
                new_g = g + cost
                if new_g + 1e-9 < best.get(nxt, math.inf):
                    best[nxt] = new_g
                    parent[nxt] = node
                    heapq.heappush(q, (new_g + heuristic(nxt), new_g, nxt))
        return None, "no_path"

    def _move_tables(self, profile_name: str, allowed, profile, risk):
        """Precompute legal moves and both edge costs once per profile.

        Exactly the same rules as _legal_edge / step_cost (8 neighbours, no corner
        cutting, edge-grade limit), stored as plain lists so A* avoids per-edge
        function calls. Cached for the engine's lifetime (engine data is immutable).
        """
        cached = self._table_cache.get(profile_name)
        if cached is not None:
            return cached
        rows, cols = self.elevation.shape
        neighbours: list[list[tuple[int, float, float]]] = [[] for _ in range(rows * cols)]
        al = allowed.tolist()
        for r in range(rows):
            row_ok = al[r]
            for c in range(cols):
                if not row_ok[c]:
                    continue
                node = (r, c)
                out = neighbours[r * cols + c]
                for dr, dc in MOVES:
                    nxt = (r + dr, c + dc)
                    if not self._legal_edge(node, nxt, allowed, profile.max_edge_grade_deg):
                        continue
                    step_m = math.hypot(dr * self.dy, dc * self.dx)
                    out.append((nxt[0] * cols + nxt[1], step_m, self.step_cost(profile, node, nxt, step_m, risk)))
        self._table_cache[profile_name] = neighbours
        return neighbours

    def _astar_fast(self, neighbours, allowed, start, goal, objective):
        """A* over precomputed tables; identical expansion order to _astar
        (node index ordering == (row, col) tuple ordering)."""
        if not self._legal_node(start, allowed) or not self._legal_node(goal, allowed):
            return None, "blocked_endpoint"
        if start == goal:
            return [start], "found"
        rows, cols = self.elevation.shape
        dx, dy = self.dx, self.dy
        gr, gc = goal
        hypot = math.hypot
        s_idx, g_idx = start[0] * cols + start[1], gr * cols + gc
        use_dist = objective == "distance"
        best = [math.inf] * (rows * cols)
        parent = {}
        best[s_idx] = 0.0
        q = [(hypot((gr - start[0]) * dy, (gc - start[1]) * dx), 0.0, s_idx)]
        pop, push = heapq.heappop, heapq.heappush
        while q:
            _, g, u = pop(q)
            if g > best[u] + 1e-9:
                continue
            if u == g_idx:
                path = [u]
                while path[-1] != s_idx:
                    path.append(parent[path[-1]])
                return [divmod(i, cols) for i in path[::-1]], "found"
            for v, step_m, risk_cost in neighbours[u]:
                new_g = g + (step_m if use_dist else risk_cost)
                if new_g + 1e-9 < best[v]:
                    best[v] = new_g
                    parent[v] = u
                    vr, vc = divmod(v, cols)
                    push(q, (new_g + hypot((gr - vr) * dy, (gc - vc) * dx), new_g, v))
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
            weighted_cost += self.step_cost(profile, a, b, float(d), risk)
        if self.thermal_k is not None:
            t = self.thermal_k[rr, cc]
            t = t[np.isfinite(t)]
        else:
            t = np.array([])
        thermal = {
            "mean_temp_k": round(float(t.mean()), 2) if t.size else None,
            "min_temp_k": round(float(t.min()), 2) if t.size else None,
            "max_temp_k": round(float(t.max()), 2) if t.size else None,
            "mean_temp_c": round(float(t.mean()) - 273.15, 2) if t.size else None,
            "min_temp_c": round(float(t.min()) - 273.15, 2) if t.size else None,
            "max_temp_c": round(float(t.max()) - 273.15, 2) if t.size else None,
            "mean_cold_index_0_1": round(float(np.mean(self.cold_index[rr, cc])), 4) if self.cold_index is not None else None,
        }
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
            **thermal,
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
            "terrain_id": "gale_5m_v1" if self.elevation.shape in ((198, 200), (200, 200)) else "custom_grid",
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
            tables = self._move_tables(profile_name, allowed, profile, risk)
            path, status = self._astar_fast(tables, allowed, start, goal, objective)
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
