"""MarsEye V4 -> Multi-Stop Mission Planner adapter.

- Loads the SAME RouteEngine used by route_api.py once.
- Exposes directed A -> B legs for shortest_feasible / risk_aware.
- Caches the two V4 route results per directed (A, B, profile) pair.
- Keeps a clear difference between distance (m) and experimental risk cost
  (weighted m-equivalent); neither is a certified safety measure.

This module does NOT optimize science-target visit order. The separate
mission_planner.py, owned by the Mission Planning teammate, will do that.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

try:  # Python -m prototypes.navigation.route_provider
    from .planner_v4 import RouteEngine, PROFILES
except ImportError:  # python prototypes/navigation/route_provider.py
    from planner_v4 import RouteEngine, PROFILES

ROUTE_TYPES = ("shortest_feasible", "risk_aware")


def _rc(point: object, label: str) -> tuple[int, int]:
    """Zero-based [row, col]; reject silently rounded or reordered indices."""
    if not isinstance(point, (tuple, list)) or len(point) != 2:
        raise ValueError(f"{label} must be [row, col]")
    if any(type(v) is not int for v in point):
        raise ValueError(f"{label} must contain two integer indices")
    return int(point[0]), int(point[1])


class RouteProvider:
    """Stable Python interface for the mission-planning teammate.

    Public methods:
      get_route(start_rc, goal_rc, profile='rover', route_type='risk_aware')
      build_cost_matrix(nodes, profile='rover', route_type='risk_aware')
      clear_cache()
    """

    def __init__(self, engine: RouteEngine):
        self.engine = engine
        self._pair_cache: dict[tuple[tuple[int, int], tuple[int, int], str], dict[str, Any]] = {}
        self.engine_compute_calls = 0  # diagnostic only

    @classmethod
    def from_repo(cls, repo_root: str | Path | None = None, *,
                  risk_rover: str | Path | None = None,
                  risk_astronaut: str | Path | None = None,
                  use_thermal: bool = True) -> "RouteProvider":
        """Load actual repo data ONCE; paths to optional risk maps are repo-relative.

        use_thermal: when data/processed/thermal/thermal_temp_k.npy exists (made by
        scripts/prepare_web_assets.py), add the THEMIS thermal criterion to risk_aware.
        """
        root = Path(repo_root).resolve() if repo_root is not None else Path(__file__).resolve().parents[2]
        data = root / "data" / "processed"
        files = [data / "elevation.npy", data / "terrain_meta.json", data / "valid_mask.npy"]
        missing = [str(f) for f in files if not f.is_file()]
        if missing:
            raise FileNotFoundError("MarsEye DEM inputs not found: " + ", ".join(missing))

        def optional_path(value: str | Path | None) -> Path | None:
            if value is None:
                return None
            path = Path(value)
            return path if path.is_absolute() else root / path

        engine = RouteEngine.from_files(
            elevation=files[0], meta=files[1], mask=files[2],
            risk_rover=optional_path(risk_rover),
            risk_astronaut=optional_path(risk_astronaut),
            thermal=(data / "thermal" / "thermal_temp_k.npy") if use_thermal else None,
        )
        return cls(engine)

    def clear_cache(self) -> None:
        """Clear cached routes after modifying an engine's risk maps or profiles."""
        self._pair_cache.clear()

    def get_route(self, start_rc: Sequence[int], goal_rc: Sequence[int],
                  profile: str = "rover", route_type: str = "risk_aware") -> dict[str, Any]:
        """Return one directed A->B leg, with both display and optimization metrics.

        On no_path / blocked_endpoint, returns status and empty points_rc, and
        objective_cost=None. The mission solver should treat such edges as absent.
        """
        if profile not in PROFILES:
            raise ValueError(f"profile must be one of {tuple(PROFILES)}")
        if route_type not in ROUTE_TYPES:
            raise ValueError(f"route_type must be one of {ROUTE_TYPES}")
        start = _rc(start_rc, "start_rc")
        goal = _rc(goal_rc, "goal_rc")
        key = (start, goal, profile)  # Direction matters: ascent cost is directional.
        if key not in self._pair_cache:
            self._pair_cache[key] = self.engine.compute(list(start), list(goal), profile, False)
            self.engine_compute_calls += 1
        result = self._pair_cache[key]
        route = deepcopy(result["routes"][route_type])
        found = route.get("status") == "found"
        if route_type == "shortest_feasible":
            objective_cost = route.get("distance_m") if found else None
            objective_unit = "m"
        else:
            objective_cost = route.get("weighted_cost_m_equiv") if found else None
            objective_unit = "model_weighted_m_equiv"
        return {
            "schema_version": "1.0",
            "terrain_id": result["terrain_id"],
            "profile": profile,
            "route_type": route_type,
            "start_rc": list(start),
            "goal_rc": list(goal),
            "risk_source": result["risk_source"],
            "experimental": result["experimental"],
            **route,
            "objective_cost": objective_cost,
            "objective_unit": objective_unit,
        }

    def build_cost_matrix(self, nodes: Sequence[Mapping[str, object]],
                          profile: str = "rover", route_type: str = "risk_aware") -> dict[str, Any]:
        """Directed matrix for A + science targets + B; unreachable -> null.

        nodes: [{"id":"A", "rc":[99,20]}, {"id":"S1", "rc":[...]} ...]
        Matrix index i->j means traveling from nodes[i] to nodes[j].
        Diagonal is zero. Never assume cost[i][j] == cost[j][i].
        """
        checked: list[dict[str, object]] = []
        seen: set[str] = set()
        for i, node in enumerate(nodes):
            if not isinstance(node, Mapping):
                raise ValueError(f"nodes[{i}] must be an object with id and rc")
            node_id = node.get("id")
            if not isinstance(node_id, str) or not node_id.strip() or node_id in seen:
                raise ValueError(f"nodes[{i}].id must be a unique nonempty string")
            seen.add(node_id)
            checked.append({"id": node_id, "rc": list(_rc(node.get("rc"), f"nodes[{i}].rc"))})
        if len(checked) < 2:
            raise ValueError("Expected at least two nodes (A and B)")
        if profile not in PROFILES or route_type not in ROUTE_TYPES:
            raise ValueError("Unknown profile or route_type")
        n = len(checked)
        matrix: list[list[float | None]] = [[0.0 if i == j else None for j in range(n)] for i in range(n)]
        for i in range(n):
            for j in range(n):
                if i == j:
                    continue
                leg = self.get_route(checked[i]["rc"], checked[j]["rc"], profile, route_type)
                matrix[i][j] = leg["objective_cost"]
        return {
            "schema_version": "1.0",
            "profile": profile,
            "route_type": route_type,
            "objective_unit": "m" if route_type == "shortest_feasible" else "model_weighted_m_equiv",
            "nodes": checked,
            "cost_matrix": matrix,
            "unreachable_value": None,
            "directed": True,
        }


def main() -> None:
    parser = argparse.ArgumentParser(description="Test MarsEye V4 route provider on the local DEM")
    parser.add_argument("--start", nargs=2, type=int, default=[99, 20], metavar=("ROW", "COL"))
    parser.add_argument("--goal", nargs=2, type=int, default=[99, 180], metavar=("ROW", "COL"))
    parser.add_argument("--profile", choices=tuple(PROFILES), default="rover")
    parser.add_argument("--route-type", choices=ROUTE_TYPES, default="risk_aware")
    parser.add_argument("--repo-root", type=Path, default=None)
    args = parser.parse_args()
    provider = RouteProvider.from_repo(args.repo_root)
    route = provider.get_route(args.start, args.goal, args.profile, args.route_type)
    summary = {
        "status": route["status"],
        "profile": route["profile"],
        "route_type": route["route_type"],
        "start_rc": route["start_rc"],
        "goal_rc": route["goal_rc"],
        "distance_m": route.get("distance_m"),
        "weighted_cost_m_equiv": route.get("weighted_cost_m_equiv"),
        "objective_cost": route["objective_cost"],
        "objective_unit": route["objective_unit"],
        "point_count": len(route.get("points_rc", [])),
        "risk_source": route["risk_source"],
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
