# MarsEye — Mission Planning integration / team handoff

## Navigation owner: existing V4, unchanged

Added only `prototypes/navigation/route_provider.py` and tests. It imports **the existing `RouteEngine.compute()`** from `planner_v4.py`; it does **not** reimplement A*, slope, roughness or traversability. The DEM is loaded once through `RouteProvider.from_repo()` and stays resident for the lifetime of the provider.

```python
from prototypes.navigation.route_provider import RouteProvider

provider = RouteProvider.from_repo()
leg = provider.get_route([99, 20], [99, 180], "rover", "risk_aware")
print(leg["status"], leg["distance_m"], leg["objective_cost"])
```

Use `python -m prototypes.mission_planning.mission_planner` from the **repository root** to enable the `from prototypes...` import when you implement your own mission module.

## Mission planning teammate (2nd-year EEE) — your task

Implement **visit-order optimization**, **not A***. Start A, end B remain fixed. All selected science targets are mandatory in v1. A mission with 3–5 targets is enough.

1. Receive A, B and science target objects `{id, rc}` plus `profile` and `route_type`.
2. Create a directed pairwise cost matrix with `provider.build_cost_matrix(nodes, profile, route_type)`; the output has `cost_matrix[i][j]` for going from node i to node j. `None` means blocked/unreachable.
3. Optimize permutations of the target indices to minimize sum of pairwise objective costs, respecting fixed A and B.
4. Reconstruct the winning mission route by calling `provider.get_route(...)` for each consecutive directed pair; these calls reuse the **cached** V4 results.
5. Merge `points_rc` legs without repeating the connecting cell. Sum `distance_m` and `weighted_cost_m_equiv` separately; do not treat weighted cost as measured energy, time or probability of failure.
6. Return `visit_order`, `legs`, `points_rc`, `total_distance_m`, `total_weighted_cost_m_equiv`, `objective_cost`, `status`, `profile`, `route_type` in your `mission_route.json`. If no feasible ordering exists, return `no_path` without manufacturing a route.
7. For `manual` mode, preserve the order explicitly chosen by the user; for `optimized`, find the best order.

### Examples

```python
nodes = [
    {"id": "A", "rc": [99, 20]},
    {"id": "S1", "rc": [45, 65]},
    {"id": "S2", "rc": [110, 100]},
    {"id": "B", "rc": [99, 180]},
]
provider = RouteProvider.from_repo()
mat = provider.build_cost_matrix(nodes, "rover", "risk_aware")
# matrix is indexed by the order in `nodes`; cells can be None.
# mat["cost_matrix"][0][1] = cost(A -> S1)
# mat["cost_matrix"][1][0] = cost(S1 -> A), not assumed equal!
```

`route_type="shortest_feasible"` gives an objective in **metres**; `risk_aware` gives the **experimental model weighted metre-equivalent**, not physical energy. Never compare cost figures between profiles as if calibrated. `unconstrained_reference` is intentionally unavailable from the provider because it must not be used for mission planning.

## Common coordinates and caveats

- Pilot DEM grid currently 198 rows x 200 columns; *read shape, do not hardcode it*.
- Coordinates: zero-based NumPy raster `[row,col]`, origin top-left.
- Profile restrictions include **cell slope, edge grade, no corner cutting, DEM validity**; optional external risk maps can further restrict cells.
- V4's profile limits are **demonstration assumptions**, not certified Mars rover/EVA safety criteria.
- A -> S1 may cost differently from S1 -> A because uphill ascent contributes to the cost.
- If A/B or a target is blocked for the profile, V4 returns `blocked_endpoint`. Do not silently shift targets.
- `build_cost_matrix` calls V4 **once per directed pair** and caches both route types. Initial matrix creation may take time on a real DEM; permutations must use the cached matrix instead of rerunning A*.
- `get_route` contains `status`, `points_rc`, `distance_m`, `weighted_cost_m_equiv`, `objective_cost`, `objective_unit`, `runtime_ms`, `risk_source`, etc.
- Cache validity: `RouteProvider` assumes engine data and profile settings do not change while it lives. After changing risk maps/profiles, build a new provider or clear the cache.
- API endpoint `POST /api/mission` and frontend multi-stop selection are **not yet implemented**. They belong to later integration with the Navigation and Frontend owners.

## Tests / getting started

From repository root (PowerShell):

```powershell
python .\prototypes\navigation\test_route_provider.py
python .\prototypes\navigation\route_provider.py --start 99 20 --goal 99 180 --profile rover --route-type risk_aware
```

Expected for the user's current Gale processed DEM: `found`, distance ~874.981 m, cost ~2135.994 (small differences possible only if data/parameters change).
