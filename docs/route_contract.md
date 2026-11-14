# MarsEye — Navigation contract v1.0

**Status:** proposed and implemented in standalone V4 starter; team review required before merging.

## Reference terrain and grid

- Terrain: Gale Crater pilot region, current processed DEM (198 rows × 200 columns, ~5m per cell).
- Source: `data/processed/elevation.npy`, `terrain_meta.json`, `valid_mask.npy`.
- Coordinates are **zero-based `[row, col]` integers**, origin **upper left** of NumPy raster. Row increases downward in raster, column rightward. This is *not* Three.js world-space X/Y/Z.
- True spacing comes from `terrain_meta.json`: `dx_m` is distance per column, `dy_m` per row. Never assume 5.0 exactly.
- Frontend must obtain the same DEM-cell coordinate from its terrain raycast. Keep the frontend's axes and mesh origin aligned with this convention. An independent frontend mapping test is required.
- No rounding to a known-good point or silent fallback if A/B is outside the grid or blocked.

## HTTP API

Run `python prototypes/navigation/route_api.py` at repository root after installing FastAPI and Uvicorn. Dev API binds to `127.0.0.1:8001` and allows cross-origin access from the team's localhost frontend ports 8000 and 5173 only.

- `GET /api/health` — checks DEM input.
- `GET /api/terrain/metadata` — grid dimensions, cell spacing and risk source.
- `POST /api/route` — calculate **two** routes for a selected profile.

### Request

```json
{
  "start_rc": [99, 20],
  "goal_rc": [99, 180],
  "profile": "rover",
  "include_reference": false
}
```

`profile`: `rover` or `astronaut`. Frontend submits another request when profile changes. Two objectives per request.

### Response (field schematic, not real computed values)

```json
{
  "schema_version": "1.0",
  "terrain_id": "gale_5m_v1",
  "grid_shape": [198, 200],
  "dx_m": 5.003,
  "dy_m": 4.997,
  "coordinate_system": "row_col_0_based_origin_upper_left",
  "start_rc": [99, 20],
  "goal_rc": [99, 180],
  "profile": "rover",
  "experimental": true,
  "risk_source": "DEM_slope_roughness_proxy",
  "profile_parameters": {},
  "routes": {
    "shortest_feasible": {"status": "found", "points_rc": [[99, 20], [99, 21]], "distance_m": 5.003},
    "risk_aware": {"status": "found", "points_rc": [[99, 20], [99, 21]], "distance_m": 0}
  }
}
```

The `points_rc` arrays above are deliberately shortened for illustration; actual responses include every traversed cell from A to B. For **real** responses, `points_rc` contains every cell along the route, and metrics include:

- `distance_m`
- `mean_slope_deg`, `max_slope_deg`, `max_edge_grade_deg`
- `mean_roughness_m`, `elevation_gain_m`, `elevation_loss_m`
- `weighted_cost_m_equiv` (experimental generalized cost; **not physical energy**)
- `mean_risk_score_0_1` (`null` until an external 0–1 risk layer is supplied)
- `risk_source`, `runtime_ms`, `status`.

Failure results: `{"status":"blocked_endpoint","points_rc":[],"runtime_ms":...}` or `no_path`. Outside-grid and malformed coordinates cause HTTP 422.

## The route types

1. **`shortest_feasible`**: shortest horizontal travel distance subject to selected profile's cell slope, edge grade, NoData, and allowed-cell restrictions. Uses Dijkstra/A* with distance as edge cost and admissible Euclidean heuristic.
2. **`risk_aware`**: minimizes nonnegative experimental terrain cost under **the same traversability constraints**. With no external risk map, uses V3-style slope/roughness/ascent penalties. When an external risk layer is provided, uses the external 0–1 risk + uphill ascent; **does not repeat slope/roughness penalties**, to avoid double counting.
3. **`unconstrained_reference`**: only when `include_reference=true`; ignores profile limits and is marked `analysis_only:true`. **Never describe it as traversable or safe.**

Movement model: 8 neighbors; block diagonal corner cutting through blocked cells; reject edge grade >= configured maximum. The profile slope limits of 20° (rover) / 25° (astronaut) are **illustrative demo settings, not NASA-approved mission safety limits**. Report `mean_risk_score_0_1` only from supplied external layer, never by pretending the DEM proxy is a calibrated risk probability.

## Integration handoffs

**Frontend developer (1st-year Computer Engineering)**:

- In 3D raycast, map cursor to `[row,col]` consistently with current DEM, then call `POST /api/route`.
- Read and display both route keys. If `status != found`, show failure instead of drawing a route.
- Buttons: `Shortest Feasible`, `Lower Risk`; optional toggle `Reference (analysis only)`.
- Keep view layer toggles separate from route calculation. Optical/thermal texture is a *visual layer* unless a validated algorithm input is explicitly supplied.
- In development, frontend can stay on port 8000 while API runs on port 8001.

**2nd-year EEE (Risk Engine)**:

- Supply `data/processed/surface_risk_rover.npy` and `surface_risk_astronaut.npy` in the **same cell grid, orientation and shape** as `elevation.npy`.
- Each finite score is `0..1` where higher means more hazardous; use `NaN` for unknown. Unknown is **nontraversable for both route types** in the selected profile, pending explicit policy change.
- Supply `risk_config.json` documenting inputs, units, normalization, weights, provenance, and validation. A pixel-classification uncertainty should not be silently recoded as a physically quantified danger.
- The present V4 external-risk path assumes risk already accounts for slope/roughness and adds *ascent* independently. If the model includes ascent, coordinate with Navigation to prevent double counting.

**Other 4th-year EEE (Optical Analysis)**:

- Validate image-to-DEM cell alignment before exporting class probabilities.
- Classes/confidence are candidate inputs for Risk Engine, not automatically hard blocked terrain. Raw CTX brightness is not a reliable stand-alone sand/rock measurement.

**1st-year EEE (Science)**:

- Provide georeferenced and provenance-documented thermal/science layers; layer viewing does not imply these values are part of navigation weights.

## Suggested independent QA tests

1. Flat 20x20 DEM: both profile routes exist and are shortest at zero risk.
2. Cliff/cell barrier: both paths obey cell slope and edge grade limits.
3. Disconnected terrain: return `no_path` without crashing.
4. Same A/B and profile: shortest distance <= risk-aware distance (allow float tolerance).
5. External risk map with high-cost direct corridor: risk-aware should select another permissible corridor when it has lower risk cost.
6. Raster/Three.js round-trip: choose known corners and confirm `[row,col]` are correct.
