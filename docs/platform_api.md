# MarsEye integrated HTTP API

Base URL: `http://127.0.0.1:8001`. Coordinates are zero-based `[row,col]`, row increasing southward and column eastward. All requests use the processed DEM grid; no endpoint relocates blocked points.

## Terrain and point inspection

- `GET /api/health`: backend and DEM health.
- `GET /api/terrain/metadata`: grid shape, metre spacing, geographic bounds/affine transform, source, V4 profile parameters, risk sources, and `dataset_hash`.
- `GET /api/point?row=99&col=20`: location, DEM validity, elevation/slope/roughness, available CTX display intensity, available THEMIS brightness temperature in K and °C, and provenance. A missing observation is `null`; an out-of-grid point returns HTTP 422.

## A/B routes

`POST /api/route` retains the V4 contract in `docs/route_contract.md`. Request:

```json
{"start_rc":[99,20],"goal_rc":[99,180],"profile":"rover","include_reference":true}
```

The response contains `routes.shortest_feasible`, `routes.risk_aware`, and, when requested, `routes.unconstrained_reference`. Only `status: "found"` routes have a drawable path. The reference has `analysis_only: true`; it must never be used as a mission leg.

## Science mission

`POST /api/mission` request:

```json
{
  "start_rc":[99,20],"goal_rc":[99,180],
  "science_targets":[{"id":"S1","rc":[99,90]}],
  "profile":"rover","route_type":"risk_aware",
  "mode":"manual","manual_order":["S1"]
}
```

`science_targets` must contain 1–5 distinct IDs and cells. `route_type` is `shortest_feasible` or `risk_aware`; `mode` is `manual` or `optimized`. In manual mode, `manual_order` may be omitted to use the target array order. A and B stay fixed. The solver reuses the existing V4 `RouteProvider` and optimizes directed route costs; uphill and downhill costs may differ.

Successful response fields include `schema_version`, `status: "found"`, `start_rc`, `goal_rc`, `science_targets`, `profile`, `route_type`, `mode`, `visit_order`, `legs`, merged `points_rc`, `total_distance_m`, `total_weighted_cost_m_equiv`, `objective_cost`, `objective_unit`, `risk_source`, `experimental`, `dataset_hash`, and total `metrics`. Every leg includes path, distance, experimental cost, slope, roughness, ascent, runtime, and source where available. `objective_unit` is `m` for shortest feasible and `model_weighted_m_equiv` for risk aware; the latter is not physical energy.

An unreachable mission returns HTTP 200 with `status: "no_path"` or `"blocked_endpoint"`, empty merged points, and null totals. Malformed, duplicate, or out-of-grid inputs return HTTP 422. The frontend discards aborted or stale requests and does not invent a fallback path.
