# MarsEye

MarsEye is a 3D Mars exploration and mission-planning prototype for Gale Crater. It combines Mars terrain data with route planning to visualize terrain-aware paths.

## Quick start

From the repository root:

```bash
pip install -r requirements.txt
python -m http.server 8000
open http://localhost:8000/frontend/
```

The Python API is optional:

```bash
python prototypes/navigation/route_api.py
```

It runs FastAPI on port 8001.

## Repository layout

| Folder | Purpose |
|---|---|
| `frontend/` | The current MarsEye web app. |
| `data/` | Raw source data and processed app data. |
| `prototypes/navigation/` | Navigation planners, route services, and API. |
| `v4/mission_planning/` | Mission-planning module and tests. |
| `scripts/` | Data preparation and web asset scripts. |
| `docs/` | Data sources, architecture, handoffs, and route contracts. |
| `archive/` | Historical and misplaced files; **not part of the app**. |

## Data sources

- The topographic input is `data/raw/topographic/gale_dem_1024.tif`. It is processed into the routing grid by `scripts/prepare_terrain_grid.py`; its exact originating NASA/USGS product identifier still needs confirmation.
- `data/raw/optical/optik_ctx_1024.tif` is the MRO CTX Gale mosaic and is documented as aligned with the DEM.
- The THEMIS daytime IR image is used only as a visual layer and has no temperature units. Routing temperatures come from the night THEMIS BTR `data/raw/thermal/I01350002PBT.IMG` in Kelvin.
- Source context: [NASA Mars Trek](https://trek.nasa.gov/mars/) and the [NASA Mars Reconnaissance Orbiter Context Camera](https://science.nasa.gov/mission/mars-reconnaissance-orbiter/).


## Prototype V1 — run it (3D God's Eye app)

All commands from the **repository root**.

```bash
pip install -r requirements.txt
python scripts/prepare_web_assets.py          # optical/thermal textures, thermal grid, static fallback JSONs
python prototypes/navigation/route_api.py     # terminal 1: FastAPI on http://127.0.0.1:8001 (docs at /docs)
python -m http.server 8000                    # terminal 2: static server for the repo root
```

Open **http://localhost:8000/frontend/**. If the API is not running (or on GitHub Pages) the page
switches to the in-browser JS planner automatically (same cost model, same output format).

| Piece | File |
|---|---|
| Frontend (Three.js via importmap, no npm) | `frontend/index.html`, `frontend/main.js`, `frontend/style.css`, `frontend/planner.js`, `frontend/chart.js` |
| Asset pipeline | `scripts/prepare_web_assets.py` |
| A* engine (+ thermal criterion, precomputed move tables) | `prototypes/navigation/planner_v4.py` |
| 3 primary routes + multi-stop mission | `prototypes/navigation/mission_service.py` (uses `route_provider.py` + `v4/mission_planning/mission_planner.py`) |
| API (`/api/health`, `/api/terrain/metadata`, `/api/route`, `/api/mission`) | `prototypes/navigation/route_api.py` |
| Static fallback data | `data/processed/terrain.json`, `routes.json`, `mission_route.json`, `thermal/thermal_grid.json` |

The 3 routes: **Shortest** = `astronaut` profile `shortest_feasible` (distance only, 25° limit);
**Astronaut Safe** = `astronaut` `risk_aware`; **Rover Optimal** = `rover` `risk_aware`.
Risk-aware cost = slope² + roughness + uphill climb + THEMIS night-time cold-trap penalty
(experimental weights, not NASA-certified limits).

Tests:

```bash
cd prototypes/navigation
python test_planner_v4.py && python test_route_provider.py && python test_navigation_validation.py && python test_benchmark_navigation.py && python test_mission_api.py
cd ../../v4/mission_planning && python test_mission_planner.py
```
