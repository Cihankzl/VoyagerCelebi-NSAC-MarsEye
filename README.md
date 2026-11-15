# voyager-nasa-apps-challenge-project
for voyager nasa apps challenge project
# MarsEye — NASA Space Apps Challenge 2026

**Challenge:** Interplanetary Survival Guide: Martian Map

## Project Overview

MarsEye is a 3D Mars exploration and mission-planning prototype that combines NASA terrain datasets with risk-aware pathfinding algorithms.

The system aims to help future astronauts explore scientifically valuable locations by comparing shortest routes with terrain-aware alternatives.

## Pilot Region

Gale Crater, Mars.

## Key Features

- 3D God's Eye View of Martian terrain
- NASA terrain and geological datasets
- Slope-based terrain risk analysis
- A* shortest-path and risk-aware navigation
- Route comparison and scientific exploration targets

## Current Development Status

**Initial navigation prototype completed.**

The A* pathfinding algorithm has been tested on synthetic terrain.

| Metric | Shortest Route | Risk-Aware Route |
|---|---:|---:|
| Distance | 178.00 m | 219.42 m |
| Maximum slope | 29.70° | 9.98° |

These are synthetic test results, **not measurements from Gale Crater**.

The next milestone is to integrate actual NASA elevation data and visualize both routes on a 3D Martian terrain model.


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
