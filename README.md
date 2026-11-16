# MarsEye

**3D mission planning and navigation for Gale Crater, Mars, built on real NASA orbital data.**
NASA Space Apps Challenge 2026 · Challenge: Interplanetary Survival Guide: Martian Map · Team Voyager

🔗 **Live demo:** https://cihankzl.github.io/VoyagerCelebi-NSAC-MarsEye/frontend/

🎬 **30-second demo video:** https://youtu.be/VrQ41ySCDbo

![MarsEye demo](docs/demo.gif)

MarsEye turns NASA terrain, optical and thermal data of a ~1 km area in Gale Crater into an interactive 3D map.
Pick a start and a goal, and MarsEye computes three routes side by side: **Shortest**, **Astronaut Safe** and
**Rover Optimal**. It weighs slope, roughness, uphill climb and night-time cold traps, and shows each route's
elevation profile, so a mission planner can see the trade-off between distance and risk at a glance.
The live demo runs fully in the browser; no installation is needed.

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

## NASA data used

| Layer | NASA source | Details | How MarsEye uses it |
|---|---|---|---|
| Elevation (DEM) | Exported via [NASA Mars Trek](https://trek.nasa.gov/mars/) | `data/raw/topographic/gale_dem_1024.tif`, 1024 × 1024 px (~0.98 m/px), float32 metres. Exact source product ID: *to be confirmed*. Southern ~25 % is coarse fill. | Averaged to a 200 × 200 routing grid; slope, roughness and climb drive the route cost. |
| Optical | MRO Context Camera (CTX) Gale mosaic, via [NASA Mars Trek](https://trek.nasa.gov/mars/) | `data/raw/optical/optik_ctx_1024.tif`, 8-bit, same grid as the DEM, verified aligned at zero shift. | Texture draped on the 3D terrain. |
| Thermal (night, Kelvin) | Mars Odyssey THEMIS IR brightness temperature, PDS product `I01350002PBT` (data set `ODY-M-THM-5-IRPBT-V2.0`), Band 9, 100 m/px, 2002-04-04, local time 03:15 | `data/raw/thermal/I01350002PBT.IMG` | Cold-trap penalty in the risk-aware route cost and temperature telemetry. |
| Thermal (day image) | Mars Odyssey THEMIS daytime IR, via [NASA Mars Trek](https://trek.nasa.gov/mars/) | `data/raw/thermal/themis_ir_day_1024.tif`, 8-bit picture, no temperature units | Visual layer only. |

All layers share one bounding box: 137.5195–137.5365 °E, −4.7010 to −4.6842 °N (Mars 2000 sphere).
More detail: [`data/raw/README.md`](data/raw/README.md) and [`data/raw/thermal/README.md`](data/raw/thermal/README.md).

**Data credits:** NASA/JPL-Caltech/MSSS (MRO CTX); NASA/JPL-Caltech/Arizona State University (Mars Odyssey THEMIS);
NASA Mars Trek (NASA Solar System Treks Project). THEMIS viewer: https://viewer.mars.asu.edu/viewer/themis

**Other open-source tools:** [Three.js](https://threejs.org/) 0.180 (3D rendering), NumPy, Pillow, FastAPI, Uvicorn.

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

## Limitations

- Route cost weights and slope limits (rover 20°, astronaut 25°) are experimental demo values, not NASA-certified traversability or EVA limits.
- THEMIS values are orbital brightness temperatures from a single night-time pass (2002), not ground or air temperatures.
- The thermal grid is 100 m/px, so the ~1 km study area holds only about 10 × 10 thermal pixels.
- The southern ~25 % of the DEM is coarse fill outside the high-resolution strip.

## Use of AI

We used AI tools during this hackathon, and we disclose them here as the Space Apps rules require:

- **Claude (Anthropic):** helped write and debug parts of the code (frontend, data-preparation scripts, route API), check data alignment, and draft documentation.
- **GitHub Copilot:** code suggestions and repository clean-up.

The team chose the idea, the NASA datasets, the route criteria and the design, and reviewed and tested the AI-assisted code.
No images or videos in this project were generated by AI, and AI was not used to create or modify any NASA branding.

## Team

Team **Voyager**, NASA Space Apps Challenge 2026, Izmir

| Name | Role |
|---|---|
| akif        |  Navigation algorithms |
| mustafa     |  Mission planning      |
| cihan       |  Frontend / 3D         |
| gokay&osman |  Data acquisition      |

## License

MIT, see [LICENSE](LICENSE). NASA data is used under NASA's media and data usage guidelines.
