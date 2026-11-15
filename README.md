# MarsEye — Gale Mission Atlas

MarsEye is a bilingual (Turkish/English) 3D exploration and experimental mission-planning application for a selected ~1 × 1 km area of Gale Crater. It uses the repository's real DEM, MRO CTX optical raster, Mars Odyssey THEMIS Band 9 brightness temperatures, and the existing Navigation V4 engine. Profile limits and weighted costs are illustrative; routes are **not approved mission plans**.

## Local setup (Windows PowerShell)

Use Python 3.12 and Node.js 24. Install their official distributions if they are not already available. Virtual environments, `node_modules`, and any portable runtimes under `.tools` are ignored by Git.

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
.\.venv\Scripts\python.exe scripts\prepare_web_data.py
cd frontend\marseye_app
npm ci
npm run lint
npm run build
```

Open two PowerShell terminals at the repository root:

```powershell
.\.venv\Scripts\python.exe prototypes\navigation\route_api.py
```

```powershell
cd frontend\marseye_app
npm run preview
```

Visit `http://127.0.0.1:4173`. For live source reload, use `npm run dev` on port 5173; the production preview is the verified local path in this environment. Set `VITE_API_BASE` in `frontend/marseye_app/.env.local` if the API runs elsewhere. The API is at `http://127.0.0.1:8001/docs`.

## Demo workflow

1. Choose **Start A**, then click a valid terrain cell. Choose **Goal B** and click another cell.
2. Select Rover or Astronaut, then calculate routes. Toggle the shortest reference, shortest feasible, and lowest model-cost paths independently. The reference is analysis only.
3. Select Optical or Thermal to inspect actual aligned observations. Click the terrain for elevation, slope, roughness, CTX intensity, and available THEMIS brightness temperature.
4. Add up to five proposed science points. Use manual order or optimized order and calculate a multi-leg V4 mission route.
5. Switch between Turkish and English in the header.

## Verification

```powershell
.\.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider prototypes\navigation\test_planner_v4.py prototypes\navigation\test_navigation_validation.py prototypes\navigation\test_route_provider.py prototypes\navigation\test_benchmark_navigation.py v4\mission_planning\test_mission_planner.py tests\test_platform_api.py
cd frontend\marseye_app
npm test
npm run lint
npm run build
```

The original V3 demo remains at `frontend/marseye_v3_demo/`. To view it, run `python -m http.server 8000` from the repository root and open `http://127.0.0.1:8000/frontend/marseye_v3_demo/`.

See [platform API](docs/platform_api.md), [scientific sources](docs/sources.md), and [implementation progress](docs/implementation_progress.md).
