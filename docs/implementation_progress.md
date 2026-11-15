# MarsEye implementation progress

## Milestone 0 — Repository audit and isolated toolchain

- Branch: `feature/marseye-integrated-platform`, based on verified `origin/main` commit `81b5cfc`; starting worktree was clean.
- Preserved the V3 demo, Navigation V4 engine, and all raw scientific rasters.
- Installed official portable CPython 3.12.10 and Node.js 24.22.3 under ignored `.tools/`; created ignored `.venv/` and frontend `node_modules/`.
- Pinned Python direct and transitive dependencies, plus npm dependencies through `package-lock.json`.
- Verification at this checkpoint: Python interpreter and pip run; Node/npm run; 29 selected navigation, mission, and real-data platform tests passed; frontend lint and production build passed. Browser testing was performed later through native Floorp control.
- Commit SHA: `2be7e98`; pushed to `origin/feature/marseye-integrated-platform`.

## Checkpoint 1 — Reproducible scientific web data

- Added a raster preparation script, aligned CTX and THEMIS PNG layers, terrain JSON, provenance manifest, and aligned point samples.
- Confirmed source raster corner/center mapping and a thermal center sample against the clipped GeoTIFF; preserved thermal NoData transparency.
- Generated 198 × 200 DEM, 39,600 optical display cells, and 36,288 thermal display cells. Dataset hash: `03b9a9b46c1659b2`.
- Verification: preparation command succeeded; real-data/API tests 6/6 passed (one dependency deprecation warning). Existing raw rasters were not changed.
- Commit SHA: `af4ba95`; pushed to `origin/feature/marseye-integrated-platform`.

## Milestone 2 — Integrated Gale explorer and V4 mission planning

- Built `frontend/marseye_app/src/` as a separate React/TypeScript/Three.js application. The original V3 demo remains untouched.
- Rendered the real Gale DEM with terrain picking, orbit controls, markers, animated route overlays, bilingual navigation/mission panels, tablet layout, profile limits, and documented scientific uncertainty.
- Connected Topographic, CTX optical, and THEMIS brightness-temperature layers with legends and opacity; point inspection reads aligned real samples from the API.
- Extended the existing V4 FastAPI service with point inspection and multi-stop mission endpoints. The production mission path reuses `RouteProvider` and `MultiStopMissionPlanner`; the V4 A* engine is unchanged. Shortest Reference remains analysis-only and cannot be a mission leg.
- Added API/geospatial regression checks, coordinate conversion tests, local setup/API documentation, and real-data browser smoke verification.
- Verification: 35 selected Python tests passed (one third-party Starlette deprecation warning), two frontend tests passed, ESLint passed, and TypeScript/Vite production build passed. Live API returned all three real V4 routes and an optimized five-target/six-leg mission; API and asset dataset hashes matched `03b9a9b46c1659b2`. Native Floorp QA covered A/B selection, point inspection, profile recalculation, route toggles, thermal/optical layers, science missions, language switching, and a stacked 760 px layout.
- Commit SHA: `422f40a`; pushed to `origin/feature/marseye-integrated-platform`.

## Next checkpoint

- Prepare a pull request from the feature branch without merging into `main`; then record its URL and final handoff state.

## Known scientific limitations

- DEM and CTX source product IDs are unverified.
- Route slope limits and cost weights are experimental. No validated external risk map is present.
- THEMIS brightness temperatures retain ~100 m source resolution despite display resampling.
- Vite's live development server failed dependency optimization in this sandbox due to ancestor-directory access; the production build and preview worked.
