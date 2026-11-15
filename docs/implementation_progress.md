# MarsEye implementation progress

## Milestone 0 — Repository audit and isolated toolchain

- Branch: `feature/marseye-integrated-platform`, based on verified `origin/main` commit `81b5cfc`; starting worktree was clean.
- Preserved the V3 demo, Navigation V4 engine, and all raw scientific rasters.
- Installed official portable CPython 3.12.10 and Node.js 24.22.3 under ignored `.tools/`; created ignored `.venv/` and frontend `node_modules/`.
- Pinned Python direct and transitive dependencies, plus npm dependencies through `package-lock.json`.
- Verification: Python interpreter and pip run; Node/npm run; 29 selected navigation, mission, and real-data platform tests passed; frontend lint and production build passed. Browser automation host is unavailable; no browser interaction result is claimed.
- Commit SHA: recorded in the next checkpoint after commit creation.

## Remaining checkpoints

- Prepare and verify reproducible CTX/THEMIS web assets and point samples.
- Deliver the integrated 3D frontend, V4 routes, and real multi-stop mission API.
- Complete final regression, documentation, local smoke checks, and pull request.

## Known scientific limitations

- DEM and CTX source product IDs are unverified.
- Route slope limits and cost weights are experimental. No validated external risk map is present.
- THEMIS brightness temperatures retain ~100 m source resolution despite display resampling.
