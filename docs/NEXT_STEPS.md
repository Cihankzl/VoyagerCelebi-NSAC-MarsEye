# MarsEye continuation handoff

Updated: 2026-11-15 (Europe/Istanbul). Worktree: `C:\Users\ghost\Desktop\coding\marseye-platform`. Branch: `feature/marseye-integrated-platform`. Never merge automatically into `main` or modify the V3 demo/raw rasters.

## Verified checkpoint

- Commit `2be7e98` (`chore: establish isolated MarsEye toolchain`) was pushed to `origin/feature/marseye-integrated-platform`.
- Scientific web data is prepared and verified; its checkpoint commit SHA will be recorded after creation.
- Official portable CPython 3.12.10 and Node.js 24.22.3 were unpacked under ignored `.tools/`. `.venv/` and frontend `node_modules/` are also ignored.
- Selected Python tests: 29 passed, 1 third-party deprecation warning. Frontend lint and production build passed before the later small changes. Do not treat subsequent unstaged code as verified until checks rerun.
- `frontend/marseye_v3_demo/` and original scientific rasters were not changed.

## Work currently in progress (uncommitted)

- FastAPI point inspection and real V4 multi-stop mission endpoint in `prototypes/navigation/route_api.py`.
- Small integration fixes in `v4/mission_planning/mission_planner.py`.
- New bilingual React/Three.js application in `frontend/marseye_app/src/` and scientific/API documentation.
- Real DEM platform tests under `tests/`.

## Current issues and limits

- The frontend Vite development server's dependency optimization failed in this sandbox with an ancestor-directory access error. `npm run build` and `npm run preview` work; use the production preview for local verification here.
- The browser access host was unavailable, but native Floorp control worked. Desktop visual QA verified A/B terrain picking, point inspection, profile switching, all three route types and the reference warning, CTX/THEMIS layers, one and two science targets, optimized mission order, and Turkish/English switching. Tablet layout still needs recheck after the latest responsive breakpoint change.
- DEM and CTX exact source product IDs remain unverified. No validated external risk map exists. Thermal source resolution is ~100 m.
- The live API process must be restarted after API source edits; the preview must be rebuilt after frontend edits.

## Next actions

1. Rebuild the frontend after the latest responsive/cost-label edits, then inspect the 760 px layout through native Floorp.
2. Rerun the selected Python tests, frontend tests/lint/build, and live API smoke checks.
3. Regenerate assets if raw/metadata inputs change. Current asset/API hash matched: `03b9a9b46c1659b2`.
4. Commit and push the tested data checkpoint, then the integrated-app checkpoint; update this file and `docs/implementation_progress.md` with real SHAs and any failures.
5. Prepare a PR on the feature branch without merging into `main`.

## Commands (PowerShell, repo root)

```powershell
.\.venv\Scripts\python.exe scripts\prepare_web_data.py
.\.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider prototypes\navigation\test_planner_v4.py prototypes\navigation\test_navigation_validation.py prototypes\navigation\test_route_provider.py v4\mission_planning\test_mission_planner.py tests\test_platform_api.py
.\.venv\Scripts\python.exe prototypes\navigation\route_api.py
```

In another terminal, with a standard Node installation on PATH:

```powershell
cd frontend\marseye_app
npm ci
npm run lint
npm run build
npm run preview
```

In this workspace, when Node is not on PATH, prepend `C:\Users\ghost\Desktop\coding\marseye-platform\.tools\node-v24.22.3-win-x64` to the terminal's PATH for npm commands. Visit `http://127.0.0.1:4173/`; the API runs at `http://127.0.0.1:8001/`.
