# MarsEye continuation handoff

Updated: 2026-11-15 (Europe/Istanbul). Worktree: `C:\Users\ghost\Desktop\coding\marseye-platform`. Branch: `feature/marseye-integrated-platform`. Never merge automatically into `main` or modify the V3 demo/raw rasters.

## Verified checkpoint

- Commit `2be7e98` (`chore: establish isolated MarsEye toolchain`) was pushed to `origin/feature/marseye-integrated-platform`.
- Scientific web data checkpoint `af4ba95` (`feat: prepare aligned Gale science layers`) was pushed to the same feature branch.
- Integrated application checkpoint `422f40a` (`feat: integrate live Gale exploration and mission planning`) was pushed to the same feature branch.
- Handoff documentation checkpoint `e68b7a6` (`docs: record verified MarsEye integration checkpoint`) was pushed to the same branch. Draft PR #9 is open: https://github.com/Cihankzl/voyager-nasa-apps-chalange-project/pull/9. It targets `main`; do not merge automatically.
- Official portable CPython 3.12.10 and Node.js 24.22.3 were unpacked under ignored `.tools/`. `.venv/` and frontend `node_modules/` are also ignored.
- Integrated-code verification: 35 selected Python tests passed, 2 frontend tests passed, ESLint passed, and TypeScript/Vite production build passed. The Python run reported one third-party Starlette deprecation warning.
- `frontend/marseye_v3_demo/` and original scientific rasters were not changed.

## Completed integration

- FastAPI point inspection and real V4 multi-stop mission endpoint in `prototypes/navigation/route_api.py`; small adapter/cache fixes in `v4/mission_planning/mission_planner.py`.
- New bilingual React/Three.js application in `frontend/marseye_app/src/` with the real DEM, scientific layers, all three route types, and up to five science targets.
- API/geospatial tests in `tests/`, frontend coordinate tests, README and API documentation are included in `422f40a`. Check `git status` before further changes.

## Current issues and limits

- The frontend Vite development server's dependency optimization failed in this sandbox with an ancestor-directory access error. `npm run build` and `npm run preview` work; use the production preview for local verification here.
- Native Floorp control verified desktop A/B terrain picking, point inspection, profile switching and recalculation, all three route types and the reference warning, CTX/THEMIS layers, one/two science targets, optimized mission order, Turkish/English switching, and the stacked 760 px layout after the responsive breakpoint change.
- DEM and CTX exact source product IDs remain unverified. No validated external risk map exists. Thermal source resolution is ~100 m.
- The production preview must be rebuilt after frontend edits. A verified production build is present in ignored `dist/` but should not be committed.
- GitHub's connector could not create the PR (HTTP 403: resource not accessible by integration); the signed-in browser successfully created draft PR #9. No PR checks were listed at creation time.

## Next actions

1. Review draft PR #9 and run the documented local checks before considering it ready for review. Do not merge automatically.
2. Regenerate web assets if raw/metadata inputs change. Current asset/API hash matched: `03b9a9b46c1659b2`.
3. Resolve exact DEM/CTX source product identities and evaluate further scientific risk layers only when independently validated; neither is a prerequisite for the current experimental application.

## Commands (PowerShell, repo root)

```powershell
.\.venv\Scripts\python.exe scripts\prepare_web_data.py
.\.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider prototypes\navigation\test_planner_v4.py prototypes\navigation\test_navigation_validation.py prototypes\navigation\test_route_provider.py prototypes\navigation\test_benchmark_navigation.py v4\mission_planning\test_mission_planner.py tests\test_platform_api.py
.\.venv\Scripts\python.exe prototypes\navigation\route_api.py
```

In another terminal, with a standard Node installation on PATH:

```powershell
cd frontend\marseye_app
npm ci
npm test
npm run lint
npm run build
npm run preview
```

In this workspace, when Node is not on PATH, prepend `C:\Users\ghost\Desktop\coding\marseye-platform\.tools\node-v24.22.3-win-x64` to the terminal's PATH for npm commands. Visit `http://127.0.0.1:4173/`; the API runs at `http://127.0.0.1:8001/`.
