"""MarsEye V4 HTTP API; run from repository root:

pip install fastapi uvicorn numpy
python prototypes/navigation/route_api.py

Test at http://127.0.0.1:8001/docs

Endpoints:
  GET  /api/health
  GET  /api/terrain/metadata
  POST /api/route    (unchanged V4 contract: shortest_feasible + risk_aware for one profile)
  POST /api/mission  (3 primary routes: shortest / astronaut / rover, multi-stop, visit order)

Optional process environment variables:
  MARSEYE_ELEVATION, MARSEYE_META, MARSEYE_MASK,
  MARSEYE_RISK_ROVER, MARSEYE_RISK_ASTRONAUT,
  MARSEYE_THERMAL (default data/processed/thermal/thermal_temp_k.npy; set to "off" to disable)
"""
from __future__ import annotations

import os
import sys
from functools import lru_cache
from pathlib import Path
from typing import Literal

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

ROOT = Path(__file__).resolve().parents[2]
for p in (str(ROOT), str(Path(__file__).resolve().parent)):
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from .planner_v4 import RouteEngine, PROFILES
    from .route_provider import RouteProvider
    from .mission_service import MissionService, PRIMARY_ROUTES
except ImportError:
    from planner_v4 import RouteEngine, PROFILES  # type: ignore
    from route_provider import RouteProvider  # type: ignore
    from mission_service import MissionService, PRIMARY_ROUTES  # type: ignore

app = FastAPI(title="MarsEye Navigation API", version="0.5.0")
# Development only: any localhost / 127.0.0.1 port (frontend on 8000, Vite on 5173, ...).
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:8000", "http://127.0.0.1:8000",
                   "http://localhost:5173", "http://127.0.0.1:5173"],
    allow_origin_regex=r"https?://(localhost|127\.0\.0\.1)(:\d+)?",
    allow_credentials=False,
    allow_methods=["GET", "POST"],
    allow_headers=["Content-Type"],
)


class RouteRequest(BaseModel):
    start_rc: tuple[int, int]
    goal_rc: tuple[int, int]
    profile: str = Field(pattern="^(rover|astronaut)$")
    include_reference: bool = False


class Waypoint(BaseModel):
    id: str = Field(min_length=1, max_length=16)
    rc: tuple[int, int]


class MissionRequest(BaseModel):
    start_rc: tuple[int, int]
    goal_rc: tuple[int, int]
    waypoints: list[Waypoint] = Field(default_factory=list)
    order_mode: Literal["optimized", "manual"] = "optimized"


def env_path(key: str, default: str | None = None) -> Path | None:
    value = os.environ.get(key, default)
    if not value or value.lower() == "off":
        return None
    path = Path(value)
    return path if path.is_absolute() else ROOT / path


@lru_cache(maxsize=1)
def get_engine() -> RouteEngine:
    return RouteEngine.from_files(
        elevation=env_path("MARSEYE_ELEVATION", "data/processed/elevation.npy"),
        meta=env_path("MARSEYE_META", "data/processed/terrain_meta.json"),
        mask=env_path("MARSEYE_MASK", "data/processed/valid_mask.npy"),
        risk_rover=env_path("MARSEYE_RISK_ROVER"),
        risk_astronaut=env_path("MARSEYE_RISK_ASTRONAUT"),
        thermal=env_path("MARSEYE_THERMAL", "data/processed/thermal/thermal_temp_k.npy"),
    )


@lru_cache(maxsize=1)
def get_mission_service() -> MissionService:
    # Shares the same engine (DEM loaded once) and keeps a leg cache across requests.
    return MissionService(RouteProvider(get_engine()))


@app.get("/api/health")
def health():
    try:
        engine = get_engine()
        return {"status": "ok", "grid_shape": list(engine.elevation.shape),
                "thermal_in_cost": engine.cold_index is not None}
    except (OSError, ValueError, KeyError) as e:
        raise HTTPException(status_code=503, detail=str(e)) from e


@app.get("/api/terrain/metadata")
def metadata():
    try:
        e = get_engine()
        return {"grid_shape": list(e.elevation.shape), "dx_m": e.dx, "dy_m": e.dy,
                "coordinate_system": "row_col_0_based_origin_upper_left",
                "profile_names": list(PROFILES),
                "primary_routes": PRIMARY_ROUTES,
                "thermal_in_cost": e.cold_index is not None,
                "risk_sources": {p: ("external_risk_map" if e.risk_maps[p] is not None else "DEM_slope_roughness_proxy") for p in PROFILES}}
    except (OSError, ValueError, KeyError) as error:
        raise HTTPException(status_code=503, detail=str(error)) from error


@app.post("/api/route")
def route(req: RouteRequest):
    try:
        return get_engine().compute(list(req.start_rc), list(req.goal_rc), req.profile, req.include_reference)
    except (ValueError, KeyError) as e:
        raise HTTPException(status_code=422, detail=str(e)) from e
    except OSError as e:
        raise HTTPException(status_code=503, detail=str(e)) from e


@app.post("/api/mission")
def mission(req: MissionRequest):
    try:
        return get_mission_service().plan_mission(
            list(req.start_rc), list(req.goal_rc),
            [{"id": w.id, "rc": list(w.rc)} for w in req.waypoints], req.order_mode)
    except (ValueError, KeyError) as e:
        raise HTTPException(status_code=422, detail=str(e)) from e
    except OSError as e:
        raise HTTPException(status_code=503, detail=str(e)) from e


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8001)
