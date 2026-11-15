"""MarsEye V4 HTTP API; run from repository root:

pip install fastapi uvicorn
python prototypes/navigation/route_api.py

Test at http://127.0.0.1:8001/docs

Optional process environment variables:
  MARSEYE_ELEVATION, MARSEYE_META, MARSEYE_MASK,
  MARSEYE_RISK_ROVER, MARSEYE_RISK_ASTRONAUT
"""
from __future__ import annotations

import os
from pathlib import Path
from functools import lru_cache

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from planner_v4 import RouteEngine, PROFILES

app = FastAPI(title="MarsEye Navigation API", version="0.4.0")
# Development only. Replace these origins with the deployed frontend URL for production.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:8000", "http://127.0.0.1:8000",
                   "http://localhost:5173", "http://127.0.0.1:5173"],
    allow_credentials=False,
    allow_methods=["GET", "POST"],
    allow_headers=["Content-Type"],
)


class RouteRequest(BaseModel):
    start_rc: tuple[int, int]
    goal_rc: tuple[int, int]
    profile: str = Field(pattern="^(rover|astronaut)$")
    include_reference: bool = False


def env_path(key: str, default: str | None = None) -> Path | None:
    value = os.environ.get(key, default)
    return Path(value) if value else None


@lru_cache(maxsize=1)
def get_engine() -> RouteEngine:
    return RouteEngine.from_files(
        elevation=env_path("MARSEYE_ELEVATION", "data/processed/elevation.npy"),
        meta=env_path("MARSEYE_META", "data/processed/terrain_meta.json"),
        mask=env_path("MARSEYE_MASK", "data/processed/valid_mask.npy"),
        risk_rover=env_path("MARSEYE_RISK_ROVER"),
        risk_astronaut=env_path("MARSEYE_RISK_ASTRONAUT"),
    )


@app.get("/api/health")
def health():
    try:
        engine = get_engine()
        return {"status": "ok", "grid_shape": list(engine.elevation.shape)}
    except (OSError, ValueError, KeyError) as e:
        raise HTTPException(status_code=503, detail=str(e)) from e


@app.get("/api/terrain/metadata")
def metadata():
    try:
        e = get_engine()
        return {"grid_shape": list(e.elevation.shape), "dx_m": e.dx, "dy_m": e.dy,
                "coordinate_system": "row_col_0_based_origin_upper_left",
                "profile_names": list(PROFILES),
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


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8001)
