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
import hashlib
import json
from pathlib import Path
from functools import lru_cache
import sys
from dataclasses import asdict

import numpy as np

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field, StrictInt

try:
    from .planner_v4 import RouteEngine, PROFILES
    from .route_provider import RouteProvider
except ImportError:
    from planner_v4 import RouteEngine, PROFILES
    from route_provider import RouteProvider

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from v4.mission_planning.mission_planner import MultiStopMissionPlanner, NavigationV4Adapter

app = FastAPI(title="MarsEye Navigation API", version="0.4.0")
# Development only. Replace these origins with the deployed frontend URL for production.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:8000", "http://127.0.0.1:8000",
                   "http://localhost:5173", "http://127.0.0.1:5173",
                   "http://localhost:4173", "http://127.0.0.1:4173"],
    allow_credentials=False,
    allow_methods=["GET", "POST"],
    allow_headers=["Content-Type"],
)


class RouteRequest(BaseModel):
    start_rc: tuple[StrictInt, StrictInt]
    goal_rc: tuple[StrictInt, StrictInt]
    profile: str = Field(pattern="^(rover|astronaut)$")
    include_reference: bool = False


class ScienceTarget(BaseModel):
    id: str
    rc: tuple[StrictInt, StrictInt]


class MissionRequest(BaseModel):
    start_rc: tuple[StrictInt, StrictInt]
    goal_rc: tuple[StrictInt, StrictInt]
    science_targets: list[ScienceTarget] = Field(min_length=1, max_length=5)
    profile: str = Field(pattern="^(rover|astronaut)$")
    route_type: str = Field(pattern="^(shortest_feasible|risk_aware)$")
    mode: str = Field(pattern="^(manual|optimized)$")
    manual_order: list[str] | None = None


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


@lru_cache(maxsize=1)
def get_provider() -> RouteProvider:
    return RouteProvider(get_engine())


@lru_cache(maxsize=1)
def terrain_source_metadata() -> dict:
    with env_path("MARSEYE_META", "data/processed/terrain_meta.json").open(encoding="utf-8") as source:
        return json.load(source)


@lru_cache(maxsize=1)
def dataset_hash() -> str:
    digest = hashlib.sha256()
    for path in (ROOT / "data/processed/elevation.npy", ROOT / "data/processed/valid_mask.npy",
                 ROOT / "data/processed/terrain_meta.json",
                 ROOT / "data/raw/optical/mosaic1km.tif",
                 ROOT / "data/processed/thermal/I01350002PBT_gale_100m.tif",
                 ROOT / "data/processed/thermal/thermal_metadata.json"):
        digest.update(path.read_bytes())
    return digest.hexdigest()[:16]


@lru_cache(maxsize=1)
def science_samples():
    path = ROOT / "data" / "processed" / "science_samples.npz"
    return np.load(path, allow_pickle=False) if path.is_file() else None


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
        source = terrain_source_metadata()
        return {"grid_shape": list(e.elevation.shape), "dx_m": e.dx, "dy_m": e.dy,
                "coordinate_system": "row_col_0_based_origin_upper_left",
                "dataset_hash": dataset_hash(),
                "bounds_degrees": source["bounds_degrees"],
                "affine_transform_gcs_mars_2000": source["affine_transform_gcs_mars_2000"],
                "source": source["source"],
                "profile_names": list(PROFILES),
                "profile_parameters": {name: asdict(config) for name, config in PROFILES.items()},
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


@app.get("/api/point")
def point(row: int, col: int):
    engine = get_engine()
    rows, cols = engine.elevation.shape
    if not (0 <= row < rows and 0 <= col < cols):
        raise HTTPException(status_code=422, detail="Point outside DEM grid")
    affine = terrain_source_metadata()["affine_transform_gcs_mars_2000"]
    lon = affine[2] + (col + 0.5) * affine[0]
    lat = affine[5] + (row + 0.5) * affine[4]
    valid = bool(engine.valid[row, col])
    samples = science_samples()
    optical = None
    thermal = None
    if samples is not None:
        optical_value = float(samples["optical_intensity"][row, col])
        if np.isfinite(optical_value):
            optical = {"intensity": optical_value, "instrument": "MRO CTX", "unit": "display intensity"}
        thermal_value = float(samples["thermal_K"][row, col])
        if np.isfinite(thermal_value):
            thermal = {"brightness_temperature_K": thermal_value,
                       "brightness_temperature_C": round(thermal_value - 273.15, 3),
                       "instrument": "Mars Odyssey THEMIS Band 9 PBT",
                       "source_resolution_m": 100, "mars_local_time": "03:15"}
    return {"row": row, "col": col, "latitude_deg": lat, "longitude_deg_east": lon,
            "valid_dem": valid,
            "elevation_m": float(engine.elevation[row, col]) if valid else None,
            "slope_deg": float(engine.slope[row, col]) if valid else None,
            "roughness_m": float(engine.roughness[row, col]) if valid else None,
            "optical": optical, "thermal": thermal,
            "source": {"topographic": "Gale DEM; source product ID unverified",
                       "optical": "MRO CTX", "thermal": "I01350002PBT"}}


@app.post("/api/mission")
def mission(req: MissionRequest):
    try:
        payload = req.model_dump()
        planner = MultiStopMissionPlanner(
            NavigationV4Adapter(get_provider().get_route), get_engine().elevation.shape)
        result = planner.plan_mission(payload)
        if result["status"] == "invalid_input":
            raise HTTPException(status_code=422, detail=result["reason"])
        result.update({"schema_version": "1.0", "start_rc": list(req.start_rc),
                       "goal_rc": list(req.goal_rc),
                       "science_targets": [target.model_dump() for target in req.science_targets],
                       "terrain_id": "gale_5m_v1", "dataset_hash": dataset_hash(),
                       "risk_source": "external_risk_map" if get_engine().risk_maps[req.profile] is not None else "DEM_slope_roughness_proxy",
                       "experimental": True,
                       "objective_cost": result["total_distance_m"] if req.route_type == "shortest_feasible" else result["total_weighted_cost_m_equiv"],
                       "objective_unit": "m" if req.route_type == "shortest_feasible" else "model_weighted_m_equiv"})
        if result["status"] == "found":
            legs = result["legs"]
            result["metrics"] = {
                "elevation_gain_m": round(sum(leg.get("elevation_gain_m") or 0 for leg in legs), 3),
                "elevation_loss_m": round(sum(leg.get("elevation_loss_m") or 0 for leg in legs), 3),
                "max_slope_deg": max(leg.get("max_slope_deg") or 0 for leg in legs),
                "max_edge_grade_deg": max(leg.get("max_edge_grade_deg") or 0 for leg in legs),
                "runtime_ms": round(sum(leg.get("runtime_ms") or 0 for leg in legs), 2),
            }
        return result
    except (ValueError, KeyError) as error:
        raise HTTPException(status_code=422, detail=str(error)) from error
    except OSError as error:
        raise HTTPException(status_code=503, detail=str(error)) from error


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8001)
