"""Build browser assets from the existing Gale DEM, CTX and THEMIS rasters.

Run from the repository root after installing requirements.txt. Source rasters are
never modified. A failed reprojection stops the build instead of showing a
misaligned layer.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import rasterio
from rasterio.crs import CRS
from rasterio.transform import Affine
from rasterio.warp import Resampling, reproject

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from prototypes.navigation.planner_v4 import RouteEngine

PROCESSED = ROOT / "data" / "processed"
OUT = ROOT / "frontend" / "marseye_app" / "public" / "data"


def dataset_hash() -> str:
    digest = hashlib.sha256()
    for path in (PROCESSED / "elevation.npy", PROCESSED / "valid_mask.npy",
                 PROCESSED / "terrain_meta.json",
                 ROOT / "data/raw/optical/mosaic1km.tif",
                 PROCESSED / "thermal/I01350002PBT_gale_100m.tif",
                 PROCESSED / "thermal/thermal_metadata.json"):
        digest.update(path.read_bytes())
    return digest.hexdigest()[:16]


def json_grid(values: np.ndarray) -> list[list[float | None]]:
    return [[float(value) if np.isfinite(value) else None for value in row] for row in values]


def align_raster(source_path: Path, dst_crs: CRS, dst_transform: Affine,
                 shape: tuple[int, int], method: Resampling) -> tuple[np.ndarray, dict]:
    with rasterio.open(source_path) as source:
        if source.crs is None:
            raise ValueError(f"No CRS in {source_path}; cannot verify spatial alignment")
        original = source.read(1, masked=True).astype("float32").filled(np.nan)
        destination = np.full(shape, np.nan, dtype="float32")
        reproject(
            source=original, destination=destination,
            src_transform=source.transform, src_crs=source.crs, src_nodata=np.nan,
            dst_transform=dst_transform, dst_crs=dst_crs, dst_nodata=np.nan,
            resampling=method, init_dest_nodata=True,
        )
        info = {"source_shape": [source.height, source.width],
                "source_crs_wkt": source.crs.to_wkt(),
                "source_resolution": list(source.res),
                "aligned_valid_cells": int(np.isfinite(destination).sum())}
    if info["aligned_valid_cells"] == 0:
        raise ValueError(f"{source_path} does not overlap the DEM after reprojection")
    return destination, info


def save_optical(values: np.ndarray, path: Path) -> None:
    valid = np.isfinite(values)
    low, high = np.percentile(values[valid], [2, 98])
    scaled = np.clip((values - low) / max(high - low, 1e-6), 0, 1)
    gray = np.where(valid, scaled * 255, 0).astype("uint8")
    rgba = np.stack([gray, gray, gray, np.where(valid, 255, 0).astype("uint8")], axis=-1)
    Image.fromarray(rgba).save(path)


def save_thermal(values: np.ndarray, path: Path) -> None:
    valid = np.isfinite(values)
    low, high = float(np.min(values[valid])), float(np.max(values[valid]))
    fraction = np.clip((values - low) / max(high - low, 1e-6), 0, 1)
    stops = np.array([[32, 73, 133], [65, 194, 183], [239, 194, 105], [208, 75, 65]])
    scaled = fraction * (len(stops) - 1)
    lower = np.minimum(np.nan_to_num(scaled, nan=0).astype(int), len(stops) - 2)
    blend = np.nan_to_num(scaled, nan=0) - lower
    rgb = stops[lower] * (1 - blend[..., None]) + stops[lower + 1] * blend[..., None]
    rgba = np.concatenate([rgb.astype("uint8"), np.where(valid, 230, 0).astype("uint8")[..., None]], axis=-1)
    Image.fromarray(rgba).save(path)


def main() -> None:
    meta = json.loads((PROCESSED / "terrain_meta.json").read_text(encoding="utf-8"))
    engine = RouteEngine.from_files(
        elevation=PROCESSED / "elevation.npy", meta=PROCESSED / "terrain_meta.json",
        mask=PROCESSED / "valid_mask.npy")
    shape = engine.elevation.shape
    if list(shape) != meta["grid_shape"]:
        raise ValueError("DEM and metadata shapes differ")
    transform = Affine(*meta["affine_transform_gcs_mars_2000"])
    crs = CRS.from_wkt(meta["source_crs_wkt"])
    bounds = meta["bounds_degrees"]
    west, north = transform * (0, 0)
    east, south = transform * (shape[1], shape[0])
    if max(abs(west - bounds["west_lon"]), abs(east - bounds["east_lon"]),
           abs(north - bounds["north_lat"]), abs(south - bounds["south_lat"])) > 1e-5:
        raise ValueError("DEM affine transform and bounds disagree")

    optical, optical_info = align_raster(
        ROOT / "data/raw/optical/mosaic1km.tif", crs, transform, shape, Resampling.bilinear)
    thermal, thermal_info = align_raster(
        PROCESSED / "thermal/I01350002PBT_gale_100m.tif", crs, transform,
        shape, Resampling.nearest)
    thermal_meta = json.loads((PROCESSED / "thermal/thermal_metadata.json").read_text(encoding="utf-8"))
    OUT.mkdir(parents=True, exist_ok=True)
    terrain = {"grid_shape": list(shape), "dx_m": engine.dx, "dy_m": engine.dy,
               "dataset_hash": dataset_hash(), "elevation_m": json_grid(engine.elevation),
               "slope_deg": json_grid(engine.slope), "roughness_m": json_grid(engine.roughness),
               "valid_mask": engine.valid.tolist()}
    (OUT / "terrain.json").write_text(json.dumps(terrain, separators=(",", ":"), allow_nan=False), encoding="utf-8")
    save_optical(optical, OUT / "optical.png")
    save_thermal(thermal, OUT / "thermal.png")
    np.savez_compressed(PROCESSED / "science_samples.npz", optical_intensity=optical, thermal_K=thermal)
    manifest = {"dataset_hash": dataset_hash(), "grid_shape": list(shape),
                "bounds_degrees": bounds, "topographic_source": meta["source"],
                "optical": {"instrument": "MRO CTX", **optical_info},
                "thermal": {"instrument": thermal_meta["instrument"],
                            "source_product_id": thermal_meta["source_product_id"],
                            "source_resolution_m": thermal_meta["source_resolution_m"],
                            "unit": "Kelvin brightness temperature",
                            "mars_local_time": thermal_meta["mars_local_time_hhmm"],
                            "stats_K": thermal_meta["stats_K"], **thermal_info}}
    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False), encoding="utf-8")
    print(f"Prepared {shape[0]}x{shape[1]} DEM; optical={optical_info['aligned_valid_cells']} cells; "
          f"thermal={thermal_info['aligned_valid_cells']} display cells; hash={dataset_hash()}")


if __name__ == "__main__":
    main()
