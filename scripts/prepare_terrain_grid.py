"""MarsEye — build the ~5 m navigation grid from the canonical DEM GeoTIFF.

Run from the repository root (before scripts/prepare_web_assets.py):

    python scripts/prepare_terrain_grid.py

Input : data/raw/topographic/gale_dem_1024.tif   (1024 x 1024, ~1 m, Mars 2000 SPHERE, metres)
Output: data/processed/elevation.npy, valid_mask.npy, terrain_meta.json  (200 x 200, ~5 m)

Resampling is an area (box) average of the ~1 m DEM, so each 5 m cell is the mean height of the
DEM pixels it covers. Row 0 = north, col 0 = west. Only numpy + Pillow are needed.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
DEM_TIF = ROOT / "data" / "raw" / "topographic" / "gale_dem_1024.tif"
OUT = ROOT / "data" / "processed"
GRID = (200, 200)          # rows, cols  (~5 m cells over the 1 km pilot box)
MARS_SPHERE_R = 3396190.0


def read_geotiff(path: Path):
    img = Image.open(path)
    arr = np.array(img, dtype=np.float64)
    tags = img.tag_v2
    scale = tuple(tags[33550])[:2]
    tie = tuple(tags[33922])
    nodata = None
    if 42113 in tags:
        try:
            nodata = float(str(tags[42113]).strip("\x00 "))
        except ValueError:
            pass
    citation = str(tags.get(34737, ""))
    return arr, scale, (tie[3], tie[4]), nodata, citation


def main() -> None:
    arr, (sx, sy), (lon0, lat0), nodata, citation = read_geotiff(DEM_TIF)
    if "Sphere" not in citation:
        raise SystemExit(f"Expected a Mars 2000 SPHERE GeoTIFF (same as optical/thermal); got: {citation!r}")
    valid_src = np.isfinite(arr)
    if nodata is not None:
        valid_src &= arr != nodata
    valid_src &= arr > -1e30
    h, w = arr.shape
    rows, cols = GRID
    # Area average, ignoring NoData pixels.
    z = np.where(valid_src, arr, 0.0).astype(np.float32)
    v = valid_src.astype(np.float32)
    zs = np.array(Image.fromarray(z, mode="F").resize((cols, rows), Image.BOX), dtype=np.float64)
    vs = np.array(Image.fromarray(v, mode="F").resize((cols, rows), Image.BOX), dtype=np.float64)
    with np.errstate(invalid="ignore", divide="ignore"):
        elev = zs / vs
    valid = vs > 0.5
    elev[~valid] = np.nan

    west, north = lon0, lat0
    east, south = lon0 + w * sx, lat0 - h * sy
    clat = 0.5 * (north + south)
    m_per_deg = MARS_SPHERE_R * math.pi / 180.0
    dx = (east - west) / cols * m_per_deg * math.cos(math.radians(clat))
    dy = (north - south) / rows * m_per_deg
    meta = {
        "region": "Gale Crater, Mars (Mount Sharp lower slopes)",
        "source": "Team export of a HiRISE-based DEM; product ID not recorded",
        "source_filename": DEM_TIF.name,
        "source_crs": "Mars 2000 Sphere (IAU), geographic, planetocentric latitude, east-positive longitude",
        "source_crs_wkt": "GEOGCS[\"Mars_2000_(Sphere)\",DATUM[\"Mars_2000_(Sphere)\",SPHEROID[\"Mars_2000_(Sphere)\",3396190,0]],PRIMEM[\"Reference_Meridian\",0],UNIT[\"degree\",0.0174532925199433]]",
        "crs": "Mars_2000_Sphere_GCS",
        "longitude_convention": "East positive",
        "grid_shape": [rows, cols],
        "dx_m": round(dx, 7),
        "dy_m": round(dy, 7),
        "nodata": None,
        "invalid_cells_stored_as": "NaN",
        "elevation_unit": "m",
        "bounds_degrees": {"west_lon": west, "east_lon": east, "south_lat": south, "north_lat": north},
        "center_lon": 0.5 * (west + east),
        "center_lat": clat,
        "affine_transform_gcs_mars_2000": [(east - west) / cols, 0.0, west, 0.0, -(north - south) / rows, north],
        "grid_axis_convention": "row increases southward, column increases eastward; pixels are cell-centred",
        "metric_scaling_note": "Local metres from the Mars 2000 sphere (R=3396190 m) at the pilot-area centre",
        "original_grid_shape": [h, w],
        "resampling": "area (box) mean of the ~1 m DEM, NoData ignored",
        "source_nodata": nodata,
    }
    OUT.mkdir(parents=True, exist_ok=True)
    np.save(OUT / "elevation.npy", elev.astype(np.float32))
    np.save(OUT / "valid_mask.npy", valid)
    (OUT / "terrain_meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(f"DEM {h}x{w} -> grid {rows}x{cols}; dx={dx:.3f} m dy={dy:.3f} m; "
          f"elevation {np.nanmin(elev):.2f} .. {np.nanmax(elev):.2f} m; invalid cells {int((~valid).sum())}")


if __name__ == "__main__":
    main()
