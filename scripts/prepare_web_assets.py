"""MarsEye — prepare every static asset the web frontend needs.

Run from the repository root:

    python scripts/prepare_web_assets.py

Inputs (canonical files only):
  data/raw/optical/mosaic1km.tif                      MRO CTX mosaic, 200 x 201, ~5 m/px
  data/processed/thermal/I01350002PBT_gale_100m.tif   THEMIS IR Band 9 PBT, 11 x 11, 100 m/px
  data/processed/elevation.npy, valid_mask.npy, terrain_meta.json   198 x 200 DEM grid

Outputs:
  data/processed/optical_mosaic.png            contrast-enhanced optical texture (north up)
  data/processed/thermal/thermal_temp_k.npy    thermal Kelvin per DEM cell (used by routing)
  data/processed/thermal/thermal_grid.json     same grid + native 10x10 values for the browser
  data/processed/thermal/thermal_100m.png      colour-mapped texture, native 100 m blocks
  data/processed/thermal/thermal_smooth.png    colour-mapped texture, bilinear smooth
  data/processed/terrain.json                  elevation / slope / roughness for the 3D mesh
  data/processed/routes.json                   3 primary routes for the default A -> B
  data/processed/mission_route.json            3 primary routes for the preset multi-stop mission

Only needs numpy + Pillow (rasterio / tifffile are used if installed, never required).
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

DATA = ROOT / "data" / "processed"
THERMAL_DIR = DATA / "thermal"
OPTICAL_TIF = ROOT / "data" / "raw" / "optical" / "mosaic1km.tif"
THERMAL_TIF = THERMAL_DIR / "I01350002PBT_gale_100m.tif"

MARS_RADIUS_M = 3396190.0          # sphere used by the THEMIS PDS Sinusoidal product
THEMIS_CENTRAL_MERIDIAN = 137.483  # from thermal_metadata.json source_crs_wkt
NODATA = -9999.0

# Deep blue/purple (cold) -> magenta -> orange -> yellow (warm)
THERMAL_STOPS = [
    (0.00, (20, 12, 70)),
    (0.22, (62, 30, 140)),
    (0.45, (150, 40, 150)),
    (0.65, (225, 80, 80)),
    (0.82, (250, 150, 40)),
    (1.00, (255, 235, 120)),
]


def read_tif(path: Path) -> tuple[np.ndarray, dict]:
    """Return (array, geotags) using whatever reader is available."""
    tags: dict = {}
    try:
        import rasterio  # type: ignore
        with rasterio.open(path) as src:
            arr = src.read(1)
            t = src.transform
            tags = {"scale": (t.a, -t.e), "tie": (t.c, t.f), "nodata": src.nodata}
            return arr, tags
    except Exception:
        pass
    img = Image.open(path)
    arr = np.array(img)
    raw = getattr(img, "tag_v2", {})
    if 33550 in raw:
        tags["scale"] = tuple(raw[33550])[:2]
    if 33922 in raw:
        tp = tuple(raw[33922])
        tags["tie"] = (tp[3], tp[4])
    if 42113 in raw:
        try:
            tags["nodata"] = float(str(raw[42113]).strip("\x00 "))
        except ValueError:
            pass
    return arr, tags


def colormap(values01: np.ndarray) -> np.ndarray:
    v = np.clip(values01, 0.0, 1.0)
    out = np.zeros(v.shape + (3,), dtype=np.float64)
    for (p0, c0), (p1, c1) in zip(THERMAL_STOPS, THERMAL_STOPS[1:]):
        m = (v >= p0) & (v <= p1)
        f = ((v - p0) / (p1 - p0))[..., None]
        out[m] = (np.array(c0) * (1 - f) + np.array(c1) * f)[m]
    return out.clip(0, 255).astype(np.uint8)


def load_terrain():
    meta = json.loads((DATA / "terrain_meta.json").read_text(encoding="utf-8"))
    elevation = np.load(DATA / "elevation.npy", allow_pickle=False)
    mask = np.load(DATA / "valid_mask.npy", allow_pickle=False)
    return meta, elevation, mask


# --------------------------------------------------------------------------- optical
def prepare_optical(meta: dict) -> dict:
    arr, tags = read_tif(OPTICAL_TIF)
    arr = arr.astype(np.float64)
    if arr.ndim == 3:
        arr = arr[..., 0]
    # GeoTIFF row 0 is north when the y pixel scale is positive in the tiepoint convention.
    tie_lat = tags.get("tie", (None, meta["bounds_degrees"]["north_lat"]))[1]
    flipped = tie_lat is not None and abs(tie_lat - meta["bounds_degrees"]["south_lat"]) < 1e-6
    if flipped:
        arr = arr[::-1]
    valid = np.isfinite(arr) & (arr > 0)
    lo, hi = np.percentile(arr[valid], [1.0, 99.5])
    stretched = np.clip((arr - lo) / max(hi - lo, 1e-6), 0, 1) ** 0.9
    img8 = (stretched * 255).round().astype(np.uint8)
    img = Image.fromarray(img8, mode="L")
    # 4x Lanczos upsample: same footprint, smoother texture filtering in WebGL.
    img = img.resize((img.width * 4, img.height * 4), Image.LANCZOS).convert("RGB")
    out = DATA / "optical_mosaic.png"
    img.save(out, optimize=True)
    return {"file": str(out.relative_to(ROOT)), "source_shape": list(arr.shape),
            "stretch_percentiles": [1.0, 99.5], "flipped_to_north_up": bool(flipped)}


# --------------------------------------------------------------------------- thermal
THERMAL_IMG = ROOT / "data" / "raw" / "thermal" / "I01350002PBT.IMG"
MARS2000_INV_FLATTENING = 169.8944472236118


# OFF by default: the team's original thermal clip is used and all layers share the same
# numeric bounding box. Turn ON (python scripts/prepare_web_assets.py --dem-lat-correction)
# only if it is CONFIRMED that the DEM latitudes are planetographic (ellipsoid) while
# THEMIS/CTX are planetocentric; then thermal is re-sampled from the full PDS3 strip.
DEM_LAT_CORRECTION = "--dem-lat-correction" in sys.argv


def dem_lat_to_planetocentric(lat_deg, meta: dict):
    """The DEM GeoTIFF is GCS_Mars_2000 on the IAU ELLIPSOID (planetographic latitude);
    THEMIS (and CTX) are on the Mars SPHERE (planetocentric). Same numbers != same place:
    at -4.69 deg the difference is ~3.25 km north-south. Convert before sampling."""
    if not DEM_LAT_CORRECTION:
        return lat_deg
    if "Mars_2000_IAU_IAG" not in meta.get("source_crs_wkt", "") or "Sphere" in meta.get("source_crs_wkt", ""):
        return lat_deg
    k = 1.0 / (1.0 - 1.0 / MARS2000_INV_FLATTENING) ** 2
    return np.degrees(np.arctan(np.tan(np.radians(lat_deg)) / k))


def read_themis_window(meta: dict):
    """Read the full PDS3 THEMIS PBT strip and cut a window around the DEM's true location.

    Pixel centres (0-based, verified against the team's original 11x11 clip):
      col = x/100 + 415.5,  row = -y/100 - 951.5   (x, y in metres, Sinusoidal, lon0=137.483)
    """
    label = THERMAL_IMG.read_bytes()[:20000].decode("latin-1")
    rec = int(label.split("RECORD_BYTES")[1].split("=")[1].split()[0])
    lines = int(label.split("\n    LINES")[1].split("=")[1].split()[0]) if "\n    LINES" in label else 3612
    samples = int(label.split("LINE_SAMPLES")[1].split("=")[1].split()[0])
    img = np.fromfile(THERMAL_IMG, dtype="<f4", offset=rec, count=lines * samples).reshape(lines, samples).astype(np.float64)
    b = meta["bounds_degrees"]
    lats = dem_lat_to_planetocentric(np.array([b["north_lat"], b["south_lat"]]), meta)
    lons = np.array([b["west_lon"], b["east_lon"]])
    LON, LAT = np.meshgrid(lons, lats)
    x = MARS_RADIUS_M * np.radians(LON - THEMIS_CENTRAL_MERIDIAN) * np.cos(np.radians(LAT))
    y = MARS_RADIUS_M * np.radians(LAT)
    cols_f, rows_f = x / 100 + 415.5, -y / 100 - 951.5
    i0, i1 = int(np.floor(rows_f.min())) - 1, int(np.ceil(rows_f.max())) + 1
    j0, j1 = int(np.floor(cols_f.min())) - 1, int(np.ceil(cols_f.max())) + 1
    win = img[i0:i1 + 1, j0:j1 + 1]
    if np.any(win <= 0):
        raise ValueError("THEMIS NULL (0) pixels inside the pilot window")
    tags = {"tie": ((j0 - 416.0) * 100.0, -(i0 + 951.0) * 100.0), "scale": (100.0, 100.0), "nodata": 0.0,
            "window_rows_cols": [i0, i1, j0, j1]}
    return win, tags


def prepare_thermal(meta: dict, shape: tuple[int, int]) -> dict:
    if DEM_LAT_CORRECTION and THERMAL_IMG.is_file():
        raw, tags = read_themis_window(meta)
        source = f"{THERMAL_IMG.relative_to(ROOT)} (full PDS3 strip, window {tags['window_rows_cols']})"
    else:  # fallback: team's 11x11 clip (NOTE: clipped with the DEM's numeric bbox, see docstring above)
        raw, tags = read_tif(THERMAL_TIF)
        source = str(THERMAL_TIF.relative_to(ROOT))
    raw = raw.astype(np.float64)
    nodata = tags.get("nodata", NODATA)
    nodata = NODATA if nodata is None else float(nodata)
    valid = np.isfinite(raw) & (raw != nodata) & (raw > 0)  # never treat -9999 / 0 NULL as Kelvin
    vr, vc = np.where(valid)
    r0, r1, c0, c1 = vr.min(), vr.max(), vc.min(), vc.max()
    native = raw[r0:r1 + 1, c0:c1 + 1]
    if not np.all(valid[r0:r1 + 1, c0:c1 + 1]):
        raise ValueError("Thermal valid block is not rectangular; check the clip")

    x_tie, y_tie = tags.get("tie", (2100.0, -277600.0))
    px_m, py_m = tags.get("scale", (100.0, 100.0))[:2]

    # Geographic centre of every DEM cell (cell-centred affine from terrain_meta.json).
    a, _, lon0, _, e, lat0 = meta["affine_transform_gcs_mars_2000"]
    rows, cols = shape
    lon = lon0 + (np.arange(cols) + 0.5) * a
    lat = lat0 + (np.arange(rows) + 0.5) * e
    LON, LAT = np.meshgrid(lon, dem_lat_to_planetocentric(lat, meta))
    # Forward Sinusoidal projection (sphere), as in the THEMIS PDS product.
    x = MARS_RADIUS_M * np.radians(LON - THEMIS_CENTRAL_MERIDIAN) * np.cos(np.radians(LAT))
    y = MARS_RADIUS_M * np.radians(LAT)
    # Fractional pixel coordinate measured between pixel CENTRES of the raw 11x11 raster.
    fc = (x - x_tie) / px_m - 0.5
    fr = (y_tie - y) / py_m - 0.5
    # The pilot box edge falls ~half a THEMIS pixel into the -9999 border row/column.
    # Clamp to the valid 10x10 block so those cells use the nearest REAL temperature.
    fr_c = np.clip(fr, r0, r1)
    fc_c = np.clip(fc, c0, c1)

    nearest = raw[np.rint(fr_c).astype(int), np.rint(fc_c).astype(int)]
    i0 = np.floor(fr_c).astype(int).clip(r0, r1)
    j0 = np.floor(fc_c).astype(int).clip(c0, c1)
    i1 = (i0 + 1).clip(r0, r1)
    j1 = (j0 + 1).clip(c0, c1)
    ti, tj = fr_c - i0, fc_c - j0
    smooth = (raw[i0, j0] * (1 - ti) * (1 - tj) + raw[i0, j1] * (1 - ti) * tj +
              raw[i1, j0] * ti * (1 - tj) + raw[i1, j1] * ti * tj)
    assert np.all(nearest > 0) and np.all(smooth > 0), "-9999 leaked into thermal grid"

    np.save(THERMAL_DIR / "thermal_temp_k.npy", smooth.astype(np.float32))

    # Stats/colour scale over the pixels that actually fall inside the DEM area.
    tmin, tmax, tmean = float(nearest.min()), float(nearest.max()), float(nearest.mean())
    norm = lambda t: (t - tmin) / (tmax - tmin)
    # Textures at 4x DEM resolution so 100 m blocks have crisp edges.
    up = 4
    # Texture pixel centres span the FULL bounding box (cell units; -0.5 .. rows-0.5),
    # matching the frontend UVs u=(c+0.5)/cols, v=1-(r+0.5)/rows and the optical mosaic footprint.
    rr = (np.arange(rows * up) + 0.5) / up - 0.5
    cc = (np.arange(cols * up) + 0.5) / up - 0.5
    def upsample(grid, nearest_mode):
        if nearest_mode:
            return grid[np.rint(rr).astype(int).clip(0, rows - 1)][:, np.rint(cc).astype(int).clip(0, cols - 1)]
        rc_, cc_ = rr.clip(0, rows - 1), cc.clip(0, cols - 1)
        r_i = np.floor(rc_).astype(int).clip(0, rows - 2); r_t = (rc_ - r_i)[:, None]
        c_i = np.floor(cc_).astype(int).clip(0, cols - 2); c_t = (cc_ - c_i)[None, :]
        g = grid
        return (g[r_i][:, c_i] * (1 - r_t) * (1 - c_t) + g[r_i][:, c_i + 1] * (1 - r_t) * c_t +
                g[r_i + 1][:, c_i] * r_t * (1 - c_t) + g[r_i + 1][:, c_i + 1] * r_t * c_t)
    # Native look: sample the raw raster at sub-cell resolution with nearest neighbour.
    lon_u = lon0 + (cc + 0.5) * a
    lat_u = lat0 + (rr + 0.5) * e
    LONu, LATu = np.meshgrid(lon_u, dem_lat_to_planetocentric(lat_u, meta))
    xu = MARS_RADIUS_M * np.radians(LONu - THEMIS_CENTRAL_MERIDIAN) * np.cos(np.radians(LATu))
    yu = MARS_RADIUS_M * np.radians(LATu)
    nr = np.rint(np.clip((y_tie - yu) / py_m - 0.5, r0, r1)).astype(int)
    nc = np.rint(np.clip((xu - x_tie) / px_m - 0.5, c0, c1)).astype(int)
    Image.fromarray(colormap(norm(raw[nr, nc]))).save(THERMAL_DIR / "thermal_100m.png", optimize=True)
    Image.fromarray(colormap(norm(upsample(smooth, False)))).save(THERMAL_DIR / "thermal_smooth.png", optimize=True)

    grid = {
        "schema_version": "1.0",
        "source_product_id": "I01350002PBT",
        "instrument": "2001 Mars Odyssey THEMIS IR Band 9 (12.57 um)",
        "local_time": "03:15 (night)",
        "quantity": "brightness_temperature",
        "unit": "K",
        "native_resolution_m": 100,
        "grid_shape": [rows, cols],
        "coordinate_system": "row_col_0_based_origin_upper_left",
        "nodata_removed": nodata,
        "native_valid_block": {"rows": [int(r0), int(r1)], "cols": [int(c0), int(c1)],
                               "shape": list(native.shape), "temp_k": np.round(native, 3).tolist()},
        "source": source,
        "alignment": (("DEM latitude converted planetographic -> planetocentric, " if DEM_LAT_CORRECTION else
                       "DEM latitude used as-is (same numeric bbox as the team's clip), ") +
                      "cell centres projected to THEMIS Sinusoidal (sphere R=3396190 m, lon0=137.483); "
                      "edge cells on the NoData border use the nearest valid THEMIS pixel."),
        "interpolation_for_routing": "bilinear between valid 100 m pixel centres",
        "stats_k": {"min": round(tmin, 3), "max": round(tmax, 3), "mean": round(tmean, 3)},
        "stats_c": {"min": round(tmin - 273.15, 2), "max": round(tmax - 273.15, 2),
                    "mean": round(tmean - 273.15, 2)},
        "colormap_stops": [{"t": t, "rgb": list(c)} for t, c in THERMAL_STOPS],
        "textures": {"native": "thermal_100m.png", "smooth": "thermal_smooth.png"},
        "temp_k": np.round(smooth, 2).tolist(),
        "caveat": "Night-time brightness temperature, not contact or air temperature. ~100 m native resolution.",
    }
    (THERMAL_DIR / "thermal_grid.json").write_text(json.dumps(grid, separators=(",", ":")), encoding="utf-8")
    return {"min_k": tmin, "max_k": tmax, "frac_row_range": [float(fr.min()), float(fr.max())],
            "frac_col_range": [float(fc.min()), float(fc.max())]}


# --------------------------------------------------------------------------- terrain + routes
def _clean(arr: np.ndarray, digits: int):
    out = np.round(arr.astype(np.float64), digits).astype(object)
    out[~np.isfinite(arr.astype(np.float64))] = None
    return out.tolist()


def prepare_terrain_and_routes(meta: dict) -> dict:
    from prototypes.navigation.mission_service import MissionService, PRIMARY_ROUTES, PRESET_MISSION
    from prototypes.navigation.planner_v4 import PROFILES
    from dataclasses import asdict

    service = MissionService.from_repo()
    eng = service.engine
    rows, cols = eng.elevation.shape
    elev = np.where(eng.valid, eng.elevation, np.nan)
    terrain = {
        "schema_version": "1.1",
        "terrain_id": "gale_5m_v1",
        "grid_shape": [rows, cols],
        "dx_m": eng.dx,
        "dy_m": eng.dy,
        "coordinate_system": "row_col_0_based_origin_upper_left",
        "bounds_degrees": meta.get("bounds_degrees"),
        "elevation_range_m": [round(float(np.nanmin(elev)), 2), round(float(np.nanmax(elev)), 2)],
        "elevation_m": _clean(elev, 2),
        "slope_deg": _clean(eng.slope, 2),
        "roughness_m": _clean(eng.roughness, 3),
        "profiles": {k: asdict(v) for k, v in PROFILES.items()},
        "primary_routes": PRIMARY_ROUTES,
        "source_metadata": {k: meta[k] for k in ("region", "source", "source_filename", "source_crs") if k in meta},
    }
    (DATA / "terrain.json").write_text(json.dumps(terrain, separators=(",", ":"), allow_nan=False), encoding="utf-8")

    default = service.plan_mission([99, 20], [99, 180], [], "optimized")
    (DATA / "routes.json").write_text(json.dumps(default, indent=1, allow_nan=False), encoding="utf-8")
    mission = service.plan_mission(PRESET_MISSION["start_rc"], PRESET_MISSION["goal_rc"],
                                   PRESET_MISSION["waypoints"], "optimized")
    (DATA / "mission_route.json").write_text(json.dumps(mission, indent=1, allow_nan=False), encoding="utf-8")
    return {"routes": {k: (v["status"], v.get("distance_m")) for k, v in default["routes"].items()},
            "mission": {k: (v["status"], " -> ".join(v.get("visit_order", [])), v.get("distance_m"))
                        for k, v in mission["routes"].items()}}


def main() -> None:
    meta, elevation, _ = load_terrain()
    print("Optical :", prepare_optical(meta))
    print("Thermal :", prepare_thermal(meta, elevation.shape))
    print("Routes  :", json.dumps(prepare_terrain_and_routes(meta), ensure_ascii=False))
    for f in ["optical_mosaic.png", "terrain.json", "routes.json", "mission_route.json",
              "thermal/thermal_grid.json", "thermal/thermal_temp_k.npy",
              "thermal/thermal_100m.png", "thermal/thermal_smooth.png"]:
        p = DATA / f
        print(f"  {p.relative_to(ROOT)}  {p.stat().st_size/1024:.0f} KB")


if __name__ == "__main__":
    main()
