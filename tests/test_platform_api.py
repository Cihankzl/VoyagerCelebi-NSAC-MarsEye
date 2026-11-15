"""Real Gale DEM and HTTP contract checks for the integrated application."""
from pathlib import Path

from fastapi.testclient import TestClient
import numpy as np
from PIL import Image
import rasterio
from rasterio.crs import CRS
from rasterio.transform import Affine
from rasterio.warp import transform
import json

from prototypes.navigation.route_api import app, get_engine


ROOT = Path(__file__).resolve().parents[1]
client = TestClient(app)


def test_metadata_matches_real_dem():
    response = client.get("/api/terrain/metadata")
    assert response.status_code == 200
    data = response.json()
    assert data["grid_shape"] == [198, 200]
    assert data["dx_m"] > 5 and data["dy_m"] < 5
    assert len(data["dataset_hash"]) == 16
    assert data["profile_parameters"]["rover"]["max_cell_slope_deg"] == 20.0
    assert data["profile_parameters"]["astronaut"]["max_cell_slope_deg"] == 25.0


def test_point_uses_real_dem_and_center_coordinates():
    response = client.get("/api/point", params={"row": 99, "col": 20})
    assert response.status_code == 200
    point = response.json()
    assert point["elevation_m"] == float(get_engine().elevation[99, 20])
    assert point["valid_dem"] is True
    assert -4.71 < point["latitude_deg"] < -4.68
    assert 137.51 < point["longitude_deg_east"] < 137.54
    assert client.get("/api/point", params={"row": -1, "col": 20}).status_code == 422


def test_real_route_reference_and_science_mission():
    route = client.post("/api/route", json={"start_rc": [99, 20], "goal_rc": [99, 180],
                                            "profile": "rover", "include_reference": True})
    assert route.status_code == 200
    routes = route.json()["routes"]
    assert routes["unconstrained_reference"]["analysis_only"] is True
    assert routes["shortest_feasible"]["status"] == "found"
    assert routes["risk_aware"]["status"] == "found"
    assert routes["shortest_feasible"]["distance_m"] <= routes["risk_aware"]["distance_m"] + 0.01
    path = routes["risk_aware"]["points_rc"]
    science_rc = path[len(path) // 2]
    mission = client.post("/api/mission", json={"start_rc": [99, 20], "goal_rc": [99, 180],
                                               "science_targets": [{"id": "S1", "rc": science_rc}],
                                               "profile": "rover", "route_type": "risk_aware", "mode": "optimized"})
    assert mission.status_code == 200
    result = mission.json()
    assert result["status"] == "found"
    assert result["visit_order"] == ["A", "S1", "B"]
    assert len(result["legs"]) == 2
    assert result["points_rc"][0] == [99, 20]
    assert result["points_rc"][-1] == [99, 180]
    assert result["legs"][0]["points_rc"][-1] == result["legs"][1]["points_rc"][0]
    assert result["objective_unit"] == "model_weighted_m_equiv"
    assert result["legs"][0]["risk_source"] == "DEM_slope_roughness_proxy"


def test_invalid_and_blocked_endpoints_are_not_relocated():
    invalid = client.post("/api/route", json={"start_rc": [-1, 20], "goal_rc": [99, 180], "profile": "rover"})
    assert invalid.status_code == 422
    blocked = client.post("/api/route", json={"start_rc": [0, 0], "goal_rc": [99, 180], "profile": "rover"})
    assert blocked.status_code == 200
    assert blocked.json()["routes"]["shortest_feasible"]["status"] == "blocked_endpoint"
    bad_mission = client.post("/api/mission", json={"start_rc": [99, 20], "goal_rc": [99, 180],
                                                   "science_targets": [{"id": "S1", "rc": [99, 20]}],
                                                   "profile": "rover", "route_type": "risk_aware", "mode": "manual"})
    assert bad_mission.status_code == 422


def test_prepared_layers_keep_missing_data_and_temperature_units():
    samples = ROOT / "data/processed/science_samples.npz"
    assert samples.exists(), "Run python scripts/prepare_web_data.py first"
    arrays = np.load(samples)
    optical, thermal = arrays["optical_intensity"], arrays["thermal_K"]
    assert optical.shape == thermal.shape == (198, 200)
    assert np.isfinite(optical).any() and np.isfinite(thermal).any()
    point = client.get("/api/point", params={"row": 99, "col": 20}).json()
    if point["thermal"]:
        assert abs(point["thermal"]["brightness_temperature_K"] - point["thermal"]["brightness_temperature_C"] - 273.15) < 0.002
        assert point["thermal"]["source_resolution_m"] == 100
    for name, values in (("optical", optical), ("thermal", thermal)):
        with Image.open(ROOT / f"frontend/marseye_app/public/data/{name}.png") as image:
            assert image.size == (200, 198)
            alpha = np.asarray(image)[..., 3]
            assert np.array_equal(alpha > 0, np.isfinite(values))


def test_raster_corners_and_thermal_center_align_with_sources():
    meta = json.loads((ROOT / "data/processed/terrain_meta.json").read_text(encoding="utf-8"))
    affine = Affine(*meta["affine_transform_gcs_mars_2000"])
    dem_crs = CRS.from_wkt(meta["source_crs_wkt"])
    optical_path = ROOT / "data/raw/optical/mosaic1km.tif"
    thermal_path = ROOT / "data/processed/thermal/I01350002PBT_gale_100m.tif"
    with rasterio.open(optical_path) as optical:
        for row, col in ((0, 0), (0, 199), (197, 0), (197, 199), (99, 100)):
            lon, lat = affine @ (col + 0.5, row + 0.5)
            x, y = transform(dem_crs, optical.crs, [lon], [lat])
            source_row, source_col = optical.index(x[0], y[0])
            assert 0 <= source_row < optical.height
            assert 0 <= source_col < optical.width
    with rasterio.open(thermal_path) as thermal:
        lon, lat = affine @ (100.5, 99.5)
        x, y = transform(dem_crs, thermal.crs, [lon], [lat])
        source_row, source_col = thermal.index(x[0], y[0])
        source_value = thermal.read(1)[source_row, source_col]
    aligned_value = np.load(ROOT / "data/processed/science_samples.npz")["thermal_K"][99, 100]
    assert np.isclose(source_value, aligned_value)
