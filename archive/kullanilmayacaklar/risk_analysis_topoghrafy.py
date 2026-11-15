import heapq
import json
from collections import deque
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch

# ==============================================================================
# 1. YAPILANDIRMA VE EŞİK PARAMETRELERİ
# ==============================================================================
BASE_DIR = Path(__file__).resolve().parent
TIF_FILENAME = "1kmistenilenyer.tif"
FALLBACK_CELL_SIZE_M = 1.0
MARS_RADIUS_M = 3_396_190.0  # Mars ortalama yarıçapı (metre)

# 20° ve 25° Eşik Karşılaştırma Testleri
THRESHOLD_TESTS = {
    "20_deg": {"low_max": 10.0, "high_min": 20.0, "label": "20° Eşik Testi (10° / 20°)"},
    "25_deg": {"low_max": 15.0, "high_min": 25.0, "label": "25° Eşik Testi (15° / 25°)"},
}
PRIMARY_THRESHOLD = "25_deg"  # Final risk.npy ve Three.js katmanı için ana eşik

NASA_DISCLAIMER = (
    "UYARI: Bu analizde kullanılan 10°–25° eğim sınırları yalnızca rota planlama ve arazi kaçınma "
    "testleri içindir; NASA tarafından doğrulanmış astronot güvenlik eşikleri değildir."
)


# ==============================================================================
# 2. GEOTIFF (.tif) TOPOGRAFYA OKUMA VE EĞİM HESAPLAMA
# ==============================================================================
def find_tif_file(directory: Path, explicit_name: str | None) -> Path:
    if explicit_name:
        target = directory / explicit_name
        if not target.exists():
            raise FileNotFoundError(f"Belirtilen dosya bulunamadı: {target}")
        return target

    tif_files = sorted(list(directory.glob("*.tif")) + list(directory.glob("*.tiff")))
    if not tif_files:
        raise FileNotFoundError(f"'{directory}' klasöründe .tif veya .tiff uzantılı dosya bulunamadı.")
    return tif_files[0]


def load_dem_from_tif(tif_path: Path, fallback_res_m: float) -> tuple[np.ndarray, float, float]:
    try:
        import rasterio

        with rasterio.open(tif_path) as src:
            dem = src.read(1).astype(np.float64)
            if src.nodata is not None:
                dem[np.isclose(dem, src.nodata)] = np.nan
            dem[(dem < -15000.0) | (dem > 30000.0)] = np.nan

            res_x = abs(float(src.transform.a))
            res_y = abs(float(src.transform.e))
            is_geographic = (src.crs is not None and src.crs.is_geographic) or (res_x < 0.05)

            if is_geographic and res_x > 0 and res_y > 0:
                bounds = src.bounds
                center_lat_rad = np.radians(0.5 * (bounds.bottom + bounds.top))
                dy_m = np.radians(res_y) * MARS_RADIUS_M
                dx_m = np.radians(res_x) * MARS_RADIUS_M * np.cos(center_lat_rad)
                print(f"[GeoTIFF] Coğrafi koordinat (derece) tespit edildi -> Piksel: {dx_m:.2f}m x {dy_m:.2f}m")
            elif res_x > 0 and res_y > 0:
                dx_m, dy_m = res_x, res_y
                print(f"[GeoTIFF] Metrik projeksiyon tespit edildi -> Piksel: {dx_m:.2f}m x {dy_m:.2f}m")
            else:
                dx_m = dy_m = fallback_res_m
                print(f"[GeoTIFF] Varsayılan çözünürlük kullanılıyor: {fallback_res_m}m")

            return dem, dx_m, dy_m

    except ImportError:
        import tifffile

        dem = tifffile.imread(tif_path).astype(np.float64)
        if dem.ndim == 3:
            dem = dem[:, :, 0]
        dem[(dem < -15000.0) | (dem > 30000.0)] = np.nan
        print(f"[Tifffile] Varsayılan çözünürlük kullanılıyor: {fallback_res_m}m")
        return dem, fallback_res_m, fallback_res_m


def compute_slope_degrees(dem: np.ndarray, dx_m: float, dy_m: float) -> np.ndarray:
    dz_dy, dz_dx = np.gradient(dem, dy_m, dx_m)
    return np.degrees(np.arctan(np.hypot(dz_dx, dz_dy)))


def classify_risk(slope_deg: np.ndarray, low_max: float, high_min: float) -> np.ndarray:
    risk = np.zeros(slope_deg.shape, dtype=np.int32)
    risk[(slope_deg >= low_max) & (slope_deg < high_min)] = 1
    risk[(slope_deg >= high_min) | np.isnan(slope_deg)] = 2
    return risk


# ==============================================================================
# 3. A* ROTA PLANLAMA VE RİSK KAÇINMA DOĞRULAMASI
# ==============================================================================
def find_nearest_safe_cell(risk_grid: np.ndarray, target_rc: tuple[int, int]) -> tuple[int, int]:
    rows, cols = risk_grid.shape
    r0 = int(np.clip(target_rc[0], 0, rows - 1))
    c0 = int(np.clip(target_rc[1], 0, cols - 1))
    if risk_grid[r0, c0] == 0:
        return (r0, c0)

    visited = {(r0, c0)}
    q = deque([(r0, c0)])
    while q:
        r, c = q.popleft()
        if risk_grid[r, c] == 0:
            return (r, c)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    q.append((nr, nc))
    raise RuntimeError("Haritada güvenli (risk=0) başlangıç/hedef hücresi bulunamadı.")


def run_astar(
    risk_grid: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    risk_aware: bool = True,
    medium_penalty: float = 3.5,
) -> list[tuple[int, int]]:
    rows, cols = risk_grid.shape
    open_set = [(0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score = {start: 0.0}

    moves = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142),
    ]

    while open_set:
        _, curr = heapq.heappop(open_set)
        if curr == goal:
            path = [curr]
            while curr in came_from:
                curr = came_from[curr]
                path.append(curr)
            return path[::-1]

        cr, cc = curr
        for dr, dc, step_cost in moves:
            nr, nc = cr + dr, cc + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue

            r_val = risk_grid[nr, nc]
            if risk_aware and r_val == 2:
                continue

            weight = (medium_penalty if r_val == 1 else 1.0) if risk_aware else 1.0
            tentative_g = g_score[curr] + step_cost * weight
            nxt = (nr, nc)

            if tentative_g < g_score.get(nxt, float("inf")):
                came_from[nxt] = curr
                g_score[nxt] = tentative_g
                h = np.hypot(goal[0] - nr, goal[1] - nc)
                heapq.heappush(open_set, (tentative_g + h, nxt))

    return []


def evaluate_route(
    path: list[tuple[int, int]],
    slope_deg: np.ndarray,
    risk_grid: np.ndarray,
    dx_m: float,
    dy_m: float,
) -> dict:
    if not path:
        return {"valid": False, "reason": "Bu eşik altında güvenli rota bulunamadı."}

    slopes = np.array([slope_deg[r, c] for r, c in path], dtype=float)
    risks = np.array([risk_grid[r, c] for r, c in path], dtype=int)

    pts = np.array(path, dtype=float)
    step_dists_m = np.hypot(np.diff(pts[:, 0]) * dy_m, np.diff(pts[:, 1]) * dx_m)
    total_dist_m = float(np.sum(step_dists_m))

    high_hits = int(np.sum(risks == 2))
    med_hits = int(np.sum(risks == 1))
    low_hits = int(np.sum(risks == 0))

    return {
        "valid": True,
        "avoided_high_slope_cells": high_hits == 0,
        "cell_count": len(path),
        "distance_m": round(total_dist_m, 2),
        "max_slope_deg": round(float(np.nanmax(slopes)), 2),
        "mean_slope_deg": round(float(np.nanmean(slopes)), 2),
        "low_risk_cells": low_hits,
        "medium_risk_cells": med_hits,
        "high_risk_violations": high_hits,
    }


# ==============================================================================
# 4. THREE.JS KATMANLARI VE PREVIEW GÖRSELİ
# ==============================================================================
def export_threejs_outputs(
    risk_grid: np.ndarray,
    threshold_summary: dict,
    route_summary: dict,
    dx_m: float,
    dy_m: float,
) -> None:
    h, w = risk_grid.shape
    rgba = np.zeros((h, w, 4), dtype=np.uint8)
    rgba[risk_grid == 0] = [46, 204, 113, 40]
    rgba[risk_grid == 1] = [241, 196, 15, 150]
    rgba[risk_grid == 2] = [231, 76, 60, 210]

    plt.imsave(BASE_DIR / "risk_overlay.png", rgba)

    payload = {
        "layer_name": "Gale_Crater_Real_DEM_Risk_Layer",
        "grid_shape": [int(h), int(w)],
        "pixel_size_m": {"dx": round(dx_m, 3), "dy": round(dy_m, 3)},
        "overlay_png": "risk_overlay.png",
        "legend": {
            "0": {"label": "Dusuk Risk", "hex": "#2ecc71", "traversable": True},
            "1": {"label": "Orta Risk", "hex": "#f1c40f", "traversable": True},
            "2": {"label": "Yuksek Risk (Engel)", "hex": "#e74c3c", "traversable": False},
        },
        "threshold_comparison_20_vs_25": threshold_summary,
        "astar_route_evaluation": route_summary,
        "disclaimer": NASA_DISCLAIMER,
    }

    with open(BASE_DIR / "risk_layer.json", "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)


def save_preview_figure(
    dem: np.ndarray,
    slope_deg: np.ndarray,
    risk_20: np.ndarray,
    risk_25: np.ndarray,
    route_naive: list[tuple[int, int]],
    route_20: list[tuple[int, int]],
    route_25: list[tuple[int, int]],
    eval_naive: dict,
    eval_20: dict,
    eval_25: dict,
) -> None:
    fig, axes = plt.subplots(1, 4, figsize=(22, 5.8), dpi=180)
    cmap_risk = ListedColormap(["#2ecc71", "#f1c40f", "#e74c3c"])

    im0 = axes[0].imshow(dem, cmap="terrain")
    axes[0].set_title("1. Gale Krateri DEM (m)", fontweight="bold")
    plt.colorbar(im0, ax=axes[0], fraction=0.046, pad=0.04)

    vmax_slope = min(40.0, float(np.nanpercentile(slope_deg, 99)))
    im1 = axes[1].imshow(slope_deg, cmap="inferno", vmin=0, vmax=vmax_slope)
    axes[1].set_title("2. Yüzey Eğimi (Derece)", fontweight="bold")
    plt.colorbar(im1, ax=axes[1], fraction=0.046, pad=0.04)

    axes[2].imshow(risk_20, cmap=cmap_risk, vmin=0, vmax=2)
    axes[2].set_title(
        f"3. 20° Eşik Testi\nMaks: {eval_20.get('max_slope_deg', 'N/A')}° | "
        f"Mesafe: {eval_20.get('distance_m', 'N/A')} m",
        fontsize=9.5,
        fontweight="bold",
    )
    if route_20:
        axes[2].plot([c for _, c in route_20], [r for r, _ in route_20], color="#00ffff", lw=2.0, label="Risk-Aware (20°)")
        axes[2].legend(loc="lower right", fontsize=8)

    axes[3].imshow(risk_25, cmap=cmap_risk, vmin=0, vmax=2)
    axes[3].set_title(
        f"4. 25° Eşik & A* Karşılaştırması\n"
        f"Güvenli İhlal: {eval_25.get('high_risk_violations', 'N/A')} | "
        f"Risksiz İhlal: {eval_naive.get('high_risk_violations', 'N/A')}",
        fontsize=9.5,
        fontweight="bold",
    )
    if route_naive:
        axes[3].plot([c for _, c in route_naive], [r for r, _ in route_naive], color="white", ls="--", lw=1.6, label="Risk-Blind A*")
    if route_25:
        axes[3].plot([c for _, c in route_25], [r for r, _ in route_25], color="#00ffff", lw=2.1, label="Risk-Aware (25°)")
    axes[3].legend(loc="lower right", fontsize=8)

    patches = [
        Patch(facecolor="#2ecc71", label="0: Düşük Risk"),
        Patch(facecolor="#f1c40f", label="1: Orta Risk (Cezalı)"),
        Patch(facecolor="#e74c3c", label="2: Yüksek Risk (Engel)"),
    ]
    fig.legend(handles=patches, loc="upper center", ncol=3, frameon=True, bbox_to_anchor=(0.5, 0.99))
    fig.text(0.5, 0.01, NASA_DISCLAIMER, ha="center", fontsize=8.8, style="italic", color="#922b21")

    plt.tight_layout(rect=[0, 0.04, 1, 0.93])
    plt.savefig(BASE_DIR / "risk_preview.png", dpi=200)
    plt.close(fig)


# ==============================================================================
# 5. ANA ÇALIŞTIRMA AKIŞI
# ==============================================================================
def main() -> None:
    tif_path = find_tif_file(BASE_DIR, TIF_FILENAME)
    print(f"Okunan Topografya Dosyası: {tif_path.name}")

    dem, dx_m, dy_m = load_dem_from_tif(tif_path, FALLBACK_CELL_SIZE_M)
    slope_deg = compute_slope_degrees(dem, dx_m, dy_m)

    np.save(BASE_DIR / "dem.npy", dem)
    np.save(BASE_DIR / "slope.npy", slope_deg)

    h, w = slope_deg.shape
    total_cells = slope_deg.size

    risk_maps = {}
    threshold_summary = {}
    for key, cfg in THRESHOLD_TESTS.items():
        r_grid = classify_risk(slope_deg, cfg["low_max"], cfg["high_min"])
        risk_maps[key] = r_grid
        threshold_summary[key] = {
            "label": cfg["label"],
            "thresholds_deg": {"low_max": cfg["low_max"], "high_min": cfg["high_min"]},
            "low_risk_pct": round(float(np.sum(r_grid == 0) / total_cells * 100), 2),
            "medium_risk_pct": round(float(np.sum(r_grid == 1) / total_cells * 100), 2),
            "high_risk_pct": round(float(np.sum(r_grid == 2) / total_cells * 100), 2),
        }

    start_rc = find_nearest_safe_cell(risk_maps["20_deg"], (int(h * 0.15), int(w * 0.15)))
    goal_rc = find_nearest_safe_cell(risk_maps["20_deg"], (int(h * 0.85), int(w * 0.85)))

    route_naive = run_astar(risk_maps["25_deg"], start_rc, goal_rc, risk_aware=False)
    route_20 = run_astar(risk_maps["20_deg"], start_rc, goal_rc, risk_aware=True)
    route_25 = run_astar(risk_maps["25_deg"], start_rc, goal_rc, risk_aware=True)

    eval_naive = evaluate_route(route_naive, slope_deg, risk_maps["25_deg"], dx_m, dy_m)
    eval_20 = evaluate_route(route_20, slope_deg, risk_maps["20_deg"], dx_m, dy_m)
    eval_25 = evaluate_route(route_25, slope_deg, risk_maps["25_deg"], dx_m, dy_m)

    route_summary = {
        "start_rc": list(start_rc),
        "goal_rc": list(goal_rc),
        "risk_blind_astar": eval_naive,
        "risk_aware_20_deg": eval_20,
        "risk_aware_25_deg": eval_25,
    }

    final_risk = risk_maps[PRIMARY_THRESHOLD]
    np.save(BASE_DIR / "risk.npy", final_risk)
    export_threejs_outputs(final_risk, threshold_summary, route_summary, dx_m, dy_m)
    save_preview_figure(
        dem, slope_deg, risk_maps["20_deg"], risk_maps["25_deg"],
        route_naive, route_20, route_25, eval_naive, eval_20, eval_25
    )

    print("\n================ GALE KRATERİ GERÇEK DEM RİSK RAPORU ================")
    print(f"Grid Boyutu   : {h} x {w} piksel | Piksel Ölçeği: {dx_m:.2f}m x {dy_m:.2f}m")
    print(f"Yükseklik     : {np.nanmin(dem):.1f} m .. {np.nanmax(dem):.1f} m | Maks. Eğim: {np.nanmax(slope_deg):.2f}°")
    print(f"Test Rotası   : {start_rc} -> {goal_rc}\n")

    for key in ("20_deg", "25_deg"):
        ts = threshold_summary[key]
        print(f"[{ts['label']}] Düşük: %{ts['low_risk_pct']} | Orta: %{ts['medium_risk_pct']} | Yüksek (Engel): %{ts['high_risk_pct']}")

    print("\n--- A* Rota Güvenilirliği Sonuçları ---")
    print(f"Risk-Blind A* (Referans) : Mesafe={eval_naive.get('distance_m')}m | Maks Eğim={eval_naive.get('max_slope_deg')}° | İhlal={eval_naive.get('high_risk_violations')} hücre")
    print(f"Risk-Aware A* (20° Eşik) : Mesafe={eval_20.get('distance_m')}m | Maks Eğim={eval_20.get('max_slope_deg')}° | İhlal={eval_20.get('high_risk_violations')} hücre")
    print(f"Risk-Aware A* (25° Eşik) : Mesafe={eval_25.get('distance_m')}m | Maks Eğim={eval_25.get('max_slope_deg')}° | İhlal={eval_25.get('high_risk_violations')} hücre")
    print(f"\n{NASA_DISCLAIMER}")
    print("=====================================================================\n")


if __name__ == "__main__":
    main()