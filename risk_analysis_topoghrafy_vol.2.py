import heapq
import io
import json
from collections import deque
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

# ==============================================================================
# 1. YAPILANDIRMA VE PARAMETRELER
# ==============================================================================
BASE_DIR = Path(__file__).resolve().parent

TIF_FILENAME = "1kmistenilenyer.tif"            # Birincil doğrudan okunacak GeoTIFF topografya dosyası
DEM_PATH = BASE_DIR / "dem.npy"                 # .tif yoksa kullanılacak yedek DEM matrisi (örn. 198x200)
SLOPE_PATH = BASE_DIR / "slope.npy"             # Senkronize edilecek / okunacak eğim matrisi (derece)
ROUGHNESS_PATH = BASE_DIR / "roughness.npy"     # Senkronize edilecek / okunacak TRI pürüzlülük matrisi (m)
CTX_HAZARD_PATH = BASE_DIR / "ctx_hazard.npy"   # Opsiyonel: CTX kaya/kum tehlike katmanı [0.0 - 1.0]

MARS_RADIUS_M = 3_396_190.0  # Mars ortalama yarıçapı (metre)
AREA_SPAN_M = 1000.0         # GeoTIFF başlığı olmayan .npy dosyaları için varsayılan kenar (m)

# Rover ve Astronaut Risk Profilleri (Ağırlıklar + Deneysel Geçilmez Eşikler)
AGENT_PROFILES = {
    "rover": {
        "description": "Tekerlekli keşif aracı (yüzey pürüzlülüğü ve şasi açıklığına duyarlı)",
        "weights": {
            "slope": 0.45,
            "roughness": 0.55,
            "ctx_rock_sand": 0.20,  # Yalnızca ctx_hazard.npy mevcutsa ağırlığa katılır
        },
        "experimental_impassable": {
            "max_slope_deg": 25.0,
            "max_roughness_ratio": 0.45,  # max_roughness_m = ratio * pixel_size_m
            "max_ctx_hazard": 0.85,
        },
        "astar_risk_multiplier": 4.0,
    },
    "astronaut": {
        "description": "EVA uzay giysili astronot (dik eğim ve kayma riskine duyarlı)",
        "weights": {
            "slope": 0.65,
            "roughness": 0.35,
            "ctx_rock_sand": 0.25,  # Yalnızca ctx_hazard.npy mevcutsa ağırlığa katılır
        },
        "experimental_impassable": {
            "max_slope_deg": 18.0,
            "max_roughness_ratio": 0.40,
            "max_ctx_hazard": 0.80,
        },
        "astar_risk_multiplier": 6.0,
    },
}

# Deneysel 20° / 25° Eşik Karşılaştırma Testleri (Rover profili üzerinde hassasiyet analizi)
THRESHOLD_COMPARISON_DEGS = (20.0, 25.0)

NASA_DISCLAIMER = (
    "NOT: Kullanılan eğim ve pürüzlülük eşikleri algoritmik arazi analizi ve rota testi içindir; "
    "NASA tarafından doğrulanmış astronot güvenlik sınırları değildir."
)


# ==============================================================================
# 2. GEOTIFF (.tif) TOPOGRAFYA OKUMA VE EĞİM / PÜRÜZLÜLÜK HESABI
# ==============================================================================
def compute_tri_roughness(dem: np.ndarray) -> np.ndarray:
    """3x3 komşuluk üzerinden Riley Terrain Ruggedness Index (TRI - metre) hesaplar."""
    padded = np.pad(dem, pad_width=1, mode="edge")
    center = padded[1:-1, 1:-1]
    sq_diff = np.zeros_like(dem, dtype=np.float64)

    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            nbr = padded[1 + dr : padded.shape[0] - 1 + dr, 1 + dc : padded.shape[1] - 1 + dc]
            sq_diff += (nbr - center) ** 2

    return np.sqrt(sq_diff / 8.0)


def read_geotiff_dem(tif_path: Path) -> tuple[np.ndarray, float, float]:
    """GeoTIFF topografya dosyasını okur ve Mars yarıçapına göre piksel boyutunu (dx_m, dy_m) hesaplar."""
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
                h, w = dem.shape
                dx_m = dy_m = AREA_SPAN_M / max(h, w)
                print(f"[GeoTIFF] Varsayılan ölçek kullanılıyor -> Piksel: {dx_m:.2f}m x {dy_m:.2f}m")

            return dem, dx_m, dy_m

    except ImportError:
        import tifffile

        dem = tifffile.imread(tif_path).astype(np.float64)
        if dem.ndim == 3:
            dem = dem[:, :, 0]
        dem[(dem < -15000.0) | (dem > 30000.0)] = np.nan
        h, w = dem.shape
        dx_m = dy_m = AREA_SPAN_M / max(h, w)
        print(f"[Tifffile] Piksel ölçeği: {dx_m:.2f}m x {dy_m:.2f}m")
        return dem, dx_m, dy_m


def load_topography_from_tif() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray | None, float, float, str]:
    """
    Öncelikle doğrudan .tif topografya dosyasını okur; .tif yoksa dem.npy dosyasına geçer.
    Tüm katmanların aynı (H, W) satır/sütun ızgarasında hizalı olduğunu doğrular.
    """
    tif_path = BASE_DIR / TIF_FILENAME
    if not tif_path.exists():
        found_tifs = sorted(list(BASE_DIR.glob("*.tif")) + list(BASE_DIR.glob("*.tiff")))
        if found_tifs:
            tif_path = found_tifs[0]

    if tif_path.exists():
        dem, dx_m, dy_m = read_geotiff_dem(tif_path)
        dz_dy, dz_dx = np.gradient(dem, dy_m, dx_m)
        slope_deg = np.degrees(np.arctan(np.hypot(dz_dx, dz_dy)))
        roughness_m = compute_tri_roughness(dem)

        # Diğer ekip modülleriyle senkronizasyon için temel katmanları kaydet
        np.save(DEM_PATH, dem)
        np.save(SLOPE_PATH, slope_deg)
        np.save(ROUGHNESS_PATH, roughness_m)
        source_label = f"{tif_path.name} (Doğrudan GeoTIFF)"

    elif DEM_PATH.exists():
        dem = np.load(DEM_PATH, allow_pickle=False).astype(np.float64)
        h, w = dem.shape
        dx_m = dy_m = AREA_SPAN_M / max(h, w)

        if SLOPE_PATH.exists():
            slope_deg = np.load(SLOPE_PATH, allow_pickle=False).astype(np.float64)
            if slope_deg.shape != dem.shape:
                raise ValueError(f"Boyut uyuşmazlığı: slope.npy {slope_deg.shape} != DEM {dem.shape}")
        else:
            dz_dy, dz_dx = np.gradient(dem, dy_m, dx_m)
            slope_deg = np.degrees(np.arctan(np.hypot(dz_dx, dz_dy)))

        if ROUGHNESS_PATH.exists():
            roughness_m = np.load(ROUGHNESS_PATH, allow_pickle=False).astype(np.float64)
            if roughness_m.shape != dem.shape:
                raise ValueError(f"Boyut uyuşmazlığı: roughness.npy {roughness_m.shape} != DEM {dem.shape}")
        else:
            roughness_m = compute_tri_roughness(dem)

        source_label = "dem.npy (NumPy Yedek Kaynak)"
    else:
        raise FileNotFoundError(f"Klasörde '{TIF_FILENAME}' veya 'dem.npy' bulunamadı.")

    ctx_hazard = None
    if CTX_HAZARD_PATH.exists():
        ctx_raw = np.load(CTX_HAZARD_PATH, allow_pickle=False).astype(np.float64)
        if ctx_raw.shape != dem.shape:
            raise ValueError(f"Boyut uyuşmazlığı: ctx_hazard.npy {ctx_raw.shape} != DEM {dem.shape}")
        ctx_hazard = np.clip(ctx_raw, 0.0, 1.0)

    return dem, slope_deg, roughness_m, ctx_hazard, dx_m, dy_m, source_label


# ==============================================================================
# 3. [0.0 - 1.0] RİSK MODELİ VE AYRI UNKNOWN MASKESİ
# ==============================================================================
def build_surface_risk(
    profile: dict,
    slope_deg: np.ndarray,
    roughness_m: np.ndarray,
    unknown_mask: np.ndarray,
    pixel_size_m: float,
    ctx_hazard: np.ndarray | None = None,
    override_max_slope_deg: float | None = None,
) -> tuple[np.ndarray, dict]:
    """
    Eğim, pürüzlülük ve varsa CTX kaya/kum tehlikesini birleştirerek
    [0.0, 1.0] aralığında float32 yüzey risk haritası üretir.
    """
    imp = profile["experimental_impassable"]
    max_slope = float(override_max_slope_deg) if override_max_slope_deg is not None else float(imp["max_slope_deg"])
    max_rough_m = float(imp["max_roughness_ratio"]) * pixel_size_m
    max_ctx = float(imp["max_ctx_hazard"])

    norm_slope = np.clip(np.nan_to_num(slope_deg, nan=max_slope) / max_slope, 0.0, 1.0)
    norm_rough = np.clip(np.nan_to_num(roughness_m, nan=max_rough_m) / max_rough_m, 0.0, 1.0)

    raw_w = profile["weights"]
    if ctx_hazard is not None:
        w_ctx = float(raw_w["ctx_rock_sand"])
        rem = 1.0 - w_ctx
        base_sum = float(raw_w["slope"] + raw_w["roughness"])
        w_slope = rem * (float(raw_w["slope"]) / base_sum)
        w_rough = rem * (float(raw_w["roughness"]) / base_sum)
        norm_ctx = np.nan_to_num(ctx_hazard, nan=1.0)
        risk = (w_slope * norm_slope) + (w_rough * norm_rough) + (w_ctx * norm_ctx)
        active_weights = {"slope": round(w_slope, 4), "roughness": round(w_rough, 4), "ctx_rock_sand": round(w_ctx, 4)}
    else:
        base_sum = float(raw_w["slope"] + raw_w["roughness"])
        w_slope = float(raw_w["slope"]) / base_sum
        w_rough = float(raw_w["roughness"]) / base_sum
        risk = (w_slope * norm_slope) + (w_rough * norm_rough)
        active_weights = {"slope": round(w_slope, 4), "roughness": round(w_rough, 4), "ctx_rock_sand": 0.0}

    # Deneysel geçilmez alanları 1.0 olarak işaretle
    impassable = (slope_deg >= max_slope) | (roughness_m >= max_rough_m)
    if ctx_hazard is not None:
        impassable = impassable | (ctx_hazard >= max_ctx)

    risk[impassable] = 1.0
    # Eksik/NoData pikseller ayrı unknown_mask.npy içinde tutulur; güvenlik için 1.0 atanır
    risk[unknown_mask == 1] = 1.0

    risk_f32 = np.clip(risk, 0.0, 1.0).astype(np.float32)
    valid_cells = unknown_mask == 0

    meta = {
        "description": profile["description"],
        "active_weights": active_weights,
        "experimental_impassable_thresholds": {
            "max_slope_deg": max_slope,
            "max_roughness_m": round(max_rough_m, 3),
            "max_ctx_hazard": max_ctx if ctx_hazard is not None else None,
        },
        "stats": {
            "mean_risk": round(float(np.mean(risk_f32[valid_cells])), 4) if np.any(valid_cells) else 1.0,
            "low_risk_lt_03_pct": round(float(np.mean(risk_f32 < 0.30) * 100.0), 2),
            "impassable_1_0_pct": round(float(np.mean(risk_f32 >= 0.999) * 100.0), 2),
        },
    }
    return risk_f32, meta


# ==============================================================================
# 4. ÇİFTE CEZALANDIRMASIZ (NO DOUBLE-COUNTING) A* ROTA DOĞRULAMA
# ==============================================================================
def find_nearest_safe_cell(
    surface_risk: np.ndarray,
    unknown_mask: np.ndarray,
    target_rc: tuple[int, int],
    max_risk: float = 0.35,
) -> tuple[int, int]:
    rows, cols = surface_risk.shape
    r0 = int(np.clip(target_rc[0], 0, rows - 1))
    c0 = int(np.clip(target_rc[1], 0, cols - 1))

    if unknown_mask[r0, c0] == 0 and surface_risk[r0, c0] <= max_risk:
        return (r0, c0)

    visited = {(r0, c0)}
    q = deque([(r0, c0)])
    while q:
        r, c = q.popleft()
        if unknown_mask[r, c] == 0 and surface_risk[r, c] <= max_risk:
            return (r, c)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    q.append((nr, nc))
    raise RuntimeError("Haritada uygun güvenli başlangıç/hedef hücresi bulunamadı.")


def run_astar_no_double_counting(
    surface_risk: np.ndarray,
    unknown_mask: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    risk_multiplier: float,
    use_risk: bool = True,
) -> list[tuple[int, int]]:
    """
    A* yalnızca geometrik adım mesafesini ve tekil surface_risk[r, c] değerini kullanır;
    eğim veya pürüzlülüğü tekrar hesaplayarak aynı riski iki kez saymaz.
    """
    rows, cols = surface_risk.shape
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
        for dr, dc, step_dist in moves:
            nr, nc = cr + dr, cc + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue

            if use_risk:
                if unknown_mask[nr, nc] == 1 or surface_risk[nr, nc] >= 0.999:
                    continue
                step_cost = step_dist * (1.0 + risk_multiplier * float(surface_risk[nr, nc]))
            else:
                step_cost = step_dist

            tentative_g = g_score[curr] + step_cost
            nxt = (nr, nc)
            if tentative_g < g_score.get(nxt, float("inf")):
                came_from[nxt] = curr
                g_score[nxt] = tentative_g
                h = np.hypot(goal[0] - nr, goal[1] - nc)
                heapq.heappush(open_set, (tentative_g + h, nxt))

    return []


def verify_route(
    name: str,
    path: list[tuple[int, int]],
    surface_risk: np.ndarray,
    unknown_mask: np.ndarray,
    slope_deg: np.ndarray,
    dx_m: float,
    dy_m: float,
) -> dict:
    if not path:
        return {"scenario": name, "valid": False, "safe": False}

    pts = np.array(path, dtype=float)
    dist_m = float(np.sum(np.hypot(np.diff(pts[:, 0]) * dy_m, np.diff(pts[:, 1]) * dx_m)))

    risks = np.array([surface_risk[r, c] for r, c in path], dtype=float)
    unknowns = np.array([unknown_mask[r, c] for r, c in path], dtype=int)
    slopes = np.array([slope_deg[r, c] for r, c in path], dtype=float)

    impassable_hits = int(np.sum(risks >= 0.999))
    unknown_hits = int(np.sum(unknowns == 1))

    return {
        "scenario": name,
        "valid": True,
        "avoided_impassable_and_unknown": (impassable_hits == 0 and unknown_hits == 0),
        "distance_m": round(dist_m, 2),
        "mean_risk_0_1": round(float(np.mean(risks)), 4),
        "max_risk_0_1": round(float(np.max(risks)), 4),
        "max_slope_deg": round(float(np.nanmax(slopes)), 2),
        "impassable_violations": impassable_hits,
        "unknown_violations": unknown_hits,
    }


# ==============================================================================
# 5. THREE.JS KATMANI VE ÖNİZLEME GÖRSELİ (ONEDRIVE GÜVENLİ KAYIT)
# ==============================================================================
def save_threejs_overlay_and_preview(
    dem: np.ndarray,
    risk_rover: np.ndarray,
    risk_astronaut: np.ndarray,
    unknown_mask: np.ndarray,
    path_blind: list[tuple[int, int]],
    path_rover_20: list[tuple[int, int]],
    path_rover_25: list[tuple[int, int]],
    path_astro: list[tuple[int, int]],
    eval_rover_20: dict,
    eval_rover_25: dict,
    eval_astro: dict,
) -> None:
    # 1) Three.js için DEM ile birebir (H, W) boyutunda RGBA risk kaplaması
    rgba = plt.get_cmap("RdYlGn_r")(risk_rover)
    rgba_u8 = (rgba * 255).astype(np.uint8)
    rgba_u8[..., 3] = np.where(risk_rover < 0.25, 45, np.where(risk_rover >= 0.999, 215, 155))
    rgba_u8[unknown_mask == 1] = [52, 73, 94, 220]

    overlay_buf = io.BytesIO()
    plt.imsave(overlay_buf, rgba_u8, format="png")
    try:
        (BASE_DIR / "risk_overlay.png").write_bytes(overlay_buf.getvalue())
    except OSError:
        (BASE_DIR / "risk_overlay_v2.png").write_bytes(overlay_buf.getvalue())

    # 2) 4 Panelli Rapor Görseli (risk_preview.png)
    fig, axes = plt.subplots(1, 4, figsize=(22, 5.8), dpi=180)

    im0 = axes[0].imshow(dem, cmap="terrain")
    axes[0].set_title("1. Gale Krateri DEM (m)", fontweight="bold")
    plt.colorbar(im0, ax=axes[0], fraction=0.046, pad=0.04)

    im1 = axes[1].imshow(risk_rover, cmap="RdYlGn_r", vmin=0.0, vmax=1.0)
    axes[1].set_title(
        f"2. Rover [0–1] & 20°/25° Testi\n"
        f"25°: {eval_rover_25.get('distance_m', 'N/A')}m (Maks {eval_rover_25.get('max_slope_deg', 'N/A')}°) | "
        f"20°: {eval_rover_20.get('distance_m', 'N/A')}m",
        fontsize=9.2,
        fontweight="bold",
    )
    if path_blind:
        axes[1].plot([c for _, c in path_blind], [r for r, _ in path_blind], color="white", ls="--", lw=1.4, label="Risksiz A*")
    if path_rover_25:
        axes[1].plot([c for _, c in path_rover_25], [r for r, _ in path_rover_25], color="#00ffff", lw=2.1, label="Rover (25°)")
    if path_rover_20:
        axes[1].plot([c for _, c in path_rover_20], [r for r, _ in path_rover_20], color="#2980b9", lw=2.0, label="Rover (20°)")
    axes[1].legend(loc="lower left", fontsize=7.5)
    plt.colorbar(im1, ax=axes[1], fraction=0.046, pad=0.04)

    im2 = axes[2].imshow(risk_astronaut, cmap="RdYlGn_r", vmin=0.0, vmax=1.0)
    axes[2].set_title(
        f"3. Astronaut [0–1] Risk Haritası\n"
        f"Mesafe: {eval_astro.get('distance_m', 'N/A')}m | Maks Eğim: {eval_astro.get('max_slope_deg', 'N/A')}°",
        fontsize=9.2,
        fontweight="bold",
    )
    if path_astro:
        axes[2].plot([c for _, c in path_astro], [r for r, _ in path_astro], color="#8e44ad", lw=2.2, label="Astronaut (18°)")
        axes[2].legend(loc="lower left", fontsize=8)
    plt.colorbar(im2, ax=axes[2], fraction=0.046, pad=0.04)

    cmap_unk = ListedColormap(["#2ecc71", "#34495e"])
    axes[3].imshow(unknown_mask, cmap=cmap_unk, vmin=0, vmax=1)
    unk_pct = float(np.mean(unknown_mask == 1) * 100.0)
    axes[3].set_title(f"4. Ayrı Unknown Maskesi\n(Geçerli: %{100.0 - unk_pct:.2f} | Veri Yok: %{unk_pct:.2f})", fontsize=9.5, fontweight="bold")

    fig.text(0.5, 0.01, NASA_DISCLAIMER, ha="center", fontsize=8.8, style="italic", color="#922b21")
    plt.tight_layout(rect=[0, 0.04, 1, 0.98])

    prev_buf = io.BytesIO()
    plt.savefig(prev_buf, format="png", dpi=200)
    plt.close(fig)

    try:
        (BASE_DIR / "risk_preview.png").write_bytes(prev_buf.getvalue())
    except OSError:
        (BASE_DIR / "risk_preview_v2.png").write_bytes(prev_buf.getvalue())


# ==============================================================================
# 6. ANA ÇALIŞTIRMA AKIŞI
# ==============================================================================
def main() -> None:
    dem, slope_deg, roughness_m, ctx_hazard, dx_m, dy_m, source_label = load_topography_from_tif()
    h, w = dem.shape
    pixel_size_m = round(0.5 * (dx_m + dy_m), 3)

    # 1. Ayrı Unknown Maskesi (1 = veri yok / NaN, 0 = geçerli veri)
    unknown_mask = (np.isnan(dem) | np.isnan(slope_deg) | np.isnan(roughness_m)).astype(np.uint8)
    if ctx_hazard is not None:
        unknown_mask = np.maximum(unknown_mask, np.isnan(ctx_hazard).astype(np.uint8))

    # 2. Rover ve Astronaut [0.0, 1.0] Risk Haritaları
    risk_rover, meta_rover = build_surface_risk(
        AGENT_PROFILES["rover"], slope_deg, roughness_m, unknown_mask, pixel_size_m, ctx_hazard
    )
    risk_astronaut, meta_astronaut = build_surface_risk(
        AGENT_PROFILES["astronaut"], slope_deg, roughness_m, unknown_mask, pixel_size_m, ctx_hazard
    )

    # 20° Eşik Hassasiyet Testi (Rover modeli üzerinde 20° vs 25° karşılaştırması)
    risk_rover_20, meta_rover_20 = build_surface_risk(
        AGENT_PROFILES["rover"], slope_deg, roughness_m, unknown_mask, pixel_size_m, ctx_hazard, override_max_slope_deg=20.0
    )

    # Boyut Tutarlılığı Doğrulaması (DEM ile birebir aynı satır/sütun)
    assert risk_rover.shape == (h, w)
    assert risk_astronaut.shape == (h, w)
    assert unknown_mask.shape == (h, w)

    # Zorunlu Teslim Dosyalarını Kaydet
    np.save(BASE_DIR / "surface_risk_rover.npy", risk_rover)
    np.save(BASE_DIR / "surface_risk_astronaut.npy", risk_astronaut)
    np.save(BASE_DIR / "unknown_mask.npy", unknown_mask)

    # 3. Çifte Saymasız A* Rota Doğrulaması
    start_rc = find_nearest_safe_cell(risk_astronaut, unknown_mask, (int(h * 0.18), int(w * 0.18)))
    goal_rc = find_nearest_safe_cell(risk_astronaut, unknown_mask, (int(h * 0.85), int(w * 0.83)))

    mult_rover = AGENT_PROFILES["rover"]["astar_risk_multiplier"]
    mult_astro = AGENT_PROFILES["astronaut"]["astar_risk_multiplier"]

    path_blind = run_astar_no_double_counting(risk_rover, unknown_mask, start_rc, goal_rc, 0.0, use_risk=False)
    path_rover_20 = run_astar_no_double_counting(risk_rover_20, unknown_mask, start_rc, goal_rc, mult_rover, use_risk=True)
    path_rover_25 = run_astar_no_double_counting(risk_rover, unknown_mask, start_rc, goal_rc, mult_rover, use_risk=True)
    path_astro = run_astar_no_double_counting(risk_astronaut, unknown_mask, start_rc, goal_rc, mult_astro, use_risk=True)

    eval_blind = verify_route("Risk-Blind A* (Referans)", path_blind, risk_rover, unknown_mask, slope_deg, dx_m, dy_m)
    eval_rover_20 = verify_route("Rover A* (20° Eşik)", path_rover_20, risk_rover_20, unknown_mask, slope_deg, dx_m, dy_m)
    eval_rover_25 = verify_route("Rover A* (25° Eşik)", path_rover_25, risk_rover, unknown_mask, slope_deg, dx_m, dy_m)
    eval_astro = verify_route("Astronaut A* (18° Eşik)", path_astro, risk_astronaut, unknown_mask, slope_deg, dx_m, dy_m)

    # 4. Ortak Teknik Sözleşme Dosyası: risk_config.json
    risk_config = {
        "data_source": source_label,
        "grid_contract": {
            "shape": [int(h), int(w)],
            "pixel_size_m": {"dx": round(dx_m, 3), "dy": round(dy_m, 3)},
            "alignment_rule": "Row/col (r, c) maps 1:1 to DEM pixel coordinates.",
            "value_range": [0.0, 1.0],
            "dtype": "float32",
            "impassable_value": 1.0,
            "unknown_mask_file": "unknown_mask.npy",
            "unknown_mask_encoding": {"0": "valid_data", "1": "unknown_or_nodata"},
        },
        "double_counting_prevention": {
            "included_in_surface_risk": ["slope_deg", "roughness_tri_m"] + (["ctx_rock_sand"] if ctx_hazard is not None else []),
            "excluded_from_surface_risk_for_astar": ["euclidean_step_distance_m"],
            "astar_cost_formula": "step_dist * (1.0 + astar_risk_multiplier * surface_risk[r, c])",
        },
        "ctx_analysis_integrated": ctx_hazard is not None,
        "deliverables": {
            "rover_risk_map": "surface_risk_rover.npy",
            "astronaut_risk_map": "surface_risk_astronaut.npy",
            "unknown_mask": "unknown_mask.npy",
            "config": "risk_config.json",
            "threejs_overlay": "risk_overlay.png",
        },
        "profiles": {
            "rover": meta_rover,
            "astronaut": meta_astronaut,
        },
        "threshold_comparison_20_vs_25_deg": {
            "20_deg_stats": meta_rover_20["stats"],
            "25_deg_stats": meta_rover["stats"],
            "route_20_deg": eval_rover_20,
            "route_25_deg": eval_rover_25,
        },
        "route_verification": {
            "start_rc": list(start_rc),
            "goal_rc": list(goal_rc),
            "risk_blind_reference": eval_blind,
            "rover_astar": eval_rover_25,
            "astronaut_astar": eval_astro,
        },
        "disclaimer": NASA_DISCLAIMER,
    }

    with open(BASE_DIR / "risk_config.json", "w", encoding="utf-8") as f:
        json.dump(risk_config, f, ensure_ascii=False, indent=2)

    save_threejs_overlay_and_preview(
        dem, risk_rover, risk_astronaut, unknown_mask,
        path_blind, path_rover_20, path_rover_25, path_astro,
        eval_rover_20, eval_rover_25, eval_astro,
    )

    # 5. Terminal Raporu
    print("\n================ TOPOGRAFYA & TERRAIN RISK ANALYSIS RAPORU ================")
    print(f"Veri Kaynağı     : {source_label}")
    print(f"Grid Boyutu      : {h} x {w} piksel | Piksel Ölçeği: {dx_m:.2f}m x {dy_m:.2f}m")
    print(f"Yükseklik Aralığı: {np.nanmin(dem):.1f} m .. {np.nanmax(dem):.1f} m | Maks. Eğim: {np.nanmax(slope_deg):.2f}°")
    print(f"CTX Katmanı      : {'Aktif' if ctx_hazard is not None else 'Bekleniyor (eklenince otomatik aktifleşir)'}\n")

    print(f"[Rover (25° Eşik)]     Geçilmez (1.0): %{meta_rover['stats']['impassable_1_0_pct']} | Ort. Risk: {meta_rover['stats']['mean_risk']}")
    print(f"[Rover (20° Eşik)]     Geçilmez (1.0): %{meta_rover_20['stats']['impassable_1_0_pct']} | Ort. Risk: {meta_rover_20['stats']['mean_risk']}")
    print(f"[Astronaut (18° Eşik)] Geçilmez (1.0): %{meta_astronaut['stats']['impassable_1_0_pct']} | Ort. Risk: {meta_astronaut['stats']['mean_risk']}\n")

    for ev in (eval_blind, eval_rover_25, eval_rover_20, eval_astro):
        if not ev["valid"]:
            print(f"- {ev['scenario']:<26}: ROTA BULUNAMADI")
        else:
            print(
                f"- {ev['scenario']:<26}: Mesafe={ev['distance_m']:>7.1f}m | "
                f"Ort.Risk={ev['mean_risk_0_1']:.3f} | "
                f"Maks.Eğim={ev['max_slope_deg']:>5.2f}° | "
                f"İhlal={ev['impassable_violations']} | "
                f"Güvenli={ev['avoided_impassable_and_unknown']}"
            )
    print(f"\n{NASA_DISCLAIMER}")
    print("===========================================================================\n")


if __name__ == "__main__":
    main()