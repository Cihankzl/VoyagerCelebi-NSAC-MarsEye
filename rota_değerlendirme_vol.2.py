import heapq
import io
import json
from collections import deque
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

# ==============================================================================
# 1. YAPILANDIRMA VE PROFİL PARAMETRELERİ
# ==============================================================================
BASE_DIR = Path(__file__).resolve().parent

TIF_FILENAME = "1kmistenilenyer.tif"            # Birincil doğrudan okunacak GeoTIFF dosyası
DEM_PATH = BASE_DIR / "dem.npy"                 # .tif yoksa kullanılacak yedek DEM (örn. 198x200)
SLOPE_PATH = BASE_DIR / "slope.npy"             # Senkronize edilecek / okunacak eğim matrisi (°)
ROUGHNESS_PATH = BASE_DIR / "roughness.npy"     # Senkronize edilecek / okunacak TRI matrisi (m)
CTX_HAZARD_PATH = BASE_DIR / "ctx_hazard.npy"   # Opsiyonel: CTX kaya/kum tehlike katmanı [0..1]

MARS_RADIUS_M = 3_396_190.0  # Mars ortalama yarıçapı (metre)
AREA_SPAN_M = 1000.0         # GeoTIFF başlığı olmayan .npy dosyaları için varsayılan kenar (m)
SUITABILITY_PASS_SCORE = 70.0

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

NASA_DISCLAIMER = (
    "NOT: Kullanılan eğim ve pürüzlülük eşikleri algoritmik arazi uygunluğu testi içindir; "
    "NASA tarafından doğrulanmış astronot güvenlik sınırları değildir."
)


# ==============================================================================
# 2. GEOTIFF (.tif) TOPOGRAFYA OKUMA VE BİLEŞEN KATMANLARI
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
            elif res_x > 0 and res_y > 0:
                dx_m, dy_m = res_x, res_y
            else:
                h, w = dem.shape
                dx_m = dy_m = AREA_SPAN_M / max(h, w)

            return dem, dx_m, dy_m

    except ImportError:
        import tifffile

        dem = tifffile.imread(tif_path).astype(np.float64)
        if dem.ndim == 3:
            dem = dem[:, :, 0]
        dem[(dem < -15000.0) | (dem > 30000.0)] = np.nan
        h, w = dem.shape
        dx_m = dy_m = AREA_SPAN_M / max(h, w)
        return dem, dx_m, dy_m


def load_topography_from_tif() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray | None, float, float, str]:
    """Öncelikle doğrudan .tif dosyasını okur; yoksa dem.npy dosyasına geçer."""
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

        source_label = "dem.npy (NumPy Matrisi)"
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
# 3. [0.0 - 1.0] RİSK MODELLERİ, UNKNOWN MASKESİ VE TAMPON MESAFESİ
# ==============================================================================
def build_surface_risk(
    profile: dict,
    slope_deg: np.ndarray,
    roughness_m: np.ndarray,
    unknown_mask: np.ndarray,
    pixel_size_m: float,
    ctx_hazard: np.ndarray | None = None,
) -> tuple[np.ndarray, dict]:
    """[0.0, 1.0] aralığında float32 ajan yüzey risk haritası üretir."""
    imp = profile["experimental_impassable"]
    max_slope = float(imp["max_slope_deg"])
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

    impassable = (slope_deg >= max_slope) | (roughness_m >= max_rough_m)
    if ctx_hazard is not None:
        impassable = impassable | (ctx_hazard >= max_ctx)

    risk[impassable] = 1.0
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


def compute_impassable_clearance_m(
    surface_risk: np.ndarray,
    unknown_mask: np.ndarray,
    pixel_size_m: float,
    max_track_m: float = 25.0,
) -> np.ndarray:
    """Her hücrenin en yakın geçilmez (1.0) veya bilinmeyen (unknown=1) hücreye uzaklığını (m) hesaplar."""
    rows, cols = surface_risk.shape
    dist_cells = np.full((rows, cols), np.inf, dtype=np.float64)
    q = deque()

    blocked_r, blocked_c = np.where((surface_risk >= 0.999) | (unknown_mask == 1))
    dist_cells[blocked_r, blocked_c] = 0.0
    for r, c in zip(blocked_r.tolist(), blocked_c.tolist()):
        q.append((r, c))

    max_cells = int(np.ceil(max_track_m / pixel_size_m))
    moves = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142),
    ]

    while q:
        r, c = q.popleft()
        d_curr = dist_cells[r, c]
        if d_curr >= max_cells:
            continue
        for dr, dc, step in moves:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                nd = d_curr + step
                if nd < dist_cells[nr, nc]:
                    dist_cells[nr, nc] = nd
                    q.append((nr, nc))

    return np.minimum(dist_cells * pixel_size_m, max_track_m)


# ==============================================================================
# 4. ALTERNATİF ROTA ALGORİTMALARI (NO DOUBLE-COUNTING)
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
    raise RuntimeError("Haritada güvenli başlangıç/hedef hücresi bulunamadı.")


def plan_algorithm_route(
    mode: str,
    start: tuple[int, int],
    goal: tuple[int, int],
    unknown_mask: np.ndarray,
    slope_only_risk: np.ndarray,
    risk_rover: np.ndarray,
    risk_astronaut: np.ndarray,
    clearance_rover_m: np.ndarray,
) -> list[tuple[int, int]]:
    """
    Çifte saymayı (double-counting) önleyen 5 karşılaştırma modu:
      - 'shortest'        : Riski yok sayan klasik en kısa mesafe A* (referans)
      - 'slope_only'      : Yalnızca normalize eğim riskini [0-1] kullanan A*
      - 'rover_multi'     : surface_risk_rover.npy [0-1] (eğim+pürüzlülük[+CTX]) kullanan A*
      - 'rover_clearance' : surface_risk_rover.npy + engel kenarlarından güvenlik tamponu bırakan A*
      - 'astronaut_multi' : surface_risk_astronaut.npy [0-1] (18° eşikli, eğim ağırlıklı) kullanan A*
    """
    rows, cols = unknown_mask.shape
    open_set = [(0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score = {start: 0.0}

    mult_rover = AGENT_PROFILES["rover"]["astar_risk_multiplier"]
    mult_astro = AGENT_PROFILES["astronaut"]["astar_risk_multiplier"]

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

            if mode == "shortest":
                step_cost = step_dist

            elif mode == "slope_only":
                if unknown_mask[nr, nc] == 1 or slope_only_risk[nr, nc] >= 0.999:
                    continue
                step_cost = step_dist * (1.0 + mult_rover * float(slope_only_risk[nr, nc]))

            elif mode == "rover_multi":
                if unknown_mask[nr, nc] == 1 or risk_rover[nr, nc] >= 0.999:
                    continue
                step_cost = step_dist * (1.0 + mult_rover * float(risk_rover[nr, nc]))

            elif mode == "rover_clearance":
                if unknown_mask[nr, nc] == 1 or risk_rover[nr, nc] >= 0.999:
                    continue
                clr = float(clearance_rover_m[nr, nc])
                buffer_pen = max(0.0, (8.0 - clr) / 2.5) ** 2
                step_cost = step_dist * (1.0 + mult_rover * float(risk_rover[nr, nc]) + buffer_pen)

            elif mode == "astronaut_multi":
                if unknown_mask[nr, nc] == 1 or risk_astronaut[nr, nc] >= 0.999:
                    continue
                step_cost = step_dist * (1.0 + mult_astro * float(risk_astronaut[nr, nc]))

            else:
                raise ValueError(f"Bilinmeyen algoritma modu: {mode}")

            tentative_g = g_score[curr] + step_cost
            nxt = (nr, nc)
            if tentative_g < g_score.get(nxt, float("inf")):
                came_from[nxt] = curr
                g_score[nxt] = tentative_g
                h = np.hypot(goal[0] - nr, goal[1] - nc)
                heapq.heappush(open_set, (tentative_g + h, nxt))

    return []


# ==============================================================================
# 5. ÇOK KRİTERLİ ARAZİ UYGUNLUK DEĞERLENDİRMESİ
# ==============================================================================
def evaluate_route_suitability(
    name: str,
    path: list[tuple[int, int]],
    baseline_dist_m: float,
    dem: np.ndarray,
    slope_deg: np.ndarray,
    roughness_m: np.ndarray,
    eval_risk_map: np.ndarray,
    unknown_mask: np.ndarray,
    clearance_m: np.ndarray,
    max_allowed_slope_deg: float,
    dx_m: float,
    dy_m: float,
) -> dict:
    """Rotaların yalnızca kısa değil, arazi koşullarına uygun olup olmadığını çok kriterli puanlar."""
    if not path:
        return {"algorithm": name, "valid": False, "terrain_suitable": False}

    pts = np.array(path, dtype=float)
    dist_m = float(np.sum(np.hypot(np.diff(pts[:, 0]) * dy_m, np.diff(pts[:, 1]) * dx_m)))
    detour_pct = ((dist_m - baseline_dist_m) / baseline_dist_m * 100.0) if baseline_dist_m > 0 else 0.0

    elevs = np.array([dem[r, c] for r, c in path], dtype=float)
    total_climb_m = float(np.sum(np.maximum(0.0, np.diff(elevs))))
    net_gain_m = max(0.0, float(elevs[-1] - elevs[0]))
    extra_climb_m = max(0.0, total_climb_m - net_gain_m)

    risks = np.array([eval_risk_map[r, c] for r, c in path], dtype=float)
    unknowns = np.array([unknown_mask[r, c] for r, c in path], dtype=int)
    slopes = np.array([slope_deg[r, c] for r, c in path], dtype=float)
    roughs = np.array([roughness_m[r, c] for r, c in path], dtype=float)
    clearances = np.array([clearance_m[r, c] for r, c in path], dtype=float)

    impassable_hits = int(np.sum(risks >= 0.999))
    unknown_hits = int(np.sum(unknowns == 1))
    mean_risk = float(np.mean(risks))
    max_risk = float(np.max(risks))
    max_slope = float(np.nanmax(slopes))
    mean_rough = float(np.nanmean(roughs))
    mean_clearance = float(np.mean(clearances))

    # Arazi Uygunluk Puanı (0 - 100)
    score = 100.0
    score -= (impassable_hits + unknown_hits) * 10.0   # Geçilmez / bilinmeyen hücre ihlali
    score -= mean_risk * 38.0                          # Ortalama yüzey riski yükü [0-1]
    score -= max(0.0, max_risk - 0.55) * 22.0          # Tepe risk cezası
    score -= (extra_climb_m / 10.0) * 1.5              # Gereksiz iniş-çıkış / tırmanış enerjisi
    score += min(10.0, mean_clearance * 0.6)           # Engel kenarlarından uzak durma ödülü
    score -= max(0.0, detour_pct - 12.0) * 0.25        # Aşırı yol uzaması dengesi
    suitability_score = round(float(np.clip(score, 0.0, 100.0)), 1)

    is_suitable = (
        (impassable_hits == 0)
        and (unknown_hits == 0)
        and (max_slope < max_allowed_slope_deg)
        and (suitability_score >= SUITABILITY_PASS_SCORE)
    )

    return {
        "algorithm": name,
        "valid": True,
        "terrain_suitable": is_suitable,
        "suitability_score_100": suitability_score,
        "distance_m": round(dist_m, 2),
        "detour_pct": round(detour_pct, 1),
        "mean_risk_0_1": round(mean_risk, 4),
        "max_risk_0_1": round(max_risk, 4),
        "max_slope_deg": round(max_slope, 2),
        "mean_roughness_m": round(mean_rough, 3),
        "extra_climb_m": round(extra_climb_m, 2),
        "mean_clearance_m": round(mean_clearance, 2),
        "impassable_violations": impassable_hits,
        "unknown_violations": unknown_hits,
    }


# ==============================================================================
# 6. KARŞILAŞTIRMA GRAFİĞİ (ONEDRIVE KİLİT KORUMALI)
# ==============================================================================
def save_comparison_figure(
    risk_rover: np.ndarray,
    risk_astronaut: np.ndarray,
    routes: dict[str, list[tuple[int, int]]],
    evaluations: list[dict],
) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(20, 6.2), dpi=180)

    # Panel 1: Rover [0-1] Risk Haritası + Rover & Referans Rotaları
    im0 = axes[0].imshow(risk_rover, cmap="RdYlGn_r", vmin=0.0, vmax=1.0)
    axes[0].set_title("1. Rover [0–1] Risk Katmanı & Rotalar", fontweight="bold")
    rover_styles = {
        "shortest": ("#ffffff", "--", 1.5, "1. En Kısa A* (İhlalli)"),
        "slope_only": ("#00ffff", "-", 1.9, "2. Sadece Eğim A*"),
        "rover_multi": ("#2980b9", "-", 2.3, "3. Çok Katmanlı Rover A*"),
        "rover_clearance": ("#8e44ad", "-", 2.3, "4. Tamponlu Rover A*"),
    }
    for key, (color, ls, lw, label) in rover_styles.items():
        pts = routes.get(key, [])
        if pts:
            axes[0].plot([c for _, c in pts], [r for r, _ in pts], color=color, ls=ls, lw=lw, label=label)

    ref_path = routes.get("rover_multi") or routes.get("shortest")
    if ref_path:
        axes[0].scatter([ref_path[0][1]], [ref_path[0][0]], c="white", edgecolors="black", s=50, zorder=6, label="Başlangıç")
        axes[0].scatter([ref_path[-1][1]], [ref_path[-1][0]], c="#1f618d", edgecolors="white", s=60, marker="s", zorder=6, label="Hedef")
    axes[0].legend(loc="lower left", fontsize=7.3, framealpha=0.92)
    plt.colorbar(im0, ax=axes[0], fraction=0.046, pad=0.04)

    # Panel 2: Astronaut [0-1] Risk Haritası (18° Eşik) + Astronaut Rotası
    im1 = axes[1].imshow(risk_astronaut, cmap="RdYlGn_r", vmin=0.0, vmax=1.0)
    axes[1].set_title("2. Astronaut [0–1] Risk Katmanı (18° Eşik)", fontweight="bold")
    if routes.get("shortest"):
        pts = routes["shortest"]
        axes[1].plot([c for _, c in pts], [r for r, _ in pts], color="white", ls="--", lw=1.5, label="1. En Kısa A* (İhlalli)")
    if routes.get("astronaut_multi"):
        pts = routes["astronaut_multi"]
        axes[1].plot([c for _, c in pts], [r for r, _ in pts], color="#2980b9", lw=2.4, label="5. Çok Katmanlı Astronaut A*")
    axes[1].legend(loc="lower left", fontsize=7.5, framealpha=0.92)
    plt.colorbar(im1, ax=axes[1], fraction=0.046, pad=0.04)

    # Panel 3: Arazi Uygunluk Puanları (0 - 100)
    valid_evals = [e for e in evaluations if e.get("valid")]
    labels = [e["algorithm"].split(" (")[0] for e in valid_evals]
    scores = [e["suitability_score_100"] for e in valid_evals]
    dists = [e["distance_m"] for e in valid_evals]

    x = np.arange(len(labels))
    bar_colors = ["#e74c3c", "#1abc9c", "#2980b9", "#8e44ad", "#f39c12"][: len(labels)]
    bars = axes[2].bar(x, scores, color=bar_colors, width=0.52)
    axes[2].set_ylim(0, 114)
    axes[2].set_ylabel("Arazi Uygunluk Skoru (0–100)")
    axes[2].set_title("3. Çok Kriterli Arazi Uygunluk Skoru", fontweight="bold")
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(labels, rotation=18, ha="right", fontsize=8.2)
    axes[2].axhline(SUITABILITY_PASS_SCORE, color="#2c3e50", ls=":", lw=1.3, label=f"Uygunluk Eşiği ({SUITABILITY_PASS_SCORE:.0f})")
    axes[2].legend(loc="upper left", fontsize=8)

    for bar, d_val, s_val in zip(bars, dists, scores):
        axes[2].text(
            bar.get_x() + bar.get_width() / 2,
            s_val + 2.2,
            f"{s_val}p\n({d_val:.0f}m)",
            ha="center",
            fontsize=7.8,
            fontweight="bold",
        )

    fig.text(0.5, 0.01, NASA_DISCLAIMER, ha="center", fontsize=8.8, style="italic", color="#922b21")
    plt.tight_layout(rect=[0, 0.04, 1, 0.98])

    buf = io.BytesIO()
    plt.savefig(buf, format="png", dpi=200)
    plt.close(fig)

    out_path = BASE_DIR / "algorithms_hazard_comparison.png"
    try:
        out_path.write_bytes(buf.getvalue())
    except OSError:
        alt_path = BASE_DIR / "algorithms_hazard_comparison_v2.png"
        alt_path.write_bytes(buf.getvalue())
        print(f"[Bilgi] Eski görsel açık olduğu için yeni dosya kaydedildi: {alt_path.name}")


# ==============================================================================
# 7. ANA ÇALIŞTIRMA AKIŞI
# ==============================================================================
def main() -> None:
    dem, slope_deg, roughness_m, ctx_hazard, dx_m, dy_m, source_label = load_topography_from_tif()
    h, w = dem.shape
    pixel_size_m = round(0.5 * (dx_m + dy_m), 3)

    # 1. Ayrı Unknown Maskesi (1 = veri yok / NaN, 0 = geçerli)
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

    # Sadece eğim karşılaştırması için normalize eğim risk haritası [0.0, 1.0]
    max_rover_slope = AGENT_PROFILES["rover"]["experimental_impassable"]["max_slope_deg"]
    slope_only_risk = np.clip(np.nan_to_num(slope_deg, nan=max_rover_slope) / max_rover_slope, 0.0, 1.0).astype(np.float32)
    slope_only_risk[unknown_mask == 1] = 1.0

    # Boyut Tutarlılığı Doğrulaması
    assert risk_rover.shape == (h, w)
    assert risk_astronaut.shape == (h, w)
    assert unknown_mask.shape == (h, w)

    # Standart Teslim Dosyalarını Kaydet
    np.save(BASE_DIR / "surface_risk_rover.npy", risk_rover)
    np.save(BASE_DIR / "surface_risk_astronaut.npy", risk_astronaut)
    np.save(BASE_DIR / "unknown_mask.npy", unknown_mask)

    clearance_rover_m = compute_impassable_clearance_m(risk_rover, unknown_mask, pixel_size_m, max_track_m=25.0)
    clearance_astro_m = compute_impassable_clearance_m(risk_astronaut, unknown_mask, pixel_size_m, max_track_m=25.0)

    # 3. Ortak Başlangıç / Hedef Koridoru Üzerinde 5 Algoritma & Ajan Testi
    start_rc = find_nearest_safe_cell(risk_astronaut, unknown_mask, (int(h * 0.18), int(w * 0.18)))
    goal_rc = find_nearest_safe_cell(risk_astronaut, unknown_mask, (int(h * 0.85), int(w * 0.83)))

    scenarios = [
        ("shortest", "1. En Kısa A* (Risk-Blind)", risk_rover, clearance_rover_m, 25.0),
        ("slope_only", "2. Sadece Eğim A* (Slope-Only)", risk_rover, clearance_rover_m, 25.0),
        ("rover_multi", "3. Çok Katmanlı Rover A*", risk_rover, clearance_rover_m, 25.0),
        ("rover_clearance", "4. Tamponlu Rover A*", risk_rover, clearance_rover_m, 25.0),
        ("astronaut_multi", "5. Çok Katmanlı Astronaut A*", risk_astronaut, clearance_astro_m, 18.0),
    ]

    routes = {}
    for mode, _, _, _, _ in scenarios:
        routes[mode] = plan_algorithm_route(
            mode, start_rc, goal_rc, unknown_mask, slope_only_risk, risk_rover, risk_astronaut, clearance_rover_m
        )

    pts_base = np.array(routes["shortest"], dtype=float)
    baseline_dist = float(np.sum(np.hypot(np.diff(pts_base[:, 0]) * dy_m, np.diff(pts_base[:, 1]) * dx_m)))

    evaluations = []
    for mode, label, eval_map, clr_map, max_slp in scenarios:
        ev = evaluate_route_suitability(
            label, routes[mode], baseline_dist, dem, slope_deg, roughness_m,
            eval_map, unknown_mask, clr_map, max_slp, dx_m, dy_m
        )
        evaluations.append(ev)

    # 4. Standart risk_config.json ve Değerlendirme Raporunu Kaydet
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
        },
        "profiles": {
            "rover": meta_rover,
            "astronaut": meta_astronaut,
        },
        "corridor": {"start_rc": list(start_rc), "goal_rc": list(goal_rc)},
        "algorithm_evaluations": evaluations,
        "disclaimer": NASA_DISCLAIMER,
    }

    with open(BASE_DIR / "risk_config.json", "w", encoding="utf-8") as f:
        json.dump(risk_config, f, ensure_ascii=False, indent=2)

    save_comparison_figure(risk_rover, risk_astronaut, routes, evaluations)

    # 5. Terminal Karşılaştırma Tablosu
    print("\n================ ROTA DEĞERLENDİRMESİ & ARAZİ UYGUNLUK RAPORU ================")
    print(f"Veri Kaynağı  : {source_label} | Boyut: {h} x {w} ({dx_m:.2f}m x {dy_m:.2f}m)")
    print(f"Test Koridoru : {start_rc} -> {goal_rc} | CTX Katmanı: {'Aktif' if ctx_hazard is not None else 'Bekleniyor'}\n")

    header = f"{'Algoritma / Ajan':<30} | {'Mesafe':>8} | {'OrtRisk':>7} | {'MaksEğim':>8} | {'Ort.TRI':>7} | {'Tampon':>6} | {'İhlal':>5} | {'Skor':>5} | {'Uygunluk'}"
    print(header)
    print("-" * len(header))
    for ev in evaluations:
        if not ev["valid"]:
            print(f"{ev['algorithm']:<30} | ROTA BULUNAMADI")
            continue
        status = "UYGUN" if ev["terrain_suitable"] else "UYGUN DEĞİL"
        total_viol = ev["impassable_violations"] + ev["unknown_violations"]
        print(
            f"{ev['algorithm']:<30} | "
            f"{ev['distance_m']:>7.1f}m | "
            f"{ev['mean_risk_0_1']:>7.3f} | "
            f"{ev['max_slope_deg']:>7.2f}° | "
            f"{ev['mean_roughness_m']:>6.3f}m | "
            f"{ev['mean_clearance_m']:>5.1f}m | "
            f"{total_viol:>5d} | "
            f"{ev['suitability_score_100']:>5.1f} | "
            f"{status}"
        )
    print(f"\n{NASA_DISCLAIMER}")
    print("============================================================================\n")


if __name__ == "__main__":
    main()