import heapq
import json
from collections import deque
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch

# ==============================================================================
# 1. DOSYA YOLLARI VE ARAZİ TEHLİKE EŞİKLERİ
# ==============================================================================
BASE_DIR = Path(__file__).resolve().parent
DEM_PATH = BASE_DIR / "dem.npy"
SLOPE_PATH = BASE_DIR / "slope.npy"
PIXEL_SIZE_M = 1.0  # Gerçek Gale DEM çözünürlüğü (1.00 m/piksel)

# Eğim ve Yüzey Engebeliliği (TRI) Eşikleri
SLOPE_MED_DEG = 15.0
SLOPE_HIGH_DEG = 25.0
TRI_MED_M = 0.22     # 1m piksel ölçeğinde orta düzey yüzey pürüzlülüğü (metre)
TRI_HIGH_M = 0.45    # Yüksek engebelilik / kaya-krater kenarı engeli (metre)

SUITABILITY_PASS_SCORE = 70.0  # Arazi uygunluk minimum geçer puanı

NASA_DISCLAIMER = (
    "NOT: Kullanılan eğim ve engebelilik eşikleri algoritmik arazi uygunluğu testi içindir; "
    "NASA tarafından doğrulanmış astronot güvenlik sınırları değildir."
)


# ==============================================================================
# 2. ÇOK KATMANLI ARAZİ TEHLİKE HARİTALARI (EĞİM + ENGEBELİLİK + TAMPON)
# ==============================================================================
def compute_terrain_ruggedness_index(dem: np.ndarray) -> np.ndarray:
    """
    Riley Terrain Ruggedness Index (TRI):
    Her hücrenin 8 komşusuyla olan yükseklik farklarının karekök ortalamasını (metre) hesaplar.
    """
    padded = np.pad(dem, pad_width=1, mode="edge")
    center = padded[1:-1, 1:-1]
    sq_diff_sum = np.zeros_like(dem, dtype=np.float64)

    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            nbr = padded[1 + dr : padded.shape[0] - 1 + dr, 1 + dc : padded.shape[1] - 1 + dc]
            sq_diff_sum += (nbr - center) ** 2

    return np.sqrt(sq_diff_sum / 8.0)


def build_composite_hazard(slope_deg: np.ndarray, tri_m: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Eğim ve yüzey engebeliliği katmanlarını birleştirir:
      0 -> Düşük Tehlike (Düz ve pürüzsüz zemin)
      1 -> Orta Tehlike  (Orta eğim veya engebeli zemin)
      2 -> Yüksek Tehlike (Dik yamaç, keskin krater kenarı veya NaN)
    """
    slope_risk = np.zeros(slope_deg.shape, dtype=np.int32)
    slope_risk[(slope_deg >= SLOPE_MED_DEG) & (slope_deg < SLOPE_HIGH_DEG)] = 1
    slope_risk[(slope_deg >= SLOPE_HIGH_DEG) | np.isnan(slope_deg)] = 2

    roughness_risk = np.zeros(tri_m.shape, dtype=np.int32)
    roughness_risk[(tri_m >= TRI_MED_M) & (tri_m < TRI_HIGH_M)] = 1
    roughness_risk[(tri_m >= TRI_HIGH_M) | np.isnan(tri_m)] = 2

    composite_hazard = np.maximum(slope_risk, roughness_risk)
    return slope_risk, roughness_risk, composite_hazard


def compute_hazard_clearance_m(
    hazard_grid: np.ndarray,
    pixel_size_m: float = 1.0,
    max_track_m: float = 25.0,
) -> np.ndarray:
    """Her hücrenin en yakın yüksek tehlikeli (2) hücreye olan mesafesini (metre) hesaplar."""
    rows, cols = hazard_grid.shape
    dist_cells = np.full((rows, cols), np.inf, dtype=np.float64)
    q = deque()

    high_r, high_c = np.where(hazard_grid == 2)
    dist_cells[high_r, high_c] = 0.0
    for r, c in zip(high_r.tolist(), high_c.tolist()):
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
# 3. ALTERNATİF ROTA ALGORİTMALARI
# ==============================================================================
def nearest_safe_cell(hazard_grid: np.ndarray, target_rc: tuple[int, int]) -> tuple[int, int]:
    rows, cols = hazard_grid.shape
    r0 = int(np.clip(target_rc[0], 0, rows - 1))
    c0 = int(np.clip(target_rc[1], 0, cols - 1))
    if hazard_grid[r0, c0] == 0:
        return (r0, c0)

    visited = {(r0, c0)}
    q = deque([(r0, c0)])
    while q:
        r, c = q.popleft()
        if hazard_grid[r, c] == 0:
            return (r, c)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    q.append((nr, nc))
    raise RuntimeError("Güvenli başlangıç/hedef hücresi bulunamadı.")


def plan_route(
    mode: str,
    start: tuple[int, int],
    goal: tuple[int, int],
    dem: np.ndarray,
    slope_deg: np.ndarray,
    tri_m: np.ndarray,
    slope_risk: np.ndarray,
    composite_hazard: np.ndarray,
    clearance_m: np.ndarray,
) -> list[tuple[int, int]]:
    """
    4 farklı maliyet fonksiyonuna göre rota üretir:
      - 'shortest'    : Yalnızca Öklid mesafesi (arazi koşullarını yok sayar)
      - 'slope_only'  : Yalnızca eğim sınırlarını dikkate alan klasik Risk-Aware A*
      - 'multi_hazard': Eğim + Yüzey Engebeliliği (TRI) + Tırmanış Enerjisi (dz > 0) A*
      - 'clearance'   : Tehlike tampon mesafesini koruyan ve kümülatif riski minimize eden rota
    """
    rows, cols = dem.shape
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
        curr_elev = dem[cr, cc]

        for dr, dc, step_dist in moves:
            nr, nc = cr + dr, cc + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue

            if mode == "shortest":
                multiplier = 1.0
                extra_cost = 0.0

            elif mode == "slope_only":
                if slope_risk[nr, nc] == 2:
                    continue
                multiplier = 3.0 if slope_risk[nr, nc] == 1 else 1.0
                extra_cost = 0.0

            elif mode == "multi_hazard":
                if composite_hazard[nr, nc] == 2:
                    continue
                climb = max(0.0, float(dem[nr, nc] - curr_elev))
                slope_pen = 0.08 * float(slope_deg[nr, nc])
                rough_pen = 4.5 * float(tri_m[nr, nc])
                energy_pen = 3.0 * climb
                multiplier = 1.0 + slope_pen + rough_pen
                extra_cost = energy_pen

            elif mode == "clearance":
                if composite_hazard[nr, nc] == 2:
                    continue
                clr = float(clearance_m[nr, nc])
                buffer_pen = max(0.0, (8.0 - clr) / 2.0) ** 2
                hazard_pen = 3.5 if composite_hazard[nr, nc] == 1 else 0.0
                multiplier = 1.0 + hazard_pen + buffer_pen
                extra_cost = 0.0

            else:
                raise ValueError(f"Bilinmeyen mod: {mode}")

            tentative_g = g_score[curr] + (step_dist * multiplier) + extra_cost
            nxt = (nr, nc)

            if tentative_g < g_score.get(nxt, float("inf")):
                came_from[nxt] = curr
                g_score[nxt] = tentative_g
                h = np.hypot(goal[0] - nr, goal[1] - nc)
                heapq.heappush(open_set, (tentative_g + h, nxt))

    return []


# ==============================================================================
# 4. ÇOK KRİTERLİ ARAZİ UYGUNLUK DEĞERLENDİRMESİ
# ==============================================================================
def evaluate_terrain_suitability(
    name: str,
    path: list[tuple[int, int]],
    baseline_dist_m: float,
    dem: np.ndarray,
    slope_deg: np.ndarray,
    tri_m: np.ndarray,
    composite_hazard: np.ndarray,
    clearance_m: np.ndarray,
) -> dict:
    """
    Rotaların yalnızca kısa değil, arazi koşullarına uygun olup olmadığını
    eğim, pürüzlülük, gereksiz ekstra tırmanış ve tehlike tamponuna göre puanlar.
    """
    if not path:
        return {"algorithm": name, "valid": False}

    pts = np.array(path, dtype=float)
    dist_m = float(np.sum(np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1])) * PIXEL_SIZE_M))
    detour_pct = ((dist_m - baseline_dist_m) / baseline_dist_m * 100.0) if baseline_dist_m > 0 else 0.0

    elevs = np.array([dem[r, c] for r, c in path], dtype=float)
    total_climb_m = float(np.sum(np.maximum(0.0, np.diff(elevs))))
    net_elev_gain = max(0.0, float(elevs[-1] - elevs[0]))
    extra_climb_m = max(0.0, total_climb_m - net_elev_gain)

    slopes = np.array([slope_deg[r, c] for r, c in path], dtype=float)
    tris = np.array([tri_m[r, c] for r, c in path], dtype=float)
    hazards = np.array([composite_hazard[r, c] for r, c in path], dtype=int)
    clearances = np.array([clearance_m[r, c] for r, c in path], dtype=float)

    high_violations = int(np.sum(hazards == 2))
    med_cells = int(np.sum(hazards == 1))
    max_slope = float(np.nanmax(slopes))
    mean_slope = float(np.nanmean(slopes))
    mean_tri = float(np.nanmean(tris))
    mean_clearance = float(np.nanmean(clearances))

    # Arazi Uygunluk Puanı (0 - 100)
    score = 100.0
    score -= high_violations * 8.0                 # Geçilmez engel ihlali cezası
    score -= max(0.0, max_slope - 15.0) * 1.4      # Tepe eğim cezası
    score -= mean_slope * 1.1                      # Ortalama eğim yükü
    score -= mean_tri * 45.0                       # Yüzey engebeliliği (sarsıntı/tekerlek riski)
    score -= (extra_climb_m / 10.0) * 1.5          # Gereksiz iniş-çıkış / ekstra tırmanış maliyeti
    score += min(10.0, mean_clearance * 0.6)       # Tehlike bölgelerinden uzak durma ödülü
    score -= max(0.0, detour_pct - 10.0) * 0.25    # Aşırı yol uzaması dengesi
    suitability_score = round(float(np.clip(score, 0.0, 100.0)), 1)

    is_terrain_suitable = (
        (high_violations == 0)
        and (max_slope < SLOPE_HIGH_DEG)
        and (suitability_score >= SUITABILITY_PASS_SCORE)
    )

    return {
        "algorithm": name,
        "valid": True,
        "terrain_suitable": is_terrain_suitable,
        "suitability_score_100": suitability_score,
        "distance_m": round(dist_m, 2),
        "detour_vs_shortest_pct": round(detour_pct, 1),
        "max_slope_deg": round(max_slope, 2),
        "mean_slope_deg": round(mean_slope, 2),
        "mean_roughness_tri_m": round(mean_tri, 3),
        "total_climb_m": round(total_climb_m, 2),
        "extra_climb_m": round(extra_climb_m, 2),
        "mean_hazard_clearance_m": round(mean_clearance, 2),
        "medium_hazard_cells": med_cells,
        "high_hazard_violations": high_violations,
    }


# ==============================================================================
# 5. GÖRSELLEŞTİRME VE ÇIKTI ÜRETİMİ
# ==============================================================================
def save_comparison_figure(
    tri_m: np.ndarray,
    composite_hazard: np.ndarray,
    routes: dict[str, list[tuple[int, int]]],
    evaluations: list[dict],
) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(20, 6.2), dpi=180)
    cmap_haz = ListedColormap(["#2ecc71", "#f1c40f", "#e74c3c"])

    # Panel 1: Yüzey Engebeliliği (TRI)
    im0 = axes[0].imshow(tri_m, cmap="magma", vmin=0, vmax=float(np.nanpercentile(tri_m, 98)))
    axes[0].set_title("1. Yüzey Engebeliliği Katmanı (TRI - m)", fontweight="bold")
    plt.colorbar(im0, ax=axes[0], fraction=0.046, pad=0.04)

    # Panel 2: Bileşik Tehlike Katmanı ve 4 Algoritmanın Rotaları
    axes[1].imshow(composite_hazard, cmap=cmap_haz, vmin=0, vmax=2)
    axes[1].set_title("2. Bileşik Tehlike Katmanı & 4 Rota Karşılaştırması", fontweight="bold")

    styles = {
        "shortest": ("#ffffff", "--", 1.7, "1. En Kısa A* (İhlalli)"),
        "slope_only": ("#00ffff", "-", 2.0, "2. Sadece Eğim A*"),
        "multi_hazard": ("#2980b9", "-", 2.4, "3. Çok Katmanlı + Enerji A*"),
        "clearance": ("#9b59b6", "-", 2.4, "4. Güvenlik Tamponlu Rota"),
    }
    for key, (color, ls, lw, label) in styles.items():
        pts = routes.get(key, [])
        if pts:
            axes[1].plot([c for _, c in pts], [r for r, _ in pts], color=color, ls=ls, lw=lw, label=label)

    # Başlangıç ve hedef noktalarını işaretle
    sample_route = routes.get("multi_hazard") or routes.get("shortest")
    if sample_route:
        axes[1].scatter([sample_route[0][1]], [sample_route[0][0]], c="white", edgecolors="black", s=55, zorder=6, label="Başlangıç")
        axes[1].scatter([sample_route[-1][1]], [sample_route[-1][0]], c="#1f618d", edgecolors="white", s=65, marker="s", zorder=6, label="Hedef")

    patches = [
        Patch(facecolor="#2ecc71", label="0: Düşük Tehlike"),
        Patch(facecolor="#f1c40f", label="1: Orta Tehlike"),
        Patch(facecolor="#e74c3c", label="2: Yüksek Tehlike (Engel)"),
    ]
    handles, labels = axes[1].get_legend_handles_labels()
    # Hedef noktasını kapatmaması için sol alt köşeye yerleştirildi
    axes[1].legend(handles=handles + patches, loc="lower left", fontsize=7.2, framealpha=0.92)

    # Panel 3: Mesafe vs. Arazi Uygunluk Puanı Karşılaştırma Grafiği
    valid_evals = [e for e in evaluations if e.get("valid")]
    alg_labels = [e["algorithm"].split(" (")[0] for e in valid_evals]
    scores = [e["suitability_score_100"] for e in valid_evals]
    dists = [e["distance_m"] for e in valid_evals]

    x = np.arange(len(alg_labels))
    bars = axes[2].bar(x, scores, color=["#e74c3c", "#1abc9c", "#2980b9", "#9b59b6"], width=0.52)
    axes[2].set_ylim(0, 112)
    axes[2].set_ylabel("Arazi Uygunluk Skoru (0–100)")
    axes[2].set_title("3. Arazi Uygunluk Değerlendirmesi", fontweight="bold")
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(alg_labels, rotation=15, ha="right", fontsize=8.5)
    axes[2].axhline(SUITABILITY_PASS_SCORE, color="#2c3e50", ls=":", lw=1.3, label=f"Uygunluk Eşiği ({SUITABILITY_PASS_SCORE:.0f})")
    axes[2].legend(loc="upper left", fontsize=8)

    for bar, d_val, s_val in zip(bars, dists, scores):
        axes[2].text(
            bar.get_x() + bar.get_width() / 2,
            s_val + 2.2,
            f"{s_val} puan\n({d_val:.0f}m)",
            ha="center",
            fontsize=8,
            fontweight="bold",
        )

    fig.text(0.5, 0.01, NASA_DISCLAIMER, ha="center", fontsize=8.8, style="italic", color="#922b21")
    plt.tight_layout(rect=[0, 0.04, 1, 0.98])
    plt.savefig(BASE_DIR / "algorithms_hazard_comparison.png", dpi=200)
    plt.close(fig)


def main() -> None:
    if not DEM_PATH.exists() or not SLOPE_PATH.exists():
        raise FileNotFoundError("Masaüstünde 'dem.npy' ve 'slope.npy' dosyalarının mevcut olduğundan emin olun.")

    dem = np.load(DEM_PATH, allow_pickle=False).astype(np.float64)
    slope_deg = np.load(SLOPE_PATH, allow_pickle=False).astype(np.float64)

    # 1. Yeni Arazi Tehlike Katmanlarını Üret
    tri_m = compute_terrain_ruggedness_index(dem)
    slope_risk, roughness_risk, composite_hazard = build_composite_hazard(slope_deg, tri_m)
    clearance_m = compute_hazard_clearance_m(composite_hazard, PIXEL_SIZE_M, max_track_m=25.0)

    np.save(BASE_DIR / "composite_hazard.npy", composite_hazard)

    # 2. Ortak Başlangıç ve Hedef Noktaları Üzerinde 4 Algoritmayı Çalıştır
    start_rc = nearest_safe_cell(composite_hazard, (176, 176))
    goal_rc = nearest_safe_cell(composite_hazard, (875, 828))

    alg_configs = [
        ("shortest", "1. En Kısa A* (Risk-Blind)"),
        ("slope_only", "2. Sadece Eğim A* (Slope-Only)"),
        ("multi_hazard", "3. Çok Katmanlı + Enerji A*"),
        ("clearance", "4. Güvenlik Tamponlu Rota"),
    ]

    routes = {}
    for mode, _ in alg_configs:
        routes[mode] = plan_route(
            mode, start_rc, goal_rc, dem, slope_deg, tri_m, slope_risk, composite_hazard, clearance_m
        )

    pts_base = np.array(routes["shortest"], dtype=float)
    baseline_dist = float(np.sum(np.hypot(np.diff(pts_base[:, 0]), np.diff(pts_base[:, 1])) * PIXEL_SIZE_M))

    evaluations = []
    for mode, label in alg_configs:
        ev = evaluate_terrain_suitability(
            label, routes[mode], baseline_dist, dem, slope_deg, tri_m, composite_hazard, clearance_m
        )
        evaluations.append(ev)

    # 3. Görsel ve JSON Raporunu Kaydet
    save_comparison_figure(tri_m, composite_hazard, routes, evaluations)

    report = {
        "corridor": {"start_rc": list(start_rc), "goal_rc": list(goal_rc)},
        "hazard_layer_distribution_pct": {
            "low_0": round(float(np.mean(composite_hazard == 0) * 100), 2),
            "medium_1": round(float(np.mean(composite_hazard == 1) * 100), 2),
            "high_2": round(float(np.mean(composite_hazard == 2) * 100), 2),
        },
        "algorithm_evaluations": evaluations,
        "disclaimer": NASA_DISCLAIMER,
    }
    with open(BASE_DIR / "algorithm_comparison_report.json", "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    # 4. Terminal Karşılaştırma Tablosu
    print("\n================ ALTERNATİF ALGORİTMA VE ARAZİ UYGUNLUK RAPORU ================")
    print(f"Bileşik Tehlike Dağılımı -> Düşük: %{report['hazard_layer_distribution_pct']['low_0']} | "
          f"Orta: %{report['hazard_layer_distribution_pct']['medium_1']} | "
          f"Yüksek: %{report['hazard_layer_distribution_pct']['high_2']}")
    print(f"Test Koridoru            -> {start_rc} -> {goal_rc}\n")

    header = f"{'Algoritma':<32} | {'Mesafe':>8} | {'MaksEğim':>8} | {'Ort.TRI':>7} | {'Tırmanış':>8} | {'Tampon':>6} | {'İhlal':>5} | {'Skor':>6} | {'Uygunluk'}"
    print(header)
    print("-" * len(header))
    for ev in evaluations:
        if not ev["valid"]:
            print(f"{ev['algorithm']:<32} | ROTA BULUNAMADI")
            continue
        status = "UYGUN" if ev["terrain_suitable"] else "UYGUN DEĞİL"
        print(
            f"{ev['algorithm']:<32} | "
            f"{ev['distance_m']:>7.1f}m | "
            f"{ev['max_slope_deg']:>7.2f}° | "
            f"{ev['mean_roughness_tri_m']:>6.3f}m | "
            f"{ev['total_climb_m']:>7.1f}m | "
            f"{ev['mean_hazard_clearance_m']:>5.1f}m | "
            f"{ev['high_hazard_violations']:>5d} | "
            f"{ev['suitability_score_100']:>5.1f} | "
            f"{status}"
        )
    print(f"\n{NASA_DISCLAIMER}")
    print("===============================================================================\n")


if __name__ == "__main__":
    main()