import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch

# ==============================================================================
# 1. GİRİŞ YAPILANDIRMASI (Gerçek DEM geldiğinde YALNIZCA burayı değiştirin)
# ==============================================================================
BASE_DIR = Path(__file__).resolve().parent
INPUT_DEM_PATH = BASE_DIR / "terrain.json"   # Örn. gerçek DEM için: BASE_DIR / "mars_dem.npy"
FALLBACK_CELL_SIZE_M = 2.0                   # Dosya içinde çözünürlük yoksa kullanılacak piksel boyutu (m)

# Çıktı dosya yolları
OUTPUT_RISK_NPY = BASE_DIR / "risk.npy"
OUTPUT_PREVIEW_PNG = BASE_DIR / "risk_preview.png"

# Eğim Risk Eşikleri (Derece)
LOW_RISK_MAX_DEG = 15.0    # 0 <= eğim < 15°  -> 0 (Düşük Risk)
HIGH_RISK_MIN_DEG = 25.0   # eğim >= 25°       -> 2 (Yüksek Risk / Engel)


# ==============================================================================
# 2. VERİ YÜKLEME KATMANI (.json, .npy ve .tif destekli)
# ==============================================================================
def load_dem_source(file_path: Path, default_cell_size: float) -> tuple[np.ndarray, float]:
    """
    Farklı kaynaklardan (sentetik JSON veya gerçek DEM .npy/.tif)
    2B yükseklik matrisi (DEM) ve hücre boyutunu (metre) okur.
    """
    if not file_path.exists():
        raise FileNotFoundError(f"Giriş dosyası bulunamadı: {file_path}")

    suffix = file_path.suffix.lower()

    if suffix == ".json":
        with open(file_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        dem = np.asarray(data["elevation_m"], dtype=np.float64)
        cell_size = float(data.get("cell_size_m", default_cell_size))
        return dem, cell_size

    if suffix == ".npy":
        dem = np.asarray(np.load(file_path, allow_pickle=False), dtype=np.float64)
        return dem, float(default_cell_size)

    if suffix in {".tif", ".tiff"}:
        try:
            import rasterio
            with rasterio.open(file_path) as src:
                dem = src.read(1).astype(np.float64)
                cell_size = float(abs(src.transform.a)) if src.transform.a != 0 else default_cell_size
            return dem, cell_size
        except ImportError as exc:
            raise ImportError("GeoTIFF (.tif) okumak için 'pip install rasterio' kurun veya veriyi .npy formatına çevirin.") from exc

    raise ValueError(f"Desteklenmeyen dosya formatı: {suffix}")


# ==============================================================================
# 3. EĞİM HESAPLAMA VE RİSK SINIFLANDIRMA
# ==============================================================================
def compute_slope_deg(dem: np.ndarray, cell_size_m: float) -> np.ndarray:
    """DEM matrisinden derece cinsinden yüzey eğimini hesaplar."""
    dy, dx = np.gradient(dem, cell_size_m)
    slope_rad = np.arctan(np.hypot(dx, dy))
    return np.degrees(slope_rad)


def classify_risk(slope_deg: np.ndarray, low_max: float, high_min: float) -> np.ndarray:
    """
    Eğim haritasını A* algoritmasının doğrudan okuyabileceği 2B tam sayı
    ızgarasına (grid) dönüştürür:
      0 -> Düşük risk (güvenli geçiş)
      1 -> Orta risk  (yüksek maliyetli geçiş)
      2 -> Yüksek risk (geçilmez engel)
    """
    risk_grid = np.zeros(slope_deg.shape, dtype=np.int32)

    # Orta risk bandı
    medium_mask = (slope_deg >= low_max) & (slope_deg < high_min)
    risk_grid[medium_mask] = 1

    # Yüksek risk veya gerçek DEM'deki geçersiz (NaN) pikseller -> Engel (2)
    high_or_invalid_mask = (slope_deg >= high_min) | np.isnan(slope_deg)
    risk_grid[high_or_invalid_mask] = 2

    return risk_grid


# ==============================================================================
# 4. GÖRSELLEŞTİRME VE ÇIKTI ÜRETİMİ
# ==============================================================================
def save_preview_plot(
    dem: np.ndarray,
    slope_deg: np.ndarray,
    risk_grid: np.ndarray,
    output_path: Path,
    low_max: float,
    high_min: float,
) -> None:
    """Yükseklik, Eğim ve 3 sınıflı Risk haritasını tek görselde kaydeder."""
    fig, axes = plt.subplots(1, 3, figsize=(16, 5), dpi=180)

    # Panel 1: Yükseklik (DEM)
    im0 = axes[0].imshow(dem, cmap="terrain", origin="lower")
    axes[0].set_title("1. Yükseklik Haritası (DEM - m)")
    axes[0].set_xlabel("X (Hücre)")
    axes[0].set_ylabel("Y (Hücre)")
    plt.colorbar(im0, ax=axes[0], fraction=0.046, pad=0.04)

    # Panel 2: Eğim (Derece)
    im1 = axes[1].imshow(slope_deg, cmap="inferno", origin="lower")
    axes[1].set_title("2. Yüzey Eğimi (Derece)")
    axes[1].set_xlabel("X (Hücre)")
    plt.colorbar(im1, ax=axes[1], fraction=0.046, pad=0.04)

    # Panel 3: Ayrık Risk Sınıflandırması (0: Yeşil, 1: Sarı, 2: Kırmızı)
    risk_cmap = ListedColormap(["#2ecc71", "#f1c40f", "#e74c3c"])
    axes[2].imshow(risk_grid, cmap=risk_cmap, vmin=0, vmax=2, origin="lower")
    axes[2].set_title("3. A* Risk Haritası")
    axes[2].set_xlabel("X (Hücre)")

    legend_elements = [
        Patch(facecolor="#2ecc71", label=f"0: Düşük (< {low_max:.0f}°)"),
        Patch(facecolor="#f1c40f", label=f"1: Orta ({low_max:.0f}°–{high_min:.0f}°)"),
        Patch(facecolor="#e74c3c", label=f"2: Yüksek / Engel (≥ {high_min:.0f}°)"),
    ]
    axes[2].legend(handles=legend_elements, loc="upper right", framealpha=0.92, fontsize=8.5)

    plt.tight_layout()
    plt.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    # 1. Veriyi yükle
    dem, cell_size_m = load_dem_source(INPUT_DEM_PATH, FALLBACK_CELL_SIZE_M)

    # 2. Eğimi hesapla ve risk sınıflarına ayır
    slope_deg = compute_slope_deg(dem, cell_size_m)
    risk_grid = classify_risk(slope_deg, LOW_RISK_MAX_DEG, HIGH_RISK_MIN_DEG)

    # 3. A* uyumlu .npy çıktısını kaydet
    np.save(OUTPUT_RISK_NPY, risk_grid)

    # 4. Önizleme görselini kaydet
    save_preview_plot(
        dem=dem,
        slope_deg=slope_deg,
        risk_grid=risk_grid,
        output_path=OUTPUT_PREVIEW_PNG,
        low_max=LOW_RISK_MAX_DEG,
        high_min=HIGH_RISK_MIN_DEG,
    )

    # 5. Doğrulama özeti yazdır
    total_cells = risk_grid.size
    low_cnt = int(np.sum(risk_grid == 0))
    med_cnt = int(np.sum(risk_grid == 1))
    high_cnt = int(np.sum(risk_grid == 2))

    print("=== Risk Haritası Üretimi Tamamlandı ===")
    print(f"Giriş Dosyası  : {INPUT_DEM_PATH.name} (Çözünürlük: {cell_size_m} m)")
    print(f"Çıktı Matrisi  : {OUTPUT_RISK_NPY.name} | Boyut: {risk_grid.shape} | Tip: {risk_grid.dtype}")
    print(f"Önizleme Görsel: {OUTPUT_PREVIEW_PNG.name}")
    print(f"Sınıf Dağılımı -> Düşük (0): {low_cnt} (%{low_cnt/total_cells*100:.1f}) | "
          f"Orta (1): {med_cnt} (%{med_cnt/total_cells*100:.1f}) | "
          f"Yüksek (2): {high_cnt} (%{high_cnt/total_cells*100:.1f})")


if __name__ == "__main__":
    main()
