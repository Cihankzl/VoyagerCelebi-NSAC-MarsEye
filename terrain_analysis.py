import json
import os
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np


def generate_sample_dem(size=200):
  """Gerçek Gale DEM gelene kadar kullanılacak örnek arazi (Mount Sharp / sırt engeli).

  1 piksel = 1 metre çözünürlükte 200x200 metrelik alan üretir.
  """
  x = np.linspace(0, size - 1, size)
  y = np.linspace(0, size - 1, size)
  xx, yy = np.meshgrid(x, y)

  # Güneye doğru hafif yükselen taban eğimi
  base_slope = 0.08 * yy

  # Ortada dik bir sırt engeli (x=85, y=100), sağ tarafta (x>150) güvenli geçiş boynu
  ridge = (
      28.0
      * np.exp(-((yy - 100) ** 2) / (2 * 14**2))
      * np.exp(-((xx - 85) ** 2) / (2 * 45**2))
  )

  # Doğal yüzey pürüzlülüğü
  rng = np.random.default_rng(42)
  roughness = rng.normal(0, 0.15, size=(size, size))

  dem = -4150.0 + base_slope + ridge + roughness
  return dem.astype(np.float32)


def calculate_slope_and_risk(
    dem, cell_size_m=1.0, low_thresh=8.0, high_thresh=15.0
):
  """DEM matrisinden eğim (derece) ve 3 seviyeli risk haritası hesaplar.

  0: Düşük Risk (< 8°)
  1: Orta Risk  (8° - 15°)
  2: Yüksek Risk (>= 15° - Astronot EVA devrilme sınırı)
  """
  dz_dy, dz_dx = np.gradient(dem, cell_size_m, cell_size_m)
  slope_deg = np.degrees(np.arctan(np.hypot(dz_dx, dz_dy))).astype(np.float32)

  risk = np.zeros_like(slope_deg, dtype=np.uint8)
  risk[(slope_deg >= low_thresh) & (slope_deg < high_thresh)] = 1
  risk[slope_deg >= high_thresh] = 2

  return slope_deg, risk


def test_astar_routes(slope_deg, risk, routes_file="routes.json"):
  """4. sınıf EEE'den routes.json geldiğinde A* rotalarını otomatik test eder."""
  with open(routes_file, "r", encoding="utf-8") as f:
    routes = json.load(f)

  print("\n--- A* ROTA DOĞRULAMA TESTİ ---")
  parsed_paths = {}

  for route_key in ["shortest_route", "risk_aware_route"]:
    if route_key not in routes:
      continue
    path = np.array(routes[route_key]["path"])
    rows, cols = path[:, 0], path[:, 1]

    route_slopes = slope_deg[rows, cols]
    route_risks = risk[rows, cols]

    high_risk_steps = int(np.sum(route_risks == 2))
    med_risk_steps = int(np.sum(route_risks == 1))
    max_slope = float(np.max(route_slopes))

    status = (
        "BAŞARILI (Güvenli)"
        if high_risk_steps == 0
        else "RİSKLİ (15° Sınırı Aşıldı)"
    )
    print(f"[{route_key}] -> {status}")
    print(
        f"  Max Eğim: {max_slope:.2f}° | Orta Risk Adım: {med_risk_steps} |"
        f" Yüksek Risk Adım: {high_risk_steps}"
    )
    parsed_paths[route_key] = path

  return parsed_paths


def create_2d_visuals(dem, slope_deg, risk, parsed_paths=None):
  """2D analiz haritasını (risk_analysis_2d.png) ve arayüz kaplamasını kaydeder."""
  fig, axes = plt.subplots(1, 3, figsize=(16, 5))

  # 1. Yükseklik (DEM)
  im0 = axes[0].imshow(dem, cmap="terrain")
  axes[0].set_title("Örnek DEM Yükseklik (m)")
  axes[0].set_xlabel("X (metre)")
  axes[0].set_ylabel("Y (metre)")
  plt.colorbar(im0, ax=axes[0], fraction=0.046)

  # 2. Eğim Haritası
  im1 = axes[1].imshow(slope_deg, cmap="inferno", vmin=0, vmax=25)
  axes[1].set_title("Arazi Eğimi (Derece)")
  axes[1].set_xlabel("X (metre)")
  plt.colorbar(im1, ax=axes[1], fraction=0.046)

  # 3. Risk Haritası (Yeşil: Düşük, Sarı: Orta, Kırmızı: Yüksek)
  cmap_risk = mcolors.ListedColormap(["#2ecc71", "#f1c40f", "#e74c3c"])
  norm = mcolors.BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap_risk.N)
  im2 = axes[2].imshow(risk, cmap=cmap_risk, norm=norm)
  axes[2].set_title("Risk Sınıflandırması (0:Düşük, 1:Orta, 2:Yüksek)")
  axes[2].set_xlabel("X (metre)")
  cbar = plt.colorbar(im2, ax=axes[2], ticks=[0, 1, 2], fraction=0.046)
  cbar.ax.set_yticklabels(["Düşük (<8°)", "Orta (8-15°)", "Yüksek (≥15°)"])

  # Eğer routes.json klasörde varsa rotaları 2D risk haritası üzerine çiz
  if parsed_paths:
    if "shortest_route" in parsed_paths:
      p_short = parsed_paths["shortest_route"]
      axes[2].plot(
          p_short[:, 1], p_short[:, 0], "b--", linewidth=2, label="Shortest Route"
      )
    if "risk_aware_route" in parsed_paths:
      p_safe = parsed_paths["risk_aware_route"]
      axes[2].plot(
          p_safe[:, 1], p_safe[:, 0], "w-", linewidth=2.5, label="Risk-Aware Route"
      )
    axes[2].legend(loc="upper right")

  plt.tight_layout()
  plt.savefig("risk_analysis_2d.png", dpi=200)
  plt.show()

  # 3D arayüz ekibi için kenar boşluksuz saf risk dokusu
  plt.imsave("risk_overlay.png", risk, cmap=cmap_risk, vmin=0, vmax=2)


if __name__ == "__main__":
  # 1. Örnek DEM üret ve kaydet
  dem_data = generate_sample_dem(size=200)
  np.save("sample_dem.npy", dem_data)

  # 2. Eğim ve risk matrislerini hesapla ve teslim dosyalarını kaydet
  slope_data, risk_data = calculate_slope_and_risk(dem_data, cell_size_m=1.0)
  np.save("slope.npy", slope_data)
  np.save("risk.npy", risk_data)

  print("Teslim dosyaları oluşturuldu: slope.npy, risk.npy")
  print(
      f"Matris Boyutu: {slope_data.shape} | Max Eğim: {slope_data.max():.2f}°"
  )
  print(
      f"Risk Oranları -> Düşük: {(risk_data==0).mean()*100:.1f}%, "
      f"Orta: {(risk_data==1).mean()*100:.1f}%, "
      f"Yüksek: {(risk_data==2).mean()*100:.1f}%"
  )

  # 3. Klasörde routes.json varsa A* testini otomatik çalıştır
  routes_to_plot = None
  if os.path.exists("routes.json"):
    routes_to_plot = test_astar_routes(slope_data, risk_data, "routes.json")
  else:
    print(
        "\nBilgi: 'routes.json' henüz klasörde yok. 4. sınıf EEE bu dosyayı"
        " atınca"
    )
    print(
        "aynı klasöre koyup kodu tekrar çalıştırdığında A* testi otomatik"
        " yapılacak."
    )

  # 4. 2D haritayı oluştur ve ekranda göster
  create_2d_visuals(dem_data, slope_data, risk_data, routes_to_plot)