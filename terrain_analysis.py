import json
import numpy as np
import matplotlib.pyplot as plt

# 1. Sentetik arazi üretim bloğu yerine terrain.json dosyasını yükle
with open("terrain.json", "r", encoding="utf-8") as f:
    terrain_data = json.load(f)

dem = np.array(terrain_data["elevation_m"], dtype=float)
slope_deg = np.array(terrain_data["slope_deg"], dtype=float)
cell_size = float(terrain_data["cell_size_m"])  # 2.0 m

# (İsteğe bağlı) Önceki koddaki np.gradient hesabını doğrulama:
dy, dx = np.gradient(dem, cell_size)
calc_slope_deg = np.degrees(np.arctan(np.hypot(dx, dy)))
print(f"Hazır eğim ile np.gradient arası maks. fark: {np.max(np.abs(slope_deg - calc_slope_deg)):.4f}°")

# 2. Eğim eşiklerine göre risk haritası oluştur (0: Düşük, 1: Orta, 2: Yüksek)
LOW_RISK_MAX = 15.0
HIGH_RISK_MIN = 25.0

risk_map = np.zeros_like(slope_deg, dtype=int)
risk_map[(slope_deg >= LOW_RISK_MAX) & (slope_deg < HIGH_RISK_MIN)] = 1
risk_map[slope_deg >= HIGH_RISK_MIN] = 2

# 3. Rota doğrulama testi (Merkezdeki [50, 50] tepesinden geçen örnek rota)
test_route = [(r, 50) for r in range(10, 91)]  # (satır, sütun)
route_slopes = [slope_deg[r, c] for r, c in test_route]
max_route_slope = max(route_slopes)

status = "GÜVENSİZ (Yüksek Eğim Engeli)" if max_route_slope >= HIGH_RISK_MIN else "GÜVENLİ"
print(f"Rota Maksimum Eğimi: {max_route_slope:.2f}° -> Durum: {status}")

# 4. Yükseklik, Eğim ve Risk Haritalarını Görselleştir
fig, axes = plt.subplots(1, 3, figsize=(16, 5))

im0 = axes[0].imshow(dem, cmap="terrain", origin="lower")
axes[0].set_title("Yükseklik Haritası (m)")
plt.colorbar(im0, ax=axes[0], fraction=0.046)

im1 = axes[1].imshow(slope_deg, cmap="inferno", origin="lower")
axes[1].set_title("Eğim Haritası (Derece)")
plt.colorbar(im1, ax=axes[1], fraction=0.046)

im2 = axes[2].imshow(risk_map, cmap="RdYlGn_r", vmin=0, vmax=2, origin="lower")
axes[2].set_title("Risk Sınıflandırması (0: Düşük, 2: Yüksek)")
plt.colorbar(im2, ax=axes[2], ticks=[0, 1, 2], fraction=0.046)

# Test rotasını grafiklerin üzerine çiz
route_rows = [p[0] for p in test_route]
route_cols = [p[1] for p in test_route]
for ax in axes:
    ax.plot(route_cols, route_rows, color="cyan", linewidth=2, label="Test Rotası")
    ax.legend(loc="upper right")

plt.tight_layout()
plt.show()
