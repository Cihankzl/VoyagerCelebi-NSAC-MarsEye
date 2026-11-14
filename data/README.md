# MarsEye Terrain Data — İlk Teslimat

Bu paket, HiRISE `DTEEC_040770_1755_039280_1755_A01` stereo DTM ürününden
Pahrump Hills, Gale Krateri için seçilen yaklaşık 1 × 1 km pilot alanı içerir.

## Dosya yapısı

```text
data/
└── processed/
    ├── gale_dem_1km.tif
    ├── elevation.npy
    ├── valid_mask.npy
    ├── terrain_meta.json
    └── elevation_preview.png
```

## Doğrulanmış veri sözleşmesi

- Grid boyutu: `500 × 500`
- Nominal yer örneklemesi: `2 m`
- Yükseklik tipi: little-endian `float32`
- Yükseklik birimi: metre
- Dikey referans: Mars 2000 / MOLA areoid
- Satır yönü: kuzeyden güneye
- Sütun yönü: batıdan doğuya
- Geçersiz değer: `elevation.npy` içinde `NaN`
- Maske tipi: Boolean; `true` geçerli hücre demektir
- CRS: Mars equirectangular, doğu-pozitif boylam, planetosentrik enlem

Seçilen alanın coğrafi sınırları:

```text
Kuzey: -4.6675646976°
Güney: -4.6844353024°
Batı:  137.3705365279°E
Doğu:  137.3874634721°E
Merkez: -4.676°, 137.379°E
```

Equirectangular projeksiyon nedeniyle GeoTIFF içindeki doğu-batı piksel aralığı
`2.006679 projeksiyon metresi` görünür. Merkez enlemindeki gerçek yüzey karşılığı
`2.000000 m`dir. Kuzey-güney aralığı `2.000000 m`dir. Bu ayrım
`terrain_meta.json` içinde açıkça kaydedilmiştir.

## A* kullanımı

```python
import numpy as np

elevation = np.load("data/processed/elevation.npy")
valid = np.load("data/processed/valid_mask.npy")

row = 100
column = 200

if valid[row, column]:
    print(elevation[row, column])
```

Düz komşu mesafesi yaklaşık `2 m`, çapraz komşu mesafesi yaklaşık
`2√2 m` alınabilir. `valid == false` veya `NaN` hücreler geçilemez kabul
edilmelidir.

## Risk/eğim hesabı

```python
import numpy as np

elevation = np.load("data/processed/elevation.npy")
valid = np.load("data/processed/valid_mask.npy")

dz_drow, dz_dcolumn = np.gradient(elevation, 2.0, 2.0)
slope_deg = np.degrees(np.arctan(np.hypot(dz_dcolumn, dz_drow)))
slope_deg[~valid] = np.nan
```

Geçilemez eğim sınırı araç/astronot hareket modeline göre ayrıca
belirlenmelidir; veri hazırlama aşamasında keyfî bir sınır uygulanmamıştır.

## Three.js eksen eşlemesi

Önerilen eşleme:

```text
Three.js +X = doğu
Three.js +Y = yukarı
Three.js +Z = güney
```

Bir hücrenin yaklaşık yerel konumu:

```js
const x = (column + 0.5) * 2.0;
const z = (row + 0.5) * 2.0;
const y = elevation - referenceElevation;
```

`referenceElevation`, sayısal hassasiyeti ve görüntüleme kolaylığını korumak için
minimum veya merkez yüksekliği seçilebilir. Mutlak MOLA yüksekliği veri dosyasında
korunmalıdır.

## Kaynak ve tekrar üretilebilirlik

- Kaynak ürün: `DTEEC_040770_1755_039280_1755_A01`
- Stereo çift: `ESP_040770_1755` + `ESP_039280_1755`
- Kaynak SHA-256:
  `143708C00B11765DA8DFE3ABA7CE894E7CC89314B3FD406DF9FEFB9FDA9CBA26`
- Kaynak bağlantısı:
  <https://www.uahirise.org/dtm/dtm.php?ID=ESP_040770_1755>
- Yeniden örnekleme: bilinear
- Çıktı CRS, sınırlar, dönüşüm matrisi ve istatistikler:
  `data/processed/terrain_meta.json`

Ham 303 MB IMG dosyası GitHub deposuna eklenmemelidir. İşlenmiş dosyalar ve bu
kaynak bağlantısı depoda tutulabilir.

## Kalan ikinci aşama

Optik HiRISE ortogörüntüsünün aynı sınırlara kırpılması ve DEM ile piksel bazında
hizalanması henüz bu ilk teslimata dahil değildir. Bu çalışma, A* ve risk modüllerini
bekletmeden ayrı bir texture teslimatı olarak yapılmalıdır.
