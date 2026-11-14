
import rasterio
from rasterio.windows import Window

url = "BURAYA_DOGRUDAN_GEOTIFF_URL"

with rasterio.Env(
    GDAL_DISABLE_READDIR_ON_OPEN="EMPTY_DIR"
):
    with rasterio.open(url) as src:
        print("Format:", src.driver)
        print("Boyut:", src.width, "x", src.height)
        print("Koordinat sistemi:", src.crs)
        print("Piksel boyutu:", src.res)
        print("Sınırlar:", src.bounds)
        print("Veri tipi:", src.dtypes)
        print("NoData:", src.nodata)

        # Tam dosya yerine küçük örnek okuyalım
        window = Window(
            src.width // 2,
            src.height // 2,
            32,
            32
        )

        data = src.read(1, window=window)
        print("Test başarılı:", data.shape)
