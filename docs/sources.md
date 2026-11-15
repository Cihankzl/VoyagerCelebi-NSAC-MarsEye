# Scientific data and provenance

| Layer | Local source | Instrument / meaning | Source resolution | Status |
| --- | --- | --- | --- | --- |
| Topographic | `data/raw/topographic/1kmistenilenyer.tif` | User-provided Mars elevation GeoTIFF, metres | Original 1001 × 1004 grid; processed navigation grid 198 × 200 at 5.0033693 × 4.9968054 m | Exact source product ID unverified |
| Optical | `data/raw/optical/mosaic1km.tif` | MRO CTX grayscale observation | Source mosaic 200 × 201 pixels; metadata records geographic pixel size | Exact source product ID unverified |
| Thermal | `data/raw/thermal/I01350002PBT.IMG` | Mars Odyssey THEMIS Band 9 projected brightness temperature, Kelvin | Approx. 100 m/pixel; local clipped GeoTIFF 11 × 11 pixels | Product ID `I01350002PBT` in repository metadata |

The map is a selected ~1 × 1 km pilot area centered near 137.5280067° E, −4.6926009°. Gale Crater itself is roughly 154 km across; the pilot area is not the whole crater. The processed DEM range is −3954.3018 to −3623.204 m. The thermal observation metadata records a nighttime Mars local time of ~03:15. A displayed 198 × 200 thermal texture repeats/resamples source pixels for visualization; it does not provide 5 m thermal measurements.

`scripts/prepare_web_data.py` uses the DEM's Mars 2000 affine grid as the display reference, transforms the CTX and THEMIS rasters through their embedded CRS, uses bilinear resampling for optical display and nearest-neighbor resampling for thermal display, and preserves missing-data transparency. It writes browser assets under `frontend/marseye_app/public/data/` and aligned numeric samples under `data/processed/science_samples.npz`. It does not modify raw rasters. A hash of the DEM, mask, optical and thermal sources, and metadata must match between the API and browser assets.

The V4 route engine derives slope and detrended roughness from the DEM's true metre spacing. Visual vertical exaggeration is excluded from route costs. CTX image brightness is not a rock or soil-strength classification. THEMIS brightness temperature is not certified ambient or contact temperature. No external validated risk map is supplied; the route cost currently uses the DEM slope/roughness proxy. Risk probability is therefore unavailable.

## Context references

- [NASA: Context of Curiosity Landing Site in Gale Crater](https://science.nasa.gov/resource/context-of-curiosity-landing-site-in-gale-crater/) — Gale's approximate diameter, layered mound, and Mars Science Laboratory context.
- [USGS: Gazetteer of Planetary Nomenclature, Gale](https://planetarynames.wr.usgs.gov/Feature/2071) — recognized crater name and global feature information.
- [NASA: Context Camera image near Curiosity's landing](https://science.nasa.gov/resource/before-and-after-curiositys-touchdown/) — MRO CTX instrument context; this page does not identify the local mosaic's product ID.
- [ASU: THEMIS FAQ](https://themis.asu.edu/faq) — infrared bands and nominal 100 m/pixel IR resolution.

These references support instrument and regional context. They do not prove the exact DEM or CTX product identity; those identifiers remain explicitly unverified.
