# MarsEye Gale — THEMIS infrared brightness temperatures

Original NASA PDS3 product: `I01350002PBT.IMG` (Mars Odyssey THEMIS, IR Band 9, 12.57 μm).

- Observation: 2002-04-04T17:11:57.369 UTC; Mars local time 03:15 (night)
- Original source grid: 3612 rows × 804 columns; 100 m/pixel; Sinusoidal Mars projection
- PBT units: Kelvin (K), **brightness temperature**
- Data set ID: ODY-M-THM-5-IRPBT-V2.0
- Official image explorer: https://viewer.mars.asu.edu/viewer/themis
- Product ID: I01350002

## MarsEye pilot region

Planetocentric latitude (north-positive): -4.701044656 to -4.684157180 degrees
East-positive longitude: 137.519537688 to 137.536475775 degrees

Clipped data: `data/processed/thermal/I01350002PBT_gale_100m.tif`

- Clip output: 11 rows × 11 columns, 100 valid ~100m pixels.
- Observed valid pixel min/mean/max: 188.988 K / 193.268 K / 196.032 K.
- No-data in clipped TIFF: -9999.
- The clipped TIFF preserves **100 m native resolution**, not the 198 × 200 (~5 m) navigation grid.
- The Mars PDS sinusoidal CRS is preserved. Do not overlay by pixel index on the DEM/CTX; reproject with the correct Mars CRS.

## Scientific caveats

- These are calibrated brightness temperatures, not direct contact measurements of soil or air temperatures.
- These correspond to one night-time observation; not a modern or 24-h thermal forecast.
- The native image has an overall temperature distribution; only the clipped pixels are relevant to the pilot.
- The pilot region is only about 1 km wide (~10 native thermal pixels across).
