# MarsEye — Gale Crater raw data

This directory stores the two original, georeferenced TIFF exports used for the MarsEye Gale Crater pilot area. Preserve the files without modifying their raster content.

## Files

- `topographic/1kmistenilenyer.tif`: elevation GeoTIFF, float32, 1004 columns x 1001 rows, 1 band, reported CRS `ESRI:104905`. The exact originating NASA/USGS elevation product identifier must still be documented.
- `optical/mosaic1km.tif`: optical grayscale mosaic GeoTIFF, uint8, 201 columns x 200 rows, 1 band, reported CRS `ESRI:104971`. Downloaded from NASA Mars Trek layer **MRO CTX, Gale Crater Mosaic** (Context Camera, Mars Reconnaissance Orbiter).

The rasters have nearly identical geographic bounding coordinates (longitude 137.5195377–137.5364758 E, latitude -4.7010447–-4.6841572), but **different CRS definitions and pixel grids**. Before using as a texture, confirm alignment, orientation and projection rather than mapping pixels blindly. The optical mosaic has an approximate 5 m export pixel spacing, not rock-by-rock spatial detail.

## Processed data

The Python route planners use `data/processed/elevation.npy`, `valid_mask.npy`, and `terrain_meta.json` (approximately 5 m grid), which are already committed separately. Do not substitute the optical raster as the elevation array.

## Source documentation

- NASA Mars Trek: https://trek.nasa.gov/mars/
- NASA MRO Context Camera: https://science.nasa.gov/mission/mars-reconnaissance-orbiter/

The identity and processing lineage of the original elevation product still need confirmation, and the user-supplied cropped exports may have been resampled.
