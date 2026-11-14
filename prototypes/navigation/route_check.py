
import json
from pathlib import Path
import numpy as np

folder = Path(__file__).resolve().parent

terrain = json.loads((folder / "terrain.json").read_text())
routes = json.loads((folder / "routes.json").read_text())

elevation = np.array(terrain["elevation_m"], dtype=float)
slope = np.array(terrain["slope_deg"], dtype=float)

cell_size = terrain["cell_size_m"]

for name, route in routes["routes"].items():
    points = np.array(route["points_rc"], dtype=int)

    rows, cols = points[:, 0], points[:, 1]
    heights = elevation[rows, cols]
    slopes = slope[rows, cols]

    steps = np.diff(points, axis=0) * cell_size
    distance = np.linalg.norm(steps, axis=1).sum()

    gain = np.maximum(np.diff(heights), 0).sum()

    print(f"\n{name.upper()}")
    print(f"Distance: {distance:.2f} m")
    print(f"Maximum slope: {np.max(slopes):.2f} deg")
    print(f"Mean slope: {np.mean(slopes):.2f} deg")
    print(f"Cells >= 20 deg: {np.sum(slopes >= 20)}")
    print(f"Elevation gain: {gain:.2f} m")


    # Save additional metrics to routes.json
    route["mean_slope_deg"] = round(float(np.mean(slopes)), 2)
    route["high_slope_cells"] = int(np.sum(slopes >= 20))
    route["elevation_gain_m"] = round(float(gain), 2)


(folder / "routes.json").write_text(
    json.dumps(routes, indent=2),
    encoding="utf-8"
)

print("\nRoute metrics saved to routes.json")
