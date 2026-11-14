import json
import heapq
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
N = 100
CELL_M = 2.0
START = (50, 5)    # row, col
GOAL = (50, 94)

# Synthetic test surface (NOT NASA data): Gaussian hill, height in meters.
y, x = np.mgrid[0:N, 0:N]
elevation = 24.0 * np.exp(-(((x - 50) / 18) ** 2 + ((y - 50) / 15) ** 2))
dz_dy, dz_dx = np.gradient(elevation, CELL_M, CELL_M)
slope = np.degrees(np.arctan(np.hypot(dz_dx, dz_dy)))

# Prototype thresholds, NOT NASA-certified EVA safety limits.
risk = np.ones((N, N), dtype=float)
risk[(slope >= 10) & (slope < 20)] = 3.0
risk[slope >= 20] = 10.0

MOVES = [(dr, dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]

def astar(weights, start, goal):
    rows, cols = weights.shape
    def valid(r, c):
        return 0 <= r < rows and 0 <= c < cols and np.isfinite(weights[r, c])
    if not valid(*start) or not valid(*goal):
        raise ValueError('Start/goal outside valid terrain')
    def h(node):
        return CELL_M * np.hypot(goal[0] - node[0], goal[1] - node[1])
    frontier = [(h(start), 0.0, start)]
    best = {start: 0.0}
    parent = {}
    while frontier:
        _, g, node = heapq.heappop(frontier)
        if g > best[node] + 1e-9:
            continue
        if node == goal:
            path = [node]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return path[::-1]
        for dr, dc in MOVES:
            nr, nc = node[0] + dr, node[1] + dc
            if not valid(nr, nc):
                continue
            if dr and dc and (not valid(node[0] + dr, node[1]) or not valid(node[0], node[1] + dc)):
                continue
            step_m = CELL_M * np.hypot(dr, dc)
            step_cost = step_m * (weights[node] + weights[nr, nc]) / 2.0
            ng = g + step_cost
            next_node = (nr, nc)
            if ng < best.get(next_node, float('inf')):
                best[next_node] = ng
                parent[next_node] = node
                heapq.heappush(frontier, (ng + h(next_node), ng, next_node))
    raise RuntimeError('No route found')

def stats(path):
    length = sum(CELL_M * np.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(path,path[1:]))
    return {'points_rc': [[int(r), int(c)] for r,c in path],
            'distance_m': round(float(length), 2),
            'max_slope_deg': round(float(max(slope[r,c] for r,c in path)), 2)}

shortest = astar(np.ones_like(risk), START, GOAL)
safer = astar(risk, START, GOAL)
result = {
    'data_type': 'SYNTHETIC_TEST_TERRAIN_NOT_NASA',
    'grid_shape': [N, N],
    'cell_size_m': CELL_M,
    'start_rc': list(START),
    'goal_rc': list(GOAL),
    'routes': {'shortest': stats(shortest), 'risk_aware': stats(safer)}
}
(OUT / 'routes.json').write_text(json.dumps(result, indent=2), encoding='utf-8')


# Export synthetic terrain for the 3D frontend
terrain_data = {
    "data_type": "SYNTHETIC_TEST_TERRAIN_NOT_NASA",
    "grid_shape": [N, N],
    "cell_size_m": CELL_M,
    "elevation_m": np.round(elevation, 2).tolist(),
    "slope_deg": np.round(slope, 2).tolist()
}

(OUT / "terrain.json").write_text(
    json.dumps(terrain_data),
    encoding="utf-8"
)

print("Terrain exported:", OUT / "terrain.json")


fig, ax = plt.subplots(figsize=(10, 7))
im = ax.imshow(slope, origin='upper', cmap='YlOrRd')
fig.colorbar(im, ax=ax, label='Slope (degrees)')
for path, color, label in [(shortest, 'dodgerblue', 'Shortest'), (safer, 'lime', 'Risk-aware')]:
    p = np.asarray(path)
    ax.plot(p[:,1], p[:,0], color=color, linewidth=2.6, label=label)
ax.scatter([START[1],GOAL[1]], [START[0],GOAL[0]], c='black', s=45, zorder=5)
ax.set_title('MarsEye A* Demo — synthetic terrain (NOT NASA data)')
ax.set_xlabel('Column'); ax.set_ylabel('Row'); ax.legend()
fig.tight_layout()
fig.savefig(OUT / 'routes_preview.png', dpi=160)
plt.close(fig)
print('Done:', OUT / 'routes.json', OUT / 'routes_preview.png')
print('Shortest:', result['routes']['shortest']['distance_m'], 'm')
print('Risk-aware:', result['routes']['risk_aware']['distance_m'], 'm')
