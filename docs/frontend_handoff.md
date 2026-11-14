# MarsEye frontend handoff — 1st-year Computer Engineering

**This milestone is now ready for the frontend teammate.** The Python API has been smoke-tested with synthetic terrain. Integration with the team's real Three.js scene is a separate task; no frontend was modified by this starter package.

## What to implement now

1. Two map modes: `Select Start (A)` and `Select Goal (B)`.
2. On mesh raycast, convert the clicked terrain point to its original **0-based** `[row,col]` integer index, with row 0 at the raster's north/top edge. Do not send raw Three.js X/Y/Z coordinates.
3. Dropdown or toggle for `rover` / `astronaut`.
4. `Calculate Route` sends **one** request per profile selection. It receives **both** route types.
5. Display `shortest_feasible` and `risk_aware` simultaneously with distinct colors; user can toggle each.
6. Display metrics in the result panel; show `blocked_endpoint`/`no_path` as a warning rather than drawing an incorrect route.
7. `unconstrained_reference` is optional, hidden by default, labeled `Reference only — may violate constraints`.

Example request (for integration testing, not a finished frontend):

```javascript
const response = await fetch('http://127.0.0.1:8001/api/route', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    start_rc: [99, 20],
    goal_rc: [99, 180],
    profile: 'rover',
    include_reference: false
  })
});
if (!response.ok) throw new Error(await response.text());
const result = await response.json();
const shortest = result.routes.shortest_feasible;
const lowerRisk = result.routes.risk_aware;
// Only render a route when its status === 'found'.
// Convert each route point [row,col] into the actual Three.js terrain mesh coordinates.
```

Before connecting to API, verify indices on four DEM corners and at least one known interior cell against the actual mesh vertices. Use `GET /api/terrain/metadata` for grid spacing and dimensions. Never flip the north/south direction without this test.

**Blocked:** optical texture alignment (other 4th-year EEE); validated external risk layers (2nd-year EEE); thermal layer (1st-year EEE). These are independent of the A/B routing milestone and do not need to delay it.
