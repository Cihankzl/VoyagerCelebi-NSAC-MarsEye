// MarsEye — in-browser fallback planner.
// Mirrors prototypes/navigation/planner_v4.py (8-neighbour A*, no corner cutting,
// cell-slope + edge-grade limits, slope/roughness/ascent/thermal step cost) and
// v4/mission_planning/mission_planner.py (brute-force visit-order permutations).
// Used automatically when the FastAPI backend (port 8001) is not reachable, e.g.
// `python -m http.server 8000` only, or GitHub Pages. Output shape == POST /api/mission.

export const PRIMARY_ROUTES = {
  shortest:  { label: 'Shortest Route',       profile: 'astronaut', route_type: 'shortest_feasible', color: '#61c8ff' },
  astronaut: { label: 'Astronaut Safe Route', profile: 'astronaut', route_type: 'risk_aware',        color: '#ad86ff' },
  rover:     { label: 'Rover Optimal Route',  profile: 'rover',     route_type: 'risk_aware',        color: '#42d5bd' },
};
export const MAX_WAYPOINTS = 5;
const DEG = 180 / Math.PI;

class MinHeap {
  constructor() { this.f = []; this.g = []; this.n = []; }
  get size() { return this.n.length; }
  less(i, j) { return this.f[i] < this.f[j] || (this.f[i] === this.f[j] && (this.g[i] < this.g[j] || (this.g[i] === this.g[j] && this.n[i] < this.n[j]))); }
  swap(i, j) { for (const a of [this.f, this.g, this.n]) { const t = a[i]; a[i] = a[j]; a[j] = t; } }
  push(f, g, n) {
    this.f.push(f); this.g.push(g); this.n.push(n);
    let i = this.n.length - 1;
    while (i > 0) { const p = (i - 1) >> 1; if (!this.less(i, p)) break; this.swap(i, p); i = p; }
  }
  pop() {
    const out = [this.f[0], this.g[0], this.n[0]];
    const last = this.n.length - 1;
    if (last > 0) { this.f[0] = this.f[last]; this.g[0] = this.g[last]; this.n[0] = this.n[last]; }
    this.f.pop(); this.g.pop(); this.n.pop();
    let i = 0;
    for (;;) {
      const l = 2 * i + 1, r = l + 1; let m = i;
      if (l < this.n.length && this.less(l, m)) m = l;
      if (r < this.n.length && this.less(r, m)) m = r;
      if (m === i) break; this.swap(i, m); i = m;
    }
    return out;
  }
}

function* permutations(arr) {
  if (arr.length <= 1) { yield arr.slice(); return; }
  for (let i = 0; i < arr.length; i++) {
    const rest = arr.slice(0, i).concat(arr.slice(i + 1));
    for (const p of permutations(rest)) yield [arr[i], ...p];
  }
}

const r2 = (v, d = 3) => (v === null || !Number.isFinite(v)) ? null : Math.round(v * 10 ** d) / 10 ** d;

export class BrowserPlanner {
  constructor(terrain, thermal) {
    const [rows, cols] = terrain.grid_shape;
    Object.assign(this, { rows, cols, dx: terrain.dx_m, dy: terrain.dy_m, profiles: terrain.profiles });
    const N = rows * cols;
    this.z = new Float64Array(N); this.slope = new Float64Array(N); this.rough = new Float64Array(N);
    this.valid = new Uint8Array(N); this.temp = new Float64Array(N).fill(NaN); this.cold = null;
    for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
      const k = r * cols + c, z = terrain.elevation_m[r][c], s = terrain.slope_deg[r][c], g = terrain.roughness_m[r][c];
      this.z[k] = z ?? NaN; this.slope[k] = s ?? NaN; this.rough[k] = g ?? NaN;
      this.valid[k] = Number.isFinite(z) && Number.isFinite(s) && Number.isFinite(g) ? 1 : 0;
    }
    if (thermal?.temp_k) {
      let lo = Infinity, hi = -Infinity;
      for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
        const t = thermal.temp_k[r][c];
        if (Number.isFinite(t) && t > 0) { this.temp[r * cols + c] = t; lo = Math.min(lo, t); hi = Math.max(hi, t); }
      }
      if (hi > lo) {
        this.cold = new Float64Array(N);
        for (let k = 0; k < N; k++) this.cold[k] = Number.isFinite(this.temp[k]) ? Math.min(1, Math.max(0, (hi - this.temp[k]) / (hi - lo))) : 0.5;
      }
    }
    this.moves = [];
    for (const dr of [-1, 0, 1]) for (const dc of [-1, 0, 1]) if (dr || dc) this.moves.push([dr, dc, Math.hypot(dr * this.dy, dc * this.dx)]);
    this._allowed = {}; this._legs = new Map();
  }

  allowed(profileName) {
    if (!this._allowed[profileName]) {
      const lim = this.profiles[profileName].max_cell_slope_deg, a = new Uint8Array(this.valid.length);
      for (let k = 0; k < a.length; k++) a[k] = this.valid[k] && this.slope[k] < lim ? 1 : 0;
      this._allowed[profileName] = a;
    }
    return this._allowed[profileName];
  }

  stepCost(p, a, b, step) {
    const ascent = Math.max(0, this.z[b] - this.z[a]);
    const avgS = 0.5 * (this.slope[a] + this.slope[b]), avgR = 0.5 * (this.rough[a] + this.rough[b]);
    let pen = p.slope_weight * (avgS / 15) ** 2 + p.roughness_weight * avgR;
    if (this.cold) pen += (p.thermal_weight ?? 1) * 0.5 * (this.cold[a] + this.cold[b]);
    return step * (1 + pen) + p.ascent_weight * ascent;
  }

  astar(s, g, profileName, objective) {
    const { rows, cols } = this, allowed = this.allowed(profileName), p = this.profiles[profileName];
    if (!allowed[s] || !allowed[g]) return { status: 'blocked_endpoint', path: [] };
    if (s === g) return { status: 'found', path: [s] };
    const gr = Math.floor(g / cols), gc = g % cols, dx = this.dx, dy = this.dy;
    const h = (k) => Math.hypot((gr - Math.floor(k / cols)) * dy, (gc - k % cols) * dx);
    const best = new Float64Array(rows * cols).fill(Infinity), parent = new Int32Array(rows * cols).fill(-1);
    const heap = new MinHeap(); best[s] = 0; heap.push(h(s), 0, s);
    const lim = p.max_edge_grade_deg, useDist = objective === 'distance';
    while (heap.size) {
      const [, gc0, u] = heap.pop();
      if (gc0 > best[u] + 1e-9) continue;
      if (u === g) {
        const path = [u]; while (path[path.length - 1] !== s) path.push(parent[path[path.length - 1]]);
        return { status: 'found', path: path.reverse() };
      }
      const ur = Math.floor(u / cols), uc = u % cols;
      for (const [dr, dc, step] of this.moves) {
        const vr = ur + dr, vc = uc + dc;
        if (vr < 0 || vc < 0 || vr >= rows || vc >= cols) continue;
        const v = vr * cols + vc;
        if (!allowed[v]) continue;
        if (dr && dc && (!allowed[(ur + dr) * cols + uc] || !allowed[ur * cols + uc + dc])) continue;
        if (Math.atan2(Math.abs(this.z[v] - this.z[u]), step) * DEG >= lim) continue;
        const ng = gc0 + (useDist ? step : this.stepCost(p, u, v, step));
        if (ng + 1e-9 < best[v]) { best[v] = ng; parent[v] = u; heap.push(ng + h(v), ng, v); }
      }
    }
    return { status: 'no_path', path: [] };
  }

  summarize(points, profileName) {
    const p = this.profiles[profileName], cols = this.cols, idx = points.map(([r, c]) => r * cols + c);
    let dist = 0, gain = 0, loss = 0, cost = 0, maxGrade = 0, sS = 0, mS = 0, sR = 0, sCold = 0;
    let tN = 0, tS = 0, tMin = Infinity, tMax = -Infinity;
    idx.forEach((k, i) => {
      sS += this.slope[k]; mS = Math.max(mS, this.slope[k]); sR += this.rough[k];
      if (this.cold) sCold += this.cold[k];
      const t = this.temp[k]; if (Number.isFinite(t)) { tN++; tS += t; tMin = Math.min(tMin, t); tMax = Math.max(tMax, t); }
      if (i === 0) return;
      const a = idx[i - 1], [ar, ac] = points[i - 1], [br, bc] = points[i];
      const step = Math.hypot((br - ar) * this.dy, (bc - ac) * this.dx), dz = this.z[k] - this.z[a];
      dist += step; gain += Math.max(0, dz); loss += Math.max(0, -dz);
      maxGrade = Math.max(maxGrade, Math.atan2(Math.abs(dz), step) * DEG);
      cost += this.stepCost(p, a, k, step);
    });
    const n = idx.length || 1, tMean = tN ? tS / tN : null;
    return {
      distance_m: r2(dist), mean_slope_deg: r2(sS / n), max_slope_deg: r2(mS), max_edge_grade_deg: r2(maxGrade),
      mean_roughness_m: r2(sR / n, 4), elevation_gain_m: r2(gain), elevation_loss_m: r2(loss), weighted_cost_m_equiv: r2(cost),
      mean_temp_k: r2(tMean, 2), min_temp_k: tN ? r2(tMin, 2) : null, max_temp_k: tN ? r2(tMax, 2) : null,
      mean_temp_c: tN ? r2(tMean - 273.15, 2) : null, min_temp_c: tN ? r2(tMin - 273.15, 2) : null, max_temp_c: tN ? r2(tMax - 273.15, 2) : null,
      mean_cold_index_0_1: this.cold ? r2(sCold / n, 4) : null,
    };
  }

  leg(a, b, profileName, routeType) {
    const key = `${a}|${b}|${profileName}|${routeType}`;
    if (!this._legs.has(key)) {
      const s = a[0] * this.cols + a[1], g = b[0] * this.cols + b[1];
      const res = this.astar(s, g, profileName, routeType === 'shortest_feasible' ? 'distance' : 'risk');
      const leg = { status: res.status, points_rc: res.path.map((k) => [Math.floor(k / this.cols), k % this.cols]) };
      if (res.status === 'found') {
        const m = this.summarize(leg.points_rc, profileName);
        Object.assign(leg, m, { objective_cost: routeType === 'shortest_feasible' ? m.distance_m : m.weighted_cost_m_equiv });
      } else leg.objective_cost = null;
      this._legs.set(key, leg);
    }
    return this._legs.get(key);
  }

  _order(start, goal, wps, spec, order) {
    const coords = { A: start, B: goal }; wps.forEach((w) => { coords[w.id] = w.rc; });
    const seq = ['A', ...order, 'B'], legs = [];
    let obj = 0, dist = 0;
    for (let i = 0; i < seq.length - 1; i++) {
      const leg = this.leg(coords[seq[i]], coords[seq[i + 1]], spec.profile, spec.route_type);
      if (leg.status !== 'found') return { status: leg.status, visit_order: [], failed: [seq[i], seq[i + 1]] };
      legs.push({ from: seq[i], to: seq[i + 1], leg }); obj += leg.objective_cost; dist += leg.distance_m;
    }
    return { status: 'found', visit_order: seq, legs, objective_cost: obj, distance_m: dist };
  }

  planOne(key, start, goal, wps, orderMode) {
    const spec = PRIMARY_ROUTES[key];
    const base = { key, label: spec.label, color: spec.color, profile: spec.profile, route_type: spec.route_type,
      objective_unit: spec.route_type === 'shortest_feasible' ? 'm' : 'model_weighted_m_equiv' };
    const ids = wps.map((w) => w.id);
    const manual = this._order(start, goal, wps, spec, ids);
    let opt = null;
    for (const perm of permutations(ids)) {
      const r = this._order(start, goal, wps, spec, perm);
      if (r.status === 'found' && (!opt || opt.status !== 'found' || r.objective_cost < opt.objective_cost - 1e-9)) opt = r;
      else if (!opt) opt = r;
    }
    const chosen = orderMode === 'manual' ? manual : opt;
    const brief = (r) => ({ status: r.status, visit_order: r.visit_order, objective_cost: r.status === 'found' ? r2(r.objective_cost) : null, distance_m: r.status === 'found' ? r2(r.distance_m, 2) : null });
    let savings = null;
    if (ids.length && manual.status === 'found' && opt.status === 'found') {
      const saved = manual.objective_cost - opt.objective_cost;
      savings = { objective_saved: r2(saved, 2), objective_saved_pct: manual.objective_cost ? r2(100 * saved / manual.objective_cost, 2) : 0,
        distance_saved_m: r2(manual.distance_m - opt.distance_m, 2), manual_visit_order: manual.visit_order, optimized_visit_order: opt.visit_order };
    }
    if (chosen.status !== 'found') {
      return { ...base, status: chosen.status, reason: chosen.failed ? `${chosen.failed[0]} → ${chosen.failed[1]}: ${chosen.status}` : '', visit_order: [], points_rc: [], legs: [],
        manual: ids.length ? brief(manual) : null, optimized: ids.length ? brief(opt) : null, savings_vs_manual: savings };
    }
    const merged = []; const legs = []; let idx = 0;
    chosen.legs.forEach(({ from, to, leg }, i) => {
      const pts = leg.points_rc; merged.push(...(i === 0 ? pts : pts.slice(1)));
      const { points_rc, objective_cost, status, ...m } = leg;
      legs.push({ from, to, status, point_index_range: [idx, idx + pts.length - 1], ...m });
      idx += pts.length - 1;
    });
    return { ...base, status: 'found', visit_order: chosen.visit_order, points_rc: merged, ...this.summarize(merged, spec.profile),
      objective_cost: r2(chosen.objective_cost), legs, manual: ids.length ? brief(manual) : null, optimized: ids.length ? brief(opt) : null, savings_vs_manual: savings };
  }

  planMission(start, goal, waypoints = [], orderMode = 'optimized') {
    const t0 = performance.now();
    const wps = waypoints.slice(0, MAX_WAYPOINTS).map((w) => ({ id: w.id, rc: [w.rc[0], w.rc[1]] }));
    const routes = {};
    for (const key of Object.keys(PRIMARY_ROUTES)) routes[key] = this.planOne(key, start, goal, wps, orderMode);
    return { schema_version: '1.1', grid_shape: [this.rows, this.cols], dx_m: this.dx, dy_m: this.dy,
      coordinate_system: 'row_col_0_based_origin_upper_left', start_rc: start, goal_rc: goal, waypoints: wps, order_mode: orderMode,
      thermal_in_cost: !!this.cold, experimental: true, route_definitions: PRIMARY_ROUTES, routes,
      runtime_ms: Math.round(performance.now() - t0), engine: 'browser_js' };
  }
}
