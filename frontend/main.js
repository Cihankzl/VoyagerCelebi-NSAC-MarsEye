// MarsEye Prototype V1 — Gale Crater 3D God's Eye mission planner.
// Serve the REPOSITORY ROOT:  python -m http.server 8000  ->  http://localhost:8000/frontend/
// Backend (optional):         python prototypes/navigation/route_api.py   (http://127.0.0.1:8001)
// Without the backend every route is computed by ./planner.js in the browser.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { BrowserPlanner, PRIMARY_ROUTES, MAX_WAYPOINTS } from './planner.js';
import { ProfileChart, slopeClass } from './chart.js';

const $ = (s) => document.querySelector(s);
const $$ = (s) => [...document.querySelectorAll(s)];
const params = new URLSearchParams(location.search);
const API = params.get('api') || 'http://127.0.0.1:8001';
const DATA = '../data/processed/';
const ROUTE_KEYS = ['astronaut', 'rover', 'shortest'];
// Set to true once data/raw/optical/mosaic1km.tif is re-clipped at the DEM's true location
// (planetocentric lat -4.646110 .. -4.629418, lon 137.519538 .. 137.536476).
const OPTICAL_ALIGNED = true;    // DEM and CTX image currently show different terrain (cause under investigation)
const PRESET = { start: [100, 22], goal: [15, 181], waypoints: [[45, 100], [150, 35], [75, 75]] };
const fmt = (v, d = 1) => (v === null || v === undefined || !Number.isFinite(+v)) ? '—' : (+v).toLocaleString('en-US', { minimumFractionDigits: d, maximumFractionDigits: d });
const signed = (v, d = 1) => (v >= 0 ? '+' : '−') + fmt(Math.abs(v), d);

function fatal(msg) { const el = $('#fatal'); el.hidden = false; el.textContent = msg; }
let toastTimer;
function toast(msg, ms = 2600) { const t = $('#toast'); t.textContent = msg; t.classList.add('show'); clearTimeout(toastTimer); toastTimer = setTimeout(() => t.classList.remove('show'), ms); }

// ------------------------------------------------------------------ data
async function getJSON(path) {
  const r = await fetch(path);
  if (!r.ok) throw new Error(`${path}: HTTP ${r.status}`);
  return r.json();
}
let terrain, thermal, staticMission;
try {
  [terrain, thermal, staticMission] = await Promise.all([
    getJSON(DATA + 'terrain.json'),
    getJSON(DATA + 'thermal/thermal_grid.json'),
    getJSON(DATA + 'mission_route.json').catch(() => null),
  ]);
} catch (e) {
  fatal(`Could not load MarsEye data (${e.message}).\n\n1) Run once:  python scripts/prepare_web_assets.py\n2) Serve the REPOSITORY ROOT:  python -m http.server 8000\n3) Open  http://localhost:8000/frontend/`);
  throw e;
}

const [rows, cols] = terrain.grid_shape;
const dx = terrain.dx_m, dy = terrain.dy_m;
const elev = terrain.elevation_m, slopes = terrain.slope_deg, rough = terrain.roughness_m, temps = thermal.temp_k;
let base = Infinity, top = -Infinity;
for (const row of elev) for (const z of row) if (Number.isFinite(z)) { base = Math.min(base, z); top = Math.max(top, z); }
const relief = top - base, width = (cols - 1) * dx, depth = (rows - 1) * dy;
const zAt = (r, c) => (Number.isFinite(elev[r]?.[c]) ? elev[r][c] : base);
const planner = new BrowserPlanner(terrain, thermal);

// Same mapping as frontend/marseye_v3_demo/main.js: X = c*dx (east), Y = (z-base)*vScale (up), Z = r*dy (south)
const defaultVScale = Math.min(2.5, Math.max(1, Math.max(width, depth) / (Math.max(relief, 1) * 2.9)));
let vScale = defaultVScale;
const world = (r, c, offset = 0) => new THREE.Vector3(c * dx, (zAt(r, c) - base) * vScale + offset, r * dy);
function bilinear(grid, rf, cf) {
  const r0 = Math.max(0, Math.min(rows - 1, Math.floor(rf))), c0 = Math.max(0, Math.min(cols - 1, Math.floor(cf)));
  const r1 = Math.min(rows - 1, r0 + 1), c1 = Math.min(cols - 1, c0 + 1), tr = Math.min(1, Math.max(0, rf - r0)), tc = Math.min(1, Math.max(0, cf - c0));
  const g = (r, c) => (Number.isFinite(grid[r]?.[c]) ? grid[r][c] : NaN);
  return g(r0, c0) * (1 - tr) * (1 - tc) + g(r0, c1) * (1 - tr) * tc + g(r1, c0) * tr * (1 - tc) + g(r1, c1) * tr * tc;
}

// ------------------------------------------------------------------ scene
const viewport = $('#viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
viewport.prepend(renderer.domElement);
const scene = new THREE.Scene();
scene.fog = new THREE.Fog('#0a0f18', 2600, 5200);
const camera = new THREE.PerspectiveCamera(45, 1, 1, 12000);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true; controls.dampingFactor = 0.12;
controls.maxPolarAngle = THREE.MathUtils.degToRad(78);   // never edge-on / under the ground
controls.minDistance = 25; controls.maxDistance = Math.max(width, depth) * 3.2;
controls.zoomToCursor = true;                            // wheel zooms towards the mouse pointer
controls.screenSpacePanning = true;
controls.rotateSpeed = 0.7; controls.panSpeed = 0.9; controls.zoomSpeed = 1.1;
// Surface height (world Y) under any world x/z, so the orbit pivot always sits ON the terrain.
function surfaceY(x, z) {
  const c = Math.min(cols - 1, Math.max(0, x / dx)), r = Math.min(rows - 1, Math.max(0, z / dy));
  const r0 = Math.floor(r), c0 = Math.floor(c), r1 = Math.min(rows - 1, r0 + 1), c1 = Math.min(cols - 1, c0 + 1), tr = r - r0, tc = c - c0;
  const zz = zAt(r0, c0) * (1 - tr) * (1 - tc) + zAt(r0, c1) * (1 - tr) * tc + zAt(r1, c0) * tr * (1 - tc) + zAt(r1, c1) * tr * tc;
  return (zz - base) * vScale;
}
// Frame the whole 1 km block for the current canvas aspect. kind: 'oblique' | 'top'
function fitView(kind = 'oblique') {
  const span = Math.hypot(width, depth) * 0.47;
  const vfov = THREE.MathUtils.degToRad(camera.fov);
  const hfov = 2 * Math.atan(Math.tan(vfov / 2) * camera.aspect);
  const dist = span / Math.tan(Math.min(vfov, hfov) / 2) * 1.02;
  const tgt = new THREE.Vector3(width / 2, surfaceY(width / 2, depth / 2), depth / 2);
  const polar = kind === 'top' ? 0.001 : THREE.MathUtils.degToRad(52), az = kind === 'top' ? 0 : THREE.MathUtils.degToRad(-28);
  controls.target.copy(tgt);
  camera.position.set(tgt.x + dist * Math.sin(polar) * Math.sin(az), tgt.y + dist * Math.cos(polar), tgt.z + dist * Math.sin(polar) * Math.cos(az));
  camera.up.set(0, 1, 0);
  controls.update();
}
// Keep the pivot inside the 1 km block and on the surface; keep the camera above the ground.
controls.addEventListener('change', () => {
  const t = controls.target;
  const cx = Math.min(width, Math.max(0, t.x)), cz = Math.min(depth, Math.max(0, t.z));
  if (cx !== t.x || cz !== t.z) { camera.position.x += cx - t.x; camera.position.z += cz - t.z; t.x = cx; t.z = cz; }
  const ground = surfaceY(camera.position.x, camera.position.z) + 12;
  if (camera.position.y < ground) camera.position.y = ground;
});
controls.update();
scene.add(new THREE.HemisphereLight('#dbe8ff', '#3a2416', 1.25));
const sun = new THREE.DirectionalLight('#fff1dc', 2.4);
sun.position.set(-width * 0.6, Math.max(width, depth) * 1.1, -depth * 0.4);  // low NW sun -> hillshade
sun.target.position.set(width / 2, 0, depth / 2);
scene.add(sun, sun.target);

// stars
{
  const g = new THREE.BufferGeometry(), n = 1200, p = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) { const th = Math.random() * Math.PI * 2, ph = Math.random() * Math.PI * 0.45, R = 6000; p.set([width / 2 + R * Math.sin(ph) * Math.cos(th), R * Math.cos(ph), depth / 2 + R * Math.sin(ph) * Math.sin(th)], i * 3); }
  g.setAttribute('position', new THREE.BufferAttribute(p, 3));
  scene.add(new THREE.Points(g, new THREE.PointsMaterial({ color: '#9fb5d6', size: 6, sizeAttenuation: true, fog: false })));
}

// ------------------------------------------------------------------ terrain mesh
const N = rows * cols;
const positions = new Float32Array(N * 3), colors = new Float32Array(N * 3), uvs = new Float32Array(N * 2);
const indices = [];
const ok = (r, c) => Number.isFinite(elev[r][c]);
for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
  const k = r * cols + c;
  // Exact geo UVs: DEM cell centres sit half a cell inside the shared bounding box that the
  // optical and thermal textures cover edge-to-edge. Row 0 (north) = top of image.
  uvs[2 * k] = (c + 0.5) / cols; uvs[2 * k + 1] = 1 - (r + 0.5) / rows;
  if (r < rows - 1 && c < cols - 1) {
    const a = k, b = k + 1, d = k + cols, e = d + 1;
    if (ok(r, c) && ok(r + 1, c) && ok(r, c + 1)) indices.push(a, d, b);
    if (ok(r, c + 1) && ok(r + 1, c) && ok(r + 1, c + 1)) indices.push(b, d, e);
  }
}
const geometry = new THREE.BufferGeometry();
geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
geometry.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
geometry.setIndex(indices);
function updateHeights() {
  for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
    const k = r * cols + c;
    positions[3 * k] = c * dx; positions[3 * k + 1] = (zAt(r, c) - base) * vScale; positions[3 * k + 2] = r * dy;
  }
  geometry.attributes.position.needsUpdate = true;
  geometry.computeVertexNormals();
  geometry.computeBoundingSphere(); geometry.computeBoundingBox();
}
updateHeights();

// Side walls ("skirt") so the 1 km block reads as a solid 3D slab; follows vertical exaggeration.
const SKIRT_FLOOR = -25;
const edge = [];
for (let c = 0; c < cols; c++) edge.push([0, c]);
for (let r = 1; r < rows; r++) edge.push([r, cols - 1]);
for (let c = cols - 2; c >= 0; c--) edge.push([rows - 1, c]);
for (let r = rows - 2; r >= 0; r--) edge.push([r, 0]);
const skirtPos = new Float32Array(edge.length * 2 * 3), skirtIdx = [];
for (let i = 0; i < edge.length; i++) { const j = (i + 1) % edge.length; skirtIdx.push(2 * i, 2 * i + 1, 2 * j, 2 * j, 2 * i + 1, 2 * j + 1); }
const skirtGeo = new THREE.BufferGeometry();
skirtGeo.setAttribute('position', new THREE.BufferAttribute(skirtPos, 3)); skirtGeo.setIndex(skirtIdx);
const skirt = new THREE.Mesh(skirtGeo, new THREE.MeshStandardMaterial({ color: '#5a4033', roughness: 1, side: THREE.DoubleSide }));
const floor = new THREE.Mesh(new THREE.PlaneGeometry(width, depth), new THREE.MeshStandardMaterial({ color: '#1a1412', side: THREE.DoubleSide }));
floor.rotation.x = -Math.PI / 2; floor.position.set(width / 2, SKIRT_FLOOR, depth / 2);
scene.add(skirt, floor);
function updateSkirt() {
  edge.forEach(([r, c], i) => {
    skirtPos.set([c * dx, (zAt(r, c) - base) * vScale, r * dy, c * dx, SKIRT_FLOOR, r * dy], i * 6);
  });
  skirtGeo.attributes.position.needsUpdate = true; skirtGeo.computeVertexNormals(); skirtGeo.computeBoundingSphere();
}
updateSkirt();

// colour ramps
const lerpStops = (stops, t) => {
  t = Math.min(1, Math.max(0, t));
  for (let i = 0; i < stops.length - 1; i++) if (t <= stops[i + 1][0]) {
    const [t0, c0] = stops[i], [t1, c1] = stops[i + 1];
    return new THREE.Color(c0).lerp(new THREE.Color(c1), (t - t0) / (t1 - t0 || 1));
  }
  return new THREE.Color(stops[stops.length - 1][1]);
};
const HYPSO = [[0, '#2b1a1e'], [0.2, '#5a2a1f'], [0.45, '#9b4a2a'], [0.7, '#d4874d'], [0.88, '#e9bc84'], [1, '#f6e2c0']];
const ROUGH = [[0, '#2e4a6b'], [0.35, '#4f8f8a'], [0.65, '#e3c35a'], [1, '#e2493a']];
const ROUGH_MAX = 0.5;
const slopeColor = (s) => !Number.isFinite(s) ? new THREE.Color('#444') : s < 10 ? new THREE.Color('#2fae5f').lerp(new THREE.Color('#7fcf4f'), s / 10)
  : s < 20 ? new THREE.Color('#f2b13a').lerp(new THREE.Color('#ee7a2a'), (s - 10) / 10) : s < 25 ? new THREE.Color('#e5412f') : new THREE.Color('#9e1730');
function paintTopo(mode) {
  const bad = new THREE.Color('#1c222b');
  for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
    let col;
    if (!ok(r, c)) col = bad;
    else if (mode === 'slope') col = slopeColor(slopes[r][c]);
    else if (mode === 'roughness') col = Number.isFinite(rough[r][c]) ? lerpStops(ROUGH, rough[r][c] / ROUGH_MAX) : bad;
    else col = lerpStops(HYPSO, (elev[r][c] - base) / relief);
    colors.set([col.r, col.g, col.b], 3 * (r * cols + c));
  }
  geometry.attributes.color.needsUpdate = true;
}

const loader = new THREE.TextureLoader();
const loadTex = (url) => new Promise((res) => loader.load(url, (t) => { t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = renderer.capabilities.getMaxAnisotropy(); res(t); }, undefined, () => res(null)));
const [texOptical, texThermalNative, texThermalSmooth, texThermalDay] = await Promise.all([
  loadTex(DATA + 'optical_mosaic.png'), loadTex(DATA + 'thermal/thermal_100m.png'), loadTex(DATA + 'thermal/thermal_smooth.png'),
  loadTex(DATA + 'thermal/themis_day_ir.png'),
]);
if (!texThermalDay) document.querySelector('[data-thermal="day"]')?.remove();
if (texThermalNative) { texThermalNative.magFilter = THREE.NearestFilter; texThermalNative.minFilter = THREE.LinearFilter; texThermalNative.generateMipmaps = false; }
const matTopo = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.95, metalness: 0, side: THREE.DoubleSide });
const matOptical = new THREE.MeshStandardMaterial({ map: texOptical, color: '#f1e3d3', roughness: 1, metalness: 0, side: THREE.DoubleSide });
const matThermal = new THREE.MeshStandardMaterial({ map: texThermalNative, emissiveMap: texThermalNative, emissive: '#ffffff', emissiveIntensity: 0.45, roughness: 1, side: THREE.DoubleSide });
const terrainMesh = new THREE.Mesh(geometry, matTopo);
scene.add(terrainMesh);

// ------------------------------------------------------------------ state
const state = {
  view: 'topo', topo: 'elevation', thermalMode: 'native', mode: 'navigate',
  start: null, goal: null, waypoints: [], orderMode: 'optimized',
  active: 'rover', visible: { shortest: true, astronaut: true, rover: true },
  result: null, source: '', progress: 0, playing: false, speed: 1, backend: false,
};

// ------------------------------------------------------------------ labels / markers
function labelSprite(text, bg, fg = '#fff') {
  const cv = document.createElement('canvas'); cv.width = 128; cv.height = 64;
  const g = cv.getContext('2d');
  g.fillStyle = bg; g.strokeStyle = '#ffffffcc'; g.lineWidth = 4;
  g.beginPath(); g.roundRect(8, 8, 112, 48, 14); g.fill(); g.stroke();
  g.fillStyle = fg; g.font = 'bold 30px Inter, system-ui, sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(text, 64, 33);
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(cv), depthTest: false, transparent: true }));
  sp.scale.set(36, 18, 1); sp.renderOrder = 20;
  return sp;
}
function beacon(kind, text) {
  const color = kind === 'A' ? '#2fe07a' : kind === 'B' ? '#ffc23a' : '#ff8a3a';
  const g = new THREE.Group();
  const poleH = kind === 'S' ? 22 : 34;
  const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.7, 0.7, poleH, 8), new THREE.MeshBasicMaterial({ color }));
  pole.position.y = poleH / 2;
  const head = new THREE.Mesh(kind === 'S' ? new THREE.ConeGeometry(3.4, 7, 16) : new THREE.SphereGeometry(4.2, 20, 14), new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.8 }));
  head.position.y = poleH + 2; if (kind === 'S') head.rotation.x = Math.PI;
  const ring = new THREE.Mesh(new THREE.RingGeometry(5, 7.5, 40), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.85, side: THREE.DoubleSide, depthWrite: false }));
  ring.rotation.x = -Math.PI / 2; ring.position.y = 0.8;
  const lbl = labelSprite(text, kind === 'A' ? '#1f8f4e' : kind === 'B' ? '#c18a10' : (kind === 'S' ? '#c45a14' : '#333'));
  lbl.position.y = poleH + 16;
  g.add(pole, head, ring, lbl);
  g.userData = { ring };
  return g;
}
const markerGroup = new THREE.Group(); scene.add(markerGroup);
function placeAt(obj, rc) { obj.position.copy(world(rc[0], rc[1], 0)); obj.userData.rc = rc; }
function rebuildMarkers() {
  markerGroup.clear();
  if (state.start) { const b = beacon('A', 'A'); placeAt(b, state.start); markerGroup.add(b); }
  if (state.goal) { const b = beacon('B', 'B'); placeAt(b, state.goal); markerGroup.add(b); }
  state.waypoints.forEach((w) => { const b = beacon('S', w.id); placeAt(b, w.rc); markerGroup.add(b); });
}

// ------------------------------------------------------------------ route tubes
const routeGroup = new THREE.Group(); scene.add(routeGroup);
const ROUTE_LIFT = { shortest: 2.6, rover: 3.3, astronaut: 4.0 };
function rebuildRoutes() {
  routeGroup.traverse((o) => { o.geometry?.dispose?.(); });
  routeGroup.clear();
  const res = state.result; if (!res) return;
  for (const key of ROUTE_KEYS) {
    const r = res.routes[key];
    if (r?.status !== 'found' || r.points_rc.length < 2 || !state.visible[key]) continue;
    const pts = r.points_rc.map(([rr, cc]) => world(rr, cc, ROUTE_LIFT[key]));
    const curve = new THREE.CatmullRomCurve3(pts, false, 'centripetal', 0.2);
    const active = key === state.active;
    const tube = new THREE.Mesh(new THREE.TubeGeometry(curve, Math.min(3000, pts.length * 3), active ? 2.3 : 1.35, 10, false),
      new THREE.MeshStandardMaterial({ color: PRIMARY_ROUTES[key].color, emissive: PRIMARY_ROUTES[key].color, emissiveIntensity: active ? 0.75 : 0.35, transparent: !active, opacity: active ? 1 : 0.72, roughness: 0.4 }));
    tube.renderOrder = active ? 6 : 5;
    routeGroup.add(tube);
  }
}

// ------------------------------------------------------------------ moving marker
const mover = new THREE.Group();
const moverCore = new THREE.Mesh(new THREE.SphereGeometry(4.6, 24, 16), new THREE.MeshStandardMaterial({ color: '#ffffff', emissive: '#42d5bd', emissiveIntensity: 1.6 }));
const moverGlow = new THREE.Mesh(new THREE.SphereGeometry(8, 24, 16), new THREE.MeshBasicMaterial({ color: '#42d5bd', transparent: true, opacity: 0.22, depthWrite: false }));
const moverBeam = new THREE.Mesh(new THREE.CylinderGeometry(0.5, 0.5, 40, 8), new THREE.MeshBasicMaterial({ color: '#42d5bd', transparent: true, opacity: 0.45, depthWrite: false }));
moverBeam.position.y = 20;
moverCore.position.y = 7; moverGlow.position.y = 7;
const halo = new THREE.Mesh(new THREE.RingGeometry(6, 9, 48), new THREE.MeshBasicMaterial({ color: '#42d5bd', transparent: true, opacity: 0.8, side: THREE.DoubleSide, depthWrite: false }));
halo.rotation.x = -Math.PI / 2; halo.position.y = 1;
const moverLight = new THREE.PointLight('#42d5bd', 4000, 120, 2); moverLight.position.y = 12;
mover.add(moverCore, moverGlow, moverBeam, halo, moverLight);
mover.visible = false;
scene.add(mover);

// ------------------------------------------------------------------ track (active route)
let track = null;   // {pts, cum, z, slope, total, marks, color}
function buildTrack() {
  const r = state.result?.routes?.[state.active];
  if (r?.status !== 'found' || r.points_rc.length < 2) { track = null; mover.visible = false; chart.setTrack(null); updateHUD(); return; }
  const pts = r.points_rc, cum = [0], z = [], sl = [];
  for (let i = 0; i < pts.length; i++) {
    z.push(zAt(pts[i][0], pts[i][1])); sl.push(slopes[pts[i][0]][pts[i][1]]);
    if (i) cum.push(cum[i - 1] + Math.hypot((pts[i][0] - pts[i - 1][0]) * dy, (pts[i][1] - pts[i - 1][1]) * dx));
  }
  const marks = [{ id: 'A', d: 0 }];
  (r.legs || []).forEach((leg) => marks.push({ id: leg.to, d: cum[leg.point_index_range?.[1] ?? pts.length - 1] }));
  if (marks.length === 1) marks.push({ id: 'B', d: cum[cum.length - 1] });
  track = { pts, cum, z, slope: sl, total: cum[cum.length - 1], marks, color: PRIMARY_ROUTES[state.active].color, startZ: z[0] };
  const col = new THREE.Color(track.color);
  for (const m of [moverCore.material]) m.emissive = col;
  moverGlow.material.color = col; moverBeam.material.color = col; halo.material.color = col; moverLight.color = col;
  mover.visible = true;
  chart.setTrack(track);
  setProgress(state.progress);
}
function sampleTrack(f) {
  const t = track, d = Math.min(1, Math.max(0, f)) * t.total;
  let lo = 0, hi = t.cum.length - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (t.cum[m] <= d) lo = m; else hi = m; }
  const seg = t.cum[hi] - t.cum[lo] || 1, u = Math.min(1, Math.max(0, (d - t.cum[lo]) / seg));
  const [r0, c0] = t.pts[lo], [r1, c1] = t.pts[hi];
  const rf = r0 + (r1 - r0) * u, cf = c0 + (c1 - c0) * u;
  const zz = t.z[lo] + (t.z[hi] - t.z[lo]) * u;
  const nearest = u < 0.5 ? lo : hi;
  let leg = null; for (const m of t.marks) if (m.d > d + 1e-6) { leg = m.id; break; }
  return { d, rf, cf, z: zz, slope: t.slope[nearest], temp: bilinear(temps, rf, cf), leg };
}
function setProgress(f) {
  state.progress = Math.min(1, Math.max(0, f));
  if (!scrubbing) $('#scrubber').value = Math.round(state.progress * 1000);
  $('#scrubPct').textContent = `${Math.round(state.progress * 100)}%`;
  chart.setProgress(state.progress);
  if (!track) return;
  const s = sampleTrack(state.progress);
  mover.position.set(s.cf * dx, (s.z - base) * vScale, s.rf * dy);
  updateHUD(s);
}

// ------------------------------------------------------------------ HUD
$('#altMax').textContent = fmt(top, 0); $('#altMin').textContent = fmt(base, 0);
function updateHUD(s) {
  $('#hudProfile').textContent = { rover: 'Rover', astronaut: 'Astronaut', shortest: 'Shortest' }[state.active];
  if (!s || !track) {
    for (const id of ['#tElev', '#tClimb', '#tSlope', '#tTemp', '#tProg']) $(id).textContent = '—';
    $('#hudLeg').textContent = ''; $('#altNeedle').style.bottom = '0%'; $('#altFill').style.height = '0%';
    return;
  }
  const pct = ((s.z - base) / relief) * 100;
  $('#altNeedle').style.bottom = `${pct}%`; $('#altFill').style.height = `${pct}%`;
  $('#tElev').textContent = `${fmt(s.z, 1)} m`;
  $('#tClimb').textContent = `${signed(s.z - track.startZ, 1)} m`;
  const cls = slopeClass(s.slope);
  $('#tSlope').innerHTML = `<span class="lvl-${cls}">${fmt(s.slope, 1)}°</span>`;
  $('#tTemp').textContent = Number.isFinite(s.temp) ? `${fmt(s.temp, 1)} K (${fmt(s.temp - 273.15, 1)} °C)` : '—';
  $('#tProg').textContent = `${fmt(s.d, 0)} m / ${fmt(track.total, 0)} m`;
  $('#hudLeg').textContent = s.leg ? `→ ${s.leg}` : 'arrived ✓';
}

// ------------------------------------------------------------------ chart
const chart = new ProfileChart($('#profile'), { onSeek: (f) => { state.playing = false; syncPlay(); setProgress(f); } });

// ------------------------------------------------------------------ views
function legendTopo() {
  const el = $('#legendTopo');
  if (state.view === 'optical') {
    el.hidden = false;
    el.innerHTML = `<b>MRO CTX optical mosaic · ~5 m/px</b>` + (OPTICAL_ALIGNED ? '' :
      `<div class="row warning">⚠ Preview only: this photo does not match the DEM relief yet (georeference check pending with the data team).</div>`);
    return;
  }
  if (state.view !== 'topo') { el.hidden = true; return; }
  el.hidden = false;
  const ramp = (stops) => `linear-gradient(90deg, ${stops.map(([t, c]) => `${c} ${t * 100}%`).join(',')})`;
  if (state.topo === 'slope') el.innerHTML = `<b>Slope risk</b>
    <div class="row"><i style="background:#2fae5f"></i>&lt; 10° low</div><div class="row"><i style="background:#f2b13a"></i>10°–20° caution</div>
    <div class="row"><i style="background:#e5412f"></i>20°–25° rover limit</div><div class="row"><i style="background:#9e1730"></i>&gt; 25° astronaut limit</div>`;
  else if (state.topo === 'roughness') el.innerHTML = `<b>Surface roughness (detrended 3×3 RMS)</b><div class="ramp" style="background:${ramp(ROUGH)}"></div><div class="ends"><span>0 m</span><span>${ROUGH_MAX / 2} m</span><span>≥ ${ROUGH_MAX} m</span></div>`;
  else el.innerHTML = `<b>Elevation (m)</b><div class="ramp" style="background:${ramp(HYPSO)}"></div><div class="ends"><span>${fmt(base, 0)} m</span><span>${fmt(top, 0)} m</span></div>`;
}
function thermalLegend() {
  const el = $('#legendThermal');
  el.hidden = state.view !== 'thermal';
  const stops = thermal.colormap_stops.map((s) => `rgb(${s.rgb.join(',')}) ${s.t * 100}%`).join(',');
  $('#thermalScale').style.background = `linear-gradient(90deg, ${stops})`;
  const lo = thermal.stats_k.min, hi = thermal.stats_k.max, mid = (lo + hi) / 2;
  const lab = (k) => `${fmt(k, 2)} K / ${fmt(k - 273.15, 2)} °C`;
  if (state.thermalMode === 'day') {
    $('#legendThermal .prov').textContent = 'THEMIS IR daytime image • relative brightness only (no temperature units) • visual layer';
    $('#tkMin').textContent = 'cooler (darker IR)'; $('#tkMid').textContent = ''; $('#tkMax').textContent = 'warmer (brighter IR)';
    return;
  }
  $('#legendThermal .prov').textContent = 'THEMIS IR Band 9 • ~100m native resolution • 03:15 Local Mars Time (Brightness Temp)';
  $('#tkMin').textContent = lab(lo); $('#tkMid').textContent = lab(mid); $('#tkMax').textContent = lab(hi);
}
function applyView() {
  $$('.view-card').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.view === state.view)));
  $$('[data-topo]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.topo === state.topo)));
  $$('[data-thermal]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.thermal === state.thermalMode)));
  $('#subPills').hidden = state.view !== 'topo';
  $('#thermalPills').hidden = state.view !== 'thermal';
  if (state.view === 'topo') { paintTopo(state.topo); terrainMesh.material = matTopo; }
  else if (state.view === 'optical') {
    if (!texOptical) toast('optical_mosaic.png missing — run scripts/prepare_web_assets.py');
    terrainMesh.material = matOptical;
  } else {
    const t = state.thermalMode === 'smooth' ? texThermalSmooth : state.thermalMode === 'day' ? texThermalDay : texThermalNative;
    matThermal.map = t; matThermal.emissiveMap = t; matThermal.needsUpdate = true;
    terrainMesh.material = matThermal;
  }
  legendTopo(); thermalLegend();
}
$$('.view-card').forEach((b) => b.addEventListener('click', () => { state.view = b.dataset.view; applyView(); }));
$$('[data-topo]').forEach((b) => b.addEventListener('click', () => { state.topo = b.dataset.topo; applyView(); }));
$$('[data-thermal]').forEach((b) => b.addEventListener('click', () => { state.thermalMode = b.dataset.thermal; applyView(); }));

// ------------------------------------------------------------------ picking (click vs drag)
const raycaster = new THREE.Raycaster();
const ndc = new THREE.Vector2();
function pickCell(ev) {
  const rect = renderer.domElement.getBoundingClientRect();
  ndc.set(((ev.clientX - rect.left) / rect.width) * 2 - 1, -((ev.clientY - rect.top) / rect.height) * 2 + 1);
  raycaster.setFromCamera(ndc, camera);
  const hit = raycaster.intersectObject(terrainMesh, false)[0];
  if (!hit) return null;
  const row = Math.round(hit.point.z / dy), col = Math.round(hit.point.x / dx);
  if (row < 0 || col < 0 || row >= rows || col >= cols) return null;
  return [row, col];
}
let down = null;
renderer.domElement.addEventListener('pointerdown', (e) => { down = { x: e.clientX, y: e.clientY, button: e.button }; });
renderer.domElement.addEventListener('pointerup', (e) => {
  if (!down) return;
  const moved = Math.hypot(e.clientX - down.x, e.clientY - down.y);
  const btn = down.button; down = null;
  if (moved > 5 || btn !== 0 || state.mode === 'navigate') return;   // drag = camera, never a marker
  const rc = pickCell(e);
  if (!rc) return;
  if (!ok(rc[0], rc[1])) { toast('No DEM data at that cell.'); return; }
  if (state.mode === 'start') state.start = rc;
  else if (state.mode === 'goal') state.goal = rc;
  else if (state.mode === 'waypoint') {
    if (state.waypoints.length >= MAX_WAYPOINTS) { toast(`Maximum ${MAX_WAYPOINTS} science waypoints (visit-order search is exhaustive).`); return; }
    if ([state.start, state.goal, ...state.waypoints.map((w) => w.rc)].some((p) => p && p[0] === rc[0] && p[1] === rc[1])) { toast('A marker already exists at that cell.'); return; }
    state.waypoints.push({ id: `S${state.waypoints.length + 1}`, rc });
  }
  onMissionEdited();
});
let hoverT = 0;
renderer.domElement.addEventListener('pointermove', (e) => {
  const now = performance.now(); if (now - hoverT < 70 || down) return; hoverT = now;
  const rc = pickCell(e);
  $('#hover').textContent = rc && ok(rc[0], rc[1]) ? `row ${rc[0]}, col ${rc[1]} · ${fmt(elev[rc[0]][rc[1]], 1)} m · slope ${fmt(slopes[rc[0]][rc[1]], 1)}° · ${fmt(temps[rc[0]][rc[1]], 1)} K` : '';
});
renderer.domElement.addEventListener('pointerleave', () => { $('#hover').textContent = ''; });

function setMode(m) {
  state.mode = m;
  $$('[data-mode]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.mode === m)));
  viewport.classList.toggle('picking', m !== 'navigate');
}
$$('[data-mode]').forEach((b) => b.addEventListener('click', () => setMode(b.dataset.mode)));

// ------------------------------------------------------------------ mission panel
const cellInfo = (rc) => `[${rc[0]}, ${rc[1]}] · ${fmt(zAt(rc[0], rc[1]), 0)} m`;
function renderMissionPanel() {
  $('#startLbl').textContent = state.start ? cellInfo(state.start) : 'click map';
  $('#goalLbl').textContent = state.goal ? cellInfo(state.goal) : 'click map';
  const ul = $('#wpList'); ul.innerHTML = '';
  if (!state.waypoints.length) { ul.innerHTML = '<li class="empty">No science waypoints — use “Add Waypoints” and click the terrain (max 5, any order).</li>'; }
  state.waypoints.forEach((w, i) => {
    const li = document.createElement('li');
    const s = slopes[w.rc[0]][w.rc[1]];
    li.innerHTML = `<span class="chip chip-s">${w.id}</span>[${w.rc[0]}, ${w.rc[1]}]<span class="meta ${s >= 20 ? 'lvl-high' : ''}">${fmt(zAt(w.rc[0], w.rc[1]), 0)} m · ${fmt(s, 1)}°</span><button title="Delete ${w.id}" aria-label="Delete ${w.id}">✕</button>`;
    li.querySelector('button').addEventListener('click', () => { state.waypoints.splice(i, 1); renumber(); onMissionEdited(); });
    ul.appendChild(li);
  });
  $$('[data-order]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.order === state.orderMode)));
}
function renumber() { state.waypoints.forEach((w, i) => { w.id = `S${i + 1}`; }); }
function renderSequence() {
  const el = $('#sequence'), r = state.result?.routes?.[state.active];
  if (!r) { el.innerHTML = ''; return; }
  if (r.status !== 'found') { el.innerHTML = `<div class="save lvl-high">⚠ ${PRIMARY_ROUTES[state.active].label}: ${r.status.replace('_', ' ')}${r.reason ? ' — ' + r.reason : ''}. Move the marker off steep/blocked terrain.</div>`; return; }
  const seq = r.visit_order.map((id) => `<span class="chip ${id === 'A' ? 'chip-a' : id === 'B' ? 'chip-b' : 'chip-s'}">${id}</span>`).join('<span class="arrow">→</span>');
  let save = '';
  const sv = r.savings_vs_manual, unit = r.objective_unit === 'm' ? 'm' : 'cost units';
  if (sv) {
    if (state.orderMode === 'optimized') {
      save = sv.objective_saved > 0.01
        ? `<div class="save">✔ Optimized vs manual click order (${sv.manual_visit_order.join(' → ')}): saves <b>${fmt(sv.objective_saved, 1)} ${unit}</b> (${fmt(sv.objective_saved_pct, 1)}%)${unit !== 'm' ? `, distance ${sv.distance_saved_m >= 0 ? '−' : '+'}${fmt(Math.abs(sv.distance_saved_m), 0)} m` : ''}.</div>`
        : `<div class="save neutral">Your click order is already the best sequence for this route.</div>`;
    } else {
      save = sv.objective_saved > 0.01
        ? `<div class="save neutral">Manual order. Optimized (${sv.optimized_visit_order.join(' → ')}) would save ${fmt(sv.objective_saved, 1)} ${unit} (${fmt(sv.objective_saved_pct, 1)}%).</div>`
        : `<div class="save neutral">Manual order is already optimal.</div>`;
    }
  }
  el.innerHTML = `<div class="seq">${seq}</div>${save}`;
}
function renderTable() {
  const t = $('#cmpTable'), res = state.result;
  if (!res) { t.innerHTML = '<tr><td>Set Start (A) and Goal (B) on the map.</td></tr>'; return; }
  const R = ROUTE_KEYS.map((k) => res.routes[k]);
  const head = `<thead><tr><th>Metric</th>${ROUTE_KEYS.map((k) => `<th class="c-${k} ${k === state.active ? 'active' : ''}">${{ astronaut: 'Astronaut', rover: 'Rover', shortest: 'Shortest' }[k]}</th>`).join('')}</tr></thead>`;
  const rowsDef = [
    ['Distance (m)', 'distance_m', 1, 'min'], ['Mean slope (°)', 'mean_slope_deg', 2, 'min'], ['Max slope (°)', 'max_slope_deg', 2, 'min'],
    ['Max edge grade (°)', 'max_edge_grade_deg', 2, 'min'], ['Total climb (m)', 'elevation_gain_m', 1, 'min'], ['Mean roughness (m)', 'mean_roughness_m', 3, 'min'],
    ['Mean temp (K)', 'mean_temp_k', 2, 'max'], ['Mean temp (°C)', 'mean_temp_c', 2, 'max'], ['Min temp (K)', 'min_temp_k', 2, 'max'],
  ];
  const body = rowsDef.map(([label, key, d, better]) => {
    const vals = R.map((r) => (r?.status === 'found' ? r[key] : null));
    const finite = vals.filter((v) => Number.isFinite(v));
    const best = finite.length > 1 ? (better === 'min' ? Math.min(...finite) : Math.max(...finite)) : null;
    return `<tr><td>${label}</td>${vals.map((v, i) => `<td class="${ROUTE_KEYS[i] === state.active ? 'active' : ''} ${v !== null && v === best ? 'best' : ''}">${v === null ? (R[i]?.status ?? '—').replace('_', ' ') : fmt(v, d)}</td>`).join('')}</tr>`;
  }).join('');
  const seqRow = `<tr><td>Visit sequence</td>${R.map((r, i) => `<td class="seqcell ${ROUTE_KEYS[i] === state.active ? 'active' : ''}">${r?.status === 'found' ? r.visit_order.join('→') : '—'}</td>`).join('')}</tr>`;
  const saveRow = res.waypoints?.length ? `<tr><td>Saved vs manual</td>${R.map((r, i) => `<td class="${ROUTE_KEYS[i] === state.active ? 'active' : ''}">${r?.savings_vs_manual ? fmt(r.savings_vs_manual.objective_saved_pct, 1) + '%' : '—'}</td>`).join('')}</tr>` : '';
  t.innerHTML = head + `<tbody>${body}${seqRow}${saveRow}</tbody>`;
  $('#routeSource').textContent = `${state.source} · ${res.runtime_ms ?? '—'} ms · thermal in cost: ${res.thermal_in_cost ? 'yes' : 'no'} · Shortest = astronaut limits (25°), distance only.`;
}

// ------------------------------------------------------------------ compute
let backendOnline = false, reqId = 0, editTimer = null;
async function checkBackend() {
  const badge = $('#backendBadge');
  try {
    const ctrl = new AbortController(); const to = setTimeout(() => ctrl.abort(), 1500);
    const r = await fetch(API + '/api/health', { signal: ctrl.signal }); clearTimeout(to);
    backendOnline = r.ok && (await r.json()).status === 'ok';
  } catch { backendOnline = false; }
  badge.textContent = backendOnline ? 'FastAPI backend online · Python A*' : 'Offline mode · in-browser JS planner';
  badge.className = 'badge ' + (backendOnline ? 'online' : 'offline');
}
function onMissionEdited() {
  rebuildMarkers(); renderMissionPanel();
  clearTimeout(editTimer); editTimer = setTimeout(recompute, 120);
}
async function recompute() {
  if (!state.start || !state.goal) { state.result = null; rebuildRoutes(); buildTrack(); renderTable(); renderSequence(); return; }
  const my = ++reqId;
  const body = { start_rc: state.start, goal_rc: state.goal, waypoints: state.waypoints.map((w) => ({ id: w.id, rc: w.rc })), order_mode: state.orderMode };
  $('#busy').hidden = false; $('#busyText').textContent = backendOnline ? 'Computing routes (FastAPI)…' : 'Computing routes in browser…';
  let res = null, src = '';
  if (backendOnline) {
    try {
      const ctrl = new AbortController(); const to = setTimeout(() => ctrl.abort(), 60000);
      const r = await fetch(API + '/api/mission', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body), signal: ctrl.signal });
      clearTimeout(to);
      if (r.status === 422) { const e = await r.json(); toast('Invalid request: ' + JSON.stringify(e.detail).slice(0, 160)); }
      else if (!r.ok) throw new Error('HTTP ' + r.status);
      else { res = await r.json(); src = 'Computed by FastAPI (Python V4 A* + mission_planner)'; }
    } catch (e) { console.warn('Backend failed, switching to browser planner', e); await checkBackend(); }
  }
  if (!res) {
    await new Promise((r) => setTimeout(r, 30));   // let the spinner paint
    res = planner.planMission(body.start_rc, body.goal_rc, body.waypoints, body.order_mode);
    src = 'Computed in browser (JS fallback planner — same cost model)';
  }
  if (my !== reqId) return;   // a newer click superseded this result
  $('#busy').hidden = true;
  applyResult(res, src, true);
}
function applyResult(res, src, resetProgress) {
  state.result = res; state.source = src;
  if (res.routes[state.active]?.status !== 'found') {
    const alt = ROUTE_KEYS.find((k) => res.routes[k]?.status === 'found');
    if (alt) state.active = alt;
  }
  if (resetProgress) { state.progress = 0; }
  syncActive(); rebuildRoutes(); buildTrack(); renderTable(); renderSequence();
}

// ------------------------------------------------------------------ route selection / visibility
function syncActive() { $$('[data-active]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.active === state.active))); }
$$('[data-active]').forEach((b) => b.addEventListener('click', () => {
  state.active = b.dataset.active; state.visible[state.active] = true;
  $(`[data-vis="${state.active}"]`).checked = true;
  syncActive(); rebuildRoutes(); buildTrack(); renderTable(); renderSequence();
}));
$$('[data-vis]').forEach((c) => c.addEventListener('change', () => { state.visible[c.dataset.vis] = c.checked; rebuildRoutes(); }));
$$('[data-order]').forEach((b) => b.addEventListener('click', () => { state.orderMode = b.dataset.order; renderMissionPanel(); recompute(); }));
$('#btnPreset').addEventListener('click', () => {
  state.start = [...PRESET.start]; state.goal = [...PRESET.goal];
  state.waypoints = PRESET.waypoints.map((rc, i) => ({ id: `S${i + 1}`, rc: [...rc] }));
  onMissionEdited();
});
$('#btnClearWp').addEventListener('click', () => { state.waypoints = []; onMissionEdited(); });
$('#btnClearAll').addEventListener('click', () => { state.start = state.goal = null; state.waypoints = []; state.playing = false; syncPlay(); onMissionEdited(); });

// ------------------------------------------------------------------ playback
let scrubbing = false;
function syncPlay() { $('#btnPlay').textContent = state.playing ? '❚❚' : '▶'; }
$('#btnPlay').addEventListener('click', () => {
  if (!track) { toast('Set Start (A) and Goal (B) first.'); return; }
  if (state.progress >= 1) setProgress(0);
  state.playing = !state.playing; syncPlay();
});
$('#btnReset').addEventListener('click', () => { state.playing = false; syncPlay(); setProgress(0); });
$$('[data-speed]').forEach((b) => b.addEventListener('click', () => {
  state.speed = +b.dataset.speed;
  $$('[data-speed]').forEach((x) => x.setAttribute('aria-pressed', String(x === b)));
}));
const scrub = $('#scrubber');
scrub.addEventListener('pointerdown', () => { scrubbing = true; state.playing = false; syncPlay(); });
scrub.addEventListener('pointerup', () => { scrubbing = false; });
scrub.addEventListener('input', () => { state.playing = false; syncPlay(); setProgress(+scrub.value / 1000); });

// ------------------------------------------------------------------ vertical exaggeration
const vsInput = $('#vScale');
vsInput.value = vScale.toFixed(2); $('#vsVal').textContent = `${vScale.toFixed(2)}x`;
let vsPending = false;
vsInput.addEventListener('input', () => {
  vScale = +vsInput.value; $('#vsVal').textContent = `${vScale.toFixed(2)}x`;
  if (vsPending) return; vsPending = true;
  requestAnimationFrame(() => {
    vsPending = false;
    updateHeights(); updateSkirt();
    markerGroup.children.forEach((m) => placeAt(m, m.userData.rc));
    rebuildRoutes(); setProgress(state.progress);
    const t = controls.target, ny = surfaceY(t.x, t.z), d = ny - t.y; t.y = ny; camera.position.y += d; controls.update();
  });
});

// ------------------------------------------------------------------ view buttons / panels
$('#btnFit').addEventListener('click', () => fitView('oblique'));
$('#btnTop').addEventListener('click', () => fitView('top'));
$('#btnFocus').addEventListener('click', () => {
  if (!track) { fitView('oblique'); return; }
  const off = camera.position.clone().sub(controls.target);
  controls.target.copy(mover.position); camera.position.copy(mover.position).add(off.setLength(Math.min(off.length(), 420))); controls.update();
});
function setPanel(open) {
  document.body.classList.toggle('panel-closed', !open);
  $('#btnPanel').setAttribute('aria-pressed', String(!open));
  $('#btnPanel').title = open ? 'Hide side panel (bigger map)' : 'Show side panel';
  try { localStorage.setItem('marseye.panel', open ? '1' : '0'); } catch {}
  requestAnimationFrame(() => { resize(); chart.resize(); });
}
$('#btnPanel').addEventListener('click', () => setPanel(document.body.classList.contains('panel-closed')));
$('#btnHud').addEventListener('click', () => {
  const off = document.body.classList.toggle('overlays-min');
  $('#btnHud').setAttribute('aria-pressed', String(off));
});
$('.hud-title').addEventListener('click', () => $('.hud').classList.toggle('collapsed'));
try { if (localStorage.getItem('marseye.panel') === '0') setPanel(false); } catch {}

// ------------------------------------------------------------------ boot
function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  if (w < 1 || h < 1) return;
  camera.aspect = w / h; camera.updateProjectionMatrix(); renderer.setSize(w, h, false);
  if (!resize.fitted) { resize.fitted = true; fitView('oblique'); }
}
new ResizeObserver(resize).observe(viewport);
window.addEventListener('resize', resize);
resize();
applyView();
renderMissionPanel(); renderTable();

// Start with the preset mission from the static fallback file (instant, no backend needed).
state.start = [...PRESET.start]; state.goal = [...PRESET.goal];
state.waypoints = PRESET.waypoints.map((rc, i) => ({ id: `S${i + 1}`, rc: [...rc] }));
rebuildMarkers(); renderMissionPanel();
const staticMatches = staticMission && JSON.stringify(staticMission.start_rc) === JSON.stringify(state.start)
  && JSON.stringify(staticMission.waypoints?.map((w) => w.rc)) === JSON.stringify(PRESET.waypoints);
if (staticMatches) applyResult(staticMission, 'Pre-computed by Python (data/processed/mission_route.json)', true);
checkBackend().then(() => { if (!staticMatches) recompute(); });

const clock = new THREE.Clock();
let t = 0;
renderer.setAnimationLoop(() => {
  const dt = Math.min(0.1, clock.getDelta()); t += dt;
  if (state.playing && track) {
    const seconds = Math.max(8, track.total / 45);       // 1x ≈ 45 m/s of simulated travel
    let p = state.progress + (dt * state.speed) / seconds;
    if (p >= 1) { p = 1; state.playing = false; syncPlay(); }
    setProgress(p);
  }
  const pulse = 1 + 0.35 * Math.sin(t * 4);
  halo.scale.setScalar(pulse); halo.material.opacity = 0.9 - 0.45 * (pulse - 0.65);
  moverGlow.scale.setScalar(0.9 + 0.15 * Math.sin(t * 6));
  markerGroup.children.forEach((m, i) => { const s = 1 + 0.18 * Math.sin(t * 3 + i); m.userData.ring?.scale.setScalar(s); });
  controls.update();
  renderer.render(scene, camera);
});

// Debug hooks for QA (raster <-> world round-trip):  MarsEye.world(0,0), MarsEye.state
window.MarsEye = {
  screenOf(r, c) { const v = world(r, c).project(camera), rect = renderer.domElement.getBoundingClientRect(); return [rect.left + (v.x + 1) / 2 * rect.width, rect.top + (1 - v.y) / 2 * rect.height]; },
  state, world, pickCell, planner, terrain, thermal, get track() { return track; } };
