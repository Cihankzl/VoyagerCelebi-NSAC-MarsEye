import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const $ = (selector) => document.querySelector(selector);
const viewport = $('#viewport');
const status = $('#status');
const fmt = (v, digits=2) => Number.isFinite(v) ? v.toLocaleString('tr-TR', {minimumFractionDigits: digits, maximumFractionDigits: digits}) : '—';

// Repo-root http server: /frontend/marseye_v3_demo/ -> /prototypes/navigation/output_v3/
const dataDir = '../../prototypes/navigation/output_v3/';
const [terrain, routeData] = await Promise.all([
  fetch(dataDir + 'terrain.json').then((r) => { if (!r.ok) throw Error('terrain.json: HTTP ' + r.status); return r.json(); }),
  fetch(dataDir + 'routes.json').then((r) => { if (!r.ok) throw Error('routes.json: HTTP ' + r.status); return r.json(); }),
]).catch((e) => {
  status.textContent = 'Veriler okunamadı: ' + e.message + '\nRepo kökünde python -m http.server 8000 çalıştırın.';
  status.classList.add('error');
  throw e;
});

const [rows, cols] = terrain.grid_shape;
if (rows !== routeData.grid_shape[0] || cols !== routeData.grid_shape[1]) throw Error('Terrain/route grid size mismatch');
if (Math.abs(terrain.dx_m-routeData.dx_m)>1e-6 || Math.abs(terrain.dy_m-routeData.dy_m)>1e-6) throw Error('Terrain/route metre scale mismatch');
if (terrain.elevation_m.length!==rows || terrain.elevation_m[0].length!==cols) throw Error('Elevation grid invalid');
if (rows*cols>310000) throw Error('Demo optimized for <=310k cells, reduce DEM size');
const {dx_m: dx,dy_m: dy} = terrain;
const elevation=terrain.elevation_m, slopes=terrain.slope_deg, roughness=terrain.roughness_m;
let base=Infinity, top=-Infinity;
for (const row of elevation) for (const z of row) if (Number.isFinite(z)) { base=Math.min(base,z); top=Math.max(top,z); }
if (!Number.isFinite(base)) throw Error('No valid elevations');
const width=(cols-1)*dx, depth=(rows-1)*dy;
const relief=top-base;
// Same vertical scale for terrain, routes and markers; metrics never exaggerated.
const vScale=Math.min(2.5,Math.max(1,Math.max(width,depth)/(Math.max(relief,1)*2.9)));
const world = (r,c,offset=0) => new THREE.Vector3(c*dx,(elevation[r][c]-base)*vScale+offset,r*dy);

const scene = new THREE.Scene();
scene.background = new THREE.Color('#101926');
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
viewport.prepend(renderer.domElement);
const camera = new THREE.PerspectiveCamera(48,1,0.5,10000);
const controls = new OrbitControls(camera,renderer.domElement);
controls.enableDamping=true;
controls.target.set(width/2, relief*vScale*.35,depth/2);
camera.position.set(width*.8,Math.max(width,depth)*.8,depth*1.45);
controls.minDistance=12;
controls.maxDistance=Math.max(width,depth)*5;
controls.update();
scene.add(new THREE.AmbientLight('#d7e8ff',1.8));
const light = new THREE.DirectionalLight('#ffdbb0',2.3);
light.position.set(-width*.4, Math.max(width,depth)*1.8, -depth*.7);
scene.add(light);

const total=rows*cols;
const positions = new Float32Array(total*3);
const colors = new Float32Array(total*3);
const indices = [];
const naturalLow = new THREE.Color('#644c46');
const naturalHigh = new THREE.Color('#e5ad73');
const slopeLow = new THREE.Color('#d2c59b'), slopeMid=new THREE.Color('#d97d47'), slopeHigh=new THREE.Color('#ad3240');
const roughLow=new THREE.Color('#8ca7aa'), roughHigh=new THREE.Color('#ba4b4b');
let activeLayer='terrain';

function getColor(r,c) {
  if(activeLayer==='slope') {
    const s=slopes[r][c];
    if(!Number.isFinite(s)) return slopeHigh;
    if(s<10) return slopeLow.clone().lerp(slopeMid,s/20);
    if(s<20) return slopeMid.clone().lerp(slopeHigh,(s-10)/20);
    return slopeHigh;
  }
  if(activeLayer==='roughness') {
    const v=roughness[r][c];
    return roughLow.clone().lerp(roughHigh,THREE.MathUtils.clamp(v/0.5,0,1));
  }
  return naturalLow.clone().lerp(naturalHigh,THREE.MathUtils.clamp((elevation[r][c]-base)/(relief||1),0,1));
}
for(let r=0;r<rows;r++) for(let c=0;c<cols;c++) {
  const k=r*cols+c;
  const good=Number.isFinite(elevation[r][c]);
  const pos=good ? world(r,c) : new THREE.Vector3(c*dx,0,r*dy);
  positions.set([pos.x,pos.y,pos.z],3*k);
  const tint=good?getColor(r,c):new THREE.Color('#232b35');
  colors.set([tint.r,tint.g,tint.b],3*k);
  if(r<rows-1 && c<cols-1) {
    const a=k,b=k+1,d=k+cols,e=d+1;
    const goodAt=(rr,cc)=>Number.isFinite(elevation[rr][cc]);
    // Skip triangles with invalid (NoData) vertices.
    if(goodAt(r,c)&&goodAt(r+1,c)&&goodAt(r,c+1)) indices.push(a,d,b);
    if(goodAt(r,c+1)&&goodAt(r+1,c)&&goodAt(r+1,c+1)) indices.push(b,d,e);
  }
}
const geometry = new THREE.BufferGeometry();
geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));
geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));
geometry.setIndex(indices);
geometry.computeVertexNormals();
const mesh = new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({vertexColors:true,roughness:1,side:THREE.DoubleSide}));
scene.add(mesh);
function redrawColors() {
  for(let r=0;r<rows;r++) for(let c=0;c<cols;c++) {
    const tint=Number.isFinite(elevation[r][c])?getColor(r,c):new THREE.Color('#232b35');
    colors.set([tint.r,tint.g,tint.b],3*(r*cols+c));
  }
  geometry.attributes.color.needsUpdate=true;
}

const drawn={};
const routeColors={rover:0x42d5bd,astronaut:0xad86ff,shortest_reference:0x61c8ff};
function routeLine(name,record) {
  if(record?.status!=='found') return null;
  const coords=record.points_rc.map(([r,c])=>{
    if(!Number.isInteger(r)||!Number.isInteger(c)||r<0||c<0||r>=rows||c>=cols||!Number.isFinite(elevation[r][c])) throw Error('Route outside valid DEM: '+r+','+c);
    return world(r,c,2.4);
  });
  const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(coords),new THREE.LineBasicMaterial({color:routeColors[name],depthTest:false}));
  line.renderOrder=5;
  scene.add(line);
  drawn[name]=line;
  return line;
}
for(const name of ['shortest_reference','rover','astronaut']) routeLine(name,routeData.routes[name]);

const markers = new THREE.Group();
for(const [point,color] of [[routeData.start_rc,0x5ac8ff],[routeData.goal_rc,0xffd37b]]) {
  const [r,c]=point;
  const marker = new THREE.Mesh(new THREE.SphereGeometry(Math.max(2.7,dx*.65),16,12),new THREE.MeshBasicMaterial({color}));
  marker.position.copy(world(r,c,6.0));
  markers.add(marker);
}
scene.add(markers);

let selected='rover';
const addMetric = (label,value) => `<div class="metric"><span>${label}</span><strong>${value}</strong></div>`;
function renderUI() {
  const rover=selected==='rover';
  $('#mode-rover').setAttribute('aria-pressed',String(rover));
  $('#mode-astronaut').setAttribute('aria-pressed',String(!rover));
  $('#mode-desc').textContent = rover
    ? 'Rover modu, pürüzlülük ve geçilebilir eğim kısıtlarına daha fazla önem verir.'
    : 'Astronaut modu, toplam tırmanışa ve eğim riskine daha fazla ceza uygular.';
  for(const [name,line] of Object.entries(drawn)) line.visible = name===selected || (name==='shortest_reference' && $('#showShortest').checked);
  markers.visible=$('#showMarkers').checked;
  const route=routeData.routes[selected];
  if(route?.status!=='found') {
    $('#metrics').textContent='Bu profil için rota bulunamadı: '+(route?.status||'unknown');
    return;
  }
  $('#metrics').innerHTML = [
    addMetric('Mesafe',fmt(route.distance_m)+' m'),
    addMetric('Ortalama eğim',fmt(route.mean_slope_deg)+'°'),
    addMetric('Maksimum hücre eğimi',fmt(route.max_slope_deg)+'°'),
    addMetric('Maksimum geçiş eğimi',fmt(route.max_edge_grade_deg)+'°'),
    addMetric('Toplam tırmanış',fmt(route.elevation_gain_m)+' m'),
    addMetric('Ortalama pürüzlülük',fmt(route.mean_roughness_m,3)+' m'),
  ].join('');
  const p=routeData.profiles[selected];
  $('#limitText').textContent=`Örnek eğim sınırı: <${fmt(p.max_cell_slope_deg,0)}° (yerel hücre ve hücreler arası geçiş).`;
}
$('#mode-rover').addEventListener('click',()=>{selected='rover';renderUI();});
$('#mode-astronaut').addEventListener('click',()=>{selected='astronaut';renderUI();});
$('#showShortest').addEventListener('change',renderUI);
$('#showMarkers').addEventListener('change',renderUI);
$('#layer').addEventListener('change',(e)=>{
  activeLayer=e.target.value;
  redrawColors();
  $('#layerNote').textContent=activeLayer==='slope'?'Eğim: yeşilimsi/bej düşük; kırmızı yüksek. Eşikler temsili.'
    :activeLayer==='roughness'?'Pürüzlülük: yaklaşık 0–0,5 m DEM ölçeğinde RMS; taş tespiti değildir.'
    :'Yükseklik renklendirmesi: DEM tabanlı görünüm, gerçek optik kaplama değil.';
});
$('#status').textContent=`Gale DEM: ${rows}×${cols} • ${fmt(dx,2)}×${fmt(dy,2)} m`;
$('#dataNote').textContent=`Rotalar Python A* ile önceden hesaplandı. Görsel düşey ölçek ×${fmt(vScale,2)}.`;
$('#source').textContent=`Gale Krateri • 198×200 ilk sürüm, gerçek GeoTIFF'den türetilen yükseklik grid'i. Kaynak ürün kimliği henüz doğrulanmadı. Koordinat sistemi: ${terrain.source_metadata?.source_crs||'Mars koordinat sistemi'}; pilottaki tüm katmanlar aynı konum ve grid ile hizalanmalıdır.`;
renderUI();
function resize() {
  const w=viewport.clientWidth, h=viewport.clientHeight;
  if(w<1||h<1) return;
  camera.aspect=w/h;
  camera.updateProjectionMatrix();
  renderer.setSize(w,h,false);
}
window.addEventListener('resize',resize);
resize();
(function animate(){requestAnimationFrame(animate);controls.update();renderer.render(scene,camera);})();
