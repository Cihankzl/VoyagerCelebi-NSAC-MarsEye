// MarsEye — 2D elevation profile (canvas), synchronised with the 3D marker.
// X: cumulative horizontal route distance (m). Y: elevation (m).
// Fill colour = local slope class; dashed lines = science waypoints; dot = live position.

export const slopeClass = (deg) => (!Number.isFinite(deg) ? 'na' : deg < 10 ? 'low' : deg < 20 ? 'mid' : 'high');
export const SLOPE_COLORS = { low: '#3ecf73', mid: '#f2b13a', high: '#ef4b4b', na: '#5b6675' };

export class ProfileChart {
  constructor(canvas, { onSeek } = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.onSeek = onSeek;
    this.track = null;
    this.progress = 0;
    this.base = document.createElement('canvas');
    this.pad = { l: 52, r: 14, t: 14, b: 26 };
    let dragging = false;
    const seek = (e) => {
      if (!this.track) return;
      const rect = canvas.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const f = (x - this.pad.l) / (rect.width - this.pad.l - this.pad.r);
      this.onSeek?.(Math.min(1, Math.max(0, f)));
    };
    canvas.addEventListener('pointerdown', (e) => { dragging = true; canvas.setPointerCapture(e.pointerId); seek(e); });
    canvas.addEventListener('pointermove', (e) => { if (dragging) seek(e); });
    canvas.addEventListener('pointerup', () => { dragging = false; });
    canvas.addEventListener('pointercancel', () => { dragging = false; });
    new ResizeObserver(() => this.resize()).observe(canvas);
  }

  resize() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.max(10, this.canvas.clientWidth), h = Math.max(10, this.canvas.clientHeight);
    for (const c of [this.canvas, this.base]) { c.width = Math.round(w * dpr); c.height = Math.round(h * dpr); }
    this.dpr = dpr; this.w = w; this.h = h;
    this.drawBase(); this.draw();
  }

  // track: {cum:[m], z:[m], slope:[deg], total, color, marks:[{id, d}]}
  setTrack(track) { this.track = track; this.drawBase(); this.draw(); }
  setProgress(f) { this.progress = f; this.draw(); }

  sx(d) { return this.pad.l + (d / (this.track.total || 1)) * (this.w - this.pad.l - this.pad.r); }
  sy(z) { return this.pad.t + (1 - (z - this.zMin) / (this.zMax - this.zMin || 1)) * (this.h - this.pad.t - this.pad.b); }

  drawBase() {
    const ctx = this.base.getContext('2d');
    if (!this.w) return;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.clearRect(0, 0, this.w, this.h);
    const t = this.track;
    ctx.font = '10px Inter, system-ui, sans-serif';
    if (!t || t.cum.length < 2) {
      ctx.fillStyle = '#7f93a8'; ctx.fillText('Set Start (A) and Goal (B) to see the elevation profile', this.pad.l, this.h / 2);
      return;
    }
    let lo = Infinity, hi = -Infinity;
    for (const z of t.z) { lo = Math.min(lo, z); hi = Math.max(hi, z); }
    const padZ = Math.max(4, (hi - lo) * 0.12);
    this.zMin = lo - padZ; this.zMax = hi + padZ;
    const bottom = this.h - this.pad.b;
    // grid + axes
    ctx.strokeStyle = '#22344a'; ctx.fillStyle = '#7f93a8'; ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const z = this.zMin + (i / 4) * (this.zMax - this.zMin), y = this.sy(z);
      ctx.beginPath(); ctx.moveTo(this.pad.l, y); ctx.lineTo(this.w - this.pad.r, y); ctx.stroke();
      ctx.textAlign = 'right'; ctx.fillText(z.toFixed(0), this.pad.l - 6, y + 3);
    }
    const step = niceStep(t.total / 6);
    ctx.textAlign = 'center';
    for (let d = 0; d <= t.total + 1e-6; d += step) ctx.fillText(`${d.toFixed(0)}`, this.sx(d), bottom + 14);
    ctx.textAlign = 'left'; ctx.fillText('m', this.w - this.pad.r - 8, bottom + 14);
    ctx.save(); ctx.translate(11, this.h / 2); ctx.rotate(-Math.PI / 2); ctx.textAlign = 'center'; ctx.fillText('Elevation (m)', 0, 0); ctx.restore();
    // slope-coloured fill
    for (let i = 0; i < t.cum.length - 1; i++) {
      const x0 = this.sx(t.cum[i]), x1 = this.sx(t.cum[i + 1]);
      ctx.fillStyle = SLOPE_COLORS[slopeClass(t.slope[i])] + '66';
      ctx.beginPath(); ctx.moveTo(x0, bottom); ctx.lineTo(x0, this.sy(t.z[i])); ctx.lineTo(x1, this.sy(t.z[i + 1])); ctx.lineTo(x1, bottom); ctx.closePath(); ctx.fill();
    }
    // slope-coloured profile line
    ctx.lineWidth = 2.2; ctx.lineJoin = 'round';
    for (let i = 0; i < t.cum.length - 1; i++) {
      ctx.strokeStyle = SLOPE_COLORS[slopeClass(t.slope[i])];
      ctx.beginPath(); ctx.moveTo(this.sx(t.cum[i]), this.sy(t.z[i])); ctx.lineTo(this.sx(t.cum[i + 1]), this.sy(t.z[i + 1])); ctx.stroke();
    }
    // waypoint markers
    ctx.setLineDash([4, 4]); ctx.lineWidth = 1;
    for (const m of t.marks) {
      const x = this.sx(m.d);
      ctx.strokeStyle = m.id === 'A' ? '#48e07e' : m.id === 'B' ? '#ffcc4d' : '#ff9d4d';
      ctx.beginPath(); ctx.moveTo(x, this.pad.t); ctx.lineTo(x, bottom); ctx.stroke();
      ctx.fillStyle = ctx.strokeStyle; ctx.textAlign = 'center'; ctx.fillText(m.id, Math.min(this.w - 12, Math.max(this.pad.l + 8, x)), this.pad.t - 3 + 10);
    }
    ctx.setLineDash([]);
  }

  draw() {
    const ctx = this.ctx;
    if (!this.w) return;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    ctx.drawImage(this.base, 0, 0);
    const t = this.track;
    if (!t || t.cum.length < 2) return;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    const d = this.progress * t.total;
    let i = 0; while (i < t.cum.length - 2 && t.cum[i + 1] < d) i++;
    const seg = t.cum[i + 1] - t.cum[i] || 1, f = Math.min(1, Math.max(0, (d - t.cum[i]) / seg));
    const z = t.z[i] + (t.z[i + 1] - t.z[i]) * f, x = this.sx(d), y = this.sy(z);
    ctx.strokeStyle = '#ffffff55'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x, this.pad.t); ctx.lineTo(x, this.h - this.pad.b); ctx.stroke();
    ctx.fillStyle = t.color; ctx.strokeStyle = '#fff'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(x, y, 5.5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    const label = `${z.toFixed(1)} m · ${d.toFixed(0)} m`;
    ctx.font = '600 10px Inter, system-ui, sans-serif';
    const w = ctx.measureText(label).width + 10, lx = Math.min(this.w - this.pad.r - w, Math.max(this.pad.l, x - w / 2));
    ctx.fillStyle = '#0b1420e6'; ctx.fillRect(lx, y - 26, w, 16);
    ctx.fillStyle = '#e8f1fb'; ctx.textAlign = 'left'; ctx.fillText(label, lx + 5, y - 15);
  }
}

function niceStep(raw) {
  const p = 10 ** Math.floor(Math.log10(raw || 1)), n = raw / p;
  return (n < 1.5 ? 1 : n < 3.5 ? 2 : n < 7.5 ? 5 : 10) * p;
}
