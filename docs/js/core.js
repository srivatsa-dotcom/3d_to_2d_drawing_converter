// core.js: geometry + drawing-sheet engine (no DOM). Used by the web app and by Node tests.
// Pipeline: mesh -> parts (dedupe) -> canonical orientation -> feature edges ->
// hidden-line projection (2D grid z-test) -> hole detection -> sheet primitives
// -> SVG / PDF (jsPDF) / DXF.

export const SHEETS = { A3: [420, 297], A4: [297, 210] };
const SCALES = [20, 10, 5, 4, 2, 1, 1 / 2, 1 / 2.5, 1 / 5, 1 / 10, 1 / 20, 1 / 50, 1 / 100];
const HARDWARE = /(screw|bolt|nut\b|nut_|washer|bearing|rivet|insert|circlip|spring|pin\b|\bdin[ _-]?\d|\biso[ _-]?\d|\bm[2-9]x|\bm1[0-6]x|grub|setscrew|socket[_ ]?head|hex[_ ]?head|countersunk|\bshcs\b|\bbhcs\b|\bfhcs\b)/i;
const GENERIC = /^(open cascade.*|solid\d*|body\d*|part\d*|shape\d*|compound\d*|unnamed.*|none|nauo\d*|[-_ .\d]*)$/i;

export const fmt = (v) => {
  const s = (Math.round(v * 100) / 100).toFixed(2).replace(/\.?0+$/, '');
  return s === '-0' || s === '' ? '0' : s;
};
export const scaleLabel = (s) => (s >= 1 ? `${Math.round(s)}:1` : `1:${fmt(1 / s)}`);
const yieldUI = () => new Promise((r) => setTimeout(r, 0));

// ---------------------------------------------------------------- vectors
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };

export const VIEWS = {
  front: [[0, -1, 0], [1, 0, 0]],
  top: [[0, 0, 1], [1, 0, 0]],
  left: [[-1, 0, 0], [0, -1, 0]],
  right: [[1, 0, 0], [0, 1, 0]],
  iso: [[1, -1, 1], [1, 1, 0]],
};
export function viewAxes(v) {
  const n = norm(VIEWS[v][0]); const x = norm(VIEWS[v][1]);
  return { n, x, y: cross(n, x) };
}

// ---------------------------------------------------------------- mesh basics
// Mesh = { V: Float64Array(3n), T: Int32Array(3m), F: Int32Array(m)|null }
export function weld(pos, idx, faceIds) {
  const map = new Map(); const V = []; const nv = pos.length / 3; const remap = new Int32Array(nv);
  for (let i = 0; i < nv; i++) {
    const x = pos[3 * i], y = pos[3 * i + 1], z = pos[3 * i + 2];
    const k = `${Math.round(x * 1e4)},${Math.round(y * 1e4)},${Math.round(z * 1e4)}`;
    let j = map.get(k);
    if (j === undefined) { j = V.length / 3; map.set(k, j); V.push(x, y, z); }
    remap[i] = j;
  }
  const m = idx ? idx.length / 3 : nv / 3; const T = []; const F = [];
  for (let t = 0; t < m; t++) {
    const a = remap[idx ? idx[3 * t] : 3 * t], b = remap[idx ? idx[3 * t + 1] : 3 * t + 1], c = remap[idx ? idx[3 * t + 2] : 3 * t + 2];
    if (a === b || b === c || a === c) continue;
    T.push(a, b, c); if (faceIds) F.push(faceIds[t]);
  }
  return { V: Float64Array.from(V), T: Int32Array.from(T), F: faceIds ? Int32Array.from(F) : null };
}

function vtx(M, i) { return [M.V[3 * i], M.V[3 * i + 1], M.V[3 * i + 2]]; }

export function bounds(M) {
  const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < M.V.length; i += 3) for (let k = 0; k < 3; k++) {
    const v = M.V[i + k]; if (v < mn[k]) mn[k] = v; if (v > mx[k]) mx[k] = v;
  }
  return { mn, mx, ext: [mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]] };
}

export function massProps(M) {
  let area = 0, vol = 0;
  for (let t = 0; t < M.T.length; t += 3) {
    const a = vtx(M, M.T[t]), b = vtx(M, M.T[t + 1]), c = vtx(M, M.T[t + 2]);
    const cr = cross([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
    area += Math.hypot(...cr) / 2; vol += dot(a, cross(b, c)) / 6;
  }
  return { area, vol: Math.abs(vol) };
}

function triNormals(M) {
  const n = M.T.length / 3; const N = new Float64Array(3 * n);
  for (let t = 0; t < n; t++) {
    const a = vtx(M, M.T[3 * t]), b = vtx(M, M.T[3 * t + 1]), c = vtx(M, M.T[3 * t + 2]);
    const cr = norm(cross([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]));
    N[3 * t] = cr[0]; N[3 * t + 1] = cr[1]; N[3 * t + 2] = cr[2];
  }
  return N;
}

export function splitComponents(M) {
  const nv = M.V.length / 3; const parent = new Int32Array(nv).map((_, i) => i);
  const find = (i) => { while (parent[i] !== i) { parent[i] = parent[parent[i]]; i = parent[i]; } return i; };
  for (let t = 0; t < M.T.length; t += 3) {
    const a = find(M.T[t]), b = find(M.T[t + 1]), c = find(M.T[t + 2]);
    parent[b] = a; parent[find(c)] = a;
  }
  const groups = new Map();
  for (let t = 0; t < M.T.length; t += 3) {
    const r = find(M.T[t]); if (!groups.has(r)) groups.set(r, []); groups.get(r).push(t / 3);
  }
  const out = [];
  for (const tris of groups.values()) {
    const vm = new Map(); const V = []; const T = []; const F = [];
    for (const t of tris) {
      for (let k = 0; k < 3; k++) {
        const g = M.T[3 * t + k]; let j = vm.get(g);
        if (j === undefined) { j = V.length / 3; vm.set(g, j); V.push(M.V[3 * g], M.V[3 * g + 1], M.V[3 * g + 2]); }
        T.push(j);
      }
      if (M.F) F.push(M.F[t]);
    }
    out.push({ V: Float64Array.from(V), T: Int32Array.from(T), F: M.F ? Int32Array.from(F) : null });
  }
  return out.sort((a, b) => b.T.length - a.T.length);
}

// symmetric 3x3 eigen decomposition (Jacobi)
function eigSym(A) {
  const a = A.map((r) => r.slice()); const v = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  for (let it = 0; it < 50; it++) {
    let p = 0, q = 1;
    if (Math.abs(a[0][2]) > Math.abs(a[p][q])) { p = 0; q = 2; }
    if (Math.abs(a[1][2]) > Math.abs(a[p][q])) { p = 1; q = 2; }
    if (Math.abs(a[p][q]) < 1e-12) break;
    const th = 0.5 * Math.atan2(2 * a[p][q], a[q][q] - a[p][p]);
    const c = Math.cos(th), s = Math.sin(th);
    for (let k = 0; k < 3; k++) { const akp = a[k][p], akq = a[k][q]; a[k][p] = c * akp - s * akq; a[k][q] = s * akp + c * akq; }
    for (let k = 0; k < 3; k++) { const apk = a[p][k], aqk = a[q][k]; a[p][k] = c * apk - s * aqk; a[q][k] = s * apk + c * aqk; }
    for (let k = 0; k < 3; k++) { const vkp = v[k][p], vkq = v[k][q]; v[k][p] = c * vkp - s * vkq; v[k][q] = s * vkp + c * vkq; }
  }
  return [0, 1, 2].map((i) => [v[0][i], v[1][i], v[2][i]]);
}

function applyFrame(M, ax, origin = [0, 0, 0]) {
  const V = new Float64Array(M.V.length);
  for (let i = 0; i < M.V.length; i += 3) {
    const p = [M.V[i] - origin[0], M.V[i + 1] - origin[1], M.V[i + 2] - origin[2]];
    V[i] = dot(p, ax[0]); V[i + 1] = dot(p, ax[1]); V[i + 2] = dot(p, ax[2]);
  }
  return { V, T: M.T, F: M.F };
}

// Keep designer orientation if axis-aligned, otherwise rotate onto principal axes.
// Always move the bounding-box minimum to the origin.
export function orient(M) {
  let b = bounds(M);
  const aabb = Math.max(b.ext[0] * b.ext[1] * b.ext[2], 1e-9);
  const n = M.V.length / 3; const c = [0, 0, 0];
  for (let i = 0; i < n; i++) for (let k = 0; k < 3; k++) c[k] += M.V[3 * i + k] / n;
  const C = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < n; i++) {
    const d = [M.V[3 * i] - c[0], M.V[3 * i + 1] - c[1], M.V[3 * i + 2] - c[2]];
    for (let r = 0; r < 3; r++) for (let s = 0; s < 3; s++) C[r][s] += d[r] * d[s];
  }
  let R = M;
  if (n > 3) {
    const ev = eigSym(C);
    const P = applyFrame(M, ev); const pb = bounds(P);
    const obb = pb.ext[0] * pb.ext[1] * pb.ext[2];
    if (obb > 0 && aabb / obb > 1.03) {
      const order = [0, 1, 2].sort((i, j) => pb.ext[j] - pb.ext[i]);
      const X = ev[order[0]], Y = ev[order[1]]; const Z = cross(X, Y);
      R = applyFrame(M, [X, Y, Z]);
    }
  }
  b = bounds(R);
  const V = new Float64Array(R.V.length);
  for (let i = 0; i < V.length; i += 3) for (let k = 0; k < 3; k++) V[i + k] = R.V[i + k] - b.mn[k];
  return { V, T: R.T, F: R.F };
}

// ---------------------------------------------------------------- edges
export function buildEdges(M, creaseDeg = 25) {
  const N = triNormals(M); const nv = M.V.length / 3; const map = new Map();
  for (let t = 0; t < M.T.length / 3; t++) {
    for (let k = 0; k < 3; k++) {
      let a = M.T[3 * t + k], b = M.T[3 * t + (k + 1) % 3]; if (a > b) [a, b] = [b, a];
      const key = a * nv + b; let l = map.get(key); if (!l) { l = []; map.set(key, l); } l.push(t);
    }
  }
  const stat = []; const smooth = []; const crease = Math.cos(creaseDeg * Math.PI / 180); const tangent = Math.cos(Math.PI / 180);
  for (const [key, ts] of map) {
    const a = Math.floor(key / nv), b = key - a * nv;
    if (ts.length !== 2) { stat.push(a, b); continue; }
    const [t1, t2] = ts; const c = N[3 * t1] * N[3 * t2] + N[3 * t1 + 1] * N[3 * t2 + 1] + N[3 * t1 + 2] * N[3 * t2 + 2];
    const sharp = M.F ? (M.F[t1] !== M.F[t2] && c < tangent) : c < crease;
    if (sharp) stat.push(a, b); else smooth.push(a, b, t1, t2);
  }
  return { N, map, nv, stat: Int32Array.from(stat), smooth: Int32Array.from(smooth) };
}

// ---------------------------------------------------------------- 2D grid depth test
function buildGrid(M, n, x, y) {
  const nv = M.V.length / 3; const U = new Float64Array(nv), W = new Float64Array(nv), D = new Float64Array(nv);
  let mnu = Infinity, mnv = Infinity, mxu = -Infinity, mxv = -Infinity;
  for (let i = 0; i < nv; i++) {
    const p = vtx(M, i); U[i] = dot(p, x); W[i] = dot(p, y); D[i] = dot(p, n);
    mnu = Math.min(mnu, U[i]); mxu = Math.max(mxu, U[i]); mnv = Math.min(mnv, W[i]); mxv = Math.max(mxv, W[i]);
  }
  const nt = M.T.length / 3; const G = Math.max(8, Math.min(400, Math.ceil(Math.sqrt(nt) * 1.2)));
  const span = Math.max(mxu - mnu, mxv - mnv, 1e-6); const cs = span / G;
  const gx = Math.max(1, Math.ceil((mxu - mnu) / cs) + 1), gy = Math.max(1, Math.ceil((mxv - mnv) / cs) + 1);
  const cells = new Array(gx * gy);
  for (let t = 0; t < nt; t++) {
    const a = M.T[3 * t], b = M.T[3 * t + 1], c = M.T[3 * t + 2];
    const i0 = Math.floor((Math.min(U[a], U[b], U[c]) - mnu) / cs), i1 = Math.floor((Math.max(U[a], U[b], U[c]) - mnu) / cs);
    const j0 = Math.floor((Math.min(W[a], W[b], W[c]) - mnv) / cs), j1 = Math.floor((Math.max(W[a], W[b], W[c]) - mnv) / cs);
    for (let i = i0; i <= i1; i++) for (let j = j0; j <= j1; j++) { const k = j * gx + i; (cells[k] || (cells[k] = [])).push(t); }
  }
  return { M, U, W, D, mnu, mnv, cs, gx, gy, cells, n, x, y };
}

function depthsAt(g, u, v, out) {
  out.length = 0;
  const i = Math.floor((u - g.mnu) / g.cs), j = Math.floor((v - g.mnv) / g.cs);
  if (i < 0 || j < 0 || i >= g.gx || j >= g.gy) return out;
  const cell = g.cells[j * g.gx + i]; if (!cell) return out;
  const { U, W, D, M } = g;
  for (const t of cell) {
    const a = M.T[3 * t], b = M.T[3 * t + 1], c = M.T[3 * t + 2];
    const den = (W[b] - W[c]) * (U[a] - U[c]) + (U[c] - U[b]) * (W[a] - W[c]);
    if (Math.abs(den) < 1e-14) continue;
    const l0 = ((W[b] - W[c]) * (u - U[c]) + (U[c] - U[b]) * (v - W[c])) / den;
    const l1 = ((W[c] - W[a]) * (u - U[c]) + (U[a] - U[c]) * (v - W[c])) / den;
    const l2 = 1 - l0 - l1;
    if (l0 < -1e-9 || l1 < -1e-9 || l2 < -1e-9) continue;
    out.push(l0 * D[a] + l1 * D[b] + l2 * D[c]);
  }
  return out;
}

// ---------------------------------------------------------------- projection
// returns { vis:[[x1,y1,x2,y2]...], hid:[...] } in model units, view coordinates
export function project(M, E, view, hidden = true) {
  const { n, x, y } = viewAxes(view);
  const g = buildGrid(M, n, x, y);
  const b = bounds(M); const diag = Math.hypot(...b.ext) || 1;
  const step = diag / 250; const eps = 1e-4 * diag + 1e-3;
  const edges = [];
  for (let i = 0; i < E.stat.length; i += 2) edges.push(E.stat[i], E.stat[i + 1]);
  const N = E.N;
  for (let i = 0; i < E.smooth.length; i += 4) {
    const t1 = E.smooth[i + 2], t2 = E.smooth[i + 3];
    const d1 = N[3 * t1] * n[0] + N[3 * t1 + 1] * n[1] + N[3 * t1 + 2] * n[2];
    const d2 = N[3 * t2] * n[0] + N[3 * t2 + 1] * n[1] + N[3 * t2 + 2] * n[2];
    if ((d1 > 1e-9) !== (d2 > 1e-9)) edges.push(E.smooth[i], E.smooth[i + 1]);
  }
  const vis = [], hid = []; const buf = [];
  for (let e = 0; e < edges.length; e += 2) {
    const A = vtx(M, edges[e]), B = vtx(M, edges[e + 1]);
    const L = Math.hypot(B[0] - A[0], B[1] - A[1], B[2] - A[2]);
    const k = Math.max(1, Math.min(60, Math.ceil(L / step)));
    let runStart = 0, runHidden = null;
    const P = (t) => [A[0] + (B[0] - A[0]) * t, A[1] + (B[1] - A[1]) * t, A[2] + (B[2] - A[2]) * t];
    const emit = (t0, t1, h) => {
      const p = P(t0), q = P(t1);
      const seg = [dot(p, x), dot(p, y), dot(q, x), dot(q, y)];
      if (!h) vis.push(seg); else if (hidden) hid.push(seg);
    };
    for (let j = 0; j < k; j++) {
      const pm = P((j + 0.5) / k);
      const d = dot(pm, n);
      const ds = depthsAt(g, dot(pm, x), dot(pm, y), buf);
      let h = false; for (const dd of ds) if (dd > d + eps) { h = true; break; }
      if (runHidden === null) runHidden = h;
      if (h !== runHidden) { emit(runStart / k, j / k, runHidden); runStart = j; runHidden = h; }
    }
    emit(runStart / k, 1, runHidden);
  }
  return { vis, hid };
}

export function segBBox(segs) {
  let a = Infinity, b = Infinity, c = -Infinity, d = -Infinity;
  for (const s of segs) {
    a = Math.min(a, s[0], s[2]); c = Math.max(c, s[0], s[2]);
    b = Math.min(b, s[1], s[3]); d = Math.max(d, s[1], s[3]);
  }
  return isFinite(a) ? [a, b, c, d] : [0, 0, 1, 1];
}

// ---------------------------------------------------------------- holes
export function findHoles(M, E) {
  // vertex adjacency from feature edges
  const adj = new Map();
  for (let i = 0; i < E.stat.length; i += 2) {
    const a = E.stat[i], b = E.stat[i + 1];
    (adj.get(a) || adj.set(a, []).get(a)).push(b);
    (adj.get(b) || adj.set(b, []).get(b)).push(a);
  }
  const seen = new Set(); const loops = [];
  for (const [s, nb] of adj) {
    if (seen.has(s) || nb.length !== 2) continue;
    const loop = [s]; seen.add(s); let prev = s, cur = nb[0], ok = true;
    while (cur !== s) {
      const nn = adj.get(cur);
      if (!nn || nn.length !== 2 || seen.has(cur)) { ok = false; break; }
      seen.add(cur); loop.push(cur);
      const nx = nn[0] === prev ? nn[1] : nn[0]; prev = cur; cur = nx;
      if (loop.length > 5000) { ok = false; break; }
    }
    if (ok && loop.length >= 8) loops.push(loop);
  }
  const grids = {};
  const axisGrid = (k) => grids[k] || (grids[k] = buildGrid(M, [0, 1, 2].map((i) => (i === k ? 1 : 0)), [0, 1, 2].map((i) => (i === (k + 1) % 3 ? 1 : 0)), [0, 1, 2].map((i) => (i === (k + 2) % 3 ? 1 : 0))));
  const buf = [];
  const rims = [];
  for (const loop of loops) {
    const P = loop.map((i) => vtx(M, i));
    const c = [0, 1, 2].map((k) => P.reduce((s, p) => s + p[k], 0) / P.length);
    let axis = -1;
    for (let k = 0; k < 3; k++) {
      const vals = P.map((p) => p[k]); const sp = Math.max(...vals) - Math.min(...vals);
      if (sp < 1e-3) { axis = k; break; }
    }
    if (axis < 0) continue;
    const u = (axis + 1) % 3, w = (axis + 2) % 3;
    const rs = P.map((p) => Math.hypot(p[u] - c[u], p[w] - c[w]));
    const r = rs.reduce((s, v) => s + v, 0) / rs.length;
    if (r < 1e-3 || Math.max(...rs) - Math.min(...rs) > 0.01 * r + 1e-4) continue;
    // side of the flat face bordering the loop
    let sgn = 0;
    for (let i = 0; i < loop.length && !sgn; i++) {
      let a = loop[i], b = loop[(i + 1) % loop.length]; if (a > b) [a, b] = [b, a];
      for (const t of E.map.get(a * E.nv + b) || []) {
        const nk = E.N[3 * t + axis]; if (Math.abs(nk) > 0.99) { sgn = Math.sign(nk); break; }
      }
    }
    if (!sgn) continue;
    const g = axisGrid(axis);
    const plane = c[axis];
    const delta = Math.min(0.2, 0.2 * r);
    const testD = plane - sgn * delta;
    // sample just inside the rim (below the flat face) at a few angles:
    // air there = hole, material there = boss / outer edge
    const rin = r - Math.min(0.25, 0.1 * r);
    let air = 0;
    for (const a of [0.31, 2.4, 4.3]) {
      const ds0 = depthsAt(g, c[u] + rin * Math.cos(a), c[w] + rin * Math.sin(a), buf);
      if (ds0.filter((d) => d > testD).length % 2 === 0) air++;
    }
    const inside = air < 3;
    const ju = c[u] + 1.37e-4 * r, jw = c[w] + 2.71e-4 * r; // jitter off triangle edges
    const ds = depthsAt(g, ju, jw, buf).slice();
    if (inside) continue; // boss / outer edge, not a hole
    // next surface going into the material (for blind holes)
    let next = null;
    for (const d of ds) {
      const dist = (plane - d) * sgn;
      if (dist > 1e-4 && (next === null || dist < next)) next = dist;
    }
    rims.push({ axis, c, r, plane, sgn, next });
  }
  // group by axis line
  const lines = new Map();
  for (const rm of rims) {
    const u = (rm.axis + 1) % 3, w = (rm.axis + 2) % 3;
    const key = `${rm.axis}|${Math.round(rm.c[u] * 100)}|${Math.round(rm.c[w] * 100)}`;
    (lines.get(key) || lines.set(key, []).get(key)).push(rm);
  }
  const holes = [];
  for (const list of lines.values()) {
    const byD = new Map();
    for (const rm of list) { const k = Math.round(rm.r * 200); (byD.get(k) || byD.set(k, []).get(k)).push(rm); }
    const groups = [...byD.values()].sort((a, b) => a[0].r - b[0].r);
    const main = groups[0];
    const planes = main.map((m) => m.plane);
    const thru = main.length >= 2;
    const depth = thru ? null : main[0].next;
    const cbore = groups.slice(1).map((gr) => {
      const ps = gr.map((m) => m.plane);
      const d = gr.length >= 2 ? Math.max(...ps) - Math.min(...ps)
        : Math.min(...planes.map((p) => Math.abs(p - ps[0])));
      return [2 * gr[0].r, d];
    });
    const entry = groups.length > 1 ? groups[groups.length - 1][0] : main.reduce((a, b) => (a.plane > b.plane ? a : b));
    const center = main[0].c.slice(); center[main[0].axis] = entry.plane;
    holes.push({ axis: main[0].axis, center, dia: 2 * main[0].r, depth, cbore });
  }
  return holes;
}

function holeHint(d) {
  const h = [[2.5, 'M3 tap drill?'], [3.3, 'M4 tap drill?'], [4.2, 'M5 tap drill?'], [3.4, 'M3 clearance?'], [3.2, 'M3 clearance?'],
    [4.5, 'M4 clearance?'], [5.5, 'M5 clearance?'], [6.6, 'M6 clearance?'], [6.8, 'M8 tap drill?'], [9.0, 'M8 clearance?']];
  for (const [v, s] of h) if (Math.abs(d - v) < 0.051) return s;
  return '';
}

// ---------------------------------------------------------------- loading
function cleanName(name, qty) {
  name = (name || '').trim() || 'part';
  if (qty > 1) name = name.replace(/([ _\-.]\d+|\s*\(\d+\)|<\d+>)$/, '') || name;
  return name;
}

// meshes: [{name, mesh}] (instances). Returns unique parts with qty.
export function makeParts(instances, { kind = 'step' } = {}) {
  const parts = [];
  const totalTris = instances.reduce((s, i) => s + i.mesh.T.length / 3, 0);
  for (const inst of instances) {
    const mp = massProps(inst.mesh);
    const sig = `${inst.mesh.T.length}|${Math.round(mp.area)}|${Math.round(mp.vol)}`;
    const ex = parts.find((p) => p.sig === sig);
    if (ex) { ex.qty++; continue; }
    parts.push({ sig, rawName: inst.name, mesh: inst.mesh, qty: 1, vol: mp.vol, area: mp.area });
  }
  const used = new Set();
  for (const p of parts) {
    p.mesh = orient(p.mesh);
    const b = bounds(p.mesh); p.ext = b.ext;
    const d = [...b.ext].sort((a, c) => c - a);
    const plate = d[2] <= 12 && d[2] <= 0.2 * d[1];
    p.plate = plate; p.thickness = d[2];
    let name = cleanName(p.rawName, p.qty);
    if (!p.rawName || GENERIC.test(name)) {
      let k = plate ? 'plate' : 'part';
      if (kind === 'stl' && p.mesh.T.length / 3 === 20 && d[0] < 15) k = 'hex_head_or_nut';
      name = `${k}_${fmt(d[0])}x${fmt(d[1])}x${fmt(d[2])}`;
      if (kind === 'stl' && instances.length > 1 && p.mesh.T.length / 3 > 0.5 * totalTris) name = 'main_body_' + name.split('_').slice(1).join('_');
    }
    let n2 = name, k = 2; while (used.has(n2)) n2 = `${name}_${k++}`; used.add(n2);
    p.name = n2;
    p.hardware = HARDWARE.test(p.name) || p.name.startsWith('hex_head_or_nut');
    p.fused = p.name.startsWith('main_body');
  }
  return parts;
}

export function analysePart(p, crease) {
  p.edges = buildEdges(p.mesh, crease);
  p.holes = findHoles(p.mesh, p.edges);
  return p;
}

// ---------------------------------------------------------------- sheet
class Sheet {
  constructor(w, h) { this.w = w; this.h = h; this.items = []; }
  line(x1, y1, x2, y2, l = 'visible') { this.items.push(['L', l, [x1, y1, x2, y2]]); }
  circle(cx, cy, r, l = 'visible') { this.items.push(['C', l, [cx, cy, r]]); }
  tri(p, l = 'dim') { this.items.push(['T', l, p]); }
  text(x, y, s, h = 3.5, ha = 'center', va = 'center', rot = 0, l = 'text', bold = false) { this.items.push(['X', l, [x, y, String(s), h, ha, va, rot, bold]]); }
  rect(x, y, w, h, l = 'border') { this.line(x, y, x + w, y, l); this.line(x + w, y, x + w, y + h, l); this.line(x + w, y + h, x, y + h, l); this.line(x, y + h, x, y, l); }
  place(segs, ox, oy, s, l) { for (const g of segs) this.line(ox + s * g[0], oy + s * g[1], ox + s * g[2], oy + s * g[3], l); }
  arrow(x, y, dx, dy, L = 3, W = 0.6) {
    const n = Math.hypot(dx, dy) || 1; const ux = dx / n, uy = dy / n; const bx = x - ux * L, by = y - uy * L;
    this.tri([[x, y], [bx - uy * W, by + ux * W], [bx + uy * W, by - ux * W]]);
  }
  hdim(x1, x2, yo, yd, val) {
    const s = yd > yo ? 1 : -1;
    this.line(x1, yo + s, x1, yd + 2 * s, 'dim'); this.line(x2, yo + s, x2, yd + 2 * s, 'dim'); this.line(x1, yd, x2, yd, 'dim');
    if (x2 - x1 > 7) { this.arrow(x1, yd, -1, 0); this.arrow(x2, yd, 1, 0); } else { this.arrow(x1, yd, 1, 0); this.arrow(x2, yd, -1, 0); }
    this.text((x1 + x2) / 2, yd + 1.2, val, 3.5, 'center', 'bottom', 0, 'dim');
  }
  vdim(y1, y2, xo, xd, val) {
    const s = xd > xo ? 1 : -1;
    this.line(xo + s, y1, xd + 2 * s, y1, 'dim'); this.line(xo + s, y2, xd + 2 * s, y2, 'dim'); this.line(xd, y1, xd, y2, 'dim');
    if (y2 - y1 > 7) { this.arrow(xd, y1, 0, -1); this.arrow(xd, y2, 0, 1); } else { this.arrow(xd, y1, 0, 1); this.arrow(xd, y2, 0, -1); }
    this.text(xd - 1.2, (y1 + y2) / 2, val, 3.5, 'center', 'bottom', 90, 'dim');
  }
}

function pickScale(nw, nh, aw, ah, gw, gh) {
  for (const s of SCALES) if (nw * s + gw <= aw && nh * s + gh <= ah) return s;
  return SCALES[SCALES.length - 1];
}

// opts: {sheet:'A3', projection:'first', hidden:true, material, tolerance, source, item, date}
export async function makeSheet(p, opts) {
  const [W, H] = SHEETS[opts.sheet || 'A3'];
  const first = (opts.projection || 'first') === 'first';
  const side = first ? 'left' : 'right';
  const names = ['front', 'top', side];
  const proj = {};
  for (const v of names) { proj[v] = project(p.mesh, p.edges, v, opts.hidden !== false); await yieldUI(); }
  const iso = project(p.mesh, p.edges, 'iso', false); await yieldUI();
  const bb = {}; const size = {};
  for (const v of names) bb[v] = segBBox(proj[v].vis.concat(proj[v].hid));
  bb.iso = segBBox(iso.vis);
  for (const v in bb) size[v] = [bb[v][2] - bb[v][0], bb[v][3] - bb[v][1]];

  // hole rows
  const axisView = { 0: side, 1: 'front', 2: 'top' };
  const groups = new Map();
  for (const h of p.holes) {
    const key = `${fmt(h.dia)}|${h.depth == null ? 'T' : fmt(h.depth)}|${h.cbore.map((c) => fmt(c[0]) + 'x' + fmt(c[1])).join(',')}`;
    (groups.get(key) || groups.set(key, []).get(key)).push(h);
  }
  const pcd = new Map();
  for (const lst of groups.values()) {
    if (lst.length < 3) continue;
    const c = [0, 1, 2].map((k) => lst.reduce((s, h) => s + h.center[k], 0) / lst.length);
    const rr = lst.map((h) => Math.hypot(h.center[0] - c[0], h.center[1] - c[1], h.center[2] - c[2]));
    const mean = rr.reduce((a, b) => a + b, 0) / rr.length;
    if (mean > 1e-3 && Math.max(...rr) - Math.min(...rr) < 0.02) for (const h of lst) pcd.set(h, `${lst.length}x on PCD Ø${fmt(2 * mean)}`);
  }
  const rows = []; const letters = 'ABCDEFGHJKLMNPQRSTUVWXYZ';
  [...groups.values()].sort((a, b) => b[0].dia - a[0].dia).forEach((lst, gi) => {
    lst.forEach((h, i) => {
      h.id = `${letters[Math.min(gi, letters.length - 1)]}${i + 1}`;
      h.view = axisView[h.axis];
      const { x, y } = viewAxes(h.view);
      h.xy = [dot(h.center, x) - bb[h.view][0], dot(h.center, y) - bb[h.view][1]];
      const cb = h.cbore.map((c) => `CBORE Ø${fmt(c[0])} x ${fmt(c[1])}`).join('; ');
      const note = [cb, pcd.get(h) || '', holeHint(h.dia)].filter(Boolean).join('; ');
      rows.push([h.id, h.view.toUpperCase(), fmt(h.xy[0]), fmt(h.xy[1]), `Ø${fmt(h.dia)}`, h.depth == null ? 'THRU' : fmt(h.depth), note]);
    });
  });

  // layout
  const m = 10, rc = W >= 400 ? 120 : 95, rx0 = W - m - rc;
  const vx0 = m + 18, vy0 = m + 14, vx1 = rx0 - 14, vy1 = H - m - 14, gap = 26;
  const [fw, fh] = size.front, [, th] = size.top, [sw] = size[side];
  const s = pickScale(fw + sw, fh + th, vx1 - vx0, vy1 - vy0, gap, gap);
  const tw = s * (fw + sw) + gap, thh = s * (fh + th) + gap;
  const bx = vx0 + (vx1 - vx0 - tw) / 2, by = vy0 + (vy1 - vy0 - thh) / 2;
  let pos, isoBox;
  if (first) {
    pos = { front: [bx, by + s * th + gap], top: [bx, by], [side]: [bx + s * fw + gap, by + s * th + gap] };
    isoBox = [bx + s * fw + gap, vy0, vx1, by + s * th + gap * 0.6];
  } else {
    pos = { front: [bx, by], top: [bx, by + s * fh + gap], [side]: [bx + s * fw + gap, by] };
    isoBox = [bx + s * fw + gap, by + s * fh + gap * 0.4, vx1, vy1];
  }
  const S = new Sheet(W, H);
  S.rect(m, m, W - 2 * m, H - 2 * m);
  const labels = { front: 'FRONT VIEW', top: 'TOP VIEW', left: 'LEFT VIEW', right: 'RIGHT VIEW' };
  for (const v of names) {
    const [ox, oy] = pos[v]; const X0 = ox - s * bb[v][0], Y0 = oy - s * bb[v][1];
    S.place(proj[v].hid, X0, Y0, s, 'hidden'); S.place(proj[v].vis, X0, Y0, s, 'visible');
    S.text(ox + s * size[v][0] / 2, oy - 13, labels[v], 2.5, 'center', 'top');
  }
  const corners = (v) => [pos[v][0], pos[v][1], pos[v][0] + s * size[v][0], pos[v][1] + s * size[v][1]];
  const f = corners('front'); S.hdim(f[0], f[2], f[3], f[3] + 9, fmt(size.front[0])); S.vdim(f[1], f[3], f[0], f[0] - 9, fmt(size.front[1]));
  const t = corners('top'); S.vdim(t[1], t[3], t[0], t[0] - 9, fmt(size.top[1]));
  const sd = corners(side); S.hdim(sd[0], sd[2], sd[3], sd[3] + 9, fmt(size[side][0]));
  for (const v of names) S.circle(pos[v][0], pos[v][1], 0.8, 'dim');
  for (const h of p.fused ? [] : p.holes) {
    if (!pos[h.view]) continue;
    const cx = pos[h.view][0] + s * h.xy[0], cy = pos[h.view][1] + s * h.xy[1]; const r = s * h.dia / 2; const e = r + 1.5;
    S.line(cx - e, cy, cx + e, cy, 'center'); S.line(cx, cy - e, cx, cy + e, 'center');
    S.text(cx + r * 0.75 + 0.8, cy + r * 0.75 + 0.8, h.id, 2.2, 'left', 'bottom', 0, 'dim');
  }
  const [ix0, iy0, ix1, iy1] = isoBox; const [iw, ih] = size.iso;
  if (ix1 - ix0 > 25 && iy1 - iy0 > 25) {
    const si = pickScale(iw, ih, ix1 - ix0 - 6, iy1 - iy0 - 10, 0, 0);
    const cx = (ix0 + ix1) / 2 - si * iw / 2, cy = (iy0 + iy1) / 2 - si * ih / 2 + 3;
    S.place(iso.vis, cx - si * bb.iso[0], cy - si * bb.iso[1], si, 'visible');
    S.text((ix0 + ix1) / 2, cy - 4, `ISOMETRIC VIEW (scale ${scaleLabel(si)})`, 2.5, 'center', 'top');
  }
  // title block
  const [L, Wd, Ht] = p.ext;
  const tb = [
    ['PART', p.name], ['ITEM / QTY', `${opts.item || '01'}   /   ${p.qty} off`],
    ['SIZE (X×Y×Z)', `${fmt(L)} × ${fmt(Wd)} × ${fmt(Ht)} mm`],
    ['MATERIAL', opts.material || '________________'], ['TOLERANCE', opts.tolerance || '________________'],
    ['SCALE', `${scaleLabel(s)}   UNITS: mm`], ['PROJECTION', first ? 'FIRST ANGLE' : 'THIRD ANGLE'],
    ['VOLUME', `${fmt(p.vol / 1000)} cm³`], ['SOURCE', (opts.source || '').slice(0, 34)], ['DATE', opts.date || ''],
  ];
  const rh = 5.2, tbh = rh * tb.length;
  S.rect(rx0, m, rc, tbh);
  tb.forEach(([k, v], i) => {
    const yy = m + tbh - (i + 1) * rh;
    if (i) S.line(rx0, yy + rh, rx0 + rc, yy + rh, 'thin');
    S.text(rx0 + 1.5, yy + rh / 2, k, 2.0, 'left', 'center');
    S.text(rx0 + 30, yy + rh / 2, v, k === 'PART' ? 3.2 : 2.6, 'left', 'center', 0, 'text', k === 'PART');
  });
  S.line(rx0 + 28, m, rx0 + 28, m + tbh, 'thin');
  S.text(m + 2, m + 2, `Auto-generated from a 3D model${opts.kind === 'stl' ? ' (STL mesh: round features are faceted)' : ''} - check all dimensions before manufacturing. Hole positions are from the view datum (o = bottom-left of each view).`, 2.0, 'left', 'bottom');
  // hole table
  const sheets = [S];
  const cols = [['HOLE', 11], ['VIEW', 13], ['X', 14], ['Y', 14], ['SIZE', 14], ['DEPTH', 14], ['NOTE', rc - 80]];
  const top = H - m - 2; const rh2 = 4.4; const maxRows = Math.floor((top - (m + tbh + 6)) / rh2) - 3;
  const drawTable = (sh, chunk, x, tp, title) => {
    sh.text(x, tp - 1, title, 2.8, 'left', 'top', 0, 'text', true);
    let y = tp - 7, cx = x;
    for (const [nm, w] of cols) { sh.text(cx + 1, y, nm, 2.0, 'left', 'center', 0, 'text', true); cx += w; }
    sh.line(x, y - rh2 / 2, x + cols.reduce((a, c) => a + c[1], 0), y - rh2 / 2, 'thin');
    for (const row of chunk) {
      y -= rh2; cx = x;
      cols.forEach(([nm, w], i) => { sh.text(cx + 1, y, row[i], nm === 'NOTE' ? 1.9 : 2.1, 'left', 'center'); cx += w; });
    }
  };
  if (p.fused) {
    let y = top - 2;
    for (const line of ['Several parts are fused into one body in this', 'file (chassis, wheels, ...). Export each part', 'separately (or as STEP) for individual drawings.']) { S.text(rx0, y, line, 2.4, 'left', 'top'); y -= 4.2; }
  } else if (rows.length) {
    drawTable(S, rows.slice(0, maxRows), rx0, top, `HOLE TABLE (${rows.length} holes)`);
    for (let i = maxRows, k = 2; i < rows.length; i += 50, k++) {
      const X = new Sheet(W, H); X.rect(m, m, W - 2 * m, H - 2 * m);
      drawTable(X, rows.slice(i, i + 50), m + 10, H - m - 4, `${p.name} - HOLE TABLE (continued)`);
      sheets.push(X);
    }
  } else {
    S.text(rx0, top - 1, 'No round holes detected', 2.6, 'left', 'top');
  }
  // cut profile (plate parts): visible edges looking along the thinnest axis
  let cut = null;
  if (p.plate) {
    const thin = p.ext.indexOf(Math.min(...p.ext));
    const cv = { 0: side, 1: 'front', 2: 'top' }[thin];
    cut = proj[cv].vis;
  }
  return { sheets, scale: s, rows, cut };
}

export function bomSheet(parts, src, sheet = 'A3', date = '') {
  const [W, H] = SHEETS[sheet]; const m = 10; const S = new Sheet(W, H);
  S.rect(m, m, W - 2 * m, H - 2 * m);
  const made = parts.filter((p) => !p.hardware), hw = parts.filter((p) => p.hardware);
  S.text(m + 8, H - m - 8, `BILL OF MATERIALS - ${src}`, 5, 'left', 'top', 0, 'text', true);
  S.text(m + 8, H - m - 16, `${made.length} manufactured part(s) with drawings, ${hw.length} hardware item(s). Generated ${date}.`, 2.6, 'left', 'top');
  const cols = [['ITEM', 14], ['PART', 95], ['QTY', 14], ['SIZE X×Y×Z (mm)', 50], ['HOLES', 16], ['TYPE', 70]];
  let y = H - m - 30, cx = m + 8;
  for (const [n, w] of cols) { S.text(cx, y, n, 2.6, 'left', 'center', 0, 'text', true); cx += w; }
  S.line(m + 8, y - 3, W - m - 8, y - 3, 'thin');
  for (const p of [...made, ...hw]) {
    y -= 5.5; if (y < m + 8) break; cx = m + 8;
    const row = [p.hardware ? 'HW' : p.item, p.name.slice(0, 50), p.qty, p.ext.map(fmt).join(' × '), p.hardware ? '-' : p.holes.length, partType(p)];
    cols.forEach(([, w], i) => { S.text(cx, y, row[i], 2.4, 'left', 'center'); cx += w; });
  }
  return S;
}

export function partType(p) {
  if (p.hardware) return 'Purchased hardware (no drawing)';
  if (p.fused) return 'Several parts merged in the file';
  if (p.plate) return `Plate t=${fmt(p.thickness)} (laser/CNC cut)`;
  return 'Machined / printed part';
}

// ---------------------------------------------------------------- output
export const STYLE = {
  visible: { w: 0.35, dash: null, c: '#111820' },
  hidden: { w: 0.18, dash: [1.8, 1.0], c: '#6b747c' },
  center: { w: 0.13, dash: [4, 1, 0.7, 1], c: '#1f5fbf' },
  dim: { w: 0.13, dash: null, c: '#1f5fbf' },
  thin: { w: 0.15, dash: null, c: '#111820' },
  border: { w: 0.45, dash: null, c: '#111820' },
  text: { w: 0.15, dash: null, c: '#111820' },
};
const esc = (s) => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');

export function toSVG(S) {
  const H = S.h; const out = [];
  out.push(`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${S.w} ${S.h}" width="${S.w}mm" height="${S.h}mm" font-family="Barlow, 'Helvetica Neue', Arial, sans-serif">`);
  out.push(`<rect width="${S.w}" height="${S.h}" fill="#fff"/>`);
  const byLayer = {};
  for (const [k, l, d] of S.items) {
    if (k === 'L') (byLayer[l] = byLayer[l] || []).push(`M${d[0].toFixed(3)} ${(H - d[1]).toFixed(3)}L${d[2].toFixed(3)} ${(H - d[3]).toFixed(3)}`);
  }
  for (const l in byLayer) {
    const st = STYLE[l] || STYLE.thin;
    out.push(`<path d="${byLayer[l].join('')}" fill="none" stroke="${st.c}" stroke-width="${st.w}" stroke-linecap="round"${st.dash ? ` stroke-dasharray="${st.dash.join(' ')}"` : ''}/>`);
  }
  for (const [k, l, d] of S.items) {
    const st = STYLE[l] || STYLE.thin;
    if (k === 'C') out.push(`<circle cx="${d[0]}" cy="${H - d[1]}" r="${d[2]}" fill="none" stroke="${st.c}" stroke-width="${st.w}"/>`);
    else if (k === 'T') out.push(`<path d="M${d.map((p) => `${p[0].toFixed(3)} ${(H - p[1]).toFixed(3)}`).join('L')}Z" fill="${st.c}"/>`);
    else if (k === 'X') {
      const [x, y, s, h, ha, va, rot, bold] = d;
      const anchor = { left: 'start', center: 'middle', right: 'end' }[ha];
      const base = { bottom: 'auto', center: 'central', top: 'hanging' }[va];
      out.push(`<text x="${x}" y="${H - y}" font-size="${(h / 0.72).toFixed(2)}" text-anchor="${anchor}" dominant-baseline="${base}"${bold ? ' font-weight="600"' : ''} fill="${st.c}"${rot ? ` transform="rotate(${-rot} ${x} ${H - y})"` : ''}>${esc(s)}</text>`);
    }
  }
  out.push('</svg>');
  return out.join('\n');
}

export function drawPDF(doc, S) {
  const H = S.h;
  const setStyle = (l) => {
    const st = STYLE[l] || STYLE.thin;
    doc.setLineWidth(st.w); doc.setDrawColor(st.c); doc.setFillColor(st.c); doc.setTextColor(st.c);
    doc.setLineDashPattern(st.dash || [], 0);
  };
  let cur = null;
  for (const [k, l, d] of S.items) {
    if (l !== cur || k === 'X') { setStyle(l); cur = l; }
    if (k === 'L') doc.line(d[0], H - d[1], d[2], H - d[3]);
    else if (k === 'C') doc.circle(d[0], H - d[1], d[2], 'S');
    else if (k === 'T') { doc.setLineDashPattern([], 0); doc.triangle(d[0][0], H - d[0][1], d[1][0], H - d[1][1], d[2][0], H - d[2][1], 'F'); cur = null; }
    else if (k === 'X') {
      const [x, y, s, h, ha, va, rot, bold] = d;
      doc.setFont('helvetica', bold ? 'bold' : 'normal');
      doc.setFontSize((h / 0.72) * 72 / 25.4);
      doc.text(s.replace(/³/g, '3'), x, H - y, { align: ha === 'center' ? 'center' : ha === 'right' ? 'right' : 'left', baseline: { bottom: 'bottom', center: 'middle', top: 'top' }[va], angle: rot || 0 });
    }
  }
}

// rotated centred text in jsPDF needs manual centring: pre-process items
export function pdfReady(S) {
  const C = new Sheet(S.w, S.h);
  for (const it of S.items) {
    if (it[0] === 'X' && it[2][6] && it[2][4] === 'center') {
      const [x, y, s, h, , va, rot, bold] = it[2];
      const wEst = s.length * h * 0.55; // approximate text width
      C.items.push(['X', it[1], [x, y - wEst / 2, s, h, 'left', va, rot, bold]]);
    } else C.items.push(it);
  }
  return C;
}

export function toDXF(sheets, scale, cutOnly = null) {
  const k = cutOnly ? 1 : 1 / scale;
  const o = [];
  const p = (c, v) => o.push(String(c), String(v));
  p(0, 'SECTION'); p(2, 'HEADER'); p(9, '$INSUNITS'); p(70, 4); p(9, '$LTSCALE'); p(40, Math.max(k, 0.01)); p(0, 'ENDSEC');
  p(0, 'SECTION'); p(2, 'TABLES');
  p(0, 'TABLE'); p(2, 'LTYPE'); p(70, 3);
  p(0, 'LTYPE'); p(2, 'CONTINUOUS'); p(70, 0); p(3, 'Solid line'); p(72, 65); p(73, 0); p(40, 0);
  p(0, 'LTYPE'); p(2, 'DASHED'); p(70, 0); p(3, '__ __ __'); p(72, 65); p(73, 2); p(40, 3); p(49, 2); p(49, -1);
  p(0, 'LTYPE'); p(2, 'CENTER'); p(70, 0); p(3, '____ _ ____'); p(72, 65); p(73, 4); p(40, 7); p(49, 4); p(49, -1); p(49, 1); p(49, -1);
  p(0, 'ENDTAB');
  const layers = cutOnly ? [['CUT', 7, 'CONTINUOUS']] : [['VISIBLE', 7, 'CONTINUOUS'], ['HIDDEN', 8, 'DASHED'], ['CENTER', 5, 'CENTER'], ['DIM', 5, 'CONTINUOUS'], ['THIN', 7, 'CONTINUOUS'], ['BORDER', 7, 'CONTINUOUS'], ['TEXT', 7, 'CONTINUOUS']];
  p(0, 'TABLE'); p(2, 'LAYER'); p(70, layers.length);
  for (const [n, c, lt] of layers) { p(0, 'LAYER'); p(2, n); p(70, 0); p(62, c); p(6, lt); }
  p(0, 'ENDTAB'); p(0, 'ENDSEC');
  p(0, 'SECTION'); p(2, 'ENTITIES');
  if (cutOnly) {
    const bb = segBBox(cutOnly);
    for (const s of cutOnly) { p(0, 'LINE'); p(8, 'CUT'); p(10, s[0] - bb[0]); p(20, s[1] - bb[1]); p(30, 0); p(11, s[2] - bb[0]); p(21, s[3] - bb[1]); p(31, 0); }
  } else {
    let yShift = 0;
    for (const S of sheets) {
      for (const [kind, l, d] of S.items) {
        const L = l.toUpperCase();
        if (kind === 'L') { p(0, 'LINE'); p(8, L); p(10, d[0] * k); p(20, (d[1] + yShift) * k); p(30, 0); p(11, d[2] * k); p(21, (d[3] + yShift) * k); p(31, 0); }
        else if (kind === 'C') { p(0, 'CIRCLE'); p(8, L); p(10, d[0] * k); p(20, (d[1] + yShift) * k); p(30, 0); p(40, d[2] * k); }
        else if (kind === 'T') { p(0, 'SOLID'); p(8, L); const q = [d[0], d[1], d[2], d[2]]; q.forEach((pt, i) => { p(10 + i, pt[0] * k); p(20 + i, (pt[1] + yShift) * k); p(30 + i, 0); }); }
        else if (kind === 'X') {
          const [x, y, s, h, ha, va, rot] = d;
          const hj = { left: 0, center: 1, right: 2 }[ha]; const vj = { bottom: 0, center: 2, top: 3 }[va];
          p(0, 'TEXT'); p(8, L); p(10, x * k); p(20, (y + yShift) * k); p(30, 0); p(40, h * k * 0.75); p(1, s.replace(/Ø/g, '%%c').replace(/×/g, 'x').replace(/³/g, '3'));
          if (rot) p(50, rot);
          if (hj || vj) { p(72, hj); p(11, x * k); p(21, (y + yShift) * k); p(31, 0); p(73, vj); }
        }
      }
      yShift -= S.h + 20;
    }
  }
  p(0, 'ENDSEC'); p(0, 'EOF');
  return o.join('\n');
}

export const safeName = (n) => n.replace(/[^A-Za-z0-9_\-]+/g, '_').replace(/^_+|_+$/g, '').slice(0, 60) || 'part';

// ---------------------------------------------------------------- file readers
export function parseSTL(buf) {
  const u8 = new Uint8Array(buf);
  const head = new TextDecoder().decode(u8.slice(0, 512)).toLowerCase();
  const dv = new DataView(buf);
  const nBin = u8.length >= 84 ? dv.getUint32(80, true) : 0;
  const isBinary = u8.length === 84 + nBin * 50 || !head.trimStart().startsWith('solid') || !head.includes('facet');
  if (isBinary) {
    const pos = new Float32Array(nBin * 9);
    for (let i = 0; i < nBin; i++) {
      const o = 84 + i * 50 + 12;
      for (let k = 0; k < 9; k++) pos[i * 9 + k] = dv.getFloat32(o + 4 * k, true);
    }
    return pos;
  }
  const txt = new TextDecoder().decode(u8);
  const re = /vertex\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)/g; const pos = []; let mm;
  while ((mm = re.exec(txt))) pos.push(+mm[1], +mm[2], +mm[3]);
  return Float32Array.from(pos);
}

export function stepInstances(res) {
  const out = [];
  const names = new Map();
  const walk = (node) => { for (const i of node.meshes || []) names.set(i, node.name); for (const c of node.children || []) walk(c); };
  if (res.root) walk(res.root);
  res.meshes.forEach((m, i) => {
    const pos = m.attributes.position.array; const idx = m.index ? m.index.array : null;
    const nt = idx ? idx.length / 3 : pos.length / 9;
    const fid = new Int32Array(nt);
    (m.brep_faces || []).forEach((f, fi) => { for (let t = f.first; t <= f.last && t < nt; t++) fid[t] = fi; });
    out.push({ name: m.name || names.get(i) || '', mesh: weld(pos, idx, m.brep_faces && m.brep_faces.length ? fid : null) });
  });
  return out;
}
