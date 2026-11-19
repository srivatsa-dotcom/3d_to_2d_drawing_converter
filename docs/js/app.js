import * as C from './core.js';

const $ = (s) => document.querySelector(s);
const state = { parts: [], src: '', kind: 'step', current: null, cache: new Map(), busy: false };
const today = new Date().toISOString().slice(0, 10);

const opts = () => ({
  sheet: $('#opt-sheet').value,
  projection: $('#opt-proj').value,
  hidden: $('#opt-hidden').checked,
  material: $('#opt-material').value.trim(),
  tolerance: $('#opt-tol').value.trim(),
  source: state.src,
  date: today,
  kind: state.kind,
});
const optKey = () => JSON.stringify(opts());

function status(msg, kind = '') {
  const el = $('#status');
  el.textContent = msg;
  el.dataset.kind = kind;
}

let occtPromise = null;
function occt() {
  if (!occtPromise) {
    occtPromise = window.occtimportjs({ locateFile: (f) => `vendor/${f}` });
  }
  return occtPromise;
}

// ---------------------------------------------------------------- loading
async function loadFile(name, buf) {
  if (state.busy) return;
  state.busy = true;
  setBusy(true);
  try {
    const ext = name.split('.').pop().toLowerCase();
    state.src = name;
    state.cache.clear();
    let inst;
    if (ext === 'stl') {
      status('Reading STL mesh…');
      await tick();
      state.kind = 'stl';
      const pos = C.parseSTL(buf);
      if (!pos.length) throw new Error('This STL file has no triangles.');
      inst = C.splitComponents(C.weld(pos, null, null)).map((m) => ({ name: '', mesh: m }));
    } else if (['step', 'stp', 'iges', 'igs', 'brep'].includes(ext)) {
      status('Loading the STEP reader (7 MB, first time only)…');
      const o = await occt();
      status('Reading CAD file…');
      await tick();
      state.kind = 'step';
      const params = { linearUnit: 'millimeter', linearDeflectionType: 'bounding_box_ratio', linearDeflection: 0.001, angularDeflection: 0.5 };
      const data = new Uint8Array(buf);
      const res = ext === 'step' || ext === 'stp' ? o.ReadStepFile(data, params)
        : ext === 'brep' ? o.ReadBrepFile(data, params) : o.ReadIgesFile(data, params);
      if (!res.success || !res.meshes.length) throw new Error('Could not read this CAD file. Check that it is a valid STEP, IGES or BREP file.');
      inst = C.stepInstances(res);
    } else {
      throw new Error('Use a .step, .stp, .iges, .igs, .brep or .stl file.');
    }
    status(`Splitting ${inst.length} bodies into parts…`);
    await tick();
    const parts = C.makeParts(inst, { kind: state.kind });
    const crease = +$('#opt-crease').value || 25;
    let item = 0;
    for (const p of parts) {
      C.analysePart(p, crease);
      if (!p.hardware) p.item = String(++item).padStart(2, '0');
      await tick();
    }
    state.parts = parts;
    renderParts();
    const first = parts.find((p) => !p.hardware) || parts[0];
    const made = parts.filter((p) => !p.hardware).length;
    status(`${made} part drawing${made === 1 ? '' : 's'} from ${inst.length} bodies in ${name}.`, 'ok');
    state.busy = false;
    setBusy(false);
    if (first) await showPart(first);
  } catch (e) {
    console.error(e);
    status(e.message || String(e), 'error');
  } finally {
    state.busy = false;
    setBusy(false);
  }
}

const tick = () => new Promise((r) => setTimeout(r, 0));

function setBusy(b) {
  document.body.classList.toggle('busy', b);
  for (const el of document.querySelectorAll('[data-needs-model]')) el.disabled = b || !state.parts.length;
}

// ---------------------------------------------------------------- parts list
function renderParts() {
  const tb = $('#parts tbody');
  tb.innerHTML = '';
  for (const p of state.parts) {
    const tr = document.createElement('tr');
    tr.className = p.hardware ? 'hw' : '';
    tr.innerHTML = `<td>${p.hardware ? 'HW' : p.item}</td><td class="name"></td><td>${p.qty}</td><td class="size">${p.ext.map(C.fmt).join('×')}</td><td>${p.hardware ? '–' : p.holes.length}</td>`;
    tr.querySelector('.name').textContent = p.name;
    tr.title = C.partType(p);
    if (!p.hardware) {
      tr.tabIndex = 0;
      tr.addEventListener('click', () => showPart(p));
      tr.addEventListener('keydown', (e) => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); showPart(p); } });
    }
    p.row = tr;
    tb.appendChild(tr);
  }
  $('#parts-count').textContent = `${state.parts.length} unique part${state.parts.length === 1 ? '' : 's'}`;
  setBusy(false);
}

async function sheetFor(p) {
  const key = p.item + optKey();
  if (!state.cache.has(key)) state.cache.set(key, await C.makeSheet(p, { ...opts(), item: p.item }));
  return state.cache.get(key);
}

async function showPart(p) {
  if (state.busy && state.current === p) return;
  state.current = p;
  for (const q of state.parts) q.row && q.row.classList.toggle('active', q === p);
  $('#sheet-title').textContent = `${p.item}  ${p.name}`;
  $('#sheet-meta').textContent = `${p.qty} off, ${C.partType(p).toLowerCase()}`;
  status(`Drawing ${p.name}…`);
  document.body.classList.add('drawing');
  await tick();
  try {
    const r = await sheetFor(p);
    if (state.current !== p) return;
    $('#sheet').innerHTML = C.toSVG(r.sheets[0]);
    resetView();
    $('#dl-cut').hidden = !r.cut;
    $('#more-sheets').textContent = r.sheets.length > 1 ? `+${r.sheets.length - 1} hole-table sheet(s) in the PDF` : '';
    status(`${p.name}: scale ${C.scaleLabel(r.scale)}, ${r.rows.length} hole${r.rows.length === 1 ? '' : 's'}.`, 'ok');
  } catch (e) {
    console.error(e);
    status(`Could not draw ${p.name}: ${e.message}`, 'error');
  } finally {
    document.body.classList.remove('drawing');
  }
}

// ---------------------------------------------------------------- zoom / pan
const vb = { x: 0, y: 0, w: 420, h: 297 };
function resetView() {
  const [W, H] = C.SHEETS[$('#opt-sheet').value];
  Object.assign(vb, { x: 0, y: 0, w: W, h: H });
  applyVB();
}
function applyVB() {
  const svg = $('#sheet svg');
  if (svg) svg.setAttribute('viewBox', `${vb.x} ${vb.y} ${vb.w} ${vb.h}`), svg.removeAttribute('width'), svg.removeAttribute('height');
}
function setupZoom() {
  const el = $('#sheet');
  el.addEventListener('wheel', (e) => {
    const svg = el.querySelector('svg'); if (!svg) return;
    e.preventDefault();
    const r = svg.getBoundingClientRect();
    const fx = (e.clientX - r.left) / r.width, fy = (e.clientY - r.top) / r.height;
    const k = Math.exp(e.deltaY * 0.0015);
    const [W] = C.SHEETS[$('#opt-sheet').value];
    const nw = Math.min(W, Math.max(20, vb.w * k)); const s = nw / vb.w;
    vb.x += (vb.w - nw) * fx; vb.y += (vb.h - vb.h * s) * fy; vb.w = nw; vb.h *= s;
    applyVB();
  }, { passive: false });
  let drag = null;
  el.addEventListener('pointerdown', (e) => { drag = { x: e.clientX, y: e.clientY, vx: vb.x, vy: vb.y }; el.setPointerCapture(e.pointerId); });
  el.addEventListener('pointermove', (e) => {
    if (!drag) return; const svg = el.querySelector('svg'); if (!svg) return;
    const r = svg.getBoundingClientRect();
    vb.x = drag.vx - (e.clientX - drag.x) * vb.w / r.width; vb.y = drag.vy - (e.clientY - drag.y) * vb.h / r.height; applyVB();
  });
  el.addEventListener('pointerup', () => { drag = null; });
  el.addEventListener('dblclick', resetView);
  $('#fit').addEventListener('click', resetView);
}

// ---------------------------------------------------------------- downloads
function save(blob, name) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob); a.download = name;
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 4000);
}
const base = (p) => `${p.item}_${C.safeName(p.name)}`;
function pdfOf(sheets) {
  const [W, H] = C.SHEETS[$('#opt-sheet').value];
  const doc = new window.jspdf.jsPDF({ orientation: 'landscape', unit: 'mm', format: [W, H] });
  sheets.forEach((S, i) => { if (i) doc.addPage([W, H], 'landscape'); C.drawPDF(doc, C.pdfReady(S)); });
  return doc;
}
async function dl(kind) {
  const p = state.current; if (!p) return;
  const r = await sheetFor(p);
  if (kind === 'pdf') save(pdfOf(r.sheets).output('blob'), `${base(p)}.pdf`);
  if (kind === 'svg') save(new Blob([C.toSVG(r.sheets[0])], { type: 'image/svg+xml' }), `${base(p)}.svg`);
  if (kind === 'dxf') save(new Blob([C.toDXF(r.sheets, r.scale)], { type: 'application/dxf' }), `${base(p)}.dxf`);
  if (kind === 'cut' && r.cut) save(new Blob([C.toDXF(null, 1, r.cut)], { type: 'application/dxf' }), `${base(p)}_CUT.dxf`);
}

async function dlAll() {
  if (!state.parts.length || state.busy) return;
  state.busy = true; setBusy(true);
  try {
    const zip = new window.JSZip();
    const made = state.parts.filter((p) => !p.hardware);
    const all = [C.bomSheet(state.parts, state.src, $('#opt-sheet').value, today)];
    const csv = [['item', 'part', 'qty', 'size_x_mm', 'size_y_mm', 'size_z_mm', 'holes', 'volume_cm3', 'type']];
    for (let i = 0; i < made.length; i++) {
      const p = made[i];
      status(`Drawing ${i + 1} of ${made.length}: ${p.name}…`);
      await tick();
      const r = await sheetFor(p);
      all.push(...r.sheets);
      zip.file(`${base(p)}.pdf`, pdfOf(r.sheets).output('arraybuffer'));
      zip.file(`${base(p)}.dxf`, C.toDXF(r.sheets, r.scale));
      zip.file(`${base(p)}.svg`, C.toSVG(r.sheets[0]));
      if (r.cut) zip.file(`${base(p)}_CUT.dxf`, C.toDXF(null, 1, r.cut));
    }
    for (const p of state.parts) {
      csv.push([p.hardware ? 'HW' : p.item, p.name, p.qty, ...p.ext.map(C.fmt), p.hardware ? '' : p.holes.length, C.fmt(p.vol / 1000), C.partType(p)]);
    }
    zip.file('BOM.csv', csv.map((r) => r.map((v) => `"${String(v).replace(/"/g, '""')}"`).join(',')).join('\n'));
    status('Building ALL_DRAWINGS.pdf…'); await tick();
    zip.file('ALL_DRAWINGS.pdf', pdfOf(all).output('arraybuffer'));
    const blob = await zip.generateAsync({ type: 'blob' });
    save(blob, `${C.safeName(state.src.replace(/\.[^.]+$/, ''))}_drawings.zip`);
    status(`Saved ${made.length} drawings, the BOM and ALL_DRAWINGS.pdf as a ZIP.`, 'ok');
  } catch (e) {
    console.error(e); status(`Could not build the ZIP: ${e.message}`, 'error');
  } finally {
    state.busy = false; setBusy(false);
  }
}

// ---------------------------------------------------------------- wiring
function setup() {
  const drop = $('#drop');
  const input = $('#file');
  input.addEventListener('change', async () => {
    const f = input.files[0]; if (f) loadFile(f.name, await f.arrayBuffer());
    input.value = '';
  });
  ['dragenter', 'dragover'].forEach((ev) => drop.addEventListener(ev, (e) => { e.preventDefault(); drop.classList.add('over'); }));
  ['dragleave', 'drop'].forEach((ev) => drop.addEventListener(ev, (e) => { e.preventDefault(); drop.classList.remove('over'); }));
  drop.addEventListener('drop', async (e) => {
    const f = e.dataTransfer.files[0]; if (f) loadFile(f.name, await f.arrayBuffer());
  });
  // allow dropping anywhere on the page
  window.addEventListener('dragover', (e) => e.preventDefault());
  window.addEventListener('drop', async (e) => {
    e.preventDefault();
    if (e.target.closest && e.target.closest('#drop')) return;
    const f = e.dataTransfer.files[0]; if (f) loadFile(f.name, await f.arrayBuffer());
  });
  $('#sample').addEventListener('click', loadSample);
  for (const id of ['#opt-sheet', '#opt-proj', '#opt-hidden', '#opt-material', '#opt-tol']) {
    $(id).addEventListener('change', () => state.current && showPart(state.current));
  }
  $('#opt-crease').addEventListener('change', () => {
    if (!state.parts.length) return;
    const crease = +$('#opt-crease').value || 25;
    state.cache.clear();
    for (const p of state.parts) C.analysePart(p, crease);
    renderParts();
    state.current && showPart(state.current);
  });
  $('#dl-pdf').addEventListener('click', () => dl('pdf'));
  $('#dl-dxf').addEventListener('click', () => dl('dxf'));
  $('#dl-svg').addEventListener('click', () => dl('svg'));
  $('#dl-cut').addEventListener('click', () => dl('cut'));
  $('#dl-all').addEventListener('click', dlAll);
  setupZoom();
  setBusy(false);
  loadSample();
}

async function loadSample() {
  try {
    status('Loading the sample robot…');
    const r = await fetch('samples/sample_robot.step');
    await loadFile('sample_robot.step', await r.arrayBuffer());
  } catch (e) {
    status('Drop a STEP or STL file to start.');
  }
}

setup();
