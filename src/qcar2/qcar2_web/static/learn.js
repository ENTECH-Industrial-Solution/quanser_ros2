// "เรียนรู้" page: collect RealSense images, label boxes, train YOLO, use the trained detector.
// Loaded after index.html's main script; uses robot_web_ui's core.js (window.RobotUI).
(() => {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const root = $('learn-box');
  if (!root) return;
  const esc = (s) => String(s).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  const COLORS = ['#ff3838', '#ff9d97', '#ff701f', '#ffb21d', '#cfd231', '#48f90a', '#92cc17', '#3ddb86',
                  '#1a9334', '#00d4bb', '#2c99a8', '#00c2ff', '#344593', '#6473ff', '#0018ec'];
  const color = (c) => COLORS[c % COLORS.length];
  let ds = null, st = null, classes = [], images = [], imgSig = '', filter = 'all';
  let cur = -1;          // index into the filtered image list
  let curCls = 0;
  let on = false, pollT = null;
  try { ds = localStorage.getItem('qcar2.learn.ds'); filter = localStorage.getItem('qcar2.learn.filter') || 'all'; } catch (e) {}

  async function api(path, body) {
    const r = await fetch('/api/learn' + path, body === undefined ? {} : {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ ds, ...body }) });
    if (!r.ok) throw new Error((await r.text()) || r.statusText);
    return r.json();
  }
  function toast(msg, bad) {
    const t = $('learn-toast');
    t.textContent = msg; t.className = 'learn-toast show' + (bad ? ' bad' : '');
    clearTimeout(toast.t); toast.t = setTimeout(() => t.classList.remove('show'), bad ? 5000 : 2200);
  }
  const fail = (e) => toast(String(e.message || e), true);

  // ------------------------------------------------------------ status polling
  async function poll() {
    clearTimeout(pollT);
    if (!on) return;
    try {
      const s = await api('?ds=' + encodeURIComponent(ds || ''));
      st = s;
      if (ds !== s.ds) { ds = s.ds; try { localStorage.setItem('qcar2.learn.ds', ds); } catch (e) {} }
      if (JSON.stringify(classes) !== JSON.stringify(s.classes)) { classes = s.classes; renderClasses(); draw(); }
      const sig = ds + ':' + s.counts.images + ':' + s.counts.labelled + ':' + s.counts.boxes + ':' + s.counts.pending;
      if (sig !== imgSig) { imgSig = sig; await loadImages(); }
      renderStatus();
    } catch (e) { $('learn-cam-state').textContent = 'ติดต่อ server ไม่ได้: ' + e.message; }
    pollT = setTimeout(poll, st && st.job ? 1000 : 1500);
  }
  async function loadImages() {
    try { images = (await api('/images?ds=' + encodeURIComponent(ds))).images; } catch (e) { images = []; }
    renderGallery();
    if (cur < 0 || !lab.id) { const list = shown(); if (list.length && !lab.id) openImage(list.length - 1); }
  }
  let camFilter = '';
  const CAM_SHORT = { realsense: 'RS', front: 'หน้า', back: 'หลัง', left: 'ซ้าย', right: 'ขวา' };
  const shown = () => images.filter(im => (!camFilter || im.cam === camFilter) && (filter === 'all' || (filter === 'todo' ? !im.labelled
    : filter === 'review' ? im.pending : im.labelled)));

  function renderStatus() {
    const s = st;
    // datasets
    const sel = $('learn-ds');
    if (sel.dataset.sig !== s.datasets.join('|')) {
      sel.innerHTML = s.datasets.map(n => `<option value="${esc(n)}">${esc(n)}</option>`).join('');
      sel.dataset.sig = s.datasets.join('|');
    }
    sel.value = s.ds;
    const c = s.counts;
    $('learn-counts').textContent = `${c.images} ภาพ · label แล้ว ${c.labelled} · ${c.boxes} กรอบ` + (c.pending ? ` · รอตรวจ ${c.pending}` : '');
    renderHelper();
    renderCams(s.camera);
    const a = s.auto;
    $('learn-auto').classList.toggle('on', a.on);
    $('learn-auto').textContent = a.on ? 'หยุดถ่ายอัตโนมัติ' : 'ถ่ายอัตโนมัติ';
    $('learn-auto-state').textContent = a.on ? `บันทึก ${a.saved} · ข้าม ${a.skipped}${a.reason ? ' (ล่าสุด: ' + a.reason + ')' : ''}`
      : (a.saved || a.skipped ? `รอบล่าสุด: บันทึก ${a.saved} · ข้าม ${a.skipped}` : '') + (a.reason && !a.on && !a.saved ? ' ' + a.reason : '');
    renderJob(); renderRuns(); renderDetect();
  }

  // ------------------------------------------------------------ dataset + classes
  $('learn-ds').onchange = (e) => { ds = e.target.value; try { localStorage.setItem('qcar2.learn.ds', ds); } catch (err) {} reset(); poll(); };
  $('learn-ds-new').onclick = async () => {
    const name = prompt('ชื่อชุดข้อมูลใหม่ (ตัวอักษร ตัวเลข _ -)');
    if (!name) return;
    try { await api('/dataset', { name: name.trim() }); ds = name.trim(); reset(); poll(); } catch (e) { fail(e); }
  };
  $('learn-ds-del').onclick = async () => {
    if (!confirm(`ลบชุดข้อมูล "${ds}" ทั้งภาพ label และโมเดลที่ train ไว้? ย้อนกลับไม่ได้`)) return;
    try { await api('/dataset', { delete: true }); ds = null; reset(); poll(); } catch (e) { fail(e); }
  };
  function reset() { images = []; imgSig = ''; cur = -1; lab.id = null; lab.boxes = []; lab.img = null; classes = []; renderGallery(); draw(); }

  function renderClasses() {
    if (curCls >= classes.length) curCls = Math.max(0, classes.length - 1);
    const box = $('learn-classes');
    box.innerHTML = classes.map((c, i) => `<button class="lcls${i === curCls ? ' on' : ''}" data-i="${i}" title="กด ${i + 1} เพื่อเลือก · ดับเบิลคลิก = เปลี่ยนชื่อ">
      <i style="background:${color(i)}"></i>${i < 9 ? `<kbd>${i + 1}</kbd>` : ''}${esc(c)}</button>`).join('')
      + `<button id="learn-cls-add" title="เพิ่ม class">+ class</button>`;
    box.querySelectorAll('.lcls').forEach(b => {
      b.onclick = () => pickClass(+b.dataset.i);
      b.ondblclick = () => editClass(+b.dataset.i);
    });
    $('learn-cls-add').onclick = addClass;
    const counts = st ? st.counts.per_class_boxes : {};
    $('learn-cls-list').innerHTML = classes.length ? classes.map((c, i) => `<div class="lcls-row">
      <i style="background:${color(i)}"></i><span>${esc(c)}</span><small>${counts[i] || 0} กรอบ</small>
      <button data-r="${i}">เปลี่ยนชื่อ</button><button data-d="${i}">ลบ</button></div>`).join('')
      : '<p class="hint">ยังไม่มี class เพิ่มอย่างน้อย 1 อย่าง เช่น "กรวย", "ป้ายหยุด"</p>';
    $('learn-cls-list').querySelectorAll('[data-r]').forEach(b => b.onclick = () => editClass(+b.dataset.r));
    $('learn-cls-list').querySelectorAll('[data-d]').forEach(b => b.onclick = async () => {
      const i = +b.dataset.d;
      if (!confirm(`ลบ class "${classes[i]}" และกรอบทั้งหมดของ class นี้?`)) return;
      try { await saveIfDirty(); classes = (await api('/classes', { op: 'delete', i })).classes; renderClasses(); imgSig = ''; poll(); if (lab.id) openId(lab.id); }
      catch (e) { fail(e); }
    });
  }
  async function addClass(name) {
    if (typeof name !== 'string') name = prompt('ชื่อ class ใหม่ (เช่น กรวย, ป้ายหยุด, cone)');
    if (!name || !name.trim()) return null;
    try {
      classes = (await api('/classes', { op: 'add', name: name.trim() })).classes;
      curCls = classes.length - 1; renderClasses(); return curCls;
    } catch (e) { fail(e); return null; }
  }
  async function editClass(i) {
    const name = prompt('ชื่อใหม่ของ class', classes[i]);
    if (!name || name.trim() === classes[i]) return;
    try { classes = (await api('/classes', { op: 'rename', i, name: name.trim() })).classes; renderClasses(); draw(); } catch (e) { fail(e); }
  }
  function pickClass(i) {
    if (i < 0 || i >= classes.length) return;
    curCls = i;
    // re-class only an object the user clicked on; a just-drawn one stays as drawn
    if (lab.sel != null && lab.picked) { push(); lab.boxes[lab.sel][0] = i; dirty(); }
    renderClasses(); draw();
  }

  // ------------------------------------------------------------ gallery
  document.querySelectorAll('#learn-filter button').forEach(b => {
    b.classList.toggle('on', b.dataset.f === filter);
    b.onclick = () => {
      filter = b.dataset.f; try { localStorage.setItem('qcar2.learn.filter', filter); } catch (e) {}
      document.querySelectorAll('#learn-filter button').forEach(x => x.classList.toggle('on', x === b));
      renderGallery();
    };
  });
  function renderGallery() {
    const list = shown(), g = $('learn-gallery');
    cur = lab.id ? list.findIndex(im => im.id === lab.id) : -1;
    $('learn-gal-n').textContent = `${list.length} ภาพ`;
    if (!list.length) { g.innerHTML = `<p class="hint" style="padding:10px">${images.length ? 'ไม่มีภาพในตัวกรองนี้' : 'ยังไม่มีภาพ กด "ถ่ายภาพ" หรือ "ถ่ายอัตโนมัติ" ทางขวา'}</p>`; return; }
    g.innerHTML = list.map((im, k) => `<button class="lthumb${im.id === lab.id ? ' on' : ''}" data-k="${k}" title="${esc(im.id)}${im.val ? ' (ใช้ตรวจผล)' : ''}">
      <img loading="lazy" src="/api/learn/image/${encodeURIComponent(ds)}/${im.id}.jpg?thumb=1" alt="">
      <span class="${im.labelled ? (im.n ? 'ok' : 'empty') : im.pending ? 'review' : 'todo'}">${im.labelled ? (im.n ? im.n + ' กรอบ' : 'ไม่มีวัตถุ') : im.pending ? `รอตรวจ ${im.np}` : 'ยังไม่ label'}</span>
      ${im.val ? '<b title="ภาพชุดตรวจผล (validation)">V</b>' : ''}<i class="lcam">${esc(CAM_SHORT[im.cam] || im.cam)}</i></button>`).join('');
    g.querySelectorAll('.lthumb').forEach(b => b.onclick = () => openImage(+b.dataset.k));
    const on_ = g.querySelector('.lthumb.on');
    if (on_) on_.scrollIntoView({ block: 'nearest', inline: 'nearest' });
  }

  // ------------------------------------------------------------ labeler
  const cv = $('learn-canvas'), ctx = cv.getContext('2d');
  const lab = { id: null, img: null, boxes: [], sel: null, dirty: false, labelled: false, undo: [], drag: null, hover: null,
                draft: [], mask: null, painted: false, painting: null };
  // An object is [class, cx, cy, w, h] (0..1) plus, for a shape, .poly = [[x, y], ...] (0..1);
  // its box is then the outline's bounding box. .sug = confidence of a suggestion.
  const mk = (b, poly, sug) => { const nb = b.slice(0, 5); if (poly) nb.poly = poly.map(p => p.slice()); if (sug) nb.sug = sug; return nb; };
  const polyBox = (P) => { const xs = P.map(p => p[0]), ys = P.map(p => p[1]);
    const l = Math.min(...xs), r = Math.max(...xs), t = Math.min(...ys), b = Math.max(...ys);
    return [(l + r) / 2, (t + b) / 2, r - l, b - t]; };
  const ser = () => JSON.stringify(lab.boxes.map(b => [b.slice(0, 5), b.poly || null, b.sug || 0]));
  const deser = (txt) => JSON.parse(txt).map(([b, p, g]) => mk(b, p, g));
  let tool = 'box', brushPx = 24, wandTol = 20;
  try { tool = localStorage.getItem('qcar2.learn.tool') || 'box'; brushPx = +(localStorage.getItem('qcar2.learn.brush') || 24);
        wandTol = +(localStorage.getItem('qcar2.learn.tol') || 20); } catch (e) {}
  const fit = () => {
    const r = cv.getBoundingClientRect(), dpr = devicePixelRatio || 1;
    const W = Math.max(1, Math.round(r.width * dpr)), H = Math.max(1, Math.round(r.height * dpr));
    if (cv.width !== W || cv.height !== H) { cv.width = W; cv.height = H; }
    if (!lab.img) return null;
    const iw = lab.img.naturalWidth, ih = lab.img.naturalHeight;
    const s = Math.min(r.width / iw, r.height / ih);
    return { s, ox: (r.width - iw * s) / 2, oy: (r.height - ih * s) / 2, iw, ih, dpr };
  };
  // box [c, cx, cy, w, h] (0..1) <-> screen rect
  const toRect = (b, f) => ({ x: f.ox + (b[1] - b[3] / 2) * f.iw * f.s, y: f.oy + (b[2] - b[4] / 2) * f.ih * f.s, w: b[3] * f.iw * f.s, h: b[4] * f.ih * f.s });
  const toNorm = (x, y, f) => [Math.min(1, Math.max(0, (x - f.ox) / (f.iw * f.s))), Math.min(1, Math.max(0, (y - f.oy) / (f.ih * f.s)))];
  function draw() {
    const f = fit(), r = cv.getBoundingClientRect();
    ctx.setTransform(f ? f.dpr : devicePixelRatio || 1, 0, 0, f ? f.dpr : devicePixelRatio || 1, 0, 0);
    ctx.clearRect(0, 0, r.width, r.height);
    if (!f) {
      ctx.fillStyle = '#8b95a3'; ctx.font = '14px system-ui'; ctx.textAlign = 'center';
      ctx.fillText(lab.id ? 'กำลังโหลดภาพ…' : 'เลือกภาพจากแถบด้านล่าง หรือถ่ายภาพใหม่', r.width / 2, r.height / 2);
      return;
    }
    ctx.drawImage(lab.img, f.ox, f.oy, f.iw * f.s, f.ih * f.s);
    const S = (p) => [f.ox + p[0] * f.iw * f.s, f.oy + p[1] * f.ih * f.s];
    if (lab.mask && lab.painted) { ctx.globalAlpha = 0.5; ctx.drawImage(lab.mask, f.ox, f.oy, f.iw * f.s, f.ih * f.s); ctx.globalAlpha = 1; }
    ctx.font = '600 12px system-ui'; ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
    lab.boxes.forEach((b, k) => {
      const q = toRect(b, f), col = color(b[0]), sel = k === lab.sel;
      if (b.poly) {   // a shape: filled outline, its box faint
        ctx.beginPath(); b.poly.forEach((p, j) => { const [sx, sy] = S(p); j ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy); }); ctx.closePath();
        ctx.fillStyle = col + (sel ? '66' : '3a'); ctx.fill();
        ctx.lineWidth = sel ? 2.5 : 1.8; ctx.strokeStyle = col; ctx.setLineDash(b.sug ? [6, 4] : []); ctx.stroke();
        ctx.lineWidth = 1; ctx.setLineDash([3, 3]); ctx.strokeStyle = col + '99';
        ctx.strokeRect(q.x, q.y, q.w, q.h); ctx.setLineDash([]);
      } else {
        ctx.lineWidth = sel ? 3 : 2; ctx.strokeStyle = col;
        ctx.setLineDash(b.sug ? [6, 4] : []);
        ctx.strokeRect(q.x, q.y, q.w, q.h);
        ctx.setLineDash([]);
        if (sel) { ctx.fillStyle = col + '22'; ctx.fillRect(q.x, q.y, q.w, q.h); }
      }
      const name = (classes[b[0]] || '?') + (b.sug ? ` ${Math.round(b.sug * 100)}%` : '');
      const tw = ctx.measureText(name).width + 8, ty = q.y > 18 ? q.y : q.y + 18;
      ctx.fillStyle = col; ctx.fillRect(q.x - (sel ? 1.5 : 1), ty - 17, tw, 17);
      ctx.fillStyle = '#000'; ctx.fillText(name, q.x + 3, ty - 3);
      if (sel) {
        ctx.fillStyle = '#fff';
        handles(q).forEach(([hx, hy]) => { ctx.fillRect(hx - 4, hy - 4, 8, 8); ctx.strokeRect(hx - 4, hy - 4, 8, 8); });
      }
    });
    const d = lab.drag;
    if (d && d.kind === 'new') {
      ctx.strokeStyle = color(curCls); ctx.lineWidth = 2; ctx.setLineDash([5, 3]);
      ctx.strokeRect(Math.min(d.x0, d.x1), Math.min(d.y0, d.y1), Math.abs(d.x1 - d.x0), Math.abs(d.y1 - d.y0));
      ctx.setLineDash([]);
    }
    if (lab.draft.length) {   // polygon being drawn
      const col = color(curCls);
      ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.beginPath();
      lab.draft.forEach((p, j) => { const [sx, sy] = S(p); j ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy); });
      if (lab.hover) ctx.lineTo(lab.hover[0], lab.hover[1]);
      ctx.stroke();
      lab.draft.forEach((p, j) => { const [sx, sy] = S(p); ctx.fillStyle = j ? '#fff' : col;
        ctx.beginPath(); ctx.arc(sx, sy, j ? 3.5 : 6, 0, 7); ctx.fill(); ctx.strokeStyle = '#000'; ctx.lineWidth = 1; ctx.stroke(); });
    }
    if (lab.hover && tool === 'brush') {
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(lab.hover[0], lab.hover[1], brushPx / 2, 0, 7); ctx.stroke();
    }
    if (lab.hover && !d && classes.length && (tool === 'box' || tool === 'poly')) {   // crosshair to aim the corner
      ctx.strokeStyle = 'rgba(255,255,255,.35)'; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(lab.hover[0], f.oy); ctx.lineTo(lab.hover[0], f.oy + f.ih * f.s);
      ctx.moveTo(f.ox, lab.hover[1]); ctx.lineTo(f.ox + f.iw * f.s, lab.hover[1]); ctx.stroke();
    }
    const im = images.find(x => x.id === lab.id);
    $('learn-lab-state').textContent = `${shown().indexOf(im) + 1}/${shown().length} · ` +
      (lab.draft.length ? `polygon ${lab.draft.length} จุด: คลิกจุดแรก / ดับเบิลคลิก / Enter = ปิดรูป · Backspace = ลบจุดล่าสุด` :
       lab.painted ? 'ระบายอยู่: Enter = สร้างรูปร่างจากที่ระบาย · Esc = ล้าง' :
       lab.dirty ? 'ยังไม่บันทึก' : lab.pending ? 'กรอบจากตัวช่วย รอตรวจ: ถูกแล้วกด S · ผิดก็แก้/ลบก่อน' : lab.labelled ? (lab.boxes.length ? 'บันทึกแล้ว' : 'บันทึกว่า "ไม่มีวัตถุ"') : 'ยังไม่ label') +
      (im && im.val ? ' · ภาพชุดตรวจผล' : '');
    $('learn-save').classList.toggle('on', lab.dirty);
  }
  const handles = (q) => [[q.x, q.y], [q.x + q.w, q.y], [q.x, q.y + q.h], [q.x + q.w, q.y + q.h],
                          [q.x + q.w / 2, q.y], [q.x + q.w / 2, q.y + q.h], [q.x, q.y + q.h / 2], [q.x + q.w, q.y + q.h / 2]];
  const HANDLE_MODE = ['nw', 'ne', 'sw', 'se', 'n', 's', 'w', 'e'];
  // The selected box: its corners/edges resize it, its inside moves it.
  function hitSel(x, y, f) {
    if (lab.sel == null || !lab.boxes[lab.sel]) return null;
    const q = toRect(lab.boxes[lab.sel], f);
    const k = handles(q).findIndex(([hx, hy]) => Math.abs(hx - x) < 9 && Math.abs(hy - y) < 9);
    if (k >= 0) return { box: lab.sel, mode: HANDLE_MODE[k] };
    return x >= q.x && x <= q.x + q.w && y >= q.y && y <= q.y + q.h ? { box: lab.sel, mode: 'move' } : null;
  }
  function hit(x, y, f) {
    const h = hitSel(x, y, f);
    if (h) return h;
    // smallest box under the pointer (nested boxes stay reachable)
    let best = null, area = Infinity;
    lab.boxes.forEach((b, k) => {
      const q = toRect(b, f);
      if (x >= q.x && x <= q.x + q.w && y >= q.y && y <= q.y + q.h && q.w * q.h < area) { best = k; area = q.w * q.h; }
    });
    return best == null ? null : { box: best, mode: 'move' };
  }
  const CURSOR = { nw: 'nwse-resize', se: 'nwse-resize', ne: 'nesw-resize', sw: 'nesw-resize', n: 'ns-resize', s: 'ns-resize', w: 'ew-resize', e: 'ew-resize', move: 'move' };
  const pos = (e) => { const r = cv.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; };
  const push = () => { lab.undo.push(ser()); if (lab.undo.length > 50) lab.undo.shift(); };
  const dirty = () => { lab.dirty = true; };
  cv.addEventListener('pointerdown', (e) => {
    const f = fit(); if (!f || e.button > 0) return;
    const [x, y] = pos(e);
    try { cv.setPointerCapture(e.pointerId); } catch (err) {}
    if (tool === 'poly') { polyClick(x, y, f); return; }
    if (tool === 'brush') { paintStart(e, x, y, f); return; }
    if (tool === 'wand') { wandClick(e, x, y, f); return; }
    // Dragging always draws a new box, also on top of other boxes (a face inside a person);
    // only the selected box moves. A click without dragging selects the box under the pointer.
    const h = hitSel(x, y, f);
    if (h) {
      push();
      const b0 = lab.boxes[h.box];
      lab.drag = { kind: h.mode, x0: x, y0: y, orig: b0.slice(0, 5), origPoly: b0.poly ? b0.poly.map(p => p.slice()) : null, moved: false };
    } else {
      const under = hit(x, y, f);
      lab.drag = { kind: 'new', x0: x, y0: y, x1: x, y1: y, pick: under ? under.box : null };
    }
    draw();
  });
  cv.addEventListener('pointermove', (e) => {
    const f = fit(); if (!f) return;
    const [x, y] = pos(e);
    lab.hover = [x, y];
    if (lab.painting) { paintTo(x, y, f); draw(); return; }
    if (tool !== 'box') { cv.style.cursor = tool === 'brush' ? 'none' : 'crosshair'; draw(); return; }
    const d = lab.drag;
    if (!d) { const h = hitSel(x, y, f); cv.style.cursor = h ? CURSOR[h.mode] : 'crosshair'; draw(); return; }
    if (d.kind === 'new') { d.x1 = x; d.y1 = y; draw(); return; }
    const b = lab.boxes[lab.sel], o = d.orig;
    const dx = (x - d.x0) / (f.iw * f.s), dy = (y - d.y0) / (f.ih * f.s);
    if (Math.abs(x - d.x0) + Math.abs(y - d.y0) > 2) d.moved = true;
    let l = o[1] - o[3] / 2, t = o[2] - o[4] / 2, r = o[1] + o[3] / 2, bt = o[2] + o[4] / 2;
    if (d.kind === 'move') {
      const w = r - l, h = bt - t;
      l = Math.min(1 - w, Math.max(0, l + dx)); t = Math.min(1 - h, Math.max(0, t + dy)); r = l + w; bt = t + h;
    } else {
      if (d.kind.includes('w')) l += dx;
      if (d.kind.includes('e')) r += dx;
      if (d.kind.includes('n')) t += dy;
      if (d.kind.includes('s')) bt += dy;
      [l, r] = [Math.max(0, Math.min(l, r)), Math.min(1, Math.max(l, r))];
      [t, bt] = [Math.max(0, Math.min(t, bt)), Math.min(1, Math.max(t, bt))];
    }
    b[1] = (l + r) / 2; b[2] = (t + bt) / 2; b[3] = r - l; b[4] = bt - t;
    if (d.origPoly) {
      const ol = o[1] - o[3] / 2, ot = o[2] - o[4] / 2;
      b.poly = d.origPoly.map(([px, py]) => [l + (px - ol) / (o[3] || 1e-6) * (r - l), t + (py - ot) / (o[4] || 1e-6) * (bt - t)]);
    }
    delete b.sug;
    draw();
  });
  const endDrag = (e) => {
    if (lab.painting) { lab.painting = null; draw(); return; }
    const d = lab.drag, f = fit();
    if (!d || !f) { lab.drag = null; return; }
    lab.drag = null;
    if (d.kind === 'new') {
      const w = Math.abs(d.x1 - d.x0), h = Math.abs(d.y1 - d.y0);
      if (w <= 4 && h <= 4) {          // a click: select the box under it (or nothing)
        lab.sel = d.pick; lab.picked = d.pick != null;
        if (d.pick != null) { curCls = lab.boxes[d.pick][0]; renderClasses(); }
      } else if (!classes.length) {
        toast('เพิ่ม class ก่อน (ปุ่ม "+ class")', true);
      } else if (w > 4 && h > 4) {
        const [a0, b0] = toNorm(Math.min(d.x0, d.x1), Math.min(d.y0, d.y1), f);
        const [a1, b1] = toNorm(Math.max(d.x0, d.x1), Math.max(d.y0, d.y1), f);
        push();
        lab.boxes.push([curCls, (a0 + a1) / 2, (b0 + b1) / 2, a1 - a0, b1 - b0]);
        lab.sel = lab.boxes.length - 1; lab.picked = false; dirty();
      }
    } else if (d.moved) dirty();
    else lab.undo.pop();
    draw();
  };
  cv.addEventListener('pointerup', endDrag);
  cv.addEventListener('pointercancel', endDrag);
  cv.addEventListener('pointerleave', () => { lab.hover = null; if (!lab.drag) draw(); });
  cv.addEventListener('dblclick', (e) => { if (tool === 'poly') { e.preventDefault(); closePoly(); } });
  cv.addEventListener('contextmenu', (e) => {   // right click = delete the box under the pointer
    e.preventDefault();
    if (tool === 'poly' && lab.draft.length) { lab.draft.pop(); draw(); return; }
    if (tool === 'brush' || tool === 'wand') return;
    const f = fit(); if (!f) return;
    const h = hit(...pos(e), f);
    if (h) { push(); lab.boxes.splice(h.box, 1); lab.sel = null; dirty(); draw(); }
  });
  // polygon: click points, close on the first point / double-click / Enter
  function polyClick(x, y, f) {
    if (!classes.length) { toast('เพิ่ม class ก่อน (ปุ่ม "+ class")', true); return; }
    const P = lab.draft, [u, v] = toNorm(x, y, f);
    if (P.length >= 3) {
      const [sx, sy] = [f.ox + P[0][0] * f.iw * f.s, f.oy + P[0][1] * f.ih * f.s];
      if (Math.hypot(sx - x, sy - y) < 10) { closePoly(); return; }
    }
    const last = P[P.length - 1];
    if (last && Math.hypot((last[0] - u) * f.iw * f.s, (last[1] - v) * f.ih * f.s) < 3) return;   // double-click's 2nd click
    P.push([u, v]); draw();
  }
  function closePoly() {
    const P = lab.draft;
    if (P.length < 3) { if (P.length) toast('polygon ต้องมีอย่างน้อย 3 จุด', true); return false; }
    push();
    lab.boxes.push(mk([curCls, ...polyBox(P)], P));
    lab.sel = lab.boxes.length - 1; lab.picked = false; lab.draft = []; dirty(); draw();
    return true;
  }
  // paint layer (brush + magic wand), at the image's own size; Enter turns it into outlines
  function maskCtx() {
    if (!lab.mask || lab.mask.width !== lab.img.naturalWidth) {
      lab.mask = document.createElement('canvas');
      lab.mask.width = lab.img.naturalWidth; lab.mask.height = lab.img.naturalHeight;
    }
    return lab.mask.getContext('2d');
  }
  const erasing = (e) => e.shiftKey || $('learn-erase').checked;
  function paintStart(e, x, y, f) {
    if (!classes.length) { toast('เพิ่ม class ก่อน (ปุ่ม "+ class")', true); return; }
    const m = maskCtx(), [u, v] = toNorm(x, y, f);
    lab.painting = { erase: erasing(e), last: [u * f.iw, v * f.ih] };
    m.globalCompositeOperation = lab.painting.erase ? 'destination-out' : 'source-over';
    m.fillStyle = color(curCls);
    m.beginPath(); m.arc(u * f.iw, v * f.ih, brushPx / f.s / 2, 0, 7); m.fill();
    lab.painted = true; draw();
  }
  function paintTo(x, y, f) {
    const m = maskCtx(), [u, v] = toNorm(x, y, f), p = [u * f.iw, v * f.ih], q = lab.painting.last;
    m.globalCompositeOperation = lab.painting.erase ? 'destination-out' : 'source-over';
    m.strokeStyle = color(curCls); m.lineWidth = brushPx / f.s; m.lineCap = m.lineJoin = 'round';
    m.beginPath(); m.moveTo(q[0], q[1]); m.lineTo(p[0], p[1]); m.stroke();
    lab.painting.last = p;
  }
  async function wandClick(e, x, y, f) {
    if (!classes.length) { toast('เพิ่ม class ก่อน (ปุ่ม "+ class")', true); return; }
    const [u, v] = toNorm(x, y, f), erase = erasing(e), id = lab.id;
    cv.style.cursor = 'progress';
    try {
      const r = await fetch('/api/learn/wand', { method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ ds, id, x: u, y: v, tol: wandTol }) });
      if (!r.ok) throw new Error(await r.text());
      const area = +r.headers.get('X-Area');
      const bmp = await createImageBitmap(await r.blob());
      if (lab.id !== id) return;
      // colour the region (alpha) with the class colour, then add it to / cut it from the layer
      const t = document.createElement('canvas'); t.width = bmp.width; t.height = bmp.height;
      const tc = t.getContext('2d'); tc.drawImage(bmp, 0, 0);
      tc.globalCompositeOperation = 'source-in'; tc.fillStyle = color(curCls); tc.fillRect(0, 0, t.width, t.height);
      const m = maskCtx();
      m.globalCompositeOperation = erase ? 'destination-out' : 'source-over';
      m.drawImage(t, 0, 0, lab.mask.width, lab.mask.height);
      lab.painted = true; draw();
      if (area > 0.5) toast('เลือกไปเกินครึ่งภาพ ลดค่า "ความต่างสี" แล้วลองใหม่ (Ctrl+Z ไม่ย้อนการระบาย ใช้ Esc ล้าง)', true);
    } catch (err) { fail(err); }
    cv.style.cursor = 'crosshair';
  }
  async function finishPaint() {
    if (!lab.painted || !lab.mask) return 0;
    const id = lab.id, cls = curCls;
    const blob = await new Promise(res => lab.mask.toBlob(res, 'image/png'));
    try {
      const r = await fetch('/api/learn/mask2poly', { method: 'POST', headers: { 'Content-Type': 'image/png' }, body: blob });
      if (!r.ok) throw new Error(await r.text());
      const { polys } = await r.json();
      if (lab.id !== id) return 0;
      lab.mask.getContext('2d').clearRect(0, 0, lab.mask.width, lab.mask.height); lab.painted = false;
      if (!polys.length) { toast('ไม่มีพื้นที่ที่ระบาย', true); draw(); return 0; }
      push();
      polys.forEach(P => lab.boxes.push(mk([cls, ...polyBox(P)], P)));
      lab.sel = lab.boxes.length - 1; lab.picked = false; dirty(); draw();
      if (polys.length > 1) toast(`ได้ ${polys.length} ชิ้น (ส่วนที่ระบายไม่ติดกัน) ลบชิ้นที่ไม่ต้องการได้`);
      return polys.length;
    } catch (err) { fail(err); return 0; }
  }
  function clearPaint() { if (lab.mask) lab.mask.getContext('2d').clearRect(0, 0, lab.mask.width, lab.mask.height); lab.painted = false; lab.draft = []; draw(); }
  // a shape in progress is part of the label: finish it before saving / leaving the image
  async function finishShapes() { if (lab.draft.length >= 3) closePoly(); lab.draft = []; if (lab.painted) await finishPaint(); }
  function setTool(t) {
    if (t !== 'poly' && lab.draft.length >= 3) closePoly(); else if (t !== 'poly') lab.draft = [];
    tool = t;
    try { localStorage.setItem('qcar2.learn.tool', t); } catch (e) {}
    document.querySelectorAll('#learn-tool button').forEach(b => b.classList.toggle('on', b.dataset.t === t));
    $('learn-brush-opts').hidden = t !== 'brush' && t !== 'wand';
    $('learn-brush-size').hidden = t !== 'brush';
    $('learn-wand-tol').hidden = t !== 'wand';
    $('learn-tool-hint').textContent = {
      box: 'ลากเพื่อตีกรอบ',
      poly: 'คลิกทีละจุดรอบวัตถุ · คลิกจุดแรก / ดับเบิลคลิก / Enter = ปิดรูป · คลิกขวา/Backspace = ลบจุดล่าสุด',
      brush: 'ระบายทับวัตถุ · Shift (หรือติ๊ก "ลบ") = ยางลบ · [ ] = ขนาดพู่กัน · Enter = สร้างรูปร่าง',
      wand: 'คลิกที่วัตถุ = เลือกบริเวณสีใกล้เคียง (คลิกหลายจุดได้) · Shift = เอาออก · Enter = สร้างรูปร่าง',
    }[t];
    cv.style.cursor = t === 'brush' ? 'none' : 'crosshair';
    draw();
  }
  document.querySelectorAll('#learn-tool button').forEach(b => b.onclick = () => { setTool(b.dataset.t); b.blur(); });
  $('learn-brush').value = brushPx; $('learn-brush-v').textContent = brushPx;
  $('learn-brush').oninput = (e) => { brushPx = +e.target.value; $('learn-brush-v').textContent = brushPx; try { localStorage.setItem('qcar2.learn.brush', brushPx); } catch (err) {} };
  $('learn-tol').value = wandTol; $('learn-tol-v').textContent = wandTol;
  $('learn-tol').oninput = (e) => { wandTol = +e.target.value; $('learn-tol-v').textContent = wandTol; try { localStorage.setItem('qcar2.learn.tol', wandTol); } catch (err) {} };
  $('learn-paint-done').onclick = () => finishPaint();
  $('learn-paint-clear').onclick = clearPaint;
  if (window.ResizeObserver) new ResizeObserver(() => draw()).observe(cv);
  addEventListener('resize', () => { if (on) draw(); });   // drag bars / maximize send this

  async function openImage(k) {
    const list = shown();
    if (!list.length) return;
    k = Math.max(0, Math.min(list.length - 1, k));
    await saveIfDirty();
    openId(list[k].id);
  }
  async function openId(id) {
    lab.id = id; lab.img = null; lab.boxes = []; lab.sel = null; lab.dirty = false; lab.undo = [];
    lab.draft = []; lab.mask = null; lab.painted = false; lab.painting = null;
    $('learn-other').innerHTML = '';
    renderGallery(); draw();
    const img = new Image();
    img.onload = () => { if (lab.id === id) { lab.img = img; draw(); } };
    img.src = `/api/learn/image/${encodeURIComponent(ds)}/${id}.jpg`;
    try {
      const l = await api('/label', { id });
      if (lab.id !== id) return;
      lab.boxes = l.boxes.map((b, k) => mk(b, (l.polys || [])[k]));
      lab.labelled = l.labelled;
      lab.pending = !l.labelled && !!l.pending;
      if (lab.pending) lab.boxes = l.pending.map((b, k) => mk(b, (l.pending_polys || [])[k], b[5] || 1));
      draw();
    } catch (e) { fail(e); }
  }
  async function save(empty) {
    if (!lab.id) return;
    if (empty) { push(); lab.boxes = []; lab.sel = null; clearPaint(); } else await finishShapes();
    try {
      const l = await api('/label', { id: lab.id, boxes: lab.boxes.map(b => b.slice(0, 5)), polys: lab.boxes.map(b => b.poly || null) });
      lab.boxes = l.boxes.map((b, k) => mk(b, (l.polys || [])[k])); lab.labelled = true; lab.dirty = false; lab.pending = false;
      const im = images.find(x => x.id === lab.id);
      if (im) { im.labelled = true; im.n = l.boxes.length; im.pending = false; }
      draw();
    } catch (e) { fail(e); }
  }
  async function saveIfDirty() { if (lab.dirty || lab.painted || lab.draft.length >= 3) await save(); }
  async function saveNext(empty) {
    await save(empty);
    const list = shown();
    let k = list.findIndex(im => im.id === lab.id);
    // "ยังไม่ label" filter: the saved image leaves the list, the next one moves into its place
    if (filter === 'todo' || filter === 'review') { renderGallery(); const l2 = shown(); if (l2.length) openId(l2[Math.min(Math.max(k, 0), l2.length - 1)].id); return; }
    if (k < list.length - 1) openImage(k + 1); else { renderGallery(); toast('ภาพสุดท้ายแล้ว'); }
  }
  async function suggest() {
    if (!lab.id) return;
    const btn = $('learn-suggest'); btn.disabled = true; btn.textContent = 'กำลังหา…';
    try {
      const only = $('learn-only').checked && classes.length ? [curCls] : null;
      const r = await api('/suggest', { id: lab.id, conf: 0.25, only });
      const id = lab.id;
      if (lab.id !== id) return;
      push();
      let added = 0;
      r.boxes.forEach((b, k) => {
        const dup = lab.boxes.some(o => o[0] === b[0] && iou(o, b) > 0.5);
        if (!dup) { lab.boxes.push(mk(b, (r.polys || [])[k], b[5])); added++; }
      });
      if (added) dirty();
      const other = only ? [] : Object.entries(r.other);
      $('learn-other').innerHTML = other.length ? '<span class="hint">โมเดลเห็นแต่ไม่มีใน class:</span> ' + other.map(([n, bs]) =>
        `<button data-n="${esc(n)}">+ ${esc(n)} (${bs.length})</button>`).join(' ') : '';
      $('learn-other').querySelectorAll('button').forEach(b => b.onclick = async () => {
        const n = b.dataset.n, k = await addClass(n);
        if (k == null) return;
        push(); r.other[n].forEach(bx => { const nb = [k, ...bx.slice(0, 4)]; nb.sug = bx[4]; lab.boxes.push(nb); });
        dirty(); b.remove(); draw();
      });
      const who = r.model === 'coco' ? 'โมเดลเริ่มต้น (COCO)' : 'ตัวช่วยที่สอนจาก label ของเรา';
      toast(added ? `${who}: เพิ่มกรอบ ${added} กรอบ (เส้นประ) ตรวจดูแล้วกดบันทึก` :
        r.model === 'coco' ? (r.known ? 'โมเดลเริ่มต้น (COCO) ไม่เห็นวัตถุ' : `โมเดลเริ่มต้น (COCO) ไม่รู้จัก "${only ? classes[curCls] : 'class ของเรา'}" — label สัก 5+ ภาพแล้วกด "สอนตัวช่วย"`)
        : `${who}: ไม่พบวัตถุ${only ? ' class ' + classes[curCls] : ''} (ถ้าพลาดบ่อย label เพิ่มแล้วกด "สอนตัวช่วย" อีกครั้ง)`);
      draw();
    } catch (e) { fail(e); }
    btn.disabled = false; btn.textContent = 'ช่วยตีกรอบ (Q)';
  }
  const iou = (a, b) => {
    const ax0 = a[1] - a[3] / 2, ax1 = a[1] + a[3] / 2, ay0 = a[2] - a[4] / 2, ay1 = a[2] + a[4] / 2;
    const bx0 = b[1] - b[3] / 2, bx1 = b[1] + b[3] / 2, by0 = b[2] - b[4] / 2, by1 = b[2] + b[4] / 2;
    const i = Math.max(0, Math.min(ax1, bx1) - Math.max(ax0, bx0)) * Math.max(0, Math.min(ay1, by1) - Math.max(ay0, by0));
    return i / (a[3] * a[4] + b[3] * b[4] - i || 1);
  };
  async function copyPrev() {
    const list = shown(), k = list.findIndex(im => im.id === lab.id);
    const all = images, ka = all.findIndex(im => im.id === lab.id);
    const prev = [...all.slice(0, ka)].reverse().find(im => im.labelled && im.n);   // nearest earlier labelled image
    if (!prev) { toast('ไม่มีภาพก่อนหน้าที่ label ไว้', true); return; }
    try {
      const l = await api('/label', { id: prev.id });
      push();
      let added = 0;
      l.boxes.forEach((b, k) => { if (!lab.boxes.some(o => o[0] === b[0] && iou(o, b) > 0.6)) { lab.boxes.push(mk(b, (l.polys || [])[k])); added++; } });
      if (added) dirty();
      lab.sel = null; draw();
      toast(`คัดลอก ${added} กรอบจากภาพ ${prev.id.slice(9)} ลากปรับให้ตรงแล้วกด S`);
    } catch (e) { fail(e); }
    void k;
  }
  function delBox() { if (lab.sel == null) return; push(); lab.boxes.splice(lab.sel, 1); lab.sel = null; dirty(); draw(); }
  function undo() {
    if (lab.draft.length) { lab.draft.pop(); draw(); return; }
    if (!lab.undo.length) return; lab.boxes = deser(lab.undo.pop()); lab.sel = null; dirty(); draw();
  }
  async function delImage() {
    if (!lab.id || !confirm('ลบภาพนี้?')) return;
    const list = shown(), k = list.findIndex(im => im.id === lab.id);
    try {
      await api('/delete', { ids: [lab.id] });
      images = images.filter(im => im.id !== lab.id); lab.dirty = false; lab.id = null;
      imgSig = ''; renderGallery();
      const l2 = shown(); if (l2.length) openId(l2[Math.min(k, l2.length - 1)].id); else draw();
    } catch (e) { fail(e); }
  }
  $('learn-prev').onclick = () => openImage(cur - 1);
  $('learn-next').onclick = () => openImage(cur + 1);
  $('learn-save').onclick = () => saveNext(false);
  $('learn-empty').onclick = () => saveNext(true);
  $('learn-suggest').onclick = suggest;
  $('learn-copy').onclick = copyPrev;
  // helper model + auto-label the whole set
  function renderHelper() {
    const h = st.helper, sel = $('learn-helper-model');
    const opts = '<option value="auto">อัตโนมัติ (โมเดลล่าสุดของชุดนี้)</option>' + st.runs.filter(r => r.best && !r.running).map(r =>
      `<option value="${esc(r.run)}">${r.quick ? 'ตัวช่วย' : 'โมเดล'} ${esc(r.run.slice(9, 11) + ':' + r.run.slice(11, 13))} · ${esc((r.classes || []).join(', '))} · mAP50 ${(r.best_map50 ?? 0).toFixed(2)}</option>`).join('')
      + '<option value="coco">โมเดลทั่วไป (COCO 80 ชนิด)</option>';
    if (sel.dataset.sig !== opts) { sel.innerHTML = opts; sel.dataset.sig = opts; }
    if (document.activeElement !== sel) sel.value = [...sel.options].some(o => o.value === h.mode) ? h.mode : 'auto';
    $('learn-helper-now').textContent = h.own ? `ใช้: ${h.run}${h.why ? ' · ' + h.why : ''}` : `ใช้: COCO${h.why ? ' · ' + h.why : ''}`;
    const a = st.autolabel, b = $('learn-autolabel');
    if (a && a.running) { b.textContent = `หยุด (${a.done}/${a.total})`; b.classList.add('on'); }
    else { b.textContent = 'label อัตโนมัติทั้งชุด'; b.classList.remove('on'); }
    $('learn-auto-res').textContent = a && !a.running ? (a.error ? 'ผิดพลาด: ' + a.error : `ล่าสุด: ตีกรอบ ${a.done} ภาพ ได้ ${a.boxes} กรอบ`) : '';
  }
  $('learn-helper-model').onchange = async (e) => { try { st.helper = (await api('/helper', { mode: e.target.value })).helper; renderHelper(); } catch (err) { fail(err); } };
  $('learn-autolabel').onclick = async () => {
    if (st && st.autolabel && st.autolabel.running) { api('/autolabel', { cancel: true }).catch(fail); return; }
    const todo = images.filter(im => !im.labelled && !im.pending).length;
    if (!todo) { toast('ทุกภาพ label แล้วหรือรอตรวจอยู่แล้ว', true); return; }
    const only = $('learn-only').checked && classes.length ? [curCls] : null;
    if (!confirm(`ให้ตัวช่วยตีกรอบ${only ? ' class "' + classes[curCls] + '"' : 'ทุก class'} ใน ${todo} ภาพที่ยังไม่ label?\nผลจะอยู่ในตัวกรอง "รอตรวจ" ยังไม่ถูกใช้ train จนกว่าจะกด S ยืนยันทีละภาพ`)) return;
    const poller = setInterval(poll, 800);
    try {
      const r = await api('/autolabel', { conf: +$('learn-auto-conf').value, only });
      toast(`ตีกรอบ ${r.done} ภาพ ได้ ${r.boxes} กรอบ ไปที่ "รอตรวจ" เพื่อตรวจทีละภาพ`);
      filter = 'review'; document.querySelectorAll('#learn-filter button').forEach(x => x.classList.toggle('on', x.dataset.f === 'review'));
      imgSig = ''; await poll(); const l = shown(); if (l.length) openId(l[0].id);
    } catch (e) { fail(e); }
    clearInterval(poller);
  };
  $('learn-clear-pending').onclick = async () => {
    if (!confirm('ทิ้งกรอบที่รอตรวจทั้งหมด? (ภาพที่ยืนยันแล้วไม่หาย)')) return;
    try { await api('/autolabel', { clear: true }); imgSig = ''; poll(); if (lab.pending) openId(lab.id); } catch (e) { fail(e); }
  };
  $('learn-helper').onclick = async () => {
    if (st && st.job) { toast('มีงาน train ทำอยู่แล้ว รอให้เสร็จก่อน', true); return; }
    await saveIfDirty();
    try { await api('/train', { quick: true, task: $('learn-task').value }); toast('กำลังสอนตัวช่วยจากภาพที่ label แล้ว (ประมาณ 1-3 นาที) label ต่อไปได้เลย'); poll(); }
    catch (e) { fail(e); }
  };
  $('learn-delbox').onclick = delBox;
  $('learn-undo').onclick = undo;
  $('learn-delimg').onclick = delImage;

  addEventListener('keydown', (e) => {
    if (!on || e.ctrlKey && e.key !== 'z' || e.metaKey || e.altKey) return;
    const t = e.target;
    if (t && t.closest && (t.closest('input, select, textarea') || t.isContentEditable)) return;
    const k = e.key;
    if (/^[1-9]$/.test(k)) { pickClass(+k - 1); }
    else if (k === 'ArrowLeft' || k === 'a' || k === 'A') openImage(cur - 1);
    else if (k === 'ArrowRight' || k === 'd' || k === 'D') openImage(cur + 1);
    else if (k === 'Enter' && lab.draft.length) closePoly();
    else if (k === 'Enter' && lab.painted) finishPaint();
    else if (k === 'Enter' || k === 's' || k === 'S') saveNext(false);
    else if (k === 'b' || k === 'B') setTool('box');
    else if (k === 'p' || k === 'P') setTool('poly');
    else if (k === 'g' || k === 'G') setTool('brush');
    else if (k === 'w' || k === 'W') setTool('wand');
    else if (k === '[' || k === ']') { brushPx = Math.max(4, Math.min(150, brushPx + (k === ']' ? 4 : -4))); $('learn-brush').value = brushPx; $('learn-brush-v').textContent = brushPx; draw(); }
    else if (k === 'n' || k === 'N') saveNext(true);
    else if (k === 'q' || k === 'Q') suggest();
    else if (k === 'r' || k === 'R') copyPrev();
    else if (k === 'c' || k === 'C') shot();
    else if (k === 'Backspace' && lab.draft.length) { lab.draft.pop(); draw(); }
    else if (k === 'Delete' || k === 'Backspace') delBox();
    else if (k === 'z' && e.ctrlKey) undo();
    else if (k === 'Escape') { if (lab.draft.length || lab.painted) clearPaint(); else { lab.sel = null; draw(); } }
    else return;
    e.preventDefault();
  });

  // ------------------------------------------------------------ capture
  async function shot() {
    try {
      const r = await api('/capture', {});
      const names = r.ids.map(i => CAM_SHORT[i.split('_')[1]] || i.split('_')[1]).join(', ');
      const miss = Object.entries(r.why || {}).map(([c, w]) => `${CAM_SHORT[c] || c} ${w}`).join(', ');
      toast(`บันทึก ${r.ids.length} ภาพ (${names})` + (miss ? ` · ไม่ได้: ${miss}` : ''), !!miss);
      r.ids.forEach(id => images.push({ id, cam: id.split('_')[1], labelled: false, n: 0, cls: [], val: false }));
      imgSig = ''; renderGallery(); poll();
    } catch (e) { fail(e); }
  }
  // which cameras to capture (several = the same moment from each), which one to preview / detect on
  let camSig = '';
  function renderCams(c) {
    const sig = JSON.stringify([c.sources, c.selected, c.view]);
    if (sig !== camSig) {
      camSig = sig;
      $('learn-cams').innerHTML = '<span class="hint">ถ่ายจาก</span>' + c.sources.map(x =>
        `<label class="lcamchk"><input type="checkbox" value="${x.name}"${c.selected.includes(x.name) ? ' checked' : ''}> ${esc(x.label)} <i data-age="${x.name}"></i></label>`).join('')
        + '<button id="learn-cams-all" title="เลือกทุกกล้อง">ทั้งหมด</button>';
      $('learn-cams').querySelectorAll('input').forEach(i => i.onchange = () => {
        const cams = [...$('learn-cams').querySelectorAll('input:checked')].map(x => x.value);
        if (!cams.length) { i.checked = true; toast('ต้องเลือกอย่างน้อย 1 กล้อง', true); return; }
        api('/cams', { cams }).then(() => poll()).catch(fail);
      });
      $('learn-cams-all').onclick = () => api('/cams', { cams: c.sources.map(x => x.name) }).then(() => poll()).catch(fail);
      const v = $('learn-view');
      v.innerHTML = c.sources.map(x => `<option value="${x.name}">${esc(x.label)}</option>`).join('');
      v.value = c.view;
      const cf = $('learn-camf'), keep = camFilter;
      cf.innerHTML = '<option value="">ทุกกล้อง</option>' + c.sources.map(x => `<option value="${x.name}">${esc(x.label)}</option>`).join('');
      cf.value = keep;
      if (liveMode === 'cam') setLive('cam');   // the preview camera may have changed
    }
    const fresh = (n) => c.ages[n] != null && c.ages[n] <= 2;
    $('learn-cams').querySelectorAll('[data-age]').forEach(el => {
      const n = el.dataset.age, sel = c.selected.includes(n);
      el.className = !sel ? '' : fresh(n) ? 'ok' : 'wait';
      el.textContent = !sel ? '' : fresh(n) ? '●' : '…';
      el.title = !sel ? '' : fresh(n) ? 'พร้อม' : 'กำลังเปิดกล้อง / ยังไม่มีภาพ';
    });
    const ready = c.selected.filter(fresh), wait = c.selected.filter(n => !fresh(n));
    $('learn-cam-state').textContent = !ready.length ? 'กำลังเปิดกล้อง… (กล้องรอบคันใช้เวลาเปิดสักครู่)'
      : wait.length ? `พร้อม ${ready.length}/${c.selected.length} กล้อง · รอ ${wait.map(n => CAM_SHORT[n] || n).join(', ')}`
      : `พร้อม ${ready.length} กล้อง`;
    $('learn-shot').disabled = !ready.length;
    $('learn-shot').textContent = c.selected.length > 1 ? `ถ่าย ${c.selected.length} กล้อง (C)` : 'ถ่ายภาพ (C)';
  }
  $('learn-view').onchange = (e) => api('/cams', { view: e.target.value }).then(() => { camSig = ''; poll(); }).catch(fail);
  $('learn-camf').onchange = (e) => { camFilter = e.target.value; renderGallery(); };
  $('learn-shot').onclick = shot;
  $('learn-auto').onclick = async () => {
    try {
      const want = !(st && st.auto.on);
      await api('/auto', { on: want, interval: +$('learn-interval').value });
      poll();
    } catch (e) { fail(e); }
  };

  // live view: plain camera or the trained detector
  const liveView = RobotUI.streamView($('learn-live'));
  let liveMode = 'cam', liveShown = null;
  try { liveMode = localStorage.getItem('qcar2.learn.live') || 'cam'; } catch (e) {}
  if (liveMode === 'realsense') liveMode = 'cam';
  function setLive(m) {
    liveMode = m;
    try { localStorage.setItem('qcar2.learn.live', m); } catch (e) {}
    document.querySelectorAll('#learn-live-seg button').forEach(b => b.classList.toggle('on', b.dataset.m === m));
    const name = !on || m === 'off' ? null : m === 'detect' ? 'detect' : (st && st.camera.view) || 'realsense';
    if (liveView && name !== liveShown) { liveShown = name; liveView(name, name); }
  }
  document.querySelectorAll('#learn-live-seg button').forEach(b => b.onclick = () => setLive(b.dataset.m));

  // ------------------------------------------------------------ training
  $('learn-train').onclick = async () => {
    if (st && st.job) { if (confirm('หยุดงานที่กำลังทำอยู่?')) { try { await api('/stop', {}); poll(); } catch (e) { fail(e); } } return; }
    await saveIfDirty();
    const from = $('learn-from').value;
    try {
      await api('/train', { size: $('learn-size').value, epochs: +$('learn-epochs').value, imgsz: +$('learn-imgsz').value,
                            batch: +$('learn-batch').value, from_run: from || null, patience: +$('learn-patience').value,
                            task: $('learn-task').value });
      toast('เริ่ม train แล้ว'); poll();
    } catch (e) { fail(e); }
  };
  let lastJobKey = null;
  function renderJob() {
    const j = st.job, btn = $('learn-train');
    const key = j ? j.ds + '/' + j.run : null;
    if (lastJobKey && !key && st.last_job) {
      const r = st.runs.find(x => x.run === st.last_job.run);
      if (r && r.state === 'done' && st.last_job.type === 'train')
        toast(r.quick ? `ตัวช่วยพร้อมแล้ว (mAP50 ${(r.best_map50 ?? 0).toFixed(2)}) กด Q เพื่อให้ช่วยตีกรอบ`
          : r.early_stop ? `train จบที่รอบ ${r.epoch}/${r.epochs} (early stopping: ผลไม่ดีขึ้นแล้ว) เก็บโมเดลที่ดีที่สุดไว้แล้ว` : 'train เสร็จแล้ว');
    }
    lastJobKey = key;
    const hb = $('learn-helper');
    hb.disabled = !!j;
    hb.textContent = j && j.type === 'train' && j.quick ? `กำลังสอนตัวช่วย ${j.epoch}/${j.epochs ?? '?'}` : 'สอนตัวช่วย';
    btn.textContent = j ? (j.type === 'train' ? 'หยุด train' : 'หยุดแปลงโมเดล') : 'เริ่ม train';
    btn.classList.toggle('danger', !!j);
    const c = st.counts;
    const warn = [];
    if (st.drive_auto) warn.push('รถอยู่ในโหมดอัตโนมัติ: train ไม่ได้จนกว่าจะหยุด');
    if (!j && c.labelled < 4) warn.push(`ต้อง label อย่างน้อย 4 ภาพ (ตอนนี้ ${c.labelled}) · แนะนำ 50+ ภาพต่อ class`);
    if (!j && st.nav_method) warn.push('ROS/การนำทางทำงานอยู่: train จะแย่ง GPU ทำให้ระบบอื่นช้าลงระหว่างนั้น');
    $('learn-train-warn').textContent = warn.join(' · ');
    const box = $('learn-job');
    if (!j) {
      box.hidden = !st.last_job;
      if (st.last_job) {
        const r = st.runs.find(x => x.run === st.last_job.run);
        box.innerHTML = r ? `<span class="hint">ล่าสุด: ${st.last_job.type === 'train' ? 'train' : 'แปลง TensorRT'} ${esc(r.run)} — ${
          r.state === 'done' && st.last_job.type === 'train' ? 'เสร็จแล้ว ดูผลในรายการโมเดลด้านล่าง' :
          st.last_job.type === 'export' ? (r.export === 'done' ? 'แปลงเสร็จ' : 'ไม่สำเร็จ: ' + esc(r.export_error || '')) :
          esc(r.state) + (r.error ? ': ' + esc(r.error) : '')}</span>` : '';
      }
      chart([]);
      return;
    }
    box.hidden = false;
    const pct = j.type === 'train' && j.epochs ? Math.round(j.epoch / j.epochs * 100) : null;
    const last = j.history[j.history.length - 1];
    const eta = j.epoch > 0 && j.epochs ? (j.elapsed / j.epoch) * (j.epochs - j.epoch) : null;
    box.innerHTML = `<div class="lbar"><i style="width:${pct == null ? 100 : pct}%"${pct == null ? ' class="busy"' : ''}></i></div>
      <div class="hint">${j.type === 'train' ? `รอบ ${j.epoch}/${j.epochs ?? '?'}` : 'กำลังแปลงเป็น TensorRT (ใช้เวลาหลายนาที)'} · ${fmtT(j.elapsed)}${
      eta != null ? ' · เหลือประมาณ ' + fmtT(eta) : ''}${last ? ` · mAP50 ${(last.map50 ?? 0).toFixed(3)}` : ''} · ${esc(j.state)}</div>
      <pre class="llog">${esc(j.log.slice(-4).join('\n'))}</pre>`;
    chart(j.history);
  }
  const fmtT = (s) => s < 60 ? `${Math.round(s)} วิ` : s < 3600 ? `${Math.floor(s / 60)} นาที ${Math.round(s % 60)} วิ` : `${(s / 3600).toFixed(1)} ชม.`;
  function chart(h) {
    const c = $('learn-chart'), g = c.getContext('2d');
    c.hidden = !h.length;
    if (!h.length) return;
    const r = c.getBoundingClientRect(), dpr = devicePixelRatio || 1;
    c.width = r.width * dpr; c.height = r.height * dpr;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    const W = r.width, H = r.height, P = 22;
    g.clearRect(0, 0, W, H);
    g.strokeStyle = '#2a313a'; g.lineWidth = 1;
    for (let k = 0; k <= 4; k++) { const y = P / 2 + (H - P) * k / 4; g.beginPath(); g.moveTo(28, y); g.lineTo(W - 4, y); g.stroke(); }
    const n = Math.max(2, h[h.length - 1].epoch), X = (e) => 28 + (W - 32) * (e - 1) / (n - 1);
    const maxLoss = Math.max(...h.map(p => p.box_loss || 0), 0.1);
    const line = (key, scale, col) => {
      g.strokeStyle = col; g.lineWidth = 2; g.beginPath();
      h.forEach((p, k) => { const v = (p[key] || 0) / scale, y = P / 2 + (H - P) * (1 - Math.min(1, v)); k ? g.lineTo(X(p.epoch), y) : g.moveTo(X(p.epoch), y); });
      g.stroke();
    };
    line('box_loss', maxLoss, '#f0a92b'); line('map50', 1, '#2fbf71'); line('map', 1, '#3b8cff');
    g.fillStyle = '#8b95a3'; g.font = '10px system-ui'; g.textBaseline = 'middle';
    g.fillText('1.0', 4, P / 2); g.fillText('0', 12, H - P / 2);
    g.textBaseline = 'alphabetic';
    [['mAP50', '#2fbf71'], ['mAP50-95', '#3b8cff'], ['box loss', '#f0a92b']].forEach(([t, col], k) => {
      g.fillStyle = col; g.fillText(t, 32 + k * 70, H - 2);
    });
  }

  // ------------------------------------------------------------ models (runs)
  function renderRuns() {
    const runs = st.runs, a = st.active;
    const from = $('learn-from'), keep = from.value;
    const opts = '<option value="">เริ่มใหม่จากโมเดล COCO</option>' + runs.filter(r => r.best).map(r =>
      `<option value="${esc(r.run)}">ต่อจาก ${esc(r.run)} (mAP50 ${(r.best_map50 ?? 0).toFixed(2)})</option>`).join('');
    if (from.dataset.sig !== opts) { from.innerHTML = opts; from.dataset.sig = opts; from.value = runs.some(r => r.run === keep) ? keep : ''; }
    const box = $('learn-runs');
    if (!runs.length) { box.innerHTML = '<p class="hint">ยังไม่มีโมเดลที่ train ไว้ในชุดข้อมูลนี้</p>'; return; }
    const sig = JSON.stringify([runs.map(r => [r.run, r.state, r.epoch, r.best, r.engine, r.export, r.running, r.early_stop]), a && [a.ds, a.run, a.path], !!st.job]);
    if (box.dataset.sig === sig) return;
    box.dataset.sig = sig;
    const STATE = { done: 'เสร็จ', training: 'กำลัง train', starting: 'กำลังเริ่ม', stopped: 'หยุดกลางทาง', error: 'ผิดพลาด' };
    box.innerHTML = runs.map(r => {
      const isActive = a && a.ds === st.ds && a.run === r.run;
      const l = r.last || {};
      const d = new Date((r.created || 0) * 1000);
      return `<div class="lrun${isActive ? ' active' : ''}">
        <div class="lrun-h">${r.quick ? '<span class="chip">ตัวช่วย</span>' : ''}${r.task === 'segment' ? '<span class="chip">รูปร่าง</span>' : ''}<b>${d.toLocaleDateString('th-TH', { day: 'numeric', month: 'short' })} ${d.toLocaleTimeString('th-TH', { hour: '2-digit', minute: '2-digit' })}</b>
          <span class="chip">${esc(STATE[r.state] || r.state)}</span>${isActive ? '<span class="chip ok">ใช้งานอยู่' + (a.engine ? ' · TensorRT' : '') + '</span>' : ''}</div>
        <div class="hint">YOLOv8${esc(r.size)} · ${r.imgsz}px · รอบ ${r.epoch}/${r.epochs} · train ${r.n_train} / ตรวจ ${r.n_val} ภาพ · ${esc((r.classes || []).join(', '))}</div>
        ${r.history.length ? `<div class="lrun-m"><span>mAP50 <b>${(r.best_map50 ?? 0).toFixed(3)}</b></span><span>precision ${(l.precision ?? 0).toFixed(2)}</span><span>recall ${(l.recall ?? 0).toFixed(2)}</span></div>` : ''}
        ${r.early_stop ? `<div class="hint">หยุดก่อนครบ ${r.epochs} รอบ เพราะผลบนภาพตรวจไม่ดีขึ้นแล้ว ${r.patience ?? Math.max(10, Math.floor(r.epochs / 3))} รอบติด (early stopping) — เก็บโมเดลรอบที่ดีที่สุดไว้แล้ว ถ้าอยากให้ train ครบ ตั้ง "หยุดเองถ้าไม่ดีขึ้น" = 0</div>` : ''}
        ${r.error ? `<div class="hint bad">${esc(r.error)}</div>` : ''}${r.export === 'error' ? `<div class="hint bad">แปลง TensorRT ไม่สำเร็จ: ${esc(r.export_error || '')}</div>` : ''}
        <div class="row">
          ${r.best && !isActive ? `<button data-use="${esc(r.run)}">ใช้งาน</button>` : ''}
          ${isActive ? '<button data-unuse="1">เลิกใช้</button>' : ''}
          ${r.best && !r.engine && !st.job ? `<button data-exp="${esc(r.run)}" title="เร็วขึ้นประมาณ 2-3 เท่า ใช้เวลาแปลงหลายนาที">แปลงเป็น TensorRT</button>` : ''}
          ${!r.running ? `<button data-del="${esc(r.run)}">ลบ</button>` : ''}
        </div></div>`;
    }).join('');
    box.querySelectorAll('[data-use]').forEach(b => b.onclick = async () => { try { await api('/active', { run: b.dataset.use }); toast('ใช้โมเดลนี้แล้ว ดูผลที่ภาพสด "ผลตรวจจับ"'); setLive('detect'); poll(); } catch (e) { fail(e); } });
    box.querySelectorAll('[data-unuse]').forEach(b => b.onclick = async () => { try { await api('/active', { run: null }); poll(); } catch (e) { fail(e); } });
    box.querySelectorAll('[data-exp]').forEach(b => b.onclick = async () => { try { await api('/export', { run: b.dataset.exp }); toast('เริ่มแปลงเป็น TensorRT'); poll(); } catch (e) { fail(e); } });
    box.querySelectorAll('[data-del]').forEach(b => b.onclick = async () => {
      if (!confirm('ลบโมเดลนี้?')) return;
      try { await api('/delete_run', { run: b.dataset.del }); box.dataset.sig = ''; poll(); } catch (e) { fail(e); }
    });
  }
  function renderDetect() {
    const d = st.detect, a = st.active;
    $('learn-active').textContent = a ? `โมเดลที่ใช้: ${a.ds} / ${a.run}${a.engine ? ' (TensorRT)' : ' (PyTorch)'} · ${a.classes.join(', ')}` : 'ยังไม่ได้เลือกโมเดล (กด "ใช้งาน" ที่โมเดลด้านล่าง)';
    const objs = liveMode === 'detect' ? d.objects : [];
    $('learn-objs').innerHTML = liveMode !== 'detect' ? '' : d.state === 'error' ? `<span class="bad">${esc(d.error)}</span>`
      : d.state === 'loading' ? 'กำลังโหลดโมเดล…' : !a ? '' : objs.length ?
      objs.map(o => `<span class="chip" style="border-color:${color(o.cls)}"><i style="background:${color(o.cls)}"></i>${o.cls} ${esc(o.name)} ${Math.round(o.conf * 100)}%</span>`).join(' ') + ` <span class="hint">${d.fps} fps</span>`
      : `<span class="hint">ไม่พบวัตถุ · ${d.fps} fps</span>`;
    const cf = $('learn-conf');
    if (document.activeElement !== cf) { cf.value = d.conf; $('learn-conf-v').textContent = Math.round(d.conf * 100) + '%'; }
  }
  $('learn-conf').oninput = (e) => { $('learn-conf-v').textContent = Math.round(e.target.value * 100) + '%'; };
  $('learn-conf').onchange = (e) => api('/detect', { conf: +e.target.value }).catch(fail);

  // ------------------------------------------------------------ page on/off
  addEventListener(RobotUI.PAGE_EVENT, (e) => {
    const was = on;
    on = e.detail.page === 'learn';
    if (on && !was) { poll(); setLive(liveMode); setTimeout(draw, 50); }
    if (!on && was) { saveIfDirty(); clearTimeout(pollT); if (liveView) liveView(null); liveShown = null; }
  });
  if (document.body.dataset.page === 'learn') { on = true; poll(); setLive(liveMode); }
  renderClasses(); setTool(tool);
})();
