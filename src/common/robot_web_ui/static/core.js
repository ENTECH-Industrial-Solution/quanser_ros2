// robot_web_ui core: the parts of the robot web page that do not depend on the robot.
//
//   <html data-store="myrobot">          prefix of everything kept in localStorage
//   <link rel="stylesheet" href="/ui/theme.css"> <link rel="stylesheet" href="/ui/ui.css">
//   <script src="/ui/core.js"></script>   then the robot's own script uses window.RobotUI
//
// What is here (see README.md for the page structure each part expects):
//   shell      header height, control panel left/right, phone bottom sheet + dock, "จัดหน้าต่าง"
//   windows    maximise button on every .cam window (winify), drag bars between windows
//              (splitPair) and where two bars meet (junction), remembered per browser
//   pages      one page at a time (header tabs #page-tabs .ptab, phone dock #dock .tab),
//              mouse wheel past the end of a page = next page
//   streams    camera views over /ws/stream/<name> (streamView), the split-screen camera page
//   control    /ws status socket with reconnect, chips, joystick, driving keys
//
// Text is Thai; i18n.js translates it when the page is switched to English.
(() => {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const PREFIX = document.documentElement.dataset.store || 'robot';
  const store = {
    get(k, def = null) { try { const v = localStorage.getItem(PREFIX + '.' + k); return v === null ? def : v; } catch (e) { return def; } },
    set(k, v) { try { localStorage.setItem(PREFIX + '.' + k, v); } catch (e) {} },
    json(k, def) { try { return JSON.parse(localStorage.getItem(PREFIX + '.' + k) || 'null') || def; } catch (e) { return def; } },
    setJson(k, v) { try { localStorage.setItem(PREFIX + '.' + k, JSON.stringify(v)); } catch (e) {} },
  };
  // Phone layout (portrait, or landscape: wide but very low). Same query as in ui.css.
  const PHONE = '(max-width: 700px), (max-height: 520px) and (max-width: 1100px)';
  const PAGE_EVENT = 'ui-page';   // CustomEvent on window, detail: { page, old }
  const wsProto = location.protocol === 'https:' ? 'wss' : 'ws';
  const wsUrl = (path) => `${wsProto}://${location.host}${path}`;
  const isDesk = () => document.body.classList.contains('desk');

  // ---- Windows: the button in the title bar (or double-click the title) fills the work area ----
  const nudge = () => dispatchEvent(new Event('resize'));   // canvases re-measure themselves
  let maxed = null;
  const ICON_MAX = '<svg viewBox="0 0 24 24"><path d="M4 9V4h5M15 4h5v5M20 15v5h-5M9 20H4v-5"/></svg>';
  const ICON_MIN = '<svg viewBox="0 0 24 24"><path d="M9 4v5H4M20 9h-5V4M15 20v-5h5M4 15h5v5"/></svg>';
  function placeMax() {
    if (!maxed) return;
    let r;
    if (isDesk()) r = document.querySelector('main > section').getBoundingClientRect();
    else {
      const top = Math.max(0, document.querySelector('header').getBoundingClientRect().bottom) + 8;
      const dock = $('dock').getBoundingClientRect();
      r = { top, left: 8, right: innerWidth - 8, bottom: (dock.height ? dock.top : innerHeight) - 8 };
    }
    Object.assign(maxed.style, { top: r.top + 'px', left: r.left + 'px',
                                 right: (innerWidth - r.right) + 'px', bottom: (innerHeight - r.bottom) + 'px' });
  }
  function maximize(el) {
    const was = maxed;
    if (was) {
      was.classList.remove('win-max');
      ['top', 'left', 'right', 'bottom'].forEach(k => { was.style[k] = ''; });
      const b = was.querySelector('.win-max-btn'); if (b) { b.innerHTML = ICON_MAX; b.title = 'ขยายเต็มพื้นที่'; }
      maxed = null;
    }
    if (el && el !== was) {
      maxed = el; el.classList.add('win-max'); placeMax();
      const b = el.querySelector('.win-max-btn'); if (b) { b.innerHTML = ICON_MIN; b.title = 'กลับขนาดเดิม (Esc)'; }
    }
    if (was || maxed) nudge();
  }
  addEventListener('resize', placeMax);
  addEventListener('keydown', (e) => { if (e.key === 'Escape' && maxed) maximize(null); });
  // A window: an element with a .cam-head title bar gets the maximise button.
  function winify(el) {
    if (el.classList.contains('win')) return;
    el.classList.add('win');
    const head = el.querySelector(':scope > .cam-head');
    if (!head) return;
    const b = document.createElement('button');
    b.className = 'win-max-btn'; b.innerHTML = ICON_MAX; b.title = 'ขยายเต็มพื้นที่';
    b.addEventListener('click', () => { maximize(el); b.blur(); });   // blur: keep Space for E-STOP
    head.appendChild(b);
    head.addEventListener('dblclick', (e) => { if (!e.target.closest('button, input, select, label')) maximize(el); });
  }

  // ---- Drag bars: between two windows (share of the row/column) and under a block (its height) ----
  // Remembered per browser; double-click a bar = back to the default.
  const splits = store.json('split', {});
  const saveSplits = () => store.setJson('split', splits);
  const resets = [];       // [bar, reset()] for the "จัดหน้าต่าง" button
  const extraResets = [];  // other resetters (split screen), see onWindowReset
  function dragBar(bar, axis, onStart, onMove, onEnd) {
    bar.addEventListener('pointerdown', (e) => {
      if (e.button) return;
      e.preventDefault();
      try { bar.setPointerCapture(e.pointerId); } catch (err) {}
      const st = onStart();
      bar.classList.add('drag'); document.body.classList.add('splitting', { x: 'sx', y: 'sy', xy: 'sxy' }[axis]);
      const move = (ev) => onMove(st, ev.clientX, ev.clientY);
      const up = () => {
        bar.removeEventListener('pointermove', move); bar.removeEventListener('pointerup', up);
        bar.removeEventListener('pointercancel', up);
        bar.classList.remove('drag'); document.body.classList.remove('splitting', 'sx', 'sy', 'sxy');
        if (onEnd) onEnd();
        saveSplits(); nudge();
      };
      bar.addEventListener('pointermove', move); bar.addEventListener('pointerup', up);
      bar.addEventListener('pointercancel', up);
    });
  }
  const frac = (v) => Math.min(0.9, Math.max(0.1, v));
  // Two children a, b of box side by side (axis 'x') or one over the other ('y'), a bar between
  // them. key: name the share is remembered under; def: a's share by default (0..1).
  function splitPair(box, a, b, axis, key, def) {
    box.classList.add('has-split'); a.classList.add('sa'); b.classList.add('sb');
    const bar = document.createElement('div');
    bar.className = 'split-bar ' + axis; bar.title = 'ลากเพื่อปรับขนาด · ดับเบิลคลิก = ค่าเดิม';
    box.insertBefore(bar, b);
    const set = (f) => { a.style.setProperty('--fa', String(f)); b.style.setProperty('--fb', String(1 - f)); };
    set(splits[key] ?? def);
    const ctl = {
      bar,
      begin: () => {
        const ra = a.getBoundingClientRect(), rb = b.getBoundingClientRect();
        return axis === 'x' ? { start: ra.left, total: ra.width + rb.width } : { start: ra.top, total: ra.height + rb.height };
      },
      move: (st, x, y) => {
        splits[key] = Math.round(frac(((axis === 'x' ? x : y) - st.start - 6) / st.total) * 1000) / 1000;
        set(splits[key]); nudge();
      },
      reset: () => { delete splits[key]; set(def); },
    };
    dragBar(bar, axis, ctl.begin, ctl.move);
    bar.addEventListener('dblclick', () => { ctl.reset(); saveSplits(); nudge(); });
    resets.push([bar, ctl.reset]);
    return ctl;
  }
  // Where a horizontal bar meets a vertical one (a T): a handle that moves both at once.
  const junctions = [];
  const watched = new Set();
  function junction(box, xCtl, yCtl) {
    const h = document.createElement('div');
    h.className = 'split-joint'; h.title = 'ลากเพื่อปรับทั้งกว้างและสูงพร้อมกัน · ดับเบิลคลิก = ค่าเดิม';
    box.classList.add('has-joint'); box.appendChild(h);
    dragBar(h, 'xy', () => ({ x: xCtl.begin(), y: yCtl.begin() }), (st, x, y) => {
      xCtl.move(st.x, x, y); yCtl.move(st.y, x, y); placeJunctions();
    });
    h.addEventListener('dblclick', () => { xCtl.reset(); yCtl.reset(); saveSplits(); nudge(); });
    junctions.push({ box, h, xCtl, yCtl });
    if (!watched.has(box) && window.ResizeObserver) { watched.add(box); new ResizeObserver(placeJunctions).observe(box); }
    setTimeout(placeJunctions, 0);
  }
  function placeJunctions() {
    junctions.forEach(({ box, h, xCtl, yCtl }) => {
      const show = isDesk() && xCtl.bar.getClientRects().length && yCtl.bar.getClientRects().length;
      h.hidden = !show;
      if (!show) return;
      const rb = box.getBoundingClientRect(), rx = xCtl.bar.getBoundingClientRect(), ry = yCtl.bar.getBoundingClientRect();
      h.style.left = (rx.left + rx.width / 2 - rb.left) + 'px';
      h.style.top = (ry.top + ry.height / 2 - rb.top) + 'px';
    });
  }
  addEventListener('resize', placeJunctions);
  addEventListener(PAGE_EVENT, () => setTimeout(placeJunctions, 0));
  const onWindowReset = (fn) => extraResets.push(fn);

  // ---- Shell: header height, control panel side, phone bottom sheet, dock E-STOP, #win-reset ----
  function initShell() {
    const main = document.querySelector('main'), hdr = document.querySelector('header'), btn = $('btn-side');
    const setHdr = () => document.documentElement.style.setProperty('--hdr-h', hdr.offsetHeight + 'px');
    setHdr();
    if (window.ResizeObserver) new ResizeObserver(setHdr).observe(hdr); else addEventListener('resize', setHdr);
    // the language button is added at DOMContentLoaded; do not rely on the observer alone for that
    document.addEventListener('DOMContentLoaded', setHdr);
    addEventListener('load', setHdr);
    if (btn) {
      const apply = (left) => {
        main.classList.toggle('ctrl-left', left);
        btn.textContent = left ? 'ย้ายไปขวา ⇆' : '⇆ ย้ายไปซ้าย';
      };
      let left = store.get('ctrlSide') === 'left';
      apply(left);
      btn.addEventListener('click', () => {
        left = !left; apply(left);
        store.set('ctrlSide', left ? 'left' : 'right');
        btn.blur();  // keep Space for E-STOP, not re-clicking this button
      });
    }

    // Phone: control panel is a bottom sheet toggled from the dock; E-STOP lives in the dock.
    const aside = $('ctrl-aside'), dockCtrl = $('dock-ctrl');
    const setSheet = (open) => {
      aside.classList.toggle('open', open);
      if (!dockCtrl) return;
      dockCtrl.classList.toggle('on', open);
      dockCtrl.textContent = open ? 'ซ่อน ▼' : 'ควบคุม ▲';
      dockCtrl.setAttribute('aria-expanded', String(open));
    };
    if (dockCtrl) dockCtrl.addEventListener('click', () => { setSheet(!aside.classList.contains('open')); dockCtrl.blur(); });
    if ($('btn-sheet-close')) $('btn-sheet-close').addEventListener('click', () => setSheet(false));
    if ($('dock-estop')) $('dock-estop').addEventListener('click', (e) => { $('btn-estop').click(); e.currentTarget.blur(); setSheet(true); });

    if ($('win-reset')) $('win-reset').addEventListener('click', (e) => {
      maximize(null);
      resets.forEach(([bar, reset]) => { if (bar.getClientRects().length) reset(); });
      extraResets.forEach(f => f());
      saveSplits(); nudge(); e.currentTarget.blur();
    });
  }

  // ---- Pages: one at a time on every screen (header tabs; on phones the bottom dock) ----
  // Elements carry data-pg="<page> [<page>...]"; the rules that hide the other pages' elements
  // are generated here from the tabs, so a robot only lists its pages in the HTML.
  //   home:     the first page (no #hash in the address)
  //   always:   pages whose panels in the control panel (aside) show on every page (driving)
  //   streams:  { page: ['toggle-button-id', ...] } streams owned by a page: switched off when
  //             leaving it and the ones that were on are switched on again on return
  //   defaults: { page: [...] } the ones switched on at the first visit
  //   deskHost: { page: 'element-id' } tablet/computer: that page's panels move out of the
  //             control panel into this element (the page gets them to itself)
  //   onLeave(old, page): called before switching
  let page = null;
  let go = () => {};
  function pageList() {
    const seen = [];
    document.querySelectorAll('#page-tabs .ptab, #dock .tab').forEach(t => {
      if (t.dataset.page && !seen.includes(t.dataset.page)) seen.push(t.dataset.page);
    });
    return seen;
  }
  function pageRules(pages, always) {
    const A = always.map(a => `:not([data-pg~="${a}"])`).join('');
    const phone = pages.map(p => `body[data-page="${p}"] [data-pg]:not([data-pg~="${p}"])`).join(',\n');
    const desk = pages.map(p => `body.desk[data-page="${p}"] section [data-pg]:not([data-pg~="${p}"]),\n` +
                                `body.desk[data-page="${p}"] aside > [data-pg]:not([data-pg~="${p}"])${A}`).join(',\n');
    return `@media ${PHONE} {\n${phone} { display: none; }\n}\n${desk} { display: none; }\n`;
  }
  function initPages(opt = {}) {
    const pages = pageList();
    const home = opt.home || pages[0], always = opt.always || [home];
    const OWN = opt.streams || {}, DEFAULT = opt.defaults || {}, HOST = opt.deskHost || {};
    const style = document.createElement('style');
    style.id = 'ui-page-rules';
    style.textContent = pageRules(pages, always);
    document.head.appendChild(style);

    const phone = matchMedia(PHONE);
    const tabs = [...document.querySelectorAll('#dock .tab, #page-tabs .ptab')];
    const sec = document.querySelector('main > section'), aside = $('ctrl-aside');
    const hosted = Object.entries(HOST).map(([p, id]) =>
      ({ p, host: $(id), panels: [...aside.querySelectorAll(`:scope > [data-pg="${p}"]`)] }));
    const wasOn = {};
    page = document.body.dataset.page || home;
    const isOn = (b) => b && b.classList.contains('on');
    const click = (id, wantOn) => { const b = $(id); if (b && isOn(b) !== wantOn) b.click(); };
    // Tablet/computer: those panels get the page to themselves; phone: they stay in the panel list.
    function placeHosted() {
      hosted.forEach(({ p, host, panels }) => {
        const to = !phone.matches && page === p ? host : aside;
        panels.forEach(el => { if (el.parentElement !== to) to.appendChild(el); });
      });
    }
    function layout() { document.body.classList.toggle('desk', !phone.matches); placeHosted(); placeMax(); }
    go = (p) => {
      if (p === page) return;
      const old = page; page = p;
      maximize(null);
      document.body.dataset.page = p;
      tabs.forEach(t => t.classList.toggle('on', t.dataset.page === p));
      try { history.replaceState(null, '', p === home ? location.pathname + location.search : '#' + p); } catch (e) {}
      if (opt.onLeave) opt.onLeave(old, p);
      if (OWN[old]) { wasOn[old] = OWN[old].filter(id => isOn($(id))); wasOn[old].forEach(id => click(id, false)); }
      setTimeout(() => (wasOn[p] || DEFAULT[p] || []).forEach(id => click(id, true)), 80);
      dispatchEvent(new CustomEvent(PAGE_EVENT, { detail: { page: p, old } }));
      placeHosted();
      scrollTo(0, 0); sec.scrollTop = 0;
      nudge();
    };
    tabs.forEach(t => t.addEventListener('click', () => { go(t.dataset.page); t.blur(); }));
    if (phone.addEventListener) phone.addEventListener('change', () => { layout(); nudge(); });
    layout();
    const first = location.hash.slice(1);   // reload keeps the page
    if (first !== home && pages.includes(first)) go(first);
    if (opt.wheel !== false) initWheel(pages);
    return go;
  }

  // ---- Mouse wheel past the top/bottom of a page: go to the previous/next page ----
  // Only once the page (and whatever is under the mouse) cannot scroll further that way; the
  // bar at the bottom shows how far, so a normal scroll never switches by accident.
  function initWheel(ORDER) {
    const NEED = 400;   // px of wheel past the edge (about 4 notches)
    const hint = $('page-hint');
    if (!hint) return;
    const name = (p) => { const t = document.querySelector(`#page-tabs .ptab[data-page="${p}"]`); return t ? t.textContent.trim() : p; };
    let acc = 0, dir = 0, timer = null, lockUntil = 0;
    const canScroll = (el, dy) => {
      for (; el && el !== document.documentElement; el = el.parentElement) {
        if (el.scrollHeight <= el.clientHeight + 1) continue;
        if (!/(auto|scroll)/.test(getComputedStyle(el).overflowY) && el !== document.body) continue;
        if (dy > 0 ? el.scrollTop + el.clientHeight < el.scrollHeight - 1 : el.scrollTop > 0) return true;
      }
      const doc = document.scrollingElement;
      return dy > 0 ? doc.scrollTop + innerHeight < doc.scrollHeight - 1 : doc.scrollTop > 0;
    };
    const reset = () => { acc = 0; dir = 0; hint.classList.remove('show'); hint.style.setProperty('--p', '0%'); };
    addEventListener('wheel', (e) => {
      if (e.defaultPrevented || e.ctrlKey || Math.abs(e.deltaY) <= Math.abs(e.deltaX)) return;
      const t = e.target;
      if (!t.closest || !t.closest('main') || t.closest('aside, input, select, textarea, .win-max')) return;
      if (Date.now() < lockUntil) return;
      const dy = e.deltaMode === 1 ? e.deltaY * 33 : e.deltaMode === 2 ? e.deltaY * 600 : e.deltaY;
      if (canScroll(t, dy)) { reset(); return; }
      const d = Math.sign(dy), next = ORDER[ORDER.indexOf(document.body.dataset.page) + d];
      if (!next) return;
      if (d !== dir) { acc = 0; dir = d; }
      acc += Math.min(Math.abs(dy), 150);
      hint.querySelector('span').textContent = (d > 0 ? 'เลื่อนต่อเพื่อไปหน้า ' : 'เลื่อนต่อเพื่อกลับไปหน้า ') + name(next);
      hint.style.setProperty('--p', Math.min(100, acc / NEED * 100) + '%');
      hint.classList.add('show');
      clearTimeout(timer); timer = setTimeout(reset, 600);
      if (acc >= NEED) { reset(); lockUntil = Date.now() + 800; go(next); }
    }, { passive: true });
  }

  // ---- Camera view ----
  // A camera shown in `view`; returns a setter that switches it to another camera (or off).
  // Frames come as JPEG messages on a WebSocket rather than an MJPEG <img>: browsers keep at
  // most 6 HTTP connections per host and do not always drop an MJPEG stream when its src
  // changes, so switching cameras used to leave new streams waiting forever (frozen view).
  function streamView(view) {
    let sock = null, img = null, shownUrl = null, loadingUrl = null, next = null, decoding = false, retry = null;
    // One ack per frame shown or dropped: the server sends only a couple of frames ahead of
    // the acks, so on slow Wi-Fi the view drops frames instead of lagging behind.
    const ack = () => { if (sock && sock.readyState === WebSocket.OPEN) sock.send('1'); };
    const show = (blob) => {
      if (decoding) { if (next) ack(); next = blob; return; }   // keep only the newest frame while one decodes
      decoding = true;
      const url = loadingUrl = URL.createObjectURL(blob);
      img.onload = img.onerror = () => {
        if (shownUrl) URL.revokeObjectURL(shownUrl);
        shownUrl = url; loadingUrl = null; decoding = false;
        ack();
        if (next) { const b = next; next = null; show(b); }
      };
      img.src = url;
    };
    const stop = () => {
      clearTimeout(retry);
      if (sock) { sock.onmessage = sock.onclose = null; sock.close(); sock = null; }
      if (img) { img.onload = img.onerror = null; img = null; }
      [shownUrl, loadingUrl].forEach(u => { if (u) URL.revokeObjectURL(u); });
      shownUrl = loadingUrl = next = null; decoding = false;
    };
    return (name, alt) => {
      stop();
      if (!name) { view.innerHTML = '<span>ปิดอยู่</span>'; return; }
      view.innerHTML = '<span>กำลังรอภาพ… (ถ้านานผิดปกติ node กล้องทำงานอยู่ไหม?)</span>';
      const open = () => {
        const s = sock = new WebSocket(wsUrl(`/ws/stream/${encodeURIComponent(name)}`));
        s.onmessage = (ev) => {
          if (!img) { img = new Image(); img.alt = alt || name; view.replaceChildren(img); }
          show(ev.data);
        };
        s.onclose = () => {   // server restarted or network dropped: keep the last frame, retry
          if (sock !== s) return;
          sock = null;
          retry = setTimeout(open, 1000);
        };
      };
      open();
    };
  }

  // ---- Split-screen camera page ----
  // 1-9 tiles in #grid, each showing any camera from /api/info; bars between the tiles resize
  // them. Tiles stream only while `page` is open. Controls: #split-n, #split-fit, #split-even.
  //   cameras: [{name, label}] (from /api/info), labels: { name: shown name }, defaults: tile order
  const SPLIT_LAYOUTS = { 1: [1, 1], 2: [2, 1], 3: [3, 1], 4: [2, 2], 6: [3, 2], 9: [3, 3] };
  function splitScreen(cameras, opt = {}) {
    const LABELS = opt.labels || {}, PAGE = opt.page || 'cams';
    const box = $(opt.grid || 'grid'), names = cameras.map(c => c.name);
    const st = Object.assign({ n: 4, tiles: [], cols: {}, rows: {} }, store.json('splitview', {}));
    if (!SPLIT_LAYOUTS[st.n]) st.n = 4;
    const save = () => store.setJson('splitview', st);
    const spare = (opt.defaults || []).filter(n => names.includes(n)).concat(names);
    const tiles = [];
    for (let i = 0; i < 9; i++) {
      if (!names.includes(st.tiles[i])) st.tiles[i] = spare[i % spare.length];
      const el = document.createElement('div');
      el.className = 'cam tile';
      el.innerHTML = '<div class="cam-head"><div><select title="เลือกกล้องของช่องนี้"></select></div></div>' +
                     '<div class="view"><span>ปิดอยู่</span></div>';
      const sel = el.querySelector('select');
      cameras.forEach(c => sel.add(new Option(LABELS[c.name] || c.label, c.name)));
      sel.value = st.tiles[i];
      const t = { el, stream: streamView(el.querySelector('.view')), on: null };
      sel.onchange = () => { st.tiles[i] = sel.value; save(); refresh(); sel.blur(); };
      box.appendChild(el); winify(el); tiles.push(t);
    }
    let bars = [];
    const dims = () => SPLIT_LAYOUTS[st.n];
    function tracks() {
      const [c, r] = dims();
      const cf = st.cols[st.n] || Array(c).fill(1), rf = st.rows[st.n] || Array(r).fill(1);
      box.style.setProperty('--cols', cf.map(f => `minmax(0, ${f}fr)`).join(' 12px '));
      box.style.setProperty('--rows', rf.map(f => `minmax(0, ${f}fr)`).join(' 12px '));
    }
    function gridBar(axis, k) {
      const [c, r] = dims(), list = axis === 'x' ? 'cols' : 'rows', count = axis === 'x' ? c : r;
      const bar = document.createElement('div');
      bar.className = 'split-bar ' + axis; bar.title = 'ลากเพื่อปรับขนาด · ดับเบิลคลิก = เท่ากัน';
      bar.style.setProperty(axis === 'x' ? '--gc' : '--gr', String(2 * k + 2));
      bar.style.setProperty(axis === 'x' ? '--gr' : '--gc', '1 / -1');
      box.appendChild(bar); bars.push(bar);
      const ctl = {
        begin: () => {
          const ra = tiles[axis === 'x' ? k : k * c].el.getBoundingClientRect();
          const rb = tiles[axis === 'x' ? k + 1 : (k + 1) * c].el.getBoundingClientRect();
          const fr = (st[list][st.n] || Array(count).fill(1)).slice();
          return axis === 'x' ? { fr, start: ra.left, total: ra.width + rb.width } : { fr, start: ra.top, total: ra.height + rb.height };
        },
        move: (s0, x, y) => {
          const f = frac(((axis === 'x' ? x : y) - s0.start - 6) / s0.total), sum = s0.fr[k] + s0.fr[k + 1], fr = s0.fr.slice();
          fr[k] = sum * f; fr[k + 1] = sum * (1 - f);
          st[list][st.n] = fr.map(v => Math.round(v * 1000) / 1000);
          tracks(); nudge();
        },
        reset: () => { delete st[list][st.n]; },
      };
      dragBar(bar, axis, ctl.begin, ctl.move, save);
      bar.addEventListener('dblclick', () => { ctl.reset(); save(); tracks(); nudge(); });
      return ctl;
    }
    // where a column bar crosses a row bar: one handle moves both
    function gridJoint(cx, ry, kx, ky) {
      const h = document.createElement('div');
      h.className = 'split-joint'; h.title = 'ลากเพื่อปรับทั้งกว้างและสูงพร้อมกัน · ดับเบิลคลิก = เท่ากัน';
      h.style.setProperty('--gc', String(2 * kx + 2)); h.style.setProperty('--gr', String(2 * ky + 2));
      box.appendChild(h); bars.push(h);
      dragBar(h, 'xy', () => ({ x: cx.begin(), y: ry.begin() }), (s0, x, y) => { cx.move(s0.x, x, y); ry.move(s0.y, x, y); }, save);
      h.addEventListener('dblclick', () => { cx.reset(); ry.reset(); save(); tracks(); nudge(); });
    }
    function layout() {
      bars.forEach(b => b.remove()); bars = [];
      const [c, r] = dims();
      tracks();
      tiles.forEach((t, i) => {
        t.el.hidden = i >= st.n;
        t.el.style.setProperty('--gc', String(2 * (i % c) + 1));
        t.el.style.setProperty('--gr', String(2 * Math.floor(i / c) + 1));
      });
      const colBars = [], rowBars = [];
      for (let k = 0; k < c - 1; k++) colBars.push(gridBar('x', k));
      for (let k = 0; k < r - 1; k++) rowBars.push(gridBar('y', k));
      colBars.forEach((cb, kx) => rowBars.forEach((rb, ky) => gridJoint(cb, rb, kx, ky)));
      document.querySelectorAll('#split-n button').forEach(b => b.classList.toggle('on', +b.dataset.n === st.n));
      refresh(); nudge();
    }
    function refresh() {
      const active = document.body.dataset.page === PAGE;
      tiles.forEach((t, i) => {
        const want = active && i < st.n ? st.tiles[i] : null;
        if (want === t.on) return;
        t.on = want;
        t.stream(want, want ? (LABELS[want] || want) : '');
      });
    }
    document.querySelectorAll('#split-n button').forEach(b => {
      b.onclick = () => { maximize(null); st.n = +b.dataset.n; save(); layout(); b.blur(); };
    });
    const even = () => { delete st.cols[st.n]; delete st.rows[st.n]; save(); tracks(); };
    const setFit = (f) => {
      st.fit = f; save();
      box.classList.remove('fit-cover', 'fit-contain', 'fit-fill'); box.classList.add('fit-' + f);
      document.querySelectorAll('#split-fit button').forEach(b => b.classList.toggle('on', b.dataset.fit === f));
    };
    document.querySelectorAll('#split-fit button').forEach(b => b.onclick = () => { setFit(b.dataset.fit); b.blur(); });
    setFit(['cover', 'contain', 'fill'].includes(st.fit) ? st.fit : 'cover');
    if ($('split-even')) $('split-even').onclick = (e) => { even(); nudge(); e.currentTarget.blur(); };
    onWindowReset(() => { if (document.body.dataset.page === PAGE) even(); });
    addEventListener(PAGE_EVENT, refresh);
    layout();
  }

  // ---- Status chips and the status WebSocket ----
  function setChip(id, text, cls) {
    const c = $(id); c.textContent = text; c.className = 'chip' + (cls ? ' ' + cls : '');
  }
  // JSON messages both ways, reconnects after 1 s. h: { chip, onopen, onclose, onmessage(msg) }
  // chip: id of the connection chip (default 'chip-conn', set to null for none).
  function connect(path, h = {}) {
    const chip = h.chip === undefined ? 'chip-conn' : h.chip;
    let ws = null;
    const open = () => {
      ws = new WebSocket(wsUrl(path));
      ws.onopen = () => { if (chip && $(chip)) setChip(chip, 'เชื่อมต่อแล้ว', 'ok'); if (h.onopen) h.onopen(); };
      ws.onclose = () => {
        if (chip && $(chip)) setChip(chip, 'การเชื่อมต่อหลุด', 'bad');
        if (h.onclose) h.onclose();
        setTimeout(open, 1000);
      };
      ws.onmessage = (ev) => { if (h.onmessage) h.onmessage(JSON.parse(ev.data)); };
    };
    open();
    return { send: (obj) => { if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj)); } };
  }

  // ---- Driving inputs: joystick and keys ----
  // Both are released when the window loses focus or the tab is hidden.
  const releases = [];
  const releaseAll = () => releases.forEach(f => f());
  window.addEventListener('blur', releaseAll);
  document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });
  // #pad with a #knob inside. Returns { active, x, y } (x right, y up, -1..1), kept up to date.
  // canStart(): whether a touch may grab the knob (e.g. only while this browser has control).
  function joystick(padEl, knob, canStart = () => true) {
    const pad = { active: false, x: 0, y: 0 };
    function moveKnob(x, y) {
      const r = padEl.clientWidth / 2 - knob.offsetWidth / 2;
      knob.style.transform = `translate(${x * r}px, ${-y * r}px)`;
    }
    pad.reset = () => { pad.active = false; pad.x = 0; pad.y = 0; moveKnob(0, 0); };
    function fromEvent(e) {
      const rect = padEl.getBoundingClientRect();
      const r = rect.width / 2;
      let x = (e.clientX - rect.left - r) / r;
      let y = -(e.clientY - rect.top - r) / r;
      const len = Math.hypot(x, y);
      if (len > 1) { x /= len; y /= len; }
      pad.x = x; pad.y = y; moveKnob(x, y);
    }
    padEl.addEventListener('pointerdown', (e) => {
      if (!canStart()) return;
      pad.active = true; padEl.setPointerCapture(e.pointerId); fromEvent(e);
    });
    padEl.addEventListener('pointermove', (e) => { if (pad.active) fromEvent(e); });
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(t => padEl.addEventListener(t, pad.reset));
    releases.push(pad.reset);
    return pad;
  }
  // W A S D / arrows -> a Set of 'up' 'down' 'left' 'right'; Space -> onEstop().
  // skip(): true while the keys belong to something else (e.g. a labelling tool). Space stays E-STOP.
  const KEYMAP = { KeyW: 'up', ArrowUp: 'up', KeyS: 'down', ArrowDown: 'down',
                   KeyA: 'left', ArrowLeft: 'left', KeyD: 'right', ArrowRight: 'right' };
  function driveKeys(opt = {}) {
    const keys = new Set();
    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
      if (e.code === 'Space') { e.preventDefault(); if (opt.onEstop) opt.onEstop(); return; }
      if (opt.skip && opt.skip()) return;
      const k = KEYMAP[e.code];
      if (k) { e.preventDefault(); keys.add(k); }
    });
    window.addEventListener('keyup', (e) => { const k = KEYMAP[e.code]; if (k) keys.delete(k); });
    releases.push(() => keys.clear());
    return keys;
  }

  window.RobotUI = {
    $, store, PHONE, PAGE_EVENT, wsUrl,
    nudge, maximize, winify, dragBar, frac, splitPair, junction, placeJunctions, onWindowReset,
    initShell, initPages, go: (p) => go(p), page: () => page,
    streamView, splitScreen, setChip, connect, joystick, driveKeys,
  };
})();
