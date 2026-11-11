// Language switch (ไทย / English) for the whole web page.
//
// The page is written in Thai. In English mode this translates every piece of Thai text that
// shows up (page text, tooltips, messages from the server, dialogs, text drawn on canvases)
// with window.QCAR_EN (i18n_en.js, generated from i18n_en.tsv: Thai phrase -> English).
// Text built from several phrases is translated phrase by phrase; a phrase the dictionary does
// not have is split into the longest known phrases it contains, and anything left stays Thai.
// Switching language stores the choice and reloads the page.
(() => {
  'use strict';
  const KEY = 'qcar2.lang';
  let lang = 'th';
  try { lang = localStorage.getItem(KEY) === 'en' ? 'en' : 'th'; } catch (e) {}
  window.qcarLang = lang;
  document.documentElement.lang = lang;

  // the switch, top right of the header (added once the header exists)
  function addSwitch() {
    const header = document.querySelector('header');
    if (!header || document.getElementById('lang-switch')) return;
    const b = document.createElement('button');
    b.id = 'lang-switch';
    b.setAttribute('data-notr', '');
    b.textContent = lang === 'en' ? 'ไทย' : 'EN';
    b.title = lang === 'en' ? 'เปลี่ยนเป็นภาษาไทย' : 'Switch to English';
    b.onclick = () => {
      try { localStorage.setItem(KEY, lang === 'en' ? 'th' : 'en'); } catch (e) {}
      location.reload();
    };
    header.appendChild(b);
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', addSwitch);
  else addSwitch();

  if (lang !== 'en') return;
  const DICT = window.QCAR_EN || {};
  const THAI = /[฀-๿]/;
  const RUN = /[฀-๿]+(?:[  /-]*[฀-๿]+)*/g;
  // longest-first phrase lists by first character, for runs that are not in the dictionary whole
  const byFirst = new Map();
  Object.keys(DICT).sort((a, b) => b.length - a.length).forEach(k => {
    const c = k[0];
    if (!byFirst.has(c)) byFirst.set(c, []);
    byFirst.get(c).push(k);
  });
  function runTr(run) {
    if (Object.prototype.hasOwnProperty.call(DICT, run)) return DICT[run];
    const parts = [];
    let i = 0, rest = '';
    while (i < run.length) {
      const ch = run[i];
      if (ch === ' ' || ch === ' ') { i++; continue; }
      const cands = byFirst.get(ch) || [];
      const hit = cands.find(k => run.startsWith(k, i));
      if (hit) {
        if (rest) { parts.push(rest); rest = ''; }
        if (DICT[hit]) parts.push(DICT[hit]);
        i += hit.length;
      } else { rest += ch; i++; }
    }
    if (rest) parts.push(rest);
    return parts.join(' ');
  }
  // Whole phrases that mix Thai with English words or numbers (word order differs), applied
  // before the phrase-by-phrase translation. Specific ones first.
  const T = '(?![\u0E00-\u0E7F])';
  const PRE = [
    [/ยังไม่ label/g, 'not labelled'], [/label แล้ว/g, 'labelled'],
    [/label อัตโนมัติทั้งชุด/g, 'Auto-label all'],
    [/เฉพาะ class ที่เลือก(อยู่)?/g, 'only the selected class'], [/class ที่เลือก(อยู่)?/g, 'the selected class'],
    [/ถ่าย (\d+) กล้อง/g, 'Capture $1 cameras'], [/พร้อม (\d+)\/(\d+) กล้อง/g, '$1/$2 cameras ready'],
    [/พร้อม (\d+) กล้อง/g, '$1 cameras ready'], [/กล้อง CSI (\d+) ตัว/g, '$1 CSI cameras'],
    [/ตำแหน่งกล้อง (\d+) ตัว/g, '$1 camera positions'],
    [/รอบ (\d+)\/(\d+|\?)/g, 'epoch $1/$2'], [/รอบ \(epochs\)/g, 'Epochs'],
    [/หยุดเองถ้าไม่ดีขึ้น \(รอบ\)/g, 'Early stop after (epochs)'],
    [/เริ่มใหม่จากโมเดล COCO/g, 'Fresh start from the COCO model'],
    [new RegExp('(\\d+) ภาพ' + T, 'g'), '$1 images'], [new RegExp('(\\d+) กรอบ' + T, 'g'), '$1 boxes'],
    [new RegExp('(\\d+) กล้อง' + T, 'g'), '$1 cameras'],
    [new RegExp('(\\d+(?:\\.\\d+)?) วินาที' + T, 'g'), '$1 s'], [new RegExp('(\\d+(?:\\.\\d+)?) วิ' + T, 'g'), '$1 s'],
  ];
  const cache = new Map();
  function tr(s) {
    if (typeof s !== 'string' || !THAI.test(s)) return s;
    let out = cache.get(s);
    if (out === undefined) {
      const src = PRE.reduce((t, [re, to]) => t.replace(re, to), s);
      out = src.replace(RUN, (m, off) => {
        let t = runTr(m);
        const before = src[off - 1], after = src[off + m.length];
        if (t && before && /[A-Za-z0-9)]/.test(before)) t = ' ' + t;
        if (t && after && /[A-Za-z0-9(]/.test(after)) t = t + ' ';
        return t;
      }).replace(/ {2,}/g, ' ');
      if (cache.size > 5000) cache.clear();
      cache.set(s, out);
    }
    return out;
  }
  window.qcarTr = tr;

  // ---- the page
  const ATTRS = ['title', 'placeholder', 'alt', 'aria-label'];
  const skip = (el) => el && (el.closest('script, style, textarea, [data-notr]'));
  function doText(node) {
    const p = node.parentElement;
    if (!p || skip(p)) return;
    const v = node.data, t = tr(v);
    if (t !== v && allow(node, v)) node.data = t;
  }
  // Only write a changed value: setting an attribute, even to the same value, is a mutation of
  // its own, and text with Thai left untranslated would otherwise go round forever.
  // Guard: something on the page that keeps putting the Thai text back the moment it is
  // translated (it reads the text it wrote and corrects it) would go round forever. After 50
  // rewrites of the same node within a second, leave that node alone (window.qcarTrLoops).
  const writes = new WeakMap();
  window.qcarTrLoops = [];
  function allow(node, text) {
    const now = performance.now();
    let w = writes.get(node);
    if (!w || now - w.t > 1000) { w = { t: now, n: 0 }; writes.set(node, w); }
    if (++w.n === 50) window.qcarTrLoops.push(text.slice(0, 120));
    return w.n < 50;
  }
  function setAttr(el, a) {
    const v = el.getAttribute(a);
    if (!v || !THAI.test(v)) return;
    const t = tr(v);
    if (t !== v && allow(el, a + ': ' + v)) el.setAttribute(a, t);
  }
  function doAttrs(el) {
    for (const a of ATTRS) setAttr(el, a);
    if (el.tagName === 'INPUT' && (el.type === 'button' || el.type === 'submit') && THAI.test(el.value)) {
      const t = tr(el.value);
      if (t !== el.value) el.value = t;
    }
  }
  function walk(root) {
    if (root.nodeType === 3) { doText(root); return; }
    if (root.nodeType !== 1 || skip(root)) return;
    doAttrs(root);
    const it = document.createTreeWalker(root, NodeFilter.SHOW_TEXT | NodeFilter.SHOW_ELEMENT);
    for (let n = it.nextNode(); n; n = it.nextNode()) {
      if (n.nodeType === 3) doText(n);
      else if (!skip(n)) doAttrs(n);
    }
  }
  new MutationObserver((list) => {
    for (const m of list) {
      if (m.type === 'childList') m.addedNodes.forEach(walk);
      else if (m.type === 'characterData') doText(m.target);
      else if (m.type === 'attributes' && m.target.nodeType === 1 && !skip(m.target)) setAttr(m.target, m.attributeName);
    }
  }).observe(document.documentElement, { childList: true, subtree: true, characterData: true,
                                          attributes: true, attributeFilter: ATTRS });
  document.addEventListener('DOMContentLoaded', () => walk(document.body));
  if (document.body) walk(document.body);

  // ---- dialogs and canvas text
  // only the message: a prompt's default value is data (e.g. a class name being renamed)
  const wrap = (fn) => function (msg, ...rest) { return fn.call(this, tr(String(msg ?? '')), ...rest); };
  window.alert = wrap(window.alert);
  window.confirm = wrap(window.confirm);
  window.prompt = wrap(window.prompt);
  const C2D = window.CanvasRenderingContext2D && CanvasRenderingContext2D.prototype;
  if (C2D) ['fillText', 'strokeText', 'measureText'].forEach(name => {
    const orig = C2D[name];
    C2D[name] = function (text, ...rest) { return orig.call(this, tr(String(text)), ...rest); };
  });
})();
