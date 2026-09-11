/* The viewer as the ground station serves it -- over HTTP, on a phone.

   viewer_checks.js loads the page as a local file, which is Web Serial mode,
   so it never touches the network path. That path now sends commands to a
   real actuator, so it gets its own run: location is http:, EventSource and
   fetch are fakes that record what the page does, and the checks are about
   which commands go where, and when the page refuses to send them at all.

   Driven by run_checks.py, for the dual-transport build only.
       node viewer_net_checks.js <extracted.js> */
const fs = require('fs');
const src = fs.readFileSync(process.argv[2], 'utf8');

/* ---- stub DOM ---------------------------------------------------------- */
const els = new Map();
function mkEl(id) {
  const e = {
    id, textContent: '', innerHTML: '', disabled: false, style: {}, value: '',
    hidden: false, checked: false, options: [], children: [],
    _cls: new Set(),
    classList: {
      add: (c) => e._cls.add(c), remove: (c) => e._cls.delete(c),
      toggle: (c, on) => { if (on) e._cls.add(c); else e._cls.delete(c); },
      contains: (c) => e._cls.has(c),
    },
    _h: {},
    addEventListener: (t, f) => { (e._h[t] = e._h[t] || []).push(f); },
    setAttribute: () => {}, getAttribute: () => null, removeAttribute: () => {},
    setPointerCapture: () => {}, appendChild: (c) => { e.children.push(c); return c; },
    removeChild: () => {},
    getBoundingClientRect: () => ({ width: 800, height: 480 }),
    getContext: () => ctx2d,
    scrollHeight: 0, scrollTop: 0, clientHeight: 0, width: 800, height: 480,
  };
  return e;
}
const getEl = (id) => { if (!els.has(id)) els.set(id, mkEl(id)); return els.get(id); };
const ctx2d = new Proxy({}, {
  get(t, k) {
    if (k === 'measureText') return (s) => ({ width: String(s).length * 6 });
    return () => undefined;
  },
  set() { return true; },
});

// The source selector needs real options for syncSource() to pick from.
const sel = getEl('selMode');
sel.options = ['uart', 'bench', 'flight', 'still'].map(v => ({ value: v }));

global.window = global;
global.document = {
  getElementById: getEl, createElement: () => mkEl(''),
  querySelector: () => getEl('__q'), querySelectorAll: () => [],
  addEventListener: () => {}, hidden: false, body: mkEl('body'),
};
global.location = { protocol: 'http:', href: 'http://192.168.4.1/' };
global.navigator = {};
global.performance = { now: () => Date.now() };
global.requestAnimationFrame = () => 1;
global.addEventListener = () => {};
global.matchMedia = () => ({ matches: false, addEventListener: () => {}, addListener: () => {} });
global.devicePixelRatio = 1;
global.localStorage = { getItem: () => null, setItem: () => {}, removeItem: () => {} };
global.setInterval = () => 0;
global.clearInterval = () => {};
global.TextEncoder = require('util').TextEncoder;

/* fetch: records every URL, answers from a table the checks can change */
const calls = [];
let health = { mode: 'uart', source: 'uart', link: { last_line_age_ms: 40 } };
let cmdReply = (c) => ({ ok: true, sent: c });
global.fetch = (url) => {
  calls.push(url);
  let body;
  if (url.startsWith('/health')) body = health;
  else if (url.startsWith('/cmd')) body = cmdReply(decodeURIComponent(url.split('c=')[1] || ''));
  else body = { ok: true };
  return Promise.resolve({ status: body.ok === false ? 400 : 200, json: () => Promise.resolve(body) });
};

/* EventSource: one fake, whose listeners the checks can fire */
let es = null;
global.EventSource = function (url) {
  es = this; this.url = url; this.l = {};
  this.addEventListener = (t, f) => { this.l[t] = f; };
  this.close = () => {};
};

const settle = () => new Promise(r => setTimeout(r, 0));
const fails = [];
const check = (c, m) => { if (!c) { fails.push(m); console.log('  FAIL ' + m); } };

let api;
try {
  api = new Function(src + `
    ;return { sendChar, syncSource, get depArmed(){return depArmed},
              depSetArmed, get conBuf(){return conBuf}, get NET_MODE(){return NET_MODE} };`)();
} catch (e) { console.log('LOAD FAILED: ' + e.message); process.exit(1); }

(async () => {
  check(api.NET_MODE === true, 'page did not detect it was served over HTTP');
  check(es && es.url === '/stream', 'the page did not open the telemetry stream');

  /* ---- connecting proves nothing about data ------------------------- */
  es.onopen();
  await settle(); await settle();
  check(getEl('statusText').textContent.indexOf('waiting') >= 0,
        'status claimed something other than "waiting" before any frame: "'
        + getEl('statusText').textContent + '"');
  console.log('  connect: status waits for telemetry instead of claiming Live');

  /* ---- on the wire, the board controls come alive ------------------- */
  check(getEl('cmdZ').disabled === false, 'board commands stayed disabled on the wired source');
  check(getEl('depArm').disabled === false, 'the arm button stayed disabled on the wired source');

  calls.length = 0;
  check(await api.sendChar('!fire\n') === true, 'an accepted command did not report success');
  check(calls[0] === '/cmd?c=!fire', 'fire went to ' + calls[0]);
  await api.sendChar('!us 1500\n');
  check(calls[1] === '/cmd?c=!us%201500', 'the pulse-width command went to ' + calls[1]);
  await api.sendChar('z');
  check(calls[2] === '/cmd?c=z', 'zero went to ' + calls[2]);
  console.log('  commands: routed to /cmd, encoded, trailing newline stripped');

  /* ---- a refusal is shown, not swallowed ---------------------------- */
  cmdReply = () => ({ ok: false, why: 'not an allowed command' });
  const n0 = api.conBuf.length;
  check(await api.sendChar('!nonsense\n') === false, 'a refused command reported success');
  check(api.conBuf.slice(n0).some(l => l.indexOf('refused by the ground station') >= 0),
        'the refusal never reached the console');
  cmdReply = (c) => ({ ok: true, sent: c });
  console.log('  refusal: reported to the console, and not counted as sent');

  /* ---- board messages land in the console --------------------------- */
  es.l.msg({ data: '  SERVO FIRED -> 2000 us' });
  check(api.conBuf[api.conBuf.length - 1] === '  SERVO FIRED -> 2000 us',
        'a board message did not reach the console');
  console.log('  messages: the board’s own words appear on the phone');

  /* ---- switch to synthetic: controls die, and an armed latch disarms - */
  api.depSetArmed(true, 'test');
  check(api.depArmed === true, 'could not arm on the wired source');
  health = { mode: 'bench', source: 'synthetic', link: { last_line_age_ms: -1 } };
  calls.length = 0;
  api.syncSource();
  await settle(); await settle();
  check(getEl('cmdZ').disabled === true, 'board commands stayed live on a synthetic source');
  check(getEl('depArm').disabled === true, 'the arm button stayed live on a synthetic source');
  check(api.depArmed === false, 'the latch stayed armed after the source went synthetic');
  check(calls.indexOf('/cmd?c=!safe') >= 0, 'no !safe was sent when the source went synthetic');
  console.log('  synthetic source: controls disabled, armed latch told to safe');

  console.log();
  if (fails.length) { console.log(fails.length + ' CHECK(S) FAILED'); process.exit(1); }
  console.log('all network-mode viewer checks passed');
})();
