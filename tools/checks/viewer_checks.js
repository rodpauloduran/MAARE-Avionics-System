/* Load a viewer build's script under Node with a stub DOM, feed it real wire
   lines, and check the parser, the trajectory model and a render pass.

   Driven by run_checks.py; run that rather than this directly.
       node viewer_checks.js <extracted.js> <frames.txt>

   The DOM stub is deliberately thin. It is not trying to be a browser -- it
   only has to be enough that the script initialises, so that the parsing and
   trajectory logic underneath can be exercised against real frames. A render
   pass is counted, not inspected: the point is that it completes without
   throwing on both stage modes. */
const fs = require('fs');

const scriptPath = process.argv[2];
const framesPath = process.argv[3];
const src = fs.readFileSync(scriptPath, 'utf8');
const frames = fs.readFileSync(framesPath, 'utf8').split('\n').filter(Boolean);

/* ---- stub DOM ---------------------------------------------------------- */
const els = new Map();
function mkEl(id) {
  const e = {
    id, textContent: '', innerHTML: '', disabled: false, style: {},
    _cls: new Set(),
    classList: {
      add: (c) => e._cls.add(c),
      remove: (c) => e._cls.delete(c),
      toggle: (c, on) => { if (on) e._cls.add(c); else e._cls.delete(c); },
      contains: (c) => e._cls.has(c),
    },
    _handlers: {},
    addEventListener: (t, f) => { (e._handlers[t] = e._handlers[t] || []).push(f); },
    setPointerCapture: () => {},
    getBoundingClientRect: () => ({ width: 800, height: 480 }),
    getContext: () => ctx2d,
    width: 800, height: 480,
  };
  return e;
}
function getEl(id) {
  if (!els.has(id)) els.set(id, mkEl(id));
  return els.get(id);
}

const ctxCalls = { fillText: 0, stroke: 0, arc: 0, moveTo: 0, lineTo: 0 };
const ctx2d = new Proxy({}, {
  get(t, k) {
    if (k === 'canvas') return getEl('view');
    return (...a) => { if (k in ctxCalls) ctxCalls[k]++; return undefined; };
  },
  set() { return true; },
});

let rafCb = null;
global.window = global;
global.document = {
  getElementById: getEl,
  querySelector: () => getEl('__q'),
  querySelectorAll: () => [],
  addEventListener: () => {},
  hidden: false,
  body: mkEl('body'),
};
global.location = { protocol: 'file:', href: 'file:///x.html' };
global.navigator = {};
global.performance = { now: () => Date.now() };
global.requestAnimationFrame = (cb) => { rafCb = cb; return 1; };
global.addEventListener = () => {};
global.matchMedia = () => ({ matches: false, addEventListener: () => {}, addListener: () => {} });
global.devicePixelRatio = 1;
global.localStorage = { getItem: () => null, setItem: () => {}, removeItem: () => {} };
global.setTimeout = global.setTimeout;
global.TextEncoder = require('util').TextEncoder;
global.EventSource = function () { throw new Error('EventSource should not be used in file mode'); };

/* ---- run --------------------------------------------------------------- */
let fails = [];
function check(cond, msg) { if (!cond) { fails.push(msg); console.log('  FAIL ' + msg); } }

const sandbox = { module: { exports: {} }, exports: {} };
let api;
try {
  // Expose the internals we want to assert on.
  api = new Function(src + `
    ;return {get tel(){return tel}, get traj(){return traj},
             handleLine, drawTrajectory, drawScene, uiExtra,
             setStage, get stageMode(){return stageMode},
             niceStep, get FIX_NAMES(){return FIX_NAMES}};`)();
} catch (e) {
  console.log('LOAD FAILED: ' + e.message);
  process.exit(1);
}
console.log('script loaded and initialised without throwing');

/* ---- feed real frames from the ground station producer ------------------ */
let parsed = 0;
for (const line of frames) {
  api.handleLine(line);
  parsed++;
}
const tel = api.tel, traj = api.traj;
console.log(`fed ${parsed} frames`);
console.log(`  tel.frames=${tel.frames} bad=${tel.bad}`);
check(tel.bad === 0, `parser rejected ${tel.bad} well-formed frames`);
check(tel.frames === parsed, `ingested ${tel.frames} of ${parsed} frames`);

console.log(`  hg=${tel.hg} hgPeak=${tel.hgPeak.toFixed(2)} fix=${tel.fix} ` +
            `sats=${tel.sats} hacc=${tel.hacc}`);
check(tel.hg !== null, 'high-g field not parsed');
check(tel.hgPeak > 5, `high-g peak ${tel.hgPeak} should reach ~6 g`);
check(tel.fix !== null, 'GPS fix field not parsed');
check(tel.hacc !== null, 'GPS accuracy field not parsed');

console.log(`  traj points=${traj.pts.length} origin=${traj.origin ? 'set' : 'null'}`);
check(traj.pts.length > 2, 'trajectory recorded too few points');
check(traj.origin !== null, 'trajectory never established an origin');

const locked = traj.pts.filter(p => p.locked).length;
const unlocked = traj.pts.length - locked;
console.log(`  locked=${locked} unlocked=${unlocked}`);
check(unlocked > 0, 'no unlocked segment -- the gap path is untested');
check(locked > 0, 'no locked segment');

/* horizontal must be FROZEN across an unlocked run, never interpolated */
let frozenOK = true;
for (let i = 1; i < traj.pts.length; i++) {
  const a = traj.pts[i - 1], b = traj.pts[i];
  if (!b.locked && !a.locked && (b.x !== a.x || b.y !== a.y)) frozenOK = false;
}
check(frozenOK, 'horizontal moved during an unlocked run -- position was invented');
console.log(`  horizontal frozen across unlocked runs: ${frozenOK}`);

/* altitude must still advance while unlocked -- it is measured */
const unl = traj.pts.filter(p => !p.locked);
const altVaries = unl.length > 1 && new Set(unl.map(p => Math.round(p.z))).size > 1;
check(altVaries, 'altitude did not vary during the unlocked segment');
console.log(`  altitude still varies while unlocked: ${altVaries}`);

/* ---- render passes ----------------------------------------------------- */
const before = ctxCalls.stroke;
api.setStage('traj');
check(api.stageMode === 'traj', 'setStage did not switch to trajectory');
api.drawScene();
check(ctxCalls.stroke > before, 'trajectory render issued no strokes');
console.log(`  trajectory render: ${ctxCalls.stroke - before} strokes, ${ctxCalls.fillText} labels`);

api.setStage('attitude');
check(api.stageMode === 'attitude', 'setStage did not switch back');
api.drawScene();
console.log('  attitude render ran without throwing');

api.uiExtra();
console.log(`  readouts: hg="${getEl('oHg').textContent}" fix="${getEl('oFix').textContent}" ` +
            `pos="${getEl('oPos').textContent}" fromPad="${getEl('oTrack').textContent}"`);
check(getEl('oFix').textContent !== '', 'GPS fix readout empty');
check(getEl('oHg').textContent.indexOf('g') >= 0, 'high-g readout not formatted');

/* ---- the pad datum must not be taken from a bad first fix --------------
   This is the defect the bench found: a 3D fix on five satellites sat 238 m
   from where it first locked, and every distance was measured from that. */
traj.reset();
const marginal = 'V,0,0,1,0,0,0,1.0,0,%T%,1.0,3,4,14.5906000,120.9878000,42.5';
for (let i = 0; i < 40; i++) api.handleLine(marginal.replace('%T%', 200000 + i * 200));
check(traj.origin === null,
      'pad datum was taken from a 4-satellite ±42.5 m fix -- the gate did not hold');
check(traj.ground() === null, 'reported a distance from a datum it should not have');
console.log(`  marginal fix (4 sats, ±42.5 m): origin=${traj.origin} — correctly refused`);

const goodFix = 'V,0,0,1,0,0,0,1.0,0,%T%,1.0,3,9,14.5906000,120.9878000,2.5';
for (let i = 0; i < 10; i++) api.handleLine(goodFix.replace('%T%', 300000 + i * 200));
check(traj.origin !== null, 'pad datum never accepted from a 9-satellite ±2.5 m fix');
console.log(`  good fix (9 sats, ±2.5 m): datum accepted`);

/* a later poor fix is still plotted, but flagged rather than trusted */
const poorAfter = 'V,0,0,1,0,0,0,1.0,0,%T%,1.0,3,4,14.5910000,120.9878000,38.0';
for (let i = 0; i < 10; i++) api.handleLine(poorAfter.replace('%T%', 400000 + i * 200));
const poorPts = traj.pts.filter(p => p.poor).length;
check(poorPts > 0, 'a ±38 m fix after the datum was not flagged as poor');
console.log(`  low-confidence points flagged: ${poorPts}`);

/* Set pad re-datums on the next trustworthy fix, not immediately */
const padBefore = traj.origin;
traj.setPad();
check(traj.origin === padBefore, 'setPad re-datumed before a good fix arrived');
for (let i = 0; i < 10; i++) api.handleLine(poorAfter.replace('%T%', 500000 + i * 200));
check(traj.origin === padBefore, 'setPad re-datumed on a poor fix');
const moved = 'V,0,0,1,0,0,0,1.0,0,%T%,1.0,3,9,14.5920000,120.9890000,3.0';
for (let i = 0; i < 10; i++) api.handleLine(moved.replace('%T%', 600000 + i * 200));
check(traj.origin !== padBefore, 'setPad never took the good fix that followed');
check(traj.pts.length < 20, 'old-frame points survived a re-datum');
console.log(`  set pad: waited for a good fix, then re-datumed and cleared`);

traj.reset();
for (const line of frames) api.handleLine(line);

/* ---- backward compatibility: an old 9-field frame ---------------------- */
const tel0 = tel.frames;
api.handleLine('V,0.0,0.0,1.0,0,0,0,1.5,0.2,12345');
check(tel.frames === tel0 + 1, 'old 9-field frame was rejected');
check(tel.hg === null, 'old frame should leave high-g as null, got ' + tel.hg);
check(tel.fix === null, 'old frame should leave fix as null, got ' + tel.fix);
console.log(`  legacy 9-field frame accepted, hg=${tel.hg} fix=${tel.fix}`);

/* ---- malformed frame must be rejected ---------------------------------- */
const bad0 = tel.bad;
api.handleLine('V,0.0,junk,1.0,0,0,0,1.5,0.2,1,1,3,9,14.5,120.9');
check(tel.bad === bad0 + 1, 'malformed frame was not counted as bad');
console.log(`  malformed frame rejected (bad=${tel.bad})`);

/* ---- clear ------------------------------------------------------------- */
traj.reset();
check(traj.pts.length === 0 && traj.origin === null, 'reset did not clear the path');
console.log('  path reset works');

console.log();
if (fails.length) { console.log(`${fails.length} CHECK(S) FAILED`); process.exit(1); }
console.log('all viewer checks passed');
