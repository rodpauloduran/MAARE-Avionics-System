import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';

// ---------------------------------------------------------------------------
// Helix curve for the separation spring, built by hand since Three r128 has
// no built-in spring/coil primitive.
// ---------------------------------------------------------------------------
class HelixCurve extends THREE.Curve {
  constructor(radius, height, turns) {
    super();
    this.radius = radius;
    this.height = height;
    this.turns = turns;
  }
  getPoint(t, target = new THREE.Vector3()) {
    const angle = t * this.turns * Math.PI * 2;
    const x = Math.cos(angle) * this.radius;
    const z = Math.sin(angle) * this.radius;
    const y = t * this.height - this.height / 2;
    return target.set(x, y, z);
  }
}

function makeFinGeometry() {
  // trapezoidal, swept fin in a local XY plane (X = radially outward, Y = up
  // the bottle's long axis), two triangles.
  const pts = new Float32Array([
    0, 8, 0,   0, 0, 0,   5, 1, 0,
    0, 8, 0,   5, 1, 0,   5, 4, 0,
  ]);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pts, 3));
  geo.computeVertexNormals();
  return geo;
}

function hemisphere(radius, flip) {
  const geo = new THREE.SphereGeometry(radius, 24, 12, 0, Math.PI * 2, 0, Math.PI / 2);
  if (flip) geo.scale(1, -1, 1);
  return geo;
}

export default function RocketAssemblyViewer() {
  const mountRef = useRef(null);
  const stateRef = useRef({});
  const [status, setStatus] = useState('LAUNCH CONFIG — LATCHED');
  const [busy, setBusy] = useState(false);
  const [deployed, setDeployed] = useState(false);
  const [autoRotate, setAutoRotate] = useState(true);

  useEffect(() => {
    const mount = mountRef.current;
    let width = mount.clientWidth;
    let height = mount.clientHeight;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x08121c);
    scene.fog = new THREE.Fog(0x08121c, 90, 220);

    const grid = new THREE.GridHelper(140, 28, 0x1f4a68, 0x122436);
    grid.position.y = -2;
    scene.add(grid);

    const camera = new THREE.PerspectiveCamera(42, width / height, 0.1, 1000);
    const target = new THREE.Vector3(0, 30, 0);
    let radius = 100, theta = 0.55, phi = 1.2;
    function updateCamera() {
      camera.position.set(
        target.x + radius * Math.sin(phi) * Math.sin(theta),
        target.y + radius * Math.cos(phi),
        target.z + radius * Math.sin(phi) * Math.cos(theta)
      );
      camera.lookAt(target);
    }
    updateCamera();

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    mount.appendChild(renderer.domElement);

    scene.add(new THREE.AmbientLight(0x9fb4c8, 0.65));
    const key = new THREE.DirectionalLight(0xffffff, 0.95);
    key.position.set(50, 90, 40);
    scene.add(key);
    const rim = new THREE.DirectionalLight(0x2ec4e6, 0.5);
    rim.position.set(-50, 20, -50);
    scene.add(rim);

    // ---- materials -----------------------------------------------------
    const matMotor    = new THREE.MeshStandardMaterial({ color: 0x1c7a99, transparent: true, opacity: 0.55, roughness: 0.35, metalness: 0.1 });
    const matAvionics = new THREE.MeshStandardMaterial({ color: 0x3fae6a, transparent: true, opacity: 0.55, roughness: 0.35, metalness: 0.1 });
    const matNose     = new THREE.MeshStandardMaterial({ color: 0xe8edf2, roughness: 0.45 });
    const matCollar   = new THREE.MeshStandardMaterial({ color: 0x8a97a5, roughness: 0.5, metalness: 0.6 });
    const matPin      = new THREE.MeshStandardMaterial({ color: 0xff5a3c, roughness: 0.3, metalness: 0.7 });
    const matSpring   = new THREE.MeshStandardMaterial({ color: 0xffcc33, roughness: 0.3, metalness: 0.6 });
    const matFin      = new THREE.MeshStandardMaterial({ color: 0x1a2632, roughness: 0.6, side: THREE.DoubleSide });
    const matPort     = new THREE.MeshStandardMaterial({ color: 0x08121c });
    const matChute    = new THREE.MeshStandardMaterial({ color: 0xff6a3c, side: THREE.DoubleSide, roughness: 0.6, transparent: true, opacity: 0.92 });
    const matAntenna  = new THREE.MeshStandardMaterial({ color: 0xdcdcdc, roughness: 0.4 });

    // ---- MOTOR GROUP (Bottle A) — local y=0 is the nozzle tip ----------
    const motorGroup = new THREE.Group();

    const neckA = new THREE.Mesh(new THREE.CylinderGeometry(1.05, 1.05, 3, 20), matMotor);
    neckA.position.y = 1.5;
    motorGroup.add(neckA);

    const bodyA = new THREE.Mesh(new THREE.CylinderGeometry(4.5, 4.5, 15, 28), matMotor);
    bodyA.position.y = 3 + 7.5;
    motorGroup.add(bodyA);

    const domeA = new THREE.Mesh(hemisphere(4.5, false), matMotor);
    domeA.position.y = 18;
    motorGroup.add(domeA);

    const collarLower = new THREE.Mesh(new THREE.CylinderGeometry(4.65, 4.65, 2, 28), matCollar);
    collarLower.position.y = 22.5 + 1;
    motorGroup.add(collarLower);

    for (let i = 0; i < 4; i++) {
      const fin = new THREE.Mesh(makeFinGeometry(), matFin);
      fin.position.set(0, 2, 0);
      fin.rotation.y = (Math.PI / 2) * i;
      fin.translateX(4.5);
      fin.rotation.y += Math.PI / 2;
      motorGroup.add(fin);
    }
    scene.add(motorGroup);

    // ---- AVIONICS GROUP (Bottle B + nose) — local y=0 is the joint face
    const avioGroup = new THREE.Group();
    avioGroup.position.y = 26.5;

    const collarUpper = new THREE.Mesh(new THREE.CylinderGeometry(4.65, 4.65, 2, 28), matCollar);
    collarUpper.position.y = -1;
    avioGroup.add(collarUpper);

    const domeB = new THREE.Mesh(hemisphere(4.5, true), matAvionics);
    domeB.position.y = 4.5;
    avioGroup.add(domeB);

    const bodyB = new THREE.Mesh(new THREE.CylinderGeometry(4.5, 4.5, 12, 28), matAvionics);
    bodyB.position.y = 4.5 + 6;
    avioGroup.add(bodyB);

    const neckB = new THREE.Mesh(new THREE.CylinderGeometry(1.05, 1.05, 3, 20), matAvionics);
    neckB.position.y = 16.5 + 1.5;
    avioGroup.add(neckB);

    const nose = new THREE.Mesh(new THREE.ConeGeometry(1.05, 12, 24), matNose);
    nose.position.y = 19.5 + 6;
    avioGroup.add(nose);

    const antenna = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 8.2, 8), matAntenna);
    antenna.position.set(0.9, 27.5, 0);
    antenna.rotation.z = 0.35;
    avioGroup.add(antenna);

    for (let i = 0; i < 4; i++) {
      const port = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.18, 0.3, 10), matPort);
      const a = (Math.PI / 2) * i + 0.4;
      port.position.set(Math.cos(a) * 4.5, 11, Math.sin(a) * 4.5);
      port.rotation.z = Math.PI / 2;
      port.rotation.y = a;
      avioGroup.add(port);
    }
    scene.add(avioGroup);

    // ---- PIN (independent, retracts sideways in phase 1) ---------------
    const pin = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.15, 10, 12), matPin);
    pin.rotation.z = Math.PI / 2;
    pin.position.set(0, 24.5, 0);
    scene.add(pin);

    // ---- SPRING (independent, stretches with the gap) -------------------
    const springBaseHeight = 4;
    const springGeo = new THREE.TubeGeometry(new HelixCurve(1.3, springBaseHeight, 5), 100, 0.16, 8, false);
    const spring = new THREE.Mesh(springGeo, matSpring);
    spring.position.set(2.6, 24.5, 0);
    scene.add(spring);

    // ---- SHOCK CORD ------------------------------------------------------
    const cordGeo = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 20, 0), new THREE.Vector3(0, 24.5, 0),
    ]);
    const cord = new THREE.Line(cordGeo, new THREE.LineBasicMaterial({ color: 0xdcecf5 }));
    scene.add(cord);
    const anchorA = new THREE.Vector3(0, 20, 0);
    const anchorB = new THREE.Vector3(0, 2, 0);

    // ---- PARACHUTE (packed + deployed representations) ------------------
    const chuteGroup = new THREE.Group();
    const packed = new THREE.Mesh(new THREE.IcosahedronGeometry(1.6, 0), matChute);
    chuteGroup.add(packed);

    const canopy = new THREE.Mesh(hemisphere(6, false), matChute);
    canopy.position.y = 6;
    canopy.visible = false;
    chuteGroup.add(canopy);
    const lines = [];
    for (let i = 0; i < 6; i++) {
      const a = (Math.PI * 2 / 6) * i;
      const g = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(Math.cos(a) * 5.6, 6, Math.sin(a) * 5.6),
        new THREE.Vector3(0, -2, 0),
      ]);
      const line = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0xffd9c8 }));
      line.visible = false;
      chuteGroup.add(line);
      lines.push(line);
    }
    chuteGroup.position.set(0, 24.5, 0);
    scene.add(chuteGroup);

    // ---- interaction: manual drag-orbit + wheel zoom (no OrbitControls
    //      in three r128) -------------------------------------------------
    let dragging = false, lastX = 0, lastY = 0;
    const dom = renderer.domElement;
    function onDown(e) { dragging = true; lastX = e.clientX; lastY = e.clientY; }
    function onUp() { dragging = false; }
    function onMove(e) {
      if (!dragging) return;
      const dx = e.clientX - lastX, dy = e.clientY - lastY;
      lastX = e.clientX; lastY = e.clientY;
      theta -= dx * 0.006;
      phi = Math.min(1.5, Math.max(0.35, phi - dy * 0.006));
      updateCamera();
    }
    function onWheel(e) {
      e.preventDefault();
      radius = Math.min(180, Math.max(45, radius + e.deltaY * 0.08));
      updateCamera();
    }
    dom.addEventListener('pointerdown', onDown);
    window.addEventListener('pointerup', onUp);
    window.addEventListener('pointermove', onMove);
    dom.addEventListener('wheel', onWheel, { passive: false });

    function onResize() {
      width = mount.clientWidth; height = mount.clientHeight;
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      renderer.setSize(width, height);
    }
    window.addEventListener('resize', onResize);

    // ---- animation state, driven imperatively (avoid React re-renders
    //      on every frame) ------------------------------------------------
    const anim = { playing: false, start: 0, duration: 2600, progress: 0, reversing: false };
    stateRef.current.trigger = () => {
      if (anim.playing) return;
      anim.playing = true; anim.reversing = false; anim.start = performance.now();
    };
    stateRef.current.reset = () => {
      if (anim.playing) return;
      anim.playing = true; anim.reversing = true; anim.start = performance.now();
    };
    stateRef.current.setAutoRotate = (v) => { stateRef.current._auto = v; };
    stateRef.current._auto = true;

    let rafId;
    function tick(now) {
      rafId = requestAnimationFrame(tick);

      if (anim.playing) {
        const t = Math.min(1, (now - anim.start) / anim.duration);
        const eased = 1 - Math.pow(1 - t, 3);
        anim.progress = anim.reversing ? 1 - eased : eased;
        if (t >= 1) {
          anim.playing = false;
          setBusy(false);
          if (anim.reversing) {
            setDeployed(false);
            setStatus('LAUNCH CONFIG — LATCHED');
          } else {
            setDeployed(true);
            setStatus('DEPLOYED — RECOVERY CONFIG');
          }
        } else if (!anim.reversing && anim.progress > 0.02 && anim.progress < 0.16) {
          setStatus('PIN WITHDRAWING…');
        } else if (!anim.reversing && anim.progress >= 0.16) {
          setStatus('SEPARATING — CHUTE DEPLOYING…');
        }
      }

      const p = anim.progress;
      const sep = Math.max(0, (p - 0.15) / 0.85);

      motorGroup.position.y = -16 * sep;
      avioGroup.position.y = 26.5 + 16 * sep;

      const pinOut = Math.min(1, p / 0.15);
      pin.position.x = pinOut * 9;
      pin.visible = pinOut < 1;

      const lowerFace = motorGroup.position.y + 24.5;
      const upperFace = avioGroup.position.y - 2;
      const gapMid = (lowerFace + upperFace) / 2;
      const gapSize = Math.max(0.6, upperFace - lowerFace);
      spring.position.y = gapMid;
      spring.scale.y = gapSize / springBaseHeight;

      chuteGroup.position.y = gapMid;
      const deployT = Math.min(1, Math.max(0, (sep - 0.25) / 0.35));
      packed.visible = deployT < 0.05;
      packed.scale.setScalar(1 - deployT);
      canopy.visible = deployT > 0.05;
      canopy.scale.setScalar(0.15 + 0.85 * deployT);
      lines.forEach(l => { l.visible = deployT > 0.05; });

      motorGroup.localToWorld(anchorA.set(0, 20, 0));
      avioGroup.localToWorld(anchorB.set(0, 2, 0));
      const posAttr = cordGeo.attributes.position;
      posAttr.setXYZ(0, anchorA.x, anchorA.y, anchorA.z);
      posAttr.setXYZ(1, anchorB.x, anchorB.y, anchorB.z);
      posAttr.needsUpdate = true;

      if (stateRef.current._auto && !dragging) {
        theta += 0.0022;
        updateCamera();
      }

      renderer.render(scene, camera);
    }
    rafId = requestAnimationFrame(tick);

    return () => {
      cancelAnimationFrame(rafId);
      dom.removeEventListener('pointerdown', onDown);
      window.removeEventListener('pointerup', onUp);
      window.removeEventListener('pointermove', onMove);
      dom.removeEventListener('wheel', onWheel);
      window.removeEventListener('resize', onResize);
      renderer.dispose();
      mount.removeChild(dom);
    };
  }, []);

  const spec = [
    ['Bottle A — motor', '1 L PET · ⌀9.0 cm · ~22.5 cm'],
    ['Bottle B — avionics', '⌀9.0 cm · ~19.5 cm, dry'],
    ['Nose cone', 'PETG · 12 cm'],
    ['Antenna', '82 mm wire, 868 MHz'],
    ['Static ports', '×4, ⌀1 mm, Bottle B wall'],
    ['Latch pin', '⌀2 mm steel · SF 6.6× vs 478 N'],
    ['Separation spring', 'k ≈ 0.9–1.0 N/mm'],
    ['Fins', '×4 · root 8 / tip 5 / span 5 cm'],
    ['Peak thrust', '478 N @ 100 psi, 21 mm nozzle'],
    ['Overall height', '~66 cm, launch config'],
  ];

  return (
    <div style={{ fontFamily: "'Space Grotesk', sans-serif" }}
         className="w-full h-full min-h-screen bg-slate-950 text-slate-200 flex flex-col">
      <style>{`
        @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;700&family=Space+Grotesk:wght@500;600;700&display=swap');
        .mono { font-family: 'JetBrains Mono', ui-monospace, monospace; }
      `}</style>

      <div className="flex items-center justify-between px-5 py-3 border-b" style={{ borderColor: '#122436' }}>
        <div>
          <div className="text-xs mono tracking-widest" style={{ color: '#2ec4e6' }}>ASSEMBLY VIEWER — REV A</div>
          <div className="text-lg font-semibold" style={{ color: '#e8edf2' }}>Two-Bottle Water Rocket Airframe</div>
        </div>
        <div className="mono text-xs px-3 py-1.5 rounded" style={{
          background: deployed ? 'rgba(255,90,60,0.15)' : 'rgba(46,196,230,0.12)',
          color: deployed ? '#ff8a6a' : '#6fdcf5',
          border: `1px solid ${deployed ? '#7a2f22' : '#1f4a68'}`,
        }}>{status}</div>
      </div>

      <div className="flex-1 flex flex-col md:flex-row min-h-0">
        <div ref={mountRef} className="relative flex-1 min-h-[420px]" />

        <div className="w-full md:w-72 shrink-0 border-t md:border-t-0 md:border-l p-4 space-y-4 overflow-y-auto"
             style={{ borderColor: '#122436', background: '#0a1622' }}>

          <div className="flex flex-col gap-2">
            <button
              onClick={() => { if (!busy) { setBusy(true); stateRef.current.trigger(); } }}
              disabled={busy || deployed}
              className="w-full py-2.5 rounded text-sm font-semibold transition"
              style={{
                background: (busy || deployed) ? '#16232f' : '#ff5a3c',
                color: (busy || deployed) ? '#4a5a68' : '#0a1420',
                cursor: (busy || deployed) ? 'not-allowed' : 'pointer',
              }}
            >Trigger Separation</button>

            <button
              onClick={() => { if (!busy && deployed) { setBusy(true); stateRef.current.reset(); } }}
              disabled={busy || !deployed}
              className="w-full py-2.5 rounded text-sm font-semibold transition border"
              style={{
                borderColor: '#1f4a68',
                color: (busy || !deployed) ? '#3a4a58' : '#6fdcf5',
                cursor: (busy || !deployed) ? 'not-allowed' : 'pointer',
                background: 'transparent',
              }}
            >Reset to Launch Config</button>

            <label className="flex items-center gap-2 text-xs mono mt-1" style={{ color: '#8aa0b2' }}>
              <input type="checkbox" defaultChecked
                onChange={(e) => stateRef.current.setAutoRotate(e.target.checked)} />
              auto-rotate
            </label>
            <div className="text-[11px] mono" style={{ color: '#4a6070' }}>drag to rotate · scroll to zoom</div>
          </div>

          <div className="h-px" style={{ background: '#122436' }} />

          <div>
            <div className="text-xs mono tracking-widest mb-2" style={{ color: '#2ec4e6' }}>DIMENSIONS</div>
            <div className="space-y-1.5">
              {spec.map(([k, v]) => (
                <div key={k} className="flex justify-between gap-3 text-xs">
                  <span style={{ color: '#8aa0b2' }}>{k}</span>
                  <span className="mono text-right" style={{ color: '#dceaf2' }}>{v}</span>
                </div>
              ))}
            </div>
          </div>

          <div className="h-px" style={{ background: '#122436' }} />

          <div className="space-y-1.5 text-xs">
            <div className="flex items-center gap-2"><span className="w-3 h-3 rounded-sm inline-block" style={{ background: '#1c7a99' }} /><span style={{ color: '#8aa0b2' }}>motor (Bottle A)</span></div>
            <div className="flex items-center gap-2"><span className="w-3 h-3 rounded-sm inline-block" style={{ background: '#3fae6a' }} /><span style={{ color: '#8aa0b2' }}>avionics (Bottle B)</span></div>
            <div className="flex items-center gap-2"><span className="w-3 h-3 rounded-sm inline-block" style={{ background: '#8a97a5' }} /><span style={{ color: '#8aa0b2' }}>collar joint</span></div>
            <div className="flex items-center gap-2"><span className="w-3 h-3 rounded-sm inline-block" style={{ background: '#ff5a3c' }} /><span style={{ color: '#8aa0b2' }}>latch pin</span></div>
            <div className="flex items-center gap-2"><span className="w-3 h-3 rounded-sm inline-block" style={{ background: '#ffcc33' }} /><span style={{ color: '#8aa0b2' }}>separation spring</span></div>
            <div className="flex items-center gap-2"><span className="w-3 h-3 rounded-sm inline-block" style={{ background: '#ff6a3c' }} /><span style={{ color: '#8aa0b2' }}>parachute</span></div>
          </div>

          <div className="text-[11px] mono leading-relaxed pt-1" style={{ color: '#3a5060' }}>
            To-scale concept visualizer, not manufacturing CAD. For toleranced
            parts and drawings, model this in OnShape (free, browser-based,
            team-collaborative) or FreeCAD (free, open-source).
          </div>
        </div>
      </div>
    </div>
  );
}
