#!/usr/bin/env python3
"""Headless checks for the ground station model and the viewer builds.

Neither half of the ground segment needs hardware to be wrong, so neither
needs hardware to be checked. This runs both:

  1. The Pico W's `Telemetry` model under CPython with a MicroPython shim,
     over a full 40 s synthetic flight loop.
  2. Each viewer build under Node with a stub DOM, fed the frames that model
     just produced.

    python tools/checks/run_checks.py

Requires Node on PATH for step 2; step 1 runs without it.

These exist because they have already earned their keep. Between them they
caught the trajectory recorder throttling on arrival time instead of the board
clock, a double-count in the bad-line counter, and a function that exists in
one viewer build but not the other.
"""
import io
import json
import math
import os
import re
import subprocess
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
GS = os.path.join(ROOT, "pico_w_ground_station", "main.py")
VIEWERS = [
    os.path.join(ROOT, "rocket_attitude_viewer.html"),
    os.path.join(ROOT, "rocket_attitude_viewer_serial.html"),
    os.path.join(ROOT, "pico_w_ground_station", "index.html"),
]

fails = []


def check(cond, msg):
    if not cond:
        fails.append(msg)
        print("  FAIL " + msg)


# ---------------------------------------------------------------------------
#  1. the ground station telemetry model
# ---------------------------------------------------------------------------
def load_model():
    """Pull the constants and the Telemetry class out of main.py.

    Importing the module outright would try to bring up a Wi-Fi AP, which
    CPython has no business doing.
    """
    src = io.open(GS, encoding="utf-8").read()
    # Constants from the shared block, then the class. Anchored on the class
    # rather than on any single constant name, and filtered to CAPS-style
    # assignments -- so adding a constant to that block cannot silently drop it
    # from the extract, and the hardware setup interleaved with it (a
    # machine.Pin) cannot get dragged in. That drop is exactly what happened
    # when the GNSS acquisition constants were added.
    c0 = src.index("SOURCE_STALE_MS = 1000")
    k0 = src.index("class Telemetry")
    k1 = src.index("tel = Telemetry()")
    keep = re.compile(r"^([A-Z_][A-Z0-9_]*\s*=|#|\s*$)")
    consts = chr(10).join(ln for ln in src[c0:k0].splitlines() if keep.match(ln))
    body = consts + chr(10) * 2 + src[k0:k1]

    clock = {"t": 0}
    fake_time = types.ModuleType("time")
    fake_time.ticks_ms = lambda: clock["t"]
    fake_time.ticks_diff = lambda a, b: a - b

    ns = {"time": fake_time, "math": math}
    exec(compile(body, "main.py<extract>", "exec"), ns)
    return ns, clock


def check_producer(ns, clock):
    print("== ground station telemetry model ==")
    Telemetry = ns["Telemetry"]
    LOOP, PAD = ns["FL_LOOP"], ns["FL_T_PAD"]
    APO, REACQ = ns["FL_T_APOGEE"], ns["FL_T_REACQ"]
    print("  profile: pad %.1fs  apogee %.1fs  reacquire %.1fs  loop %.1fs"
          % (PAD, APO, REACQ, LOOP))

    n0 = len(Telemetry().latest.split(","))
    check(n0 == 18, "initial frame has %d fields, expected 18 (V + 17)" % n0)

    tel = Telemetry()
    tel.set_mode("flight")
    dt = 1.0 / 25.0
    frames, phases = [], {}
    peak_hg = peak_lsm6 = 0.0
    locked = unlocked = 0

    for _ in range(int(LOOP / dt)):
        clock["t"] += 40
        line = tel.step(dt)
        frames.append(line)
        parts = line.split(",")
        check(len(parts) == 18, "frame at t=%.2f has %d fields" % (tel.t, len(parts)))
        v = [float(x) for x in parts[1:]]
        peak_hg = max(peak_hg, v[9])
        peak_lsm6 = max(peak_lsm6, math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2))
        if v[10] >= 2:
            locked += 1
            check(v[14] >= 0, "fixed frame at t=%.2f has no accuracy" % tel.t)
        else:
            unlocked += 1
            check(v[12] == 0.0 and v[13] == 0.0,
                  "no-fix frame at t=%.2f still carries a position" % tel.t)
            check(v[14] < 0, "no-fix frame at t=%.2f claims an accuracy" % tel.t)
        phases.setdefault(round(tel.t % LOOP), v[10])

    # This station has no latch of its own, and says so rather than implying
    # a safe one. -1 is "not on this source"; 0 would mean "safe", which is a
    # claim about hardware that is not here.
    v = [float(x) for x in frames[-1].split(",")[1:]]
    check(v[15] == -1, "synthetic source should report srv = -1, got %s" % v[15])
    check(v[16] == 0, "synthetic source should report srvus = 0, got %s" % v[16])

    print("  peak |a| on LSM6 fields %.2f g   peak high-g %.2f g" % (peak_lsm6, peak_hg))
    check(peak_hg > 5.0, "high-g never reached ~6 g (got %.2f)" % peak_hg)
    check(peak_lsm6 < 3.5, "LSM6 fields not clipped (peak %.2f g)" % peak_lsm6)

    print("  frames with lock %d, without %d" % (locked, unlocked))
    check(unlocked > 0, "lock never lost -- the trajectory gap goes untested")
    check(locked > 0, "lock never acquired")

    for t, want, label in [(2, True, "on the pad"), (8, False, "boost/coast"),
                           (14, False, "near apogee"), (30, True, "under chute")]:
        fix = phases.get(t)
        if fix is not None:
            check((fix >= 2) == want,
                  "t=%ds (%s): expected lock=%s, got fix=%s" % (t, label, want, fix))

    # Long enough to clear the acquisition dead time -- a receiver that has
    # been powered for two seconds is SUPPOSED to have no fix.
    b = Telemetry()
    b.set_mode("bench")
    for _ in range(int(20 / dt)):
        clock["t"] += 40
        line = b.step(dt)
    v = [float(x) for x in line.split(",")[1:]]
    check(v[10] >= 2, "bench mode has no GPS fix after 20 s")

    # Cold-start convergence: the early fix must be BAD, or the pad-datum gate
    # in the viewer is never exercised by the synthetic source.
    c = Telemetry()
    c.set_mode("bench")
    early = late = None
    for i in range(int(40 / dt)):
        clock["t"] += 40
        line = c.step(dt)
        v = [float(x) for x in line.split(",")[1:]]
        if early is None and v[10] >= 2:
            early = (v[11], v[14])
        late = (v[11], v[14])
    print("  cold start: first fix %d sats +/-%.1f m -> settled %d sats +/-%.1f m"
          % (early[0], early[1], late[0], late[1]))
    check(early[1] > 20, "first fix accuracy %.1f m is too good to test the gate"
          % early[1])
    check(late[1] < 5, "accuracy never converged (%.1f m)" % late[1])
    check(late[0] >= 6, "satellite count never reached 6 (%d)" % late[0])

    st = Telemetry()
    clock["t"] += 40
    st.step(dt)
    check(not st.stale(), "fresh telemetry reported stale")
    clock["t"] += 2000
    check(st.stale(), "telemetry silent for 2 s not reported stale")
    print("  staleness timer OK")
    return frames


# ---------------------------------------------------------------------------
#  2. the viewer builds
# ---------------------------------------------------------------------------
def extract_script(path, out):
    import re
    html = io.open(path, encoding="utf-8").read()
    blocks = re.findall(r"<script[^>]*>(.*?)</script>", html, re.S | re.I)
    if not blocks:
        raise SystemExit("no <script> block in " + path)
    io.open(out, "w", encoding="utf-8", newline="\n").write("\n;\n".join(blocks))


def check_viewers(frames, tmp):
    print()
    print("== viewer builds ==")
    if not any(os.access(os.path.join(p, "node" + e), os.X_OK)
               for p in os.environ.get("PATH", "").split(os.pathsep)
               for e in ("", ".exe", ".cmd")):
        print("  SKIPPED - node not found on PATH")
        return

    frames_path = os.path.join(tmp, "frames.txt")
    io.open(frames_path, "w", encoding="utf-8", newline="\n").write("\n".join(frames) + "\n")
    runner = os.path.join(HERE, "viewer_checks.js")

    for v in VIEWERS:
        js = os.path.join(tmp, "extract.js")
        extract_script(v, js)
        r = subprocess.run(["node", runner, js, frames_path],
                           capture_output=True, text=True)
        name = os.path.relpath(v, ROOT).replace("\\", "/")
        if r.returncode == 0:
            print("  PASS  " + name)
        else:
            print("  FAIL  " + name)
            for ln in (r.stdout + r.stderr).strip().splitlines():
                print("        " + ln)
            fails.append("viewer checks failed for " + name)


def main():
    import tempfile
    ns, clock = load_model()
    frames = check_producer(ns, clock)
    tmp = tempfile.mkdtemp(prefix="maare-checks-")
    try:
        check_viewers(frames, tmp)
    finally:
        import shutil
        shutil.rmtree(tmp, ignore_errors=True)

    print()
    if fails:
        print("%d CHECK(S) FAILED" % len(fails))
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
