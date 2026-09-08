# Headless checks

```
python tools/checks/run_checks.py
```

No hardware. Node on `PATH` for the viewer half; the ground station half runs
without it.

## What it checks

**Ground station telemetry model** — `Telemetry` is lifted out of
`pico_w_ground_station/main.py` and run under CPython with a MicroPython shim
(`time.ticks_ms` / `ticks_diff`), over a full 40 s synthetic flight loop:

- every frame carries the 14 fields of §6.6
- the LSM6 fields clip at ±2 g while the high-g field reaches ~6 g
- GNSS lock is held on the pad, **lost from launch to a few seconds past
  apogee**, and reacquired under the chute
- a no-fix frame carries no position at all
- the staleness timer trips after a second of source silence

**Viewer builds** — each of the three is loaded under Node with a stub DOM and
fed the 1000 frames the model just produced:

- the script initialises without throwing
- high-g and GPS fields are extracted; the high-g peak reaches ~6 g
- the trajectory records points, establishes an origin, and contains both
  locked and unlocked segments
- **horizontal position is frozen across every unlocked run** — the check that
  matters most, because a path that interpolates through a GPS gap is
  inventing where the vehicle was
- altitude still varies while unlocked, since altitude is measured and
  horizontal is not
- a marginal fix (4 satellites, ±42.5 m) is **refused as a pad datum**, and a
  good one (9 satellites, ±2.5 m) is accepted — the defect the bench found
- `Set pad` waits for a fix that clears the bar instead of re-datuming
  immediately, and clears the old frame's points when it does
- fixes worse than the threshold are still plotted but flagged low-confidence
- a render pass completes in both stage modes
- a legacy 9-field frame is still accepted, and a malformed frame is rejected
  exactly once

## Why these exist

They have already earned their keep. Between them they caught:

- the trajectory recorder throttling on **arrival time** instead of the board
  clock, which collapsed burst-delivered frames into a single path point
- `tel.bad` counting every malformed line **twice**
- `invalidateSizes()` existing in the dual-transport viewer but not the
  serial-only one, so the trajectory tab threw on the build that is the bench
  path today
- the extraction in `run_checks.py` silently dropping newly added constants,
  because it was anchored on one constant name instead of the block

None of those would have shown up in a compile, and two of them would have
looked like "the display is a bit odd" rather than a defect.

## Limits

The DOM stub is not a browser. It is enough for the script to initialise so the
logic underneath can be exercised; it does not check layout, styling, or that
anything is legible. A render pass is counted, not inspected. Nothing here
replaces opening the page.
