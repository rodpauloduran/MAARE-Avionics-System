# Changelog

Notable changes to the MAARE avionics system, newest first.

Format loosely follows [Keep a Changelog](https://keepachangelog.com/).

---

## [0.3.0] — 2026-09-11

### Added

**MG90D servo latch on GP6, with a configurable trigger harness in the viewer.**

- **Firmware.** The latch is driven through the **Servo** library. New `!`
  prefixed line commands carry the values a single character cannot:
  `!arm`, `!safe`, `!fire`, `!latch`, `!us <n>`, `!pos <a> <b>`, `!srv`, `!hb`.
  The existing single-character commands are untouched and still dispatch on
  arrival, so a truncated line can never be mistaken for one.
- **Wire format** gained `srv` and `srvus` — appended, so 8/9-field parsers are
  still unaffected. The viewer shows the **board's** view of the latch, not its
  own belief; if the two disagree, that is the thing worth seeing.
- **Viewer panel** (collapsed by default, because it is an actuator control):
  arm/safe, manual fire and re-latch, endpoint calibration in microseconds, and
  an editable condition list — altitude, drop below peak, vertical velocity,
  tilt, |a|, high-g, rate magnitude, satellites, time since arm — each with a
  comparator and threshold, combined with **all** or **any**, and a **sustain
  window** the conditions must hold before firing. Settings persist across
  reloads. **Export** emits the tuned values as C constants.

**Interlocks**, in the order they matter: the pin is **not driven at boot**
(period set, pulse width zero); nothing moves while disarmed, and arming does
not itself move anything; a **3 s deadman on the board** disarms if the host
goes quiet; firing latches and auto-safes, and the viewer also disarms on
telemetry staleness or a closed port.

> **This is a bench harness, not the flight deployment path.** Conditions are
> evaluated in the browser and a fire command is sent down the wire — the right
> shape for a bench rig, and exactly the wrong shape for flight, where
> deployment must live in the board's state machine and never depend on a link
> (§2.3, §5.4). The harness produces *numbers*; those are what get compiled
> into the flight firmware.

- **LS3040 buzzer on GP7**, with non-blocking pattern playback — chirp, double,
  locator, alarm — selectable from the viewer, plus `!beep <hz> <ms>` for a
  one-shot. Arming the latch chirps and firing it double-beeps, which is
  §8.5's arming confirmation made audible. The board owns the timing, so a
  backgrounded browser tab cannot silence the locator.
- **Pin map reconciled again:** §9.1 specified the buzzer on GP13; it is on
  **GP7**. GP13 returns to the free list.

### Fixed

- **`tone()` leaks on this core, so the buzzer does not use it.**
  `Tone::stop()` nulls the `DigitalOut` *pointer* instead of writing the pin
  low, so `delete pin` frees nothing and every call leaks one `DigitalOut` —
  and the pin can be left HIGH, putting DC across a piezo after the beep.
  The buzzer drives a `DigitalOut` from a `Ticker` directly, allocated once.
- **Fire, and every condition-triggered fire, did nothing** — while `Go
  latched`, `Go released` and `Re-latch` all worked. The viewer sends `!fire`
  and then `!safe` back to back, as an interlock against a test rig staying hot.
  But `!safe` detached the servo a millisecond after `!fire` wrote the released
  position, cutting the pulse train before the horn could travel. The commands
  that worked were exactly the ones that leave the servo attached. Fixed on the
  board: safe refuses new motion immediately, but lets an already-commanded
  move finish over an **800 ms settle window** before stopping the pulse train.
  The interlock had been defeating the action it guarded.
- **The viewer no longer fires into a latch the board reports as already
  fired.** The board keeps `fired` latched until an explicit re-latch, as with
  `deployFired` in §5.4, so a second fire only earned a silent refusal. The
  condition engine now holds off, and the verdict says to re-latch first.
- **The servo did not move — because the servo was faulty.** A replacement
  moved under both a raw bit-bang and the Servo library, on VBUS, on the first
  try. Along the way the latch moved from `mbed::PwmOut` to the **Servo**
  library, which is confirmed working and stays. Attach/detach now serve as the
  arm/safe states, a stronger guarantee than a zero pulse width: detached, no
  pulse train exists at all.

  **Withdrawn:** an earlier entry here claimed `PwmOut` "compiles on this core
  and does not drive the pin", and cited the Servo library and `tone()` both
  avoiding it as evidence. Neither holds. `PwmOut` was only ever tried against
  the faulty servo, so it is *untested*; and a library's choice of mechanism is
  not evidence about a peripheral — software timing works on any pin, which is
  reason enough. Kept on the record rather than deleted, because a dead
  actuator making working firmware look broken is exactly the kind of silent
  failure this project exists to catch, and it caught this one late.
- **Added `servo_smoke/`** — one pin, one servo, no sensors or logic — so a
  stationary latch can be separated into firmware versus wiring and power in a
  single flash.
- **The design document claimed the Mbed core bundles a Servo library. It does
  not.** Verified against core 4.6.0, whose entire bundled set is MRI, PDM, SPI,
  Scheduler, ThreadDebug, USBHID, USBMSD and Wire. Corrected in §9.2 and the
  hardware reference, with the Library Manager install documented in its place.
- **Pin map reconciled to as-built.** §9.1 specified the servo on GP15; it is on
  **GP6**. Both documents now say GP6, and GP15 returns to the free list.

### Known, unresolved

- **The Fire path has not been confirmed on hardware.** The early-detach bug is
  fixed and the firmware compiles; the viewer's firing logic passes the
  headless checks. But the servo has not yet been *seen* to swing on `Fire` or
  on a condition-triggered fire since the fix. `Go latched`, `Go released` and
  `Re-latch` are confirmed. Treat conditional firing as unverified until it has
  been watched.
- **`mbed::PwmOut` is untested on this core**, not known-broken — see above.
- **VBUS is marginal for this servo on paper**, though it works in practice:
  USB 5 V behind a Schottky is roughly 4.7 V, against an MG90D specified from
  4.8 V, and a USB port current-limits near the servo's ~700 mA stall. Fine for
  an unloaded bench latch; re-check under a packed chute, and give it its own
  supply for flight.
- **A servo has no readback, and this project's standing rule cannot be applied
  to it.** `srvus` is what was *commanded*, never what the horn did — a stalled,
  stripped or unpowered servo reports exactly the same as a healthy one. The
  honest fix is mechanical: a limit or reed switch confirming the pin withdrew.
  Until then, "fired" means "commanded to fire".
- The latch has not been run on its own supply with the 220 µF fitted.

---

## [0.2.1] — 2026-09-09

### Added

- **`Fix gate` toggle** beside `Smoothing`. The pad datum is normally taken
  only from a fix that clears a quality bar (≥6 satellites, hAcc ≤10 m); this
  turns that off so any 2D fix will do — useful on a bench, where the bar is a
  launch-site bar the receiver rarely clears.
  Confidence and policy are kept apart: a weak fix accepted with the gate off
  is **still drawn faded and still counted low-confidence**. Turning the gate
  off permits a datum; it does not make the fix accurate. It also does not move
  an existing datum — that is what `Set pad` is for.
- **Serial monitor in the page.** The footer only ever showed the latest line,
  so boot output and `g` replies scrolled past at 25 Hz and the only way to
  read them was to disconnect and open the Arduino Serial Monitor — which
  means giving up the port, and therefore the live view. The console is
  collapsed by default, excludes telemetry frames unless asked, preserves the
  blank lines the boot output uses as structure, and sticks to the bottom only
  when the reader is already there. Read-only: `c` and `h` would corrupt the
  stream the page is parsing, so there is no free-text field to fire them from.

### Fixed

- **The trajectory legend overlapped the stage hint.** Both are anchored to the
  bottom edge of the stage — the hint is absolutely positioned across its full
  width — so the legend text and the "close the Arduino Serial Monitor first"
  overlay drew straight through each other, and the legend got worse as it grew
  lines. The hint is now hidden on the trajectory tab (it is about connecting
  and streaming, not the path) and restored on the attitude tab, and the legend
  draws its own backing panel so the ground grid cannot muddy it either. The
  drag-to-orbit affordance is dropped on a canvas too narrow to hold it clear
  of the legend block — on a small screen the numbers matter more than the hint.

- **A missing `hAcc` blocked the pad datum forever.** The gate hard-required
  the field, contradicting the rule that appended fields are optional (§6.6).
  Any board on firmware older than that field could never establish a datum,
  and the only symptom was a **purely vertical trajectory** with nothing on
  screen explaining why. Where `hAcc` is absent the gate now falls back to
  satellite count alone.

### Test

`tools/checks` gained cases for all three: the gate as a policy toggle that
does not launder a weak fix, a legacy 14-field frame still datuming on
satellite count, and the console's filtering, blank-line handling and cap.

### Docs

`docs/images/viewer-attitude.png` refreshed to show the release as built — the
`Fix gate` toggle, the `ACCURACY` readout and the collapsed serial monitor.
Captions in the README and §6.7 updated to match its figures, since a caption
that misdescribes its own image is worse than none.

---

## [0.2.0] — 2026-09-09

### Added

**ADXL375 high-g accelerometer and SAM-M8Q GPS are wired onto the I²C bus and
driven by `rocket_diagnostics/`.** All four sensors specified in §3.1 are now
physically present on the flight computer.

- **High-g column** in the diagnostics table, plus a peak tracker and a rest
  check. Absent hardware reads `---` rather than a fabricated `0`.
- **GPS status** — fix type, satellite count, coordinates and MSL altitude —
  printed in the 5-second summary and on demand with a new `g` command.
  Distinguishes *never acquired* from *acquired and lost*, because those point
  at different faults (sky view versus antenna or power).
- **Config readback extended to both new parts**, applying the rule §4.3
  already imposes on the IMU: the ADXL375 output data rate is decoded from
  `BW_RATE`, and the GPS dynamic model and navigation rate are read back from
  the module. Both are printed at boot, so a write that silently failed shows
  up as a wrong *printed* value.
- **Graceful degradation.** Neither new part is fatal if absent — only the
  barometer and IMU halt the sketch, because every other displayed number is
  derived from them.
- CSV mode gained a `highg_g` column.

**High-g and GPS reach the viewer, and there is a trajectory view.**

- **Wire format extended** (§6.6) by *appending* six fields, so any parser
  reading only the first eight or nine is unaffected:
  `V,ax,ay,az,gx,gy,gz,alt,vel,millis,hg,fix,sats,lat,lon,hacc`.
  `hg` and `fix` use **−1** for "part not fitted" — deliberately distinct from
  a zero reading, since 0.00 g is legal in freefall, fix type 0 is a legal
  "no fix", and 0,0 is a real position in the Gulf of Guinea.
- **High-g and GPS readouts** in the viewer: magnitude and peak, fix type,
  satellite count, position, and distance from the pad.
- **Trajectory tab** — the flown path in 3D over a metric ground grid, with a
  drag-to-orbit camera that auto-frames the data. **Only altitude is
  measured.** Horizontal comes from the GPS, which is blind through the whole
  flight, so unlocked segments are drawn dashed and red with the horizontal
  frozen at the last fix — never interpolated. A smooth arc through that gap
  would be an invention.
- **Path sampling throttles on the board's clock**, not on arrival time. Both
  transports deliver in bursts; throttling on arrival collapses a burst into a
  single point and samples the path unevenly. This is the same argument that
  put `millis()` in the format in the first place.
- **Board command buttons** — `Zero baro`, `Gyro bias`, `Reset peaks`,
  `GPS status`. Serial-only, and hidden in network mode: SSE is one-way, so a
  page served by the ground station has no back-channel to the flight computer.
  `c` and `h` are deliberately not exposed — both corrupt the stream the page
  is reading. Zeroing and bias blocking the sketch for a second or more
  suppresses the staleness flag for the duration, so it does not cry outage
  about a pause the operator asked for.
- **Ground station producer** emits the same fourteen fields, including a
  synthetic GNSS that **drops lock from launch until a few seconds past
  apogee** and reacquires drifted downwind. Without that, the trajectory gap
  path would go untested until a real flight.

**GPS position is now qualified, not just reported.** Found on the bench: a
stationary board holding a **3D fix on 5 satellites** sat **238 m** from where
it first locked, with the fix type reading `3D` the whole time. Two separate
problems, one of them ours.

- **`hacc` carried in telemetry** — the receiver's own horizontal accuracy
  estimate, already present in the NAV-PVT message it sends anyway
  (`getHorizontalAccEst()`), so it costs nothing. Shown in the viewer and in
  the sketch's `g` output, which now also warns when satellites drop below 6.
  A position with no accuracy beside it cannot be argued with.
- **The pad datum is gated.** It was taken from the *first* locked fix — the
  worst one of any session, straight out of a cold start with the fewest
  satellites and the worst geometry. Every distance was then measured from it,
  and every later and better fix appeared to *move*. The datum is now accepted
  only from a fix the receiver vouches for (**≥ 6 satellites, `hAcc` ≤ 10 m**);
  until one arrives the display reads `no pad datum` instead of a fabricated
  distance from a bad origin.
- **`Set pad` button** re-datums on the next fix that clears the same bar, and
  discards points recorded against the old datum — they live in the old frame
  and would draw a step the vehicle never took.
- **Low-confidence fixes are drawn faded** rather than dropped or drawn at full
  strength. Dropping them would hide that the vehicle was somewhere; drawing
  them normally would overstate how well we know where.

Note the drift being *along one axis* was not a bug: with five satellites in
one patch of sky the error ellipsoid is long and thin, so a poor fix wanders in
a line rather than a blob.

**Telemetry staleness is now visible, end to end.** Previously "radio silent"
and "vehicle sitting perfectly still" produced identical displays — recorded in
the design document as a safety-relevant display defect rather than a nicety.

- **Viewer** (all three builds): a gap of more than 1.5 s — about 37 missed
  frames at 25 Hz — turns the status dot amber, counts the age of the last
  frame, and dims the numeric readouts. It runs independently of each
  transport's own error handling, because neither catches the case that
  matters: an SSE connection stays open while the ground station has nothing to
  forward, and a quiet serial port raises nothing.
- **Ground station**: `/stream` now stops emitting data frames when its source
  has been quiet for more than 1 s, sending SSE comment lines instead so the
  connection and its retry timer stay alive. This half is not optional. The
  stream previously re-sent the last frame forever, which would have made the
  viewer's timeout useless once the radio is real — frames would keep arriving
  on time, all saying the same thing. `/health` also reports `stale` and
  `source_age_ms`.
- New producers must call `tel.set_line()` rather than assigning `tel.latest`,
  so the freshness timestamp cannot be forgotten. The radio-hook example in the
  ground station README is updated accordingly.

**Photographs and screenshots added** under `docs/images/`, referenced from the
README, design document, hardware reference and status report: the viewer
running live on hardware, the diagnostics boot output, and two angles of the
breadboard bench stack. `.gitattributes` gained binary rules for image formats
— `* text=auto` would otherwise apply line-ending heuristics to them.

**All four sensors are now confirmed on hardware.** The boot output shows every
device answering on the bus and every configurable one reporting its settings
back: `CTRL1_XL = 0x50`, `CTRL2_G = 0x58`, `BW_RATE = 0xD` → 800 Hz, and the
GPS `airborne <1g  OK` at 5 Hz. §13.1 steps 5 and 6 are closed, with two
qualifications recorded below.

### Fixed

- **The high-g rest check called a healthy sensor faulty.** Its ±0.15 g
  tolerance was sized against quantisation (~20 counts per g) and ignored the
  much larger effect: the ADXL375's **zero-g offset is specified in whole g,
  not milli-g**, because it is a ±200 g part and the offset scales with the
  range. On this bench it reads **0.73 g at rest against the LSM6's 1.01 g** —
  to specification, not broken. The check now flags only readings outside
  ≈0.35–2.0 g, where the faults that matter live (a dead axis reads ≈0; an
  ADXL345 in the footprint reads ≈12× low), and reports anything inside that
  band as offset with the delta against the LSM6. Trimming the offset properly
  is an open item — it needs a per-axis calibration, since a magnitude check
  cannot separate offset from scale error.
- **Malformed telemetry lines counted twice.** `ingest()` incremented
  `tel.bad` and then returned false, and `handleLine()` incremented it again on
  the false — so the "Bad lines" readout showed double. Pre-existing; found by
  the headless viewer test.
- **High-g saturation would never have been detected.** The ADXL375 is 13-bit
  sign-extended into `int16`, so it saturates near ±4095 counts, not ±32767.
  Reusing the LSM6's `32000` threshold means the flag never fires and a clipped
  boost trace is reported as a valid measurement. The sketch now keeps a
  separate `HG_SAT_COUNT`, and the design document records the trap.
- **`applyImuConfig()` only warned on `CTRL2_G`.** A failed accelerometer
  register write passed silently — the exact class of fault the readback exists
  to catch. Both registers are now checked.
- **Column alignment walked on rounding.** `pad()` computed its width from the
  un-rounded value, so 9.996 at two decimals printed `10.00` where the digit
  count predicted four characters, shifting the row left by one. Width is now
  computed from the rounded value.
- **Gyro bias averaged partly duplicated samples.** The calibration loop
  delayed 4 ms against a 208 Hz (4.8 ms) gyro, so it re-read the same sample
  often enough to weight the average toward whichever readings happened to
  repeat. Now 6 ms, which keeps every sample independent — the point of
  averaging.

### Changed

- **The synthetic flight profile now clips its LSM6 fields to ±2 g** — the
  range that part is actually configured for — while the new `hg` field
  carries the true unclipped magnitude. The profile peaks at 6 g, so the two
  now disagree exactly as the real hardware would. The producer's own docstring
  already said the clipping "is real and the viewer should show it rather than
  hide it"; it was not previously implemented. This changes what the synthetic
  source displays for `|a|` in flight mode.
- Arduino sketches now live in per-sketch folders
  (`rocket_diagnostics/rocket_diagnostics.ino`). The Arduino IDE requires a
  `.ino` to sit in a directory of the same name; a bare sketch at the
  repository root prompts the IDE to relocate it, and two of them at the root
  cannot both be opened cleanly. `rocket_flight/` follows the same shape.
- Documentation updated across `README.md`, `avionics_documentation.md`
  (§4.4, §4.5, §6.7, §13.1, §13.2, §14), `hardware_reference.md`,
  `avionics_status_report.md` and the ground station README. The design
  document's closing note now names **four** silent failure modes on this
  platform rather than two — the GPS dynamic model and the ADXL375 saturation
  ceiling join the MS5611 and the gyro scale factor.

### Build and test

`arduino:mbed_rp2040:pico`, verified with `arduino-cli compile`:

| | before | after |
|---|---|---|
| Program storage | 112,536 B (5%) | **148,883 B (7%)** |
| Global variables | 43,996 B (16%) | **44,608 B (16%)** |

Both are exercised headlessly by `tools/checks/run_checks.py` (new; no
hardware, Node needed only for the viewer half):

- The `Telemetry` producer runs under CPython with a MicroPython shim over a
  full 40 s flight loop — field count, LSM6 clipping, high-g peak, GPS phase
  transitions, and the staleness timer.
- Each viewer build loads under Node with a stub DOM, is fed 1000 real frames
  from that producer, and is checked for parse errors, high-g and GPS
  extraction, trajectory point count, **horizontal frozen across every
  unlocked run**, altitude still varying while unlocked, **a 4-satellite
  ±42.5 m fix being refused as a pad datum**, `Set pad` waiting for a good fix
  before re-datuming, low-confidence fixes being flagged, a render pass in both
  stage modes, backward compatibility with a 9-field frame, and rejection of a
  malformed one.
- The synthetic source cold-starts realistically in bench mode — no fix for
  3 s, then 4 satellites at ±42 m converging to 9 at ±2.5 m — so the datum gate
  is exercised without hardware.

Nearly all of the +35.7 KB is the u-blox library; the RAM cost is +612 B.
Neither figure threatens the 112 KB flight buffer of §10.3, but the flight
firmware should be re-measured against it rather than assumed.

### Known, unresolved

- **The ADXL375's zero-g offset is untrimmed.** 0.73 g at rest against the
  LSM6's 1.01 g. Within specification, but it leaves the channel unusable for
  absolute magnitude near 1 g. Peaks and events — launch detect and burnout,
  its actual job — are unaffected.
- **The GPS has a fix but not a usable one.** Coordinates confirmed correct
  against a map, but tested only indoors: on 5 satellites the position wandered
  238 m. Needs open sky, where 10–15 satellites and single-digit metres are
  expected.
- **A phantom `0x7E` appears in the I²C scan.** Nothing in this design lives
  there and 0x78–0x7F is the reserved range, so it points at a marginal bus
  rather than a device — four sets of pull-ups in parallel on breadboard jumper
  leads being the prime suspect.

---

## [0.1.0] — 2026-09-08

Initial version.

### What exists

**Flight computer** — MS5611 barometer and MinIMU-9 v6 wired to a Raspberry Pi
Pico on the official Arduino Mbed OS core. `rocket_diagnostics/` runs on
hardware, reporting interpreted sensor values, 1σ altitude noise with a
suggested `APOGEE_DROP_M`, sensor ranges read back from the config registers,
gyro bias calibration, peak trackers and saturation flags.

**Ground segment** — a Pico W ground station in MicroPython, bringing up its own
Wi-Fi access point and serving a 3D attitude viewer over Server-Sent Events at
25 Hz. The viewer also runs over Web Serial when opened as a local file. Both
run on hardware; telemetry is synthetic.

**Documentation** — design document, hardware bench reference, airframe build
spec, bill of materials, status report, and a 3D assembly viewer.

### What does not exist

No high-g accelerometer, GPS, radio link, deployment hardware, arming
interlock, buzzer or airframe. `rocket_flight.ino` is not written, so no
deployment code exists and none has flown. The attitude filter runs ground-side
only, and the tilt inhibit is specified but not implemented.

### Known unreconciled

- §8.3 of the design document sizes the separation spring against **0.18 kg
  forward mass**, while `airframe_build_spec.md` targets **150–200 g dry mass
  for the whole rocket**. A 2 L PET bottle is ~50 g before fins, so these do
  not both hold.

---

[Unreleased]: https://github.com/rodpauloduran/MAARE-Avionics-System/compare/v0.3.0...HEAD
[0.3.0]: https://github.com/rodpauloduran/MAARE-Avionics-System/compare/v0.2.1...v0.3.0
[0.2.1]: https://github.com/rodpauloduran/MAARE-Avionics-System/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/rodpauloduran/MAARE-Avionics-System/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/rodpauloduran/MAARE-Avionics-System/releases/tag/v0.1.0
