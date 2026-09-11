# Water Rocket Avionics System — Design Documentation

**v0.3.1 · September 2026**
Mapúa University (Intramuros)

> **What is built.** All four I²C sensors are now on the flight computer — the
> barometer, the IMU, the ADXL375 high-g accelerometer and the SAM-M8Q GPS —
> and the bench diagnostics sketch drives all of them (§13.2). **All four are
> now confirmed on hardware** — every device answers on the bus and every
> configurable one reports its settings back correctly at boot. Two
> qualifications: the ADXL375's zero-g offset is untrimmed (§4.4), and the GPS
> has been given only an indoor sky view, so it holds a fix but not an accurate
> one (§4.5). The ground segment — a 3D attitude viewer
> (§6.7) and a Pico W ground station serving it over its own Wi-Fi (§6.5) —
> works on synthetic telemetry. Not connected: radio link, deployment
> hardware, arming interlock, buzzer, airframe. This document specifies the
> whole system; treat anything outside §13.2 and §6.5 as design intent rather
> than description.

---

## Table of Contents

1. [Purpose and scope](#1-purpose-and-scope)
2. [Design philosophy](#2-design-philosophy)
3. [System architecture](#3-system-architecture)
4. [Sensor subsystem](#4-sensor-subsystem)
5. [Deployment logic — the core design argument](#5-deployment-logic--the-core-design-argument)
6. [Radio and telemetry](#6-radio-and-telemetry) — incl. ground station and viewer
7. [Power subsystem](#7-power-subsystem)
8. [Recovery and safety](#8-recovery-and-safety)
9. [Electrical reference](#9-electrical-reference)
10. [Firmware architecture](#10-firmware-architecture)
11. [Mechanical integration](#11-mechanical-integration)
12. [Regulatory compliance](#12-regulatory-compliance)
13. [Test and verification plan](#13-test-and-verification-plan)
14. [Known limitations and open items](#14-known-limitations-and-open-items)
15. [Related documents](#15-related-documents)

---

## 1. Purpose and scope

This document specifies a student-designed and student-researched (SRAD)
flight computer for a water pressure rocket. The vehicle is a testbed; the
avionics are the deliverable.

**Objectives:**

- Log full-rate flight data (acceleration, attitude, altitude, position)
  through a complete flight
- Autonomously detect apogee and actuate parachute deployment
- Transmit live telemetry to a ground station, and act as a recovery beacon
  after landing
- Build team competency in sensor fusion, deployment logic, and RF telemetry
  that transfers directly to high-power rocketry (HPR)

**Why a water rocket.** The flight envelope is hostile in the ways that
matter — 20–100 g boost, sub-second burn, ~10 second total flight, a wet
landing — while carrying no propellant hazard and no regulatory burden. Every
subsystem here (deployment state machine, static ports, telemetry link,
arming interlock) transfers unchanged to a motor-powered vehicle. The
consequence of a failure is a plastic bottle, not a motor CATO.

**Scope.** This document covers the avionics package: sensors, compute, radio,
power, firmware, and their mechanical integration. Airframe structure is
specified separately (see §15).

---

## 2. Design philosophy

Five principles drove essentially every decision in this document. They are
stated up front because the rest of the document is largely their consequences.

**2.1 — Sensors cover each other's blind spots.**
Accelerometers are good at fast and bad at slow (they drift). Barometers are
the reverse (noisy and laggy, but driftless). Gyros know rotation but nothing
about position. GPS knows position but is far too slow. No single sensor is
sufficient; fusion is the formal way of letting each cover the others'
weaknesses.

**2.2 — Differentiate, don't integrate, for anything safety-critical.**
Differentiation amplifies noise but *forgets* it. Integration smooths noise but
*remembers* it forever. For a 10-second flight beginning with a clipped 50 g
boost, forgetting is worth far more than smoothing.

**2.3 — Deployment must never depend on the radio.**
The vehicle flies itself. Telemetry is for the operators, not the rocket. A
stalled transmission, a lost link, or a crashed ground station must have zero
effect on whether the parachute comes out.

**2.4 — Bias every threshold toward late.**
Late deployment costs some data and some drift. Early deployment at 70 m/s
shreds the parachute and destroys the vehicle. Asymmetric consequences demand
asymmetric margins.

**2.5 — Log raw, filter offline.**
In flight, the computer records raw sensor counts and does the minimum
arithmetic needed for the state machine. All fusion, filtering, and analysis
happens afterwards in Python, where iteration takes seconds instead of a
reflash-and-relaunch cycle. A filter is only ported onboard once it has agreed
with raw data across many flights.

---

## 3. System architecture

```
                        ┌─────────────────────────┐
                        │   NOSE CONE (dry bay)   │
                        │                         │
   82 mm wire ──────────┤  ▸ LoRa RFM95W          │
   antenna              │  ▸ GPS SAM-M8Q ▲sky     │
                        │  ─────── upper deck ─── │
                        │  ▸ MS5611 barometer     │
   static ports ────────┤  ▸ MinIMU-9 v6          │
   (×4, ⌀1 mm)          │  ▸ ADXL375 high-g       │
                        │  ─────── lower deck ─── │
                        │  ▸ Raspberry Pi Pico    │
                        │  ▸ Power / switch / cap │
                        │  ▸ Servo, buzzer, reed  │
                        └───────────┬─────────────┘
                                    │  latch pin
                        ┌───────────┴─────────────┐
                        │  COLLAR JOINT           │
                        │  ▸ separation spring    │
                        │  ▸ parachute (packed)   │
                        │  ▸ shock cord anchor    │
                        └───────────┬─────────────┘
                                    │
                        ┌───────────┴─────────────┐
                        │  MOTOR (Bottle A)       │
                        │  pressurised, intact    │
                        └─────────────────────────┘
```

### 3.1 Subsystem summary

| Subsystem | Component | Role |
|---|---|---|
| Compute (flight) | Raspberry Pi Pico (RP2040) | Sensor loop, state machine, deployment |
| Compute (ground) | Raspberry Pi Pico W | Telemetry receiver; Wi-Fi AP serving the viewer to any phone (§6.5) |
| Altitude | MS5611 (GY-63) | Primary altimeter, apogee trigger |
| Attitude / coast accel | Pololu MinIMU-9 v6 (LSM6DSO + LIS3MDL) | Gyro attitude, fine accel, magnetometer |
| Boost accel | Adafruit ADXL375 (±200 g) | Launch detect, burnout, peak-g |
| Position | SparkFun SAM-M8Q (u-blox) | Landing coordinates, recovery |
| Radio | 2× Adafruit RFM95W (SX1276) | Telemetry downlink + landed beacon |
| Deployment | MG90D servo (180°, metal gear) | Latch pin withdrawal |
| Locator | LS3040 piezo buzzer (4 kHz) | Audible recovery, arming confirmation |
| Arming | Reed switch + N35 magnet | Physical deployment interlock |
| Power | 500 mAh 1S LiPo + MCP73831 charger | ~5 h endurance, VSYS direct |

### 3.2 Mass and power budget

| Item | Mass | Current (continuous) |
|---|---|---|
| Pico | 3 g | ~30 mA |
| SAM-M8Q GPS | 8.5 g | ~29 mA |
| RFM95W + antenna | 3.5 g | ~12 mA (RX idle) |
| MS5611 + MinIMU-9 + ADXL375 | ~4 g | ~2 mA |
| Servo (idle) | 13 g | ~5 mA |
| Buzzer | 2 g | ~2 mA |
| Battery | 10.5 g | — |
| Boards, wiring, standoffs | ~35 g | — |
| **Total** | **~80 g** | **~80–90 mA** |

Endurance ≈ 5 hours. Flight consumption is negligible (~0.3 mAh); battery life
is entirely a function of pad time. Transient loads (servo ~300 mA for 500 ms,
LoRa TX ~90 mA for 10 ms) do not meaningfully affect the budget but do drive
the decoupling requirements in §7.

---

## 4. Sensor subsystem

All four devices share a single I²C bus at 400 kHz.

### 4.1 Address map

| Device | Address | Notes |
|---|---|---|
| MS5611 | `0x77` | `0x76` if CSB high. PS pin **HIGH** selects I²C. |
| LSM6DSO | `0x6B` | Pololu pulls SA0 high (datasheet default is 0x6A) |
| LIS3MDL | `0x1E` | Pololu pulls SA1 high (datasheet default is 0x1C) |
| ADXL375 | `0x53` | `0x1D` if address jumper bridged |
| SAM-M8Q | `0x42` | u-blox DDC mode |

![The bench stack on breadboards: the Pico H and the four I²C sensor
breakouts wired with jumper leads, the SAM-M8Q's chip antenna facing up, and the
RFM95W, servo and buzzer alongside](docs/images/bench-stack-1.jpg)

*All four I²C devices on one bus, as bench-built. Note this is a breadboard with
jumper wires, not the two-deck sled of §11: long unshielded leads and four sets
of pull-ups in parallel are exactly the conditions the warning below is about.*

No conflicts. **Pull-up warning:** every breakout carries its own pull-ups.
Four in parallel can over-drive the bus at 400 kHz. If the bus misbehaves,
remove pull-ups from all but one board — start with the GY-63, whose clones
often use 2.2 kΩ.

### 4.2 MS5611 — barometer

Primary altimeter and apogee trigger. ~10 cm resolution, ~100 Hz achievable.

**Why this part.** Its two error sources both vanish at the moment of interest:

- **Dynamic pressure error** scales with v². At 70 m/s, q ≈ 3000 Pa ≈ 250 m of
  apparent altitude error — but at apogee, v ≈ 0, so q ≈ 0.
- **Inertial gradient** (air in the bay stacking up aft under acceleration,
  dP/dx = ρa) is ~60 Pa at 50 g over a 10 cm bay — but at apogee, a = −1 g,
  so the gradient is ~1 Pa.

The ascent altitude trace will be wrong. The apogee *time* will not be. This
is why barometric apogee detection is robust despite ugly ascent data.

**Alternatives considered.** BMP390 (~5 cm, 200 Hz) is marginally better.
BMP280 (~1 m) is flyable but pushes detection latency from ~0.25 s to ~0.7 s
past apogee. **MPL3115A2 was rejected** — its 1.5 Pa resolution requires
maximum oversampling at ~512 ms/conversion, giving roughly 6 samples on a
3-second ascent.

**Driver note 1 — `begin()` fails on this toolchain.** Confirmed on hardware:
`MS5611.begin()` returns false and reports the sensor absent, while a manual
I²C scan finds it at `0x77` without difficulty. The cause is not the sensor.
The library's `isConnected()` performs a zero-length `endTransmission()`, which
on Mbed-based Arduino cores issues a *read*-type transaction rather than an
address write; the MS5611 does not ACK that, so detection fails. The library
carries a workaround, but it is gated behind `#ifdef ARDUINO_ARCH_NRF52840`,
which is not defined on RP2040 — so it never compiles in.

**Use `reset()` instead of `begin()`.** `reset()` is public, skips the broken
detection step, and does the part that actually matters — resetting the chip
and loading its calibration PROM:

```cpp
if (baro.reset()) {                 // NOT baro.begin()
  Serial.println(F("MS5611 initialised"));
} else {
  // genuine wiring or power fault
}
```

The same defect affects any I²C scan written as a bare
`beginTransmission()` / `endTransmission()` pair. Every scan in this project
writes a dummy byte first — see §10.1. Symptom if this is missed: the sensor
appears absent, and because `begin()` never loaded the calibration constants,
subsequent reads return a fixed nonsense altitude (44307.70 m) rather than
failing loudly.

**Driver note 2 — this sensor cannot be read in one transaction.** Issue a
conversion command, wait ~9 ms at OSR 4096, then read the 24-bit result.
Blocking for 9 ms would destroy a 500 Hz loop, so the flight firmware runs the
barometer as its own state machine, alternating pressure and temperature
conversions (temperature every ~20th cycle; it changes slowly).

**Zeroing.** Average 100 samples on the pad at boot and store as
`groundPressure`. Altitude is *always* a delta:

```
alt_m = 44330 × (1 − (p / p_ground)^0.1902949)
```

Never trust absolute MSL altitude.

### 4.3 MinIMU-9 v6 — LSM6DSO + LIS3MDL

**Gyros are the only source of orientation.** No accelerometer can supply it.
This is the sole reason this sensor is in the stack; the fine-resolution accel
during coast is a secondary benefit.

Flight configuration requires overriding `enableDefault()` (which gives ±2 g
and ±245 dps — useless in flight):

- Accel: ±16 g, 833 Hz ODR (CTRL1_XL)
- Gyro: ±2000 dps, 833 Hz ODR (CTRL2_G)

Note the LSM6DSO's non-obvious full-scale encoding: **`01` = ±16 g**, not ±4 g.

**Never hard-code a scale factor. Read it back from the register.**

This is not a style preference. `enableDefault()` selects **±245 dps**, whose
sensitivity is `0.00875` — so pairing it with the `0.035` dps/LSB constant that
belongs to ±1000 dps reports every angular rate **4× too high** and silently
saturates anything above 245 dps.

The symptom of that mismatch is misleading, which is what makes it dangerous.
Slow motion looks fine, because gravity dominates the attitude estimate at low
rates and pulls it back to truth. Fast motion overshoots wildly and oscillates,
because it is gyro-dominated. A deliberate
180° flip could integrate toward 720° before clipping ate into it. Nothing in
the output said "scale factor wrong" — it said "the filter is unstable."

The fix is structural, not a corrected constant. The sketch now writes the
config registers explicitly, reads `CTRL1_XL` and `CTRL2_G` back, and *derives*
both scale factors from what the chip reports — then prints the decoded ranges
at boot. A hard-coded constant is a silent claim about a register you cannot
see; a readback makes a wrong range show up as a wrong printed range within
five seconds of power-on.

```cpp
uint8_t c2 = imu.readReg(LSM6::CTRL2_G);
if ((c2 >> 1) & 0x01) { GYR_RANGE_DPS = 125;  GYR_DPS_PER_LSB = 0.004375; }
else switch ((c2 >> 2) & 0x03) {
  case 0: GYR_RANGE_DPS =  245; GYR_DPS_PER_LSB = 0.00875; break;
  case 1: GYR_RANGE_DPS =  500; GYR_DPS_PER_LSB = 0.0175;  break;
  case 2: GYR_RANGE_DPS = 1000; GYR_DPS_PER_LSB = 0.035;   break;
  case 3: GYR_RANGE_DPS = 2000; GYR_DPS_PER_LSB = 0.070;   break;
}
```

**Saturation must be detected, not inferred.** A clipped reading is not a large
reading — it is an *unknown* one, and reporting the ceiling as though it were a
measurement is worse than reporting nothing. The sketch flags any axis within
~2% of int16 full scale and reports accelerometer and gyro clipping separately
from the "approaching range ceiling" warning.

**Gyro bias calibration is mandatory.** Average 400–500 samples with the
vehicle held still at boot and subtract the offset from every reading. Skipping
this produces visible attitude drift within seconds.

**Known ceiling.** ±2000 dps ≈ 333 RPM. A tumbling or spin-stabilised vehicle
can exceed this, and a saturated gyro produces attitude that is not merely
noisy but *wrong and unrecoverable*. An ICM-42688-P (±4000 dps) is the upgrade
path if flight data shows saturation.

**Magnetometer.** Logged but not flight-critical. Servo current and nichrome
switching swing the local field precisely during deployment; hard/soft-iron
calibration must be redone whenever anything in the bay moves.

### 4.4 ADXL375 — high-g accelerometer

±200 g, ~49 mg/LSB. The only sensor that does not saturate during the
0.2–0.4 s burn at 20–100 g. Used for launch detection (threshold 5 g sustained
50 ms) and burnout timing.

Set data rate to 800 Hz or higher; the default 100 Hz smears the boost profile.

**Note.** ADXL345 is a *different part* at ±16 g and cannot perform this role,
despite a nearly identical register map. The "10,000 g shock survival" figure
on ADXL345 listings is a *survival* rating, not a measurement range — the two
are routinely conflated.

**Driver note 1 — `begin()` is safe here, unlike the MS5611's.** This is worth
stating explicitly because §4.2 establishes the opposite habit. Adafruit_BusIO's
address detection carries an explicit `#ifdef ARDUINO_ARCH_MBED` that writes the
dummy byte before `endTransmission()`, so the Mbed zero-length-probe defect
(§10.1) does not bite this part. That is a property of *this library*, not of
the bus — check it per device rather than assuming either way.

**Driver note 2 — the saturation ceiling is 13-bit, not 16-bit.** `begin()` puts
the part in FULL_RES mode, so readings are 13-bit two's complement
sign-extended into `int16`. Full scale is about **±4095 counts**, not ±32767.
Saturation logic copied from the LSM6 — which genuinely does run to the integer
width — never fires at all on this part, and a clipped boost trace would be
reported as a valid measurement. The diagnostics sketch keeps a separate
`HG_SAT_COUNT` for exactly this reason.

**Driver note 3 — there is no range register to read back.** §4.3 makes reading
configuration back from the hardware the standing rule, and this part is the
exception that clarifies it: ±200 g is fixed in silicon and the library's
`setRange()`/`getRange()` are deliberate no-ops. The scale factor is a property
of the part number, so the readback rule applies instead to the thing that *is*
configurable — the output data rate, decoded from `BW_RATE` and printed at boot.

**Bench expectation — do not expect 1.00 g at rest.** Two effects, and the
second is much the larger:

- At rest the part sees 1 g across roughly **20 counts** (1 g ÷ 49 mg/LSB), so
  ±0.05 g of wobble is quantisation, not noise.
- **Its zero-g offset is specified in whole g, not milli-g.** This is a ±200 g
  part and the offset scales with the range. Measured on this bench: **0.73 g
  at rest while the LSM6 read 1.01 g.** That gap is the part behaving to
  specification, not a fault — and a ±0.15 g tolerance, sized against
  quantisation alone, flagged a healthy sensor as broken.

So the rest check is deliberately loose: it flags only readings outside roughly
**0.35–2.0 g**, which is where the faults that matter live — a dead axis reads
≈0, and an ADXL345 in this footprint reads about 12× low (≈0.08 g). Anything
inside that band is reported as offset, with the delta against the LSM6 shown.

**Trimming the offset out is an open item.** It needs a per-axis calibration
against a known orientation; a magnitude check cannot separate offset from
scale error, and the offset is per-axis while |a| is not. Until then, treat the
high-g channel as trustworthy for *peaks and events* — which is its actual job,
launch detect and burnout — and not for absolute magnitude near 1 g.

### 4.5 SAM-M8Q — GPS

Landing coordinates. Explicitly **not** a flight-phase sensor: it loses lock at
launch and takes 5–15 s to reacquire, by which time the flight is over.

**Critical configuration:**

```cpp
gnss.setI2COutput(COM_TYPE_UBX);
gnss.setNavigationFrequency(5);
gnss.setDynamicModel(DYN_MODEL_AIRBORNE1g);   // ← the line people miss
gnss.saveConfiguration();
```

The default dynamic model assumes a ground vehicle and will reject a rocket
trajectory as implausible, dropping lock exactly when needed.

**Read the dynamic model back.** A `setDynamicModel()` that silently failed
looks identical to one that worked — right up until the vehicle is airborne and
the receiver drops lock. `getDynamicModel()` returns the configured value (255
if the query itself failed), so the same readback discipline §4.3 applies to the
IMU applies here, and the decoded model is printed at boot.

**`setAutoPVT(true)` is not optional in a timed loop.** Without it `getPVT()`
polls the module and **blocks for up to 1100 ms** — 27 missed samples at the
diagnostics sketch's 25 Hz, and catastrophic in a 500 Hz flight loop. With it,
the module pushes solutions on its own schedule and `getPVT()` returns
immediately, so it is safe to call from a rate-gated loop. This is the same
class of trap as the MS5611's blocking `read()` (§4.2), and it is easy to miss
because the default *works* — it is only slow.

**Cold start is 30–60 s and needs sky view.** "No fix" on an indoor bench is the
expected result, not a fault, and the diagnostics sketch says so at boot rather
than leaving someone to debug a working receiver. The sketch also distinguishes
*never acquired* from *acquired and lost*, because those point at different
problems — sky view versus antenna or power.

**A 3D fix is not an accurate fix, and nothing in the fix type says so.**
Observed on the bench: a stationary board holding a **3D fix on 5 satellites**
sat **238 m** from where it first locked. The fix type read `3D` for the whole
session. Four satellites is the theoretical minimum for a 3D solution, so five
is barely above the floor, and with that few — all in whatever patch of sky a
window exposes — the error ellipsoid is long and thin. The wander runs along
its major axis, which is why poor fixes drift in a *line* rather than a blob.

**So carry `hAcc`.** The receiver publishes its own horizontal accuracy
estimate in the NAV-PVT message it is already sending, so reading it costs
nothing:

```cpp
int32_t hAccMm = gnss.getHorizontalAccEst();   // millimetres
```

Without it a position cannot be argued with. With it, "238 m from the pad" can
be read next to "±42 m claimed" and the display can *act* on the difference —
see the pad-datum gate in §6.7.

**Never take the pad datum from the first fix.** The first fix of a session is
the worst one you will ever get: cold start, fewest satellites, worst geometry.
Anything measured from it inherits that error silently, and every later, better
fix appears to *move* — which is precisely the 238 m above. Gate the datum on
satellites and `hAcc`, or set it deliberately once the fix has settled.

Multi-GNSS including **QZSS**, which has good coverage over the Philippines —
a meaningful advantage over GPS-only receivers at this latitude. Integrated
antenna with u-blox's own RF design removes the commonest silent failure mode
in DIY GNSS. Must face skyward with nothing metallic above it.

---

## 5. Deployment logic — the core design argument

This section documents *why* the deployment trigger is what it is, because the
reasoning is the most transferable part of this project.

### 5.1 The false dichotomy

"Deploy at apogee by altitude" versus "deploy when velocity reverses" is not a
real choice. **Barometric peak-and-drop detection *is* velocity-based** — it
triggers on altitude ceasing to rise, which is precisely the condition v = 0.

The real question is *how you obtain velocity*:

| Approach | Method | Error behaviour |
|---|---|---|
| Barometer | Differentiate measured position | **Bounded** — a bad sample is bad once; the next is fine |
| IMU | Integrate measured acceleration | **Unbounded** — every error becomes a permanent offset |

### 5.2 Why IMU-derived velocity fails here

**The gravity problem.** An accelerometer measures *specific force*, not
acceleration. In coast the vehicle is essentially in freefall, reading ≈ 0
rather than −1 g. Recovering true acceleration requires adding gravity back in
body frame, which requires knowing orientation, which comes from integrating a
gyro that drifts — and that has just been shocked by a 50 g boost. A 5°
attitude error means subtracting the wrong vector by 9.81 × sin(5°) ≈
**0.85 m/s²**. Over a 3-second coast that is ~2.5 m/s of velocity error, enough
to move the v = 0 crossing by a quarter second in an unknown direction.

**The initial condition problem.** Integration needs a starting value: burnout
velocity, obtained by integrating a boost phase where the ±16 g IMU clipped and
the ADXL375 provided perhaps 60 coarse samples of a 0.25 s burn. A 10% error in
burnout velocity is a permanent 10% offset thereafter.

**No external reference.** The IMU will report v = 0 confidently at the wrong
moment and give no indication anything is wrong.

### 5.3 Detection latency — the number that justified the sensor choice

Near apogee, altitude follows h = h_max − ½gt². The curve is flat there — that
is what makes it a peak, and it is exactly why sensor noise matters:

| Time past apogee | Distance fallen |
|---|---|
| 0.1 s | 5 cm |
| 0.2 s | 20 cm |
| 0.3 s | 44 cm |
| 0.5 s | 1.2 m |
| 0.7 s | 2.4 m |

A sensor needs roughly 3× its noise floor of confirmed drop before triggering:

- **MS5611** (~10 cm noise) → ~30 cm drop → **~0.25 s past apogee**, descending
  at ~2.5 m/s
- **BMP280** (~1 m noise) → ~2–3 m drop → **~0.7 s past apogee**, descending at
  ~7 m/s

Both deploy successfully. One deploys essentially at the peak.

### 5.4 State machine

```
IDLE      Reed switch open. Sensors live, telemetry live, deployment INERT.
  │ reed closed, stable 50 ms
ARMED     Pre-trigger ring buffer running (retains last ~1 s)
  │ high-g accel > 5 g sustained 50 ms
BOOST     Record t_launch. Arm switch now IGNORED (vibration must not disarm).
  │ accel magnitude < 1 g  (burnout)
COAST     Start lockout timer (500 ms) and backup apogee timer
  │ lockout expired AND (baro falling N samples OR backup timer fired)
DESCENT   Fire servo, latch deployFired flag (can never fire twice)
  │ altitude stable ±1 m for 2 s
LANDED    Dump buffer, buzzer locator pattern, beacon every 5 s
```

**Apogee detection (primary):**

```cpp
if (alt > altMax)                        { altMax = alt; fallCount = 0; }
else if (altMax - alt > APOGEE_DROP_M)   { fallCount++; }
else                                      { fallCount = 0; }

if (fallCount >= APOGEE_SAMPLES)  →  DESCENT
```

With the MS5611: `APOGEE_DROP_M = 0.30`, `APOGEE_SAMPLES = 5` at ~100 Hz.
Tune `APOGEE_DROP_M` to ≈ 3× the measured 1σ altitude noise from the bench
diagnostics sketch.

**Layered triggers.** Primary is barometric. Backup is a timer armed at
burnout (expected coast + 20%) — on a 3-second ascent this is genuinely
competitive, not a token fallback. Both are gated behind the burnout lockout;
no boost-phase pressure transient can fire the chute at 70 m/s.

**Where the IMU legitimately contributes:** integrated gyro pitch as a *tilt
inhibit* — if the airframe has not passed ~70–90° from vertical, deployment is
blocked regardless of what the barometer reports. This fails in a completely
different mode than the barometer, which is the point.

### 5.5 Attitude estimation for the tilt inhibit

The tilt inhibit above is only as good as the attitude estimate behind it.
Bench development of the ground-segment viewer (§6.7) produced three findings
that apply directly when this is ported into flight firmware. All numbers below
are from synthetic-signal tests with known ground truth, not flight data.

**The complementary filter must be PI, not P.** A proportional-only Mahony
filter settles at a *permanent* tilt offset of approximately `bias / K_P`. With
K_P = 2.4 that is 0.417° per dps of residual gyro bias — an error the filter
can never correct, because the proportional term has nothing to integrate
against. Since the tilt inhibit is a threshold comparison, a fixed offset is a
direct bias on the inhibit angle.

Adding an integral term that learns the residual bias online:

| Residual gyro bias | P-only tilt error | PI tilt error |
|---|---|---|
| 0.25 dps | 0.104° | 0.002° |
| 0.5 dps | 0.208° | 0.004° |
| 1.0 dps | 0.417° | 0.007° |
| 2.0 dps | 0.833° | 0.015° |

Roughly a 56× improvement, and the learned bias converged to within 2% of the
injected value. **The integrator must be clamped** (±0.12 rad/s used) or it
winds up during any extended period where gravity is not a usable reference —
which is precisely the boost phase. The implementation also freezes bias
learning whenever |a| falls outside 0.72–1.28 g, while still applying the bias
already learned.

Note this does **not** replace the startup bias calibration in §4.3. Boot
calibration removes the bulk offset; the integral term tracks what remains and
what drifts with temperature during pad time.

**Tilt must be derived from gravity direction, not quaternion composition.**
Without a magnetometer contribution, yaw is unobservable and drifts freely.
Computing tilt by composing the current orientation against a stored reference
orientation lets that yaw drift leak into the tilt number — badly, if the
reference pose was itself tilted. Comparing *gravity direction* instead is
immune, because a rotation about vertical leaves the up-vector unchanged.

Stress test — establish a reference, spin about vertical, return to the same
physical pose:

| Method | Reported tilt (truth ≈ 0°) |
|---|---|
| Quaternion composition | 37.84° |
| Gravity-direction comparison | 0.24° |

For a threshold that gates parachute deployment, this is not a cosmetic
difference.

**Zeroing must not reset the filter state.** Implementing "level here" by
resetting the tracked quaternion to identity does not work: the accelerometer
correction term pulls the estimate back to true gravity within a fraction of a
second, so any residual tilt reappears almost immediately. Record the current
orientation as a *display reference* instead, and leave the filter tracking
truth. Any flight-side "zero attitude" command must follow the same rule —
the estimator's job is to be correct, not to agree with a button press.

Done this way, returning to a levelled pose reads ≈ 0.04° residual. What
remains is sensor noise, not systematic error.

---

## 6. Radio and telemetry

### 6.1 Link design

Two RFM95W (SX1276) modules — one flight, one ground. LoRa spread spectrum was
chosen over FSK (RFM69HCW) for ~10–20 dB of additional link budget, which is
decisive in the case that actually breaks the link: a landed vehicle with its
antenna horizontal in wet grass.

**Link budget at 3 km, +14 dBm, SF7/BW250:**

```
TX power                    +14 dBm
TX antenna gain              +2 dBi
RX antenna gain              +2 dBi
Free-space path loss @3 km  −101 dB
─────────────────────────────────────
Received signal              −83 dBm
LoRa SF7/BW250 sensitivity  −120 dBm
─────────────────────────────────────
Margin                        37 dB
```

The link is not range-limited. It is limited by **antenna nulls during tumble**
and by **landed ground-to-ground geometry**. Additional TX power does not
address either; receive-side antenna quality does.

### 6.2 Packet design

18 bytes, self-contained, fire-and-forget:

```c
struct __attribute__((packed)) Telem {
  uint16_t t_ds;      // time since launch, deciseconds
  int16_t  alt_dm;    // altitude, decimetres
  int16_t  vel_dms;   // vertical velocity, decimetres/sec
  int16_t  accel_cg;  // accel magnitude, centi-g
  uint8_t  state;     // state machine enum
  uint8_t  flags;     // bit0 armed, bit1 deployed, bit2 gps fix, bit3 low batt
  int32_t  lat_e7;    // latitude ×1e7 (0 if no fix)
  int32_t  lon_e7;    // longitude ×1e7
};
```

**No acknowledgements, no retries, no sequence-dependent deltas.** Packets will
drop — the transmitter is a tumbling wet plastic tube. Every packet must stand
alone and the ground station must tolerate gaps silently.

Rate: 10–20 Hz in flight (duty-cycle permitting, see §12), 1 packet / 5 s as a
landed beacon.

### 6.3 The most useful function of the link

Not the flight stream — **pre-launch verification.** With the nose cone sealed
and the vehicle pressurised to 100 psi, the link confirms sensors are
responding, ground pressure is captured, battery is healthy, and the state
machine reads `ARMED`. Knowing the avionics are alive *before* pulling the
release is worth more than the flight data.

### 6.4 Antennas

**Flight unit — build, do not buy.** 82 mm solid-core wire (quarter-wave at
868 MHz), soldered directly to the ANT pad. The Adafruit breakout ships as
**bare pads** — no connector fitted.

- Cut accurately; wrong length costs more range than any power setting.
- Gentle curves following the nose cone interior are fine. Sharp kinks and
  tight coils are not.
- Strain-relieve the solder joint with RTV or hot glue — the joint, not the
  wire, is the fatigue point.
- Route clear of the battery, GPS antenna, and metal standoffs.

**Ground station — buy.** A 200 mm rigid whip rated for 868 MHz. This requires
**adding an SMA edge-mount connector** to the board (a separate ~₱100 part,
hand-soldered to the ANT pad), or alternatively a uFL connector plus
uFL-to-SMA pigtail.

**Rejected:** GSM/cellular antennas (900–1800 MHz, 824–960/1710–1990 MHz,
780–960/1710–2170 MHz). None presents a correct 50 Ω match at 868 MHz; they
degrade both TX efficiency and RX sensitivity. Correct search term is
"868 MHz LoRa antenna", not GSM or FONA-branded parts.

**Rejected for the ground station:** high-gain omnidirectional (colinear)
antennas squash the pattern toward the horizon, sacrificing coverage near
zenith — exactly where a launch-adjacent receiver needs it. A Yagi is
excellent as a *handheld post-landing direction-finding* tool but wrong for
live tracking without an az/el gimbal.

**Hard rules:** never key the transmitter without an antenna attached (PA
damage). Both units must be the same variant at the same configured frequency.

### 6.5 Ground station

**Status: built and working on hardware. Telemetry source is synthetic — the
radio link is not yet wired in.**

The ground station is a Raspberry Pi Pico W running MicroPython. It brings up
its **own Wi-Fi access point** and serves the attitude viewer to any phone or
laptop that joins. No router, no internet, no app install — which is the point,
because a launch site has none of those.

```
   Rocket ──LoRa──▶ [RFM95W] ──SPI───▶ Pico W ──Wi-Fi AP──▶ phone browser
                                        │                     (viewer)
                                        └─ serves index.html + SSE stream
```

> **Correction:** this diagram previously showed the RFM95W reaching the Pico W
> over UART. An RFM95W is an SPI transceiver with no UART. The ground station's
> radio producer will be a LoRa driver on SPI, decoding the §6.2 packet into a
> §6.6 line — not a serial reader.

![The v0.3.1 bench rig: the flight stack on one breadboard, the Pico W ground
station on its own breadboard beside it, joined by four jumpers, with the servo
and buzzer to one side and the whole rig on a power bank](docs/images/bench-wired-link-1.jpg)

*The v0.3.1 bench rig. The flight stack on the left, the Pico W ground station
on its own board at right, joined by four jumpers — TX and RX crossed, ground,
and VSYS — and the whole rig running off a power bank. The Pico W has no USB of
its own. There is no radio anywhere in this path.*

**Wired downlink (v0.3.1) — a stand-in for the radio.** Until the antenna
pigtails and connectors are on hand, the flight computer's UART0 is wired
straight into the ground station's. It carries the §6.6 line, is read into the
same producer slot the radio will use, and so exercises everything *downstream*
of the receiver — staleness, SSE, the viewer — and nothing about the receiver:
no loss, no range, no 868 MHz, no 18-byte packet.

**It carries commands up as well — by deliberate choice.** The wire was built
transmit-only first, precisely so that ground control of the latch could not
arrive *by accident* inside a test rig. It was then asked for on purpose:
arming, firing, zeroing and the rest from a phone. So a phone's command goes to
the station's `/cmd`, is **rebuilt from an allowlist** — never forwarded as
typed — and relayed up the wire, where the flight computer accepts **framed
`!word` lines only**, never a bare byte. That last rule matters: an unplugged
RX pin picks up noise, and a single noise byte that looked like `z` or `b` would
re-zero the barometer or start a blocking gyro average from nobody.

Three further rules. **Board commands are refused unless the wire is the
selected source**, at the station and in the page: arming a real latch while the
display shows synthetic data would put a fictional vehicle's latch state beside
a real actuator. **The deadman still rules** — the phone must heartbeat like the
USB viewer does, so locking the screen, switching apps or losing Wi-Fi stops the
heartbeats and the board disarms itself within 3 s; that property is what makes
a remote arming path tolerable at all. And **a `200` from `/cmd` means the
station put the command on the wire, not that the board acted** — the board's
own reply arrives in the phone's console, and the latch state in the next frame.

The board's messages come back down the same wire as `M,` lines, which the
station fans out to every connected phone as SSE `msg` events, replaying the
recent ones to a phone that joins late.

<p align="center">
<img src="docs/images/phone-attitude.jpg" width="31%" alt="The viewer on a phone, served by the ground station: board command buttons, the source selector reading Wired link (UART), status Live, and the 3D model">
<img src="docs/images/phone-readouts.jpg" width="31%" alt="The viewer on a phone: readouts for attitude, accelerometer, barometer, high-g, GPS and link, strip charts, and the latch reading safe">
<img src="docs/images/phone-deploy.jpg" width="31%" alt="The deployment panel on a phone, with the condition list, endpoints, buzzer buttons, and the serial monitor showing the board's own GPS messages">
</p>

*The same viewer on a phone, served by the ground station over its own Wi-Fi.
Left: the board controls live, and the source selector reading `Wired link
(UART)` — what the station is actually running, read from `/health`. Centre:
live readouts at 23 Hz on the board's clock with zero bad lines, and the latch
reading `safe` as the **board** reports it. Right: the deployment panel, with
the console showing messages the flight computer sent down the wire.*

> **None of this carries over to flight.** The flight system arms by a reed
> switch and a magnet, physically (§8.4), precisely so that nothing remote can
> arm it — and deployment must never depend on a link (§2.3). Remote arming over
> the radio would need its own design, if it is wanted at all. This is a bench
> rig with a servo on a desk.

The station selects its source **explicitly** — `uart` (the default) or a
synthetic profile — and **never falls back** from the wire to synthetic data.
Quietly substituting plausible fake motion for a dead link is the worst thing
this display could do; instead the stream goes stale and the viewer dims.
The viewer also no longer reads `Live` merely because the SSE connection
opened: with nothing on the wire the station connects and forwards nothing,
so `Live` waits for a real frame.

| Flight Pico | | Pico W | |
|---|---|---|---|
| **GP0** — UART0 TX, pin 1 | → | **GP1** — UART0 RX, pin 2 | telemetry down |
| **GP1** — UART0 RX, pin 2 | ← | **GP0** — UART0 TX, pin 1 | commands up — needed for phone control |
| **GND**, pin 3 | — | **GND**, pin 3 | required — no shared ground, no signal reference |

**Power: two setups work.**

- **Each Pico on its own USB**, nothing else between them.
- **One power bank, VSYS joined to VSYS** (pin 39 to pin 39) — the bench setup
  in use. Safe: each board's Schottky diode-ORs onto the shared rail, even if
  both are also on USB. It also satisfies the rule below automatically, since
  both boards power up together. **But move the servo's V+ from VBUS to
  VSYS.** VBUS only exists on a board whose own USB is plugged in; with the
  bank in the Pico W, the flight Pico's VBUS is dead and so is the latch. On
  VSYS it works whichever board holds the bank. Put the 220 µF across VSYS and
  GND near the servo: its inrush now sags the rail both boards and the Wi-Fi
  radio share, and a brown-out drops the phone's connection mid-test. Some
  power banks also switch off below ~100 mA of draw; if the rig dies after half
  a minute of idle, that is the bank, not the firmware.

**Never:**

- **3V3_EN is an input, not a supply.** It is the enable pin of each Pico's
  own 3.3 V regulator, pulled up to VSYS through 100 kΩ. It powers nothing.
  Tie it to VSYS and nothing changes; tie it to GND — or to the other board
  while that board is off — and that Pico switches its own 3.3 V rail off and
  appears dead.
- **Never feed one board's 3V3 into the other's VSYS.** If the second board is
  also on USB, its VSYS sits near 4.7 V, and the wire pushes that into the
  first board's 3.3 V rail — rated to 3.6 V, and shared with the RP2040 and
  every sensor on the bus.
- **Never power one board with the other off** when their UARTs are joined,
  unless there is ~1 kΩ in series with each TX. A powered board's TX idles high
  and back-feeds the unpowered one through its RX pin's protection diode.

**Why 460800 baud.** `Serial1.write()` on the Mbed core **blocks**: it goes
through `mbed::UnbufferedSerial` and busy-waits on `writeable()`. The RP2040
TX FIFO absorbs 32 bytes and every byte after that waits for the wire. A
~100-byte line at 115200 would stall the flight loop about 6 ms per frame; at
460800 it is about 1.5 ms. In the flight firmware's 500 Hz loop even that is
too much, which is one more reason §10.2 puts telemetry in the normal-priority
thread.

**Why MicroPython and not Arduino here.** Pico W networking is far better
supported, and the filesystem means the ~45 KB viewer is a file rather than a
string literal compiled into a sketch. The ground station is a separate device
from the flight computer, so the toolchains need not match. The flight computer
remains on the Mbed Arduino core per §9.2.

**Architecture: one producer, many consumers.** A single task advances the
telemetry state at a fixed rate; every connected browser reads the most recent
frame. The distinction matters: advancing the source inside each client's
stream loop instead makes the simulation run at 2× with two phones connected
and makes the two viewers disagree with each other. Verified as built: three
simultaneous clients each receive 25.0 Hz of byte-identical frames while the
source advances at 1×.

That structure is also the correct shape for the radio: the packet receiver
becomes the producer and nothing else changes.

| Endpoint | Purpose |
|---|---|
| `/` | The viewer |
| `/stream` | Server-Sent Events, one telemetry frame per event |
| `/health` | JSON — mode, frame count, client count, free RAM, uptime |
| `/mode?m=` | Switch synthetic source: `bench`, `flight`, `still` |

**Two settings that are not optional for a steady stream:**

- **Wi-Fi power management disabled** (`ap.config(pm=0xa11140)`). The CYW43
  radio parks itself between packets by default. Fine for request/response,
  visible as intermittent stutter on a continuous 25 Hz stream. Costs roughly
  20–30 mA of idle current — irrelevant on a ground power bank.
- **Nagle's algorithm disabled** on the stream socket. Telemetry frames are
  ~60 bytes; Nagle withholds a small packet until the previous is ACKed, and
  phones delay ACKs by up to ~200 ms. The interaction produces exactly the
  intermittent multi-frame stall it was reported as.

**Synthetic sources.** Until the radio exists, three profiles are selectable:
`bench` (gentle desk-scale motion, the realistic case for UI work), `flight`
(a 40 s loop — 1.6 s burn at 5 g, 377 m apogee, 78 m/s peak ascent, 6 m/s chute
descent, tipping after apogee), and `still` (flat output for noise and drift
checks). The flight profile deliberately peaks at 6 g, which clips a ±2 g
accelerometer — that clipping is real and the display should show it rather
than hide it.

### 6.6 Telemetry wire format

One line format is shared by the serial path and the Wi-Fi path, so the viewer
has a single parser regardless of how frames arrive:

```
V,ax,ay,az,gx,gy,gz,alt,vel,millis,hg,fix,sats,lat,lon,hacc,srv,srvus
```

Accelerations in g, rates in dps, altitude in metres AGL, velocity in m/s. The
tenth field is the **flight computer's own `millis()`**; fields 11–15 carry the
high-g accelerometer and the GPS.

| Field | Meaning |
|---|---|
| `hg` | High-g magnitude in g, or **−1** if the ADXL375 is not fitted |
| `fix` | u-blox fix type 0–5, or **−1** if the GPS is not fitted |
| `sats` | Satellites used in the solution |
| `lat`, `lon` | Decimal degrees, `0` when there is no fix |
| `hacc` | The receiver's **own** horizontal accuracy estimate in metres, or **−1** with no fix or no GPS |
| `srv` | Servo latch: `0` safe, `1` armed, `2` armed and fired, **−1** if the source has no latch |
| `srvus` | Pulse width **commanded** to the latch in µs; `0` when not driven |

`hacc` earns its place because a position with no accuracy beside it cannot be
argued with — see §4.5, where a 3D fix on five satellites wandered 238 m and
nothing else in the output disagreed.

**Fields are appended, never inserted.** A parser reading only the first eight
or nine fields is unaffected, which is what makes this a compatible extension
rather than a new format. The viewer treats a short line as "this firmware
predates the field", distinct from a `−1` meaning "the part is not fitted".

**−1 rather than 0 marks absence** because 0 is a legal value for every one of
them: 0.00 g is a real high-g reading in freefall, fix type 0 is a real "no
fix", and 0,0 is a real position in the Gulf of Guinea. A sentinel that
collides with valid data is not a sentinel.

**That timestamp matters more than it looks.** Without it the receiver derives
`dt` from packet *arrival* time, and USB and radio both deliver in bursts —
several frames land microseconds apart, then a gap. That jitter feeds straight
into attitude integration as noise. Timestamping at the source removes it. When
the radio link is built, this field should carry through unchanged; it is the
only thing that makes the arrival-time jitter of a lossy RF link harmless to
the attitude estimate.

This is the *display* format, distinct from the 18-byte packed LoRa packet in
§6.2 and from the 28-byte flight record in §10.3. Ground-side conversion from
LoRa packet to this line format is a formatting step in the receiver.

### 6.7 Attitude viewer

A single self-contained HTML file (`rocket_attitude_viewer.html`) that renders
live vehicle attitude as a 3D model with a tilt protractor, numeric readouts,
and strip charts for altitude, |a| and tilt.

**Dual transport, one codebase.** Served over HTTP it consumes Server-Sent
Events from the ground station; opened as a local file it uses Web Serial
directly against the flight computer over USB. Mode is detected from
`location.protocol`. This matters because **phones have no Web Serial at all** —
which is precisely why the ground station serves the page rather than asking
the phone to open a port.

**Two builds are kept, deliberately.** `rocket_attitude_viewer.html` is the
dual-transport build above, and is what the ground station serves as its
`index.html`. `rocket_attitude_viewer_serial.html` is a serial-only build,
kept because it is the transport that **works today**: with no radio
between the flight computer and the ground station yet, driving the viewer
directly off the flight computer over USB is the only path carrying real sensor
data. It is a bench instrument, not a dead file. Retire it once the radio link
in §6.5 is wired in and the SSE path carries live telemetry.

**Display smoothing is separate from measurement.** Exponential moving averages
(with median-of-3 ahead of the altitude filter, since the barometer throws
single-sample spikes an average would smear rather than reject) quiet the
readouts by roughly 2.6× on tilt, 3.1× on |a| and 4.7× on altitude, at a cost
of 0.32 s and 1.12 s respectively to reach 90% of a step.

![The attitude view: 3D rocket model with a tilt protractor and reference
axis, a column of numeric readouts, three strip charts, and the deployment test
panel and serial monitor expanded beneath](docs/images/viewer-attitude.png)

*The attitude view driven live over USB. The GPS block shows the pad-datum gate
holding: with no fix, `FROM PAD` reads `no datum` rather than a fabricated
distance, and `ACCURACY` reads `—` because there is no estimate to report. The
raw telemetry line is the §6.6 format — `hg = 0.37`, then a trailing
`0,0,0,0,-1,2,0`: no fix, no accuracy estimate, latch fired, pulse train
stopped. Beneath it the deployment panel refuses a second fire until the latch
is re-latched, and the serial monitor shows the board's own log of the fire:
`SERVO FIRED -> 2000 us`, then safe with the move finishing over the settle
window, then `pulse train stopped - move complete`.*

**Two stage views, one canvas.** The stage switches between **Attitude** —
the 3D model, tilt protractor and reference axis — and **Trajectory**, which
draws the flown path in 3D over a metric ground grid, with a drag-to-orbit
camera that auto-frames the data.

> **Only one axis of the trajectory is measured, and the drawing says so.**
> Altitude is barometric: driftless, ~10 cm noise, honest. Horizontal position
> is GPS — and the GPS is blind for the whole flight (§4.5), losing lock at
> launch and taking 5–15 s to reacquire on a ~10 s flight.
>
> The IMU cannot fill that gap. §5.2 already rejects integrating it for
> *velocity*; position is that same error integrated **twice**, which is tens
> of metres over a 3 s coast with no way to know you are wrong. So an unlocked
> segment is drawn **dashed and red with the horizontal frozen at the last
> fix**, never interpolated. A smooth arc through the gap would be an
> invention, and the one thing this view must be is trustworthy about where
> the vehicle actually was.

![The trajectory tab indoors with no GPS fix: a ground grid in perspective with
a single vertical dashed red line rising from it, and a legend in the lower
left](docs/images/viewer-trajectory.png)

*The trajectory tab on the bench, indoors, with no fix. The path is a single
vertical **dashed red** line: altitude is measured, and horizontal is frozen
because there is neither a lock nor a pad datum — which is exactly what the
legend in the lower left says, down to the gate's thresholds. Nothing has been
interpolated, and the connection hint that used to overlap the legend is hidden
on this tab.*

**The pad datum is gated, not taken from the first fix.** §4.5 explains why:
the first fix is the worst one of the session, and everything measured from it
inherits that error invisibly. The viewer accepts a datum only from a fix the
receiver itself vouches for — **≥ 6 satellites and `hAcc` ≤ 10 m** — and until
then reports `no pad datum` rather than a fabricated distance from a bad
origin. A **Set pad** button re-datums on the next fix that clears the same
bar, which is the launch-day workflow: zero the barometer and set the pad in
the same moment, once the receiver has settled.

**The gate is a toggle, and the two questions behind it are kept apart.**
A `Fix gate` button sits beside `Smoothing`, and turning it off lets any 2D
fix set the datum. The distinction that matters is between *is this fix any
good?* and *may I take a datum from it?* — only the second is the operator's
to answer. So a weak fix accepted with the gate off is still drawn faded and
still counted as low-confidence: switching the gate off permits a datum, it
does not make the fix accurate, and the display must not imply otherwise. The
trajectory legend states plainly when the gate is off.

Turning the gate off deliberately does **not** move an existing datum. Relaxing
the bar lets the next fix establish one where there is none; moving a datum that
already exists is what `Set pad` is for, and doing it silently would shift every
distance on screen without anyone asking.

**A missing `hAcc` no longer blocks the datum forever.** The gate originally
hard-required the accuracy field, which contradicted §6.6's rule that appended
fields are optional: any board running firmware older than that field could
never establish a datum, and the only symptom was a **purely vertical
trajectory** with no message explaining it. Where `hAcc` is absent the gate now
falls back to satellite count alone.

**Fixes worse than the bar are drawn, but visibly weaker.** A faded blue
segment is a position the receiver does not stand behind. Dropping it would
hide that the vehicle was somewhere; drawing it at full strength would
overstate how well we know where. The legend names the threshold.

Where it earns its keep before any flight: the §13.3 **GPS dynamic model** test
("drive with logging active → plausible track") exercises it immediately, and
after landing it gives pad-to-landing displacement for the walk to the vehicle.
A true flight path is a **post-flight reconstruction in Python** from the logged
raw data, per §2.5 — not a live view.

**Path sampling is throttled on the board's clock, not on arrival time.** Both
transports deliver in bursts, so throttling on arrival collapses whole bursts
into a single point and samples the path unevenly. The `millis()` field of §6.6
exists to make arrival jitter harmless, and the trajectory recorder uses it for
exactly that reason.

**Board command buttons.** `Zero baro`, `Gyro bias`, `Reset peaks` and
`GPS status` — and the whole deployment panel — work over both transports.
Over USB they go down the same port the telemetry arrives on; served by the
ground station, they go through `/cmd` and up the wired link (§6.5), and are
greyed out whenever the station's source is synthetic. `c` (CSV) and `h`
(reprint header) are deliberately not exposed on either — both would corrupt
the stream the page is reading — and the wire cannot carry them at all.

Zeroing the barometer and re-measuring gyro bias block the sketch for a second
or more while they average, so frames genuinely stop. The staleness indicator
below is suppressed for the duration: the flag would be *correct*, but crying
outage about a pause the operator asked for trains people to ignore it.

**A deployment test harness drives the latch.** A collapsed panel — collapsed
because it is an actuator control — holds an arm/safe toggle, a manual fire and
re-latch, endpoint calibration in microseconds, and a list of trigger conditions
you can edit live: altitude, drop below peak, vertical velocity, tilt, |a|,
high-g, rate magnitude, satellite count, and time since arming. Each takes a
comparator and a threshold, and the set combines with **all** or **any**. A
**sustain window** requires the conditions to hold continuously before firing —
the same reasoning as `APOGEE_SAMPLES` in §5.4, since a single threshold
crossing on noisy barometric data is exactly what you must not fire on.

`Drop below peak` is there deliberately: it is the primary apogee trigger of
§5.4, and tuning it against real barometer noise is the most useful thing this
harness does. **Export** emits the tuned values as C constants for the flight
firmware, because the numbers are the deliverable — the logic is not.

Four interlocks, in the order they matter:

- The board **does not drive the pin at boot**. The servo is not attached, so
  no pulse train exists at all and no position is commanded until somebody arms
  it. Arming attaches; the library emits nothing until the first write.
- **Nothing moves while disarmed**, and arming does not itself move anything —
  if arming drove the horn, preparing to test would be the test.
- A **3 s deadman on the board** disarms the latch if the host goes quiet. Note
  this is the opposite of what flight firmware must do, which is one more reason
  this is a harness and not the flight path.
- **Firing latches, and auto-safes.** The board refuses a second fire without an
  explicit re-latch; the viewer disarms itself afterwards, and also on telemetry
  staleness or a closed port. A test rig that stays hot after doing the thing is
  how the second, unintended actuation happens.

  **But safe must not truncate the move it follows** — and at first it did.
  The viewer sends `!fire` and then `!safe` back to back, and `!safe` detached
  the servo a millisecond after `!fire` had written the released position. That
  cut the pulse train long before the horn could travel, so Fire and every
  condition-triggered fire did nothing, while `!us` and `!latch` — which leave
  the servo attached — worked perfectly. The interlock defeated the action it
  guarded. Safe now refuses new motion from the instant it arrives, but lets a
  move already commanded run for an **800 ms settle window** before the pulse
  train stops. The timing lives on the board, where the actuator is, so it holds
  whatever sends the commands.

The panel shows the **board's** view of the latch, not the page's belief. If the
two disagree — the deadman having fired, say — that disagreement is the thing
you most need to see.

**Buzzer patterns are selectable** from the same panel: chirp, double, locator
and alarm, plus off. The board owns the timing, not the page — a beep that
stopped because a browser tab was backgrounded would be worst for the locator,
the one pattern that runs when nobody is looking at a screen. Arming the latch
chirps and firing it double-beeps, which is §8.5's arming confirmation made
audible.

**A serial monitor is built into the page.** The footer shows only the most
recent line, which is useless the moment the board says anything worth reading —
the boot banner, the decoded configuration registers, the reply to a `g`
command — because all of it scrolls past at 25 Hz. The alternative was
disconnecting and opening the Arduino Serial Monitor, which means surrendering
the port and therefore the live view, a poor trade for a one-line answer.

The console is collapsed by default and expands into a scrolling log. Three
details earn their place:

- **Telemetry frames are excluded by default.** At 25 Hz they bury everything
  else within a second, and everything else is the reason the log exists. A
  toggle includes them when the raw stream is what you want to see.
- **Blank lines are preserved.** The boot output uses them as structure, and a
  monitor that eats them is harder to read than the one it replaces. Lines are
  captured before the parser's trim-and-discard step for exactly this reason.
- **It sticks to the bottom only if you are already there.** Yanking the view
  down while someone is scrolled up reading is the one thing a log pane must
  not do.

It is **read-only by design**. The safe commands already have buttons, and `c`
and `h` would corrupt the very stream the page is parsing, so there is no free
text field from which to fire them.

**Telemetry staleness is shown, not inferred.** Once frames stop arriving, a
frozen readout at full contrast is indistinguishable from a vehicle sitting
perfectly still — a safety-relevant display defect rather than a cosmetic one.
The viewer flags a gap of more than **1.5 s** (≈37 missed frames at 25 Hz): the
status dot turns amber, the status line counts the age of the last frame, and
the numeric readouts dim. This is deliberately independent of each transport's
own error handling, because neither one catches the case that matters most: an
SSE connection stays happily open while the ground station has nothing to
forward, and an open serial port that has gone quiet raises nothing. The strip
charts need no equivalent treatment — they only advance on ingest, so a gap
stays a visible gap and is never back-filled.

**Peak trackers deliberately run on raw data**, because smoothing a peak
understates the maximum, which defeats its purpose. A smoothing on/off toggle
exists for the same reason: **the §13.2 altitude-noise measurement that sets
`APOGEE_DROP_M` must be taken with smoothing OFF**, or the barometer will
appear ~4.7× quieter than it is and the apogee threshold will be set far too
tight.

**Mobile performance.** Phones stuttered until the render path was fixed: the
original loop called `getBoundingClientRect()` and reassigned `canvas.width` on
all four canvases every frame — 480 layout reflows and 480 buffer
reallocations per two seconds at 60 fps, now 4 of each. Also: pixel ratio
capped at 1.5 on touch devices, render capped at 30 fps with charts at 10 fps
(telemetry only arrives at 25 Hz), DOM writes skipped when the formatted string
is unchanged, and drawing halted entirely when the tab is hidden.

---

## 7. Power subsystem

**Cell:** PKCell 500 mAh 1S LiPo (LP503035), 10.5 g, with integrated protection
(over-charge, over-discharge, short). Genuine JST-PH 2.0 mm connector.

**Why 3.7 V nominal is the convenient choice.** The Pico's VSYS accepts
1.8–5.5 V through an RT6150 buck-*boost* converter. The LiPo's full
4.2 V → 3.0 V discharge range sits entirely inside that window, so the 3.3 V
rail is stable throughout with no external regulator. Every sensor in the stack
is 3.3 V native — zero level shifting anywhere. (Contrast a 5 V Arduino, which
would need level shifters on every device.)

**Rejected: 2× 18650.** ~90 g and 30 hours of runtime for a 10-second flight —
15× the energy at 12× the mass, plus a buck converter and its switching noise.
Nose mass directly costs apogee and shifts CG forward.

**Servo voltage caveat.** The MG90D is rated 4.8–6 V; at 3.7–4.2 V it delivers
roughly 60–70% of rated torque. Acceptable *provided* the latch geometry keeps
the servo out of the load path (§8). Verify latch actuation at 3.5 V, not just
on a fresh charge.

**Decoupling — not optional.** 220 µF at the servo power pins and at the LoRa
module. Servo inrush (~700 mA stall) and LoRa TX spikes are the classic cause
of unexplained Pico resets mid-flight. Plus 100 nF at every IC.

**Battery telemetry.** 100 kΩ / 100 kΩ divider to GP26 (ADC0), reported in the
LoRa packet. Knowing battery state before pressurising to 100 psi is worth two
resistors.

**Charging.** Adafruit Micro-LiPo (MCP73831), jumper set to **500 mA** (1 C for
this cell). Charge as a separate, deliberate step with the battery
disconnected from the vehicle — in-circuit charging confuses the charger's
termination detection. Charge on a non-flammable surface, never unattended.

---

## 8. Recovery and safety

### 8.1 Deployment mechanism

Servo-actuated pin latch at the collar joint between the motor and avionics
sections. No pyrotechnics — a plastic airframe at this scale does not need
them, and a resettable mechanism can be bench-tested a hundred times.

![The MG90D micro servo and the LS3040 piezo buzzer on the desk in front of
the breadboard stack](docs/images/bench-stack-2.jpg)

*The latch actuator and the buzzer as bench-built: MG90D on GP6, LS3040 on GP7.
The servo drives nothing yet — the collar joint and pin latch of §8.2 are not
built — so on the bench a moving horn is the whole test.*

**Bench testing (§13.2).** The latch is on **GP6**, driven through the Servo
library (§9.2), and `servo_smoke/` exists to isolate it — one pin, one servo,
no sensors — for when the question is whether a stationary latch is firmware or
wiring. The diagnostics sketch drives it, with a configurable trigger harness in the viewer (§6.7). Two things
about that are worth stating here rather than leaving implied:

**The harness is not the flight path.** Its conditions are evaluated in the
browser and a fire command is sent down the wire. That is the right shape for a
bench rig — thresholds change without a reflash, and the operator is standing
next to the actuator — and exactly the wrong shape for flight, where deployment
must live in this board's own state machine and must never depend on a link
(§2.3, §5.4). What the harness produces is a set of *numbers*; those are what
get compiled into the flight firmware.

**A servo is the one device here with no readback.** §4.3 makes reading
configuration back from the hardware the standing rule, and a hobby servo
cannot honour it: there is no feedback path at all. The `srvus` field of §6.6
is what was *commanded*, never what the horn did, and a stalled, stripped or
unpowered servo reports exactly the same as a healthy one. Firmware cannot fix
this. The honest answer is mechanical — **a limit or reed switch on the latch
confirming the pin actually withdrew** — and it is an open item. Until it
exists, treat "fired" as "commanded to fire", and confirm release by eye or by
the ×50 pull test.

**The critical design rule: the servo must never carry structural load.**

The pin sits in **double shear** against the collar structure, so the full
thrust reaction goes into the airframe. The servo only overcomes pin-withdrawal
friction (typically 5–15 N on a lubricated snug fit). An MG90D at 1.8 kg-cm
through a 12 mm horn delivers ~14.7 N — adequate, but only under this
constraint.

Routing structural load through the servo gear train instead is a genuine
hazard: cyclic vibration under static load causes **backlash walk**, where the
output shaft creeps until it releases — producing uncommanded separation
*during boost*.

### 8.2 Load analysis

Peak thrust (momentum model, water incompressible: F = 2·A_nozzle·ΔP):

```
100 psi gauge, 21 mm nozzle  →  478 N  (49 kgf)
```

Pin sizing in double shear against 478 N:

| Pin | Capacity | Safety factor |
|---|---|---|
| 1 mm piano wire | 785 N | **1.6× — inadequate** |
| 2 mm piano wire | 3,142 N | 6.6× ✓ |
| 3 mm mild steel | 4,948 N | 10.4× |
| 3 mm stainless (304) | 6,362 N | 13.3× |

**Minimum 2 mm.** The 1 mm wire adequate for a nose-cone latch is *not*
adequate here.

### 8.3 Separation spring

At apogee there is no dynamic pressure to assist separation; the spring does
100% of the work.

Sizing against 0.18 kg forward mass, 1.0 m/s target separation velocity, 3 cm
stroke, 4 N assumed friction, 2× margin:

```
k ≈ 0.9–1.0 N/mm  →  ~28 N peak at full compression
```

**Design rule:** the spring must push the collar halves apart, never preload
the pin. A spring acting through the pin's withdrawal axis adds friction the
servo must fight on release.

**Prototype with rubber bands.** Friction and forward mass are both estimates
until measured. An elastic loop is tunable by hand; swap to a rated compression
spring once slow-motion video against a ruler confirms the real separation
velocity.

### 8.4 Arming interlock

A reed switch (magnet-actuated from outside a sealed airframe) separates
"powered on" from "deployment live."

**The failure it prevents:** kneeling at the pad with the bay open, packing the
chute, and bumping the vehicle. The accelerometer sees 5 g, the state machine
declares launch, and 3 seconds later the timer backup fires the servo in your
hands.

While the reed is open, the state machine cannot leave `IDLE`. Sensors run,
telemetry streams, deployment is inert.

**Implementation:** GP12 with internal pull-up, switch to GND, closed = armed,
50 ms debounce. Once `BOOST` is entered the arm input is **ignored** — vibration
must not be able to disarm mid-flight.

**Why a reed switch over a toggle:** sealed inside the airframe, no hole to
admit water, nothing protruding to snap off on landing, under 1 g. Panel-mount
toggles (5 g, 33 mm, 6 mm hole) belong on the ground station, not the vehicle.
Momentary buttons are wrong entirely — **momentary = an action, latching = a
condition**, and arming is a condition.

**Handling note:** reed switch glass is fragile. Grip leads with pliers between
the bend and the body; mount with foam or soft adhesive, never rigidly.

### 8.5 Launch sequence

1. Battery connected, board powers up → `IDLE`
2. Pack parachute, seat collar joint
3. Mount on launcher, connect fill line
4. **ARM** — magnet applied. Buzzer chirps; telemetry shows `ARMED`
5. All personnel step back
6. Pressurise and release

Recovery is the reverse: **disarm first**, before touching the vehicle.

---

## 9. Electrical reference

### 9.1 Pin map (Raspberry Pi Pico)

| Function | GPIO | Pin | Notes |
|---|---|---|---|
| UART0 TX | GP0 | 1 | **Wired downlink to the ground station** (v0.3.1 radio stand-in), 460800 baud |
| UART0 RX | GP1 | 2 | Wired uplink: framed `!` commands from the ground station, relayed from a phone |
| I²C0 SDA | GP4 | 6 | All sensors |
| I²C0 SCL | GP5 | 7 | 400 kHz |
| SPI0 MISO | GP16 | 21 | RFM95W |
| SPI0 CS | GP17 | 22 | RFM95W |
| SPI0 SCK | GP18 | 24 | RFM95W |
| SPI0 MOSI | GP19 | 25 | RFM95W |
| LoRa RESET | GP20 | 26 | Tie high if pins tight |
| LoRa DIO0 | GP21 | 27 | Or poll registers |
| Servo PWM | **GP6** | 9 | 50 Hz, separate rail. **As built** — this section specified GP15; the servo is on GP6 |
| Buzzer | **GP7** | 10 | **As built** — this section specified GP13. LS3040 piezo, ~4 kHz. Transistor if driven above 3V3 |
| Reed (arming) | GP12 | 16 | Pull-up, switch to GND |
| Battery sense | GP26 | 31 | ADC0 via 100k/100k |
| Status LED | GP25 | — | Onboard |
| VSYS (batt in) | — | 39 | 1.8–5.5 V |
| GND | — | 38 | Common with servo |

Free: GP2, GP3, GP8–GP11, GP13, GP14, GP15, GP22, GP27, GP28 — ample margin for
a nichrome backup channel, second deployment event, or an OLED. (GP15 and GP13
are free again now the servo and buzzer sit on GP6 and GP7; GP0 and GP1 carry
the wired downlink until the radio replaces it.)

### 9.2 Toolchain

**Official Arduino Mbed OS RP2040 core** (not the Earle Philhower community
core). This has non-obvious consequences — see §10.1.

> **The Mbed core does not bundle a Servo library.** This table previously said
> it did. Verified against core 4.6.0, whose entire bundled set is MRI, PDM,
> SPI, Scheduler, ThreadDebug, USBHID, USBMSD and Wire. Install **Servo** from
> the Library Manager; it declares `mbed_rp2040` support and ships an mbed
> backend. **Confirmed working on hardware.**
>
> **`mbed::PwmOut` is untested — not known to be broken.** It was tried first,
> and the servo did not move. An earlier revision of this document concluded
> from that that `PwmOut` does not drive the pin on this core, and backed the
> conclusion with the observation that the Servo library and the core's own
> `tone()` both bit-bang with `DigitalOut` and a `Ticker` rather than use it.
> **Both halves of that are withdrawn.** The servo was faulty: a replacement
> moved under a raw bit-bang and under the library on the first try, and
> `PwmOut` was never re-tested against a working one. And a library's choice of
> mechanism is not evidence about a peripheral — software timing supports any
> pin, where hardware PWM is tied to a slice, which is reason enough on its own.
>
> This is worth keeping as a lesson rather than quietly deleting, because it is
> the project's recurring failure mode wearing a new coat: **a faulty actuator
> made working firmware look broken**, and a plausible explanation was then
> built on top of the wrong premise. The defence is the one `servo_smoke/`
> provides — a raw bit-bang that removes all software from the question — and a
> known-good unit swapped in *before* the firmware is blamed.
>
> ```cpp
> #include <Servo.h>
> Servo latch;
> latch.attach(6, 600, 2400);      // no pulses until the first write
> latch.writeMicroseconds(1500);
> latch.detach();                  // stops the pulse train entirely
> ```
>
> **Consequence worth carrying:** the pulse comes from interrupts rather than
> PWM hardware, so it has some jitter and a little ISR cost. Fine for a bench
> latch; re-check it before it shares a core with a 500 Hz flight loop.

**Sketch layout.** The Arduino IDE requires a `.ino` to sit in a folder of the
same name, so each sketch gets its own directory — `rocket_diagnostics/`,
`rocket_flight/`. Opening a bare `.ino` at the repository root prompts the IDE
to relocate it, and two of them at the root cannot both be opened cleanly.

| Device | Library |
|---|---|
| MS5611 | MS5611 by Rob Tillaart |
| LSM6DSO | LSM6 by Pololu |
| LIS3MDL | LIS3MDL by Pololu |
| ADXL375 | Adafruit ADXL375 (+ Adafruit_Sensor, Adafruit_BusIO) |
| GPS | SparkFun u-blox GNSS Arduino Library |
| LoRa | **RadioLib** — RadioHead support on this core is patchy |
| Servo | **Servo** by Arduino — install from Library Manager, *not* bundled. See below |

---

## 10. Firmware architecture

### 10.1 Core-specific constraints

The official Arduino Mbed core differs from the Philhower core in ways that
materially affect this design:

| Concern | Consequence |
|---|---|
| `setup1()` / `loop1()` | **Do not exist.** Mbed runs on core 0 and leaves core 1 idle with no supported launch path. |
| Threading | Replaced with an `osPriorityRealtime` mbed Thread. Preemption substitutes for core isolation. |
| RAM | Mbed OS consumes ~40–60 KB. Ring buffer reduced from 6000 to **4000 samples** (112 KB). |
| `Wire.setSDA` / `SPI.setSCK` | Not available — but the Pico variant already defaults to GP4/GP5 and GP16–19, matching this design exactly. |
| LittleFS | Not bundled. Log dump is over USB serial as CSV. |
| `printf("%f")` | May be built without float support, failing *silently*. Use `Serial.print(value, decimals)` throughout. |
| **Zero-length `Wire.endTransmission()`** | Issues a **read**-type transaction instead of an address write, so most devices do not ACK. Any bare `beginTransmission()`/`endTransmission()` pair — I²C scanners, library `isConnected()` checks — reports devices as absent that are present and working. **Write a dummy byte first.** This is a shared Mbed-core defect, not RP2040-specific; it bit us on the MS5611 (§4.2) and would bite any future device probed the same way. |

**Correct I²C scan on this core:**

```cpp
for (byte a = 1; a < 127; a++) {
  Wire.beginTransmission(a);
  Wire.write(0);                 // ← required; forces a write-type probe
  if (Wire.endTransmission() == 0) { /* device present */ }
}
```

Note the side effect: this writes a zero byte to every address on the bus. For
the MS5611 that is a harmless ADC-read command, but on a device that treats
register 0 as configuration it would not be. Worth re-checking if a new part
joins the bus.

### 10.2 Threading model

```
flightThread   osPriorityRealtime   500 Hz sensor loop, state machine,
                                    RAM buffer.  NEVER blocks.
loop()         osPriorityNormal     LoRa TX, GPS parsing, log dump.
                                    May block freely.
```

Because `flightThread` is realtime priority, the scheduler preempts the radio
the instant a sensor tick is due. This preserves the property that matters: a
blocked `radio.transmit()` cannot delay deployment. If timing jitter later
proves measurable in flight logs, that is the one legitimate reason to
reconsider the Philhower core.

### 10.3 Flight data record

```c
struct __attribute__((packed)) Sample {
  uint32_t t_us;        // 4
  int16_t  hg[3];       // 6   ADXL375 raw
  int16_t  acc[3];      // 6   LSM6DSO accel raw
  int16_t  gyr[3];      // 6   LSM6DSO gyro raw
  int32_t  pressure_pa; // 4   MS5611 compensated
  uint8_t  state;       // 1
  uint8_t  flags;       // 1   bit0 = fresh baro sample
};                      // 28 bytes
```

4000 × 28 B = 112 KB → 8 seconds at 500 Hz, covering the full flight plus
pre-trigger. Declared as a **global static array** — never `malloc`.

**No flash writes during flight.** The entire flight buffers in RAM and dumps
after landing detect. Writing to storage during the 50 g boost is the single
most common way to lose a flight's data.

### 10.4 Timing discipline

Fixed-rate loop gated on `micros()`, with `ThisThread::yield()` between ticks
(mbed's 1 ms tick is too coarse for 500 Hz). No `delay()` anywhere in the
flight path. Every sample timestamped with `micros()` at acquisition, not at
write.

---

## 11. Mechanical integration

**Two-deck sled.** Two 5×7 cm double-sided FR4 protoboards joined by 15–20 mm
M3 **nylon** standoffs (lighter than metal, no accidental shorts).

- **Lower deck (dirty):** Pico, power, switched JST breakout, decoupling,
  servo header, buzzer, reed switch
- **Upper deck (clean):** MS5611, MinIMU-9, ADXL375, GPS (antenna facing sky,
  nothing metallic above), LoRa (antenna routed away from GPS)

This separation solves the RF problem and isolates servo current from sensor
grounds simultaneously.

**FR4 with plated-through holes — not phenolic/bakelite.** Phenolic pads lift
off under 50 g shock. The material difference costs a few pesos.

**Build as a removable sled.** Mount the stack to a foam or printed carrier
that slides into the bay as one unit, rather than fixing boards to the
airframe. Enables battery swaps and debugging in the field, and the foam
provides the shock isolation — a rigidly bolted PCB transmits the full spike.

**Static ports.** 3–4 holes of ~1 mm, evenly spaced at 120°/90°, at least one
body-diameter aft of the nose shoulder, on the *dry* side. Deburr them. Port
quality dominates sensor noise if done badly. Size for lag as well as noise —
keep the bay time constant under ~50 ms. Place the barometer in the bay's
ambient air, **not** directly in line with a port (avoid the incoming jet).
Keep packed parachute fabric clear of the ports.

**Water ingress.** The vehicle lands wet. Conformal-coat the boards (leaving
the barometer port and exposed sensor dies uncovered). Consider hydrophobic
mesh over the static ports — a droplet across a port produces wild readings.

---

## 12. Regulatory compliance

**Operating band: 868.000–870.000 MHz at ≤ +14 dBm (25 mW erp).**

NTC MC 03-06-2017 lists the Philippine non-specific SRD / telemetry
allocations as:

| Band | Limit |
|---|---|
| 868.000–868.600 MHz | 25 mW erp |
| 868.700–869.200 MHz | 25 mW erp |
| 869.300–869.700 MHz | 25 mW erp |
| 869.700–870.000 MHz | 5 mW erp |

**915 MHz was not relied upon.** Whether the AS923-3 allocation (915–918 MHz)
is permitted for this use is actively disputed in the local LoRa community, and
905–915 MHz is adjacent to cellular allocations. The SX1276 tunes 862–1020 MHz,
so band selection is a firmware constant — not a hardware commitment.

```cpp
radio.begin(869.5);
radio.setOutputPower(14);
```

**Duty cycle.** EU-style bands typically carry a 1% limit. At SF7/BW250 an
18-byte packet is ~10 ms airtime, so 20 Hz telemetry is ~20% duty. Either slow
the live stream toward ~1 Hz or accept the trade-off knowingly. The landed
beacon at 1 packet / 5 s is comfortably compliant.

> **Verify before flight.** NTC MC 004-06-2026 (29 June 2026) amended SRD
> parameters. This section reflects understanding as of September 2026 and
> should be re-checked against current NTC circulars rather than taken as
> authoritative.

---

## 13. Test and verification plan

### 13.1 Bring-up order

Non-negotiable sequence. Each step isolates one class of failure. **If an
address does not appear, stop and fix it — do not proceed hoping.**

1. Pico alone — blink GP25 ✔
2. I²C scanner with nothing attached — confirms the bus works ✔
3. Add **MS5611** → expect `0x77`. Breathe on it; pressure should move ✔
4. Add **MinIMU-9** → expect `0x6B` and `0x1E`. Tilt it ✔
5. Add **ADXL375** → expect `0x53`. Tap it ✔ — address in the scan, boot
   decodes `BW_RATE = 0xD` → 800 Hz, taps register (2.51 g peak observed).
   Note the rest reading is offset-dominated, not 1.00 g (§4.4)
6. Add **GPS** → expect `0x42`. Take it outside, wait for fix ✔ — address in
   the scan, boot reports `airborne <1g  OK` at 5 Hz, and a 3D fix has been
   obtained with coordinates confirmed against a map. **Accuracy is not yet
   adequate**: 5 satellites through a window gave a fix that wandered 238 m
   (§4.5). Needs open sky before any figure from it is usable
7. **LoRa** on SPI — test link with the second Pico before integrating
8. **Servo** on its own supply, with the 220 µF cap. Sweep it
9. **Reed switch** — confirm LOW with magnet present
10. Only now: full loop and state machine

### 13.2 Bench diagnostics

A standalone sketch (`rocket_diagnostics/`) reports interpreted
values — °C, hPa, metres AGL, m/s, g, degrees tilt — plus:

- I²C scan (with the §10.1 dummy-byte fix) and per-device pass/fail at boot
- **Sensor ranges read back from the configuration registers and printed**, so
  a wrong range is visible at boot rather than inferred from bad data (§4.3)
- **Gyro bias calibration** at startup (250 samples, held still), re-runnable
  in place with the `b` command as the board warms
- **Running 1σ altitude noise, with a suggested `APOGEE_DROP_M`** — this is the
  single most useful output; it directly sets deployment latency
- Peak trackers for altitude, acceleration, gyro rate
- Rest sanity check (|a| should read 1.00 g; flags >5% deviation)
- Drift check — |rate| at rest should be ≈ 0; prompts a re-calibration if not
- **Saturation flags**, reported separately from range-ceiling warnings, since
  a clipped reading is unknown rather than merely large
- CSV mode for the Arduino Serial Plotter (`c`)
- **Telemetry stream mode (`v`)** emitting the §6.6 wire format for the
  attitude viewer, timestamped with the board's own `millis()`
- **High-g column and peak tracker** from the ADXL375, with its own 13-bit
  saturation ceiling and its own rest check sized against quantisation rather
  than noise (§4.4)
- **GPS status** — fix type, satellite count, coordinates and MSL altitude,
  polled at 1 Hz and printed in the summary block or on demand with `g`.
  Distinguishes *never acquired* from *acquired and lost*
- **Config readback extended to the two new parts**: the ADXL375 data rate
  decoded from `BW_RATE`, and the GPS dynamic model and navigation rate read
  back from the module (§4.4, §4.5)
- **Missing sensors degrade rather than halt.** The high-g column reads `---`
  and the GPS line reads `not fitted` if either part does not answer; only the
  barometer and IMU are fatal, because every other number on the display is
  derived from them

![Serial monitor output at boot: I²C scan listing six addresses, LSM6 control
registers decoded to ±2 g and ±1000 dps, ADXL375 data rate decoded to 800 Hz,
GPS dynamic model confirmed as airborne <1g at 5 Hz, the buzzer and servo latch
reporting silent and safe, gyro bias offsets, ground pressure zeroed at
1007.15 hPa, and the first rows of the live table](docs/images/diagnostics-boot.png)

*Boot output on hardware. Every configuration value on screen was **read back
from the chip**, not echoed from the code that set it — `CTRL1_XL = 0x50`,
`CTRL2_G = 0x58`, `BW_RATE = 0xD`, and the GPS reporting `airborne <1g  OK`.
That is the §4.3 rule doing its job for all three configurable devices at once.
The two actuators announce themselves too: the buzzer silent, and the servo
latch `SAFE at boot, no pulses emitted` — the first interlock of §6.7, visible
before anything else has run.*

> **One anomaly in that scan: `0x7E  (unknown)`.** Nothing in this design lives
> there, and 0x78–0x7F is the I²C reserved range, so a device ACKing there is
> not a device. The likely cause is a marginal bus — four sets of pull-ups in
> parallel on breadboard jumper leads (§4.1) — producing a phantom ACK. Worth
> resolving before it is blamed on something else: the standing rule is that an
> address which does not appear must be fixed, and the inverse deserves the same
> attention.

**Status: all four sensors confirmed on hardware.** The boot output above is
from the assembled stack: every device answers on the bus, and every
configuration value shown was read back from the chip rather than echoed from
the code that set it. Two qualifications carry forward — the ADXL375's rest
reading is offset-dominated and its trim is an open item (§4.4), and the GPS
has a fix but not yet a *usable* one, having been tested only indoors (§4.5).

**Flash and RAM cost of the two new sensors** (`arduino:mbed_rp2040:pico`):
program storage 112,536 → **148,883 bytes** (5% → 7% of 2 MB) and globals
43,996 → **44,608 bytes** (both 16% of 264 KB). Nearly all of the +35.7 KB is
the u-blox library; the RAM cost is negligible. Neither figure threatens the
112 KB flight buffer of §10.3, but the flight firmware should be re-measured
against it rather than assumed.

**Timing constants are tied to the sample rate.** The sketch runs at 25 Hz, and
three constants must be sized against it or they quietly measure the wrong
thing:

| Constant | Value | Why it matters |
|---|---|---|
| Noise window | 100 samples = **4 s** | A shorter window *under*-estimates barometer wander and suggests an `APOGEE_DROP_M` that is too tight |
| Summary interval | 125 samples = **5 s** | Readable pacing for the interpreted block |
| Velocity filter | `0.88 / 0.12` | The same time constant `0.7 / 0.3` gives at 10 Hz |

If the sample rate is ever changed, all three move with it.

**Measurement discipline.** Take the 1σ altitude noise figure with viewer
smoothing **off** (§6.7). Smoothed data reads roughly 4.7× quieter than the
sensor actually is, and `APOGEE_DROP_M` derived from it would trigger on noise.

### 13.3 Verification tests

| Test | Method | Pass criterion |
|---|---|---|
| Static port / apogee logic | Syringe over the port, gentle vacuum | State machine advances IDLE→…→DESCENT correctly |
| Shock survival | Foam-padded drop, waist height | No reset, no I²C dropout |
| Vibration | Vigorous shake, 30 s | No bus errors, no false launch detect |
| GPS dynamic model | Drive with logging active | Fix maintained, plausible track |
| **Latch pull, ×50** | With chute **actually packed** | 100% release. Friction with a compressed chute is far higher than an empty bench test |
| Latch load | Hold vehicle inverted by the upper section | Holds full weight, servo unpowered |
| Battery low-end | Discharge to 3.5 V, actuate latch | Clean release |
| Radio range | Field walk, log RSSI + packet loss | Characterise before trusting it |
| **Ground station endurance** | Pico W on a power bank, phone streaming, 2 h | No dropped stream, no reboot, AP stays up |
| **Viewer under packet loss** | Interrupt the stream repeatedly | Reconnects without reload; charts show gaps, never fabricated data |
| **Attitude return-to-zero** | Level, manoeuvre through large angles, return | Tilt reads < 0.5° residual (§5.5) |
| **Gyro saturation check** | Spin the vehicle by hand at increasing rate | Saturation flag fires before attitude becomes wrong |

### 13.4 Flight test progression

1. **Airframe only**, no avionics — prove the structure survives pressurisation
   and landing
2. **Avionics, logging only** — no deployment. Prove the data before trusting
   the trigger
3. **Full system** — deployment enabled, after offline analysis confirms the
   apogee detector would have fired correctly on the logged flights

---

## 14. Known limitations and open items

**Limitations:**

- **Untested deployment code is the principal risk.** A commercial altimeter
  carries thousands of validated flights; this carries none. This is the
  deliberate trade for SRAD capability, and it is why flight test progression
  (§13.4) is sequenced as it is. Serious teams often fly a commercial
  altimeter as primary deployment *alongside* SRAD logging.
- **Gyro saturates at ±2000 dps** (~333 RPM). A spinning or tumbling vehicle
  will exceed this and the attitude estimate becomes unrecoverable.
- **No onboard sensor fusion yet.** Raw logging only; Kalman fusion is offline
  in Python pending validation across multiple flights. The Mahony PI filter in
  §5.5 runs ground-side in the viewer and has **not** been ported to flight
  firmware — the tilt inhibit is specified but not implemented.
- **Magnetometer is not flight-usable** without in-situ hard/soft-iron
  calibration, and servo current corrupts it during the deployment event.
  Consequence: **yaw is unobservable**, and the roll-about-vertical readout in
  the viewer drifts. No filter tuning fixes this; it needs the magnetometer.
  Tilt is unaffected (§5.5).
- **Preemptive threading, not true core isolation** — see §10.2.
- **Ground station telemetry is synthetic.** The viewer and ground station are
  working end-to-end, but nothing in that path has yet carried a real sensor
  reading. The radio link is the missing piece.
- **Attitude filter numbers are from synthetic tests**, not flight data. The
  §5.5 figures come from injected-bias simulations with known ground truth.
  They establish that the algorithm is correct, not that it survives a 50 g
  boost.
- **Ground station is AP-only** and comfortable with roughly 4 clients. It is a
  team tool, not a spectator server.

**What is built and working:**

- [x] Barometer and IMU on the flight computer; bench diagnostics running
      interpreted output, noise statistics and saturation flags (§13.2)
- [x] ADXL375 and SAM-M8Q on the same I²C bus, driven by the diagnostics
      sketch with per-part saturation ceilings and config readback, and
      **confirmed on hardware** — §13.1 steps 5 and 6 (§4.4, §4.5)
- [x] Telemetry staleness timeout, end to end — the viewer dims and counts a
      gap over 1.5 s, and the ground station stops forwarding frames after 1 s
      of source silence rather than repeating the last one (§6.5, §6.7)
- [x] High-g and GPS carried in the telemetry wire format as appended fields,
      with readouts in the viewer (§6.6)
- [x] Trajectory view — flown path in 3D, altitude measured and horizontal
      from GPS, with unlocked segments drawn as an explicit gap (§6.7)
- [x] Board command buttons in the viewer over Web Serial (§6.7)
- [x] `hAcc` carried in telemetry, with the pad datum gated on satellite count
      and accuracy rather than taken from the first fix (§4.5, §6.7)
- [x] MG90D servo latch on GP6, driven by the diagnostics sketch through the
      Servo library, with a configurable trigger harness in the viewer and
      arm / deadman / fire-latching interlocks (§6.7, §8.1). **Confirmed moving
      on hardware**, under both a raw bit-bang and the Servo library, on VBUS
- [x] LS3040 buzzer on GP7 with non-blocking pattern playback, arming chirp and
      fire confirmation (§6.7)
- [x] Wired UART downlink from the flight computer to the ground station, as a
      stand-in for the radio: validated frames, explicit source selection, no
      synthetic fallback (§6.5)
- [x] Attitude viewer, with dual serial/Wi-Fi transport (§6.7)
- [x] Pico W ground station, serving the viewer over its own Wi-Fi (§6.5)
- [x] Telemetry wire format, shared across both transports (§6.6)
- [x] Attitude filter: Mahony PI with online bias estimation, yaw-immune tilt —
      ground-side only (§5.5)

**Open items:**

- [ ] **Keep VBUS in mind as the servo supply ages or loads up.** It works
      today, but it is USB 5 V behind a Schottky — roughly 4.7 V idle, sagging
      under load — against an MG90D specified from 4.8 V, and a USB port
      current-limits near the servo's ~700 mA stall. Fine on a bench with the
      latch unloaded; re-check before the latch is working against a packed
      chute, and move to its own supply for flight regardless (§7)
- [ ] **Add mechanical confirmation that the latch released** — a limit or
      reed switch on the pin. A servo has no feedback path, so "fired" currently
      means "commanded to fire" and a stalled servo is indistinguishable from a
      healthy one (§8.1)
- [ ] Verify the latch on its **own supply with the 220 µF fitted** — servo
      inrush browning out the Pico would reboot the board holding the actuator
- [ ] **Trim the ADXL375 zero-g offset** — per-axis, against a known
      orientation. It reads 0.73 g at rest against the LSM6's 1.01 g, which is
      within specification for a ±200 g part but leaves the channel unusable
      for absolute magnitude near 1 g (§4.4)
- [ ] **Take the GPS somewhere with open sky.** It holds a 3D fix and the
      coordinates are right, but on 5 satellites indoors the position wandered
      238 m. Expect 10–15 satellites and single-digit metres outdoors (§4.5)
- [ ] **Resolve the phantom `0x7E` in the I²C scan** — nothing lives there and
      0x78–0x7F is reserved, so it points at a marginal bus rather than a
      device. Four sets of pull-ups in parallel on breadboard leads is the
      prime suspect (§4.1)
- [ ] Paired LoRa TX/RX test sketches with RSSI + packet-loss logging
- [ ] **Wire the radio into the ground station.** The producer slot is now
      proven end to end by the wired downlink; what remains is a LoRa driver on
      **SPI** (the RFM95W has no UART) that decodes the §6.2 packet into a §6.6
      line and hands it to the same `accept_line()` (§6.5)
- [ ] Build flight antenna (82 mm); source SMA edge-mount connector + 868 MHz
      whip for ground station
- [ ] Extend LSM6DSO to flight ranges (±16 g, ±2000 dps) via register writes,
      confirming via the §4.3 readback that the ranges actually took
- [ ] **Port the §5.5 attitude filter into flight firmware** and implement the
      tilt inhibit that §5.4 specifies
- [ ] Re-measure 1σ altitude noise with smoothing off, and set
      `APOGEE_DROP_M` from it (§13.2)
- [ ] **Write `rocket_flight/rocket_flight.ino`** — the flight firmware
      skeleton §15 lists. The state machine (§5.4), threading model (§10.2)
      and flight record (§10.3) are specified; nothing implements them yet
- [ ] Full state-machine integration test (syringe method)
- [ ] Collar joint CAD with real tolerances (Fusion 360); FEA the pin/collar
      under 478 N
- [ ] Latch fabrication and ×50 pull test
- [ ] Re-verify NTC SRD circulars before first RF-active flight
- [ ] **Set the ground station's Wi-Fi password — this is no longer cosmetic.**
      Joining the network now means being able to arm and fire the latch, and
      the default is published in this repository. Put the real one in
      `station_secret.py` on the board (git-ignored); the station warns at boot
      and in `/health` while it is still on the default

---

## 15. Related documents

| Document | Contents |
|---|---|
| `water_rocket_avionics_bom.xlsx` | Full bill of materials, costs, suppliers, phasing, per-part justification |
| `airframe_build_spec.md` | Airframe structure, materials, dimensions, assembly sequence |
| `hardware_reference.md` | Quick bench reference — pin map, addresses, per-sensor driver notes, bring-up order. Kept in sync with this document; **this document is the authority** where the two disagree |
| `tools/checks/` | Headless checks for the ground station telemetry model and all three viewer builds. No hardware; `python tools/checks/run_checks.py` |
| `rocket_diagnostics/` | Bench diagnostics sketch (verified working on hardware) |
| `servo_smoke/` | Minimal servo sweep on GP6. No sensors, no logic — isolates a stationary latch as firmware versus wiring and power |
| `rocket_flight/` | Flight firmware skeleton — state machine, threading, telemetry. **Not written** (§14) |
| `rocket_attitude_viewer.html` | **Live 3D attitude viewer.** Self-contained; Web Serial over USB or SSE over Wi-Fi (§6.7) |
| `rocket_attitude_viewer_serial.html` | Serial-only build of the viewer. The bench path that works *today*, driving the viewer straight off the flight computer over USB while the flight-to-ground radio does not yet exist (§6.7) |
| `pico_w_ground_station/main.py` | **Ground station firmware.** MicroPython: Wi-Fi AP, web server, SSE telemetry (§6.5) |
| `pico_w_ground_station/index.html` | Viewer as deployed to the ground station |
| `pico_w_ground_station/README.md` | Ground station setup, endpoints, radio integration notes |
| `rocket_assembly_viewer.jsx` | Interactive 3D assembly and separation-sequence visualiser |
| `avionics_status_report.md` | Project status summary |

**External references:**

- OpenRocket — flight simulation and stability analysis
- NTC Memorandum Circulars on short-range devices — `ntc.gov.ph`

---

*v0.3.1 — September 2026. This document reflects design intent and analysis,
much of it first-order rather than validated. Numbers marked as estimates
should be confirmed by test or FEA before they are relied upon for flight
safety.*

*The flight-critical path is unproven: the deployment state machine is unflown,
the tilt inhibit is unimplemented, and no part of the telemetry chain has
carried a real sensor reading over the air. Treat the §5.5 filter figures as
evidence the algorithm is correct, not as evidence it survives a 50 g boost.*

*Five failure modes on this platform are worth holding onto as a class, because
every one of them is silent. The MS5611 can report itself absent while working
perfectly (§4.2). The gyro can report every rate 4× high while looking merely
"unstable" (§4.3). The GPS can hold a flawless fix on the ground and drop it at
launch, because a `setDynamicModel()` that failed is indistinguishable from one
that worked until you are airborne (§4.5). And the ADXL375 saturates at 13-bit
while sitting in an `int16`, so saturation logic sized to the integer width
never fires and a clipped boost trace is reported as a measurement (§4.4). And
a GPS holding a confident `3D` fix on five satellites can sit 238 m from where
it started without moving, because fix type says nothing about accuracy
(§4.5).*

*None of them announces itself as a configuration error. The structural response
— read configuration back from the hardware and print it at boot, and size every
limit against the part rather than against its data type — is cheap, and is the
standing expectation for any new device on this bus.*
