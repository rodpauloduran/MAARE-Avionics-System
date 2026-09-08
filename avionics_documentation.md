# Water Rocket Avionics System — Design Documentation

**v0.1 · September 2026**
Mapúa University (Intramuros)

> **What is built.** The barometer and IMU are on the flight computer and the
> bench diagnostics sketch works (§13.2). The ground segment — a 3D attitude
> viewer (§6.7) and a Pico W ground station serving it over its own Wi-Fi
> (§6.5) — works on synthetic telemetry. Nothing else is connected: no high-g
> accelerometer, GPS, radio link, deployment hardware or airframe. This
> document specifies the whole system; treat anything outside §13.2 and §6.5
> as design intent rather than description.

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
   Rocket ──LoRa──▶ [RFM95W] ──UART──▶ Pico W ──Wi-Fi AP──▶ phone browser
                                        │                     (viewer)
                                        └─ serves index.html + SSE stream
```

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
V,ax,ay,az,gx,gy,gz,alt,vel[,millis]
```

Accelerations in g, rates in dps, altitude in metres AGL, velocity in m/s. The
tenth field is the **flight computer's own `millis()`** and is optional for
backward compatibility.

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
| I²C0 SDA | GP4 | 6 | All sensors |
| I²C0 SCL | GP5 | 7 | 400 kHz |
| SPI0 MISO | GP16 | 21 | RFM95W |
| SPI0 CS | GP17 | 22 | RFM95W |
| SPI0 SCK | GP18 | 24 | RFM95W |
| SPI0 MOSI | GP19 | 25 | RFM95W |
| LoRa RESET | GP20 | 26 | Tie high if pins tight |
| LoRa DIO0 | GP21 | 27 | Or poll registers |
| Servo PWM | GP15 | 20 | 50 Hz, separate rail |
| Buzzer | GP13 | 17 | Transistor if 5 V |
| Reed (arming) | GP12 | 16 | Pull-up, switch to GND |
| Battery sense | GP26 | 31 | ADC0 via 100k/100k |
| Status LED | GP25 | — | Onboard |
| VSYS (batt in) | — | 39 | 1.8–5.5 V |
| GND | — | 38 | Common with servo |

Free: GP0–GP3, GP6–GP11, GP14, GP22, GP27, GP28 — ample margin for a nichrome
backup channel, second deployment event, or an OLED.

### 9.2 Toolchain

**Official Arduino Mbed OS RP2040 core** (not the Earle Philhower community
core). This has non-obvious consequences — see §10.1.

| Device | Library |
|---|---|
| MS5611 | MS5611 by Rob Tillaart |
| LSM6DSO | LSM6 by Pololu |
| LIS3MDL | LIS3MDL by Pololu |
| ADXL375 | Adafruit ADXL375 (+ Adafruit_Sensor, Adafruit_BusIO) |
| GPS | SparkFun u-blox GNSS Arduino Library |
| LoRa | **RadioLib** — RadioHead support on this core is patchy |
| Servo | Bundled with core |

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

1. Pico alone — blink GP25
2. I²C scanner with nothing attached — confirms the bus works
3. Add **MS5611** → expect `0x77`. Breathe on it; pressure should move
4. Add **MinIMU-9** → expect `0x6B` and `0x1E`. Tilt it
5. Add **ADXL375** → expect `0x53`. Tap it
6. Add **GPS** → expect `0x42`. Take it outside, wait for fix
7. **LoRa** on SPI — test link with the second Pico before integrating
8. **Servo** on its own supply, with the 220 µF cap. Sweep it
9. **Reed switch** — confirm LOW with magnet present
10. Only now: full loop and state machine

### 13.2 Bench diagnostics

A standalone sketch (`rocket_diagnostics.ino`) reports interpreted
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

**Status: verified working.**

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
- [x] Attitude viewer, with dual serial/Wi-Fi transport (§6.7)
- [x] Pico W ground station, serving the viewer over its own Wi-Fi (§6.5)
- [x] Telemetry wire format, shared across both transports (§6.6)
- [x] Attitude filter: Mahony PI with online bias estimation, yaw-immune tilt —
      ground-side only (§5.5)

**Open items:**

- [ ] ADXL375 bring-up and diagnostic integration
- [ ] Paired LoRa TX/RX test sketches with RSSI + packet-loss logging
- [ ] **Wire the radio into the ground station** — replace the synthetic
      producer task with a UART packet reader. Architecture already supports
      it; no viewer changes needed (§6.5)
- [ ] **Add a telemetry staleness timeout.** "Radio silent" and "vehicle
      sitting perfectly still" currently look identical on the display. This is
      a safety-relevant display defect, not a nicety
- [ ] Build flight antenna (82 mm); source SMA edge-mount connector + 868 MHz
      whip for ground station
- [ ] Extend LSM6DSO to flight ranges (±16 g, ±2000 dps) via register writes,
      confirming via the §4.3 readback that the ranges actually took
- [ ] **Port the §5.5 attitude filter into flight firmware** and implement the
      tilt inhibit that §5.4 specifies
- [ ] Re-measure 1σ altitude noise with smoothing off, and set
      `APOGEE_DROP_M` from it (§13.2)
- [ ] **Write `rocket_flight.ino`** — the flight firmware skeleton §15 lists.
      The state machine (§5.4), threading model (§10.2) and flight record
      (§10.3) are specified; nothing implements them yet
- [ ] Full state-machine integration test (syringe method)
- [ ] Collar joint CAD with real tolerances (Fusion 360); FEA the pin/collar
      under 478 N
- [ ] Latch fabrication and ×50 pull test
- [ ] Re-verify NTC SRD circulars before first RF-active flight
- [ ] Change the ground station Wi-Fi password before any public demonstration
      — it is currently a default in plain text in the source

---

## 15. Related documents

| Document | Contents |
|---|---|
| `water_rocket_avionics_bom.xlsx` | Full bill of materials, costs, suppliers, phasing, per-part justification |
| `airframe_build_spec.md` | Airframe structure, materials, dimensions, assembly sequence |
| `hardware_reference.md` | Quick bench reference — pin map, addresses, per-sensor driver notes, bring-up order. Kept in sync with this document; **this document is the authority** where the two disagree |
| `rocket_diagnostics.ino` | Bench diagnostics sketch (verified working on hardware) |
| `rocket_flight.ino` | Flight firmware skeleton — state machine, threading, telemetry. **Not written** (§14) |
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

*v0.1 — September 2026. This document reflects design intent and analysis,
much of it first-order rather than validated. Numbers marked as estimates
should be confirmed by test or FEA before they are relied upon for flight
safety.*

*The flight-critical path is unproven: the deployment state machine is unflown,
the tilt inhibit is unimplemented, and no part of the telemetry chain has
carried a real sensor reading over the air. Treat the §5.5 filter figures as
evidence the algorithm is correct, not as evidence it survives a 50 g boost.*

*Two failure modes on this platform are worth holding onto as a class, because
both are silent. The MS5611 can report itself absent while working perfectly
(§4.2), and the gyro can report every rate 4× high while looking merely
"unstable" (§4.3). Neither announces itself as a configuration error. The
structural response — read configuration back from the hardware and print it at
boot — is cheap, and is the standing expectation for any new device on this
bus.*
