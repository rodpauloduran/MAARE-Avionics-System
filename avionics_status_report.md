# Avionics Status Report

**Water pressure rocket avionics — technical status**
September 2026 · aligned with [`avionics_documentation.md`](avionics_documentation.md) Revision B
*Supersedes the August 2026 report.*

---

## 0. Build status at a glance

The design in this project is deliberately ahead of the hardware. As of this
report, **two sensors are on the flight computer** and the ground segment runs
on synthetic data. Everything else is specified, not built.

| Item | State |
|---|---|
| MS5611 barometer | **Wired to the flight Pico, reading** |
| MinIMU-9 v6 board | **Wired.** LSM6DSO accel/gyro reading; LIS3MDL magnetometer not read by the current sketch |
| `rocket_diagnostics.ino` (rev B) | **Working on hardware** |
| Pico W ground station + viewer | **Working** — synthetic telemetry only |
| ADXL375 high-g | Not connected |
| SAM-M8Q GPS | Not connected |
| RFM95W LoRa ×2 | Not connected — no radio link exists |
| MG90D servo, latch, deployment | Not connected, not built |
| Reed switch arming interlock | Not connected |
| LS3040 buzzer | Not connected |
| Airframe | Not built |
| `rocket_flight.ino` | **Not written** |

Sections 2 and 3 below describe the *intended* architecture. Read them as
specification.

---

## 1. Project objective

Build and fly a self-designed reusable avionics package (SRAD flight computer)
on a water pressure rocket as a practice platform. The purpose is twofold:

1. Build hands-on avionics competency ahead of a longer-term goal of competing
   in IREC (Spaceport America Cup), targeting a 2027 learning-year application
   and a serious 2028 entry.
2. Generate a track record to support formation of an aerospace/rocketry
   student organization at Mapúa Intramuros.

The water rocket carries no propellant hazard, so it is being used as a
low-risk testbed for the sensing, deployment, telemetry, and recovery systems
that would later transfer to high-power motors.

---

## 2. Avionics architecture — summary

> Design intent. See [§0](#0-build-status-at-a-glance) for what is actually
> connected.

### Compute

| Role | Part |
|---|---|
| Flight computer | Raspberry Pi Pico (RP2040), official Arduino Mbed OS RP2040 core |
| Ground station | Raspberry Pi Pico W running **MicroPython** — *not* the Arduino core used on the flight computer |

The ground station brings up its own Wi-Fi access point and serves the attitude
viewer to any phone at the pad — no router, no internet, no app install. The two
devices are independent, so the toolchains do not need to match.

### Sensors (all I²C, shared two-wire bus)

| Role | Part | Addr | Notes |
|---|---|---|---|
| Barometer | MS5611 (GY-63 breakout) | `0x77` | Primary altimeter and apogee trigger |
| IMU | Pololu MinIMU-9 v6 — LSM6DSO | `0x6B` | Coast-phase acceleration, attitude (gyro) |
| Magnetometer | …+ LIS3MDL, same board | `0x1E` | Heading; logged but not flight-critical |
| High-g accelerometer | Adafruit ADXL375, ±200 g | `0x53` | Captures boost (20–100 g) without clipping; launch detection |
| GPS | SparkFun SAM-M8Q (u-blox) | `0x42` | Multi-GNSS incl. QZSS. Recovery beacon, **not** a flight-phase sensor — loses lock at launch, reacquires in 5–15 s |

### Radio

2× Adafruit RFM95W LoRa transceiver (SX1276), one flight unit and one ground
station. Frequency set to **868–870 MHz** per the Philippine NTC SRD
allocation — *not* the US 915 MHz default. TX power capped at **+14 dBm
(25 mW erp)** to stay within the licensed limit.

The duty-cycle constraint (~1%) is the practical limit on sustained live
telemetry: the design document allows 10–20 Hz in flight if the trade-off is
taken knowingly, but a sustained-legal rate is closer to ~1 Hz. The landed
beacon at 1 packet / 5 s is comfortably within limits.

### Recovery

MG90D metal-gear servo (180° positional) actuates a pin latch releasing the
nose cone under spring/rubber-band ejection. LS3040 passive piezo buzzer
(4 kHz) for audible ground recovery and pad-side status/arming confirmation.

### Safety

**Reed switch + neodymium magnet** — physical arming interlock, sealed inside
the airframe and magnet-actuated from outside. Deployment logic is inert until
armed, closing the failure path where ground-handling vibration could trigger
the latch.

**JST-PH switched connector** (Adafruit 1863) — combined battery connector and
power switch, avoiding a separate panel-mount switch and a hole in the
airframe.

### Power

PKCell 500 mAh 3.7 V LiPo with integrated protection circuitry, fed directly to
Pico VSYS (no external regulator required). Estimated 4–5 hour pad endurance
against ~90 mA average system draw. Charged via Adafruit Micro-LiPo (MCP73831)
set to 500 mA (1 C), matched via genuine JST-PH connector.

---

## 3. Firmware status

Toolchain confirmed as the **official Arduino Mbed OS RP2040 core**, not the
Earle Philhower community core. This changes several architectural assumptions
carried in earlier drafts:

- **No `setup1()`/`loop1()` dual-core API.** Replaced with a single
  high-priority (`osPriorityRealtime`) RTOS thread for the 500 Hz
  sensor/state-machine loop, running alongside a normal-priority `loop()` for
  radio and GPS. Preemption substitutes for true core isolation.
- **RAM budget reduced** from ~168 KB to ~112 KB for the flight data ring
  buffer, to leave headroom for Mbed OS overhead.
- **RadioHead replaced with RadioLib** for LoRa — better support on this core.
- **No bundled LittleFS.** Flight logs dump over USB serial as CSV
  post-recovery rather than to onboard flash.
- **Default pin mapping already matches** the intended hardware layout (Wire on
  GP4/GP5, SPI0 on GP16–19) with no manual remapping.
- **Avoiding `snprintf("%f", …)`** throughout; this core's `printf` may be built
  without float support and fails *silently*. All numeric output uses
  `Serial.print(value, decimals)`.

State machine, unchanged from design:

```
IDLE -> ARMED -> BOOST -> COAST -> DESCENT -> LANDED
```

Apogee detection is barometric peak-and-drop (primary) with a timer backup and
an IMU tilt inhibit — inhibit only, never a trigger. Deployment is locked out
until burnout + 500 ms regardless of source.

> **None of this is implemented.** `rocket_flight.ino` has not been written.
> The state machine, threading model and flight record are specified in the
> design document (§5.4, §10.2, §10.3); nothing implements them yet.

---

## 4. Bench diagnostics — status: working

[`rocket_diagnostics.ino`](rocket_diagnostics.ino) (rev B) is confirmed working
on the bench, driving the MS5611 and the LSM6DSO. It reports:

- I²C scan and per-device pass/fail at boot, using the dummy-byte probe
  required on the Mbed core
- Temperature, pressure, altitude (delta from zeroed pad pressure), vertical
  velocity
- **Running 1σ altitude noise, with a suggested `APOGEE_DROP_M`** — the single
  most useful output, since it sets deployment latency directly
- **Sensor ranges read back from the config registers and printed** at boot, so
  a wrong range is visible rather than inferred
- Gyro bias calibration at startup (250 samples), re-runnable in place with `b`
  as the board warms
- Accelerometer XYZ, magnitude, tilt-from-vertical
- Peak trackers for altitude, accel, and gyro rate
- Sanity checks — rest reading should be 1.00 g; drift check on gyro rate at
  rest; warns near sensor range ceilings
- **Saturation flags**, reported separately from range-ceiling warnings, since
  a clipped reading is *unknown* rather than merely large
- CSV output mode for the Arduino Serial Plotter (`c`)
- Telemetry stream mode (`v`) feeding the attitude viewer

### Two defects found and fixed during rev A bring-up

Both were silent, which is why they are recorded here as a class rather than as
incidents.

**1. MS5611 reported absent while working perfectly.** `MS5611.begin()` calls
`isConnected()`, which does a zero-length I²C `endTransmission()`; on Mbed cores
that issues a *read*-type transaction the sensor will not ACK. The library's
workaround is gated behind an nRF52840-only macro, so it never compiles in on
RP2040. Fixed by calling `reset()` instead of `begin()`. Left unfixed,
calibration never loads and every reading is a fixed 44307.70 m.

**2. Every gyro rate reported 4× too high.** Rev A paired `enableDefault()` with
a `0.035` dps/LSB constant — the ±1000 dps figure — but `enableDefault()`
actually selects ±245 dps, whose sensitivity is `0.00875`. Slow motion looked
correct, because gravity dominates the attitude estimate at low rates; fast
motion overshot and oscillated. Nothing in the output said "wrong scale
factor," it said "unstable filter." Fixed structurally rather than by
correcting the constant: the sketch now writes the config registers explicitly
and *derives* both scale factors from what the chip reports back.

The standing response to both: **read configuration back from the hardware and
print it at boot.** That is cheap, and is now the expectation for any new device
on this bus.

Bench ranges are currently ±2 g and ±1000 dps at 208 Hz. The ±2 g is
deliberate — it is the right choice for confirming a clean 1.00 g at rest, and
it is the first thing you will saturate, so you find out early rather than in
the air. Flight ranges (±16 g, ±2000 dps) remain an open item.

**Next in progress:** ADXL375 high-g bring-up, and paired LoRa TX/RX test
sketches with RSSI and packet-loss logging for a field range walk.

---

## 5. Ground segment — status: built and working

New since the August report. Both halves exist on hardware and run end-to-end.

> **Telemetry source is synthetic.** The radio link between the two Picos is not
> wired in, and nothing in this path has carried a real sensor reading over the
> air.

### Attitude viewer

[`rocket_attitude_viewer.html`](rocket_attitude_viewer.html) — a single
self-contained HTML file rendering live vehicle attitude as a 3D model with a
tilt protractor, numeric readouts, and strip charts for altitude, |a| and tilt.

**Dual transport from one codebase.** Served over HTTP it consumes Server-Sent
Events from the ground station; opened as a local file it uses Web Serial
directly against the flight computer over USB. Mode is detected from
`location.protocol`. This split exists because **phones have no Web Serial at
all** — which is exactly why the ground station serves the page rather than
asking the phone to open a port.

A serial-only build is kept alongside as
[`rocket_attitude_viewer_serial.html`](rocket_attitude_viewer_serial.html). It
is the bench path that works *today*, driving the viewer straight off the flight
computer over USB while the radio does not yet exist.

### Ground station

[`pico_w_ground_station/`](pico_w_ground_station/) — Pico W in MicroPython. Own
Wi-Fi AP, serves the viewer, streams telemetry over SSE at 25 Hz. Endpoints:
`/` (viewer), `/stream` (SSE), `/health` (JSON), `/mode` (switch synthetic
source).

**Architecture is one producer, many consumers.** The first version advanced the
simulation inside each client's stream loop, which made it run at 2× with two
phones connected and caused the two viewers to disagree. Verified after the fix:
three simultaneous clients each receive 25.0 Hz of byte-identical frames while
the source advances at 1×. That structure is also the correct shape for the
radio — the packet receiver becomes the producer and nothing else changes.

Two settings turned out not to be optional, and both present as intermittent
stutter:

- **Wi-Fi power management disabled.** The CYW43 parks itself between packets,
  which is fine for request/response and visible on a continuous stream.
- **Nagle's algorithm disabled** on the stream socket. 60-byte frames plus a
  phone's ~200 ms delayed ACK produce exactly the multi-frame stall it was
  reported as.

### Attitude filter

Upgraded to a Mahony **PI** complementary filter with online bias estimation. A
proportional-only filter settles at a permanent tilt offset of `bias / K_P` —
0.417° per dps of residual bias at `K_P = 2.4`. That matters because the planned
tilt inhibit is a threshold comparison, so a fixed offset biases the inhibit
angle directly. The integral term reduces the error by roughly 56×.

Tilt is derived from **gravity direction**, not quaternion composition. Without
a magnetometer contribution yaw is unobservable and drifts, and composing
against a stored reference lets that drift leak into the tilt number — 37.84°
reported versus 0.24°, on a spin-and-return test where truth is 0°.

> These figures come from synthetic-signal tests with known ground truth, not
> flight data. They establish that the algorithm is correct, not that it
> survives a 50 g boost. The filter runs **ground-side only** and has not been
> ported to flight firmware; the tilt inhibit is specified but not implemented.

---

## 6. Procurement status

Full bill of materials tracked in
[`water_rocket_avionics_bom.xlsx`](water_rocket_avionics_bom.xlsx) across two
suppliers, Circuit Rocks and Makerlab PH, both Manila-based. Current total is
approximately **PHP 13,600** across compute, sensors, radio, recovery, safety,
power, and prototyping consumables. Antennas for both LoRa units are the one
remaining unresolved line item.

**Phase 1** (minimum viable flight computer: Pico, MS5611, servo, buzzer, arming
switch, battery/charger, protoboard) is isolated from **Phase 2** (GPS, full
IMU, high-g accelerometer, LoRa pair) in the tracking sheet, on the reasoning
that early flights should validate airframe and recovery before exposing the
more expensive sensors to landing risk.

Remaining gaps: soldering tools (iron, multimeter, logic analyzer), mechanical
hardware (standoffs, foam potting, latch pin stock), and antennas.

---

## 7. Antennas — open item

**Flight unit: hand-built, not purchased.** 82 mm solid-core wire quarter-wave
monopole soldered directly to the RFM95W ANT pad, tuned for 868 MHz. Zero cost,
minimal mass, standard practice for small HPR/model rocket telemetry.

**Ground station: to be purchased.** Requirement is an SMA (or U.FL with SMA
pigtail) antenna specifically rated for 868 MHz LoRa/ISM use.

Several candidate purchases were evaluated and **rejected**: GSM/cellular-band
antennas (900–1800 MHz, 824–960/1710–1990 MHz, and 780–960/1710–2170 MHz
variants) do not present a correct 50 Ω match at 868 MHz and would degrade both
transmit efficiency and receive sensitivity. The correct search term is
"868 MHz LoRa antenna", not GSM/cellular/FONA-branded parts.

---

## 8. Regulatory status

**LoRa telemetry.** Operating band set to 868.000–870.000 MHz per NTC
MC 03-06-2017 (Philippine non-specific SRD/telemetry allocation), at or below
+14 dBm / 25 mW erp. Whether the alternative 915–918 MHz AS923-3 allocation is
separately permitted is disputed in the local LoRa community and was not relied
upon.

> NTC MC 004-06-2026 amended SRD parameters again in June 2026. **Re-check for
> further updates before any RF-active flight.**

**Longer-term (sugar/HPR motor work): legal pathway not yet secured.** Requires
either institutional (university) chemical procurement authority or a licensed
organizational body for oxidizer possession (RA 9516, PNP-FEO), plus CAAP
airspace coordination for any flight exceeding trivial altitude. This is
understood to be the binding constraint on the group's longer-term roadmap, and
is being pursued in parallel via outreach to Ateneo de Davao (an existing
IREC-flying Philippine team) and the PCSRC competition (PhilSA / Indiana
Aerospace University) as lower-barrier domestic flight-experience pathways.

---

## 9. Next steps

**Firmware**

- [ ] **Write `rocket_flight.ino`** — the flight firmware skeleton. State
      machine, threading model and flight record are specified; nothing
      implements them yet
- [ ] Port the ground-side attitude filter into flight firmware and implement
      the tilt inhibit the state machine specifies
- [ ] Extend LSM6DSO configuration to flight ranges (±16 g, ±2000 dps) via
      direct register writes, confirming via readback that the ranges took

**Sensors and radio**

- [ ] ADXL375 bring-up and diagnostic integration
- [ ] Paired LoRa TX/RX test with RSSI + packet-loss logging
- [ ] Wire the radio into the ground station — replace the synthetic producer
      task with a UART packet reader. The architecture already supports it; no
      viewer changes needed
- [ ] Build and tune flight-unit antenna (82 mm, 868 MHz)
- [ ] Source and fit ground-station 868 MHz SMA antenna

**Ground segment**

- [ ] Add a telemetry staleness timeout. "Radio silent" and "vehicle sitting
      perfectly still" currently look identical on the display — a
      safety-relevant display defect, not a nicety
- [ ] Set the real AP password on the ground station hardware; the value in
      this repository is a placeholder default

**Measurement and test**

- [ ] Re-measure 1σ altitude noise with viewer smoothing **off** and set
      `APOGEE_DROP_M` from it — smoothed data reads ~4.7× quieter than the
      sensor is, which would set the threshold far too tight
- [ ] Full state-machine integration test on bench (syringe test for
      baro-triggered deployment)

**Mechanical**

- [ ] Collar joint CAD with real tolerances; FEA the pin/collar under 478 N
- [ ] Latch mechanism fabrication and 50× pull test under packed-chute load
- [ ] Reconcile the separation-spring forward mass (0.18 kg) against the
      airframe dry-mass target (150–200 g) — the two do not currently agree

**Flight**

- [ ] First flight: logging only, no deployment

**Organisational**

- [ ] Contact Ateneo de Davao and PhilSA re: IREC/PCSRC pathways
- [ ] Register interest for PCSRC 2027
