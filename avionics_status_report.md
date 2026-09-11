# Avionics Status Report

**Water pressure rocket avionics — build status**
v0.3 · September 2026

Where the build has actually got to, subsystem by subsystem. The design is
specified in [`avionics_documentation.md`](avionics_documentation.md); this
document does not repeat it, it says how much of it exists.

---

## 1. Build status at a glance

**All four I²C sensors are now on the flight computer, and the ground segment
runs on synthetic data.** Everything else is specified, not built.

| Item | State |
|---|---|
| MS5611 barometer | **Wired to the flight Pico, reading** |
| MinIMU-9 v6 board | **Wired.** LSM6DSO accel/gyro reading; LIS3MDL magnetometer not read by the current sketch |
| ADXL375 high-g | **Wired and reading.** Zero-g offset untrimmed — 0.73 g at rest (§4.4) |
| SAM-M8Q GPS | **Wired, configured, 3D fix obtained.** Indoor sky view only; accuracy not yet usable |
| Bench diagnostics | **Working on hardware** for all four sensors |
| Pico W ground station + viewer | **Working** — synthetic telemetry only |
| MG90D servo latch | **Wired to GP6, driven by the diagnostics sketch.** Bench harness only |
| RFM95W LoRa ×2 | Not connected — no radio link exists |
| Latch mechanism, collar joint | Not built — the servo drives nothing yet |
| Reed switch arming interlock | Not connected |
| LS3040 buzzer | Not connected |
| Airframe | Not built |
| `rocket_flight.ino` | **Not written** |

---

## 2. Flight computer

**Toolchain:** official Arduino Mbed OS RP2040 core, not the Earle Philhower
community core. That choice has consequences the design document treats in
§10.1 — no `setup1()`/`loop1()`, no bundled LittleFS, a reduced RAM budget, and
a zero-length-`endTransmission()` defect that affects every I²C probe.

![Diagnostics boot output showing the I²C scan, decoded sensor configuration
registers, GPS dynamic model confirmation, the buzzer and servo latch reporting
silent and safe, gyro bias and ground pressure zeroing](docs/images/diagnostics-boot.png)

**What runs today:** [`rocket_diagnostics/`](rocket_diagnostics/), a
bench sketch driving the MS5611, LSM6DSO, ADXL375 and SAM-M8Q. It reports
interpreted values — °C, hPa, metres AGL, m/s, g, degrees tilt, and now
high-g magnitude and GPS fix state — plus:

- I²C scan and per-device pass/fail at boot, using the dummy-byte probe the
  Mbed core requires
- **Running 1σ altitude noise with a suggested `APOGEE_DROP_M`** — the single
  most useful output, since it sets deployment latency directly
- **Sensor ranges read back from the config registers and printed at boot**, so
  a wrong range is visible rather than inferred from bad data
- Gyro bias calibration at startup, re-runnable in place with `b`
- Peak trackers, rest sanity check, drift check
- **Saturation flags**, separate from range-ceiling warnings, since a clipped
  reading is *unknown* rather than merely large
- CSV mode for the Serial Plotter (`c`), and telemetry stream mode (`v`)
  feeding the attitude viewer
- **High-g column and peak tracker** from the ADXL375, with its own 13-bit
  saturation ceiling — the LSM6's `int16` threshold would never fire on it
- **GPS status** (`g`, and in the 5-second summary): fix type, satellite count,
  coordinates, MSL altitude, and *never acquired* distinguished from *acquired
  and lost*
- **Config readback extended to both new parts** — the ADXL375 data rate
  decoded from `BW_RATE`, the GPS dynamic model and nav rate read back from the
  module. Same rule that already covers the IMU
- **Graceful degradation:** a missing high-g reads `---` and a missing GPS reads
  `not fitted`. Only the barometer and IMU halt the sketch, because every other
  displayed number derives from them

Bench ranges are ±2 g and ±1000 dps at 208 Hz. The ±2 g is deliberate: it is
the right choice for confirming a clean 1.00 g at rest, and it is the first
thing to saturate, so range problems surface early rather than in the air. With
the ADXL375 now fitted, the ±2 g ceiling is also less costly — the high-g part
covers everything above it. Flight ranges (±16 g, ±2000 dps) remain an open
item.

**Confirmed on hardware, with two qualifications.** All four sensors answer on
the bus and every configurable one reports its settings back at boot. The
ADXL375's zero-g offset is untrimmed, so its absolute magnitude near 1 g is not
usable (peaks and events, its actual job, are unaffected). The GPS holds a 3D
fix with coordinates confirmed against a map, but has only been given an indoor
sky view — on 5 satellites it wandered 238 m, which is why the viewer now
refuses to take a pad datum from a fix like that.

**Cost of the two new sensors** (`arduino:mbed_rp2040:pico`): flash
112,536 → **148,883 bytes** (5% → 7%), globals 43,996 → **44,608 bytes**
(16% either way). Almost all of the +35.7 KB is the u-blox library.

**What does not exist:** the flight firmware itself. The state machine,
threading model and flight record are specified (§5.4, §10.2, §10.3); nothing
implements them. No deployment code has been written or flown.

---

## 3. Ground segment

Both halves exist on hardware and run end-to-end.

> **Telemetry is synthetic.** The radio link between the two Picos is not wired
> in, and nothing in this path has carried a real sensor reading over the air.

**Attitude viewer** — a single self-contained HTML file rendering live attitude
as a 3D model with a tilt protractor, numeric readouts, and strip charts.
Dual transport from one codebase: Server-Sent Events when served over HTTP,
Web Serial when opened as a local file. Phones have no Web Serial at all, which
is why the ground station serves the page rather than asking the phone to open
a port.

A serial-only build is kept as `rocket_attitude_viewer_serial.html`. It is the
bench path that works today — with no radio, driving the viewer straight off the
flight computer over USB is the only route carrying real sensor data.

**The viewer now shows high-g and GPS**, carried as five appended fields on
the §6.6 wire format (`hg,fix,sats,lat,lon`), plus a **Trajectory** tab that
draws the flown path in 3D. Only altitude is measured on that path; horizontal
comes from the GPS, which is blind through the flight, so unlocked segments are
drawn as an explicit dashed gap with the horizontal frozen rather than
interpolated. Board commands (`z b r g`) are available as buttons over Web
Serial — not over Wi-Fi, because SSE is one-way and there is no back-channel.

**Staleness is now displayed.** Previously a frozen readout at full contrast was
indistinguishable from a vehicle sitting perfectly still. A gap of more than
1.5 s (≈37 missed frames at 25 Hz) now turns the status dot amber, counts the
age of the last frame, and dims the readouts. It runs independently of each
transport's own error handling, because neither catches the case that matters:
an SSE connection stays open while the ground station has nothing to forward,
and a quiet serial port raises nothing. Applied to all three viewer builds.

**Ground station** — Pico W in MicroPython, own Wi-Fi AP, streaming telemetry
over SSE at 25 Hz. Verified on hardware: three simultaneous clients each
receive 25.0 Hz of byte-identical frames while the source advances at 1×.

**Attitude filter** — Mahony PI complementary filter with online bias
estimation, tilt derived from gravity direction rather than quaternion
composition. Both choices matter for the planned tilt inhibit, and §5.5 has the
numbers.

> These figures come from synthetic-signal tests with known ground truth, not
> flight data. They establish that the algorithm is correct, not that it
> survives a 50 g boost. The filter runs **ground-side only**.

---

## 4. Procurement

Bill of materials tracked in
[`water_rocket_avionics_bom.xlsx`](water_rocket_avionics_bom.xlsx) across two
Manila suppliers, Circuit Rocks and Makerlab PH. Current total is approximately
**PHP 13,600** across compute, sensors, radio, recovery, safety, power, and
prototyping consumables.

The sheet separates **Phase 1** (minimum viable flight computer: Pico, MS5611,
servo, buzzer, arming switch, battery/charger, protoboard) from **Phase 2**
(GPS, full IMU, high-g accelerometer, LoRa pair), on the reasoning that early
flights should validate airframe and recovery before exposing the more
expensive sensors to landing risk.

**Outstanding:** antennas for both LoRa units, soldering and measurement tools
(iron, multimeter, logic analyzer), and mechanical hardware (standoffs, foam
potting, latch pin stock).

**Antennas.** The flight unit is hand-built, not purchased — an 82 mm
solid-core wire quarter-wave monopole soldered to the RFM95W ANT pad. The
ground station needs a bought SMA (or U.FL plus pigtail) antenna rated
specifically for 868 MHz. GSM/cellular-band antennas do not present a correct
50 Ω match at 868 MHz and degrade both transmit efficiency and receive
sensitivity; the correct search term is "868 MHz LoRa antenna".

---

## 5. Regulatory

LoRa telemetry operates at **868.000–870.000 MHz at or below +14 dBm
(25 mW erp)** per NTC MC 03-06-2017, the Philippine non-specific SRD/telemetry
allocation — not the US 915 MHz default the modules ship configured for.
Whether the 915–918 MHz AS923-3 allocation is separately permitted is disputed
locally and is not relied upon.

> NTC MC 004-06-2026 amended SRD parameters in June 2026. **Re-check current
> circulars before any RF-active flight.** §12 of the design document is
> explicit that it is not authoritative.

---

## 6. Next steps

**Firmware**

- [ ] **Write `rocket_flight/rocket_flight.ino`** — state machine, threading
      model and flight record are specified; nothing implements them yet
- [ ] Port the attitude filter into flight firmware and implement the tilt
      inhibit the state machine specifies
- [ ] Extend LSM6DSO configuration to flight ranges (±16 g, ±2000 dps) via
      direct register writes, confirming via readback that the ranges took

**Sensors and radio**

- [ ] **Sign off the ADXL375 on the bench** — address in the scan, boot line
      decoding 800 Hz, ≈1.0 g at rest, a tap spiking past the LSM6's ceiling
- [ ] **Sign off the GPS outdoors** — boot line reading `airborne <1g  OK`,
      then a 3D fix with plausible coordinates within 30–60 s
- [ ] Paired LoRa TX/RX test with RSSI + packet-loss logging
- [ ] Wire the radio into the ground station — replace the synthetic producer
      task with a UART packet reader. The architecture already supports it; no
      viewer changes needed
- [ ] Build and tune the flight antenna (82 mm, 868 MHz)
- [ ] Source and fit the ground-station 868 MHz SMA antenna

**Ground segment**

- [x] ~~Add a telemetry staleness timeout~~ — done; 1.5 s gap dims the readouts
      and counts the age of the last frame, in all three viewer builds
- [ ] Set the operational AP password on the ground station hardware; the value
      in this repository is a placeholder default

**Measurement and test**

- [ ] Measure 1σ altitude noise with viewer smoothing **off** and set
      `APOGEE_DROP_M` from it — smoothed data reads ~4.7× quieter than the
      sensor is, which would set the threshold far too tight
- [ ] Full state-machine integration test on the bench (syringe method)

**Mechanical**

- [ ] Collar joint CAD with real tolerances; FEA the pin/collar under 478 N
- [ ] Latch fabrication and 50× pull test under packed-chute load
- [ ] Reconcile the separation-spring forward mass (0.18 kg) against the
      airframe dry-mass target (150–200 g) — the two do not currently agree

**Flight**

- [ ] First flight: logging only, no deployment
