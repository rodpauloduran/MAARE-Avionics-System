# MAARE Avionics System

Student-designed and student-researched (SRAD) flight computer for a water
pressure rocket. The vehicle is a testbed; **the avionics are the deliverable.**

Mapúa University (Intramuros) — rocketry group, pre-organisation phase.

---

## What this is

A water rocket's flight envelope is hostile in the ways that matter — 20–100 g
boost, sub-second burn, ~10 second total flight, a wet landing — while carrying
no propellant hazard and no regulatory burden. Every subsystem here (deployment
state machine, static ports, telemetry link, arming interlock) transfers
unchanged to a motor-powered vehicle. The consequence of a failure is a plastic
bottle, not a motor CATO.

**Objectives:** log full-rate flight data through a complete flight;
autonomously detect apogee and actuate parachute deployment; transmit live
telemetry and act as a recovery beacon after landing; build team competency
that transfers directly to high-power rocketry.

## Start here

**[`AVIONICS_DOCUMENTATION.md`](AVIONICS_DOCUMENTATION.md)** — the design
document (Revision B) and the authority for this project. Everything else is
either a companion spec or an implementation of something it specifies. If two
documents disagree, that one wins.

## Repository layout

| Path | What it is |
|---|---|
| [`AVIONICS_DOCUMENTATION.md`](AVIONICS_DOCUMENTATION.md) | **Design document, Revision B.** Sensors, deployment logic, radio, power, recovery, firmware, regulatory, test plan |
| [`HARDWARE_REFERENCE.md`](HARDWARE_REFERENCE.md) | Quick bench reference — pin map, I²C addresses, driver gotchas, bring-up order |
| [`airframe_build_spec.md`](airframe_build_spec.md) | Airframe structure, materials, dimensions, assembly sequence |
| [`avionics_status_report.txt`](avionics_status_report.txt) | Project status summary |
| `water_rocket_avionics_BOM.xlsx` | Bill of materials — costs, suppliers, phasing, per-part justification |
| `rocket_diagnostics.ino` | Bench diagnostics sketch, **rev B** (verified working) |
| `rocket_attitude_viewer.html` | **Live 3D attitude viewer.** Web Serial over USB, or SSE over Wi-Fi |
| `rocket_attitude_viewer_serial.html` | Serial-only build of the viewer — the bench path that works today |
| [`pico_w_ground_station/`](pico_w_ground_station/) | Ground station firmware (MicroPython) + the viewer as deployed |
| `rocket_assembly_viewer.jsx` | Interactive 3D assembly and separation-sequence visualiser |

## Hardware

| Subsystem | Component |
|---|---|
| Compute (flight) | Raspberry Pi Pico (RP2040), official Arduino Mbed OS core |
| Compute (ground) | Raspberry Pi Pico W, MicroPython |
| Altitude | MS5611 (GY-63) — primary altimeter and apogee trigger |
| Attitude | Pololu MinIMU-9 v6 (LSM6DSO + LIS3MDL) |
| Boost accel | Adafruit ADXL375 (±200 g) |
| Position | SparkFun SAM-M8Q (u-blox, multi-GNSS incl. QZSS) |
| Radio | 2× RFM95W (SX1276) LoRa, 868–870 MHz at ≤ +14 dBm |
| Deployment | MG90D servo, pin latch in double shear |
| Arming | Reed switch + N35 magnet |
| Power | 500 mAh 1S LiPo direct to VSYS |

~80 g, ~80–90 mA, roughly 5 hours of pad endurance.

## Where the project actually stands

**Working:** bench diagnostics on hardware; the attitude viewer; the Pico W
ground station serving it over its own Wi-Fi; the attitude filter (PI
complementary, with online bias estimation and yaw-immune tilt).

**Not yet:**

- **The deployment code has never flown.** This is the principal risk and the
  deliberate trade for SRAD capability.
- **No radio link.** The ground station's telemetry is synthetic — nothing in
  that path has carried a real sensor reading over the air.
- **The tilt inhibit is specified but not implemented**, and the attitude
  filter runs ground-side only.
- **Attitude filter numbers are from synthetic tests**, not flight data. They
  establish that the algorithm is correct, not that it survives a 50 g boost.
- `rocket_flight.ino` is not yet written.

§14 of the design document keeps the full list.

## Two defects worth remembering as a class

Both were found during Revision B bring-up, and both were **silent**:

- The **MS5611 reported itself absent while working perfectly** — a
  zero-length I²C `endTransmission()` on Mbed cores issues a read-type
  transaction the sensor won't ACK.
- The **gyro reported every rate 4× too high** while merely looking
  "unstable" — `enableDefault()` selects ±245 dps, not the ±1000 dps the
  hard-coded scale constant assumed.

Neither announced itself as a configuration error. The structural response —
**read configuration back from the hardware and print it at boot** — is cheap,
and is now the standing expectation for any new device on this bus.

## Safety

This vehicle is pressurised to 100 psi and deploys a parachute under stored
spring energy.

- **Arm last, disarm first.** The reed interlock exists precisely so that
  ground handling cannot fire the latch while someone's hands are in the bay.
- **Never key the LoRa transmitter without an antenna attached** — it damages
  the PA.
- Pressure-test the bulkhead in isolation, and leak-test the airframe at full
  pressure, away from anyone's face, before trusting either.
- Flight test progression is sequenced deliberately: airframe only, then
  avionics logging-only, then deployment enabled — and only after offline
  analysis confirms the apogee detector *would have* fired correctly on logged
  flights.

## Regulatory

LoRa telemetry operates at **868.000–870.000 MHz, ≤ +14 dBm (25 mW erp)** per
NTC MC 03-06-2017 (Philippine non-specific SRD allocation) — deliberately not
the US 915 MHz default. NTC MC 004-06-2026 amended SRD parameters in June 2026;
**re-check current circulars before any RF-active flight.** §12 has the detail
and is explicit that it is not authoritative.
