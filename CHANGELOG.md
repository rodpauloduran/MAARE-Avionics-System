# Changelog

Notable changes to the MAARE avionics system, newest first.

Format loosely follows [Keep a Changelog](https://keepachangelog.com/).

---

## [Unreleased]

### Changed

- Arduino sketches now live in per-sketch folders
  (`rocket_diagnostics/rocket_diagnostics.ino`). The Arduino IDE requires a
  `.ino` to sit in a directory of the same name; a bare sketch at the
  repository root prompts the IDE to relocate it, and two of them at the root
  cannot both be opened cleanly. `rocket_flight/` follows the same shape.

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

[Unreleased]: https://github.com/rodpauloduran/MAARE-Avionics-System/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/rodpauloduran/MAARE-Avionics-System/releases/tag/v0.1.0
