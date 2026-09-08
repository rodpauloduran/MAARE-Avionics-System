# Changelog

Notable changes to the MAARE avionics system, newest first.

Versions here track **the repository**. The design document carries its own
independent revision letter (currently Revision B) — the two are deliberately
not the same counter, because the design is allowed to run ahead of what is
built.

Format loosely follows [Keep a Changelog](https://keepachangelog.com/).

---

## [Unreleased]

Nothing yet.

---

## [0.1.0] — 2026-09-08

First versioned baseline. Establishes the repository and gets every document
telling the same story; **it does not add hardware capability.**

### Where the hardware actually stands at v0.1

- MS5611 barometer and MinIMU-9 v6 wired to the flight Pico and reading
- Bench diagnostics (`rocket_diagnostics.ino`, rev B) working on hardware
- Pico W ground station and attitude viewer working, on **synthetic telemetry**
- Nothing else connected: no high-g accelerometer, no GPS, no radio link, no
  servo or deployment hardware, no arming interlock, no airframe
- `rocket_flight.ino` not written

### Added

- `README.md` with an explicit build-status table, so the gap between design
  intent and built hardware is visible on the front page
- `pico_w_ground_station/README.md` — setup, endpoints, radio integration notes
- `CHANGELOG.md`
- `.gitignore`, `.gitattributes`

### Changed

- **Repository laid out to match §15 of the design document.** Ground station
  moved under `pico_w_ground_station/`
- **Filenames normalised** to lowercase `snake_case`, with `README`/`CHANGELOG`
  left uppercase per convention. Browser-download suffixes stripped
  (`HARDWARE_REFERENCE(1).md`, `rocket_attitude_viewer(5).html`,
  `rocket_assembly_viewer(2).jsx`, `water_rocket_avionics_BOM(5).xlsx`)
- Bench sketch renamed `rocket_diagnostics.ino`, matching how every document
  refers to it
- `avionics_status_report` converted from `.txt` to Markdown
- **Both viewer builds kept and distinguished.** `rocket_attitude_viewer.html`
  is the dual-transport build the ground station serves;
  `rocket_attitude_viewer_serial.html` is the serial-only build — the only path
  currently carrying *real* sensor data, since no radio link exists

### Fixed — document drift

These documents were written in separate sessions and had contradicted each
other. The design document is now the stated authority.

**`hardware_reference.md` was pre-Revision-B** and wrong on six points.
Following it would have reintroduced both defects Revision B had already fixed:

| Topic | Was | Now |
|---|---|---|
| Toolchain | Philhower core | Official Arduino Mbed OS core |
| Concurrency | "core 1, never core 0" | `osPriorityRealtime` thread — `setup1()`/`loop1()` do not exist on this core |
| LoRa library | RadioHead | RadioLib |
| Storage | LittleFS | Not bundled; CSV dump over USB serial |
| Ring buffer | 6000 samples / 168 KB | 4000 / 112 KB, for Mbed OS overhead |
| Known defects | Not mentioned | MS5611 `reset()`-not-`begin()`, dummy-byte I²C probe, gyro scale readback |

Also corrected its telemetry packet heading, which read "16 bytes" above a
struct annotated as 18.

**`avionics_status_report.md` was an August 2026 snapshot**, predating the
ground segment:

- Corrected the claim that the Pico W ground station runs the same Arduino core
  as the flight computer — it runs MicroPython
- Corrected "`enableDefault()` ranges (±2 g, **±1000 dps**)" — `enableDefault()`
  selects ±245 dps, and that exact mismatch *was* the 4× gyro defect, recorded
  in the report as though it were correct behaviour
- Added a ground-segment section and a build-status table
- Reorganised next steps by area

**Smaller corrections**

- `pico_w_ground_station/main.py` — stale viewer size (37 → 45 KB) and
  flight-profile apogee (300 → 377 m, which is what its own constants compute)
- `airframe_build_spec.md` — reconciled the 28 mm neck *finish* against the
  21 mm *bore* used in the 478 N thrust calculation; aligned the nose cone
  length range with the layout diagram
- `avionics_documentation.md` — §6.7 now explains why two viewer builds exist;
  §15 marks `rocket_flight.ino` as not written rather than implying it exists

### Known unreconciled

- §8.3 sizes the separation spring against **0.18 kg forward mass**, while
  `airframe_build_spec.md` targets **150–200 g dry mass for the whole rocket**.
  A 2 L PET bottle is ~50 g before fins, so these do not both hold. Left as-is
  pending a decision — it is an engineering question, not a documentation one

---

[Unreleased]: https://github.com/rodpauloduran/MAARE-Avionics-System/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/rodpauloduran/MAARE-Avionics-System/releases/tag/v0.1.0
