# Water Rocket Avionics — Hardware & Firmware Reference

Target: Raspberry Pi Pico (RP2040), **official Arduino Mbed OS RP2040 core**
(not the Earle Philhower community core).

> **Authority.** `avionics_documentation.md` is the design authority. This file is the quick bench reference - pin numbers, addresses,
> driver gotchas, bring-up order. Where the two disagree, the design document
> wins. Sections below cite it as §n.
>
> **The core choice is not cosmetic.** It removes the dual-core API, changes
> the LoRa library, removes LittleFS, and shrinks the RAM budget. See §10.1
> of the design document.

---

## 1. Pin map

| Function | GPIO | Physical pin | Notes |
|---|---|---|---|
| I2C0 SDA | GP4 | 6 | All four sensors share this bus |
| I2C0 SCL | GP5 | 7 | 400 kHz, short traces |
| SPI0 MISO | GP16 | 21 | RFM95W |
| SPI0 CS | GP17 | 22 | RFM95W chip select |
| SPI0 SCK | GP18 | 24 | RFM95W |
| SPI0 MOSI | GP19 | 25 | RFM95W |
| LoRa RESET | GP20 | 26 | Can tie high if pins are tight |
| LoRa DIO0 | GP21 | 27 | IRQ; can poll registers instead |
| Servo PWM | GP15 | 20 | 50 Hz, separate power rail |
| Buzzer | GP13 | 17 | Via NPN/MOSFET if running at 5 V |
| Reed switch (arming) | GP12 | 16 | Internal pull-up, switch to GND |
| Battery sense | GP26 | 31 | ADC0, via 100k/100k divider |
| Status LED | GP25 | — | Onboard |
| VSYS (battery in) | — | 39 | 1.8–5.5 V, buck-boost handles LiPo range |
| GND | — | 38 | Common ground with servo |

**Free after this:** GP0–GP3, GP6–GP11, GP14, GP22, GP27, GP28. Plenty of margin
for a nichrome MOSFET, a second deployment channel, or an OLED.

---

## 2. I2C addresses

| Device | Address | Notes |
|---|---|---|
| MS5611 (GY-63) | **0x77** | 0x76 if CSB is pulled high. PS pin HIGH selects I2C mode. |
| LSM6DSO (MinIMU-9 v6) | **0x6B** | Pololu pulls SA0 high. Drive SA0 low for 0x6A. |
| LIS3MDL (MinIMU-9 v6) | **0x1E** | Pololu pulls SA1 high. 0x1C if pulled low. |
| ADXL375 (Adafruit) | **0x53** | 0x1D if the address jumper is bridged. |
| SAM-M8Q GPS | **0x42** | u-blox DDC (I2C) mode. |

![Bench stack on breadboards: Pico H, SAM-M8Q GPS, ADXL375, MinIMU-9 v6 and
GY-63 barometer wired to one I²C bus](docs/images/bench-stack-1.jpg)

*The stack these addresses refer to. Breadboard and jumper wires — the FR4 sled
of §11 is not built.*

No conflicts. Run an I2C scanner after adding each device and confirm the
address appears *before* writing any driver code.

A real scan from this stack, with all five devices answering, is in §13.2 of the
design document. It also shows a phantom `0x7E` in the reserved range, which is
a bus-quality symptom rather than a device — see that section.

**Scan correctly on this core (§10.1).** A zero-length `endTransmission()`
issues a *read*-type transaction on Mbed cores, which most devices will not
ACK — so a naive scanner reports working devices as absent. Write a dummy
byte first:

```cpp
for (byte a = 1; a < 127; a++) {
  Wire.beginTransmission(a);
  Wire.write(0);                 // <- required, forces a write-type probe
  if (Wire.endTransmission() == 0) { /* device present */ }
}
```

Side effect: this writes a zero byte to every address on the bus. Harmless for
everything currently fitted, but re-check if a new part joins.

**Pull-up warning:** each breakout carries its own pull-up resistors, and all
four boards are now fitted, so this has gone from a caution to a live
consideration. Four sets in parallel can drag the bus too strongly at 400 kHz.
If the bus misbehaves — dropped devices in the scan, intermittent NACKs, reads
that fail only under vibration — remove the pull-ups from all but one board
(Pololu's are 10 k, Adafruit's are 10 k, GY-63 clones are often 4.7 k or 2.2 k
— remove the GY-63's first).

**Scan side effect, re-checked for the full set.** The dummy byte above writes
`0x00` to every address. For the MS5611 that is a harmless ADC-read command; on
the ADXL375 register 0 is the read-only device ID; on the u-blox it only moves
the DDC address pointer. All four are safe. Re-check if a fifth board joins.

---

## 3. Libraries

Install via Arduino Library Manager unless noted.

| Device | Library | Include |
|---|---|---|
| Board core | **Arduino Mbed OS RP2040** (Boards Manager) | — |
| MS5611 | **MS5611** by Rob Tillaart | `#include <MS5611.h>` |
| LSM6DSO | **LSM6** by Pololu | `#include <LSM6.h>` |
| LIS3MDL | **LIS3MDL** by Pololu | `#include <LIS3MDL.h>` |
| ADXL375 | **Adafruit ADXL375** (+ Adafruit_Sensor, Adafruit_BusIO) | `#include <Adafruit_ADXL375.h>` |
| GPS | **SparkFun u-blox GNSS Arduino Library** | `#include <SparkFun_u-blox_GNSS_Arduino_Library.h>` |
| LoRa | **RadioLib** by Jan Gromes | `#include <RadioLib.h>` |
| Servo | Bundled with the core | `#include <Servo.h>` |
| Filesystem | **None — LittleFS is not bundled on this core.** Logs dump over USB serial as CSV (§10.1). |

---

## 4. Per-sensor driver notes

### MS5611 — the one with two quirks

**Quirk 1: `begin()` does not work on this core (§4.2).** It returns false
and reports the sensor absent while the part is fine — `isConnected()` uses
the zero-length `endTransmission()` described above, and the library's
workaround is gated behind `#ifdef ARDUINO_ARCH_NRF52840`, so it never
compiles in on RP2040. **Use `reset()` instead**, which is public, skips
detection, and loads the calibration PROM:

```cpp
if (baro.reset()) {                 // NOT baro.begin()
  Serial.println(F("MS5611 initialised"));
} else {
  // genuine wiring or power fault
}
```

If this is missed the failure is silent: calibration never loads and every read
returns a fixed 44307.70 m rather than erroring.

**Quirk 2: it cannot be read in one transaction.** Each measurement is a
**two-step transaction**:

1. Write a command to start conversion (pressure or temperature).
2. Wait for the conversion — **9.04 ms at OSR 4096**, less at lower OSR.
3. Read the 24-bit ADC result.

You must also read 6 factory calibration coefficients from PROM at boot and
apply the compensation polynomial. Rob Tillaart's library handles the PROM and
the maths, but `read()` **blocks** for the conversion time. At 9 ms that would
destroy a 500 Hz loop.

**Solution:** run the baro as its own small state machine, alternating pressure
and temperature conversions, polling for completion rather than blocking. You
only need temperature every ~20 pressure reads — it changes slowly. This gives
you ~100 Hz pressure at full resolution.

```
BARO_IDLE -> issue D1 (pressure) conv -> wait 9ms -> read -> compute
          -> every 20th cycle, issue D2 (temp) conv instead
```

**Zeroing:** at boot, average 100 samples with the rocket on the pad and store
as `groundPressure`. Altitude is always computed as a *delta* from that:

```
alt_m = 44330.0 * (1.0 - pow(p / groundPressure, 0.1902949))
```

Never trust absolute MSL altitude.

### LSM6DSO (Pololu LSM6 library)

`imu.init()` auto-detects the device and address. `imu.enableDefault()` sets
1.66 kHz ODR, ±2 g, ±245 dps — **change this**, you need wider ranges:

- Accel: ±16 g full scale, 833 Hz ODR
- Gyro: ±2000 dps full scale, 833 Hz ODR

Write CTRL1_XL and CTRL2_G directly. Raw values land in `imu.a.x/y/z` and
`imu.g.x/y/z` as int16.

**Never hard-code the scale factor — read it back from the register (§4.3).**
`enableDefault()` selects ±245 dps, so pairing it with the `0.035` dps/LSB
constant that belongs to ±1000 dps reports every rate **4x too high** and
saturates silently above 245 dps. Note also the non-obvious encoding — for the
accelerometer, `01` = **±16 g**, not ±4 g.

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

Print the decoded range at boot. A wrong range then shows up as a wrong
*printed* range within five seconds of power-on, instead of as a filter that
merely looks unstable.

**Detect saturation, don't infer it.** A clipped reading is not a large
reading, it is an unknown one. Flag any axis within ~2% of int16 full scale,
and report accelerometer and gyro clipping separately from the "approaching
range ceiling" warning.

**Gyro bias:** average 400-500 samples with the rocket held still at boot and
subtract that offset from every subsequent reading. Skip this and your attitude
estimate drifts visibly within seconds. (`rocket_diagnostics/` uses 250 at
25 Hz — enough for bench work, and re-runnable in place with the `b` command
as the board warms. Flight firmware takes the full 400-500.)

### ADXL375

`accel.begin()` then read via the Adafruit_Sensor interface, or
`accel.getX()/getY()/getZ()` for raw counts. Fixed ±200 g range, ~49 mg/LSB.
Set the data rate to 800 Hz or higher (`accel.setDataRate(ADXL343_DATARATE_800_HZ)`)
— the default 100 Hz will smear your boost profile.

Log **raw counts**, not floats. Convert offline.

**`begin()` is safe on this core — unlike the MS5611's.** Worth stating because
the MS5611 note above teaches the opposite reflex. Adafruit_BusIO's address
detection has an explicit `#ifdef ARDUINO_ARCH_MBED` that writes the dummy byte
before `endTransmission()`, so the Mbed probe defect does not bite this part.
That is a property of the library, not the bus — check per device.

**Saturation is at 13 bits, not 16.** `begin()` puts the part in FULL_RES, so
readings are 13-bit two's complement sign-extended into `int16`: full scale is
about **±4095 counts**, not ±32767. Copying the LSM6's `32000` threshold here
means clipping is *never* detected and a clipped boost trace is reported as a
measurement. Use a separate constant:

```cpp
const int16_t SAT_COUNT    = 32000;   // LSM6, int16 full scale
const int16_t HG_SAT_COUNT = 4000;    // ADXL375, 13-bit full scale (~196 g)
```

**There is no range register to read back.** ±200 g is fixed in silicon and the
library's `setRange()`/`getRange()` are deliberate no-ops, so the "read it back"
rule applies to the one thing that *is* configurable — the data rate. Decode it
from `BW_RATE` (low nibble) and print it at boot:

```cpp
uint8_t bw = accel.readRegister(ADXL3XX_REG_BW_RATE) & 0x0F;
int hz = (bw >= 8) ? (25 << (bw - 8)) : 0;   // exact: 3.125 Hz * 2^(code-5)
```

**At rest it has ~20 counts of signal** (1 g ÷ 49 mg/LSB), so a ±0.05 g wobble
is quantisation, not noise. Don't chase it. A rest check on this part is really
looking for a dead axis or the wrong chip in the footprint — an ADXL345 reads
about 12× low here and nothing else would flag it.

### SAM-M8Q

Configure once at boot, then poll:

```cpp
gnss.begin(Wire);                    // I2C at 0x42
gnss.setI2COutput(COM_TYPE_UBX);     // turn off NMEA, use binary UBX
gnss.setNavigationFrequency(5);      // 5 Hz
gnss.setDynamicModel(DYN_MODEL_AIRBORNE1g);   // CRITICAL
gnss.saveConfiguration();
```

**The dynamic model line is the one people miss.** The default model assumes a
car and will reject your trajectory as implausible, dropping lock. Airborne <1g
tells the receiver to expect vertical motion.

**Read it back.** A `setDynamicModel()` that silently failed is indistinguishable
from one that worked until you are airborne — the worst possible time to find
out. Same discipline as the IMU:

```cpp
uint8_t dm = gnss.getDynamicModel();          // 255 = the query itself failed
if (dm != DYN_MODEL_AIRBORNE1g) Serial.println(F("NOT airborne <1g"));
```

**`getPVT()` is only non-blocking if you call `setAutoPVT(true)` first.** This
is the trap. Without it, `getPVT()` polls the module and **blocks for up to
1100 ms** — 27 missed samples at 25 Hz, and fatal in a 500 Hz flight loop. With
it, the module pushes solutions on its own schedule and the call returns
immediately. The default *works*; it is just slow, which is exactly why it gets
missed.

```cpp
gnss.setAutoPVT(true);        // <- do this, or getPVT() blocks
```

Even so, poll it from the normal-priority `loop()`, never from the realtime
sensor thread (§10.2). 1 Hz is plenty — nothing it reports changes faster than
walking pace.

**Cold start is 30–60 s and needs sky view.** "No fix" on an indoor bench is
expected, not a fault. Worth distinguishing *never acquired* from *acquired and
lost* in your output: the first points at sky view, the second at antenna or
power.

**Read `hAcc`, and do not trust a fix just because it says 3D.** Measured on
this bench: a stationary board on a **3D fix with 5 satellites** sat **238 m**
from where it first locked, with the fix type reading `3D` throughout. Four
satellites is the minimum for a 3D solution, so five has almost no geometric
margin, and the error ellipse goes long and thin — poor fixes drift in a line,
not a blob. The accuracy estimate is already in the NAV-PVT message:

```cpp
int32_t hAccMm = gnss.getHorizontalAccEst();   // mm; print it next to the fix
```

**And never datum anything on the first fix** — it is the worst one you will
get. Gate on satellites and hAcc (this project uses ≥ 6 sats and ≤ 10 m) or set
the origin deliberately once the receiver has settled.

### RFM95W

Use **RadioLib**, not RadioHead — RadioHead support on the Mbed core is patchy
(§9.2).

```cpp
SX1276 radio = new Module(RFM95_CS, RFM95_DIO0, RFM95_RST);
radio.begin(869.5);                  // PH SRD band — see BOM Design Notes
radio.setOutputPower(14);            // 25 mW erp ceiling
radio.setBandwidth(250.0);
radio.setSpreadingFactor(7);
radio.setCodingRate(5);
```

`radio.transmit()` **blocks** until the packet is sent. That is fine in the
normal-priority `loop()` and unacceptable in the flight thread. Because the
sensor loop runs at `osPriorityRealtime`, the scheduler preempts a blocked
transmit the instant a sensor tick is due — which is what preserves the rule
that the radio can never delay deployment (§2.3, §10.2).

---

## 5. State machine

```
IDLE      reed switch open. Sensors running, telemetry live, deployment inert.
  |  reed closed, stable 50 ms
ARMED     pre-trigger buffer running (keeps last 1 s of samples)
  |  high-g accel > 5 g for 50 ms
BOOST     record t_launch. Deployment inhibited.
  |  accel magnitude < 1 g  (burnout)
COAST     start lockout timer (500 ms) and backup apogee timer
  |  lockout expired AND (baro falling N samples OR backup timer fired)
DESCENT   fire servo, latch deployFired flag
  |  altitude stable +/- 1 m for 2 s
LANDED    dump RAM buffer over USB serial as CSV, buzzer pattern,
          beacon every 5 s
```

**Apogee detection (peak-and-drop), the primary trigger:**

```cpp
if (alt > altMax) { altMax = alt; fallCount = 0; }
else if (altMax - alt > APOGEE_DROP_M) { fallCount++; }
else { fallCount = 0; }

if (fallCount >= APOGEE_SAMPLES) -> DESCENT
```

With the MS5611 (~10 cm noise): `APOGEE_DROP_M = 0.3`, `APOGEE_SAMPLES = 5`
at 100 Hz baro. That fires ~0.25 s past true apogee.

Every trigger path is gated behind `state == COAST && millis() - tBurnout > LOCKOUT_MS`.
Nothing can fire during boost.

---

## 6. Telemetry packet (18 bytes)

```c
struct __attribute__((packed)) Telem {
  uint16_t t_ds;      // time since launch, deciseconds
  int16_t  alt_dm;    // altitude, decimetres
  int16_t  vel_dms;   // vertical velocity, decimetres/sec
  int16_t  accel_cg;  // accel magnitude, centi-g
  uint8_t  state;     // state machine enum
  uint8_t  flags;     // bit0 armed, bit1 deployed, bit2 gps fix, bit3 low batt
  int32_t  lat_e7;    // latitude x 1e7 (0 if no fix)
  int32_t  lon_e7;    // longitude x 1e7
};                    // = 18 bytes
```

**Fire and forget.** No acknowledgements, no retries, no sequence-dependent
deltas. Every packet stands alone; the ground station tolerates gaps silently.
Packets *will* drop — you are transmitting from a tumbling wet plastic tube.

---

## 7. RAM buffer sizing

```c
struct __attribute__((packed)) Sample {
  uint32_t t_us;        // 4
  int16_t  hg[3];       // 6   ADXL375 raw
  int16_t  acc[3];      // 6   LSM6DSO accel raw
  int16_t  gyr[3];      // 6   LSM6DSO gyro raw
  int32_t  pressure_pa; // 4   MS5611 compensated
  uint8_t  state;       // 1
  uint8_t  flags;       // 1   bit0 = fresh baro sample
};                      // = 28 bytes
```

**4000** samples x 28 B = **112 KB**. The RP2040 has 264 KB, but Mbed OS itself
consumes ~40-60 KB, so the buffer is sized against roughly 200 KB of usable
RAM, not the full 264 KB (§10.1).

At 500 Hz, 4000 samples is 8 seconds — the full flight plus pre-trigger.

Declare it as a global static array. Do **not** malloc.

---

## 8. Bring-up order

Do not skip steps. Each one isolates a class of failure.

1. Pico alone — blink GP25. **done**
2. I2C scanner — nothing connected. Confirms the bus works. **done**
3. Add **MS5611** only. Scanner shows 0x77. Read pressure, breathe on it, watch it change. **done**
4. Add **MinIMU-9**. Scanner shows 0x6B and 0x1E. Tilt it, watch accel/gyro. **done**
5. Add **ADXL375**. Scanner shows 0x53. Tap it, watch it spike. **wired, not signed off** —
   check the boot line decodes 800 Hz, `hg` reads ≈1.0 g at rest, a firm tap
   goes well past the LSM6's ±2 g ceiling.
6. Add **GPS**. Scanner shows 0x42. Take it outside, wait for fix. **wired, not
   signed off** — check the boot line says `airborne <1g  OK`, then `g` reports
   a 3D fix with plausible coordinates within 30–60 s outdoors.
7. **LoRa** on SPI — separate bus, test link with a second Pico before integrating.
8. **Servo** on its own supply. Sweep it. Add the 220 uF cap.
9. **Reed switch** — confirm the pin reads LOW with the magnet present.
10. Only now: assemble the full loop and state machine.

At every step, if the address does not appear, stop and fix it. Do not proceed
hoping it resolves itself.

---

## 9. Testing before flight

- **Syringe test:** tape a syringe over the static port, pull gently, watch the
  altitude climb and the state machine advance. This validates apogee detection
  without leaving the bench.
- **Drop test:** foam-padded, from waist height. Confirms nothing resets.
- **Shake test:** vigorous, for 30 s. Confirms no I2C dropouts under vibration.
- **Car test:** drive with GPS running. Confirms fix, dynamic model, logging.
- **Latch test x50:** with the chute *actually packed*. Friction with a
  compressed parachute is far higher than an empty bench test.
- **First flight: logging only.** No deployment. Prove the data before you trust
  the trigger.
