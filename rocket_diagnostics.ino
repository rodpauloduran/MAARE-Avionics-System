// =============================================================================
//  ROCKET AVIONICS — BENCH DIAGNOSTICS  (rev B)
//  MS5611 barometer + LSM6 accelerometer/gyro  ->  Serial Monitor / viewer
//
//  CHANGES FROM REV A
//  ------------------
//  1. GYRO RANGE.  imu.enableDefault() sets the gyro to +/-245 dps, NOT
//     +/-1000 dps.  Rev A paired that default with a 0.035 dps/LSB constant,
//     which is the +/-1000 dps figure, so every rate was reported 4x too high
//     and saturated above 245 dps.  This sketch now writes CTRL2_G explicitly
//     and derives the scale factor from the register, so the two can no longer
//     disagree.
//  2. SCALE FACTORS ARE READ BACK FROM THE CHIP, not hard-coded.  If a future
//     edit changes a range, the conversion follows automatically.
//  3. GYRO BIAS is measured at startup and subtracted.  Command 'b' redoes it.
//  4. SATURATION IS DETECTED and reported, on both sensors.
//  5. Timing constants corrected for the 25 Hz sample rate (noise window,
//     summary interval, velocity filter), plus a resync after blocking calls.
//
//  Wiring (Raspberry Pi Pico, official Arduino Mbed core):
//    Wire defaults to GP4 = SDA, GP5 = SCL on this board.
//
//      MS5611 / GY-63            LSM6 board (MinIMU-9 v6)
//        VCC -> 3V3                VIN -> 3V3
//        GND -> GND                GND -> GND
//        SDA -> GP4                SDA -> GP4
//        SCL -> GP5                SCL -> GP5
//        PS  -> VCC   (I2C mode; GY-63 has an internal pull-up)
//        CSB -> GND   (address 0x77)
//
//  Serial commands (115200 baud):
//    z   re-zero ground pressure to here
//    b   re-measure gyro bias   (hold the board still)
//    r   reset the peak trackers
//    c   toggle CSV mode for the Arduino Serial Plotter
//    v   toggle telemetry stream for the attitude viewer
//    h   reprint the column header
// =============================================================================

#include <Wire.h>
#include "MS5611.h"
#include <LSM6.h>

MS5611 baro(0x77);
LSM6   imu;

// -----------------------------------------------------------------------------
//  Sensor ranges.
//
//  208 Hz is comfortably above the 25 Hz sample rate.  +/-1000 dps is chosen
//  because a hand flip on the bench already exceeds +/-245, let alone a real
//  flight.  +/-2 g stays for now: it is the right choice for checking that the
//  accelerometer reads a clean 1.00 g at rest, and it is deliberately the FIRST
//  thing you will saturate, so you find out early rather than in the air.
//
//  For flight, widen the accelerometer (CTRL1_XL) or move to a dedicated
//  high-g part.  The readback below means you only change the register.
// -----------------------------------------------------------------------------
const uint8_t CFG_CTRL1_XL = 0b01010000;   // 208 Hz, +/-2 g
const uint8_t CFG_CTRL2_G  = 0b01011000;   // 208 Hz, +/-1000 dps

//  Filled in by applyImuConfig() from what the chip actually reports.
float ACC_G_PER_LSB   = 0.0;
float GYR_DPS_PER_LSB = 0.0;
int   ACC_RANGE_G     = 0;
int   GYR_RANGE_DPS   = 0;

const int   SAMPLE_MS   = 40;             // 25 Hz
const int   NOISE_N     = 100;            // ~4 s of altitude at 25 Hz
const int   SUMMARY_N   = 125;            // ~5 s at 25 Hz
const int   BIAS_N      = 250;            // gyro bias samples

//  Raw counts this close to full scale mean the reading is clipped and the
//  true value is unknown.  int16 saturates at 32767.
const int16_t SAT_COUNT = 32000;

// ------------------------------- state ---------------------------------------
float groundHpa = 1013.25;
float alt       = 0.0;
float altPrev   = 0.0;
float vel       = 0.0;

float altPeak   = 0.0;
float gPeak     = 0.0;
float gyroPeak  = 0.0;

float gyroBiasX = 0.0, gyroBiasY = 0.0, gyroBiasZ = 0.0;

float noiseBuf[NOISE_N];
int   noiseIdx  = 0;
bool  noiseFull = false;

bool     csvMode   = false;
bool     vizMode   = false;
bool     accClip   = false;
bool     gyrClip   = false;
uint32_t nextTime  = 0;
int      summaryCounter = 0;

// =============================================================================
//  helpers
// =============================================================================

// Right-align a float in a fixed column width using only Serial.print().
// Avoids printf and dtostrf entirely, both of which have portability traps.
void pad(float value, int width, int decimals) {
  int len = decimals + (decimals > 0 ? 1 : 0);
  if (value < 0) len++;
  long whole = (long)fabs(value);
  int digits = 1;
  while (whole >= 10) { whole /= 10; digits++; }
  len += digits;
  for (int i = len; i < width; i++) Serial.print(' ');
  Serial.print(value, decimals);
}

// -----------------------------------------------------------------------------
//  Configure the IMU, then ask the chip what it is actually set to and derive
//  the conversion constants from the answer.
//
//  This is the whole point of rev B.  A hard-coded scale factor is a silent
//  claim about a register you cannot see.  Reading it back means a wrong range
//  becomes a wrong PRINTED range, which you will notice.
// -----------------------------------------------------------------------------
void applyImuConfig() {
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL1_XL, CFG_CTRL1_XL);
  imu.writeReg(LSM6::CTRL2_G,  CFG_CTRL2_G);
  delay(20);

  uint8_t c1 = imu.readReg(LSM6::CTRL1_XL);
  uint8_t c2 = imu.readReg(LSM6::CTRL2_G);

  // CTRL1_XL bits [3:2] = FS_XL.  Datasheet order is 2 g, 16 g, 4 g, 8 g.
  switch ((c1 >> 2) & 0x03) {
    case 0: ACC_RANGE_G =  2; ACC_G_PER_LSB = 0.000061; break;
    case 1: ACC_RANGE_G = 16; ACC_G_PER_LSB = 0.000488; break;
    case 2: ACC_RANGE_G =  4; ACC_G_PER_LSB = 0.000122; break;
    case 3: ACC_RANGE_G =  8; ACC_G_PER_LSB = 0.000244; break;
  }

  // CTRL2_G bit 1 = FS_125 overrides bits [3:2] = FS_G.
  if ((c2 >> 1) & 0x01) {
    GYR_RANGE_DPS = 125; GYR_DPS_PER_LSB = 0.004375;
  } else {
    switch ((c2 >> 2) & 0x03) {
      case 0: GYR_RANGE_DPS =  245; GYR_DPS_PER_LSB = 0.00875; break;
      case 1: GYR_RANGE_DPS =  500; GYR_DPS_PER_LSB = 0.0175;  break;
      case 2: GYR_RANGE_DPS = 1000; GYR_DPS_PER_LSB = 0.035;   break;
      case 3: GYR_RANGE_DPS = 2000; GYR_DPS_PER_LSB = 0.070;   break;
    }
  }

  Serial.print(F("  CTRL1_XL = 0x")); Serial.print(c1, HEX);
  Serial.print(F("  ->  accel +/-")); Serial.print(ACC_RANGE_G);
  Serial.print(F(" g,  ")); Serial.print(ACC_G_PER_LSB, 6);
  Serial.println(F(" g/LSB"));

  Serial.print(F("  CTRL2_G  = 0x")); Serial.print(c2, HEX);
  Serial.print(F("  ->  gyro  +/-")); Serial.print(GYR_RANGE_DPS);
  Serial.print(F(" dps, ")); Serial.print(GYR_DPS_PER_LSB, 5);
  Serial.println(F(" dps/LSB"));

  if (c2 != CFG_CTRL2_G) {
    Serial.println(F("  WARNING: CTRL2_G did not take. Check I2C wiring."));
  }
}

// -----------------------------------------------------------------------------
//  Gyro zero-rate offset.  Every gyro has one, it changes with temperature,
//  and it is what walks your attitude estimate off during a flight when there
//  is no reliable gravity vector to correct against.  Board must be still.
// -----------------------------------------------------------------------------
void calibrateGyro() {
  Serial.print(F("Measuring gyro bias - hold still"));
  double sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < BIAS_N; i++) {
    imu.read();
    sx += imu.g.x; sy += imu.g.y; sz += imu.g.z;
    if (i % 50 == 0) Serial.print('.');
    delay(4);
  }
  gyroBiasX = sx / BIAS_N;
  gyroBiasY = sy / BIAS_N;
  gyroBiasZ = sz / BIAS_N;

  Serial.print(F(" done.  offset = "));
  Serial.print(gyroBiasX * GYR_DPS_PER_LSB, 2); Serial.print(F(" / "));
  Serial.print(gyroBiasY * GYR_DPS_PER_LSB, 2); Serial.print(F(" / "));
  Serial.print(gyroBiasZ * GYR_DPS_PER_LSB, 2); Serial.println(F(" dps"));
}

// Height above the launch pad.  Always a delta from a pad reading — never
// trust absolute sea-level altitude.
float altitudeFrom(float hpa) {
  return 44330.0 * (1.0 - pow(hpa / groundHpa, 0.1902949));
}

void zeroGround() {
  Serial.print(F("Zeroing ground pressure"));
  double sum = 0;
  for (int i = 0; i < 40; i++) {
    baro.read();
    sum += baro.getPressure();
    if (i % 10 == 0) Serial.print('.');
    delay(20);
  }
  groundHpa = sum / 40.0;
  alt = altPrev = vel = 0.0;
  altPeak = 0.0;
  noiseIdx = 0;
  noiseFull = false;
  Serial.print(F(" done. Ground = "));
  Serial.print(groundHpa, 2);
  Serial.println(F(" hPa"));
}

// 1-sigma spread of recent altitude readings.  The single most useful number
// here: it tells you how far past apogee your detector will fire.
float altNoise() {
  int n = noiseFull ? NOISE_N : noiseIdx;
  if (n < 5) return 0.0;
  float mean = 0;
  for (int i = 0; i < n; i++) mean += noiseBuf[i];
  mean /= n;
  float var = 0;
  for (int i = 0; i < n; i++) {
    float d = noiseBuf[i] - mean;
    var += d * d;
  }
  return sqrt(var / n);
}

void printHeader() {
  Serial.println();
  Serial.println(F("==================================================================="));
  Serial.println(F("  temp   press     alt    vel  noise |    ax    ay    az   |a|  tilt"));
  Serial.println(F("   C      hPa       m     m/s    cm  |     g     g     g     g   deg"));
  Serial.println(F("==================================================================="));
}

// =============================================================================
//  setup
// =============================================================================
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) { }

  Serial.println();
  Serial.println(F("=== ROCKET AVIONICS BENCH DIAGNOSTICS  rev B ==="));
  Serial.println();

  Wire.begin();

  // ---- I2C scan.  Always do this first.  If an address does not show up
  //      here, no amount of driver code will make that sensor work.
  //      The Wire.write(0) is required on Mbed cores, where a zero-length
  //      endTransmission() issues a read and most devices will not ACK it. ---
  Serial.println(F("I2C scan:"));
  int found = 0;
  for (byte a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    Wire.write(0);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  0x"));
      if (a < 16) Serial.print('0');
      Serial.print(a, HEX);
      Serial.print(F("  "));
      if      (a == 0x76 || a == 0x77) Serial.println(F("MS5611 barometer"));
      else if (a == 0x6A || a == 0x6B) Serial.println(F("LSM6 accel + gyro"));
      else if (a == 0x1C || a == 0x1E) Serial.println(F("LIS3MDL magnetometer"));
      else if (a == 0x53 || a == 0x1D) Serial.println(F("ADXL375 high-g"));
      else if (a == 0x42)              Serial.println(F("u-blox GPS"));
      else                             Serial.println(F("(unknown)"));
      found++;
    }
  }
  if (found == 0) Serial.println(F("  nothing found - check wiring and power"));
  Serial.println();

  // ---- barometer.  reset() loads the calibration constants and skips the
  //      isConnected() check, whose Mbed workaround is gated behind an
  //      nRF52840-only macro and so never compiles in on RP2040. -----------
  if (baro.reset()) {
    Serial.print(F("MS5611 found at 0x"));
    Serial.println(baro.getAddress(), HEX);
  } else {
    Serial.println(F("MS5611 NOT found."));
    Serial.println(F("  - PS pin must be HIGH for I2C mode"));
    Serial.println(F("  - CSB to GND = 0x77, CSB to VCC = 0x76"));
    while (1) delay(1000);
  }

  // ---- IMU ---------------------------------------------------------------
  if (imu.init()) {
    Serial.println(F("LSM6 found and initialised"));
    applyImuConfig();
  } else {
    Serial.println(F("LSM6 NOT found - check wiring (Pololu board is 0x6B)"));
    while (1) delay(1000);
  }

  Serial.println();
  calibrateGyro();
  Serial.println();
  zeroGround();

  Serial.println();
  Serial.println(F("Commands:  z = re-zero   b = gyro bias   r = reset peaks"));
  Serial.println(F("           c = CSV       v = viz stream  h = header"));
  printHeader();

  nextTime = millis();
}

// =============================================================================
//  loop
// =============================================================================
void loop() {

  // ---- commands ----------------------------------------------------------
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'z' || c == 'Z') { Serial.println(); zeroGround(); printHeader(); }
    else if (c == 'b' || c == 'B') { Serial.println(); calibrateGyro(); printHeader(); }
    else if (c == 'r' || c == 'R') {
      altPeak = alt; gPeak = 0; gyroPeak = 0;
      accClip = false; gyrClip = false;
      Serial.println(F("-- peaks reset --"));
    }
    else if (c == 'c' || c == 'C') {
      csvMode = !csvMode;
      Serial.println();
      if (csvMode) Serial.println(F("alt_m,vel_ms,accel_g,gyro_dps"));
      else printHeader();
    }
    else if (c == 'v' || c == 'V') {
      vizMode = !vizMode;
      Serial.println();
      if (!vizMode) printHeader();
    }
    else if (c == 'h' || c == 'H') printHeader();
  }

  // ---- fixed rate --------------------------------------------------------
  if ((long)(millis() - nextTime) < 0) return;
  nextTime += SAMPLE_MS;
  // zeroGround() and calibrateGyro() block for a second or more, which leaves
  // nextTime far in the past and causes a burst of catch-up iterations.
  if ((long)(millis() - nextTime) > SAMPLE_MS * 4) nextTime = millis();

  // ---- barometer ---------------------------------------------------------
  // baro.read() blocks while the chip converts.  About 1 ms at the library's
  // default oversampling, so 25 Hz is comfortable; the flight firmware
  // replaces this with a non-blocking version.
  baro.read();
  float tempC = baro.getTemperature();
  float hpa   = baro.getPressure();

  altPrev = alt;
  alt = altitudeFrom(hpa);

  // Differentiating a noisy signal amplifies the noise, so smooth it.
  // 0.88/0.12 at 25 Hz is the same time constant 0.7/0.3 gave at 10 Hz.
  float rawVel = (alt - altPrev) * (1000.0 / SAMPLE_MS);
  vel = 0.88 * vel + 0.12 * rawVel;

  noiseBuf[noiseIdx] = alt;
  noiseIdx++;
  if (noiseIdx >= NOISE_N) { noiseIdx = 0; noiseFull = true; }

  if (alt > altPeak) altPeak = alt;

  // ---- IMU ---------------------------------------------------------------
  imu.read();

  // A clipped reading is not a large reading, it is an unknown one.  Flag it
  // rather than quietly reporting the ceiling as if it were a measurement.
  if (abs(imu.a.x) > SAT_COUNT || abs(imu.a.y) > SAT_COUNT || abs(imu.a.z) > SAT_COUNT) accClip = true;
  if (abs(imu.g.x) > SAT_COUNT || abs(imu.g.y) > SAT_COUNT || abs(imu.g.z) > SAT_COUNT) gyrClip = true;

  float ax = imu.a.x * ACC_G_PER_LSB;
  float ay = imu.a.y * ACC_G_PER_LSB;
  float az = imu.a.z * ACC_G_PER_LSB;
  float aMag = sqrt(ax * ax + ay * ay + az * az);

  float gx = (imu.g.x - gyroBiasX) * GYR_DPS_PER_LSB;
  float gy = (imu.g.y - gyroBiasY) * GYR_DPS_PER_LSB;
  float gz = (imu.g.z - gyroBiasZ) * GYR_DPS_PER_LSB;
  float gMag = sqrt(gx * gx + gy * gy + gz * gz);

  // Tilt from vertical, assuming Z is the rocket's long axis.  Held upright,
  // az should read about +1.00 g and tilt about 0 degrees.
  float tilt = 0;
  if (aMag > 0.1) {
    float ratio = az / aMag;
    if (ratio >  1.0) ratio =  1.0;
    if (ratio < -1.0) ratio = -1.0;
    tilt = acos(ratio) * 57.29578;
  }

  if (aMag > gPeak)    gPeak = aMag;
  if (gMag > gyroPeak) gyroPeak = gMag;

  // ---- output ------------------------------------------------------------
  // Viewer frame.  millis() goes last so older parsers that read the first
  // eight fields are unaffected; it lets the viewer use the board's own
  // timing instead of guessing from USB arrival times, which arrive in bursts.
  if (vizMode) {
    Serial.print(F("V,"));
    Serial.print(ax, 4);  Serial.print(',');
    Serial.print(ay, 4);  Serial.print(',');
    Serial.print(az, 4);  Serial.print(',');
    Serial.print(gx, 2);  Serial.print(',');
    Serial.print(gy, 2);  Serial.print(',');
    Serial.print(gz, 2);  Serial.print(',');
    Serial.print(alt, 3); Serial.print(',');
    Serial.print(vel, 3); Serial.print(',');
    Serial.println(millis());
    return;
  }

  if (csvMode) {
    Serial.print(alt, 3);   Serial.print(',');
    Serial.print(vel, 2);   Serial.print(',');
    Serial.print(aMag, 3);  Serial.print(',');
    Serial.println(gMag, 1);
    return;
  }

  pad(tempC, 6, 2);
  pad(hpa,   9, 2);
  pad(alt,   8, 2);
  pad(vel,   7, 2);
  pad(altNoise() * 100.0, 6, 1);
  Serial.print(F("  |"));
  pad(ax,   6, 2);
  pad(ay,   6, 2);
  pad(az,   6, 2);
  pad(aMag, 6, 2);
  pad(tilt, 6, 1);
  Serial.println();

  // ---- interpretation, every 5 seconds -----------------------------------
  summaryCounter++;
  if (summaryCounter >= SUMMARY_N) {
    summaryCounter = 0;
    float noise = altNoise();

    Serial.println(F("  ---------------------------------------------------------------"));

    Serial.print(F("  altitude noise (1 sigma): "));
    Serial.print(noise * 100.0, 1);
    Serial.print(F(" cm   ->  set APOGEE_DROP_M near "));
    Serial.println(noise * 3.0, 2);

    Serial.print(F("  peaks:  alt "));
    Serial.print(altPeak, 2);
    Serial.print(F(" m    accel "));
    Serial.print(gPeak, 2);
    Serial.print(F(" g    gyro "));
    Serial.print(gyroPeak, 0);
    Serial.println(F(" dps"));

    Serial.print(F("  rest check: |a| should be 1.00 g, reading "));
    Serial.print(aMag, 3);
    if (fabs(aMag - 1.0) > 0.05) Serial.println(F("  <-- OFF, check scale or wiring"));
    else                          Serial.println(F("  OK"));

    Serial.print(F("  drift check: |rate| at rest should be ~0, reading "));
    Serial.print(gMag, 2);
    if (gMag > 2.0) Serial.println(F(" dps  <-- press b while still"));
    else            Serial.println(F(" dps  OK"));

    // Ceilings are reported as a fraction of the range actually in use, so
    // these stay correct if you widen a range later.
    if (gPeak > ACC_RANGE_G * 0.95) {
      Serial.print(F("  NOTE: accel near the +/-"));
      Serial.print(ACC_RANGE_G);
      Serial.println(F(" g ceiling"));
    }
    if (gyroPeak > GYR_RANGE_DPS * 0.95) {
      Serial.print(F("  NOTE: gyro near the +/-"));
      Serial.print(GYR_RANGE_DPS);
      Serial.println(F(" dps ceiling"));
    }
    if (accClip) Serial.println(F("  CLIPPED: accelerometer hit full scale - readings were invalid"));
    if (gyrClip) Serial.println(F("  CLIPPED: gyro hit full scale - attitude estimate is unreliable"));

    Serial.println(F("  ---------------------------------------------------------------"));
  }
}

//  -- END OF FILE --
