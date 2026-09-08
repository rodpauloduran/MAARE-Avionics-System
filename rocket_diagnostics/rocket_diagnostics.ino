// =============================================================================
//  ROCKET AVIONICS — BENCH DIAGNOSTICS
//  MS5611 barometer + LSM6 accel/gyro + ADXL375 high-g + SAM-M8Q GPS
//                                        ->  Serial Monitor / viewer
//
//  Reports interpreted values — degC, hPa, metres AGL, m/s, g, degrees tilt —
//  rather than raw counts, so a wrong reading is obvious on sight.  The most
//  useful single output is the running 1-sigma altitude noise, which is what
//  sets APOGEE_DROP_M and therefore how far past apogee deployment fires.
//
//  DESIGN RULES THIS SKETCH FOLLOWS
//  --------------------------------
//  1. SCALE FACTORS ARE READ BACK FROM THE CHIP, never hard-coded.  A constant
//     is a silent claim about a register you cannot see; a readback makes a
//     wrong range show up as a wrong PRINTED range at boot.  Note that
//     imu.enableDefault() selects +/-245 dps, so pairing it with the 0.035
//     dps/LSB constant that belongs to +/-1000 dps reports every rate 4x high
//     and saturates silently above 245 dps.
//     The same rule now covers the ADXL375 data rate and the GPS dynamic
//     model — see startHighG() and startGps().
//  2. THE BAROMETER IS STARTED WITH reset(), NOT begin().  On Mbed cores
//     begin() fails to detect a working MS5611 — see the note in setup().
//  3. GYRO BIAS is measured at startup and subtracted.  Command 'b' redoes it
//     as the board warms.
//  4. SATURATION IS DETECTED and reported on every sensor, separately from the
//     range-ceiling warning.  A clipped reading is not a large reading, it is
//     an unknown one.  Each part has its OWN ceiling: the LSM6 saturates at
//     int16 (32767), the ADXL375 at 13-bit (4095).  Sharing one constant
//     between them would mean high-g clipping is never detected at all.
//  5. TIMING CONSTANTS ARE TIED TO THE 25 Hz SAMPLE RATE — noise window,
//     summary interval and velocity filter all move if the rate changes.
//     Blocking calls resync the schedule rather than burst to catch up.
//  6. NOTHING IN THE LOOP MAY BLOCK FOR LONG.  The GPS is the live hazard
//     here: the u-blox library's default poll waits up to 1100 ms, which is
//     27 missed samples at 25 Hz.  setAutoPVT makes getPVT() return
//     immediately — see startGps().
//  7. A MISSING SENSOR IS REPORTED, NOT FATAL, for the two parts added last
//     (high-g, GPS).  The barometer and IMU still halt, because every derived
//     number on the display comes from them.
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
//      ADXL375 (Adafruit)        SAM-M8Q (SparkFun)
//        VIN -> 3V3                3V3 -> 3V3
//        GND -> GND                GND -> GND
//        SDA -> GP4                SDA -> GP4
//        SCL -> GP5                SCL -> GP5
//        addr jumper open = 0x53   u-blox DDC = 0x42
//                                  antenna faces sky, nothing metallic above
//
//  PULL-UPS: five breakouts now share this bus, each carrying its own.  In
//  parallel they can over-drive it at 400 kHz.  If the bus misbehaves, remove
//  pull-ups from all but one board — start with the GY-63, whose clones often
//  use 2.2k.
//
//  Serial commands (115200 baud):
//    z   re-zero ground pressure to here
//    b   re-measure gyro bias   (hold the board still)
//    r   reset the peak trackers
//    c   toggle CSV mode for the Arduino Serial Plotter
//    v   toggle telemetry stream for the attitude viewer
//    g   print GPS status now
//    h   reprint the column header
//
//  TELEMETRY WIRE FORMAT (command 'v').  Shared with the ground station so the
//  viewer has one parser for both transports:
//
//    V,ax,ay,az,gx,gy,gz,alt,vel,millis,hg,fix,sats,lat,lon,hacc
//
//    ax..az   g          gx..gz  dps        alt  m AGL     vel  m/s
//    millis   board clock, ms
//    hg       high-g magnitude in g, or -1 if the ADXL375 is not fitted
//    fix      u-blox fix type 0-5, or -1 if the GPS is not fitted
//    sats     satellites used in the solution
//    lat,lon  decimal degrees, 0 when there is no fix
//    hacc     the receiver's OWN horizontal accuracy estimate, metres,
//             or -1 with no fix / no GPS
//
//  Fields 10-15 were appended, never inserted, so a parser that reads only the
//  first eight or nine fields keeps working unchanged.  -1 rather than 0 marks
//  "not fitted", because 0 is a legal reading for all three of hg, fix and a
//  position on the equator.
//
//  WHY hacc IS WORTH A FIELD.  A position with no accuracy figure beside it
//  cannot be argued with.  Five satellites through a window will hold a 3D fix
//  and wander hundreds of metres, and nothing else in the output says so --
//  the fix type still reads 3D the whole time.  hAcc is already in the NAV-PVT
//  message the receiver sends anyway, so carrying it costs nothing and turns
//  "238 m from the pad" into a claim you can check.
// =============================================================================

#include <Wire.h>
#include "MS5611.h"
#include <LSM6.h>
#include <Adafruit_ADXL375.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

MS5611 baro(0x77);
LSM6   imu;
Adafruit_ADXL375 hga(375);          // sensor id is arbitrary, it only tags events
SFE_UBLOX_GNSS   gnss;

// -----------------------------------------------------------------------------
//  Sensor ranges.
//
//  208 Hz is comfortably above the 25 Hz sample rate.  +/-1000 dps is chosen
//  because a hand flip on the bench already exceeds +/-245, let alone a real
//  flight.  +/-2 g stays for now: it is the right choice for checking that the
//  accelerometer reads a clean 1.00 g at rest, and it is deliberately the FIRST
//  thing you will saturate, so you find out early rather than in the air.
//
//  For flight, widen the accelerometer (CTRL1_XL) or lean on the ADXL375, which
//  is now fitted and does not saturate until 200 g.  The readback below means
//  you only change the register.
// -----------------------------------------------------------------------------
const uint8_t CFG_CTRL1_XL = 0b01010000;   // 208 Hz, +/-2 g
const uint8_t CFG_CTRL2_G  = 0b01011000;   // 208 Hz, +/-1000 dps

//  Filled in by applyImuConfig() from what the chip actually reports.
float ACC_G_PER_LSB   = 0.0;
float GYR_DPS_PER_LSB = 0.0;
int   ACC_RANGE_G     = 0;
int   GYR_RANGE_DPS   = 0;

// -----------------------------------------------------------------------------
//  ADXL375 scale.
//
//  Unlike the LSM6 there is NO range register to read back — the part is fixed
//  at +/-200 g in silicon, and the library's setRange()/getRange() are
//  deliberate no-ops.  So the scale factor here is a property of the part
//  number rather than of a register, and rule 1 applies instead to the thing
//  that IS configurable: the output data rate.
//
//  49 mg/LSB (datasheet: 20.5 LSB/g).  begin() puts the part in FULL_RES, so
//  the output is 13-bit two's complement sign-extended into int16 — full scale
//  is about +/-4095 counts, NOT 32767.
// -----------------------------------------------------------------------------
const float HG_G_PER_LSB   = 0.049;
const float HG_RANGE_G     = 200.0;
int         HG_RATE_HZ     = 0;      // decoded from BW_RATE at boot

const int   SAMPLE_MS   = 40;             // 25 Hz
const int   NOISE_N     = 100;            // ~4 s of altitude at 25 Hz
const int   SUMMARY_N   = 125;            // ~5 s at 25 Hz
const int   BIAS_N      = 250;            // gyro bias samples

//  Raw counts this close to full scale mean the reading is clipped and the
//  true value is unknown.  Each part has its own ceiling — see design rule 4.
const int16_t SAT_COUNT    = 32000;       // LSM6, int16 full scale
const int16_t HG_SAT_COUNT = 4000;        // ADXL375, 13-bit full scale (~196 g)

//  The GPS is polled far slower than the sensor loop.  It solves at 5 Hz and
//  nothing it reports changes faster than walking pace.
const uint32_t GPS_POLL_MS = 1000;

// ------------------------------- state ---------------------------------------
float groundHpa = 1013.25;
float alt       = 0.0;
float altPrev   = 0.0;
float vel       = 0.0;

float altPeak   = 0.0;
float gPeak     = 0.0;
float gyroPeak  = 0.0;
float hgPeak    = 0.0;

float gyroBiasX = 0.0, gyroBiasY = 0.0, gyroBiasZ = 0.0;

float noiseBuf[NOISE_N];
int   noiseIdx  = 0;
bool  noiseFull = false;

bool     csvMode   = false;
bool     vizMode   = false;
bool     accClip   = false;
bool     gyrClip   = false;
bool     hgClip    = false;
uint32_t nextTime  = 0;
int      summaryCounter = 0;

//  Present-or-not for the two parts added last.  Everything that touches them
//  is guarded, so the sketch degrades to its previous behaviour if either is
//  unplugged, rather than hanging or printing a fabricated zero.
bool  hgPresent  = false;
bool  gpsPresent = false;

float hgMag = 0.0;                  // last high-g magnitude, g

uint32_t nextGpsPoll  = 0;
uint8_t  gpsFixType   = 0;
uint8_t  gpsSats      = 0;
int32_t  gpsLat       = 0;          // deg * 1e7
int32_t  gpsLon       = 0;          // deg * 1e7
int32_t  gpsAltMslMm  = 0;
int32_t  gpsHAccMm    = -1;        // receiver's own horizontal accuracy, mm
bool     gpsEverFixed = false;
uint32_t gpsLastFixMs = 0;

// =============================================================================
//  helpers
// =============================================================================

// Right-align a float in a fixed column width using only Serial.print().
// Avoids printf and dtostrf entirely, both of which have portability traps.
//
// The width is computed from the ROUNDED value, not the raw one.
// Serial.print() rounds to the requested number of decimals, so 9.996 at 2 dp
// prints "10.00" — five characters where a digit count taken from the
// un-rounded value predicts four, and the whole column walks left by one.
// Rounding first makes the prediction agree with what is actually printed.
void pad(float value, int width, int decimals) {
  float scale = 1.0;
  for (int i = 0; i < decimals; i++) scale *= 10.0;
  float rounded = floor(fabs(value) * scale + 0.5) / scale;
  if (value < 0) rounded = -rounded;

  int len = decimals + (decimals > 0 ? 1 : 0);
  if (rounded < 0) len++;
  long whole = (long)fabs(rounded);
  int digits = 1;
  while (whole >= 10) { whole /= 10; digits++; }
  len += digits;
  for (int i = len; i < width; i++) Serial.print(' ');
  Serial.print(value, decimals);
}

// Right-aligned placeholder for a column whose sensor is not fitted.  Printing
// a zero there would be a fabricated measurement; a dash is unambiguous.
void padAbsent(int width) {
  for (int i = 3; i < width; i++) Serial.print(' ');
  Serial.print(F("---"));
}

// -----------------------------------------------------------------------------
//  Configure the IMU, then ask the chip what it is actually set to and derive
//  the conversion constants from the answer.
//
//  This matters more than it looks.  A hard-coded scale factor is a silent
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

  // Both registers are checked.  Warning on only one of them would let a
  // failed accelerometer write through in silence, which is the exact class
  // of fault this readback exists to catch.
  if (c1 != CFG_CTRL1_XL) {
    Serial.println(F("  WARNING: CTRL1_XL did not take. Check I2C wiring."));
  }
  if (c2 != CFG_CTRL2_G) {
    Serial.println(F("  WARNING: CTRL2_G did not take. Check I2C wiring."));
  }
}

// -----------------------------------------------------------------------------
//  ADXL375 high-g accelerometer.
//
//  begin() is safe to use here, unlike the MS5611's.  Adafruit_BusIO's address
//  detection carries an explicit `#ifdef ARDUINO_ARCH_MBED` that writes the
//  dummy byte before endTransmission(), so the Mbed zero-length-probe defect
//  does not bite this part.  That is a property of THIS library, not of the
//  bus — worth re-checking for any new device rather than assumed.
//
//  Returns false if the part does not answer; the caller keeps running.
// -----------------------------------------------------------------------------
bool startHighG() {
  if (!hga.begin(0x53)) return false;

  // The default 100 Hz smears a 0.2-0.4 s burn into a handful of samples.
  hga.setDataRate(ADXL343_DATARATE_800_HZ);
  delay(10);

  // Readback, same rule as the IMU.  BW_RATE's low nibble is the rate code and
  // the rate is 3.125 Hz * 2^(code-5), which is exact in integers from code 8
  // upward: 25 Hz << (code - 8).  Decoding from the register rather than
  // trusting the write means a failed write prints a wrong rate.
  uint8_t bw = hga.readRegister(ADXL3XX_REG_BW_RATE) & 0x0F;
  HG_RATE_HZ = (bw >= 8) ? (25 << (bw - 8)) : 0;

  Serial.print(F("  BW_RATE  = 0x")); Serial.print(bw, HEX);
  Serial.print(F("  ->  "));
  if (HG_RATE_HZ) {
    Serial.print(HG_RATE_HZ);
    Serial.println(F(" Hz output data rate"));
  } else {
    Serial.println(F("below 25 Hz - far too slow, did the write fail?"));
  }

  Serial.print(F("  range    = +/-"));
  Serial.print(HG_RANGE_G, 0);
  Serial.print(F(" g fixed in silicon, "));
  Serial.print(HG_G_PER_LSB, 3);
  Serial.println(F(" g/LSB (no range register to read back)"));

  if (bw != ADXL343_DATARATE_800_HZ) {
    Serial.println(F("  WARNING: data rate did not take. Check I2C wiring."));
  }
  return true;
}

// -----------------------------------------------------------------------------
//  SAM-M8Q GPS.
//
//  Two things matter here and both are easy to get wrong:
//
//  1. THE DYNAMIC MODEL.  The default assumes a ground vehicle and rejects a
//     rocket trajectory as implausible, dropping lock exactly when it is
//     needed.  It is read back below, because a setDynamicModel() that failed
//     looks identical to one that worked right up until you are airborne.
//  2. setAutoPVT.  Without it, getPVT() polls and BLOCKS for up to 1100 ms.
//     With it, the module pushes solutions on its own schedule and getPVT()
//     returns immediately — which is what keeps rule 6 true.
//
//  Returns false if the module does not answer; the caller keeps running.
// -----------------------------------------------------------------------------
bool startGps() {
  if (!gnss.begin(Wire, 0x42)) return false;

  gnss.setI2COutput(COM_TYPE_UBX);            // binary UBX, no NMEA chatter
  gnss.setNavigationFrequency(5);             // 5 Hz solutions
  gnss.setDynamicModel(DYN_MODEL_AIRBORNE1g); // <- the line people miss
  gnss.setAutoPVT(true);                      // <- keeps getPVT() non-blocking
  gnss.saveConfiguration();                   // survive a power cycle

  uint8_t dm   = gnss.getDynamicModel();
  uint8_t freq = gnss.getNavigationFrequency();

  Serial.print(F("  dynamic model = "));
  if (dm == DYN_MODEL_UNKNOWN)              Serial.println(F("query FAILED"));
  else if (dm == DYN_MODEL_AIRBORNE1g)      Serial.println(F("airborne <1g   OK"));
  else { Serial.print(dm); Serial.println(F("   <-- NOT airborne <1g")); }

  Serial.print(F("  nav rate      = "));
  Serial.print(freq);
  Serial.println(F(" Hz"));

  if (dm != DYN_MODEL_AIRBORNE1g) {
    Serial.println(F("  WARNING: a ground-vehicle model rejects rocket trajectories."));
  }

  // A cold start with no almanac takes 30-60 s and needs a clear sky view.
  // "No fix" at boot on an indoor bench is expected, not a fault — say so, or
  // somebody spends an afternoon debugging a working receiver.
  Serial.println(F("  no fix at boot is normal - cold start is 30-60 s, outdoors"));
  return true;
}

// Pull the latest solution.  Cheap, and non-blocking because of setAutoPVT.
void pollGps() {
  if (!gpsPresent) return;
  if (!gnss.getPVT()) return;               // nothing new since the last call

  gpsFixType  = gnss.getFixType();
  gpsSats     = gnss.getSIV();
  gpsLat      = gnss.getLatitude();
  gpsLon      = gnss.getLongitude();
  gpsAltMslMm = gnss.getAltitudeMSL();
  // Straight out of the same NAV-PVT message as everything above it.
  gpsHAccMm   = gnss.getHorizontalAccEst();

  if (gpsFixType >= 3) { gpsEverFixed = true; gpsLastFixMs = millis(); }
}

// Degrees * 1e7 as a decimal, without printf.  Seven decimal places is ~1 cm,
// far finer than this receiver, but truncating here would be a silent loss.
void printDegrees(int32_t e7) {
  if (e7 < 0) { Serial.print('-'); e7 = -e7; }
  Serial.print(e7 / 10000000L);
  Serial.print('.');
  long frac = e7 % 10000000L;
  for (long d = 1000000L; d > 1; d /= 10) { if (frac < d) Serial.print('0'); }
  Serial.print(frac);
}

void printGpsStatus() {
  Serial.print(F("  GPS: "));
  if (!gpsPresent) { Serial.println(F("not fitted")); return; }

  Serial.print(F("fix "));
  switch (gpsFixType) {
    case 0:  Serial.print(F("none"));           break;
    case 1:  Serial.print(F("dead-reckoning")); break;
    case 2:  Serial.print(F("2D"));             break;
    case 3:  Serial.print(F("3D"));             break;
    case 4:  Serial.print(F("GNSS+DR"));        break;
    case 5:  Serial.print(F("time-only"));      break;
    default: Serial.print(gpsFixType);          break;
  }
  Serial.print(F("   sats "));
  Serial.print(gpsSats);

  if (gpsFixType >= 2) {
    Serial.print(F("   "));
    printDegrees(gpsLat);
    Serial.print(F(", "));
    printDegrees(gpsLon);
    Serial.print(F("   MSL "));
    Serial.print(gpsAltMslMm / 1000.0, 1);
    Serial.print(F(" m"));

    // The number that tells you whether to believe the two above it.
    Serial.print(F("   +/-"));
    Serial.print(gpsHAccMm / 1000.0, 1);
    Serial.print(F(" m"));
    if (gpsSats < 6) {
      Serial.print(F("   <-- only "));
      Serial.print(gpsSats);
      Serial.print(F(" sats, geometry is poor"));
    }
  } else if (gpsEverFixed) {
    // Distinguishing "never had a fix" from "had one and lost it" is the
    // difference between a sky-view problem and an antenna or power problem.
    Serial.print(F("   lock LOST "));
    Serial.print((millis() - gpsLastFixMs) / 1000);
    Serial.print(F(" s ago"));
  } else {
    Serial.print(F("   acquiring - needs sky view"));
  }
  Serial.println();
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
    // 6 ms, not 4.  The gyro runs at 208 Hz, a 4.8 ms period, so a 4 ms delay
    // re-reads the same sample often enough to weight the average toward
    // whichever readings happen to be duplicated.  Sampling slower than the
    // ODR keeps every sample independent, which is the point of averaging.
    delay(6);
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
  Serial.println(F("============================================================================"));
  Serial.println(F("  temp   press     alt    vel  noise |    ax    ay    az   |a|  tilt |    hg"));
  Serial.println(F("   C      hPa       m     m/s    cm  |     g     g     g     g   deg |     g"));
  Serial.println(F("============================================================================"));
}

// =============================================================================
//  setup
// =============================================================================
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) { }

  Serial.println();
  Serial.println(F("=== ROCKET AVIONICS BENCH DIAGNOSTICS ==="));
  Serial.println();

  Wire.begin();

  // ---- I2C scan.  Always do this first.  If an address does not show up
  //      here, no amount of driver code will make that sensor work.
  //      The Wire.write(0) is required on Mbed cores, where a zero-length
  //      endTransmission() issues a read and most devices will not ACK it.
  //
  //      Side effect: this writes a zero byte to every address on the bus.
  //      Re-checked now that five parts are fitted — on the MS5611 it is a
  //      harmless ADC-read command, on the ADXL375 register 0 is the
  //      read-only device ID, and on the u-blox it only moves the DDC address
  //      pointer.  Check again if a sixth device joins the bus. ------------
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

  // ---- high-g accelerometer.  Not fatal: the column degrades to dashes. ---
  Serial.println();
  if (startHighG()) {
    Serial.println(F("ADXL375 found and initialised"));
    hgPresent = true;
  } else {
    Serial.println(F("ADXL375 NOT found - the hg column will read ---"));
    Serial.println(F("  - address jumper open = 0x53, bridged = 0x1D"));
  }

  // ---- GPS.  Not fatal either. -------------------------------------------
  Serial.println();
  if (startGps()) {
    Serial.println(F("SAM-M8Q found and configured"));
    gpsPresent = true;
  } else {
    Serial.println(F("SAM-M8Q NOT found - GPS lines will read 'not fitted'"));
    Serial.println(F("  - u-blox DDC is 0x42; give it a moment after power-up"));
  }

  Serial.println();
  calibrateGyro();
  Serial.println();
  zeroGround();

  Serial.println();
  Serial.println(F("Commands:  z = re-zero   b = gyro bias   r = reset peaks"));
  Serial.println(F("           c = CSV       v = viz stream  g = GPS   h = header"));
  printHeader();

  nextTime    = millis();
  nextGpsPoll = millis();
}

// =============================================================================
//  loop
// =============================================================================
void loop() {

  // ---- commands ----------------------------------------------------------
  while (Serial.available()) {
    char c = Serial.read();
    // The header is for the human-readable table only.  Reprinting it while
    // the viewer is streaming injects non-telemetry lines into the feed the
    // viewer is parsing -- harmless, since it ignores anything that is not a
    // V line, but it is noise in a stream that should be clean.  The progress
    // text from these two DOES stay: it lands in the viewer's raw line, which
    // is useful feedback that the command was received.
    if (c == 'z' || c == 'Z') { Serial.println(); zeroGround(); if (!vizMode) printHeader(); }
    else if (c == 'b' || c == 'B') { Serial.println(); calibrateGyro(); if (!vizMode) printHeader(); }
    else if (c == 'r' || c == 'R') {
      altPeak = alt; gPeak = 0; gyroPeak = 0; hgPeak = 0;
      accClip = false; gyrClip = false; hgClip = false;
      Serial.println(F("-- peaks reset --"));
    }
    else if (c == 'c' || c == 'C') {
      csvMode = !csvMode;
      Serial.println();
      if (csvMode) Serial.println(F("alt_m,vel_ms,accel_g,gyro_dps,highg_g"));
      else printHeader();
    }
    else if (c == 'v' || c == 'V') {
      vizMode = !vizMode;
      Serial.println();
      if (!vizMode) printHeader();
    }
    else if (c == 'g' || c == 'G') { Serial.println(); printGpsStatus(); }
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

  // ---- high-g accelerometer ----------------------------------------------
  // Its own saturation ceiling: 13-bit, not int16.  Reusing the LSM6 constant
  // here would mean high-g clipping is never reported at all.
  if (hgPresent) {
    int16_t hx = hga.getX();
    int16_t hy = hga.getY();
    int16_t hz = hga.getZ();

    if (abs(hx) > HG_SAT_COUNT || abs(hy) > HG_SAT_COUNT || abs(hz) > HG_SAT_COUNT) hgClip = true;

    float hgx = hx * HG_G_PER_LSB;
    float hgy = hy * HG_G_PER_LSB;
    float hgz = hz * HG_G_PER_LSB;
    hgMag = sqrt(hgx * hgx + hgy * hgy + hgz * hgz);

    if (hgMag > hgPeak) hgPeak = hgMag;
  }

  // ---- GPS.  Slow cadence, and non-blocking because of setAutoPVT. -------
  if ((long)(millis() - nextGpsPoll) >= 0) {
    nextGpsPoll = millis() + GPS_POLL_MS;
    pollGps();
  }

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
  //
  // Extended with high-g and GPS by APPENDING fields, so the eight- and
  // nine-field parsers that predate them are unaffected.  The ground station's
  // synthetic producer emits the same fourteen fields, which is what lets the
  // viewer keep a single parser across both transports.
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
    Serial.print(millis());

    // -1 marks "not fitted" for both parts.  Zero would be ambiguous: 0.00 g
    // is a legal high-g reading in freefall, fix type 0 is a legal "no fix",
    // and 0,0 is a real position in the Gulf of Guinea.
    Serial.print(',');
    if (hgPresent) Serial.print(hgMag, 2); else Serial.print(-1);

    Serial.print(',');
    if (!gpsPresent) {
      Serial.print(F("-1,0,0,0,-1"));
    } else {
      Serial.print(gpsFixType); Serial.print(',');
      Serial.print(gpsSats);    Serial.print(',');
      if (gpsFixType >= 2) {
        printDegrees(gpsLat); Serial.print(',');
        printDegrees(gpsLon); Serial.print(',');
        Serial.print(gpsHAccMm / 1000.0, 2);
      } else {
        Serial.print(F("0,0,-1"));
      }
    }
    Serial.println();
    return;
  }

  if (csvMode) {
    Serial.print(alt, 3);   Serial.print(',');
    Serial.print(vel, 2);   Serial.print(',');
    Serial.print(aMag, 3);  Serial.print(',');
    Serial.print(gMag, 1);  Serial.print(',');
    // The Serial Plotter needs a number in every column, so an absent high-g
    // part logs 0 here.  The flat trace is the tell; the boot banner is where
    // "not fitted" is actually stated.
    Serial.println(hgPresent ? hgMag : 0.0f, 2);
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
  Serial.print(F("  |"));
  if (hgPresent) pad(hgMag, 6, 2);
  else           padAbsent(6);
  Serial.println();

  // ---- interpretation, every 5 seconds -----------------------------------
  summaryCounter++;
  if (summaryCounter >= SUMMARY_N) {
    summaryCounter = 0;
    float noise = altNoise();

    Serial.println(F("  ----------------------------------------------------------------------"));

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
    Serial.print(F(" dps"));
    if (hgPresent) {
      Serial.print(F("    high-g "));
      Serial.print(hgPeak, 1);
      Serial.print(F(" g"));
    }
    Serial.println();

    Serial.print(F("  rest check: |a| should be 1.00 g, reading "));
    Serial.print(aMag, 3);
    if (fabs(aMag - 1.0) > 0.05) Serial.println(F("  <-- OFF, check scale or wiring"));
    else                          Serial.println(F("  OK"));

    Serial.print(F("  drift check: |rate| at rest should be ~0, reading "));
    Serial.print(gMag, 2);
    if (gMag > 2.0) Serial.println(F(" dps  <-- press b while still"));
    else            Serial.println(F(" dps  OK"));

    // High-g cross-check.  Two things make this a LOOSE check, not a tight one:
    //
    //   1. At rest the part has only ~20 counts of signal (1 g at 0.049
    //      g/LSB), so +/-0.05 g of wobble is quantisation, not noise.
    //   2. Far more importantly, the ADXL375's ZERO-G OFFSET is specified in
    //      whole g, not milli-g -- it is a +/-200 g part and the offset scales
    //      with the range.  Measured on this bench: 0.73 g at rest while the
    //      LSM6 read 1.01 g.  That gap is the part behaving to specification,
    //      and an earlier +/-0.15 g tolerance called a healthy sensor faulty.
    //
    // So the band below discriminates the faults that actually matter -- a
    // dead axis reads ~0, and an ADXL345 in this footprint reads about 12x low
    // (~0.08 g) -- while leaving normal offset alone.  Trimming the offset out
    // properly needs a per-axis calibration against a known orientation, which
    // is an open item: a magnitude check cannot separate offset from scale.
    if (hgPresent) {
      Serial.print(F("  high-g check: |a| at rest, reading "));
      Serial.print(hgMag, 2);
      Serial.print(F(" g  (LSM6 says "));
      Serial.print(aMag, 2);
      Serial.print(F(" g)"));
      if (hgMag < 0.35 || hgMag > 2.0) {
        Serial.println(F("  <-- OFF, dead axis or wrong part?"));
      } else if (fabs(hgMag - aMag) > 0.15) {
        Serial.print(F("  offset "));
        Serial.print(hgMag - aMag, 2);
        Serial.println(F(" g, normal for this part - not a fault"));
      } else {
        Serial.println(F("  OK"));
      }
    }

    printGpsStatus();

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
    if (hgPresent && hgPeak > HG_RANGE_G * 0.95) {
      Serial.print(F("  NOTE: high-g near the +/-"));
      Serial.print(HG_RANGE_G, 0);
      Serial.println(F(" g ceiling"));
    }
    if (accClip) Serial.println(F("  CLIPPED: accelerometer hit full scale - readings were invalid"));
    if (gyrClip) Serial.println(F("  CLIPPED: gyro hit full scale - attitude estimate is unreliable"));
    if (hgClip)  Serial.println(F("  CLIPPED: high-g hit full scale - the peak is a floor, not a value"));

    Serial.println(F("  ----------------------------------------------------------------------"));
  }
}

//  -- END OF FILE --
