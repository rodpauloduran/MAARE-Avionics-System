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
//      LS3040 buzzer
//        +      -> GP7      (design doc 9.1 specified GP13; GP7 is as built)
//        -      -> GND
//        A piezo at 3V3 is quiet but audible.  Drive it through an NPN or a
//        MOSFET from a higher rail if it needs to be heard across a field.
//
//      WIRED DOWNLINK to the ground station Pico W  (radio stand-in)
//        GP0 (TX, pin 1) -> Pico W GP1 (RX, pin 2)
//        GND             -> Pico W GND     not optional: no shared ground,
//                                          no signal reference, garbage
//        GP1 (RX, pin 2) <- Pico W GP0     the uplink: phone commands arrive
//                                          here, framed '!' lines only
//        Power each Pico from its OWN USB.  No other wires between them.
//
//      MG90D servo latch
//        signal -> GP6      (design doc 9.1 specified GP15; GP6 is as built)
//        V+     -> its OWN supply, not the Pico's 3V3 rail
//        GND    -> common with the Pico
//        220 uF across the servo supply.  Servo inrush is the classic cause of
//        unexplained resets mid-test, and a brown-out here reboots the board
//        holding the actuator.
//
//  PULL-UPS: five breakouts now share this bus, each carrying its own.  In
//  parallel they can over-drive it at 400 kHz.  If the bus misbehaves, remove
//  pull-ups from all but one board — start with the GY-63, whose clones often
//  use 2.2k.
//
//  Serial commands (115200 baud).  Single characters, dispatched on arrival:
//    z   re-zero ground pressure to here
//    b   re-measure gyro bias   (hold the board still)
//    r   reset the peak trackers
//    c   toggle CSV mode for the Arduino Serial Plotter
//    v   toggle telemetry stream for the attitude viewer
//    g   print GPS status now
//    h   reprint the column header
//
//  Servo latch commands.  These need NUMBERS, which a single character cannot
//  carry, so they are introduced with '!' and terminated by a newline.  The two
//  schemes are kept visibly separate so a truncated line can never be mistaken
//  for one of the single-character commands above.
//    !arm            permit the latch to move   (nothing moves on arming)
//    !safe           stop driving the pin
//    !fire           withdraw the pin           (armed only; latches)
//    !latch          re-engage the pin          (armed only; clears fired)
//    !us <n>         drive to a raw pulse width (armed only; calibration)
//    !pos <a> <b>    set latched / released pulse widths, microseconds
//    !srv            print servo status
//    !hb             heartbeat; resets the 3 s deadman while armed
//
//  Buzzer commands (LS3040 on GP7):
//    !buz off|chirp|double|locate|alarm    select a pattern
//    !beep <hz> <ms>                       one-shot tone, for checking it works
//    !bz                                   print buzzer status
//    !hgcal [x+|x-|y+|y-|z+|z-|apply|clear|show]
//                                          ADXL375 zero-g offset trim
//
//  TELEMETRY WIRE FORMAT (command 'v').  Shared with the ground station so the
//  viewer has one parser for both transports:
//
//    V,ax,ay,az,gx,gy,gz,alt,vel,millis,hg,fix,sats,lat,lon,hacc,srv,srvus,lsw
//
//    ax..az   g          gx..gz  dps        alt  m AGL     vel  m/s
//    millis   board clock, ms
//    hg       high-g magnitude in g, or -1 if the ADXL375 is not fitted
//    fix      u-blox fix type 0-5, or -1 if the GPS is not fitted
//    sats     satellites used in the solution
//    lat,lon  decimal degrees, 0 when there is no fix
//    hacc     the receiver's OWN horizontal accuracy estimate, metres,
//             or -1 with no fix / no GPS
//    srv      servo latch: 0 safe, 1 armed, 2 armed and fired
//    srvus    pulse width COMMANDED to the latch, microseconds; 0 = not driven
//    lsw      latch switch: 0 released, 1 engaged, -1 no switch fitted.  The
//             only field here that is MEASURED rather than commanded, which is
//             the entire reason it exists
//
//  Fields 10-18 were appended, never inserted, so a parser that reads only the
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
#include <Servo.h>
#include <mbed.h>
#include <string.h>

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

// ---- zero-g offset trim -----------------------------------------------------
// The ADXL375's zero-g offset is specified in WHOLE g, not milli-g: it is a
// +/-200 g part and the offset scales with the range.  Measured on this bench,
// 0.73 g at rest against the LSM6's 1.01 g -- within specification, and
// irrelevant to the part's actual job of catching peaks, but it leaves the
// channel unusable for absolute magnitude near 1 g and makes the rest check a
// loose band rather than a real cross-check.
//
// The part can fix this itself.  OFSX/OFSY/OFSZ are hardware trim registers
// applied to the data before it reaches the output registers, so a corrected
// reading costs nothing at runtime.  The Adafruit library does not expose
// them; it does expose writeRegister(), and the register numbers come from
// Adafruit_ADXL343.h.
//
// The scale factor below is the datasheet's, but nothing depends on it being
// exact: `!hgcal apply` ADDS its correction to whatever is already in the
// register, having measured the residual WITH that value applied.  So it is a
// fixed-point iteration -- run it twice and the error falls by the square of
// however wrong this constant is.  A datasheet number used as a starting
// guess, not as a dependency.
const float HG_OFS_G_PER_LSB = 0.196;

// Calibration is not persistent: an RP2040 has no EEPROM, and writing flash at
// runtime to store three bytes is not a trade worth making.  `!hgcal apply`
// prints these three lines ready to paste; re-flash and the trim is permanent.
const int8_t HG_TRIM_X = 0;
const int8_t HG_TRIM_Y = 0;
const int8_t HG_TRIM_Z = 0;
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
//  SERVO LATCH  (GP6)
//
//  THIS IS A BENCH TEST ACTUATOR, NOT THE FLIGHT DEPLOYMENT PATH.  Flight
//  deployment belongs in the state machine on this board and must never depend
//  on a link (design doc 2.3, 5.4).  What this exists for is finding the
//  thresholds -- height, tilt, whatever else -- that later get compiled INTO
//  that state machine.  The viewer drives it so the numbers can be changed
//  without a reflash; that convenience is exactly why it must not fly.
//
//  DRIVEN BY THE Servo LIBRARY, which is confirmed working on hardware.
//
//    * The Mbed RP2040 core does NOT bundle a Servo library -- the design doc
//      said it did; core 4.6.0 ships only MRI, PDM, SPI, Scheduler,
//      ThreadDebug, USBHID, USBMSD and Wire.  Install Servo from the Library
//      Manager; it declares mbed_rp2040 support and ships an mbed backend.
//    * mbed::PwmOut was tried first and is UNTESTED, not known-broken.  It was
//      only ever run against a servo that turned out to be faulty.  An earlier
//      comment here declared PwmOut broken on this core; that was wrong, and is
//      withdrawn.  A dead actuator made working code look broken -- the lesson
//      is to swap in a known-good unit before blaming firmware.
//
//  Consequence worth knowing: the pulse is generated from interrupts, not by
//  PWM hardware, so it carries some jitter and a little ISR load.  Fine for a
//  bench latch; worth re-checking if it ever shares a core with a 500 Hz
//  flight loop.
//
//  THE ONE DEVICE WHERE THE READBACK RULE CANNOT BE HONOURED.  Rule 1 of this
//  sketch is that configuration is read back from the chip.  A hobby servo has
//  no feedback path at all: servoNowUs below is what was COMMANDED, never what
//  the horn did.  A stalled, stripped or unpowered servo reports exactly the
//  same as a healthy one.  The honest fix is mechanical -- a limit or reed
//  switch on the latch confirming the pin actually withdrew -- and it is an
//  open item, not something firmware can paper over.
//
//  SAFETY, in the order it matters:
//    * At boot the pin is NOT DRIVEN.  Period is set, pulse width stays 0, so
//      no position is commanded until somebody arms it deliberately.
//    * Nothing moves while disarmed.  Arming is a separate, explicit step.
//    * A deadman disarms the latch if the host goes quiet.  On a bench an
//      unattended armed actuator is the hazard; in flight the opposite is true,
//      which is another reason this code is not the flight path.
//    * Firing latches.  It will not fire twice without an explicit re-latch,
//      mirroring the deployFired flag of 5.4.
// =============================================================================
// =============================================================================
//  WIRED DOWNLINK  (Serial1: GP0 = TX, GP1 = RX)
//
//  A temporary stand-in for the LoRa pair while the antenna pigtails are not
//  on hand: the same telemetry line, straight into the ground station's UART.
//  It exercises everything downstream of the radio -- the ground station's
//  producer, the SSE stream, staleness, the viewer -- and nothing about the
//  radio itself: no loss, no range, no 868 MHz, no 18-byte packet (6.2).
//
//  WHY 460800 BAUD, not 115200.  Serial1.write() on this core BLOCKS: it goes
//  through mbed::UnbufferedSerial and busy-waits on writeable().  The RP2040's
//  TX FIFO absorbs the first 32 bytes, and every byte after that waits for the
//  wire.  A telemetry line is ~100 bytes, so at 115200 the loop would stall
//  ~6 ms of every 40 ms frame; at 460800 it is ~1.5 ms.  (In the flight
//  firmware's 500 Hz loop even 1.5 ms is too much -- telemetry has to go out
//  from the normal-priority thread, exactly as 10.2 already says.)
// =============================================================================
const bool     LINK_ENABLED = true;
const uint32_t LINK_BAUD    = 460800;

// -----------------------------------------------------------------------------
//  THE UPLINK -- a deliberate decision, not an accident.
//
//  The wire was built transmit-only first, precisely so that ground control of
//  the latch could not arrive by accident inside a test rig.  It was then
//  asked for on purpose: arming, firing, zeroing and the rest from a phone,
//  through the ground station.  So the wire carries commands up as well, and
//  what arrives is constrained harder than the USB port is:
//
//    * FRAMED LINES ONLY.  From the wire, a bare byte means nothing.  Only
//      '!word args' terminated by a newline is acted on.  An unplugged RX pin
//      picks up noise, and a single noise byte that looked like 'z' or 'b'
//      would re-zero the barometer or start a 1.5 s blocking gyro average --
//      from nobody.  A noise burst spelling "!arm\n" is not a real risk.
//    * NO PRESENTATION COMMANDS.  'v', 'c' and 'h' only change what the USB
//      port prints; there is no '!' form of them, so the wire cannot send them.
//    * THE DEADMAN STILL RULES.  The phone must heartbeat like the USB viewer
//      does.  Lock the phone, background the tab, lose Wi-Fi: heartbeats stop,
//      and 3 s later the board disarms itself.  That is the property that makes
//      a remote arming path tolerable at all.
//
//  FOR FLIGHT, NONE OF THIS CARRIES OVER.  The flight system arms by a reed
//  switch and a magnet, physically (8.4), precisely so that nothing remote can
//  arm it; and deployment must never depend on a link (2.3).  This is a bench
//  rig with a servo on a desk.
// -----------------------------------------------------------------------------
struct CmdPort {
  char buf[48];
  int  len;
  bool active;
  bool overflow;       // line too long: discard it rather than act on a prefix
};
CmdPort usbCmd  = { {0}, 0, false, false };
CmdPort linkCmd = { {0}, 0, false, false };

// -----------------------------------------------------------------------------
//  Everything the board SAYS about its state goes to both the USB port and the
//  wire; the 25 Hz table stays on USB.  On the wire each message line is
//  prefixed "M," so the ground station can tell a message from a telemetry
//  frame ("V,") without guessing, and forward it to the phone's console.
// -----------------------------------------------------------------------------
class LinkTee : public Print {
  bool bol = true;                     // at the start of a line on the wire
public:
  using Print::write;
  size_t write(uint8_t c) override {
    Serial.write(c);
    if (LINK_ENABLED) {
      if (c == '\r') return 1;         // wire lines end in \n alone
      if (bol && c == '\n') return 1;  // an empty message is not a message
      if (bol) { Serial1.write('M'); Serial1.write(','); bol = false; }
      Serial1.write(c);
      if (c == '\n') bol = true;
    }
    return 1;
  }
  // Called before every telemetry frame: a message left unterminated would
  // otherwise swallow the frame that follows it.
  void endLine() {
    if (LINK_ENABLED && !bol) { Serial1.write('\n'); bol = true; }
  }
};
LinkTee Out;

const int      SERVO_PIN    = 6;
const uint16_t SERVO_MIN_US = 600;      // hard clamp; protects the gear train
const uint16_t SERVO_MAX_US = 2400;
const uint32_t SERVO_DEADMAN_MS = 3000; // host silence that disarms the latch

Servo latch;

uint16_t servoLatchedUs  = 1000;        // pin engaged
uint16_t servoReleasedUs = 2000;        // pin withdrawn
uint16_t servoNowUs      = 0;           // COMMANDED, not measured.  0 = idle
bool     servoArmed      = false;
bool     servoFired      = false;
uint32_t servoLastCmdMs  = 0;

// A commanded move takes real time to happen.  An MG90D covers a 1000-2000 us
// sweep in roughly 0.15-0.2 s unloaded, longer against a latch pin under
// friction.  Detaching before that cuts the pulse train and the horn simply
// stops -- it does not finish the move on momentum.  800 ms is margin.
const uint32_t SERVO_SETTLE_MS = 800;
uint32_t servoLastMoveMs = 0;
uint32_t servoDetachAtMs = 0;           // 0 = no detach pending

// ---- latch feedback ---------------------------------------------------------
// The one place the readback rule of this sketch cannot be honoured in
// firmware: a hobby servo has no feedback path, so a stalled, stripped or
// unpowered one reports exactly the same as a healthy one.  The fix is
// mechanical -- a switch the mechanism itself closes -- and this is the
// firmware waiting for it.  Set LATCH_SWITCH_PIN to a GP number once one is
// fitted and every commanded move is checked against it.
//
// WIRING, and the polarity is not arbitrary.  Switch between the pin and GND,
// closed when the latch is RELEASED.  INPUT_PULLUP, so:
//
//     LOW   switch closed   ->  released, and something physical says so
//     HIGH  switch open     ->  still engaged
//
// A disconnected switch, a broken wire or a pin never fitted all read HIGH,
// which is "still engaged" -- so they report a fire that did not happen rather
// than confirming one that did.  The wrong way round would let a snapped wire
// silently certify every release.
const int LATCH_SWITCH_PIN = -1;        // -1 = not fitted; no pin is touched

uint32_t latchConfirmAtMs = 0;          // 0 = no confirmation pending
int      latchExpect      = -1;         // what the switch should read by then

// -1 unknown / not fitted, 0 released, 1 engaged.
int latchSensed() {
  if (LATCH_SWITCH_PIN < 0) return -1;
  return digitalRead(LATCH_SWITCH_PIN) == LOW ? 0 : 1;
}

// Same, but only after five reads agree.  Used at the confirmation point,
// where a bouncing contact would otherwise decide whether the latch worked.
int latchSensedSettled() {
  if (LATCH_SWITCH_PIN < 0) return -1;
  int v = latchSensed();
  for (int k = 0; k < 4; k++) {
    delay(2);
    if (latchSensed() != v) return -1;  // still moving: refuse to judge
  }
  return v;
}

// =============================================================================
//  BUZZER  (GP7, LS3040 piezo, ~4 kHz resonant)
//
//  NOT DRIVEN BY tone(), DELIBERATELY.  The core's tone() works, and it leaks:
//  Tone::stop() does `pin = 0`, which nulls the DigitalOut POINTER rather than
//  writing the pin low, so the destructor's `delete pin` frees nothing.  Every
//  tone() call leaks one DigitalOut.  A locator beeping once a second would
//  leak for as long as the vehicle is lost, which is precisely when you cannot
//  afford it -- and it can leave the pin HIGH, so a piezo sits with DC across
//  it after the beep is supposed to have stopped.
//
//  So this does what tone() does -- toggle a DigitalOut from a Ticker -- with
//  the object allocated once.  Same mechanism, no leak, and the pin is driven
//  low explicitly when silent.  (Same mechanism as the Servo library's mbed
//  backend.  That is a pattern in the code, not a verdict on the PWM hardware:
//  software timing simply works on any pin.)
//
//  Patterns are STEP TABLES advanced from the main loop, never delay().  A
//  buzzer that blocks is a buzzer that costs you samples.
// =============================================================================
const int BUZZER_PIN = 7;
const uint16_t BUZ_HZ = 4000;           // LS3040 resonant; loudest here

mbed::DigitalOut *buzPin = nullptr;
mbed::Ticker      buzTicker;

// {frequency Hz, duration ms} pairs.  Frequency 0 is silence.  A duration of 0
// terminates a one-shot; looping patterns simply run off the end and restart.
const uint16_t BUZ_CHIRP[]  = { BUZ_HZ,  90,      0,   0 };
const uint16_t BUZ_DOUBLE[] = { BUZ_HZ,  70,      0,  90, BUZ_HZ, 70, 0, 0 };
const uint16_t BUZ_LOCATE[] = { BUZ_HZ, 150,      0, 850 };
const uint16_t BUZ_ALARM[]  = { BUZ_HZ, 400,      0, 120 };

const uint16_t *buzSeq   = nullptr;
uint8_t   buzSteps = 0, buzIdx = 0;
bool      buzLoop  = false;
uint32_t  buzNextMs = 0;
const char *buzName = "off";

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

  Out.print(F("  CTRL1_XL = 0x")); Out.print(c1, HEX);
  Out.print(F("  ->  accel +/-")); Out.print(ACC_RANGE_G);
  Out.print(F(" g,  ")); Out.print(ACC_G_PER_LSB, 6);
  Out.println(F(" g/LSB"));

  Out.print(F("  CTRL2_G  = 0x")); Out.print(c2, HEX);
  Out.print(F("  ->  gyro  +/-")); Out.print(GYR_RANGE_DPS);
  Out.print(F(" dps, ")); Out.print(GYR_DPS_PER_LSB, 5);
  Out.println(F(" dps/LSB"));

  // Both registers are checked.  Warning on only one of them would let a
  // failed accelerometer write through in silence, which is the exact class
  // of fault this readback exists to catch.
  if (c1 != CFG_CTRL1_XL) {
    Out.println(F("  WARNING: CTRL1_XL did not take. Check I2C wiring."));
  }
  if (c2 != CFG_CTRL2_G) {
    Out.println(F("  WARNING: CTRL2_G did not take. Check I2C wiring."));
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

  Out.print(F("  BW_RATE  = 0x")); Out.print(bw, HEX);
  Out.print(F("  ->  "));
  if (HG_RATE_HZ) {
    Out.print(HG_RATE_HZ);
    Out.println(F(" Hz output data rate"));
  } else {
    Out.println(F("below 25 Hz - far too slow, did the write fail?"));
  }

  Out.print(F("  range    = +/-"));
  Out.print(HG_RANGE_G, 0);
  Out.print(F(" g fixed in silicon, "));
  Out.print(HG_G_PER_LSB, 3);
  Out.println(F(" g/LSB (no range register to read back)"));

  // The compile-time trim from `!hgcal apply`, written and read back like
  // every other setting here.  Zero is the part's own default, so a sketch
  // that has never been calibrated behaves exactly as it did before.
  hga.writeRegister(ADXL3XX_REG_OFSX, (uint8_t)HG_TRIM_X);
  hga.writeRegister(ADXL3XX_REG_OFSY, (uint8_t)HG_TRIM_Y);
  hga.writeRegister(ADXL3XX_REG_OFSZ, (uint8_t)HG_TRIM_Z);
  delay(10);
  int8_t tx = (int8_t)hga.readRegister(ADXL3XX_REG_OFSX);
  int8_t ty = (int8_t)hga.readRegister(ADXL3XX_REG_OFSY);
  int8_t tz = (int8_t)hga.readRegister(ADXL3XX_REG_OFSZ);
  Out.print(F("  offset   = X")); Out.print(tx);
  Out.print(F(" Y")); Out.print(ty);
  Out.print(F(" Z")); Out.print(tz);
  Out.print(F("  ("));
  Out.print(HG_OFS_G_PER_LSB, 3);
  Out.print(F(" g/LSB)"));
  if (tx != HG_TRIM_X || ty != HG_TRIM_Y || tz != HG_TRIM_Z) {
    Out.println(F("  <-- WRITE DID NOT TAKE"));
  } else if (!tx && !ty && !tz) {
    Out.println(F("  - untrimmed; run !hgcal"));
  } else {
    Out.println();
  }

  if (bw != ADXL343_DATARATE_800_HZ) {
    Out.println(F("  WARNING: data rate did not take. Check I2C wiring."));
  }
  return true;
}


// =============================================================================
//  ADXL375 zero-g offset calibration
//
//  WHY SIX POSITIONS.  A single reading cannot separate offset from gravity:
//  an axis reading 0.73 g might be a true 1 g with a -0.27 g bias, or a true
//  0.73 g held at an angle.  Point the axis up and then down and gravity
//  cancels itself out of the average:
//
//      up   = +1 + bias        down = -1 + bias
//      (up + down) / 2 = bias
//
//  which is the entire trick, and it needs no reference more accurate than
//  knowing which way is down.  Scale error survives it -- (up - down)/2 should
//  be 1.000 and is reported so it can be seen -- but the offset registers
//  cannot correct scale anyway, so that stays an observation.
//
//  ONE POSITION PER COMMAND, deliberately.  A guided sequence that blocks
//  waiting for each orientation would stop the loop for as long as it takes
//  somebody to turn a breadboard over.  Capturing on command instead means the
//  sketch keeps running, any single position can be redone without starting
//  again, and the order does not matter.
// =============================================================================
float hgCalUp[3], hgCalDn[3];
bool  hgCalHasUp[3] = { false, false, false };
bool  hgCalHasDn[3] = { false, false, false };

const char HG_AXIS_NAME[3] = { 'X', 'Y', 'Z' };

// Average raw counts, in g, with whatever trim is currently in the registers.
// 64 samples at 5 ms is ~320 ms of blocking -- deliberate, and in the same
// class as zeroGround() and calibrateGyro(), which block for longer.  The
// schedule resynchronises afterwards rather than bursting to catch up.
static void hgAverage(float out[3]) {
  long sx = 0, sy = 0, sz = 0;
  const int N = 64;
  for (int k = 0; k < N; k++) {
    sx += hga.getX(); sy += hga.getY(); sz += hga.getZ();
    delay(5);
  }
  out[0] = (sx / (float)N) * HG_G_PER_LSB;
  out[1] = (sy / (float)N) * HG_G_PER_LSB;
  out[2] = (sz / (float)N) * HG_G_PER_LSB;
}

static int8_t hgReadTrim(int axis) {
  return (int8_t)hga.readRegister(ADXL3XX_REG_OFSX + axis);
}

static void hgPrintTrim(const __FlashStringHelper *lead) {
  Out.print(lead);
  for (int a = 0; a < 3; a++) {
    Out.print(F("  ")); Out.print(HG_AXIS_NAME[a]); Out.print('=');
    Out.print(hgReadTrim(a));
  }
  Out.println();
}

static void hgCalCapture(int axis, bool up) {
  float a[3];
  hgAverage(a);

  // Sanity, loose on purpose: the whole reason this exists is that the reading
  // is offset by an unknown amount, so "near 1 g" cannot be a tight band.  It
  // still catches the fault that matters -- the board held the wrong way up --
  // because that shows in the SIGN and in which axis is largest.
  float m = a[axis];
  bool  ok = (up ? (m > 0.4 && m < 1.7) : (m < -0.4 && m > -1.7));
  for (int k = 0; k < 3 && ok; k++)
    if (k != axis && fabs(a[k]) > fabs(m)) ok = false;

  if (!ok) {
    Out.print(F("  !hgcal refused: "));
    Out.print(HG_AXIS_NAME[axis]);
    Out.print(up ? F("+ should read near +1 g and be the largest axis, but ")
                 : F("- should read near -1 g and be the largest axis, but "));
    Out.print(F("X/Y/Z = "));
    for (int k = 0; k < 3; k++) { Out.print(a[k], 2); if (k < 2) Out.print('/'); }
    Out.println(F("  - check which way up the board is"));
    return;
  }

  if (up) { hgCalUp[axis] = m; hgCalHasUp[axis] = true; }
  else    { hgCalDn[axis] = m; hgCalHasDn[axis] = true; }

  Out.print(F("  captured "));
  Out.print(HG_AXIS_NAME[axis]);
  Out.print(up ? F("+ = ") : F("- = "));
  Out.print(m, 3);
  Out.println(F(" g"));
}

static void hgCalStatus() {
  Out.println(F("  !hgcal - ADXL375 zero-g offset trim"));
  hgPrintTrim(F("  trim now:"));
  Out.println(F("  Rest the board on each of its six faces in turn and capture:"));
  Out.print(F("    "));
  for (int a = 0; a < 3; a++) {
    for (int u = 0; u < 2; u++) {
      Out.print(F(" !hgcal "));
      Out.print(HG_AXIS_NAME[a]);
      Out.print(u ? '+' : '-');
      Out.print((u ? hgCalHasUp : hgCalHasDn)[a] ? F("[ok]") : F("[  ]"));
    }
  }
  Out.println();
  Out.println(F("    then  !hgcal apply   (run it twice; it converges)"));
  Out.println(F("    also  !hgcal clear   !hgcal show"));
}

static void hgCalApply() {
  for (int a = 0; a < 3; a++) {
    if (!hgCalHasUp[a] || !hgCalHasDn[a]) {
      Out.print(F("  !hgcal apply refused: "));
      Out.print(HG_AXIS_NAME[a]);
      Out.println(F(" is missing a position - all six are needed"));
      return;
    }
  }

  int8_t next[3];
  Out.println(F("  axis    bias      scale     trim"));
  for (int a = 0; a < 3; a++) {
    float bias  = (hgCalUp[a] + hgCalDn[a]) / 2.0f;   // gravity cancels
    float scale = (hgCalUp[a] - hgCalDn[a]) / 2.0f;   // should be 1.000

    // Added to what is already there, because `bias` was measured WITH it
    // applied.  That is what makes repeated runs converge.
    long want = lround(hgReadTrim(a) - bias / HG_OFS_G_PER_LSB);
    if (want >  127) want =  127;
    if (want < -128) want = -128;
    next[a] = (int8_t)want;

    Out.print(F("    "));   Out.print(HG_AXIS_NAME[a]);
    Out.print(F("   "));    Out.print(bias, 3);
    Out.print(F(" g   ")); Out.print(scale, 3);
    Out.print(F("     "));  Out.print(next[a]);
    if (want == 127 || want == -128) Out.print(F("  <-- CLAMPED, register range"));
    else if (fabs(scale - 1.0) > 0.15) Out.print(F("  (scale is off; trim cannot fix that)"));
    Out.println();
  }

  for (int a = 0; a < 3; a++) hga.writeRegister(ADXL3XX_REG_OFSX + a, (uint8_t)next[a]);
  delay(20);

  // Readback, rule 1 of this sketch.  A write that silently failed would
  // otherwise look exactly like a calibration that worked.
  bool ok = true;
  for (int a = 0; a < 3; a++) if (hgReadTrim(a) != next[a]) ok = false;
  hgPrintTrim(ok ? F("  written and read back:") : F("  READBACK MISMATCH:"));
  if (!ok) { Out.println(F("  the write did not take - check I2C wiring")); return; }

  float now[3];
  hgAverage(now);
  Out.print(F("  resting magnitude now "));
  Out.print(sqrt(now[0]*now[0] + now[1]*now[1] + now[2]*now[2]), 3);
  Out.println(F(" g  (should approach 1.000 - run apply again to refine)"));

  Out.println(F("  Not persistent. Paste into the sketch and re-flash:"));
  for (int a = 0; a < 3; a++) {
    Out.print(F("      const int8_t HG_TRIM_"));
    Out.print(HG_AXIS_NAME[a]);
    Out.print(F(" = "));
    Out.print(next[a]);
    Out.println(';');
  }
}

static void hgCalClear() {
  for (int a = 0; a < 3; a++) {
    hga.writeRegister(ADXL3XX_REG_OFSX + a, 0);
    hgCalHasUp[a] = hgCalHasDn[a] = false;
  }
  delay(20);
  hgPrintTrim(F("  trim cleared and captures discarded:"));
}

// Dispatch for the '!hgcal ...' family.  Split out so handleLineCommand stays
// a flat list of one-liners.
void hgCalCommand(const char *w) {
  if (!hgPresent) { Out.println(F("  !hgcal: no ADXL375 fitted")); return; }
  if (!*w || !strcmp(w, "status")) { hgCalStatus(); return; }
  if (!strcmp(w, "apply")) { hgCalApply(); return; }
  if (!strcmp(w, "clear")) { hgCalClear(); return; }
  if (!strcmp(w, "show"))  { hgPrintTrim(F("  trim now:")); return; }

  // 'x+' / 'x-' typed at a terminal, 'xup' / 'xdn' from the ground station --
  // whose allowlist admits only letters and digits, so that a URL cannot
  // smuggle anything through.  The same six positions either way.
  int axis = -1;
  for (int a = 0; a < 3; a++)
    if (w[0] == HG_AXIS_NAME[a] || w[0] == HG_AXIS_NAME[a] + 32) axis = a;
  if (axis >= 0) {
    if (w[1] && !w[2] && (w[1] == '+' || w[1] == '-')) {
      hgCalCapture(axis, w[1] == '+'); return;
    }
    if (!strcmp(w + 1, "up")) { hgCalCapture(axis, true);  return; }
    if (!strcmp(w + 1, "dn")) { hgCalCapture(axis, false); return; }
  }
  Out.print(F("  !hgcal: unknown argument ")); Out.println(w);
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

  Out.print(F("  dynamic model = "));
  if (dm == DYN_MODEL_UNKNOWN)              Out.println(F("query FAILED"));
  else if (dm == DYN_MODEL_AIRBORNE1g)      Out.println(F("airborne <1g   OK"));
  else { Out.print(dm); Out.println(F("   <-- NOT airborne <1g")); }

  Out.print(F("  nav rate      = "));
  Out.print(freq);
  Out.println(F(" Hz"));

  if (dm != DYN_MODEL_AIRBORNE1g) {
    Out.println(F("  WARNING: a ground-vehicle model rejects rocket trajectories."));
  }

  // A cold start with no almanac takes 30-60 s and needs a clear sky view.
  // "No fix" at boot on an indoor bench is expected, not a fault — say so, or
  // somebody spends an afternoon debugging a working receiver.
  Out.println(F("  no fix at boot is normal - cold start is 30-60 s, outdoors"));
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
void printDegrees(Print &o, int32_t e7) {
  if (e7 < 0) { o.print('-'); e7 = -e7; }
  o.print(e7 / 10000000L);
  o.print('.');
  long frac = e7 % 10000000L;
  for (long d = 1000000L; d > 1; d /= 10) { if (frac < d) o.print('0'); }
  o.print(frac);
}
void printDegrees(int32_t e7) { printDegrees(Serial, e7); }

// -----------------------------------------------------------------------------
//  The telemetry line, written to any port.
//
//  One function, two destinations: the USB port when the viewer asks for it,
//  and the wired downlink on Serial1 every tick regardless.  Writing it twice
//  from one place is what guarantees the two can never drift apart -- the
//  viewer must see byte-for-byte the same format whichever way it arrives.
//
//  Floats go through Print::print(float, digits), never printf("%f"), which
//  this core may build without float support and fail silently (10.1).
// -----------------------------------------------------------------------------
void emitViz(Print &o, float ax, float ay, float az,
                       float gx, float gy, float gz) {
  o.print(F("V,"));
  o.print(ax, 4);  o.print(',');
  o.print(ay, 4);  o.print(',');
  o.print(az, 4);  o.print(',');
  o.print(gx, 2);  o.print(',');
  o.print(gy, 2);  o.print(',');
  o.print(gz, 2);  o.print(',');
  o.print(alt, 3); o.print(',');
  o.print(vel, 3); o.print(',');
  o.print(millis());

  // -1 marks "not fitted" for both parts.  Zero would be ambiguous: 0.00 g
  // is a legal high-g reading in freefall, fix type 0 is a legal "no fix",
  // and 0,0 is a real position in the Gulf of Guinea.
  o.print(',');
  if (hgPresent) o.print(hgMag, 2); else o.print(-1);

  o.print(',');
  if (!gpsPresent) {
    o.print(F("-1,0,0,0,-1"));
  } else {
    o.print(gpsFixType); o.print(',');
    o.print(gpsSats);    o.print(',');
    if (gpsFixType >= 2) {
      printDegrees(o, gpsLat); o.print(',');
      printDegrees(o, gpsLon); o.print(',');
      o.print(gpsHAccMm / 1000.0, 2);
    } else {
      o.print(F("0,0,-1"));
    }
  }

  // Servo state, so the viewer displays the BOARD's belief rather than its
  // own.  If the two ever disagree about whether the latch is armed, that
  // disagreement is the thing you most need to see.
  o.print(',');
  o.print(servoFired ? 2 : (servoArmed ? 1 : 0));
  o.print(',');
  o.print(servoNowUs);
  o.print(',');
  o.print(latchSensed());

  o.println();
}

//  WHERE this prints is a real decision, not a detail.  Everything routed
//  through `Out` is teed to the wired link and reaches every phone's console.
//  That is right when a person ASKED -- `g` at the USB terminal, `!g` from a
//  phone -- and wrong for the automatic 5-second summary, which asked nobody:
//  with no USB host attached the board stays in table mode and prints one of
//  these every 5 s forever, burying the acknowledgements and refusals that are
//  the console's actual job.  So the caller names the destination: `Out` for a
//  request, plain `Serial` for the summary.
void printGpsStatus(Print& o) {
  o.print(F("  GPS: "));
  if (!gpsPresent) { o.println(F("not fitted")); return; }

  o.print(F("fix "));
  switch (gpsFixType) {
    case 0:  o.print(F("none"));           break;
    case 1:  o.print(F("dead-reckoning")); break;
    case 2:  o.print(F("2D"));             break;
    case 3:  o.print(F("3D"));             break;
    case 4:  o.print(F("GNSS+DR"));        break;
    case 5:  o.print(F("time-only"));      break;
    default: o.print(gpsFixType);          break;
  }
  o.print(F("   sats "));
  o.print(gpsSats);

  if (gpsFixType >= 2) {
    o.print(F("   "));
    printDegrees(o, gpsLat);
    o.print(F(", "));
    printDegrees(o, gpsLon);
    o.print(F("   MSL "));
    o.print(gpsAltMslMm / 1000.0, 1);
    o.print(F(" m"));

    // The number that tells you whether to believe the two above it.
    o.print(F("   +/-"));
    o.print(gpsHAccMm / 1000.0, 1);
    o.print(F(" m"));
    if (gpsSats < 6) {
      o.print(F("   <-- only "));
      o.print(gpsSats);
      o.print(F(" sats, geometry is poor"));
    }
  } else if (gpsEverFixed) {
    // Distinguishing "never had a fix" from "had one and lost it" is the
    // difference between a sky-view problem and an antenna or power problem.
    o.print(F("   lock LOST "));
    o.print((millis() - gpsLastFixMs) / 1000);
    o.print(F(" s ago"));
  } else {
    o.print(F("   acquiring - needs sky view"));
  }
  o.println();
}

// -----------------------------------------------------------------------------
//  Buzzer.  Toggled from a Ticker in interrupt context, so the callback does
//  the least possible work: flip one pin.
// -----------------------------------------------------------------------------
void buzToggleISR() { if (buzPin) *buzPin = !*buzPin; }

void buzSilence() {
  buzTicker.detach();
  if (buzPin) *buzPin = 0;              // explicitly low, not merely un-ticked
}

void buzToneOn(uint16_t hz) {
  if (!buzPin || hz == 0) { buzSilence(); return; }
  buzTicker.detach();
  // Half-period: the pin toggles twice per cycle.
  buzTicker.attach(mbed::callback(buzToggleISR),
                   std::chrono::microseconds(500000UL / hz));
}

void buzzerStop() {
  buzSilence();
  buzSeq = nullptr; buzSteps = 0; buzIdx = 0; buzLoop = false;
  buzName = "off";
}

void buzzerPlay(const uint16_t *seq, uint8_t steps, bool loop, const char *name) {
  buzzerStop();
  buzSeq = seq; buzSteps = steps; buzLoop = loop; buzName = name;
  buzIdx = 0; buzNextMs = millis();     // first step lands on the next service
}

// Advances the pattern.  Called every loop iteration; never blocks.
void buzzerService() {
  if (!buzSeq) return;
  if ((long)(millis() - buzNextMs) < 0) return;

  if (buzIdx >= buzSteps) {
    if (!buzLoop) { buzzerStop(); return; }
    buzIdx = 0;
  }
  uint16_t hz = buzSeq[buzIdx * 2];
  uint16_t ms = buzSeq[buzIdx * 2 + 1];
  if (ms == 0) { buzzerStop(); return; }   // explicit terminator

  buzToneOn(hz);
  buzNextMs = millis() + ms;
  buzIdx++;
}

void printBuzzerStatus() {
  Out.print(F("  BUZZER: pattern "));
  Out.print(buzName);
  Out.print(F("   pin GP")); Out.print(BUZZER_PIN);
  Out.print(F("   "));
  Out.print(BUZ_HZ);
  Out.println(F(" Hz"));
}

// -----------------------------------------------------------------------------
//  Servo latch control.
//
//  servoWrite() is the only path to the pin, so the clamp and the record of
//  what was commanded cannot be bypassed by a caller in a hurry.
// -----------------------------------------------------------------------------
void servoWrite(uint16_t us) {
  if (us < SERVO_MIN_US) us = SERVO_MIN_US;
  if (us > SERVO_MAX_US) us = SERVO_MAX_US;
  if (!latch.attached()) return;        // disarmed: nothing to write to
  servoNowUs = us;
  latch.writeMicroseconds(us);
  servoLastMoveMs = millis();
  servoDetachAtMs = 0;                  // a fresh move cancels a pending detach
}

// Attach and detach ARE the arm and safe states.  Detached, the library stops
// its ticker and the pin is simply an output sitting low -- no pulse train at
// all, so the servo is not being commanded anywhere.  That is a stronger claim
// than "commanded to a pulse width of zero", which is not a thing a servo
// understands.
void servoAttach() {
  if (!latch.attached()) latch.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
}

// Detaches -- but NEVER truncates a move that is still under way.
//
// This was a real bug.  The viewer fires with `!fire` and then immediately
// sends `!safe`, which is the right interlock: a test rig must not stay hot
// after doing the thing.  But `!safe` detached the servo a millisecond after
// `!fire` had written the released position, cutting the pulse train long
// before the horn could travel.  So Fire, and every condition-triggered fire,
// did nothing -- while `!us` and `!latch`, which leave the servo attached,
// worked perfectly.  The interlock defeated the action it was guarding.
//
// Safe now keeps its whole meaning -- servoArmed is already false, so no NEW
// motion is accepted from this instant -- and simply lets the move already
// commanded finish before the pulse train stops.
void servoRelease() {
  uint32_t since = millis() - servoLastMoveMs;
  if (latch.attached() && servoNowUs && since < SERVO_SETTLE_MS) {
    servoDetachAtMs = servoLastMoveMs + SERVO_SETTLE_MS;
    if (servoDetachAtMs == 0) servoDetachAtMs = 1;   // 0 means "none pending"
    return;
  }
  servoDetachAtMs = 0;
  servoNowUs = 0;
  if (latch.attached()) latch.detach();
}

// Completes a deferred detach, and judges the move once it has had time to
// happen.  Called every loop iteration.
void servoService() {
  if (servoDetachAtMs && (long)(millis() - servoDetachAtMs) >= 0) {
    servoDetachAtMs = 0;
    servoNowUs = 0;
    if (latch.attached()) latch.detach();
    Out.println(F("  SERVO pulse train stopped - move complete"));
  }

  // The settle window has passed, so whatever the mechanism was going to do,
  // it has done.  With no switch fitted there is nothing to compare against
  // and the check quietly retires -- it does not invent a verdict.
  if (latchConfirmAtMs && (long)(millis() - latchConfirmAtMs) >= 0) {
    latchConfirmAtMs = 0;
    if (LATCH_SWITCH_PIN >= 0) {
      int got = latchSensedSettled();
      if (got < 0) {
        Out.println(F("  LATCH UNCONFIRMED - switch still bouncing or disconnected"));
      } else if (got == latchExpect) {
        Out.print(F("  LATCH CONFIRMED "));
        Out.println(got == 0 ? F("RELEASED") : F("engaged"));
      } else {
        // The fault this whole mechanism exists to catch.
        Out.print(F("  *** LATCH DID NOT "));
        Out.print(latchExpect == 0 ? F("RELEASE") : F("ENGAGE"));
        Out.println(F(" - commanded, but the switch disagrees ***"));
        // Looping, deliberately: a latch that did not release is not a
        // notification, and it should not stop because nobody looked.
        // Silence it with `!buz off` once it has been seen.
        buzzerPlay(BUZ_ALARM, 2, true, "alarm");
      }
    }
    latchExpect = -1;
  }
}

void printServoStatus() {
  Out.print(F("  SERVO: "));
  Out.print(servoArmed ? F("ARMED") : F("safe"));
  if (servoFired) Out.print(LATCH_SWITCH_PIN < 0 ? F(" (FIRE COMMANDED)")
                                                 : F(" (FIRED)"));
  Out.print(F("   pin GP")); Out.print(SERVO_PIN);
  Out.print(F("   commanded "));
  if (servoNowUs) { Out.print(servoNowUs); Out.print(F(" us")); }
  else            Out.print(F("idle - not driven"));
  Out.print(F("   latched=")); Out.print(servoLatchedUs);
  Out.print(F(" released=")); Out.print(servoReleasedUs);
  Out.println(F(" us"));
  Out.print(F("  feedback: "));
  if (LATCH_SWITCH_PIN < 0) {
    Out.println(F("NONE - 'fired' means COMMANDED to fire.  A stalled or"));
    Out.println(F("            unpowered servo reports exactly the same as a"));
    Out.println(F("            healthy one.  Fit a switch on GP<n> and set"));
    Out.println(F("            LATCH_SWITCH_PIN to close this."));
  } else {
    Out.print(F("switch on GP"));
    Out.print(LATCH_SWITCH_PIN);
    Out.print(F(" reads "));
    int v = latchSensed();
    Out.println(v == 0 ? F("RELEASED") : F("engaged"));
  }
}

void servoArm(bool on) {
  servoArmed = on;
  if (on) {
    servoLastCmdMs = millis();
    // Attaching starts no pulses of its own: the library only begins its
    // ticker on the first writeMicroseconds().  So arming still moves nothing,
    // which is the property that matters -- if arming drove the horn, the act
    // of preparing to test would be the test.
    servoAttach();
    buzzerPlay(BUZ_CHIRP, 2, false, "chirp");   // 8.5: the arming chirp
    Out.println(F("  SERVO ARMED - latch will respond to commands"));
  } else {
    servoRelease();
    if (servoDetachAtMs) {
      Out.print(F("  SERVO SAFE - no new commands; finishing move, pulse stops in "));
      Out.print((long)(servoDetachAtMs - millis()));
      Out.println(F(" ms"));
    } else {
      Out.println(F("  SERVO SAFE - pin no longer driven"));
    }
  }
}

bool servoFire() {
  if (!servoArmed) { Out.println(F("  SERVO refused: not armed")); return false; }
  if (servoFired)  { Out.println(F("  SERVO refused: already fired, re-latch first")); return false; }
  servoWrite(servoReleasedUs);
  servoFired = true;
  buzzerPlay(BUZ_DOUBLE, 4, false, "double");   // audible confirmation of a fire
  // "FIRE COMMANDED", not "FIRED".  Without a switch that is the whole truth
  // of what just happened, and the log is the record somebody reads later.
  Out.print(F("  SERVO FIRE COMMANDED -> ")); Out.print(servoNowUs);
  Out.println(LATCH_SWITCH_PIN < 0 ? F(" us (no feedback - not confirmed)")
                                   : F(" us - checking the switch"));
  latchExpect = 0;                              // should end up released
  latchConfirmAtMs = servoLastMoveMs + SERVO_SETTLE_MS;
  if (latchConfirmAtMs == 0) latchConfirmAtMs = 1;
  return true;
}

void servoRelatch() {
  if (!servoArmed) { Out.println(F("  SERVO refused: not armed")); return; }
  servoWrite(servoLatchedUs);
  servoFired = false;
  Out.print(F("  SERVO re-latch COMMANDED -> ")); Out.print(servoNowUs);
  Out.println(F(" us"));
  latchExpect = 1;                              // should end up engaged
  latchConfirmAtMs = servoLastMoveMs + SERVO_SETTLE_MS;
  if (latchConfirmAtMs == 0) latchConfirmAtMs = 1;
}

// -----------------------------------------------------------------------------
//  Buffered '!' commands.  The single-character commands are dispatched the
//  instant they arrive and stay that way; anything needing a NUMBER cannot be,
//  so those are introduced with '!' and terminated by a newline.  Keeping the
//  two schemes visibly separate means the old commands cannot be broken by a
//  parser change, and a truncated line can never be mistaken for one of them.
// -----------------------------------------------------------------------------
void zeroGround();
void calibrateGyro();
void printGpsStatus(Print& o);

void resetPeaks() {
  altPeak = alt; gPeak = 0; gyroPeak = 0; hgPeak = 0;
  accClip = false; gyrClip = false; hgClip = false;
  Out.println(F("-- peaks reset --"));
}

void handleLineCommand(char *cmd) {
  servoLastCmdMs = millis();            // any command at all is a sign of life

  // The single-character commands, in framed form -- so the wire can reach
  // them without the wire ever acting on a bare byte.
  if      (!strcmp(cmd, "z")) { zeroGround();     return; }
  else if (!strcmp(cmd, "b")) { calibrateGyro();  return; }
  else if (!strcmp(cmd, "r")) { resetPeaks();     return; }
  else if (!strcmp(cmd, "g")) { printGpsStatus(Out); return; }

  if      (!strcmp(cmd, "arm"))   servoArm(true);
  else if (!strcmp(cmd, "safe"))  servoArm(false);
  else if (!strcmp(cmd, "fire"))  servoFire();
  else if (!strcmp(cmd, "latch")) servoRelatch();
  else if (!strcmp(cmd, "srv"))   printServoStatus();
  else if (!strcmp(cmd, "bz"))    printBuzzerStatus();
  else if (!strncmp(cmd, "buz ", 4)) {
    const char *w = cmd + 4;
    if      (!strcmp(w, "off"))    { buzzerStop(); }
    else if (!strcmp(w, "chirp"))  buzzerPlay(BUZ_CHIRP,  2, false, "chirp");
    else if (!strcmp(w, "double")) buzzerPlay(BUZ_DOUBLE, 4, false, "double");
    else if (!strcmp(w, "locate")) buzzerPlay(BUZ_LOCATE, 2, true,  "locate");
    else if (!strcmp(w, "alarm"))  buzzerPlay(BUZ_ALARM,  2, true,  "alarm");
    else { Out.print(F("  !buz: unknown pattern ")); Out.println(w); return; }
    printBuzzerStatus();
  }
  else if (!strncmp(cmd, "beep ", 5)) {
    char *sp = strchr(cmd + 5, ' ');
    uint16_t hz = (uint16_t)atoi(cmd + 5);
    uint16_t ms = sp ? (uint16_t)atoi(sp + 1) : 120;
    if (hz < 100 || hz > 20000) { Out.println(F("  !beep: 100-20000 Hz")); return; }
    static uint16_t oneShot[4];
    oneShot[0] = hz; oneShot[1] = ms; oneShot[2] = 0; oneShot[3] = 0;
    buzzerPlay(oneShot, 2, false, "beep");
    Out.print(F("  BUZZER beep ")); Out.print(hz);
    Out.print(F(" Hz for ")); Out.print(ms); Out.println(F(" ms"));
  }
  else if (!strcmp(cmd, "hgcal")) hgCalCommand("");
  else if (!strncmp(cmd, "hgcal ", 6)) hgCalCommand(cmd + 6);
  else if (!strcmp(cmd, "hb"))    { /* heartbeat: the timestamp above is it */ }
  else if (!strncmp(cmd, "us ", 3)) {
    if (!servoArmed) { Out.println(F("  SERVO refused: not armed")); return; }
    servoWrite((uint16_t)atoi(cmd + 3));
    Out.print(F("  SERVO -> ")); Out.print(servoNowUs); Out.println(F(" us"));
  }
  else if (!strncmp(cmd, "pos ", 4)) {
    char *sp = strchr(cmd + 4, ' ');
    if (!sp) { Out.println(F("  !pos needs two values: !pos <latched_us> <released_us>")); return; }
    *sp = 0;
    uint16_t a = (uint16_t)atoi(cmd + 4), b = (uint16_t)atoi(sp + 1);
    if (a < SERVO_MIN_US || a > SERVO_MAX_US || b < SERVO_MIN_US || b > SERVO_MAX_US) {
      Out.println(F("  !pos refused: outside the 600-2400 us clamp"));
      return;
    }
    servoLatchedUs = a; servoReleasedUs = b;
    Out.print(F("  SERVO endpoints: latched ")); Out.print(a);
    Out.print(F(" us, released ")); Out.print(b); Out.println(F(" us"));
  }
  else { Out.print(F("  !? unknown command: ")); Out.println(cmd); }
}

// -----------------------------------------------------------------------------
//  Feed one byte from a port.  Returns true when the byte belonged to a framed
//  '!' line; false means it was a bare byte and the CALLER decides what, if
//  anything, a bare byte means on that port.
// -----------------------------------------------------------------------------
bool cmdFeed(CmdPort &p, char c) {
  if (p.active) {
    if (c == '\n' || c == '\r') {
      p.buf[p.len] = 0;
      p.active = false;
      if (p.overflow) {
        Out.println(F("  !? command too long - discarded, not truncated"));
      } else if (p.len) {
        handleLineCommand(p.buf);
      }
      p.len = 0; p.overflow = false;
    } else if (p.len < (int)sizeof(p.buf) - 1) {
      p.buf[p.len++] = c;
    } else {
      p.overflow = true;                // acting on a prefix of a longer
    }                                   // command could mean a different one
    return true;
  }
  if (c == '!') { p.active = true; p.len = 0; p.overflow = false; return true; }
  return false;
}

// -----------------------------------------------------------------------------
//  Gyro zero-rate offset.  Every gyro has one, it changes with temperature,
//  and it is what walks your attitude estimate off during a flight when there
//  is no reliable gravity vector to correct against.  Board must be still.
// -----------------------------------------------------------------------------
void calibrateGyro() {
  Out.print(F("Measuring gyro bias - hold still"));
  double sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < BIAS_N; i++) {
    imu.read();
    sx += imu.g.x; sy += imu.g.y; sz += imu.g.z;
    if (i % 50 == 0) Out.print('.');
    // 6 ms, not 4.  The gyro runs at 208 Hz, a 4.8 ms period, so a 4 ms delay
    // re-reads the same sample often enough to weight the average toward
    // whichever readings happen to be duplicated.  Sampling slower than the
    // ODR keeps every sample independent, which is the point of averaging.
    delay(6);
  }
  gyroBiasX = sx / BIAS_N;
  gyroBiasY = sy / BIAS_N;
  gyroBiasZ = sz / BIAS_N;

  Out.print(F(" done.  offset = "));
  Out.print(gyroBiasX * GYR_DPS_PER_LSB, 2); Out.print(F(" / "));
  Out.print(gyroBiasY * GYR_DPS_PER_LSB, 2); Out.print(F(" / "));
  Out.print(gyroBiasZ * GYR_DPS_PER_LSB, 2); Out.println(F(" dps"));
}

// Height above the launch pad.  Always a delta from a pad reading — never
// trust absolute sea-level altitude.
float altitudeFrom(float hpa) {
  return 44330.0 * (1.0 - pow(hpa / groundHpa, 0.1902949));
}

void zeroGround() {
  Out.print(F("Zeroing ground pressure"));
  double sum = 0;
  for (int i = 0; i < 40; i++) {
    baro.read();
    sum += baro.getPressure();
    if (i % 10 == 0) Out.print('.');
    delay(20);
  }
  groundHpa = sum / 40.0;
  alt = altPrev = vel = 0.0;
  altPeak = 0.0;
  noiseIdx = 0;
  noiseFull = false;
  Out.print(F(" done. Ground = "));
  Out.print(groundHpa, 2);
  Out.println(F(" hPa"));
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
//  I2C bus scan
//
//  A single pass over the bus answers "is it there", which is the question
//  that matters when a sensor is missing.  It is the wrong question for the
//  opposite fault: an address that answers when NOTHING is fitted there.  This
//  bench has had a persistent phantom at 0x7E, and 0x78-0x7F is reserved by
//  the I2C specification -- no device may use it, so nothing can legitimately
//  be answering.  Which leaves the master misreading an ACK.
//
//  So probe each address REPEATEDLY, and at two bus speeds.  That turns the
//  mystery into a measurement:
//
//    8/8 at both speeds            a device.  Believe it.
//    intermittent at either        the bus.  SDA is not reaching a clean low
//                                  within the ACK window, and the master reads
//                                  a floating line as an acknowledgement.
//    only at 400 kHz               the bus, specifically its rise time -- the
//                                  pull-ups cannot charge the line fast enough
//                                  once the clock shortens.
//
//  None of those is fixed in firmware.  The point is to say which physical
//  thing to change: parallel pull-ups (five breakout boards each with their
//  own is a far stiffer bus than any one of them intends), lead length, or
//  stub length off the main run.
// =============================================================================
#define SCAN_TRIES 8

//  The rate the loop actually runs at.  Previously implicit -- Wire.begin()
//  leaves the Mbed core at its 100 kHz default -- which is fine until the scan
//  below changes the clock and has to put it back.  Stated, so it can be.
#define I2C_HZ  100000

//  Reserved by the specification: 0x00-0x07 and 0x78-0x7F.  Nothing may be
//  addressed here, so an answer is by definition not a device.
static bool i2cReserved(byte a) { return a <= 0x07 || a >= 0x78; }

//  One probe.  The Wire.write(0) is required on Mbed cores, where a
//  zero-length endTransmission() issues a read and most devices will not ACK
//  it.
//
//  Side effect: this writes a zero byte to every address on the bus, and now
//  does so SCAN_TRIES times at each of two speeds.  Re-checked with five parts
//  fitted -- on the MS5611 it is a harmless ADC-read command, on the ADXL375
//  register 0 is the read-only device ID, and on the u-blox it only moves the
//  DDC address pointer.  All three are idempotent, which is what makes the
//  repeat safe.  Check again if a sixth device joins the bus.
static bool i2cProbe(byte a) {
  Wire.beginTransmission(a);
  Wire.write(0);
  return Wire.endTransmission() == 0;
}

static void i2cName(byte a) {
  if      (a == 0x76 || a == 0x77) Out.print(F("MS5611 barometer"));
  else if (a == 0x6A || a == 0x6B) Out.print(F("LSM6 accel + gyro"));
  else if (a == 0x1C || a == 0x1E) Out.print(F("LIS3MDL magnetometer"));
  else if (a == 0x53 || a == 0x1D) Out.print(F("ADXL375 high-g"));
  else if (a == 0x42)              Out.print(F("u-blox GPS"));
  else if (i2cReserved(a))         Out.print(F("RESERVED - cannot be a device"));
  else                             Out.print(F("(unknown)"));
}

//  Returns the number of addresses that answered at all; fills `solid` with
//  how many of those answered every single time.
static int scanBus(uint32_t hz, int& solid, int& phantom) {
  Wire.setClock(hz);
  Out.print(F("I2C scan at "));
  Out.print(hz / 1000);
  Out.print(F(" kHz ("));
  Out.print(SCAN_TRIES);
  Out.println(F(" probes each):"));

  int found = 0;
  solid = 0;
  phantom = 0;
  for (byte a = 1; a < 127; a++) {
    int hits = 0;
    for (int k = 0; k < SCAN_TRIES; k++) if (i2cProbe(a)) hits++;
    if (hits == 0) continue;
    found++;
    if (hits == SCAN_TRIES) solid++;
    if (i2cReserved(a) || hits < SCAN_TRIES) phantom++;

    Out.print(F("  0x"));
    if (a < 16) Out.print('0');
    Out.print(a, HEX);
    Out.print(F("  "));
    Out.print(hits);
    Out.print('/');
    Out.print(SCAN_TRIES);
    Out.print(hits == SCAN_TRIES ? F("  ") : F("  <-- INTERMITTENT  "));
    i2cName(a);
    Out.println();
  }
  if (found == 0) Out.println(F("  nothing found - check wiring and power"));
  return found;
}

// =============================================================================
//  setup
// =============================================================================
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) { }

  if (LINK_ENABLED) Serial1.begin(LINK_BAUD);

  Out.println();
  Out.println(F("=== ROCKET AVIONICS BENCH DIAGNOSTICS ==="));
  if (LINK_ENABLED) {
    Out.print(F("Wired link on GP0/GP1 at "));
    Out.print(LINK_BAUD);
    Out.println(F(" baud - telemetry down, framed commands up"));
  }
  Out.println();

  // A switch, if one is fitted.  Pulled up, so an absent or broken one reads
  // "engaged" and a fire that did nothing is reported rather than confirmed.
  if (LATCH_SWITCH_PIN >= 0) pinMode(LATCH_SWITCH_PIN, INPUT_PULLUP);

  Wire.begin();

  // ---- I2C scan.  Always do this first.  If an address does not show up
  //      here, no amount of driver code will make that sensor work. --------
  int solid100, phantom100, solid400, phantom400;
  int found100 = scanBus(100000, solid100, phantom100);
  Out.println();
  int found400 = scanBus(400000, solid400, phantom400);
  Wire.setClock(I2C_HZ);            // back to the rate the loop runs at
  Out.println();

  // The verdict.  Two numbers that disagree are the finding, so say what the
  // disagreement means rather than leaving it to be noticed.
  if (phantom100 || phantom400 || found400 != found100) {
    Out.println(F("  BUS INTEGRITY:"));
    if (found400 != found100) {
      Out.print(F("    "));
      Out.print(found100);
      Out.print(F(" addresses at 100 kHz, "));
      Out.print(found400);
      Out.println(F(" at 400 kHz - the count should not depend on speed."));
    }
    Out.println(F("    An address that is reserved, or that answers only some"));
    Out.println(F("    of the time, is not a device: SDA is not reaching a"));
    Out.println(F("    clean low inside the ACK window and the master is"));
    Out.println(F("    reading a floating line as an acknowledgement."));
    Out.println(F("    In order of suspicion: five breakout boards each with"));
    Out.println(F("    its own pull-ups in parallel (remove all but one set),"));
    Out.println(F("    then lead length, then stub length off the main run."));
    Out.println(F("    Harmless for the parts that DO answer 8/8 - it costs"));
    Out.println(F("    noise margin, not correctness, until it does not."));
  } else {
    Out.println(F("  bus integrity: every address solid at both speeds"));
  }
  Out.println();

  // ---- barometer.  reset() loads the calibration constants and skips the
  //      isConnected() check, whose Mbed workaround is gated behind an
  //      nRF52840-only macro and so never compiles in on RP2040. -----------
  if (baro.reset()) {
    Out.print(F("MS5611 found at 0x"));
    Out.println(baro.getAddress(), HEX);
  } else {
    Out.println(F("MS5611 NOT found."));
    Out.println(F("  - PS pin must be HIGH for I2C mode"));
    Out.println(F("  - CSB to GND = 0x77, CSB to VCC = 0x76"));
    while (1) delay(1000);
  }

  // ---- IMU ---------------------------------------------------------------
  if (imu.init()) {
    Out.println(F("LSM6 found and initialised"));
    applyImuConfig();
  } else {
    Out.println(F("LSM6 NOT found - check wiring (Pololu board is 0x6B)"));
    while (1) delay(1000);
  }

  // ---- high-g accelerometer.  Not fatal: the column degrades to dashes. ---
  Out.println();
  if (startHighG()) {
    Out.println(F("ADXL375 found and initialised"));
    hgPresent = true;
  } else {
    Out.println(F("ADXL375 NOT found - the hg column will read ---"));
    Out.println(F("  - address jumper open = 0x53, bridged = 0x1D"));
  }

  // ---- GPS.  Not fatal either. -------------------------------------------
  Out.println();
  if (startGps()) {
    Out.println(F("SAM-M8Q found and configured"));
    gpsPresent = true;
  } else {
    Out.println(F("SAM-M8Q NOT found - GPS lines will read 'not fitted'"));
    Out.println(F("  - u-blox DDC is 0x42; give it a moment after power-up"));
  }

  // ---- servo latch.  Deliberately NOT attached here: an unattached pin
  //      emits no pulse train, so nothing is commanded until somebody arms
  //      the latch on purpose. -------------------------------------------
  Out.println();
  buzPin = new mbed::DigitalOut(digitalPinToPinName(BUZZER_PIN));
  *buzPin = 0;
  Out.print(F("Buzzer on GP")); Out.print(BUZZER_PIN);
  Out.print(F(" - silent at boot, ")); Out.print(BUZ_HZ);
  Out.println(F(" Hz"));

  Out.print(F("Servo latch on GP")); Out.print(SERVO_PIN);
  Out.println(F(" - SAFE at boot, no pulses emitted"));
  Out.println(F("  arm it with !arm before anything will move"));

  Out.println();
  calibrateGyro();
  Out.println();
  zeroGround();

  Out.println();
  Out.println(F("Commands:  z = re-zero   b = gyro bias   r = reset peaks"));
  Out.println(F("           c = CSV       v = viz stream  g = GPS   h = header"));
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

    // A '!' opens a buffered line command; everything up to the newline is
    // collected rather than dispatched character by character.
    if (cmdFeed(usbCmd, c)) continue;
    // The header is for the human-readable table only.  Reprinting it while
    // the viewer is streaming injects non-telemetry lines into the feed the
    // viewer is parsing -- harmless, since it ignores anything that is not a
    // V line, but it is noise in a stream that should be clean.  The progress
    // text from these two DOES stay: it lands in the viewer's raw line, which
    // is useful feedback that the command was received.
    if (c == 'z' || c == 'Z') { Serial.println(); zeroGround(); if (!vizMode) printHeader(); }
    else if (c == 'b' || c == 'B') { Serial.println(); calibrateGyro(); if (!vizMode) printHeader(); }
    else if (c == 'r' || c == 'R') resetPeaks();
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
    else if (c == 'g' || c == 'G') { Serial.println(); printGpsStatus(Out); }
    else if (c == 'h' || c == 'H') printHeader();
  }

  // The wire from the ground station.  Framed lines only: cmdFeed's return
  // value is deliberately ignored, so a bare byte from here does nothing.
  while (LINK_ENABLED && Serial1.available()) {
    cmdFeed(linkCmd, (char)Serial1.read());
  }

  buzzerService();
  servoService();

  // ---- servo deadman -----------------------------------------------------
  // An armed actuator that has stopped hearing from anyone is the bench
  // hazard, so silence disarms it.  Note this is the OPPOSITE of what flight
  // firmware must do, where losing the link may not disarm anything -- one
  // more reason this code is a test harness and not the flight path.
  if (servoArmed && (millis() - servoLastCmdMs) > SERVO_DEADMAN_MS) {
    Out.println(F("  SERVO auto-safe: no host command for 3 s"));
    servoArm(false);
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
  //
  // WIRED DOWNLINK (v0.3.1 stand-in for the radio).  Written every tick,
  // whatever the USB port is doing -- a radio downlink does not wait to be
  // asked, and neither does this.  Commands now come UP this wire too, by
  // deliberate choice; see "THE UPLINK" for what is and is not accepted.
  if (LINK_ENABLED) { Out.endLine(); emitViz(Serial1, ax, ay, az, gx, gy, gz); }

  if (vizMode) {
    emitViz(Serial, ax, ay, az, gx, gy, gz);
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
        Serial.println(F(" g, normal for this part - run !hgcal to trim it out"));
      } else {
        Serial.println(F("  OK"));
      }
    }

    // Serial, not Out: the summary is unsolicited, and teeing it to the
    // link floods the phone's console every 5 s.  Ask with `g` or `!g`.
    printGpsStatus(Serial);

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
