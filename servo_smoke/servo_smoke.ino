// =============================================================================
//  SERVO SMOKE TEST  —  two drive methods, so the answer is unambiguous
//
//  For when the latch will not move.  It tries the two things that can drive a
//  servo pin and tells you which, if either, works.  Nothing else runs: no I2C,
//  no sensors, no arming, no command parser.
//
//  IT HAS ALREADY PAID FOR ITSELF.  The first servo on this bench never moved,
//  and a plausible software explanation got built around that.  It was a dead
//  servo.  A replacement moved in both phases on the first try.  So: swap in a
//  known-good unit BEFORE blaming firmware, and use Phase A to take software
//  out of the question.
//
//    PHASE A — RAW BIT-BANG.  digitalWrite plus delayMicroseconds in a tight
//              loop.  No library, no interrupts, no timers, no abstraction.
//              This is the most primitive thing that can possibly produce a
//              servo pulse.  If a servo does not move on this, the problem is
//              not software.
//
//    PHASE B — Servo LIBRARY.  What rocket_diagnostics uses.
//
//  READING THE RESULT
//    moves in A, not in B      -> the library or its timers.  Tell me.
//    moves in B, not in A      -> surprising; tell me, the bit-bang is wrong.
//    moves in neither          -> wiring, power, or the servo.  See below.
//    moves in both             -> the fault is in rocket_diagnostics.  Tell me.
//
//  NOTE ON mbed::PwmOut.  Untested, not known-broken.  It was only ever tried
//  against the faulty servo.  An earlier version of this comment declared it
//  broken on this core; that conclusion was built on the dead servo and is
//  withdrawn.  Test it here, against Phase A as the control, if you want it.
//
//  WIRING, CONFIRMED WORKING
//    signal -> GP6  (physical pin 9)
//    V+     -> VBUS (physical pin 40)
//    GND    -> common with the Pico       not optional: no shared ground, no
//                                         pulse edge, no movement
//
//  ON VBUS.  It works, and it is marginal on paper: USB 5 V behind a Schottky
//  diode idles near 4.7 V and sags under load, against an MG90D specified from
//  4.8 V, and a USB port current-limits near 500 mA where the servo can pull
//  ~700 mA stalled.  Fine for an unloaded bench latch.  Re-check it working
//  against a packed chute, and give it its own supply for flight.
// =============================================================================

#include <Servo.h>

const int      SERVO_PIN = 6;         // physical pin 9
const uint16_t US_A      = 1000;      // one end of the travel
const uint16_t US_B      = 2000;      // the other
const uint16_t FRAME_MS  = 20;        // 50 Hz
const uint32_t PHASE_MS  = 8000;      // how long to spend in each phase
const uint32_t DWELL_MS  = 1000;      // how long to hold each end

Servo latch;

// -----------------------------------------------------------------------------
//  Phase A.  One pulse, by hand.  Blocking on purpose -- there is nothing else
//  to do, and blocking removes scheduler behaviour from the list of suspects.
// -----------------------------------------------------------------------------
void bitBangHold(uint16_t us, uint32_t forMs) {
  uint32_t stop = millis() + forMs;
  while ((long)(millis() - stop) < 0) {
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(us);
    digitalWrite(SERVO_PIN, LOW);
    delay(FRAME_MS - 2);              // the gap; a servo is not fussy about it
  }
}

void phaseBitBang() {
  Serial.println();
  Serial.println(F("PHASE A - raw bit-bang, no library"));
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);

  uint32_t stop = millis() + PHASE_MS;
  while ((long)(millis() - stop) < 0) {
    Serial.print(F("  A -> ")); Serial.print(US_A); Serial.println(F(" us"));
    bitBangHold(US_A, DWELL_MS);
    Serial.print(F("  A -> ")); Serial.print(US_B); Serial.println(F(" us"));
    bitBangHold(US_B, DWELL_MS);
  }
  digitalWrite(SERVO_PIN, LOW);
}

void phaseLibrary() {
  Serial.println();
  Serial.println(F("PHASE B - Servo library"));
  latch.attach(SERVO_PIN, 600, 2400);

  uint32_t stop = millis() + PHASE_MS;
  while ((long)(millis() - stop) < 0) {
    Serial.print(F("  B -> ")); Serial.print(US_A); Serial.println(F(" us"));
    latch.writeMicroseconds(US_A);
    delay(DWELL_MS);
    Serial.print(F("  B -> ")); Serial.print(US_B); Serial.println(F(" us"));
    latch.writeMicroseconds(US_B);
    delay(DWELL_MS);
  }
  latch.detach();
  digitalWrite(SERVO_PIN, LOW);
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) { }

  Serial.println();
  Serial.println(F("=== SERVO SMOKE TEST ==="));
  Serial.print(F("pin GP")); Serial.print(SERVO_PIN);
  Serial.print(F(" (physical 9)   sweeping "));
  Serial.print(US_A); Serial.print(F(" <-> ")); Serial.print(US_B);
  Serial.println(F(" us"));
  Serial.println();
  Serial.println(F("Two phases, 8 s each, repeating. Watch the horn, not the log:"));
  Serial.println(F("  A = raw bit-bang, no library at all"));
  Serial.println(F("  B = Servo library, what rocket_diagnostics uses"));
  Serial.println();
  Serial.println(F("Moves in neither -> not software. Check in this order:"));
  Serial.println(F("  1. the servo itself - swap in a known-good unit FIRST."));
  Serial.println(F("     (This is what it was, the first time round.)"));
  Serial.println(F("  2. ground shared between servo supply and Pico"));
  Serial.println(F("  3. supply: VBUS is ~4.7 V against an MG90D's 4.8 V spec"));
}

void loop() {
  phaseBitBang();
  phaseLibrary();
}

//  -- END OF FILE --
