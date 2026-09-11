# =============================================================================
#  ROCKET GROUND STATION  —  Raspberry Pi Pico W
#
#  Brings up its own Wi-Fi access point, serves the attitude viewer, and
#  streams telemetry to any connected phone or laptop over Server-Sent
#  Events.  No router, no internet, no app install: connect to the SSID
#  below and open http://192.168.4.1
#
#  TELEMETRY SOURCES, selected explicitly -- never switched automatically:
#    uart     the flight computer, over a wire into GP1.  The v0.3.1 stand-in
#             for the radio, and the default.  See "WIRED LINK" below.
#    bench / flight / still   synthetic profiles, for UI work with no board.
#
#  There is deliberately NO fallback from uart to synthetic when the wire goes
#  quiet.  Quietly substituting plausible fake data for a dead link is the
#  single worst thing this display could do; instead the stream goes stale and
#  the viewer says so.
#
#  INSTALL
#    1. Flash MicroPython for Pico W (rp2-pico-w UF2) from micropython.org
#    2. Copy this file to the board as  main.py
#    3. Copy the viewer to the board as  index.html
#    4. Reset the board.  The LED goes solid once the AP is up.
#
#  USE
#    Phone Wi-Fi  ->  join  ROCKET-GS   (password below)
#    Browser      ->  http://192.168.4.1
#
#  Endpoints
#    /            the viewer
#    /stream      SSE telemetry, one "V,..." frame per event
#    /health      JSON status, handy for debugging from a laptop
#    /mode?m=     uart | bench | flight | still   — selects the source
#    /cmd?c=      a board command, relayed up the wire (wired source only)
# =============================================================================

import network
import socket
import time
import math
import gc
import json
import os

try:
    import asyncio
except ImportError:          # MicroPython < 1.20 calls it uasyncio
    import uasyncio as asyncio

from machine import Pin

# ----------------------------------------------------------------------------
#  Configuration
# ----------------------------------------------------------------------------
SSID      = "ROCKET-GS"
PASSWORD  = "rocket12345"        # WPA2 requires at least 8 characters

# That default is published in a public repository -- and joining this network
# now means being able to ARM AND FIRE THE LATCH.  Put a real password in
# station_secret.py on the board (it is git-ignored, never committed):
#     AP_PASSWORD = "something long"
try:
    from station_secret import AP_PASSWORD
    if 8 <= len(AP_PASSWORD) <= 63:
        PASSWORD = AP_PASSWORD
        PASSWORD_IS_DEFAULT = False
    else:
        print("station_secret.py: AP_PASSWORD must be 8-63 characters; "
              "using the default")
        PASSWORD_IS_DEFAULT = True
except ImportError:
    PASSWORD_IS_DEFAULT = True
CHANNEL   = 6
PORT      = 80
RATE_HZ   = 25                   # telemetry frames per second
FRAME_DT  = 1.0 / RATE_HZ

# How long the source may go quiet before this station stops forwarding frames.
# Deliberately tighter than the viewer's own 1.5 s staleness timeout, so the
# viewer's timer starts promptly rather than both waiting on each other.
SOURCE_STALE_MS = 1000

# --- wired link (v0.3.1 radio stand-in) -------------------------------------
# UART0 on GP0 (TX) / GP1 (RX), matching the flight computer's Serial1.  The
# baud rate must match rocket_diagnostics exactly; 460800 is chosen on THAT
# side, because the Mbed core's UART write blocks the flight loop per byte.
DEFAULT_MODE    = "uart"
LINK_UART_ID    = 0
LINK_TX_PIN     = 0
LINK_RX_PIN     = 1
LINK_BAUD       = 460800
LINK_RXBUF      = 2048           # a few hundred ms of lines; ample
LINK_MIN_FIELDS = 8              # ax..vel: the oldest frame the viewer takes
LINK_MAX_FIELDS = 40
LINK_MAX_LINE   = 512            # bytes with no newline = not our framing

# --- uplink (phone -> station -> wire -> flight computer) -------------------
# Only these reach the wire, and only in canonical form -- whatever a client
# sends is rebuilt from the allowlist, never passed through.  No 'v', 'c' or
# 'h': those only change what the flight computer's USB port prints.
CMD_PLAIN       = ("arm", "safe", "fire", "latch", "srv", "hb", "bz", "z", "b", "r", "g")
CMD_BUZ         = ("off", "chirp", "double", "locate", "alarm")
CMD_MAX_LEN     = 40
SERVO_US_MIN    = 600            # must match the flight computer's clamp
SERVO_US_MAX    = 2400
MSG_KEEP        = 120            # board messages held for late-joining phones
MSG_REPLAY      = 60             # how many a newly connected phone is sent

# --- flight profile timing, shared by the motion model and the GNSS model ----
# These were local to _flight(). They are module-level now because _gps() has
# to know the same phase boundaries -- two copies of "when is apogee" would
# drift apart the first time anyone tuned the profile.
FL_LOOP      = 40.0             # s, whole loop
FL_T_PAD     = 5.0              # s on the pad before launch
FL_BURN_T    = 1.6              # s of burn
FL_BURN_ACC  = 5.0              # g, net of gravity
FL_G         = 9.81
FL_V_BURNOUT = FL_BURN_ACC * FL_G * FL_BURN_T
FL_T_COAST   = FL_V_BURNOUT / FL_G
FL_T_APOGEE  = FL_T_PAD + FL_BURN_T + FL_T_COAST
# Lock returns a few seconds past apogee, once the vehicle is under chute and
# gentle enough for the receiver to reacquire.
FL_T_REACQ   = FL_T_APOGEE + 6.0

# The LSM6 is configured to +/-2 g on the bench, so that is what it can report.
# Anything above it is the ADXL375's job -- which is the entire reason the
# high-g part is fitted, and worth making visible in the synthetic source too.
LSM6_RANGE_G = 2.0

# Synthetic pad position (Mapua Intramuros). Only the offsets matter to the
# trajectory view; the absolute point just has to be plausible.
PAD_LAT = 14.5906
PAD_LON = 120.9878
M_PER_DEG_LAT = 111320.0

# Cold-start convergence. A receiver does not go from nothing to a good fix --
# it holds a 3D fix with four or five satellites and tens of metres of error
# for a while first. That interval is exactly when a naive viewer grabs its pad
# datum, so the synthetic source has to reproduce it or the quality gate that
# now guards the datum would never be exercised outside a real bring-up.
ACQ_DEAD_S   = 3.0      # s before any fix at all
ACQ_TAU_S    = 6.0      # s, accuracy improvement time constant
ACQ_HACC_0   = 40.0     # m of horizontal error just after first fix
ACQ_HACC_INF = 2.5      # m, settled open-sky accuracy

led = Pin("LED", Pin.OUT)

# ----------------------------------------------------------------------------
#  Synthetic telemetry
#
#  Three profiles.  "bench" is the default because it matches what you see
#  with the board sitting on a desk being waved around — the case you are
#  actually testing the UI against.  "flight" exercises the dynamic range
#  (peak altitude, high acceleration, spin) so you can check the readouts
#  and charts do not break on realistic flight numbers.
# ----------------------------------------------------------------------------
class Telemetry:
    def __init__(self):
        self.mode = DEFAULT_MODE
        self.t = 0.0
        # Wired-link bookkeeping.  Counted in EVERY mode, so /health can say
        # whether the wire works even while you are looking at a synthetic
        # profile -- a link you can only check by trusting it is no check.
        self.link_ok = 0
        self.link_bad = 0
        self.link_last = None
        self.link_buf = b""
        self.link_msgs = 0
        # Board messages ("M," lines) for the phones' consoles.  A log, not a
        # latest-value: each client must see each message once, so they carry
        # sequence numbers and every stream tracks where it has got to.
        self.msgs = []
        self.msg_seq = 0
        self.cmd_sent = 0
        self.cmd_refused = 0
        self.boot = time.ticks_ms()
        self.frames = 0
        self.latest = ("V,0.0000,0.0000,1.0000,0.00,0.00,0.00,0.000,0.000,0"
                       ",1.00,0,0,0.0000000,0.0000000,-1.00,-1,0")
        # When the SOURCE last produced a frame -- not when one was last sent
        # to a browser. The difference is the whole point: a silent radio and a
        # motionless rocket both leave `latest` unchanged, and only this
        # timestamp tells them apart.
        self.updated = time.ticks_ms()


    def set_mode(self, m):
        if m in ("uart", "bench", "flight", "still"):
            self.mode = m
            self.t = 0.0
            return True
        return False

    def source(self):
        return "uart" if self.mode == "uart" else "synthetic"

    def tick(self, dt):
        """Advance the synthetic model -- unless the wire is the source.

        In uart mode the model must not run at all.  If it did, it would keep
        calling set_line() and refreshing the freshness timestamp, and a dead
        wire would never read as stale.  That is the staleness machinery
        defeated from the inside.
        """
        if self.mode != "uart":
            self.step(dt)

    def accept_line(self, raw):
        """One line off the wire.  Adopted only if it is a whole frame.

        Validated here, before it reaches a browser, because a wire delivers
        things a clean producer never does: the tail of a line cut off when
        the cable went in, a board rebooting mid-frame, noise on a loose
        jumper.  The viewer would reject most of it too, but it should never
        have to -- and a half-line that happens to parse is worse than one
        that does not.

        Returns True if the line was a valid frame (whether or not it was
        used; it is only adopted in uart mode).
        """
        try:
            if isinstance(raw, (bytes, bytearray)):
                raw = raw.decode()
            line = raw.strip()
        except Exception:
            self.link_bad += 1
            return False
        if line.startswith("M,"):
            # Something the flight computer SAID -- an acknowledgement, a
            # refusal, a status report.  Forwarded to the phones' consoles, and
            # only while the wire is the selected source: a board message
            # scrolling past under a synthetic display would be describing a
            # vehicle that is not the one on screen.
            self.link_msgs += 1
            self.link_last = time.ticks_ms()
            if self.mode == "uart":
                self.push_msg(line[2:])
            return True
        if not line.startswith("V,"):
            self.link_bad += 1
            return False
        parts = line[2:].split(",")
        if not (LINK_MIN_FIELDS <= len(parts) <= LINK_MAX_FIELDS):
            self.link_bad += 1
            return False
        try:
            for f in parts:
                float(f)
        except ValueError:
            self.link_bad += 1
            return False

        self.link_ok += 1
        self.link_last = time.ticks_ms()
        if self.mode == "uart":
            self.set_line(line)
        return True

    def feed(self, chunk):
        """Raw bytes off the UART, in whatever pieces they arrived.

        Lives here rather than in uart_reader so it can be tested: this is
        where wire bugs actually hide -- a frame split across two reads, half
        a line left over from when the cable went in, a baud mismatch that
        produces a stream with no newlines at all.
        """
        if chunk:
            self.link_buf += chunk
        while True:
            i = self.link_buf.find(b"\n")
            if i < 0:
                break
            self.accept_line(self.link_buf[:i])
            self.link_buf = self.link_buf[i + 1:]
        if len(self.link_buf) > LINK_MAX_LINE:
            # This much without a newline is not our framing -- a baud
            # mismatch, most likely.  Count it and resynchronise.
            self.link_bad += 1
            self.link_buf = b""

    def push_msg(self, text):
        self.msg_seq += 1
        self.msgs.append((self.msg_seq, text))
        if len(self.msgs) > MSG_KEEP:
            self.msgs.pop(0)

    def msgs_since(self, seq):
        return [m for m in self.msgs if m[0] > seq]

    def prepare_command(self, raw):
        """A command from a phone -> (canonical command, None) or (None, why).

        Everything that decides what may reach the flight computer is in this
        one function, so it can be tested without a board:

          * Board commands are refused unless the WIRE is the selected source.
            Arming a real latch while the display shows synthetic data would
            mean the latch state on screen belongs to a vehicle that does not
            exist -- the display's one job, inverted.
          * The command is rebuilt from the allowlist rather than forwarded.
            Nothing a client types passes through verbatim: unknown words,
            extra arguments, out-of-range numbers, stray bytes are all refused.
        """
        def refuse(why):
            self.cmd_refused += 1
            return None, why

        if self.mode != "uart":
            return refuse("the display is on a synthetic source; board "
                          "commands are only accepted on the wired link")

        # Minimal percent-decoding: MicroPython has no urllib, and the
        # station's query parser leaves values encoded.
        text = raw.replace("+", " ")
        out, i = [], 0
        while i < len(text):
            ch = text[i]
            if ch == "%" and i + 3 <= len(text):
                try:
                    out.append(chr(int(text[i + 1:i + 3], 16)))
                    i += 3
                    continue
                except ValueError:
                    pass
            out.append(ch)
            i += 1
        c = "".join(out).strip()

        if c.startswith("!"):
            c = c[1:]
        if not c or len(c) > CMD_MAX_LEN:
            return refuse("empty or too long")
        for ch in c:
            if not (ch == " " or "a" <= ch <= "z" or "0" <= ch <= "9"):
                return refuse("unexpected character")
        w = c.split()

        if len(w) == 1 and w[0] in CMD_PLAIN:
            return "!" + w[0], None
        if len(w) == 2 and w[0] == "buz" and w[1] in CMD_BUZ:
            return "!buz " + w[1], None
        if len(w) == 2 and w[0] == "us" and w[1].isdigit():
            n = int(w[1])
            if SERVO_US_MIN <= n <= SERVO_US_MAX:
                return "!us %d" % n, None
            return refuse("pulse width outside %d-%d us"
                          % (SERVO_US_MIN, SERVO_US_MAX))
        if len(w) == 3 and w[0] == "pos" and w[1].isdigit() and w[2].isdigit():
            a, b = int(w[1]), int(w[2])
            if SERVO_US_MIN <= a <= SERVO_US_MAX and SERVO_US_MIN <= b <= SERVO_US_MAX:
                return "!pos %d %d" % (a, b), None
            return refuse("endpoint outside %d-%d us" % (SERVO_US_MIN, SERVO_US_MAX))
        if len(w) == 3 and w[0] == "beep" and w[1].isdigit() and w[2].isdigit():
            hz, ms = int(w[1]), int(w[2])
            if 100 <= hz <= 20000 and 1 <= ms <= 5000:
                return "!beep %d %d" % (hz, ms), None
            return refuse("beep outside 100-20000 Hz / 1-5000 ms")
        return refuse("not an allowed command")

    def link_age_ms(self):
        if self.link_last is None:
            return -1
        return time.ticks_diff(time.ticks_ms(), self.link_last)

    # --- helpers ---------------------------------------------------------
    @staticmethod
    def _grav_from_tilt(pitch, roll):
        """Gravity direction in the body frame for a given pitch/roll."""
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        return (-sp, cp * sr, cp * cr)

    # --- profiles --------------------------------------------------------
    def _bench(self, dt):
        t = self.t
        pitch = math.radians(14.0) * math.sin(t * 0.55) + math.radians(3.0) * math.sin(t * 2.7)
        roll  = math.radians(9.0) * math.sin(t * 0.8)
        ax, ay, az = self._grav_from_tilt(pitch, roll)

        # body rates, roughly the derivative of the above
        gx = math.degrees(math.radians(14.0) * 0.55 * math.cos(t * 0.55)
                          + math.radians(3.0) * 2.7 * math.cos(t * 2.7))
        gy = math.degrees(math.radians(9.0) * 0.8 * math.cos(t * 0.8))
        gz = 2.0 * math.sin(t * 0.35)

        alt = 0.25 * math.sin(t * 0.4) + 0.04 * math.sin(t * 3.1)
        vel = 0.10 * math.cos(t * 0.4)
        return ax, ay, az, gx, gy, gz, alt, vel

    def _flight(self, dt):
        """Idle -> boost -> coast -> apogee -> descent, on a 40 s loop.

        Numbers are sized for a realistic school / mid-power flight rather
        than something spectacular: ~5 g burn for 1.6 s, apogee near 377 m,
        chute descent at 6 m/s.  Deliberately chosen so the boost phase
        exceeds a +/-2 g accelerometer — that clipping is real and the
        viewer should show it rather than hide it.
        """
        BURN_T   = FL_BURN_T
        BURN_ACC = FL_BURN_ACC
        T_PAD    = FL_T_PAD
        g        = FL_G

        v_burnout = BURN_ACC * g * BURN_T
        a_burnout = 0.5 * BURN_ACC * g * BURN_T * BURN_T
        t_coast   = v_burnout / g
        apogee    = a_burnout + v_burnout * t_coast - 0.5 * g * t_coast * t_coast
        t_apogee  = T_PAD + BURN_T + t_coast

        t = self.t % FL_LOOP
        if t < T_PAD:                                  # on the pad
            alt, vel, thrust, spin = 0.0, 0.0, 0.0, 0.0
        elif t < T_PAD + BURN_T:                       # boost
            b = t - T_PAD
            thrust = BURN_ACC
            vel = BURN_ACC * g * b
            alt = 0.5 * BURN_ACC * g * b * b
            spin = 90.0
        elif t < t_apogee:                             # coast to apogee
            b = t - (T_PAD + BURN_T)
            thrust = 0.0
            vel = v_burnout - g * b
            alt = a_burnout + v_burnout * b - 0.5 * g * b * b
            spin = 90.0 * math.exp(-b * 0.4)
        else:                                          # descent under chute
            b = t - t_apogee
            thrust = 0.0
            vel = -6.0
            alt = apogee - 6.0 * b
            spin = 20.0 * math.sin(b * 1.7)
            if alt < 0.0:
                alt, vel = 0.0, 0.0

        # attitude: near vertical through boost and coast, tipping after apogee
        if t < t_apogee:
            pitch = math.radians(3.0) * math.sin(t * 1.3)
        else:
            b = t - t_apogee
            pitch = math.radians(min(70.0, b * 12.0)) + math.radians(6.0) * math.sin(t * 2.2)
        roll = math.radians(5.0) * math.sin(t * 0.9)

        gxv, gyv, gzv = self._grav_from_tilt(pitch, roll)
        scale = 1.0 + thrust                # 1 g at rest, 6 g under thrust
        ax, ay, az = gxv * scale, gyv * scale, gzv * scale

        gx = 6.0 * math.sin(t * 1.3)
        gy = 4.0 * math.cos(t * 0.9)
        gz = spin
        return ax, ay, az, gx, gy, gz, alt, vel

    def _still(self, dt):
        return 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0

    def _acq(self):
        """Satellites and horizontal accuracy as the receiver settles.

        Keyed on total elapsed time, not the flight loop's phase: a real
        receiver converges once and stays converged, it does not cold-start
        again every 40 s.
        """
        age = self.t
        if age < ACQ_DEAD_S:
            return 2, -1.0
        sats = min(9, 4 + int((age - ACQ_DEAD_S) / 4.0))
        hacc = ACQ_HACC_INF + ACQ_HACC_0 * math.exp(-(age - ACQ_DEAD_S) / ACQ_TAU_S)
        return sats, hacc

    def _gps(self):
        """Synthetic GNSS -> (fix, sats, lat, lon, hacc).

        The flight profile deliberately DROPS LOCK from launch until a few
        seconds past apogee, then reacquires at a drifted position. That is
        what a real receiver does -- it is not a flight-phase sensor, it loses
        lock at launch and takes 5-15 s to come back -- and it is the case the
        trajectory view has to render as an honest gap rather than interpolate
        a smooth arc across. If the synthetic source never dropped lock, that
        code path would never be exercised until a real flight, which is the
        worst possible time to discover it draws a fictional curve.
        """
        lat_scale = M_PER_DEG_LAT
        lon_scale = M_PER_DEG_LAT * math.cos(PAD_LAT * math.pi / 180.0)

        # A metre or two of the wander any stationary receiver shows, so the
        # trajectory view has something non-degenerate to draw on the bench.
        j = ((self.frames * 22695477 + 1) >> 16) & 0xFFFF
        w = (j / 32768.0) - 1.0

        def pad_fix():
            if self.mode == "flight":
                # Already converged: the vehicle has been sitting on the pad
                # for far longer than this compressed 40 s loop represents, so
                # starting it mid-acquisition would misrepresent launch day.
                sats, hacc = 9, 2.5
            else:
                sats, hacc = self._acq()
            if hacc < 0:
                return (0, sats, 0.0, 0.0, -1.0)
            # Wander scales with the accuracy the receiver is claiming, so an
            # early marginal fix moves around by tens of metres and a settled
            # one by a metre or two -- which is what the real part does.
            return (3, sats,
                    PAD_LAT + (w * hacc * 0.6) / lat_scale,
                    PAD_LON + (w * hacc * 0.5) / lon_scale,
                    hacc)

        if self.mode != "flight":
            return pad_fix()

        t = self.t % FL_LOOP
        if t < FL_T_PAD:
            return pad_fix()
        if t < FL_T_REACQ:
            # No lock. Report no position at all rather than the last one --
            # a stale position repeated is indistinguishable from a live one.
            return (0, 2, 0.0, 0.0, -1.0)

        # Reacquired under the chute, drifting downwind at ~3 m/s since launch.
        drift = 3.0 * (t - FL_T_PAD)
        return (3, 7,
                PAD_LAT + (drift * 0.60) / lat_scale,
                PAD_LON + (drift * 0.80) / lon_scale,
                4.5)

    # --- frame -----------------------------------------------------------
    def step(self, dt):
        """Advance the simulation by dt and cache the resulting wire line.

        Deliberately separate from reading it. Every connected browser gets
        its own SSE loop, so if each one advanced the clock, two viewers
        would run the simulation at double speed and disagree with each
        other. One producer, many consumers — which is also exactly the
        shape the radio link needs.
        """
        self.t += dt
        if self.mode == "flight":
            v = self._flight(dt)
        elif self.mode == "still":
            v = self._still(dt)
        else:
            v = self._bench(dt)
        ax, ay, az, gx, gy, gz, alt, vel = v

        # A little noise so the smoothing and the noise readouts have
        # something realistic to chew on.
        n = self.frames
        j = ((n * 1103515245 + 12345) >> 8) & 0xFFFF
        r = (j / 32768.0) - 1.0
        ax += r * 0.010
        ay += r * 0.008
        az += r * 0.010
        alt += r * 0.06

        self.frames += 1
        ms = time.ticks_diff(time.ticks_ms(), self.boot)

        # The high-g field carries the TRUE magnitude; the LSM6 fields are
        # clipped to the range that part is actually configured for. The
        # profile peaks at 6 g, so on the flight profile the two now disagree
        # exactly as the real hardware would -- which is the whole argument for
        # fitting an ADXL375, and it means the high-g readout can be developed
        # and tested without waiting for a launch.
        hg = math.sqrt(ax * ax + ay * ay + az * az)
        ax = max(-LSM6_RANGE_G, min(LSM6_RANGE_G, ax))
        ay = max(-LSM6_RANGE_G, min(LSM6_RANGE_G, ay))
        az = max(-LSM6_RANGE_G, min(LSM6_RANGE_G, az))

        fix, sats, lat, lon, hacc = self._gps()
        # srv = -1: this station has no latch of its own to report. Over the
        # radio the flight computer's real state would arrive here instead.
        self.set_line(
            "V,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.3f,%.3f,%d,%.2f,%d,%d,%.7f,%.7f,%.2f,-1,0" % (
                ax, ay, az, gx, gy, gz, alt, vel, ms, hg, fix, sats, lat, lon, hacc))
        return self.latest

    def set_line(self, line):
        """The only way a new frame enters. Every source -- the synthetic
        model today, the radio packet reader later -- goes through here, so
        the freshness timestamp cannot be forgotten by a new producer."""
        self.latest = line
        self.updated = time.ticks_ms()

    def stale(self):
        """True when the source has stopped producing. Consumers use this to
        stop forwarding, rather than repeating the last frame forever and
        making a dead link look like a live one."""
        return time.ticks_diff(time.ticks_ms(), self.updated) > SOURCE_STALE_MS

    def frame(self):
        """Most recent wire line. Consumers call this; they never advance."""
        return self.latest


tel = Telemetry()
clients = 0


# ----------------------------------------------------------------------------
#  Wi-Fi access point
# ----------------------------------------------------------------------------
def start_ap():
    ap = network.WLAN(network.AP_IF)
    ap.config(essid=SSID, password=PASSWORD)
    try:
        ap.config(channel=CHANNEL)
    except Exception:
        pass                      # some ports do not expose channel

    # The CYW43 radio enables power management by default, which parks the
    # chip between packets and adds bursty latency — typically fine for
    # request/response, but visible as a stutter on a steady 25 Hz stream.
    # PM_NONE keeps the radio awake. Costs a little current; this is mains
    # or power-bank powered on the ground, so that is the right trade.
    try:
        ap.config(pm=0xa11140)    # == network.WLAN.PM_NONE
    except Exception:
        try:
            ap.config(pm=ap.PM_NONE)
        except Exception:
            print("note: could not disable Wi-Fi power management")

    ap.active(True)

    t0 = time.ticks_ms()
    while not ap.active():
        if time.ticks_diff(time.ticks_ms(), t0) > 10000:
            raise RuntimeError("AP failed to start")
        led.toggle()
        time.sleep(0.2)

    led.value(1)
    cfg = ap.ifconfig()
    print()
    print("=" * 52)
    print("  ROCKET GROUND STATION")
    print("=" * 52)
    print("  SSID     :", SSID)
    print("  Password :", PASSWORD)
    print("  Open     : http://%s" % cfg[0])
    print("=" * 52)
    print()
    return ap


# ----------------------------------------------------------------------------
#  HTTP
# ----------------------------------------------------------------------------
def _exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


async def send_headers(w, status, ctype, extra=None, close=True):
    lines = ["HTTP/1.1 %s" % status,
             "Content-Type: %s" % ctype,
             "Cache-Control: no-store"]
    if extra:
        lines.extend(extra)
    lines.append("Connection: %s" % ("close" if close else "keep-alive"))
    w.write(("\r\n".join(lines) + "\r\n\r\n").encode())
    await w.drain()


async def serve_file(w, path, ctype):
    """Stream from flash in chunks — the viewer is ~45 KB and the Pico W
    does not have the RAM headroom to hold it and a TLS-free socket buffer
    comfortably at the same time."""
    if not _exists(path):
        await send_headers(w, "404 Not Found", "text/plain")
        w.write(b"index.html is not on the board.\r\n"
                b"Copy the viewer to the Pico W as index.html and reset.\r\n")
        await w.drain()
        return
    await send_headers(w, "200 OK", ctype)
    with open(path, "rb") as f:
        while True:
            chunk = f.read(1024)
            if not chunk:
                break
            w.write(chunk)
            await w.drain()


def _nodelay(w):
    """Turn off Nagle's algorithm for a stream.

    SSE frames are ~60 bytes. Nagle holds a small packet back until the
    previous one is ACKed, and the phone's TCP stack delays ACKs by up to
    ~200 ms — the two together produce exactly the intermittent multi-frame
    stall this is here to prevent. MicroPython exposes the underlying socket
    differently across versions, so try the known spellings and give up
    quietly rather than crash the stream.
    """
    s = None
    for attr in ("s", "_s"):
        s = getattr(w, attr, None)
        if s is not None:
            break
    if s is None:
        try:
            s = w.get_extra_info("socket")
        except Exception:
            s = None
    if s is None:
        return False
    try:
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return True
    except Exception:
        return False


async def serve_stream(w):
    """Server-Sent Events. One telemetry frame per event, in exactly the
    same 'V,...' format the serial path already emits, so the viewer's
    existing parser handles both without a second code path."""
    global clients
    clients += 1
    _nodelay(w)
    await send_headers(w, "200 OK", "text/event-stream",
                       extra=["Access-Control-Allow-Origin: *",
                              "X-Accel-Buffering: no"],
                       close=False)
    # tell the client how fast to expect frames
    w.write(("retry: 1000\n\nevent: hello\ndata: %d\n\n" % RATE_HZ).encode())
    await w.drain()
    # A phone that connects late still sees the recent past -- the boot
    # banner, the last few acknowledgements -- rather than a blank console.
    last_msg = tel.msg_seq - MSG_REPLAY
    try:
        while True:
            # Messages go out whether or not telemetry is stale: while the
            # board is blocked zeroing the barometer, its progress text is
            # exactly what the phone should be showing.
            for seq, text in tel.msgs_since(last_msg):
                w.write(("event: msg\ndata: %s\n\n" % text).encode())
                last_msg = seq
            if tel.stale():
                # Source has gone quiet. Send an SSE comment instead of a data
                # frame: EventSource ignores comment lines, so the connection
                # and its retry timer stay alive while the viewer's own
                # staleness timeout fires. Repeating the last frame here would
                # defeat that timeout entirely -- frames would keep arriving,
                # they would just all say the same thing, which is precisely
                # how "radio silent" ends up looking like "sitting still".
                w.write(b": stale\n\n")
            else:
                w.write(("data: %s\n\n" % tel.frame()).encode())
            await w.drain()
            await asyncio.sleep(FRAME_DT)
    except (OSError, AttributeError):
        pass                      # client went away
    finally:
        clients -= 1


async def serve_health(w):
    try:
        free = gc.mem_free()          # MicroPython only
    except AttributeError:
        free = -1
    body = json.dumps({
        "ok": True,
        "mode": tel.mode,
        "rate_hz": RATE_HZ,
        "frames": tel.frames,
        "clients": clients,
        "mem_free": free,
        "uptime_s": time.ticks_diff(time.ticks_ms(), tel.boot) // 1000,
        "source": tel.source(),
        "link": {
            "baud": LINK_BAUD,
            "lines_ok": tel.link_ok,
            "lines_bad": tel.link_bad,
            "last_line_age_ms": tel.link_age_ms(),
            "messages": tel.link_msgs,
        },
        "commands": {"sent": tel.cmd_sent, "refused": tel.cmd_refused},
        "default_password": PASSWORD_IS_DEFAULT,
        "stale": tel.stale(),
        "source_age_ms": time.ticks_diff(time.ticks_ms(), tel.updated),
    })
    await send_headers(w, "200 OK", "application/json")
    w.write(body.encode())
    await w.drain()


def _query(path):
    if "?" not in path:
        return path, {}
    base, qs = path.split("?", 1)
    out = {}
    for pair in qs.split("&"):
        if "=" in pair:
            k, v = pair.split("=", 1)
            out[k] = v
    return base, out


async def handle(r, w):
    try:
        req = await r.readline()
        if not req:
            return
        try:
            method, path, _ = req.decode().split(" ", 2)
        except ValueError:
            return

        # drain headers
        while True:
            h = await r.readline()
            if not h or h in (b"\r\n", b"\n"):
                break

        base, q = _query(path)

        if base in ("/", "/index.html"):
            await serve_file(w, "index.html", "text/html; charset=utf-8")
        elif base == "/stream":
            await serve_stream(w)
        elif base == "/health":
            await serve_health(w)
        elif base == "/mode":
            ok = tel.set_mode(q.get("m", ""))
            await send_headers(w, "200 OK" if ok else "400 Bad Request",
                               "application/json")
            w.write(json.dumps({"ok": ok, "mode": tel.mode}).encode())
            await w.drain()
        elif base == "/cmd":
            code, body = run_command(q.get("c", ""))
            await send_headers(w, code, "application/json")
            w.write(json.dumps(body).encode())
            await w.drain()
        elif base == "/favicon.ico":
            await send_headers(w, "204 No Content", "text/plain")
        else:
            await send_headers(w, "404 Not Found", "text/plain")
            w.write(b"not found")
            await w.drain()
    except Exception as e:
        print("handler error:", e)
    finally:
        try:
            await w.drain()
            w.close()
            await w.wait_closed()
        except Exception:
            pass
        gc.collect()


# ----------------------------------------------------------------------------
#  Housekeeping
# ----------------------------------------------------------------------------
async def blink():
    """Slow pulse when idle, steady when someone is streaming."""
    while True:
        if clients > 0:
            led.value(1)
            await asyncio.sleep(1)
        else:
            led.toggle()
            await asyncio.sleep(0.6)


async def sampler():
    """The single producer. Advances the synthetic model at a steady rate
    regardless of how many browsers are connected. When the radio arrives
    this task is replaced by the packet reader."""
    while True:
        tel.tick(FRAME_DT)
        await asyncio.sleep(FRAME_DT)


async def janitor():
    while True:
        await asyncio.sleep(15)
        gc.collect()


# =============================================================================
#  WIRED LINK  (v0.3.1 -- stand-in for the radio)
#
#  The flight computer's Serial1 lands on GP1 and becomes a producer like any
#  other: every whole frame goes through tel.accept_line() into set_line(), so
#  staleness, SSE forwarding and the viewer all behave exactly as they will
#  with a radio.  That is the point of the stand-in -- it tests everything
#  downstream of the receiver, and nothing about the receiver.
#
#  WHEN THE RADIO ARRIVES, this task is what gets replaced -- and note the
#  replacement will NOT be a UART.  An RFM95W is an SPI transceiver; it gets a
#  LoRa driver on SPI, decodes the 18-byte packet of 6.2, formats a "V,..."
#  line, and hands it to tel.accept_line().  Same slot, different plumbing.
#
#  It now carries commands UP as well, from the phones, through run_command()
#  and prepare_command() -- a deliberate choice for the bench rig.  In flight
#  the latch arms by a reed switch (8.4), and none of this carries over.
# =============================================================================
link_uart = None                     # set by uart_reader once the port is open


def run_command(raw):
    """Relay one allowed command to the flight computer.

    "ok" means the station put it on the wire -- NOT that the board did it.
    Confirmation comes back the way everything else does: the board's own
    message ("SERVO ARMED ...") and the latch state in the next telemetry frame.
    """
    cmd, why = tel.prepare_command(raw)
    if cmd is None:
        code = "409 Conflict" if tel.mode != "uart" else "400 Bad Request"
        return code, {"ok": False, "why": why}
    if link_uart is None:
        return "503 Service Unavailable", {"ok": False, "why": "wired link not open"}
    link_uart.write((cmd + "\n").encode())
    tel.cmd_sent += 1
    return "200 OK", {"ok": True, "sent": cmd}


def open_link():
    try:
        from machine import UART
    except ImportError:
        return None
    try:
        return UART(LINK_UART_ID, baudrate=LINK_BAUD,
                    tx=Pin(LINK_TX_PIN), rx=Pin(LINK_RX_PIN), rxbuf=LINK_RXBUF)
    except TypeError:
        # MicroPython builds without the rxbuf keyword.  The default ring
        # buffer is smaller but still holds more than one 25 Hz frame.
        return UART(LINK_UART_ID, baudrate=LINK_BAUD,
                    tx=Pin(LINK_TX_PIN), rx=Pin(LINK_RX_PIN))


async def uart_reader():
    global link_uart
    uart = open_link()
    link_uart = uart
    if uart is None:
        print("Wired link: no UART on this platform")
        return
    print("Wired link: UART%d on GP%d (RX) at %d baud"
          % (LINK_UART_ID, LINK_RX_PIN, LINK_BAUD))
    while True:
        n = uart.any()
        if n:
            tel.feed(uart.read(n))
        await asyncio.sleep(0.005)
# =============================================================================


async def main():
    start_ap()
    asyncio.create_task(sampler())
    asyncio.create_task(uart_reader())
    asyncio.create_task(blink())
    asyncio.create_task(janitor())
    server = await asyncio.start_server(handle, "0.0.0.0", PORT)
    print("HTTP server listening on port %d" % PORT)
    if PASSWORD_IS_DEFAULT:
        print("WARNING: default AP password - it is in a public repository, and")
        print("         this network can arm the latch. Create station_secret.py.")
    if tel.mode == "uart":
        print("Telemetry source: WIRED LINK (uart) - no synthetic fallback")
    else:
        print("Telemetry source: SYNTHETIC (%s) at %d Hz" % (tel.mode, RATE_HZ))
    print("Waiting for a browser...")
    while True:
        await asyncio.sleep(3600)


try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("stopped")
finally:
    asyncio.new_event_loop()
