# =============================================================================
#  ROCKET GROUND STATION  —  Raspberry Pi Pico W
#
#  Brings up its own Wi-Fi access point, serves the attitude viewer, and
#  streams telemetry to any connected phone or laptop over Server-Sent
#  Events.  No router, no internet, no app install: connect to the SSID
#  below and open http://192.168.4.1
#
#  RIGHT NOW the telemetry is SYNTHETIC.  The radio link is not wired yet,
#  so this exists to prove out the viewing and UI half of the system.  When
#  the radio arrives, there is exactly one function to replace — see the
#  block marked "RADIO HOOK" near the bottom.
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
#    /mode?m=     bench | flight | still   — switches the synthetic source
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
CHANNEL   = 6
PORT      = 80
RATE_HZ   = 25                   # telemetry frames per second
FRAME_DT  = 1.0 / RATE_HZ

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
        self.mode = "bench"
        self.t = 0.0
        self.boot = time.ticks_ms()
        self.frames = 0
        self.latest = "V,0.0000,0.0000,1.0000,0.00,0.00,0.00,0.000,0.000,0"


    def set_mode(self, m):
        if m in ("bench", "flight", "still"):
            self.mode = m
            self.t = 0.0
            return True
        return False

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
        BURN_T   = 1.6          # s
        BURN_ACC = 5.0          # g, net of gravity
        T_PAD    = 5.0
        g        = 9.81

        v_burnout = BURN_ACC * g * BURN_T
        a_burnout = 0.5 * BURN_ACC * g * BURN_T * BURN_T
        t_coast   = v_burnout / g
        apogee    = a_burnout + v_burnout * t_coast - 0.5 * g * t_coast * t_coast
        t_apogee  = T_PAD + BURN_T + t_coast

        t = self.t % 40.0
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
        self.latest = "V,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.3f,%.3f,%d" % (
            ax, ay, az, gx, gy, gz, alt, vel, ms)
        return self.latest

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
    try:
        while True:
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
        "source": "synthetic",
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
        tel.step(FRAME_DT)
        await asyncio.sleep(FRAME_DT)


async def janitor():
    while True:
        await asyncio.sleep(15)
        gc.collect()


# =============================================================================
#  RADIO HOOK
#
#  When the radio link is built, this is the only part that changes.
#  Replace Telemetry.frame() with something that reads the most recent
#  packet the receiver has decoded, e.g.:
#
#      from machine import UART
#      uart = UART(0, 115200, tx=Pin(0), rx=Pin(1))
#      _last = "V,0,0,1,0,0,0,0,0,0"
#
#      async def radio_reader():
#          global _last
#          buf = b""
#          while True:
#              if uart.any():
#                  buf += uart.read()
#                  while b"\n" in buf:
#                      line, buf = buf.split(b"\n", 1)
#                      line = line.strip()
#                      if line.startswith(b"V,"):
#                          _last = line.decode()
#              await asyncio.sleep(0.005)
#
#  ...then have the SSE loop send _last instead of tel.frame().  Keep the
#  same "V,..." wire format and the viewer needs no changes at all.
#
#  Add radio_reader() to main() as another asyncio task.
# =============================================================================


async def main():
    start_ap()
    asyncio.create_task(sampler())
    asyncio.create_task(blink())
    asyncio.create_task(janitor())
    server = await asyncio.start_server(handle, "0.0.0.0", PORT)
    print("HTTP server listening on port %d" % PORT)
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
