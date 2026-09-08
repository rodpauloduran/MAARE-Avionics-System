# Rocket Ground Station — Pico W

Turns a Raspberry Pi Pico W into a self-contained Wi-Fi access point that
serves the attitude viewer and streams telemetry to your phone. No router,
no internet, no app install.

**Telemetry is synthetic right now.** The radio link isn't wired yet — this
exists to prove out the viewing and UI half of the system. There's one
function to replace when the radio arrives (see below).

Design reasoning lives in [`avionics_documentation.md`](../avionics_documentation.md)
§6.5 (ground station) and §6.7 (viewer). That document is the authority if the
two ever disagree.

## Files

| File | Goes on the board as | What it is |
|---|---|---|
| `main.py` | `main.py` | AP + web server + telemetry source |
| `index.html` | `index.html` | The viewer (network-aware build) |

Total ~61 KB. The Pico W has roughly 1.4 MB of filesystem free after
MicroPython, so space is not a concern.

## Setup

1. **Flash MicroPython.** Download the `rp2-pico-w` UF2 from
   micropython.org. Hold BOOTSEL, plug in, drop the UF2 on the `RPI-RP2`
   drive. Make sure it's the **Pico W** build — the plain Pico build has no
   networking and `import network` will fail.

2. **Install Thonny** (or `mpremote`/`rshell` if you prefer). Set the
   interpreter to *MicroPython (Raspberry Pi Pico)*.

3. **Copy both files to the board.** In Thonny: open each file, then
   *File → Save as… → Raspberry Pi Pico*. Names must be exactly `main.py`
   and `index.html`.

4. **Reset the board.** `main.py` runs automatically on boot.

## Use

1. Phone Wi-Fi → join **`ROCKET-GS`**, password **`rocket12345`**
2. Browser → **http://192.168.4.1**

Android may warn that the network has no internet and offer to switch back
to mobile data — choose to stay connected. On iOS, turn off *Wi-Fi Assist*
if it keeps dropping you.

The onboard LED blinks slowly when idle and goes solid while someone is
streaming.

## What you'll see

The viewer detects it's being served over HTTP and switches from Web Serial
to Server-Sent Events automatically. The serial-mode buttons are replaced
with a **source selector**:

- **Bench wobble** (default) — gentle motion, like the board on a desk.
  This is the realistic case for checking the UI.
- **Flight profile** — a full flight on a 40 s loop: 1.6 s burn at 5 g,
  apogee near 377 m, 78 m/s peak ascent, chute descent at 6 m/s, tipping
  over after apogee. Use this to check the readouts and charts survive real
  flight numbers.
- **Hold still** — flat output, for checking noise and drift behaviour.

The flight profile deliberately hits 6 g, which would clip a ±2 g
accelerometer. That's intentional — it's a real limitation and the viewer
should show it rather than hide it.

## Endpoints

| Path | Purpose |
|---|---|
| `/` | The viewer |
| `/stream` | SSE telemetry, one `V,...` frame per event |
| `/health` | JSON status — mode, frame count, clients, free RAM, uptime |
| `/mode?m=bench\|flight\|still` | Switch the synthetic source |

`/health` is useful from a laptop while debugging:
`curl http://192.168.4.1/health`

## Configuration

At the top of `main.py`:

```python
SSID     = "ROCKET-GS"
PASSWORD = "rocket12345"     # WPA2 needs 8+ characters
CHANNEL  = 6
RATE_HZ  = 25
```

Change the password before any public demo — it's in plain text and anyone
who reads this repo can join your network.

## Wiring in the radio

The wire format (§6.6) is identical to what the flight sketch already emits
over serial (`V,ax,ay,az,gx,gy,gz,alt,vel,ms`), so the viewer needs **no
changes**. Keep the trailing `ms` field when the radio goes in: it is the
flight computer's own `millis()`, and it is the only thing that makes the
arrival-time jitter of a lossy RF link harmless to the attitude estimate.

The architecture is already producer/consumer: one task produces frames, any
number of browsers consume them.

Replace the `sampler()` task with a packet reader:

```python
from machine import UART, Pin
uart = UART(0, 115200, tx=Pin(0), rx=Pin(1))

async def radio_reader():
    buf = b""
    while True:
        if uart.any():
            buf += uart.read()
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if line.startswith(b"V,"):
                    tel.latest = line.decode()   # consumers pick this up
        await asyncio.sleep(0.005)
```

Then swap `asyncio.create_task(sampler())` for
`asyncio.create_task(radio_reader())` in `main()`.

Worth adding at the same time: a staleness check, so the viewer can tell
"radio silent" apart from "rocket sitting still." Both look like unchanging
numbers otherwise.

## Verified on hardware

- All routes return correct status codes and content types
- `index.html` serves byte-identical to disk (md5 match) via chunked reads
- SSE sustains 25.0 Hz with monotonic timestamps (40.6 ms mean interval)
- Three simultaneous clients each get full 25 Hz and **identical** frames,
  with the simulation still advancing at 1× — not 3×
- Flight profile physics check out: 377 m apogee, 78.5 m/s, 6.01 g peak,
  monotonic ascent
- Viewer parses and runs under both `file://` (serial) and `http://`
  (network) modes

## Mobile smoothness

Phone rendering is where this page is easiest to get wrong. These are the
things that keep it smooth — and the first things to check if stutter appears.

**Client side.** Canvas sizes are measured once and cached, not per frame.
Calling `getBoundingClientRect()` forces a synchronous layout reflow, and
reassigning `canvas.width` reallocates and clears the backing buffer; doing
either every frame across four canvases costs 480 reflows and 480
reallocations per 2 s at 60 fps, against 4 of each when sizes are re-measured
only on resize or rotation. On top of that: pixel ratio is capped at 1.5 on
touch devices (at DPR 3 a full-width canvas is ~9x the pixels, for detail
nobody can see at arm's length), render is capped at 30 fps with charts at
10 fps on mobile since telemetry only arrives at 25 Hz, DOM writes are skipped
when the formatted string is unchanged, and drawing halts entirely when the tab
is hidden.

**Server side.** Two settings:

- **Wi-Fi power management disabled** (`pm=0xa11140`). The CYW43 radio parks
  itself between packets by default. That's fine for request/response traffic
  but shows up as a stutter on a steady 25 Hz stream.
- **Nagle's algorithm disabled** on the SSE socket. Frames are ~60 bytes;
  Nagle holds a small packet until the previous is ACKed, and phones delay
  ACKs by up to ~200 ms. Together those produce exactly the intermittent
  multi-frame stall this was reported as.

If stutter appears anyway, the next things to look at are phone Wi-Fi power
saving (some Android builds throttle aggressively on networks with no internet)
and distance from the Pico W — its antenna is small.

## If something's wrong

| Symptom | Cause |
|---|---|
| `index.html is not on the board` | The viewer wasn't copied. Copy it as `index.html` and reset. |
| AP never comes up, LED keeps blinking | Plain Pico, or non-W MicroPython flashed. `import network` needs the Pico **W** build. |
| Stream stutters in multi-frame bursts | Power management or Nagle re-enabled — see above. |
| Two phones disagree | The producer is being advanced per client rather than once. See §6.5. |

## Known limits

- **Synthetic data.** Nothing here reflects a real sensor yet.
- **AP mode only.** The Pico W can't be an access point and join another
  network at the same time in this configuration.
- **Roughly 4 clients.** The Pico W's AP is fine for a small team; it isn't
  a venue-wide server.
- **No HTTPS.** Fine on an isolated field network; don't reuse the password
  anywhere that matters.
- **Phone screen sleep** will pause the stream. SSE reconnects on wake, but
  you'll have a gap in the charts.
- **Disabling Wi-Fi power management raises idle current** by roughly 20-30 mA.
  Irrelevant on a power bank; worth knowing if you ever run the ground station
  from a small cell.
