# Pico W Ground Station

Raspberry Pi Pico W running MicroPython. Brings up its **own Wi-Fi access
point**, serves the attitude viewer, and streams telemetry to any phone or
laptop that joins. No router, no internet, no app install — which is the point,
because a launch site has none of those.

> **Status: built and working on hardware. Telemetry source is synthetic — the
> radio link is not yet wired in.** Nothing in this path has carried a real
> sensor reading over the air. See [Wiring in the radio](#wiring-in-the-radio).

See §6.5 of `AVIONICS_DOCUMENTATION.md` for the design reasoning.

```
   Rocket ──LoRa──▶ [RFM95W] ──UART──▶ Pico W ──Wi-Fi AP──▶ phone browser
                                        │                     (viewer)
                                        └─ serves index.html + SSE stream
```

---

## Install

1. Flash MicroPython for Pico W (the `rp2-pico-w` UF2) from
   [micropython.org](https://micropython.org/download/RPI_PICO_W/).
2. Copy `main.py` to the board as `main.py`.
3. Copy `index.html` to the board as `index.html`.
4. Reset the board. The onboard LED goes solid once the AP is up.

Both files go in the board's filesystem root. Any tool works — `mpremote`,
Thonny, `rshell`:

```
mpremote cp main.py :main.py
mpremote cp index.html :index.html
mpremote reset
```

## Use

| | |
|---|---|
| Phone Wi-Fi | join **`ROCKET-GS`** |
| Browser | **http://192.168.4.1** |

The LED pulses slowly while idle and holds steady while someone is streaming.

> ⚠️ **The Wi-Fi password is a default in plain text in `main.py`.** Change it
> before any public demonstration — see §14 of the design document.

## Endpoints

| Endpoint | Purpose |
|---|---|
| `/` | The viewer |
| `/stream` | Server-Sent Events, one telemetry frame per event |
| `/health` | JSON — mode, frame count, client count, free RAM, uptime |
| `/mode?m=` | Switch the synthetic source: `bench`, `flight`, `still` |

`/health` is the one to hit from a laptop when something looks wrong:

```
curl http://192.168.4.1/health
```

## Synthetic sources

Until the radio exists, three profiles are selectable from the viewer's
dropdown or via `/mode`:

| Mode | What it does |
|---|---|
| `bench` | Gentle desk-scale motion. The default, and the realistic case for UI work |
| `flight` | A 40 s loop: 1.6 s burn at 5 g, 377 m apogee, 78 m/s peak ascent, 6 m/s chute descent, tipping after apogee |
| `still` | Flat output, for noise and drift checks |

The `flight` profile deliberately peaks at 6 g, which clips a ±2 g
accelerometer. That clipping is real and the display should show it rather
than hide it.

## Wire format

One line format is shared by the serial path and the Wi-Fi path, so the viewer
has a single parser regardless of how frames arrive (§6.6):

```
V,ax,ay,az,gx,gy,gz,alt,vel[,millis]
```

Accelerations in g, rates in dps, altitude in metres AGL, velocity in m/s. The
tenth field is the flight computer's own `millis()`.

**That timestamp matters more than it looks.** Without it the receiver derives
`dt` from packet *arrival* time, and USB and radio both deliver in bursts —
several frames land microseconds apart, then a gap. That jitter feeds straight
into attitude integration as noise. Timestamping at the source removes it, and
it is the only thing that makes the arrival-time jitter of a lossy RF link
harmless to the attitude estimate. **Keep this field when the radio is wired
in.**

## Wiring in the radio

The architecture already supports it and **the viewer needs no changes at
all.** `main.py` is built around one producer and many consumers: a single
task advances the telemetry state at a fixed rate, and every connected browser
reads the most recent frame. Replacing the synthetic producer with a packet
reader is the whole job.

See the block marked `RADIO HOOK` near the bottom of `main.py` for the sketch
of it — a UART reader that keeps the most recent `V,...` line, added to
`main()` as another asyncio task, with the SSE loop reading from it instead of
`tel.frame()`.

Ground-side conversion from the 18-byte packed LoRa packet (§6.2) to this line
format is a formatting step in the receiver.

## Two settings that are not optional

Both were found the hard way and both look like intermittent stutter:

- **Wi-Fi power management disabled** (`ap.config(pm=0xa11140)`). The CYW43
  radio parks itself between packets by default. Fine for request/response,
  visible as stutter on a continuous 25 Hz stream. Costs roughly 20–30 mA of
  idle current — irrelevant on a ground power bank.
- **Nagle's algorithm disabled** on the stream socket. Telemetry frames are
  ~60 bytes; Nagle withholds a small packet until the previous is ACKed, and
  phones delay ACKs by up to ~200 ms. The interaction produces exactly the
  intermittent multi-frame stall it was reported as.

## Notes and limits

- **AP-only**, comfortable with roughly 4 clients. It is a team tool, not a
  spectator server.
- **One producer, many consumers.** The first version advanced the simulation
  inside each client's stream loop, which made it run at 2× with two phones
  connected and made the two viewers disagree. Verified after the fix: three
  simultaneous clients each receive 25.0 Hz of byte-identical frames while the
  source advances at 1×. Keep that property.
- **The viewer is served from flash in chunks**, not read into RAM — the Pico W
  does not have the headroom to hold a ~45 KB file and a socket buffer
  comfortably at the same time.
- **No telemetry staleness timeout yet.** "Radio silent" and "vehicle sitting
  perfectly still" currently look identical on the display. This is a
  safety-relevant display defect, not a nicety — see §14.

## Troubleshooting

| Symptom | Cause |
|---|---|
| `index.html is not on the board` | The viewer was not copied. Copy it as `index.html` and reset. |
| AP never comes up (LED keeps blinking) | Not a Pico **W**, or non-W MicroPython flashed. |
| Stream stutters in multi-frame bursts | Power management or Nagle re-enabled — see above. |
| Two phones show different data | The producer is being advanced per client. See §6.5. |
