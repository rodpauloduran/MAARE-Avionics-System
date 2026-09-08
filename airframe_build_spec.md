# Water Rocket Airframe — Build Specification

Companion to the avionics BOM. This covers the airframe: structure, compartments,
materials, and assembly — the parts that carry the electronics rather than the
electronics themselves.

Base platform: 2 L PET carbonated-drink bottle, ~10.5 cm body diameter.
Target dry mass: 150–200 g (see apogee-vs-mass modeling — this is the sweet spot).

---

## 1. Overall layout, tip to tail

```
                    ___
                   /   \      <- nose cone tip (antenna void)
                  /     \
                 |  AVI  |    <- avionics bay (nose cone interior)
                 |  ONICS|
                 |_______|
                 |=======|    <- dry/wet bulkhead (critical seal)
                 |       |
                 | WATER |    <- pressure chamber = bottle body
                 | / AIR |       (667 mL water fill, 1/3 of 2 L)
                 |       |
                 |_______|
                  \     /
                   \___/      <- bottle neck / nozzle
                    | |
                  FINS x3-4   <- mounted on the tapered shoulder
```

Total height: **~55–65 cm** (2 L bottle body ~31 cm + nose cone ~15–18 cm +
neck/nozzle ~5 cm). Body diameter fixed by the bottle at ~10.5 cm; fin span
typically extends 3–5 cm beyond that per fin.

---

## 2. Section-by-section

### 2.1 Nose cone — avionics bay

**Material:** 3D-printed PETG or PLA+ (PETG preferred — more impact-tolerant
and slightly water-resistant), OR a commercial foam/plastic nose cone sized to
the bottle's neck diameter with an internal deck bonded in.

**Geometry:** ogive or conical, 15–18 cm long (matching the layout diagram
above). Length matters for two reasons:
enough internal volume for the two-deck avionics sled plus packed parachute,
and a fineness ratio (length:diameter) of roughly 3:1–5:1 for reasonable drag.

**Internal structure:**
- Two-deck avionics sled (the 5×7 cm FR4 protoboards from the BOM), joined by
  nylon standoffs, sliding into the cone as one removable unit.
- Static port ring: 3–4 holes of ~1 mm, evenly spaced, positioned at least one
  body-diameter aft of the nose shoulder, venting to the dry-side ambient air.
- Antenna void at the very tip — the otherwise-wasted conical volume, used for
  the 82 mm LoRa wire routed up from the sled.
- Reed switch mounted vertically near the outer wall, with its position marked
  on the outside skin (tape) for magnet placement.

**Attachment to body:** friction-fit over the bottle neck OR over a printed
coupler ring epoxied to the neck, retained by the latch pin mechanism. This
joint is also the parachute ejection separation point.

### 2.2 Dry/wet bulkhead

**Material:** 3 mm plywood disc, G10/FR4 offcut, or a printed PETG plug —
anything that seals and takes a threaded or epoxied fit at the bottle neck.

**Function (does three jobs):**
1. Seals the avionics bay from the pressurized water/air chamber.
2. Anchors the shock cord (a screw eye or drilled hole through the disc).
3. Defines the "ambient" reference side for the baro's static ports — the
   ports must see dry-side air, not the pressurized chamber.

**Mounting:** epoxied into the bottle neck, or threaded onto a printed collar
that itself epoxies to the neck. Must hold the full chamber pressure (100 psi)
without leaking or popping — test this piece under pressure on its own before
trusting it on a flight.

### 2.3 Pressure chamber (bottle body)

**Material:** the 2 L PET bottle itself. Do not modify, drill, or thin the
body wall — PET's pressure rating depends on its molded thickness and any
puncture is a structural risk at 100 psi.

**Contents:** 667 mL water (1/3 fill — the commonly-used near-optimal ratio),
pressurized air above it via the launcher's fill valve/gauge.

**Inspection:** discard any bottle with scratches, cloudiness, or a dented
body — these are stress risers that can trigger a burst under pressure.
Reuse count: retire after ~10–15 pressurizations as a conservative limit,
since repeated cycling fatigues PET.

### 2.4 Neck / nozzle

**Material:** standard bottle neck threads, mated to your launcher's seal
(usually an O-ring or rubber stopper on a tube, depending on launcher design).

**Note:** this is normally launcher hardware, not something built into the
rocket — the "nozzle" on a water rocket is really just the bottle's own
neck opening, sized to whatever launch tube your pad uses.

**On the two numbers.** "28 mm" is the *thread finish* designation (the
standard PCO-1881 / 28 mm carbonated-drink neck); the actual bore through it is
roughly 21 mm. The thrust calculation in the avionics document
(478 N at 100 psi) uses the 21 mm bore, because that is the area the water
actually accelerates through. Both figures are correct — they measure
different things. Confirm the bore on your own bottles with calipers before
relying on the thrust number.

### 2.5 Fins

**Material:** 2–3 mm G10 fiberglass sheet (best stiffness-to-weight and water
tolerant) or corrugated plastic/coroplast (cheap, adequate, slightly heavier
for the same stiffness).

**Count and shape:** 3 or 4 fins, trapezoidal, root chord ~8–10 cm, tip chord
~4–5 cm, span ~5–6 cm, swept.

**Mounting:** epoxied directly to the tapered shoulder near the bottle neck,
reinforced with fillets (epoxy + microballoons, or hot glue as a cheaper
substitute) at the root joint — this joint takes real aerodynamic load and is
a common failure point if under-filleted.

**Sizing check:** run the assembled design through OpenRocket before cutting
fins. Static margin (distance between CP and CG, in body diameters) should
land in the 1–2 caliber range. Recall CG shifts forward as water expels, so
verify margin at both "full" (wet, pre-launch) and "empty" (dry, post-burn)
conditions — both need to be stable.

---

## 3. Recovery hardware

| Component | Spec | Notes |
|---|---|---|
| Shock cord | Elastic bungee, 3–4x body tube length | Absorbs separation shock; a rigid tether will crack the nose cone or snap |
| Cord anchor (body side) | Through the dry/wet bulkhead | Screw eye or drilled + knotted hole |
| Cord anchor (nose side) | Through a printed lug in the nose cone | Same method |
| Swivel | Small fishing swivel, inline on the cord | Prevents shroud lines twisting into a knot during tumble |
| Parachute | 30–45 cm diameter, ripstop nylon or plastic sheet | Sized for a soft landing given your dry mass — larger chute for lighter mass |
| Chute protector | Nomex cloth or a folded paper wrap | Optional at these low ejection temperatures (no pyro), but cheap insurance |

---

## 4. Material summary / shopping list

| Part | Material | Approx. qty |
|---|---|---|
| Nose cone | PETG filament (printed) or commercial foam cone | 1 |
| Avionics sled | FR4 protoboard (already in BOM) | 2 |
| Bulkhead | 3 mm plywood or G10 offcut | 1 disc, ~28 mm dia. |
| Coupler/collar (if printed) | PETG filament | 1 |
| Fins | 2–3 mm G10 sheet or coroplast | 3–4 pieces |
| Fillet epoxy | 2-part epoxy + microballoons, or hot glue | small |
| Shock cord | Elastic bungee cord | ~1.5–2 m |
| Swivel | Small fishing swivel | 1 |
| Parachute | Ripstop nylon or plastic sheet | 1, ~30–45 cm |
| Static port drill | 1 mm drill bit | — |
| Body | 2 L PET bottle, uncut | 1 (+ spares — they don't last forever) |

---

## 5. Assembly sequence

1. Print/cut nose cone; dry-fit avionics sled inside, confirm clearance for
   packed parachute and shock cord routing.
2. Fabricate and pressure-test the dry/wet bulkhead in isolation before
   committing it to the airframe.
3. Epoxy bulkhead into bottle neck; leak-test at full 100 psi with water,
   away from anyone's face, before proceeding.
4. Drill static ports in the nose cone wall, aft of the shoulder, dry side
   only.
5. Cut and fillet fins onto the tapered shoulder; cure fully before handling.
6. Run the assembled mass/CG estimate through OpenRocket; confirm static
   margin wet and dry.
7. Install shock cord, swivel, and parachute; dry-pack and test the latch
   pull 50x per the earlier servo-latch testing note.
8. First flights: airframe and recovery only, no avionics, to prove the
   structure survives pressurization and landing before risking the
   electronics.
9. Add the avionics sled once structural flights are clean.

---

## 6. Open items to decide

- [ ] Printed vs. commercial nose cone — printed gives you exact avionics-bay
      geometry; commercial is faster to source but may need internal rework.
- [ ] G10 vs. coroplast fins — G10 costs more, coroplast is heavier for the
      same stiffness. Either works at this scale.
- [ ] Parachute diameter — tune once you know your actual built dry mass;
      recompute descent rate and resize if landings are too hard or too drifty.
- [ ] Number of bottles to keep in reserve — PET fatigues under repeated
      pressurization; budget for replacements across a test campaign.
