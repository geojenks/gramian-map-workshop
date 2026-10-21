# gramian-map-workshop

**Can N copies of your actuator hold, and turn, a ball in a cup?**

A small, numpy-only tool for the ARIA workshop. It pairs soft-actuator
**records** (from the Gramian capability map, `geojenks/Gramain_extraction`)
with manipulation **tasks** (from the wave-manipulation work,
`geojenks/Wave_manip`). It then says, for each arrangement of fingers, whether
the task can be done and, if not, **why**.

> Simulation and records only: comparative, not predictive. Record forces are
> uncalibrated, so strength is shown but never scored.

## Quick start (about two minutes)

```bash
git clone <this repo> && cd gramian-map-workshop
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -e .                                     # numpy is the only dependency
python demos/cup_hold.py                             # Demo 1
python demos/cup_turn.py                             # Demo 2
python demos/passive.py                              # Demo 3 (passive stiffness)
```

Then open `outputs/cup_hold.html` and `outputs/cup_turn.html` in a browser
(hover any cell). The same tables are in `outputs/*.txt`.

**Try your own actuator.** Copy `examples/template_actuator.json`, fill in your
numbers (canonical frame: X = base to tip, +Y = the main sideways output),
and run:

```bash
python demos/cup_hold.py --record my_actuator.json
python demos/cup_turn.py --record my_actuator.json
```

A datasheet-level (Tier 0) record needs only `scalars`: stroke, motion
direction, polarity and blocked force. Richer records (blocked force vectors
per input level) are read automatically.

## The two demos

**Demo 1, cup_hold.** N fingers (N = 1 to 8) on the ring where the Allegro's
pads held a ball in W20 step 4 (ball 32.9 mm, 0.05 kg, μ = 1, contacts
43.6° below the centre). The demand is the ball's weight. Mounts:
`aimed` (the finger's push points into the ball) and `palm` (the finger
stands upright on a palm), each at yaw 0° and 90°.

**Demo 2, cup_turn.** The same cup, turned about the vertical by a travelling
make/break wave (duty 0.82 from the banked ring; 0.5 as a sensitivity).
On every tick the fingers in contact alone must carry the weight. A finger
can drive the turn only if its tip moves along the azimuth.

**Demo 3, passive.** Demos 1 and 2 again, but each finger can also resist as a
spring (its record's tip stiffness) within a deflection budget of 2 mm, the
depth the Allegro's pads pressed into the ball (1 and 4 mm as sensitivity).
Only 9 of the 22 records carry stiffness data; the rest are unchanged.

## How to read a result: the reason ladder

Each arrangement is tested against nested sets, and the **first one it fails
names the reason**:

| | reason | meaning |
|---|---|---|
| – | no contact | no finger touches, or the record has no force data |
| d | direction | the fingers cannot push the object that way at all, at any force |
| f | friction | they could, but only by slipping (or by pulling) |
| e | cone edge | only at the very edge of the friction cone |
| k | deflection | held, but short on *comparative* passive spring force (Demo 3; not scored) |
| s | strength | held, but short on *comparative* force (not scored) |
| ✓ | met | can be done |

The **ideal contacts** row shows what any hand touching at the same points
could do. The gap between it and your actuator is what your actuator lacks.

Also reported: the **input count**, meaning how many independent motions the
fingers can give the object while their contacts stick (full control = 6).

## What the first run shows

From `outputs/*.txt`. These are draft results against a registration still
awaiting sign-off.

- **One finger never holds; two opposed fingers aimed at the ball always do**
  (ideal contacts and every record with force data). Comparative strength
  then sets N: 2 for the bellows, 3 to 4 for the pneu-nets, 7 for the 60°
  McKibben.
- **Palm-mounted pushers sit right at the friction-cone edge** at this cup
  height: the contact normal is 46.4° from vertical and the cone's half-angle
  is 45°.
- **Single-input fingers with sticking contacts can hold but not move the
  ball.** The input count is 0 with three or more fingers in contact, *unless*
  the arrangement's motion is itself a rigid motion. Palm-mounted benders at
  yaw 90° screw the ball (turn plus lift) with input count 1. That is the
  jar-lid helical path.
- **Why a wave.** With duty 0.82, four fingers hold on every tick while one at
  a time breaks; three fingers drop it whenever only two are in contact.
  With duty 0.5, nothing up to eight fingers holds at μ = 1 (half-ring
  contact sets need μ ≈ 1.5).
- **Turners and holders.** Only the pneu-nets and the cable finger can drive
  the turn (azimuthal share 0.96 to 0.99). The bellows and McKibbens can hold
  but not turn.
- **Counting passive stiffness removes the push-only model's brittleness.**
  Palm-mounted pneu-nets, which failed on a tiny out-of-plane part of their
  push, now hold from N = 2. Contracting McKibbens on a palm hold too: their
  stiff axial spring props the ball up while the active part pulls. The cable
  finger still fails, because its record has no out-of-plane stiffness. The
  turn results are unchanged: four fingers at duty 0.82, nothing at 0.5.

## Where every number comes from

- `data/records/` holds the gramian_map records, **copied byte for byte**
  (`tools/import_records.py`). `MANIFEST.json` pins each one by commit and
  sha256 and carries its trust flags (the 55° McKibben's floor noise is
  shown, not cleaned).
- `data/tasks/` holds the task files, written by `tools/export_tasks.py` from
  Wave_manip's banked W20 logs. Each constant names its file, key, commit
  and sha256.
- `gramian_workshop/vendor/` holds the W20 readout core (`stability.py`,
  `w20_capability.py`), **copied verbatim** by `tools/vendor_w20.py`. Do not
  edit it; regenerate it.
- `python -m gramian_workshop.gates --wave-manip ../wave_manip` runs gates
  V0 to V5. They check that the vendored code matches its source, the solver
  against brute force, the frames, closure, symmetry and provenance, and they
  write `outputs/gates.json`. `python -m unittest discover -s tests` runs them
  as tests; set `WAVE_MANIP=...` for the source checks.

The model and its predictions are registered in Wave_manip:
`experiment_log/2026-10-21_w20s6_soft_finger_registration.md` (W20 step 6).

## Limits, stated

- One input per finger. The finger pushes only along its blocked-force
  direction and moves only along its free direction. Demos 1 and 2 count no
  **passive stiffness**, which makes them brittle to small out-of-plane
  components in a record; Demo 3 counts it, as the minimum over the record's
  probed inputs, with negative or missing axes as zero.
- Placement is assumed (every tip touches its ring point); reach is not
  checked yet.
- Forces are comparative only, until a record is calibrated against a
  measured actuator.
- The turn is a kinematic duty schedule read for capability, not a
  controller; nothing is simulated.
