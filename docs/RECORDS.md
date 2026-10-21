# Actuator records: what data, in what form

A **record** describes one actuator (one finger's worth) as a JSON file. The
format is the Gramian capability map's own (`docs/RECORD_SCHEMA.md`, copied
unchanged from `geojenks/Gramain_extraction`), so records made here also go
straight onto the map.

**Easiest way to make one:** open `forms/record_form.html` in a browser, type
or paste your numbers in whatever units you have, and download the file.
Check it with:

```bash
python tools/check_record.py my_actuator.json
```

which validates it and prints exactly what the tool reads from it.

## The frame

Every direction is in the actuator's own frame:

- **X** runs from the fixed base to the tip;
- **+Y** is its main sideways output (the way a bender bends);
- **Z** completes a right-handed frame.

An extender moves along +X, a contractor along −X, a bender mostly along +Y.

## What the tool uses, and the minimum to give

| The tool needs | Minimum (a datasheet is enough) | Better (measured) | Record field |
|---|---|---|---|
| **Which way it pushes, and how hard**, with the tip held still | blocked force at full input, and its direction (+X, −X, +Y) | blocked force per input level, as a vector (Fx, Fy, Fz) | `blocked.force_N` or `scalars.blocked_force_N` |
| **Which way the tip moves, and how far**, with no load | free stroke at full input, and its direction | tip displacement (dx, dy, dz) per input level | `free.tip_displacement_m` or `scalars.free_stroke_m` + `free_direction` |
| **How springy the tip is** (optional) | tip stiffness along X, Y, Z | stiffness per input level | `stiffness.k_*_N_per_m` |
| **Its input** | what drives it, and the range | | `input.channels` |
| **Where the numbers come from** | rig, paper or model | a DOI or URL | `source` |

Units in the file are SI (m, N, Pa, N/m). The form converts mm, kPa, bar,
psi, gf, kgf and N/mm for you.

## How the tool reads a record

- **Push direction and force:** the blocked force vector at the highest input
  level; if only axial and sideways sizes are given, the sideways part is
  taken along +Y; if only a headline number, along ±X by its polarity.
- **Motion direction:** the direction of the first input level that moves more
  than 2% of the full stroke. Small-signal, because the end-of-stroke direction
  mixes up shape and amount.
- **Stroke:** the largest tip displacement.
- **Stiffness:** per axis, the smallest value over the input levels. Negative or
  missing values count as zero, so the tool never counts spring force the
  actuator might not have.

These are the same rules as `record_from_experiment.py` in Gramain_extraction.

## Honesty about magnitudes

Simulated records are **comparative**: their forces come from uncalibrated
material models, often at low pressures. The tool shows force and stiffness
limits but does not score them for simulated records. Measured and datasheet
records are in real units, and as more of them come in, more of the tool's
answers can be stated in real terms.

## Tiers

- **Tier 0:** headline numbers only. Enough for the tool.
- **Tier 1:** curves against input (free motion, blocked force, stiffness).
- **Tier 2:** spatial modes and Gramians, from simulation or instrumented rigs.
  This is what the SOFA pipeline produces from a CAD.
