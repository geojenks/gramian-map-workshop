# Test curves: the file format

Most people who bring an actuator to the workshop bring test curves for it:
the input (usually pressure) against how far the tip moves or how far it
bends, and the input against the force it makes when held still. This page
sets out one plain file for those curves. A lab can fill it from a
spreadsheet and save it as CSV. `tools/curves_to_record.py` turns the file
into an actuator record, the only way into the tool.

```bash
python tools/curves_to_record.py my_curves.csv --out my_actuator.json
python tools/check_record.py my_actuator.json
```

The converter checks the record with the schema's own validator before it
writes anything, and it never overwrites an existing file.

A filled example is in `data/test_curves/mosadegh2014_fPN.csv`. It holds the
published curves of Mosadegh and colleagues' fast pneu-net, written out from
our record of it. The worked example below shows that converting it back
gives the same curves.

## The file at a glance

One CSV file per actuator, in two parts:

1. **Details**: one `key,value` row each, at the top.
2. **The table**: a header row whose first cell is `curve`, then one row
   per measurement.

Blank rows, and rows whose first cell starts with `#`, are skipped. Every
column name carries its unit, such as `input_kPa`, `tip_dx_mm` or `force_N`.
The converter turns every value into SI units.

The table of the filled example, as stored (its details block is above it
in the file):

```
curve,input_kPa,tip_dx_mm,tip_dy_mm,tip_dz_mm,force_N
free,15.0,-32.546,-50.899,0.0,
free,30.0,-73.704,-48.863,0.0,
free,45.0,-90.93,-27.28,0.0,
free,60.0,-91.159,-12.702,0.0,
free,72.0,-84.375,-2.603,0.0,
blocked,0.344,,,,0.0096
blocked,19.846,,,,0.2059
blocked,34.873,,,,0.4312
blocked,55.038,,,,0.89
blocked,72.662,,,,1.4005
```

Free and blocked tests rarely share input levels, so each row says which
test it belongs to, and leaves the other test's cells empty.

## The frame

Directions are in the actuator's own frame, as in `docs/RECORDS.md`:

- **X** runs from the fixed base to the tip;
- **Y** is the side it bends to;
- **Z** completes a right-handed frame.

Tip displacements are measured from the tip's rest position, at no input.
If your camera gives tip positions, subtract the position at no input
first. Signs are kept as given; the converter never rotates the data. (The
fast pneu-net in the example bends towards −Y, as its source plots it.)

## Details

| Key | Needed | What it holds | Record field |
|---|---|---|---|
| `id` | yes | file-safe name: letters, digits, `_`, `-` | `id` |
| `name` | yes | a readable name | `name` |
| `source` | yes | `experiment`, `literature` or `simulation` | `source.kind` |
| `detail` | yes | the rig, paper or model, in one line | `source.detail` |
| `motion` | yes | `bends`, `extends` or `contracts` | `scalars.polarity_axial` (empty, +1 or −1) |
| `force_direction` | with a `force_*` size column | `axial` or `sideways` (below) | which blocked-force field |
| `reference` | no | a DOI or URL | `source.reference` |
| `date` | no | when it was measured | `source.date` |
| `actuation` | no | `pneumatic` (default), `cable`, `hydraulic`, ... | `metadata.actuation_physics` |
| `input_name` | no | what drives it (default `chamber pressure`) | `input.channels[0].name` |
| `blocked_method` | no | how the force was measured (default: `detail`) | `blocked.method` |
| `uncertainty` | no | fraction, one sigma (default 0.1) | `uncertainty.default_fraction` |
| `notes` | no | anything else | `metadata.notes` |

The converter adds the file's name and sha256 to `metadata.notes`, so the
record names the file it came from.

## The table

| Column | Units | On rows | What it is |
|---|---|---|---|
| `curve` | | all | `free`, `blocked` or `backbone` |
| `input_<unit>` | `kPa`, `Pa`, `bar`, `psi`; `N`; `mm`, `m`; `V`, `A` | all | the input level |
| `tip_dx_<unit>`, `tip_dy_<unit>` | `mm`, `cm`, `m`, `um` | `free` | tip displacement from rest, along X and Y |
| `tip_dz_<unit>` | as above | `free` | out of the bending plane (optional; zero if absent) |
| `angle_<unit>` | `deg`, `rad` | `free` | bending angle of the tip from rest; positive turns X towards +Y |
| `point` | | `backbone` | point number along the actuator, base first |
| `x_<unit>`, `y_<unit>` (`z_<unit>` optional) | `mm`, `cm`, `m`, `um` | `backbone` | position of that point, base at the origin |
| `force_<unit>` | `N`, `mN`, `gf`, `kgf` | `blocked` | blocked force size, along `force_direction` |
| `force_x_<unit>`, `force_y_<unit>` (`force_z_<unit>` optional) | as above | `blocked` | blocked force as a vector |

The unit factors are the record form's own (`forms/record_form.html`).
Inputs in `V` or `A` keep their unit in the channel's name, since the record
schema stores only SI unit strings. Columns the format does not know (a
repeat number, a temperature) are skipped, and the converter lists them. A
column that names a known quantity with an unknown unit is an error.

## What is required, and what is optional

- **At least one test**: free rows, backbone rows or blocked rows.
- **Free rows** give the tip displacement (`tip_dx` and `tip_dy`), the
  bending angle, or both. Whichever you give must be on every free row.
  **Input against bending angle alone is accepted.**
- **Backbone rows** are the other way to give the bending shape: points
  along the actuator at each input level. Give the rest shape at input 0
  and the same number of points at every level. Give the tip path either
  as `tip_dx`/`tip_dy` or as backbone points, not both.
- **Blocked rows** are optional. Give either the vector (`force_x` and
  `force_y`, with `force_z` if measured) or a size (`force`) with the
  detail `force_direction`:
  - `axial`: signed along X; positive pushes out, negative pulls in;
  - `sideways`: a size across the finger, with no direction stated.
- **One row per input level** in each test. Average repeats first and give
  their spread as `uncertainty`.
- **No zero row is added.** If you measured the rest state, include it.
  (The record form and `record_from_experiment.py` in the Gramian map
  project add one; the published-data records do not, and nor do we.)

## How the file becomes a record

| Record field | Comes from |
|---|---|
| `free.u`, `free.tip_displacement_m` | free rows (`tip_dx`, `tip_dy`, `tip_dz`), sorted by input |
| `free.tip_displacement_m` from a backbone | the last backbone point at each level, minus its rest position |
| `free.bend_angle_rad` | free rows' `angle` (a field we add; see below) |
| `metadata.length_m` | the rest backbone's length along its points; empty otherwise |
| `blocked.u` | blocked rows, sorted by input |
| `blocked.force_N` | `force_x`, `force_y`, `force_z` |
| `blocked.force_axial_N`, `force_transverse_N` | from the vector (X part, and the size of the Y-Z part); or the `force` size, as axial or sideways |
| `input.channels[0].range` | from the lower of zero and the lowest input to the higher of zero and the highest |
| `scalars.free_stroke_m` | the largest tip displacement |
| `scalars.free_direction` | the first level that moves more than 2% of the stroke; with angle alone, ±Y by the sign of the first angle beyond 2% of the largest |
| `scalars.polarity_axial` | the `motion` detail; empty for a bender, as in the published-data records |
| `scalars.blocked_force_N` | the largest force size over the blocked levels |

The stroke and direction rules are the tool's own (`docs/RECORDS.md`). The
backbone itself is not kept in the record: the schema's only shape block is
the Gramian one, which needs a simulation. The file keeps it.

The record carries no stiffness, no Gramian shape block and no input work.
It is a curves-only record, like the published-data ones.

## What a record built from angle alone can and cannot do

The schema has no field for how far the tip turns; its own list of known
limits says so. We keep the angle curve in the free block as
`free.bend_angle_rad`, so nothing measured is lost. The schema's validator
accepts the extra field, and the record is a curves record (tier 1).

We do not turn an angle into a tip path. That needs the actuator's length
and an assumed bending shape, and the data say neither.

**It can:**

- pass `tools/check_record.py` and be loaded with `--record`;
- give the tool a motion direction: across the finger, on the side the
  angle says;
- carry the bending curve against input onto the map, for comparison with
  other actuators' curves.

**It cannot:**

- give a **stroke**: with no tip path there is no tip travel, so reach
  cannot be judged;
- be fitted with a **stand-in finger**: the fitter
  (`gramian_workshop/record_fit.py`) needs a tip path, and refuses the
  record, saying so. So it cannot yet become a record-built hand;
- without blocked rows, **push** at all. The tool lists it as having no
  force data, so it gives no hold verdict and no strength verdict.

Adding a backbone or the tip position at each level lifts the first two
limits. Adding blocked force lifts the third.

**A sideways force size is not read as a push.** The tool reads a push from
the blocked force vector, or from an axial force with a sideways part
beside it (`docs/RECORDS.md`). A sideways size alone is neither. The
published fast pneu-net record has exactly this, and `check_record.py`
reports that it "cannot push in the tool". If you measured the force
components, give them as `force_x` and `force_y`.

## Worked example: the fast pneu-net, there and back

`data/test_curves/mosadegh2014_fPN.csv` was written from
`data/records/mosadegh2014_fPN.json` by

```bash
python tools/curves_to_record.py --from-record data/records/mosadegh2014_fPN.json \
    --out data/test_curves/mosadegh2014_fPN.csv
```

and converted back by

```bash
python tools/curves_to_record.py data/test_curves/mosadegh2014_fPN.csv \
    --out outputs/test_curves/mosadegh2014_fPN_from_curves.json \
    --compare data/records/mosadegh2014_fPN.json \
    --compare-out outputs/test_curves/mosadegh2014_fPN_round_trip
```

The comparison is stored in
`outputs/test_curves/mosadegh2014_fPN_round_trip.txt`:

- **Every curve is identical**, value for value: the input range, the free
  inputs and tip displacements, the blocked inputs and forces.
- **The blocked force and the polarity are identical**: 1.4005 N, and empty.
- **The stroke and the motion direction differ.** The stored record's
  stroke is the tip displacement at the highest input, 0.08441514220801859
  m, and its direction is that from the first level to the second,
  [−0.9987787042222235, 0.04940748923165469, 0]. The tool's own rules give
  0.09493399443824116 m (the largest displacement) and
  [−0.5387088914152084, −0.8424919764070145, 0] (the first level's
  direction). The record was built in the Gramian map project by
  `build_record_from_digitized.py`, which uses other rules than
  `docs/RECORDS.md` states.

`tests/test_curves_to_record.py` checks all of this: the example file is the
record written out; converting it back reproduces the record's curves;
the headline numbers follow the tool's rules; the result passes
`check_record.py` and matches the stored output. It also checks an
angle-only file, a backbone file, a force vector, the refusal of malformed
files, and that an existing output is never overwritten.

## Limits

- One input per actuator.
- No stiffness curves yet; the record form takes them.
- Bending in the X-Y plane is assumed for the angle; `tip_dz` and `z` carry
  any out-of-plane motion.
