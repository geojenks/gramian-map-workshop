# Joint tables: the file format

Some visitors bring a jointed finger rather than a soft actuator: a rigid
finger, a tendon-driven one, or a design they know only on paper. For
these the canonical form is a per-joint table (ruling of 28 Oct on GitHub
issue 16). For each joint it gives the axis, where the joint sits, its
range, its torque limit and its spring stiffness. Teams who start from a
written Jacobian supply the joint table it came from, since a Jacobian at
one pose changes as the finger moves.

This page sets out one plain file for that table. `tools/joint_table_to_record.py`
turns the file into an actuator record, the way into the map.

```bash
python tools/joint_table_to_record.py my_finger.csv --out my_finger.json
python tools/check_record.py my_finger.json
```

The converter checks the record with the schema's own validator before it
writes anything, and it never overwrites an existing file. When a field is
missing or a unit is unclear it refuses and says which line and column.

A filled example is in `data/joint_tables/shadow_hand_finger.csv`: the
Shadow Hand finger, from the same published dimensions as the stored record
`data/records/shadow_hand_finger.json`. The worked example below compares
the two.

## The file at a glance

One CSV file per finger, in two parts, as for test curves
(`docs/TEST_CURVE_FORMAT.md`):

1. **Details**: one `key,value` row each, at the top.
2. **The table**: a header row whose first cell is `joint`, then one row per
   joint, base first, and a last row named `tip`.

Blank rows, and rows whose first cell starts with `#`, are skipped. Every
column with a unit carries it in its name, such as `pos_x_mm`, `max_deg` or
`torque_Nm`. The converter turns every value into SI units.

The table of the filled example, as stored:

```
joint,axis_x,axis_y,axis_z,pos_x_mm,pos_y_mm,pos_z_mm,min_deg,max_deg,torque_Nm,stiffness_Nm_per_rad
J3,0,0,1,0,0,0,0,80,1,0
J2,0,0,1,45,0,0,0,100,1,0
J1,0,0,1,70,0,0,0,70,1,0
tip,,,,96,0,0,,,,
```

## The frame and the rest pose

The table describes the finger **at rest**, with every joint at angle 0.
Positions and axes are all given in one frame, the finger's own, as in
`docs/RECORDS.md`:

- **X** runs from the base to the tip;
- **Y** is the side it bends to;
- **Z** completes a right-handed frame.

The joints form one chain in the order of the rows: each joint turns every
joint after it, and the tip. A joint turns by the right-hand rule about its
axis, so a positive angle on a +Z axis turns X towards +Y. Point each axis
so that the joint flexes towards the upper end of its range.

## Details

| Key | Needed | What it holds | Record field |
|---|---|---|---|
| `id` | yes | file-safe name: letters, digits, `_`, `-` | `id` |
| `name` | yes | a readable name | `name` |
| `source` | yes | `experiment`, `literature` or `simulation` | `source.kind` |
| `detail` | yes | the drawing, paper or model, in one line | `source.detail` |
| `reference` | no | a DOI or URL | `source.reference` |
| `date` | no | when the table was made | `source.date` |
| `actuation` | no | `electric` (default), `cable`, `pneumatic`, ... | `metadata.actuation_physics` |
| `levels` | no | how many input levels to compute, 3 or more (default 21) | length of the curves |
| `uncertainty` | no | fraction, one sigma (default 0.1) | `uncertainty.default_fraction` |
| `notes` | no | anything else | `metadata.notes` |

The converter adds the file's name and sha256 to `metadata.notes`, and the
whole table in SI units, so the record names the file it came from and
keeps every number in it.

## The table

| Column | Units | What it is |
|---|---|---|
| `joint` | | the joint's name; `tip` on the last row |
| `axis_x`, `axis_y`, `axis_z` | none | the joint axis; any length, the converter makes it a unit vector |
| `pos_x_<unit>`, `pos_y_<unit>`, `pos_z_<unit>` | `mm`, `cm`, `m`, `um` | a point on the joint axis, at rest; on the `tip` row, the tip point |
| `min_<unit>`, `max_<unit>` | `deg`, `rad` | the ends of the joint's range; 0, the rest pose, must lie between them |
| `torque_<unit>` | `Nm`, `Nmm`, `mNm` | the largest torque the drive gives the joint, as a size |
| `stiffness_<unit>` | `Nm_per_rad`, `Nmm_per_rad`, `Nm_per_deg`, `Nmm_per_deg` | the joint's spring; 0 for none |

Every joint row needs every column. A torque limit or a stiffness that is
not known is written `unknown`; an empty cell is refused, so that a gap is
never read as zero. The `tip` row needs only its three positions.

Columns the format does not know (a joint's mass, a comment) are skipped,
and the converter lists them. A column that names a known quantity with no
unit or an unknown unit is an error: `torque` alone, or `torque_kgcm`, is
refused with the units it would accept.

Only hinges are taken. A sliding joint has no row type yet.

## How the file becomes a record

The Jacobian at any pose comes from the table: for a hinge, its column is
the joint axis crossed with the vector from the joint to the tip (the
ruling). The record has one input, the drive level u from 0 to 1.

| Record field | Comes from |
|---|---|
| `free.u`, `free.tip_displacement_m` | at each of `levels` inputs from 0 to 1, every joint at u times the upper end of its range; the tip's position minus its rest position |
| `blocked.u`, `blocked.force_N` | the tip pinned at rest, pushing along the direction it first moves in (the Jacobian at rest times the upper range ends). A push of size s loads each joint with s times its lever along that direction. At input u the push is u times the largest s that keeps every joint within its torque limit |
| `blocked.force_axial_N`, `force_transverse_N` | the X part of that push, and the size of its Y-Z part |
| `stiffness.k_N_per_m` | at each free level, the tip stiffness along Y: 1 / C_yy, where the tip compliance C is J K⁻¹ Jᵀ with K the joint stiffnesses. A joint with no spring that moves the tip along Y makes it 0 |
| `metadata.length_m` | the distance from the first joint to the tip at rest |
| `input.channels[0]` | the drive level, no unit, range 0 to 1 |
| `scalars.free_stroke_m` | the largest tip displacement over the levels (the tool's rule, `docs/RECORDS.md`) |
| `scalars.free_direction` | the first level that moves more than 2% of the stroke (the tool's rule) |
| `scalars.polarity_axial` | the sign of the X part of the motion direction when it is beyond 0.1, else 0: the rule of the script that made the stored Shadow record (`gramian_map/generate_literature_records.py`) |
| `scalars.blocked_force_N` | the push at full input |
| `scalars.stiffness_N_per_m` | the smallest Y stiffness over the levels |
| `scalars.effective_dof` | the number of joints, as the stored Shadow record counts it |

Without every torque limit there is no blocked block, and without every
stiffness there is no stiffness block; the converter says which.

One drive level for the free path and for the push is a stated assumption:
at level u the unloaded finger reaches the fraction u of each joint's
range, and the pinned finger pushes with the fraction u of its torque
limits.

## What a record from a table can and cannot do

**It can:**

- pass `tools/check_record.py` and be loaded with `--record`;
- give the tool a motion direction, a stroke and a push in newtons, worked
  out from the joints rather than fitted;
- carry the finger's path against input onto the map, beside the soft
  actuators' curves.

**It cannot:**

- give an **axial stiffness**. A straight jointed finger is held along X by
  its links, not its springs, so the stiffness there is unbounded at rest
  and the schema has no way to say so. The record leaves `k_axial_N_per_m`
  out, and the tool then reads zero along X, as it does for any missing
  axis;
- say that two joints share a **tendon**. The Shadow finger's J2 and J1 are
  driven together; the table reads each joint on its own, so its push and
  path are those of three independent joints;
- stand in for the table in the **force check**. The ruling sends jointed
  fingers into the force check as jointed fingers, built from the table
  itself. The record is the finger's entry on the map; building a hand
  model from a table is not done yet.

## Worked example: the Shadow finger, table against stored record

`data/joint_tables/shadow_hand_finger.csv` holds the Shadow index finger:

- three hinges about +Z at 0, 45 and 70 mm, with the tip at 96 mm, and
  ranges of 0 to 80, 100 and 70 degrees. These are the stored record's own
  numbers (its detail and notes, from
  `gramian_map/generate_literature_records.py` in Gramain_extraction);
- torque limits of 1 N.m on every joint, and stiffness 0. These are the
  MuJoCo Menagerie model's (`shadow_hand/right_hand.xml` at the commit
  `tools/fetch_hands.py` pins): its default actuator force range is −1 to
  1, which the J3 actuator and the J2-J1 tendon actuator take, and no joint
  sets a spring. The stored record has neither number.

It was converted and compared by

```bash
python tools/joint_table_to_record.py data/joint_tables/shadow_hand_finger.csv \
    --out outputs/joint_table/shadow_hand_finger_from_joint_table.json \
    --compare data/records/shadow_hand_finger.json \
    --compare-out outputs/joint_table/shadow_hand_finger_vs_stored.txt
```

The comparison is stored in
`outputs/joint_table/shadow_hand_finger_vs_stored.txt`. Numbers agree when
they are within 1e-6 of each other, the stored record's rounding (six
decimal places).

**The same:** the length (0.096 m), the motion direction
([−0.081271, 0.996692, 0]), the polarity (0), the number of joints (3) and
the actuation (`cable`). The direction agrees because the table's first
level, u = 0.05, is the very point the stored record's generator used.

**Different, and why:**

- **Stroke.** The stored record gives the tip displacement at full flexion,
  0.123687 m. The table's record gives the largest displacement over its
  levels, 0.12947240813847366 m at u = 0.8, by the tool's rule. Fully
  flexed, the tip has curled back past its farthest point. The table's
  record still holds the full-flexion displacement as its last free level,
  and it agrees with the stored stroke to within 1e-6 m
  (`tests/test_joint_table.py` checks this).
- **The input.** The stored record's input is "joint flexion (uniform
  fraction)"; the table's is the drive level, which also scales the torque
  limits. Both run from 0 to 1, and on the free path they mean the same.
- **Curves, push and stiffness.** The stored record is tier 0: headline
  numbers only, with no free, blocked or stiffness block. The table's
  record is tier 1, with all three. Its push is 10.416666666666666 N along
  +Y at full input, limited by J3 (1 N.m over the 96 mm lever). Its Y
  stiffness is 0 at every level, since the model's joints have no springs.

`tests/test_joint_table.py` checks all of this: the example converts, passes
`check_record.py` and matches the stored output; the headline numbers
agree with `shadow_hand_finger.json` where stated above; the example's
numbers are those of the stored record and the Menagerie model; the
Jacobian matches a finite difference of the tip path; and the converter
refuses a missing column, an empty cell, an unclear unit, a range that
leaves out the rest pose, and an existing output.

## Limits

- Hinges only, in one chain; no branching, no sliding joints.
- No coupling between joints (tendons, linkages).
- One drive level for every joint; a finger driven joint by joint has
  several inputs, which the record schema holds as one channel here.
- The push is worked out at the rest pose only.
