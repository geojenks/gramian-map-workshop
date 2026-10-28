# The task-file format (the leader file)

Written 28 Oct 2026, for anyone who wants to bring a manipulation from their
own simulation or demonstration and ask which hands can do it. The files
live in `data/leaders/*.json`. `tools/check_leader.py` checks one against
this page. Everything below was read from the code that writes and reads
these files, at commit `0b628b8`; the allowed-overlap fields were added
on 28 Oct (see "Allowed overlap" below).

## What a leader is

- **A task is defined by what the object does**: its position and
  orientation over time. In a leader that is `t`, `obj_pos` and
  `obj_quat` in each frame.
- **A leader is one method for doing that task**: one contact plan. Each
  frame says which contact stations touch the object, where, and the
  force each one puts on it. A cup gait, a pinch roll, a regrasp and a
  wrist roll are different methods, so they are different leaders, even
  for the same object motion.
- **The task also states how much overlap it allows** between the hand
  and the object: how deep a finger may press in at a stated contact, and
  whether, and how deep, the rest of the hand may touch it elsewhere
  (`allowed_overlap_m`, `other_contact`). Both are optional, with
  defaults.
- **The follow check** (`gramian_workshop/follow.py`) asks, frame by
  frame, whether a hand can be at those contacts and supply those forces,
  within that overlap. It adds no penalty for overlap the task allows.
- **A fail means "this hand cannot do this task by this method"**, never
  "this hand cannot do this task". The same hand may pass with another
  leader for the same object motion.
- The check is quasi-static. Each frame is a separate force balance. The
  object's weight and its acceleration reach the check only through the
  forces in the file; the check never adds gravity itself.

A station is one contact site of the method: a fingertip in a cup gait, a
digit in a human demonstration. Station `i` is the same contact site in
every frame. The follow check gives each station at most one finger for
the whole run.

## How to check a file

```bash
python tools/check_leader.py my_task.json            # one report per file
python tools/check_leader.py --table data/leaders/*.json   # Markdown tables
```

- **ERROR**: the follow check would stop on the file, or would read part
  of it as something other than this page says. Fix these.
- **warn**: the follow check runs, but reads part of the file in a way you
  may not expect. Each message says what it does instead.
- After that it prints what the follow check reads: object, friction,
  allowed overlap, stations, frames, time step, contacts per frame, and
  the net contact force against the object's weight.

The tolerances behind the warnings are constants at the top of the
checker (`SURFACE_TOL_M`, `UNIT_TOL`, `NORMAL_TOL_DEG`,
`OVERLAP_LARGE_M`). They are the checker's own, marked "proposed"; the
follow check does not use them.
`tests/test_check_leader.py` runs the checker over every stored leader.

## Conventions

- **Units are SI**: metres, kilograms, newtons, seconds. Field names carry
  the unit where it is not plain from the name (`radius_m`,
  `force_on_object_N`).
- **One frame of reference** for every position and force in the file.
  It must be right-handed. The follow check places the whole leader
  relative to the hand: a rotation and a translation about the first
  frame's object centre, found by a search. So the origin and orientation
  of your frame are free, with two exceptions:
  - the follow check's `respace` option turns each station about the
    file's **z axis** through the object's centre. Use z for the axis the
    stations go round (up, in the cup leaders);
  - a fixed placement passed to `follow(placement=...)` uses the frame as
    it stands. The hand-frame demonstration is used that way (see below).
- **Gravity is not read.** If the object has weight, the forces must
  carry it.
- **Forces are on the object, by the station**, not the reaction on the
  finger.
- **Normals point into the object**, from the contact towards the centre.
- **Quaternions are (w, x, y, z).**
- `null` means "not known" or "not touching", as stated per field.

## The file at a glance

```json
{
  "schema": "gramian-workshop/leader@1",
  "id": "my_task_my_method",
  "name": "What the object does, and how",
  "kind": "where it came from: simulation, demonstration, ...",
  "source": { "...": "provenance: repository, commit, script, settings" },
  "object": { "shape": "sphere", "radius_m": <number>, "mass_kg": <number> },
  "n_stations": <whole number>,
  "allowed_overlap_m": 0.003,
  "other_contact": { "allowed": true, "max_overlap_m": 0.003 },
  "frames": [
    {
      "t": <seconds>,
      "obj_pos": [x, y, z],
      "obj_quat": [w, x, y, z],
      "contact": [true, false, ...],
      "contact_point": [[x, y, z], null, ...],
      "force_on_object_N": [[fx, fy, fz], [0, 0, 0], ...],
      "station_pos": [[x, y, z], [x, y, z], ...],
      "contact_normal": [[nx, ny, nz], null, ...]
    }
  ],
  "caveat": "what the numbers can and cannot be used for"
}
```

Every per-station list holds exactly `n_stations` entries, in station
order.

## Top-level fields

| Field | Required | Type and units | Read by the follow check | Meaning |
|---|---|---|---|---|
| `schema` | yes | the string `gramian-workshop/leader@1` | no | Names this format and its version. |
| `id` | yes | non-empty string | yes: names the result | Unique name of this leader. By convention the file is `<id>.json`. |
| `n_stations` | yes | whole number, 1 or more | yes | How many stations; every per-station list has this many entries. |
| `object` | yes | object, below | yes | The manipulated object. |
| `frames` | yes | non-empty list, below | yes | The method, frame by frame, in time order. |
| `mu` | no | number above 0 | yes | Friction coefficient for the follow check. If absent it uses its own default (`leader_json.get("mu", 1.0)` in `follow.py`). No stored leader sets it. |
| `allowed_overlap_m` | no | number, 0 or more, metres | yes | How deep the finger touching a stated contact may sink into the object. Default 0.003 (3 mm) when absent. See "Allowed overlap". |
| `other_contact` | no | object, below | yes | Whether the rest of the hand may touch the object away from the stated contacts, and how deep. Default: allowed, as deep as `allowed_overlap_m`. See "Allowed overlap". |
| `name` | no | string | no | A readable title. |
| `kind` | no | string | no | Where the method came from. |
| `source` | no | object | no | Provenance: repository, commit, script, every setting that was overridden. |
| `caveat` | no | string | no | What the numbers can be used for. |
| `frame` | no | object | no (the checker reads `units`) | Notes on the frame of reference. `units`, if given, must be `"m"`. `gravity` may be a direction or a sentence. |
| `stations` | no | list of `n_stations` strings | no | A name per station (`thumb`, `index`, ...). |
| `derivation` | no | object | no | How derived forces were made (see below). |
| `notes` | no | list of strings | no | Free notes. |
| `supersedes` | no | string | no | In a corrected copy, the `id` of the stored leader it replaces. See "Corrected copies of stored leaders". |

Any other top-level key is allowed and is not read. The checker lists
such keys under "not read".

### `object`

| Field | Required | Type and units | Read | Meaning |
|---|---|---|---|---|
| `shape` | yes | the string `sphere` | the check assumes it | Only spheres are handled. Contact normals and finger placement use the sphere. |
| `radius_m` | yes | number above 0, metres | yes | Sphere radius. |
| `mass_kg` | yes | number above 0, kilograms | read, not used | The follow check parses it but the forces already carry the load. Writers that derive forces from motion need it. |
| `mass_note` | no | string | no | Say here if the mass is assumed rather than measured. |

## Allowed overlap

A task states how much overlap between hand and object it allows. A small
overlap is expected: a soft pad presses in. The follow check scores a
hand against exactly what the file states, and adds no penalty of its
own for overlap within it. Both fields are optional; a file without them
gets the defaults, so every leader written before these fields stays
valid and means what it meant, with the default allowance.

**`allowed_overlap_m`**: a number, 0 or more, in metres. How deep the
finger touching a stated contact may sink into the object. "The finger"
here is the link the follow check touches with: the last or middle
phalanx. Default `0.003` (3 mm), the depth the follow check allowed
before the allowance was stated (ruled 28 Oct, issue 22).

**`other_contact`**: whether the rest of the hand may touch the object
away from the stated contacts, and how deep. "The rest of the hand" is
the other links of each touching finger, and the palm.

| Field | Required | Type and units | Meaning |
|---|---|---|---|
| `allowed` | yes | `true` or `false` | Whether other contact is allowed at all. |
| `max_overlap_m` | no; only with `allowed: true` | number, 0 or more, metres | How deep other parts may sink in. Default: the same as `allowed_overlap_m`. |

- Absent: the same as `{"allowed": true}`, i.e. other parts may sink in
  as deep as the touching finger.
- `{"allowed": true, "max_overlap_m": 0}`: other parts may graze the
  surface but not sink in.
- `{"allowed": false}`: other parts must stay clear of the object by the
  distance at which the follow check counts a finger as touching
  (`TOUCH_TOL_M`, 2 mm).

How the follow check uses them (the module docstring of `follow.py`,
"Allowed overlap", has the detail):

- **Verdict.** A finger is green only if its touching link is within
  `allowed_overlap_m` and its other links within the other-contact
  allowance. It is amber ("close") if no part is deeper than its
  allowance plus `allowed_overlap_m` again, and red ("penetrates")
  beyond. The palm is graded the same way against the other-contact
  allowance, per frame ("palm penetrates").
- **Pose search.** The finger's pose, the hand's placement and the wrist
  are searched with the same allowances: depth within them costs
  nothing, depth beyond them counts against the pose.
- **Contact patch.** A soft pad pressed `allowed_overlap_m` into the
  object touches over a patch, not a point, and the leader's contact
  anywhere in that patch counts as reached. A larger allowance widens it.
- **Reported.** Each finger in each frame carries `overlap_mm` (its
  touching link; negative means clear of the surface by that much) and
  `other_overlap_mm` (its other links), each frame `palm_overlap_mm`, and
  the result's `settings` the allowances used and where they came from
  (the file, the default, or an override).

`tools/run_follow.py --allowed-overlap-m`, `--other-overlap-m` and
`--no-other-contact` override the file, as do the `allowed_overlap_m`
and `other_contact` arguments of `follow()`. Before 28 Oct the follow
check applied one allowance of its own, 3 mm (amber to 6 mm), to the
whole finger and the palm, and no file could change it. No stored leader
sets these fields, so the stored leaders now get the 3 mm default.

## Per-frame fields

| Field | Required | Type and units | Read by the follow check | Meaning |
|---|---|---|---|---|
| `t` | yes | seconds, strictly increasing | only with `inertia`, `rate_limit`, `lag_s` or `trajectory` | Time of the frame. Frames need not be evenly spaced. |
| `obj_pos` | yes | 3 numbers, metres | yes | Object centre. Torques are taken about it. |
| `obj_quat` | no (may be `null`) | 4 numbers (w, x, y, z), unit length | no | Object orientation. Part of what the object does, so part of the task, but the check reads the motion only through the forces. |
| `contact` | yes | `n_stations` of `true`/`false` | yes | Whether each station touches the object in this frame. Only stations marked `true` take part in the check. |
| `contact_point` | yes | `n_stations` of 3 numbers or `null`, metres | yes, for touching stations | Where the station touches: a point on the sphere's surface. `null` where it does not touch. |
| `force_on_object_N` | yes | `n_stations` of 3 numbers, newtons | yes, for touching stations | Force the station puts on the object. Write `[0, 0, 0]` where it does not touch. |
| `station_pos` | no | `n_stations` of 3 numbers or `null`, metres | only in place of a missing `contact_point` | Where the station itself is (a fingertip centre, a pad point). |
| `contact_normal` | no | `n_stations` of 3 numbers or `null`, unit length | no | Contact normal, into the object. |
| `derived_wrench` | no | object | no | For derived forces: the net wrench the motion needs, `force_N` and `torque_Nm` (about `obj_pos`), and `unbalanced_share`. |
| `untracked` | no | list of station names | copied into the result | Stations whose contact rests on a weakly tracked point. |

Any other per-frame key is allowed and is not read.

## What the follow check reads, and what it ignores

**Reads.**

- `n_stations`, `object.radius_m`, `id`, and `mu`, `allowed_overlap_m`
  and `other_contact` if present.
- Per frame: `contact`, `obj_pos`, and for each touching station its
  `contact_point` (or, if that is `null`, its `station_pos`) and its
  `force_on_object_N`.
- `t`, only when one of the time-linked options is on: `inertia`,
  `rate_limit`, `lag_s` or `trajectory`. Then the spacing of `t` matters:
  finger motion is fitted over a window of `t` (`FIT_HALF_S` in
  `follow.py`), and actuator limits link one checked frame to the next.
- `untracked`, copied into each frame of the result.

**Computes from what it reads.**

- The object wrench of a frame is not stored in the file. The check sums
  the forces of the touching stations, and their moments about
  `obj_pos`. That sum is the load the hand must carry.
- The inward normal at each contact is the direction from the contact to
  the object's centre (the object is a sphere).

**Parses but does not use.** `object.mass_kg`. If it is missing or not a
number, the check stops. The checker also requires it to be positive.

**Ignores.**

- `obj_quat`. The object's turning reaches the check only through the
  torques in the forces.
- `contact_normal`. The normal is recomputed, as above.
- Forces at stations marked not touching.
- `derived_wrench`, `frame`, `stations`, `source`, `derivation`, `notes`,
  `caveat`, `name`, `kind`, `schema`, and any other key.

## What is assumed rather than measured

- **The tennis-ball demonstration's mass is assumed.**
  `tools/import_demo_leader.py` sets it from `ASSUMED_MASS_KG`, the
  midpoint of the ITF tennis-ball specification, and says so in
  `object.mass_note`. The source recording has no mass.
- **Its forces are derived, not measured.** No force was recorded. The
  importer fits the tracked ball centre and orientation over a window
  (`FIT_HALF_S` in that tool) and takes the net wrench the motion needs:
  force m(a − g), torque Iα about the centre, with I a thin shell
  (assumed). It then splits that wrench over the touching digits: the
  smallest non-negative forces inside each digit's friction cone (its own
  `MU_DERIVE`, recorded in `derivation.mu`) that give it. That split is
  one choice among many with the same net wrench. Each frame records in
  `derived_wrench.unbalanced_share` how much of the wrench the cones could
  not give; `derivation` stores its median and 90th percentile.
- **The demonstration's contact flags follow a stated rule**, not a force
  measurement: the file's `notes` give it.
- **The friction coefficient used by the follow check is its default** for
  every stored leader, since none sets `mu`. The derivation's own `mu` is
  not passed on.
- **Simulated leaders** (from Wave_manip presets) are simulation only:
  "comparative, not predictive", as their `caveat` says. Their forces come
  from a dynamic MuJoCo run (`source.force_source`), so the object's
  weight and acceleration are inside them. Object size and mass come from
  the simulation, with any override recorded in `source`.

## The stored leaders, and how they differ

Three tools write leaders. `tools/export_tasks.py` does not: it writes
the static ring tasks in `data/tasks/` (`cup_hold`, `cup_turn`), a
different file read by `gramian_workshop/task.py` for the demos.

| Writer | Leaders | How the method was obtained |
|---|---|---|
| `tools/record_leader.py` | `w10_cup_physical_*`, `w12c_cup_analytic_lock` | A Wave_manip preset run headless in MuJoCo, control ticks thinned as `source.every_nth_tick` records. |
| `tools/import_demo_leader.py` | `tennis_new_cup_take3_derived` | A recorded human demonstration (three cameras) with forces derived from the ball's motion. |
| `tools/demo_handframe.py` | `tennis_new_cup_take3_handframe` | The derived leader carried into the demonstrator's palm frame, contacts re-made from the hand skeleton, forces re-derived there. |

The checker's table over every stored leader (output of
`python tools/check_leader.py --table data/leaders/*.json`, on the leaders
as stored at `0b628b8`):

| File | Frames | Stations | Touching per frame | Time span (s) | Step, median (s) | Largest step (s) | Radius (mm) | Mass (kg) | Net contact force / weight, median | Touching with zero force | station_pos gap where it stands in, median (mm) | Errors | Frames with a warning |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `tennis_new_cup_take3_derived.json` | 1319 | 5 | 1-5 | 0-33.6783 | 0.01666 | 1.183 | 36.93 | 0.0577 | 1.004 | 8 | - | 0 | 0 |
| `tennis_new_cup_take3_handframe.json` | 1240 | 5 | 1-5 | 0.06662-33.6783 | 0.01666 | 1.183 | 36.93 | 0.0577 | 1.005 | 23 | - | 0 | 0 |
| `w10_cup_physical_x0.1_a0.15_n4.json` | 610 | 4 | 2-3 | 0.512-20 | 0.032 | 0.032 | 60 | 0.1572 | 1.000 | 11 | 7.91 | 0 | 62 |
| `w10_cup_physical_x0.1_a0.15_n5.json` | 610 | 5 | 3-4 | 0.512-20 | 0.032 | 0.032 | 60 | 0.1572 | 1.000 | 320 | 9.21 | 0 | 297 |
| `w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4.json` | 297 | 4 | 2-3 | 0.512-9.984 | 0.032 | 0.032 | 33.5 | 0.06124 | 1.000 | 11 | 7.9 | 0 | 17 |
| `w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n5.json` | 297 | 5 | 3-4 | 0.512-9.984 | 0.032 | 0.032 | 33.5 | 0.06124 | 0.997 | 137 | 8.54 | 0 | 117 |
| `w10_cup_physical_x0.3_a0.15_n4.json` | 297 | 4 | 2-3 | 0.512-9.984 | 0.032 | 0.032 | 60 | 0.1572 | 1.000 | 21 | 7.88 | 0 | 45 |
| `w10_cup_physical_x0.3_a0.15_n5.json` | 297 | 5 | 3-4 | 0.512-9.984 | 0.032 | 0.032 | 60 | 0.1572 | 0.997 | 236 | 8.6 | 0 | 170 |
| `w10_cup_physical_x0.3_a0.15_sl0.3t_n4.json` | 297 | 4 | 2-3 | 0.512-9.984 | 0.032 | 0.032 | 60 | 0.1572 | 1.000 | 6 | 7.9 | 0 | 28 |
| `w10_cup_physical_x0.3_a0.15_sl0.3t_n5.json` | 297 | 5 | 3-4 | 0.512-9.984 | 0.032 | 0.032 | 60 | 0.1572 | 0.998 | 161 | 8.73 | 0 | 133 |
| `w12c_cup_analytic_lock.json` | 204 | 8 | 5-6 | 0.512-7.008 | 0.032 | 0.032 | 60 | 0.1572 | 1.000 | 205 | - | 0 | 0 |

Every stored leader passes: no errors. The warnings are differences in
structure, set out below. We have not changed any leader file; corrected
copies sit beside them (see "Corrected copies of stored leaders").

**The `w10_cup_physical_*` leaders: the contact flag and the physical
contact disagree.** `contact` is the simulation bridge's own per-station
flag (`stations.contact` in its feedback). The contact point and force
come from MuJoCo's contacts between the station and the ball. In some
frames they disagree both ways, and the checker warns:

- a station flagged touching with no MuJoCo contact: no `contact_point`,
  a zero force. The follow check then aims the finger at `station_pos`,
  the station's centre, which lies off the ball's surface by the gap in
  the table;
- a station flagged not touching that carries a MuJoCo contact force. The
  follow check ignores that force.

The checker's warnings, from the same run (messages shortened):

| File | Warning | Times | Frames |
|---|---|---|---|
| `w10_cup_physical_x0.1_a0.15_n4.json` | a station marked not touching carries a force | 56 | 51 |
| `w10_cup_physical_x0.1_a0.15_n4.json` | a touching station has no contact_point | 11 | 11 |
| `w10_cup_physical_x0.1_a0.15_n5.json` | a touching station has no contact_point | 306 | 297 |
| `w10_cup_physical_x0.1_a0.15_n5.json` | a station marked not touching carries a force | 157 | 156 |
| `w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4.json` | a station marked not touching carries a force | 14 | 9 |
| `w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4.json` | a touching station has no contact_point | 11 | 11 |
| `w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n5.json` | a touching station has no contact_point | 117 | 117 |
| `w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n5.json` | a station marked not touching carries a force | 46 | 46 |
| `w10_cup_physical_x0.3_a0.15_n4.json` | a station marked not touching carries a force | 36 | 27 |
| `w10_cup_physical_x0.3_a0.15_n4.json` | a touching station has no contact_point | 18 | 18 |
| `w10_cup_physical_x0.3_a0.15_n5.json` | a touching station has no contact_point | 187 | 170 |
| `w10_cup_physical_x0.3_a0.15_n5.json` | a station marked not touching carries a force | 57 | 57 |
| `w10_cup_physical_x0.3_a0.15_sl0.3t_n4.json` | a station marked not touching carries a force | 26 | 22 |
| `w10_cup_physical_x0.3_a0.15_sl0.3t_n4.json` | a touching station has no contact_point | 6 | 6 |
| `w10_cup_physical_x0.3_a0.15_sl0.3t_n5.json` | a touching station has no contact_point | 133 | 133 |
| `w10_cup_physical_x0.3_a0.15_sl0.3t_n5.json` | a station marked not touching carries a force | 57 | 57 |

"Times" counts station-frames; "Frames" counts frames with at least one.
The five-station leaders have far more of the first kind than the
four-station ones. These leaders also carry no `contact_normal` (every
entry `null`): the MuJoCo route does not write one.

**`w12c_cup_analytic_lock`: the analytic route.** Its forces come from the
bridge's own contact report (`source.force_source` is
`bridge grip_force.contacts`), with `contact_normal` given for every
touching station. Every contact point lies at the height of the ball's
centre. Many touching
stations carry a zero force (the table's count); the follow check still
counts them as contacts.

**`tennis_new_cup_take3_derived`: extra fields from the demonstration.**

- Top level: `frame` (units, a gravity direction, and a note on the
  frame), `stations` (digit names), `notes`, `derivation`,
  `rate_hz_median`, and counts carried over from the source recording.
  No `source.force_source`: there is no simulation.
- Per frame: `derived_wrench`.
- `n_frames` is the source recording's frame count, not this file's;
  frames were dropped where a touching digit had no contact point
  (`derivation.frames_dropped_touching_without_point`). The checker
  warns about the mismatch.
- `station_pos` is `null` for a digit with no tracked tip (the file's
  `notes` say so).
- `notes` are carried from the source and are partly out of date here:
  one says `force_on_object_N` is null, which was true of the source, not
  of this file.
- The time step is uneven: the largest step is in the table. With the
  time-linked options, the fit window and the actuator link see that gap.

**`tennis_new_cup_take3_handframe`: the same demonstration in the
demonstrator's palm frame.**

- `frame.gravity` is a sentence, not a direction: in the palm frame,
  gravity turns from frame to frame. The derived forces carry it.
- `obj_quat` is `null` in every frame. As in the derived leader,
  `station_pos` is `null` for a digit with no tracked tip.
- Contact points are re-made from the hand skeleton, and the forces
  re-derived at them from the derived leader's net wrench. The
  `derivation` block is carried over from the derived leader and does not
  describe that second step; the `handframe` block does.
- Per frame it adds `contact_phalanx`, `skeleton`, `palm_fit_rms_m`,
  `palm_points` and `untracked`, and drops `derived_wrench`.
- It is meant to be followed with a fixed placement, the palm frame as
  it stands, which `tools/demo_handframe.py` writes to
  `outputs/handframe_tennis_new_cup_take3.json`.
- Its `n_frames` and `notes` are carried over as in the derived leader.

## Corrected copies of stored leaders

We never edit a stored leader. Where one needs correcting, we write a
corrected copy beside it, named `<id>__<suffix>.json`. The copy's
`supersedes` names the leader it replaces, and its `notes` say what
changed. New runs should use the corrected copy. Results already computed
from the old leader stay as they are.

| Corrected copy | Supersedes | What changed | Written by |
|---|---|---|---|
| `w10_cup_physical_x0.1_a0.15_n4__contactfix` | `w10_cup_physical_x0.1_a0.15_n4` | `contact` follows the physical contact | `tools/record_leader.py --contact-from mujoco` |
| `w10_cup_physical_x0.1_a0.15_n5__contactfix` | `w10_cup_physical_x0.1_a0.15_n5` | as above | as above |
| `w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4__contactfix` | `w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4` | as above | as above |
| `w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n5__contactfix` | `w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n5` | as above | as above |
| `w10_cup_physical_x0.3_a0.15_n4__contactfix` | `w10_cup_physical_x0.3_a0.15_n4` | as above | as above |
| `w10_cup_physical_x0.3_a0.15_n5__contactfix` | `w10_cup_physical_x0.3_a0.15_n5` | as above | as above |
| `w10_cup_physical_x0.3_a0.15_sl0.3t_n4__contactfix` | `w10_cup_physical_x0.3_a0.15_sl0.3t_n4` | as above | as above |
| `w10_cup_physical_x0.3_a0.15_sl0.3t_n5__contactfix` | `w10_cup_physical_x0.3_a0.15_sl0.3t_n5` | as above | as above |
| `tennis_new_cup_take3_derived__nframesfix` | `tennis_new_cup_take3_derived` | `n_frames` is the file's own count; out-of-date carried notes marked | `tools/fix_demo_leader_header.py` |
| `tennis_new_cup_take3_handframe__nframesfix` | `tennis_new_cup_take3_handframe` | as above | as above |

**Why the w10 contact flags were wrong.** `tools/record_leader.py` took
`contact` from the bridge's `stations.contact`. On the route these leaders
use (the analytic contact model, helpers off), the bridge fills that flag
from the wave pattern (`wave_contact`, in `friction_torque_implicit` in
`unified_v6/bridge/server.py`): the stations the gait plans to be in
stance, not the stations touching the ball. The contact point and the
force come from MuJoCo's contacts between station and ball. So a station
in its planned stance but hovering off the ball was flagged touching with
no point, and the follow check aimed the finger at the station's centre;
a station touching the ball outside its planned stance was flagged not
touching, and its force was dropped.

**How the w10 copies were made.** We ran the recorder again on the same
Wave_manip commit as each stored leader (its `source.commit`), unpacked
with `git archive` so the Wave_manip checkout was not changed, with the
settings in its `source`. In its default mode the recorder reproduces the
stored leaders' frames exactly, except
`w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4`, where the last rounded
digit differs in the forces of its first frame and in the orientation of
one frame. With `--contact-from mujoco` the flag is on exactly when the
station has a MuJoCo contact with the ball, so every touching station has
a contact point. Every other field per frame comes from the same
recording. We invented no contact point: a flag with no MuJoCo contact is
set off. Each copy counts its changes in `source.contact_fix` (flags set
off, flags set on, frames changed) and names the recorder's commit in
`source.recorder_commit`. A station touching the ball can carry a zero
force; it stays flagged touching, as MuJoCo reports the contact.

**How the tennis copies were made.** `tools/fix_demo_leader_header.py`
copies the leader and changes only its header. `n_frames` becomes the
file's own frame count, and the source recording's count is kept as
`n_frames_source`. Each carried note is tested against the file; one that
no longer holds is kept word for word behind "OUT OF DATE in this file",
with the reason. `copied_from` gives the stored file's path and sha256.

The checker's table for the corrected copies (output of
`python tools/check_leader.py --table data/leaders/*__*.json`, on the
copies as committed with this section). No copy has an error or a warning:

| File | Frames | Stations | Touching per frame | Time span (s) | Step, median (s) | Largest step (s) | Radius (mm) | Mass (kg) | Net contact force / weight, median | Touching with zero force | station_pos gap where it stands in, median (mm) | Errors | Frames with a warning |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `tennis_new_cup_take3_derived__nframesfix.json` | 1319 | 5 | 1-5 | 0-33.6783 | 0.01666 | 1.183 | 36.93 | 0.0577 | 1.004 | 8 | - | 0 | 0 |
| `tennis_new_cup_take3_handframe__nframesfix.json` | 1240 | 5 | 1-5 | 0.06662-33.6783 | 0.01666 | 1.183 | 36.93 | 0.0577 | 1.005 | 23 | - | 0 | 0 |
| `w10_cup_physical_x0.1_a0.15_n4__contactfix.json` | 610 | 4 | 2-4 | 0.512-20 | 0.032 | 0.032 | 60 | 0.1572 | 1.000 | 3 | - | 0 | 0 |
| `w10_cup_physical_x0.1_a0.15_n5__contactfix.json` | 610 | 5 | 2-4 | 0.512-20 | 0.032 | 0.032 | 60 | 0.1572 | 1.000 | 19 | - | 0 | 0 |
| `w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4__contactfix.json` | 297 | 4 | 2-4 | 0.512-9.984 | 0.032 | 0.032 | 33.5 | 0.06124 | 1.000 | 5 | - | 0 | 0 |
| `w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n5__contactfix.json` | 297 | 5 | 2-4 | 0.512-9.984 | 0.032 | 0.032 | 33.5 | 0.06124 | 1.000 | 20 | - | 0 | 0 |
| `w10_cup_physical_x0.3_a0.15_n4__contactfix.json` | 297 | 4 | 2-4 | 0.512-9.984 | 0.032 | 0.032 | 60 | 0.1572 | 1.000 | 11 | - | 0 | 0 |
| `w10_cup_physical_x0.3_a0.15_n5__contactfix.json` | 297 | 5 | 2-4 | 0.512-9.984 | 0.032 | 0.032 | 60 | 0.1572 | 1.000 | 49 | - | 0 | 0 |
| `w10_cup_physical_x0.3_a0.15_sl0.3t_n4__contactfix.json` | 297 | 4 | 2-4 | 0.512-9.984 | 0.032 | 0.032 | 60 | 0.1572 | 1.000 | 7 | - | 0 | 0 |
| `w10_cup_physical_x0.3_a0.15_sl0.3t_n5__contactfix.json` | 297 | 5 | 2-4 | 0.512-9.984 | 0.032 | 0.032 | 60 | 0.1572 | 1.000 | 28 | - | 0 | 0 |

Compared with the stored leaders' table above, the touching count per
frame now follows the ball: some frames of the five-station copies have
fewer stations touching than any frame of the stored ones, and some frames
of the four-station copies have more.

## Bringing your own task

1. **Say what the object does.** Write `t`, `obj_pos` and, if you have it,
   `obj_quat` for each frame. That is the task.
2. **Say how it is done.** Choose the stations. For each frame write
   `contact`, `contact_point` (on the sphere, `null` where not touching)
   and `force_on_object_N` (on the object, `[0, 0, 0]` where not
   touching). That is the method. Another method for the same motion is
   another file.
3. **Put the load in the forces.** The check does not add gravity. In a
   still hold the forces should sum to the object's weight; the checker
   prints the median ratio.
4. **If you have no forces**, derive them from the motion as
   `tools/import_demo_leader.py` does. You then need the object's mass
   and inertia; state in `object.mass_note` and `derivation` what you
   assumed.
5. **Keep the contact flag and the force in step.** A touching station
   needs a contact point; a station not touching should carry no force.
6. **Say how much overlap you allow**, if not the default: how deep a
   finger may press in at a contact (`allowed_overlap_m`), and whether the
   rest of the hand may touch the object (`other_contact`).
7. **Record where it came from** in `source`, and what the numbers are
   good for in `caveat`.
8. **Check it**: `python tools/check_leader.py your_file.json`. Then put
   it in `data/leaders/` and run the follow check on it for each hand:
   `python tools/run_follow.py --leader-id <id>` (needs MuJoCo and the
   hand models; see the README).

## Limits of the format today

- One rigid object, a sphere.
- No tolerance on the object's path is stored, though the task is the
  object's motion within a tolerance. The follow check uses its own
  tolerance on the wrench instead (`RESID_TOL` in `follow.py`).
- The file names the leader (`id`, `name`), not the task and the method
  separately. Two leaders for the same task are linked only by their
  names.
- Station positions off the object (`station_pos`) are used only as a
  stand-in for a missing contact point.
- The allowed overlap is one pair of numbers for the whole run and every
  station. Fingers that are not touching are not checked for overlap,
  and fingers are not checked against each other.
