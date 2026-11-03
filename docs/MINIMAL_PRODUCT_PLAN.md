# Plan to a minimal product

Written 27 Oct 2026, after the workshop talk was agreed. It sits beside
`docs/WORKSHOP_PLAN.md` and does not replace it: that plan tracks the
research steps; this one lists what must work, end to end, for someone to
bring an actuator or a hand and get a verdict on a task. Moving to dynamics
is being planned in another session and is left out here.

## What the minimal product is

One flow that a visitor can follow with their own data:

1. **Actuator in.** An actuator enters the map from one of three routes,
   and every route ends in the same actuator record (schema in the M-11
   project, `gramian_map/docs/RECORD_SCHEMA.md`). The record is the only
   way in.
2. **Hand.** Actuators are placed into a hand, or a ready-made hand is
   loaded from a model file.
3. **Task.** A task is taken from an example: a simulation's contacts, or a
   recorded human demonstration.
4. **Verdict.** The frame-by-frame force check says, for each frame,
   whether the hand can follow the task by this method.

The verdict is narrow by design. A fail means "this hand cannot do this
task by this method", never "this hand cannot do this task". A task can be
done by one method (a gait, a regrasp, a wrist roll), and a given hand may
need a different one.

## Where each stage stands

| Stage | Working now | Part way | Not started |
|---|---|---|---|
| Actuator in: mesh | SOFA pipeline in the M-11 project; pneunet, bellows, McKibben, cable-finger records | | Run live at the workshop (not planned: hours of compute) |
| Actuator in: test data | Test-curve file format (`docs/TEST_CURVE_FORMAT.md`) and converter to a record (`tools/curves_to_record.py`, both ways); records digitised from published curves (`mosadegh2014_*`, `abboodi2026_*`); `forms/record_form.html` and `tools/check_record.py` | Angle-only curves are kept as `free.bend_angle_rad` and make no tip path, so the stand-in fitter refuses them (ruled, issue 23) | |
| Actuator in: analytical | `shadow_hand_finger` (forward kinematics from published dimensions) | | A worked example starting from a per-joint table (axis, position, range, torque limit, stiffness; ruled on issue 16) |
| Actuator onto the map | `make_atlas.py` in the M-11 project rebuilds the map from the records | | Adding one uploaded record without a full rebuild |
| Hand from a model file | Allegro, Shadow, LEAP (MuJoCo Menagerie), Linker L20 (vendor URDF), human-finger hands | | Loading a visitor's own URDF or MJCF |
| Hand from actuator records | Hand file and build tool (`tools/build_record_hand.py`): fitted stand-in fingers on a palm, fit check, round trip; `wall_x3_ring` (three `wall_0.0010m` stand-ins on a 30 mm ring) | Verdicts provisional: the drive is the record's unitless level (ruling 2) | |
| Task from a simulation | `tools/record_leader.py`, `tools/export_tasks.py`; the task-file format (`docs/TASK_FILE_FORMAT.md`) with `tools/check_leader.py`; corrected `__contactfix` copies of the w10 cup leaders; a static hold leader per hand from its own kinematics (`tools/make_hold_leader.py`, `data/leaders/hold_<hand>__self.json`), checked with margins (`outputs/hold_summary.md` from `tools/tabulate_holds.py`) | The Shadow's hold is red (torque): its coupled J1/J2 pair cannot carry the pad load as posed, and the verdict moves with the pose (an earlier pose of the same search was amber) | |
| Task from a human demo | `tools/import_demo_leader.py` (tennis ball, take 3); corrected `__nframesfix` copies | Object mass is assumed, not measured | |
| Verdict | `follow.py` for model-file hands; springs read from the model; fingers with fewer drives than joints posed by equilibrium; each joint type checked in its own unit (per-finger test); overlap scored against the task's stated allowance (default 3 mm); stored placements reused | Both margins computed but not stored with results (issue 8); redistribution and margins still put all joints under one tolerance; fresh placement searches take 45 to 65 min a hand (screening on a few frames not yet built) | |

### Stored results that predate the current check

Nothing in `outputs/` written before 28 Oct can be quoted as a verdict:

- Results written before 6922372 (26 Oct) predate the check that a finger
  must not sink into the object. 35 of the 49 stored follow results flip
  under the current code (issue 21, 28 Oct). They stay as they are; their
  verdicts are superseded by a fresh placement search.
- Results from the stored `w10_cup_physical_*` leaders aimed, in up to half
  the frames, at a station flagged touching with no contact point: the
  station centre, 8 to 9 mm off the ball. The `__contactfix` copies replace
  those leaders (issue 22, 28 Oct); no stored result has been re-run on them
  yet.
- The two `follow_wall_x3_ring__*__provisional` files were written before
  equilibrium posing (89a6b86). Their reds are an artefact of a pose choice
  from geometry alone, not verdicts on the finger (issue 21 ruling 3).

## Rulings needed before the work

1. **How a record-built finger enters the force check (issue 5).** This
   decides what a soft hand is in this tool. It is the one join between
   "actuator in" and "verdict"; until it exists, an uploaded actuator stops
   at the record.
2. **Units.** The simulated records run at low pressures (the pneunet
   `wall_0.0010m` spans 0 to 398 Pa) and are comparative only. The
   published-data records are in physical units (`mosadegh2014_fPN` spans
   0 to 72 662 Pa). On one map they will mislead. Options: keep two maps,
   or scale the simulated records to physical units by a stated rule.
3. **The hand file.** A plain file that says which record sits at which
   mounting point on a palm. A drag-and-drop page can come later and write
   the same file.

## Order of work

1. **One example per actuator route, shown entering the map.**
   - Mesh: `wall_0.0010m`, pre-run, shown from mesh to record to map.
   - Test data: `mosadegh2014_fPN`, from digitised curves to record.
   - Analytical: a finger from a written Jacobian, through the record form.
   - Copy the published-data records into `data/records/` with
     `tools/import_records.py`; the workshop copy predates them.
2. **State the test-data format** (done 28 Oct, da65543). Most visitors will bring test curves for
   one actuator: input against displacement, input against blocked force,
   with units. Write the format down with one filled example.
3. **The hand file and the join into the force check** (rulings 1 and 3;
   done 28 Oct, 0b628b8 and a479ad7 to 9b74767, provisional on ruling 2).
   Then one hand built from records, run on the cup tasks: run on the stored
   cup leaders only; still to run on the corrected copies.
4. **Loading a visitor's model file.** One hand loaded from a URDF that we
   did not prepare ourselves.
5. **The task-file spec** (done 28 Oct, faa5b37 to cfbc15b). Publish the leader file format so others can
   bring tasks from their own simulations.
6. **Store and show both margins** with every verdict (issue 8).

## The task library

Show the whole tree; fill a few branches completely. Each branch needs a
recorded demonstration, and they cost effort. The empty branches are the
invitation to other groups. Elliott and Connolly's patterns give a citable
way to lay out the top of the tree.

| Task | Method | State |
|---|---|---|
| Hold a ball | Fingertip cup, static | Filled: `cup_hold`; and `hold_<hand>__self`, one self-made hold per hand (the known answer for the check) |
| Turn about the vertical axis | Make/break wave in a fingertip cup | Filled: `cup_turn` |
| Turn a ball in the hand | Human demonstration | Filled: tennis ball, take 3 |
| Turn about each horizontal axis | Gait, regrasp | Empty |
| Turn a screwdriver half a turn | Contacts held, wrist roll | Concept only |

## Abstractions to test, to find the limits

- Fewer fingers: a five-finger demonstration given to four or three
  fingers, by searching ways to reassign the contacts.
- Two hands sharing one task.
- Looser tolerance on the object's path.

Where we expect the method to break, stated in advance:

- soft fingers whose contact point moves as they deform;
- anything dynamic (the check is quasi-static);
- tasks whose contact plan nobody has written yet;
- mixing simulated and physical units (ruling 2).
