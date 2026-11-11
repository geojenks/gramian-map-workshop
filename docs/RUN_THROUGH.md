# One run through the whole chain

Run on 11 Nov 2026 in a fresh clone of the repository at 84fed67, to find
out how far a newcomer gets from an actuator or a hand to a verdict, and
where they stop. `docs/MINIMAL_PRODUCT_PLAN.md` says what the chain is.

Every number on this page is copied from a file in
`outputs/run_through_11nov/`: the logs of the commands as they ran
(`logs/`), the files they wrote, and `tables.md`, which
`make_tables.py` in that folder writes from them. The paths of the machine
it ran on are replaced by `<clone>`, `<hands env>` and `<analysis env>`.

`outputs/walkthrough_11nov.html` is the same run as a page to step
through, built by `tools/make_walkthrough.py` from that folder.

## The result

Two of the three ways in that were tried reach a verdict. The third stops
at the hand. The mesh route was not run.

| Way in | Reaches | Where it stops or what it needed |
|---|---|---|
| A hand as a model file (left Allegro, MuJoCo Menagerie) | A verdict, with both margins and a failures report | Nothing beyond that page's steps. It repeats `docs/BRING_YOUR_HAND.md` and gives the same verdict, green in all 40 frames |
| An actuator as test curves (`data/test_curves/mosadegh2014_fPN.csv`) | A verdict, with both margins and a failures report | Two choices in the hand file that the docs do not give (a contact radius, and which way the fingers bend), and the result carries no units caveat (issues 32 and 35) |
| An actuator as a joint table (`data/joint_tables/shadow_hand_finger.csv`) | A record | The hand build crashes on the record (issue 33) |

The verdicts, as `tables.md` gives them:

| Result | Verdicts | Scale margin | Disturbance margin | Joints posed past a limit | Follow time (s) | Units caveat in the result |
|---|---|---|---|---|---|---|
| `results/follow_allegro_left__hold_allegro_left__self__selfoverlapfix__fixed.json` | green 40/40 | 217 | 59.1 (-ty) | none | 61.4 | no |
| `results/follow_mosadegh_x3_ring__hold_mosadegh_x3_ring__self__selfoverlapfix__fixed.json` | red 40/40 (no_contact) | not computed | not computed | none | 24.1 | no |
| `results/follow_mosadegh_x3_ring_flipped__hold_mosadegh_x3_ring_flipped__self__selfoverlapfix__fixed.json` | green 40/40 | 5.76 | 0.497 (-tx) | f0_hinge0 upper by 0.115 to 0.115 in 40 finger-frames; f1_hinge0 upper by 0.115 to 0.115 in 40 finger-frames; f2_hinge0 upper by 0.115 to 0.115 in 40 finger-frames | 103.8 | no |

What these verdicts are, and are not:

- Each is a hand holding the hold made from its own kinematics
  (`tools/make_hold_leader.py`): the known answer for the check. No hand
  made here was run on a task from elsewhere. That needs a fresh placement
  search, and the plan puts one at 45 to 65 min a hand.
- The green for the hand made from test curves is not a strength result.
  `tools/check_record.py` says of its record "no force data: this actuator
  cannot push in the tool", and the motor's force in the built model is the
  record's unitless drive level, not newtons. The registered record-built
  hand carries a "PROVISIONAL" caveat for exactly this; a hand added with
  `tools/add_hand.py` does not (issue 35), which is why the last column
  reads "no". Read its two margins as provisional until the units ruling
  (ruling 2 in the plan; issue 31 is the related fault in the check).
- That same green stands with the base hinge of each finger posed 0.115 rad
  past its upper limit in every frame (issue 34). The built model gives
  that hinge the range -0.193 to 0 rad
  (`hands/mosadegh2014_fPN_from_curves_x3_ring_flipped/hand.xml`).
- The contact radius (0.008 m), the palm and the 30 mm ring of the
  test-curve hand are choices made for this run, not data.

## What was run

The full list of steps, with each command, exit code and wall time, is the
first table of `outputs/run_through_11nov/tables.md`. All the logged steps
together took 266 s. The laptop was shared with other jobs, so the times
are a guide only.

Two environments were used: one with the hands extra (MuJoCo), and one
for analysis that has matplotlib. The README does not mention the second.

### Actuator in

- Test curves to a record: `tools/curves_to_record.py` wrote a tier 1
  record in 1 s. `tools/check_record.py` reads a stroke of 94.9 mm, motion
  along [-0.539, -0.842, 0], and no force or stiffness data.
- Joint table to a record: `tools/joint_table_to_record.py` wrote a tier 1
  record. The checker reads a push force of 10.42 N and a stroke of 129 mm,
  with two stiffness entries missing.
- The mesh route was not run. It is hours of SOFA compute in the M-11
  project; its records are already in `data/records/`.
- `tools/figure_gramians.py` drew the stored records' Gramians in the
  analysis environment. In the hands environment it fails for want of
  matplotlib, which no install extra provides (issue 36).

### Hand

- From the test-curve record, three stand-in fingers on a ring
  (`tools/build_record_hand.py`). Three tries were needed
  (`logs/run0_log.txt` for the first two; `logs/run_log.txt` and
  `logs/run2_log.txt` for the red hold; `logs/run3_log.txt` for the hand
  that closes):
  1. Naming the record by its id fails with a `KeyError`: only records in
     `data/records/MANIFEST.json` are known by id. A file path works.
  2. By path, the build stops with "no contact radius: record gives no
     material volume and length". The test-curve format has no place for
     the actuator's size, so the hand file must give `contact_radius_m`.
  3. With the radius given, the build passes its fit check (tip error
     0.279 mm). But this record moves along -Y, and with the ring layout
     copied from the stored example hand
     (`data/hands/wall_0.0010m_x3_ring/hand.json`) the fingers curl away from the
     ring's centre: the hold is red in all 40 frames, each finger 18.9 mm
     off its contact. Reversing each finger's `bend_axis` in the hand
     file gives a hand that closes on the ball (facing 6.0 degrees, where
     it was 98.3).
- From the joint-table record, the build crashes with
  `numpy.linalg.LinAlgError: Singular matrix` in
  `gramian_workshop/record_fit.py`. We have not found the cause.
- From a model file, `tools/add_hand.py` took the left Allegro hand and a
  short description file and added it in 1 s. The built test-curve hands
  were added the same way, since a record-built hand that is not in
  `gramian_workshop/hands.py` has no other route into the check.

### Task

- `tools/check_leader.py` read one simulated task
  (`w10_cup_physical_x0.1_a0.15_n4__contactfix`: 4 stations, 610 frames)
  and one human demonstration
  (`tennis_new_cup_take3_handframe__nframesfix`: 5 stations, 1240 frames)
  without complaint.
- `tools/make_hold_leader.py` made a hold for each hand: 43 s for the
  Allegro, 14 s and 8 s for the two test-curve hands. We passed
  `--out-dir` to keep the run's files apart. `tools/run_follow.py
  --leader-id` reads only from `data/leaders/`, where the tool writes by
  default, so each task file was then copied there.

### Verdict

- `tools/run_follow.py --margins` took 62 s for the Allegro, 25 s for
  the test-curve hand that curls outward, and 105 s for the one that
  closes. These are whole steps; the table above gives the check's own
  time.
- `tools/report_failures.py` and `tools/view_follow.py` wrote the failures
  report and the margins table for each
  (`results/*__failures.md`, `results/follow_runthrough_3.md`).

## What the run did not cover

- Any task other than each hand's own hold.
- The mesh route, and the registered record-built hand `wall_x3_ring`.
- Fetching the hand models: `external/` was copied from an existing
  checkout, not fetched with `tools/fetch_hands.py`.
- Installing from nothing: both environments already existed.
- The steps before the logs begin. The first build of the test-curve hand
  with a contact radius was run by hand and is not in a log; its files are
  in `hands/mosadegh2014_fPN_from_curves_x3_ring/`.

## Follow-ups raised

| Issue | What |
|---|---|
| 32 | Test-curve record to a hand: not found by id, no contact radius, bend direction not documented |
| 33 | The hand build crashes on the joint-table record |
| 34 | A finger posed 0.115 rad past a joint limit still reads green |
| 35 | A record-built hand added with `tools/add_hand.py` loses the provisional units caveat |
| 36 | `tools/figure_gramians.py` needs matplotlib that no extra installs; `tools/check_record.py --help` gives a traceback |
