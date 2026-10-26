# Workshop plan

Written 26 October 2026 against `main` at b1207b7. This file arranges the
workshop's story and says, step by step, what the repo already has, what is
missing and what to do next. It is kept current as work is reported in.

**Task state lives in the repo's GitHub Issues, not here** (agreed 26
October). The Issues list is this repo's status board: open or closed, and
who holds a task. This file keeps the story, the claims and the overclaim
flags, and links each task to its issue. Labels: one per story step
(`step: ...`), plus `blocked`, `needs ruling` and `session`. Each running
session has a `session` issue. A worker puts `Closes #n` in the commit that
finishes a task and comments a short summary on the issue. The M-11 board
(`PLAN_M11_NEXT_QUARTER.md` in Gramain_extraction) stays separate.
Every "exists" below names a file we opened; nothing here is taken from a
summary. No numbers are quoted: where a result matters, the plan names the
output file that holds it.

The story has six steps:

1. Making gramians (controllability matrices) for a range of actuators.
2. Using these to find feasible grasps once arranged into manipulators or
   hands.
3. Assessing the quality of static grasps through a given manipulation, by
   classic, established methods.
4. Moving to a dynamic assessment.
5. (Future) Soft: why it is harder, what we can do, what we actually need.
6. (Future) When and why this whole thing is useful.

Steps 1 to 4 are being worked on now. Steps 5 and 6 are future.

## The result in brief

- **Steps 1 and 3 are on firm ground**, with two honest caveats: this repo
  does not make gramians (the M-11 pipeline does), and our grasp tests are
  the standard ones, which we must cite as such.
- **Step 2 has a seam.** Fingers built from actuator records and rigid hands
  are two separate code paths. Record-built fingers are placed on a ring by
  assumption, with no reach check. Rigid hands are checked for reach but use
  no records. Neither path reads a record's Gramian: they read blocked
  force, free motion and stiffness.
- **Step 4 is not yet dynamic.** `follow.py` is a static check repeated
  frame by frame along a gait. The object's own acceleration is in the
  leader's forces, because the leader was recorded from a dynamic
  simulation, but nothing about the hand moves: no hand inertia, no joint
  speed, no impacts, and no link between one frame's pose and the next.

## Who is on what now

| Session | Working on | Where it lands | Issue |
|---|---|---|---|
| workshop orchestrator | this plan and the issues | `docs/WORKSHOP_PLAN.md` | #16 |
| [Review literature to situate the Gramian Map](#local_448ca5b2-491c-42bc-b6f1-138783366f27) | gaps and commonalities with the literature | `docs/LITERATURE_REVIEW.md` | #17 |
| [Apply-side wrench measure in follow.py](#local_6d742daf-932e-4b0a-8c14-58e933454c24) | per frame, the largest force and twist the hand can apply to the object along each axis | `gramian_workshop/follow.py` (not on `main` yet) | #18 |
| hand-models session (reported, not linked) | measured human hands | `data/hands/`, `tools/build_human_hand.py` | #19 |
| Allegro-results session (reported, not linked) | Allegro follow results | `outputs/follow_allegro__*.json` | #20 |

The session links open in the Claude app only. The workshop plan does not
hand these sessions new work; tasks below that touch their files wait for
them.

## Step 1. Making gramians for a range of actuators

**What the workshop shows.** Each soft actuator gets a record: how far and
which way its tip moves for each input, how hard it pushes when held, and
how springy it is. The richest records carry the empirical controllability
Gramian of tip motion, built from a finite-element sweep. The literature
review found no other work that does this for soft actuators
(`docs/LITERATURE_REVIEW.md`, "The result in brief").

**What exists.**

- The records: `data/records/*.json`, copied byte for byte from the M-11
  repo by `tools/import_records.py`, pinned by commit and sha256 in
  `data/records/MANIFEST.json`, with trust flags.
- The format: `docs/RECORD_SCHEMA.md` (copied from the M-11 repo) and the
  plain guide `docs/RECORDS.md`, which sets out tiers 0 to 2.
- Making a record by hand: `forms/record_form.html` (browser form, converts
  units) and `tools/check_record.py` (validates and prints what the tool
  reads).
- One Gramian built in this repo: the human index finger's force Gramian
  from the published Valero-Cuevas 1998 model,
  `tools/valero_cuevas_finger.py`, writing
  `outputs/vc1998_index_finger.json` and the draft records
  `data/records_draft/human_index_vc1998_{nominal,adjusted}.json`.
- The gramian formulas themselves live in the M-11 repo
  (`gramian_map/jacobian.py` in Gramain_extraction), not here.

**What is missing.**

- No code in this repo reads a record's Gramian. `gramian_workshop/finger.py`
  reads the push direction, motion direction, stroke and stiffness only
  (its `push_direction`, `passive_stiffness`, and the rules in
  `docs/RECORDS.md`, "How the tool reads a record"). The record's `shape`
  block (mode vectors, eigenvalues, authority) is not used.
- No calibrated record. All simulated forces are comparative
  (`README.md`, "Honesty about magnitudes" in `docs/RECORDS.md`).
- The human-finger records are drafts, and our rebuild is off in one of five
  force directions by about half (`docs/LITERATURE_REVIEW.md`, "What not to
  claim").

**Next tasks.**

1. (#1) Decide what the workshop says a Gramian is for. Either the tool reads the
   Gramian (for example, the reachable tip-motion ellipse from W_c as the
   finger's motion set, in place of the single free direction), or the talk
   says plainly that the Gramian characterises the actuator and the grasp
   check uses its Tier 0 and 1 summaries. Needs George's ruling.
2. (#2) One figure that shows the range: the records' Gramians side by side, made
   by a script from `data/records/`. Belongs with the M-11 capability map
   work, so check there first.
3. (#3) One measured or datasheet record in real units, so at least one strength
   result can be stated in real terms.
4. (#4) Check the human-finger rebuild against the other two published routes
   (literature review, ranked list, item 4). Wait for the literature session.

**Where the narrative overclaims.** "Making gramians for a range of
actuators" is true of the M-11 pipeline, and the workshop should say the
gramians come from there. Here, only the human finger's force Gramian is
made, and the grasp checks use none of them.

## Step 2. From records to feasible grasps in hands

**What the workshop shows.** N copies of an actuator on a cup ring: can they
hold a ball, and can a make/break wave turn it? Then real hands: can a rigid
hand follow a recorded manipulation, frame by frame?

**What exists.**

- Record-built fingers on a ring: `gramian_workshop/finger.py` (a finger
  from a record), `gramian_workshop/arrange.py` (copies on the ring, mounts
  `aimed` and `palm` at yaw 0 and 90 degrees), `gramian_workshop/readout.py`
  (`ideal_hold`, `soft_hold`, `soft_hold_passive`, all on the vendored W20
  core in `gramian_workshop/vendor/capability_core.py`).
- The three demos: `demos/cup_hold.py`, `demos/cup_turn.py`,
  `demos/passive.py`, writing `outputs/cup_hold*.{txt,json,html}`,
  `outputs/cup_turn*.{txt,json,html}`. The reason ladder (no contact,
  direction, friction, cone edge, deflection, strength, met) is in
  `README.md`.
- The turn: `gramian_workshop/turn.py`, a kinematic duty schedule (its
  docstring declares it is not a controller and nothing is stepped).
- Leaders: `tools/record_leader.py` records a Wave_manip preset as a leader;
  `data/leaders/*.json`; `tools/view_leader.py` draws them
  (`outputs/leader_*.html`).
- Rigid and human hands: `gramian_workshop/hands.py` (`HANDS`: Allegro,
  Shadow, LEAP, Linker L20, the human-finger hands, and the measured hands),
  `tools/build_human_hand.py`, `data/hands/*/hand.xml`. The robot hand models
  are fetched into the git-ignored `external/` by `tools/fetch_hands.py`.
- Following a leader: `gramian_workshop/follow.py` (`follow`,
  `placement_search`, `best_assignment`, `FingerIK`), run by
  `tools/run_follow.py`, drawn by `tools/view_follow.py` into
  `outputs/follow.html` and `outputs/follow_respaced.html`.
- Why rigid hands reach badly: `tools/reach_ladder.py` (`outputs/ladder/`,
  `outputs/reach_ladder.log`), `tools/reach_checks.py`
  (`outputs/reach_only_*.json`, `outputs/wrist_shadow.log`), and re-scoring
  without a new search, `tools/rescore.py`, `tools/rescore_side.py`
  (`outputs/rescore_*.json`, `outputs/follow_*__side_kq*.json`).
- Hand and finger comparisons: `tools/hand_cards.py`
  (`outputs/hand_cards.txt`), `tools/finger_compare.py`
  (`outputs/finger_compare.md`).
- Checks: `python -m gramian_workshop.gates` (gates V0 to V5,
  `outputs/gates.json`) and `tests/test_gates.py`.

**What is missing.**

- The seam between the two halves. `follow.py` does not import
  `finger.py`: rigid hands use no records. Record-built fingers have no
  kinematics, so their placement is assumed and reach is not checked
  (`README.md`, "Limits, stated").
- A soft hand that can follow a leader: record-built fingers mounted on a
  palm, with reach from the record's free motion, run through `follow.py`.
- One input per soft finger, pushing only along its blocked-force direction
  (`README.md`, "Limits, stated").
- The demo results are draft against a registration awaiting sign-off
  (`README.md`, "What the first run shows").

**Next tasks.**

1. (#5) Bridge the two halves: a record-built finger as a `follow.py` finger
   (tip motion from the record's free displacement per input level, force
   set from its blocked force). Design first, with George, since it decides
   what a soft hand is in this tool.
2. (#6) Get the W20 step 6 registration signed off, or say in the talk that the
   demo results are draft.
3. (#7) Fold the hand-models session's measured hands into `outputs/follow.html`
   once it reports.

**Where the narrative overclaims.** "Using these [gramians] to find feasible
grasps" is not what the code does: the grasp tests use record summaries, not
gramians. "Arranged into manipulators or hands" is true for rigid hands and
for a ring of soft fingers, but not yet for a soft hand.

## Step 3. Static grasp quality through a manipulation

**What the workshop shows.** For every frame of a manipulation, whether the
hand can make the leader's contacts and carry the leader's load: green,
amber or red, with the first test failed as the reason. Two optional
margins grade the frames that pass.

**What exists.**

- The per-frame tests in `gramian_workshop/follow.py`: reach and facing
  (`FingerIK.best`), the per-finger friction cone, the joint-torque balance
  (`_finger_columns`, `_columns_hold`), and redistribution over the contacts
  made (`_redistribute`, `_torque_stack`).
- Scale margin, `_scale_margin`: how many times larger the leader's load
  could get (`follow(margin=True)`; off by default).
- Disturbance margin, `_disturbance_margin`: the smallest extra push or twist
  that breaks the grip, tried along 12 signed axes, so it is an upper bound
  on the true smallest (`follow(disturbance=True)`; off by default).
- The mapping to the literature: `docs/LITERATURE_REVIEW.md`, section 1 and
  the part comparing our code with SynGrasp. Grasp matrix, hand Jacobian,
  pyramid friction cone and linear-programme feasibility are the handbook's;
  the zero-unactuated-torque rule is Bicchi and Prattichizzo 2000; the scale
  margin is Li and Sastry's single-wrench form; the disturbance margin is the
  worst-direction largest-ball measure limited by torque.

**In flight.** The apply-side measure: per frame, the largest force and
twist the hand can apply along each axis
([Apply-side wrench measure in follow.py](#local_6d742daf-932e-4b0a-8c14-58e933454c24)).
Not on `main` yet.

**What is missing.**

- No stored follow result carries either margin: in every
  `outputs/follow_*.json` on `main` at b1207b7, the settings have the
  margins off or absent (our check, 26 October). The margins are off by
  default, and `tools/view_follow.py` does not draw them.
- No scalar indices (minimum singular value, isotropy, manipulability
  volume): literature review, ranked list, item 6.
- No check of our margins against a published worked example.
- Only the tennis-ball cup leaders; no standard objects (YCB, ranked item 5)
  or standard in-hand patterns (Elliott and Connolly, ranked item 7).
- Hard-finger contact only; no soft-finger contact type.

**Next tasks.**

1. (#8) Re-run the published follow results with both margins on and show them
   on `outputs/follow.html`. New output names; do not overwrite. Wait for
   the Allegro-results session so the runs are not duplicated.
2. (#9) Merge the apply-side measure when its session reports, and add it to the
   page.
3. (#10) One worked check: our margins on a textbook grasp whose answer is
   published.
4. (#11) Scalar indices as extra report columns (hours).

**Where the narrative overclaims.** "Classic, established methods" is fair
for the tests, and the talk must say so and cite them (literature review,
"What not to claim"). What is ours is scoring a hand frame by frame against
one exact wrench per frame, set by a leader motion. The margins are related
forms of published measures, not the published ones: say "in the spirit
of" and cite.

## Step 4. Towards a dynamic assessment

**What the workshop shows today.** The same static check, at every frame of
a moving gait. That is a quasi-static sweep, not a dynamic assessment.

**What exists.**

- Leaders carry time: each frame has `t`, `obj_pos` and `obj_quat`
  (`data/leaders/*.json`, written by `tools/record_leader.py`).
- The leaders' contact forces come from a dynamic MuJoCo run
  (`force_source` in each leader's `source`), so the object's weight and
  acceleration are already inside the load each frame asks for.
- `RESID_TOL` in `follow.py` lets an amber frame leave a share of the
  object load unbalanced, described in the code as "a slight acceleration of
  the object". That is a tolerance, not a model.

**What is missing for "dynamic" to be true.** In plain terms, the hand
itself has to move and be checked as moving:

1. **Continuity.** Each frame's pose is solved afresh from fixed starting
   poses (`FingerIK.best`), so a finger's joint path can jump between
   frames. A dynamic check must carry the pose from one frame to the next
   and fail a frame the hand cannot get to in time.
2. **Joint speed.** From the frame-to-frame joint path and `t`, joint
   velocities, checked against each hand's speed limits.
3. **Hand inertia and weight.** The torque each joint needs is the contact
   load plus M(q)q̈ + C(q,q̇)q̇ + g(q). Today the hand's own weight is not
   counted (`follow.py` docstring).
4. **Torque at speed.** Motor torque falls with speed. `hands.py` holds one
   datasheet torque each for Allegro and LEAP; a dynamic check needs the
   torque-speed curve.
5. **Making and breaking contact.** The leader's `contact` flags mark when a
   station touches. At each touch, the impact the finger takes and whether
   the grip survives it.
6. **Checking against a simulation.** Replay a hand's solved joint path in
   MuJoCo and see whether the object follows the leader. This is the test
   the static check cannot pass or fail.

**Next tasks.**

1. (#12) Continuity and joint speed first (items 1 and 2): cheap, uses data we
   have, and shows whether the static green frames are reachable in time.
2. (#13) Then hand inertia and weight (item 3), from the MuJoCo models.
3. (#14) The replay in MuJoCo (item 6) as the step 4 demonstration.
4. (#15) Rename the step in the talk until then: "along a manipulation,
   quasi-statically".

**Where the narrative overclaims.** Step 4 as written. Until item 1 and at
least one of items 2 to 6 land, the workshop should not call any result
dynamic.

## Step 5 (future). Soft: why harder, what we can do, what we need

What exists to build on: passive stiffness in the ring demos
(`demos/passive.py`, `readout.soft_hold_passive`), joint stiffness in
`follow.py` (`joint_stiffness`, as a bound rather than a solved
deflection), and the SynGrasp comparison (literature review, ranked item 3:
solve for the deflection; add contact stiffness).

Points the step must make, from what the code already shows: a soft finger
deforms under the load it carries, so its contact moves; the record gives
one push direction per input, not a force set; and its forces are
uncalibrated. No tasks until steps 1 to 4 are settled.

## Step 6 (future). When and why this is useful

What exists: the record form and checker, so anyone can bring an actuator;
the literature review's novelty claims. Missing: a statement of the
question a designer would bring to this tool and a worked case answering
it. No tasks yet.

## Rules this plan follows

- Comparative, not predictive: simulated magnitudes compare designs; they
  do not predict a real device.
- Never overwrite stored outputs: re-runs get new names.
- No hand-typed numbers in docs.
- Hand-model files under `external/` are git-ignored and fetched by
  `tools/fetch_hands.py`.
