# Design: record-built fingers in the force check (issue 5)

Decided 27 Oct 2026: each actuator record is turned into a small jointed
stand-in finger, fitted to the record, written out as a MuJoCo hand file,
and run through `gramian_workshop/follow.py` like any other hand. A fit
check compares the stand-in with the record's free path and blocked force.
This note is the design; no code is written yet.

Why this route: the record stays the only way in, and a hand is the only
thing the force check reads. A soft hand is then just another hand, and
reach, contact along the finger body, the penetration check, placement,
both margins and inertia all work without a second code path. Modelling
soft fingers as rigid links with joint springs is standard (the
pseudo-rigid-body model; constant-curvature models), so the choice can be
cited.

## What the stand-in is

**Bending records** (most of the map: pneunets, bellows benders, the cable
finger, Mosadegh):

- A chain of N hinges in the record's bending plane (canonical X-Y), N = 3
  to start. Link lengths split the record's rest length; for simulated
  records the backbone at rest (`shape.spine_rest_m`) sets the split, and
  for records without it (Mosadegh) the links are equal.
- Every hinge has a spring: stiffness k_j about a rest angle of zero.
- One input drives all hinges together through one tendon with fixed
  moment arms g_j, pulling with force T(u), 0 <= u <= the record's input
  range. Bending one way only, as the record does.
- No coupling constraints: the hinges are free to deflect under contact
  load, so the finger is compliant across its whole body, not just at
  the tip.

**Axial records** (extenders and contractors: OSSA, bellows extenders,
McKibben): one slide joint along canonical X with spring k and drive force
T(u), pushing or pulling as the record's polarity says. Off-axis
compliance is left out at first and flagged.

**Records with neither** (twist-dominant, or no force data): no stand-in;
the tool says so.

## How it is fitted

The free equilibrium of the chain is k_j q_j = g_j T(u): each hinge turns
by g_j T(u) / k_j. So:

1. **Free path fixes the ratios g_j / k_j.** Fit the chain's tip path to
   the record's `free.tip_displacement_m` against input. With N = 3 there
   are three ratios plus the link split; the free path alone fixes the
   shape of the motion.
2. **Blocked force fixes T(u).** Hold the tip at its rest position, let
   the remaining internal freedom settle, and set T(u) so the tip reaction
   matches the record's blocked force magnitude at each blocked level.
   Between levels, interpolate as the record does.
3. **Then everything else is predicted, not fitted:** the blocked force
   direction, and the tip stiffness at each input level.

Contact radius (the capsule round each link) is not in the record schema.
It goes in the hand file (below), as part of how a hand is built. This
needs no schema change.

## The fit check

Reported for every stand-in, and stored beside the generated hand:

| Check | Compared against | Pass rule |
|---|---|---|
| Free path | `free.tip_displacement_m`, every level | worst tip error as a share of free stroke, within a tolerance |
| Blocked force size | `blocked` magnitude, every level | fitted, so it should match; a miss means the fit failed |
| Blocked force direction | `blocked.force_N` where present | angle within a tolerance; predicted, so this is a real test |
| Tip stiffness | `stiffness`, where present | reported, not gated at first |

A stand-in that fails is still written, flagged, and its verdicts carry
the flag. Records with few levels (Mosadegh has five) will give coarse
fits, and the check shows that rather than hiding it.

**Tolerances (ruled 27 Oct 2026):** free path within 5% of stroke, force
direction within 10 degrees. These measure the stand-in against the
record, not against the physics: the settle audit in the M-11 project
found simulated records under-state stroke by 10 to 82% and blocked force
by 10 to 24%. The fit report says so.

Blocked force size is reported, not gated (ruled 27 Oct 2026, for now).

## The hand file

A plain file that builds a hand from records:

- palm: shape and size;
- per finger: record id, mount position and orientation on the palm,
  contact radius, and the input range to use (default: the record's own).

A build tool reads it, fits each record, writes the MuJoCo model
(`data/hands/<name>/hand.xml`) and the fit-check report beside it, and the
hand is registered in `gramian_workshop/hands.py` as a local hand. A
drag-and-drop page can come later and write the same file.

## What changes in follow.py

One change, general rather than soft-specific: **joint springs count.**
The model's joint stiffness pushes back by k_j (q_j - rest) at every pose
the inverse kinematics finds. That is a pose-dependent torque, added to
the joint balance the same way finger inertia already is (the `tau` term
in `_joint_rhs`; the hold is C x = Tr f + tau). With both inertia and
springs on, the two add. Rigid hands have no joint springs in their models,
so their results do not change.

The drive needs no change: a tendon with a one-sided force range is an
actuator MuJoCo already reports, and `torque_limits` reads it through the
actuator moment as it does for Shadow.

This edit waits until the apply-side wrench work in `follow.py` is on
`main`, so two sessions do not edit the file at once.

## Assumptions, stated

- The drive force T(u) does not depend on pose. A real pneumatic finger
  pushes differently part-way through its stroke; the stand-in does not.
- The fit matches the record at the tip. Forces and deflection along the
  body come from the stand-in, not from the record.
- Bending stays in one plane; out-of-plane compliance is left out at first.
- Simulated records are in low-pressure units and their forces are
  comparative. Until the units question is settled (see
  `docs/MINIMAL_PRODUCT_PLAN.md`), strength verdicts for record-built hands
  against leaders in physical newtons are not meaningful; reach and contact
  verdicts are.

## Checks before results are shown

1. The fit check passes for the three example records: `wall_0.0010m`
   (simulated mesh), `mosadegh2014_fPN` (published data) and
   `shadow_hand_finger` (analytical). The workshop copy does not yet hold
   `mosadegh2014_fPN`; it is copied in with `tools/import_records.py`
   first, as the minimal product plan says.
2. Copies of one stand-in on the cup ring agree with the existing cup-ring
   demos on push direction and strength at the rest pose.
3. Every rigid-hand result in `outputs/` re-runs unchanged with the spring
   term in place.
4. `python -m gramian_workshop.gates` passes.

## Order of work

1. The fitter and fit check, on the three example records (new module; no
   existing file touched).
2. The hand file and the build tool; one hand of three stand-ins.
3. The spring term in `follow.py`, after the apply-side wrench work lands.
4. Checks 2 to 4 above, then the record-built hand on the cup tasks.

## Amended 27 Oct 2026, after reading the records

Reading the three example records changed four parts of the design above.
Where this section and the text above disagree, this section wins.

**The pneunet extends as well as bends.** `wall_0.0010m`'s blocked force
points almost entirely along the finger (canonical X); its sideways part
is a few per cent. Its free path also moves the tip forward at low
pressure, which bending alone cannot do (bending pulls the tip back). A
chain of hinges held straight at the tip pushes only sideways, so it would
miss the blocked-force direction by close to 90 degrees. The stand-in is
therefore built from hinges, a slide along X, or both, chosen by what the
record's free path and blocked force show. "Bending records" and "axial
records" above become one case.

**What the fit actually fits.** The free path is motion only: it fixes how
far each joint moves at each input, in ratios fixed across the input, and
says nothing about stiffness. Because the drive does not depend on pose
(stated below), one drive curve serves both the free and blocked cases.
So:

1. The free path fixes the joint motions against input, and through the
   springs, the drive.
2. The springs are the only parameters left. They are set from the
   record's `stiffness` block where it exists; otherwise fitted to the
   blocked force size.
3. The blocked force, both size and direction, is then a prediction: the
   tip is pinned at rest, the joints left free settle under drive and
   springs, and the reaction is read off.

One spring for all hinges (plus one for the slide) to start; one per
hinge only when the record has the data to pin them. The fit report
counts the numbers each record supplies against the parameters fitted.

**Records with nothing to fit get no stand-in.** `shadow_hand_finger`
holds no free path, blocked force or stiffness. The fitter refuses it and
says why. The analytical route therefore cannot yet reach the force check;
that needs a ruling (below).

**Mosadegh needs three allowances.** Its tip moves in −Y, against the +Y
convention; hinge ratios are signed and the record is never rotated. Its
length is not recorded; the rest length is fitted from the path and
reported as fitted. Its blocked force is a size only, on an axis the
source does not state, so the direction check is reported as not
possible. With five points curling most of the way round, three hinges
may not be enough; the number of hinges is a setting, and the report
gives the error against it.

**Extrapolation is flagged.** `wall_0.0010m`'s blocked force stops at
40 Pa while its free path runs to 398 Pa. Force above the last blocked
level is extrapolated and the report says from where.

**New open ruling: the analytical route.** A record from the analytical
route, such as `shadow_hand_finger`, carries kinematics but no force
curves. Either the route must also emit a free path and blocked force (or
joint stiffness and drive limits), or analytical fingers enter the force
check as model-file hands instead of through the stand-in.

## The spring term, 28 Oct 2026 (issue 21)

`follow.py` now reads each finger joint's spring from the hand model:
MuJoCo's joint stiffness and its rest position (`springref`), as
`tools/build_record_hand.py` writes them. At the pose the inverse
kinematics finds, the spring pushes back by k (q - rest), and the finger's
inputs must supply that torque on top of the contact load. It enters the
joint balance where the finger's motion torque does, and the two add when
both are on.

What we chose, and why:

- **No switch.** The springs are read whenever the model has them. A hand
  whose finger joints have no stiffness takes exactly the old path and its
  results carry no new keys. None of our rigid hands has joint or tendon
  stiffness in its model.
- **`joint_stiffness=` is a separate term.** It still adds the uniform
  deflection allowance (a spring that lets the contact move a little
  from the solved pose). The model's springs give the torque at that pose.
  The two add. Whether a hand's own springs should set that allowance is
  open (below).
- **The apply limit counts inputs only.** A spring is never an input. At
  the solved pose its torque is a load the inputs must hold before they
  can push, so it shifts the apply limit: less push one way, more the
  other.
- **Record-built hands carry a caveat.** `hands.py` gives `wall_x3_ring`
  one, and every result for it repeats it: the motor force is the
  record's unitless drive level, so strength verdicts are provisional
  until the units ruling.

Checks:

- Rigid hands: every stored rigid-hand result was re-evaluated with the
  new code and with the code before the change, at the stored placement.
  The two agree exactly. Neither reproduces the stored files, which were
  made by earlier versions of the force check; that is not from this
  change.
- The model's springs balance the drive on the free path and leave an
  unbalanced torque off it (`tests/test_follow_springs.py`).
- Copies on the cup ring against the demos: `tools/record_hand_vs_demo.py`,
  output `outputs/record_hand_vs_demo__wall_x3_ring.json`. The stand-in's
  push at rest matches the fit report's prediction in direction and size.
  It differs from the record's push by the fit check's direction error, and
  is larger by the fit report's size ratio. The demos' readout fails the
  record's own push on direction for three copies on the palm mount, but
  passes it once its small out-of-plane part is dropped. That part makes
  every copy twist the ball the same way about the vertical, and pushes
  are one-sided, so nothing cancels it. The planar stand-in cannot push out
  of the plane, so it passes.
- The hand on the cup leaders:
  `outputs/follow_wall_x3_ring__w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4__provisional.json`
  and the same for `_n5`. Every checked frame is red. Frames fail on
  direction or friction where too few stations are touched, and on torque
  where the force check is reached. Turning the springs off gives the same
  frame verdicts: with one drive for four joints, the redistribution's
  torque test fails either way.

Open, for a ruling:

- **The tolerance mixes units.** The per-finger test allows UNACT_TOL of
  the whole joint torque. With a slide, that norm adds newtons (the slide)
  to newton-metres (the hinges). The slide's spring force is far larger in
  number, so the hinge rows are hardly tested. The redistribution instead
  holds the joint rows to a share of the object's load, so it is much
  stricter. The two disagree for this hand.
- **The pose ignores the springs.** The inverse kinematics chooses the pose
  from geometry alone. A finger with one drive and several sprung joints
  can only sit where the drive balances the springs and the contact load.
  Most poses the inverse kinematics finds are off that set, so the torque
  test fails there.
- **Which leaders are the cup tasks.** No leader file is called `cup_hold`
  or `cup_turn`. We ran the two tennis-size cup leaders, the ones the rigid
  hands use, and the demos' `cup_hold` task through the check above. No
  static hold leader exists.
- **Whether the hand's own springs should set the deflection allowance**,
  in place of the uniform `joint_stiffness=`.

## The fingertip of a record-built finger, 28 Oct 2026 (issue 16)

Ruled on issue 16: for a hand built from records, the fingertip is the
record's own tip point, the end of the fitted chain. Hands from model files
keep their rule (the point of the last link's collision shape farthest from
its joint). `hands.py` tells the two apart by the build tool's
`fit_report.json`: a hand is record-built when that report sits beside its
model and names it. We chose the report over a flag in the hand list
because the build tool always writes it, so a new record-built hand cannot
be registered without it. The tip is then one link length (the record's
length over the hinge count) along the distal link.

For `wall_x3_ring` the fingertip moves from 11.2 mm past the chain's end
(a capsule radius along the finger and a corner off its axis) to the
chain's end itself. The build tool's round trip now reports that offset
(0) as `loader_tip_offset_m`, and the old rule's as
`model_file_tip_rule_offset_m`. The stored `fit_report.json` was written
before this change and still carries the old figure; it is not rebuilt.
In `follow.py` the fingertip sets which way a distal link's pad faces and
where the placement search starts; contact itself is the link's capsule.

## Poses for fingers with fewer drives than joints, 28 Oct 2026 (issue 21)

Ruled on issue 21: a finger with one drive and four sprung joints cannot be
posed freely. Its shape is set by the drive level, and contact only
deflects it against the springs. Until now `follow.py` chose the pose from
geometry alone, so `wall_x3_ring` landed in shapes it cannot hold and every
frame was red; those verdicts were an artefact of the pose choice, not
verdicts on the finger.

What `follow.py` does now, for a finger whose free joints all have springs
in the model and whose inputs are fewer than those joints:

- **The pose is an equilibrium.** For a drive level u and a contact force f
  on the object at the touch, the joints sit where the spring torque
  equals the drive torque less the object's push carried back to the
  joints: K (q - rest) = M^T u - J^T f.
- **The search is over u and f, not over joint angles.** u stays in the
  drive's range and f in the friction cone at the touch. The search scores
  the touch exactly as the geometric pose search does (reach, pad side,
  overlap beyond the task's allowance), and keeps f near the leader's force
  at that contact: departing from it by its own size costs as much as
  missing the contact by the amber reach (10 mm). On ten frames of the n4
  cup leader, any weight from a hundredth to ten times that gave the same
  reach; much weaker, and the forces wander far from the leader's for no
  gain in reach.
- **The touching link must sit on the surface.** A contact force needs a
  contact, so the search weighs the gap between link and surface as it
  weighs the balance, and a link left standing clear of the surface counts
  that gap in its reach: it does not touch there. A first run without this
  left the fingers a median 5 mm clear of the ball, pushing on nothing,
  and the reach (measured round the surface only) did not show it.
- **The pose returned is solved exactly** from the u and f found (Newton on
  the finger's energy, so it is a stable equilibrium), and it is that pose
  the touch and the force tests are scored at.
- **Joint ranges are not imposed on it.** The build tool writes each range
  as the free motion only, and leaves how far contact pushes a joint past
  it to the springs. A hinge stays within plus or minus 180 degrees.
- **The force tests are unchanged.** At the pose, the leader's force passes
  the per-finger test only if it is (near) a force that holds the finger
  there; the redistribution can use the force found, which does.

Fingers with as many inputs as joints can hold any pose, and fingers
without springs have no equilibrium to solve for, so both keep the
geometric pose. That covers every rigid hand here, Shadow's
tendon-coupled fingers included; their results are unchanged (checked
against the code before this change on Allegro, Shadow, L20 with
`joint_stiffness`, a human-finger hand with inertia, the margins, the
apply limit and a rate limit, and one fresh placement search).

Each finger posed this way records `equilibrium`: the drive (in the
record's unitless level), its share of the range, the contact force, how
far that force is from the leader's, and the balance left. The result's
settings say which fingers were posed this way.

## Each joint type in its own unit, 28 Oct 2026 (issue 21)

Ruled on issue 21: the per-finger torque test must check each joint
against its own limit, slide force against the slide's and hinge torque
against the hinges', never summed across joint types. Before, it allowed
5% (UNACT_TOL) of the norm of all the finger's joint torques, which for
this hand added the slide's newtons to the hinges' newton-metres; the
slide's spring force is far larger in number, so the hinge rows were
hardly tested.

For a finger with both hinges and slides, the test now allows the hinges
5% of the hinges' torque and the slides 5% of the slides' force, and
finds the drive use with each type's rows weighed by its own allowance.
The finger's record gives `unbalanced_Nm` for the hinges and
`unbalanced_N` for the slides. Fingers with one joint type (every rigid
hand) keep the test exactly as it was.

We read "its own limit" as each joint type's own allowance, a share of
the torque (or force) asked of that type, rather than a share of what the
drive could make at each joint: the second would loosen the test for
every strong rigid hand and change all their verdicts, which the ruling
did not ask for. The redistribution and the margins still hold all joint
rows to one tolerance on the object's load, slide rows included; that is
not changed here.
