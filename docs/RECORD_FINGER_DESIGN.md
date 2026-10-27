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
