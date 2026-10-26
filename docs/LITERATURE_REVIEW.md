# Literature review: where the Gramian Map sits

Written 23 October 2026 for the ARIA workshop. This was a reading and
writing task: no code was changed. Updated the same day after reading
Bicchi and Prattichizzo 2000 and 1998, and after the scale margin was added
to `follow.py`. Updated on 25 October when the disturbance margin was added, and on
26 October when the apply limit was added.

## The result in brief

- **What we reinvent.** The core of our rigid-hand check is standard grasp
  analysis. The grasp matrix, the hand Jacobian, the pyramid friction cone
  and the linear-programme feasibility test are all in the Springer Handbook
  chapter by Prattichizzo and Trinkle. Our rule that "unactuated torque must
  be zero" is the statics of Bicchi and Prattichizzo 2000 (eq. 15), which we
  have now read; the handbook's force-closure definition states a related
  condition on the hand's null space. We should cite these, not present
  them as ours.
- **What we can borrow.** SynGrasp (Siena) holds the full compliance model:
  joint, contact and synergy springs. We have only joint stiffness, as a
  bound on held torque. Our zero-torque rule and our rigid
  coupling of slaved joints are, by our reading of its equations, the two
  limits of that model. The Valero-Cuevas
  index-finger model gives a human-finger force set from published numbers,
  but at one posture only.
- **What is new.** We found no work that does any of these three things:
  - scores a hand frame by frame against one exact wrench per frame, set by a
    leader motion;
  - builds an empirical controllability Gramian of tip motion from
    finite-element simulation to characterise soft actuators;
  - keeps shared per-actuator records of stroke, force and stiffness against
    input.
  The nearest records work is Wang et al. 2026, which we must cite.
- **What not to claim.** That we invented grasp feasibility or the friction
  pyramid. That our human-finger numbers are measurements. That the 1998
  finger model is reproduced (our rebuild is off in one of five directions,
  by half).

The ranked list of what to port first is in section 7. References we could
not verify are in section 8.

## How each claim is marked

Every citation below with a DOI was checked against its Crossref record
(title, authors, year, venue) and its DOI resolved. Where Crossref gives no
year, we say so and take the year from the proceedings name. Where a source has no DOI, the
URL was fetched. We mark how far each source was read:

- **[read]**: we read the full text, or the sections cited.
- **[abstract]**: we read the metadata and abstract only. Content beyond the
  abstract comes from another source, which is named.
- **[UNVERIFIED]**: we could not check this claim. It must not be cited as
  fact until someone checks it.

Effort estimates use one coarse scale: **hours** (under a day), **days**
(one to five days), **weeks** (more than a week).

## 1. Our tool, in the terms the literature uses

This summary is from the code on `main`, so the reader can match each
source to a line. It was first written against the strength model before
commit fbf16fd. It was updated on 23 October, after that commit, and again
on 24 October when the scale margin was added, and on 25 October when the
disturbance margin was added, and on 26 October when the apply limit was
added; the line numbers below are for that version.

- **Contact model.** Hard finger, point contact with friction. The contact
  Jacobian is `Jf = jacp[:, f.dof]` from MuJoCo's `mj_jac` at the pad point
  ([follow.py:577](../gramian_workshop/follow.py)).
- **Friction cone.** Per finger, the leader's force must lie inside the cone:
  `fv @ n_in >= cos(atan(mu))*|fv|` (follow.py:579). For redistribution, the
  cone is an inscribed pyramid of `CONE_EDGES = 8` edges
  ([capability_core.py:17](../gramian_workshop/vendor/capability_core.py),
  `generators` at line 89). `OUTER_FACTOR = 1/cos(π/8)` (line 20) gives the
  circumscribed pyramid as an outer bracket
  ([readout.py:53, 96, 234](../gramian_workshop/readout.py)).
- **Transmission and strength.** `_finger_columns` (follow.py:642–673)
  folds slaved joints into their masters (the matrix E, with each slaved
  entry the derivative of the polynomial coupling). A contact force f needs
  the reduced joint torque Tr f, with Tr = E^T Jf^T. The finger can supply
  C x with every x between 0 and 1. C has one column per input sense: one
  for a muscle, which only pulls, and two for a motor with a two-sided
  range. With a joint stiffness kq above zero, each free joint adds two
  passive columns, sized so the contact moves at most `DEFLECT_MAX_M`. The
  default kq is 0, so by default torque that no input can make must be
  zero.
- **Torque check.** `_columns_hold` (follow.py:676–685) solves for x by
  non-negative least squares. The unbalanced torque must be at most
  `UNACT_TOL` (0.05) of |Tr f|.
- **Redistribution.** `_redistribute` (follow.py:701–725) asks whether the
  contacts we made can produce the leader's object wrench with forces of
  their own choosing. It runs three tests in turn: span (least squares, line
  714), friction (non-negative least squares on the pyramid, line 718), then
  torque. The torque system is built by `_torque_stack` (follow.py:728–755).
  It stacks the wrench balance, each finger's joint balance Tr F λ = C x,
  and x plus a slack equal to 1. The wrench may be missed by `RESID_TOL`
  (0.05) of its norm.
- **Scale margin.** `_scale_margin` (follow.py:758–835) gives each frame the
  largest factor s by which the leader's object wrench could grow and still
  pass the torque test of `_redistribute`. It uses the same system, with
  s times the wrench on the right-hand side, and a tolerance that grows
  with s. Above s = 1 the joint-balance and input-bound rows keep the
  slack they have at s = 1: every row is divided by s and the contact
  forces are taken per unit s, which is the same as weighting those rows
  by s but better conditioned. Without this, a 5 % slack on a large wrench
  let the inputs run several times past their limits, and every passing
  frame read as capped (our check on the stored outputs, 23 October). The
  search starts as the vendored W20 one: every point of `cap.SCALE_GRID`
  (0 to 16 in steps of 0.25), then bisection to `cap.BISECT_TOL` (1e-4).
  W20's bar is exact, so it needs neither change. Where 16 still holds, we
  double the scale up to `MARGIN_MAX` (1024), because the cup leaders
  clear 16, and bisect only to `MARGIN_REL` (1 %) of the margin, because
  each failed solve there is slow. Even so the margin makes a run several
  times slower, so `follow` leaves it off unless asked (`margin=True`), as
  `readout.soft_hold` does (`want_margin=False`). It reports
  `capped` when `MARGIN_MAX` still holds, and `contiguous` when the grid
  points that hold form one run. The margin is 0 when the span or friction
  test fails, since neither depends on scale. With the default kq = 0, a
  frame whose unactuated joints cannot reach zero torque fails at every
  scale, so it also reads 0: the margin grades only frames that pass that
  rule. It is absent when no finger has its torque checked.
- **Disturbance margin.** `_disturbance_margin` (follow.py:838–912) gives
  each frame the smallest extra wrench, added on top of the leader's, that
  the made contacts could no longer produce along with it. In plain terms:
  how hard a push or twist from outside the grip, in the worst direction,
  before the motors or friction give out. It is the worst-direction
  reading of the largest-ball measure (Ferrari and Canny 1992, via Roa and
  Suárez 2015), limited by torque as our test is. It uses the
  `_torque_stack` system unchanged, with the extra wrench added to the
  right-hand side and the tolerance of `_redistribute` held fixed, so at
  zero disturbance it is exactly the torque test. The wrenches that system
  can produce form a convex set, so along each direction the disturbances
  that hold form one run from zero, and bisection is sound. We try the 12
  signed axes of the scaled wrench (push along x, y and z; twist about x,
  y and z: `DISTURB_DIRS`, proposed), so the result is an upper bound on
  the true smallest disturbance. Each direction is bisected to
  `MARGIN_REL` of the margin (floor `cap.BISECT_TOL`), and a direction
  that still holds at the smallest margin so far is skipped. The margin is
  a share of the leader's wrench norm, which is also reported (torque
  divided by L, as W20), with the worst direction. Its shape depends on
  that L scaling. It is 0 when the leader's own load fails, and absent
  when no finger has its torque checked (without input limits the cones
  are unbounded). It runs continuously from 0: a frame that only just
  passes at the leader's load, with close to `RESID_TOL` of it unbalanced,
  reads near 0. It is off unless asked (`disturbance=True`) and changes no
  verdict.
- **Apply limit.** `_apply_limit` (follow.py:915–996) gives each frame the
  largest push and twist the made contacts could put on the object by
  actuation alone, with no leader load, along each of the 12 signed axes of
  `DISTURB_DIRS`. In plain terms: how hard the hand could drive the object
  itself, rather than hold it against the leader. It is the torque-limited
  wrench set (Jeong and Cheong 2012; Zheng and Yamane 2013; both known to
  us only through Roa and Suárez 2015, section 8), read along those axes.
  It uses the `_torque_stack` system with right-hand side [e d, 0, 1] and
  the input columns only: a passive spring (kq above zero) resists a load
  but cannot push. As in `_scale_margin` above s = 1, every row is divided
  by e and the cone weights are taken per unit e, with the tolerance fixed
  at `RESID_TOL`. So the wrench may be missed by that share of e, and the
  inputs cannot run past their limits as e grows. A tolerance on the
  leader's wrench norm, as the other margins use, would tie a leader-free
  quantity to the leader. At e = 0 the test is the friction span alone.
  Feasible e form one run from 0, so each direction tries
  `APPLY_FIRST_N` (proposed) first, then doubles up to `MARGIN_MAX` or
  bisects below it, to `MARGIN_REL` of the value (floor
  `cap.BISECT_TOL`). Pushes are in N and twists in N·m (the scaled value
  times L). Directions that still hold at `MARGIN_MAX` are listed as
  capped. A finger whose unactuated joints would carry torque does not
  fail here, as it does under the leader's load: it supplies only forces
  that load no such joint. Dividing by the object's mass and inertia would
  give the largest acceleration the hand can give it, to compare with the
  leader's motion; that step is not built. It is off unless asked
  (`apply=True`) and changes no verdict. On the 13 stored cup runs (our
  check, 26 October) no direction reads capped, no twist is ever open,
  and only three of the six pushes ever are. In the frames we examined,
  every made contact sits on the lower half of the cup, so with no
  leader load the cones alone cannot push or twist along most axes: these
  grips hold the cup against the leader's load but cannot drive it.
- **Task.** One exact wrench per frame, given by the leader motion. There is
  no task set and no worst case over a set. Each frame is green, amber or
  red, and, when asked (`margin=True`, `disturbance=True`, `apply=True`),
  carries its scale margin, its disturbance margin and its apply limit.
- **Search.** Cross-entropy placement (`placement_search`, line 355) and a
  permutation assignment of fingers to stations (`best_assignment`, line
  408).
- **Constants.** follow.py:83–117. They include `REACH_GREEN_M` 2 mm,
  `REACH_AMBER_M` 10 mm, `FACE_GREEN_DEG` 30, `FACE_AMBER_DEG` 60,
  `CONTACT_CONE_DEG` 85 (line 87), `IK_ITERS` 60 (line 91), `RESID_TOL`
  (line 96), `RESPACE_DEG` 20 (line 100), `UNACT_TOL` (line 101),
  `DEFLECT_MAX_M` 2 mm (line 102), `JOINT_STIFFNESS` 0 (line 103),
  `FACE_WEIGHT_M` 0.02 m per radian (line 104), the disturbance
  directions `DISTURB_DIRS` (line 109) and the apply limit's first try
  `APPLY_FIRST_N` (line 115).
- **Compliance.** Passive joint stiffness is in `_finger_columns`, as
  above. Passive tip stiffness also enters `readout.soft_hold_passive`
  (readout.py:197), used by `demos/passive.py`, as a spring force within a
  deflection budget.
- **Gramians** (in the Gramain_extraction repo, `gramian_map/jacobian.py`,
  not this one): W_c = Σ_k |dp_k| J_k J_k^T and
  W_o = Σ_k |dp_k| (y_k − y0)(y_k − y0)^T, over the input steps dp_k of one
  finite-element sweep.

## 2. Q1: SynGrasp

### What it is

**Citations.**
- Malvezzi M, Gioioso G, Salvietti G, Prattichizzo D, Bicchi A. "SynGrasp: A
  MATLAB toolbox for grasp analysis of human and robotic hands". *2013 IEEE
  ICRA*, pp. 1088–1093. DOI 10.1109/ICRA.2013.6630708. **[metadata only: full
  text closed, not read]**
- Malvezzi M, Gioioso G, Salvietti G, Prattichizzo D. "SynGrasp: A MATLAB
  Toolbox for Underactuated and Compliant Hands". *IEEE Robotics & Automation
  Magazine* 22(4):52–68, 2015. DOI 10.1109/MRA.2015.2408772. Bicchi is not an
  author of this one. **[metadata only: the open copy returned 403/410, not
  read]**
- Prattichizzo D, Malvezzi M, Gabiccini M, Bicchi A. "On the manipulability
  ellipsoids of underactuated robotic hands with compliance". *Robotics and
  Autonomous Systems* 60(3):337–346, 2012. DOI 10.1016/j.robot.2011.07.014.
  **[read, from the archived preprint]**
- Prattichizzo D, Malvezzi M, Gabiccini M, Bicchi A. "On Motion and Force
  Controllability of Precision Grasps with Hands Actuated by Soft Synergies".
  *IEEE Transactions on Robotics* 29(6):1440–1456, 2013. DOI
  10.1109/TRO.2013.2273849. **[read, from the archived preprint]**
- Malvezzi M, Prattichizzo D. "Evaluation of grasp stiffness in underactuated
  compliant hands". *2013 IEEE ICRA*, pp. 2074–2079. DOI
  10.1109/icra.2013.6630855. **[metadata only]**
- Toolbox site: https://sites.google.com/unisi.it/syngrasp. Download folder
  (version 2.6, dated 20 August 2024):
  https://drive.google.com/drive/folders/1qr2IEWoIYO2nCGLM5ICFuuJZa-dK6p3W.

**What it computes.** SynGrasp models a hand and object as a linearised,
quasi-static system with three sets of springs: at the contacts (Ks), at the
joints (Kq) and on the synergy inputs (Kz). The synergies set only a
*reference* posture, qr = S z (T-RO 2013 eq 7), so the real joints may
deflect away from it. From this model it solves for the object motion,
contact forces and joint motions caused by a change in synergy input, and
derives the controllable internal forces, the rigid-body motions and the
redundant motions. It also gives manipulability ellipsoids and five scalar
grasp-quality indices. **[read: T-RO 2013 and the v2.2 source]**

**Equivalent in our tool, and verdict.** **Borrow the compliance model;
cite the toolbox; do not port it whole.** Effort: **days** for a Python
version of the quasi-static solution for one finger, which is what we would
use.

### SynGrasp against `follow.py`, part by part

The SynGrasp code below is from third-party copies of versions 2.2 and 2.3 on
GitHub. We did not download the official 2.6 zip, so the 2.6 source is
**[UNVERIFIED]**.

| SynGrasp | What it does | Our equivalent | Same or different |
|---|---|---|---|
| `SGgraspMatrix` (G from contact points and object centre) | Maps contact forces to object wrench, w = −Gλ (T-RO eq 2) | `capability_core.direction_basis` (line 76) and `generators` (line 89); torques divided by a length L | Same object. We scale torque by L. |
| `SGselectionMatrix`, `SGhMatrix` (contact type: hard finger, soft finger, full constraint) | Selects which wrench components a contact carries | Hard finger only, implicit in using the 3-row `jacp` | Same as their hard-finger type. We have no soft-finger type. |
| `SGjacobianMatrix`, then J = H J̃ | Hand Jacobian at the contacts | `Jf = jacp[:, f.dof]` from `mj_jac` (follow.py:577) | Same. MuJoCo gives it; they build it from Denavit–Hartenberg chains. |
| `SGdefineSynergies` (S, with qr = S z) | Inputs set a reference posture through S | E and the input columns C in `_finger_columns` (follow.py:642–673) | Different. Our E is a rigid coupling: q is exactly E z. Theirs is a reference that springs pull towards. |
| `SGjointStiffness` (Kq), `SGcontactStiffness` (Ks), `SGsynergyStiffness` (Kz) | Springs at joints, contacts and inputs | Joint stiffness kq as passive columns in `_finger_columns`, within a deflection budget (default kq 0). Tip stiffness in `readout.soft_hold_passive` | Partly. Our kq bounds the torque a joint can hold; it does not solve for the deflection. We have no Ks or Kz. |
| `SGquasistatic` (X = inv(Uq+Kq) Kq, and so on) | Solves the linearised system for a change in input | None. We test one static load per frame | Different question: they ask "what happens", we ask "can it hold". |
| `SGgraspAnalysis`: E = orth(A B1), V = null(J), N = null(J') | Active internal forces, internal motions, structural forces | The joint balance Tr F λ = C x in `_torque_stack` (follow.py:728–755) | Related. Their N = null(J^T) is the handbook's "defective" class; ours is the torque that the input columns cannot reach. |
| `SGcheckFriction`, `SGVcost` (barrier cost on the cone) | Checks friction, hard finger only | Per-finger cone test (follow.py:579) and the pyramid NNLS in `_redistribute` | Same test. We use an 8-edge pyramid with an outer bracket; they use the circular cone. |
| `SGminSVG`, `SGgraspIsotropyIndex`, `SGdistSingularConfiguration`, `SGmanipEllipsoidVolume`, `SGunifTransf` | Scalar quality indices from G and from H_O = (G^+)^T J | None. We give no scalar index | We could report these as extras, but they score a grasp, not a task. |
| `SGkinManipulability`, `SGforceManipulability` | Ellipsoids under compliant synergies (RAS 2012 section 4) | None | Different: they describe a set; we test one wrench. |

One possible sign question, unresolved: in v2.2 the Kz row of the block
matrix in `SGquasistaticHsolution` reads `[… eye(nz) … Kz Kz]`, where the
spring law Kz(zr − z) would suggest opposite signs. We did not determine
whether this is intended. **[UNVERIFIED]**

### Licence, Python port, and effort

- **Licence.** No licence is named on the site or in the download folder. The
  source headers of the third-party v2.2 and v2.3 copies carry "all rights
  reserved" over a BSD 3-Clause text whose "<organization>" placeholder was
  never filled in. The official 2.6 header is **[UNVERIFIED]**: the zip was
  not downloaded. Before copying any code, we should ask the Siena group.
  Re-implementing the published equations needs no permission.
- **Python port.** None found. We searched GitHub (repository and code
  search) and PyPI. The nearest Python tool, `Ric-27/Task-Oriented-Grasp-Analysis`,
  is a separate tool marked as CEA property and not for distribution. A C++
  copy and several MATLAB forks exist; none declares a licence.
- **Effort to port.** A full port (hand building, planner, plots) is
  **weeks**, and we do not need it. The part we need is the quasi-static
  solution for one finger with Kq and Ks: about forty lines of MATLAB in
  `SGquasistatic` and `SGquasistaticMaps`. That is **days**, including a
  check against a SynGrasp example run in MATLAB or Octave.

### Compliance against our "unactuated torque must be zero" rule

The T-RO 2013 paper says that joints with elastic elements should be treated
"as uncontrollable or passively driven instead of unactuated" (Introduction,
p. 1). In its model every joint has a spring. Torque that the inputs cannot
make is taken up by joint deflection through Kq; it is not forced to zero
(eqs 7–10). **[read]**

Our two rules are the two ends of that model:

- **Rigid limit.** With joint compliance Cq = 0, T-RO Remark 1 gives
  q = S z (eq 27). That is our E folding of slaved joints in
  `_finger_columns`: the coupling is exact, and torques across it are
  reaction loads.
- **Zero-stiffness limit.** Our requirement that torque no input column
  covers be zero (the joint balance in `_columns_hold` and `_torque_stack`,
  with kq = 0) is what holds when those directions have no stiffness at
  all. In SynGrasp terms, our input columns play the role of S.
- **Between the two.** A finite Kq lets the finger hold some unactuated
  torque by deflecting. The deflection must then stay within a budget. This
  is what `readout.soft_hold_passive` does for tip stiffness, and what we
  would need for real soft fingers.

This is our inference from the equations, not a statement in any source.
The zero-torque passive-joint model is attributed to Bicchi and Prattichizzo
2000 by the RAS and T-RO papers. We have now read that paper: its eq. 15
sets J_p^T f = 0 at passive joints (section 5).

Since commit fbf16fd, `follow.py` has the middle case in a simple form.
With a joint stiffness kq above zero, `_finger_columns` lets each free joint
hold a passive torque up to kq times the joint angle that moves the contact
by `DEFLECT_MAX_M`. This bounds the deflection rather than solving for it,
as SynGrasp's quasi-static model would. The default kq is 0, the
zero-stiffness limit.

A practical note: `SGgraspStiffness` inverts Kq, so it cannot take a Kq that
is zero in some directions. `SGquasistatic` uses X = inv(Uq+Kq) Kq, which can.
**[read: v2.2 source]**

### The video

"Lecture 4.1: Introduction to SynGrasp", SIRSLab Siena Robotics Systems Lab
channel, published 24 October 2018,
https://www.youtube.com/watch?v=13LSsPE6GtQ. We checked the title, channel
and date through YouTube's metadata. We did not watch it, so its content is
**[UNVERIFIED]**. Verdict: **cite** as the lab's own introduction, if useful
for the audience.

## 3. Q2: Li and Sastry's task ellipsoids

**Citations.**
- Li Z, Sastry SS. "Task-oriented optimal grasping by multifingered robot
  hands". *IEEE Journal of Robotics and Automation* 4(1):32–44, 1988. DOI
  10.1109/56.769. **[metadata only]**
- Same title, *Proc. 1987 IEEE ICRA*, vol. 4, pp. 389–394. DOI
  10.1109/robot.1987.1087852. Crossref gives no year; 1987 is from the
  proceedings name. **[metadata only]**
- Same title, Electronics Research Laboratory memo UCB/ERL M86/43, UC
  Berkeley, 30 May 1986.
  https://www2.eecs.berkeley.edu/Pubs/TechRpts/1986/ERL-86-43.pdf.
  **[read]**

We could not get the full text of the 1987 or 1988 papers. **All content and
every equation number below is from the 1986 memo.** The journal version may
number them differently.

**What it does.** The task is an ellipsoid of object wrenches, A_β, scaled by
β (memo eq 3.3-3). The grasp's capability is the image of a single 2-norm
ball of contact forces, intersected with quadratic Coulomb cones, through the
grasp matrix (eq 2.5-2). The quality is the largest β for which the task
ellipsoid fits inside that image (eq 3.3-4a), with a normalised variant
(eq 3.3-4b). Section 3.6 lets the ellipsoid vary along a trajectory.

**Against our approach.**

| | Li and Sastry (1986 memo) | Our tool |
|---|---|---|
| Task | An ellipsoid of wrenches, all of which must be resisted | One exact wrench per frame, from the leader |
| Force bound | One 2-norm ball over all contact forces together | Input limits per actuator, through B |
| Hand | No Jacobian and no joint torques; the hand enters only as reach and geometry limits (eqs 3.5-15, 3.5-16) | Full Jacobian, transmission and torque limits |
| Output | A scalar β | A per-frame verdict: green, amber or red |
| Over time | A time-varying ellipsoid (section 3.6) | One wrench per frame along the motion |

Their single-wrench test, w = G x with x in the cone (eq 3.5-13), is our
check reduced to one point. Our per-frame check is therefore their
feasibility condition applied to a task set of one wrench, with the hand's
torques added.

**The scale margin, now built.** With one wrench, their β becomes a scale
margin: the largest factor by which the leader's wrench could grow and still
be held. No source we read names this per-frame quantity; it is our
suggestion. The nearest published scalar is the margin d of the handbook's
LP2 (section 5, Prattichizzo and Trinkle). The nearest in Bicchi and
Prattichizzo 2000 is passive force manipulability (section 5). `follow.py`
reports it for every frame when asked, as `scale_margin` (section 1). It is a
search over scale on the existing torque test, not the handbook's LP2, and
it does not change any verdict. Unlike β it is not a worst case over a
task set, and with kq = 0 it reads 0 for every frame that fails the
zero-torque rule, so it grades only the frames that pass it.

**Verdict.** **Cite** as the origin of task-oriented grasp quality. The
scale margin is done. Its worst-direction companion, the disturbance
margin, is also built (section 1), as is the apply limit, which drops the
leader's load.

Related work, citation only:
- Li Z, Hsu P, Sastry S. "Grasping and coordinated manipulation by a
  multifingered robot hand". *IJRR* 8(4):33–50, 1989. DOI
  10.1177/027836498900800402. Checked on Crossref; we did not read it.
  It may bring in the hand Jacobian. **[metadata only]**

## 4. Q3: the Valero-Cuevas index finger

### The answers in brief

- **Do the published R, F0 and J give A A^T?** Yes, at one posture: neutral
  abduction, MCP 45°, PIP 45°, DIP 10°. The 1998 paper's Tables 1 and 2 give
  moment arms R, maximal forces F0 (from PCSA) and segment lengths for J. Our
  rebuild of A = J^-T R diag(F0) matches the paper's joint-torque directions
  (Table 6) but not all its maximal forces (Table 3). Details below.
- **Is measured data downloadable?** No. The measured maximal forces are
  five means with standard deviations, in the paper's Table 3. We found no
  data download for any of the papers. The lab's 2018 code repository holds a
  numeric matrix, not measurements.
- **Which tables we would use.** 1998 Table 1 (moment arms, extensor
  parameters, PCSA), Table 2 (lengths and dependent moment arms), Table 3
  (measured and model forces) and Table 6 (joint-torque directions, as a
  check). 2000 Table 1 (assumed maximal tendon forces) and the Fig. 3 inset
  (measured tendon actions).
- **Can a human-finger record be built from published numbers alone?** Yes,
  at that one posture, and only as a model. There are three independent
  routes to A, listed below. For more postures one would have to mix sources.

### Sources

- Valero-Cuevas FJ, Zajac FE, Burgar CG. "Large index-fingertip forces are
  produced by subject-independent patterns of muscle excitation". *Journal of
  Biomechanics* 31(8):693–703, 1998. DOI 10.1016/s0021-9290(98)00082-7.
  **[read: Tables 1–3 and 6]**
  - Eight subjects pushed maximally in five directions at one posture. The
    model is f = M e with M = J^-T R F0 (4×7: three force rows and one
    torque row). The feasible set is the polyhedron with distal-phalanx
    torque held at zero.
  - The "adjusted" parameters were fitted to the five measured forces, so
    checking the adjusted model against them is circular. Only the nominal
    model is an independent prediction.
  - Equivalent in our tool: a draft human-finger record. Verdict: **borrow**
    (done, see below).
- Valero-Cuevas FJ. "An integrative approach to the biomechanical function and
  neuromuscular control of the fingers". *J Biomech* 38(4):673–684, 2005.
  DOI 10.1016/j.jbiomech.2004.04.006. **[read]**
  - A review. Fig. 6 is captioned as the feasible force set of the index
    finger (p. 680), so "feasible force set" is the field's term for what our
    force-set proposal measures. No parameter tables.
  - Verdict: **cite** for the term.
- Valero-Cuevas FJ, Towles JD, Hentz VR. "Quantification of fingertip force
  reduction in the forefinger following simulated paralysis of extensor and
  intrinsic muscles". *J Biomech* 33(12):1601–1609, 2000. DOI
  10.1016/s0021-9290(00)00131-7. **[read: Table 1 and the Fig. 3 inset]**
  - Eleven cadaver hands at the 1998 posture. Each tendon was pulled at 25%
    of its assumed maximal force (Table 1: 24 to 240 N) and the fingertip
    wrench measured. The Fig. 3 inset gives mean and SD for each of seven
    tendons. Four times those columns is a *measured* A at full activation.
  - The inset is an image; we transcribed its numbers by eye from a 400 dpi
    render, so they need a second reading before use.
  - An erratum exists: *J Biomech* 34(1):151, 2001, DOI
    10.1016/s0021-9290(00)00201-3. Its content is **[UNVERIFIED]**: the page
    returned 403.
  - Verdict: **borrow** as the second, measured route. Effort: **hours**.
- Cohn BA, Szedlák M, Gärtner B, Valero-Cuevas FJ. "Feasibility Theory
  Reconciles and Informs Alternative Approaches to Neuromuscular Control".
  *Frontiers in Computational Neuroscience* 12:62, 2018. DOI
  10.3389/fncom.2018.00062. **[read]**
  - Eq. 3 is the same 4×7 matrix at the 1998 posture. The code at
    https://github.com/bc/space (commit 6b48d48) holds a numeric matrix and
    maximal forces. Solving it by linear programme gives 28.812 N for the
    maximal distal force, matching the paper's 28.81 N. That only works if
    the other three outputs are held at zero, so the published set is the
    zero-torque slice, not a projection.
  - The column order (FDP, FDS, EIP, EDC, LUM, DI, PI) is presumed from the
    paper's Eq. 4, not confirmed in the code. **[UNVERIFIED]**
  - The paper attributes the model to Valero-Cuevas et al. 2009, *IEEE
    Reviews in Biomedical Engineering*, DOI 10.1109/RBME.2009.2034981, which
    we did not check and which may tabulate the matrix. **[UNVERIFIED]**
  - Verdict: **borrow** as the third route and as a check. Effort: **hours**.
- Kutch JJ, Valero-Cuevas FJ. "Muscle redundancy does not imply robustness to
  muscle dysfunction". *J Biomech* 44(7):1264–1270, 2011. DOI
  10.1016/j.jbiomech.2011.02.014. **[read]**
  - Cadaver tendon actions at three to five postures, but plotted only, with
    joint angles not reported. Verdict: **cite**; not usable as data.

### What the repo already does (commits 3f4fe31 and 2304241)

`tools/valero_cuevas_finger.py` rebuilds route one. It writes
`outputs/vc1998_index_finger.json` and two draft records,
`data/records_draft/human_index_vc1998_{nominal,adjusted}.json`.

- **The model.** M = J^-T R F0 from Table 1 (nominal and adjusted columns) and
  Table 2 (lengths 50, 31 and 16 mm), with F0 = PCSA × 35 N/cm². The 1998
  paper states the 35 N/cm². The script's comment credits it to "Zajac 1989";
  that attribution is **[UNVERIFIED]**, since we did not check the paper's
  reference for it.
- **The reconstructed part.** The paper does not give the full split of the
  lumbrical and palmar interosseous tensions through the extensor hood. The
  script reconstructs it as a two-band node in equilibrium and marks it
  RECONSTRUCTED. In the adjusted model the lateral band takes a share of
  1.11, so the two bands together carry 1.82 times the input tension. That is
  physically doubtful and is flagged here for whoever owns the script.
- **Check against Table 6.** All five joint-torque directions match the paper
  to two decimals, in both models.
- **Check against Table 3.** The maximal forces do not all match. In
  newtons, rebuilt against the paper's model column:

  | Direction | Nominal, ours | Nominal, paper | Adjusted, ours | Adjusted, paper | Measured, mean ± SD |
  |---|---|---|---|---|---|
  | lateral | 31.4 | 27.2 | 33.4 | 32.1 | 14.7 ± 4.8 |
  | medial | 8.1 | 15.0 | 20.2 | 23.7 | 22.9 ± 7.8 |
  | distal | 20.4 | 17.4 | 45.6 | 38.9 | 24.3 ± 8.3 |
  | palmar | 19.7 | 17.3 | 28.1 | 24.2 | 27.9 ± 4.1 |
  | dorsal | 4.6 | 4.1 | 8.0 | 6.9 | 7.5 ± 1.5 |

  In the nominal model, medial is about half the paper's value and the other
  four are 12–17% high. In the adjusted model, medial is 15% low and the
  other four are 4–17% high.
  The torque directions match, so the moment arms are right where they act
  directly. The likeliest cause is the reconstructed hood split, which
  carries the intrinsic muscles' share; this is our inference, not checked.
  **We should not say the rebuild reproduces the 1998 model.**
- **Three measures of the set.** The output reports three matrices. Their
  eigenvalues (N²) and authority √trace (N) are:

  | Measure | What it is | Nominal eigenvalues | Nominal √trace | Adjusted eigenvalues | Adjusted √trace |
  |---|---|---|---|---|---|
  | W_in | M_f M_f^T over all muscles at unit excitation, as W_c | 1318.7, 696.1, 357.5 | 48.7 | 2985.8, 1453.1, 818.1 | 72.5 |
  | W_held | The same, restricted to excitation changes giving no distal torque | 1318.5, 677.5, 224.3 | 47.1 | 2893.4, 904.4, 783.8 | 67.7 |
  | W_f | Second moment of the 26 linear-programme maxima, with distal torque held at zero | 178.1, 116.7, 53.0 | 18.7 | 355.1, 218.7, 156.5 | 27.0 |

  The draft records hold W_f: the `force_set` block of the force-set
  proposal, which is the zero-torque slice the paper publishes. W_in is the
  projection, comparable to our actuator W_c, and is about 2.6 times larger
  in authority. The two must not be compared with each other.
- **The records are drafts.** They are tagged schema version "0.2-draft
  (PROPOSAL_FORCE_SET_RECORDS.md, not approved)", with the caveat "model, not
  measurement". The force-set proposal is on a Gramain_extraction branch and
  is not approved.
- **The MuJoCo version** (`tools/vc_finger_mujoco.py`, commit 2304241) builds
  the same finger as a physical model. Its statics reproduce the matrix to
  machine precision (largest Jacobian difference 1.1 × 10⁻¹⁶). Each muscle
  alone moves the joints the textbook way in 16 of 16 cases. Under the pinned
  test, the distal and dorsal maximal-force patterns are unstable along the
  pinned finger's one spare motion; palmar is stable; lateral and medial are
  neutral. Its dynamic test drifts by 12° to 197° from the posture, so only
  its statics should be quoted.

### How far the numbers can be trusted

- **One posture only.** No published table gives A at other postures. Lee et
  al. 2015 gives moment arms against joint angle, but no F0, so a second
  posture means mixing sources.
- **Specific tension varies.** The sources use 35 N/cm² (1998), 45 N/cm²
  (Holzbaur 2005, for forearm and hand muscles) and 50.8 N/cm² (McFarland
  2023, intrinsics). The lab's 2018 maximal forces are close to the adjusted
  PCSA × 30 N/cm², by our inference. Across 30 to 50.8, F0 changes by up to
  about 1.7 times.
- **Five directions measured, not 26.** Our force-set measure samples 26
  directions; the human data has five. Li ZM et al. 2003 measured 16
  directions, but at the proximal phalanx, not the fingertip.

### Other finger and muscle models (citation only unless marked)

- Holzbaur KRS, Murray WM, Delp SL. "A Model of the Upper Extremity for
  Simulating Musculoskeletal Surgery and Analyzing Neuromuscular Control".
  *Annals of Biomedical Engineering* 33(6):829–840, 2005. DOI
  10.1007/s10439-005-3320-7. **[read: Table 1]** No intrinsic hand muscles.
  Verdict: **ignore** for the finger record.
- Lee JH, Asakawa DS, Dennerlein JT, Jindrich DL. "Finger Muscle Attachments
  for an OpenSim Upper-Extremity Model". *PLOS ONE* 10(4):e0121712, 2015. DOI
  10.1371/journal.pone.0121712; correction DOI 10.1371/journal.pone.0267620.
  **[read: first part]** Moment arms against angle (Tables 2–6), no F0.
  Verdict: **borrow** if we need a second posture. Effort: **days**.
- McFarland DC et al. "A Musculoskeletal Model of the Hand and Wrist Capable
  of Simulating Functional Tasks". *IEEE TBME* 70(5):1424–1435, 2023. DOI
  10.1109/tbme.2022.3217722. **[read: preprint, first part]** 43 muscles,
  intrinsic F0 at 50.8 N/cm². Verdict: **cite**; the model file was not
  downloaded.
- Barry AJ, Murray WM, Kamper DG. "Development of a dynamic index finger and
  thumb model to study impairment". *J Biomech* 77:206–210, 2018. DOI
  10.1016/j.jbiomech.2018.06.017. **[metadata only]** Verdict: **cite**.
- Synek A, Pahr DH. "The effect of the extensor mechanism on maximum isometric
  fingertip forces: A numerical study on the index finger". *J Biomech*
  49(14):3423–3429, 2016. DOI 10.1016/j.jbiomech.2016.09.004.
  **[abstract]** Feasible force sets at nine postures; it bears directly on
  the hood split we reconstructed. Verdict: **cite**, and read it before
  trusting the reconstruction. Effort: **hours**.
- Li ZM, Pfaeffle HJ, Sotereanos DG, Goitz RJ, Woo SL-Y. "Multi-directional
  strength and force envelope of the index finger". *Clinical Biomechanics*
  18(10):908–915, 2003. DOI 10.1016/S0268-0033(03)00178-5. **[abstract]**
  Verdict: **cite**.
- Lee SW, Qiu D, Fischer HC, Conrad MO, Kamper DG. "Modulation of finger muscle
  activation patterns across postures is coordinated across all muscle
  groups". *J Neurophysiol* 124(2):330–341, 2020. DOI 10.1152/jn.00088.2020.
  **[abstract]** Verdict: **cite**.
- Milner TE, Dhaliwal SS. "Activation of intrinsic and extrinsic finger
  muscles in relation to the fingertip force vector". *Experimental Brain
  Research* 146(2):197–204, 2002. DOI 10.1007/s00221-002-1177-7.
  **[metadata only]** Verdict: **cite**.
- An KN, Ueba Y, Chao EY, Cooney WP, Linscheid RL. "Tendon excursion and
  moment arm of index finger muscles". *J Biomech* 16(6):419–425, 1983. DOI
  10.1016/0021-9290(83)90074-x. The 1998 paper cites it for moment arms.
  **[metadata only]** Verdict: **cite** through the 1998 paper.

## 5. Q4: the other sources, placed

### Grasp quality and the handbook

- **Prattichizzo D, Trinkle JC. "Grasping".** In Siciliano B, Khatib O (eds),
  *Springer Handbook of Robotics*.
  - 1st edition, 2008, pp. 671–700, DOI 10.1007/978-3-540-30301-5_29. The
    chapter number 28 is inferred from the author PDF; the DOI suffix is
    offset. **[read]**
  - 2nd edition, 2016, chapter 38, pp. 955–988, DOI
    10.1007/978-3-319-32552-1_38. **[read]**
  - What it does: linear models of hand and object from contact kinematics
    and quasi-static force balance. Grasp classes come from null spaces:
    redundant N(J), indeterminate N(G^T), graspable N(G), defective N(J^T),
    and hyperstatic N(J^T) ∩ N(G). **There is no "manipulable" class** in
    either edition. Table 38.6 lists the desirable properties. Closure is
    tested by linear programmes: LP2 gives a margin d, and LP3 adds J^T λ = 0.
    Eq 38.59 gives the inscribed pyramid; replacing μ by μ/cos(π/n_g) gives
    the circumscribed one. The 2nd edition adds section 38.3 on compliant
    grasps (eqs 38.30, 38.39 and 38.41).
  - Our equivalent: nearly all of section 1. Our generators and outer bracket
    are eq 38.59. Definition 38.10 (force closure) adds the condition
    N(G) ∩ N(J^T) = 0 to frictional form closure; this is the closest
    published match in the handbook to our zero-unactuated-torque block;
    Bicchi and Prattichizzo 2000 eq. 15 is the direct one.
  - Notation clash: the handbook's H is the contact selection matrix; Roa and
    Suárez use H for the hand-object Jacobian.
  - Verdict: **cite** as the reference for what we reinvent. Effort: none.
- **Roa MA, Suárez R. "Grasp quality measures: review and performance".**
  *Autonomous Robots* 38(1):65–88, 2015. DOI 10.1007/s10514-014-9402-3.
  **[read]**
  - What it does: reviews about twenty grasp-quality measures in two groups:
    those based on contact positions (on G) and those based on hand
    configuration (on H = (G^T)^+ J_h, eq 7). It covers the minimum singular
    value (eq 8), ellipsoid volume (eq 9), the largest-ball measure (eq 24),
    the hull volume (eq 25), the task-oriented measure (eq 31) and the force
    ellipsoid ω^T (HH^T) ω ≤ 1 (eq 40). Joint torques enter only through a
    torque-limited wrench set (Jeong and Cheong 2012; Zheng and Yamane 2013).
  - Our equivalent: none directly; we give no scalar measure. The
    torque-limited wrench set is the nearest to our torque test.
  - Verdict: **cite** as the map of measures we chose not to use, and say why
    (they score a grasp; we score a grasp against a given wrench). Effort:
    none.
- **Ferrari C, Canny J. "Planning optimal grasps".** *Proc. 1992 IEEE ICRA*,
  pp. 2290–2295. DOI 10.1109/robot.1992.219918. Crossref has no year; 1992 is
  from the proceedings. OpenAlex gives 2003, which is wrong. **[abstract;
  details from Roa and Suárez]**
  - What it does: two optimality criteria, total and maximum finger force,
    posed as metrics on wrench space. Roa credits the largest-ball measure
    (eq 24) to this paper, with the cone as an m-sided pyramid and two force
    bounds (L∞ per finger, L1 in total).
  - Our equivalent: the pyramid of primitive wrenches, our `generators`.
    The largest-ball measure is the source of our disturbance margin
    (section 1), taken about the leader's wrench rather than zero, sampled
    along 12 axes, and limited by torque.
  - Verdict: **cite**. Effort: none.
- **Pollard NS. "Parallel Methods for Synthesizing Whole-Hand Grasps from
  Generalized Prototypes".** MIT AI Technical Report AITR-1464, 1994.
  https://dspace.mit.edu/handle/1721.1/7055. **[read: chapter 2]**
  - What it does: defines nested wrench spaces (object, grasp and task) and
    quality measures (eqs 2.23, 2.24) with an L1-type force bound. On p. 39 it
    says the ideal measure would map the hand's joint torques to object
    wrenches; on p. 42 it uses summed contact forces instead, as a
    device-independent proxy.
  - Our equivalent: our torque test is the "ideal" the report sets aside.
  - Verdict: **cite** for that point. Effort: none.
  - Related papers, citation only: ICRA 1996, vol. 3, pp. 2124–2130, DOI
    10.1109/robot.1996.506184; *IJRR* 23:595–613, 2004, DOI
    10.1177/0278364904044402. **[metadata only]**
- **Borst C, Fischer M, Hirzinger G. "Grasp planning: how to choose a suitable
  task wrench space".** *Proc. 2004 IEEE ICRA*, pp. 319–325. DOI
  10.1109/robot.2004.1307170. **[metadata only]** That it fits an ellipsoid
  to the object wrench space is **[UNVERIFIED]**: second-hand. Verdict:
  **cite** only after reading.
- **Haschke R, Steil JJ, Steuwer I, Ritter H. "Task-Oriented Quality Measures
  for Dextrous Grasping".** *2005 IEEE CIRA*, pp. 689–694. DOI
  10.1109/cira.2005.1554357. Crossref gives no year. **[metadata only;
  content UNVERIFIED]** Verdict: read before citing.

### Manipulability and force manipulability

- **Yoshikawa T. "Manipulability of Robotic Mechanisms".** *IJRR* 4(2):3–9,
  1985. DOI 10.1177/027836498500400201. **[metadata only]**
- Yoshikawa T. "Manipulability and redundancy control of robotic mechanisms".
  *Proc. 1985 IEEE ICRA*, vol. 2, pp. 1004–1009. DOI
  10.1109/robot.1985.1087283. **[metadata only]**
- Yoshikawa T. "Dynamic manipulability of robot manipulators". *Proc. 1985
  IEEE ICRA*, vol. 2, pp. 1033–1038. DOI 10.1109/robot.1985.1087277.
  **[metadata only]**
- **Chiu SL. "Task Compatibility of Manipulator Postures".** *IJRR*
  7(5):13–21, 1988. DOI 10.1177/027836498800700502. **[metadata only;
  content via Roa eq 45]**
  - What they do: the velocity manipulability ellipsoid from J J^T, and its
    dual force ellipsoid. Chiu scores a posture by how well the ellipsoid
    matches a task direction.
  - Which paper first defined the *force* ellipsoid is **[UNVERIFIED]**: no
    primary text we fetched confirms it.
  - Our equivalent: our W_c is a manipulability-type sum, Σ |dp_k| J_k J_k^T,
    over an input path rather than at one posture. Chiu's task compatibility
    is the closest published idea to scoring against a given direction.
  - Verdict: **cite** Yoshikawa 1985 (IJRR) for the ellipsoid and Chiu for task
    compatibility. Effort: none.
- **Bicchi A, Prattichizzo D. "Manipulability of cooperating robots with
  unactuated joints and closed-chain mechanisms".** *IEEE T-RA* 16(4):336–345,
  2000. DOI 10.1109/70.864226. Precursor: "Manipulability of cooperating
  robots with passive joints", *Proc. 1998 IEEE ICRA*, vol. 2, pp. 1038–1044,
  DOI 10.1109/robot.1998.677226. **[read: the 2000 paper in full; the 1998
  paper from an archived copy]** The 2000 paper's reference [4] confirms
  the 1998 pages.
  - What it does: kinematic and force manipulability for cooperating arms
    and closed chains whose joints are split into active (J_a) and passive
    (J_p). The statics are eq. 15: the object wrench is w = G f, the active
    torques are τ_a = −J_a^T f, and J_p^T f = 0, since "no torque is applied
    at passive joints". The Remark after eq. 15 gives eq. 16: with
    unrestricted active torques, w can be balanced if and only if it lies in
    the range of G restricted to the null space of J_p^T. Balancing every w
    needs eqs. 17–18: N(G^T) = 0 and R(G^T) ∩ R(J_p) = 0. Structural forces
    lie in N(J_a^T) ∩ N(J_p^T). A mechanism is statically indeterminate when
    N(G) ∩ N(J_a^T) ≠ 0. The forces decompose as eq. 19. Active force
    manipulability (eq. 20) is a worst case over structural forces. Passive
    force manipulability (eq. 21) takes, for each external wrench, the
    least-cost torque that opposes it. Section IV.D shows that force
    manipulability does not follow from kinematic manipulability by duality;
    duality holds only between the optimised and worst-case pair (eqs. 13
    and 24). Section IV.C notes that friction and unilateral contact bound
    the internal forces nonlinearly, and points to Bicchi 1995 (IJRR) for
    that. The case studies are a five-bar linkage and a Stewart platform.
    The 1998 paper has the same statics (its eq. 12, read from a scan) and
    the same active/passive split.
  - What it does not do: no torque limits, no friction cones and no linear
    programmes. Its indices are quadratic forms, read as ellipsoids.
  - Our equivalent: eq. 15 is our default strength rule (joint stiffness 0):
    torque that no input column covers must be zero. Eq. 16 is our span
    test with the torque limits removed. We add one-sided inputs, torque
    limits and friction pyramids, which turn their linear-algebra condition
    into a non-negative least-squares feasibility test. So we inherit their
    constraint, not their index. Our scale margin is, by our reading, the
    single-wrench analogue of their passive force manipulability (eq. 21),
    with a polytope of torques in place of their quadratic cost. That
    mapping is our inference; neither paper states it.
  - Verdict: **cite** as the direct precedent for our zero-torque rule. The
    SynGrasp papers' attribution of the passive-joint model to them is
    correct. Effort: none further.
- **Bicchi A, Melchiorri C, Balluchi D. "On the mobility and manipulability of
  general multiple limb robots".** *IEEE T-RA* 11(2):215–228, 1995. DOI
  10.1109/70.370503. **[metadata only]** Verdict: **cite**.
- **Bicchi A. "On the Closure Properties of Robotic Grasping".** *IJRR*
  14(4):319–334, 1995. DOI 10.1177/027836499501400402. **[metadata only]**
  Verdict: **cite** alongside the handbook.
- **Bicchi A. "Hands for dexterous manipulation and robust grasping: a
  difficult road toward simplicity".** *IEEE T-RA* 16(6):652–662, 2000. DOI
  10.1109/70.897777. **[metadata only]** Verdict: **cite** for the
  motivation of simple hands.

### Synergies

- **Santello M, Flanders M, Soechting JF. "Postural Hand Synergies for Tool
  Use".** *Journal of Neuroscience* 18(23):10105–10115, 1998. DOI
  10.1523/jneurosci.18-23-10105.1998. **[metadata only]** The often-quoted
  share of posture variance explained by the first two or three synergies
  (84% and 90%) comes to us second-hand, from Bicchi, Gabiccini and Santello
  2011. **[UNVERIFIED in the primary]**
- **Santello M et al. "Hand synergies: Integration of robotics and
  neuroscience for understanding the control of biological and artificial
  hands".** *Physics of Life Reviews* 17:1–23, 2016. DOI
  10.1016/j.plrev.2016.02.001. **[metadata only]**
- **Bicchi A, Gabiccini M, Santello M. "Modelling natural and artificial hands
  with synergies".** *Phil Trans R Soc B* 366(1581):3153–3161, 2011. DOI
  10.1098/rstb.2011.0152. **[read, PMC]**
- **Gabiccini M, Bicchi A, Prattichizzo D, Malvezzi M. "On the role of hand
  synergies in the optimal choice of grasping forces".** *Autonomous Robots*
  31(2–3):235–252, 2011. DOI 10.1007/s10514-011-9244-1. Conference version:
  Gabiccini M, Bicchi A, *RSS VI*, 2010, DOI 10.15607/rss.2010.vi.030.
  **[read: RSS 2010, eqs 6 and 11–13]**
- **Catalano MG et al. "Adaptive synergies for the design and control of the
  Pisa/IIT SoftHand".** *IJRR* 33(5):768–782, 2014. DOI
  10.1177/0278364913518998. **[metadata only]**
  - What they do: describe hand posture by a few synergies, and choose
    grasping forces subject to what the synergies can produce. Catalano
    builds a soft hand with one synergy.
  - Our equivalent: our B = M E, with the motor map M as a synergy matrix.
  - Verdict: **cite** Santello 1998 and Bicchi, Gabiccini and Santello 2011 for
    the idea, and Gabiccini et al. 2011 for force choice under synergies.
    Effort: none.

### In-hand benchmarks and YCB

- **Cruciani S, Sundaralingam B, Hang K, Kumar V, Hermans T, Kragic D.
  "Benchmarking In-Hand Manipulation".** *IEEE RA-L* 5(2):588–595, 2020. DOI
  10.1109/LRA.2020.2964160. **[read: arXiv 2001.03070]**
  - What it does: tasks on YCB objects, each from an initial to a goal state
    (object pose and fingertip placements), scored by error at the goal.
  - Our equivalent: none. We score every frame along the motion, not the end
    state.
  - Verdict: **cite** as the standard in-hand benchmark, and contrast. Effort:
    none.
- **Calli B, Walsman A, Singh A, Srinivasa S, Abbeel P, Dollar AM.
  "Benchmarking in Manipulation Research: Using the Yale-CMU-Berkeley Object
  and Model Set".** *IEEE RAM* 22(3):36–52, 2015. DOI
  10.1109/MRA.2015.2448951. The arXiv title differs; cite the journal title.
  **[metadata only]**
- **Calli B et al. "Yale-CMU-Berkeley dataset for robotic manipulation
  research".** *IJRR* 36(3):261–268, 2017. DOI 10.1177/0278364917700714.
  **[metadata only]**
  - Our equivalent: object meshes we could use in place of our test cup.
  - Verdict: **borrow** YCB meshes for the workshop demos. Effort: **hours**.
- **Coulson R, Li C, Majidi C, Pollard NS. "The Elliott and Connolly
  Benchmark: A Test for Evaluating the In-Hand Dexterity of Robot Hands".**
  *2020 IEEE-RAS Humanoids*, pp. 238–245. DOI
  10.1109/HUMANOIDS47582.2021.9555798. **[read: pp. 1, 2 and 8]**
  - What it does: thirteen in-hand patterns from a human classification,
    each pass or fail. A soft hand (CMU Foam Hand III) passed 10 of 13.
  - Our equivalent: the closest benchmark to our leader motions, which are
    sequences rather than end states.
  - Verdict: **cite**; consider using its patterns as leader motions. Effort:
    **days**.
- **Falco J, Van Wyk K, Messina E. "Performance Metrics and Test Methods for
  Robotic Hands".** NIST SP 1227 (draft), 2018. DOI
  10.6028/nist.sp.1227-draft. **[metadata only]** That its in-hand test scores
  desired against measured object trajectory is **[UNVERIFIED]**: from
  Coulson's Table I, not the document. If true, it is the only
  trajectory-tracking hand test we found. Companion: NIST SP 1229 (draft),
  DOI 10.6028/nist.sp.1229-draft. Verdict: read before citing. Effort:
  **hours**.
- Morrow J, Campbell J, Balasubramanian R, Grimm C. "Benchmarking a Robot
  Hand's Ability to Translate Objects Using Two Fingers". *IEEE RA-L*
  7(1):588–593, 2022. DOI 10.1109/lra.2021.3129131. Also "Benchmarking Planar
  Rotation Capabilities of Robot Hands with Fingers", *2023 Ubiquitous
  Robots*, DOI 10.1109/ur57808.2023.10202227. **[metadata only]** Verdict:
  **cite**.
- Bullock IM, Ma RR, Dollar AM. "A Hand-Centric Classification of Human and
  Robot Dexterous Manipulation". *IEEE Transactions on Haptics* 6(2):129–144,
  2013. DOI 10.1109/toh.2012.53. **[metadata only]** Verdict: **cite**.
- Ma RR, Dollar AM. "On dexterity and dexterous manipulation". *2011 ICAR*,
  pp. 1–7. DOI 10.1109/ICAR.2011.6088576. **[metadata only]** Verdict:
  **cite** for definitions.
- Liconti D, Zhou Y, Toshimitsu Y, Hinchet R, Katzschmann RK. "A Benchmark of
  Dexterity for Anthropomorphic Robotic Hands". arXiv 2604.09294, April 2026.
  **[arXiv page only; no journal version]** Verdict: **cite** as recent.
- Peticco M, Agrawal P. "KaRMA: A Kinematic Metric for Fine Manipulation
  Ability in Robotic Hands". arXiv 2605.15548 v2, September 2026; the arXiv
  page says accepted at IROS 2026. **[arXiv page only]** Counts reachable
  object poses in a two-finger rolling pinch across 16 hands. The closest in
  spirit to our rigid-hand check. Verdict: **cite**.

### Anthropomorphism

- **Liarokapis MV, Artemiadis PK, Kyriakopoulos KJ. "Quantifying
  anthropomorphism of robot hands".** *2013 IEEE ICRA*, pp. 2041–2046. DOI
  10.1109/ICRA.2013.6630850. **[metadata only]** The method below is
  **[UNVERIFIED]**: the PDF returned 404, so it comes from search excerpts
  and the Llop-Harillo review. It scores overlap of convex hulls of phalanx
  workspaces and finger base frames against the human hand, as volume of
  intersection over union, combined by hand-set weights into an index from 0
  to 1.
- **Feix T, Romero J, Ek CH, Schmiedmayer H-B, Kragic D. "A Metric for
  Comparing the Anthropomorphic Motion Capability of Artificial Hands".**
  *IEEE T-RO* 29(1):82–93, 2013. DOI 10.1109/TRO.2012.2217675. **[metadata
  only; method UNVERIFIED, from snippets and a review]** Compares reachable
  fingertip poses with human ones in a learned latent space.
- Llop-Harillo I, Pérez-González A, Gracia-Ibáñez V. "Anthropomorphism Index
  of Mobility for Artificial Hands". *Applied Bionics and Biomechanics*
  2019:7169034. DOI 10.1155/2019/7169034. **[read]** Reviews both indices.
- Llop-Harillo I, Pérez-González A, Starke J, Asfour T. "The Anthropomorphic
  Hand Assessment Protocol (AHAP)". *Robotics and Autonomous Systems*
  121:103259, 2019. DOI 10.1016/j.robot.2019.103259. **[metadata only]**
- Feix T, Romero J, Schmiedmayer H-B, Dollar AM, Kragic D. "The GRASP Taxonomy
  of Human Grasp Types". *IEEE THMS* 46(1):66–77, 2016. DOI
  10.1109/THMS.2015.2470657. **[metadata only]**
- Our equivalent for all of these: none. They score likeness to the human
  hand; we score whether a hand can do a given task. Verdict: **cite** to say
  so. Effort: none. The GRASP taxonomy could supply leader motions.

### Gramians for hands or soft actuators

We found **no** work that builds an empirical controllability Gramian of tip
motion from finite-element simulation to characterise soft actuators or
fingers. The searches that came back empty are listed in the verification
notes. The nearest work:

- **Lall S, Marsden JE, Glavaški S. "A subspace approach to balanced
  truncation for model reduction of nonlinear control systems".** *Int J
  Robust and Nonlinear Control* 12(6):519–535, 2002. DOI 10.1002/rnc.657.
  **[metadata only]** Defines empirical Gramians from simulated responses.
  Verdict: **cite** as the method origin.
- Himpe C. "emgr—The Empirical Gramian Framework". *Algorithms* 11(7):91,
  2018. DOI 10.3390/a11070091. And "emgr – EMpirical GRamian Framework Version
  5.99", *ACM TOMS* 49(3):1–8, 2023. DOI 10.1145/3609860. **[metadata only]**
  Verdict: **cite**; its conventions could check ours. Effort: **hours**.
- Kawano Y, Scherpen JMA. "Empirical differential Gramians for nonlinear model
  reduction". *Automatica* 127:109534, 2021. DOI
  10.1016/j.automatica.2021.109534. **[metadata only]** Gramians of the
  linearised system along a trajectory: conceptually the nearest to our sum of
  J_k J_k^T over steps. Verdict: **cite**.
- Smocot S, Forbes JR, Sedal A. "Observability-Informed Optimal Sensor
  Placement for Soft Robots". *2026 IEEE RoboSoft*, pp. 430–437. DOI
  10.1109/RoboSoft67810.2026.11522828; arXiv 2609.33681. **[abstract]** An
  observability Gramian on a soft continuum arm; the only soft-robot Gramian
  work found. Verdict: **cite**, as the nearest soft-robotics use.
- da Silveira OAA, Fonseca JSO, Santos IF. "Actuator topology design using the
  controllability Gramian". *Structural and Multidisciplinary Optimization*
  51(1):145–157, 2015. DOI 10.1007/s00158-014-1121-z. **[abstract]** Finite
  elements with the trace of a controllability Gramian, for linear vibration
  control. Verdict: **cite** as the nearest pairing of finite elements and a
  controllability Gramian.
- Wang H, Zhang Y, Iida F. "Reduced-Order Modeling of a Soft Anthropomorphic
  Finger for Piano Keystrokes". *TAROS 2023*, LNCS, pp. 405–416. DOI
  10.1007/978-3-031-43360-3_33. **[metadata only]** Gramians only implicitly,
  through balanced truncation. Verdict: **cite**.
- SOFA model reduction, which uses snapshots, not Gramians: Goury O, Duriez C.
  "Fast, Generic, and Reliable Control and Simulation of Soft Robots Using
  Model Order Reduction". *IEEE T-RO* 34(6):1565–1576, 2018. DOI
  10.1109/TRO.2018.2861900. Thieffry M, Kruszewski A, Duriez C, Guerra T-M.
  "Control Design for Soft Robots based on Reduced Order Model". *IEEE RA-L*,
  2018 online (Crossref gives no volume or pages), DOI
  10.1109/lra.2018.2876734. Thieffry M, Kruszewski A, Guerra T-M, Duriez C.
  "Trajectory Tracking Control Design for Large-Scale Linear Dynamical
  Systems With Applications to Soft Robotics". *IEEE TCST* 29(2):556–566,
  2021. DOI 10.1109/tcst.2019.2953624. **[metadata only]** Verdict: **cite** to show the same simulator community has not
  used Gramians this way.
- Verdict for the group: our W_c is new as applied. Describe it as "a
  quasi-static analogue of an empirical controllability Gramian (Lall et al.
  2002)", which is equally a manipulability-type sum over the input path.

### Soft-actuator databases

We found **no** shared database of per-actuator stroke, blocked force and
stiffness against input level, and no shared schema for actuator records.

- **Wang Q, Cheng L, Kim J, Tsai S, Roach D, Tawfick S. "Characterization and
  Selection of Contractile Actuators for Soft Robotics Operating in the
  Quasistatic Regime".** *Advanced Intelligent Systems* 8(10), October 2026.
  DOI 10.1002/aisy.70490. **[abstract]**
  - What it does: passive and active force–displacement curves bound an
    operational range between free stroke and blocking force, for twisted
    coiled polymer, liquid-crystal elastomer and McKibben actuators. It
    chooses series or parallel combinations by an incidence-matrix method.
  - Our equivalent: our actuator records. **This is the closest prior work.**
  - Verdict: **cite** and set ours apart: we add stiffness against input, a
    finite-element Gramian, and a link to a hand-feasibility check. Effort:
    **hours** to read it in full.
- Huber JE, Fleck NA, Ashby MF. "The selection of mechanical actuators based
  on performance indices". *Proc R Soc Lond A* 453(1965):2185–2205, 1997.
  DOI 10.1098/rspa.1997.0117. **[metadata only]** Class-level charts. Verdict:
  **cite** as the chart template.
- Zupan M, Ashby MF, Fleck NA. "Actuator Classification and Selection—The
  Development of a Database". *Advanced Engineering Materials* 4(12):933–940,
  2002. DOI 10.1002/adem.200290009. **[metadata only]** The only actuator
  database found; scalar limits for classical types, no soft actuators.
  Verdict: **cite**.
- Holland DP, Park EJ, Polygerinos P, Bennett GJ, Walsh CJ. "The Soft Robotics
  Toolkit: Shared Resources for Research and Design". *Soft Robotics*
  1(3):224–230, 2014. DOI 10.1089/soro.2014.0010. **[metadata only]**
  Guides, not a database. Verdict: **cite**.
- Marechal L et al. "Toward a Common Framework and Database of Materials for
  Soft Robotics". *Soft Robotics* 8(3):284–297, 2021. DOI
  10.1089/soro.2019.0115. **[abstract]** Material data on GitHub. Verdict:
  **cite** as a precedent for a shared format, at material level.
- Sotiropoulos P et al. "A Benchmarking Framework for Systematic Evaluation of
  Compliant Under-Actuated Soft End Effectors in an Industrial Context".
  *2018 IEEE-RAS Humanoids*, pp. 280–283. DOI 10.1109/humanoids.2018.8624924.
  **[metadata only]** Verdict: **cite**.
- Shintake J, Cacucciolo V, Floreano D, Shea H. "Soft Robotic Grippers".
  *Advanced Materials* 30(29), 2018. DOI 10.1002/adma.201707035.
  **[metadata only]** Per-type tables, not per-device curves. Verdict:
  **cite**.
- Wong DCY, Luo Y, Scharff RBN. "Durability of Soft Pneumatic Actuators: A
  Review and Benchmarking Protocol". *Advanced Robotics Research*, 2026. DOI
  10.1002/adrr.202500172. **[metadata only]** Verdict: **cite**.
- Hardman D, Dai B, Guan Q, Georgopoulou A, Iida F, Hughes J. "Automated
  Benchmarking of Variable-Property Soft Robotic Fingertips to Enable
  Task-Optimized Sensor Selection". *Advanced Science* 13(15), 2026 (online
  August 2025). DOI 10.1002/advs.202509991. **[metadata only]** Verdict: **cite**.
- Greenland BG et al. "SoGraB: A Visual Method for Soft Grasping Benchmarking
  and Evaluation". arXiv 2411.19408, 2024. **[arXiv metadata only; no venue
  found]** Verdict: **cite** as a preprint, if at all.

## 6. Q5: paragraphs for the talk

**What we borrow.** Our feasibility check is standard grasp analysis. We
model each fingertip as a hard point contact with Coulomb friction,
approximate the friction cone by an inscribed pyramid, and test by linear
programme whether the contacts can balance a given object wrench, as set out
in the Springer Handbook chapter on grasping (Prattichizzo and Trinkle 2008,
2016). For our requirement that torque the actuators cannot produce must
vanish, the direct precedent is the passive-joint model of Bicchi and
Prattichizzo (2000), whose statics require zero torque at unactuated
joints; we add torque limits and friction cones to it. The handbook's
force-closure definition states a related condition on the hand's null
space. Task-oriented
grasp quality goes back to Li and Sastry (1988), and Pollard (1994) already
noted that the ideal measure would map joint torques rather than contact
forces. For compliance we follow the soft-synergy model of Prattichizzo and
colleagues (2012, 2013), implemented in SynGrasp. Our human-finger reference
is the index-finger model of Valero-Cuevas, Zajac and Burgar (1998), whose
"feasible force set" (Valero-Cuevas 2005) is what our force records measure.

**What is new.** Three things, as far as our search found. First, the task:
we score a hand frame by frame against one exact wrench per frame, set by a
leader motion, rather than against a worst case over a wrench set or a goal
state (compare Li and Sastry 1988; Cruciani et al. 2020). Second, the
actuator fingerprint: we build a quasi-static analogue of an empirical
controllability Gramian (Lall et al. 2002) from finite-element sweeps of
soft actuators. We found Gramians used for soft-robot sensor placement
(Smocot et al. 2026) and finite-element actuator layout in linear vibration
(da Silveira et al. 2015), but not to characterise soft actuators. Third, the
records: a shared, machine-readable record of each actuator's stroke, force
and stiffness against input, linked to the hand check. The closest prior work
is Wang et al. (2026), which characterises contractile actuators by
force–displacement operating ranges.

**What not to claim.** We should not claim to have invented grasp
feasibility, the friction pyramid, the zero-unactuated-torque condition or
synergy-based hands. We should not present the human-finger numbers as
measurements: they come from a 1998 model at one posture, our rebuild of it
matches the paper's torque directions but not all its maximal forces, and
one internal split is our reconstruction. We should not compare the finger's
zero-torque force set with our actuators' unconstrained Gramian, since they
measure different things. All simulated magnitudes are comparative, not
predictive.

## 7. What to port first, ranked

1. **Read Bicchi and Prattichizzo 2000.** Done on 23 October (section 5).
   It is the direct precedent for our zero-torque rule.
2. **A per-frame scale margin.** Done on 23 October and corrected on 24
   October (input-bound slack, cost; now off by default): `_scale_margin` in
   `follow.py` (section 1). It is the largest factor by which the leader's
   wrench could grow and still be held, on the existing `_redistribute`
   system (Li and Sastry's β for a single wrench; the handbook's LP2 margin
   is the nearest published form).
   A worst-direction disturbance margin followed on 25 October:
   `_disturbance_margin` (section 1), the largest-ball measure limited by
   torque, about the leader's wrench. Off by default. The apply limit
   followed on 26 October: `_apply_limit` (section 1), the torque-limited
   wrench set with no leader load, by actuation alone. Off by default.
3. **Joint compliance from SynGrasp.** Partly done in commit fbf16fd: a
   finite kq lets each free joint hold torque within a deflection budget
   (section 2). Still to do: solve for the deflection, as SynGrasp's
   quasi-static model does, rather than bound it; add contact stiffness;
   and check against a SynGrasp example run in MATLAB or Octave. Effort:
   **days**.
4. **The two other routes to the human finger.** The 2000 cadaver tendon
   actions and the 2018 numeric matrix, as checks on our rebuild, and Synek
   and Pahr 2016 before trusting the hood split. Effort: **hours** each; the
   Fig. 3 inset needs a second reading.
5. **YCB objects** in the workshop demos, in place of the test cup. Effort:
   **hours**.
6. **SynGrasp's scalar indices** (minimum singular value, isotropy,
   manipulability volume) as extra columns in the report. They do not change
   a verdict. Effort: **hours**.
7. **Elliott and Connolly patterns** (Coulson et al. 2020) as leader motions.
   Effort: **days**.

We would not port SynGrasp whole (weeks, for features we do not need), nor
the anthropomorphism indices (they answer a different question).

## 8. References and claims we could not verify

Do not cite these as fact until someone checks them.

- Jeong and Cheong 2012 and Zheng and Yamane 2013, the torque-limited
  wrench set behind our disturbance margin and apply limit. We know them only through Roa
  and Suárez 2015 and have not checked either.
- The full texts of Li and Sastry 1987 and 1988. All Li and Sastry content
  here is from the 1986 memo.
- The full texts of the SynGrasp ICRA 2013 and RAM 2015 papers.
- The official SynGrasp 2.6 source and its licence header (zip not
  downloaded).
- The SynGrasp v2.2 sign question in the Kz row of `SGquasistaticHsolution`.
- The content of the SynGrasp video (title, channel and date only).
- That Borst, Fischer and Hirzinger 2004 fit an ellipsoid to the object
  wrench space; the content of Haschke et al. 2005.
- Where the force manipulability ellipsoid was first defined.
- The 84% and 90% synergy figures in Santello et al. 1998 (we have them
  second-hand).
- The method of the NIST SP 1227 in-hand test (second-hand, from Coulson).
- The method details of Liarokapis et al. 2013 and Feix et al. 2013, and the
  Liarokapis scores reported for particular hands (about 10% and 40%).
- Whether Chalon et al. 2010 ("The thumb: guidelines for a robotic design",
  IROS 2010, DOI 10.1109/iros.2010.5650454) uses the Kapandji test.
- SoGraB's venue.
- That the 1998 paper credits its 35 N/cm² to Zajac 1989, as our script's
  comment says.
- The content of the 2001 erratum to Valero-Cuevas et al. 2000.
- The Fig. 3 inset values of Valero-Cuevas et al. 2000 (transcribed by eye).
- The column order and row meanings of the matrix in github.com/bc/space.
- Valero-Cuevas et al. 2009, *IEEE Reviews in Biomedical Engineering*, DOI
  10.1109/RBME.2009.2034981.
- The provenance of the SimTK "Index finger model" (no authors shown) and the
  MyoSuite paper (no Crossref record found).
- Found by search but not checked on Crossref: "Anthropomorphism Indexes of
  the Kinematic Chain for Artificial Hands", *J Bionic Eng* 17(3):501–511,
  2020; Liang et al. 2020, DOI 10.1177/1687814020933409; a review in
  *Microsystems & Nanoengineering*, s41378-021-00323-5; NIST IR 8508 "FY24
  Soft Robotics Report".
- Shared datasets not fetched: Zenodo records 18433594 and 18662373; Dryad
  doi:10.5061/dryad.jsxksn0mt.
- Soft-robot manipulability-ellipsoid papers (an ASME JMR paper and an OctArm
  dexterity study), not identified exactly.
