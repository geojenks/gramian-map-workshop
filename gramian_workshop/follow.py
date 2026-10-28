"""Follow the leader: can this rigid hand do what the leader's contacts do?

A leader (data/leaders/*.json) says, frame by frame, where each contact
station is, whether it touches, and the force it puts on the object. A hand
follows it if its fingers can be at those contacts, pad facing the object,
and supply those forces.

Choices, stated up front (George, 21 Oct: "yes that's good"):
  placement   the leader (object and stations) is placed relative to the hand
              by a search over position and orientation that best lets the
              fingers reach the stations they serve
  assignment  each finger serves at most one station, the pairing that best
              suits the hand (all pairings tried); stations left without a
              finger must have their load carried by the others
  contact     (the thumb: its last phalanx only, 26 Oct)
              anywhere on the last or middle phalanx except the nail (back)
              half: the pad and both sides (George 23 Oct: "a lot of human
              grasping is done with the side of the finger"; middle phalanx
              on by default since 22 Oct). "Palm side" is the side the
              fingertip moves toward when the finger curls; points up to
              CONTACT_CONE_DEG from it count, spread along the phalanx and
              round it. Facing is then a contact check, not a pad
              preference: the surface at the chosen point must face the
              ball (a surface turned away cannot touch it there)
  wrist       joints every finger passes through (Shadow's WRJ1/WRJ2) are held
              at the model's rest pose, unless wrist=True: then they are
              re-chosen at every moment to suit the fingers (a 5x5 grid over
              their ranges, refined once); the wrist's own torque and rate
              are not checked
  object      the leader's own, not rescaled
  spacing     the stations keep the leader's angles round the ring, unless
              respace=True (George 23 Oct, "be kind to the robot hands"):
              then each station's whole path is turned about the vertical
              through the object by up to RESPACE_DEG, chosen with the
              placement. The respaced gait is not simulated: the object's
              wrench stays the leader's, and the hand must produce it from
              the moved contacts (so a respaced frame always goes through
              the redistribution)
  strength    a finger's joints must be balanced by its inputs, each within
              its own range (one-sided for muscles, which only pull):
              joints slaved by an equality (L20) pass their torque to the
              joint that drives them, and torque no input can make
              (Shadow's J1/J2 share one tendon motor) must be carried by
              passive joint stiffness: joint_stiffness (N.m/rad, default 0,
              i.e. none) within a deflection that moves the contact at most
              DEFLECT_MAX_M (as the soft finger's passive model). Solved
              exactly as one non-negative system (the SynGrasp-style
              compliant reading, 23 Oct). Joint-limit reactions are not
              credited (conservative).

Verdict per finger per frame (and per frame: the worst finger):
  green   reaches the leader's contact within REACH_GREEN_M with the pad
          facing within FACE_GREEN_DEG, and the leader's own force at that
          station lies in the friction cone there and within torque limits
  amber   close (within REACH_AMBER_M / FACE_AMBER_DEG), or a station has no
          finger, and the object wrench the leader applies can still be
          produced with different forces at the contacts the hand does make
          (the W20 readout, vendored: direction, friction cone, torque)
  red     neither; the reason is the first test failed
With follow(margin=True), each frame also carries a scale_margin: the
largest factor by which the leader's object wrench could grow and the
contacts made still produce it (see _scale_margin). It does not change the
verdict. It is off by default, as in readout.soft_hold(want_margin=False), because it
makes a run several times slower. With follow(disturbance=True), each frame
also carries a disturbance_margin: the smallest extra push or twist on the
object, on top of the leader's load, that the contacts made could no longer
resist (see _disturbance_margin; also off by default, and also no change to
the verdict). With follow(apply=True), each frame also carries an
apply_limit: the largest push and twist the made contacts could put on the
object by actuation alone, with no leader load, along each of the 12 signed
axes (see _apply_limit; off by default, no change to the verdict).
follow(apply_free=True) adds a second reading that lets the other five
components take any value; it is far slower (see _apply_limit).
With follow(inertia=True), each touching finger's joints must also supply
the torque its own motion needs, M(q) qdd + C(q, qd) qd, on top of the
contact load, in the per-finger torque test, the redistribution and the
scale and disturbance margins (not the apply limit); see _finger_dynamics.
Off by default, so earlier results reproduce exactly. The fingers' weight
stays out, as before.
With follow(rate_limit=r) and/or follow(lag_s=T), each actuator's net
torque at a checked frame must be reachable from what it gave at the
previous checked frame: within r times its limit per second of it, and/or
along a first-order lag with time constant T towards any torque in its
range (both given: both apply). The test that decided the previous checked
frame gives that torque (see _reach). This box holds in the per-finger
torque test, the redistribution and the scale and disturbance margins (not
the apply limit, and not the passive stiffness columns). Off by default;
no constant is sourced, so none is set.

Joint springs (28 Oct, issue 21): where the hand model gives a finger's
joints a spring (MuJoCo joint stiffness k about springref, as
tools/build_record_hand.py writes for record-built fingers), the spring
pushes back by k (q - springref) at the pose the IK finds, and the
finger's inputs must supply that torque too, on top of the contact load:
it is added to the joint balance as the finger's motion torque is (C x =
Tr f + tau; with inertia=True the two add), in the per-finger torque
test, the redistribution, the scale and disturbance margins, the
whole-motion check and the apply limit (there as a load the inputs must
hold before they push; a spring is never counted as an input). Each
touching finger's record gets "spring": the torque on its free joints
(N.m for hinges, N for slides) and its share of what the inputs can make
(see _spring_item). It is read from the model, never switched: a hand whose
fingers have no joint stiffness (every rigid hand here) takes exactly the
old path, and its results carry no new keys. Tendon springs are not read
(no hand here has one). joint_stiffness is a separate term and is
unchanged: it adds its uniform deflection allowance (columns, within
DEFLECT_MAX_M) on top of the model's springs, whatever the model holds.
The tolerance stays UNACT_TOL of the whole joint torque, springs
included, and a hand with slides mixes N and N.m in that norm.

Contact (26 Oct): a TANGENT contact. A finger touches with its last or
middle phalanx, each as a capsule (the model's own for capsule links; one
fitted to the link's vertices for meshes): the IK puts the capsule axis at
R + r from the object's centre, along the direction to the leader's contact,
so the surfaces touch there, at whatever point along the link that falls.
Reach is how far that touch is from the leader's contact; "facing" is then
the side angle: between the link's palm-side direction and the direction to
the object (0 = middle of the pad, 90 = the side; green up to
SIDE_GREEN_DEG, i.e. anywhere but the nail half, amber to SIDE_AMBER_DEG).
This replaces the pre-chosen pad points, which made a tilted finger cut into
the object.

Penetration (26 Oct, George: "the fingers go inside the ball ... that is of
course non-physical"): no part of a touching finger, nor the palm, may sit
inside the object beyond what the task allows. Fingers are not checked
against each other, and lifted fingers are not posed.

Allowed overlap is part of the task (ruling of 28 Oct, issue 22): the
leader file states it (docs/TASK_FILE_FORMAT.md), and the check scores the
hand against exactly that, with no other penalty for sinking in. Two
allowances (see overlap_allowance):
  at the contact  allowed_overlap_m: how deep the touching link (the
                  phalanx at the leader's contact) may sink in; default
                  OVERLAP_ALLOWED_M
  elsewhere       other_contact: whether the other links of a touching
                  finger and the palm may touch the object, and how deep;
                  default allowed, as deep as at the contact. Not allowed:
                  they must keep TOUCH_TOL_M clear of it (the distance at
                  which the check counts a finger as touching)
Both are used the same way in the four places sinking enters:
  pose search   a pose within both allowances ranks as green; the
                placement and wrist searches pay only for depth beyond
                them
  push-out      each IK solve is tried twice. First the push-out keeps
                clear of the allowance (PUSH_CLEAR: it acts on a part
                deeper than a fifth of its allowance and pushes it back to
                zero). If that pose is not green, a second try leaves
                overlap within the allowance alone (PUSH_USE: it acts only
                beyond the allowance, and only back to it), and the better
                of the two is kept. The second try alone settles on the
                bound and, pulled by reach, often just past it (tenths of a
                mm); the first alone can give up reach to stay clear of an
                overlap the task allows
  verdict       a finger is green only within both allowances; amber
                ("close") if no part is deeper than its allowance plus the
                contact allowance again; red ("penetrates") beyond. The
                palm likewise, per frame ("palm penetrates"). A finger that
                misses its contact still helps hold the object only within
                the amber depths
  contact patch a soft pad pressed the contact allowance into the object
                touches over a patch (see _follow), so the contact
                allowance also widens the reach that counts as at the
                contact
Before 28 Oct the same tests ran with one hidden allowance, PEN_GREEN_M (3
mm; amber to PEN_AMBER_M, 6 mm), on the whole finger at once, and the
push-out began at a fifth of it and pushed back to zero (the first try
alone). Setting both allowances to PEN_GREEN_M and PUSH_USE = None gives
that behaviour back exactly. Each finger record carries "overlap_mm"
(deepest point of the touching link; negative: clear of the surface by
that much) and "other_overlap_mm" (the finger's other links), each frame
"palm_overlap_mm", and the result's settings the allowances used.

Simulation only; comparative, not predictive. The hand's own weight is not
counted.
"""
import itertools
import math

import numpy as np

from .hands import Hand, _mj
from .vendor import capability_core as cap

REACH_GREEN_M = 0.002      # W20 readout (b)'s IK residual bar
REACH_AMBER_M = 0.010      # proposed: "close"
FACE_GREEN_DEG = 30.0      # proposed: pad facing the object
FACE_AMBER_DEG = 60.0      # proposed
CONTACT_CONE_DEG = 85.0    # surface points within this of the palm-side direction
                           # (90 = the whole non-nail half; 60 until 23 Oct)
N_PAD_POINTS = 6           # candidate points along each phalanx ...
PAD_AROUND_DEG = (-70.0, -35.0, 0.0, 35.0, 70.0)   # ... at these angles round it
IK_ITERS = 60              # 200 changed nothing (reach ladder, 22 Oct)
PLACE_FINE = {"n_frames": 10, "n_samples": 80, "iters": 4, "pad_step": 6}   # pad_step 2 until
                           # 23 Oct, when side contact tripled the candidates
PLACE_COARSE = {"n_frames": 6, "n_samples": 60, "iters": 3, "pad_step": 3}
IK_STARTS = 3              # starting poses per pad point (see FingerIK.best)
RESID_TOL = 0.05           # proposed: amber may leave this share of the
                           # leader's object wrench unbalanced (a slight
                           # acceleration of the object), as a fraction of its norm
IK_DAMP = 1e-4
RESPACE_DEG = 20.0         # proposed: how far a station may be moved round the ring
PEN_GREEN_M = 0.003        # before 28 Oct: a pad may compress this far into the object (George 26 Oct, from his index: p90 ~6 mm)
PEN_AMBER_M = 0.006        # before 28 Oct: "close" (George 26 Oct). follow() no longer reads
                           # either; the task's allowance replaces them. The pad-point IK
                           # (FingerIK.solve and .best, which follow() no longer calls) and
                           # tools/feasibility_probe.py still use them
OVERLAP_ALLOWED_M = 0.003  # allowed overlap at a contact when the task states none (George 28 Oct, issue 22)
PUSH_CLEAR = (0.2, 0.0)    # IK push-out, first try: on a part deeper than 0.2 of its allowance,
                           # back to 0 (keeps clear of the bound; as before 28 Oct)
PUSH_USE = (1.0, 1.0)      # second try, where the first is not green: only beyond the
                           # allowance, back to it (None: no second try, as before 28 Oct)
PEN_WEIGHT = 1.0           # IK weight of the push-out row (as position)
TOUCH_TOL_M = 0.002        # a finger this close to the object still counts as touching it
SIDE_GREEN_DEG = 90.0      # tangent contact: anywhere but the nail half (George 23 Oct)
SIDE_AMBER_DEG = 120.0     # proposed: onto the edge of the nail half
SIDE_WEIGHT_M = 0.005      # soft IK preference for the pad over the side (m per rad)
UNACT_TOL = 0.05           # unbalanced joint torque allowed, share of |J^T f|
DEFLECT_MAX_M = 0.002      # passive joint stiffness may deflect the contact this far
JOINT_STIFFNESS = 0.0      # N.m/rad, passive, every free joint (follow(joint_stiffness=))
FACE_WEIGHT_M = 0.02       # metres of position error per radian of facing error
MARGIN_MAX = 1024.0        # proposed: scale margin search stops here (W20's grid
                           # ends at cap.SCALE_MAX = 16; the cup leaders clear that)
MARGIN_REL = 0.01          # proposed: above that grid, bisect to this share of the
                           # margin (a failed solve there costs ~0.7 s)
DISTURB_DIRS = [(i, sg) for i in range(6) for sg in (1.0, -1.0)]
                           # proposed: disturbance directions, the 12 signed axes
                           # of the scaled object wrench (push along x, y, z,
                           # twist about x, y, z); their minimum is an upper
                           # bound on the true smallest disturbance
DISTURB_LABELS = ["fx", "fy", "fz", "tx", "ty", "tz"]
APPLY_FIRST_N = 1.0        # proposed: the apply limit's search tries this push
                           # (N; a twist of this times L) first, then doubles up
                           # to MARGIN_MAX or bisects below it
FIT_HALF_S = 0.10          # finger inertia: joint velocity and acceleration from a
                           # local quadratic fit over +/- this (s), the fit and window
                           # tools/import_demo_leader.py uses for the object's motion
FIT_MIN_POINTS = 5         # as that tool's local_fit


# ----------------------------------------------------------------------
# contact candidates on the palm side of a phalanx
# ----------------------------------------------------------------------
def _link_vertices(m, body):
    mj = _mj()
    pts = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != body or (m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0):
            continue
        Rm = np.zeros(9)
        mj.mju_quat2Mat(Rm, m.geom_quat[g])
        t = m.geom_type[g]
        s = m.geom_size[g]
        if t == int(mj.mjtGeom.mjGEOM_MESH):
            mid = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        else:
            # surface samples of the primitive, in its own frame
            u = np.random.default_rng(0).standard_normal((600, 3))
            u /= np.linalg.norm(u, axis=1, keepdims=True)
            if t == int(mj.mjtGeom.mjGEOM_SPHERE):
                v = s[0] * u
            elif t == int(mj.mjtGeom.mjGEOM_CAPSULE):
                # on the surface: the cylinder side, then the two caps (until
                # 26 Oct this mixed in points inside the capsule)
                ang = np.random.default_rng(1).uniform(0, 2 * np.pi, 300)
                z = np.random.default_rng(2).uniform(-s[1], s[1], 300)
                side = np.column_stack([s[0] * np.cos(ang), s[0] * np.sin(ang), z])
                cap_ = s[0] * u[:300] + np.outer(np.sign(u[:300, 2]), [0, 0, s[1]])
                v = np.vstack([side, cap_])
            elif t == int(mj.mjtGeom.mjGEOM_CYLINDER):
                ang = np.linspace(0, 2 * np.pi, 600)
                z = np.random.default_rng(1).uniform(-s[1], s[1], 600)
                v = np.column_stack([s[0] * np.cos(ang), s[0] * np.sin(ang), z])
            elif t == int(mj.mjtGeom.mjGEOM_BOX):
                v = u / np.abs(u).max(1, keepdims=True) * s[:3]
            else:
                continue
        pts.append(v @ Rm.reshape(3, 3).T + m.geom_pos[g])
    return np.vstack(pts) if pts else None


def _body_shapes(m, body, max_pts=300):
    """Collision shapes of a body for penetration tests, in its frame:
    ("seg", a, b, r) for capsules and spheres (exact), ("pts", V) for
    meshes, boxes and cylinders (surface points)."""
    mj = _mj()
    out = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != body or (m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0):
            continue
        Rm = np.zeros(9)
        mj.mju_quat2Mat(Rm, m.geom_quat[g])
        Rm = Rm.reshape(3, 3)
        t, sz, pos = int(m.geom_type[g]), m.geom_size[g], np.array(m.geom_pos[g])
        if t == int(mj.mjtGeom.mjGEOM_CAPSULE):
            ax = Rm[:, 2] * sz[1]
            out.append(("seg", pos - ax, pos + ax, float(sz[0])))
        elif t == int(mj.mjtGeom.mjGEOM_SPHERE):
            out.append(("seg", pos, pos, float(sz[0])))
    V = None
    others = [g for g in range(m.ngeom) if m.geom_bodyid[g] == body
              and int(m.geom_type[g]) not in (int(mj.mjtGeom.mjGEOM_CAPSULE), int(mj.mjtGeom.mjGEOM_SPHERE))
              and not (m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0)]
    if others:
        V = _link_vertices(m, body)
        if V is not None:
            if len(V) > max_pts:
                V = V[np.random.default_rng(0).choice(len(V), max_pts, replace=False)]
            out.append(("pts", V))
    return out


def _deepest(d, shapes, c, R):
    """(penetration m, body, world point, outward unit) of the deepest point
    of these bodies inside the sphere (c, R); penetration <= 0 if outside."""
    best = (-np.inf, None, None, None)
    for b, shp in shapes.items():
        Rb, pb = d.xmat[b].reshape(3, 3), d.xpos[b]
        for s_ in shp:
            if s_[0] == "seg":
                a, e, r = pb + Rb @ s_[1], pb + Rb @ s_[2], s_[3]
                ab = e - a
                L2 = float(ab @ ab)
                tt = 0.0 if L2 < 1e-16 else min(1.0, max(0.0, float((c - a) @ ab) / L2))
                q = a + tt * ab
                v = c - q                # from the segment toward the centre
                dist = float(np.linalg.norm(v))
                x = q + r * (v / dist if dist > 1e-12 else np.array([0.0, 0.0, 1.0]))
                pen = R - (dist - r)     # the surface point nearest c is dist - r from it
                if pen > best[0]:
                    best = (pen, b, x, (x - c) / max(np.linalg.norm(x - c), 1e-12))
            else:
                X = pb + s_[1] @ Rb.T
                dd = np.linalg.norm(X - c, axis=1)
                i = int(np.argmin(dd))
                pen = R - float(dd[i])
                if pen > best[0]:
                    best = (pen, b, X[i], (X[i] - c) / max(float(dd[i]), 1e-12))
    return best


def palm_shapes(hand):
    """Shapes of every body with geometry that belongs to no finger."""
    m = hand.model
    fb = set()
    for f in hand.fingers:
        first = m.jnt_bodyid[f.joints[0]] if f.joints else f.tip_body
        b = f.tip_body
        while b > 0:
            fb.add(b)
            if b == first:
                break
            b = int(m.body_parentid[b])
    return {b: sh for b in range(1, m.nbody) if b not in fb and (sh := _body_shapes(m, b))}


def palm_penetration(hand, c, R, _cache={}):
    """Deepest penetration (m) of the palm (at the hand's base pose) into the
    sphere (c, R)."""
    mj = _mj()
    key = id(hand)
    if key not in _cache:
        d = mj.MjData(hand.model)
        d.qpos[:] = hand.apply_coupling(np.array(hand.base_q))
        mj.mj_kinematics(hand.model, d)
        _cache[key] = (d, palm_shapes(hand))
    d, sh = _cache[key]
    return float(_deepest(d, sh, np.asarray(c, float), R)[0]) if sh else -np.inf


def overlap_allowance(leader_json=None, allowed_overlap_m=None, other_contact=None):
    """The task's allowed overlap between hand and object, as a dict:
    contact_m (the touching link at a stated contact), other_m (the other
    links of a touching finger and the palm; negative = a clearance they
    must keep), other_allowed, and source ("leader file", "default" or
    "override", per allowance). Read from the leader's allowed_overlap_m
    and other_contact (docs/TASK_FILE_FORMAT.md); the arguments override
    them: allowed_overlap_m a number, other_contact True, False, a number
    (allowed, to that depth) or a dict as in the file."""
    L = leader_json or {}
    src = {}
    if allowed_overlap_m is not None:
        a_c, src["contact"] = float(allowed_overlap_m), "override"
    elif L.get("allowed_overlap_m") is not None:
        a_c, src["contact"] = float(L["allowed_overlap_m"]), "leader file"
    else:
        a_c, src["contact"] = OVERLAP_ALLOWED_M, "default"
    if a_c < 0:
        raise ValueError("allowed_overlap_m must be 0 or more (metres)")
    oc, src["other"] = other_contact, "override"
    if oc is None:
        oc, src["other"] = L.get("other_contact"), "leader file"
        if oc is None:
            oc, src["other"] = True, "default"
    if isinstance(oc, bool):
        oc = {"allowed": oc}
    elif not isinstance(oc, dict):
        oc = {"allowed": True, "max_overlap_m": float(oc)}
    allowed = bool(oc.get("allowed", True))
    if allowed:
        a_o = oc.get("max_overlap_m")
        a_o = a_c if a_o is None else float(a_o)
        if a_o < 0:
            raise ValueError("other_contact.max_overlap_m must be 0 or more (metres)")
    else:
        if oc.get("max_overlap_m") is not None:
            raise ValueError("other_contact: max_overlap_m only goes with allowed: true")
        a_o = -TOUCH_TOL_M
    return {"contact_m": a_c, "other_m": a_o, "other_allowed": allowed, "source": src}


def _over(pc, po, allow):
    """Depth beyond the allowances (m, 0 within them), the larger of the
    touching link's and the other parts'."""
    return max(0.0, pc - allow["contact_m"], po - allow["other_m"])


def _overlap_rank(pc, po, allow):
    """0 within both allowances, 1 close (no part deeper than its allowance
    plus the contact allowance again), 2 beyond."""
    a_c, a_o = allow["contact_m"], allow["other_m"]
    if pc <= a_c and po <= a_o:
        return 0
    return 1 if pc <= a_c + a_c and po <= a_o + a_c else 2


def _palm_side(hand, finger, body):
    """Unit direction (in `body`'s frame) the phalanx's surface moves when
    the finger curls: the tip's velocity under the finger's joints moving
    toward the larger end of their ranges."""
    mj = _mj()
    m = hand.model
    d = mj.MjData(m)
    q = np.array(m.qpos0)
    mid = finger.ranges.mean(1)
    q[finger.qadr] = mid
    d.qpos[:] = hand.apply_coupling(q)
    mj.mj_kinematics(m, d)
    mj.mj_comPos(m, d)
    jacp = np.zeros((3, m.nv))
    p = d.xpos[body] + d.xmat[body].reshape(3, 3) @ finger.tip_local * (body == finger.tip_body)
    mj.mj_jac(m, d, jacp, None, p, body)
    # curl = toward the larger end of each joint's range
    sign = np.sign(np.abs(finger.ranges[:, 1]) - np.abs(finger.ranges[:, 0]))
    v = jacp[:, finger.dof] @ sign
    v_local = d.xmat[body].reshape(3, 3).T @ v
    n = np.linalg.norm(v_local)
    return v_local / n if n > 0 else np.array([1.0, 0.0, 0.0]), d


def pad_candidates(hand, finger, middle=False):
    """[(body, point_local, normal_local)] spread along the palm side."""
    m = hand.model
    bodies = [finger.tip_body]
    if middle:
        bodies.append(int(m.body_parentid[finger.tip_body]))
    out = []
    for b in bodies:
        V = _link_vertices(m, b)
        if V is None:
            continue
        side, _ = _palm_side(hand, finger, b)
        c = V.mean(0)
        r = V - c
        nr = np.linalg.norm(r, axis=1)
        ok = (r @ side) > math.cos(math.radians(CONTACT_CONE_DEG)) * nr
        P = V[ok]
        if len(P) == 0:
            continue
        # spread along the link's long axis and round it
        axis = V[np.argmax(np.linalg.norm(V - c, axis=1))] - c
        axis /= np.linalg.norm(axis)
        s = (P - c) @ axis
        rad = (P - c) - np.outer(s, axis)
        side_p = side - (side @ axis) * axis
        side_p /= np.linalg.norm(side_p)
        lat = np.cross(axis, side_p)
        ang = np.degrees(np.arctan2(rad @ lat, rad @ side_p))
        span = max(float(np.ptp(s)), 1e-9)
        seen = set()
        for qn in np.linspace(0.1, 0.9, N_PAD_POINTS):
            sq = np.quantile(s, qn)
            for a in PAD_AROUND_DEG:
                i = int(np.argmin(np.abs(s - sq) / span + np.abs(ang - a) / 90.0))
                if i in seen:
                    continue
                seen.add(i)
                nrm = rad[i]                    # outward, across the axis
                nrm = nrm / np.linalg.norm(nrm) if np.linalg.norm(nrm) > 0 else side
                out.append((b, P[i].copy(), nrm))
    return out


# ----------------------------------------------------------------------
# inverse kinematics for one finger, one pad point
# ----------------------------------------------------------------------
class FingerIK:
    def __init__(self, hand, finger, allow=None):
        """allow: overlap_allowance(...) (default: OVERLAP_ALLOWED_M at the
        contact, other contact allowed as deep)."""
        mj = _mj()
        self.mj, self.hand, self.f = mj, hand, finger
        self.allow = overlap_allowance() if allow is None else allow
        m = hand.model
        self.d = mj.MjData(m)
        # independent joints and their coupled dependants
        self.free = [j for j in finger.joints if j not in hand.coupled]
        self.dep = {j: hand.coupled[j] for j in finger.joints if j in hand.coupled}
        self.qadr = np.array([m.jnt_qposadr[j] for j in self.free])
        self.dofs = np.array([m.jnt_dofadr[j] for j in self.free])
        self.rng = np.array([m.jnt_range[j] for j in self.free])
        # the finger's links (tip to the first joint's body), for penetration
        first = m.jnt_bodyid[finger.joints[0]] if finger.joints else finger.tip_body
        self.shapes, b = {}, finger.tip_body
        while b > 0:
            sh = _body_shapes(m, b)
            if sh:
                self.shapes[b] = sh
            if b == first:
                break
            b = int(m.body_parentid[b])

    def proxies(self, middle=True):
        """{body: (a_local, b_local, r)}: the contact capsule of the last
        (and, with middle, the middle) phalanx, and the palm-side direction."""
        if hasattr(self, "_proxy") and self._proxy[0] == middle:
            return self._proxy[1]
        m = self.hand.model
        # the thumb touches with its distal pad only (George 26 Oct: "a HUGE
        # amount of the force in a real hand comes from the mobility of the
        # thumb which should be in contact with its pad"); its middle link is
        # the segment behind the knuckle
        middle = middle and self.f.label != "thumb"
        bodies = [self.f.tip_body] + ([int(m.body_parentid[self.f.tip_body])] if middle else [])
        out = {}
        for b in bodies:
            caps = [s_ for s_ in _body_shapes(m, b) if s_[0] == "seg"]
            if caps:
                a, e, r = max(caps, key=lambda s_: s_[3] + np.linalg.norm(s_[2] - s_[1]))[1:]
            else:
                V = _link_vertices(m, b)
                if V is None:
                    continue
                c0 = V.mean(0)
                w, U = np.linalg.eigh(np.cov((V - c0).T))
                ax = U[:, -1]
                sproj = (V - c0) @ ax
                rad = np.linalg.norm((V - c0) - np.outer(sproj, ax), axis=1)
                r = float(np.percentile(rad, 75))
                lo, hi = sproj.min() + r, sproj.max() - r
                if hi < lo:
                    lo = hi = 0.5 * (sproj.min() + sproj.max())
                a, e = c0 + lo * ax, c0 + hi * ax
            side, _ = _palm_side(self.hand, self.f, b)
            out[b] = (np.array(a), np.array(e), float(r), side)
        self._proxy = (middle, out)
        return out

    def rank(self, r):
        """(green 0 / amber 1 / red 2, reach, side) of a solve_tangent
        result, before the force tests: the order best_tangent picks by."""
        o = _overlap_rank(r[6], r[7], self.allow)
        return (0 if r[1] <= REACH_GREEN_M and r[2] <= SIDE_GREEN_DEG and o == 0 else
                1 if r[1] <= REACH_AMBER_M and r[2] <= SIDE_AMBER_DEG and o <= 1
                else 2, r[1], r[2])

    def solve_tangent(self, body, target, c, R, q0=None, ball_pen=True):
        """Touch the object (centre c, radius R) with this link's contact
        capsule at the leader's contact `target`. Returns (q_free, reach_m,
        side_deg, contact point, link outward normal there, penetration_m,
        overlap of this link m, overlap of the finger's other links m);
        penetration_m is the larger of the two.
        With ball_pen, two tries (module docstring, "Allowed overlap"):
        the push-out PUSH_CLEAR, and, unless that is green, PUSH_USE; the
        better by rank() (the first on a tie)."""
        r = self._solve_tangent(body, target, c, R, q0, ball_pen, PUSH_CLEAR)
        if not ball_pen or PUSH_USE is None or self.rank(r)[0] == 0:
            return r
        r2 = self._solve_tangent(body, target, c, R, q0, ball_pen, PUSH_USE)
        if self.rank(r2) < self.rank(r):
            return r2
        q = np.array(self.hand.base_q)        # leave self.d at the pose returned
        q[self.qadr] = r[0]
        self.d.qpos[:] = self.hand.apply_coupling(q)
        self.mj.mj_kinematics(self.hand.model, self.d)
        return r

    def _solve_tangent(self, body, target, c, R, q0, ball_pen, push):
        """One damped least-squares solve for solve_tangent; push = (from,
        to): the push-out acts on a part deeper than `from` times its
        allowance and pushes it back to `to` times it."""
        push_from, push_to = push
        mj, m, d = self.mj, self.hand.model, self.d
        a_l, b_l, r, side_l = self.proxies()[body]
        n_t = (target - c) / np.linalg.norm(target - c)
        P = c + (R + r) * n_t                  # where the capsule axis must pass
        q = np.array(self.hand.base_q)
        q[self.qadr] = self.rng.mean(1) if q0 is None else q0
        jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))

        def axis_point():
            Rb, pb = d.xmat[body].reshape(3, 3), d.xpos[body]
            A, B = pb + Rb @ a_l, pb + Rb @ b_l
            ab = B - A
            L2 = float(ab @ ab)
            tt = 0.0 if L2 < 1e-16 else min(1.0, max(0.0, float((P - A) @ ab) / L2))
            return A + tt * ab, Rb @ side_l

        for _ in range(IK_ITERS):
            d.qpos[:] = self.hand.apply_coupling(q.copy())
            mj.mj_kinematics(m, d)
            mj.mj_comPos(m, d)
            x, s_w = axis_point()
            e = P - x
            mj.mj_jac(m, d, jacp, jacr, x, body)
            Jp, Jr = self._eff_jac(jacp), self._eff_jac(jacr)
            # soft preference for the pad: turn the palm side toward the object
            e = np.concatenate([e, SIDE_WEIGHT_M * np.cross(s_w, -n_t)])
            J = np.vstack([Jp, SIDE_WEIGHT_M * Jr])
            if ball_pen:
                # the part furthest past where the push-out starts: the
                # touching link against the contact allowance, the others
                # against theirs
                best_ = None
                for (pen, bb, xp, u), a_ in zip(self.overlaps(body, (c, R)),
                                                 (self.allow["contact_m"], self.allow["other_m"])):
                    if pen - a_ * push_from > 0 and (best_ is None or pen - a_ * push_from > best_[0]):
                        best_ = (pen - a_ * push_from, pen - a_ * push_to, bb, xp, u)
                if best_ is not None:
                    _, err, bb, xp, u = best_
                    jx = np.zeros((3, m.nv))
                    mj.mj_jac(m, d, jx, None, xp, bb)
                    J = np.vstack([J, PEN_WEIGHT * (u @ self._eff_jac(jx))[None, :]])
                    e = np.concatenate([e, [PEN_WEIGHT * err]])
            dq = J.T @ np.linalg.solve(J @ J.T + IK_DAMP * np.eye(J.shape[0]), e)
            q[self.qadr] = np.clip(q[self.qadr] + dq, self.rng[:, 0], self.rng[:, 1])
        d.qpos[:] = self.hand.apply_coupling(q.copy())
        mj.mj_kinematics(m, d)
        x, s_w = axis_point()
        v = x - c
        n_out = v / np.linalg.norm(v)          # from the centre through the touch
        cp = c + R * n_out                     # the touch, on the object's surface
        side = math.degrees(math.acos(max(-1.0, min(1.0, float(s_w @ -n_out)))))
        pc, po = (float(x_[0]) for x_ in self.overlaps(body, (c, R)))
        pen = max(pc, po)
        return (q[self.qadr].copy(), float(np.linalg.norm(cp - target)), side, cp, -n_out, pen,
                pc, po)

    def best_tangent(self, target, c, R, starts=None, middle=True):
        """Best link and pose for a tangent touch at target: green before
        amber before red, then reach, then side."""
        if starts is None:
            lo, hi = self.rng[:, 0], self.rng[:, 1]
            starts = [None, lo + 0.25 * (hi - lo), lo + 0.75 * (hi - lo)][:IK_STARTS]
        best = None
        for b in self.proxies(middle):
            for q0 in starts:
                r = self.solve_tangent(b, target, c, R, q0)
                key = self.rank(r)
                if best is None or key < best[0]:
                    best = (key, (b, None, None), r)
        return best[1], best[2]

    def penetration(self, ball):
        """(depth m, body, point, outward) of the finger's deepest point
        inside ball = (centre, radius), at the pose in self.d."""
        return _deepest(self.d, self.shapes, np.asarray(ball[0], float), float(ball[1]))

    def overlaps(self, body, ball):
        """The deepest point, as penetration() gives it, of the link `body`
        (the touching link), then of the finger's other links."""
        c, R = np.asarray(ball[0], float), float(ball[1])
        own = {b: s_ for b, s_ in self.shapes.items() if b == body}
        rest = {b: s_ for b, s_ in self.shapes.items() if b != body}
        return _deepest(self.d, own, c, R), _deepest(self.d, rest, c, R)

    def _eff_jac(self, J):
        m = self.hand.model
        Je = J[:, self.dofs].copy()
        for dep, (master, c) in self.dep.items():
            if master in self.free:
                x = self.d.qpos[m.jnt_qposadr[master]]
                dd = c[1] + 2 * c[2] * x + 3 * c[3] * x ** 2 + 4 * c[4] * x ** 3
                Je[:, self.free.index(master)] += dd * J[:, m.jnt_dofadr[dep]]
        return Je

    def solve(self, body, p_local, n_local, target, n_target, q0=None, fw=None, ball=None):
        """Damped least squares on pad position and facing, and, with ball =
        (centre, radius), on pushing the deepest link point out of the
        object. Returns (q_free, residual_m, facing_deg, world point, world
        normal, penetration_m) (penetration -inf without a ball)."""
        mj, m, d = self.mj, self.hand.model, self.d
        q = np.array(self.hand.base_q)
        q[self.qadr] = self.rng.mean(1) if q0 is None else q0
        jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        fw = FACE_WEIGHT_M if fw is None else fw
        for _ in range(IK_ITERS):
            d.qpos[:] = self.hand.apply_coupling(q.copy())
            mj.mj_kinematics(m, d)
            mj.mj_comPos(m, d)
            R = d.xmat[body].reshape(3, 3)
            p = d.xpos[body] + R @ p_local
            n = R @ n_local
            e = np.concatenate([target - p, fw * np.cross(n, -n_target)])
            mj.mj_jac(m, d, jacp, jacr, p, body)
            Jp, Jr = self._eff_jac(jacp), self._eff_jac(jacr)
            J = np.vstack([Jp, fw * Jr])
            if ball is not None:
                pen, bb, x, u = self.penetration(ball)
                if pen > 0.2 * PEN_GREEN_M:
                    jx = np.zeros((3, m.nv))
                    mj.mj_jac(m, d, jx, None, x, bb)
                    J = np.vstack([J, PEN_WEIGHT * (u @ self._eff_jac(jx))[None, :]])
                    e = np.concatenate([e, [PEN_WEIGHT * pen]])
            dq = J.T @ np.linalg.solve(J @ J.T + IK_DAMP * np.eye(J.shape[0]), e)
            q[self.qadr] = np.clip(q[self.qadr] + dq, self.rng[:, 0], self.rng[:, 1])
        d.qpos[:] = self.hand.apply_coupling(q.copy())
        mj.mj_kinematics(m, d)
        R = d.xmat[body].reshape(3, 3)
        p = d.xpos[body] + R @ p_local
        n = R @ n_local
        face = math.degrees(math.acos(max(-1.0, min(1.0, float(-n @ n_target)))))
        pen = float(self.penetration(ball)[0]) if ball is not None else -np.inf
        return q[self.qadr].copy(), float(np.linalg.norm(target - p)), face, p, n, pen

    def best(self, cands, target, n_target, starts=None, weights=None, ball=None):
        """Best pad point for this target: smallest residual, then facing,
        over IK_STARTS starting poses (mid-range, and fixed spreads toward
        each end of the joint ranges) against local minima, and over facing
        weights (default FACE_WEIGHT_M only; the final check adds a
        position-first weight, since the weighted solve can settle for a
        few mm off with good facing when a green pose exists)."""
        if starts is None:
            lo, hi = self.rng[:, 0], self.rng[:, 1]
            starts = [None, lo + 0.25 * (hi - lo), lo + 0.75 * (hi - lo)][:IK_STARTS]
        best = None
        for b, pl, nl in cands:
            for q0, fw in itertools.product(starts, weights or (FACE_WEIGHT_M,)):
                r = self.solve(b, pl, nl, target, n_target, q0, fw, ball)
                # green before amber before red, then reach, then facing
                rank = (0 if r[1] <= REACH_GREEN_M and r[2] <= FACE_GREEN_DEG and r[5] <= PEN_GREEN_M else
                        1 if r[1] <= REACH_AMBER_M and r[2] <= FACE_AMBER_DEG and r[5] <= PEN_AMBER_M
                        else 2)
                key = (rank, r[1], r[2])
                if best is None or key < best[0]:
                    best = (key, (b, pl, nl), r)
        return best[1], best[2]


# ----------------------------------------------------------------------
# placing the leader relative to the hand
# ----------------------------------------------------------------------
def _rot(axis_angle):
    th = np.linalg.norm(axis_angle)
    if th < 1e-12:
        return np.eye(3)
    k = axis_angle / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


class Leader:
    def __init__(self, L):
        self.L = L
        F = L["frames"]
        self.c0 = np.array(F[0]["obj_pos"])
        self.n = L["n_stations"]
        self.R_obj = float(L["object"]["radius_m"])
        self.mass = float(L["object"]["mass_kg"])
        self.frames = F
        self.phi = np.zeros(self.n)          # station offsets round the ring (rad)

    @property
    def respaced(self):
        return bool(np.any(self.phi != 0))

    def _turn(self, i, raw):
        a = 0.0 if raw else float(self.phi[i])
        c, s_ = math.cos(a), math.sin(a)
        return np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])

    def center(self, k, Rw, t):
        """The object's centre at frame k, placed."""
        return Rw @ (np.array(self.frames[k]["obj_pos"]) - self.c0) + t

    def station(self, k, i, Rw, t, raw=False):
        """World (hand-frame) contact point and inward normal of station i at
        frame k, after placing the leader by rotation Rw and translation t
        (applied about the leader's starting object centre)."""
        f = self.frames[k]
        cp = f["contact_point"][i] or f["station_pos"][i]
        oc = np.array(f["obj_pos"])
        cp = oc + self._turn(i, raw) @ (np.array(cp) - oc)   # vertical: the sim's z
        p = Rw @ (np.array(cp) - self.c0) + t
        c = Rw @ (oc - self.c0) + t
        nrm = (c - p)
        nrm /= np.linalg.norm(nrm)
        return p, nrm, c

    def force(self, k, i, Rw, raw=False):
        return Rw @ self._turn(i, raw) @ np.array(self.frames[k]["force_on_object_N"][i])

    def made(self, k):
        return [i for i, x in enumerate(self.frames[k]["contact"]) if x]


def placement_search(hand, leader, cands, iks, n_frames=10, n_samples=80, seed=0,
                     iters=4, pad_step=6, face_weight=None, respace=False):
    """Cross-entropy search over the leader's placement (rotation vector,
    translation); cost = the best finger-to-station assignment's mean
    reach residual over sample frames where that station touches."""
    rng = np.random.default_rng(seed)
    ks = np.linspace(0, len(leader.frames) - 1, n_frames).astype(int)
    # start: object centre at the mean of the fingertips' reach, 1 radius out
    # along the mean palm-side direction
    tips = []
    for f in hand.fingers:
        pts = hand.workspace(f, 300, seed)
        tips.append(pts.mean(0))
    t_mu = np.mean(tips, 0)
    nr = leader.n if respace else 0
    lim = math.radians(RESPACE_DEG)
    mu = np.concatenate([np.zeros(3), t_mu, np.zeros(nr)])
    sd = np.concatenate([np.full(3, 1.2), np.full(3, 0.04), np.full(nr, lim / 2)])
    best = None
    for it in range(iters):
        X = mu + sd * rng.standard_normal((n_samples, 6 + nr))
        X[0] = mu
        X[:, 6:] = np.clip(X[:, 6:], -lim, lim)
        scores = []
        for x in X:
            Rw, t = _rot(x[:3]), x[3:6]
            if nr:
                leader.phi = x[6:].copy()
            C = np.zeros((len(hand.fingers), leader.n))
            for a, ik in enumerate(iks):
                for i in range(leader.n):
                    res = []
                    for k in ks:
                        if i not in leader.made(k):
                            continue
                        p, nrm, cc = leader.station(k, i, Rw, t)
                        _, r = ik.best_tangent(p, cc, leader.R_obj, starts=[None])
                        # reach error, plus touching beyond the pad's side
                        # (toward the nail), plus sinking into the object
                        # beyond the task's allowance
                        fw = FACE_WEIGHT_M if face_weight is None else face_weight
                        res.append(r[1] + fw * math.radians(max(0.0, r[2] - SIDE_GREEN_DEG))
                                   + _over(r[6], r[7], ik.allow))
                    C[a, i] = np.mean(res) if res else 1.0
            assign, cost = best_assignment(C)
            # the palm must not sit inside the object either, beyond the
            # allowance for contact away from the stated points
            a_o = iks[0].allow["other_m"]
            cost += float(np.mean([max(0.0, palm_penetration(hand, leader.center(k, Rw, t),
                                                             leader.R_obj) - a_o) for k in ks]))
            scores.append(cost)
            if best is None or cost < best[0]:
                best = (cost, Rw, t, assign, x)
        elite = X[np.argsort(scores)[:max(4, n_samples // 6)]]
        mu, sd = elite.mean(0), elite.std(0) + 1e-3
    leader.phi = best[4][6:].copy() if nr else np.zeros(leader.n)
    return best


def best_assignment(C):
    """Fingers (rows) to distinct stations (columns): the pairing with the
    lowest total cost (all pairings; hands have at most 5 fingers)."""
    nf, ns = C.shape
    best = (None, float("inf"))
    for perm in itertools.permutations(range(ns), nf):
        c = sum(C[a, perm[a]] for a in range(nf))
        if c < best[1]:
            best = (perm, c)
    return list(best[0]), best[1] / nf


# ----------------------------------------------------------------------
# the per-frame readout
# ----------------------------------------------------------------------
def torque_limits(hand, finger):
    """(M inputs x finger.joints, limits inputs x [lo, hi]), or (None, None)."""
    m = hand.model
    if m.nu and not hand.spec.get("datasheet_torque_Nm"):
        mj = _mj()
        d = hand.data
        mom = np.zeros((m.nu, m.nv))
        mj.mju_sparse2dense(mom, d.actuator_moment, d.moment_rownnz,
                            d.moment_rowadr, d.moment_colind)
        acts = [a for a in range(m.nu) if np.any(mom[a, finger.dof] != 0)]
        if not acts or not all(m.actuator_forcelimited[a] for a in acts):
            return None, None
        M = mom[np.ix_(acts, finger.dof)]
        lim = np.array([m.actuator_forcerange[a] for a in acts], float)
        return M, lim
    free = [j for j in finger.joints if j not in hand.coupled]
    if not all(m.jnt_actfrclimited[j] for j in free):
        ds = hand.spec.get("datasheet_torque_Nm")
        if ds is None:
            return None, None
        M = np.zeros((len(free), len(finger.joints)))
        for r, j in enumerate(free):
            M[r, finger.joints.index(j)] = 1.0
        return M, np.array([[-ds[0], ds[0]]] * len(free))
    idx = [finger.joints.index(j) for j in free]
    M = np.zeros((len(free), len(finger.joints)))
    for r, c in enumerate(idx):
        M[r, c] = 1.0
    lim = np.array([m.jnt_actfrcrange[j] for j in free], float)
    return M, lim


def follow(hand_key, leader_json, middle=True, n_eval=None, seed=0,
           resid_tol=None, place=None, ik_iters=None, wrist=False, respace=False,
           placement=None, joint_stiffness=None, margin=False, disturbance=False,
           apply=False, apply_free=False, inertia=False, rate_limit=None,
           lag_s=None, trajectory=False, allowed_overlap_m=None, other_contact=None):
    """place: placement_search settings (n_frames, n_samples, iters,
    pad_step; default PLACE_FINE); ik_iters: IK iterations (default
    IK_ITERS); wrist: move the shared (wrist) joints at every moment;
    respace: move stations round the ring to suit the hand; placement: a
    previous result's "placement" and "assignment" (and "respace_deg"), to
    re-evaluate without searching again; margin: give each frame a
    scale_margin (see _scale_margin); disturbance: give each frame a
    disturbance_margin (see _disturbance_margin); apply: give each frame an
    apply_limit (see _apply_limit); apply_free: add its second reading,
    other_components_free (implies apply); inertia: add the torque each
    touching finger's own motion needs (see _finger_dynamics); rate_limit
    (per second, as a share of each actuator's limit) and lag_s (s): link
    each actuator's torque to the previous checked frame's (see _reach).
    Their link is only as tight as the checked frames are dense: n_eval sets
    the time between them. trajectory: also solve each run of checked frames
    as one linear programme, with rate_limit and lag_s as rows between
    frames (see trajectory.py; needs scipy); the result's "trajectory".
    joint_stiffness (N.m/rad, default JOINT_STIFFNESS = 0; read as N/m on a
    slide): the uniform passive deflection allowance on every free joint
    (see _finger_columns).
    The model's own joint springs are not an option: they are always read,
    and add to it (module docstring, "Joint springs"). A hand whose spec
    has a "caveat" (record-built hands: the drive is not in newtons) passes
    it on in the result. allowed_overlap_m and other_contact override the
    leader's own (see overlap_allowance; module docstring, "Allowed
    overlap")."""
    global IK_ITERS
    old_iters = IK_ITERS
    if ik_iters is not None:
        IK_ITERS = int(ik_iters)
    try:
        return _follow(hand_key, leader_json, middle, n_eval, seed, resid_tol,
                       place or {}, wrist, respace, placement,
                       JOINT_STIFFNESS if joint_stiffness is None else float(joint_stiffness),
                       margin, disturbance, apply or apply_free, apply_free, inertia,
                       rate_limit, lag_s, trajectory,
                       overlap_allowance(leader_json, allowed_overlap_m, other_contact))
    finally:
        IK_ITERS = old_iters


def _wrist_search(hand, leader, cands, iks, assign, k, Rw, t, made, grid=5):
    """Shared-joint (wrist) angles for frame k that best suit the touching
    fingers: a grid over their ranges, then one finer grid around the best.
    Sets hand.base_q and returns the chosen angles."""
    m = hand.model
    adr = np.array([m.jnt_qposadr[j] for j in hand.shared_joints])
    rng = np.array([m.jnt_range[j] for j in hand.shared_joints])
    q0 = np.array(m.qpos0)

    def cost(w):
        hand.base_q = q0.copy()
        hand.base_q[adr] = w
        c = 0.0
        for a, ik in enumerate(iks):
            i = assign[a]
            if i not in made:
                continue
            p, nrm, cc = leader.station(k, i, Rw, t)
            _, r = ik.best_tangent(p, cc, leader.R_obj, starts=[None])
            c += r[1] + FACE_WEIGHT_M * math.radians(max(0.0, r[2] - SIDE_GREEN_DEG)) + _over(r[6], r[7], ik.allow)
        return c

    axes = [np.linspace(lo, hi, grid) for lo, hi in rng]
    best = min((cost(np.array(w)), w) for w in itertools.product(*axes))
    step = (rng[:, 1] - rng[:, 0]) / (grid - 1) / 2
    axes = [np.clip(np.array([-1, 0, 1]) * st + c, lo, hi)
            for st, c, (lo, hi) in zip(step, best[1], rng)]
    best = min([best] + [(cost(np.array(w)), w) for w in itertools.product(*axes)])
    hand.base_q = q0.copy()
    hand.base_q[adr] = best[1]
    return [float(x) for x in best[1]]


def _follow(hand_key, leader_json, middle, n_eval, seed, resid_tol, place, wrist,
            respace, placement, kq, margin=True, disturbance=False, apply=False,
            apply_free=False, inertia=False, rate_limit=None, lag_s=None,
            trajectory=False, allow=None):
    allow = overlap_allowance(leader_json) if allow is None else allow
    a_c, a_o = allow["contact_m"], allow["other_m"]
    hand = Hand(hand_key)
    leader = Leader(leader_json)
    cands = [pad_candidates(hand, f, middle) for f in hand.fingers]
    iks = [FingerIK(hand, f, allow) for f in hand.fingers]
    if placement is None:
        cost, Rw, t, assign, _ = placement_search(hand, leader, cands, iks, seed=seed,
                                                  respace=respace, **place)
    else:
        Rw = np.array(placement["placement"]["rotation_matrix"])
        t = np.array(placement["placement"]["translation_m"])
        assign = [int(placement["assignment"][f.label]) for f in hand.fingers]
        leader.phi = np.radians(placement.get("respace_deg") or np.zeros(leader.n))
        cost = placement["placement_cost_mm"] / 1e3
    wrist = wrist and bool(hand.shared_joints)
    mu = float(leader_json.get("mu", 1.0))
    L = leader.R_obj
    lims = [torque_limits(hand, f) for f in hand.fingers]
    # the model's joint springs, per finger (None: none on its joints)
    springs = [model_springs(hand, f) for f in hand.fingers]
    sprung = any(s is not None for s in springs)
    # the joint torque each finger must also supply goes to the force tests
    # when either term is on; otherwise exactly the old calls (None)
    use_taus = inertia or sprung
    mj = _mj()
    m = hand.model
    if inertia:
        # joint velocities are indexed by joint position below
        assert m.nq == m.nv, "inertia needs a hinge/slide-only hand"
    ks = range(len(leader.frames)) if n_eval is None else \
        np.linspace(0, len(leader.frames) - 1, n_eval).astype(int)
    link = rate_limit is not None or lag_s is not None
    # each finger's actuator torques at the previous checked frame (None: not
    # in the force check there, taken as 0), and that frame's time
    u_state, t_prev = [None] * len(hand.fingers), None
    frames = []
    if trajectory:
        from . import trajectory as tj
        recs = []
    for k in ks:
        made = leader.made(k)
        dt = None
        if link:
            tk = float(leader.frames[k]["t"])
            dt = None if t_prev is None else tk - t_prev
            bounds, xs_hold = [], {}
        wq = _wrist_search(hand, leader, cands, iks, assign, k, Rw, t, made) if wrist else None
        c = leader.center(k, Rw, t)
        W = np.zeros(6)
        for i in made:
            p, _, _ = leader.station(k, i, Rw, t, raw=True)
            fv = leader.force(k, i, Rw, raw=True)
            W += np.concatenate([fv, np.cross(p - c, fv)])
        per, pts, nrms, Js, Js_act, fingers_on, taus = [], [], [], [], [], [], []
        springs_on = []
        for a, f in enumerate(hand.fingers):
            i = assign[a]
            if i not in made:
                per.append({"finger": f.label, "station": i, "state": "off"})
                continue
            p, nrm, _ = leader.station(k, i, Rw, t)
            (b, pl, nl), (q, res, face, pw, nw, pen, pc, po) = iks[a].best_tangent(
                p, c, leader.R_obj, middle=middle)
            # a soft pad pressed the allowed overlap into the object touches
            # over a patch, not a point (Hertz: radius sqrt(2 R_eff delta));
            # the leader's contact anywhere in it is reached (26 Oct)
            r_link = iks[a].proxies(middle)[b][2]
            Reff = leader.R_obj * r_link / (leader.R_obj + r_link)
            patch = math.sqrt(2.0 * Reff * a_c)
            o_rank = _overlap_rank(pc, po, allow)
            near = res <= REACH_AMBER_M + patch and face <= SIDE_AMBER_DEG and o_rank <= 1
            exact = res <= REACH_GREEN_M + patch and face <= SIDE_GREEN_DEG and o_rank == 0
            item = {"finger": f.label, "station": i, "reach_mm": res * 1e3,
                    "facing_deg": face, "phalanx": "distal" if b == f.tip_body else "middle",
                    # the solved pose, for rendering (tools/render_follow.py)
                    "q": [float(x) for x in q], "target": p.tolist(), "pad_world": pw.tolist(),
                    "pad_normal": nw.tolist(), "penetration_mm": max(0.0, pen) * 1e3,
                    "side_deg": face, "patch_mm": patch * 1e3,
                    # depth into the object against the task's allowance
                    # (negative: clear of it by that much; None: no shape)
                    "overlap_mm": pc * 1e3 if math.isfinite(pc) else None,
                    "other_overlap_mm": po * 1e3 if math.isfinite(po) else None,
                    "overlap_within_allowance": o_rank == 0}
            if not near:
                item["state"] = "red"
                item["reason"] = ("reach" if res > REACH_AMBER_M + patch else
                                  "nail side" if face > SIDE_AMBER_DEG else "penetrates")
                # it misses the leader's contact, but if it still touches the
                # object (allowed side, not sunk too deep) that contact can
                # still help hold it (26 Oct: one near miss should not fail
                # the moment); otherwise it carries nothing
                if not (-TOUCH_TOL_M <= pen and o_rank <= 1 and face <= SIDE_AMBER_DEG):
                    per.append(item)
                    continue
                item["used_in_force_check"] = True
            # actual contact: the pad point projected onto the object
            v = pw - c
            cp = c + leader.R_obj * v / np.linalg.norm(v)
            n_in = (c - cp) / np.linalg.norm(c - cp)
            M, lim = lims[a]
            dyn = _finger_dynamics(iks[a], leader, k, i, Rw, t, b, q) \
                if inertia and M is not None else None
            iks[a].d.qpos[:] = hand.apply_coupling(_q_full(m, iks[a], q))
            mj.mj_kinematics(m, iks[a].d)
            mj.mj_comPos(m, iks[a].d)
            jacp = np.zeros((3, m.nv))
            mj.mj_jac(m, iks[a].d, jacp, None, pw, b)
            Jf = jacp[:, f.dof]
            fv = leader.force(k, i, Rw)
            in_cone = fv @ n_in >= math.cos(math.atan(mu)) * np.linalg.norm(fv) - 1e-12
            torque_ok = True
            cols = None
            tau = None
            box = None
            spring = None
            if M is not None:
                cols = _finger_columns(hand, f, iks[a], M, lim, Jf, pw, kq)
                if inertia:
                    item["inertia"], tau = _inertia_item(hand, f, iks[a], M, lim, Jf, pw, dyn)
                if springs[a] is not None:
                    item["spring"], spring = _spring_item(hand, f, iks[a], M, lim, Jf, pw,
                                                          springs[a])
                    tau = spring if tau is None else tau + spring
                if dt is not None:
                    u_lo, u_hi = _reach(lim, u_state[a], dt, rate_limit, lag_s)
                    box = _input_box(lim, cols[1].shape[1], u_lo, u_hi)
                    item["actuator_link"] = {
                        "u_prev": None if u_state[a] is None else u_state[a].tolist(),
                        "u_lo": u_lo.tolist(), "u_hi": u_hi.tolist()}
                torque_ok, un, xh = _hold_solve(cols, fv, tau, box)
                item["unbalanced_Nm"] = un
                if link:
                    xs_hold[a] = xh
            if item.get("state") != "red":
                item["state"] = "green" if (exact and in_cone and torque_ok) else "amber?"
            item["torque_checked"] = M is not None
            per.append(item)
            pts.append(cp)
            nrms.append(n_in)
            Js.append(cols)
            taus.append(tau)
            springs_on.append(spring)
            if link:
                bounds.append(box)
            # the apply limit counts inputs only: a spring resists but cannot push
            # (the joint_stiffness columns are dropped; the model's springs
            # stay only as the load at this pose, in springs_on)
            Js_act.append(_finger_columns(hand, f, iks[a], M, lim, Jf, pw, 0.0)
                          if apply and kq > 0 and M is not None else cols)
            fingers_on.append(a)
        if trajectory:
            recs.append(tj.frame_record(k, float(leader.frames[k]["t"]), pts, nrms, Js, c, W,
                                        mu, L, taus if use_taus else None, fingers_on,
                                        [x[1] for x in lims], resid_tol))
        unserved = [i for i in made if i not in assign]
        need_redistribute = unserved or leader.respaced or \
            any(x.get("state") == "amber?" for x in per)
        all_green = not unserved and all(x["state"] in ("green", "off") for x in per)
        verdict = "green"
        reason = None
        if any(x.get("state") == "red" for x in per):
            # a red finger's station load must also be carried by the others
            need_redistribute = True
        bnd = bounds if dt is not None else None
        stack = {} if link else None
        if need_redistribute:
            ok, reason = _redistribute(pts, nrms, Js, c, W, mu, L, resid_tol,
                                       taus if use_taus else None, bnd, stack)
            # a red finger whose contact the force check could still use
            # makes the moment amber at best, not red, when the forces work
            verdict = ("green" if all_green else "amber") if ok else "red"
            for x in per:
                if x.get("state") == "amber?":
                    x["state"] = "amber" if ok else "red"
        # the palm is contact away from the stated points
        palm_raw = palm_penetration(hand, c, leader.R_obj)
        palm_pen = max(0.0, palm_raw)
        if palm_raw > a_o + a_c:
            verdict, reason = "red", "palm penetrates"
        elif palm_raw > a_o and verdict == "green":
            verdict = "amber"
        if margin:
            sm = _scale_margin(pts, nrms, Js, c, W, mu, L, resid_tol,
                               taus if use_taus else None, bnd)
        if disturbance:
            dm = _disturbance_margin(pts, nrms, Js, c, W, mu, L, resid_tol,
                                     taus if use_taus else None, bnd)
        if link:
            # the next frame links to the input use of the test that decided
            # this one: the redistribution's when its torque stack ran, else
            # each finger's own hold test (NNLS gives one of the solutions,
            # deterministic but arbitrary among them)
            xs = _split_stack(Js, fingers_on, stack["x"]) if "x" in stack else xs_hold
            u_state = [None] * len(hand.fingers)
            for a, xa in xs.items():
                u_state[a] = _net_torque(lims[a][1], xa)
                rec = per[[x["finger"] for x in per].index(hand.fingers[a].label)]
                rec.setdefault("actuator_link", {})["u"] = u_state[a].tolist()
            t_prev = tk
        if apply:
            al = _apply_limit(pts, nrms, Js_act, c, mu, L, resid_tol, apply_free,
                              springs_on if sprung else None)
        frames.append({"k": int(k), "verdict": verdict, "reason": reason,
                       "ball_center": c.tolist(), "palm_penetration_mm": palm_pen * 1e3,
                       "palm_overlap_mm": palm_raw * 1e3 if math.isfinite(palm_raw) else None,
                       "unserved": unserved, "fingers": per,
                       **({"untracked": leader.frames[k]["untracked"]} if "untracked" in leader.frames[k] else {}),
                       **({"scale_margin": sm} if margin else {}),
                       **({"disturbance_margin": dm} if disturbance else {}),
                       **({"apply_limit": al} if apply else {}),
                       **({"actuator_dt_s": dt} if link else {}),
                       **({"wrist_rad": wq} if wq is not None else {})})
    return {"hand": hand_key, "hand_name": hand.spec["name"],
            "leader": leader_json["id"], "placement_cost_mm": cost * 1e3,
            "resid_tol": RESID_TOL if resid_tol is None else resid_tol,
            "settings": {"middle": middle, "ik_iters": IK_ITERS,
                         "place": {**PLACE_FINE, **place}, "wrist": wrist,
                         "respace": respace, "reused_placement": placement is not None,
                         "strength": "input columns, one-sided where the model says (23 Oct)",
                         "joint_stiffness_Nm_per_rad": kq, "deflect_max_m": DEFLECT_MAX_M,
                         "scale_margin": margin, "disturbance_margin": disturbance,
                         "apply_limit": apply, "apply_limit_free": apply_free,
                         "inertia": inertia,
                         **({"fit_half_s": FIT_HALF_S} if inertia else {}),
                         "rate_limit_per_s": rate_limit, "lag_s": lag_s,
                         # the task's allowed overlap, as used (module
                         # docstring, "Allowed overlap")
                         "allowed_overlap_m": a_c,
                         "other_contact": {"allowed": allow["other_allowed"],
                                           "max_overlap_m": a_o if allow["other_allowed"] else None,
                                           **({} if allow["other_allowed"]
                                              else {"clearance_m": -a_o})},
                         "overlap_source": allow["source"],
                         "push_out": {"first": list(PUSH_CLEAR),
                                      "second": None if PUSH_USE is None else list(PUSH_USE)},
                         **({"trajectory": True} if trajectory else {}),
                         # off (no joint stiffness in the model): no key, so
                         # earlier outputs reproduce exactly
                         **({"joint_springs": "from the model: k (q - springref) "
                             "at each solved pose, on the joint balance"} if sprung else {})},
            **({"caveat": hand.spec["caveat"]} if hand.spec.get("caveat") else {}),
            "assignment": {hand.fingers[a].label: int(assign[a])
                           for a in range(len(hand.fingers))},
            "respace_deg": np.degrees(leader.phi).tolist(),
            "placement": {"rotation_matrix": Rw.tolist(), "translation_m": t.tolist()},
            "frames": frames,
            # off: no key at all, so earlier outputs reproduce exactly
            **({"trajectory": tj.check(recs, rate_limit, lag_s)} if trajectory else {})}


def _finger_columns(hand, finger, ik, M, lim, Jf, p_contact, kq):
    """(Tr, C): Tr @ f is the reduced joint torque a contact force f needs;
    the finger can supply C @ x for 0 <= x <= 1, one column per input sense
    (a muscle pulls only; a motor with a two-sided range gives two) and two
    per free joint for passive stiffness kq within DEFLECT_MAX_M of contact
    motion. A hinge turns by DEFLECT_MAX_M / arm to move the contact that
    far, so its column is kq DEFLECT_MAX_M / arm (N.m); a slide moves the
    contact by its own travel, with no lever arm, so its column is the
    force kq DEFLECT_MAX_M along its axis (N; kq read as N/m on a slide, as
    MuJoCo reads joint stiffness). Before 28 Oct a slide was given a lever
    arm too (issue 21)."""
    E = _free_map(hand, finger, ik)
    B = M @ E
    Tr = E.T @ Jf.T
    cols = []
    for a, (lo, hi) in enumerate(lim):
        if hi > 0:
            cols.append(B[a] * hi)
        if lo < 0:
            cols.append(B[a] * lo)
    if kq > 0:
        slide = int(_mj().mjtJoint.mjJNT_SLIDE)
        for c_, j in enumerate(ik.free):
            if int(hand.model.jnt_type[j]) == slide:
                t = kq * DEFLECT_MAX_M
            else:
                arm = float(np.linalg.norm(p_contact - ik.d.xanchor[j]))
                t = kq * DEFLECT_MAX_M / max(arm, 1e-3)
            e = np.zeros(len(ik.free))
            e[c_] = t
            cols += [e, -e]
    return Tr, np.array(cols).T


def _free_map(hand, finger, ik):
    """E (finger.joints x free joints): how each joint moves with the free
    ones at the pose in ik.d, couplings included; E.T maps joint torque to
    the free joints."""
    m = hand.model
    nj = len(finger.joints)
    E = np.zeros((nj, len(ik.free)))
    for c_, j in enumerate(ik.free):
        E[finger.joints.index(j), c_] = 1.0
    for dep, (master, cf) in ik.dep.items():
        if master in ik.free:
            x = ik.d.qpos[m.jnt_qposadr[master]]
            E[finger.joints.index(dep), ik.free.index(master)] = \
                cf[1] + 2 * cf[2] * x + 3 * cf[3] * x ** 2 + 4 * cf[4] * x ** 3
    return E


def _local_derivs(dt, Q):
    """First and second derivative at dt = 0 of the quadratic fitted to Q
    (rows) at times dt: tools/import_demo_leader.py's local_fit, with the
    first derivative kept too."""
    A = np.column_stack([np.ones(len(dt)), dt, dt ** 2])
    coef = np.linalg.lstsq(A, Q, rcond=None)[0]
    return coef[1], 2 * coef[2]


def _window_q(ik, leader, k, i, Rw, t, body, q_k, half=None):
    """(dt, Q): times from frame k, and full joint positions, of this finger
    following station i through the leader frames within +/- half (default
    FIT_HALF_S) of frame k while the station stays made. Each frame is
    solved with the link frame k chose, starting from its neighbour nearer
    k, so the path stays on frame k's solution (a fresh solve per frame can
    jump to another one)."""
    half = FIT_HALF_S if half is None else half
    F, m = leader.frames, ik.hand.model
    tk = float(F[k]["t"])
    qs = {k: np.asarray(q_k, float)}
    for step in (1, -1):
        q, j = qs[k], k + step
        while 0 <= j < len(F) and abs(float(F[j]["t"]) - tk) <= half + 1e-9 \
                and i in leader.made(j):
            p, _, cc = leader.station(j, i, Rw, t)
            q = ik.solve_tangent(body, p, cc, leader.R_obj, q0=q)[0]
            qs[j] = q
            j += step
    js = sorted(qs)
    dt = np.array([float(F[j]["t"]) - tk for j in js])
    Q = np.array([ik.hand.apply_coupling(_q_full(m, ik, qs[j])) for j in js])
    return dt, Q


def _joint_dynamics(ik, q, qd, qdd):
    """(motion, weight): joint torques (every dof) the hand needs at full
    pose q to move with velocity qd and acceleration qdd, M(q) qdd +
    C(q, qd) qd, from MuJoCo's recursive Newton-Euler (mj_rne at (q, qd,
    qdd) less mj_rne at (q, 0, 0)); and its weight alone, mj_rne at
    (q, 0, 0). Contacts and passive forces are not in mj_rne."""
    mj, m, d = ik.mj, ik.hand.model, ik.d
    out = []
    for v, acc in ((np.zeros(m.nv), np.zeros(m.nv)), (qd, qdd)):
        d.qpos[:] = q
        d.qvel[:] = v
        d.qacc[:] = acc
        mj.mj_kinematics(m, d)
        mj.mj_comPos(m, d)
        mj.mj_comVel(m, d)
        r = np.zeros(m.nv)
        mj.mj_rne(m, d, 1, r)
        out.append(r)
    d.qvel[:] = 0.0
    d.qacc[:] = 0.0
    return out[1] - out[0], out[0]


def _finger_dynamics(ik, leader, k, i, Rw, t, body, q_k):
    """The finger's own motion at frame k while it follows station i: its
    joint path over +/- FIT_HALF_S (_window_q), velocity and acceleration
    from a local quadratic fit, and the joint torques that motion needs
    (_joint_dynamics). The other fingers and the wrist are held still (with
    wrist=True, at frame k's angles), so their motion is not counted.
    Returns {n, step_max_rad, motion, weight} (torques on every dof), with
    motion None if the window has fewer than FIT_MIN_POINTS frames.
    step_max_rad is the largest change of any of the finger's joints
    between neighbouring frames of the window, to show where the path still
    jumps. Leaves ik.d at an arbitrary pose."""
    dt, Q = _window_q(ik, leader, k, i, Rw, t, body, q_k)
    dofs = ik.f.dof
    out = {"n": len(dt),
           "step_max_rad": float(np.abs(np.diff(Q[:, dofs], axis=0)).max()) if len(dt) > 1 else 0.0}
    q = Q[int(np.argmin(np.abs(dt)))]
    if len(dt) < FIT_MIN_POINTS:
        out["motion"] = None
        out["weight"] = _joint_dynamics(ik, q, np.zeros(len(q)), np.zeros(len(q)))[1]
        return out
    qd, qdd = _local_derivs(dt, Q)
    out["motion"], out["weight"] = _joint_dynamics(ik, q, qd, qdd)
    out["qd_max_rad_s"] = float(np.abs(qd[dofs]).max())
    out["qdd_max_rad_s2"] = float(np.abs(qdd[dofs]).max())
    return out


def _input_share(C_in, tau):
    """Per free joint, tau_j as a share of the largest torque the inputs can
    make at that joint in tau_j's sense (every input that helps at its
    limit); None where no input can. For a hand with one motor per joint
    this is tau over that motor's limit."""
    out = []
    for j, tj in enumerate(tau):
        row = C_in[j]
        cap_ = row[row > 0].sum() if tj > 0 else -row[row < 0].sum()
        out.append(None if cap_ <= 0 else float(abs(tj) / cap_))
    return out


def _inertia_item(hand, finger, ik, M, lim, Jf, pw, dyn):
    """(record, tau): the frame's inertia readout for one finger and the
    torque its motion adds on its free joints (None when not fitted), from
    _finger_dynamics' output; ik.d must be at the frame's pose."""
    E = _free_map(hand, finger, ik)
    C_in = _finger_columns(hand, finger, ik, M, lim, Jf, pw, 0.0)[1]
    g = E.T @ dyn["weight"][finger.dof]
    rec = {"window_n": dyn["n"], "window_step_max_rad": dyn["step_max_rad"],
           "weight_Nm": g.tolist(), "weight_share": _input_share(C_in, g)}
    if dyn["motion"] is None:
        rec["fitted"] = False
        return rec, None
    tau = E.T @ dyn["motion"][finger.dof]
    rec.update(fitted=True, motion_Nm=tau.tolist(), motion_share=_input_share(C_in, tau),
               qd_max_rad_s=dyn["qd_max_rad_s"], qdd_max_rad_s2=dyn["qdd_max_rad_s2"])
    return rec, tau


def model_springs(hand, finger):
    """(k, ref): the model's spring stiffness and rest position for each of
    finger.joints (MuJoCo jnt_stiffness, and qpos_spring, which holds the
    joint's springref), or None if none of them has a spring. k is N.m/rad
    for a hinge, N/m for a slide."""
    m = hand.model
    k = np.array([m.jnt_stiffness[j] for j in finger.joints], float)
    if not np.any(k > 0):
        return None
    ref = np.array([m.qpos_spring[m.jnt_qposadr[j]] for j in finger.joints], float)
    return k, ref


def _spring_item(hand, finger, ik, M, lim, Jf, pw, spr):
    """(record, tau): the torque the finger's inputs must supply to hold its
    joint springs at the pose in ik.d, k (q - ref), mapped to the free joints
    as the motion torque is (couplings pass a dependant's torque to its
    master), and its share of what the inputs can make there."""
    k, ref = spr
    q = np.array([ik.d.qpos[a] for a in finger.qadr], float)
    tau = _free_map(hand, finger, ik).T @ (k * (q - ref))
    C_in = _finger_columns(hand, finger, ik, M, lim, Jf, pw, 0.0)[1]
    return {"torque": tau.tolist(), "share": _input_share(C_in, tau)}, tau


def _joint_rhs(Js, taus, L):
    """Right-hand side of _torque_stack's joint-balance rows: -tau / L for
    each finger whose motion needs torque tau (C x = Tr F lambda + tau),
    zero for the rest."""
    parts = []
    for cols, tau in zip(Js, taus):
        if cols is None:
            continue
        parts.append(np.zeros(cols[0].shape[0]) if tau is None else -np.asarray(tau) / L)
    return np.concatenate(parts) if parts else np.zeros(0)


def _columns_hold(cols, fv, tau=None):
    """Can the finger hold contact force fv (and, with tau, also supply the
    torque tau its own motion needs)? (ok, unbalanced joint torque N.m)"""
    return _hold_solve(cols, fv, tau)[:2]


def _hold_solve(cols, fv, tau=None, box=None):
    """_columns_hold, also giving the input use x it found; with box (l, h),
    each column's x is held in [l, h] instead of [0, 1] (x = l + y, y in
    [0, h - l])."""
    Tr, C = cols
    t = Tr @ fv
    if tau is not None:
        t = t + tau
    n = C.shape[1]
    A = np.block([[C, np.zeros((C.shape[0], n))], [np.eye(n), np.eye(n)]])
    if box is None:
        b = np.concatenate([t, np.ones(n)])
        x, res = cap.solve(A, b)
        x = x[:n]
    else:
        l, h = box
        y, res = cap.solve(A, np.concatenate([t - C @ l, h - l]))
        x = l + y[:n]
    resid = float(np.linalg.norm(C @ x - t))
    return resid <= UNACT_TOL * float(np.linalg.norm(t)) + 1e-9, resid, x


def _input_senses(lim):
    """(actuator, its limit in that sense) for each input column of
    _finger_columns, in its order (hi, then lo)."""
    out = []
    for a, (lo, hi) in enumerate(lim):
        if hi > 0:
            out.append((a, float(hi)))
        if lo < 0:
            out.append((a, float(lo)))
    return out


def _net_torque(lim, x):
    """Each actuator's net torque for input use x (the passive stiffness
    columns after the inputs are ignored)."""
    u = np.zeros(len(lim))
    for j, (a, v) in enumerate(_input_senses(lim)):
        u[a] += v * x[j]
    return u


def _reach(lim, u_prev, dt, rate=None, lag=None):
    """(u_lo, u_hi) per actuator: the net torques reachable dt seconds after
    u_prev (None: 0), inside the actuator's range [lo, hi]. rate: change at
    most rate * max(hi, -lo) per second; lag: first-order lag with time
    constant lag towards any torque in [lo, hi], so u = u_prev e^(-dt/lag)
    + (1 - e^(-dt/lag)) u_cmd. Both given: both hold. u_prev is clipped into
    [lo, hi] first (the NNLS can overshoot its bound by its residual)."""
    lo, hi = np.asarray(lim, float)[:, 0], np.asarray(lim, float)[:, 1]
    u = np.zeros(len(lo)) if u_prev is None else np.clip(u_prev, lo, hi)
    u_lo, u_hi = lo.copy(), hi.copy()
    if rate is not None:
        r = rate * np.maximum(hi, -lo) * dt
        u_lo, u_hi = np.maximum(u_lo, u - r), np.minimum(u_hi, u + r)
    if lag is not None:
        e = math.exp(-dt / lag)
        u_lo = np.maximum(u_lo, u * e + (1 - e) * lo)
        u_hi = np.minimum(u_hi, u * e + (1 - e) * hi)
    return u_lo, u_hi


def _input_box(lim, ncols, u_lo, u_hi):
    """(l, h): bounds on each of a finger's ncols columns that give exactly
    net torques in [u_lo, u_hi] per actuator (a two-sided motor's pull
    column x+ and push column x-: x+ in [max(u_lo, 0), max(u_hi, 0)] / hi,
    x- in [max(-u_hi, 0), max(-u_lo, 0)] / -lo). Passive stiffness columns
    keep [0, 1]."""
    l, h = np.zeros(ncols), np.ones(ncols)
    for j, (a, v) in enumerate(_input_senses(lim)):
        if v > 0:
            l[j], h[j] = max(u_lo[a], 0.0) / v, max(u_hi[a], 0.0) / v
        else:
            l[j], h[j] = max(-u_hi[a], 0.0) / -v, max(-u_lo[a], 0.0) / -v
    return l, np.maximum(h, l)


def _split_stack(Js, fingers_on, x):
    """{finger index: its input use} from _torque_stack's input block x."""
    out, c0 = {}, 0
    for cols, a in zip(Js, fingers_on):
        if cols is None:
            continue
        n = cols[1].shape[1]
        out[a] = x[c0:c0 + n]
        c0 += n
    return out


def _stack_rhs(Js, taus, bounds, L):
    """(joint rows, input rows, l) of _torque_stack's right-hand side with
    each finger's inputs held in its box (l, h) (None: [0, 1]): x = l + y,
    so Tr F lambda - C y = C l - tau (over L) and y + slack = h - l."""
    jr, span, low = [], [], []
    for j, cols in enumerate(Js):
        if cols is None:
            continue
        C = cols[1]
        n = C.shape[1]
        l, h = (np.zeros(n), np.ones(n)) if bounds[j] is None else bounds[j]
        r = C @ l
        if taus is not None and taus[j] is not None:
            r = r - np.asarray(taus[j])
        jr.append(r / L)
        span.append(h - l)
        low.append(l)
    cat = lambda p: np.concatenate(p) if p else np.zeros(0)  # noqa: E731
    return cat(jr), cat(span), cat(low)


def _q_full(m, ik, qfree):
    q = np.array(ik.hand.base_q)
    q[ik.qadr] = qfree
    return q


def _feasible(A, b, tol):
    return _feasible_x(A, b, tol)[:2]


def _feasible_x(A, b, tol):
    """_feasible, also giving the solution of the solve that decided it."""
    x, res = cap.solve(A, b)
    if res > tol:
        x, res = cap.solve(A, b, cap.NNLS_RECHECK_MULT)
    return res <= tol, res, x


def _feasible_retry(A, b, tol):
    """_feasible, solved again with the rows in reverse order when the solve
    breaks down: numpy's least squares can fail to converge on a finite,
    well-conditioned matrix ("SVD did not converge"), and the vendored NNLS
    can step by 0/0 and return NaN. The reversed system has the same
    solutions. Raises if both orders break down. Used by the apply limit
    only, so the other measures are unchanged."""
    for AA, bb in ((A, b), (A[::-1], b[::-1])):
        try:
            ok, res = _feasible(AA, bb, tol)
        except np.linalg.LinAlgError:
            continue
        if np.isfinite(res):
            return ok, res
    raise np.linalg.LinAlgError("solve broke down in both row orders")


def _redistribute(pts, nrms, Js, c, W, mu, L, resid_tol=None, taus=None,
                  bounds=None, out=None):
    """Can the made, reachable contacts produce the leader's object wrench
    with forces of their own choosing (inside the cone, within torque
    limits), to within resid_tol (default RESID_TOL) of its norm? The W20
    construction (vendored generators, solver and stacking), with that
    tolerance in place of W20's exact bar. taus (one per Js entry, or None):
    torque each finger's own motion needs, which its inputs must also
    supply. bounds (one (l, h) or None per Js entry, or None): each input
    column's range in place of [0, 1] (see _stack_rhs). out (a dict, or
    None): given "x", the input use the torque stack's solve found, when it
    ran (pass or fail)."""
    tol_frac = RESID_TOL if resid_tol is None else resid_tol
    if not pts:
        return False, "no_contact"
    pts, nrms = np.array(pts), np.array(nrms)
    ws = cap.scale_wrench(W, L)
    tol = max(tol_frac * float(np.linalg.norm(ws)), cap.FEAS_REL)
    D = cap.direction_basis(pts, nrms, c, L)
    x, *_ = np.linalg.lstsq(D, ws, rcond=None)
    if np.linalg.norm(D @ x - ws) > tol:
        return False, "direction"
    Gs, cidx, Fc = cap.generators(pts, nrms, c, mu, L)
    if not _feasible(Gs, ws, tol)[0]:
        return False, "friction"
    if all(c_ is None for c_ in Js):
        return True, None
    Mx, nt, nx = _torque_stack(Gs, cidx, Fc, Js, L)
    if bounds is None:
        jr = np.zeros(nt) if taus is None else _joint_rhs(Js, taus, L)
        span, low = np.ones(nx), np.zeros(nx)
    else:
        jr, span, low = _stack_rhs(Js, taus, bounds, L)
    rhs = np.concatenate([ws, jr, span])
    ok, _, xs = _feasible_x(Mx, rhs, tol)
    if out is not None:
        ng = Gs.shape[1]
        out["x"] = low + xs[ng:ng + nx]
    return (True, None) if ok else (False, "torque")


def _torque_stack(Gs, cidx, Fc, Js, L):
    """(Mx, nt, nx): the stacked system for the torque test, with the object
    wrench as the first 6 rows of its right-hand side."""
    # unknowns: cone weights (Gs columns), each finger's input use x in
    # [0, 1] (its C columns), slacks; rows: the object wrench, each finger's
    # joint balance Tr F lambda = C x (N.m / L), and x + slack = 1
    T_rows, C_blocks = [], []
    for ci, cols in enumerate(Js):
        if cols is None:
            continue
        Tr, C = cols
        Tb = np.zeros((Tr.shape[0], Gs.shape[1]))
        for e in np.where(cidx == ci)[0]:
            Tb[:, e] = Tr @ Fc[:, e]
        T_rows.append(Tb / L)
        C_blocks.append(C / L)
    nx = sum(C.shape[1] for C in C_blocks)
    nt = sum(T.shape[0] for T in T_rows)
    Cb = np.zeros((nt, nx))
    r0 = c0 = 0
    for C in C_blocks:
        Cb[r0:r0 + C.shape[0], c0:c0 + C.shape[1]] = C
        r0, c0 = r0 + C.shape[0], c0 + C.shape[1]
    ng = Gs.shape[1]
    Mx = np.block([[Gs, np.zeros((6, 2 * nx))],
                   [np.vstack(T_rows), -Cb, np.zeros((nt, nx))],
                   [np.zeros((nx, ng)), np.eye(nx), np.eye(nx)]])
    return Mx, nt, nx


def _scale_margin(pts, nrms, Js, c, W, mu, L, resid_tol=None, taus=None,
                  bounds=None):
    """Largest factor s by which the leader's object wrench could grow and
    the made contacts still produce it: the system and tolerance of
    _redistribute, the tolerance growing with the wrench; above s = 1 the
    joint-balance and input-bound rows are held to their s = 1 slack (see
    held). The search of the vendored W20 cap.scale_margin, whose bar is
    exact and so needs no such weighting: every point of cap.SCALE_GRID, then
    bisection to cap.BISECT_TOL above the largest feasible one; if the top of
    the grid holds, doubling up to MARGIN_MAX first and bisecting only to
    MARGIN_REL of the margin. 1.0 is a grid
    point, so when the feasible points form one run, margin >= 1 exactly when
    _redistribute says ok. Returns {margin, capped, contiguous}; margin 0 if
    the direction or friction test fails (neither depends on scale); None
    with no contact or no torque checked (the cones alone set no scale).
    With kq = 0 a failed zero-torque row on an unactuated joint fails at
    every scale, so such frames read 0: the margin grades only frames that
    pass that rule. taus as _redistribute: the fingers' own motion does not
    grow with s, so its torque stays as it is at every scale. bounds as
    _redistribute: the inputs' box stays as it is at every scale too."""
    tol_frac = RESID_TOL if resid_tol is None else resid_tol
    if not pts or all(c_ is None for c_ in Js):
        return None
    pts, nrms = np.array(pts), np.array(nrms)
    ws = cap.scale_wrench(W, L)
    wn = float(np.linalg.norm(ws))
    none = {"margin": 0.0, "capped": False, "contiguous": True}
    tol = max(tol_frac * wn, cap.FEAS_REL)
    D = cap.direction_basis(pts, nrms, c, L)
    x, *_ = np.linalg.lstsq(D, ws, rcond=None)
    if np.linalg.norm(D @ x - ws) > tol:
        return none
    Gs, cidx, Fc = cap.generators(pts, nrms, c, mu, L)
    if not _feasible(Gs, ws, tol)[0]:
        return none
    Mx, nt, nx = _torque_stack(Gs, cidx, Fc, Js, L)
    if bounds is None:
        jr = np.zeros(nt) if taus is None else _joint_rhs(Js, taus, L)
        span = np.ones(nx)
    else:
        jr, span, _ = _stack_rhs(Js, taus, bounds, L)

    ng = Gs.shape[1]

    def held(s):
        # above s = 1, every row divided by s and the cone weights taken per
        # unit s: the wrench rows keep their s = 1 form, the contact torque
        # grows by s, and the joint-balance and input-bound rows keep the
        # slack they have at s = 1. With one shared slack of 5 % of s times
        # the wrench, a large wrench would let the inputs run far past their
        # limits.
        if s <= 1.0:
            rhs = np.concatenate([s * ws, jr, span])
            return _feasible(Mx, rhs, max(tol_frac * s * wn, cap.FEAS_REL))[0]
        Ms = Mx.copy()
        Ms[6:6 + nt, :ng] *= s
        rhs = np.concatenate([ws, jr, span])
        return _feasible(Ms, rhs, max(tol_frac * wn, cap.FEAS_REL))[0]

    grid = np.array([held(s) for s in cap.SCALE_GRID])
    idx = np.where(grid)[0]
    if len(idx) == 0:
        return none
    out = {"margin": 0.0, "capped": False,
           "contiguous": bool(idx[-1] - idx[0] + 1 == len(idx))}
    top = idx[-1]
    if top == len(cap.SCALE_GRID) - 1:
        # past the W20 grid: double until it fails or MARGIN_MAX holds
        a = float(cap.SCALE_GRID[top])
        while a < MARGIN_MAX and held(2 * a):
            a *= 2
        if a >= MARGIN_MAX:
            out.update(margin=a, capped=True)
            return out
        b, step = 2 * a, MARGIN_REL * a
    else:
        a, b = float(cap.SCALE_GRID[top]), float(cap.SCALE_GRID[top + 1])
        step = cap.BISECT_TOL
    while b - a > step:
        m = 0.5 * (a + b)
        if held(m):
            a = m
        else:
            b = m
    out["margin"] = a
    return out


def _disturbance_margin(pts, nrms, Js, c, W, mu, L, resid_tol=None, taus=None,
                        bounds=None):
    """Smallest extra object wrench, added to the leader's and in any of
    DISTURB_DIRS, that the made contacts can no longer produce with the
    leader's one: the worst-direction margin of the largest-ball measure
    (Ferrari & Canny 1992), with the torque limits of _redistribute (the
    torque-limited variant; see docs/LITERATURE_REVIEW.md). The system of
    _torque_stack, unchanged, with right-hand side ws + e*|ws|*d and the
    tolerance of _redistribute, tol_frac*|ws|, held fixed (it does not grow
    with e), so e = 0 is exactly _redistribute's torque test. The wrenches
    that system can produce form a convex set, so along each direction the
    feasible e form one run from 0: each direction is bisected (relative
    MARGIN_REL, floor cap.BISECT_TOL), and a direction that holds at the
    smallest e so far is skipped, as it cannot lower the minimum.

    Returns {margin, leader_wrench, direction, capped}: margin is e as a
    share of |ws|, the scaled leader wrench norm (torque / L, as W20), which
    is leader_wrench; direction is the worst one ("+fx" a push along +x,
    "-tz" a twist about -z, ...), None if none breaks it; capped if every
    direction holds at MARGIN_MAX. Margin 0 if the leader's own load already
    fails (direction, friction or torque). None with no contact or no torque
    checked: without input limits the cones are unbounded, so a grip that
    encloses the object would resist any disturbance. The margin runs
    continuously from 0: a frame that only just passes at the leader's load
    (unbalanced share near tol_frac) reads near 0. The ball's shape depends
    on the L scaling of torque against force. taus and bounds as
    _redistribute."""
    tol_frac = RESID_TOL if resid_tol is None else resid_tol
    if not pts or all(c_ is None for c_ in Js):
        return None
    pts, nrms = np.array(pts), np.array(nrms)
    ws = cap.scale_wrench(W, L)
    wn = float(np.linalg.norm(ws))
    out = {"margin": 0.0, "leader_wrench": wn, "direction": None, "capped": False}
    tol = max(tol_frac * wn, cap.FEAS_REL)
    D = cap.direction_basis(pts, nrms, c, L)
    x, *_ = np.linalg.lstsq(D, ws, rcond=None)
    if np.linalg.norm(D @ x - ws) > tol:
        return out
    Gs, cidx, Fc = cap.generators(pts, nrms, c, mu, L)
    if not _feasible(Gs, ws, tol)[0]:
        return out
    Mx, nt, nx = _torque_stack(Gs, cidx, Fc, Js, L)
    if bounds is None:
        jr = np.zeros(nt) if taus is None else _joint_rhs(Js, taus, L)
        tail = np.concatenate([jr, np.ones(nx)])
    else:
        tail = np.concatenate(_stack_rhs(Js, taus, bounds, L)[:2])

    def held(i, sg, e):
        w = ws.copy()
        w[i] += sg * e * wn
        return _feasible(Mx, np.concatenate([w, tail]), tol)[0]

    if not held(0, 1.0, 0.0):
        return out
    best, worst = None, None
    for i, sg in DISTURB_DIRS:
        if best is None:
            a, b = 0.0, 1.0
            if held(i, sg, 1.0):
                a = 1.0
                while a < MARGIN_MAX and held(i, sg, 2 * a):
                    a *= 2
                if a >= MARGIN_MAX:
                    best = a
                    continue
                b = 2 * a
        else:
            if held(i, sg, best):
                continue
            a, b = 0.0, best
        while b - a > max(MARGIN_REL * a, cap.BISECT_TOL):
            m = 0.5 * (a + b)
            if held(i, sg, m):
                a = m
            else:
                b = m
        best, worst = a, ("+" if sg > 0 else "-") + DISTURB_LABELS[i]
    out.update(margin=best, direction=worst, capped=worst is None)
    return out


def _apply_limit(pts, nrms, Js, c, mu, L, resid_tol=None, free=False, taus=None):
    """Largest push and twist the made contacts can put on the object, with
    no leader load, along each of DISTURB_DIRS: the torque-limited wrench set
    of the grasp-quality literature (Jeong and Cheong 2012; Zheng and Yamane
    2013; both known to us only through Roa and Suarez 2015), read along the
    12 signed axes. Js must hold the input columns only (kq = 0): a passive
    spring resists a load but cannot push on its own. Set against the
    disturbance margin (what the grip can resist), this separates holding
    by stiffness from driving by actuation.

    The system of _torque_stack with right-hand side [e*d, 0, 1], every row
    divided by e and the cone weights taken per unit e, as _scale_margin
    does above s = 1: the wrench rows read Gs lambda = d, the contact part
    of the joint-balance rows grows by e, and the input-bound rows keep their
    form. The tolerance is tol_frac (RESID_TOL, proposed) on that system: the
    wrench may be missed by tol_frac of e, and the joint-balance and
    input-bound rows keep a fixed slack of tol_frac (as the other margins
    have for a 1 N leader wrench), so the inputs cannot run past their
    limits as e grows. A tolerance on the leader's wrench norm would tie a
    leader-free quantity to the leader. At e = 0 the test is the friction
    span alone: a direction the cones cannot push at all reads 0. Feasible
    e form one run from 0 (a solution at e, with the inputs scaled down,
    serves any smaller e), so each direction is searched as in
    _disturbance_margin: APPLY_FIRST_N, then doubling up to MARGIN_MAX, or
    bisection below it, to MARGIN_REL of the value (floor cap.BISECT_TOL).

    That reading asks for a pure push or twist: the other five components
    must be zero. With free, a second reading (other_components_free,
    proposed) drops those five wrench rows, so it asks how hard the hand
    can push along the axis with whatever other push and twist come with
    it. It is the
    projection of the torque-limited wrench set onto the axis, not the
    largest motion along it, since the extra components act on the object
    too. With fewer rows it is never below the pure reading (to within the
    bisection's resolution). Every direction it opens must be bisected from
    tens of newtons, and each failed solve runs to the recheck cap, so it
    costs tens to hundreds of times the first reading.

    Returns {push_N, twist_Nm, capped, L_m}, plus other_components_free
    with free: push_N and twist_Nm map "+x", "-x", ... to the largest push
    (N) along, and twist (N.m) about, that
    axis (twist = e * L, as the scaled wrench divides torque by L); capped
    lists the directions ("+fx", "-tz", ...) that still hold at MARGIN_MAX;
    other_components_free holds the same three for the second reading.
    None with no contact or no torque checked. A finger with no torque check
    is unbounded, so the directions it can push read capped. Unlike the
    leader-load test, a finger whose unactuated joints would carry torque
    does not fail here: it supplies only forces that load no such joint, or
    none. Dividing these by the object's mass and inertia gives the largest
    acceleration the hand can give it, to compare with the leader's motion
    (not built).

    taus (one per Js entry, or None): the torque each finger's joint springs
    need at its pose (the model's springs; see _spring_item). The inputs
    must hold it before they can push, so it is a load on the joint-balance
    rows, as in _redistribute, and is not scaled by e; with it the e = 0
    test also asks whether the inputs can hold the springs at all. None
    (every hand without joint springs): the rows are zero, as before."""
    tol_frac = RESID_TOL if resid_tol is None else resid_tol
    if not pts or all(c_ is None for c_ in Js):
        return None
    pts, nrms = np.array(pts), np.array(nrms)
    Gs, cidx, Fc = cap.generators(pts, nrms, c, mu, L)
    Mx, nt, nx = _torque_stack(Gs, cidx, Fc, Js, L)
    ng = Gs.shape[1]
    tol = max(tol_frac, cap.FEAS_REL)
    jr = np.zeros(nt) if taus is None else _joint_rhs(Js, taus, L)
    tail = np.concatenate([jr, np.ones(nx)])

    def held(i, sg, e, free):
        d = np.zeros(6)
        d[i] = sg
        w = [i] if free else list(range(6))      # wrench rows kept
        if e == 0.0 and taus is None:
            # the joint rows hold with no input use, so only the cones remain
            return _feasible_retry(Gs[w], d[w], tol)[0]
        Ms = Mx.copy()
        Ms[6:6 + nt, :ng] *= e
        keep = w + list(range(6, Mx.shape[0]))
        return _feasible_retry(Ms[keep], np.concatenate([d, tail])[keep], tol)[0]

    def search(free):
        res = {"push_N": {}, "twist_Nm": {}, "capped": []}
        for i, sg in DISTURB_DIRS:
            lab = ("+" if sg > 0 else "-") + DISTURB_LABELS[i]
            a = 0.0
            if held(i, sg, 0.0, free):
                b = APPLY_FIRST_N
                if held(i, sg, b, free):
                    a = b
                    while a < MARGIN_MAX and held(i, sg, 2 * a, free):
                        a *= 2
                    b = 2 * a
                if a >= MARGIN_MAX:
                    res["capped"].append(lab)
                else:
                    while b - a > max(MARGIN_REL * a, cap.BISECT_TOL):
                        m = 0.5 * (a + b)
                        if held(i, sg, m, free):
                            a = m
                        else:
                            b = m
            if i < 3:
                res["push_N"][lab[0] + "xyz"[i]] = a
            else:
                res["twist_Nm"][lab[0] + "xyz"[i - 3]] = a * L
        return res

    out = search(False)
    if free:
        out["other_components_free"] = search(True)
    out["L_m"] = float(L)
    return out


def reach_bound(hand_key, leader_json, result, middle=False, n=6000, seed=0,
                refine=3, iters=200):
    """Geometry only: with the placement and assignment of `result`, how
    close can any palm-side pad point of each finger get to its station's
    contact path? Ignores facing, force and collisions, so it is a floor on
    how short the hand falls. Joints are sampled within their ranges
    (couplings applied); the `refine` nearest samples to each target then
    seed a position-only solve of `iters` iterations.
    Returns {finger: {median_mm, p90_mm, max_mm}}."""
    global FACE_WEIGHT_M, IK_ITERS
    hand = Hand(hand_key)
    leader = Leader(leader_json)
    Rw = np.array(result["placement"]["rotation_matrix"])
    t = np.array(result["placement"]["translation_m"])
    if result.get("respace_deg"):
        leader.phi = np.radians(result["respace_deg"])
    m = hand.model
    rng = np.random.default_rng(seed)
    out = {}
    saved = FACE_WEIGHT_M, IK_ITERS
    FACE_WEIGHT_M, IK_ITERS = 0.0, iters
    try:
        for f in hand.fingers:
            ik = FingerIK(hand, f)
            cands = pad_candidates(hand, f, middle)
            pts, idx = [], []
            for _ in range(max(n // max(len(cands), 1), 1)):
                qf = rng.uniform(ik.rng[:, 0], ik.rng[:, 1])
                q = np.array(hand.base_q)
                q[ik.qadr] = qf
                ik.d.qpos[:] = hand.apply_coupling(q)
                ik.mj.mj_kinematics(m, ik.d)
                for c, (b, pl, _nl) in enumerate(cands):
                    pts.append(ik.d.xpos[b] + ik.d.xmat[b].reshape(3, 3) @ pl)
                    idx.append((c, qf))
            pts = np.array(pts)
            i = result["assignment"][f.label]
            dist = []
            for k in range(len(leader.frames)):
                if i not in leader.made(k):
                    continue
                p, nrm, _ = leader.station(k, i, Rw, t)
                dd = np.linalg.norm(pts - p, axis=1)
                best = float(dd.min())
                for j in np.argsort(dd)[:refine]:
                    c, qf = idx[j]
                    b, pl, nl = cands[c]
                    best = min(best, ik.solve(b, pl, nl, p, nrm, qf)[1])
                dist.append(best)
            dist = np.array(dist) * 1e3
            out[f.label] = ({"median_mm": float(np.median(dist)),
                             "p90_mm": float(np.percentile(dist, 90)),
                             "max_mm": float(dist.max())} if len(dist) else None)
    finally:
        FACE_WEIGHT_M, IK_ITERS = saved
    return out


def reach_only(hand_key, leader_json, middle=True, seed=0, place=None):
    """Is there any placement at all from which every finger reaches its
    station? The placement search scored on reach alone (no facing in the
    cost or the IK), then reach_bound at that placement. Returns
    (mean reach error m, assignment, {finger: floor stats})."""
    global FACE_WEIGHT_M
    hand = Hand(hand_key)
    leader = Leader(leader_json)
    cands = [pad_candidates(hand, f, middle) for f in hand.fingers]
    iks = [FingerIK(hand, f, overlap_allowance(leader_json)) for f in hand.fingers]
    saved = FACE_WEIGHT_M
    FACE_WEIGHT_M = 0.0
    try:
        cost, Rw, t, assign, _ = placement_search(hand, leader, cands, iks, seed=seed,
                                                  face_weight=0.0, **(place or {}))
    finally:
        FACE_WEIGHT_M = saved
    res = {"placement": {"rotation_matrix": Rw.tolist(), "translation_m": t.tolist()},
           "assignment": {hand.fingers[a].label: int(assign[a]) for a in range(len(hand.fingers))}}
    return cost, res["assignment"], reach_bound(hand_key, leader_json, res, middle=middle), res
