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
  contact     anywhere on the last or middle phalanx except the nail (back)
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

Penetration (26 Oct, George: "the fingers go inside the ball ... that is of
course non-physical"): no part of a touching finger, nor the palm, may sit
inside the object. The IK pushes any link point that sinks in back out; a
finger is green only if it sinks in at most PEN_GREEN_M (pad compression),
amber up to PEN_AMBER_M, red ("penetrates") beyond; the palm likewise, per
frame ("palm penetrates"), and the placement search pays for both. Fingers
are not checked against each other, and lifted fingers are not posed.

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
PEN_GREEN_M = 0.001        # proposed: a pad may compress this far into the object
PEN_AMBER_M = 0.003        # proposed: "close"
PEN_WEIGHT = 1.0           # IK weight of the push-out row (as position)
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
        if t == mj.mjtGeom.mjGEOM_MESH:
            mid = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        else:
            # surface samples of the primitive, in its own frame
            u = np.random.default_rng(0).standard_normal((600, 3))
            u /= np.linalg.norm(u, axis=1, keepdims=True)
            if t == mj.mjtGeom.mjGEOM_SPHERE:
                v = s[0] * u
            elif t == mj.mjtGeom.mjGEOM_CAPSULE:
                # on the surface: the cylinder side, then the two caps (until
                # 26 Oct this mixed in points inside the capsule)
                ang = np.random.default_rng(1).uniform(0, 2 * np.pi, 300)
                z = np.random.default_rng(2).uniform(-s[1], s[1], 300)
                side = np.column_stack([s[0] * np.cos(ang), s[0] * np.sin(ang), z])
                cap_ = s[0] * u[:300] + np.outer(np.sign(u[:300, 2]), [0, 0, s[1]])
                v = np.vstack([side, cap_])
            elif t == mj.mjtGeom.mjGEOM_CYLINDER:
                ang = np.linspace(0, 2 * np.pi, 600)
                z = np.random.default_rng(1).uniform(-s[1], s[1], 600)
                v = np.column_stack([s[0] * np.cos(ang), s[0] * np.sin(ang), z])
            elif t == mj.mjtGeom.mjGEOM_BOX:
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
        t, sz, pos = m.geom_type[g], m.geom_size[g], np.array(m.geom_pos[g])
        if t == mj.mjtGeom.mjGEOM_CAPSULE:
            ax = Rm[:, 2] * sz[1]
            out.append(("seg", pos - ax, pos + ax, float(sz[0])))
        elif t == mj.mjtGeom.mjGEOM_SPHERE:
            out.append(("seg", pos, pos, float(sz[0])))
    V = None
    others = [g for g in range(m.ngeom) if m.geom_bodyid[g] == body
              and m.geom_type[g] not in (mj.mjtGeom.mjGEOM_CAPSULE, mj.mjtGeom.mjGEOM_SPHERE)
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
    def __init__(self, hand, finger):
        mj = _mj()
        self.mj, self.hand, self.f = mj, hand, finger
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

    def penetration(self, ball):
        """(depth m, body, point, outward) of the finger's deepest point
        inside ball = (centre, radius), at the pose in self.d."""
        return _deepest(self.d, self.shapes, np.asarray(ball[0], float), float(ball[1]))

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
                        _, r = ik.best(cands[a][::pad_step], p, nrm, starts=[None],
                                       ball=(cc, leader.R_obj))
                        # reach error plus facing, weighted as in the IK
                        # (FACE_WEIGHT_M metres per radian), so a finger
                        # touching with its nail is not a good placement,
                        # plus any sinking into the object beyond a pad's give
                        fw = FACE_WEIGHT_M if face_weight is None else face_weight
                        res.append(r[1] + fw * math.radians(r[2]) + max(0.0, r[5] - PEN_GREEN_M))
                    C[a, i] = np.mean(res) if res else 1.0
            assign, cost = best_assignment(C)
            # the palm must not sit inside the object either
            cost += float(np.mean([max(0.0, palm_penetration(hand, leader.station(k, 0, Rw, t)[2],
                                                             leader.R_obj) - PEN_GREEN_M) for k in ks]))
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
           apply=False):
    """place: placement_search settings (n_frames, n_samples, iters,
    pad_step; default PLACE_FINE); ik_iters: IK iterations (default
    IK_ITERS); wrist: move the shared (wrist) joints at every moment;
    respace: move stations round the ring to suit the hand; placement: a
    previous result's "placement" and "assignment" (and "respace_deg"), to
    re-evaluate without searching again; margin: give each frame a
    scale_margin (see _scale_margin); disturbance: give each frame a
    disturbance_margin (see _disturbance_margin); apply: give each frame an
    apply_limit (see _apply_limit)."""
    global IK_ITERS
    old_iters = IK_ITERS
    if ik_iters is not None:
        IK_ITERS = int(ik_iters)
    try:
        return _follow(hand_key, leader_json, middle, n_eval, seed, resid_tol,
                       place or {}, wrist, respace, placement,
                       JOINT_STIFFNESS if joint_stiffness is None else float(joint_stiffness),
                       margin, disturbance, apply)
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
            _, r = ik.best(cands[a][::2], p, nrm, starts=[None], ball=(cc, leader.R_obj))
            c += r[1] + FACE_WEIGHT_M * math.radians(r[2]) + max(0.0, r[5] - PEN_GREEN_M)
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
            respace, placement, kq, margin=True, disturbance=False, apply=False):
    hand = Hand(hand_key)
    leader = Leader(leader_json)
    cands = [pad_candidates(hand, f, middle) for f in hand.fingers]
    iks = [FingerIK(hand, f) for f in hand.fingers]
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
    mj = _mj()
    m = hand.model
    ks = range(len(leader.frames)) if n_eval is None else \
        np.linspace(0, len(leader.frames) - 1, n_eval).astype(int)
    frames = []
    for k in ks:
        made = leader.made(k)
        wq = _wrist_search(hand, leader, cands, iks, assign, k, Rw, t, made) if wrist else None
        _, _, c = leader.station(k, 0, Rw, t)
        W = np.zeros(6)
        for i in made:
            p, _, _ = leader.station(k, i, Rw, t, raw=True)
            fv = leader.force(k, i, Rw, raw=True)
            W += np.concatenate([fv, np.cross(p - c, fv)])
        per, pts, nrms, Js, Js_act, fingers_on = [], [], [], [], [], []
        for a, f in enumerate(hand.fingers):
            i = assign[a]
            if i not in made:
                per.append({"finger": f.label, "station": i, "state": "off"})
                continue
            p, nrm, _ = leader.station(k, i, Rw, t)
            (b, pl, nl), (q, res, face, pw, nw, pen) = iks[a].best(
                cands[a], p, nrm, weights=(FACE_WEIGHT_M, FACE_WEIGHT_M / 10),
                ball=(c, leader.R_obj))
            near = res <= REACH_AMBER_M and face <= FACE_AMBER_DEG and pen <= PEN_AMBER_M
            exact = res <= REACH_GREEN_M and face <= FACE_GREEN_DEG and pen <= PEN_GREEN_M
            item = {"finger": f.label, "station": i, "reach_mm": res * 1e3,
                    "facing_deg": face, "phalanx": "distal" if b == f.tip_body else "middle",
                    # the solved pose, for rendering (tools/render_follow.py)
                    "q": [float(x) for x in q], "target": p.tolist(), "pad_world": pw.tolist(),
                    "pad_normal": nw.tolist(), "penetration_mm": max(0.0, pen) * 1e3}
            if not near:
                item["state"] = "red"
                item["reason"] = ("reach" if res > REACH_AMBER_M else
                                  "facing" if face > FACE_AMBER_DEG else "penetrates")
                per.append(item)
                continue
            # actual contact: the pad point projected onto the object
            v = pw - c
            cp = c + leader.R_obj * v / np.linalg.norm(v)
            n_in = (c - cp) / np.linalg.norm(c - cp)
            iks[a].d.qpos[:] = hand.apply_coupling(_q_full(m, iks[a], q))
            mj.mj_kinematics(m, iks[a].d)
            mj.mj_comPos(m, iks[a].d)
            jacp = np.zeros((3, m.nv))
            mj.mj_jac(m, iks[a].d, jacp, None, pw, b)
            Jf = jacp[:, f.dof]
            fv = leader.force(k, i, Rw)
            in_cone = fv @ n_in >= math.cos(math.atan(mu)) * np.linalg.norm(fv) - 1e-12
            M, lim = lims[a]
            torque_ok = True
            cols = None
            if M is not None:
                cols = _finger_columns(hand, f, iks[a], M, lim, Jf, pw, kq)
                torque_ok, un = _columns_hold(cols, fv)
                item["unbalanced_Nm"] = un
            item["state"] = "green" if (exact and in_cone and torque_ok) else "amber?"
            item["torque_checked"] = M is not None
            per.append(item)
            pts.append(cp)
            nrms.append(n_in)
            Js.append(cols)
            # the apply limit counts inputs only: a spring resists but cannot push
            Js_act.append(_finger_columns(hand, f, iks[a], M, lim, Jf, pw, 0.0)
                          if apply and kq > 0 and M is not None else cols)
            fingers_on.append(a)
        unserved = [i for i in made if i not in assign]
        need_redistribute = unserved or leader.respaced or \
            any(x.get("state") == "amber?" for x in per)
        all_green = not unserved and all(x["state"] in ("green", "off") for x in per)
        verdict = "green"
        reason = None
        if any(x.get("state") == "red" for x in per):
            # a red finger's station load must also be carried by the others
            need_redistribute = True
        if need_redistribute:
            ok, reason = _redistribute(pts, nrms, Js, c, W, mu, L, resid_tol)
            verdict = ("green" if all_green else "amber") if ok else "red"
            for x in per:
                if x.get("state") == "amber?":
                    x["state"] = "amber" if ok else "red"
        palm_pen = max(0.0, palm_penetration(hand, c, leader.R_obj))
        if palm_pen > PEN_AMBER_M:
            verdict, reason = "red", "palm penetrates"
        elif palm_pen > PEN_GREEN_M and verdict == "green":
            verdict = "amber"
        if margin:
            sm = _scale_margin(pts, nrms, Js, c, W, mu, L, resid_tol)
        if disturbance:
            dm = _disturbance_margin(pts, nrms, Js, c, W, mu, L, resid_tol)
        if apply:
            al = _apply_limit(pts, nrms, Js_act, c, mu, L, resid_tol)
        frames.append({"k": int(k), "verdict": verdict, "reason": reason,
                       "ball_center": c.tolist(), "palm_penetration_mm": palm_pen * 1e3,
                       "unserved": unserved, "fingers": per,
                       **({"scale_margin": sm} if margin else {}),
                       **({"disturbance_margin": dm} if disturbance else {}),
                       **({"apply_limit": al} if apply else {}),
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
                         "apply_limit": apply},
            "assignment": {hand.fingers[a].label: int(assign[a])
                           for a in range(len(hand.fingers))},
            "respace_deg": np.degrees(leader.phi).tolist(),
            "placement": {"rotation_matrix": Rw.tolist(), "translation_m": t.tolist()},
            "frames": frames}


def _finger_columns(hand, finger, ik, M, lim, Jf, p_contact, kq):
    """(Tr, C): Tr @ f is the reduced joint torque a contact force f needs;
    the finger can supply C @ x for 0 <= x <= 1, one column per input sense
    (a muscle pulls only; a motor with a two-sided range gives two) and two
    per free joint for passive stiffness kq within DEFLECT_MAX_M of contact
    motion."""
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
    B = M @ E
    Tr = E.T @ Jf.T
    cols = []
    for a, (lo, hi) in enumerate(lim):
        if hi > 0:
            cols.append(B[a] * hi)
        if lo < 0:
            cols.append(B[a] * lo)
    if kq > 0:
        for c_, j in enumerate(ik.free):
            arm = float(np.linalg.norm(p_contact - ik.d.xanchor[j]))
            t = kq * DEFLECT_MAX_M / max(arm, 1e-3)
            e = np.zeros(len(ik.free))
            e[c_] = t
            cols += [e, -e]
    return Tr, np.array(cols).T


def _columns_hold(cols, fv):
    """Can the finger hold contact force fv? (ok, unbalanced joint torque N.m)"""
    Tr, C = cols
    t = Tr @ fv
    n = C.shape[1]
    A = np.block([[C, np.zeros((C.shape[0], n))], [np.eye(n), np.eye(n)]])
    b = np.concatenate([t, np.ones(n)])
    x, res = cap.solve(A, b)
    resid = float(np.linalg.norm(C @ x[:n] - t))
    return resid <= UNACT_TOL * float(np.linalg.norm(t)) + 1e-9, resid


def _q_full(m, ik, qfree):
    q = np.array(ik.hand.base_q)
    q[ik.qadr] = qfree
    return q


def _feasible(A, b, tol):
    x, res = cap.solve(A, b)
    if res > tol:
        x, res = cap.solve(A, b, cap.NNLS_RECHECK_MULT)
    return res <= tol, res


def _redistribute(pts, nrms, Js, c, W, mu, L, resid_tol=None):
    """Can the made, reachable contacts produce the leader's object wrench
    with forces of their own choosing (inside the cone, within torque
    limits), to within resid_tol (default RESID_TOL) of its norm? The W20
    construction (vendored generators, solver and stacking), with that
    tolerance in place of W20's exact bar."""
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
    rhs = np.concatenate([ws, np.zeros(nt), np.ones(nx)])
    ok = _feasible(Mx, rhs, tol)[0]
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


def _scale_margin(pts, nrms, Js, c, W, mu, L, resid_tol=None):
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
    pass that rule."""
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

    ng = Gs.shape[1]

    def held(s):
        # above s = 1, every row divided by s and the cone weights taken per
        # unit s: the wrench rows keep their s = 1 form, the contact torque
        # grows by s, and the joint-balance and input-bound rows keep the
        # slack they have at s = 1. With one shared slack of 5 % of s times
        # the wrench, a large wrench would let the inputs run far past their
        # limits.
        if s <= 1.0:
            rhs = np.concatenate([s * ws, np.zeros(nt), np.ones(nx)])
            return _feasible(Mx, rhs, max(tol_frac * s * wn, cap.FEAS_REL))[0]
        Ms = Mx.copy()
        Ms[6:6 + nt, :ng] *= s
        rhs = np.concatenate([ws, np.zeros(nt), np.ones(nx)])
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


def _disturbance_margin(pts, nrms, Js, c, W, mu, L, resid_tol=None):
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
    on the L scaling of torque against force."""
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
    tail = np.concatenate([np.zeros(nt), np.ones(nx)])

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


def _apply_limit(pts, nrms, Js, c, mu, L, resid_tol=None):
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

    Returns {push_N, twist_Nm, capped, L_m}: push_N and twist_Nm map "+x",
    "-x", ... to the largest push (N) along, and twist (N.m) about, that
    axis (twist = e * L, as the scaled wrench divides torque by L); capped
    lists the directions ("+fx", "-tz", ...) that still hold at MARGIN_MAX.
    None with no contact or no torque checked. A finger with no torque check
    is unbounded, so the directions it can push read capped. Unlike the
    leader-load test, a finger whose unactuated joints would carry torque
    does not fail here: it supplies only forces that load no such joint, or
    none. Dividing these by the object's mass and inertia gives the largest
    acceleration the hand can give it, to compare with the leader's motion
    (not built)."""
    tol_frac = RESID_TOL if resid_tol is None else resid_tol
    if not pts or all(c_ is None for c_ in Js):
        return None
    pts, nrms = np.array(pts), np.array(nrms)
    Gs, cidx, Fc = cap.generators(pts, nrms, c, mu, L)
    Mx, nt, nx = _torque_stack(Gs, cidx, Fc, Js, L)
    ng = Gs.shape[1]
    tol = max(tol_frac, cap.FEAS_REL)
    tail = np.concatenate([np.zeros(nt), np.ones(nx)])

    def held(i, sg, e):
        d = np.zeros(6)
        d[i] = sg
        if e == 0.0:
            # the joint rows hold with no input use, so only the cones remain
            return _feasible(Gs, d, tol)[0]
        Ms = Mx.copy()
        Ms[6:6 + nt, :ng] *= e
        return _feasible(Ms, np.concatenate([d, tail]), tol)[0]

    out = {"push_N": {}, "twist_Nm": {}, "capped": [], "L_m": float(L)}
    for i, sg in DISTURB_DIRS:
        lab = ("+" if sg > 0 else "-") + DISTURB_LABELS[i]
        a = 0.0
        if held(i, sg, 0.0):
            b = APPLY_FIRST_N
            if held(i, sg, b):
                a = b
                while a < MARGIN_MAX and held(i, sg, 2 * a):
                    a *= 2
                b = 2 * a
            if a >= MARGIN_MAX:
                out["capped"].append(lab)
            else:
                while b - a > max(MARGIN_REL * a, cap.BISECT_TOL):
                    m = 0.5 * (a + b)
                    if held(i, sg, m):
                        a = m
                    else:
                        b = m
        if i < 3:
            out["push_N"][lab[0] + "xyz"[i]] = a
        else:
            out["twist_Nm"][lab[0] + "xyz"[i - 3]] = a * L
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
    iks = [FingerIK(hand, f) for f in hand.fingers]
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
