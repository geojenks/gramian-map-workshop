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
  contact     anywhere on the palm side of the last phalanx (optionally the
              middle phalanx), never the back; "palm side" is the side the
              fingertip moves toward when the finger curls
  object      the leader's own, not rescaled

Verdict per finger per frame (and per frame: the worst finger):
  green   reaches the leader's contact within REACH_GREEN_M with the pad
          facing within FACE_GREEN_DEG, and the leader's own force at that
          station lies in the friction cone there and within torque limits
  amber   close (within REACH_AMBER_M / FACE_AMBER_DEG), or a station has no
          finger, and the object wrench the leader applies can still be
          produced with different forces at the contacts the hand does make
          (the W20 readout, vendored: direction, friction cone, torque)
  red     neither; the reason is the first test failed

Simulation only; comparative, not predictive. The hand's own weight is not
counted, and only position-level reach is checked (no collisions).
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
PAD_CONE_DEG = 60.0        # vertices within this of the palm-side direction
N_PAD_POINTS = 8           # candidate contact points per phalanx
IK_ITERS = 60
IK_DAMP = 1e-4
FACE_WEIGHT_M = 0.02       # metres of position error per radian of facing error


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
                z = np.linspace(-s[1], s[1], 600)
                v = s[0] * u + np.column_stack([0 * z, 0 * z, z]) * (np.abs(u[:, 2:3]) < 2)
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
        ok = (r @ side) > math.cos(math.radians(PAD_CONE_DEG)) * nr
        P = V[ok]
        if len(P) == 0:
            continue
        # spread along the link's long axis
        axis = V[np.argmax(np.linalg.norm(V - c, axis=1))] - c
        axis /= np.linalg.norm(axis)
        s = (P - c) @ axis
        for qn in np.linspace(0.1, 0.9, N_PAD_POINTS):
            i = int(np.argmin(np.abs(s - np.quantile(s, qn))))
            nrm = P[i] - c - ((P[i] - c) @ axis) * axis   # outward, across the axis
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

    def _eff_jac(self, J):
        m = self.hand.model
        Je = J[:, self.dofs].copy()
        for dep, (master, c) in self.dep.items():
            if master in self.free:
                x = self.d.qpos[m.jnt_qposadr[master]]
                dd = c[1] + 2 * c[2] * x + 3 * c[3] * x ** 2 + 4 * c[4] * x ** 3
                Je[:, self.free.index(master)] += dd * J[:, m.jnt_dofadr[dep]]
        return Je

    def solve(self, body, p_local, n_local, target, n_target, q0=None):
        """Damped least squares on pad position and facing. Returns
        (q_free, residual_m, facing_deg, world point, world normal)."""
        mj, m, d = self.mj, self.hand.model, self.d
        q = np.array(m.qpos0)
        q[self.qadr] = self.rng.mean(1) if q0 is None else q0
        jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        for _ in range(IK_ITERS):
            d.qpos[:] = self.hand.apply_coupling(q.copy())
            mj.mj_kinematics(m, d)
            mj.mj_comPos(m, d)
            R = d.xmat[body].reshape(3, 3)
            p = d.xpos[body] + R @ p_local
            n = R @ n_local
            e = np.concatenate([target - p, FACE_WEIGHT_M * np.cross(n, -n_target)])
            mj.mj_jac(m, d, jacp, jacr, p, body)
            Jp, Jr = self._eff_jac(jacp), self._eff_jac(jacr)
            J = np.vstack([Jp, FACE_WEIGHT_M * Jr])
            dq = J.T @ np.linalg.solve(J @ J.T + IK_DAMP * np.eye(6), e)
            q[self.qadr] = np.clip(q[self.qadr] + dq, self.rng[:, 0], self.rng[:, 1])
        d.qpos[:] = self.hand.apply_coupling(q.copy())
        mj.mj_kinematics(m, d)
        R = d.xmat[body].reshape(3, 3)
        p = d.xpos[body] + R @ p_local
        n = R @ n_local
        face = math.degrees(math.acos(max(-1.0, min(1.0, float(-n @ n_target)))))
        return q[self.qadr].copy(), float(np.linalg.norm(target - p)), face, p, n

    def best(self, cands, target, n_target):
        """Best pad point for this target: smallest residual, then facing."""
        best = None
        for b, pl, nl in cands:
            r = self.solve(b, pl, nl, target, n_target)
            key = (r[1] > REACH_GREEN_M, r[1], r[2])
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

    def station(self, k, i, Rw, t):
        """World (hand-frame) contact point and inward normal of station i at
        frame k, after placing the leader by rotation Rw and translation t
        (applied about the leader's starting object centre)."""
        f = self.frames[k]
        cp = f["contact_point"][i] or f["station_pos"][i]
        oc = np.array(f["obj_pos"])
        p = Rw @ (np.array(cp) - self.c0) + t
        c = Rw @ (oc - self.c0) + t
        nrm = (c - p)
        nrm /= np.linalg.norm(nrm)
        return p, nrm, c

    def force(self, k, i, Rw):
        return Rw @ np.array(self.frames[k]["force_on_object_N"][i])

    def made(self, k):
        return [i for i, x in enumerate(self.frames[k]["contact"]) if x]


def placement_search(hand, leader, cands, iks, n_frames=6, n_samples=60, seed=0,
                     iters=3):
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
    mu = np.concatenate([np.zeros(3), t_mu])
    sd = np.concatenate([np.full(3, 1.2), np.full(3, 0.04)])
    best = None
    for it in range(iters):
        X = mu + sd * rng.standard_normal((n_samples, 6))
        X[0] = mu
        scores = []
        for x in X:
            Rw, t = _rot(x[:3]), x[3:]
            C = np.zeros((len(hand.fingers), leader.n))
            for a, ik in enumerate(iks):
                for i in range(leader.n):
                    res = []
                    for k in ks:
                        if i not in leader.made(k):
                            continue
                        p, nrm, _ = leader.station(k, i, Rw, t)
                        _, r = ik.best(cands[a][::3], p, nrm)
                        res.append(r[1] + 0.001 * r[2] / 10.0)
                    C[a, i] = np.mean(res) if res else 1.0
            assign, cost = best_assignment(C)
            scores.append(cost)
            if best is None or cost < best[0]:
                best = (cost, Rw, t, assign, x)
        elite = X[np.argsort(scores)[:max(4, n_samples // 6)]]
        mu, sd = elite.mean(0), elite.std(0) + 1e-3
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
    """Per independent input of the finger: |tau| limit, or None."""
    m = hand.model
    if m.nu:
        mj = _mj()
        d = hand.data
        mom = np.zeros((m.nu, m.nv))
        mj.mju_sparse2dense(mom, d.actuator_moment, d.moment_rownnz,
                            d.moment_rowadr, d.moment_colind)
        acts = [a for a in range(m.nu) if np.any(mom[a, finger.dof] != 0)]
        if not acts or not all(m.actuator_forcelimited[a] for a in acts):
            return None, None
        M = mom[np.ix_(acts, finger.dof)]
        lim = np.array([max(abs(m.actuator_forcerange[a][0]),
                            abs(m.actuator_forcerange[a][1])) for a in acts])
        return M, lim
    free = [j for j in finger.joints if j not in hand.coupled]
    if not all(m.jnt_actfrclimited[j] for j in free):
        return None, None
    idx = [finger.joints.index(j) for j in free]
    M = np.zeros((len(free), len(finger.joints)))
    for r, c in enumerate(idx):
        M[r, c] = 1.0
    lim = np.array([max(abs(m.jnt_actfrcrange[j][0]), abs(m.jnt_actfrcrange[j][1]))
                    for j in free])
    return M, lim


def follow(hand_key, leader_json, middle=False, n_eval=None, seed=0):
    hand = Hand(hand_key)
    leader = Leader(leader_json)
    cands = [pad_candidates(hand, f, middle) for f in hand.fingers]
    iks = [FingerIK(hand, f) for f in hand.fingers]
    cost, Rw, t, assign, _ = placement_search(hand, leader, cands, iks, seed=seed)
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
        _, _, c = leader.station(k, 0, Rw, t)
        W = np.zeros(6)
        for i in made:
            p, _, _ = leader.station(k, i, Rw, t)
            fv = leader.force(k, i, Rw)
            W += np.concatenate([fv, np.cross(p - c, fv)])
        per, pts, nrms, Js, fingers_on = [], [], [], [], []
        for a, f in enumerate(hand.fingers):
            i = assign[a]
            if i not in made:
                per.append({"finger": f.label, "station": i, "state": "off"})
                continue
            p, nrm, _ = leader.station(k, i, Rw, t)
            (b, pl, nl), (q, res, face, pw, nw) = iks[a].best(cands[a], p, nrm)
            near = res <= REACH_AMBER_M and face <= FACE_AMBER_DEG
            exact = res <= REACH_GREEN_M and face <= FACE_GREEN_DEG
            item = {"finger": f.label, "station": i, "reach_mm": res * 1e3,
                    "facing_deg": face, "phalanx": "distal" if b == f.tip_body else "middle"}
            if not near:
                item["state"] = "red"
                item["reason"] = "reach" if res > REACH_AMBER_M else "facing"
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
            if M is not None:
                tau = Jf.T @ fv
                u = np.linalg.lstsq(M.T, tau, rcond=None)[0]
                torque_ok = bool(np.all(np.abs(u) <= lim))
            item["state"] = "green" if (exact and in_cone and torque_ok) else "amber?"
            item["torque_checked"] = M is not None
            per.append(item)
            pts.append(cp)
            nrms.append(n_in)
            Js.append((Jf, M, lim))
            fingers_on.append(a)
        unserved = [i for i in made if i not in assign]
        need_redistribute = unserved or any(x.get("state") == "amber?" for x in per)
        verdict = "green"
        reason = None
        if any(x.get("state") == "red" for x in per):
            # a red finger's station load must also be carried by the others
            need_redistribute = True
        if need_redistribute:
            ok, reason = _redistribute(pts, nrms, Js, c, W, mu, L)
            verdict = "amber" if ok else "red"
            for x in per:
                if x.get("state") == "amber?":
                    x["state"] = "amber" if ok else "red"
        frames.append({"k": int(k), "verdict": verdict, "reason": reason,
                       "unserved": unserved, "fingers": per})
    return {"hand": hand_key, "hand_name": hand.spec["name"],
            "leader": leader_json["id"], "placement_cost_mm": cost * 1e3,
            "assignment": {hand.fingers[a].label: int(assign[a])
                           for a in range(len(hand.fingers))},
            "placement": {"rotation_matrix": Rw.tolist(), "translation_m": t.tolist()},
            "frames": frames}


def _q_full(m, ik, qfree):
    q = np.array(m.qpos0)
    q[ik.qadr] = qfree
    return q


def _redistribute(pts, nrms, Js, c, W, mu, L):
    """Can the made, reachable contacts produce the leader's object wrench
    with forces of their own choosing (inside the cone, within torque
    limits)? The W20 tests, vendored."""
    if not pts:
        return False, "no_contact"
    pts, nrms = np.array(pts), np.array(nrms)
    ws = cap.scale_wrench(W, L)
    D = cap.direction_basis(pts, nrms, c, L)
    if not cap.direction_test(D, ws)[0]:
        return False, "direction"
    Gs, cidx, Fc = cap.generators(pts, nrms, c, mu, L)
    ok = cap.friction_test(Gs, ws, recheck=cap.NNLS_RECHECK_MULT)[0]
    if not ok:
        return False, "friction"
    rows = []
    for (Jf, M, lim) in Js:
        rows.append((Jf, M, lim))
    if all(M is None for _, M, _ in rows):
        return True, None
    # stack the torque-limited fingers' input rows: u = pinv(M^T) J^T f
    A, U = [], []
    for ci, (Jf, M, lim) in enumerate(rows):
        if M is None:
            continue
        P = np.linalg.pinv(M.T) @ Jf.T                   # inputs per unit force
        for e in np.where(cidx == ci)[0]:
            pass
        Ab = np.zeros((P.shape[0], Gs.shape[1]))
        for e in np.where(cidx == ci)[0]:
            Ab[:, e] = P @ Fc[:, e]
        A.append(Ab / lim[:, None])
        U.append(np.zeros(P.shape[0]))
    A = np.vstack(A)
    U = np.concatenate(U)
    ok = cap.torque_test(Gs, A, U, ws, recheck=cap.NNLS_RECHECK_MULT)[0]
    return (True, None) if ok else (False, "torque")
