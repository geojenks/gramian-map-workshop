"""A finger as an actuator in its own right, found by a forward model.

follow.py tests each finger with a per-joint torque balance: the pad force's
joint torques must be met by the actuators' torque columns, with any joint
left unbalanced counted against the finger. A finger whose two distal joints
hang off one tendon (the Shadow's J1/J2) fails that balance whenever the pad
load's moment about J1 differs from its moment about J2, although the real
finger simply flexes along the tendon's free direction until a joint stop or
the contact geometry takes the difference.

This module treats the finger as a unit with its own input-to-holdable-force
map, found by MuJoCo itself:

* `build_scene` compiles the hand's model with the leader's object added as a
  sphere (fixed for the per-finger search, free for the release test) and
  gravity off, as follow.py puts no weight on the links; the object's weight
  is applied explicitly where it matters.
* `FingerUnit` owns one finger's inputs (the ctrl of every actuator whose
  moment touches the finger's joints: the tendon servo counts once), settles
  the model from a starting pose under a given input, and reads back what the
  finger does: the force it puts on the object and where, which joints sit at
  their stops, and how the joint torques close (actuator, object contact,
  stop reaction, dry friction, other contacts), straight from the constraint
  rows.
* `solve_share` searches the inputs for the one that puts the leader's
  share (force and station) on the object: the input that presses hardest
  is bisected for the force magnitude, inside a small damped Gauss-Newton on
  the other inputs for the point and the force direction (every evaluation
  is a settled forward run). It says why the finger falls short when it
  does: no contact, out of reach, wrong direction, or strength (actuator
  saturated or a stop carrying the load).
* `self_collisions` lists finger links penetrating one another or the palm
  at a pose, which follow.py's per-finger placement never checks.
* `release_test` lets the object go under its own weight with every finger
  under its found input and reports the drift.

Only numpy and the optional mujoco import are used; follow.py is imported for
its tolerances and never changed.
"""
from __future__ import annotations

import re

import numpy as np

from .hands import Hand, _mj
from . import follow as fo

SPHERE_GEOM = "hold_object"
SPHERE_BODY = "hold_object_body"

# settle: step until the finger's joints are this slow (rad/s), at least
# n_min steps, at most n_max steps
SETTLE_VEL = 1e-3
SETTLE_MIN = 500
SETTLE_MAX = 8000
# a joint counts as at a stop when a limit row is active on it and its angle
# is within this of the range end (rad)
STOP_TOL = 2e-3
# the input search: a press input (the one whose move raises the pad force
# most) is bisected for the force magnitude; the other, aiming inputs are
# moved by damped Gauss-Newton on finite differences for the point and the
# force direction
PROBE_STEP = 0.05          # rad, move used to pick the press input
PRESS_STEP = 0.05          # rad, bracket expansion step on the press input
PRESS_EXPAND = 14          # at most this many expansion steps (0.7 rad)
PRESS_BISECT = 20           # bisection halvings on the press input
PRESS_TOL_FRAC = 0.1       # the press stops within this share of the force tolerance
AIM_FD = 0.03              # rad, finite-difference step on the aiming inputs
AIM_ITERS = 12
AIM_FD_MIN = 0.003         # rad; when an iteration finds no better input the step is
                           # divided by 3 while still above this, so the finest step
                           # actually tried is about a third of it
AIM_STEP0 = 0.05           # rad, first trust radius on the aiming move
AIM_STEP_MAX = 0.2
AIM_TRIES = 6
SAT_REL = 1 - 1e-6         # |actuator force| above this share of its range counts as saturated

# the gravity string a leader frame carries, e.g. "9.81 m/s^2 along [-0.79, 0.12, -0.61]"
_GRAV = re.compile(r"([0-9.]+)\s*m/s\^?2\s*along\s*\[([^\]]+)\]")


def gravity_of(frame):
    """(g, unit direction) from a leader frame's "gravity" string; the sim's
    own -z and 9.81 when the frame has none."""
    s = frame.get("gravity") if isinstance(frame, dict) else None
    if s:
        m = _GRAV.search(s)
        if m:
            g = float(m.group(1))
            d = np.array([float(x) for x in m.group(2).split(",")])
            return g, d / np.linalg.norm(d)
    return 9.81, np.array([0.0, 0.0, -1.0])


def build_scene(hand: Hand, center, radius, mu=1.0, free=False, mass=None,
                friction_loss=True):
    """The hand's model with the leader's sphere added: a world geom when
    `free` is False, a free body of mass `mass` otherwise. The sphere's
    friction wins (priority 1) so mu is the leader's. Gravity is off; when
    friction_loss is False every joint's dry friction is zeroed too."""
    mj = _mj()
    spec = mj.MjSpec.from_file(hand.path)
    if free:
        if mass is None:
            raise ValueError("a free sphere needs a mass")
        b = spec.worldbody.add_body()
        b.name = SPHERE_BODY
        b.pos = np.asarray(center, float)
        b.add_freejoint()
        g = b.add_geom()
        g.pos = np.zeros(3)
        g.mass = float(mass)
    else:
        g = spec.worldbody.add_geom()
        g.pos = np.asarray(center, float)
    g.name = SPHERE_GEOM
    g.type = mj.mjtGeom.mjGEOM_SPHERE
    g.size[0] = float(radius)
    g.friction[0] = float(mu)
    g.priority = 1
    m = spec.compile()
    m.opt.gravity[:] = 0.0
    if not friction_loss:
        m.dof_frictionloss[:] = 0.0
    return m


def finger_bodies(m, finger):
    """Every body from the finger's first joint's body down to its tip."""
    root = int(m.jnt_bodyid[finger.joints[0]])
    out = set()
    for b in range(m.nbody):
        x = b
        while x > 0 and x != root:
            x = int(m.body_parentid[x])
        if x == root:
            out.add(b)
    return out


def finger_inputs(m, finger):
    """Actuators whose moment touches the finger's dofs, at the model's
    reference pose (a tendon servo over two joints appears once)."""
    mj = _mj()
    d = mj.MjData(m)
    mj.mj_forward(m, d)
    mom = np.zeros((m.nu, m.nv))
    mj.mju_sparse2dense(mom, d.actuator_moment, d.moment_rownnz,
                        d.moment_rowadr, d.moment_colind)
    return [a for a in range(m.nu) if np.any(mom[a, finger.dof] != 0)]


def ctrl_for_pose(m, qpos):
    """Each position servo's ctrl that is at rest at qpos: the joint angle, or
    the fixed tendon's length (sum of coef x angle)."""
    mj = _mj()
    ctrl = np.zeros(m.nu)
    for a in range(m.nu):
        tid = int(m.actuator_trnid[a][0])
        if m.actuator_trntype[a] == mj.mjtTrn.mjTRN_JOINT:
            ctrl[a] = qpos[m.jnt_qposadr[tid]]
        elif m.actuator_trntype[a] == mj.mjtTrn.mjTRN_TENDON:
            adr, n = int(m.tendon_adr[tid]), int(m.tendon_num[tid])
            ctrl[a] = sum(m.wrap_prm[adr + i] * qpos[m.jnt_qposadr[m.wrap_objid[adr + i]]]
                          for i in range(n))
    return ctrl


def set_collisions(m, bodies, on):
    """Turn the collision bits of every geom on `bodies` on or off, returning
    what they were (geom id -> (contype, conaffinity)) so they can be put
    back."""
    was = {}
    for g in range(m.ngeom):
        if int(m.geom_bodyid[g]) in bodies:
            was[g] = (int(m.geom_contype[g]), int(m.geom_conaffinity[g]))
            if on is not None:
                m.geom_contype[g] = 1 if on else 0
                m.geom_conaffinity[g] = 1 if on else 0
    return was


def restore_collisions(m, was):
    for g, (ct, ca) in was.items():
        m.geom_contype[g], m.geom_conaffinity[g] = ct, ca


def settle(m, d, qpos, ctrl, dofs=None, vel_tol=SETTLE_VEL, n_min=SETTLE_MIN,
           n_max=SETTLE_MAX, extra=None):
    """Reset to qpos at rest under ctrl and step until the watched dofs
    (all when None) move slower than vel_tol. `extra(d)` runs before each
    step (to apply a weight). Returns (steps, settled)."""
    mj = _mj()
    mj.mj_resetData(m, d)
    d.qpos[:] = qpos
    d.qvel[:] = 0.0
    d.ctrl[:] = ctrl
    mj.mj_forward(m, d)
    watch = slice(None) if dofs is None else dofs
    for k in range(n_max):
        if extra is not None:
            extra(d)
        mj.mj_step(m, d)
        if k + 1 >= n_min and np.abs(d.qvel[watch]).max() < vel_tol:
            mj.mj_forward(m, d)
            return k + 1, True
    mj.mj_forward(m, d)
    return n_max, False


def object_contacts(m, d, sph, bodies=None):
    """Contacts between the sphere and the given bodies (any body when None):
    list of (body id, force on the object in world, contact point, depth m).
    mj_contactForce gives the force on the second geom along a normal that
    points from the first geom to the second, so the force on the object is
    negated when the sphere is the first geom."""
    mj = _mj()
    out = []
    f6 = np.zeros(6)
    for i in range(d.ncon):
        con = d.contact[i]
        if sph not in (con.geom1, con.geom2):
            continue
        other = con.geom2 if con.geom1 == sph else con.geom1
        b = int(m.geom_bodyid[other])
        if bodies is not None and b not in bodies:
            continue
        mj.mj_contactForce(m, d, i, f6)
        fw = con.frame.reshape(3, 3).T @ f6[:3]
        if con.geom1 == sph:
            fw = -fw
        out.append((b, fw, con.pos.copy(), float(con.dist)))
    return out


def torque_closure(m, d, finger, sph):
    """How each finger joint's torque closes at the settled state, from the
    constraint rows: actuator, object contact, stop reaction, dry friction,
    other contacts (links on links or the palm), equality; and the inertial
    remainder (M qacc, ~0 when settled). All N.m, one per finger joint."""
    mj = _mj()
    T = mj.mjtConstraint
    dof = finger.dof
    nefc = d.nefc
    J = d.efc_J.reshape(-1)[:nefc * m.nv].reshape(nefc, m.nv) if nefc else np.zeros((0, m.nv))
    parts = {k: np.zeros(len(dof)) for k in ("object", "stop", "friction", "other", "equality")}
    for i in range(nefc):
        t = int(d.efc_type[i])
        tau = J[i, dof] * d.efc_force[i]
        if t == T.mjCNSTR_LIMIT_JOINT:
            parts["stop"] += tau
        elif t == T.mjCNSTR_FRICTION_DOF:
            parts["friction"] += tau
        elif t == T.mjCNSTR_EQUALITY:
            parts["equality"] += tau
        elif t in (T.mjCNSTR_CONTACT_FRICTIONLESS, T.mjCNSTR_CONTACT_PYRAMIDAL,
                   T.mjCNSTR_CONTACT_ELLIPTIC):
            con = d.contact[int(d.efc_id[i])]
            key = "object" if sph in (con.geom1, con.geom2) else "other"
            parts[key] += tau
        else:
            parts["other"] += tau
    parts["actuator"] = d.qfrc_actuator[dof].copy()
    parts["passive"] = d.qfrc_passive[dof].copy()
    Mq = np.zeros(m.nv)
    mj.mj_mulM(m, d, Mq, d.qacc)
    parts["inertial"] = Mq[dof]
    # the identity: actuator + passive + all constraint parts - M qacc = 0
    total = (parts["actuator"] + parts["passive"] + parts["object"] + parts["stop"]
             + parts["friction"] + parts["other"] + parts["equality"] - parts["inertial"])
    parts["unclosed"] = total
    return parts


class FingerUnit:
    """One finger of `hand` on scene `m`, as an actuator: inputs, settle,
    read."""

    def __init__(self, hand: Hand, m, finger, sph):
        self.hand, self.m, self.finger, self.sph = hand, m, finger, sph
        self.bodies = finger_bodies(m, finger)
        self.inputs = finger_inputs(m, finger)
        self.ctrlrange = np.array([m.actuator_ctrlrange[a] for a in self.inputs])
        self.forcerange = np.array([m.actuator_forcerange[a] for a in self.inputs])
        self.data = _mj().MjData(m)

    def clip(self, u):
        lo, hi = self.ctrlrange[:, 0], self.ctrlrange[:, 1]
        return np.minimum(np.maximum(np.asarray(u, float), lo), hi)

    def run(self, qpos, ctrl, u):
        """Settle from qpos with this finger's inputs set to u (clipped to
        their ranges) and the rest of ctrl as given; return the reading."""
        m, d, f = self.m, self.data, self.finger
        ctrl = np.array(ctrl, float)
        ctrl[self.inputs] = self.clip(u)
        steps, ok = settle(m, d, qpos, ctrl, dofs=f.dof)
        return self.read(steps, ok, ctrl)

    def read(self, steps, settled, ctrl):
        m, d, f = self.m, self.data, self.finger
        cons = object_contacts(m, d, self.sph, self.bodies)
        force = np.zeros(3)
        point = None
        depth = 0.0
        if cons:
            fs = np.array([c[1] for c in cons])
            ps = np.array([c[2] for c in cons])
            force = fs.sum(0)
            w = np.linalg.norm(fs, axis=1)
            point = (ps * w[:, None]).sum(0) / w.sum() if w.sum() > 0 else ps.mean(0)
            depth = -min(c[3] for c in cons)
        q = d.qpos[f.qadr].copy()
        lo, hi = f.ranges[:, 0], f.ranges[:, 1]
        closure = torque_closure(m, d, f, self.sph)
        at_stop = [("lo" if abs(q[i] - lo[i]) <= STOP_TOL else
                    "hi" if abs(q[i] - hi[i]) <= STOP_TOL else None)
                   if abs(closure["stop"][i]) > 0 else None for i in range(len(q))]
        act_force = np.array([d.actuator_force[a] for a in self.inputs])
        sat = [bool(abs(act_force[i]) >= SAT_REL * max(abs(self.forcerange[i, 0]), abs(self.forcerange[i, 1])))
               for i in range(len(self.inputs))]
        return {
            "u": ctrl[self.inputs].copy(), "steps": steps, "settled": settled,
            "contact": bool(cons), "n_contacts": len(cons),
            "force_on_object_N": force, "point": point, "depth_m": depth,
            "tip": f.tip_pos(d).copy(), "q": q, "at_stop": at_stop,
            "closure": closure, "actuator_force": act_force, "saturated": sat,
            "qpos_all": d.qpos.copy(),
        }


def residual(reading, p_star, f_star, f_tol, p_tol):
    """Scaled residual: force departure over f_tol, point (tip when nothing
    touches) departure over p_tol; both at or under 1 means the share is
    held."""
    p = reading["point"] if reading["contact"] else reading["tip"]
    rf = (reading["force_on_object_N"] - f_star) / f_tol
    rp = (p - p_star) / p_tol
    return np.concatenate([rf, rp])


def _fmag(reading):
    return float(np.linalg.norm(reading["force_on_object_N"]))


def pick_press(unit, qpos0, ctrl0, u, base):
    """The input (and sign) whose PROBE_STEP move raises the pad force
    most, from the reading `base` at u."""
    f0 = _fmag(base)
    best, gain, n = (len(u) - 1, +1), -np.inf, 0
    for j in range(len(u)):
        for sgn in (+1, -1):
            du = np.zeros(len(u))
            du[j] = sgn * PROBE_STEP
            if not (unit.ctrlrange[j, 0] <= u[j] + du[j] <= unit.ctrlrange[j, 1]):
                continue
            g = _fmag(unit.run(qpos0, ctrl0, u + du)) - f0
            n += 1
            if g > gain:
                best, gain = (j, sgn), g
    return best[0], best[1], gain, n


def press_to(unit, qpos0, ctrl0, u, j, sgn, target, f_tol):
    """Bisect input j (moving in direction sgn to press harder) until the
    pad force magnitude is within f_tol/2 of target. Returns (u, reading,
    note, n_eval): note is None, "limit" (the input's range ran out before
    the force was reached), "slipped" (the pad left the object while
    pressing) or "no_contact"."""
    lo, hi = unit.ctrlrange[j]
    u = np.array(u, float)
    rd = unit.run(qpos0, ctrl0, u)
    n = 1
    f = _fmag(rd)
    if abs(f - target) <= f_tol * PRESS_TOL_FRAC:
        return u, rd, None, n
    # bracket: a below-target input and an above-target input
    direction = sgn if f < target else -sgn
    u_a, r_a = u.copy(), rd         # one side of the target
    u_b, r_b = None, None
    for _ in range(PRESS_EXPAND):
        u_try = u_a.copy()
        u_try[j] = float(np.clip(u_a[j] + direction * PRESS_STEP, lo, hi))
        if u_try[j] == u_a[j]:
            break
        r_try = unit.run(qpos0, ctrl0, u_try)
        n += 1
        f_try = _fmag(r_try)
        if abs(f_try - target) <= f_tol * PRESS_TOL_FRAC:
            return u_try, r_try, None, n
        if direction == sgn and f < target and not r_try["contact"] and r_a["contact"]:
            # pressing harder lost the contact: the pad slid off
            return u_a, r_a, "slipped", n
        if (f_try > target) != (f < target) or (f_try > target) == (f > target):
            pass
        if (f_try - target) * (_fmag(r_a) - target) < 0:
            u_b, r_b = u_try, r_try
            break
        u_a, r_a = u_try, r_try
    if u_b is None:
        best = u_a, r_a
        note = "no_contact" if not r_a["contact"] else "limit"
        return best[0], best[1], note, n
    # bisect between u_a and u_b
    best_u, best_r = (u_a, r_a) if abs(_fmag(r_a) - target) < abs(_fmag(r_b) - target) else (u_b, r_b)
    for _ in range(PRESS_BISECT):
        u_m = u_a.copy()
        u_m[j] = 0.5 * (u_a[j] + u_b[j])
        r_m = unit.run(qpos0, ctrl0, u_m)
        n += 1
        f_m = _fmag(r_m)
        if abs(f_m - target) < abs(_fmag(best_r) - target):
            best_u, best_r = u_m, r_m
        if abs(f_m - target) <= f_tol * PRESS_TOL_FRAC:
            break
        if (f_m - target) * (_fmag(r_a) - target) < 0:
            u_b, r_b = u_m, r_m
        else:
            u_a, r_a = u_m, r_m
    return best_u, best_r, None, n


def solve_share(unit: FingerUnit, qpos0, ctrl0, p_star, f_star, resid_tol=None,
                reach_tol=None, iters=AIM_ITERS, log=None):
    """Search the finger's inputs for the one that puts force f_star on the
    object at station p_star, starting from the inputs at rest at qpos0.
    The press input is bisected for the force magnitude inside a damped
    Gauss-Newton on the aiming inputs for the point and force direction.
    Force tolerance: resid_tol (follow's RESID_TOL) of |f_star|; point
    tolerance: reach_tol (follow's REACH_AMBER_M). Returns (best reading,
    verdict dict)."""
    fn = float(np.linalg.norm(f_star))
    f_tol = (fo.RESID_TOL if resid_tol is None else resid_tol) * fn
    p_tol = fo.REACH_AMBER_M if reach_tol is None else reach_tol
    u = ctrl0[unit.inputs].copy()
    base = unit.run(qpos0, ctrl0, u)
    n_eval = 1
    j, sgn, gain, n = pick_press(unit, qpos0, ctrl0, u, base)
    n_eval += n
    aim = [i for i in range(len(u)) if i != j]
    notes = []

    def pressed(u_in):
        nonlocal n_eval
        u_out, rd, note, n = press_to(unit, qpos0, ctrl0, u_in, j, sgn, fn, f_tol)
        n_eval += n
        return u_out, rd, note

    u, best, note = pressed(u)
    notes.append(note)
    r = residual(best, p_star, f_star, f_tol, p_tol)
    cost = float(r @ r)
    lam, rad = 1e-2, AIM_STEP0
    history = [{"iter": 0, "cost": cost, "max_resid": float(np.abs(r).max()), "note": note}]
    if log is not None:
        log(f"    press input {unit.m.actuator(unit.inputs[j]).name} ({'+' if sgn > 0 else '-'}), "
            f"start cost {cost:.3g} max resid {np.abs(r).max():.3g} note {note}")
    fd = AIM_FD
    for it in range(1, iters + 1):
        if np.abs(r).max() <= 1.0 or not aim:
            break
        J = np.zeros((len(r), len(aim)))
        for col, i in enumerate(aim):
            du = np.zeros(len(u))
            du[i] = fd if u[i] + fd <= unit.ctrlrange[i, 1] else -fd
            _, rd_i, _ = pressed(u + du)
            J[:, col] = (residual(rd_i, p_star, f_star, f_tol, p_tol) - r) / du[i]
        improved = False
        for _ in range(AIM_TRIES):
            A = J.T @ J + lam * np.eye(len(aim))
            step = -np.linalg.solve(A, J.T @ r)
            s_ = np.abs(step).max()
            if s_ > rad:
                step *= rad / s_
            u_try = u.copy()
            u_try[aim] += step
            u_try = unit.clip(u_try)
            u_try, rd, note = pressed(u_try)
            r_try = residual(rd, p_star, f_star, f_tol, p_tol)
            c_try = float(r_try @ r_try)
            if c_try < cost:
                u, best, r, cost = u_try, rd, r_try, c_try
                notes.append(note)
                lam = max(lam / 3, 1e-4)
                rad = min(rad * 2, AIM_STEP_MAX)
                improved = True
                break
            lam *= 4
            rad /= 3
        history.append({"iter": it, "cost": cost, "max_resid": float(np.abs(r).max()),
                        "lambda": lam, "radius": rad, "fd": fd})
        if log is not None:
            log(f"    iter {it}: cost {cost:.3g} max resid {np.abs(r).max():.3g} radius {rad:.2g} fd {fd:.3g}")
        if not improved:
            # the map may be too curved for this difference step: try a finer one
            if fd > AIM_FD_MIN:
                fd /= 3
                lam, rad = 1e-2, AIM_STEP0
                continue
            break
    verdict = classify(best, p_star, f_star, f_tol, p_tol)
    verdict.update({"n_eval": n_eval, "history": history, "press_input": int(unit.inputs[j]),
                    "press_input_name": unit.m.actuator(unit.inputs[j]).name,
                    "press_sign": int(sgn), "press_gain_N": float(gain),
                    "press_note": notes[-1]})
    return best, verdict


def classify(reading, p_star, f_star, f_tol, p_tol):
    """Can the finger hold its share at the best input found, and if not,
    why: no_contact, reach (point beyond p_tol of the station), direction
    (force magnitude there but pointing wrongly), strength (short, with the
    actuator saturated or a stop taking the load), force_excess (cannot
    press lightly enough). `green` also asks the point within
    REACH_GREEN_M."""
    f = reading["force_on_object_N"]
    fn = float(np.linalg.norm(f_star))
    df = float(np.linalg.norm(f - f_star))
    p = reading["point"] if reading["contact"] else reading["tip"]
    dp = float(np.linalg.norm(p - p_star))
    fm = float(np.linalg.norm(f))
    cosang = float(f @ f_star / (fm * fn)) if fm > 0 else None
    ang = float(np.degrees(np.arccos(np.clip(cosang, -1, 1)))) if cosang is not None else None
    held = df <= f_tol and dp <= p_tol
    why = None
    if not held:
        if not reading["contact"]:
            why = "no_contact"
        elif dp > p_tol:
            why = "reach"
        elif abs(fm - fn) <= f_tol:
            why = "direction"
        elif fm < fn - f_tol:
            why = "strength_saturated" if any(reading["saturated"]) else (
                "strength_stop" if any(s is not None for s in reading["at_stop"]) else "strength")
        else:
            why = "force_excess"
    return {"holds": bool(held), "green": bool(held and dp <= fo.REACH_GREEN_M),
            "why": why, "force_departure_N": df, "force_magnitude_N": fm,
            "point_departure_m": dp, "angle_deg": ang, "force_tol_N": f_tol,
            "point_tol_m": p_tol}


def self_collisions(m, qpos, sph=None):
    """Pairs of the hand's own bodies penetrating at qpos (object contacts
    left out): list of (body1, body2, depth m), deepest per pair."""
    mj = _mj()
    d = mj.MjData(m)
    d.qpos[:] = qpos
    mj.mj_forward(m, d)
    pairs = {}
    for i in range(d.ncon):
        con = d.contact[i]
        if sph is not None and sph in (con.geom1, con.geom2):
            continue
        b1, b2 = sorted((int(m.geom_bodyid[con.geom1]), int(m.geom_bodyid[con.geom2])))
        depth = -float(con.dist)
        if depth > pairs.get((b1, b2), -1.0):
            pairs[(b1, b2)] = depth
    return [(m.body(b1).name, m.body(b2).name, dep) for (b1, b2), dep in sorted(pairs.items())]


def disable_self_collision(m, sph):
    """The hand's geoms meet the sphere but not one another: contype and
    conaffinity bits that only match across the two groups."""
    for gid in range(m.ngeom):
        if gid != sph and (m.geom_contype[gid] or m.geom_conaffinity[gid]):
            m.geom_contype[gid], m.geom_conaffinity[gid] = 2, 1
    m.geom_contype[sph], m.geom_conaffinity[sph] = 1, 2


def release_test(hand: Hand, center, radius, mass, weight, qpos, ctrl, mu=1.0,
                 friction_loss=True, self_collide=True, duration_s=1.0):
    """Free the sphere under the force vector `weight` (N, on the object;
    the hand's links stay weightless, as follow.py has them) with the hand
    at qpos under ctrl, and report how far the centre drifts over
    duration_s and which fingers still touch. The caller gives the weight
    as the negative of the leader's net station force, so the stations'
    forces balance it exactly."""
    mj = _mj()
    m = build_scene(hand, center, radius, mu=mu, free=True, mass=mass,
                    friction_loss=friction_loss)
    sph = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, SPHERE_GEOM)
    sb = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, SPHERE_BODY)
    if not self_collide:
        disable_self_collision(m, sph)
    d = mj.MjData(m)
    # the hand's dofs come first in this model, the free joint last
    nq_hand = hand.model.nq
    d.qpos[:nq_hand] = qpos[:nq_hand]
    d.qpos[nq_hand:nq_hand + 3] = center
    d.qpos[nq_hand + 3:nq_hand + 7] = [1, 0, 0, 0]
    d.ctrl[:] = ctrl
    weight = np.asarray(weight, float)
    n = int(round(duration_s / m.opt.timestep))
    path = []
    for k in range(n):
        d.xfrc_applied[sb, :3] = weight
        mj.mj_step(m, d)
        if k % 50 == 0:
            path.append(float(np.linalg.norm(d.xpos[sb] - center)))
    mj.mj_forward(m, d)
    drift = float(np.linalg.norm(d.xpos[sb] - center))
    touching = {}
    for f in hand.fingers:
        cons = object_contacts(m, d, sph, finger_bodies(m, f))
        touching[f.label] = float(np.linalg.norm(sum((c[1] for c in cons), np.zeros(3)))) if cons else 0.0
    return {"duration_s": duration_s, "drift_m": drift, "drift_max_m": max(path) if path else drift,
            "drift_path_m": path, "finger_force_N": touching,
            "self_collisions": self_collisions(m, d.qpos.copy(), sph) if self_collide else None,
            "held": bool(drift <= fo.DEFLECT_MAX_M)}
