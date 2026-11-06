"""Instantaneous feed-forward check of one finger at one contact (6 Nov).

The question: with the finger at the pose follow.py found and touching at
the pad point follow.py chose, can the finger, at that instant and with no
motion, apply the force the leader demands there? Nothing is settled, the
object is not released and its stability is not tested: the check only asks
whether the hand could feed the demanded contact state forward.

The demanded force f on the object needs the joint torque J^T f from the
finger (J: the pad point's Jacobian over the finger's joints). What the
finger has, all read from the MuJoCo model at that pose:

  inputs    each actuator's force through the model's own transmission
            (actuator_moment; a tendon servo over two joints is one input
            and puts its moment on both). Its static range is what a ctrl
            inside ctrlrange can command at this pose (gain x ctrl + bias,
            a position servo: kp (ctrl - length)), clipped to forcerange.
            With servo_range=False the range is forcerange alone, as
            follow.py's torque test has it.
  stops     a joint within stop_tol of a range limit takes any moment into
            the stop: +1 sense at the lower limit, -1 at the upper
            (follow.py's rule, issue 26).
  friction  each joint's dry friction, any moment within +/- frictionloss
            (friction=True), or none (friction=False). Whether stiction
            counts as something a hand can feed forward is a judgement the
            check does not make: it reports both.
  passive   the model's spring moments at rest (qfrc_passive at zero
            velocity), a fixed term. Gravity on the hand's links is left
            out, as follow.py leaves it; damping is zero at rest.

Two answers per finger: the smallest unbalanced joint torque over those
terms (balance), and the nearest force to the demanded one that the finger
balances exactly at that contact (nearest_force), with its departure. The
leader carries contact forces only (no contact torques), so this is a
force check.

numpy only. bvls() is a small bounded least squares (active set, after
Stark and Parker); nearest_force() and the least-friction split use it with
the balance as heavily weighted rows (a penalty, so the balance left over
is always reported next to the answer).
"""
from __future__ import annotations

import numpy as np

EQ_WEIGHT = 1e6            # weight of the balance rows in the penalty solves
TIE_WEIGHT = 1e-6          # weight that picks the least friction, then the least input use
STOP_TOL = 1e-4            # rad (m on a slide); follow.py's STOP_TOL


def _mj():
    import mujoco
    return mujoco


def bvls(A, b, lo, hi, tol=1e-11, max_iter=None):
    """x minimising ||A x - b|| with lo <= x <= hi (either bound may be
    infinite). Active set: variables sit on a bound or are free; a bound
    variable whose gradient points inwards is freed, largest first, and the
    free set is solved by least squares, stepping back to the first bound
    hit."""
    A = np.asarray(A, float)
    b = np.asarray(b, float)
    lo = np.asarray(lo, float)
    hi = np.asarray(hi, float)
    n = A.shape[1]
    if np.any(lo > hi):
        raise ValueError("bvls: a lower bound is above its upper bound")
    x = np.clip(np.zeros(n), lo, hi)
    free = (x > lo) & (x < hi)
    fixed = lo == hi
    barred = np.zeros(n, bool)          # freed and went straight back: not a candidate
    max_iter = 20 * n + 20 if max_iter is None else max_iter
    cn = np.linalg.norm(A, axis=0)      # a bound variable is freed when its column leans into
                                        # the residual by more than tol (cosine), whatever the units
    last = -1
    for _ in range(max_iter):
        while free.any():
            idx = np.where(free)[0]
            z = np.linalg.lstsq(A[:, idx], b - A[:, ~free] @ x[~free], rcond=None)[0]
            under, over = z < lo[idx], z > hi[idx]
            if not (under | over).any():
                if not np.allclose(z, x[idx], rtol=0, atol=1e-15):
                    barred[:] = False
                x[idx] = z
                break
            xs = x[idx]
            dz = z - xs
            with np.errstate(divide="ignore", invalid="ignore"):
                al = np.where(under, (lo[idx] - xs) / dz, np.where(over, (hi[idx] - xs) / dz, np.inf))
            al = np.where(np.isnan(al), 0.0, al)
            a = max(float(al.min()), 0.0)
            hit = (under | over) & (al <= a + 1e-14)
            xn = xs + a * dz
            xn[hit & under] = lo[idx][hit & under]
            xn[hit & over] = hi[idx][hit & over]
            if a > 0:
                barred[:] = False
            elif last >= 0 and hit[idx == last].any():
                barred[last] = True
            x[idx] = xn
            free[idx[hit]] = False
        r = b - A @ x
        w = A.T @ r
        thr = tol * cn * np.linalg.norm(r)
        cand = ~free & ~fixed & ~barred & (cn > 0) & (((x <= lo) & (w > thr)) | ((x >= hi) & (w < -thr)))
        if not cand.any():
            break
        last = int(np.argmax(np.where(cand, np.abs(w) / np.where(cn > 0, cn, 1.0), -1.0)))
        free[last] = True
    return x


def input_force_range(m, d, a, servo_range=True):
    """(lo, hi, [what sets lo, what sets hi]): the static force actuator a
    can hold at the pose in d. forcerange where the model limits it; with
    servo_range, also what ctrl within ctrlrange commands at rest (fixed
    gain, no or affine bias, no activation dynamics), whichever is
    tighter."""
    mj = _mj()
    lo, hi, src = -np.inf, np.inf, ["none", "none"]
    if m.actuator_forcelimited[a]:
        lo, hi = (float(v) for v in m.actuator_forcerange[a])
        src = ["force range", "force range"]
    plain = (int(m.actuator_gaintype[a]) == int(mj.mjtGain.mjGAIN_FIXED)
             and int(m.actuator_biastype[a]) in (int(mj.mjtBias.mjBIAS_NONE), int(mj.mjtBias.mjBIAS_AFFINE))
             and int(m.actuator_dyntype[a]) == int(mj.mjtDyn.mjDYN_NONE))
    if servo_range and m.actuator_ctrllimited[a] and plain:
        g = float(m.actuator_gainprm[a][0])
        b0 = b1 = 0.0
        if int(m.actuator_biastype[a]) == int(mj.mjtBias.mjBIAS_AFFINE):
            b0, b1 = float(m.actuator_biasprm[a][0]), float(m.actuator_biasprm[a][1])
        f = sorted(g * float(c) + b0 + b1 * float(d.actuator_length[a]) for c in m.actuator_ctrlrange[a])
        f1, f2 = float(np.clip(f[0], lo, hi)), float(np.clip(f[1], lo, hi))
        if f1 > lo:
            lo, src[0] = f1, "ctrl range"
        if f2 < hi:
            hi, src[1] = f2, "ctrl range"
    return lo, hi, src


def input_ctrl(m, d, a, force):
    """The ctrl that makes actuator a hold `force` at rest at the pose in d
    (the feed-forward command), or None where the actuator is not a plain
    fixed-gain one. Not clipped to ctrlrange."""
    mj = _mj()
    if not (int(m.actuator_gaintype[a]) == int(mj.mjtGain.mjGAIN_FIXED)
            and int(m.actuator_biastype[a]) in (int(mj.mjtBias.mjBIAS_NONE), int(mj.mjtBias.mjBIAS_AFFINE))
            and int(m.actuator_dyntype[a]) == int(mj.mjtDyn.mjDYN_NONE)):
        return None
    g = float(m.actuator_gainprm[a][0])
    if g == 0:
        return None
    bias = 0.0
    if int(m.actuator_biastype[a]) == int(mj.mjtBias.mjBIAS_AFFINE):
        bias = float(m.actuator_biasprm[a][0]) + float(m.actuator_biasprm[a][1]) * float(d.actuator_length[a])
    return (float(force) - bias) / g


def finger_terms(m, d, joints, point, body, servo_range=True, stop_tol=STOP_TOL):
    """What the finger with `joints` (ids, hinge or slide) has at the pose
    already in d.qpos, for a contact at world `point` on `body`. Runs
    mj_forward at zero velocity. Returns a dict:
      J        3 x nj, the point's Jacobian over the joints
      inputs   actuator ids whose moment reaches the joints
      B        inputs x nj, their moments (N.m per unit actuator force)
      a_lo, a_hi, a_src   their static force ranges (input_force_range)
      stops    [(joint index, "lo" or "hi", sign)]
      friction nj, the joints' frictionloss
      passive  nj, the model's spring moments at rest
    """
    mj = _mj()
    joints = [int(j) for j in joints]
    dof = np.array([m.jnt_dofadr[j] for j in joints])
    if np.any(m.jnt_actfrclimited[joints]):
        raise ValueError("feedforward: a joint-level actuator force limit is not handled")
    d.qvel[:] = 0
    d.qacc[:] = 0
    mj.mj_forward(m, d)
    jacp = np.zeros((3, m.nv))
    mj.mj_jac(m, d, jacp, None, np.asarray(point, float), int(body))
    mom = np.zeros((m.nu, m.nv))
    if m.nu:
        mj.mju_sparse2dense(mom, d.actuator_moment, d.moment_rownnz, d.moment_rowadr, d.moment_colind)
    inputs = [a for a in range(m.nu) if np.any(mom[a, dof] != 0)]
    rng = [input_force_range(m, d, a, servo_range) for a in inputs]
    stops = []
    for jx, j in enumerate(joints):
        if not m.jnt_limited[j]:
            continue
        q = float(d.qpos[m.jnt_qposadr[j]])
        lo, hi = m.jnt_range[j]
        if abs(q - lo) <= stop_tol:
            stops.append((jx, "lo", 1.0))
        if abs(q - hi) <= stop_tol:
            stops.append((jx, "hi", -1.0))
    # springs only: qfrc_passive at rest, less gravity compensation if any
    passive = np.array(d.qfrc_spring[dof]) if hasattr(d, "qfrc_spring") else np.array(d.qfrc_passive[dof])
    return {"J": jacp[:, dof], "inputs": inputs, "B": mom[np.ix_(inputs, dof)] if inputs else np.zeros((0, len(dof))),
            "a_lo": np.array([r[0] for r in rng]), "a_hi": np.array([r[1] for r in rng]),
            "a_src": [r[2] for r in rng], "stops": stops,
            "friction": np.array(m.dof_frictionloss[dof], float), "passive": passive}


def _system(terms, friction):
    """(G, lo, hi, parts): the joint torque the finger supplies is
    G @ y + passive for lo <= y <= hi, y = [input forces, joint friction
    (friction only), stop reactions]; parts gives each block's slice."""
    nj = terms["J"].shape[1]
    na = len(terms["inputs"])
    fl = terms["friction"] if friction else np.zeros(nj)
    fj = [j for j in range(nj) if fl[j] > 0]
    S = np.zeros((nj, len(terms["stops"])))
    for c, (jx, _, sg) in enumerate(terms["stops"]):
        S[jx, c] = sg
    Fm = np.zeros((nj, len(fj)))
    for c, j in enumerate(fj):
        Fm[j, c] = 1.0
    G = np.hstack([terms["B"].T, Fm, S])
    lo = np.concatenate([terms["a_lo"], -fl[fj], np.zeros(S.shape[1])])
    hi = np.concatenate([terms["a_hi"], fl[fj], np.full(S.shape[1], np.inf)])
    parts = {"inputs": slice(0, na), "friction": slice(na, na + len(fj)),
             "stops": slice(na + len(fj), G.shape[1]), "friction_joints": fj}
    return G, lo, hi, parts


def _widths(lo, hi):
    """A size for each unknown, for the tie-breaking rows: the width of its
    range where finite, else 1."""
    return np.where(np.isfinite(hi - lo) & (hi > lo), hi - lo, 1.0)


def _split(terms, parts, y):
    nj = terms["J"].shape[1]
    phi = np.zeros(nj)
    phi[parts["friction_joints"]] = y[parts["friction"]]
    stop = np.zeros(nj)
    for c, (jx, _, sg) in enumerate(terms["stops"]):
        stop[jx] += sg * y[parts["stops"]][c]
    a = y[parts["inputs"]]
    return {"input_force": a, "input_torque_Nm": terms["B"].T @ a, "friction_Nm": phi, "stop_Nm": stop}


def balance(terms, f, friction=True):
    """The smallest unbalanced joint torque when the finger is asked for
    force f on the object at the contact, and who carries the rest. Among
    the ways of carrying it, the one that leans least on joint friction
    (share of each joint's frictionloss), then least on the inputs."""
    f = np.asarray(f, float)
    need = terms["J"].T @ f - terms["passive"]
    G, lo, hi, parts = _system(terms, friction)
    y = bvls(G, need, lo, hi) if G.shape[1] else np.zeros(0)
    resid = need - G @ y
    un = float(np.linalg.norm(resid))
    # the split: same balance (heavily weighted), least friction, then least input use
    if G.shape[1]:
        ts = max(float(np.linalg.norm(need)), 1e-12)
        wd = _widths(lo, hi)
        pen = np.zeros(G.shape[1])
        pen[parts["friction"]] = 1.0
        pen[parts["inputs"]] = TIE_WEIGHT
        pen[parts["stops"]] = TIE_WEIGHT
        A = np.vstack([EQ_WEIGHT * G / ts, np.diag(pen / wd)])
        rhs = np.concatenate([EQ_WEIGHT * need / ts, np.zeros(G.shape[1])])
        y2 = bvls(A, rhs, lo, hi)
        if np.linalg.norm(need - G @ y2) <= un * (1 + 1e-6) + 1e-12:
            y = y2
    out = _split(terms, parts, y)
    out.update({"needed_Nm": need, "needed_norm_Nm": float(np.linalg.norm(need)),
                "unbalanced_Nm": un, "unbalanced_by_joint_Nm": need - G @ y if G.shape[1] else need})
    fl = terms["friction"]
    with np.errstate(divide="ignore", invalid="ignore"):
        share = np.where(fl > 0, np.abs(out["friction_Nm"]) / fl, 0.0)
    out["friction_share"] = share if friction else np.zeros_like(share)
    return out


def nearest_force(terms, f, friction=True):
    """The force nearest f that the finger balances at this contact with
    what it has (inputs within range, stops, friction if credited). Returns
    the force, its departure from f, and the joint torque still unbalanced
    (the balance is a weighted penalty, so this is small but not zero)."""
    f = np.asarray(f, float)
    J = terms["J"]
    G, lo, hi, parts = _system(terms, friction)
    fs = max(float(np.linalg.norm(f)), 1e-12)
    ts = max(float(np.linalg.norm(J.T @ f)), 1e-12)
    ny = G.shape[1]
    wd = _widths(lo, hi)
    pen = np.zeros(ny)
    pen[parts["friction"]] = TIE_WEIGHT
    # unknowns [df (3), y]: J^T (f + df) - passive = G y, least |df|
    A = np.vstack([np.hstack([EQ_WEIGHT * J.T / ts, -EQ_WEIGHT * G / ts]),
                   np.hstack([np.eye(3) / fs, np.zeros((3, ny))]),
                   np.hstack([np.zeros((ny, 3)), np.diag(pen / wd)])])
    rhs = np.concatenate([-EQ_WEIGHT * (J.T @ f - terms["passive"]) / ts, np.zeros(3 + ny)])
    sol = bvls(A, rhs, np.concatenate([np.full(3, -np.inf), lo]), np.concatenate([np.full(3, np.inf), hi]))
    df, y = sol[:3], sol[3:]
    fh = f + df
    out = _split(terms, parts, y)
    out.update({"force_N": fh, "departure_N": float(np.linalg.norm(df)),
                "balance_left_Nm": float(np.linalg.norm(J.T @ fh - terms["passive"] - G @ y))})
    return out


def describe_force(fh, f, normal_in):
    """How force fh sits against the demanded f and the contact's inward
    normal: angle to f (deg), its normal and shear parts on the contact."""
    fh, f, n = (np.asarray(v, float) for v in (fh, f, normal_in))
    n = n / np.linalg.norm(n)
    c = float(fh @ f) / max(float(np.linalg.norm(fh) * np.linalg.norm(f)), 1e-300)
    fn = float(fh @ n)
    sh = float(np.linalg.norm(fh - fn * n))
    return {"angle_to_demanded_deg": float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))),
            "normal_N": fn, "shear_N": sh, "shear_over_normal": sh / fn if fn > 0 else float("inf")}
