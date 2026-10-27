"""Fit a jointed stand-in finger to an actuator record, and check the fit.

Design: docs/RECORD_FINGER_DESIGN.md (issue 5), including its amended
section. A record is read, never edited. Everything here is planar, in the
record's canonical X-Y plane (X = base -> tip), and numpy only.

The stand-in: an optional slide along X at the base, then N hinges about Z
at the starts of N equal links. One input drives every joint in fixed
ratios r (a tendon with fixed moment arms against joint springs), so at
free equilibrium the joints sit at q(u) = r a(u):

  1. the free path fixes r, the drive curve a(u) and, if the record gives
     no length, the rest length;
  2. the springs come from the record's stiffness at zero input, or, with
     no stiffness block, one scale is fitted to the blocked force size;
  3. the drive torque is then K r a(u), and the blocked force (size and
     direction) is predicted: tip pinned at rest, joints settle, reaction
     read off.

    python -m gramian_workshop.record_fit wall_0.0010m mosadegh2014_fPN

Writes outputs/record_fit/<id>.json and outputs/record_fit/report.txt.
"""
import argparse
import json
import os

import numpy as np

from .finger import load_record

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "outputs", "record_fit")

# Ruled 27 Oct 2026 (design doc, "The fit check").
TOL_FREE_SHARE_OF_STROKE = 0.05
TOL_FORCE_DIRECTION_DEG = 10.0
HINGE_COUNTS = (2, 3, 4, 5)
DEFAULT_HINGES = 3


class Chain:
    """Optional base slide, then n_h hinges at the starts of equal links."""

    def __init__(self, n_h, slide, length):
        self.n_h, self.slide, self.L = n_h, slide, float(length)
        self.n = n_h + int(slide)
        self.l = self.L / n_h
        self.x_hinge = np.arange(n_h) * self.l

    def tip(self, q):
        q = np.asarray(q, float)
        s = q[0] if self.slide else 0.0
        phi = np.cumsum(q[int(self.slide):])
        return np.array([s + self.l * np.cos(phi).sum(),
                         self.l * np.sin(phi).sum()])

    def jac(self, q):
        q = np.asarray(q, float)
        phi = np.cumsum(q[int(self.slide):])
        cols = []
        if self.slide:
            cols.append([1.0, 0.0])
        sx = -self.l * np.sin(phi)[::-1].cumsum()[::-1]
        sy = self.l * np.cos(phi)[::-1].cumsum()[::-1]
        cols += list(zip(sx, sy))
        return np.array(cols).T                      # (2, n)

    def hess(self, q):
        """(2, n, n): second derivatives of the tip."""
        q = np.asarray(q, float)
        phi = np.cumsum(q[int(self.slide):])
        cx = -self.l * np.cos(phi)[::-1].cumsum()[::-1]
        cy = -self.l * np.sin(phi)[::-1].cumsum()[::-1]
        H = np.zeros((2, self.n, self.n))
        o = int(self.slide)
        for i in range(self.n_h):
            for m in range(self.n_h):
                j = max(i, m)
                H[0, o + i, o + m] = cx[j]
                H[1, o + i, o + m] = cy[j]
        return H


def _lm(res_jac, p, iters=300):
    """Levenberg-Marquardt on (r, J) = res_jac(p)."""
    r, J = res_jac(p)
    cost = r @ r
    lam = 1e-3
    for _ in range(iters):
        A = J.T @ J
        g = J.T @ r
        step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
        r2, J2 = res_jac(p + step)
        c2 = r2 @ r2
        if c2 < cost:
            done = cost - c2 < 1e-14 * max(cost, 1e-30)
            p, r, J, cost, lam = p + step, r2, J2, c2, lam * 0.3
            if done:
                break
        else:
            lam *= 10.0
            if lam > 1e12:
                break
    return p, cost


def fit_free(u, d, n_h, slide, length=None, starts=None):
    """Fit r, a(u_k) and (if length is None) the rest length to the free
    tip displacements d (K, 2). Returns dict or None."""
    K = len(u)
    nw = n_h - 1
    fit_len = length is None
    stroke = float(np.max(np.linalg.norm(d, axis=1)))
    best = None
    for L0 in (starts or ([length] if not fit_len else
                          [0.6 * stroke, 1.0 * stroke, 1.5 * stroke])):

        def unpack(p):
            w = p[:nw]
            r_h = np.append(w, 1.0 - w.sum())
            k = nw
            rho = 0.0
            if slide:
                rho = p[k]
                k += 1
            L = float(np.exp(p[k])) if fit_len else float(length)
            k += int(fit_len)
            return r_h, rho, L, p[k:]

        def res_jac(p):
            r_h, rho, L, a = unpack(p)
            ch = Chain(n_h, slide, L)
            t0 = ch.tip(np.zeros(ch.n))
            rv = np.concatenate([[rho] if slide else [], r_h])
            R = np.zeros((K, 2))
            Jg = np.zeros((K, 2, len(p) - K))
            Ja = np.zeros((K, 2))
            for k in range(K):
                q = rv * a[k]
                Jq = ch.jac(q)
                R[k] = ch.tip(q) - t0 - d[k]
                Ja[k] = Jq @ rv
                o = int(slide)
                for i in range(nw):
                    Jg[k, :, i] = (Jq[:, o + i] - Jq[:, o + n_h - 1]) * a[k]
                c = nw
                if slide:
                    Jg[k, :, c] = Jq[:, 0] * a[k]
                    c += 1
                if fit_len:
                    # at fixed joints, every point scales with L except the
                    # slide travel
                    Jg[k, :, c] = ch.tip(q) - t0 - (
                        [rho * a[k], 0.0] if slide else 0.0)
            Jfull = np.zeros((2 * K, len(p)))
            Jfull[:, :len(p) - K] = Jg.reshape(2 * K, -1)
            for k in range(K):
                Jfull[2 * k:2 * k + 2, len(p) - K + k] = Ja[k]
            return R.ravel(), Jfull

        # initial a_k by grid search with equal hinge ratios, no slide
        ch0 = Chain(n_h, slide, L0)
        t0 = ch0.tip(np.zeros(ch0.n))
        rv0 = np.concatenate([[0.0] if slide else [], np.full(n_h, 1.0 / n_h)])
        grid = np.linspace(-3 * np.pi, 3 * np.pi, 1441)
        tips = np.array([ch0.tip(rv0 * g) - t0 for g in grid])
        a0 = np.array([grid[np.argmin(np.linalg.norm(tips - dk, axis=1))]
                       for dk in d])
        p0 = np.concatenate([np.full(nw, 1.0 / n_h), [0.0] if slide else [],
                             [np.log(L0)] if fit_len else [], a0])
        p, cost = _lm(res_jac, p0)
        if best is None or cost < best[1]:
            best = (p, cost, unpack, res_jac)
    p, cost, unpack, res_jac = best
    r_h, rho, L, a = unpack(p)
    R, _ = res_jac(p)
    err = np.linalg.norm(R.reshape(K, 2), axis=1)
    return {"n_hinges": n_h, "slide": slide, "length_m": L,
            "length_fitted": fit_len,
            "ratios": np.concatenate([[rho] if slide else [], r_h]).tolist(),
            "a": a.tolist(), "u": list(map(float, u)),
            "free_err_m": err.tolist(), "free_err_max_m": float(err.max()),
            "n_params": len(p)}


def a_of_u(fit, u):
    """The drive curve, piecewise linear, through a(0) = 0."""
    uu = np.concatenate([[0.0], fit["u"]])
    aa = np.concatenate([[0.0], fit["a"]])
    return float(np.interp(u, uu, aa))


def springs_from_stiffness(ch, st):
    """Diagonal K from the record's stiffness at zero input: at the straight
    rest pose the tip compliance is J K^-1 J^T, so the axial value is the
    slide spring and the transverse value, with one spring for all hinges,
    gives k_h = k_y * sum(arm^2). None if no zero-input level."""
    us = np.asarray(st["u"], float).ravel()
    i0 = np.where(us == 0.0)[0]
    if len(i0) == 0:
        return None, "no zero-input stiffness level"
    i0 = int(i0[0])
    k_y = float(st["k_N_per_m"][i0])
    arms = ch.L - ch.x_hinge
    k_h = k_y * float((arms ** 2).sum())
    k = [k_h] * ch.n_h
    note = f"springs from stiffness at u=0 (k_y={k_y:.4g} N/m"
    if ch.slide:
        k_x = float(st["k_axial_N_per_m"][i0])
        k = [k_x] + k
        note += f", k_x={k_x:.4g} N/m"
    return np.array(k), note + ")"


def blocked_solve(ch, Kd, tau, held=(0, 1), q_start=None, iters=80):
    """Tip held at its rest position along the axes in `held` (0 = X,
    1 = Y): K q - tau = J_h^T lam, tip_h(q) = tip_h(0). Returns (push on
    the holder as a 2-vector, q, stable, converged)."""
    held = list(held)
    n, m = ch.n, len(held)
    p0 = ch.tip(np.zeros(n))[held]
    q = np.zeros(n) if q_start is None else np.array(q_start, float)
    # lstsq: a hinge-only chain held straight cannot move along X, so an
    # axial hold carries no load from the joints and is left at 0
    J = ch.jac(q)[held]
    KKT = np.block([[Kd, -J.T], [J, np.zeros((m, m))]])
    lam = np.linalg.lstsq(KKT, np.concatenate([tau, np.zeros(m)]),
                          rcond=None)[0][n:]
    converged = False
    scale = 1.0 + np.abs(tau).sum()
    for _ in range(iters):
        J = ch.jac(q)[held]
        H = ch.hess(q)[held]
        F = np.concatenate([Kd @ q - tau - J.T @ lam, ch.tip(q)[held] - p0])
        if np.linalg.norm(F[:n]) < 1e-10 * scale and                 np.linalg.norm(F[n:]) < 1e-12 * ch.L:
            converged = True
            break
        A = Kd - np.tensordot(lam, H, axes=1)
        KKT = np.block([[A, -J.T], [J, np.zeros((m, m))]])
        dx = np.linalg.lstsq(KKT, -F, rcond=None)[0]
        q, lam = q + dx[:n], lam + dx[n:]
    J = ch.jac(q)[held]
    A = Kd - np.tensordot(lam, ch.hess(q)[held], axes=1)
    sv, Vt = np.linalg.svd(J)[1:]
    rank = int((sv > 1e-9 * max(sv.max(), 1e-30)).sum())
    Z = Vt[rank:].T                                  # null space of J
    stable = bool(Z.size == 0 or np.linalg.eigvalsh(Z.T @ A @ Z).min() > 0)
    push = np.zeros(2)
    push[held] = -lam
    return push, q, stable, converged


def _angle_deg(a, b):
    c = a @ b / (np.linalg.norm(a) * np.linalg.norm(b))
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def choose_slide(rec):
    """Slide when the record shows axial output: the free tip moves forward
    (+X) at some level, or a blocked level pushes more along X than
    across."""
    d = np.asarray(rec["free"]["tip_displacement_m"], float)
    if (d[:, 0] > 0).any():
        return True, "free tip moves forward (+X) at some level"
    b = rec.get("blocked") or {}
    if b.get("force_N"):
        f = np.asarray(b["force_N"], float)
        if (np.abs(f[:, 0]) > np.abs(f[:, 1])).any():
            return True, "blocked force mostly along X at some level"
    return False, "no forward motion or axial blocked force"


def fit_record(rid, n_h=DEFAULT_HINGES, slide=None):
    rec = load_record(rid)
    out = {"record": rec["id"], "schema_version": rec.get("schema_version")}
    free = rec.get("free")
    if not free or not free.get("tip_displacement_m"):
        out["status"] = "refused"
        out["reason"] = ("no free path in the record (free, blocked and "
                         "stiffness: %s)" % ", ".join(
                             "present" if rec.get(k) else "absent"
                             for k in ("free", "blocked", "stiffness")))
        return out
    u = np.asarray(free["u"], float).ravel()
    d3 = np.asarray(free["tip_displacement_m"], float)
    d = d3[:, :2]
    stroke = float(rec["scalars"]["free_stroke_m"])
    if slide is None:
        slide, why = choose_slide(rec)
    else:
        why = "set by caller"
    shape = rec.get("shape") or {}
    if shape.get("spine_rest_m"):
        length = float(np.asarray(shape["spine_rest_m"])[-1][0])
        len_src = "shape.spine_rest_m, last point x"
    elif (rec.get("metadata") or {}).get("length_m"):
        length = float(rec["metadata"]["length_m"])
        len_src = "metadata.length_m"
    else:
        length, len_src = None, "fitted from the free path (not recorded)"
    fit = fit_free(u, d, n_h, slide, length)
    out.update(status="fitted", slide_rule=why, length_source=len_src, fit=fit)
    out["free_out_of_plane_max_m"] = float(np.abs(d3[:, 2]).max())

    ch = Chain(n_h, slide, fit["length_m"])
    r = np.asarray(fit["ratios"])
    free_share = fit["free_err_max_m"] / stroke
    checks = {"free_path": {
        "worst_err_m": fit["free_err_max_m"], "stroke_m": stroke,
        "share_of_stroke": free_share,
        "pass": bool(free_share <= TOL_FREE_SHARE_OF_STROKE)}}

    # springs
    st = rec.get("stiffness")
    Kd, k_note, scale_fitted = None, None, False
    if st and st.get("k_N_per_m"):
        Kd, k_note = springs_from_stiffness(ch, st)
        if Kd is not None:
            Kd = np.diag(Kd)
    b = rec.get("blocked") or {}
    bu = np.asarray(b.get("u") or [], float).ravel()
    if Kd is None:
        if slide:
            out["status"] = "refused"
            out["reason"] = ("slide and hinges but no stiffness: the ratio "
                             "of slide to hinge spring is not pinned")
            out["checks"] = checks
            return out
        if not len(bu):
            out["status"] = "fitted, no force"
            out["checks"] = checks
            return out
        Kd = np.eye(ch.n)                            # scaled below
        scale_fitted = True
        k_note = "one spring scale fitted to the blocked force size"

    # blocked prediction, warm-started up the levels
    # Blocked = tip held at its rest position. A hinge-only chain held
    # straight cannot move along X, so an X hold carries no joint load;
    # only the axes the chain can move along at rest are held.
    held = (0, 1) if slide else (1,)
    held_note = ("tip held in X and Y" if slide else
                 "tip held in Y; X is carried by the structure (hinge-only)")

    def predict(Kd, substeps=20):
        """Prediction: the small-motion balance at the rest pose. Check
        beside it: the large-deflection balance, followed from u = 0 so the
        joints stay on one branch."""
        J0 = ch.jac(np.zeros(ch.n))[list(held)]
        m = len(held)
        KKT0 = np.block([[Kd, -J0.T], [J0, np.zeros((m, m))]])
        rows, q, u_prev = [], np.zeros(ch.n), 0.0
        for uk in bu:
            tau = Kd @ (r * a_of_u(fit, uk))
            sol = np.linalg.solve(KKT0, np.concatenate([tau, np.zeros(m)]))
            P = np.zeros(2)
            P[list(held)] = -sol[ch.n:]
            ok = True
            for t in np.linspace(u_prev, uk, substeps + 1)[1:]:
                tau_t = Kd @ (r * a_of_u(fit, t))
                Pn, q, stable, conv = blocked_solve(ch, Kd, tau_t, held, q)
                ok = ok and conv
            rows.append((P, {"push_N": Pn.tolist(), "stable": stable,
                             "converged": ok,
                             "max_joint_rad": float(np.abs(
                                 q[int(slide):]).max()) if ch.n_h else 0.0,
                             "small_motion_max_joint_rad": float(np.abs(
                                 sol[int(slide):ch.n]).max())
                             if ch.n_h else 0.0}))
            u_prev = uk
        return rows

    rows = predict(Kd)
    if b.get("force_N"):
        rec_f = np.asarray(b["force_N"], float)[:, :2]
        rec_mag = np.linalg.norm(np.asarray(b["force_N"], float), axis=1)
        mag_kind = "|force_N|"
    elif b.get("force_transverse_N") or b.get("force_axial_N"):
        ax = np.asarray(b.get("force_axial_N") or [0.0] * len(bu), float)
        tr = np.asarray(b.get("force_transverse_N") or [0.0] * len(bu), float)
        rec_f = None
        rec_mag = np.hypot(ax, tr)
        mag_kind = ("hypot(axial, transverse) magnitude; axis of the source "
                    "gauge not stated, compared with |predicted push|")
    else:
        rec_f, rec_mag, mag_kind = None, None, None
    if scale_fitted and rec_mag is not None:
        m1 = np.array([np.linalg.norm(P) for P, _ in rows])
        c = float(rec_mag @ m1 / (m1 @ m1))
        Kd = Kd * c
        rows = predict(Kd)
        k_note += f" (k_h = {c:.4g} N m/rad)"
    out["springs"] = {"k": np.diag(Kd).tolist(), "note": k_note,
                      "units": "slide N/m, hinges N m/rad"}

    blk = []
    for i, (P, big) in enumerate(rows):
        row = {"u": float(bu[i]), "pred_push_N": P.tolist(),
               "pred_mag_N": float(np.linalg.norm(P)),
               "large_deflection": big}
        if rec_mag is not None:
            row["rec_mag_N"] = float(rec_mag[i])
            row["mag_ratio_pred_over_rec"] = float(
                np.linalg.norm(P) / rec_mag[i]) if rec_mag[i] else None
        if rec_f is not None:
            row["rec_push_N"] = rec_f[i].tolist()
            row["direction_err_deg"] = _angle_deg(P, rec_f[i])
        blk.append(row)
    checks["blocked_size"] = {
        "compared": mag_kind, "fitted": scale_fitted,
        "note": ("one scale fitted, so only the shape across levels is "
                 "tested" if scale_fitted else
                 "predicted from stiffness and free path; reported, not "
                 "gated (no tolerance ruled)")}
    if rec_f is not None:
        worst = max(rw["direction_err_deg"] for rw in blk)
        checks["blocked_direction"] = {
            "worst_deg": worst,
            "pass": bool(worst <= TOL_FORCE_DIRECTION_DEG)}
    else:
        checks["blocked_direction"] = {
            "pass": None, "note": "not possible: record gives no direction"}
    out["blocked"] = blk
    out["blocked_hold"] = held_note
    if len(bu):
        hi = float(np.asarray(rec["input"]["channels"][0]["range"])[1])
        if hi > bu.max():
            out["extrapolated_above_u"] = float(bu.max())

    # tip stiffness, predicted along the free path, against the record
    if st and st.get("k_N_per_m"):
        srows = []
        for i, uk in enumerate(np.asarray(st["u"], float).ravel()):
            q = r * a_of_u(fit, uk)
            J = ch.jac(q)
            C = J @ np.linalg.solve(Kd, J.T)
            srows.append({"u": float(uk),
                          "pred_k_y": float(1.0 / C[1, 1]),
                          "rec_k_y": float(st["k_N_per_m"][i]),
                          "pred_k_x": float(1.0 / C[0, 0]) if C[0, 0] > 0
                          else None,
                          "rec_k_x": float(st["k_axial_N_per_m"][i])
                          if st.get("k_axial_N_per_m") else None})
        checks["tip_stiffness"] = {"rows": srows,
                                   "note": "reported, not gated"}

    # what the record supplies against what was fitted
    n_blk = (2 * len(bu) if rec_f is not None else
             (len(bu) if rec_mag is not None else 0))
    n_st = 2 * len(st["u"]) if st and st.get("k_N_per_m") else 0
    out["counts"] = {
        "free_numbers": 2 * len(u), "blocked_numbers": n_blk,
        "stiffness_numbers": n_st,
        "free_params": fit["n_params"],
        "spring_params": 1 if scale_fitted else 0,
        "stiffness_numbers_used_for_springs":
            0 if scale_fitted else (2 if slide else 1)}
    out["checks"] = checks
    out["pass"] = bool(checks["free_path"]["pass"] and
                       checks["blocked_direction"]["pass"] is not False)
    return out


def sweep(rid, slide):
    """Free-path error against hinge count, for one slide choice."""
    rec = load_record(rid)
    if not (rec.get("free") or {}).get("tip_displacement_m"):
        return []
    rows = []
    for n_h in HINGE_COUNTS:
        o = fit_record(rid, n_h, slide)
        rows.append((n_h, o["checks"]["free_path"]["share_of_stroke"],
                     o["fit"]["length_m"], o["fit"]["n_params"],
                     2 * len(o["fit"]["u"])))
    return rows


def _fmt(o):
    L = [f"== {o['record']}  ({o['status']})"]
    if o["status"] == "refused" and "fit" not in o:
        L.append("  " + o["reason"])
        return L
    f = o["fit"]
    L.append(f"  stand-in: {'slide + ' if f['slide'] else ''}"
             f"{f['n_hinges']} hinges; slide rule: {o['slide_rule']}")
    L.append(f"  length {f['length_m']:.4g} m ({o['length_source']})")
    L.append("  joint ratios r: " + ", ".join(f"{x:.4g}" for x in f["ratios"])
             + ("  (slide in m/rad first)" if f["slide"] else ""))
    c = o["checks"]["free_path"]
    L.append(f"  free path: worst {c['worst_err_m']*1e3:.3g} mm = "
             f"{100*c['share_of_stroke']:.2f}% of stroke -> "
             f"{'PASS' if c['pass'] else 'FAIL'} (tol "
             f"{100*TOL_FREE_SHARE_OF_STROKE:.0f}%)")
    if o.get("reason"):
        L.append("  " + o["reason"])
    if "springs" in o:
        L.append("  springs: " + ", ".join(f"{k:.4g}" for k in
                                           o["springs"]["k"])
                 + f"  [{o['springs']['units']}]; {o['springs']['note']}")
    if "blocked_hold" in o:
        L.append("  blocked: " + o["blocked_hold"])
    for rw in o.get("blocked", []):
        s = (f"  blocked u={rw['u']:g}: pred ({rw['pred_push_N'][0]:.4g}, "
             f"{rw['pred_push_N'][1]:.4g}) N")
        if "rec_push_N" in rw:
            s += (f", rec ({rw['rec_push_N'][0]:.4g}, "
                  f"{rw['rec_push_N'][1]:.4g}) N, "
                  f"angle {rw['direction_err_deg']:.2f} deg")
        if rw.get("rec_mag_N") is not None:
            s += f", |pred|/|rec| {rw['mag_ratio_pred_over_rec']:.3g}"
        big = rw["large_deflection"]
        s += (f"\n      large-deflection check: ({big['push_N'][0]:.4g}, "
              f"{big['push_N'][1]:.4g}) N, joints up to "
              f"{big['max_joint_rad']:.3g} rad (small-motion "
              f"{big['small_motion_max_joint_rad']:.3g})")
        if not big["stable"]:
            s += ", UNSTABLE"
        if not big["converged"]:
            s += ", NOT CONVERGED"
        L.append(s)
    bd = o["checks"].get("blocked_direction", {})
    if bd:
        L.append("  blocked direction: " + (
            bd.get("note") if bd["pass"] is None else
            f"worst {bd['worst_deg']:.2f} deg -> "
            f"{'PASS' if bd['pass'] else 'FAIL'} (tol "
            f"{TOL_FORCE_DIRECTION_DEG:g} deg)"))
    bs = o["checks"].get("blocked_size")
    if bs:
        L.append(f"  blocked size: {bs['note']}")
    for sr in o["checks"].get("tip_stiffness", {}).get("rows", []):
        kx = (f"; k_x pred {sr['pred_k_x']:.4g} rec {sr['rec_k_x']:.4g}"
              if sr["pred_k_x"] is not None and sr["rec_k_x"] is not None
              else "")
        L.append(f"  tip stiffness u={sr['u']:g}: k_y pred "
                 f"{sr['pred_k_y']:.4g} rec {sr['rec_k_y']:.4g} N/m{kx}")
    if "extrapolated_above_u" in o:
        L.append(f"  force above u={o['extrapolated_above_u']:g} is "
                 "extrapolated from the free-path drive curve")
    L.append(f"  out-of-plane free motion (left out): max "
             f"{o['free_out_of_plane_max_m']*1e3:.3g} mm")
    L.append("  numbers supplied / fitted: " + json.dumps(o["counts"]))
    big = [rw["large_deflection"] for rw in o.get("blocked", [])]
    if any(not b["stable"] or not b["converged"] for b in big):
        L.append("  large-deflection balance unstable or unconverged at some "
                 "level: the small-motion prediction is the one used")
    if big:
        worst = max(big, key=lambda b: b["small_motion_max_joint_rad"])
        L.append(f"  small-motion joint angles reach "
                 f"{worst['small_motion_max_joint_rad']:.3g} rad; where they "
                 f"are not small the blocked prediction is outside its range")
    ratios = [rw["mag_ratio_pred_over_rec"] for rw in o.get("blocked", [])
              if rw.get("mag_ratio_pred_over_rec")]
    if ratios:
        L.append(f"  blocked size |pred|/|rec| spans {min(ratios):.3g} to "
                 f"{max(ratios):.3g} across levels (not gated)")
    if "pass" in o:
        L.append(f"  FIT CHECK (free path and force direction, the ruled "
                 f"tolerances): {'PASS' if o['pass'] else 'FLAGGED'}")
    return L


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("records", nargs="+")
    ap.add_argument("--hinges", type=int, default=DEFAULT_HINGES)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    lines = ["Stand-in finger fit check (docs/RECORD_FINGER_DESIGN.md).",
             "Tolerances measure the stand-in against the record, not the "
             "physics: simulated records under-state stroke and blocked "
             "force (M-11 settle audit).", ""]
    for rid in a.records:
        o = fit_record(rid, a.hinges)
        with open(os.path.join(OUT, f"{o['record']}.json"), "w") as f:
            json.dump(o, f, indent=1)
        lines += _fmt(o)
        if o.get("fit"):
            for sl in sorted({o["fit"]["slide"], False, True}):
                rows = sweep(rid, sl)
                lines.append(f"  free-path error against hinge count "
                             f"({'with' if sl else 'no'} slide): " + "; ".join(
                                 f"N={n}: {100*e:.2f}% (L {Lf:.3g} m, "
                                 f"{npar} params / {nnum} numbers)"
                                 for n, e, Lf, npar, nnum in rows))
        lines.append("")
    txt = "\n".join(lines)
    with open(os.path.join(OUT, "report.txt"), "w") as f:
        f.write(txt + "\n")
    print(txt)


if __name__ == "__main__":
    main()
