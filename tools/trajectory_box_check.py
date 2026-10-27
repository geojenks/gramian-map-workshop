"""How far the LP chain steps outside its input box.

    python tools/trajectory_box_check.py HAND RATE [K]

HAND picks one of the dense cases in tools/trajectory_sweep.py (for
example l20, george_kp); RATE is the rate limit per second. In the chain
the box rows sit inside the residual, so a solve may leave its box and
pay for it from the L1 budget, while a run's link rows are hard. For every
chain solve at scale 1 that holds and has a box, this prints the budget,
the residual by row block (wrench, joint, box) and the largest box excess
in input units, then counts how many solves left their box. Solves are
counted as the chain makes them, so repeats from the margin search are
included. With K, it also prints, for frame K and the frame before it,
each actuator's net torque step in the chain against the rate step.
"""
import json
import os
import sys

import numpy as np
import scipy.optimize as so

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trajectory_sweep as S  # noqa: E402

sys.path.insert(0, S.ROOT)
from gramian_workshop import follow as F, trajectory as T  # noqa: E402
from gramian_workshop.vendor import capability_core as cap  # noqa: E402


def records(tag, rate):
    """The trajectory records follow builds for one dense case, without solving them."""
    fn = [f for f in S.dense_files() if f"follow_{tag}__" in f][0]
    got = {}
    check = T.check

    def grab(recs, *a, **k):
        got["recs"] = recs
        raise SystemExit  # records only
    T.check = grab
    old = json.load(open(fn, encoding="utf-8"))
    s = old["settings"]
    L = json.load(open(os.path.join(S.ROOT, "data", "leaders", old["leader"] + ".json"), encoding="utf-8"))
    try:
        F.follow(old["hand"], L, middle=s["middle"], ik_iters=s.get("ik_iters"), wrist=bool(s.get("wrist")),
                 respace=bool(s.get("respace")), joint_stiffness=s.get("joint_stiffness_Nm_per_rad"),
                 placement=old, n_eval=300 if "tennis" in old["leader"] else None, margin=False,
                 disturbance=False, rate_limit=rate, lag_s=None, trajectory=True)
    except SystemExit:
        pass
    finally:
        T.check = check
    return got["recs"]


def main(tag, rate, k=None):
    recs = records(tag, rate)
    last, rows, carried = {}, [], {}
    linprog, solve = so.linprog, T._solve

    def lp(*a, **kw):
        last["res"] = linprog(*a, **kw)
        return last["res"]

    def spy(rr, sc, rate=None, lag=None, bounds=None, budget=True, stats=None):
        out = solve(rr, sc, rate=rate, lag=lag, bounds=bounds, budget=budget, stats=stats)
        if len(rr) == 1 and sc == [1.0] and budget and out[0]:
            r = rr[0]
            carried.setdefault(r["k"], out[1][0])
            if bounds is not None:
                res = last["res"]
                m, nz = r["Mx"].shape
                e = res.x[nz:nz + m] + res.x[nz + m:nz + 2 * m]
                nw, nj = len(r["ws"]), len(r["jr"])
                tol = max(r["tol_frac"] * r["wn"], cap.FEAS_REL)
                _, span, low = F._stack_rhs(r["Js"], r["taus"], bounds[0], r["L"])
                y = np.concatenate([out[1][0][a] for a, _, _, _ in T._columns(r)]) - low
                rows.append((r["k"], tol, e[:nw].sum(), e[nw:nw + nj].sum(), e[nw + nj:].sum(),
                             float(np.max(y - span, initial=0.0))))
        return out
    so.linprog, T._solve = lp, spy
    try:
        T._chain(recs, [{"k": r["k"]} for r in recs], rate, None, T._Stats())
    finally:
        so.linprog, T._solve = linprog, solve
    A = np.array(rows)
    used = A[:, 5] > 1e-6
    print(f"{tag} rate {rate}: boxed chain solves that hold at scale 1: {len(A)}")
    print(f" outside the box (excess > 1e-6): {int(used.sum())}; largest excess {A[:, 5].max():.4g}")
    if used.any():
        print(f" box residual over budget where used: median {np.median(A[used, 4] / A[used, 1]):.3g}")
    if k is None:
        return
    ks = [r["k"] for r in recs]
    i = ks.index(k)
    for row in rows:
        if row[0] == k:
            print(f" k {k}: budget {row[1]:.4g}, residual wrench {row[2]:.4g}, joint {row[3]:.4g}, "
                  f"box {row[4]:.4g}; box excess {row[5]:.4g}")
            break
    r0, r1 = recs[i - 1], recs[i]
    dt = r1["t"] - r0["t"]
    print(f" frames {r0['k']} to {k}, dt {dt:.4g}")
    for a, lim in zip(r0["fingers_on"], r0["lims"]):
        du = F._net_torque(lim, carried[k][a]) - F._net_torque(lim, carried[r0["k"]][a])
        step = rate * np.maximum(np.asarray(lim)[:, 1], -np.asarray(lim)[:, 0]) * dt
        print(f"  finger {a}: net torque step {np.round(du, 4)}, rate step {np.round(step, 4)}")


if __name__ == "__main__":
    main(sys.argv[1], float(sys.argv[2]), int(sys.argv[3]) if len(sys.argv) > 3 else None)
