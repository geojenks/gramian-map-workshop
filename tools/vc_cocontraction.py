"""Strongest *stable* fingertip pushes of the Valero-Cuevas 1998 finger, with
muscle stiffness that grows with activation (co-contraction), and a MuJoCo
check that the finger holds them.

    python tools/vc_cocontraction.py [--column adjusted]  ->  outputs/vc1998_cocontraction_<column>.json

Assumption (George, 23 Oct: "assume co-contraction ... to accommodate the
stiffness"): each muscle's stiffness is kappa * (its force), kappa in 1/m,
the short-range stiffness form gamma * F / l0 (gamma about 23 and finger
fibre lengths l0 of a few cm are commonly quoted, NOT checked here, so
kappa is swept: 0, 100, 300, 1000 1/m). Constant moment arms (the paper's).

With the tip pinned, the finger has one spare motion (tools/vc_finger_mujoco.
selfmotion: DIP moved, the rest solved to keep the tip still), with tangent
t = dq/ds and curvature c = d2q/ds2 at the posture. Along it the potential
of the muscles is V = -tau.q + 1/2 dl^T diag(k) dl (dl = R^T dq the tendon
stretches), so
    V'' = -tau . c + t^T R diag(kappa F0 e) R^T t,       tau = R F0 e,
linear in the excitations e. Stable iff V'' > 0, so the strongest stable
push along d is the linear program
    max s  s.t.  M_f e = s d,  m_tx e = 0,  V''(e) >= margin,  0 <= e <= 1.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from scipy.optimize import linprog

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import valero_cuevas_finger as vc  # noqa: E402
import vc_finger_mujoco as fm  # noqa: E402

KAPPAS = (0.0, 100.0, 300.0, 1000.0)
MARGIN = 1e-3        # N.m per rad^2 along the self-motion: strictly stable


def selfmotion_derivs(h=math.radians(0.25)):
    qm, q0, qp = fm.selfmotion(-h), fm.posture(), fm.selfmotion(h)
    return (qp - qm) / (2 * h), (qp - 2 * q0 + qm) / h ** 2


def stability_row(R_m, F0, kappa, t, c):
    """V''(e) = g . e: geometric part -(R F0 e).c plus stiffness part."""
    geo = -(R_m.T @ c) * F0
    stretch = R_m.T @ t                    # tendon stretch per unit s (m)
    stiff = kappa * F0 * stretch ** 2
    return geo + stiff, geo, stiff


def max_stable(M, d, g, margin=MARGIN):
    n = M.shape[1]
    d = np.asarray(d, float) / np.linalg.norm(d)
    cobj = np.zeros(n + 1)
    cobj[-1] = -1.0
    A = np.zeros((4, n + 1))
    A[:3, :n], A[:3, -1] = M[:3], -d
    A[3, :n] = M[3]
    Aub = np.zeros((1, n + 1))
    Aub[0, :n] = -g
    r = linprog(cobj, A_ub=Aub, b_ub=[-margin], A_eq=A, b_eq=np.zeros(4),
                bounds=[(0, 1)] * n + [(0, None)], method="highs")
    return (float(r.x[-1]), r.x[:n]) if r.success else (0.0, None)


def hold_in_mujoco(mj, R_m, F0, e, kappa, T=1.0):
    """Pinned tip (stiff point constraint), tendons with stiffness kappa*F0*e
    about their lengths at the posture; returns (tip force N, max drift deg)."""
    m = mj.MjModel.from_xml_string(fm.mjcf(R_m, F0))
    m.eq_solref[0] = [0.002, 1.0]
    d = mj.MjData(m)
    d.qpos[:] = fm.posture()
    mj.mj_forward(m, d)
    m.tendon_stiffness[:] = kappa * F0 * e
    m.tendon_lengthspring[:, 0] = d.ten_length
    m.tendon_lengthspring[:, 1] = d.ten_length
    q0 = fm.settle(mj, m, d, e, True, T=T)
    return fm.tip_force(mj, m, d), float(np.degrees(np.abs(d.qpos - q0)).max())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--column", default="adjusted", choices=["nominal", "adjusted"])
    a = ap.parse_args()
    import mujoco as mj
    R_mm, F0, _ = vc.build(a.column)
    R_m = R_mm / 1e3
    J, M = vc.model(fm.posture(), a.column)
    t, c = selfmotion_derivs()
    out = {"column": a.column, "kappas_per_m": KAPPAS, "margin": MARGIN, "directions": {}}
    print(f"== {a.column}: strongest stable push (N) by muscle stiffness kappa (1/m); "
          "co-contraction = total excitation above the no-stiffness optimum")
    for k, dvec in vc.DIRS.items():
        s0, e0 = vc.max_force(M, dvec)
        row = {"unstable_max_N": s0, "by_kappa": {}}
        cells = []
        for kappa in KAPPAS:
            g, geo, stiff = stability_row(R_m, F0, kappa, t, c)
            s, e = max_stable(M, dvec, g)
            if e is None:
                cells.append(f"k{kappa:<5g} none")
                row["by_kappa"][str(kappa)] = None
                continue
            f, drift = hold_in_mujoco(mj, R_m, F0, e, kappa)
            co = float(e.sum() - e0.sum())
            row["by_kappa"][str(kappa)] = {"max_N": s, "excitation": e.tolist(), "extra_excitation": co,
                                           "mujoco_tip_force_N": f.tolist(), "mujoco_drift_deg": drift}
            cells.append(f"k{kappa:<5g} {s:5.1f} N (co {co:+.2f}; sim {np.linalg.norm(f):5.1f} N, drift {drift:5.2f} deg)")
        out["directions"][k] = row
        print(f"   {k:8s} unconstrained {s0:5.1f} | " + " | ".join(cells))
    with open(os.path.join(ROOT, "outputs", f"vc1998_cocontraction_{a.column}.json"), "w") as f:
        json.dump(out, f, indent=1)
    print(f"wrote outputs/vc1998_cocontraction_{a.column}.json")


if __name__ == "__main__":
    main()
