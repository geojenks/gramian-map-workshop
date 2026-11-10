"""Our two margins (follow._scale_margin, follow._disturbance_margin) on
grasps whose answer follows from the friction pyramid by hand (issue 10;
derivations in docs/MARGINS_WORKED_CHECK.md).

Each contact is a one-joint finger that pushes along the contact normal:
Tr = n^T, one input column of F newtons, so the normal force is in [0, F]
and the friction is the model's 8-edge pyramid of coefficient mu. The
expected values are computed here from the parameters by the hand formulas,
never through follow.py or the vendored solver.

Two tolerances: TIGHT (resid_tol 1e-6, close to an exact test) and the
default that follow() uses (follow.RESID_TOL). Prints the table and writes
outputs/margins_worked_check.json and outputs/margins_worked_check.md
(refuses to overwrite either).

    python tools/margins_worked_check.py
"""
import datetime
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

from gramian_workshop import follow as fo  # noqa: E402
from gramian_workshop.vendor import capability_core as cap  # noqa: E402
import _provenance as prov  # noqa: E402

TIGHT = 1e-6
A = 0.05            # m: contacts sit this far from the object centre
L = A               # m: torque scale of the scaled wrench (torque / L)
MU = 0.5
F = 10.0            # N: largest normal force each finger can push with
W_LOAD = 4.0        # N: leader's load, along +z (the wrench the contacts produce)
COM = np.zeros(3)

PINCH = ([(A, 0, 0), (-A, 0, 0)], [(-1, 0, 0), (1, 0, 0)])
FOUR = ([(A, 0, 0), (-A, 0, 0), (0, A, 0), (0, -A, 0)],
        [(-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0)])

OUT_JSON = os.path.join(ROOT, "outputs", "margins_worked_check.json")
OUT_MD = os.path.join(ROOT, "outputs", "margins_worked_check.md")


def fingers(normals, f_max):
    """One (Tr, C) per contact: the joint torque is the normal force, the
    single input supplies up to f_max of it."""
    return [(np.array([n], float), np.array([[f_max]])) for n in normals]


def wrench(fz=0.0, fy=0.0, tx=0.0):
    return np.array([0.0, fy, fz, tx, 0.0, 0.0])


def cases():
    """(name, grasp, W, mu, F, expected scale margin, expected disturbance
    margin at TIGHT, what the case shows). Expected values by the hand
    formulas of docs/MARGINS_WORKED_CHECK.md."""
    c8 = math.cos(math.pi / 8)
    th = math.pi / 8
    return [
        ("pinch", PINCH, wrench(fz=W_LOAD), MU, F,
         2 * MU * F / W_LOAD, 0.0,
         "two contacts, load on a pyramid edge"),
        ("pinch, load between pyramid edges", PINCH,
         W_LOAD * np.array([0, math.sin(th), math.cos(th), 0, 0, 0]), MU, F,
         2 * MU * F * c8 / W_LOAD, 0.0,
         "load at 22.5 degrees, midway between two edges"),
        ("pinch at its limit", PINCH, wrench(fz=2 * MU * F), MU, F,
         1.0, 0.0,
         "boundary: scale margin one"),
        ("pinch at twice its limit", PINCH, wrench(fz=4 * MU * F), MU, F,
         0.5, 0.0,
         "must fail: the fingers cannot push hard enough"),
        ("pinch, frictionless", PINCH, wrench(fz=W_LOAD), 0.0, F,
         0.0, 0.0,
         "must fail: no friction, so no grip on a load along the faces"),
        ("pinch, twist about its own axis", PINCH, wrench(tx=W_LOAD * L), MU, F,
         0.0, 0.0,
         "must fail: two point contacts cannot twist about the line through them"),
        ("four contacts", FOUR, wrench(fz=W_LOAD), MU, F,
         4 * MU * F / W_LOAD, 2 * MU * F / W_LOAD,
         "force closure: both margins non-zero"),
    ]


def ours(grasp, W, mu, f_max, resid_tol):
    pts, nrms = grasp
    Js = fingers(nrms, f_max)
    ok, why = fo._redistribute(pts, nrms, Js, COM, W, mu, L, resid_tol=resid_tol)
    sm = fo._scale_margin(pts, nrms, Js, COM, W, mu, L, resid_tol=resid_tol)
    dm = fo._disturbance_margin(pts, nrms, Js, COM, W, mu, L, resid_tol=resid_tol)
    return {"holds": bool(ok), "reason": why, "scale_margin": sm["margin"],
            "disturbance_margin": dm["margin"], "disturbance_direction": dm["direction"]}


def run():
    tol = fo.RESID_TOL
    rows = []
    for name, grasp, W, mu, f_max, s_exp, d_exp, note in cases():
        t = ours(grasp, W, mu, f_max, TIGHT)
        d = ours(grasp, W, mu, f_max, None)
        rows.append({
            "case": name, "note": note, "mu": mu, "F_N": f_max,
            "load_N": float(np.linalg.norm(cap.scale_wrench(W, L))),
            "expected_scale": s_exp, "expected_disturbance": d_exp,
            "tight": t, "default": d,
            # the default tolerance read as "the wrench may be missed by
            # RESID_TOL of its norm, the inputs within their limits"
            "default_reading_scale": s_exp / (1 - tol) if s_exp > 0 else 0.0,
        })
    # the same pinch with load and finger strength both multiplied by k:
    # every hand answer is unchanged
    units = []
    for k in (1.0, 10.0, 100.0):
        for frac, label in ((0.4, "pinch"), (2.0, "pinch at twice its limit")):
            w = frac * 2 * MU * F * k
            r = ours(PINCH, wrench(fz=w), MU, F * k, None)
            t = ours(PINCH, wrench(fz=w), MU, F * k, TIGHT)
            units.append({"k": k, "case": label, "load_N": w, "F_N": F * k,
                          "tight_scale_margin": t["scale_margin"], "tight_holds": t["holds"],
                          "expected_scale": 2 * MU * F * k / w,
                          "expected_holds": frac <= 1.0, **r})
    return rows, units


def fmt(x):
    return "-" if x is None else f"{x:.4g}"


def table(rows, units):
    out = ["| Case | Expected scale | Ours, tight | Ours, default | "
           "Default, wrench rows only | Expected disturbance | Ours, tight | Ours, default | "
           "Holds (tight, default) |",
           "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        t, d = r["tight"], r["default"]
        out.append(f"| {r['case']} | {fmt(r['expected_scale'])} | {fmt(t['scale_margin'])} | "
                   f"{fmt(d['scale_margin'])} | {fmt(r['default_reading_scale'])} | "
                   f"{fmt(r['expected_disturbance'])} | "
                   f"{fmt(t['disturbance_margin'])} | {fmt(d['disturbance_margin'])} | "
                   f"{t['holds']}, {d['holds']} |")
    out += ["", "| Units multiplied by | Case | Load (N) | Finger limit (N) | Expected scale | "
            "Ours, tight | Ours, default | Expected to hold | Holds (tight, default) |",
            "|---|---|---|---|---|---|---|---|---|"]
    for u in units:
        out.append(f"| {fmt(u['k'])} | {u['case']} | {fmt(u['load_N'])} | {fmt(u['F_N'])} | "
                   f"{fmt(u['expected_scale'])} | {fmt(u['tight_scale_margin'])} | "
                   f"{fmt(u['scale_margin'])} | {u['expected_holds']} | "
                   f"{u['tight_holds']}, {u['holds']} |")
    return "\n".join(out)


def main():
    for p in (OUT_JSON, OUT_MD):
        if os.path.exists(p):
            sys.exit(f"{p} exists; give the new output a new name")
    rows, units = run()
    tab = table(rows, units)
    print(tab)
    meta = {
        "made_by": "tools/margins_worked_check.py",
        "date": datetime.date.today().isoformat(),
        "follow_py_commit": prov.git_commit_for(ROOT, "gramian_workshop/follow.py"),
        "follow_py_dirty": prov.git_dirty(ROOT, "gramian_workshop/follow.py"),
        "follow_py_sha256": prov.sha256(os.path.join(ROOT, "gramian_workshop", "follow.py")),
        "tight_resid_tol": TIGHT, "default_resid_tol": fo.RESID_TOL,
        "margin_rel": fo.MARGIN_REL, "bisect_tol": cap.BISECT_TOL,
        "a_m": A, "L_m": L, "mu": MU, "F_N": F, "load_N": W_LOAD,
    }
    with open(OUT_JSON, "w") as f:
        json.dump({"meta": meta, "cases": rows, "units": units}, f, indent=1)
    with open(OUT_MD, "w", newline="\r\n") as f:
        f.write(f"Made by tools/margins_worked_check.py on {meta['date']}, "
                f"follow.py at {meta['follow_py_commit']}.\n\n{tab}\n")


if __name__ == "__main__":
    main()
