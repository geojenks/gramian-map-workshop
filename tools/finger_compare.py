"""Human index finger against the robot hands' index fingers: the largest
fingertip push in each direction at the same posture.

    python tools/finger_compare.py  ->  outputs/finger_compare.json, outputs/finger_compare.md

Posture: the Valero-Cuevas 1998 test posture, ad/abduction 0, MCP 45 deg,
PIP 45 deg, DIP 10 deg (L20's DIP follows its coupling from PIP). Force at
the fingertip point (follow's tip point; the paper's: the end of the distal
phalanx), in the distal phalanx's frame: distal = from the DIP axis to the
tip, palmar = the side the finger curls toward, lateral = distal x palmar.
The push is the largest s with J^T (s d) supplied by the finger's inputs
within their limits (follow._finger_columns: pull-only muscles, two-sided
motors, slaved joints folded in, no passive stiffness unless stated), a
point contact, so every joint must balance.

Torque limits: Shadow and L20 from their models; Allegro and LEAP have none
in their models, so DATASHEET values are used and marked: LEAP 0.93 N.m
(Dynamixel XC330-M288-T stall at 5 V, ROBOTIS e-manual), Allegro 0.70 N.m
(V4 "max joint torque", a reseller's page; Wonik publishes none).
"""
import json
import math
import os
import sys

import numpy as np
from scipy.optimize import linprog

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
from gramian_workshop.follow import FingerIK, _finger_columns, _palm_side, torque_limits  # noqa: E402
from gramian_workshop.hands import Hand, _mj  # noqa: E402
import valero_cuevas_finger as vc  # noqa: E402

POSE_DEG = {"abd": 0.0, "mcp": 45.0, "pip": 45.0, "dip": 10.0}
ROLES = {   # joint names of each index finger by role
    "allegro": {"abd": "ffj0", "mcp": "ffj1", "pip": "ffj2", "dip": "ffj3"},
    "shadow": {"abd": "rh_FFJ4", "mcp": "rh_FFJ3", "pip": "rh_FFJ2", "dip": "rh_FFJ1"},
    "leap": {"abd": "if_rot", "mcp": "if_mcp", "pip": "if_pip", "dip": "if_dip"},
    "l20": {"abd": "index_mcp_roll", "mcp": "index_mcp_pitch", "pip": "index_pip", "dip": "index_dip"},
    "human_vc": {"abd": "index_mcp_add", "mcp": "index_mcp_flex", "pip": "index_pip", "dip": "index_dip"},
}
DATASHEET_NM = {"allegro": (0.70, "datasheet (reseller): Allegro V4 max joint torque"),
                "leap": (0.93, "datasheet: Dynamixel XC330-M288-T stall, 5 V")}
DIRS = {"palmar": [0, 0, 1], "dorsal": [0, 0, -1], "distal": [0, 1, 0],
        "lateral": [1, 0, 0], "medial": [-1, 0, 0]}


def finger_pushes(hk, kq=0.0):
    mj = _mj()
    h = Hand(hk)
    f = [x for x in h.fingers if x.label == "index"][0]
    m = h.model
    ik = FingerIK(h, f)
    q = np.array(h.base_q)
    for role, name in ROLES[hk].items():
        j = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, name)
        if j in h.coupled:
            continue
        lo, hi = m.jnt_range[j]
        q[m.jnt_qposadr[j]] = float(np.clip(math.radians(POSE_DEG[role]), lo, hi))
    ik.d.qpos[:] = h.apply_coupling(q)
    mj.mj_kinematics(m, ik.d)
    mj.mj_comPos(m, ik.d)
    b = f.tip_body
    Rb = ik.d.xmat[b].reshape(3, 3)
    tip = ik.d.xpos[b] + Rb @ f.tip_local
    jdip = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, ROLES[hk]["dip"])
    distal = tip - ik.d.xanchor[jdip]
    length_distal = float(np.linalg.norm(distal))
    distal /= length_distal
    side, _ = _palm_side(h, f, b)
    palmar = Rb @ side
    palmar -= (palmar @ distal) * distal
    palmar /= np.linalg.norm(palmar)
    lateral = np.cross(distal, palmar)
    F = np.column_stack([lateral, distal, palmar])     # frame -> world
    jacp = np.zeros((3, m.nv))
    mj.mj_jac(m, ik.d, jacp, None, tip, b)
    Jf = jacp[:, f.dof]
    M, lim = torque_limits(h, f)
    source = "model"
    if M is None:
        tau, source = DATASHEET_NM[hk]
        free = [j for j in f.joints if j not in h.coupled]
        M = np.zeros((len(free), len(f.joints)))
        for r, j in enumerate(free):
            M[r, f.joints.index(j)] = 1.0
        lim = np.array([[-tau, tau]] * len(free))
    Tr, C = _finger_columns(h, f, ik, M, lim, Jf, tip, kq)
    out = {}
    for k, dv in DIRS.items():
        d = F @ np.array(dv, float)
        n = C.shape[1]
        A = np.hstack([-C, (Tr @ d)[:, None]])
        r = linprog(np.r_[np.zeros(n), -1.0], A_eq=A, b_eq=np.zeros(A.shape[0]),
                    bounds=[(0, 1)] * n + [(0, None)], method="highs")
        out[k] = float(r.x[-1]) if r.success else 0.0
    mass = 0.0
    bb = b
    root = int(m.body_parentid[m.jnt_bodyid[f.joints[0]]])
    while bb != root and bb > 0:
        mass += float(m.body_mass[bb])
        bb = int(m.body_parentid[bb])
    j0 = f.joints[0]
    reach = float(np.linalg.norm(tip - ik.d.xanchor[j0]))
    n_in = int(np.sum(lim[:, 1] > 0) if source == "model" else len(lim))
    return {"pushes_N": out, "limit_source": source, "inputs": int(M.shape[0]),
            "one_sided_inputs": bool(np.all(lim[:, 0] >= 0)), "finger_mass_kg": mass,
            "mcp_to_tip_m": reach, "joint_stiffness": kq, "n_input_senses": n_in}


def main():
    rows = {}
    for col in ("adjusted", "nominal"):
        J, M = vc.model([0, math.radians(45), math.radians(45), math.radians(10)], col)
        rows[f"human_vc1998_{col}"] = {"pushes_N": {k: vc.max_force(M, v)[0] for k, v in DIRS.items()},
                                       "limit_source": f"Valero-Cuevas 1998 {col} model", "inputs": 7,
                                       "one_sided_inputs": True}
    for hk in ("human_vc", "allegro", "shadow", "leap", "l20"):
        rows[hk] = finger_pushes(hk)
    rows["shadow_kq0.2"] = finger_pushes("shadow", kq=0.2)
    with open(os.path.join(ROOT, "outputs", "finger_compare.json"), "w") as f:
        json.dump({"posture_deg": POSE_DEG, "rows": rows}, f, indent=1)
    names = {"human_vc1998_adjusted": "Human index, VC 1998 adjusted (paper's numbers)",
             "human_vc1998_nominal": "Human index, VC 1998 nominal",
             "human_vc": "Human-finger hand, index (MuJoCo, adjusted)",
             "allegro": "Allegro index", "shadow": "Shadow index",
             "shadow_kq0.2": "Shadow index, joint stiffness 0.2 N.m/rad",
             "leap": "LEAP index", "l20": "Linker L20 index"}
    lines = ["| Finger | palmar | dorsal | distal | lateral | medial | inputs | limits from |",
             "|---|---|---|---|---|---|---|---|"]
    for k, r in rows.items():
        p = r["pushes_N"]
        lines.append(f"| {names[k]} | " + " | ".join(f"{p[d]:.1f}" for d in DIRS)
                     + f" | {r['inputs']}{' (pull only)' if r['one_sided_inputs'] else ''} | {r['limit_source']} |")
    md = "\n".join(lines)
    with open(os.path.join(ROOT, "outputs", "finger_compare.md"), "w") as f:
        f.write(md + "\n")
    print(md)


if __name__ == "__main__":
    main()
