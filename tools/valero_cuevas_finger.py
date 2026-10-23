"""The human index finger's force Gramian, from the published Valero-Cuevas
1998 model (nominal parameters), checked against the paper's own numbers.

    python tools/valero_cuevas_finger.py   ->  outputs/vc1998_index_finger.json
                                              data/records_draft/human_index_vc1998_{nominal,adjusted}.json

Source: F. J. Valero-Cuevas, F. E. Zajac, C. G. Burgar (1998), "Large
index-fingertip forces are produced by subject-independent patterns of
muscle excitation", J. Biomech. 31:693-703
(https://valerolab.org/Papers/Valero-CuevasJOB1998.pdf).

The model (their Eq. 1-2): [f_lat, f_dist, f_palm, t_x] = M e, with
M = J^-T R F0, e in [0,1]^7 the excitations of FDP, FDS, EIP, EDC, LUM, DI,
PI; four joints (MCP ad/abduction, MCP, PIP and DIP flexion); fingertip
forces in a frame fixed to the distal phalanx (x lateral, y distal, z
palmar) and t_x the distal phalanx's sagittal torque, which must be zero
for a point contact. Every number below is from their Tables 1-2 (nominal
and adjusted columns), except the extensor-hood split marked RECONSTRUCTED: the paper
gives the bifurcation angles (10 and 30 deg) and the central-band share
(50 %) but not the network equations, so the split here is the static
equilibrium of a two-band node, an assumption checked against their
Table 3 below.
"""
import itertools
import json
import math
import os
import sys

import numpy as np
from scipy.optimize import linprog

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MUSCLES = ["FDP", "FDS", "EIP", "EDC", "LUM", "DI", "PI"]
JOINTS = ["MCP adduction", "MCP flexion", "PIP flexion", "DIP flexion"]

# Table 2: phalanx lengths (m)
L_PROX, L_MID, L_DIST = 0.050, 0.031, 0.016
# Fig. 1: the test posture (rad)
POSTURE = {"MCP_add": 0.0, "MCP_flex": math.radians(45), "PIP": math.radians(45),
           "DIP": math.radians(10)}
# Table 1 (nominal and adjusted columns) and Table 2 (dependent), moment arms
# in mm; + = adduction / flexion
PARAMS = {
    "nominal": {"FDP": {"add": 2.90, "flex": 12.00, "pip": 6.50, "dip": 3.64},
                "EIP_add": 0.30, "EDC_add": -1.19, "LUM_add": -3.84, "DI_add": -6.77, "PI_add": 4.08,
                "EIP_flex": -7.77, "LUM_flex": 7.00, "DI_flex": 2.00, "PI_flex": 4.00,
                "prox_slip": -2.75, "term_slip": -1.50, "central_share": 0.50,
                "diag_deg": 10.0, "lat_deg": 30.0,
                "PCSA": [4.10, 3.65, 1.12, 1.39, 0.36, 4.16, 1.60]},
    "adjusted": {"FDP": {"add": 2.90, "flex": 9.00, "pip": 5.07, "dip": 3.64},
                 "EIP_add": 0.30, "EDC_add": -1.19, "LUM_add": -4.61, "DI_add": -6.77, "PI_add": 6.94,
                 "EIP_flex": -9.32, "LUM_flex": 7.00, "DI_flex": 2.00, "PI_flex": 4.00,
                 "prox_slip": -3.44, "term_slip": -1.50, "central_share": 0.625,
                 "diag_deg": 79.0, "lat_deg": 39.0,
                 "PCSA": [4.10, 7.30, 0.78, 3.06, 0.72, 4.16, 4.32]},
}


def build(name):
    """R (4x7, mm) and F0 (N) for a Table 1 column. EIP/EDC: central_share of
    their tension to the proximal slip, the rest (the two lateral offshoots)
    to the terminal slip. LUM/PI, RECONSTRUCTED: a node in equilibrium
    splitting tension into bands at angles a (diagonal -> proximal slip) and
    b (lateral -> terminal slip): shares sin b / sin(a+b), sin a / sin(a+b)."""
    P = PARAMS[name]
    a, b = math.radians(P["diag_deg"]), math.radians(P["lat_deg"])
    diag, lat = math.sin(b) / math.sin(a + b), math.sin(a) / math.sin(a + b)
    cs, ps, ts, F = P["central_share"], P["prox_slip"], P["term_slip"], P["FDP"]
    R = np.array([
        [F["add"], 0.5 * F["add"], P["EIP_add"], P["EDC_add"], P["LUM_add"], P["DI_add"], P["PI_add"]],
        [F["flex"], 1.1 * F["flex"], P["EIP_flex"], P["EIP_flex"], P["LUM_flex"], P["DI_flex"], P["PI_flex"]],
        [F["pip"], 0.9 * F["pip"], cs * ps, cs * ps, diag * ps, 0.0, diag * ps],
        [F["dip"], 0.0, (1 - cs) * ts, (1 - cs) * ts, lat * ts, 0.0, lat * ts],
    ])
    F0 = np.array(P["PCSA"]) * 35.0            # PCSA (cm^2) x 35 N/cm^2 (Zajac 1989)
    return R, F0, {"diagonal_band_share": diag, "lateral_band_share": lat}


# Table 3 (nominal model column) and Table 6, the checks
TABLE3 = {"nominal": {"palmar": 17.3, "distal": 17.4, "medial": 15.0, "lateral": 27.2, "dorsal": 4.1},
          "adjusted": {"palmar": 24.2, "distal": 38.9, "medial": 23.7, "lateral": 32.1, "dorsal": 6.9}}
TABLE3_MEASURED = {"palmar": (27.9, 4.1), "distal": (24.3, 8.3), "medial": (22.9, 7.8),
                   "lateral": (14.7, 4.8), "dorsal": (7.5, 1.5)}
TABLE6 = {"dorsal": [0, -1, -0.62, -0.21], "palmar": [0, 1, 0.62, 0.21],
          "distal": [0, 1, 0.11, 0], "lateral": [-1, 0, 0, 0], "medial": [1, 0, 0, 0]}
DIRS = {"lateral": [1, 0, 0], "medial": [-1, 0, 0], "distal": [0, 1, 0],
        "palmar": [0, 0, 1], "dorsal": [0, 0, -1]}


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def jacobian(q):
    """4x4: joint rates -> [v (distal-phalanx frame, 3), omega_x]. Metacarpal
    frame x lateral, y distal, z palmar; adduction about +z (toward -x),
    flexion about the (abducted) +x, which swings y toward z."""
    q1, q2, q3, q4 = q
    R1 = _rz(q1)
    Rs = [R1 @ _rx(q2), R1 @ _rx(q2 + q3), R1 @ _rx(q2 + q3 + q4)]
    origins = [np.zeros(3)]
    for R, Lk in zip(Rs, (L_PROX, L_MID, L_DIST)):
        origins.append(origins[-1] + R @ np.array([0, Lk, 0]))
    tip = origins[-1]
    axes = [np.array([0, 0, 1.0]), R1 @ [1, 0, 0], R1 @ [1, 0, 0], R1 @ [1, 0, 0]]
    pts = [origins[0], origins[0], origins[1], origins[2]]
    Rd = Rs[-1]
    J = np.zeros((4, 4))
    for j, (a, o) in enumerate(zip(axes, pts)):
        J[:3, j] = Rd.T @ np.cross(a, tip - o)
        J[3, j] = (Rd.T @ a)[0]
    return J


def model(q, name="nominal"):
    J = jacobian(q)
    R, F0, _ = build(name)
    M = np.linalg.inv(J.T) @ (R / 1e3) @ np.diag(F0)
    return J, M


def max_force(M, d):
    """Largest s with M[:3] e = s d, M[3] e = 0 (no distal phalanx torque),
    0 <= e <= 1; returns (s, e)."""
    d = np.asarray(d, float) / np.linalg.norm(d)
    n = M.shape[1]
    c = np.zeros(n + 1)
    c[-1] = -1.0
    A = np.zeros((4, n + 1))
    A[:3, :n], A[:3, -1] = M[:3], -d
    A[3, :n] = M[3]
    r = linprog(c, A_eq=A, b_eq=np.zeros(4), bounds=[(0, 1)] * n + [(0, None)], method="highs")
    return (float(r.x[-1]), r.x[:n]) if r.success else (0.0, None)


def eig(W):
    w, v = np.linalg.eigh(W)
    o = np.argsort(w)[::-1]
    return w[o], v[:, o]


def analyse(name, q):
    J, M = model(q, name)
    out = {"column": name, "reconstructed": build(name)[2], "M": M.tolist(), "muscles": MUSCLES}
    print(f"== {name} model (Table 1 {name} column)")
    print("Check 1, Table 6 (normalised joint torques per unit force, no distal torque):")
    c1 = {}
    for k, d in DIRS.items():
        tau = J.T @ np.concatenate([d, [0.0]])
        tau = tau / np.max(np.abs(tau))
        c1[k] = tau.round(3).tolist()
        print(f"   {k:8s} model {np.round(tau, 2)}   paper {TABLE6[k]}")
    out["check_table6"] = c1
    print(f"Check 2, Table 3 (maximal force, N):")
    c2 = {}
    for k, d in DIRS.items():
        s_, _ = max_force(M, d)
        c2[k] = s_
        m, sd = TABLE3_MEASURED[k]
        print(f"   {k:8s} rebuilt {s_:5.1f}   paper's {name} model {TABLE3[name][k]:5.1f}   measured {m} +/- {sd}")
    out["check_table3"] = c2
    Mf = M[:3]
    W_in = Mf @ Mf.T
    m4 = M[3] / np.linalg.norm(M[3])
    P = np.eye(7) - np.outer(m4, m4)
    W_held = (Mf @ P) @ (Mf @ P).T
    dirs = [np.array(v, float) for v in itertools.product((-1, 0, 1), repeat=3) if any(v)]
    dirs = [v / np.linalg.norm(v) for v in dirs]
    F = np.array([max_force(M, d)[0] * d for d in dirs])
    W_set = F.T @ F / len(F)
    print("Gramians (N^2): eigenvalues, leading direction [lateral, distal, palmar], authority sqrt(trace)")
    for nm, W, note in (("W_in", W_in, "M_f M_f^T, all muscles, unit excitation (as W_c)"),
                        ("W_held", W_held, "the same, restricted to excitation changes giving no distal phalanx torque"),
                        ("W_f", W_set, "force-set second moment over 26 maximal pushes (the rig's measure)")):
        w, v = eig(W)
        print(f"   {nm:7s} eig {np.round(w, 1)}  lead {np.round(v[:, 0], 2)}  authority {math.sqrt(w.sum()):.1f} N")
        out[nm] = {"W": W.tolist(), "eigenvalues": w.tolist(), "vectors": v.tolist(),
                   "authority_N": math.sqrt(float(w.sum())), "definition": note}
    out["force_set"] = {"directions": [d.tolist() for d in dirs], "force_N": F.tolist()}
    return out, M


def main():
    q = [POSTURE["MCP_add"], POSTURE["MCP_flex"], POSTURE["PIP"], POSTURE["DIP"]]
    res = {}
    for name in ("nominal", "adjusted"):
        res[name], M = analyse(name, q)
        if name == "nominal":
            Mn = M
    out = {"source": "Valero-Cuevas, Zajac, Burgar 1998, J Biomech 31:693-703",
           "posture_deg": {k: math.degrees(v) for k, v in POSTURE.items()},
           "frame": "distal phalanx: x lateral, y distal, z palmar; forces the fingertip applies, N",
           **res}
    os.makedirs(os.path.join(ROOT, "outputs"), exist_ok=True)
    with open(os.path.join(ROOT, "outputs", "vc1998_index_finger.json"), "w") as f:
        json.dump(out, f, indent=1)
    recs = []
    for name in ("nominal", "adjusted"):
        r = res[name]
        M = np.array(r["M"])
        recs.append({"schema_version": "0.2-draft (PROPOSAL_FORCE_SET_RECORDS.md, not approved)",
           "id": f"human_index_vc1998_{name}",
           "name": f"Human index finger, Valero-Cuevas 1998 {name} model",
           "source": {"kind": "literature",
                      "detail": f"{name} model rebuilt from Tables 1-2; extensor-hood split reconstructed",
                      "reference": "Valero-Cuevas FJ, Zajac FE, Burgar CG (1998) J Biomech 31:693-703"},
           "force_set": {"convention": out["frame"],
                         "effort": {"kind": "max_voluntary", "fraction": 1.0},
                         "postures": [{"id": "vc1998_standard",
                                       "joint_angles_rad": {"MCP_abd": 0.0, "MCP_flex": q[1], "PIP": q[2], "DIP": q[3]},
                                       "joint_angles_source": "model",
                                       "directions": r["force_set"]["directions"],
                                       "force_N": r["force_set"]["force_N"], "force_sd_N": None}],
                         "model": {"A": M[:3].tolist(), "inputs": MUSCLES,
                                   "constraint": "row t_x = 0", "t_x_row": M[3].tolist()}},
           "caveat": "model, not measurement"})
    os.makedirs(os.path.join(ROOT, "data", "records_draft"), exist_ok=True)
    for rec in recs:
        with open(os.path.join(ROOT, "data", "records_draft", rec["id"] + ".json"), "w") as f:
            json.dump(rec, f, indent=1)
    print("wrote outputs/vc1998_index_finger.json and data/records_draft/human_index_vc1998_*.json")


if __name__ == "__main__":
    sys.exit(main())
