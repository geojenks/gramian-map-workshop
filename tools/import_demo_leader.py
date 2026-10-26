"""Turn a recorded human demonstration (a gramian-workshop/leader@1 file with
no forces, from Wave_manip human_demo/derived/leaders/) into a leader the
follow check can use, by deriving the contact forces.

    python tools/import_demo_leader.py --wave-manip ../wave_manip \
        --path human_demo/derived/leaders/tennis_new_cup_take3_leader.json [--rev origin/master]
        ->  data/leaders/<name>_derived.json

What is derived, and how (all marked in the output):
  mass        not measured in the demo: ASSUMED_MASS_KG, the ITF tennis-ball
              specification midpoint (56.0-59.4 g), used only here; the
              source file is left as it is (mass null)
  inertia     a thin shell, 2/3 m R^2 (ASSUMED)
  net wrench  F = m (a - g) and tau = I alpha at the ball centre: a and
              alpha from local quadratic fits over +/- FIT_HALF_S of the
              tracked centre and orientation
  per contact the smallest forces (least squares, cone generators,
              non-negative) inside each touching digit's friction cone
              (MU_DERIVE, 8-edge pyramid) that give that wrench; the frame
              records how much of it they leave unbalanced
Frames are kept only where every digit marked touching has a contact
point (some touching frames lack a three-camera fingertip position).
"""
import argparse
import json
import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.vendor import stability_core as stability  # noqa: E402

ASSUMED_MASS_KG = 0.0577
FIT_HALF_S = 0.10
MU_DERIVE = 1.0
G = 9.81


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def rotvec(q):
    """Rotation vector of a unit quaternion (w x y z)."""
    q = q / np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    s = np.linalg.norm(q[1:])
    return np.zeros(3) if s < 1e-12 else q[1:] / s * 2 * np.arctan2(s, q[0])


def local_fit(t, X, half):
    """Second derivative of X(t) at each t from a quadratic fit over +/- half."""
    acc = np.full_like(X, np.nan)
    for i, ti in enumerate(t):
        w = np.abs(t - ti) <= half
        if w.sum() < 5:
            continue
        A = np.column_stack([np.ones(w.sum()), t[w] - ti, (t[w] - ti) ** 2])
        coef = np.linalg.lstsq(A, X[w], rcond=None)[0]
        acc[i] = 2 * coef[2]
    return acc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", required=True)
    ap.add_argument("--path", required=True)
    ap.add_argument("--rev", default="origin/master")
    a = ap.parse_args()
    raw = subprocess.run(["git", "-C", a.wave_manip, "show", f"{a.rev}:{a.path}"],
                         capture_output=True, text=True, check=True).stdout
    commit = subprocess.run(["git", "-C", a.wave_manip, "rev-parse", a.rev],
                            capture_output=True, text=True).stdout.strip()
    L = json.loads(raw)
    F = L["frames"]
    R = float(L["object"]["radius_m"])
    m = ASSUMED_MASS_KG
    I = 2.0 / 3.0 * m * R ** 2
    g = np.array(L.get("frame", {}).get("gravity", [0, 0, -1]), float)
    g = G * g / np.linalg.norm(g)
    t = np.array([f["t"] for f in F])
    pos = np.array([f["obj_pos"] for f in F])
    acc = local_fit(t, pos, FIT_HALF_S)
    # orientation: rotation vector of each frame relative to the previous
    # one, accumulated (small steps), then differentiated twice
    rv = np.zeros((len(F), 3))
    qs = [np.array(f["obj_quat"], float) if f.get("obj_quat") else None for f in F]
    for i in range(1, len(F)):
        if qs[i] is None or qs[i - 1] is None:
            rv[i] = rv[i - 1]
            continue
        qi = qs[i - 1] * np.array([1, -1, -1, -1])
        rv[i] = rv[i - 1] + rotvec(quat_mul(qs[i], qi))
    alpha = local_fit(t, rv, FIT_HALF_S)
    out_frames, dropped, resid = [], 0, []
    for i, f in enumerate(F):
        touch = [j for j, c in enumerate(f["contact"]) if c]
        if any(f["contact_point"][j] is None for j in touch) or np.isnan(acc[i]).any():
            dropped += 1
            continue
        c = np.array(f["obj_pos"])
        Fnet = m * (acc[i] - g)
        tau = I * (np.nan_to_num(alpha[i]))
        forces = [None] * L["n_stations"]
        res = None
        if touch:
            pts = [np.array(f["contact_point"][j]) for j in touch]
            nrm = [(c - p) / np.linalg.norm(c - p) for p in pts]
            Gm = stability.cone_generators(pts, nrm, c, MU_DERIVE, 8)
            W = np.concatenate([Fnet, tau])
            scale = np.r_[np.ones(3), np.ones(3) / R]
            A = Gm * scale[:, None]
            lam_reg = 1e-3
            from scipy.optimize import nnls
            Aa = np.vstack([A, lam_reg * np.eye(A.shape[1])])
            ba = np.concatenate([W * scale, np.zeros(A.shape[1])])
            lam, _ = nnls(Aa, ba)
            fsum = Gm[:3] * lam
            per = 1 if MU_DERIVE <= 0 else 8
            for k_, j in enumerate(touch):
                forces[j] = np.round(fsum[:, k_ * per:(k_ + 1) * per].sum(1), 5).tolist()
            res = float(np.linalg.norm(A @ lam - W * scale) / max(np.linalg.norm(W * scale), 1e-12))
            resid.append(res)
        out_frames.append({**{k: f[k] for k in ("t", "obj_pos", "obj_quat", "station_pos", "contact",
                                                 "contact_point", "contact_normal")},
                           "force_on_object_N": [x if x is not None else [0.0, 0.0, 0.0] for x in forces],
                           "derived_wrench": {"force_N": np.round(Fnet, 5).tolist(),
                                              "torque_Nm": np.round(tau, 7).tolist(),
                                              "unbalanced_share": res}})
    name = os.path.splitext(os.path.basename(a.path))[0].replace("_leader", "") + "_derived"
    lead = {**{k: v for k, v in L.items() if k != "frames"},
            "id": name, "name": L.get("name", name) + " (forces derived)",
            "kind": "human demonstration, three-camera keypoints (Wave_manip human_demo)",
            "object": {**L["object"], "mass_kg": m,
                       "mass_note": f"ASSUMED {m} kg (ITF tennis-ball spec midpoint) for the derived forces "
                                    "only; not measured in the demo"},
            "derivation": {"script": "tools/import_demo_leader.py", "source_rev": commit, "source_path": a.path,
                           "inertia": "thin shell 2/3 m R^2 (assumed)", "fit_half_s": FIT_HALF_S,
                           "mu": MU_DERIVE, "per_contact": "least-squares non-negative cone weights, ridge 1e-3",
                           "frames_dropped_touching_without_point": dropped,
                           "unbalanced_share_median": float(np.median(resid)) if resid else None,
                           "unbalanced_share_p90": float(np.percentile(resid, 90)) if resid else None},
            "frames": out_frames, "caveat": "forces derived, not measured"}
    out = os.path.join(ROOT, "data", "leaders", name + ".json")
    json.dump(lead, open(out, "w"), separators=(",", ":"))
    print(f"wrote {out}: {len(out_frames)} frames kept, {dropped} dropped (touching without a point); "
          f"wrench left unbalanced by the cones: median {np.median(resid):.3f}, p90 {np.percentile(resid, 90):.3f}")


if __name__ == "__main__":
    main()
