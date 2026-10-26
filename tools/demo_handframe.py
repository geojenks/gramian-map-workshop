"""Put a recorded human demonstration into the demonstrator's own palm frame,
so the follow check needs no placement search: the ball, the contacts, the
derived forces and the whole skeleton are carried, frame by frame, into the
frame of the model hand built from the same person's measurements.

    python tools/demo_handframe.py \
        --leader data/leaders/tennis_new_cup_take3_derived.json \
        --keypoints data/hands/measured/george_tennis_new_cup_take3_keypoints.json \
        --dims data/hands/measured/george_right_2026-10-25.json
        ->  data/leaders/tennis_new_cup_take3_handframe.json
            outputs/handframe_tennis_new_cup_take3.json (the fixed placement and assignment)
            data/hands/measured/george_thumb_from_keypoints.json (thumb base, metacarpal, curl)

Per frame: a rigid fit (Kabsch, no scaling) of the observed wrist and
knuckles (keypoints 0, 5, 9, 13, 17; those triangulated) onto the model's
(tools/build_human_hand.measured_geometry, arch as in the model hand); kept
if at least 3 points fit within FIT_RMS_MAX_M. The thumb from the same
frames: base (CMC) position, metacarpal direction (CMC to MCP) and curl
direction (the thumb tip's offset from the metacarpal's line, square to it),
medians over the kept frames, in the model frame.

Contacts from the skeleton (26 Oct, after grading George's own poses with
the test: the source's contact_point, the ball point in the direction of the
nail-end tip, lay 8-25 mm from where his fingers actually touch): each
touching digit's contact is the point on the ball nearest its three finger
segments (MCP-PIP, PIP-DIP, DIP-tip as capsules of its pad radius; the
thumb's MCP-IP and IP-tip), and the segment says which phalanx touches. The
per-contact forces are then re-derived at those points from the frame's
net wrench (the source's derived_wrench, carried into the palm frame), as in
tools/import_demo_leader.py. Thumb curl (its pad side): opposite to the bend
at its end (IP) joint, since pressing with the pad bends that joint
backwards (hyperextension under load).
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import build_human_hand as bh  # noqa: E402

FIT_RMS_MAX_M = 0.006
PALM = {0: "wrist", 5: "index", 9: "middle", 13: "ring", 17: "little"}
SEGS = {"thumb": ((2, 3, "middle"), (3, 4, "distal")),
        "index": ((5, 6, "proximal"), (6, 7, "middle"), (7, 8, "distal")),
        "middle": ((9, 10, "proximal"), (10, 11, "middle"), (11, 12, "distal")),
        "ring": ((13, 14, "proximal"), (14, 15, "middle"), (15, 16, "distal")),
        "little": ((17, 18, "proximal"), (18, 19, "middle"), (19, 20, "distal"))}


def skeleton_contact(S, digit, c, R, pad, pad_point=None):
    """(point on the ball where the digit touches, gap m, phalanx) or None.
    Candidates: each segment whose two ends are both tracked (a capsule of
    the pad radius), and the tracked pad point itself (a point contact, the
    distal phalanx); the one whose surface comes nearest the ball's wins.
    (Until 26 Oct a lone proximal segment could win when the joints beyond
    it were untracked, putting the contact on a radial line toward the
    knuckle; George, moment 430.)"""
    best = None
    for a_, b_, ph in SEGS[digit]:
        if S[a_] is None or S[b_] is None:
            continue
        A, B = np.array(S[a_]), np.array(S[b_])
        ab = B - A
        t_ = min(1.0, max(0.0, float((c - A) @ ab) / max(float(ab @ ab), 1e-16)))
        x = A + t_ * ab
        gap = float(np.linalg.norm(x - c)) - pad - R
        if best is None or abs(gap) < abs(best[0]):
            best = (gap, x, ph)
    if pad_point is not None:
        pp = np.array(pad_point)
        gap = float(np.linalg.norm(pp - c)) - R
        if best is None or abs(gap) < abs(best[0]):
            best = (gap, pp, "distal")
    if best is None:
        return None
    gap, x, ph = best
    return c + R * (x - c) / np.linalg.norm(x - c), gap, ph


def cone_split(points, c, W, mu=1.0):
    """Least-squares non-negative cone forces at points giving wrench W."""
    from scipy.optimize import nnls
    sys.path.insert(0, ROOT)
    from gramian_workshop.vendor import stability_core as stability
    R = float(np.linalg.norm(points[0] - c))
    nrm = [(c - p) / np.linalg.norm(c - p) for p in points]
    Gm = stability.cone_generators(list(points), nrm, c, mu, 8)
    sc = np.r_[np.ones(3), np.ones(3) / R]
    A = Gm * sc[:, None]
    lam, _ = nnls(np.vstack([A, 1e-3 * np.eye(A.shape[1])]), np.concatenate([W * sc, np.zeros(A.shape[1])]))
    f = Gm[:3] * lam
    return [f[:, 8 * i:8 * i + 8].sum(1) for i in range(len(points))], \
        float(np.linalg.norm(A @ lam - W * sc) / max(np.linalg.norm(W * sc), 1e-12))


def kabsch(P, Q):
    """R, t minimising |R P + t - Q| (rows are points)."""
    pc, qc = P.mean(0), Q.mean(0)
    H = (P - pc).T @ (Q - qc)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T
    return R, qc - R @ pc, d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--leader", required=True)
    ap.add_argument("--keypoints", required=True)
    ap.add_argument("--dims", required=True)
    ap.add_argument("--arch", action="store_true", default=True)
    a = ap.parse_args()
    L = json.load(open(a.leader))
    K = json.load(open(a.keypoints))
    geo = bh.measured_geometry(a.dims)
    model = {0: np.zeros(3)}
    for j, f in PALM.items():
        if j:
            p0 = geo[f][0].copy()
            p0[2] = bh.ARCH_PALMAR_M[f] if a.arch else 0.0
            model[j] = p0
    kp = {round(f["t"], 4): f for f in K["frames"]}
    pads = dict(zip(K["pad_digits"], K["pad_offset_m"]))
    R_ball = float(L["object"]["radius_m"])
    digits = L["stations"]
    gaps, moved = [], []
    out, rms_all, refl, thumb = [], [], 0, []
    for f in L["frames"]:
        kf = kp.get(round(f["t"], 4))
        if kf is None:
            continue
        J = kf["joints"]
        have = [j for j in PALM if J[j] is not None]
        if len(have) < 3:
            continue
        P = np.array([J[j] for j in have])
        Q = np.array([model[j] for j in have])
        R, t, d = kabsch(P, Q)
        refl += d < 0
        rms = float(np.sqrt(np.mean(np.sum((P @ R.T + t - Q) ** 2, 1))))
        if rms > FIT_RMS_MAX_M or d < 0:
            continue
        rms_all.append(rms)
        X = lambda v: None if v is None else np.round(R @ np.array(v) + t, 6).tolist()
        V = lambda v: None if v is None else np.round(R @ np.array(v), 6).tolist()
        skel = [X(v) for v in J]
        if all(J[i] is not None for i in (1, 2, 3, 4)):
            cmc, mcp, ip, tip = (np.array(skel[i]) for i in (1, 2, 3, 4))
            y = (mcp - cmc) / np.linalg.norm(mcp - cmc)
            u = (ip - mcp) / np.linalg.norm(ip - mcp)
            w = (tip - ip) - ((tip - ip) @ u) * u       # the bend at the IP joint
            if np.linalg.norm(w) > 1e-6:
                # pressing with the pad bends the thumb's IP (and MCP) joint
                # backwards, so the measured bend points to the back of the
                # thumb: the pad faces the other way (George 26 Oct: "it's the
                # pad that touches for the thumb")
                thumb.append((cmc, y, -w / np.linalg.norm(w)))
        # contacts from the skeleton, and forces re-derived at them
        c_h = R @ np.array(f["obj_pos"]) + t
        cps, phal, ok_frame = [None] * len(digits), [None] * len(digits), True
        for i, dg in enumerate(digits):
            if not f["contact"][i]:
                continue
            sc_ = skeleton_contact(skel, dg, c_h, R_ball, pads[dg], X(f["station_pos"][i]))
            if sc_ is None:
                ok_frame = False
                break
            cps[i], phal[i] = sc_[0], sc_[2]
            gaps.append(sc_[1])
            if f["contact_point"][i] is not None:
                moved.append(float(np.linalg.norm(sc_[0] - np.array(X(f["contact_point"][i])))))
        if not ok_frame:
            continue
        Wd = f.get("derived_wrench")
        touch = [i for i in range(len(digits)) if cps[i] is not None]
        forces = [[0.0, 0.0, 0.0]] * len(digits)
        if touch and Wd:
            W = np.concatenate([R @ np.array(Wd["force_N"]), R @ np.array(Wd["torque_Nm"])])
            fs, _ = cone_split([cps[i] for i in touch], c_h, W)
            for i, fv in zip(touch, fs):
                forces[i] = np.round(fv, 5).tolist()
        out.append({"t": f["t"], "obj_pos": X(f["obj_pos"]), "obj_quat": None,
                    "station_pos": [X(v) for v in f["station_pos"]], "contact": f["contact"],
                    "contact_point": [None if v is None else np.round(v, 6).tolist() for v in cps],
                    "contact_phalanx": phal,
                    "contact_normal": [None if v is None else np.round((c_h - v) / np.linalg.norm(c_h - v), 5).tolist()
                                       for v in cps],
                    "force_on_object_N": forces,
                    "skeleton": skel, "palm_fit_rms_m": round(rms, 5), "palm_points": len(have)})
    name = L["id"].replace("_derived", "") + "_handframe"
    lead = {**{k: v for k, v in L.items() if k != "frames"}, "id": name,
            "name": L["name"] + ", in the demonstrator's palm frame",
            "frame": {"units": "m", "note": "the model hand's frame (tools/build_human_hand.py --measured): "
                      "wrist at the origin, x toward the thumb, y distal, z palmar; per-frame rigid fit of the "
                      "observed wrist and knuckles", "gravity": "varies per frame (the derived forces carry it)"},
            "handframe": {"script": "tools/demo_handframe.py", "fit_rms_max_m": FIT_RMS_MAX_M,
                          "frames_kept": len(out), "frames_in": len(L["frames"]),
                          "fit_rms_median_m": float(np.median(rms_all)), "reflections_rejected": int(refl)},
            "frames": out}
    p_out = os.path.join(ROOT, "data", "leaders", name + ".json")
    json.dump(lead, open(p_out, "w"), separators=(",", ":"))
    c0 = out[0]["obj_pos"]
    fixed = {"placement": {"rotation_matrix": np.eye(3).tolist(), "translation_m": c0},
             "assignment": {d_: i for i, d_ in enumerate(L["stations"])},
             "respace_deg": [0.0] * L["n_stations"], "placement_cost_mm": 0.0,
             "note": "the demonstrator's own palm frame: no placement search"}
    json.dump(fixed, open(os.path.join(ROOT, "outputs", "handframe_" + name.replace("_handframe", "") + ".json"), "w"),
              indent=1)
    cmc = np.median([x[0] for x in thumb], 0)
    y = np.median([x[1] for x in thumb], 0)
    y /= np.linalg.norm(y)
    w = np.median([x[2] for x in thumb], 0)
    w -= (w @ y) * y
    w /= np.linalg.norm(w)
    th = {"source": a.keypoints, "frames": len(thumb), "frame": "model hand frame",
          "cmc_m": cmc.round(5).tolist(), "metacarpal_dir": y.round(4).tolist(), "curl_dir": w.round(4).tolist()}
    json.dump(th, open(os.path.join(ROOT, "data", "hands", "measured", "george_thumb_from_keypoints.json"), "w"),
              indent=1)
    print(f"wrote {p_out}: {len(out)} of {len(L['frames'])} frames kept, palm fit rms median "
          f"{np.median(rms_all) * 1e3:.1f} mm, {refl} mirrored fits rejected")
    print(f"contacts from the skeleton: median {np.median(moved) * 1e3:.1f} mm from the source's tip-direction "
          f"points (p90 {np.percentile(moved, 90) * 1e3:.1f}); finger-ball gap at contact median "
          f"{np.median(gaps) * 1e3:.1f} mm (p10 {np.percentile(gaps, 10) * 1e3:.1f}, p90 {np.percentile(gaps, 90) * 1e3:.1f})")
    print(f"thumb from {len(thumb)} frames: base {np.round(cmc * 1e3, 1)} mm, metacarpal {y.round(3)}, "
          f"curl {w.round(3)}")
    ball = np.array([fr["obj_pos"] for fr in out])
    print("ball centre in the palm frame, median", np.round(np.median(ball, 0) * 1e3, 1), "mm; spread (p10-p90)",
          np.round((np.percentile(ball, 90, 0) - np.percentile(ball, 10, 0)) * 1e3, 1), "mm")


if __name__ == "__main__":
    main()
