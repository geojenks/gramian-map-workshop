"""Can the model hand take the demonstrator's real poses? For sampled frames
of a palm-frame demo (tools/demo_handframe.py), fit each digit's model joint
angles to the measured joint centres (PIP, DIP and nail-end tip; the
thumb's MCP, IP and tip) by damped least squares from several starts, and
report the residual and which joints end at a range limit.

    python tools/skeleton_fit.py --hand george_kp --leader tennis_new_cup_take3_handframe [--frames 40]
"""
import argparse
import collections
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.follow import FingerIK  # noqa: E402
from gramian_workshop.hands import Hand, _mj  # noqa: E402

KP = {"thumb": (2, 3, 4), "index": (6, 7, 8), "middle": (10, 11, 12), "ring": (14, 15, 16), "little": (18, 19, 20)}


def points(m, d, f):
    """Model joint centres matching KP: the middle and distal bodies' origins
    and the nail-end tip, world frame, with the body each sits on."""
    dist = f.tip_body
    mid = int(m.body_parentid[dist])
    Rd = d.xmat[dist].reshape(3, 3)
    tip_local = np.array([0.0, f.hand.model.geom_size[[g for g in range(m.ngeom) if m.geom_bodyid[g] == dist][0]][1] * 2
                          + f.hand.model.geom_size[[g for g in range(m.ngeom) if m.geom_bodyid[g] == dist][0]][0], 0.0])
    return [(d.xpos[mid].copy(), mid), (d.xpos[dist].copy(), dist), (d.xpos[dist] + Rd @ tip_local, dist)]


def fit(hand, f, ik, target, starts=6, iters=150, rng=None):
    mj = _mj()
    m, d = hand.model, ik.d
    lo, hi = ik.rng[:, 0], ik.rng[:, 1]
    best = None
    for s in range(starts):
        q = np.array(hand.base_q)
        q[ik.qadr] = (lo + hi) / 2 if s == 0 else rng.uniform(lo, hi)
        for _ in range(iters):
            d.qpos[:] = hand.apply_coupling(q.copy())
            mj.mj_kinematics(m, d)
            mj.mj_comPos(m, d)
            rows, err = [], []
            for (p, b), tgt in zip(points(m, d, f), target):
                if tgt is None:
                    continue
                jp = np.zeros((3, m.nv))
                mj.mj_jac(m, d, jp, None, p, b)
                rows.append(ik._eff_jac(jp))
                err.append(np.asarray(tgt) - p)
            J, e = np.vstack(rows), np.concatenate(err)
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-5 * np.eye(len(e)), e)
            q[ik.qadr] = np.clip(q[ik.qadr] + dq, lo, hi)
        d.qpos[:] = hand.apply_coupling(q.copy())
        mj.mj_kinematics(m, d)
        res = [float(np.linalg.norm(np.asarray(tgt) - p)) for (p, _), tgt in zip(points(m, d, f), target)
               if tgt is not None]
        r = float(np.sqrt(np.mean(np.square(res))))
        if best is None or r < best[0]:
            at = [bool(abs(q[a] - l) < 1e-3 or abs(q[a] - h) < 1e-3) for a, l, h in zip(ik.qadr, lo, hi)]
            best = (r, res, np.degrees(q[ik.qadr]).round(0).tolist(), at)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hand", required=True)
    ap.add_argument("--leader", required=True)
    ap.add_argument("--frames", type=int, default=40)
    a = ap.parse_args()
    L = json.load(open(os.path.join(ROOT, "data", "leaders", a.leader + ".json")))
    hand = Hand(a.hand)
    m = hand.model
    iks = {f.label: FingerIK(hand, f) for f in hand.fingers}
    jn = {f.label: [_mj().mj_id2name(m, _mj().mjtObj.mjOBJ_JOINT, j) for j in iks[f.label].free] for f in hand.fingers}
    rng = np.random.default_rng(0)
    ks = np.linspace(0, len(L["frames"]) - 1, a.frames).astype(int)
    out = collections.defaultdict(list)
    for k in ks:
        S = L["frames"][k]["skeleton"]
        for f in hand.fingers:
            tgt = [S[j] for j in KP[f.label]]
            if sum(t is not None for t in tgt) < 2:
                continue
            out[f.label].append(fit(hand, f, iks[f.label], tgt, rng=rng))
    print(f"{a.hand} on {a.leader}: model joint centres fitted to the measured ones, {a.frames} frames")
    for lab, rs in out.items():
        r = np.array([x[0] for x in rs]) * 1e3
        lim = collections.Counter(j for x in rs for j, at in zip(jn[lab], x[3]) if at)
        ang = np.median(np.array([x[2] for x in rs]), 0)
        print(f"  {lab:6s} rms median {np.median(r):4.1f} mm, p90 {np.percentile(r, 90):4.1f} mm; "
              f"median angles {dict(zip(jn[lab], ang))}; at a range limit: {dict(lim)}")


if __name__ == "__main__":
    main()


def score_real(hand_key, leader_id, n=40):
    """Grade the demonstrator's own (fitted) poses with the follow test's
    contact measures: for each touching digit, how far the model link's
    touch is from the recorded contact, the side angle of that touch, the
    gap between link and ball (negative: sunk in) and the deepest sinking of
    any of the digit's links."""
    from gramian_workshop import follow as F
    L = json.load(open(os.path.join(ROOT, "data", "leaders", leader_id + ".json")))
    hand = Hand(hand_key)
    mj = _mj()
    iks = {f.label: FingerIK(hand, f) for f in hand.fingers}
    st = {d: i for i, d in enumerate(L["stations"])}
    R = float(L["object"]["radius_m"])
    rng = np.random.default_rng(0)
    rows = collections.defaultdict(list)
    for k in np.linspace(0, len(L["frames"]) - 1, n).astype(int):
        fr = L["frames"][k]
        S, c = fr["skeleton"], np.array(fr["obj_pos"])
        for f in hand.fingers:
            i = st[f.label]
            if not fr["contact"][i]:
                continue
            tgt = [S[j] for j in KP[f.label]]
            if sum(t is not None for t in tgt) < 2:
                continue
            ik = iks[f.label]
            r = fit(hand, f, ik, tgt, rng=rng)
            # fit leaves ik.d at the best start's pose only if it was the last; refit from it
            q = np.array(hand.base_q)
            q[ik.qadr] = np.radians(r[2])
            ik.d.qpos[:] = hand.apply_coupling(q)
            mj.mj_kinematics(hand.model, ik.d)
            target = np.array(fr["contact_point"][i])
            best = None
            for b, (a_l, b_l, rad, side_l) in ik.proxies(True).items():
                Rb, pb = ik.d.xmat[b].reshape(3, 3), ik.d.xpos[b]
                A, B = pb + Rb @ a_l, pb + Rb @ b_l
                ab = B - A
                tt = min(1.0, max(0.0, float((c - A) @ ab) / max(float(ab @ ab), 1e-16)))
                x = A + tt * ab                      # the link axis point nearest the centre
                n_out = (x - c) / np.linalg.norm(x - c)
                gap = float(np.linalg.norm(x - c)) - rad - R
                touch = c + R * n_out
                side = float(np.degrees(np.arccos(np.clip((Rb @ side_l) @ -n_out, -1, 1))))
                cand = (abs(gap), float(np.linalg.norm(touch - target)), side, gap, b)
                if best is None or cand < best:
                    best = cand
            pen = float(ik.penetration((c, R))[0])
            rows[f.label].append((r[0], best[1], best[2], best[3], pen))
    print(f"{hand_key}: the demonstrator's own poses graded by the follow test's contact measures")
    print("  digit   fit rms | touch-to-contact | side angle | link gap (+ apart, - sunk) | deepest sinking  (medians, p90)")
    for lab, rs in rows.items():
        A = np.array(rs)
        med, p90 = np.median(A, 0), np.percentile(A, 90, 0)
        print(f"  {lab:6s} {med[0]*1e3:5.1f} | {med[1]*1e3:5.1f} ({p90[1]*1e3:4.1f}) mm | {med[2]:4.0f} ({p90[2]:4.0f}) deg | "
              f"{med[3]*1e3:5.1f} mm | {med[4]*1e3:5.1f} ({p90[4]*1e3:4.1f}) mm   n={len(rs)}")
