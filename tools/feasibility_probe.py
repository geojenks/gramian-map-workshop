"""Is a non-green finger limited by the hand, or by the solver? For each
touching finger at each moment of a follow result, re-solve the tangent
contact from N random starting poses on each allowed phalanx (same placement,
same targets, same rules) and compare the best grade with the one reported.

    python tools/feasibility_probe.py outputs/follow_<...>.json [--starts 24]
"""
import argparse
import collections
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop import follow as F  # noqa: E402
from gramian_workshop.hands import Hand  # noqa: E402


def grade(r):
    if r[1] <= F.REACH_GREEN_M and r[2] <= F.SIDE_GREEN_DEG and r[5] <= F.PEN_GREEN_M:
        return "green"
    if r[1] <= F.REACH_AMBER_M and r[2] <= F.SIDE_AMBER_DEG and r[5] <= F.PEN_AMBER_M:
        return "amber"
    return "red"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("result")
    ap.add_argument("--starts", type=int, default=24)
    a = ap.parse_args()
    src = json.load(open(a.result))
    L = json.load(open(os.path.join(ROOT, "data", "leaders", src["leader"] + ".json")))
    hand = Hand(src["hand"])
    leader = F.Leader(L)
    leader.phi = np.radians(src.get("respace_deg") or np.zeros(leader.n))
    Rw = np.array(src["placement"]["rotation_matrix"])
    t = np.array(src["placement"]["translation_m"])
    iks = {f.label: F.FingerIK(hand, f) for f in hand.fingers}
    rng = np.random.default_rng(0)
    tally = collections.Counter()
    worst = []
    for fr in src["frames"]:
        k = fr["k"]
        c = leader.center(k, Rw, t)
        for x in fr["fingers"]:
            if x["state"] == "off" or "reach_mm" not in x:
                continue
            ik = iks[x["finger"]]
            p, _, _ = leader.station(k, x["station"], Rw, t)
            lo, hi = ik.rng[:, 0], ik.rng[:, 1]
            best = None
            for b in ik.proxies(True):
                for _ in range(a.starts):
                    r = ik.solve_tangent(b, p, c, leader.R_obj, rng.uniform(lo, hi))
                    key = ({"green": 0, "amber": 1, "red": 2}[grade(r)], r[1], r[2])
                    if best is None or key < best[0]:
                        best = (key, r)
            g = grade(best[1])
            reported = "green" if x["state"] == "green" else ("amber" if x["state"] == "amber" else "red")
            # the finger's own geometric grade at the reported pose (its state
            # also folds in force, so compare like with like)
            rep_geo = grade((None, x["reach_mm"] / 1e3, x["side_deg"], None, None, x["penetration_mm"] / 1e3))
            tally[(rep_geo, g)] += 1
            if {"green": 0, "amber": 1, "red": 2}[g] < {"green": 0, "amber": 1, "red": 2}[rep_geo]:
                worst.append((k, x["finger"], rep_geo, g, round(x["reach_mm"], 1), round(best[1][1] * 1e3, 1),
                              round(x["side_deg"]), round(best[1][2]), round(x["penetration_mm"], 1),
                              round(best[1][5] * 1e3, 1)))
    print("geometry grade: (reported, best of random starts) -> count")
    for kk, v in sorted(tally.items()):
        print("  ", kk, v)
    print(f"{len(worst)} finger-moments where more starts find a better grade "
          "(k, finger, reported, found, reach mm rep/found, side deg rep/found, sunk mm rep/found):")
    for w in worst[:25]:
        print("  ", w)


if __name__ == "__main__":
    main()
