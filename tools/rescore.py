"""Re-score saved placements without searching again (follow(placement=...)).

    python tools/rescore.py published   # outputs/follow_<hand>__<tennis leader>.json
    python tools/rescore.py reach-only  # outputs/reach_only_<hand>.json (L20, Shadow)

Writes outputs/rescore_<which>_<hand>.json and prints, per finger, reach,
facing and state.
"""
import collections
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.follow import follow  # noqa: E402
from gramian_workshop.hands import HANDS  # noqa: E402

LEADER = "w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n{n}"


def main():
    which = sys.argv[1]
    hands = sys.argv[2:] or (list(HANDS) if which == "published" else ["l20", "shadow"])
    for hk in hands:
        lid = LEADER.format(n=len(HANDS[hk]["fingers"]))
        L = json.load(open(os.path.join(ROOT, "data", "leaders", lid + ".json")))
        if which == "published":
            src = json.load(open(os.path.join(ROOT, "outputs", f"follow_{hk}__{lid}.json")))
        else:
            src = json.load(open(os.path.join(ROOT, "outputs", f"reach_only_{hk}.json")))
            src["placement_cost_mm"] = src["mean_reach_mm"]
        r = follow(hk, L, n_eval=40, placement=src)
        r["rescored_from"] = which
        json.dump(r, open(os.path.join(ROOT, "outputs", f"rescore_{which}_{hk}.json"), "w", encoding="utf-8"),
                  indent=1, default=float)
        v = collections.Counter(f["verdict"] for f in r["frames"])
        why = collections.Counter(f["reason"] for f in r["frames"] if f["verdict"] == "red")
        print(f"{hk} [{which}]: {dict(v)}; red {dict(why)}", flush=True)
        per = collections.defaultdict(list)
        for f in r["frames"]:
            for x in f["fingers"]:
                if x["state"] != "off":
                    per[x["finger"]].append(x)
        for k, xs in per.items():
            rm = np.array([x["reach_mm"] for x in xs])
            fd = np.array([x["facing_deg"] for x in xs])
            ub = [x["unbalanced_Nm"] for x in xs if "unbalanced_Nm" in x]
            st = collections.Counter(x.get("reason") or x["state"] for x in xs)
            print(f"   {k:7s} reach med {np.median(rm):5.1f} max {rm.max():5.1f} mm; facing med "
                  f"{np.median(fd):4.0f} max {fd.max():4.0f} deg"
                  + (f"; unbalanced joint torque med {1e3 * np.median(ub):.2f} mN.m" if ub else "")
                  + f"; {dict(st)}", flush=True)


if __name__ == "__main__":
    main()
