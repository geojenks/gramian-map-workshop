"""Why do rigid hands reach the leader badly? Change one thing at a time,
cumulatively, and record where each hand fails at each step.

    python tools/reach_ladder.py [--hands allegro shadow leap l20] [--frames 40]

Steps (each keeps the ones before it):
  S0  baseline: distal phalanx only, standard placement search, 60 IK iterations
  S1  contact also allowed on the middle phalanx
  S2  finer placement search: 10 sample moments (6), 80 tries (60),
      4 rounds (3), every 2nd pad point (every 3rd)
  S3  longer joint solve: 200 IK iterations (60)
and, for every step, a geometry-only floor (follow.reach_bound): how far each
station's contact path lies from anywhere that finger's pad can be, with the
step's placement, ignoring facing, force and collisions.

Writes outputs/reach_ladder.json; each step's full result goes to
outputs/ladder/follow_<hand>__<leader>__<step>.json.
"""
import argparse
import collections
import json
import multiprocessing as mp
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.follow import follow, reach_bound  # noqa: E402
from gramian_workshop.hands import HANDS  # noqa: E402

LEADER = "w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n{n}"
FINE = {"n_frames": 10, "n_samples": 80, "iters": 4, "pad_step": 2}
STEPS = [("S0", "baseline", {}),
         ("S1", "+ middle phalanx", {"middle": True}),
         ("S2", "+ finer placement search", {"middle": True, "place": FINE}),
         ("S3", "+ 200 IK iterations", {"middle": True, "place": FINE, "ik_iters": 200})]


def summarise(r, bound):
    fr = r["frames"]
    v = collections.Counter(f["verdict"] for f in fr)
    why = collections.Counter(f["reason"] for f in fr if f["verdict"] == "red" and f["reason"])
    per = collections.defaultdict(lambda: collections.Counter())
    reach = collections.defaultdict(list)
    face = collections.defaultdict(list)
    mid = collections.Counter()
    for f in fr:
        for x in f["fingers"]:
            if x["state"] == "off":
                continue
            per[x["finger"]][x.get("reason") or x["state"]] += 1
            reach[x["finger"]].append(x["reach_mm"])
            face[x["finger"]].append(x["facing_deg"])
            mid[x["finger"]] += x.get("phalanx") == "middle"
    fingers = {}
    for k in reach:
        fingers[k] = {"states": dict(per[k]),
                      "reach_mm_median": float(np.median(reach[k])),
                      "reach_mm_max": float(np.max(reach[k])),
                      "facing_deg_median": float(np.median(face[k])),
                      "middle_phalanx_share": mid[k] / len(reach[k]),
                      "floor_mm": bound.get(k)}
    return {"verdicts": {s: v.get(s, 0) for s in ("green", "amber", "red")},
            "red_frame_reasons": dict(why), "placement_cost_mm": r["placement_cost_mm"],
            "assignment": r["assignment"], "fingers": fingers}


def run_hand(args):
    hk, frames = args
    n = len(HANDS[hk]["fingers"])
    lid = LEADER.format(n=n)
    with open(os.path.join(ROOT, "data", "leaders", lid + ".json")) as f:
        L = json.load(f)
    out = {}
    os.makedirs(os.path.join(ROOT, "outputs", "ladder"), exist_ok=True)
    for sid, label, kw in STEPS:
        t0 = time.time()
        r = follow(hk, L, n_eval=frames, **kw)
        r["elapsed_s"] = round(time.time() - t0, 1)
        with open(os.path.join(ROOT, "outputs", "ladder", f"follow_{hk}__{lid}__{sid}.json"), "w") as f:
            json.dump(r, f, indent=1, default=float)
        bound = reach_bound(hk, L, r, middle=kw.get("middle", False))
        s = summarise(r, bound)
        s.update({"step": sid, "label": label, "elapsed_s": r["elapsed_s"]})
        out[sid] = s
        print(f"{hk} {sid} {label}: {s['verdicts']} red {s['red_frame_reasons']}; "
              f"placement {s['placement_cost_mm']:.1f} mm; {r['elapsed_s']} s", flush=True)
        for k, x in s["fingers"].items():
            fl = x["floor_mm"]
            print(f"   {k:7s} reach med {x['reach_mm_median']:5.1f} max {x['reach_mm_max']:5.1f} mm, "
                  f"facing med {x['facing_deg_median']:4.0f} deg, middle {x['middle_phalanx_share']:.0%}, "
                  f"floor med {fl['median_mm'] if fl else float('nan'):5.1f} max "
                  f"{fl['max_mm'] if fl else float('nan'):5.1f} mm; {x['states']}", flush=True)
    return hk, lid, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hands", nargs="*", default=list(HANDS))
    ap.add_argument("--frames", type=int, default=40)
    a = ap.parse_args()
    with mp.Pool(min(len(a.hands), os.cpu_count() or 1)) as pool:
        res = pool.map(run_hand, [(h, a.frames) for h in a.hands])
    doc = {"steps": [{"id": s, "label": l, "settings": kw} for s, l, kw in STEPS],
           "hands": {hk: {"leader": lid, "steps": out} for hk, lid, out in res}}
    with open(os.path.join(ROOT, "outputs", "reach_ladder.json"), "w") as f:
        json.dump(doc, f, indent=1, default=float)
    print("wrote outputs/reach_ladder.json")


if __name__ == "__main__":
    main()
