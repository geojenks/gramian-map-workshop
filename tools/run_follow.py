"""Run the follow-the-leader check for each rigid hand on the leader that
matches its finger count, and save outputs/follow_<hand>__<leader>.json.

    python tools/run_follow.py [--hands allegro shadow leap l20] [--frames 40] [--respace]

--respace lets the stations move round the ring to suit each hand (outputs
get a __respaced suffix).

Needs mujoco and the hand models (tools/fetch_hands.py).
"""
import argparse
import collections
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.follow import follow  # noqa: E402
from gramian_workshop.hands import HANDS  # noqa: E402

LEADERS = {"big": "w10_cup_physical_x0.3_a0.15_sl0.3t_n{n}",
           "tennis": "w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n{n}"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hands", nargs="*", default=list(HANDS))
    ap.add_argument("--leader", choices=list(LEADERS), default="tennis")
    ap.add_argument("--frames", type=int, default=40,
                    help="leader frames evaluated, spread over the recording")
    ap.add_argument("--tag", default="", help="suffix for the output file names")
    ap.add_argument("--leader-id", default=None,
                    help="a leader file in data/leaders/ by id, for every hand (e.g. a human demo)")
    ap.add_argument("--respace", action="store_true",
                    help="move stations round the ring to suit the hand")
    a = ap.parse_args()
    for hk in a.hands:
        n = len(HANDS[hk]["fingers"])
        lid = a.leader_id or LEADERS[a.leader].format(n=n)
        with open(os.path.join(ROOT, "data", "leaders", lid + ".json")) as f:
            L = json.load(f)
        t0 = time.time()
        r = follow(hk, L, n_eval=a.frames, respace=a.respace)
        r["elapsed_s"] = round(time.time() - t0, 1)
        out = os.path.join(ROOT, "outputs", f"follow_{hk}__{lid}" + ("__respaced" if a.respace else "") + (f"__{a.tag}" if a.tag else "") + ".json")
        with open(out, "w", encoding="utf-8") as f:
            json.dump(r, f, indent=1, default=float)
        v = collections.Counter(fr["verdict"] for fr in r["frames"])
        why = collections.Counter(fr["reason"] for fr in r["frames"] if fr["reason"])
        fin = collections.Counter((x["finger"], x["state"]) for fr in r["frames"]
                                  for x in fr["fingers"])
        print(f"{hk} on {lid}: {dict(v)}; red because {dict(why)}; "
              f"assignment {r['assignment']}; placement cost "
              f"{r['placement_cost_mm']:.2f} mm; respaced by {[round(x) for x in r['respace_deg']]} deg; "
              f"{r['elapsed_s']} s", flush=True)
        print("   per finger:", dict(sorted(fin.items())), flush=True)


if __name__ == "__main__":
    main()
