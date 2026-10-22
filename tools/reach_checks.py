"""Two follow-ups to the reach ladder (22 Oct), on the tennis-ball leaders.

    python tools/reach_checks.py reach-only l20 shadow
    python tools/reach_checks.py wrist shadow

reach-only  is there any placement from which every finger reaches its
            station? The placement search scored on reach alone (facing left
            out of the cost and the IK), then the geometry floor
            (follow.reach_bound) per finger at that placement.
wrist       the full follow check with the wrist joints re-chosen at every
            moment (follow(wrist=True)), against the fixed-wrist default.

Writes outputs/reach_only_<hand>.json and outputs/follow_<hand>__<leader>__wrist.json.
"""
import collections
import json
import multiprocessing as mp
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.follow import follow, reach_only  # noqa: E402
from gramian_workshop.hands import HANDS  # noqa: E402

LEADER = "w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n{n}"


def job(args):
    kind, hk = args
    lid = LEADER.format(n=len(HANDS[hk]["fingers"]))
    with open(os.path.join(ROOT, "data", "leaders", lid + ".json")) as f:
        L = json.load(f)
    t0 = time.time()
    if kind == "reach-only":
        cost, assign, floor, res = reach_only(hk, L)
        out = {"hand": hk, "leader": lid, "mean_reach_mm": cost * 1e3, "assignment": assign,
               "floor": floor, **res, "elapsed_s": round(time.time() - t0, 1)}
        with open(os.path.join(ROOT, "outputs", f"reach_only_{hk}.json"), "w") as f:
            json.dump(out, f, indent=1)
        lines = [f"{hk} reach-only placement: mean reach {cost * 1e3:.1f} mm; {assign}; {out['elapsed_s']} s"]
        lines += [f"   {k:7s} floor med {v['median_mm']:5.1f} p90 {v['p90_mm']:5.1f} max {v['max_mm']:5.1f} mm"
                  for k, v in floor.items() if v]
    else:
        r = follow(hk, L, n_eval=40, wrist=True)
        r["elapsed_s"] = round(time.time() - t0, 1)
        with open(os.path.join(ROOT, "outputs", f"follow_{hk}__{lid}__wrist.json"), "w") as f:
            json.dump(r, f, indent=1, default=float)
        v = collections.Counter(fr["verdict"] for fr in r["frames"])
        why = collections.Counter(fr["reason"] for fr in r["frames"] if fr["verdict"] == "red")
        lines = [f"{hk} moving wrist: {dict(v)}; red {dict(why)}; placement {r['placement_cost_mm']:.1f} mm; {r['elapsed_s']} s"]
        per = collections.defaultdict(list)
        for fr in r["frames"]:
            for x in fr["fingers"]:
                if x["state"] != "off":
                    per[x["finger"]].append(x)
        for k, xs in per.items():
            st = collections.Counter(x.get("reason") or x["state"] for x in xs)
            rm = sorted(x["reach_mm"] for x in xs)
            lines.append(f"   {k:7s} reach med {rm[len(rm) // 2]:5.1f} max {rm[-1]:5.1f} mm; {dict(st)}")
    return "\n".join(lines)


def main():
    kind, hands = sys.argv[1], sys.argv[2:]
    with mp.Pool(len(hands)) as pool:
        for s in pool.imap(job, [(kind, h) for h in hands]):
            print(s, flush=True)


if __name__ == "__main__":
    main()
