"""Compare a follow result made with continuity off against one made with
it on (follow(continuity=True), issue 12): the same hand, leader, placement
and checked frames. Prints counts and quantiles only; writes nothing.

    python tools/continuity_summary.py <off.json> <on.json>

The jump of a finger between checked frames is its pose at this frame less
its pose at the previous frame where it touched (from the recorded "q" and
the leader's frame times), for both runs alike; the on run also carries its
own readout ("continuity"), which is checked against that recomputation.
"""
import json
import os
import sys
from collections import Counter

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def jumps(result, times):
    """{finger: [(k, max |dq| rad, dt s)]} between successive touching frames."""
    last, out = {}, {}
    for fr in result["frames"]:
        for x in fr["fingers"]:
            if x["state"] == "off":
                continue
            q = np.array(x["q"], float)
            if x["finger"] in last:
                k0, q0 = last[x["finger"]]
                out.setdefault(x["finger"], []).append(
                    (fr["k"], float(np.abs(q - q0).max()), times[fr["k"]] - times[k0]))
            last[x["finger"]] = (fr["k"], q)
    return out


def quantiles(v):
    v = np.asarray(v, float)
    if not len(v):
        return "none"
    return "median %.3f  p90 %.3f  max %.3f (n=%d)" % (np.median(v), np.percentile(v, 90), v.max(), len(v))


def main(off_path, on_path):
    off, on = (json.load(open(p, encoding="utf-8")) for p in (off_path, on_path))
    if [f["k"] for f in off["frames"]] != [f["k"] for f in on["frames"]]:
        sys.exit("the two runs checked different frames")
    if off["placement"] != on["placement"] or off.get("assignment") != on.get("assignment"):
        sys.exit("the two runs used different placements or assignments")
    leader = os.path.join(ROOT, "data", "leaders", on["leader"] + ".json") \
        if isinstance(on.get("leader"), str) else None
    if leader is None or not os.path.exists(leader):
        sys.exit("leader file not found: " + str(leader))
    times = [float(f["t"]) for f in json.load(open(leader, encoding="utf-8"))["frames"]]
    print("hand", on["hand"], "leader", on["leader"], "checked frames", len(on["frames"]),
          "same placement: yes")
    print("verdicts  off:", dict(Counter(f["verdict"] for f in off["frames"])))
    print("verdicts  on: ", dict(Counter(f["verdict"] for f in on["frames"])))
    changed = [(a["k"], a["verdict"], b["verdict"]) for a, b in zip(off["frames"], on["frames"])
               if a["verdict"] != b["verdict"]]
    print("frames whose verdict changed:", len(changed), changed)
    states = Counter()
    for fa, fb in zip(off["frames"], on["frames"]):
        for xa, xb in zip(fa["fingers"], fb["fingers"]):
            states[(xa["state"], xb["state"])] += 1
    print("finger states (off, on):", dict(states))
    joff, jon = jumps(off, times), jumps(on, times)
    print("max |dq| per step, rad   off:", quantiles([j for v in joff.values() for _, j, _ in v]))
    print("max |dq| per step, rad   on: ", quantiles([j for v in jon.values() for _, j, _ in v]))
    print("step dt, s:", quantiles([d for v in jon.values() for _, _, d in v]))
    # the on run's own readout
    start, link, spans, pts, speeds, mism = Counter(), Counter(), 0, [], [], 0
    recomputed = {f: dict((k, j) for k, j, _ in v) for f, v in jon.items()}
    for fr in on["frames"]:
        for x in fr["fingers"]:
            co = x.get("continuity")
            if co is None:
                continue
            start[co["start"]] += 1
            link[co["link_changed"]] += 1
            pts.append(co["speed_points"])
            if "jump" in co:
                spans += bool(co["spans_off"])
                mine = max(abs(j["dq"]) for j in co["jump"])
                if abs(mine - recomputed[x["finger"]][fr["k"]]) > 1e-9:
                    mism += 1
            if co["max_hinge_rad_s"] is not None:
                speeds.append(co["max_hinge_rad_s"])
    print("on run starts:", dict(start), " link changed:", dict(link), " jumps over an off spell:", spans,
          " readout jumps disagreeing with the recomputation:", mism)
    print("fitted max hinge speed, rad/s:", quantiles(speeds), " fits", len(speeds), "of", len(pts),
          "(window points median %s)" % (np.median(pts) if pts else "none"))
    lim = on["settings"].get("continuity", {}).get("speed_limit_rad_s")
    print("speed limit:", lim, " red for speed:",
          sum("speed" in x.get("missed", []) for f in on["frames"] for x in f["fingers"]))
    print("elapsed s  off:", off.get("elapsed_s"), " on:", on.get("elapsed_s"))


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
