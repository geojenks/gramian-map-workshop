"""Margin summary of tools/trajectory_sweep.py's outputs.

    python tools/trajectory_summary.py [dir] [--nnls DIR] [--table]

dir defaults to outputs/trajectory/. Per hand, leader and setting, over
checked frames that hold on their own and carry torque, the scale margins
from: the unlinked LP (one frame, no link, the L1 residual budget); the LP
chain (sequential single-frame LPs, each boxed by the previous solution);
the pair (the lower of the frame's two pairwise margins); the run (the
whole run of consecutive holding frames, one scale); and, given --nnls,
follow's own chain and unlinked margins from follow runs with the same
link (dense__<follow output name>__<setting>.json and __off.json in DIR,
each with frames[].k and frames[].scale_margin). A run's margin is set by
its weakest frame, so each test is compared per run: its weakest frame's
margin over the run margin. --table prints only the medians of those
ratios as a markdown table. Order violations: run above pair, pair above
unlinked (beyond 0.02, above the bisection tolerance), or the LP chain
holding through a run's first break (the chain's input box is soft, so
this one can happen; see trajectory.py).
"""
import glob
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
args = [a for a in sys.argv[1:]]
TABLE = "--table" in args
# table labels for the measured human hands (the ids are in gramian_workshop/hands.py)
LABEL = {"george_clip": "human, clip skeleton", "george_kp": "human, measured",
         "george_kp_wide": "human, measured, wide ranges", "george_measured_thumb": "human, measured thumb base"}
B = None
if "--nnls" in args:
    B = args[args.index("--nnls") + 1]
    del args[args.index("--nnls"):args.index("--nnls") + 2]
args = [a for a in args if a != "--table"]
D = args[0] if args else os.path.join(ROOT, "outputs", "trajectory")


def m(x):
    return None if x is None else x.get("margin")


def stats(a):
    a = np.asarray([v for v in a if v is not None], float)
    if not len(a):
        return "n 0"
    return f"n {len(a)}, median {np.median(a):.3g}, 10% {np.percentile(a, 10):.3g}, min {a.min():.3g}, max {a.max():.3g}"


def ratio(num, den):
    return [a / b for a, b in zip(num, den) if a is not None and b is not None and b > 0]


def baseline(file, setting):
    if B is None:
        return None
    fn = os.path.join(B, f"dense__{os.path.splitext(file)[0]}__{setting}.json")
    if not os.path.exists(fn):
        return None
    r = json.load(open(fn, encoding="utf-8"))
    return {fr["k"]: m(fr["scale_margin"]) for fr in r["frames"]}


NICE = {"lag0.03": "lag 0.03 s", "lag0.1": "lag 0.1 s", "lag0.3": "lag 0.3 s",
        "rate1.0": "rate 1 /s", "rate3.0": "rate 3 /s"}
rows = []


def out(*a):
    if not TABLE:
        print(*a)


ORDER = ["lag0.03", "lag0.1", "lag0.3", "rate1.0", "rate3.0"]
res = {}
for f in glob.glob(os.path.join(D, "traj__*.json")):
    r = json.load(open(f, encoding="utf-8"))
    res[(r["hand"], r["leader"], r["setting"], r["per_frame"])] = r

for hand, leader in sorted({k[:2] for k in res}):
    out(f"\n== {hand} on {leader} ==")
    for s in ORDER:
        r = res.get((hand, leader, s, False))
        if r is None:
            continue
        tj = r["trajectory"]
        fr = tj["frames"]
        assert [x["k"] for x in fr] == r["frame_k"]
        pair_by_k = {}
        for p in tj["pairs"]:
            for k in (p["k_a"], p["k_b"]):
                v = m(p["margin"])
                pair_by_k[k] = v if k not in pair_by_k or v is None else min(pair_by_k[k], v)
        run_m = [m(x["margin"]) for x in tj["runs"]]
        keep = [x for x in fr if x["run"] is not None and x["unlinked_margin"] is not None]
        ks = [x["k"] for x in keep]
        unl = [m(x["unlinked_margin"]) for x in keep]
        chain = [m(x.get("chain_lp_margin")) for x in keep]
        pair = [pair_by_k.get(k) for k in ks]
        run = [run_m[x["run"]] for x in keep]
        nn = baseline(r["file"], s)
        off = baseline(r["file"], "off")
        for b in (nn, off):
            if b is not None:
                assert set(b) == {x["k"] for x in fr}, (r["file"], s)
        nnc = [nn[k] for k in ks] if nn else []
        nno = [off[k] for k in ks] if off else []
        lens = [x["n_frames"] for x in tj["runs"]] or [0]
        broke = [x for x in tj["runs"] if not x["holds"]]
        out(f" {s}: {len(fr)} checked frames, {len(tj['runs'])} runs (length median {np.median(lens):.3g}, "
              f"max {max(lens)}), runs that do not hold at scale 1: {len(broke)}; LP wall {tj['lp']['wall_s']:.0f} s, "
              f"follow wall {r['wall_s']:.0f} s; solver other status {tj['lp']['other_status']}")
        for name, v in (("unlinked LP", unl), ("LP chain", chain), ("pair", pair), ("run", run),
                        ("NNLS chain", nnc), ("NNLS unlinked", nno)):
            out(f"   {name:13s} {stats(v)}")
        out(f"   ratio LP chain / run      {stats(ratio(chain, run))}")
        out(f"   ratio run / pair          {stats(ratio(run, pair))}")
        out(f"   ratio pair / unlinked LP  {stats(ratio(pair, unl))}")
        if nnc:
            out(f"   ratio NNLS chain / run    {stats(ratio(nnc, run))}")
            out(f"   ratio LP chain / NNLS chain {stats(ratio(chain, nnc))}")
        if nno:
            out(f"   ratio unlinked LP / NNLS unlinked {stats(ratio(unl, nno))}")
        # per run of 2+ frames: the run margin is set by its weakest frame, so compare it with
        # the weakest frame's margin under each other test (a ratio below 1: harsher than the run)
        rr = {"LP chain": [], "NNLS chain": [], "unlinked LP": [], "pair": []}
        pos = {k: i for i, k in enumerate(ks)}
        for ri, x in enumerate(tj["runs"]):
            if x["n_frames"] < 2 or run_m[ri] is None or run_m[ri] <= 0:
                continue
            ii = [pos[k] for k in ks if keep[pos[k]]["run"] == ri]
            for name, v in (("LP chain", chain), ("NNLS chain", nnc), ("unlinked LP", unl), ("pair", pair)):
                w = [v[i] for i in ii if v and v[i] is not None]
                if w and len(w) == len(ii):
                    rr[name].append(min(w) / run_m[ri])
        if rr["LP chain"]:
            rows.append((LABEL.get(hand, hand), NICE[s], str(len(rr["LP chain"]))) + tuple(
                f"{np.median(rr[c]):.3g}" if rr[c] else "-" for c in ("LP chain", "NNLS chain", "pair", "unlinked LP")))
        for name, v in rr.items():
            if v:
                out(f"   per run: weakest {name:11s} / run margin  {stats(v)}")
        if rr["pair"]:
            out(f"   per run: weakest pair within 2% of the run margin in "
                f"{np.mean(np.asarray(rr['pair']) < 1.02):.2f} of runs")
        tol = 0.02  # above the bisection tolerance
        bad = sum(1 for a, b in zip(run, pair) if a is not None and b is not None and a > b + tol) + \
            sum(1 for a, b in zip(pair, unl) if a is not None and b is not None and a > b + tol)
        by_k = {x["k"]: x for x in fr}
        for x in broke:
            # the chain is one sequence: if the run fails, the chain should fail at or before the
            # break, but only up to the residual budget (the chain's box is soft, the run's link hard)
            ch = [by_k[k]["chain_lp_holds"] for k in r["frame_k"] if x["k_first"] <= k <= x["first_break_k"]]
            if all(ch):
                bad += 1
            out(f"   run {x['k_first']}-{x['k_last']} ({x['n_frames']} frames) first break at k {x['first_break_k']}")
        out(f"   order violations (run > pair, pair > unlinked, chain holds through a break): {bad}")
        pf = res.get((hand, leader, s, True))
        if pf is not None:
            pfr = pf["trajectory"]["frames"]
            assert [x["k"] for x in pfr] == [x["k"] for x in fr]
            tm = [m(x.get("trajectory_margin")) for x in pfr if x["run"] is not None and x["unlinked_margin"] is not None]
            out(f"   per-frame trajectory margin {stats(tm)}")
            out(f"   ratio per-frame / run     {stats(ratio(tm, run))}")

if TABLE:
    print("| hand | link | runs | LP chain | NNLS chain | pair | unlinked |")
    print("|---|---|---|---|---|---|---|")
    for r in rows:
        print("| " + " | ".join(r) + " |")
