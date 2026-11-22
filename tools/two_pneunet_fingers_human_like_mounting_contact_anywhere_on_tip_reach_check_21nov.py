"""Four two-pneu-net fingers mounted like the fingers of a hand, following the
contact points of one round of a recorded learned rotation, with the contact
allowed ANYWHERE ON THE FINGER'S TIP. Reach check only.
GEOMETRY ONLY, no simulation, comparative.

Read first the header of
tools/two_pneunet_fingers_human_like_mounting_reach_check_21nov.py (the
hand-like mounting, the real settled shapes, the roll rule). That check put
the CENTRE OF THE TIP FACE on each logged contact point, and two of the four
fingers lost the contact late in the stroke. A fingertip does not have to
touch with one fixed patch. Here the contact may be any part of the tip.

EVERYTHING BELOW WAS FIXED BEFORE THE FIRST RUN OF THIS TOOL:
  - The tip: every skin node in the last 1/12 of the finger's rest length
    (the 20 Nov analysis's own measure of "tip").
  - Contact error at a touching sample: the distance from the logged contact
    point to the NEAREST tip node, at the input pair used. Pass: median no
    more than 3.8 mm (the reach check's limit, unchanged).
  - Nothing may sink into the cylinder to get there. At each touching sample
    the input pair used is the one with the smallest (contact error + 10 x
    deepest cut of the skin into the logged cylinder), and the same weighted
    cut is added to what the search minimises (weight as the earlier refit).
    Pass: deepest cut of the WHOLE skin (every node) at the touching samples
    no more than SKIN_CUT_MM = 1.0 mm (ASSUMED: about one node spacing of the
    skin; a soft tip pressed on a rigid cylinder gives by about that).
  - Returns: clearance is now that of the whole finger (nearest skin node to
    the cylinder), at least 2.06 mm on every return (second rule: less the
    largest gap in the stays either side), limits unchanged. An input pair
    whose skin cuts in may not be passed through.
  - Mountings fitted: (1) contact anywhere on the tip, hand-like mounting
    with roll up to 15 degrees (the mounting chosen for the clips by the
    earlier check); (2) the same mounting limits with the contact allowed on
    ANY skin node of the finger, as a comparison. No roll ladder this time.
  - The search sees a thinned finger, to keep it affordable: K_TIP tip nodes
    (farthest-point sampling, from the node nearest the tip face's centre),
    for (2) K_REST more spread over the whole skin; the cut uses those plus
    the 32 body nodes; returns use the 32 body nodes plus the first K_OFF tip
    nodes. THE REPORTED MEASURES DO NOT: once a mounting is found, the input
    pairs are picked again and everything is measured with every tip node
    (or every skin node for (2)) as contact and every skin node for the cut
    and the returns.
  - Starts: the mountings already found by the earlier check that meet the
    limits (to within SEED_TOL_DEG), plus N_FRESH new upright ones. A
    tip-face-centre placement is a valid placement here, so the result can
    be no worse on contact error than the earlier one. Weights as the reach
    check. One cost was timed before the first run (--time-one-cost) and
    the fit as first written projected to 22 hours, so before any fit was
    run: one fresh start instead of three, and half the reach check's
    iterations (250 then 125).
  - Which goes on to the clip: (1). (2) is a comparison only and is run
    separately (--contact finger), because it takes about twice as long.
  - Also reported, not judged: how far the touching node wanders over the
    skin within one touching stretch (a viewer would read it as sliding or
    rolling on the tip).
  - If no placement passes, that is reported as "this search found none".

NOT checked: forces; speed; the palm; fingers against each other (the clip
tool reports it); whether the skin at the touching node faces the cylinder.

Each fit writes its own file when it finishes; a rerun picks up the finished
ones and never overwrites them.

Run from the repository root:
    python tools/two_pneunet_fingers_human_like_mounting_contact_anywhere_on_tip_reach_check_21nov.py \
        --wave-manip <path to Wave_manip> --gramian-map <path to gramian_map>
Writes outputs/two_pneunet_fingers_human_like_mounting_contact_anywhere_on_tip_21nov/.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import two_pneunet_fingers_follow_rl_gait_reach_check_21nov as RC                   # noqa: E402
import two_pneunet_fingers_follow_rl_gait_reach_check_bodies_clear_21nov as BC      # noqa: E402
import two_pneunet_fingers_human_like_mounting_reach_check_21nov as HM              # noqa: E402

ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "outputs", "two_pneunet_fingers_human_like_mounting_contact_anywhere_on_tip_21nov")
ROLL_LIMIT_DEG = 15.0
SKIN_CUT_MM = 1.0         # ASSUMED, see the header
K_TIP = 120               # ASSUMED
K_REST = 136              # ASSUMED
K_OFF = 24                # ASSUMED
N_FRESH = 1               # ASSUMED
ITERS = 250               # half the reach check's, see the header
SEED_TOL_DEG = 0.5        # ASSUMED
CONFIGS = [("contact anywhere on the tip", "tip"), ("contact anywhere on the finger", "skin")]
SLUG = {"tip": "contact_anywhere_on_tip", "skin": "contact_anywhere_on_finger"}


def farthest(r, pool, picked, n):
    """n more nodes from pool, each the farthest from those picked so far."""
    picked = list(picked)
    d = np.min([np.linalg.norm(r - r[k], axis=1) for k in picked], axis=0)
    ok = np.zeros(len(r), bool)
    ok[pool] = True
    for _ in range(n):
        k = int(np.argmax(np.where(ok, d, -1)))
        picked.append(k)
        d = np.minimum(d, np.linalg.norm(r - r[k], axis=1))
    return picked


def node_sets(sh):
    r = sh["skin"][0, 0].astype(float)
    tip = np.where(r[:, 0] >= r[:, 0].max() * 11 / 12)[0]
    first = int(tip[np.argmin(np.linalg.norm(r[tip] - sh["pad"][0, 0], axis=1))])
    tk = farthest(r, tip, [first], K_TIP - 1)
    sk = farthest(r, np.arange(len(r)), tk, K_REST)
    body = [int(v) for v in sh["body_nodes"]]

    def spacing(nodes):
        p = r[nodes]
        near = np.empty(len(p))                      # in blocks: the full table is 544 MB per worker
        for i in range(0, len(p), 256):
            D = np.linalg.norm(p[i:i + 256, None] - p[None], axis=-1)
            D[np.arange(len(D)), np.arange(i, i + len(D))] = 1e9
            near[i:i + 256] = D.min(1)
        return float(np.median(near) * 1e3)
    allk = np.arange(len(r))
    return {"rest": r, "tip_all": tip, "all": allk,
            "search": {"tip": (np.array(tk), np.unique(tk + body), np.unique(body + tk[:K_OFF])),
                       "skin": (np.array(sk), np.unique(sk + body), np.unique(body + tk[:K_OFF]))},
            "full": {"tip": (tip, allk, allk), "skin": (allk, allk, allk)},
            "spacing_mm": {"tip nodes the search sees": spacing(tk), "every tip node": spacing(tip),
                           "nodes the search sees over the whole finger": spacing(sk), "every skin node": spacing(allk)}}


class Anywhere(HM.Mounted):
    """The hand-like mounting's finger, touching with whichever of a set of skin nodes is nearest."""

    def __init__(self, L, idx, j, tab, R, half, roll_limit, skin, sets):
        super().__init__(L, idx, j, tab, R, half, roll_limit)
        cn, pn, on_ = sets
        self.cn = np.asarray(cn)
        self.Cn = skin[:, cn].astype(float)          # (pairs, K, 3) nodes that may touch
        self.Pn = skin[:, pn].astype(float)          # nodes kept out of the cylinder while touching
        self.On = skin[:, on_].astype(float)         # nodes kept out of the cylinder on returns
        self.C2 = (self.Cn ** 2).sum(-1)

    def local(self, x):
        Rm = RC.rodrigues(x[3:6])
        return Rm, (self.c - x[:3]) @ Rm, self.u @ Rm

    def sdf_min(self, N, c, u):
        """Smallest signed distance of nodes N (m, nodes, 3) from cylinders c, u (m, 3), in pieces."""
        out = np.zeros(len(N))
        step = max(1, int(3e6 // N.shape[1]))
        for a in range(0, len(N), step):
            s = slice(a, a + step)
            out[s] = RC.cyl_sdf(N[s], c[s][:, None, :], u[s][:, None, :], self.R, self.half).min(-1)
        return out

    def parts(self, x, mask=None):
        assert mask is None
        Rm, cf, uf = self.local(x)
        n1, n2 = self.tab["S"].shape[:2]
        on, off = np.where(self.on)[0], np.where(~self.touch)[0]
        Pf = (self.P[on] - x[:3]) @ Rm
        nq, K = self.C2.shape
        d = np.zeros((len(on), nq))
        node = np.zeros((len(on), nq), int)
        step = max(1, int(4e6 // (nq * K)))
        Cf = self.Cn.reshape(-1, 3)
        for a in range(0, len(on), step):
            p = Pf[a:a + step]
            D = ((p ** 2).sum(-1)[:, None] + self.C2.reshape(-1)[None] - 2 * p @ Cf.T).reshape(len(p), nq, K)
            node[a:a + step] = D.argmin(-1)
            d[a:a + step] = np.sqrt(np.maximum(D.min(-1), 0))
        rows = np.arange(len(on))

        def cut_of(r, q):
            return np.maximum(0.0, -self.sdf_min(self.Pn[q], cf[on][r], uf[on][r]))

        k0 = d.argmin(1)
        bound = d[rows, k0] + BC.BODY_WEIGHT * cut_of(rows, k0)
        r, q = np.where(d <= bound[:, None])
        eff = np.full(d.shape, np.inf)
        eff[r, q] = d[r, q] + BC.BODY_WEIGHT * cut_of(r, q)
        k = eff.argmin(1)
        self.cut_on = (eff[rows, k] - d[rows, k]) / BC.BODY_WEIGHT
        self.node_on = node[rows, k]
        cl = np.zeros((len(off), nq))
        step = max(1, int(3e6 // (nq * self.On.shape[1])))
        for a in range(0, len(off), step):
            s = off[a:a + step]
            cl[a:a + step] = RC.cyl_sdf(self.On[None], cf[s][:, None, None, :], uf[s][:, None, None, :], self.R, self.half).min(-1)
        cl = np.where(cl >= 0, cl, -np.inf)          # a pair whose skin cuts in may not be passed through
        return on, d[rows, k], k, off, cl.reshape(len(off), n1, n2)

    def gaps(self, x, on, arg):
        """Touching nodes in the world, the gap of the nearest contact node to the cylinder, largest gap of each stay."""
        Rm, cf, uf = self.local(x)
        tip = self.Cn[arg, self.node_on] @ Rm.T + x[:3]
        gap = self.sdf_min(self.Cn[arg], cf[on], uf[on])
        gap_at = {int(k): float(g) for k, g in zip(on, gap)}
        stay = {}
        for touching, a, b in self.spans:
            if touching:
                g = max([gap_at[k] for k in range(a, b + 1) if k in gap_at] + [0.0])
                stay[a], stay[b] = g, g
        return tip, gap, stay

    def measure(self, x, mask=None, skin=None):
        x = np.array(x, float)
        on, d, arg, off, cl = self.parts(x)
        n2 = self.tab["S"].shape[1]
        tip, gap, stay = self.gaps(x, on, arg)
        d_at = {int(k): float(v) * 1e3 for k, v in zip(on, d)}
        returns = []
        for a, b, c, v, w in self.returns(on, arg, off, cl):
            g = max(stay.get(a - 1, 0.0), stay.get(b + 1, 0.0))
            returns.append({"from_s": float(self.t[a]), "to_s": float(self.t[b]), "complete": c,
                            "path_clearance_between_end_pairs_mm": w * 1e3, "largest_gap_in_stays_either_side_mm": g * 1e3,
                            "clearance_minus_gap_mm": (w - g) * 1e3,
                            "from_logged_contact_at_lift_off_mm": d_at.get(a - 1, float("nan")),
                            "from_logged_contact_at_landing_mm": d_at.get(b + 1, float("nan"))})
        Rm = RC.rodrigues(x[3:6])
        tilt, roll = self.mounting(Rm)
        w = Rm @ HM.unit(self.tab["wag_dir"])
        nodes = self.cn[self.node_on]
        rest = self.rest[nodes]
        wander, thirds = [], [[], [], []]
        pos = {int(s): k for k, s in enumerate(on)}
        for touching, a, b in self.spans:
            ks = [pos[s] for s in range(a, b + 1) if s in pos]
            if touching and ks:
                p = rest[ks]
                wander.append(float(np.linalg.norm(p[:, None] - p[None], axis=-1).max() * 1e3))
                for i, k in enumerate(ks):
                    thirds[min(2, 3 * i // len(ks))].append(d[k] * 1e3)
        p1 = [float(self.tab["p1"][int(a) // n2]) for a in arg]
        p2 = [float(self.tab["p2"][int(a) % n2]) for a in arg]
        return {"finger": self.name, "x": [float(v) for v in x],
                "contact_error_median_mm": float(np.median(d) * 1e3), "contact_error_rms_mm": float(np.sqrt((d ** 2).mean()) * 1e3),
                "contact_error_max_mm": float(d.max() * 1e3),
                "contact_error_median_by_thirds_of_each_touching_stretch_mm": [float(np.median(v)) if v else float("nan") for v in thirds],
                "returns": returns,
                "worst_return_clearance_mm": min([r["path_clearance_between_end_pairs_mm"] for r in returns], default=float("nan")),
                "worst_clearance_minus_gap_mm": min([r["clearance_minus_gap_mm"] for r in returns], default=float("nan")),
                "gap_median_mm": float(np.median(gap) * 1e3), "gap_max_mm": float(gap.max() * 1e3), "gap_min_mm": float(gap.min() * 1e3),
                "skin_cut_while_touching_max_mm": float(self.cut_on.max() * 1e3),
                "skin_cut_while_touching_median_mm": float(np.median(self.cut_on) * 1e3),
                "touching_node_wander_within_a_stretch_max_mm": max(wander, default=float("nan")),
                "touching_node_rest_x_range_mm": [float(rest[:, 0].min() * 1e3), float(rest[:, 0].max() * 1e3)],
                "tilt_deg": float(np.degrees(tilt)), "roll_towards_the_biased_side_deg": float(np.degrees(self.sign * roll)),
                "share_of_wag_pointing_at_cylinder_on_the_stroke": self.drive_inward(Rm),
                "recorded_stroke_goes_with": "raising the first input" if w @ self.stroke > 0 else "lowering the first input",
                "base_below_cylinder_centre_mm": float((self.c[:, 2].mean() - x[2]) * 1e3),
                "input_proximal_range": [min(p1), max(p1)], "input_distal_range": [min(p2), max(p2)],
                "sample_on": [int(self.idx[s]) for s in on], "t_on": [float(v) for v in self.t[on]], "p1_on": p1, "p2_on": p2,
                "skin_node_on": [int(v) for v in nodes], "err_on_mm": [float(v * 1e3) for v in d],
                "cost_seen_by_the_search": None}


def good(m):
    return bool(RC.passes(m) and RC.passes_touching(m) and m["skin_cut_while_touching_max_mm"] <= SKIN_CUT_MM)


def line(m):
    r = "; ".join(f"{x['from_s']:.2f}-{x['to_s']:.2f} s: {x['path_clearance_between_end_pairs_mm']:.1f} mm"
                  f"{'' if x['complete'] else ' (cut by the window)'}" for x in m["returns"]) or "none in the window"
    th = m["contact_error_median_by_thirds_of_each_touching_stretch_mm"]
    return (f"  {m['finger']:6s} contact error median {m['contact_error_median_mm']:.2f} mm, rms {m['contact_error_rms_mm']:.2f}, "
            f"max {m['contact_error_max_mm']:.2f}; by thirds of each touching stretch {th[0]:.2f}, {th[1]:.2f}, {th[2]:.2f}\n"
            f"         whole-finger clearance on returns {r}; first rule {'PASS' if RC.passes(m) else 'FAIL'}, second rule "
            f"(less the gap either side, worst {m['worst_clearance_minus_gap_mm']:.1f} mm) {'PASS' if RC.passes_touching(m) else 'FAIL'}\n"
            f"         whole skin while touching: deepest cut {m['skin_cut_while_touching_max_mm']:.2f} mm (median "
            f"{m['skin_cut_while_touching_median_mm']:.2f}), limit {SKIN_CUT_MM:g}: {'PASS' if m['skin_cut_while_touching_max_mm'] <= SKIN_CUT_MM else 'FAIL'}; "
            f"gap of the nearest allowed node to the surface median {m['gap_median_mm']:+.2f} mm, range {m['gap_min_mm']:+.2f} to {m['gap_max_mm']:+.2f}\n"
            f"         touching node: wanders up to {m['touching_node_wander_within_a_stretch_max_mm']:.1f} mm over the skin within one "
            f"touching stretch; between {m['touching_node_rest_x_range_mm'][0]:.1f} and {m['touching_node_rest_x_range_mm'][1]:.1f} mm "
            f"along the finger\n"
            f"         mounting: axis {m['tilt_deg']:.1f} deg from the cylinder's axis, roll {m['roll_towards_the_biased_side_deg']:+.1f} deg "
            f"towards the biased side; {100 * m['share_of_wag_pointing_at_cylinder_on_the_stroke']:+.0f}% of the wag points at the "
            f"cylinder on the stroke; the recorded stroke goes with {m['recorded_stroke_goes_with']}; inputs first "
            f"{m['input_proximal_range'][0]:.0f} to {m['input_proximal_range'][1]:.0f}, second {m['input_distal_range'][0]:.0f} to "
            f"{m['input_distal_range'][1]:.0f}\n"
            f"         ALL RULES: {'PASS' if good(m) else 'FAIL'}")


_W = {}


def setup(wave_manip, gm, iters, out):
    W = HM.setup(wave_manip, gm)
    _W.update(W)
    _W["sets"] = node_sets(W["sh"])
    _W["skin"] = W["sh"]["skin"].reshape(-1, W["sh"]["skin"].shape[2], 3)
    _W["old"] = json.load(open(os.path.join(HM.OUT, "reach_check_results.json")))
    _W["iters"], _W["out"] = iters, out
    return _W


def make(kind, j, which):
    W = _W
    F = Anywhere(W["L"], W["idx"], j, W["tab"], W["R"], W["half"], ROLL_LIMIT_DEG, W["skin"], W["sets"][which][kind])
    F.rest = W["sets"]["rest"]
    return F


def seeds(F, j):
    out, used = [], []
    for name, ms in _W["old"]["mountings"].items():
        x = np.array(ms[j]["x"])
        tilt, roll = np.degrees(F.mounting(RC.rodrigues(x[3:6])))
        r = F.sign * roll
        if tilt <= HM.TILT_LIMIT_DEG + SEED_TOL_DEG and -SEED_TOL_DEG <= r <= ROLL_LIMIT_DEG + SEED_TOL_DEG:
            out.append(x)
            used.append(name)
    return out, used


def task(a):
    ci, j = a
    name, kind = CONFIGS[ci]
    f = os.path.join(_W["out"], f"fit_{SLUG[kind]}_{_W['L']['names'][j]}.json")
    if os.path.exists(f):
        return ci, j, json.load(open(f))
    t0 = time.time()
    F = make(kind, j, "search")
    xs, used = seeds(F, j)
    xs += F.starts(np.random.default_rng([RC.SEED, 7, ci, j]))[:N_FRESH]
    best, it = None, _W["iters"]
    for x0 in xs:
        x, v = RC.nelder_mead(F.cost, x0, np.array([.004, .004, .004, .08, .08, .08]), it)
        x, v = RC.nelder_mead(F.cost, x, np.array([.001, .001, .001, .02, .02, .02]), it // 2)
        if best is None or v < best[1]:
            best = (x, v)
    m = make(kind, j, "full").measure(best[0])
    m.update({"cost_seen_by_the_search": float(best[1]), "starts_from_the_earlier_check": used, "fresh_starts": N_FRESH,
              "iterations": [it, it // 2], "fit_minutes": (time.time() - t0) / 60, "contact": name})
    with open(f, "x", encoding="utf-8") as fh:
        json.dump(m, fh, indent=1)
    return ci, j, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", required=True)
    ap.add_argument("--gramian-map", required=True)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--contact", choices=("tip", "finger"), default="tip", help="which fit to run; each writes its own report")
    ap.add_argument("--time-one-cost", action="store_true", help="time the search's cost and stop; writes nothing")
    ap.add_argument("--trial", action="store_true", help="a few iterations for one finger into a 'trial' folder, to prove the path")
    args = ap.parse_args()
    out = os.path.join(OUT, "trial") if args.trial else OUT
    iters = 4 if args.trial else ITERS
    W = setup(args.wave_manip, args.gramian_map, iters, out)
    sp = W["sets"]["spacing_mm"]
    print("Typical spacing of neighbouring nodes, mm:", {k: round(v, 2) for k, v in sp.items()}, flush=True)
    if args.time_one_cost:
        for kind in ("tip", "skin"):
            for j in range(4):
                F = make(kind, j, "search")
                xs, used = seeds(F, j)
                a = time.time()
                v = [F.cost(x) for x in xs]
                per = (time.time() - a) / len(xs)
                n = (len(xs) + N_FRESH) * ITERS * 1.5 * 1.7
                print(kind, F.name, f"{per:.2f} s a cost; starts from the earlier check {len(xs)} {used}; their costs "
                      f"{np.round(v, 4).tolist()}; projected fit {n * per / 3600:.1f} h", flush=True)
        return
    os.makedirs(out, exist_ok=True)
    ci_run = 0 if args.contact == "tip" else 1
    jobs = [(ci, j) for ci in (ci_run,) for j in ((0,) if args.trial else range(4))]
    res = {CONFIGS[ci_run][0]: [None] * 4}
    if args.trial:
        for a in jobs:
            ci, j, m = task(a)
            print(CONFIGS[ci][0], "\n" + line(m), flush=True)
        return
    import multiprocessing as mp
    with mp.Pool(args.workers, initializer=setup, initargs=(args.wave_manip, args.gramian_map, iters, out)) as pool:
        for ci, j, m in pool.imap_unordered(task, jobs):
            res[CONFIGS[ci][0]][j] = m
            print(time.strftime("%H:%M"), CONFIGS[ci][0], "\n" + line(m), flush=True)
    L, t = W["L"], W["L"]["t"]
    rep = ["Four two-pneu-net fingers mounted like the fingers of a hand, against a recorded learned rotation: reach check with the "
           "contact allowed anywhere on the tip",
           "GEOMETRY ONLY, no simulation. Comparative, not predictive.", "",
           f"Recording: data/rl_logs/hora_seed42_30s*.npz. Cylinder radius {W['R'] * 1e3:.1f} mm, half length {W['half'] * 1e3:.1f} mm. "
           f"Window: log {t[W['s0']]:.2f} to {t[W['s1']]:.2f} s.",
           f"Finger: real settled shapes, {os.path.relpath(HM.SHAPES, ROOT)}. Tip: the {len(W['sets']['tip_all'])} skin nodes in the last "
           f"1/12 of the rest length, of {len(W['sets']['all'])}. Typical spacing of neighbouring nodes: "
           + "; ".join(f"{k} {v:.2f} mm" for k, v in sp.items()) + ".",
           f"Rules (fixed before the first run, see the header): median distance from the logged contact to the nearest allowed node "
           f"<= {RC.MAX_CONTACT_MM} mm; whole-finger clearance on every return >= {RC.MIN_CLEAR_MM} mm (second rule: less the largest "
           f"gap either side); deepest cut of the whole skin while touching <= {SKIN_CUT_MM:g} mm (ASSUMED). Mounting: axis within "
           f"{HM.TILT_LIMIT_DEG:g} deg of the cylinder's axis, roll towards the biased side up to {ROLL_LIMIT_DEG:g} deg.",
           "Measures below use every tip node (or every skin node) and the whole skin; the search saw a thinned finger."]
    score = {}
    for name, kind in CONFIGS[ci_run:ci_run + 1]:
        ms = res[name]
        rep += ["", name + ":"] + [line(m) for m in ms]
        score[name] = sum(good(m) for m in ms)
        rep.append(f"  Passing all rules: {score[name]} of 4. Contact error rule alone: "
                   f"{sum(m['contact_error_median_mm'] <= RC.MAX_CONTACT_MM for m in ms)} of 4. Mean of the median contact errors "
                   f"{np.mean([m['contact_error_median_mm'] for m in ms]):.2f} mm.")
    old = W["old"]["mountings"]["hand-like, roll up to 15 deg"]
    rep += ["", "Earlier check (centre of the tip face on the contact, roll up to 15 deg), median contact error: "
            + ", ".join(f"{m['finger']} {m['contact_error_median_mm']:.2f} mm" for m in old) + "."]
    chosen = CONFIGS[0][0]
    rep += ["", f"Goes on to the clip, by the rule fixed in the header: {chosen}. The other is a comparison only.",
            "", "Not checked here: forces; speed; the palm; fingers against each other; whether the skin at the touching node faces "
            "the cylinder."]
    with open(os.path.join(out, f"reach_check_report_{SLUG[CONFIGS[ci_run][1]]}.txt"), "x", encoding="utf-8") as fh:
        fh.write("\n".join(rep) + "\n")
    with open(os.path.join(out, f"reach_check_results_{SLUG[CONFIGS[ci_run][1]]}.json"), "x", encoding="utf-8") as fh:
        json.dump({"mountings": res, "chosen_for_the_clip": chosen, "skin_cut_limit_mm": SKIN_CUT_MM}, fh, indent=1)
    print("\n".join(rep))


if __name__ == "__main__":
    main()
