"""Can four two-pneu-net fingers follow one round of a recorded learned
rotation, including lifting off and landing again? Reach check only.
GEOMETRY ONLY, no simulation, comparative.

The recording is the learned policy on the four-finger hand turning a
cylinder (data/rl_logs/hora_seed42_30s*.npz). One round of its gait is taken:
from the start of the stroke used in
tools/pneunet_upright_arrangement_replicates_rl_stroke_21nov.py to the next
time the same three fingertips carry a stroke of that kind.

The finger is the measured 90 degree member of the two-pneu-nets-in-series
family: two 36.5 mm pneu-nets end to end, the far one rolled 90 degrees, one
input each. Its tip reach is read from the table of settled free shapes
(tip_surface_points.csv of the 20 Nov knuckle analysis, read only). No load
on the tip is modelled.

For each recorded fingertip one such finger is placed (six numbers: where its
clamped end is and how it is turned). Test, at every log sample of the round:
  - fingertip touching in the log: the finger's tip centre should sit at the
    logged contact point, stood off the surface by half the finger's depth;
  - fingertip not touching in the log: the inputs must be able to get from
    the lift-off pair to the landing pair with the tip first moving steadily
    away from the cylinder, then travelling clear of it, then moving steadily
    in to land. The clearance of a return is the smallest clearance on the
    travelling part, for the best such path.

Pass rule, FIXED BEFORE THE FIT and reused from earlier work:
  - median distance from the logged contact no more than 3.8 mm (the worst
    finger of the upright one-input re-enactment of 21 Nov);
  - clearance on every return at least 2.06 mm (the rule of the 20 Nov
    report on the two-pneu-net finger).

HISTORY OF THE MEASURES, all on 21 Nov, kept here so nothing is hidden:
  - The first run judged a return by the smallest clearance anywhere between
    the lift-off and landing input pairs. That counts the lift and the landing
    themselves as "not clear"; 1 of 4 passed. Saved report of that run:
    reach_check_report_first_clearance_measure.txt.
  - The clearance of a return was then redefined as written above (changed
    AFTER seeing that result). It is close to the largest height the tip
    reaches on the best return. The 20 Nov report tabulated the smallest,
    mean and largest clearance along one fixed return, so the two differ.
  - With that, 4 of 4 passed, but the control passed 3 of 4: the rule allows
    a 3.8 mm contact error and asks for only 2.06 mm clearance, so a tip that
    hovers passes. ADDED AFTER SEEING THAT: the gap between the tip and the
    cylinder while the log says touching, the offset along the surface, and
    a second verdict that judges each return by its clearance minus the
    largest gap in the stays either side. Both verdicts are printed.
  - Also added then: every finger's clamped end must be below the cylinder's
    bottom face (fingers on a palm under the object; two had hung from
    above), and the round is extended until every fingertip has one whole
    lift and landing.
  - Fourth and last change, also after seeing results: the search had been
    told to stop improving a return's clearance at 1.5 times the threshold
    and did not see the gap, so several returns sat at exactly that value.
    The search now aims at the quantity the second rule judges (clearance
    less the gap either side), with its target far above the threshold
    (CLEAR_AIM_MM, weighted by CLEAR_WEIGHT against the contact error; both
    ASSUMED). The window now also runs through each fingertip's whole stay
    after its landing, and the distance of the tip from the logged contact
    at lift-off and at landing is printed for every return. Report of the
    run before this change: reach_check_report_before_search_target_change.txt.

Control: the same finger with one of its two inputs held at a constant level
(every level tried, each input in turn), fitted the same way.

NOT checked here: the finger's body against the cylinder or the other
fingers (the table holds the tip only); the tilt of the tip; forces; speed.

Run from the repository root:
    python tools/two_pneunet_fingers_follow_rl_gait_reach_check_21nov.py \
        --wave-manip <path to Wave_manip> --gramian-map <path to gramian_map>
Writes outputs/two_pneunet_fingers_follow_rl_gait_reach_check_21nov/.
"""
import argparse
import csv
import heapq
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pneunet_arrangement_replicates_rl_stroke_20nov as A  # noqa: E402

ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "outputs", "two_pneunet_fingers_follow_rl_gait_reach_check_21nov")
T_START = 2.56            # s: start of the stroke used on 21 Nov (its report)
MIN_STROKE_S = 0.3        # a recurrence of the first set of fingertips counts if it lasts this long
MERGE_S = 0.05            # ASSUMED: a lift shorter than this is not counted as a lift
MAX_CONTACT_MM = 3.8      # pass rule, see the header
MIN_CLEAR_MM = 2.06       # pass rule, see the header
CLEAR_AIM_MM = 10.0       # ASSUMED: the search wants this much clearance less gap on every return
CLEAR_WEIGHT = 0.25       # ASSUMED: weight of the shortfall from that against the contact error
REFINE = 4                # tip table refined this many times between measured levels
STARTS = 12
STEP = 4                  # every fourth log sample
SEED = 0


def refine(a, axis):
    """Catmull-Rom through evenly spaced levels, REFINE points per interval."""
    a = np.moveaxis(a, axis, 0)
    p = np.concatenate([2 * a[:1] - a[1:2], a, 2 * a[-1:] - a[-2:-1]])
    out = []
    for i in range(len(a) - 1):
        p0, p1, p2, p3 = p[i], p[i + 1], p[i + 2], p[i + 3]
        for k in range(REFINE):
            u = k / REFINE
            out.append(0.5 * (2 * p1 + (p2 - p0) * u + (2 * p0 - 5 * p1 + 4 * p2 - p3) * u * u
                              + (3 * p1 - p0 - 3 * p2 + p3) * u ** 3))
    out.append(a[-1])
    return np.moveaxis(np.array(out), 0, axis)


def tip_table(gm):
    f = os.path.join(gm, "experiments", "pneunet_tandem", "knuckle_analysis_20nov", "tip_surface_points.csv")
    rows = list(csv.DictReader(open(f, newline="")))
    p1 = sorted({float(r["p1"]) for r in rows})
    p2 = sorted({float(r["p2"]) for r in rows})
    G = np.full((len(p1), len(p2), 3), np.nan)
    meas = np.zeros((len(p1), len(p2)), bool)
    for r in rows:
        i, j = p1.index(float(r["p1"])), p2.index(float(r["p2"]))
        G[i, j] = [float(r["tip_x_mm"]), float(r["tip_y_mm"]), float(r["tip_z_mm"])]
        meas[i, j] = r["kind"] == "measured"
    assert not np.isnan(G).any() and np.allclose(np.diff(p1), p1[1] - p1[0]) and np.allclose(np.diff(p2), p2[1] - p2[0])
    S = refine(refine(G, 0), 1) * 1e-3
    a1, a2 = np.linspace(p1[0], p1[-1], S.shape[0]), np.linspace(p2[0], p2[-1], S.shape[1])
    summ = json.load(open(os.path.join(os.path.dirname(f), "summary.json")))
    mp = np.array([[p1[i], p2[j]] for i in range(len(p1)) for j in range(len(p2)) if meas[i, j]])
    return {"S": S, "p1": a1, "p2": a2, "n_measured": int(meas.sum()), "n": int(meas.size), "measured_pairs": mp,
            "level_step": p1[1] - p1[0],
            "depth": summ["rest_bounding_box_mm"][1] * 1e-3, "length": summ["finger_length_mm"] * 1e-3, "file": f}


def rodrigues(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def cyl_sdf(x, c, u, R, half):
    """Distance from points x (..., 3) to a cylinder; c, u broadcast with x."""
    d = x - c
    z = (d * u).sum(-1)
    r = np.linalg.norm(d - z[..., None] * u, axis=-1)
    dr, dz = r - R, np.abs(z) - half
    return np.minimum(np.maximum(dr, dz), 0) + np.hypot(np.maximum(dr, 0), np.maximum(dz, 0))


def nelder_mead(f, x0, step, iters):
    n = len(x0)
    X = np.vstack([x0] + [x0 + step * np.eye(n)[i] for i in range(n)])
    F = np.array([f(x) for x in X])
    for _ in range(iters):
        o = np.argsort(F)
        X, F = X[o], F[o]
        c = X[:-1].mean(0)
        xr = c + (c - X[-1])
        fr = f(xr)
        if fr < F[0]:
            xe = c + 2 * (c - X[-1])
            fe = f(xe)
            X[-1], F[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < F[-2]:
            X[-1], F[-1] = xr, fr
        else:
            xc = c + 0.5 * (X[-1] - c)
            fc = f(xc)
            if fc < F[-1]:
                X[-1], F[-1] = xc, fc
            else:
                X[1:] = X[0] + 0.5 * (X[1:] - X[0])
                F[1:] = [f(x) for x in X[1:]]
    o = np.argmin(F)
    return X[o], F[o]


def widest(clear, a, b):
    """Largest smallest clearance over paths of neighbouring input pairs from a to b (or to anywhere if b is None)."""
    n1, n2 = clear.shape
    best = np.full(clear.shape, -np.inf)
    best[a] = clear[a]
    h = [(-clear[a], a)]
    while h:
        w, (i, j) = heapq.heappop(h)
        w = -w
        if w < best[i, j]:
            continue
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            k = (i + di, j + dj)
            if 0 <= k[0] < n1 and 0 <= k[1] < n2:
                v = min(w, clear[k])
                if v > best[k]:
                    best[k] = v
                    heapq.heappush(h, (-v, k))
    return float(best.max() if b is None else best[b])


def climb(clear, a):
    """Input pairs reachable from a by steps that never bring the tip nearer the cylinder."""
    n1, n2 = clear.shape
    seen = np.zeros(clear.shape, bool)
    seen[a] = True
    stack = [a]
    while stack:
        i, j = stack.pop()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            k = (i + di, j + dj)
            if 0 <= k[0] < n1 and 0 <= k[1] < n2 and not seen[k] and clear[k] >= clear[i, j]:
                seen[k] = True
                stack.append(k)
    return seen


def lift_and_land(clear, a, b):
    """Clearance of the best return from input pair a to input pair b: the tip first moves steadily away
    from the cylinder, then travels, then moves steadily in to land. The value is the smallest clearance
    on the travelling part, for the best such path. a or b None: that end is cut by the round, any pair."""
    A = climb(clear, a) if a is not None else np.isfinite(clear)
    B = climb(clear, b) if b is not None else np.isfinite(clear)
    n1, n2 = clear.shape
    best = np.where(A, clear, -np.inf)
    h = [(-best[k], k) for k in zip(*np.where(A))]
    heapq.heapify(h)
    while h:
        w, (i, j) = heapq.heappop(h)
        w = -w
        if w < best[i, j]:
            continue
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            k = (i + di, j + dj)
            if 0 <= k[0] < n1 and 0 <= k[1] < n2:
                v = min(w, clear[k])
                if v > best[k]:
                    best[k] = v
                    heapq.heappush(h, (-v, k))
    return float(best[B].max())


class Finger:
    def __init__(self, L, idx, j, tab, R, half):
        self.j, self.name = j, L["names"][j]
        self.idx = idx
        t = L["t"][idx]
        touch = L["lam"][idx, j] > A.TOUCH_N
        # a lift shorter than MERGE_S is not counted
        k = 0
        while k < len(t):
            if not touch[k]:
                s = k
                while k < len(t) and not touch[k]:
                    k += 1
                if 0 < s and k < len(t) and t[k] - t[s] < MERGE_S:
                    touch[s:k] = True
            else:
                k += 1
        self.touch, self.t = touch, t
        self.c, self.u = L["op"][idx], L["axis"][idx]
        P = L["P"][idx, j]
        n = L["nrm"][idx, j]
        n = n * np.sign((n * (P - self.c)).sum(-1, keepdims=True))
        raw = L["lam"][idx, j] > A.TOUCH_N
        self.on = touch & raw                       # samples with a logged contact point
        self.P, self.n = P, n
        self.target = P + 0.5 * tab["depth"] * n    # where the tip centre should be
        self.R, self.half, self.tab = R, half, tab
        self.base_top = float(self.c[:, 2].mean() - half)
        self.spans = []
        k = 0
        while k < len(t):
            s = k
            while k < len(t) and touch[k] == touch[s]:
                k += 1
            self.spans.append((bool(touch[s]), s, k - 1))

    def world(self, x, S=None):
        S = self.tab["S"] if S is None else S
        return S @ rodrigues(x[3:6]).T + x[:3]

    def parts(self, x, mask=None):
        """Per sample: distance of the nearest reachable tip centre from the target (touching), and the
        clearance of each input pair from the cylinder (not touching). mask limits the input pairs."""
        W = self.world(x)
        n1, n2, _ = W.shape
        Wf = W.reshape(-1, 3)
        ok = np.ones(n1 * n2, bool) if mask is None else mask.reshape(-1)
        on = np.where(self.on)[0]
        d = np.linalg.norm(self.target[on][:, None, :] - Wf[None, ok, :], axis=-1)
        arg = np.where(ok)[0][d.argmin(1)]
        off = np.where(~self.touch)[0]
        cl = cyl_sdf(Wf[None, :, :], self.c[off][:, None, :], self.u[off][:, None, :], self.R, self.half) \
            - 0.5 * self.tab["depth"]
        cl = np.where(ok[None, :], cl, -np.inf)
        return on, d.min(1), arg, off, cl.reshape(len(off), n1, n2)

    def returns(self, on, arg, off, cl):
        """For each stretch off the cylinder: (first sample, last sample, both ends inside the round,
        best clearance of any one input pair held throughout, clearance of the best return)."""
        n2 = self.tab["S"].shape[1]
        pair = {int(s): (int(a) // n2, int(a) % n2) for s, a in zip(on, arg)}
        row = {int(s): k for k, s in enumerate(off)}
        out = []
        for touching, s, e in self.spans:
            if touching:
                continue
            node = cl[[row[k] for k in range(s, e + 1)]].min(0)   # cylinder moves a little while the tip is off
            a, b = pair.get(s - 1), pair.get(e + 1)
            out.append((s, e, a is not None and b is not None, float(node.max()), lift_and_land(node, a, b)))
        return out

    def gaps(self, x, on, arg):
        """Tip centres at the touching samples, their gap to the cylinder, and the largest gap of each stay
        (keyed by the stay's first and last sample)."""
        tip = self.world(x).reshape(-1, 3)[arg]
        gap = cyl_sdf(tip, self.c[on], self.u[on], self.R, self.half) - 0.5 * self.tab["depth"]
        gap_at = {int(k): float(g) for k, g in zip(on, gap)}
        stay = {}
        for touching, a, b in self.spans:
            if touching:
                g = max([gap_at[k] for k in range(a, b + 1) if k in gap_at] + [0.0])
                stay[a], stay[b] = g, g
        return tip, gap, stay

    def cost(self, x, mask=None):
        on, d, arg, off, cl = self.parts(x, mask)
        stay = self.gaps(x, on, arg)[2]
        lack = [max(0.0, CLEAR_AIM_MM * 1e-3 - (max(r[4], -0.05) - max(stay.get(r[0] - 1, 0.0), stay.get(r[1] + 1, 0.0))))
                for r in self.returns(on, arg, off, cl)] or [0.0]
        above = max(0.0, x[2] - self.base_top)     # clamped end must be below the cylinder's bottom face
        return float(np.sqrt((d ** 2).mean()) + CLEAR_WEIGHT * np.sqrt(np.mean(np.square(lack))) + above)

    def starts(self, rng):
        on = np.where(self.on)[0]
        Pm, nm, um = self.target[on].mean(0), self.n[on].mean(0), self.u.mean(0)
        nm /= np.linalg.norm(nm)
        S = self.tab["S"]
        mid = S[S.shape[0] // 2, S.shape[1] // 2]
        out = []
        for k in range(STARTS):
            up = np.array([0.0, 0.0, 1.0])            # standing on a palm under the cylinder (world z is up)
            roll = (k % 4) * np.pi / 2 + rng.normal(0, 0.2)
            ex = up + rng.normal(0, 0.15, 3)
            ex /= np.linalg.norm(ex)
            ey = np.cross(nm, ex)
            ey /= np.linalg.norm(ey)
            ez = np.cross(ex, ey)
            Rm = np.stack([ex, ey, ez], 1) @ rodrigues(np.array([roll, 0, 0]))
            # rotation vector of Rm
            th = np.arccos(np.clip((np.trace(Rm) - 1) / 2, -1, 1))
            w = np.array([Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]])
            w = w / (2 * np.sin(th)) * th if abs(np.sin(th)) > 1e-6 else np.zeros(3)
            x0 = np.concatenate([Pm - Rm @ mid, w])
            x0[2] = min(x0[2], self.base_top - 0.002)
            out.append(x0)
        return out

    def fit(self, rng, mask=None, iters=500):
        best = None
        for x0 in self.starts(rng):
            f = lambda x: self.cost(x, mask)  # noqa: E731
            x, v = nelder_mead(f, x0, np.array([.004, .004, .004, .08, .08, .08]), iters)
            x, v = nelder_mead(f, x, np.array([.001, .001, .001, .02, .02, .02]), iters // 2)
            if best is None or v < best[1]:
                best = (x, v)
        return self.measure(best[0], mask)

    def measure(self, x, mask=None):
        on, d, arg, off, cl = self.parts(x, mask)
        n1, n2 = self.tab["S"].shape[:2]
        pair = {int(s): (int(a) // n2, int(a) % n2) for s, a in zip(on, arg)}
        tip, gap, stay = self.gaps(x, on, arg)
        e = tip - self.target[on]
        along = np.linalg.norm(e - (e * self.n[on]).sum(-1, keepdims=True) * self.n[on], axis=-1)
        d_at = {int(k): float(v) * 1e3 for k, v in zip(on, d)}
        returns = []
        for a, b, c, v, w in self.returns(on, arg, off, cl):
            g = max(stay.get(a - 1, 0.0), stay.get(b + 1, 0.0))
            returns.append({"from_s": float(self.t[a]), "to_s": float(self.t[b]), "complete": c,
                            "best_single_pair_clearance_mm": v * 1e3, "path_clearance_between_end_pairs_mm": w * 1e3,
                            "largest_gap_in_stays_either_side_mm": g * 1e3, "clearance_minus_gap_mm": (w - g) * 1e3,
                            "tip_from_logged_contact_at_lift_off_mm": d_at.get(a - 1, float("nan")),
                            "tip_from_logged_contact_at_landing_mm": d_at.get(b + 1, float("nan"))})
        k = int(np.argmax(d))
        frac = float("nan")
        for touching, a, b in self.spans:
            if touching and a <= on[k] <= b and b > a:
                frac = float((on[k] - a) / (b - a))
        p1 = [float(self.tab["p1"][pair[int(q)][0]]) for q in on]
        p2 = [float(self.tab["p2"][pair[int(q)][1]]) for q in on]
        mp = self.tab["measured_pairs"]
        near = np.abs(np.array([p1, p2]).T[:, None, :] - mp[None]).max(-1).min(1) <= 0.5 * self.tab["level_step"]
        Rm = rodrigues(x[3:6])
        return {"finger": self.name, "x": [float(v) for v in x],
                "contact_error_median_mm": float(np.median(d) * 1e3), "contact_error_rms_mm": float(np.sqrt((d ** 2).mean()) * 1e3),
                "contact_error_max_mm": float(d.max() * 1e3), "returns": returns,
                "worst_return_clearance_mm": min([r["path_clearance_between_end_pairs_mm"] for r in returns], default=float("nan")),
                "worst_clearance_minus_gap_mm": min([r["clearance_minus_gap_mm"] for r in returns], default=float("nan")),
                "has_complete_return": any(r["complete"] for r in returns),
                "gap_median_mm": float(np.median(gap) * 1e3), "gap_max_mm": float(gap.max() * 1e3), "gap_min_mm": float(gap.min() * 1e3),
                "share_of_touching_samples_with_gap_over_1mm": float((gap > 1e-3).mean()),
                "along_surface_median_mm": float(np.median(along) * 1e3), "along_surface_max_mm": float(along.max() * 1e3),
                "largest_error_at_s": float(self.t[on[k]]), "largest_error_fraction_through_stay": frac,
                "share_of_touching_samples_near_a_measured_pair": float(near.mean()),
                "base_below_cylinder_centre_mm": float((self.c[:, 2].mean() - x[2]) * 1e3),
                "gap_on_mm": [float(v * 1e3) for v in gap],
                "input_proximal_range": [min(p1), max(p1)], "input_distal_range": [min(p2), max(p2)],
                "finger_axis_against_cylinder_axis_deg": float(np.degrees(np.arccos(np.clip(Rm[:, 0] @ self.u.mean(0), -1, 1)))),
                "t_on": [float(v) for v in self.t[on]], "p1_on": p1, "p2_on": p2, "err_on_mm": [float(v * 1e3) for v in d]}


def passes(m):
    return bool(m["contact_error_median_mm"] <= MAX_CONTACT_MM
                and (not m["returns"] or m["worst_return_clearance_mm"] >= MIN_CLEAR_MM))


def passes_touching(m):
    """Added after seeing the first verdicts: a return counts by its clearance less the largest gap either side."""
    return bool(m["contact_error_median_mm"] <= MAX_CONTACT_MM
                and (not m["returns"] or m["worst_clearance_minus_gap_mm"] >= MIN_CLEAR_MM))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", required=True)
    ap.add_argument("--gramian-map", required=True)
    ap.add_argument("--no-control", action="store_true")
    args = ap.parse_args()
    for sub in ("a3h_sandbox", os.path.join("experiment_log", "scripts")):
        sys.path.insert(0, os.path.join(args.wave_manip, sub))
    from w16s1_hora_scout import CYL_URDF_HALFLEN_M, CYL_URDF_RADIUS_M
    os.makedirs(OUT, exist_ok=True)
    L = A.load_recording()
    R, half = CYL_URDF_RADIUS_M * L["scale"], CYL_URDF_HALFLEN_M * L["scale"]
    tab = tip_table(args.gramian_map)
    t, names = L["t"], L["names"]
    touch = L["lam"] > A.TOUCH_N
    key = (touch * [1, 2, 4, 8]).sum(1)
    s0 = int(np.argmin(np.abs(t - T_START)))
    runs, i = [], s0
    while i < len(t):
        s = i
        while i < len(t) and key[i] == key[s]:
            i += 1
        runs.append((s, i - 1))
        if len(runs) > 1 and key[s] == key[s0] and t[i - 1] - t[s] >= MIN_STROKE_S:
            break
    s1 = runs[-1][0]
    first_end = s1

    def whole(i1):
        ok = []
        for j in range(4):
            sp = Finger(L, np.arange(s0, i1 + 1, STEP), j, tab, R, half).spans
            ok.append(any(not sp[k][0] and k > 0 and k + 2 < len(sp) for k in range(len(sp))))   # lift, landing, whole stay after
        return all(ok)

    while not whole(s1) and t[s1] - t[first_end] < 4.0:
        s1 += 6
    idx = np.arange(s0, s1 + 1, STEP)
    lab = lambda s: "+".join(n for n, k in zip(names, touch[s]) if k) or "none"  # noqa: E731
    rep = ["Four two-pneu-net fingers against one round of a recorded learned rotation: reach check",
           "GEOMETRY ONLY, no simulation. Comparative, not predictive.", "",
           f"Recording: data/rl_logs/hora_seed42_30s*.npz. Cylinder radius {R * 1e3:.1f} mm, half length {half * 1e3:.1f} mm.",
           f"Round: log {t[s0]:.2f} to {t[s1]:.2f} s ({t[s1] - t[s0]:.2f} s), cylinder turn "
           f"{np.degrees(L['turn'][s1] - L['turn'][s0]):.1f} deg. One round of the gait ends at {t[first_end]:.2f} s, when {lab(s0)} "
           f"next touch alone for at least {MIN_STROKE_S} s; the window runs on until every fingertip has one whole lift, landing and the whole stay after it.",
           f"Cylinder axis is {np.degrees(np.arccos(abs(L['axis'][s0:s1 + 1, 2]).mean())):.1f} deg from vertical on average; world z is up.",
           "Sets of touching fingertips in the round (lifts and landings as logged):"]
    for s, e in runs[:-1]:
        rep.append(f"  {t[s]:.2f} to {t[e]:.2f} s ({t[e] - t[s]:.2f} s)  {lab(s):28s} turn {np.degrees(L['turn'][e] - L['turn'][s]):+.1f} deg")
    S = tab["S"]
    span = lambda a: float(np.linalg.norm(a[:, None] - a[None, :], axis=-1).max() * 1e3)  # noqa: E731
    reach_mm = span(S.reshape(-1, 3))
    rep += ["", f"Finger: {tab['file']}",
            f"  {tab['n_measured']} of {tab['n']} input pairs measured, the rest filled in by that analysis; refined {REFINE} times between levels here.",
            f"  Length {tab['length'] * 1e3:.1f} mm, depth {tab['depth'] * 1e3:.1f} mm. Inputs {tab['p1'][0]:g} to {tab['p1'][-1]:g} (first pneu-net) and "
            f"{tab['p2'][0]:g} to {tab['p2'][-1]:g} (second), simulation units.",
            f"  Tip centre travel: first input alone {span(S[:, 0]):.1f} mm, second alone {span(S[0, :]):.1f} mm, furthest two reachable points {reach_mm:.1f} mm apart."]
    fingers = [Finger(L, idx, j, tab, R, half) for j in range(4)]
    rep += ["", f"What each recorded fingertip does in the round (a lift shorter than {MERGE_S} s is not counted, ASSUMED):"]
    need = []
    for F in fingers:
        for touching, s, e in F.spans:
            if touching:
                on = [k for k in range(s, e + 1) if F.on[k]]
                P = F.P[on]
                rep.append(f"  {F.name:6s} touching {F.t[s]:.2f} to {F.t[e]:.2f} s ({F.t[e] - F.t[s]:.2f} s): contact moves {span(P):.1f} mm "
                           f"in the world (start to end {np.linalg.norm(P[-1] - P[0]) * 1e3:.1f} mm), cylinder turns "
                           f"{np.degrees(L['turn'][idx[e]] - L['turn'][idx[s]]):+.1f} deg")
                need.append({"finger": F.name, "touching": True, "from_s": float(F.t[s]), "to_s": float(F.t[e]), "contact_travel_mm": span(P)})
            else:
                if s > 0 and e + 1 < len(F.t):
                    hop = np.linalg.norm(F.P[e + 1] - F.P[s - 1]) * 1e3
                    gap = f"lands {hop:.1f} mm from where it lifted" + (
                        f" (further than the tip's furthest reachable points are apart, by {hop - reach_mm:.1f} mm)" if hop > reach_mm else "")
                else:
                    gap = "cut by the end of the round"
                rep.append(f"  {F.name:6s} off      {F.t[s]:.2f} to {F.t[e]:.2f} s ({F.t[e] - F.t[s]:.2f} s): {gap}")
                need.append({"finger": F.name, "touching": False, "from_s": float(F.t[s]), "to_s": float(F.t[e])})
    rep += ["", f"First rule: median distance from the logged contact <= {MAX_CONTACT_MM} mm; clearance on every return >= "
            f"{MIN_CLEAR_MM} mm. The thresholds were fixed before the fit; the way a return's clearance is measured was changed after "
            f"the first run (see the header of the script).",
            f"Second rule, ADDED AFTER the control passed the first: the same, with each return's clearance reduced by the largest gap "
            f"between tip and cylinder in the stays either side.",
            "Every clamped end is kept below the cylinder's bottom face (ASSUMED: a palm under the object).",
            "The tip centre is asked to sit half the finger's depth off the logged contact point along the surface normal (ASSUMED; tip tilt ignored).",
            "", "Two-pneu-net finger, both inputs free:"]
    rng = np.random.default_rng(SEED)
    res = {"round": [float(t[s0]), float(t[s1])], "needs": need, "two_inputs": [], "one_input": []}

    def line(m):
        r = "; ".join(f"{x['from_s']:.2f}-{x['to_s']:.2f} s: {x['path_clearance_between_end_pairs_mm']:.1f} mm"
                      f"{'' if x['complete'] else ' (cut by the round)'}, tip {x['tip_from_logged_contact_at_lift_off_mm']:.1f} mm from "
                      f"the logged contact at lift-off and {x['tip_from_logged_contact_at_landing_mm']:.1f} mm at landing"
                      for x in m["returns"]) or "none in the round"
        return (f"  {m['finger']:6s} contact error median {m['contact_error_median_mm']:.2f} mm, rms {m['contact_error_rms_mm']:.2f}, "
                f"max {m['contact_error_max_mm']:.2f}; return clearance {r}; inputs first {m['input_proximal_range'][0]:.0f} to "
                f"{m['input_proximal_range'][1]:.0f}, second {m['input_distal_range'][0]:.0f} to {m['input_distal_range'][1]:.0f}; "
                f"finger at {m['finger_axis_against_cylinder_axis_deg']:.0f} deg to the cylinder's axis, clamped end "
                f"{m['base_below_cylinder_centre_mm']:.0f} mm below the cylinder's centre: first rule {'PASS' if passes(m) else 'FAIL'}\n"
                f"         while the log says touching: gap to the surface median {m['gap_median_mm']:+.2f} mm, range {m['gap_min_mm']:+.2f} to "
                f"{m['gap_max_mm']:+.2f} (negative = pressed in), over 1 mm in {100 * m['share_of_touching_samples_with_gap_over_1mm']:.0f}% of samples; "
                f"offset along the surface median {m['along_surface_median_mm']:.2f} mm, max {m['along_surface_max_mm']:.2f}; largest error at "
                f"{m['largest_error_at_s']:.2f} s, {100 * m['largest_error_fraction_through_stay']:.0f}% of the way through its stay; "
                f"{100 * m['share_of_touching_samples_near_a_measured_pair']:.0f}% of touching samples within half a level of a measured input pair\n"
                f"         worst return clearance less the largest gap either side {m['worst_clearance_minus_gap_mm']:.1f} mm: "
                f"second rule {'PASS' if passes_touching(m) else 'FAIL'}"
                + ("" if passes_touching(m) or m["contact_error_median_mm"] > MAX_CONTACT_MM else
                   f" (short of {MIN_CLEAR_MM} mm by {MIN_CLEAR_MM - m['worst_clearance_minus_gap_mm']:.1f} mm)"))

    def save():
        with open(os.path.join(OUT, "reach_check_report.txt"), "w", encoding="utf-8") as fh:
            fh.write("\n".join(rep) + "\n")
        with open(os.path.join(OUT, "reach_check_results.json"), "w", encoding="utf-8") as fh:
            json.dump(res, fh, indent=1)

    for F in fingers:
        m = F.fit(rng)
        res["two_inputs"].append(m)
        rep.append(line(m))
        print(rep[-1], flush=True)
        save()
    rep.append(f"  Passing: first rule {sum(passes(m) for m in res['two_inputs'])} of 4, "
               f"second rule {sum(passes_touching(m) for m in res['two_inputs'])} of 4.")
    if not args.no_control:
        rep += ["", "Control, the same finger with one input held constant (best level of each input, by the fit's cost):"]
        n1, n2 = S.shape[:2]
        for F in fingers:
            best = None
            for which, n in (("second", n2), ("first", n1)):
                for lev in range(0, n, REFINE):          # the measured levels
                    mask = np.zeros((n1, n2), bool)
                    if which == "second":
                        mask[:, lev] = True
                    else:
                        mask[lev, :] = True
                    m = F.fit(rng, mask, iters=250)
                    m["held"] = f"{which} input held at {(tab['p2'] if which == 'second' else tab['p1'])[lev]:g}"
                    c = F.cost(np.array(m["x"]), mask)
                    if best is None or c < best[0]:
                        best = (c, m)
            m = best[1]
            res["one_input"].append(m)
            rep.append(line(m) + f"  ({m['held']})")
            print(rep[-1], flush=True)
            save()
        rep.append(f"  Passing: first rule {sum(passes(m) for m in res['one_input'])} of 4, "
                   f"second rule {sum(passes_touching(m) for m in res['one_input'])} of 4.")
    rep += ["", "Not checked: finger bodies against the cylinder and each other (fingers lying close alongside the cylinder make "
            "this the likeliest thing to fail next); tip tilt; forces; speed. The return paths' input pairs are not checked against "
            "which table points were measured."]
    save()
    print("\n".join(rep))


if __name__ == "__main__":
    main()
