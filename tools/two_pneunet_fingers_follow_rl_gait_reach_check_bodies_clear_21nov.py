"""Refit of where the four two-pneu-net fingers stand, so that the drawn
finger bodies stay clear of the cylinder. Reach check only.
GEOMETRY ONLY, no simulation, comparative.

This is tools/two_pneunet_fingers_follow_rl_gait_reach_check_21nov.py (the
reach check; read its header first) with one thing added. That check placed
each finger from its tip alone, and the clip made from it
(tools/two_pneunet_fingers_follow_rl_gait_clip_21nov.py) showed the drawn
bodies of two fingers sitting up to 6.6 mm inside the cylinder. Here the fit
also sees the body.

The drawn body, exactly as in the clip tool's body check: straight pieces from
the clamped end to the measured junction of the two pneu-nets to the tip
centre, as wide as half the finger's depth each side, taking the points more
than one depth back from the tip (12 along the path). The junction is the
measured one of the 20 Nov knuckle analysis (read only); its height in the
finger's own frame is taken as zero (ASSUMED). Only the tip and the junction
are tabulated, so the pieces are straight where the real finger curves.

EVERYTHING BELOW WAS FIXED BEFORE THE FIRST RUN OF THIS TOOL:
  - The measures and thresholds of the reach check are unchanged: contact
    error is still the plain distance of the tip centre from its target at
    the input pair used (median no more than 3.8 mm), a return's clearance is
    still the tip's clearance (at least 2.06 mm; second rule: less the largest
    gap in the stays either side).
  - Touching samples: the input pair used is the one with the smallest
    (distance from the target + BODY_WEIGHT x depth of the body's cut into
    the cylinder at that sample). BODY_WEIGHT = 10 (ASSUMED): 1 mm of cut
    counts as 10 mm of contact error, so a cutting pair is used only when
    nothing clear is anywhere near. The same weighted cut (root mean square
    over the touching samples) is added to what the search minimises.
  - Returns: an input pair whose body cuts into the cylinder at any sample of
    the lift may not be passed through.
  - Verdict on the bodies: deepest cut of the drawn pieces into the LOGGED
    cylinder, over the whole window including the returns, no more than
    0 mm, no margin. It is measured by the clip tool's body check, not here;
    this tool prints the cut at the touching samples only.
  - The fit clears the logged cylinder only. The simulated cylinder moves
    differently (it tipped up to 52 degrees in the first clip run); cut-in
    there is reported by the clip tool, not fitted.
  - Fingers against each other: each finger is fitted on its own, as before.
    The smallest gap between different fingers' drawn pieces is reported by
    the clip tool; below 0 mm is an overlap and is reported as a failure.
    Nothing is refitted for it.
  - Search: the reach check's own (same starts, seed, iterations, weights),
    for the two-input fit and for the control.
  - Control: one input held constant, every measured level of each input
    tried, under the same body rule.
  - If no placement both passes the reach rules and keeps its body clear,
    that is reported as "this search found none".

NOT checked: tip tilt; forces; speed; the palm; the real curved shape of the
finger between the clamped end, the junction and the tip.

Run from the repository root:
    python tools/two_pneunet_fingers_follow_rl_gait_reach_check_bodies_clear_21nov.py \
        --wave-manip <path to Wave_manip> --gramian-map <path to gramian_map>
Writes outputs/two_pneunet_fingers_follow_rl_gait_reach_check_bodies_clear_21nov/.
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pneunet_arrangement_replicates_rl_stroke_20nov as A              # noqa: E402
import two_pneunet_fingers_follow_rl_gait_reach_check_21nov as RC       # noqa: E402

ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "outputs", "two_pneunet_fingers_follow_rl_gait_reach_check_bodies_clear_21nov")
BODY_WEIGHT = 10.0        # ASSUMED, see the header
BODY_POINTS = 12          # as the clip tool's body check


def junction_table(gm):
    """As in the clip tool: the junction's rest place and its measured shift with the first input."""
    d = os.path.join(gm, "experiments", "pneunet_tandem", "knuckle_analysis_20nov")
    s = json.load(open(os.path.join(d, "summary.json")))
    rows = list(csv.DictReader(open(os.path.join(d, "knuckle_levels_from_rest.csv"), newline="")))
    p1 = np.array([0.0] + [float(r["p1"]) for r in rows])
    dd = np.array([[0, 0, 0.0]] + [[float(r[f"junction_d{c}_mm"]) for c in "xyz"] for r in rows]) * 1e-3
    rest = np.array([s["knuckle_from_rest"]["pivot_x_mm"] + s["lever_split"]["along_finger_mm"],
                     s["lever_split"]["junction_centroid_rest_y_mm"], 0.0]) * 1e-3
    return {"p1": p1, "d": dd, "rest": rest}


def body_points(B3, depth):
    """As in the clip tool: points along the drawn finger more than one depth back from the tip (path length)."""
    seg = np.linalg.norm(np.diff(B3, axis=0), axis=1)
    keep = max(seg.sum() - depth, 0.0)
    s = np.linspace(0, keep, BODY_POINTS)
    cum = np.concatenate([[0], np.cumsum(seg)])
    return np.array([np.array([np.interp(v, cum, B3[:, i]) for i in range(3)]) for v in s])


def add_body(tab, gm):
    """Body points of every input pair of the tip table, in the finger's own frame (clamped end at the origin)."""
    J = junction_table(gm)
    S = tab["S"]
    n1, n2, _ = S.shape
    BP = np.zeros((n1, n2, BODY_POINTS, 3))
    for i in range(n1):
        jn = J["rest"] + np.array([np.interp(tab["p1"][i], J["p1"], J["d"][:, c]) for c in range(3)])
        for j in range(n2):
            BP[i, j] = body_points(np.array([np.zeros(3), jn, S[i, j]]), tab["depth"])
    tab["body"] = BP
    return tab


class Finger(RC.Finger):
    """The reach check's finger, with the drawn body kept clear of the cylinder. tab must hold "body"."""

    def body_clear(self, x, samples, ok):
        """Clearance of the drawn body from the cylinder (negative = cutting in), samples x allowed pairs."""
        B = self.tab["body"].reshape(-1, BODY_POINTS, 3)[ok] @ RC.rodrigues(x[3:6]).T + x[:3]
        c, u = self.c[samples][:, None, None, :], self.u[samples][:, None, None, :]
        return RC.cyl_sdf(B[None], c, u, self.R, self.half).min(-1) - 0.5 * self.tab["depth"]

    def parts(self, x, mask=None):
        W = self.world(x)
        n1, n2, _ = W.shape
        Wf = W.reshape(-1, 3)
        ok = np.ones(n1 * n2, bool) if mask is None else mask.reshape(-1)
        on = np.where(self.on)[0]
        d = np.linalg.norm(self.target[on][:, None, :] - Wf[None, ok, :], axis=-1)
        # the cut is worked out only for pairs that can win: those no further from the target than the
        # nearest pair's distance plus its weighted cut (same answer as working it out for all, and quicker)
        Bw = self.tab["body"].reshape(-1, BODY_POINTS, 3)[ok] @ RC.rodrigues(x[3:6]).T + x[:3]
        rows = np.arange(len(on))

        def cut_of(r, q):
            cu = RC.cyl_sdf(Bw[q], self.c[on][r][:, None, :], self.u[on][r][:, None, :], self.R, self.half).min(-1)
            return np.maximum(0.0, 0.5 * self.tab["depth"] - cu)

        k0 = d.argmin(1)
        bound = d[rows, k0] + BODY_WEIGHT * cut_of(rows, k0)
        r, q = np.where(d <= bound[:, None])
        eff = np.full(d.shape, np.inf)
        eff[r, q] = d[r, q] + BODY_WEIGHT * cut_of(r, q)
        cut = (eff - d) / BODY_WEIGHT
        k = eff.argmin(1)
        arg = np.where(ok)[0][k]
        self.cut_on = cut[rows, k]                  # body cut at the pair used, per touching sample
        off = np.where(~self.touch)[0]
        cl = RC.cyl_sdf(Wf[None, :, :], self.c[off][:, None, :], self.u[off][:, None, :], self.R, self.half) \
            - 0.5 * self.tab["depth"]
        cl = np.where(ok[None, :], cl, -np.inf)
        if len(off):
            bc = np.full(cl.shape, -np.inf)
            bc[:, ok] = self.body_clear(x, off, ok)
            cl = np.where(bc >= 0, cl, -np.inf)     # a pair whose body cuts in may not be passed through
        return on, d[rows, k], arg, off, cl.reshape(len(off), n1, n2)

    def cost(self, x, mask=None):
        v = super().cost(x, mask)                    # calls parts, which leaves cut_on
        return float(v + BODY_WEIGHT * np.sqrt(np.mean(np.square(self.cut_on))))

    def measure(self, x, mask=None):
        m = super().measure(x, mask)
        self.parts(x, mask)
        m["body_cut_while_touching_max_mm"] = float(self.cut_on.max() * 1e3)
        m["share_of_touching_samples_with_body_cutting_in"] = float((self.cut_on > 0).mean())
        m["returns_with_no_body_clear_route"] = int(sum(not np.isfinite(r["path_clearance_between_end_pairs_mm"])
                                                        for r in m["returns"]))
        return m


def window(L, tab, R, half):
    """The reach check's window, found the same way."""
    t = L["t"]
    touch = L["lam"] > A.TOUCH_N
    key = (touch * [1, 2, 4, 8]).sum(1)
    s0 = int(np.argmin(np.abs(t - RC.T_START)))
    runs, i = [], s0
    while i < len(t):
        s = i
        while i < len(t) and key[i] == key[s]:
            i += 1
        runs.append((s, i - 1))
        if len(runs) > 1 and key[s] == key[s0] and t[i - 1] - t[s] >= RC.MIN_STROKE_S:
            break
    s1 = first_end = runs[-1][0]

    def whole(i1):
        ok = []
        for j in range(4):
            sp = RC.Finger(L, np.arange(s0, i1 + 1, RC.STEP), j, tab, R, half).spans
            ok.append(any(not sp[k][0] and k > 0 and k + 2 < len(sp) for k in range(len(sp))))
        return all(ok)

    while not whole(s1) and t[s1] - t[first_end] < 4.0:
        s1 += 6
    return s0, s1


def line(m):
    r = "; ".join(f"{x['from_s']:.2f}-{x['to_s']:.2f} s: {x['path_clearance_between_end_pairs_mm']:.1f} mm"
                  f"{'' if x['complete'] else ' (cut by the window)'}, tip {x['tip_from_logged_contact_at_lift_off_mm']:.1f} mm from "
                  f"the logged contact at lift-off and {x['tip_from_logged_contact_at_landing_mm']:.1f} mm at landing"
                  for x in m["returns"]) or "none in the window"
    return (f"  {m['finger']:6s} contact error median {m['contact_error_median_mm']:.2f} mm, rms {m['contact_error_rms_mm']:.2f}, "
            f"max {m['contact_error_max_mm']:.2f}; return clearance {r}; inputs first {m['input_proximal_range'][0]:.0f} to "
            f"{m['input_proximal_range'][1]:.0f}, second {m['input_distal_range'][0]:.0f} to {m['input_distal_range'][1]:.0f}; "
            f"finger at {m['finger_axis_against_cylinder_axis_deg']:.0f} deg to the cylinder's axis, clamped end "
            f"{m['base_below_cylinder_centre_mm']:.0f} mm below the cylinder's centre: first rule {'PASS' if RC.passes(m) else 'FAIL'}\n"
            f"         while the log says touching: gap to the surface median {m['gap_median_mm']:+.2f} mm, range {m['gap_min_mm']:+.2f} to "
            f"{m['gap_max_mm']:+.2f} (negative = pressed in), over 1 mm in {100 * m['share_of_touching_samples_with_gap_over_1mm']:.0f}% of samples; "
            f"offset along the surface median {m['along_surface_median_mm']:.2f} mm, max {m['along_surface_max_mm']:.2f}\n"
            f"         worst return clearance less the largest gap either side {m['worst_clearance_minus_gap_mm']:.1f} mm: "
            f"second rule {'PASS' if RC.passes_touching(m) else 'FAIL'}\n"
            f"         drawn body while the log says touching: deepest cut into the cylinder {m['body_cut_while_touching_max_mm']:.2f} mm, "
            f"cutting in {100 * m['share_of_touching_samples_with_body_cutting_in']:.0f}% of samples; returns with no route that keeps "
            f"the body clear: {m['returns_with_no_body_clear_route']} of {len(m['returns'])}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", required=True)
    ap.add_argument("--gramian-map", required=True)
    ap.add_argument("--no-control", action="store_true")
    ap.add_argument("--time-one-cost", action="store_true", help="time the search's cost once per finger and stop; writes nothing")
    args = ap.parse_args()
    for sub in ("a3h_sandbox", os.path.join("experiment_log", "scripts")):
        sys.path.insert(0, os.path.join(args.wave_manip, sub))
    from w16s1_hora_scout import CYL_URDF_HALFLEN_M, CYL_URDF_RADIUS_M
    L = A.load_recording()
    R, half = CYL_URDF_RADIUS_M * L["scale"], CYL_URDF_HALFLEN_M * L["scale"]
    tab = add_body(RC.tip_table(args.gramian_map), args.gramian_map)
    t = L["t"]
    s0, s1 = window(L, tab, R, half)
    idx = np.arange(s0, s1 + 1, RC.STEP)
    fingers = [Finger(L, idx, j, tab, R, half) for j in range(4)]
    rng = np.random.default_rng(RC.SEED)
    if args.time_one_cost:
        import time
        for F in fingers:
            x0 = F.starts(np.random.default_rng(RC.SEED))[0]
            a = time.time()
            v = F.cost(x0)
            print(F.name, f"{time.time() - a:.3f} s, cost {v:.4f}, touching samples {int(F.on.sum())}")
        return
    os.makedirs(OUT, exist_ok=True)
    old = json.load(open(os.path.join(RC.OUT, "reach_check_results.json")))
    assert np.allclose(old["round"], [t[s0], t[s1]]), "the window differs from the reach check's"
    rep = ["Four two-pneu-net fingers against a recorded learned rotation: reach check refitted with the drawn finger bodies kept clear",
           "GEOMETRY ONLY, no simulation. Comparative, not predictive.", "",
           f"Recording: data/rl_logs/hora_seed42_30s*.npz. Cylinder radius {R * 1e3:.1f} mm, half length {half * 1e3:.1f} mm. "
           f"Window: log {t[s0]:.2f} to {t[s1]:.2f} s, the same as the reach check's.",
           f"Finger: {tab['file']}; length {tab['length'] * 1e3:.1f} mm, depth {tab['depth'] * 1e3:.1f} mm.",
           f"Rules (fixed before the first run, see the header): median distance from the logged contact <= {RC.MAX_CONTACT_MM} mm; "
           f"clearance on every return >= {RC.MIN_CLEAR_MM} mm (second rule: less the largest gap in the stays either side). "
           f"Added here: the drawn body (clamped end, measured junction, tip; half depth each side; more than one depth back from "
           f"the tip) is kept out of the logged cylinder: weight {BODY_WEIGHT:g} on its cut while touching (ASSUMED), and no return "
           f"may pass through an input pair whose body cuts in. The verdict on the bodies over the whole window is the clip tool's.",
           "", "Two-pneu-net finger, both inputs free:"]
    res = {"round": old["round"], "needs": old["needs"], "two_inputs": [], "one_input": [], "body_weight": BODY_WEIGHT}

    def save():
        with open(os.path.join(OUT, "reach_check_report.txt"), "w", encoding="utf-8") as fh:
            fh.write("\n".join(rep) + "\n")
        with open(os.path.join(OUT, "reach_check_results.json"), "w", encoding="utf-8") as fh:
            json.dump(res, fh, indent=1)

    def before(m, o):
        return (f"         before the refit (reach check): contact error median {o['contact_error_median_mm']:.2f} mm, worst return clearance "
                f"{o['worst_return_clearance_mm']:.1f} mm, less the gap {o['worst_clearance_minus_gap_mm']:.1f} mm; the clamped end has moved "
                f"{np.linalg.norm(np.array(m['x'][:3]) - np.array(o['x'][:3])) * 1e3:.0f} mm")

    for F, o in zip(fingers, old["two_inputs"]):
        m = F.fit(rng)
        res["two_inputs"].append(m)
        rep += [line(m), before(m, o)]
        print("\n".join(rep[-2:]), flush=True)
        save()
    rep.append(f"  Passing: first rule {sum(RC.passes(m) for m in res['two_inputs'])} of 4, "
               f"second rule {sum(RC.passes_touching(m) for m in res['two_inputs'])} of 4; body clear at every touching sample "
               f"{sum(m['body_cut_while_touching_max_mm'] <= 0 for m in res['two_inputs'])} of 4.")
    if not args.no_control:
        rep += ["", "Control, the same finger with one input held constant (best level of each input, by the fit's cost), same body rule:"]
        n1, n2 = tab["S"].shape[:2]
        for F, o in zip(fingers, old["one_input"]):
            best = None
            for which, n in (("second", n2), ("first", n1)):
                for lev in range(0, n, RC.REFINE):          # the measured levels
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
            rep += [line(m) + f"  ({m['held']})", before(m, o)]
            print("\n".join(rep[-2:]), flush=True)
            save()
        rep.append(f"  Passing: first rule {sum(RC.passes(m) for m in res['one_input'])} of 4, "
                   f"second rule {sum(RC.passes_touching(m) for m in res['one_input'])} of 4; body clear at every touching sample "
                   f"{sum(m['body_cut_while_touching_max_mm'] <= 0 for m in res['one_input'])} of 4.")
    rep += ["", "Not checked here: the bodies over the returns and against each other (the clip tool's body check, run on these "
            "placements, gives the verdict); tip tilt; forces; speed; the palm."]
    save()
    print("\n".join(rep))


if __name__ == "__main__":
    main()
