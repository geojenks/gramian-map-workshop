"""Four two-pneu-net fingers mounted like the fingers of a hand, following the
contact points of one round of a recorded learned rotation. Reach check only.
GEOMETRY ONLY, no simulation, comparative.

Read first the headers of
tools/two_pneunet_fingers_follow_rl_gait_reach_check_21nov.py (the reach
check: recording, window, measures, pass rules) and
tools/two_pneunet_fingers_follow_rl_gait_reach_check_bodies_clear_21nov.py
(the refit that keeps the body out of the cylinder). This tool reuses both
and changes three things.

1. THE FINGER IS ITS REAL SETTLED SHAPE. The earlier checks knew only the tip
   centre and stood it half a finger depth off the surface, whichever way the
   finger was turned. Here the shapes come from
   tools/two_pneunet_finger_shapes_export_21nov.py (the 20 Nov shape model on
   the same 25 x 25 input pairs): the centre of the tip face (the patch that
   faces the way the far pneu-net curls) is what should sit ON the logged
   contact point, with no stand-off, and the body is 32 nodes spread over the
   real skin, with no thickness added.

2. THE MOUNTING IS LIKE A HAND'S. Each finger stands on a palm under the
   cylinder (clamped end below the cylinder's bottom face, as before), and:
     - its long axis is within TILT_LIMIT_DEG of the cylinder's mean axis,
       pointing up;
     - the near pneu-net's input wags the finger sideways (abduction) and the
       far one's curls the tip towards the cylinder: the tip face's rest
       normal, seen along the finger's axis, points at the cylinder's axis
       to within the roll below.

3. A SLIGHT ROLL ABOUT THE FINGER'S OWN AXIS, to bias it towards turning one
   way. With a roll, the sideways wag also moves the tip towards or away from
   the cylinder. The roll's sign is fixed by this rule: the wag that goes
   with the recorded stroke (the mean travel of that fingertip's logged
   contact point while touching) moves the tip towards the cylinder, so the
   wag back moves it away. The fit may use any roll of that sign up to a
   limit.

EVERYTHING BELOW WAS FIXED BEFORE THE FIRST RUN OF THIS TOOL:
  - TILT_LIMIT_DEG = 25 (ASSUMED).
  - Roll limits tried: 0, 15 and 30 degrees (ASSUMED ladder), all reported.
    Beside them, the same fit with no limit on tilt or roll ("any mounting
    on the palm"), so the cost of the hand-like mounting is seen.
  - Limits are held by a penalty of MOUNT_WEIGHT metres per radian of excess
    (ASSUMED); the tilt and roll actually reached are printed.
  - Measures and pass rules are the reach check's, unchanged: median distance
    of the tip face's centre from the logged contact no more than 3.8 mm;
    clearance on every return at least 2.06 mm (second rule: less the largest
    gap in the stays either side). The clearance is now that of the tip
    face's centre from the cylinder.
  - Body rule as in the refit: BODY_WEIGHT 10 on the body nodes' cut into
    the logged cylinder while touching; no return may pass through an input
    pair whose body nodes cut in. Because the tip face itself is part of the
    skin, a tip face pressed into the logged cylinder counts as cut, so the
    fit prefers a small gap to pressing in.
  - Search as the reach check: 12 starts, the same iterations and weights.
    Starts for the hand-like fits: upright with noise, roll at none, half
    and all of the limit.
  - Which mounting goes on to the clips: of the three hand-like ones, the one
    where most fingers pass both reach rules with body nodes clear at every
    touching sample; ties go to the smaller mean over the fingers of the
    median contact error.
  - The one-input control is not repeated here (it failed in both earlier
    checks; see their reports).
  - If the hand-like mounting follows the contacts worse than the free one,
    that is reported as it is.

Also reported, not judged: the deepest cut of the whole skin (every node)
into the logged cylinder at the touching samples; the angle between the tip
face's normal and the cylinder's inward normal at the contact; whether the
recorded stroke goes with raising or lowering the first input.

NOT checked: forces; speed; the palm; fingers against each other (the clip
tool reports it).

Run from the repository root (after the shape export):
    python tools/two_pneunet_fingers_human_like_mounting_reach_check_21nov.py \
        --wave-manip <path to Wave_manip> --gramian-map <path to gramian_map>
Writes outputs/two_pneunet_fingers_human_like_mounting_21nov/reach_check_report.txt
and reach_check_results.json.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pneunet_arrangement_replicates_rl_stroke_20nov as A                          # noqa: E402
import two_pneunet_fingers_follow_rl_gait_reach_check_21nov as RC                   # noqa: E402
import two_pneunet_fingers_follow_rl_gait_reach_check_bodies_clear_21nov as BC      # noqa: E402

ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "outputs", "two_pneunet_fingers_human_like_mounting_21nov")
SHAPES = os.path.join(OUT, "finger_shapes.npz")
TILT_LIMIT_DEG = 25.0             # ASSUMED
ROLL_LIMITS_DEG = (0.0, 15.0, 30.0)   # ASSUMED ladder
MOUNT_WEIGHT = 0.5                # ASSUMED: metres of cost per radian beyond a limit
CONFIGS = [("hand-like, no roll", 0.0), ("hand-like, roll up to 15 deg", 15.0), ("hand-like, roll up to 30 deg", 30.0),
           ("any mounting on the palm", None)]


def rotvec(Rm):
    th = np.arccos(np.clip((np.trace(Rm) - 1) / 2, -1, 1))
    w = np.array([Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]])
    return w / (2 * np.sin(th)) * th if abs(np.sin(th)) > 1e-6 else np.zeros(3)


def unit(v):
    return v / np.linalg.norm(v)


def load_shapes(gm):
    sh = np.load(SHAPES)
    t0 = RC.tip_table(gm)
    assert np.allclose(sh["levels"], t0["p1"]) and np.allclose(sh["levels"], t0["p2"]), "input grids differ"
    BC.BODY_POINTS = int(len(sh["body_nodes"]))        # the refit's code reshapes by this module value
    tab = {"S": sh["pad"].astype(float), "p1": sh["levels"].astype(float), "p2": sh["levels"].astype(float), "depth": 0.0,
           "length": t0["length"], "measured_pairs": t0["measured_pairs"], "level_step": t0["level_step"],
           "n_measured": t0["n_measured"], "n": t0["n"], "file": SHAPES,
           "body": sh["skin"][:, :, sh["body_nodes"]].astype(float), "wag_dir": sh["wag_dir"].astype(float),
           "pad_normal": sh["pad_normal"].astype(float)}
    return tab, t0, sh


class Mounted(BC.Finger):
    """The refit's finger on the real shape. roll_limit None: no limit on tilt or roll."""

    def __init__(self, L, idx, j, tab, R, half, roll_limit):
        super().__init__(L, idx, j, tab, R, half)
        self.roll_limit = roll_limit
        on = np.where(self.on)[0]
        up = unit(self.u.mean(0))
        self.up = up if up[2] > 0 else -up
        nm = self.n[on].mean(0)
        self.r_out = unit(nm - (nm @ self.up) * self.up)
        step = np.array([self.P[b] - self.P[a] for a, b in zip(on[:-1], on[1:]) if b == a + 1]).sum(0)
        step -= (step @ self.r_out) * self.r_out
        self.stroke = unit(step)                   # mean travel of the logged contact point while touching
        self.Pm = self.P[on].mean(0)
        v = [self.drive_inward(self.frame(self.up, r)) for r in (0.2, -0.2)]
        assert v[0] * v[1] < 0, "the roll's sign rule is not decisive"
        self.sign = 1.0 if v[0] > 0 else -1.0

    def frame(self, a, roll):
        """Finger frame with long axis a and the tip face's rest normal rolled by roll from the inward radial."""
        a = unit(a)
        r_in = unit(-self.r_out + (self.r_out @ a) * a)
        c = np.cos(roll) * r_in + np.sin(roll) * np.cross(a, r_in)
        ez = -c
        return np.stack([a, np.cross(ez, a), ez], 1)

    def mounting(self, Rm):
        a = Rm[:, 0]
        r_in = unit(-self.r_out + (self.r_out @ a) * a)
        c = Rm @ np.array([0.0, 0.0, -1.0])
        tilt = np.arccos(np.clip(a @ self.up, -1, 1))
        roll = np.arctan2(np.cross(r_in, c) @ a, r_in @ c)
        return tilt, roll

    def drive_inward(self, Rm):
        """Share of the wag that points at the cylinder, for the wag sense that goes with the recorded stroke."""
        w = Rm @ unit(self.tab["wag_dir"])
        return float(np.sign(w @ self.stroke) * (w @ -self.r_out))

    def cost(self, x, mask=None):
        v = super().cost(x, mask)
        if self.roll_limit is None:
            return v
        tilt, roll = self.mounting(RC.rodrigues(x[3:6]))
        r = self.sign * roll
        over = max(0.0, tilt - np.radians(TILT_LIMIT_DEG)) + max(0.0, -r) + max(0.0, r - np.radians(self.roll_limit))
        return float(v + MOUNT_WEIGHT * over)

    def starts(self, rng):
        if self.roll_limit is None:
            return super().starts(rng)
        S = self.tab["S"]
        mid = S[S.shape[0] // 2, S.shape[1] // 2]
        out = []
        for k in range(RC.STARTS):
            roll = self.sign * np.radians(self.roll_limit) * (k % 3) / 2
            Rm = self.frame(self.up + rng.normal(0, 0.1, 3), roll)
            x0 = np.concatenate([self.Pm - Rm @ mid, rotvec(Rm)])
            x0[2] = min(x0[2], self.base_top - 0.002)
            out.append(x0)
        return out

    def measure(self, x, mask=None, skin=None):
        m = super().measure(x, mask)
        Rm = RC.rodrigues(np.array(x[3:6]))
        tilt, roll = self.mounting(Rm)
        on, d, arg, off, cl = self.parts(np.array(x), mask)
        n2 = self.tab["S"].shape[1]
        N = self.tab["pad_normal"].reshape(-1, 3)[arg] @ Rm.T
        ang = np.degrees(np.arccos(np.clip((N * -self.n[on]).sum(-1), -1, 1)))
        w = Rm @ unit(self.tab["wag_dir"])
        m.update({"tilt_deg": float(np.degrees(tilt)), "roll_towards_the_biased_side_deg": float(np.degrees(self.sign * roll)),
                  "share_of_wag_pointing_at_cylinder_on_the_stroke": self.drive_inward(Rm),
                  "recorded_stroke_goes_with": "raising the first input" if w @ self.stroke > 0 else "lowering the first input",
                  "tip_face_normal_against_inward_normal_median_deg": float(np.median(ang)),
                  "tip_face_normal_against_inward_normal_max_deg": float(ang.max())})
        if skin is not None:
            cut = []
            for s, a in zip(on, arg):
                W = skin[a // n2, a % n2].astype(float) @ Rm.T + np.array(x[:3])
                cut.append(max(0.0, -float(RC.cyl_sdf(W, self.c[s], self.u[s], self.R, self.half).min())))
            m["whole_skin_cut_while_touching_max_mm"] = float(max(cut) * 1e3)
            m["whole_skin_cut_while_touching_median_mm"] = float(np.median(cut) * 1e3)
        return m


def line(m):
    r = "; ".join(f"{x['from_s']:.2f}-{x['to_s']:.2f} s: {x['path_clearance_between_end_pairs_mm']:.1f} mm"
                  f"{'' if x['complete'] else ' (cut by the window)'}" for x in m["returns"]) or "none in the window"
    return (f"  {m['finger']:6s} contact error median {m['contact_error_median_mm']:.2f} mm, rms {m['contact_error_rms_mm']:.2f}, "
            f"max {m['contact_error_max_mm']:.2f}; return clearance {r}; first rule {'PASS' if RC.passes(m) else 'FAIL'}, "
            f"second rule (less the gap either side, worst {m['worst_clearance_minus_gap_mm']:.1f} mm) "
            f"{'PASS' if RC.passes_touching(m) else 'FAIL'}\n"
            f"         mounting: axis {m['tilt_deg']:.1f} deg from the cylinder's axis, roll {m['roll_towards_the_biased_side_deg']:+.1f} deg "
            f"towards the biased side; {100 * m['share_of_wag_pointing_at_cylinder_on_the_stroke']:+.0f}% of the wag points at the "
            f"cylinder on the stroke; the recorded stroke goes with {m['recorded_stroke_goes_with']}; clamped end "
            f"{m['base_below_cylinder_centre_mm']:.0f} mm below the cylinder's centre\n"
            f"         inputs first {m['input_proximal_range'][0]:.0f} to {m['input_proximal_range'][1]:.0f}, second "
            f"{m['input_distal_range'][0]:.0f} to {m['input_distal_range'][1]:.0f}; gap to the surface while touching median "
            f"{m['gap_median_mm']:+.2f} mm, range {m['gap_min_mm']:+.2f} to {m['gap_max_mm']:+.2f}; tip face normal "
            f"{m['tip_face_normal_against_inward_normal_median_deg']:.0f} deg from the inward normal (median, max "
            f"{m['tip_face_normal_against_inward_normal_max_deg']:.0f})\n"
            f"         body nodes while touching: deepest cut {m['body_cut_while_touching_max_mm']:.2f} mm, cutting in "
            f"{100 * m['share_of_touching_samples_with_body_cutting_in']:.0f}% of samples; whole skin deepest cut "
            f"{m['whole_skin_cut_while_touching_max_mm']:.2f} mm (median {m['whole_skin_cut_while_touching_median_mm']:.2f}); "
            f"returns with no body-clear route {m['returns_with_no_body_clear_route']} of {len(m['returns'])}")


def good(m):
    return bool(RC.passes(m) and RC.passes_touching(m) and m["body_cut_while_touching_max_mm"] <= 0)


_W = {}


def setup(wave_manip, gm):
    for sub in ("a3h_sandbox", os.path.join("experiment_log", "scripts")):
        sys.path.insert(0, os.path.join(wave_manip, sub))
    from w16s1_hora_scout import CYL_URDF_HALFLEN_M, CYL_URDF_RADIUS_M
    L = A.load_recording()
    R, half = CYL_URDF_RADIUS_M * L["scale"], CYL_URDF_HALFLEN_M * L["scale"]
    tab, t0, sh = load_shapes(gm)
    s0, s1 = BC.window(L, t0, R, half)
    _W.update(L=L, R=R, half=half, tab=tab, sh=sh, s0=s0, s1=s1, idx=np.arange(s0, s1 + 1, RC.STEP))
    return _W


def task(a):
    ci, j = a
    W = _W
    F = Mounted(W["L"], W["idx"], j, W["tab"], W["R"], W["half"], CONFIGS[ci][1])
    m = F.fit(np.random.default_rng([RC.SEED, ci, j]))
    m = F.measure(np.array(m["x"]), skin=W["sh"]["skin"])
    m["roll_sign_rule"] = F.sign
    return ci, j, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", required=True)
    ap.add_argument("--gramian-map", required=True)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--time-one-cost", action="store_true", help="time the search's cost once per finger and stop; writes nothing")
    args = ap.parse_args()
    W = setup(args.wave_manip, args.gramian_map)
    L, R, half, tab = W["L"], W["R"], W["half"], W["tab"]
    t = L["t"]
    if args.time_one_cost:
        for j in range(4):
            F = Mounted(L, W["idx"], j, tab, R, half, 15.0)
            x0 = F.starts(np.random.default_rng(RC.SEED))[1]
            a = time.time()
            v = F.cost(x0)
            print(F.name, f"{time.time() - a:.3f} s, cost {v:.4f}, touching samples {int(F.on.sum())}, samples {len(F.t)}, "
                  f"roll sign {F.sign:+.0f}, stroke {np.round(F.stroke, 2)}, mounting at start "
                  f"{np.round(np.degrees(F.mounting(RC.rodrigues(x0[3:6]))), 1)}")
        return
    old = json.load(open(os.path.join(RC.OUT, "reach_check_results.json")))
    assert np.allclose(old["round"], [t[W["s0"]], t[W["s1"]]]), "the window differs from the reach check's"
    f_rep, f_res = os.path.join(OUT, "reach_check_report.txt"), os.path.join(OUT, "reach_check_results.json")
    assert not os.path.exists(f_res), "results exist already; this tool does not overwrite them"
    import multiprocessing as mp
    jobs = [(ci, j) for ci in range(len(CONFIGS)) for j in range(4)]
    res = {"round": old["round"], "tilt_limit_deg": TILT_LIMIT_DEG, "mount_weight": MOUNT_WEIGHT, "body_weight": BC.BODY_WEIGHT,
           "mountings": {c[0]: [None] * 4 for c in CONFIGS}}
    with mp.Pool(args.workers, initializer=setup, initargs=(args.wave_manip, args.gramian_map)) as pool:
        for ci, j, m in pool.imap_unordered(task, jobs):
            res["mountings"][CONFIGS[ci][0]][j] = m
            print(CONFIGS[ci][0], "\n" + line(m), flush=True)
    rep = ["Four two-pneu-net fingers mounted like the fingers of a hand, against a recorded learned rotation: reach check",
           "GEOMETRY ONLY, no simulation. Comparative, not predictive.", "",
           f"Recording: data/rl_logs/hora_seed42_30s*.npz. Cylinder radius {R * 1e3:.1f} mm, half length {half * 1e3:.1f} mm. "
           f"Window: log {t[W['s0']]:.2f} to {t[W['s1']]:.2f} s, the same as the reach check's.",
           f"Finger: real settled shapes, {os.path.relpath(SHAPES, ROOT)} (see finger_shapes_report.txt). The tip face's centre is "
           f"put on the logged contact point; the body is {BC.BODY_POINTS} skin nodes.",
           f"Rules (fixed before the first run, see the header): median distance from the logged contact <= {RC.MAX_CONTACT_MM} mm; "
           f"clearance on every return >= {RC.MIN_CLEAR_MM} mm (second rule: less the largest gap either side); body weight "
           f"{BC.BODY_WEIGHT:g}. Hand-like mounting: axis within {TILT_LIMIT_DEG:g} deg of the cylinder's axis, tip face towards "
           f"the cylinder's axis, roll only towards the biased side and no more than the limit."]
    score = {}
    for name, lim in CONFIGS:
        ms = res["mountings"][name]
        rep += ["", name + ":"] + [line(m) for m in ms]
        ok = sum(good(m) for m in ms)
        med = float(np.mean([m["contact_error_median_mm"] for m in ms]))
        rep.append(f"  Passing: first rule {sum(RC.passes(m) for m in ms)} of 4, second rule {sum(RC.passes_touching(m) for m in ms)} of 4, "
                   f"both rules with body nodes clear at every touching sample {ok} of 4. Mean of the median contact errors {med:.2f} mm.")
        if lim is not None:
            score[name] = (-ok, med)
    chosen = min(score, key=score.get)
    res["chosen_for_the_clips"] = chosen
    rep += ["", f"Mounting that goes on to the clips, by the rule fixed in the header: {chosen}.",
            "", "Not checked here: forces; speed; the palm; fingers against each other; the bodies over the returns with every skin "
            "node (the clip tool reports these)."]
    with open(f_rep, "w", encoding="utf-8") as fh:
        fh.write("\n".join(rep) + "\n")
    with open(f_res, "w", encoding="utf-8") as fh:
        json.dump(res, fh, indent=1)
    print("\n".join(rep))


if __name__ == "__main__":
    main()
