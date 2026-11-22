"""Four two-pneu-net fingers turn a cylinder carried on an axle, with the tips
aimed a fixed depth inside the cylinder's surface while the recording has
them touching. SIMULATION ONLY (MuJoCo). Comparative, not predictive. Runs
no SOFA. Numbers only; the clip is drawn by
two_pneunet_fingers_turn_cylinder_on_axle_clip_22nov.py from the frames
saved here.

Why: in the 22 Nov axle run (two_pneunet_fingers_turn_cylinder_on_axle_21nov.py,
folder ..._constraint_carry) the tips were aimed exactly at the surface, so
millimetre fit errors decided whether they touched. They made and broke
contact many times in each stretch where the recording has one contact, and
pressed with about a tenth of the logged force. Here the tips are aimed a
fixed depth inside the surface instead.

Everything is as that axle run (same model, axle, carry, stand-in ball,
stiffnesses, timeline, friction; its build() and run() are used unchanged)
except where the spring's rest position goes:

  PRESS-IN DEPTH (ASSUMED, fixed before any run, not tuned afterwards):
  RC.MAX_CONTACT_MM, the contact limit already in use in the reach checks.
  One value for every finger, stretch and stiffness.

  Reading shown on the clip, "fixed depth": while the recording has the
  finger touching, the ball's rest position keeps the free finger's own
  position along and around the cylinder's axis, but its distance from the
  axis is set so that the ball's surface is the press-in depth inside the
  cylinder's side. So in the radial direction the finger's own reach is
  REPLACED by an assumption. How far that moves the rest position from the
  finger's own is reported per finger.

  Comparison reading, "own reach pushed in": the finger's own rest position
  (as in the earlier axle run) moved the press-in depth towards the axis
  while the recording has the finger touching. Fit errors stay in.

  In both, "the recording has the finger touching" is the reach check's own
  flag (plans[k].F.touch on F.t), interpolated linearly between log
  samples, so the press-in comes on and goes off over one log sample. On
  returns the rest position is the finger's own, unchanged.

Measures, FIXED BEFORE THE FIRST RUN (beside all the earlier axle run's):
  1. For each stretch where the recording has a finger touching: how many
     separate touching runs the simulated tip has in it (normal force above
     the touch threshold, on the 0.01 s log), and the share of the stretch
     it touches. A stretch with exactly one touching run has one "contact
     on" and one "contact off".
  2. Share of the logged return time in which the simulated tip touches.
  3. How far the rest position was moved from the finger's own, in the
     touching stretches (median, largest), and the share of touching time
     in which the finger's own position is beyond the cylinder's flat ends
     (there the fixed depth does not mean the ball is on the side).
  RULE for the clip: the "fixed depth" run at the main stiffness goes on
  the clip, whatever it shows. The earlier axle clip is kept. "One contact
  on, one contact off" is said only for fingers where every logged
  touching stretch has exactly one touching run.

Not modelled, as before: the finger's body and its bending under load, the
palm, anything holding the cylinder other than the axle, pressure dynamics.
The drawn skin is the free finger at the commanded inputs; it is not
pressed in, and the ball that is simulated is not the drawn skin.
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import pneunet_arrangement_replicates_rl_stroke_20nov as A                     # noqa: E402
import two_pneunet_fingers_follow_rl_gait_reach_check_21nov as RC              # noqa: E402
import two_pneunet_fingers_follow_rl_gait_clip_21nov as C                      # noqa: E402
import two_pneunet_fingers_human_like_mounting_reach_check_21nov as HM         # noqa: E402
import two_pneunet_fingers_turn_cylinder_on_axle_21nov as X                    # noqa: E402

OUT = os.path.join(ROOT, "outputs", "two_pneunet_fingers_turn_cylinder_on_axle_fixed_press_in_22nov")
EARLIER = os.path.join(ROOT, "outputs", "two_pneunet_fingers_turn_cylinder_on_axle_21nov_constraint_carry", "axle_results.json")
PRESS = RC.MAX_CONTACT_MM * 1e-3      # m, ASSUMED, see the header


def setup(args):
    """Everything the earlier axle run sets up, the same way."""
    W = HM.setup(args.wave_manip, args.gramian_map)
    L, R, half, tab, sh, idx, s0 = W["L"], W["R"], W["half"], W["tab"], W["sh"], W["idx"], W["s0"]
    t = L["t"]
    S = {"L": L, "R": R, "half": half, "idx": idx, "s0": s0, "t0": float(t[idx[0]]), "t1": float(t[idx[-1]])}
    S["radius"] = 0.5 * RC.tip_table(args.gramian_map)["depth"]
    C.FINGER = lambda L_, idx_, j_, tab_, R_, half_: HM.Mounted(L_, idx_, j_, tab_, R_, half_, None)   # noqa: E731
    fits = json.load(open(os.path.join(HM.OUT, "reach_check_results.json")))["mountings"][X.MOUNTING]
    S["plans"] = [C.Plan(L, idx, j, tab, R, half, fits[j], None) for j in range(4)]
    S["grids"] = [X.Grid(sh, fits[j]["x"]) for j in range(4)]
    S["carry"] = X.Carry(L, s0, int(idx[-1]))
    up = json.load(open(C.UPRIGHT))
    ks = (float(np.mean([f["k_along_surface_N_per_m"] for f in up["fits"]])),
          float(np.mean([f["k_out_of_plane_N_per_m"] for f in up["fits"]])))
    S["stiff"] = {"main": float(np.sqrt(ks[0] * ks[1])), "soft": ks[0], "stiff": ks[1]}
    S["rec_turn"] = float(np.degrees(L["turn"][idx[-1]] - L["turn"][idx[0]]))
    S["names"] = [L["names"][j] for j in range(4)]
    return S


def centres(S, pose=None):
    """The ball's rest position: the finger's own, and the two press-in readings.
    pose(lt) gives the cylinder's centre and axis the tips are aimed at (default: the recording)."""
    plans, grids, radius, R = S["plans"], S["grids"], S["radius"], S["R"]
    pose = pose or S["carry"].pose

    def own(k, lt):
        p1, p2 = plans[k].inputs(lt)
        return grids[k].pad(p1, p2) - radius * grids[k].normal(p1, p2)

    def on(k, lt):
        F = plans[k].F
        return float(np.interp(lt, F.t, F.touch.astype(float)))

    def split(k, lt):
        p = own(k, lt)
        cc, uu = pose(lt)
        r = p - cc
        along = r @ uu
        perp = r - along * uu
        rho = float(np.linalg.norm(perp))
        return p, cc, uu, along, perp / rho, rho

    def fixed_depth(k, lt):
        w = on(k, lt)
        if w <= 0:
            return own(k, lt)
        p, cc, uu, along, e, rho = split(k, lt)
        return cc + along * uu + ((1 - w) * rho + w * (R + radius - PRESS)) * e

    def pushed_in(k, lt):
        w = on(k, lt)
        if w <= 0:
            return own(k, lt)
        p, cc, uu, along, e, rho = split(k, lt)
        return p - w * PRESS * e

    return own, on, split, {"fixed_depth": fixed_depth, "own_reach_pushed_in": pushed_in}


def stretch_measures(S, g, own, split, centre, c0=X.C0, nb=X.NB):
    """Measures 1 to 3 of the header from the gait rows g of a run's log."""
    out = {}
    for k, P in enumerate(S["plans"]):
        F = P.F
        c = c0 + nb * k
        sim = g[:, c + 2] > 0.5
        lt = g[:, 2]
        log_touch = np.interp(lt, F.t, F.touch.astype(float)) > 0.5
        runs, share = [], []
        for touching, a, b in F.spans:
            if not touching:
                continue
            sel = (lt >= F.t[a]) & (lt <= F.t[b])
            if sel.sum() < 2:
                continue
            x = sim[sel].astype(int)
            runs.append(int(x[0] + (np.diff(x) == 1).sum()))
            share.append(float(x.mean()))
        tt = lt[log_touch][::5]
        moved = np.array([np.linalg.norm(centre(k, v) - own(k, v)) for v in tt]) * 1e3
        beyond = np.array([abs(split(k, v)[3]) > S["half"] for v in tt])
        out[P.name] = {"logged_touching_stretches": len(runs), "touching_runs_in_each_stretch": runs,
                       "stretches_with_exactly_one_touching_run": int(sum(r == 1 for r in runs)),
                       "share_of_each_stretch_touching": share,
                       "share_of_logged_return_time_touching": float(sim[~log_touch].mean()) if (~log_touch).any() else 0.0,
                       "rest_position_moved_from_own_median_mm": float(np.median(moved)),
                       "rest_position_moved_from_own_max_mm": float(moved.max()),
                       "share_of_touching_time_own_position_beyond_the_flat_ends": float(beyond.mean())}
    return out


def stretch_lines(sm):
    rows = []
    for fn, fo in sm.items():
        rows.append(f"    {fn}: {fo['logged_touching_stretches']} logged touching stretches; touching runs in each "
                    f"{fo['touching_runs_in_each_stretch']}; exactly one run in {fo['stretches_with_exactly_one_touching_run']}; "
                    "share of each stretch touching " + ", ".join(f"{100 * v:.0f}%" for v in fo["share_of_each_stretch_touching"])
                    + f"; touching for {100 * fo['share_of_logged_return_time_touching']:.0f}% of the logged return time; rest "
                    f"position moved from the finger's own by a median {fo['rest_position_moved_from_own_median_mm']:.1f} mm, largest "
                    f"{fo['rest_position_moved_from_own_max_mm']:.1f} mm; own position beyond the flat ends for "
                    f"{100 * fo['share_of_touching_time_own_position_beyond_the_flat_ends']:.0f}% of touching time.")
    return rows


def finger_lines(ms):
    rows = []
    for fn, fo in ms["fingers"].items():
        rows.append(f"    {fn}: touching agrees with the log {100 * fo['share_of_gait_agreeing_with_log']:.0f}% of the gait "
                    f"(touching {100 * fo['share_touching']:.0f}%, log {100 * fo['log_share_touching']:.0f}%); whole lifts "
                    f"{fo['whole_lifts']} (log {fo['log_whole_lifts']}); normal force {fo['normal_force_mean_N']:.3f} N "
                    f"(log {fo['log_normal_force_mean_N']:.3f} N), tangential {fo['tangential_force_mean_N']:.3f} N; ball from "
                    f"its rest position median {fo['ball_from_rest_position_median_mm']:.1f} mm, largest "
                    f"{fo['ball_from_rest_position_max_mm']:.1f} mm"
                    + (f"; drawn skin cuts into the cylinder at most {fo['drawn_skin_deepest_cut_mm']:.1f} mm "
                       f"({100 * fo['drawn_skin_share_of_gait_cutting_in']:.0f}% of the gait)" if "drawn_skin_deepest_cut_mm" in fo else "")
                    + ".")
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", required=True)
    ap.add_argument("--gramian-map", required=True)
    ap.add_argument("--tag", default="")
    args = ap.parse_args()
    out = OUT + (("_" + args.tag) if args.tag else "")
    os.makedirs(out, exist_ok=False)
    S = setup(args)
    L, R, half, radius, carry, plans, grids = S["L"], S["R"], S["half"], S["radius"], S["carry"], S["plans"], S["grids"]
    t0, t1, rec_turn, stiff = S["t0"], S["t1"], S["rec_turn"], S["stiff"]
    own, on, split, readings = centres(S)
    earlier = json.load(open(EARLIER))
    what = {"fixed_depth": "FIXED DEPTH (on the clip): ball's surface the press-in depth inside the cylinder's side while the recording has the finger touching",
            "own_reach_pushed_in": "COMPARISON, own reach pushed in: the finger's own rest position moved the press-in depth towards the axis"}
    rep = ["Four two-pneu-net fingers turn a cylinder carried on an axle, tips aimed a fixed depth inside the surface: simulation with contact",
           "SIMULATION ONLY. Comparative, not predictive.", "",
           "Everything as the earlier axle run (" + os.path.relpath(os.path.dirname(EARLIER), ROOT).replace(os.sep, "/")
           + ") except the tips' rest positions while the recording has them touching.",
           f"Press-in depth {PRESS * 1e3:.1f} mm (ASSUMED: the reach checks' contact limit; fixed before any run, not tuned). "
           f"With the tip stiffnesses below, a tip held that far back by the surface pushes with "
           + ", ".join(f"{PRESS * v:.2f} N ({k})" for k, v in stiff.items())
           + f"; the recording's cylinder weighs {L['mass'] * 9.81:.2f} N.",
           f"Recording: cylinder radius {R * 1e3:.1f} mm, half length {half * 1e3:.1f} mm, friction {L['mu']:.2f}. Window: log "
           f"{t0:.2f} to {t1:.2f} s; the cylinder turns {rec_turn:+.1f} deg in it.",
           f"Finger stand-in: a tip ball of radius {radius * 1e3:.1f} mm and {X.TIP_MASS * 1e3:g} g (ASSUMED) on a spring. "
           "Tip stiffness (ASSUMED): " + ", ".join(f"{k} {v:.1f} N/m" for k, v in stiff.items()) + ".",
           f"Gait run {X.SLOW:g} times slower than the recording. Friction tip to cylinder {L['mu']:.2f} (ASSUMED: the log's value).",
           "Earlier axle run, tips aimed at the surface, two inputs: "
           + ", ".join(f"{k} {earlier['runs']['two_inputs_' + k]['turn_gait_deg']:+.1f} deg" for k in stiff) + ".", ""]
    results = {"window_log_s": [t0, t1], "recorded_turn_deg": rec_turn, "stiffness_N_per_m": stiff, "tip_ball_radius_m": radius,
               "press_in_depth_m": PRESS, "mounting": X.MOUNTING, "runs": {}}
    for sname, k_tip in stiff.items():
        for name, centre in readings.items():
            key = f"{name}_{sname}"
            homes = [(S["names"][k], centre(k, t0)) for k in range(4)]
            B = X.build(L, S["s0"], R, half, radius, homes, k_tip, key, out)
            r = X.run(B, plans, centre, carry, t0, t1)
            np.savetxt(os.path.join(out, f"log_{key}.csv"), r["log"], delimiter=",", header=X.HEADER, comments="", fmt="%.7g")
            np.savetxt(os.path.join(out, f"frames_{key}.csv"), r["frames"], delimiter=",", header=X.HEADER, comments="", fmt="%.9g")
            ms = X.measures(L, plans, r, rec_turn, grids, carry, R, half)
            lg = r["log"]
            ms["stretches"] = stretch_measures(S, lg[lg[:, 1] == 2], own, split, centre)
            results["runs"][key] = ms
            n0 = len(rep)
            rep += [f"{what[name]}; stiffness {sname} ({k_tip:.1f} N/m):",
                    f"  Turn over the gait {ms['turn_gait_deg']:+.1f} deg, {100 * ms['share_of_recorded_turn']:.0f}% of the recorded "
                    f"{rec_turn:+.1f} deg (the axle's own angle changed {ms['axle_angle_change_gait_deg']:+.1f} deg). In the wait "
                    f"before the gait {ms['turn_in_wait_deg']:+.1f} deg; in the {X.T_AFTER:g} s after it {ms['turn_after_the_gait_deg']:+.1f} deg.",
                    f"  Coasting: {ms['turn_with_a_tip_pressing_deg']:+.1f} deg turned while at least one tip pressed, "
                    f"{ms['turn_with_no_tip_pressing_deg']:+.1f} deg while none did "
                    f"({100 * ms['share_of_gait_time_with_no_tip_pressing']:.0f}% of the gait's time had no tip pressing above "
                    f"{A.TOUCH_N:g} N). The carried cylinder is off the recording by at most {ms['carry_off_target_largest_mm']:.3f} mm "
                    f"and {ms['carry_off_target_largest_deg']:.3f} deg. Tip-to-tip contacts seen: {ms['tip_tip_contacts_logged']}."
                    + (f" STOPPED: {ms['stopped']}." if ms["stopped"] else "")]
            rep += finger_lines(ms) + ["  Logged touching stretches:"] + stretch_lines(ms["stretches"]) + [""]
            print("\n".join(rep[n0:]), flush=True)
    m2 = results["runs"]["fixed_depth_main"]
    clean = [fn for fn, fo in m2["stretches"].items()
             if fo["stretches_with_exactly_one_touching_run"] == fo["logged_touching_stretches"]]
    results["fingers_with_one_touching_run_in_every_stretch_fixed_depth_main"] = clean
    rep += ["RULE (fixed in the header before the first run): the fixed-depth run at the main stiffness goes on the clip.",
            f"  It turns {m2['turn_gait_deg']:+.1f} deg, {100 * m2['share_of_recorded_turn']:.0f}% of the recorded turn. Fingers with "
            f"exactly one touching run in every logged touching stretch: {', '.join(clean) if clean else 'none'}.",
            "", "Not modelled: the finger's body and its bending under load (only a tip ball on a spring touches); the palm; "
            "anything holding the cylinder other than the axle; pressure dynamics. In the fixed-depth reading the finger's own "
            "radial reach is replaced by the assumed depth while touching. The drawn skin is not pressed in."]
    with open(os.path.join(out, "axle_report.txt"), "x", encoding="utf-8") as fh:
        fh.write("\n".join(rep) + "\n")
    with open(os.path.join(out, "axle_results.json"), "x", encoding="utf-8") as fh:
        json.dump(results, fh, indent=1)
    print("\n".join(rep[-5:]))


if __name__ == "__main__":
    main()
