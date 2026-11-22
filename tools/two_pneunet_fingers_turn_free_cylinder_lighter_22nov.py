"""Four two-pneu-net fingers, tips aimed a fixed depth inside the surface,
with the cylinder OFF the axle: free to slide, tip, fall and turn. Its mass
is swept downward. SIMULATION ONLY (MuJoCo). Comparative, not predictive.
Runs no SOFA. Numbers only; frames are saved in the axle clip tool's layout.

Why: on the axle the cylinder's weight and position are carried for it. The
question here is whether the same tips, pressing in by the same fixed
assumed depth, can hold a free cylinder and turn it, and if not at the
recorded mass, at what lower mass. A lighter cylinder means the same path at
a lower force.

What is simulated:
  - The cylinder: the 21 Nov contact simulation's "cylinder free" model
    (two_pneunet_fingers_follow_rl_gait_clip_21nov.py, its build() and
    run() used unchanged): free to slide three ways and to rotate any way,
    held at the recorded starting pose until let go, then nothing holds it
    but the tips. No palm, no floor.
  - The tips: a ball on a spring each, as on the axle. Rest positions are
    the "fixed depth" reading of
    two_pneunet_fingers_turn_cylinder_on_axle_fixed_press_in_22nov.py,
    unchanged: aimed at the RECORDED cylinder (not the simulated one), the
    press-in depth RC.MAX_CONTACT_MM inside its side while the recording has
    the finger touching; the finger's own position on returns. Nothing
    reacts to where the simulated cylinder has gone.
  - Tip stiffness: the main value only (ASSUMED).

FIXED BEFORE THE FIRST RUN:
  Runs, in this order:
    1. GATE: no gravity, recorded mass. If the cylinder is not held and
       turned with no weight at all, no lighter cylinder will be, and the
       cause is unbalanced pushes, not weight. The sweep is still run and
       reported, but nothing is claimed from it.
    2. The recorded mass, then 1/2, 1/4, 1/8, 1/16, 1/32 of it, with
       gravity. The moments of inertia are scaled with the mass.
  HELD: the run is not stopped (the 21 Nov rule: stopped once the cylinder
    has dropped A.DROP_STOP below its start) and the cylinder's centre
    stays within A.DROP_STOP of the recorded centre at the same log time
    through the whole gait.
  TURNS: turn about the cylinder's own axis over the gait has the
    recording's sign and is a larger share of the recorded turn than the
    21 Nov contact simulation's two-input run (the rule already used for
    the axle run).
  Reported beside them, with no threshold: the largest tilt of the axis
    from the recorded axis, the largest distance of the centre from the
    recorded centre, the touching-stretch measures of the press-in script.
  RESULT LINE: the heaviest mass in the list that is HELD and TURNS, or
    "none in this list".

Not modelled: the finger's body and its bending under load, the palm,
pressure dynamics. The finger's own radial reach is replaced by the assumed
depth while touching. The drawn skin is not pressed in.
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import pneunet_arrangement_replicates_rl_stroke_20nov as A                     # noqa: E402
import two_pneunet_fingers_follow_rl_gait_clip_21nov as C                      # noqa: E402
import two_pneunet_fingers_turn_cylinder_on_axle_21nov as X                    # noqa: E402
import two_pneunet_fingers_turn_cylinder_on_axle_fixed_press_in_22nov as PI    # noqa: E402

OUT = os.path.join(ROOT, "outputs", "two_pneunet_fingers_turn_free_cylinder_lighter_22nov")
MASS_SHARES = (1.0, 1 / 2, 1 / 4, 1 / 8, 1 / 16, 1 / 32)      # FIXED, see the header


class Aim:
    """What run() of the 21 Nov contact simulation needs from a finger: where the ball's spring rests, and the inputs."""
    def __init__(self, k, plan, centre):
        self.k, self.plan, self.centre, self.name, self.F = k, plan, centre, plan.name, plan.F

    def target(self, lt):
        return self.centre(self.k, lt)

    def inputs(self, lt):
        return self.plan.inputs(lt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", required=True)
    ap.add_argument("--gramian-map", required=True)
    ap.add_argument("--tag", default="")
    args = ap.parse_args()
    out = OUT + (("_" + args.tag) if args.tag else "")
    os.makedirs(out, exist_ok=False)
    S = PI.setup(args)
    L, R, half, radius, carry, plans = S["L"], S["R"], S["half"], S["radius"], S["carry"], S["plans"]
    t0, t1, rec_turn = S["t0"], S["t1"], S["rec_turn"]
    k_tip = S["stiff"]["main"]
    own, on, split, readings = PI.centres(S)
    centre = readings["fixed_depth"]
    aims = [Aim(k, plans[k], centre) for k in range(4)]
    C.OUT, C.RADIUS = out, radius            # build() of the 21 Nov simulation writes its model file into C.OUT
    earlier = json.load(open(X.EARLIER))
    e_share = earlier["runs"]["two_inputs_main_weight_carried"]["turn_gait_deg"] / earlier["recorded_turn_deg"]
    c0, u0 = carry.pose(t0)
    base_low = float(min(g.x[2] for g in S["grids"]))
    press_N = PI.PRESS * k_tip
    rep = ["Four two-pneu-net fingers, tips aimed a fixed depth inside the surface, cylinder off the axle; its mass swept downward",
           "SIMULATION ONLY. Comparative, not predictive.", "",
           "Cylinder free to slide, tip, fall and turn (the 21 Nov contact simulation's 'cylinder free' model); held at the recorded "
           f"start until let go at {C.T_LET_GO:g} s. No palm, no floor.",
           f"Tips: ball of radius {radius * 1e3:.1f} mm on a spring of {k_tip:.1f} N/m (ASSUMED, main value only), aimed at the "
           f"RECORDED cylinder, {PI.PRESS * 1e3:.1f} mm (ASSUMED, fixed beforehand) inside its side while the recording has the finger "
           f"touching. A tip held that far back pushes with {press_N:.2f} N; with friction {L['mu']:.2f} it can carry up to "
           f"{press_N * L['mu']:.2f} N along the surface.",
           f"Recording: mass {L['mass'] * 1e3:.1f} g (weight {L['mass'] * 9.81:.3f} N), radius {R * 1e3:.1f} mm, half length "
           f"{half * 1e3:.1f} mm. Window: log {t0:.2f} to {t1:.2f} s; the cylinder turns {rec_turn:+.1f} deg in it. Gait run "
           f"{C.SLOW:g} times slower.",
           f"HELD: not dropped {A.DROP_STOP * 1e3:.0f} mm and centre within {A.DROP_STOP * 1e3:.0f} mm of the recorded centre through "
           f"the gait. TURNS: the recording's sign and more than {100 * e_share:.0f}% of the recorded turn. Both fixed in the header "
           "before the first run.", ""]
    results = {"window_log_s": [t0, t1], "recorded_turn_deg": rec_turn, "tip_ball_radius_m": radius, "stiffness_N_per_m": k_tip,
               "press_in_depth_m": PI.PRESS, "recorded_mass_kg": float(L["mass"]), "turn_share_to_beat": e_share,
               "mounting": X.MOUNTING, "runs": {}}
    cases = [("no_gravity_recorded_mass", 1.0, False)] + [(f"mass_{1e3 * L['mass'] * f:.2f}_g".replace(".", "p"), f, True)
                                                           for f in MASS_SHARES]
    for key, f, gravity in cases:
        L2 = {k: L[k] for k in ("mu", "q", "I", "op", "com", "mass")}
        L2["mass"], L2["I"] = float(L["mass"]) * f, np.array(L["I"]) * f
        homes = [(S["names"][k], centre(k, t0)) for k in range(4)]
        B = C.build(L2, S["s0"], R, half, homes, k_tip, "free", key, base_low)
        if not gravity:
            B["m"].opt.gravity[:] = 0
        r = C.run(B, aims, t0, t1, c0, u0)
        lg = r["log"]
        np.savetxt(os.path.join(out, f"log_{key}.csv"), lg, delimiter=",", header=C.LOG_HEADER, comments="", fmt="%.7g")
        # frames in the axle clip tool's layout (the axle-angle column holds the turn; the pressing flag is recomputed)
        fr = []
        for rec, _ in r["frames"]:
            rec = np.asarray(rec)
            row = [rec[0], rec[1], rec[2], rec[3], rec[3], 0.0, *rec[7:14]]
            anyp = 0.0
            for k in range(4):
                b = rec[C.C0 + C.NB * k:C.C0 + C.NB * (k + 1)]
                row += [*b[0:5], *b[8:11], *b[11:14]]
                anyp = max(anyp, b[2])
            row[5] = anyp
            fr.append(row)
        np.savetxt(os.path.join(out, f"frames_{key}.csv"), np.array(fr), delimiter=",", header=X.HEADER, comments="", fmt="%.9g")
        g = lg[lg[:, 1] == 2]
        ms = {"mass_kg": L2["mass"], "gravity": gravity, "stopped": r["stopped"], "tip_tip_contacts_logged": r["tip_tip_contacts_logged"]}
        if len(g) > 1:
            off, tilt = [], []
            for row in g:
                cc, uu = carry.pose(row[2])
                off.append(np.linalg.norm(row[7:10] - cc))
                tilt.append(np.degrees(np.arccos(np.clip(A.rot(row[10:14], np.array([0, 0, 1.0])) @ uu, -1, 1))))
            ms.update({"turn_gait_deg": float(g[-1, 3] - g[0, 3]), "share_of_recorded_turn": float((g[-1, 3] - g[0, 3]) / rec_turn),
                       "gait_log_time_reached_s": float(g[-1, 2]),
                       "centre_from_recorded_largest_mm": float(1e3 * max(off)), "centre_from_recorded_at_end_mm": float(1e3 * off[-1]),
                       "axis_from_recorded_largest_deg": float(max(tilt)), "axis_from_recorded_at_end_deg": float(tilt[-1]),
                       "centre_drop_largest_mm": float(g[:, 6].max()),
                       "share_touching": {a.name: float((g[:, C.C0 + C.NB * k + 2] > 0.5).mean()) for k, a in enumerate(aims)},
                       "normal_force_mean_N": {a.name: (float(g[g[:, C.C0 + C.NB * k + 2] > 0.5, C.C0 + C.NB * k + 3].mean())
                                                        if (g[:, C.C0 + C.NB * k + 2] > 0.5).any() else 0.0) for k, a in enumerate(aims)},
                       "stretches": PI.stretch_measures(S, g, own, split, centre, c0=C.C0, nb=C.NB)})
            ms["held"] = bool(r["stopped"] is None and ms["centre_from_recorded_largest_mm"] < A.DROP_STOP * 1e3)
            ms["turns"] = bool(np.sign(ms["turn_gait_deg"]) == np.sign(rec_turn) and ms["share_of_recorded_turn"] > e_share)
        else:
            ms.update({"held": False, "turns": False})
        results["runs"][key] = ms
        n0 = len(rep)
        rep.append(("GATE, no gravity, recorded mass" if not gravity else
                    f"Mass {L2['mass'] * 1e3:.2f} g ({f:.4g} of the recorded; weight {L2['mass'] * 9.81:.3f} N)")
                   + f": {'HELD' if ms['held'] else 'NOT HELD'}, {'TURNS' if ms['turns'] else 'DOES NOT TURN'}."
                   + (f" STOPPED: {ms['stopped']}." if ms["stopped"] else ""))
        if "turn_gait_deg" in ms:
            rep += [f"  Turn over the gait {ms['turn_gait_deg']:+.1f} deg, {100 * ms['share_of_recorded_turn']:.0f}% of the recorded "
                    f"{rec_turn:+.1f} deg (gait reached log time {ms['gait_log_time_reached_s']:.2f} of {t1:.2f} s). Centre from the "
                    f"recorded centre: largest {ms['centre_from_recorded_largest_mm']:.1f} mm, at the end "
                    f"{ms['centre_from_recorded_at_end_mm']:.1f} mm; largest drop below the start {ms['centre_drop_largest_mm']:.1f} mm. "
                    f"Axis from the recorded axis: largest {ms['axis_from_recorded_largest_deg']:.1f} deg, at the end "
                    f"{ms['axis_from_recorded_at_end_deg']:.1f} deg. Tip-to-tip contacts seen: {ms['tip_tip_contacts_logged']}.",
                    "  Touching share of the gait: " + ", ".join(f"{n} {100 * v:.0f}%" for n, v in ms["share_touching"].items())
                    + ". Mean normal force while touching: " + ", ".join(f"{n} {v:.3f} N" for n, v in ms["normal_force_mean_N"].items()) + ".",
                    "  Logged touching stretches:"] + PI.stretch_lines(ms["stretches"])
        rep.append("")
        print("\n".join(rep[n0:]), flush=True)
    gate = results["runs"]["no_gravity_recorded_mass"]
    ok = [(k, results["runs"][k]["mass_kg"]) for k, _, gv in cases if gv and results["runs"][k]["held"] and results["runs"][k]["turns"]]
    results["gate_held_and_turns"] = bool(gate["held"] and gate["turns"])
    results["heaviest_mass_held_and_turned_kg"] = max(v for _, v in ok) if ok else None
    rep += ["RESULT (rules fixed in the header before the first run):",
            f"  Gate with no gravity: {'held and turned' if results['gate_held_and_turns'] else 'NOT held and turned'}.",
            ("  Heaviest cylinder in the list that was held and turned: "
             f"{1e3 * results['heaviest_mass_held_and_turned_kg']:.2f} g of the recorded {1e3 * L['mass']:.1f} g." if ok else
             "  No mass in this list was both held and turned."),
            "", "Not modelled: the finger's body and its bending under load; the palm; pressure dynamics. The tips are aimed at the "
            "recorded cylinder and do not react to the simulated one. The finger's own radial reach is replaced by the assumed depth "
            "while touching."]
    with open(os.path.join(out, "free_cylinder_report.txt"), "x", encoding="utf-8") as fh:
        fh.write("\n".join(rep) + "\n")
    with open(os.path.join(out, "axle_results.json"), "x", encoding="utf-8") as fh:
        json.dump(results, fh, indent=1)
    print("\n".join(rep[-6:]))


if __name__ == "__main__":
    main()
