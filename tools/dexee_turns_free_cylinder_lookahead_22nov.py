"""The Shadow DEX-EE turning a free cylinder, with sampled look-ahead control.

Simulation only: comparative, not predictive. The controller knows the full
state and plans with the same model.

Same task, same controller, same settings and same pass test as
tools/hands_turn_free_cylinder_lookahead_21nov.py (read its header first);
its run, measure and film are used as they are. Only the hand and the way its
reference is made are new.

The hand
--------
MuJoCo Menagerie shadow_dexee/shadow_dexee.xml, not edited: three identical
fingers, four joints each, one position input per joint (the model's own PID
controllers, with their integral term and 3.14 rad/s slew limit). The model
is weightless as shipped (like the Allegro here) and has no wrist, so its
base is fixed.

Three fingers from a four-finger recording
------------------------------------------
Decided before any run: the three fingers follow the recording's thumb,
first and ring fingertips; the middle finger, which sits between first and
ring, is dropped. Which DEX-EE finger takes which, and where the base sits,
come from a search over base poses that minimises fingertip miss plus the
depth by which other hand parts cut into the logged cylinder.

Fingertip: a ball fitted to the model's tip collision mesh (centre on the
mesh's lateral middle, radius half its thinner lateral width, touching the
mesh's far end). Targets are the Allegro's fingertip ball centres moved along
the cylinder's outward normal by the difference in ball radius. Joint targets
are held to at most REF_SPEED rad/s, below the model's slew limit.

Reference commands: joint target plus (fingertip force from the recording,
through the finger's Jacobian) divided by that joint's proportional gain, the
same recipe as for the Shadow Hand. The model's integral term adds force on
top of this for as long as a position error lasts, so the hand presses harder
than the recording asks; the report prints the cylinder's speed just after
release and the executed effort so this can be seen.

What to expect, written before the first run
--------------------------------------------
In the recording, three of the four fingers touch 90 % of the time. Of thumb,
first and ring, all three touch 37 % of the time and only two touch 63 % of
the time (printed again below from the log). The Allegro holds the cylinder
on fingertips alone. So a three-fingered hand playing this gait holds the
cylinder on two contacts most of the time. If the DEX-EE fails, that points
first at a four-finger gait played on three fingers, and at this controller
(which fails narrowly on the Allegro itself), not at the hand. Only a pass is
evidence about the hand.

Also to state beside the result: the same 0.03 rad spread of sampled changes
gives much less force on the DEX-EE's last joints (gain 0.6 against the
Allegro's 3), so the controller has less authority there.

What happened (added 22 Nov after the three-seed run; the text above is as
written before it). The expectation above was not what showed. The hand kept
the cylinder for 9 to 10 s and tilt passed on all three seeds; it failed
because actuators sat at their force limit 38 to 62 % of the time (and the
centre drifted 2 to 3 mm too far on two seeds). The joints at the limit are
the first two of two fingers, the high-gain ones, always on their positive
side (0.53 N.m on the first joint, its weaker side). At those moments the
joint is typically half a radian from its commanded angle while the sampled
change is about 0.05 rad, so the finger is blocked far from the reference,
not pushed there by the controller. Other finger parts were pressing on the
cylinder 30 to 58 % of the time. The remark about the last joints did not
matter.

Folders: ..._smoke_1s is empty (run cut off) and ..._smoke_1s_b is invalid
(the model file hangs the fingers on the world, so they had not moved with
the base); both are kept. ..._smoke_1s_c is the valid 1 s check and holds the
reference the three-seed run read.

The pass test is the one in the other tool's header, unchanged.
Nothing is overwritten: the output folder must not exist.
"""
import argparse
import json
import os
import pickle
import sys
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shadow_follows_hora_gait_mujoco_21nov as G  # noqa: E402
import hands_turn_free_cylinder_lookahead_21nov as K  # noqa: E402

DEX_FILE = "shadow_dexee/shadow_dexee.xml"
LOG_FINGERS = [0, 2, 3]                    # of the log's first, middle, ring, thumb
NAMES = ["first", "ring", "thumb"]
REF_SPEED = 3.0                            # rad/s, joint targets (model slew limit 3.14)
SPINS = 6                                  # starting base orientations per finger order
SCOUT_ITERS, SCOUT_MOMENTS, FIT_ITERS, FIT_MOMENTS = 100, 12, 300, 30
CUT_WEIGHT = G.CUT_WEIGHT
EARLIER = os.path.join(G.ROOT, "outputs", "hands_turn_free_cylinder_lookahead_21nov_both_hands")


def dex_hand(order):
    """order: which DEX-EE finger (0, 1, 2) follows first, ring, thumb."""
    return {"file": DEX_FILE, "root": "hand_base", "tied": [], "moves_with_root": ["F0/", "F1/", "F2/"],
            "fingers": [{"body": f"F{k}/finger_distal", "groups": [[f"F{k}/J{j}"] for j in range(4)],
                         "acts": [f"F{k}/J{j}_actuator" for j in range(4)]} for k in order]}


def gains():
    """Proportional gain of each joint's controller, read from the model file."""
    root = ET.parse(os.path.join(G.MEN, DEX_FILE)).getroot()
    kp = {}
    for inst in root.iter("instance"):
        for c in inst.iter("config"):
            if c.get("key") == "kp":
                kp[inst.get("name")] = float(c.get("value"))
    return [kp[f"actuator_J{j}"] for j in range(4)]


class DexIK(G.ShadowIK):
    def __init__(self, L, hand):
        self.L = L
        self.m = m = G.build(hand, np.eye(3), np.zeros(3), "free", False, L)
        self.d = mujoco.MjData(m)
        self.f = []
        for f in hand["fingers"]:
            g = [[m.joint(j).id for j in grp] for grp in f["groups"]]
            b = m.body(f["body"]).id
            gid = m.geom(f["body"].split("/")[0] + "/distal_geom_tip_col").id
            mid = m.geom_dataid[gid]
            V = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            Vb = V @ G.qmat(m.geom_quat[gid]).T + m.geom_pos[gid]
            lo, hi = Vb.min(0), Vb.max(0)
            r = float(min(hi[0] - lo[0], hi[1] - lo[1]) / 2)
            loc = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, hi[2] - r])
            f["loc"], f["r"] = loc.tolist(), r
            self.f.append({"body": b, "loc": loc, "r": r,
                           "qadr": [[m.jnt_qposadr[j] for j in grp] for grp in g],
                           "dadr": [[m.jnt_dofadr[j] for j in grp] for grp in g],
                           "lo": np.array([m.jnt_range[grp[0]][0] for grp in g]),
                           "hi": np.array([m.jnt_range[grp[0]][1] for grp in g])})
        self.distal = {f["body"] for f in self.f}
        self.obj_geom = m.geom("obj").id

    def track(self, R, tr, targets, idx, cut=False):
        L, m, d = self.L, self.m, self.d
        nf = len(self.f)
        d.qpos[:] = 0
        x = [self.neutral(k) for k in range(nf)]
        step = REF_SPEED * float(np.mean(np.diff(L["t"]))) if len(idx) == len(L["t"]) else None
        X = [np.zeros((len(idx), len(x[k]))) for k in range(nf)]
        miss = np.zeros((len(idx), nf)); depth = np.zeros(len(idx))
        for n, i in enumerate(idx):
            for k in range(nf):
                new, miss[n, k] = self.solve(k, R.T @ (targets[i, k] - tr), x[k])
                if step is not None and n > 0:
                    new = np.clip(new, x[k] - step, x[k] + step)
                    for a_, v_ in zip(self.f[k]["qadr"], new):
                        d.qpos[a_] = v_
                    mujoco.mj_kinematics(m, d)
                    miss[n, k] = float(np.linalg.norm(R.T @ (targets[i, k] - tr) - self.tip(k)))
                x[k] = new
                X[k][n] = x[k]
            if cut:
                G.set_obj(m, d, "free", R.T @ (L["op"][i] - tr), G.matq(R.T @ G.qmat(L["oq"][i])))
                mujoco.mj_kinematics(m, d); mujoco.mj_collision(m, d)
                for c in d.contact[:d.ncon]:
                    o = c.geom2 if c.geom1 == self.obj_geom else c.geom1 if c.geom2 == self.obj_geom else -1
                    if o >= 0 and m.geom_bodyid[o] not in self.distal and c.dist < 0:
                        depth[n] = max(depth[n], -c.dist)
        return X, miss, depth


def dexee_reference(L, al, say):
    n = len(L["t"])
    kp = gains()
    say("DEX-EE proportional gains read from the model file, joints 0 to 3: " + ", ".join(f"{v:g}" for v in kp))
    best = None
    for order in ([0, 1, 2], [1, 0, 2]):          # the other four orders are these two with the base spun by a third of a turn
        hand = dex_hand(order)
        ik = DexIK(L, hand)
        m, d = ik.m, ik.d
        out = np.zeros((n, 3, 3)); targets = np.zeros((n, 3, 3))
        for i in range(n):
            Ro = G.qmat(L["oq"][i])
            for k, lk in enumerate(LOG_FINGERS):
                out[i, k] = Ro @ G.cyl_out(Ro.T @ (al["tips"][i, lk] - L["op"][i]), L["R"], L["half"])
                targets[i, k] = al["tips"][i, lk] + (ik.f[k]["r"] - G.ALLEGRO["tips"][lk][2]) * out[i, k]
        d.qpos[:] = 0
        for k in range(3):
            for a, v in zip(ik.f[k]["qadr"], ik.neutral(k)):
                d.qpos[a] = v
        mujoco.mj_kinematics(m, d)
        tips0 = np.array([ik.tip(k).copy() for k in range(3)])
        goal = targets.mean(0)
        idx_s = np.linspace(0, n - 1, SCOUT_MOMENTS).astype(int)
        idx_f = np.linspace(0, n - 1, FIT_MOMENTS).astype(int)
        for s in range(SPINS):
            # fingers pointing up (the recording's hand is palm up), spun about the vertical
            R0 = G.rodrigues(np.array([0, 0, 2 * np.pi * s / SPINS])) if s else np.eye(3)
            t0 = goal.mean(0) - R0 @ tips0.mean(0)

            def cost(p, idx=idx_s, R0=R0, t0=t0):
                R = G.rodrigues(p[:3]) @ R0
                _, miss, depth = ik.track(R, t0 + p[3:], targets, idx, cut=True)
                return float((miss ** 2).mean() + CUT_WEIGHT * (depth ** 2).mean())

            p, c = G.nelder_mead(cost, np.zeros(6), [0.3] * 3 + [0.03] * 3, SCOUT_ITERS)
            say(f"  base search, finger order {order}, start spun {360 * s // SPINS} deg: cost {c:.3e}")
            if best is None or c < best[0]:
                best = (c, order, R0, t0, p)
    c_scout, order, R0, t0, p0 = best
    hand = dex_hand(order)
    ik = DexIK(L, hand)
    out = np.zeros((n, 3, 3)); targets = np.zeros((n, 3, 3))
    for i in range(n):
        Ro = G.qmat(L["oq"][i])
        for k, lk in enumerate(LOG_FINGERS):
            out[i, k] = Ro @ G.cyl_out(Ro.T @ (al["tips"][i, lk] - L["op"][i]), L["R"], L["half"])
            targets[i, k] = al["tips"][i, lk] + (ik.f[k]["r"] - G.ALLEGRO["tips"][lk][2]) * out[i, k]
    idx_f = np.linspace(0, n - 1, FIT_MOMENTS).astype(int)

    def cost(p):
        R = G.rodrigues(p[:3]) @ R0
        _, miss, depth = ik.track(R, t0 + p[3:], targets, idx_f, cut=True)
        return float((miss ** 2).mean() + CUT_WEIGHT * (depth ** 2).mean())

    p, c = G.nelder_mead(cost, p0, [0.1] * 3 + [0.01] * 3, FIT_ITERS)
    R, tr = G.rodrigues(p[:3]) @ R0, t0 + p[3:]
    say(f"DEX-EE base: fingers {order} follow first, ring, thumb; cost {c_scout:.3e} after the scouting search, "
        f"{c:.3e} after the fine one ({FIT_MOMENTS} moments); fingertip ball radius {1e3 * ik.f[0]['r']:.1f} mm")
    X, miss, depth = ik.track(R, tr, targets, np.arange(n), cut=True)
    reach = {}
    for k in range(3):
        f = ik.f[k]
        at = float(np.mean(np.any((X[k] <= f["lo"] + 1e-4) | (X[k] >= f["hi"] - 1e-4), axis=1)))
        reach[NAMES[k]] = {"mean_mm": float(1e3 * miss[:, k].mean()), "max_mm": float(1e3 * miss[:, k].max()),
                           "share_of_time_at_a_joint_limit": at}
        say(f"  reach, DEX-EE finger following the {NAMES[k]} finger: mean miss {reach[NAMES[k]]['mean_mm']:.1f} mm, "
            f"largest {reach[NAMES[k]]['max_mm']:.1f} mm; at a joint limit {100 * at:.0f} % of the time")
    say(f"  other hand parts cut into the logged cylinder by at most {1e3 * depth.max():.1f} mm "
        f"(mean {1e3 * depth.mean():.1f} mm)")
    m = G.build(hand, R, tr, "none", False, L)
    d = mujoco.MjData(m)
    ctrl = np.zeros((n, m.nu)); plain = np.zeros((n, m.nu)); qref = np.zeros((n, m.nq))
    for i in range(n):
        for k, f in enumerate(hand["fingers"]):
            for grp, v in zip(f["groups"], X[k][i]):
                qref[i, m.joint(grp[0]).qposadr[0]] = v
        d.qpos[:] = qref[i]; d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        for k, f in enumerate(hand["fingers"]):
            b = m.body(f["body"]).id
            tau = np.zeros(m.nv)
            lk = LOG_FINGERS[k]
            if L["N"][i, lk] > 0:
                cpt = d.xpos[b] + d.xmat[b].reshape(3, 3) @ np.array(f["loc"])
                tau = G.point_jac(m, d, b, cpt - f["r"] * out[i, k]).T @ L["f"][i, lk]
            for j, (grp, a, v) in enumerate(zip(f["groups"], f["acts"], X[k][i])):
                a = m.actuator(a).id
                plain[i, a] = v
                ctrl[i, a] = v + tau[m.joint(grp[0]).dofadr[0]] / kp[j]
    lo, hi = m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
    say(f"  reference commands outside the input range (clipped when played): "
        f"{100 * np.mean((ctrl < lo) | (ctrl > hi)):.1f} % of input samples; "
        f"largest push beyond the joint target {np.abs(ctrl - plain).max():.2f} rad")
    return {"R": R, "t": tr, "ctrl": ctrl, "plain": plain, "q0": qref[0], "qref": qref, "targets": targets,
            "reach": reach, "order": order, "hand": hand, "kp": kp,
            "cut_mm": [float(1e3 * depth.mean()), float(1e3 * depth.max())]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--seeds", default="0,1,2")
    ap.add_argument("--seconds", type=float, default=G.DUR)
    ap.add_argument("--refs", default="")                    # a references.pkl from an earlier run of this tool
    ap.add_argument("--no-film", action="store_true")
    a = ap.parse_args()
    out = os.path.join(G.ROOT, "outputs", "dexee_turns_free_cylinder_lookahead_22nov_" + a.tag)
    if os.path.exists(out):
        print("REFUSING to overwrite:", out)
        return 1
    os.makedirs(out)
    lines = []

    def say(s):
        print(s, flush=True)
        lines.append(s)

    L = G.load()
    sgn = np.sign(sum(float(np.einsum("ij,ij->", L["nrm"][:, k] * L["N"][:, k:k + 1],
                                      L["P"][:, k] - L["op"])) for k in range(4)))
    L["f"] = -sgn * L["nrm"] * L["N"][:, :, None]
    L["turn"] = G.own_axis_turn(L["oq"])
    L["axle_pos"], L["axle_quat"] = L["op"].mean(0), L["oq"][0]
    G.ALLEGRO["tips"] = G.hora_pads()
    say(f"Score settings: turn {K.TURN_TOL} rad, centre {K.POS_TOL} m, tilt {K.TILT_TOL} rad, change weight "
        f"{K.CHANGE_WEIGHT}, speed {K.SPEED_TOL} m/s, wobble {K.WOBBLE_TOL} rad/s")
    say(f"Look-ahead {K.HORIZON_S} s, new choice every {K.REPLAN_S} s, {K.SAMPLES} sampled changes, offsets spread "
        f"{K.SPREAD_RAD} rad, {K.KNOTS} knots; hold {K.HOLD_S} s before release; {a.seconds} s run; logged turn in "
        f"that time {np.degrees(np.interp(a.seconds, L['t'], L['turn'])):+.0f} deg")
    T = L["N"] > 0
    c3, c4 = T[:, LOG_FINGERS].sum(1), T.sum(1)
    say(f"Recording: of thumb, first and ring, all three touch {100 * (c3 == 3).mean():.0f} % of the time, two "
        f"{100 * (c3 == 2).mean():.0f} %, fewer {100 * (c3 < 2).mean():.1f} %; of all four fingers, three or more touch "
        f"{100 * (c4 >= 3).mean():.0f} %")
    if a.refs:
        with open(a.refs, "rb") as fo:
            ref = pickle.load(fo)
        say(f"reference read from {os.path.relpath(a.refs, G.ROOT)}")
    else:
        al = G.allegro_reference(L, say)
        t0 = time.time()
        ref = dexee_reference(L, al, say)
        say(f"  reference took {time.time() - t0:.0f} s to make")
    with open(os.path.join(out, "references.pkl"), "xb") as fo:
        pickle.dump(ref, fo)
    hand = ref["hand"]
    tips = [(f["body"], f["loc"]) for f in hand["fingers"]]
    xml = os.path.join(out, "model_dexee.xml")
    res, panels, keep = {}, [], {}
    for label, fb, seeds in (("no feedback", False, [0]), ("feedback", True, [int(x) for x in a.seeds.split(",")])):
        for seed in seeds:
            t0 = time.time()
            m, rec, stopped, p0, ax0 = K.run(hand, ref, L, False, tips, seed, fb,
                                             None if os.path.exists(xml) else xml, a.seconds)
            ms = K.measure(m, rec, stopped, p0, ax0, L, a.seconds, NAMES)
            ms["wall_s"] = time.time() - t0
            early = rec["t"] <= 0.1
            v = np.linalg.norm(np.diff(rec["op"][early], axis=0), axis=1) / np.diff(rec["t"][early])
            ms["centre_speed_first_0p1_s_max_m_s"] = float(v.max()) if len(v) else None
            ms["largest_share_of_force_range_at_release"] = float(rec["act"][0].max())
            key = f"dexee, {label}" + (f", seed {seed}" if fb else "")
            res[key] = ms; keep[key] = rec
            say(f"{key}: {'PASS' if ms['passes'] else 'FAIL'}; held until {ms['held_until_s']:.2f} s; turned "
                f"{ms['turn_deg']:+.0f} deg (recording {ms['logged_turn_deg_same_time']:+.0f}); centre moved at most "
                f"{ms['centre_from_release_mm_max']:.0f} mm, axis tilted at most {ms['axis_tilt_deg_max']:.0f} deg; "
                f"most time any actuator spent at its limit {100 * ms['largest_share_of_time_any_actuator_at_limit']:.1f} %; "
                f"other hand parts pressing {100 * ms['other_hand_parts_pressing_share']:.0f} % of the time; "
                f"controller's mean change to the commands {ms['controller_offset_rad_mean_abs']:.3f} rad; "
                f"took {ms['wall_s']:.0f} s to compute")
            say("    failed: " + "; ".join(k for k, v_ in ms["pass_test"].items() if not v_)) if not ms["passes"] else None
            say("    hardest-worked actuators: " + "; ".join(
                f"{n} largest {100 * v_['largest_share_of_force_range']:.0f} %, mean {100 * v_['mean_share_of_force_range']:.0f} % of range"
                for n, v_ in ms["actuators_hardest_worked"].items()))
            say(f"    fingertips touching, share of time: " + ", ".join(
                f"{n} {100 * s_:.0f} %" for n, s_ in ms["fingertips_touching_share"].items())
                + f"; cylinder's largest speed in the first 0.1 s {ms['centre_speed_first_0p1_s_max_m_s']:.3f} m/s; "
                f"hardest-pressed actuator at release {100 * ms['largest_share_of_force_range_at_release']:.0f} % of range")
            if (not fb) or seed == seeds[0]:
                panels.append(("Shadow DEX-EE" + (", no feedback" if not fb else ", with feedback"), xml, rec,
                               "no feedback: the reference commands as they are" if not fb else f"feedback, sampler seed {seed}"))
    np.savez(os.path.join(out, "runs.npz"), **{f"{k}|{n}": v for k, r in keep.items() for n, v in r.items()})
    with open(os.path.join(out, "results.json"), "x", encoding="utf-8") as fo:
        json.dump({"made_by": "tools/dexee_turns_free_cylinder_lookahead_22nov.py",
                   "settings": {"hold_s": K.HOLD_S, "horizon_s": K.HORIZON_S, "replan_s": K.REPLAN_S, "knots": K.KNOTS,
                                "samples": K.SAMPLES, "spread_rad": K.SPREAD_RAD, "turn_tol_rad": K.TURN_TOL,
                                "pos_tol_m": K.POS_TOL, "tilt_tol_rad": K.TILT_TOL, "change_weight": K.CHANGE_WEIGHT,
                                "speed_tol_m_s": K.SPEED_TOL, "wobble_tol_rad_s": K.WOBBLE_TOL,
                                "ref_speed_rad_s": REF_SPEED},
                   "reference": {k: ref[k] for k in ("order", "reach", "cut_mm", "kp")},
                   "pass_test": K.PASS, "runs": res}, fo, indent=1, default=lambda o: np.asarray(o).tolist())
    with open(os.path.join(out, "report.md"), "x", encoding="utf-8") as fo:
        fo.write("# The Shadow DEX-EE turning a free cylinder with sampled look-ahead control (22 Nov 2026)\n\n"
                 "Simulation only: comparative, not predictive. The controller knows the full state and plans with the "
                 "same model. Printed by `tools/dexee_turns_free_cylinder_lookahead_22nov.py`; what is assumed and what "
                 "was expected before the run are in its header, and the pass test is in the header of "
                 "`tools/hands_turn_free_cylinder_lookahead_21nov.py`.\n\n"
                 + "\n".join("- " + s.strip() for s in lines) + "\n")
    print("wrote", os.path.join(out, "results.json"))
    if not a.no_film:
        earlier = []
        Z = os.path.join(EARLIER, "runs.npz")
        if os.path.exists(Z) and a.seconds == G.DUR:
            Z = np.load(Z)
            for key, model, title in (("allegro, feedback, seed 0", "model_allegro.xml", "Allegro, with feedback"),
                                      ("shadow_tied, feedback, seed 0", "model_shadow_tied.xml",
                                       "Shadow Hand, last two joints tied, with feedback")):
                earlier.append((title, os.path.join(EARLIER, model),
                                {n: Z[f"{key}|{n}"] for n in ("t", "qpos", "turn")}, "feedback, sampler seed 0"))
        for f in K.film(out, L, earlier + panels, a.seconds, "dexee_turns_free_cylinder_22nov"):
            print("wrote", f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
