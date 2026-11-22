r"""Do the Allegro and the Shadow Hand turn a FREE cylinder, with feedback?
(21 Nov 2026)

Simulation only: comparative, not predictive.

What this is
------------
`shadow_follows_hora_gait_mujoco_21nov.py` moved the cylinder as recorded and
simulated only the hands. Here the cylinder is a free body with gravity, and
the hand has to hold and turn it. The same commands replayed with no feedback
lose the cylinder, so a controller is added.

The controller (sampled look-ahead)
-----------------------------------
Every REPLAN_S the controller tries SAMPLES small changes to the reference
commands over the next HORIZON_S, simulates each one ahead with the SAME model
from the true current state, scores them, and plays the best for REPLAN_S.
A change is one offset per actuator at KNOTS moments, joined by straight
lines; the best one is carried forward as the starting point of the next
round. The finger targets themselves stay fixed in the hand: feedback enters
only through the choice among the sampled changes. The controller knows the
full state of hand and cylinder and plans with the model it is run in. That
is far more than a real hand's controller has; the clip is labelled so.

Score (lower is better), averaged over the look-ahead:
  (turn about the cylinder's own axis minus the logged turn at that time)^2 / TURN_TOL^2
  + (centre minus where it was released)^2 / POS_TOL^2
  + (angle of its axis from the axis at release)^2 / TILT_TOL^2
  + CHANGE_WEIGHT * mean (offset / spread)^2
  + (speed of the centre)^2 / SPEED_TOL^2
  + (rate of turning about any axis but its own)^2 / WOBBLE_TOL^2
The task is scored, not the Allegro's path: the logged wander of the centre
is not asked for.

Start: the cylinder is held still at its first logged pose for HOLD_S while
the hand settles on its first command, then let go with its logged velocity.

Pass test, written before any run and not to be moved
-----------------------------------------------------
All of, for each of three sampler seeds, over the 10 s after release:
  1. the cylinder's centre never goes further than 30 mm from where it was
     released;
  2. its axis never tilts more than 30 deg from its direction at release;
  3. it turns about its own axis in the logged direction by between 0.5 and
     1.5 times the logged turn;
  4. no actuator is at its force limit for more than 5 % of the time.
A Shadow failure would point at the reference commands or this controller,
not at the hand. Only a pass is evidence about the hand.

Changed after the first Allegro run (folder ..._allegro_first: the cylinder
was turned -362 deg on two of three seeds but tossed about, failing the tilt
and centre bounds, with actuators at their limit 5 to 18 % of the time):
look-ahead 0.3 -> 0.5 s, 30 -> 100 samples, spread 0.05 -> 0.03 rad, charge
for changing the commands 0.02 -> 0.2, and the two speed terms added to the
score. The pass test is unchanged.

Then three other score settings were tried on the Allegro with --set, one
seed each (folders ..._allegro_third_a, _b, _c: smaller charge for changing
the commands, tighter centre tolerance, wider spread). All three did worse,
but seeds differ by more than those settings did, so this is weak evidence.
The constants were left as they are and tuning stopped there.

To state beside any result: every actuator receives sampled changes,
including the Shadow's two wrist inputs, so the Shadow can move its palm and
the fixed-base Allegro cannot; the Allegro is weightless (as in the
recording) and the Shadow carries its own weight; effort is counted only
after release.

Changed on 22 Nov so the DEX-EE tool can reuse run, measure and film: effort
is now divided by the force limit on the side the force is on (the same
number as before for the Allegro and the Shadow, whose limits are equal both
ways), the number of fingertips is no longer fixed at four, and the film's
file name can be given.

Nothing is overwritten: the output folder must not exist.
"""
import argparse
import json
import os
import pickle
import subprocess
import sys
import time

import mujoco
import numpy as np
from mujoco import rollout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shadow_follows_hora_gait_mujoco_21nov as G  # noqa: E402

HOLD_S = 0.5
HORIZON_S, REPLAN_S, KNOTS, SAMPLES = 0.50, 0.02, 4, 100
SPREAD_RAD = 0.03            # one-sigma size of a sampled offset (twice this on the two-joint tendon inputs)
TURN_TOL, POS_TOL, TILT_TOL, CHANGE_WEIGHT = 0.2, 0.01, 0.2, 0.2
SPEED_TOL, WOBBLE_TOL = 0.10, 2.0   # m/s of the centre; rad/s of turning about any axis but its own
THREADS = 8
PASS = {"centre_mm": 30.0, "tilt_deg": 30.0, "turn_share": (0.5, 1.5), "time_at_limit": 0.05}
W, H, FPS = 640, 600, 30


def qmul(a, b):
    w1, x1, y1, z1 = np.moveaxis(a, -1, 0); w2, x2, y2, z2 = np.moveaxis(b, -1, 0)
    return np.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], -1)


def qconj(q):
    return q * np.array([1, -1, -1, -1.0])


def twist(q_from, q_to):
    """Turn about the body's own z axis from q_from to q_to (rad), as G.own_axis_turn."""
    r = qmul(qconj(q_from), q_to)
    return 2 * np.arctan2(r[..., 3], r[..., 0]) - 2 * np.pi * (np.abs(2 * np.arctan2(r[..., 3], r[..., 0])) > np.pi) * np.sign(np.arctan2(r[..., 3], r[..., 0]))


def axis_of(q):
    w, x, y, z = np.moveaxis(q, -1, 0)
    return np.stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)], -1)


def run(hand, ref, L, tied, tips, seed, feedback, xml_path, seconds):
    m = G.build(hand, ref["R"], ref["t"], "free", tied, L, xml_path)
    d = mujoco.MjData(m)
    dt = m.opt.timestep
    nh = len(ref["q0"])
    qa, va = m.joint("obj_free").qposadr[0], m.joint("obj_free").dofadr[0]
    d.qpos[:nh] = ref["q0"]
    lim = m.actuator_ctrllimited.astype(bool)
    lo = np.where(lim, m.actuator_ctrlrange[:, 0], -np.inf); hi = np.where(lim, m.actuator_ctrlrange[:, 1], np.inf)
    spread = np.where(m.actuator_trntype == mujoco.mjtTrn.mjTRN_TENDON, 2 * SPREAD_RAD, SPREAD_RAD)
    # hold, then let go
    for _ in range(int(round(HOLD_S / dt))):
        d.qpos[qa:qa + 3], d.qpos[qa + 3:qa + 7] = L["op"][0], L["oq"][0]
        d.qvel[va:va + 6] = 0
        d.ctrl[:] = np.clip(ref["ctrl"][0], lo, hi)
        mujoco.mj_step(m, d)
    G.set_obj(m, d, "free", L["op"][0], L["oq"][0], L["ovel"][0])
    d.time = 0.0
    mujoco.mj_forward(m, d)
    p0, ax0 = d.qpos[qa:qa + 3].copy(), axis_of(d.qpos[qa + 3:qa + 7].copy())

    def ref_ctrl(ts):
        return np.column_stack([np.interp(ts, L["t"], ref["ctrl"][:, a]) for a in range(m.nu)])

    nH, nR = int(round(HORIZON_S / dt)), int(round(REPLAN_S / dt))
    kt = np.linspace(0, HORIZON_S, KNOTS)
    th = (np.arange(nH) + 0.0) * dt

    def weights(ts):          # straight-line joining of the knots, held flat past the last
        Wm = np.zeros((len(ts), KNOTS))
        for j, t_ in enumerate(ts):
            t_ = min(t_, kt[-1])
            i = min(int(np.searchsorted(kt, t_, side="right")) - 1, KNOTS - 2)
            w = (t_ - kt[i]) / (kt[i + 1] - kt[i])
            Wm[j, i], Wm[j, i + 1] = 1 - w, w
        return Wm
    Wh, Wshift = weights(th), weights(kt + REPLAN_S)
    sub = np.arange(4, nH, 5)
    rng = np.random.default_rng(seed)
    pool = rollout.Rollout(nthread=THREADS) if feedback else None
    datas = [mujoco.MjData(m) for _ in range(THREADS)] if feedback else None
    spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
    ns = mujoco.mj_stateSize(m, spec)
    state0 = np.zeros(ns)
    knots = np.zeros((KNOTS, m.nu))
    steps = int(round(seconds / dt))
    every = max(1, int(round(1 / (120 * dt))))
    ob, og = m.body("obj").id, m.geom("obj").id
    tb = [(m.body(b).id, np.array(loc)) for b, loc in tips]
    tip_bodies = [b for b, _ in tb]
    fr_lo, fr_hi = m.actuator_forcerange[:, 0], m.actuator_forcerange[:, 1]   # may be lopsided (DEX-EE)
    rec = {k: [] for k in ("t", "qpos", "op", "oq", "turn", "touch", "N", "other", "act", "offset")}
    turn, q_prev = 0.0, d.qpos[qa + 3:qa + 7].copy()
    stopped = None
    plan = None
    for s in range(steps + 1):
        t = s * dt
        if feedback and s % nR == 0:
            mujoco.mj_getState(m, d, state0, spec)
            base = ref_ctrl(t + th)
            cand = np.repeat(knots[None], SAMPLES + 2, 0)
            cand[1] = 0.0
            cand[2:] += rng.normal(size=(SAMPLES, KNOTS, m.nu)) * spread
            U = np.clip(base[None] + np.einsum("hk,bka->bha", Wh, cand), lo, hi)
            st, _ = pool.rollout(m, datas, state0[None].repeat(len(cand), 0), U)
            P = st[:, sub, 1 + qa:1 + qa + 3]; Q = st[:, sub, 1 + qa + 3:1 + qa + 7]
            V = st[:, sub, 1 + m.nq + va:1 + m.nq + va + 6]
            tw = turn + twist(np.broadcast_to(q_prev, Q.shape), Q)
            want = np.interp(t + (sub + 1) * dt, L["t"], L["turn"])
            tilt = np.arccos(np.clip(axis_of(Q) @ ax0, -1, 1))
            cost = (((tw - want) / TURN_TOL) ** 2 + (np.linalg.norm(P - p0, axis=2) / POS_TOL) ** 2
                    + (tilt / TILT_TOL) ** 2 + (np.linalg.norm(V[..., :3], axis=2) / SPEED_TOL) ** 2
                    + (np.linalg.norm(V[..., 3:5], axis=2) / WOBBLE_TOL) ** 2).mean(1) + CHANGE_WEIGHT * ((cand / spread) ** 2).mean((1, 2))
            cost[~np.isfinite(cost)] = np.inf
            b = int(np.argmin(cost))
            plan = np.clip(base[:nR] + Wh[:nR] @ cand[b], lo, hi)
            knots = Wshift @ cand[b]
        c = plan[s % nR] if feedback else np.clip(ref_ctrl(np.array([t]))[0], lo, hi)
        d.ctrl[:] = c
        q_now = d.qpos[qa + 3:qa + 7].copy()
        turn += float(twist(q_prev, q_now)); q_prev = q_now
        if s % every == 0:
            touch = np.zeros(len(tb), bool); N = np.zeros(len(tb)); other = False
            for ci in range(d.ncon):
                cn = d.contact[ci]
                if og not in (cn.geom1, cn.geom2):
                    continue
                f6 = np.zeros(6); mujoco.mj_contactForce(m, d, ci, f6)
                b_ = m.geom_bodyid[cn.geom1 if cn.geom2 == og else cn.geom2]
                if b_ in tip_bodies:
                    touch[tip_bodies.index(b_)] = True; N[tip_bodies.index(b_)] += f6[0]
                elif f6[0] > 0:
                    other = True
            rec["t"].append(t); rec["qpos"].append(d.qpos.copy()); rec["turn"].append(turn)
            rec["op"].append(d.xpos[ob].copy()); rec["oq"].append(d.xquat[ob].copy())
            rec["touch"].append(touch); rec["N"].append(N); rec["other"].append(other)
            af = d.actuator_force
            rec["act"].append(np.where(af >= 0, af / np.where(fr_hi > 0, fr_hi, np.inf),
                                       af / np.where(fr_lo < 0, fr_lo, -np.inf)))
            rec["offset"].append(c - np.clip(ref_ctrl(np.array([t]))[0], lo, hi))
        mujoco.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)) or np.linalg.norm(d.qpos[qa:qa + 3] - p0) > 0.15:
            stopped = t
            break
    if pool is not None:
        pool.close()
    return m, {k: np.array(v) for k, v in rec.items()}, stopped, p0, ax0


def measure(m, rec, stopped, p0, ax0, L, seconds, names=None):
    t = rec["t"]
    off = 1e3 * np.linalg.norm(rec["op"] - p0, axis=1)
    tilt = np.degrees(np.arccos(np.clip(axis_of(rec["oq"]) @ ax0, -1, 1)))
    logturn = float(np.interp(seconds, L["t"], L["turn"]))
    at_lim = (rec["act"] >= 1 - 1e-6).mean(0)
    first_out = np.where((off > PASS["centre_mm"]) | (tilt > PASS["tilt_deg"]))[0]
    share = float(rec["turn"][-1] / logturn)
    ok = {"centre stays within 30 mm": bool(off.max() <= PASS["centre_mm"] and stopped is None),
          "axis stays within 30 deg": bool(tilt.max() <= PASS["tilt_deg"] and stopped is None),
          "turn is 0.5 to 1.5 times the logged turn": bool(PASS["turn_share"][0] <= share <= PASS["turn_share"][1] and stopped is None),
          "no actuator at its limit over 5 % of the time": bool(at_lim.max() <= PASS["time_at_limit"])}
    names = names or ["first", "middle", "ring", "thumb"]
    order = np.argsort(-rec["act"].max(0))[:4]
    return {"ran_s": float(t[-1]), "stopped_at_s": stopped,
            "held_until_s": float(t[first_out[0]]) if len(first_out) else float(t[-1]),
            "turn_deg": float(np.degrees(rec["turn"][-1])), "logged_turn_deg_same_time": float(np.degrees(logturn)),
            "turn_deg_by_own_axis_sum": float(np.degrees(G.own_axis_turn(rec["oq"])[-1])),
            "centre_from_release_mm_max": float(off.max()), "axis_tilt_deg_max": float(tilt.max()),
            "fingertips_touching_share": {n: float(rec["touch"][:, k].mean()) for k, n in enumerate(names)},
            "fingertip_mean_normal_force_N": {n: float(rec["N"][:, k].mean()) for k, n in enumerate(names)},
            "fingertip_largest_normal_force_N": {n: float(rec["N"][:, k].max()) for k, n in enumerate(names)},
            "other_hand_parts_pressing_share": float(rec["other"].mean()),
            "actuators_hardest_worked": {m.actuator(int(a)).name: {
                "largest_share_of_force_range": float(rec["act"][:, a].max()),
                "mean_share_of_force_range": float(rec["act"][:, a].mean()),
                "share_of_time_at_limit": float(at_lim[a])} for a in order},
            "largest_share_of_time_any_actuator_at_limit": float(at_lim.max()),
            "controller_offset_rad_mean_abs": float(np.abs(rec["offset"]).mean()),
            "controller_offset_rad_largest": float(np.abs(rec["offset"]).max()),
            "pass_test": ok, "passes": bool(all(ok.values()))}


def film(out, L, panels, seconds, stem="hands_turn_free_cylinder_21nov"):
    from PIL import Image, ImageDraw, ImageFont
    cols = []
    for title, xml, rec, note in panels:
        m = mujoco.MjModel.from_xml_path(xml)
        m.vis.global_.offwidth, m.vis.global_.offheight = W, H
        m.vis.headlight.ambient[:] = 0.5
        m.vis.headlight.diffuse[:] = 0.7
        seg = mujoco.Renderer(m, H, W); seg.enable_segmentation_rendering()
        cols.append((title, m, mujoco.MjData(m), mujoco.Renderer(m, H, W), seg, rec, note))
    cam = mujoco.MjvCamera()
    cam.lookat[:] = L["op"].mean(0) + np.array([0, 0, -0.02])
    cam.distance, cam.azimuth, cam.elevation = 0.42, 135, -40
    font, small = ImageFont.truetype("arial.ttf", 20), ImageFont.truetype("arial.ttf", 15)
    mp4 = os.path.join(out, stem + ".mp4")
    still = os.path.join(out, stem + "_still.png")
    p = subprocess.Popen(["ffmpeg", "-n", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                          "-s", f"{len(cols) * W}x{H}", "-r", str(FPS), "-i", "-", "-pix_fmt", "yuv420p",
                          "-crf", "20", mp4], stdin=subprocess.PIPE)
    nf = int(seconds * FPS)
    for fi in range(nf + 1):
        t = fi / FPS
        row = []
        for title, m, d, r, seg, rec, note in cols:
            i = min(int(np.searchsorted(rec["t"], t)), len(rec["t"]) - 1)
            d.qpos[:] = rec["qpos"][i]
            mujoco.mj_forward(m, d)
            r.update_scene(d, cam); rgb = r.render().copy()
            seg.update_scene(d, cam); rgb[seg.render()[:, :, 0] < 0] = (236, 236, 232)
            im = Image.fromarray(rgb); dr = ImageDraw.Draw(im)
            dr.text((12, 10), title, fill=(30, 30, 30), font=font)
            gone = t > rec["t"][-1] + 1e-9
            dr.text((12, 38), f"t = {t:4.1f} s   turned {np.degrees(rec['turn'][i]):+.0f} deg "
                              f"(recording {np.degrees(np.interp(t, L['t'], L['turn'])):+.0f})"
                              + ("   cylinder lost" if gone else ""), fill=(30, 30, 30), font=small)
            dr.text((12, H - 66), note, fill=(90, 90, 90), font=small)
            dr.text((12, H - 46), "the cylinder is free; the controller knows the full state and plans with the same model",
                    fill=(90, 90, 90), font=small) if "feedback" in note and "no feedback" not in note else None
            dr.text((12, H - 26), "simulation only: comparative, not predictive", fill=(90, 90, 90), font=small)
            row.append(np.asarray(im))
        f = np.hstack(row)
        p.stdin.write(f.tobytes())
        if fi == nf // 2:
            Image.fromarray(f).save(still)
    p.stdin.close(); p.wait()
    return mp4, still


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--hands", default="allegro")           # comma list: allegro, shadow, shadow_tied
    ap.add_argument("--seeds", default="0")
    ap.add_argument("--seconds", type=float, default=G.DUR)
    ap.add_argument("--refs", default="")                    # a references.pkl from an earlier run
    ap.add_argument("--no-film", action="store_true")
    ap.add_argument("--set", default="")                     # e.g. CHANGE_WEIGHT=0.05,POS_TOL=0.006 (recorded in results)
    a = ap.parse_args()
    for kv in filter(None, a.set.split(",")):
        k_, v_ = kv.split("=")
        assert k_ in globals() and isinstance(globals()[k_], (int, float)), k_
        globals()[k_] = type(globals()[k_])(float(v_))
    out = os.path.join(G.ROOT, "outputs", "hands_turn_free_cylinder_lookahead_21nov_" + a.tag)
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
    hands = a.hands.split(",")
    say(f"Score settings: turn {TURN_TOL} rad, centre {POS_TOL} m, tilt {TILT_TOL} rad, change weight {CHANGE_WEIGHT}, "
        f"speed {SPEED_TOL} m/s, wobble {WOBBLE_TOL} rad/s")
    say(f"Look-ahead {HORIZON_S} s, new choice every {REPLAN_S} s, {SAMPLES} sampled changes, offsets spread "
        f"{SPREAD_RAD} rad, {KNOTS} knots; hold {HOLD_S} s before release; {a.seconds} s run; logged turn in that time "
        f"{np.degrees(np.interp(a.seconds, L['t'], L['turn'])):+.0f} deg")
    if a.refs:
        with open(a.refs, "rb") as fo:
            al, sh = pickle.load(fo)
        say(f"references read from {os.path.relpath(a.refs, G.ROOT)}")
    else:
        al = G.allegro_reference(L, say)
        sh = G.shadow_reference(L, al, say) if any(h.startswith("shadow") for h in hands) else None
    with open(os.path.join(out, "references.pkl"), "xb") as fo:
        pickle.dump((al, sh), fo)
    sh_tips = [(f["body"], f["loc"]) for f in G.SHADOW["fingers"]]
    al_tips = [(b, loc) for b, loc, _ in G.ALLEGRO["tips"]]
    setup = {"allegro": (G.ALLEGRO, al, False, al_tips, "Allegro"),
             "shadow": (G.SHADOW, sh, False, sh_tips, "Shadow Hand, model as it is"),
             "shadow_tied": (G.SHADOW, sh, True, sh_tips, "Shadow Hand, last two joints tied")}
    res, panels, keep = {}, [], {}
    for h in hands:
        hand, ref, tied, tips, title = setup[h]
        xml = os.path.join(out, f"model_{h}.xml")
        for label, fb, seeds in (("no feedback", False, [0]), ("feedback", True, [int(x) for x in a.seeds.split(",")])):
            for seed in seeds:
                t0 = time.time()
                m, rec, stopped, p0, ax0 = run(hand, ref, L, tied, tips, seed, fb,
                                               None if os.path.exists(xml) else xml, a.seconds)
                ms = measure(m, rec, stopped, p0, ax0, L, a.seconds)
                ms["wall_s"] = time.time() - t0
                key = f"{h}, {label}" + (f", seed {seed}" if fb else "")
                res[key] = ms; keep[key] = rec
                say(f"{key}: {'PASS' if ms['passes'] else 'FAIL'}; held until {ms['held_until_s']:.2f} s; turned "
                    f"{ms['turn_deg']:+.0f} deg (recording {ms['logged_turn_deg_same_time']:+.0f}); centre moved at most "
                    f"{ms['centre_from_release_mm_max']:.0f} mm, axis tilted at most {ms['axis_tilt_deg_max']:.0f} deg; "
                    f"most time any actuator spent at its limit {100 * ms['largest_share_of_time_any_actuator_at_limit']:.1f} %; "
                    f"other hand parts pressing {100 * ms['other_hand_parts_pressing_share']:.0f} % of the time; "
                    f"controller's mean change to the commands {ms['controller_offset_rad_mean_abs']:.3f} rad; "
                    f"took {ms['wall_s']:.0f} s to compute")
                say("    failed: " + "; ".join(k for k, v in ms["pass_test"].items() if not v)) if not ms["passes"] else None
                say("    hardest-worked actuators: " + "; ".join(
                    f"{n} largest {100 * v['largest_share_of_force_range']:.0f} %, mean {100 * v['mean_share_of_force_range']:.0f} % of range"
                    for n, v in ms["actuators_hardest_worked"].items()))
                if (not fb) or seed == seeds[0]:
                    panels.append((title + (", no feedback" if not fb else ", with feedback"), xml, rec,
                                   "no feedback: the reference commands as they are" if not fb else f"feedback, sampler seed {seed}"))
    np.savez(os.path.join(out, "runs.npz"), **{f"{k}|{n}": v for k, r in keep.items() for n, v in r.items()})
    with open(os.path.join(out, "results.json"), "x", encoding="utf-8") as fo:
        json.dump({"made_by": "tools/hands_turn_free_cylinder_lookahead_21nov.py",
                   "settings": {"hold_s": HOLD_S, "horizon_s": HORIZON_S, "replan_s": REPLAN_S, "knots": KNOTS,
                                "samples": SAMPLES, "spread_rad": SPREAD_RAD, "turn_tol_rad": TURN_TOL,
                                "pos_tol_m": POS_TOL, "tilt_tol_rad": TILT_TOL, "change_weight": CHANGE_WEIGHT,
                                "speed_tol_m_s": SPEED_TOL, "wobble_tol_rad_s": WOBBLE_TOL},
                   "pass_test": PASS, "runs": res}, fo, indent=1, default=lambda o: np.asarray(o).tolist())
    with open(os.path.join(out, "report.md"), "x", encoding="utf-8") as fo:
        fo.write("# Hands turning a free cylinder with sampled look-ahead control (21 Nov 2026)\n\n"
                 "Simulation only: comparative, not predictive. The controller knows the full state and plans with the "
                 "same model. Printed by `tools/hands_turn_free_cylinder_lookahead_21nov.py`; the pass test and what is "
                 "assumed are in its header.\n\n" + "\n".join("- " + s.strip() for s in lines) + "\n")
    print("wrote", os.path.join(out, "results.json"))
    if not a.no_film:
        for f in film(out, L, panels, a.seconds):
            print("wrote", f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
