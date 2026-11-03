"""Check the Shadow's self-hold with each finger modelled as an actuator in
its own right (gramian_workshop/finger_actuator.py), follow.py unchanged.

    python tools/hold_shadow_finger_actuators.py [--frames 2] [--variant NAME] [--table-only]

follow.py's per-finger test asks whether the finger's drive torques can
balance the pad load joint by joint at the posed angles. Here each finger
is a unit with a map from its inputs to the force it puts on the object,
found by a small forward model: the menagerie Shadow with the leader's
sphere fixed in the hand frame, every input set, the finger settled against
the sphere, and the contact force read. The tendon coupling of J1 and J2,
the joint end stops, joint damping, armature and dry friction, and the
servo force limits are all in the model, so they enter the map as they
are. For each finger the inputs are searched for the leader's share (its
station's force at its station): the input that presses hardest is bisected
for the force magnitude inside a damped Gauss-Newton on the others for the
point and the force direction. A finger holds its share when the force is
within follow's RESID_TOL of the demanded force and the point within
REACH_AMBER_M of the station; otherwise the row says why (no_contact,
reach, direction, strength with the actuator saturated or a stop carrying
the load, force_excess), and reports the stop and friction torques read
from the constraint rows at the best input found. The hand is then
assembled from the finger units: every finger's best input together, all
collisions on, the settled forces on the object summed to a wrench and
compared with the leader's (follow's RESID_TOL, torques over the object's
radius), and the sphere released under the leader's own net station force
reversed as its weight (hand links weightless, as follow.py has them) with
and without self-collision. Two variants: joint dry friction as the model
file has it (0.01 N.m) and with it zeroed. The pose is the stored follow
run's (outputs/follow_shadow__hold_shadow__self__fixed.json), the leader
data/leaders/hold_shadow__self.json with the stored fixed placement
outputs/hold_placement_shadow.json. Writes

    outputs/hold_shadow__finger_actuators__<variant>.json   every reading
    outputs/hold_shadow__finger_actuators.md                tables (from the JSONs)

Every number in the tables is read from the JSONs.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import follow as fo  # noqa: E402
from gramian_workshop.vendor import capability_core as cap  # noqa: E402

LEADER = os.path.join(ROOT, "data", "leaders", "hold_shadow__self.json")
PLACEMENT = os.path.join(ROOT, "outputs", "hold_placement_shadow.json")
POSE = os.path.join(ROOT, "outputs", "follow_shadow__hold_shadow__self__fixed.json")
OUT_STEM = os.path.join(ROOT, "outputs", "hold_shadow__finger_actuators")
VARIANTS = {"as_modelled": True, "no_joint_friction": False}   # name -> keep joint dry friction


def out_json(variant):
    return f"{OUT_STEM}__{variant}.json"


def _log(msg):
    print(msg, flush=True)


def _wrench(forces, points, c):
    W = np.zeros(6)
    for f, p in zip(forces, points):
        W += np.concatenate([f, np.cross(np.asarray(p) - c, f)])
    return W


def run(frames, variant, log=_log):
    from gramian_workshop import finger_actuator as fa
    from gramian_workshop.hands import Hand
    mj = fa._mj()
    with open(LEADER, encoding="utf-8") as fh:
        L = json.load(fh)
    with open(PLACEMENT, encoding="utf-8") as fh:
        P = json.load(fh)
    with open(POSE, encoding="utf-8") as fh:
        S = json.load(fh)
    hand = Hand("shadow")
    leader = fo.Leader(L)
    Rw = np.array(P["placement"]["rotation_matrix"])
    t = np.array(P["placement"]["translation_m"])
    assign = P["assignment"]
    mu = float(L.get("mu", 1.0))
    friction_loss = VARIANTS[variant]
    out = {"hand": "shadow", "leader": L["id"], "placement": "fixed", "pose_from": os.path.relpath(POSE, ROOT).replace("\\", "/"),
           "variant": variant, "joint_friction_as_modelled": friction_loss, "mu": mu,
           "object_radius_m": leader.R_obj, "object_mass_kg": leader.mass,
           "tolerances": {"resid_tol": fo.RESID_TOL, "reach_amber_m": fo.REACH_AMBER_M,
                          "reach_green_m": fo.REACH_GREEN_M, "deflect_max_m": fo.DEFLECT_MAX_M},
           "solver": {k: getattr(fa, k) for k in ("SETTLE_VEL", "SETTLE_MIN", "SETTLE_MAX", "PROBE_STEP",
                                                   "PRESS_STEP", "PRESS_EXPAND", "PRESS_BISECT", "PRESS_TOL_FRAC",
                                                   "AIM_FD", "AIM_ITERS", "AIM_STEP0", "AIM_STEP_MAX", "AIM_TRIES")},
           "frames": []}
    path = out_json(variant)

    def save():
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(out, fh, indent=1, default=lambda x: x.tolist() if hasattr(x, "tolist") else float(x))

    for k in range(min(frames, len(S["frames"]))):
        t_frame = time.time()
        c = leader.center(k, Rw, t)
        m = fa.build_scene(hand, c, leader.R_obj, mu=mu, friction_loss=friction_loss)
        sph = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, fa.SPHERE_GEOM)
        q0 = m.qpos0.copy()
        for x in S["frames"][k]["fingers"]:
            f = next(g for g in hand.fingers if g.label == x["finger"])
            q0[f.qadr] = x["q"]
        ctrl0 = fa.ctrl_for_pose(m, q0)
        made = leader.made(k)
        stations = {}
        for i in made:
            p, nrm, _ = leader.station(k, i, Rw, t, raw=True)
            stations[i] = (p, nrm, leader.force(k, i, Rw, raw=True))
        W = _wrench([stations[i][2] for i in made], [stations[i][0] for i in made], c)
        weight = -W[:3]
        fr = {"k": k, "t": float(leader.frames[k]["t"]), "center": c, "gravity_stated": leader.frames[k].get("gravity") or L["frame"].get("gravity"),
              "weight_N": weight, "leader_wrench": W,
              "self_collisions_at_pose": [{"body1": a, "body2": b, "depth_m": d} for a, b, d in fa.self_collisions(m, q0, sph)],
              "fingers": []}
        out["frames"].append(fr)
        save()
        log(f"[{variant}] frame {k}: {len(fr['self_collisions_at_pose'])} self-colliding pairs at the pose, "
            f"deepest {max([x['depth_m'] for x in fr['self_collisions_at_pose']] or [0]) * 1e3:.2f} mm")
        ctrl_all = ctrl0.copy()
        iso_forces, iso_points = [], []
        for f in hand.fingers:
            i = assign[f.label]
            t0 = time.time()
            rec = {"finger": f.label, "station": i, "joints": [m.joint(j).name for j in f.joints]}
            fr["fingers"].append(rec)
            if i not in made:
                rec["state"] = "off"
                continue
            p_star, nrm, f_star = stations[i]
            others = set().union(*[fa.finger_bodies(m, g) for g in hand.fingers if g is not f])
            was = fa.set_collisions(m, others, False)
            try:
                unit = fa.FingerUnit(hand, m, f, sph)
                names = [m.actuator(a).name for a in unit.inputs]
                start = unit.run(q0, ctrl0, ctrl0[unit.inputs])
                best, v = fa.solve_share(unit, q0, ctrl0, p_star, f_star, log=log)
            finally:
                fa.restore_collisions(m, was)
            ctrl_all[unit.inputs] = best["u"]
            iso_forces.append(best["force_on_object_N"])
            iso_points.append(best["point"] if best["contact"] else best["tip"])
            cl = best["closure"]
            rec.update({
                "inputs": names, "ctrlrange": unit.ctrlrange, "forcerange": unit.forcerange,
                "demanded_force_N": f_star, "station_m": p_star,
                "start": {"force_on_object_N": start["force_on_object_N"], "contact": start["contact"],
                          "point_departure_m": float(np.linalg.norm((start["point"] if start["contact"] else start["tip"]) - p_star)),
                          "at_stop": start["at_stop"], "q": start["q"], "steps": start["steps"], "settled": start["settled"]},
                "verdict": v,
                "best": {"u": best["u"], "q": best["q"], "at_stop": best["at_stop"], "saturated": best["saturated"],
                         "actuator_force_N": best["actuator_force"], "force_on_object_N": best["force_on_object_N"],
                         "point_m": best["point"], "tip_m": best["tip"], "contact": best["contact"],
                         "n_contacts": best["n_contacts"], "steps": best["steps"], "settled": best["settled"],
                         "torque_Nm": {key: cl[key] for key in ("actuator", "object", "stop", "friction", "other",
                                                                 "equality", "passive", "inertial", "unclosed")}},
                "elapsed_s": round(time.time() - t0, 1)})
            stops = [f"{n} {s} {float(tq):+.4f}" for n, s, tq in zip(rec["joints"], best["at_stop"], cl["stop"]) if s]
            log(f"[{variant}] frame {k} {f.label}: holds {v['holds']} why {v['why']} |f| {v['force_magnitude_N']:.4f} "
                f"of {np.linalg.norm(f_star):.4f} N, dF {v['force_departure_N']:.4f} N, angle "
                f"{v['angle_deg'] if v['angle_deg'] is None else round(v['angle_deg'], 1)} deg, dp {v['point_departure_m'] * 1e3:.2f} mm, "
                f"stops {stops or '-'}, friction max {np.abs(cl['friction']).max():.4f} N.m, "
                f"{v['n_eval']} evals, {rec['elapsed_s']} s")
            save()
        # the hand assembled from the finger units: every best input together
        t0 = time.time()
        d = mj.MjData(m)
        steps, settled = fa.settle(m, d, q0, ctrl_all)
        forces, points, per = [], [], {}
        for f in hand.fingers:
            cons = fa.object_contacts(m, d, sph, fa.finger_bodies(m, f))
            fsum = sum((x[1] for x in cons), np.zeros(3))
            per[f.label] = {"force_on_object_N": fsum, "n_contacts": len(cons)}
            for x in cons:
                forces.append(x[1])
                points.append(x[2])
        Wa = _wrench(forces, points, c)
        Wi = _wrench(iso_forces, iso_points, c)
        ws = cap.scale_wrench(W, leader.R_obj)
        tol = max(fo.RESID_TOL * float(np.linalg.norm(ws)), cap.FEAS_REL)
        res_a = float(np.linalg.norm(cap.scale_wrench(Wa - W, leader.R_obj)))
        res_i = float(np.linalg.norm(cap.scale_wrench(Wi - W, leader.R_obj)))
        fr["assembled"] = {"ctrl": ctrl_all, "steps": steps, "settled": settled, "finger": per,
                           "wrench": Wa, "residual_scaled": res_a, "isolated_sum_wrench": Wi,
                           "isolated_sum_residual_scaled": res_i, "tol_scaled": tol,
                           "wrench_ok": bool(res_a <= tol), "isolated_sum_ok": bool(res_i <= tol),
                           "self_collisions": [{"body1": a, "body2": b, "depth_m": dd} for a, b, dd in fa.self_collisions(m, d.qpos.copy(), sph)],
                           "elapsed_s": round(time.time() - t0, 1)}
        log(f"[{variant}] frame {k} assembled: settled {settled} in {steps} steps, wrench residual {res_a:.4f} "
            f"(isolated sum {res_i:.4f}) against {tol:.4f}")
        save()
        rel = {}
        for name, sc in (("all_collisions", True), ("no_self_collision", False)):
            t0 = time.time()
            r = fa.release_test(hand, c, leader.R_obj, leader.mass, weight, q0, ctrl_all, mu=mu,
                                friction_loss=friction_loss, self_collide=sc)
            r["elapsed_s"] = round(time.time() - t0, 1)
            rel[name] = r
            log(f"[{variant}] frame {k} release ({name}): drift {r['drift_m'] * 1e3:.2f} mm, held {r['held']}, "
                f"finger forces {[round(x, 3) for x in r['finger_force_N'].values()]} N")
        fr["release"] = rel
        fr["elapsed_s"] = round(time.time() - t_frame, 1)
        save()
    out["elapsed_s"] = sum(f.get("elapsed_s", 0) for f in out["frames"])
    save()
    return out


def _fmt_stops(rec):
    b = rec["best"]
    tq = b["torque_Nm"]["stop"]
    s = [f"{n} {st} {float(q):+.4f}" for n, st, q in zip(rec["joints"], b["at_stop"], tq) if st]
    return "; ".join(s) or "-"


def table(variants):
    """Markdown tables from the stored JSONs, plus the statement they
    support. Returns (markdown, data by variant)."""
    data = {}
    for v in variants:
        p = out_json(v)
        if os.path.exists(p):
            with open(p, encoding="utf-8") as fh:
                data[v] = json.load(fh)
    if not data:
        raise SystemExit("no stored JSON to tabulate")
    hdr = ("variant", "frame", "finger", "holds", "why", "demanded N", "achieved N", "force departure N",
           "angle deg", "point departure mm", "start point departure mm", "joints at stop (N.m)",
           "friction max N.m", "press input (note)", "J0 tendon input rad", "saturated", "evals", "s")
    lines = ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
    hand_hdr = ("variant", "frame", "fingers holding", "assembled wrench residual (scaled)", "isolated sum residual",
                "tolerance", "wrench ok", "release drift mm, all collisions (held)",
                "release drift mm, no self-collision (held)", "self-colliding pairs at pose (deepest mm)")
    hand_lines = ["| " + " | ".join(hand_hdr) + " |", "|" + "---|" * len(hand_hdr)]
    passes, fails = [], []
    for v, d in data.items():
        for fr in d["frames"]:
            holding = []
            for rec in fr["fingers"]:
                if rec.get("state") == "off":
                    lines.append(f"| {v} | {fr['k']} | {rec['finger']} | off |" + " |" * (len(hdr) - 4))
                    continue
                vd, b = rec["verdict"], rec["best"]
                j0 = [i for i, n in enumerate(rec["inputs"]) if n.endswith("J0")]
                if vd["holds"]:
                    holding.append(rec["finger"])
                lines.append("| " + " | ".join([
                    v, str(fr["k"]), rec["finger"], "yes" + (" (green)" if vd.get("green") else " (amber)") if vd["holds"] else "no",
                    vd["why"] or "-", f"{np.linalg.norm(rec['demanded_force_N']):.4f}", f"{vd['force_magnitude_N']:.4f}",
                    f"{vd['force_departure_N']:.4f}", "-" if vd["angle_deg"] is None else f"{vd['angle_deg']:.1f}",
                    f"{vd['point_departure_m'] * 1e3:.2f}", f"{rec['start']['point_departure_m'] * 1e3:.2f}",
                    _fmt_stops(rec), f"{np.abs(b['torque_Nm']['friction']).max():.4f}",
                    f"{vd['press_input_name']} {'+' if vd['press_sign'] > 0 else '-'} ({vd['press_note'] or 'ok'})",
                    f"{b['u'][j0[0]]:.3f} of {rec['ctrlrange'][j0[0]][1]:.3f}" if j0 else "-",
                    ",".join(n for n, s in zip(rec["inputs"], b["saturated"]) if s) or "-",
                    str(vd["n_eval"]), f"{rec['elapsed_s']}"]) + " |")
            if "assembled" not in fr:
                continue
            a, rel = fr["assembled"], fr.get("release", {})
            sc = fr["self_collisions_at_pose"]
            n_on = sum(1 for r in fr["fingers"] if r.get("state") != "off")
            hand_lines.append("| " + " | ".join([
                v, str(fr["k"]), f"{len(holding)} of {n_on}" + (f" ({', '.join(holding)})" if holding else ""),
                f"{a['residual_scaled']:.4f}", f"{a['isolated_sum_residual_scaled']:.4f}", f"{a['tol_scaled']:.4f}",
                "yes" if a["wrench_ok"] else "no",
                f"{rel['all_collisions']['drift_m'] * 1e3:.2f} ({'held' if rel['all_collisions']['held'] else 'not held'})" if "all_collisions" in rel else "-",
                f"{rel['no_self_collision']['drift_m'] * 1e3:.2f} ({'held' if rel['no_self_collision']['held'] else 'not held'})" if "no_self_collision" in rel else "-",
                f"{len(sc)} ({max([x['depth_m'] for x in sc] or [0]) * 1e3:.2f})"]) + " |")
            ok = len(holding) == n_on and a["wrench_ok"] and rel.get("all_collisions", {}).get("held", False)
            (passes if ok else fails).append((v, fr["k"], [r["finger"] for r in fr["fingers"] if r.get("state") != "off" and not r["verdict"]["holds"]],
                                              a["wrench_ok"], rel.get("all_collisions", {}).get("held")))
    if fails:
        why = {}
        for v, d in data.items():
            for fr in d["frames"]:
                for rec in fr["fingers"]:
                    if rec.get("state") != "off" and not rec["verdict"]["holds"]:
                        why.setdefault(rec["finger"], set()).add(rec["verdict"]["why"])
        statement = ("**The Shadow does not pass the finger-as-actuator check.** Frames failing: "
                     + "; ".join(f"{v} frame {k}: fingers not holding {fl or 'none'}, wrench ok {w}, release held {h}" for v, k, fl, w, h in fails)
                     + ". Reasons by finger: " + ", ".join(f"{f} ({'/'.join(sorted(x for x in s if x))})" for f, s in sorted(why.items())) + ".")
    else:
        statement = "**The Shadow passes the finger-as-actuator check** in every frame and variant stored."
    md = "\n".join(lines) + "\n\n" + "\n".join(hand_lines)
    return statement, md, data


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--frames", type=int, default=2)
    ap.add_argument("--variant", choices=sorted(VARIANTS), help="run one variant only (default: both, in turn)")
    ap.add_argument("--table-only", action="store_true", help="only rewrite the .md from the stored .json files")
    a = ap.parse_args()
    variants = [a.variant] if a.variant else list(VARIANTS)
    if not a.table_only:
        for v in variants:
            run(a.frames, v)
    statement, md, data = table(list(VARIANTS))
    first = next(iter(data.values()))
    out_md = OUT_STEM + ".md"
    pairs = first["frames"][0]["self_collisions_at_pose"]
    pair_lines = ["| body 1 | body 2 | depth mm |", "|---|---|---|"] + [
        f"| {x['body1']} | {x['body2']} | {x['depth_m'] * 1e3:.2f} |" for x in pairs]
    with open(out_md, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("# shadow: self-hold with each finger as an actuator\n\n"
                 f"Leader `{first['leader']}`, fixed placement, pose from `{first['pose_from']}`, "
                 f"variants {', '.join(data)} (joint dry friction as modelled / zeroed). Written by "
                 f"`tools/hold_shadow_finger_actuators.py` from `outputs/{os.path.basename(OUT_STEM)}__<variant>.json`. "
                 "A finger holds its share when its settled force on the object is within RESID_TOL "
                 f"({first['tolerances']['resid_tol']:g}) of the leader's and its contact within REACH_AMBER_M "
                 f"({first['tolerances']['reach_amber_m'] * 1e3:g} mm) of the station (green: within "
                 f"{first['tolerances']['reach_green_m'] * 1e3:g} mm). The hand row sums the assembled fingers' "
                 "forces to a wrench about the object's centre (torques over the radius) against the leader's, and "
                 "releases the sphere under the leader's net station force reversed as its weight; held means the "
                 f"centre drifts at most DEFLECT_MAX_M ({first['tolerances']['deflect_max_m'] * 1e3:g} mm) in 1 s.\n\n"
                 + statement + "\n\n## Per finger\n\n" + md.split("\n\n")[0] + "\n\n## Hand\n\n" + md.split("\n\n")[1]
                 + f"\n\n## Self-colliding finger pairs at the follow pose (frame {first['frames'][0]['k']})\n\n"
                 + "\n".join(pair_lines) + "\n")
    print(statement)
    print(f"wrote {os.path.relpath(out_md, ROOT)}")


if __name__ == "__main__":
    main()
