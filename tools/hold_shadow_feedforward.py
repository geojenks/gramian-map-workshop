"""Can each finger feed the hold's contact state forward? (6 Nov)

For the hold leader data/leaders/hold_<hand>__self.json with the stored
fixed placement, runs follow() as the stiffness bracket does (no added
joint stiffness, stops credited) to get each finger's pose and pad point,
then asks gramian_workshop/feedforward.py, per frame and finger: at that
pose and that contact, at that instant, can the finger apply the force the
leader demands? Nothing is settled or released and the object's stability
is not tested. follow.py is not changed.

Three readings of what the finger has:

  follow_bounds      inputs within the model's force range, stops, no joint
                     friction: follow.py's own torque test, here as a check
                     that the statics agree with it
  no_joint_friction  inputs within what a command inside the control range
                     can hold at this pose (and the force range), stops
  as_modelled        the same, plus each joint's dry friction as the model
                     file has it

Writes outputs/hold_<hand>__feedforward.json and .md (new names; nothing
stored is overwritten unless --overwrite is given).

  python tools/hold_shadow_feedforward.py [--hand shadow] [--frames N]
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

VARIANTS = {"follow_bounds": {"servo_range": False, "friction": False},
            "no_joint_friction": {"servo_range": True, "friction": False},
            "as_modelled": {"servo_range": True, "friction": True}}
TITLES = {"follow_bounds": "Force range only, no joint friction (follow.py's torque test)",
          "no_joint_friction": "Commands within the control range, no joint friction (the verdict)",
          "as_modelled": "Commands within the control range, plus the model's joint friction (necessary condition)"}
ORDER = ["no_joint_friction", "as_modelled", "follow_bounds"]
FOLLOW_ARGS = {"margin": True, "joint_stiffness": 0.0, "stop_reactions": True}


def run(hand_key="shadow", frames=None):
    from gramian_workshop import feedforward as ff
    from gramian_workshop import follow as fo
    from gramian_workshop.hands import Hand
    mj = ff._mj()
    leader_path = os.path.join(ROOT, "data", "leaders", f"hold_{hand_key}__self.json")
    place_path = os.path.join(ROOT, "outputs", f"hold_placement_{hand_key}.json")
    with open(leader_path, encoding="utf-8") as fh:
        L = json.load(fh)
    with open(place_path, encoding="utf-8") as fh:
        P = json.load(fh)
    n = len(L["frames"]) if frames is None else min(frames, len(L["frames"]))
    rec = fo.follow(hand_key, L, n_eval=n, placement=P, **FOLLOW_ARGS)
    hand = Hand(hand_key)
    m = hand.model
    d = mj.MjData(m)
    leader = fo.Leader(L)
    Rw = np.array(P["placement"]["rotation_matrix"])
    t = np.array(P["placement"]["translation_m"])

    def rel(p):
        return os.path.relpath(p, ROOT).replace("\\", "/")

    out = {"hand": hand_key, "leader": L["id"], "leader_file": rel(leader_path), "placement": rel(place_path),
           "pose_from": {"call": "follow()", "n_eval": n, **FOLLOW_ARGS},
           "mu": float(L.get("mu", 1.0)),
           "tolerances": {"resid_tol": fo.RESID_TOL, "unact_tol": fo.UNACT_TOL, "stop_tol": fo.STOP_TOL,
                          "reach_green_m": fo.REACH_GREEN_M, "reach_amber_m": fo.REACH_AMBER_M},
           "solver": {"eq_weight": ff.EQ_WEIGHT, "tie_weight": ff.TIE_WEIGHT},
           "variants": VARIANTS, "frames": []}
    for F in rec["frames"]:
        k = int(F["k"])
        c = leader.center(k, Rw, t)
        q = m.qpos0.copy()
        for x in F["fingers"]:
            f = next(g for g in hand.fingers if g.label == x["finger"])
            q[f.qadr] = x["q"]
        fr = {"k": k, "t": float(leader.frames[k]["t"]), "follow_verdict": F["verdict"],
              "follow_reason": F["reason"], "center": c, "fingers": []}
        for x in F["fingers"]:
            f = next(g for g in hand.fingers if g.label == x["finger"])
            if "pad_world" not in x or "q" not in x:
                continue
            body = f.tip_body if x["phalanx"] == "distal" else int(m.body_parentid[f.tip_body])
            pw = np.array(x["pad_world"])
            fv = leader.force(k, x["station"], Rw)
            n_in = (c - pw) / np.linalg.norm(c - pw)
            fmag = float(np.linalg.norm(fv))
            item = {"finger": x["finger"], "station": x["station"], "phalanx": x["phalanx"],
                    "joints": [m.joint(j).name for j in f.joints], "q": x["q"],
                    "joint_range": f.ranges, "pad_world": pw, "target": x["target"],
                    "reach_mm": x["reach_mm"], "follow_state": x["state"],
                    "follow_unbalanced_Nm": x.get("unbalanced_Nm"), "follow_stops": x.get("stops", []),
                    "demanded_force_N": fv, "demanded_force_norm_N": fmag, "contact_normal_in": n_in,
                    "demanded": ff.describe_force(fv, fv, n_in),
                    "departure_limit_N": fo.RESID_TOL * fmag, "variants": {}}
            for name, v in VARIANTS.items():
                d.qpos[:] = q
                T = ff.finger_terms(m, d, f.joints, pw, body, servo_range=v["servo_range"], stop_tol=fo.STOP_TOL)
                bal = ff.balance(T, fv, v["friction"])
                near = ff.nearest_force(T, fv, v["friction"])
                desc = ff.describe_force(near["force_N"], fv, n_in)
                ctrl = [ff.input_ctrl(m, d, a, bal["input_force"][r]) for r, a in enumerate(T["inputs"])]
                rest = [ff.input_ctrl(m, d, a, 0.0) for a in T["inputs"]]
                within = near["departure_N"] <= fo.RESID_TOL * fmag
                item["variants"][name] = {
                    "inputs": [m.actuator(a).name for a in T["inputs"]],
                    "input_moment": T["B"], "input_force_lo": T["a_lo"], "input_force_hi": T["a_hi"],
                    "input_range_set_by": T["a_src"],
                    "stops": [{"joint": m.joint(f.joints[jx]).name, "side": side} for jx, side, _ in T["stops"]],
                    "frictionloss_Nm": T["friction"] if v["friction"] else np.zeros(len(f.joints)),
                    "passive_Nm": T["passive"],
                    "needed_Nm": bal["needed_Nm"], "needed_norm_Nm": bal["needed_norm_Nm"],
                    "unbalanced_Nm": bal["unbalanced_Nm"], "unbalanced_by_joint_Nm": bal["unbalanced_by_joint_Nm"],
                    "unbalanced_share": bal["unbalanced_Nm"] / max(bal["needed_norm_Nm"], 1e-300),
                    "torque_within_follow_tol": bool(bal["unbalanced_Nm"]
                                                     <= fo.UNACT_TOL * bal["needed_norm_Nm"] + 1e-9),
                    "carried": {"input_force": bal["input_force"], "input_ctrl": ctrl, "input_ctrl_at_rest": rest,
                                "input_torque_Nm": bal["input_torque_Nm"], "friction_Nm": bal["friction_Nm"],
                                "friction_share": bal["friction_share"], "stop_Nm": bal["stop_Nm"]},
                    "nearest": {"force_N": near["force_N"], "departure_N": near["departure_N"],
                                "within_limit": bool(within), "balance_left_Nm": near["balance_left_Nm"],
                                "in_cone": bool(desc["normal_N"] > 0
                                                and desc["shear_N"] <= out["mu"] * desc["normal_N"]),
                                **desc},
                    "force": "yes" if within else "no"}
            fr["fingers"].append(item)
        out["frames"].append(fr)
    return out


def _enc(x):
    return x.tolist() if hasattr(x, "tolist") else float(x)


def _same(a, b):
    """Two frames' finger results agree to rounding."""
    def flat(o):
        if isinstance(o, dict):
            return [v for k in sorted(o) for v in flat(o[k])]
        if isinstance(o, list):
            return [v for x in o for v in flat(x)]
        return [o]
    x, y = flat(a["fingers"]), flat(b["fingers"])
    if len(x) != len(y):
        return False
    for u, v in zip(x, y):
        num = lambda z: isinstance(z, (int, float)) and not isinstance(z, bool)  # noqa: E731
        if num(u) and num(v):
            if abs(u - v) > 1e-9 + 1e-6 * abs(v):
                return False
        elif u != v:
            return False
    return True


def _short(name):
    return name.replace("rh_A_", "").replace("rh_", "")


def _carry(v, joints):
    c = v["carried"]
    bits = []
    for nm, a, lo, hi in zip(v["inputs"], c["input_force"], v["input_force_lo"], v["input_force_hi"]):
        if abs(a) > 1e-9:
            lim = hi if a > 0 else lo
            bits.append(f"{_short(nm)} {a:+.4f}" + (f" ({100 * abs(a) / abs(lim):.1f} % of its range)"
                                                      if abs(lim) > 0 else ""))
    stops = [f"{_short(j)} {s:+.4f} N.m" for j, s in zip(joints, c["stop_Nm"]) if abs(s) > 1e-9]
    fric = [f"{_short(j)} {p:+.4f} N.m ({100 * sh:.0f} %)" for j, p, sh in
            zip(joints, c["friction_Nm"], c["friction_share"]) if abs(p) > 1e-9]
    return "; ".join(bits) or "none", "; ".join(stops) or "none", "; ".join(fric) or "none"


def table(out):
    """The markdown report from a result as stored (plain lists)."""
    fr0 = out["frames"][0]
    nf = len(out["frames"])
    same = all(_same(fr0, fr) for fr in out["frames"][1:])
    tol = out["tolerances"]
    shown = "." if nf == 1 else (f", all giving the same numbers; frame {fr0['k']} is shown." if same
                                 else f"; frame {fr0['k']} is shown and the others differ (see the JSON).")
    lines = [f"# Can each {out['hand']} finger feed the hold's contact state forward?", "",
             f"Leader `{out['leader']}`, stored fixed placement, {nf} frame(s){shown} "
             "Made by `tools/hold_shadow_feedforward.py` from `gramian_workshop/feedforward.py`; "
             f"every number is in `outputs/hold_{out['hand']}__feedforward.json`.", "",
             "The check: with the finger at the pose follow.py found and touching at the pad point it chose, "
             "can the finger apply the leader's contact force at that instant, with nothing moving? "
             "Nothing is settled, the object is not released and its stability is not tested. "
             "The leader gives contact forces only, so no contact torque is checked.", "",
             "## Result", ""]

    def count(name):
        yes = [x["finger"] for x in fr0["fingers"] if x["variants"][name]["force"] == "yes"]
        no = [x["finger"] for x in fr0["fingers"] if x["variants"][name]["force"] == "no"]
        return (f"{len(yes)} of {len(fr0['fingers'])} fingers can apply the demanded force"
                + (f" ({', '.join(yes)})" if yes and no else "") + (f"; {', '.join(no)} cannot." if no else "."))

    lines += [f"- Verdict, with what the hand commands or leans on (servos within their control range, end "
              f"stops): {count('no_joint_friction')}",
              f"- With the model's joint friction as well: {count('as_modelled')}",
              f"- Check against follow.py (its force-range bounds, no friction): {count('follow_bounds')}",
              "", "Joint friction is not something the hand commands, so it is not an input to the verdict; it is "
              "a property the hand has (ruling of 6 Nov). What the hold needs of it is the minimum friction each "
              "joint must supply once the servos and stops have done all they can, against what the model has:",
              "", "| finger | joint | stiction needed (N.m) | the model's (N.m) | needed / model's |",
              "|---|---|---|---|---|"]
    clipped = False
    for x in fr0["fingers"]:
        v = x["variants"]["as_modelled"]
        rows = [(j, abs(p), fl) for j, p, fl in zip(x["joints"], v["carried"]["friction_Nm"], v["frictionloss_Nm"])
                if abs(p) > 1e-9]
        if v["force"] != "yes":
            lines.append(f"| {x['finger']} | - | not enough: {v['unbalanced_Nm']:.5f} N.m left unbalanced at the "
                         f"model's friction | | |")
            continue
        if not rows:
            lines.append(f"| {x['finger']} | none | 0 | | |")
        for j, p, fl in rows:
            at_ceiling = fl > 0 and p >= fl * (1 - 1e-6)
            clipped |= at_ceiling
            lines.append(f"| {x['finger']} | {_short(j)} | {p:.4f}{' (at least)' if at_ceiling else ''} | {fl:g} | "
                         f"{p / fl:.2f} |" if fl > 0 else f"| {x['finger']} | {_short(j)} | {p:.4f} | 0 | - |")
    lines += ["", ("A joint marked 'at least' uses all the friction the model has, so its true minimum is unknown. "
                   if clipped else "")
              + "Stiction holds a joint only while nothing else moves it, so this is a necessary condition for the "
              "hold, not a demonstration of it; whether the real hand's joints have this much is a question for "
              "its data sheet or a measurement, not this model.",
              "", f"Location: the pad point is {min(x['reach_mm'] for x in fr0['fingers']):.2f} to "
              f"{max(x['reach_mm'] for x in fr0['fingers']):.2f} mm from the leader's station "
              f"(follow.py's reach; green within {1e3 * tol['reach_green_m']:g} mm, amber within "
              f"{1e3 * tol['reach_amber_m']:g} mm).", ""]
    for name in ORDER:
        lines += [f"## {TITLES[name]}", "",
                  "| finger | reach (mm) | demanded (N) | torque needed (N.m) | left unbalanced (N.m) | share | "
                  "follow.py's unbalanced (N.m) | nearest force it can hold: off by (N) | limit (N) | "
                  "angle (deg) | shear / normal | force reproduced |",
                  "|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for x in fr0["fingers"]:
            v = x["variants"][name]
            nr = v["nearest"]
            fu = x["follow_unbalanced_Nm"]
            lines.append(f"| {x['finger']} | {x['reach_mm']:.2f} | {x['demanded_force_norm_N']:.4f} | "
                         f"{v['needed_norm_Nm']:.5f} | {v['unbalanced_Nm']:.5f} | "
                         f"{100 * v['unbalanced_share']:.1f} % | "
                         f"{'' if fu is None else format(fu, '.5f')} | {nr['departure_N']:.4f} | "
                         f"{x['departure_limit_N']:.4f} | {nr['angle_to_demanded_deg']:.1f} | "
                         f"{nr['shear_over_normal']:.2f} | {v['force']} |")
        lines += ["", "Who carries the demanded force's joint torque (least friction, then least input use):", "",
                  "| finger | inputs (N or N.m) | stops | joint friction (share of the model's) | "
                  "inputs with one side closed by the control range |",
                  "|---|---|---|---|---|"]
        for x in fr0["fingers"]:
            v = x["variants"][name]
            a, s, p = _carry(v, x["joints"])
            closed = [f"{_short(nm)} [{lo:+.3f}, {hi:+.3f}]" for nm, lo, hi, src in
                      zip(v["inputs"], v["input_force_lo"], v["input_force_hi"], v["input_range_set_by"])
                      if "ctrl range" in src and (lo >= 0 or hi <= 0)]
            lines.append(f"| {x['finger']} | {a} | {s} | {p} | {'; '.join(closed) or 'none'} |")
        lines.append("")
    lines += ["## How to read it", "",
              "- Torque needed is the joint torque the demanded force asks of the finger (J^T f at the pad point). "
              "Left unbalanced is the smallest part of it that the finger's inputs, stops and (where credited) "
              "joint friction cannot supply; follow.py passes the finger when this is within "
              f"{100 * tol['unact_tol']:g} % of the torque needed.",
              "- The nearest force it can hold is the force closest to the demanded one that the finger balances "
              "exactly at this contact. The force counts as reproduced when it is within "
              f"{100 * tol['resid_tol']:g} % of the demanded force. A small unbalanced torque can still mean a "
              "distant force: the two end joints of a finger share one tendon, which gives them equal torque, "
              "so without friction or a stop the finger can only hold forces that load those two joints equally.",
              "- An input with one side closed by the control range sits where its position command cannot go "
              "further (the joint is posed at the end of the command range), so at this exact pose it can push "
              "one way only. The command each input needs is in the JSON (`carried.input_ctrl`, next to "
              "`input_ctrl_at_rest`).",
              "- This leader was made from kinematics and asks for pure normal forces. It is a dry run of the "
              "method; a leader extracted from a simulation with its shear forces is the real test.", ""]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hand", default="shadow")
    ap.add_argument("--frames", type=int, default=None)
    ap.add_argument("--overwrite", action="store_true")
    ap.add_argument("--table-only", action="store_true", help="re-render the report from the stored JSON")
    ap.add_argument("--report", default=None, help="where the report goes (default outputs/hold_<hand>__feedforward.md)")
    a = ap.parse_args()
    base = os.path.join(ROOT, "outputs", f"hold_{a.hand}__feedforward")
    report = a.report or base + ".md"
    if a.table_only:
        if not a.overwrite and os.path.exists(report):
            sys.exit(f"{report} exists: stored outputs are not overwritten (use --overwrite)")
    else:
        if not a.overwrite and (os.path.exists(base + ".json") or os.path.exists(report)):
            sys.exit(f"{base}.json or {report} exists: stored outputs are not overwritten (use --overwrite)")
        out = run(a.hand, a.frames)
        with open(base + ".json", "w", encoding="utf-8") as fh:
            json.dump(out, fh, indent=1, default=_enc)
    with open(base + ".json", encoding="utf-8") as fh:
        text = table(json.load(fh))
    with open(report, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    print(text)


if __name__ == "__main__":
    main()
