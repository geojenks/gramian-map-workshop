"""Check the Shadow's self-hold on model copies that carry a joint spring,
through follow.py unchanged.

    python tools/hold_shadow_model_spring.py [--frames 2] [--table-only]

For each variant below, tools/make_shadow_spring_hand.py writes a copy of
the menagerie Shadow with stiffness k (N.m/rad) and springref (rad) on the
named joints of the four fingers, the copy is registered as a hand for this
process only, and follow() runs on the stored hold leader
data/leaders/hold_shadow__self.json with the stored fixed placement
outputs/hold_placement_shadow.json and scale margins, with no deflection
allowance (joint_stiffness=0). Writes

    outputs/hold_shadow__model_spring.json   every run's frames
    outputs/hold_shadow__model_spring.md     one row per variant (from the JSON)

What follow.py does with a model spring: the finger keeps its geometric
pose (equilibrium posing needs a spring on every free joint of the finger,
follow.equilibrium_spec), and the spring enters the torque test as a load
k (q - springref) on its joint that the inputs must supply
(follow._spring_item). The Shadow's J1 and J2 are separate joint rows with
one tendon input giving both the same torque, so the pad load's unbalanced
part is the J1-J2 difference; a J1-only spring shifts that difference by
k q1 at the pose, a spring on both J1 and J2 (equal k) by k (q1 - q2). The
pose is the check's own: follow() re-poses each finger from the leader's
contacts, so the posed angles are not the generator's (J1 != J2, and J1
differs per finger). The controls are the "posed" variants, whose
springref is each sprung joint's angle read from the no-spring run's
frame 1: k (q - springref) = 0 there, so they must reproduce the no-spring
copy whatever k is. The fixed-springref variants (0, and the generator's
0.94248) are kept as what they are: loads at the posed angles. The J1
series brackets k q1 around the no-spring run's unbalanced torque.
Whatever it shows, this is a torque at the pose, not a reaction to
deflection, so it cannot test the "J2 flexes first, J1 at the stop"
reading; crediting the stop is issue 26. Every number in the table is read
from the JSON.
"""
import argparse
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

# (k N.m/rad, joints, springref rad)
VARIANTS = [
    (0.0, "J1", 0.0),          # control: the copy with no spring, must match "shadow"
    (0.0005, "J1", 0.0),
    (0.001, "J1", 0.0),
    (0.0017, "J1", 0.0),
    (0.002, "J1", 0.0),
    (0.005, "J1", 0.0),
    (0.02, "J1", 0.0),
    (0.02, "J1J2", 0.0),       # equal k on J1 and J2: load k (q1 - q2) at the posed angles
    (0.02, "J1", 0.94248),     # springref at the generator's J1 angle, which the check does not keep
    (0.02, "J1", "posed"),     # control: springref at each posed J1, zero load
    (0.02, "J1J2", "posed"),   # control: springref at each posed J1 and J2, zero load
]

SPRUNG = {"J1": ("1",), "J1J2": ("1", "2")}
FINGER_PREFIX = {"index": "FF", "middle": "MF", "ring": "RF", "little": "LF"}


def posed_refs(frame, hand):
    """{joint name: posed angle} for every J1 and J2 of the four fingers,
    read from a frame's per-finger q (in the order of each finger's joints)."""
    import mujoco
    m = hand.model
    names = {f.label: [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in f.joints]
             for f in hand.fingers}
    out = {}
    for x in frame["fingers"]:
        if x["finger"] in FINGER_PREFIX:
            for n, q in zip(names[x["finger"]], x["q"]):
                if n[-2:] in ("J1", "J2"):
                    out[n] = float(q)
    return out

OUT = os.path.join(ROOT, "outputs", "hold_shadow__model_spring.json")


def run(frames, out_json):
    from gramian_workshop import hands
    from gramian_workshop.follow import follow
    from tools.make_shadow_spring_hand import make
    with open(os.path.join(ROOT, "data", "leaders", "hold_shadow__self.json"), encoding="utf-8") as fh:
        leader = json.load(fh)
    with open(os.path.join(ROOT, "outputs", "hold_placement_shadow.json"), encoding="utf-8") as fh:
        fixed = json.load(fh)
    runs = []
    refs = None
    for k, joints, ref in VARIANTS:
        if ref == "posed":
            if refs is None:
                refs = posed_refs(runs[0]["frames"][0], hands.Hand("shadow"))
            name, path = make(k, joints, refs, ref_label="posed")
            ref_rec = {n: refs[n] for n in refs if n[-1] in SPRUNG[joints]}
        else:
            name, path = make(k, joints, ref)
            ref_rec = ref
        key = f"shadow_spring_{name}"
        hands.HANDS[key] = dict(hands.HANDS["shadow"], base="local",
                                file=os.path.relpath(path, os.path.join(ROOT, "data", "hands")),
                                name=f"Shadow copy, {joints} spring {k:g} N.m/rad, springref {ref}")
        t0 = time.time()
        r = follow(key, leader, n_eval=frames, placement=fixed, margin=True, joint_stiffness=0.0)
        runs.append({"variant": name, "k_Nm_per_rad": k, "joints": joints, "springref_rad": ref_rec,
                     "model": os.path.relpath(path, ROOT).replace("\\", "/"),
                     "elapsed_s": round(time.time() - t0, 1), "frames": r["frames"]})
        print(f"{name}: {[f['verdict'] for f in r['frames']]} "
              f"{sorted({f['reason'] for f in r['frames'] if f['reason']})} "
              f"{[(x['finger'], x['state'], round(x.get('unbalanced_Nm') or 0, 5)) for x in r['frames'][0]['fingers']]} "
              f"{runs[-1]['elapsed_s']} s", flush=True)
        with open(out_json, "w", encoding="utf-8") as fh:
            json.dump({"hand": "shadow", "leader": leader["id"], "n_eval": frames, "placement": "fixed",
                       "joint_stiffness": 0.0,
                       "note": "model copies with a joint spring (tools/make_shadow_spring_hand.py), "
                               "checked through follow.py unchanged, which reads the spring as a "
                               "torque load k (q - springref) at the geometric pose",
                       "runs": runs}, fh, indent=1, default=float)


def table(out_json):
    with open(out_json, encoding="utf-8") as fh:
        d = json.load(fh)
    hdr = ("variant", "k N.m/rad", "joints", "springref rad", "verdict", "reason",
           "finger states (frame 1)", "unbalanced N.m per finger (frame 1)", "scale margin", "s")
    lines = ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
    for r in d["runs"]:
        frs = r["frames"]
        fing = frs[0]["fingers"]
        lines.append("| " + " | ".join([
            r["variant"], f"{r['k_Nm_per_rad']:g}", r["joints"],
            ("posed (per joint)" if isinstance(r["springref_rad"], dict) else f"{r['springref_rad']:g}"),
            ",".join(sorted({f["verdict"] for f in frs})),
            ",".join(sorted({f["reason"] for f in frs if f["reason"]})) or "-",
            ", ".join(f"{x['finger']} {x['state']}" for x in fing),
            ", ".join(f"{(x.get('unbalanced_Nm') or 0.0):.5f}" for x in fing),
            ",".join(f"{(f.get('scale_margin') or {}).get('margin', float('nan')):.3g}" for f in frs),
            f"{r['elapsed_s']}"]) + " |")
    return "\n".join(lines), d


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--frames", type=int, default=2)
    ap.add_argument("--table-only", action="store_true", help="only rewrite the .md from the stored .json")
    a = ap.parse_args()
    if not a.table_only:
        run(a.frames, OUT)
    t, d = table(OUT)
    out_md = OUT[:-5] + ".md"
    with open(out_md, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("# shadow: self-hold on model copies with a joint spring\n\n"
                 f"Leader `{d['leader']}`, fixed placement, {d['n_eval']} frames, scale margins, "
                 f"joint_stiffness {d['joint_stiffness']:g}. Written by `tools/hold_shadow_model_spring.py` "
                 f"from `outputs/{os.path.basename(OUT)}`. {d['note']}. Fingers in order index, "
                 "middle, ring, little, thumb.\n\n" + t + "\n")
    print(t)
    print(f"wrote {os.path.relpath(out_md, ROOT)}")


if __name__ == "__main__":
    main()
