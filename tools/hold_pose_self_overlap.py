"""Does the hand overlap itself at the stored self-hold poses?

    python tools/hold_pose_self_overlap.py                  # every stored hold result
    python tools/hold_pose_self_overlap.py --out-dir DIR    # write somewhere else

Reads the stored hold results (outputs/follow_<hand>__hold_<hand>__self__fixed.json)
and their leaders (data/leaders/hold_<hand>__self.json); runs no follow check.
For each hand it sets three poses in MuJoCo and lists the pairs of the hand's
own bodies whose collision geoms penetrate (the object is left out):

  rest       the model's rest pose (hand.base_q), couplings applied
  generator  the pose the hold generator chose (the leader's source.pose_qpos)
  follow     the pose the follow check solved, each stored frame (each
             finger's "q", set as the check sets it: FingerIK's free joints,
             then the couplings)

A pair that penetrates at rest is how the model is built; a pair that
penetrates only at a hold pose is made by the pose.

Before the pairs are read, the follow pose is checked against the stored
result: each finger's depth into the ball is recomputed at the pose with the
check's own measure (FingerIK.overlaps) and compared with the stored
overlap_mm and other_overlap_mm. If these agree the pose is the one the
verdict was given at.

What the pair list cannot see: MuJoCo drops contacts between a body and its
parent, between pairs the model excludes, and for geoms with no collision
bits. So the list is a lower bound on the overlap.

Nothing here is a new verdict: these are facts about the geometry of poses
whose verdicts are already stored.
"""
import argparse
import glob
import json
import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop import finger_actuator as fa       # noqa: E402
from gramian_workshop.follow import FingerIK              # noqa: E402
from gramian_workshop.hands import Hand                   # noqa: E402

SCHEMA = "gramian-workshop/hold-pose-self-overlap@1"
DEEP_M = 0.0005          # pairs deeper than this are listed by name in the table


def _git():
    try:
        sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
        dirty = bool(subprocess.check_output(["git", "status", "--porcelain", "tools/hold_pose_self_overlap.py"],
                                            cwd=ROOT, text=True).strip())
        return sha + ("-dirty" if dirty else "")
    except Exception:                                   # noqa: BLE001
        return None


def owners(m, hand):
    """body id -> the finger it belongs to (None: palm, wrist or base)."""
    own = {}
    for f in hand.fingers:
        for b in fa.finger_bodies(m, f):
            own[b] = f.label
    return own


def pairs_at(mj, m, sph, q, own):
    """Penetrating pairs of the hand's own bodies at q: one record per body
    pair with its deepest contact."""
    d = mj.MjData(m)
    d.qpos[:] = q
    mj.mj_forward(m, d)
    best = {}
    for i in range(d.ncon):
        con = d.contact[i]
        if sph in (con.geom1, con.geom2) or con.dist >= 0:
            continue
        g1, g2 = int(con.geom1), int(con.geom2)
        if m.geom_bodyid[g1] > m.geom_bodyid[g2]:
            g1, g2 = g2, g1
        key = (int(m.geom_bodyid[g1]), int(m.geom_bodyid[g2]))
        if key not in best or -con.dist > best[key][0]:
            best[key] = (-float(con.dist), g1, g2)
    out = []
    for (b1, b2), (dep, g1, g2) in sorted(best.items()):
        out.append({"body1": m.body(b1).name, "body2": m.body(b2).name,
                    "finger1": own.get(b1), "finger2": own.get(b2),
                    "depth_mm": 1e3 * dep,
                    "geom_types": [mj.mjtGeom(int(m.geom_type[g])).name[len("mjGEOM_"):].lower() for g in (g1, g2)]})
    return out


def kind(p):
    a, b = p["finger1"], p["finger2"]
    if a is None and b is None:
        return "base-base"
    if a is None or b is None:
        return "base-finger"
    return "same finger" if a == b else "finger-finger"


def one(mj, key, res_path):
    with open(res_path, encoding="utf-8") as fh:
        S = json.load(fh)
    with open(os.path.join(ROOT, "data", "leaders", f"hold_{key}__self.json"), encoding="utf-8") as fh:
        L = json.load(fh)
    hand = Hand(key)
    R = float(L["object"]["radius_m"])
    c0 = np.array(S["frames"][0]["ball_center"], float)
    m = fa.build_scene(hand, c0, R, mu=float(L.get("mu", 1.0)))
    assert m.nbody == hand.model.nbody and m.nq == hand.model.nq
    sph = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, fa.SPHERE_GEOM)
    own = owners(m, hand)
    iks = {f.label: FingerIK(hand, f) for f in hand.fingers}
    by = {f.label: f for f in hand.fingers}

    ncol = int(sum(1 for g in range(m.ngeom) if g != sph and (m.geom_contype[g] or m.geom_conaffinity[g])))
    rec = {"hand": key, "result": os.path.relpath(res_path, ROOT).replace("\\", "/"),
           "leader": f"data/leaders/hold_{key}__self.json", "leader_commit": L.get("source", {}).get("commit"),
           "radius_m": R, "verdicts": sorted({f["verdict"] for f in S["frames"]}),
           "model": {"colliding_geoms": ncol, "bodies": int(m.nbody) - 1, "contact_excludes": int(m.nexclude)}}

    rest = pairs_at(mj, m, sph, hand.apply_coupling(np.array(hand.base_q)), own)
    rec["rest"] = rest
    at_rest = {(p["body1"], p["body2"]) for p in rest}

    def mark(ps):
        for p in ps:
            p["kind"] = kind(p)
            p["at_rest_too"] = (p["body1"], p["body2"]) in at_rest
        return ps
    mark(rest)

    gq = L.get("source", {}).get("pose_qpos")
    if gq:
        q = np.array(hand.base_q)
        for lab, v in gq.items():
            q[by[lab].qadr] = v
        rec["generator"] = mark(pairs_at(mj, m, sph, hand.apply_coupling(q), own))
    else:
        rec["generator"] = None

    rec["follow"] = []
    for fr in S["frames"]:
        c = np.array(fr["ball_center"], float)
        q = np.array(hand.base_q)
        check = []
        for x in fr["fingers"]:
            f, ik = by[x["finger"]], iks[x["finger"]]
            qf = np.array(x["q"], float)
            assert len(qf) == len(ik.qadr), (key, x["finger"], len(qf), len(ik.qadr))
            q[ik.qadr] = qf
            ik._set_pose(qf)
            body = f.tip_body if x["phalanx"] == "distal" else int(hand.model.body_parentid[f.tip_body])
            pc, po = (float(o[0]) for o in ik.overlaps(body, (c, R)))
            d1 = None if x["overlap_mm"] is None else 1e3 * pc - x["overlap_mm"]
            d2 = None if x["other_overlap_mm"] is None else 1e3 * po - x["other_overlap_mm"]
            check.append({"finger": x["finger"], "overlap_mm_recomputed": 1e3 * pc if np.isfinite(pc) else None,
                          "overlap_mm_stored": x["overlap_mm"], "difference_mm": d1,
                          "other_overlap_mm_recomputed": 1e3 * po if np.isfinite(po) else None,
                          "other_overlap_mm_stored": x["other_overlap_mm"], "other_difference_mm": d2})
        q = hand.apply_coupling(q)
        # the fingers are separate chains: the whole hand's pose puts every
        # finger's links where its own solve had them
        dd = mj.MjData(hand.model)
        dd.qpos[:] = q
        mj.mj_kinematics(hand.model, dd)
        shift = 0.0
        for x in fr["fingers"]:
            ik = iks[x["finger"]]
            ik._set_pose(np.array(x["q"], float))
            for b in fa.finger_bodies(hand.model, by[x["finger"]]):
                shift = max(shift, float(np.linalg.norm(dd.xpos[b] - ik.d.xpos[b])))
        ps = mark(pairs_at(mj, m, sph, q, own))
        # where two fingers' touching links overlap: how far apart their
        # contact points on the ball are, against the links' thickness
        touch = {}
        for x in fr["fingers"]:
            f = by[x["finger"]]
            body = f.tip_body if x["phalanx"] == "distal" else int(hand.model.body_parentid[f.tip_body])
            prox = iks[x["finger"]].proxies(middle=True)
            touch[hand.model.body(body).name] = (np.array(x["pad_world"], float),
                                                 float(prox[body][2]) if body in prox else None)
        for p in ps:
            if p["body1"] in touch and p["body2"] in touch:
                (p1, r1), (p2, r2) = touch[p["body1"]], touch[p["body2"]]
                p["contact_points_apart_mm"] = 1e3 * float(np.linalg.norm(p1 - p2))
                p["link_radii_sum_mm"] = None if r1 is None or r2 is None else 1e3 * (r1 + r2)
        diffs = [abs(v) for ch in check for v in (ch["difference_mm"], ch["other_difference_mm"]) if v is not None]
        rec["follow"].append({"k": fr["k"], "verdict": fr["verdict"], "pairs": ps,
                              "pose_check": {"fingers": check,
                                             "max_abs_difference_mm": max(diffs) if diffs else None,
                                             "whole_hand_shift_mm": 1e3 * shift}})
    return rec


def _deep(ps, only_new=False):
    ps = [p for p in ps if not (only_new and p["at_rest_too"])]
    return max([p["depth_mm"] for p in ps] or [0.0])


def _names(ps):
    ps = sorted([p for p in ps if p["depth_mm"] > 1e3 * DEEP_M], key=lambda p: -p["depth_mm"])
    out = []
    for p in ps:
        s = f"{p['body1']} / {p['body2']} {p['depth_mm']:.1f}"
        if p["at_rest_too"]:
            s += " (rest too)"
        if p.get("contact_points_apart_mm") is not None and p.get("link_radii_sum_mm") is not None:
            s += f" (contact points {p['contact_points_apart_mm']:.1f} apart, radii sum {p['link_radii_sum_mm']:.1f})"
        out.append(s)
    return "; ".join(out) or "none"


def markdown(recs, commit):
    L = ["# Self-overlap of the hand at the stored self-hold poses", "",
         f"Written by `tools/hold_pose_self_overlap.py` at {commit}. Depths in mm. A pair is two of the hand's own "
         "bodies whose collision geoms penetrate in MuJoCo (the ball left out). MuJoCo drops contacts between a body "
         "and its parent, between pairs the model excludes, and for geoms with no collision bits, so the counts are "
         "a lower bound. These are facts about the poses of stored results, not new verdicts.", "",
         "## Pairs by pose", "",
         "| hand | stored verdict | colliding geoms | model's contact excludes | pairs at rest (deepest) | "
         "pairs at generator pose (deepest; not at rest) | pairs at follow pose, first frame (deepest; not at rest) | "
         "of those, finger against another finger (deepest) |",
         "|---|---|---|---|---|---|---|---|"]
    for r in recs:
        f0 = r["follow"][0]["pairs"]
        g = r["generator"]
        ff = [p for p in f0 if p["kind"] == "finger-finger"]
        gen = "not stored" if g is None else \
            f"{len(g)} ({_deep(g):.1f}; {sum(1 for p in g if not p['at_rest_too'])} new, {_deep(g, True):.1f})"
        L.append(f"| {r['hand']} | {', '.join(r['verdicts'])} | {r['model']['colliding_geoms']} | "
                 f"{r['model']['contact_excludes']} | {len(r['rest'])} ({_deep(r['rest']):.1f}) | {gen} | "
                 f"{len(f0)} ({_deep(f0):.1f}; {sum(1 for p in f0 if not p['at_rest_too'])} new, {_deep(f0, True):.1f}) | "
                 f"{len(ff)} ({_deep(ff):.1f}) |")
    L += ["", "## Is the follow pose the one the verdict was given at?", "",
          "Each finger's depth into the ball, recomputed at the pose set here with the check's own measure, against "
          "the stored value; and how far any finger link sits from where its own solve had it once the whole hand is "
          "posed together.", "",
          "| hand | frames | largest difference from the stored depths | largest link shift in the whole-hand pose |",
          "|---|---|---|---|"]
    for r in recs:
        md = [f["pose_check"]["max_abs_difference_mm"] for f in r["follow"]
              if f["pose_check"]["max_abs_difference_mm"] is not None]
        sh = max(f["pose_check"]["whole_hand_shift_mm"] for f in r["follow"])
        L.append(f"| {r['hand']} | {len(r['follow'])} | {('%.2g' % max(md)) if md else 'no depth stored'} | {sh:.2g} |")
    L += ["", f"## Pairs deeper than {1e3 * DEEP_M:g} mm, by name", ""]
    for r in recs:
        L.append(f"**{r['hand']}** ({', '.join(r['verdicts'])})")
        L.append("")
        L.append(f"- rest: {_names(r['rest'])}")
        if r["generator"] is not None:
            L.append(f"- generator pose: {_names(r['generator'])}")
        for f in r["follow"]:
            L.append(f"- follow pose, frame {f['k']}: {_names(f['pairs'])}")
        L.append("")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("hands", nargs="*", help="hand keys (default: every stored hold result)")
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "outputs"))
    ap.add_argument("--name", default="hold_pose_self_overlap")
    ap.add_argument("--overwrite", action="store_true")
    a = ap.parse_args()
    mj = fa._mj()
    paths = sorted(glob.glob(os.path.join(ROOT, "outputs", "follow_*__hold_*__self__fixed.json")))
    found = {os.path.basename(p)[len("follow_"):].split("__hold_")[0]: p for p in paths}
    keys = a.hands or sorted(found)
    outs = [os.path.join(a.out_dir, a.name + ext) for ext in (".json", ".md")]
    for p in outs:
        if os.path.exists(p) and not a.overwrite:
            raise SystemExit(f"{p} exists; pass --overwrite, --name or another --out-dir")
    recs, skipped = [], []
    for k in keys:
        try:
            recs.append(one(mj, k, found[k]))
            f0 = recs[-1]["follow"][0]
            print(f"{k}: rest {len(recs[-1]['rest'])} pairs, follow pose {len(f0['pairs'])} pairs "
                  f"(deepest {_deep(f0['pairs']):.2f} mm), pose check {f0['pose_check']['max_abs_difference_mm']}",
                  flush=True)
        except (FileNotFoundError, KeyError) as e:
            skipped.append({"hand": k, "why": f"{type(e).__name__}: {e}"})
            print(f"{k}: skipped, {type(e).__name__}: {e}", flush=True)
    commit = _git()
    os.makedirs(a.out_dir, exist_ok=True)
    with open(outs[0], "w", encoding="utf-8", newline="\n") as fh:
        json.dump({"schema": SCHEMA, "script": "tools/hold_pose_self_overlap.py", "commit": commit,
                   "deep_threshold_m": DEEP_M, "hands": recs, "skipped": skipped}, fh, indent=1)
    with open(outs[1], "w", encoding="utf-8", newline="\n") as fh:
        fh.write(markdown(recs, commit))
    print("wrote", *outs)


if __name__ == "__main__":
    main()
