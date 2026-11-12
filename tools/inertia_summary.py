"""Compare a follow result made without the inertia term against one made
with it (follow(inertia=True), issue 13: the hand's own weight and its
motion torque added to the joint torque tested): the same hand, leader,
placement and checked frames. Prints counts and quantiles only; writes
nothing.

    python tools/inertia_summary.py <without.json> <with_inertia.json>

What it reports: how the weight was taken (settings["inertia_weight"]: the
gravity read from the leader, its source, the check against the leader's
own forces, the direction in the hand's frame, or why it was left out);
the frame verdicts of both runs and the frames whose verdict changed; the
finger states that changed; how many finger records carry "inertia", how
many of those fitted a motion and how many carry the weight; the weight,
motion and total shares of the torque limit (each the largest over the
finger's inputs) as quantiles; the largest weight torque and where; and
how many fingers miss "torque" in each run.
"""
import collections
import json
import sys


def _share(v):
    """A record's share: the largest over the finger's inputs (shares are
    stored per input), or None."""
    if v is None:
        return None
    return max(abs(x) for x in v) if isinstance(v, list) else abs(v)


def _q(xs, name):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        print(f"{name}: none")
        return
    print(f"{name}: n={len(xs)} max={xs[-1]:.4f} median={xs[len(xs) // 2]:.4f} "
          f"p90={xs[int(0.9 * (len(xs) - 1))]:.4f}")


def main(a, b):
    A = json.load(open(a, encoding="utf-8"))
    B = json.load(open(b, encoding="utf-8"))
    w = B["settings"].get("inertia_weight")
    print("settings.inertia_weight:", json.dumps(w, indent=1))
    if len(A["frames"]) != len(B["frames"]):
        sys.exit(f"the runs checked different frames: {len(A['frames'])} and {len(B['frames'])}")
    vA = collections.Counter(f["verdict"] for f in A["frames"])
    vB = collections.Counter(f["verdict"] for f in B["frames"])
    print("verdicts without inertia:", dict(vA))
    print("verdicts with inertia   :", dict(vB))
    changed = [(fa["k"], fa["verdict"], fb["verdict"]) for fa, fb in zip(A["frames"], B["frames"])
               if fa["verdict"] != fb["verdict"]]
    print("frames whose verdict changed:", len(changed), changed[:10])
    st_changed = sum(xa["state"] != xb["state"] for fa, fb in zip(A["frames"], B["frames"])
                     for xa, xb in zip(fa["fingers"], fb["fingers"]))
    print("finger states changed:", st_changed)
    rows = [(f["k"], x["finger"], x["inertia"]) for f in B["frames"] for x in f["fingers"]
            if "inertia" in x]
    print("finger records with inertia:", len(rows),
          "| motion fitted:", sum(r["fitted"] for _, _, r in rows),
          "| weight counted:", sum("weight_Nm" in r for _, _, r in rows))
    _q([_share(r.get("weight_share")) for _, _, r in rows], "weight_share (weight torque / limit)")
    _q([_share(r.get("motion_share")) for _, _, r in rows], "motion_share (motion torque / limit)")
    _q([_share(r.get("total_share")) for _, _, r in rows], "total_share (weight + motion / limit)")
    wt = [(max(abs(v) for v in r["weight_Nm"]), fi, k) for k, fi, r in rows if "weight_Nm" in r]
    if wt:
        print("largest weight torque: %.4f N m, %s, frame k=%d" % max(wt))
    mo = [(max(abs(v) for v in r["motion_Nm"]), fi, k) for k, fi, r in rows if "motion_Nm" in r]
    if mo:
        print("largest motion torque: %.5f N m, %s, frame k=%d" % max(mo))
    print("'torque' in missed: with inertia",
          sum("torque" in (x.get("missed") or []) for f in B["frames"] for x in f["fingers"]),
          "| without",
          sum("torque" in (x.get("missed") or []) for f in A["frames"] for x in f["fingers"]))
    print("elapsed_s:", A.get("elapsed_s"), B.get("elapsed_s"))


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
