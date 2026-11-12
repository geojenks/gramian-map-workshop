"""Compare a follow result made with continuity alone against one made with
torque at speed as well (follow(continuity=True, torque_speed=True), issue
12): the same hand, leader, placement and checked frames. Prints counts and
quantiles only; writes nothing.

    python tools/torque_speed_summary.py <continuity.json> <torque_speed.json>

What it reports: the frame verdicts of both runs and the frames whose
verdict changed; the finger states that changed; how many finger records
carry "torque_speed" and how many were derated at all (a numerically tiny
speed still counts, so the share quantiles are the honest measure); the
speed share |rate| / no-load speed over every derated joint, with the
largest derating (which joint, in which frame, its limit at rest and at
speed); how many fingers and frames failed only because of the speed; and
whether the speed bound ever overrode the actuator link's reachable range.
"""
import collections
import json
import sys


def main(a, b):
    A = json.load(open(a, encoding="utf-8"))
    B = json.load(open(b, encoding="utf-8"))
    print("settings.torque_speed:", json.dumps(B["settings"]["torque_speed"], indent=1))
    if len(A["frames"]) != len(B["frames"]):
        sys.exit(f"the runs checked different frames: {len(A['frames'])} and {len(B['frames'])}")
    vA = collections.Counter(f["verdict"] for f in A["frames"])
    vB = collections.Counter(f["verdict"] for f in B["frames"])
    print("verdicts continuity  :", dict(vA))
    print("verdicts torque-speed:", dict(vB))
    changed = [(fa["k"], fa["verdict"], fb["verdict"]) for fa, fb in zip(A["frames"], B["frames"])
               if fa["verdict"] != fb["verdict"]]
    print("frames whose verdict changed:", len(changed), changed[:10])
    st_changed = sum(xa["state"] != xb["state"] for fa, fb in zip(A["frames"], B["frames"])
                     for xa, xb in zip(fa["fingers"], fb["fingers"]))
    print("finger states changed:", st_changed)
    rows = [(f["k"], x["finger"], x["torque_speed"]) for f in B["frames"] for x in f["fingers"]
            if "torque_speed" in x]
    print("finger records with torque_speed:", len(rows),
          "| derated at all (any nonzero speed):", sum(r["derated"] for _, _, r in rows))
    shares = sorted(j["speed_share"] for _, _, r in rows for j in r["joints"]
                    if j["speed_share"] is not None)
    if shares:
        print(f"speed_share (|rate| / no-load): n={len(shares)} max={shares[-1]:.4f} "
              f"median={shares[len(shares) // 2]:.4f} p90={shares[int(0.9 * (len(shares) - 1))]:.4f}")
        worst = max((j["speed_share"], j["joint"], j["limit_rest"], j["limit_at_speed"], k)
                    for k, _, r in rows for j in r["joints"] if j["speed_share"] is not None)
        print("largest derating: share=%.4f joint=%s rest=%s at_speed=%s frame k=%d" % worst)
    print("fingers failing only at speed:",
          sum(r.get("fails_only_at_speed") is True for _, _, r in rows),
          "| 'torque at speed' in missed:",
          sum("torque at speed" in (x.get("missed") or []) for f in B["frames"] for x in f["fingers"]),
          "| 'torque' in missed (torque-speed run):",
          sum("torque" in (x.get("missed") or []) for f in B["frames"] for x in f["fingers"]),
          "(continuity run):",
          sum("torque" in (x.get("missed") or []) for f in A["frames"] for x in f["fingers"]))
    fr = [f["torque_speed"] for f in B["frames"] if "torque_speed" in f]
    print("frames with derated fingers:", sum(bool(t["derated_fingers"]) for t in fr),
          "| frame fails_only_at_speed True:", sum(t["fails_only_at_speed"] is True for t in fr),
          "False:", sum(t["fails_only_at_speed"] is False for t in fr))
    print("link overridden by the speed bound anywhere:",
          sum(bool(r["link_overridden"]) for _, _, r in rows))
    print("elapsed_s:", A.get("elapsed_s"), B.get("elapsed_s"))


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
