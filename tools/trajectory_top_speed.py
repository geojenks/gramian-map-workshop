"""How fast a hand can do a stored motion: the speed search of the
whole-motion check (follow(trajectory=True, top_speed=True)) on a stored
placement, a retimed copy of the leader at the speed found, and the
whole-motion check again on that copy.

    python tools/trajectory_top_speed.py outputs/follow_<hand>__<leader>.json \
        [--rate 3] [--lag 0.1] [--inertia] [--n-eval N] [--no-retime]

The stored follow result gives the hand, the leader and the placement,
which is reused as it is (follow(placement=)); only the time between
frames changes with speed. Speed is a multiple of the recorded speed: 1 as
recorded, 0.5 half as fast. Per run of frames that hold alone, the search
finds the largest speed at which the whole run holds under the rate limit
and lag (illustrative values, unsourced: speeds are comparative, not
predictions); the motion's speed is its slowest run's. Frames that hold at
no speed are listed with the reason.

Writes, never over a stored leader:
  outputs/top_speed/topspeed__<result>__<setting>[__inertia].json
      the search's readout (trajectory.top_speed), the recorded-speed
      check beside it, and the confirmation below.
  outputs/retimed_leaders/<leader>__x<speed>[__inertia].json
      the leader with every frame time divided by the speed found, so the
      same motion plays at that speed (rate_hz_median scaled; id, name,
      notes and retimed_from say so). Only when the speed found is finite
      and below 1 (a hand that holds at the recorded speed needs no
      retiming; --retime-always writes the copy at any finite speed).
Then follow(trajectory=True) runs on the retimed copy with the same
placement and settings, and the readout says whether every run holds
there. Without --inertia that holds by construction, up to the search's
tolerance. With --inertia the finger motion torque is refitted from the
retimed times, over a window of FIT_HALF_S / speed so it spans the same
recorded frames as the search's fit (a fixed window would span fewer
frames as the motion slows and, below follow's FIT_MIN_POINTS, drop the
motion torque, passing for nothing); the readout gives both runs' counts
of fitted fingers, which should agree.

Frames checked: every frame unless --n-eval spreads N over the recording
(the dense sweep, tools/trajectory_sweep.py, checks every frame of the
cup-wave leaders and 300 of the tennis demonstration). The link rows act
between checked frames, so a sparse check loosens them and raises the
speed found: compare speeds only at the same density.
"""
import argparse
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
from _provenance import git_commit, sha256  # noqa: E402

OUT = os.path.join(ROOT, "outputs", "top_speed")
RETIMED = os.path.join(ROOT, "outputs", "retimed_leaders")


def setting_label(rate, lag):
    return "__".join(([f"rate{rate:g}"] if rate is not None else []) +
                     ([f"lag{lag:g}"] if lag is not None else [])) or "unlinked"


def run_follow(F, old, L, rate, lag, inertia, n_eval, top_speed):
    s = old["settings"]
    return json.loads(json.dumps(F.follow(
        old["hand"], L, middle=s["middle"], ik_iters=s.get("ik_iters"), wrist=bool(s.get("wrist")),
        respace=bool(s.get("respace")), joint_stiffness=s.get("joint_stiffness_Nm_per_rad"),
        placement=old, n_eval=n_eval, margin=False, disturbance=False, inertia=inertia,
        rate_limit=rate, lag_s=lag, trajectory=True, top_speed=top_speed)))


def fitted(r):
    """Finger-frames whose motion torque was fitted (follow(inertia=True))."""
    return sum(1 for fr in r["frames"] for fg in fr.get("fingers", [])
               if (fg.get("inertia") or {}).get("fitted"))


def retimed_leader(L, speed, src_path, hand, inertia, setting):
    """A copy of leader L with every frame's t divided by speed."""
    R = json.loads(json.dumps(L))
    for fr in R["frames"]:
        fr["t"] = float(fr["t"]) / speed
    if R.get("rate_hz_median") is not None:
        R["rate_hz_median"] = float(R["rate_hz_median"]) * speed
    tag = f"x{speed:.4g}" + ("__inertia" if inertia else "")
    R["id"] = L.get("id", os.path.splitext(os.path.basename(src_path))[0]) + "__" + tag
    R["name"] = L.get("name", R["id"]) + f", retimed to {speed:.4g} x the recorded speed"
    R["retimed_from"] = {"path": os.path.relpath(src_path, ROOT).replace(os.sep, "/"),
                         "sha256": sha256(src_path), "speed": speed, "hand": hand,
                         "setting": setting, "inertia": inertia,
                         "tool": "tools/trajectory_top_speed.py", "workshop_rev": git_commit(ROOT)}
    R.setdefault("notes", []).append(
        f"retimed: every frame's t divided by {speed:.4g} (the top speed tools/trajectory_top_speed.py "
        f"found for the {hand} hand at {setting}); poses, contacts and forces unchanged, "
        "so the object's own inertia is not re-derived")
    return R


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("result", help="a stored outputs/follow_*.json (hand, leader, placement)")
    ap.add_argument("--rate", type=float, default=None, help="rate limit, per second")
    ap.add_argument("--lag", type=float, default=None, help="lag time constant, s")
    ap.add_argument("--inertia", action="store_true", help="count the fingers' motion torque")
    ap.add_argument("--n-eval", type=int, default=None,
                    help="frames checked, spread over the recording (default: every frame)")
    ap.add_argument("--no-retime", action="store_true", help="search only; write no leader")
    ap.add_argument("--retime-always", action="store_true",
                    help="write the retimed copy at any finite speed, not only below 1")
    a = ap.parse_args()
    if a.rate is None and a.lag is None:
        sys.exit("give --rate and/or --lag: without a link between frames every speed holds")

    from gramian_workshop import follow as F
    old = json.load(open(a.result, encoding="utf-8"))
    src = os.path.join(ROOT, "data", "leaders", old["leader"] + ".json")
    L = json.load(open(src, encoding="utf-8"))
    n_eval = a.n_eval
    setting = setting_label(a.rate, a.lag)
    base = os.path.splitext(os.path.basename(a.result))[0]
    name = f"topspeed__{base}__{setting}" + ("__inertia" if a.inertia else "")
    os.makedirs(OUT, exist_ok=True)
    out_path = os.path.join(OUT, name + ".json")

    t0 = time.time()
    r = run_follow(F, old, L, a.rate, a.lag, a.inertia, n_eval, top_speed=True)
    ts = r["trajectory"]["top_speed"]
    res = {"result": os.path.relpath(a.result, ROOT).replace(os.sep, "/"), "hand": old["hand"],
           "leader": old["leader"], "leader_sha256": sha256(src), "setting": setting,
           "rate_limit_per_s": a.rate, "lag_s": a.lag, "inertia": a.inertia, "n_eval": n_eval,
           "frame_k": [fr["k"] for fr in r["frames"]],
           "frame_t": [float(L["frames"][fr["k"]]["t"]) for fr in r["frames"]],
           "trajectory": r["trajectory"], "search_wall_s": time.time() - t0,
           "workshop_rev": git_commit(ROOT)}
    chk = {k: v for k, v in r["trajectory"].items() if k != "top_speed"}
    held = [ru["holds"] for ru in chk["runs"]]
    print(f"{old['hand']} on {old['leader']} at {setting}" + (" with inertia" if a.inertia else "")
          + f": {len(r['frames'])} frames checked")
    print(f"  at the recorded speed: {sum(held)} of {len(held)} runs hold")
    for ru in ts["runs"]:
        sp = ru["speed"]
        print(f"  run k {ru['k_first']}..{ru['k_last']} ({ru['n_frames']} frames): speed "
              + ("no torque check" if sp is None else
                 f"{sp:.4g}" + (" (capped)" if ru["capped"] else "")
                 + ("" if ru.get("contiguous") in (True, None) else " (not contiguous)")))
    for u in ts["unholdable"]:
        print(f"  frame k {u['k']} (t {u['t']:.3f} s) holds at no speed: {u['reason']}")
    sp = ts["speed"]
    print("  top speed: " + ("none (no run checks torque)" if sp is None else
                             f"{sp:.4g} x recorded" + (" (capped)" if ts["capped"] else ""))
          + ("" if ts["holds_everywhere"] else
             f"; {len(ts['unholdable'])} frames hold at no speed, so this is the speed "
             "of the stretches between them"))

    write = (not a.no_retime and sp is not None and sp > 0 and not ts["capped"]
             and (sp < 1 or a.retime_always))
    if write:
        os.makedirs(RETIMED, exist_ok=True)
        R = retimed_leader(L, sp, src, old["hand"], a.inertia, setting)
        rp = os.path.join(RETIMED, R["id"] + ".json")
        json.dump(R, open(rp, "w", encoding="utf-8"), indent=1)
        print(f"  retimed leader: {os.path.relpath(rp, ROOT)} (t / {sp:.4g}, "
              f"{R['frames'][-1]['t'] - R['frames'][0]['t']:.2f} s long, "
              f"was {L['frames'][-1]['t'] - L['frames'][0]['t']:.2f} s)")
        t1 = time.time()
        fit_half = F.FIT_HALF_S
        F.FIT_HALF_S = fit_half / sp      # the same recorded frames in the fit window
        try:
            r2 = run_follow(F, old, R, a.rate, a.lag, a.inertia, n_eval, top_speed=False)
        finally:
            F.FIT_HALF_S = fit_half
        held2 = [ru["holds"] for ru in r2["trajectory"]["runs"]]
        res["retimed"] = {"leader": os.path.relpath(rp, ROOT).replace(os.sep, "/"),
                          "sha256": sha256(rp), "speed": sp,
                          "fit_half_s": fit_half / sp if a.inertia else None,
                          "n_fitted": fitted(r2), "n_fitted_search": fitted(r),
                          "trajectory": r2["trajectory"], "wall_s": time.time() - t1}
        print(f"  check on the retimed leader: {sum(held2)} of {len(held2)} runs hold"
              + ("" if all(held2) else "  <- not all; see the readout")
              + (f"; motion torque fitted for {fitted(r2)} finger-frames (search: {fitted(r)})"
                 if a.inertia else ""))
    elif not a.no_retime:
        print("  no retimed leader written: " +
              ("no speed" if sp is None else "speed 0" if sp <= 0 else
               "search capped" if ts["capped"] else
               "holds at the recorded speed (--retime-always to write the copy)"))
    json.dump(res, open(out_path, "w", encoding="utf-8"), indent=1)
    print(f"  wrote {os.path.relpath(out_path, ROOT)}  ({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    main()
