"""Run the follow-the-leader check for each rigid hand on the leader that
matches its finger count, and save outputs/follow_<hand>__<leader>.json.

    python tools/run_follow.py [--hands allegro shadow leap l20] [--frames 40] [--respace]

--respace lets the stations move round the ring to suit each hand (outputs
get a __respaced suffix).

The allowed overlap between hand and object is the leader's own
(allowed_overlap_m, other_contact; docs/TASK_FILE_FORMAT.md), or the
default when the leader states none. --allowed-overlap-m, --other-overlap-m
and --no-other-contact override it. --out-dir writes somewhere other than
outputs/.

The placement is searched for afresh, unless:
  --reuse-placement   an earlier result already holds the placement this
                      run's search would find: same hand model, same leader
                      file, same search options and allowances, same search
                      code (its "placement_key"; follow.placement_key). It
                      is looked for in --placement-from (a result file or a
                      folder of them), else in --out-dir. Its placement and
                      assignment are then re-scored, which takes seconds;
                      with no match, the search runs as usual
  --placement-from    alone: that result's placement and assignment are
                      re-scored whatever its key (one hand only); a key
                      that differs from this run's is reported

--scale-margin and --disturbance-margin give each frame its scale margin
and disturbance margin (follow(margin=, disturbance=); neither changes the
verdict, and both make a run slower); --margins turns both on. The margins
are printed per hand, smallest over the frames.

After each hand it prints which fingers are amber or red and when, and the
joints posed past a limit; tools/report_failures.py draws the same from the
saved result as a page.

Needs mujoco and the hand models (tools/fetch_hands.py).
"""
import argparse
import collections
import glob
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.follow import (follow, overlap_allowance, placement_key,  # noqa: E402
                                     same_placement_key)
from gramian_workshop import failures  # noqa: E402
from gramian_workshop.hands import HANDS  # noqa: E402

LEADERS = {"big": "w10_cup_physical_x0.3_a0.15_sl0.3t_n{n}",
           "tennis": "w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n{n}"}


def stored_placement(where, hk, key):
    """(path, result) of the first result file in `where` (a file or a
    folder of follow_<hand>__*.json) whose placement key is `key`, or
    (None, None)."""
    paths = [where] if os.path.isfile(where) else         sorted(glob.glob(os.path.join(where, f"follow_{hk}__*.json")))
    for p in paths:
        try:
            with open(p, encoding="utf-8") as f:
                r = json.load(f)
        except (OSError, ValueError):
            continue
        if r.get("hand") == hk and same_placement_key(r.get("placement_key"), key):
            return p, r
    return None, None


def smallest_margin(frames, key):
    """The smallest `key` margin over the frames that carry one, as
    (margin, capped, frame k, direction), or None if no frame has a number
    (the key is absent, or every frame's is None: no contact or no torque
    checked). A capped frame's margin is a lower bound."""
    got = [(fr[key]["margin"], bool(fr[key].get("capped")), fr["k"], fr[key].get("direction"))
           for fr in frames if isinstance(fr.get(key), dict)]
    return min(got, key=lambda g: g[0]) if got else None


def margin_lines(r):
    """One printed line per margin the result carries."""
    out = []
    for key, name in (("scale_margin", "scale margin"),
                      ("disturbance_margin", "disturbance margin")):
        if not any(key in fr for fr in r["frames"]):
            continue
        sm = smallest_margin(r["frames"], key)
        if sm is None:
            out.append(f"{name}: none (no contact or no torque checked)")
            continue
        m, capped, k, d = sm
        out.append(f"{name}: smallest {'at least ' if capped else ''}{m:.4g} "
                   f"(frame {k}" + (f", worst direction {d}" if d else "") + ")")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hands", nargs="*", default=list(HANDS))
    ap.add_argument("--leader", choices=list(LEADERS), default="tennis")
    ap.add_argument("--frames", type=int, default=40,
                    help="leader frames evaluated, spread over the recording")
    ap.add_argument("--tag", default="", help="suffix for the output file names")
    ap.add_argument("--leader-id", default=None,
                    help="a leader file in data/leaders/ by id, for every hand (e.g. a human demo)")
    ap.add_argument("--respace", action="store_true",
                    help="move stations round the ring to suit the hand")
    ap.add_argument("--allowed-overlap-m", type=float, default=None,
                    help="override the leader's allowed overlap at a contact (m)")
    ap.add_argument("--other-overlap-m", type=float, default=None,
                    help="override: other contact allowed, to this depth (m)")
    ap.add_argument("--no-other-contact", action="store_true",
                    help="override: no contact away from the stated points")
    ap.add_argument("--placement-from", default=None,
                    help="an earlier result file: re-score its placement and assignment "
                         "instead of searching (one hand only); with --reuse-placement, "
                         "a result file or folder to look in")
    ap.add_argument("--reuse-placement", action="store_true",
                    help="reuse a stored placement whose placement_key matches this run's "
                         "(looked for in --placement-from, else --out-dir); search if none")
    ap.add_argument("--scale-margin", action="store_true",
                    help="give each frame its scale margin (follow(margin=True))")
    ap.add_argument("--disturbance-margin", action="store_true",
                    help="give each frame its disturbance margin (follow(disturbance=True))")
    ap.add_argument("--margins", action="store_true",
                    help="both margins: --scale-margin and --disturbance-margin")
    ap.add_argument("--continuity", action="store_true",
                    help="carry each finger's pose from frame to frame and report its jump "
                         "and joint speed (follow(continuity=True)); the file name gets "
                         "__continuity")
    ap.add_argument("--speed-limit", type=float, default=None,
                    help="with --continuity: a hinge faster than this (rad/s) fails its finger "
                         "(follow(speed_limit=...)); none is sourced for any hand")
    ap.add_argument("--torque-speed", action="store_true",
                    help="with --continuity: derate each joint's torque limit by its fitted "
                         "speed, from the stall limit at rest to 0 at the hand's published "
                         "no-load speed (follow(torque_speed=True)); hands with no published "
                         "speed are unchanged; the file name gets __torque_speed")
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "outputs"),
                    help="where to write the results (default outputs/)")
    a = ap.parse_args()
    if a.no_other_contact and a.other_overlap_m is not None:
        ap.error("--no-other-contact and --other-overlap-m contradict each other")
    other = False if a.no_other_contact else a.other_overlap_m
    if a.speed_limit is not None and not a.continuity:
        ap.error("--speed-limit goes with --continuity")
    if a.torque_speed and not a.continuity:
        ap.error("--torque-speed goes with --continuity (the speed it derates by is the "
                 "continuity readout's)")
    want_scale = a.scale_margin or a.margins
    want_dist = a.disturbance_margin or a.margins
    fixed = None
    if a.placement_from and not a.reuse_placement:
        if len(a.hands) != 1:
            ap.error("--placement-from goes with one hand")
        with open(a.placement_from, encoding="utf-8") as f:
            fixed = json.load(f)
    os.makedirs(a.out_dir, exist_ok=True)
    for hk in a.hands:
        n = len(HANDS[hk]["fingers"])
        lid = a.leader_id or LEADERS[a.leader].format(n=n)
        with open(os.path.join(ROOT, "data", "leaders", lid + ".json")) as f:
            L = json.load(f)
        placement, src = fixed, a.placement_from if fixed else None
        if fixed is not None or a.reuse_placement:
            # the key this run's own search would carry
            key = placement_key(hk, L, respace=a.respace,
                                allow=overlap_allowance(L, a.allowed_overlap_m, other))
        if fixed is not None and not same_placement_key(fixed.get("placement_key"), key):
            print(f"{hk}: note: {a.placement_from} "
                  + ("has no placement key" if not fixed.get("placement_key") else
                     "was placed by a different search, hand, leader or options")
                  + "; re-scoring its placement anyway", flush=True)
        if a.reuse_placement:
            src, placement = stored_placement(a.placement_from or a.out_dir, hk, key)
            print(f"{hk}: " + (f"reusing the placement in {src}" if src else
                               "no stored placement matches; searching"), flush=True)
        t0 = time.time()
        r = follow(hk, L, n_eval=a.frames, respace=a.respace, placement=placement,
                   allowed_overlap_m=a.allowed_overlap_m, other_contact=other,
                   margin=want_scale, disturbance=want_dist,
                   continuity=a.continuity, speed_limit=a.speed_limit,
                   torque_speed=a.torque_speed)
        r["elapsed_s"] = round(time.time() - t0, 1)
        if src:
            r["placement_reused_from"] = os.path.relpath(src, ROOT)                 if os.path.abspath(src).startswith(ROOT) else os.path.abspath(src)
        out = os.path.join(a.out_dir, f"follow_{hk}__{lid}" + ("__respaced" if a.respace else "")
                           + ("__continuity" if a.continuity else "")
                           + ("__torque_speed" if a.torque_speed else "")
                           + (f"__{a.tag}" if a.tag else "") + ".json")
        with open(out, "w", encoding="utf-8") as f:
            json.dump(r, f, indent=1, default=float)
        v = collections.Counter(fr["verdict"] for fr in r["frames"])
        why = collections.Counter(fr["reason"] for fr in r["frames"] if fr["reason"])
        fin = collections.Counter((x["finger"], x["state"]) for fr in r["frames"]
                                  for x in fr["fingers"])
        print(f"{hk} on {lid}: {dict(v)}; red because {dict(why)}; "
              f"assignment {r['assignment']}; placement cost "
              f"{r['placement_cost_mm']:.2f} mm; respaced by {[round(x) for x in r['respace_deg']]} deg; "
              f"{r['elapsed_s']} s", flush=True)
        print("   per finger:", dict(sorted(fin.items())), flush=True)
        ov = [max(x["overlap_mm"] or 0.0, x["other_overlap_mm"] or 0.0)
              for fr in r["frames"] for x in fr["fingers"] if "overlap_mm" in x]
        st = r["settings"]
        if ov:
            print(f"   overlap (deepest part of each posed finger, per frame): median "
                  f"{sorted(ov)[len(ov) // 2]:.2f} mm, max {max(ov):.2f} mm; allowed "
                  f"{st['allowed_overlap_m'] * 1e3:g} mm at the contact, other contact "
                  f"{st['other_contact']}", flush=True)
        for ln in margin_lines(r):
            print("   " + ln, flush=True)
        # which fingers and joints fail, and when (tools/report_failures.py
        # draws the same as a page)
        # the result is already saved; a report that cannot read it must
        # not stop the hands still to run
        try:
            for ln in failures.brief(failures.summarise(r)).splitlines()[1:]:
                print("  " + ln, flush=True)
        except Exception as e:
            print(f"  (no failure report: {type(e).__name__}: {e})", flush=True)


if __name__ == "__main__":
    main()
