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
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "outputs"),
                    help="where to write the results (default outputs/)")
    a = ap.parse_args()
    if a.no_other_contact and a.other_overlap_m is not None:
        ap.error("--no-other-contact and --other-overlap-m contradict each other")
    other = False if a.no_other_contact else a.other_overlap_m
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
                   allowed_overlap_m=a.allowed_overlap_m, other_contact=other)
        r["elapsed_s"] = round(time.time() - t0, 1)
        if src:
            r["placement_reused_from"] = os.path.relpath(src, ROOT)                 if os.path.abspath(src).startswith(ROOT) else os.path.abspath(src)
        out = os.path.join(a.out_dir, f"follow_{hk}__{lid}" + ("__respaced" if a.respace else "") + (f"__{a.tag}" if a.tag else "") + ".json")
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


if __name__ == "__main__":
    main()
