"""Run the follow-the-leader check for each rigid hand on the leader that
matches its finger count, and save outputs/follow_<hand>__<leader>.json.

    python tools/run_follow.py [--hands allegro shadow leap l20] [--frames 40] [--respace]

--respace lets the stations move round the ring to suit each hand (outputs
get a __respaced suffix).

The allowed overlap between hand and object is the leader's own
(allowed_overlap_m, other_contact; docs/TASK_FILE_FORMAT.md), or the
default when the leader states none. --allowed-overlap-m, --other-overlap-m
and --no-other-contact override it. The placement is searched for afresh
unless --placement-from names an earlier result whose placement and
assignment are re-scored. --out-dir writes somewhere other than outputs/.

Needs mujoco and the hand models (tools/fetch_hands.py).
"""
import argparse
import collections
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.follow import follow  # noqa: E402
from gramian_workshop.hands import HANDS  # noqa: E402

LEADERS = {"big": "w10_cup_physical_x0.3_a0.15_sl0.3t_n{n}",
           "tennis": "w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n{n}"}


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
                         "instead of searching (one hand only)")
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "outputs"),
                    help="where to write the results (default outputs/)")
    a = ap.parse_args()
    if a.no_other_contact and a.other_overlap_m is not None:
        ap.error("--no-other-contact and --other-overlap-m contradict each other")
    other = False if a.no_other_contact else a.other_overlap_m
    placement = None
    if a.placement_from:
        if len(a.hands) != 1:
            ap.error("--placement-from goes with one hand")
        with open(a.placement_from, encoding="utf-8") as f:
            placement = json.load(f)
    os.makedirs(a.out_dir, exist_ok=True)
    for hk in a.hands:
        n = len(HANDS[hk]["fingers"])
        lid = a.leader_id or LEADERS[a.leader].format(n=n)
        with open(os.path.join(ROOT, "data", "leaders", lid + ".json")) as f:
            L = json.load(f)
        t0 = time.time()
        r = follow(hk, L, n_eval=a.frames, respace=a.respace, placement=placement,
                   allowed_overlap_m=a.allowed_overlap_m, other_contact=other)
        r["elapsed_s"] = round(time.time() - t0, 1)
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
