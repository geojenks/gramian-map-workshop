"""Re-score the side-contact, respaced placements (outputs/follow_<hand>__<leader>
__respaced__side.json) with the current solver and a joint-stiffness sweep,
without searching again.

    python tools/rescore_side.py allegro shadow l20 [--kq 0 0.05 0.2]

Writes outputs/follow_<hand>__<leader>__respaced__side_kq<k>.json and prints
green / amber / red per hand and stiffness.
"""
import argparse
import collections
import glob
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.follow import follow  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("hands", nargs="+")
    ap.add_argument("--kq", nargs="+", type=float, default=[0.0, 0.05, 0.2])
    a = ap.parse_args()
    for hk in a.hands:
        src_p = glob.glob(os.path.join(ROOT, "outputs", f"follow_{hk}__*__respaced__side.json"))[0]
        src = json.load(open(src_p))
        L = json.load(open(os.path.join(ROOT, "data", "leaders", src["leader"] + ".json")))
        for kq in a.kq:
            r = follow(hk, L, n_eval=40, placement=src, joint_stiffness=kq)
            r["rescored_from"] = os.path.relpath(src_p, ROOT)
            out = src_p.replace("__side.json", f"__side_kq{kq:g}.json")
            json.dump(r, open(out, "w", encoding="utf-8"), indent=1, default=float)
            v = collections.Counter(f["verdict"] for f in r["frames"])
            why = collections.Counter(f["reason"] for f in r["frames"] if f["verdict"] == "red")
            fin = collections.Counter((x["finger"], x["state"]) for f in r["frames"]
                                      for x in f["fingers"] if x["state"] not in ("off", "green"))
            print(f"{hk} kq {kq:g}: green {v.get('green', 0)} amber {v.get('amber', 0)} red {v.get('red', 0)}"
                  f"; red because {dict(why)}; non-green fingers {dict(fin)}", flush=True)


if __name__ == "__main__":
    main()
