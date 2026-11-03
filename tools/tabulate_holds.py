"""Tabulate the self-hold checks: one row per hand, from the stored leaders
and results.

    python tools/tabulate_holds.py [--leaders data/leaders] [--results outputs]
                                   [--out outputs/hold_summary.md]

Reads every outputs/follow_<hand>__hold_<hand>__self__fixed.json
(tools/make_hold_leader.py --run) with its leader
data/leaders/hold_<hand>__self.json, and prints a Markdown table: the
generator's fit (curl per finger, worst contact miss, worst facing, palm
penetration, other-link overlap, the ball's mass) and the check's verdict
(reasons, finger states, worst reach, side and penetration over every
finger and frame, whether every finger's torque was checked, the scale
margin per frame, run time). Every number is read from those files.
"""
import argparse
import glob
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

HDR = ("hand", "fingers", "curl", "fit miss mm", "fit face deg", "fit palm mm", "fit other mm",
       "ball g", "verdict", "reason", "finger states", "reach mm", "side deg", "pen mm",
       "torque", "margin", "s")


def rows(leaders_dir, results_dir):
    out = []
    for p in sorted(glob.glob(os.path.join(results_dir, "follow_*__hold_*__self__fixed.json"))):
        with open(p, encoding="utf-8") as fh:
            r = json.load(fh)
        h = r["hand"]
        with open(os.path.join(leaders_dir, f"hold_{h}__self.json"), encoding="utf-8") as fh:
            L = json.load(fh)
        sf = L["source"]["sphere_fit"]
        frs = r["frames"]
        verd = ",".join(sorted({f["verdict"] for f in frs}))
        why = ",".join(sorted({f["reason"] for f in frs if f["reason"]})) or "-"
        fing = [x for f in frs for x in f["fingers"] if x["state"] != "off"]
        marg = [f.get("scale_margin") or {} for f in frs]
        mv = ",".join(f"{m['margin']:.3g}" if m.get("margin") is not None else "none" for m in marg)
        out.append((h, len(L["stations"]),
                    ",".join(f"{v:.2f}" for v in L["source"]["curl_fraction"].values()),
                    1e3 * sf["miss_max_m"], sf["facing_max_deg"],
                    1e3 * sf["palm_penetration_m"], 1e3 * sf["other_link_overlap_m"],
                    1e3 * L["object"]["mass_kg"], verd, why,
                    ",".join(sorted({x["state"] for x in fing})),
                    max(x["reach_mm"] for x in fing), max(x["side_deg"] for x in fing),
                    max(x["penetration_mm"] for x in fing),
                    "yes" if all(x.get("torque_checked") for x in fing) else "no",
                    mv, r["elapsed_s"]))
    return out


def table(rs):
    lines = ["| " + " | ".join(HDR) + " |", "|" + "---|" * len(HDR)]
    for r in rs:
        lines.append("| " + " | ".join(f"{v:.2f}" if isinstance(v, float) else str(v) for v in r) + " |")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--leaders", default=os.path.join(ROOT, "data", "leaders"))
    ap.add_argument("--results", default=os.path.join(ROOT, "outputs"))
    ap.add_argument("--out", default=None, help="also write the table here (Markdown)")
    a = ap.parse_args()
    t = table(rows(a.leaders, a.results))
    print(t)
    if a.out:
        with open(a.out, "w", encoding="utf-8", newline="\n") as fh:
            fh.write("# Self-hold checks, one row per hand\n\n"
                     "Written by `tools/tabulate_holds.py` from `data/leaders/hold_*__self.json` "
                     "and `outputs/follow_*__hold_*__self__fixed.json`.\n\n" + t + "\n")
        print(f"wrote {os.path.relpath(a.out, ROOT)}")


if __name__ == "__main__":
    main()
