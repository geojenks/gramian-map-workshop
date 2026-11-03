"""Bracket a hand's self-hold on the passive joint stiffness.

    python tools/hold_stiffness_bracket.py shadow [--kq 0 0.001 ...] [--frames 2]
    python tools/hold_stiffness_bracket.py shadow --table-only

Runs follow() on data/leaders/hold_<hand>__self.json (tools/make_hold_leader.py)
with the stored fixed placement (outputs/hold_placement_<hand>.json) and
scale margins, once per joint stiffness kq (N.m/rad, the same on every free
joint; follow(joint_stiffness=)), and writes

    outputs/hold_<hand>__kq_bracket.json   every run's frames
    outputs/hold_<hand>__kq_bracket.md     one row per kq (from the JSON)

kq = 0 is the pessimistic bound: the model's own springs only (the Shadow's
coupled J1/J2 pair has none, so the pair is a free internal degree of
freedom and the pad load has nothing to balance it). Larger kq stands in
for a spring the real hand carries or could be fitted with; the smallest
kq at which every finger is green is the stiffness the hold needs. The
scale margin at each kq says how many times the leader's pad forces the
hand then carries. Every number in the table is read from the JSON.
"""
import argparse
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

DEFAULT_KQ = [0.0, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2]


def run(hand, kqs, frames, out_json):
    from gramian_workshop.follow import follow
    with open(os.path.join(ROOT, "data", "leaders", f"hold_{hand}__self.json"), encoding="utf-8") as fh:
        leader = json.load(fh)
    with open(os.path.join(ROOT, "outputs", f"hold_placement_{hand}.json"), encoding="utf-8") as fh:
        fixed = json.load(fh)
    runs = []
    for kq in kqs:
        t0 = time.time()
        r = follow(hand, leader, n_eval=frames, placement=fixed, margin=True, joint_stiffness=kq)
        runs.append({"joint_stiffness_Nm_per_rad": kq, "elapsed_s": round(time.time() - t0, 1),
                     "frames": r["frames"]})
        print(f"kq {kq:g}: {[f['verdict'] for f in r['frames']]} "
              f"{sorted({f['reason'] for f in r['frames'] if f['reason']})} {runs[-1]['elapsed_s']} s",
              flush=True)
        with open(out_json, "w", encoding="utf-8") as fh:
            json.dump({"hand": hand, "leader": leader["id"], "n_eval": frames, "placement": "fixed",
                       "note": "passive joint stiffness bracket (follow(joint_stiffness=)); kq 0 is "
                               "the pessimistic bound (the model's own springs only), larger kq "
                               "stands in for a spring the real hand carries or could be fitted with",
                       "runs": runs}, fh, indent=1, default=float)


def table(out_json):
    with open(out_json, encoding="utf-8") as fh:
        d = json.load(fh)
    hdr = ("kq N.m/rad", "verdict", "reason", "finger states (frame 1)", "worst unbalanced N.m", "scale margin", "s")
    lines = ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
    for r in d["runs"]:
        frs = r["frames"]
        fing = frs[0]["fingers"]
        lines.append("| " + " | ".join([
            f"{r['joint_stiffness_Nm_per_rad']:g}",
            ",".join(sorted({f["verdict"] for f in frs})),
            ",".join(sorted({f["reason"] for f in frs if f["reason"]})) or "-",
            ", ".join(f"{x['finger']} {x['state']}" for x in fing),
            f"{max(x.get('unbalanced_Nm') or 0.0 for x in fing):.5f}",
            ",".join(f"{(f.get('scale_margin') or {}).get('margin', float('nan')):.3g}" for f in frs),
            f"{r['elapsed_s']}"]) + " |")
    return "\n".join(lines), d


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("hand")
    ap.add_argument("--kq", type=float, nargs="+", default=DEFAULT_KQ)
    ap.add_argument("--frames", type=int, default=2)
    ap.add_argument("--table-only", action="store_true", help="only rewrite the .md from the stored .json")
    a = ap.parse_args()
    out_json = os.path.join(ROOT, "outputs", f"hold_{a.hand}__kq_bracket.json")
    if not a.table_only:
        run(a.hand, a.kq, a.frames, out_json)
    t, d = table(out_json)
    out_md = out_json[:-5] + ".md"
    with open(out_md, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(f"# {d['hand']}: self-hold against passive joint stiffness\n\n"
                 f"Leader `{d['leader']}`, fixed placement, {d['n_eval']} frames, scale margins. "
                 "Written by `tools/hold_stiffness_bracket.py` from "
                 f"`outputs/{os.path.basename(out_json)}`. {d['note']}.\n\n" + t + "\n")
    print(t)
    print(f"wrote {os.path.relpath(out_md, ROOT)}")


if __name__ == "__main__":
    main()
