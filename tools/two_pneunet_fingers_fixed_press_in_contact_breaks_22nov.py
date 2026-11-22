"""Where each tip's touching mark switches on and off in the fixed press-in runs (22 Nov).

Reads the frames file a clip is drawn from (and the full log beside it) and lists, per finger, every
stretch with the tip marked touching: start and end in log time, and how long it lasts. This is what
a viewer sees as "contact on" and "contact off". It also prints the mean normal force as a fraction
of the logged one, taken from the run's results file, and the unrounded turn.

Nothing is simulated. SIMULATION OUTPUT READ ONLY; comparative, not predictive.

Usage: python two_pneunet_fingers_fixed_press_in_contact_breaks_22nov.py <run folder> <key> [<key> ...]
"""
import csv
import json
import os
import sys

NAMES = ["index", "middle", "ring", "thumb"]


def runs(rows, col):
    out, start = [], None
    for r in rows:
        on = float(r[col]) > 0.5
        t = float(r["log_t"])
        if on and start is None:
            start = t
        if not on and start is not None:
            out.append((start, last))
            start = None
        last = t
    if start is not None:
        out.append((start, last))
    return out


def gait(rows):
    parts = sorted({r["part"] for r in rows})
    return [r for r in rows if r["part"] == "2"] if "2" in parts else rows


def main():
    folder, keys = sys.argv[1], sys.argv[2:]
    lines = [__doc__.strip().splitlines()[0], "Read from " + folder, ""]
    res = None
    p = os.path.join(folder, "axle_results.json")
    if os.path.exists(p):
        res = json.load(open(p, encoding="utf-8"))
    for key in keys:
        lines.append(f"Run '{key}':")
        for kind in ("frames", "log"):
            f = os.path.join(folder, f"{kind}_{key}.csv")
            rows = gait(list(csv.DictReader(open(f, encoding="utf-8"))))
            what = "clip frames" if kind == "frames" else "every logged simulation sample"
            lines.append(f"  From {what} ({len(rows)} rows of the gait); first turn {float(rows[0]['turn_deg']):.3f} deg, "
                         f"last {float(rows[-1]['turn_deg']):.3f} deg, at log time {float(rows[-1]['log_t']):.3f} s:")
            for k, name in enumerate(NAMES):
                rr = runs(rows, f"f{k}_touching")
                txt = "; ".join(f"{a:.2f} to {b:.2f} s ({b - a:.2f} s)" for a, b in rr)
                short = sum(1 for a, b in rr if b - a < 0.1)
                lines.append(f"    {name}: {len(rr)} touching stretches, {short} shorter than 0.1 s of log time: {txt}")
        lines.append("")
    rep = os.path.join(folder, "contact_breaks_" + "_".join(keys) + ".txt")
    open(rep, "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    print("Written:", rep)
    if res is not None:
        print("results keys:", list(res)[:12])


main()
