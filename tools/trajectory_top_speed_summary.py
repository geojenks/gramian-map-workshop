"""Summary of tools/trajectory_top_speed_sweep.py's readouts.

    python tools/trajectory_top_speed_summary.py [dir] [--table]

dir defaults to outputs/top_speed/. Per hand, leader and setting, with and
without the fingers' motion torque: the frames checked, the runs of frames
that hold alone, the frames that hold at no speed, the speed found (the
slowest run's, as a multiple of the recorded speed; "capped" where the
search passed its ceiling without failing), and whether every run of the
retimed leader held when checked again. --table prints a markdown table
of the speeds alone. Speeds are comparable only at the same frame
density and setting; the time constants are illustrative.
"""
import glob
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from trajectory_sweep import SETTINGS  # noqa: E402
from trajectory_top_speed import setting_label  # noqa: E402

# table labels for the measured human hands, as tools/trajectory_summary.py
LABEL = {"george_clip": "human, clip skeleton", "george_kp": "human, measured",
         "george_kp_wide": "human, measured, wide ranges", "george_measured_thumb": "human, measured thumb base"}
ORDER = [setting_label(r, l) for _, r, l in SETTINGS]


def nice(setting):
    """'rate3__lag0.1' -> 'rate 3 /s, lag 0.1 s'."""
    parts = []
    for p in setting.split("__"):
        if p.startswith("lag"):
            parts.append(f"lag {p[3:]} s")
        elif p.startswith("rate"):
            parts.append(f"rate {p[4:]} /s")
        else:
            parts.append(p)
    return ", ".join(parts)

args = [a for a in sys.argv[1:] if a != "--table"]
TABLE = "--table" in sys.argv
D = args[0] if args else os.path.join(ROOT, "outputs", "top_speed")


def speed_text(ts):
    sp = ts["speed"]
    if sp is None:
        return "-"
    return f"{sp:.3g}" + (" (capped)" if ts["capped"] else "")


def retimed_text(r):
    rt = r.get("retimed")
    if rt is None:
        return "-"
    held = [ru["holds"] for ru in rt["trajectory"]["runs"]]
    s = f"{sum(held)}/{len(held)}"
    if r["inertia"] and rt["n_fitted"] != rt["n_fitted_search"]:
        s += f" (fitted {rt['n_fitted']} vs {rt['n_fitted_search']})"
    return s


res = {}
for f in glob.glob(os.path.join(D, "topspeed__*.json")):
    r = json.load(open(f, encoding="utf-8"))
    res[(r["hand"], r["leader"], r["setting"], r["inertia"])] = r

if TABLE:
    print("| hand | link | frames | runs | hold at no speed | speed | speed with motion torque | retimed holds | with motion torque |")
    print("|---|---|---|---|---|---|---|---|---|")
for hand, leader in sorted({k[:2] for k in res}):
    if not TABLE:
        print(f"\n== {hand} on {leader} ==")
    here = {k[2] for k in res if k[:2] == (hand, leader)}
    for s in ORDER + sorted(here - set(ORDER)):
        pair = [res.get((hand, leader, s, i)) for i in (False, True)]
        if all(r is None for r in pair):
            continue
        if TABLE:
            r0 = pair[0] or pair[1]
            ts = [r["trajectory"]["top_speed"] if r else None for r in pair]
            runs = "/".join(str(len(t["runs"])) if t else "-" for t in ts)
            unh = "/".join(str(len(t["unholdable"])) if t else "-" for t in ts)
            print(f"| {LABEL.get(hand, hand)} | {nice(s)} | {len(r0['frame_k'])} | {runs} | {unh} | "
                  + " | ".join(speed_text(t) if t else "-" for t in ts) + " | "
                  + " | ".join(retimed_text(r) if r else "-" for r in pair) + " |")
            continue
        for r in pair:
            if r is None:
                continue
            ts = r["trajectory"]["top_speed"]
            print(f" {s}{' with motion torque' if r['inertia'] else ''}: {len(r['frame_k'])} frames checked, "
                  f"{len(ts['runs'])} runs, {len(ts['unholdable'])} frames hold at no speed "
                  f"({', '.join(sorted({u['reason'] for u in ts['unholdable']})) or 'none'}); "
                  f"speed {speed_text(ts)}; search wall {r['search_wall_s']:.0f} s; "
                  f"solver other status {ts['lp']['other_status']}")
            for ru in ts["runs"]:
                print(f"   run k {ru['k_first']}..{ru['k_last']} ({ru['n_frames']} frames): "
                      + ("no torque check" if ru["speed"] is None else
                         f"{ru['speed']:.3g}" + (" (capped)" if ru["capped"] else "")
                         + ("" if ru.get("contiguous") in (True, None) else " (not contiguous)")))
            rt = r.get("retimed")
            if rt:
                print(f"   retimed to {rt['speed']:.3g}: runs holding {retimed_text(r)}, wall {rt['wall_s']:.0f} s")
