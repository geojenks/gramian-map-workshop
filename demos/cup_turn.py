"""Demo 2: turning the ball in the cup - why a wave?

    python demos/cup_turn.py

N copies of a record on the cup ring, made and broken by a travelling
make/break wave (duty from the banked ring, 0.82; 0.5 as sensitivity). On
every tick the made fingers alone must carry the ball's weight (the reason
ladder), and a finger can drive the turn only if its tangent-plane motion
lies along the azimuth (assuming it complies freely along the normal).
Writes outputs/cup_turn.{txt,json,html} and scores predictions 5-7.

A kinematic duty schedule for a capability readout, not a controller; see
the declaration in gramian_workshop/turn.py.
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import report  # noqa: E402
from gramian_workshop.arrange import Arrangement  # noqa: E402
from gramian_workshop.finger import all_fingers  # noqa: E402
from gramian_workshop.readout import HELD  # noqa: E402
from gramian_workshop.task import load_task  # noqa: E402
from gramian_workshop.turn import run  # noqa: E402

NS = list(range(1, 9))
MOUNTS = [("aimed", 90.0), ("palm", 90.0)]
CAVEAT = ("SIMULATION AND RECORDS ONLY: comparative, not predictive. A "
          "kinematic make/break schedule read for capability; nothing is "
          "stepped.")
TURNER_AZ, TURNER_OFF = 0.9, 0.01        # prediction 7's stated thresholds
EXAMPLE = "wall_0.0010m"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--record", action="append", default=[],
                    help="add your own actuator record (JSON, gramian_map "
                         "schema); repeatable")
    a = ap.parse_args()
    task = load_task("cup_turn")
    duties = [task["wave"]["duty"]] + list(task["wave"]["duty_sensitivity"])
    fingers = all_fingers(a.record)
    res = {}
    for f in fingers:
        for style, yaw in MOUNTS:
            for duty in duties:
                for n in NS:
                    res[(f.id, f"{style}_yaw{int(yaw)}", duty, n)] = run(
                        Arrangement(f, task, n, style, yaw), duty)
    scored = [f for f in fingers if f.b is not None and not f.quality]
    d0, d1 = duties[0], duties[1]
    m = "aimed_yaw90"
    P, detail = {}, {}
    bad5 = [(f.id, n) for f in scored for n in NS
            if res[(f.id, m, d0, n)]["soft_held_all"] != (n >= 4)]
    ideal5 = [n for n in NS
              if res[(scored[0].id, m, d0, n)]["ideal_held_all"] != (n >= 4)]
    P["5"] = not bad5 and not ideal5
    detail["5"] = {"soft": bad5[:10], "ideal": ideal5}
    bad6s = [(f.id, n) for f in scored for n in NS
             if res[(f.id, m, d1, n)]["soft_held_all"]]
    ideal6 = {n: res[(scored[0].id, m, d1, n)]["ideal_held_all"] for n in NS}
    bad6i = [n for n in NS if ideal6[n] != (n >= 6)]
    P["6"] = not bad6s and not bad6i
    detail["6"] = {"soft_held_all": bad6s[:10], "ideal_held_all": ideal6}
    turners = sorted(f.id for f in scored
                     if (res[(f.id, m, d0, 4)]["drive_azimuth"] or 0) > TURNER_AZ
                     and res[(f.id, m, d0, 4)]["drive_offaxis"] < TURNER_OFF)
    expect7 = sorted([f.id for f in scored if f.id.startswith("wall_")]
                     + ["cable_finger"])
    P["7"] = turners == expect7
    detail["7"] = {"turners": turners, "expected": expect7}

    lines = ["Demo 2 - cup_turn (W20 step 6; registration sections 4, 5)",
             CAVEAT, "",
             f"duties: {duties} (first from the banked ring's --duty); "
             f"{res[(scored[0].id, m, d0, 4)]['made_count']} fingers made at "
             f"N=4, duty {d0}", ""]
    for style, yaw in MOUNTS:
        mk = f"{style}_yaw{int(yaw)}"
        for duty in duties:
            lines.append(f"mount {mk}, duty {duty}: fraction of ticks held "
                         "(soft fingers | ideal contacts)")
            lines.append(f"{'':32s}" + "".join(f"{n:>7d}" for n in NS)
                         + "   az    off  turn/stroke")
            r0 = res[(scored[0].id, mk, duty, 1)]
            lines.append(f"{'ideal contacts':32s}" + "".join(
                f"{res[(scored[0].id, mk, duty, n)]['ideal_held_frac']:7.2f}"
                for n in NS))
            for f in fingers:
                r4 = res[(f.id, mk, duty, 4)]
                tps = r4["turn_per_stroke_deg"]
                lines.append(
                    f"{f.id[:32]:32s}" + "".join(
                        f"{res[(f.id, mk, duty, n)]['soft_held_frac']:7.2f}"
                        for n in NS)
                    + f" {r4['drive_azimuth']:6.3f} {r4['drive_offaxis']:6.4f}"
                    + ("" if tps is None else f" {tps:8.1f} deg")
                    + (" [" + " ".join(f.flags()) + "]" if f.flags() else ""))
            del r0
            lines.append("")
    lines.append("input count with the made fingers sticking, aimed yaw 90, "
                 f"duty {d0}:")
    for f in scored:
        ics = sorted({x for n in NS for x in res[(f.id, m, d0, n)]["input_count"]
                      if x is not None})
        lines.append(f"  {f.id}: {ics}")
    lines += ["", f"turners at aimed yaw 90 (azimuthal share > {TURNER_AZ}, "
              f"off-axis < {TURNER_OFF}): {turners}", "",
              "predictions (registration section 10):"]
    for k, v in P.items():
        lines.append(f"  P{k}: {'HIT' if v else 'MISS'}  {detail[k]}")
    txt = "\n".join(lines) + "\n"
    os.makedirs(os.path.join(ROOT, "outputs"), exist_ok=True)
    with open(os.path.join(ROOT, "outputs", "cup_turn.txt"), "w") as fo:
        fo.write(txt)
    js = {"task": "cup_turn", "caveat": CAVEAT, "duties": duties,
          "predictions": P, "prediction_detail": detail,
          "summary": {f"{k[0]}|{k[1]}|{k[2]}|{k[3]}": {
              kk: v[kk] for kk in ("soft_held_frac", "ideal_held_frac",
                                   "soft_held_all", "ideal_held_all",
                                   "made_count", "drive_azimuth",
                                   "drive_offaxis", "turn_per_stroke_deg")}
              for k, v in res.items()}}
    with open(os.path.join(ROOT, "outputs", "cup_turn.json"), "w") as fo:
        json.dump(js, fo, indent=1, default=str)

    body = ['<p class="note">Each cell is the fraction of the wave cycle on '
            "which the fingers in contact can carry the ball's weight "
            "(stronger blue = more). 1.00 means held on every tick. Hover for "
            "the drive numbers.</p>"]
    for style, yaw in MOUNTS:
        mk = f"{style}_yaw{int(yaw)}"
        for duty in duties:
            body.append(f"<h2>{report.esc(mk)}, duty {duty}</h2>")
            rows = ["ideal contacts"] + [f.id + (" ⚑" if f.quality else "")
                                         for f in fingers]

            def cell(i, j, mk=mk, duty=duty):
                n = NS[j]
                if i == 0:
                    v = res[(scored[0].id, mk, duty, n)]["ideal_held_frac"]
                    fill, ink = report.seq_fill(v)
                    return (fill, f"{v:.2f}",
                            f"ideal contacts, N={n}, duty {duty}: held "
                            f"{v:.2f} of ticks", ink)
                f = fingers[i - 1]
                r = res[(f.id, mk, duty, n)]
                v = r["soft_held_frac"]
                tip = (f"{f.id}, {mk}, N={n}, duty {duty}: held {v:.2f} of "
                       f"ticks; azimuthal share {r['drive_azimuth']:.3f}, "
                       f"off-axis {r['drive_offaxis']:.4f}")
                if r["turn_per_stroke_deg"] is not None:
                    tip += f"; turn per stroke {r['turn_per_stroke_deg']:.1f} deg"
                fill, ink = report.seq_fill(v)
                return fill, f"{v:.2f}", tip, ink
            body.append('<div class="card">' + report.grid_svg(
                rows, [f"N={n}" for n in NS], cell, cw=44) + "</div>")
    body.append(f"<h2>The wave, tick by tick: {EXAMPLE}, aimed yaw 90, "
                f"duty {d0}</h2>")
    for n in (3, 4):
        r = res[(EXAMPLE, m, d0, n)]
        held = np.isin(r["soft_reason"], HELD)
        body.append('<div class="card">' + report.raster_svg(
            r["made"], held, f"N = {n}: made (blue) and broken per finger; "
            "held strip below") + "</div>")
    body.append("<details><summary>Table view and prediction scores"
                f'</summary><div class="card"><pre>{report.esc(txt)}</pre>'
                "</div></details>")
    with open(os.path.join(ROOT, "outputs", "cup_turn.html"), "w") as fo:
        fo.write(report.page("Cup turn readout", CAVEAT, "".join(body)))
    print(txt)


if __name__ == "__main__":
    main()
