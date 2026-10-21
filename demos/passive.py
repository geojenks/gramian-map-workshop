"""Demo 3: the same cup with passive stiffness counted (registration
addendum section 13, Q1).

    python demos/passive.py [--record my_actuator.json]

Each finger's force is its active push plus a passive spring force from the
record's tip stiffness, within a deflection budget (2 mm, the banked cup's
press depth; 1 and 4 mm as sensitivity). Writes
outputs/cup_hold_passive_{1,2,4}mm.{txt,json,html} and
outputs/cup_turn_passive_2mm.{txt,json} and scores Q1-P1 to Q1-P6.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import report  # noqa: E402
from gramian_workshop.arrange import Arrangement  # noqa: E402
from gramian_workshop.finger import all_fingers  # noqa: E402
from gramian_workshop.readout import (PASSIVE_HELD, soft_hold,  # noqa: E402
                                      soft_hold_passive)
from gramian_workshop.task import load_task  # noqa: E402
from gramian_workshop.turn import run  # noqa: E402

NS = list(range(1, 9))
MOUNTS = [("aimed", 0.0), ("aimed", 90.0), ("palm", 0.0), ("palm", 90.0)]
BUDGETS_MM = (1.0, 2.0, 4.0)
HEADLINE_MM = 2.0
BAND_DEG = 2.3            # Q1-G1's circle-versus-pyramid band about atan(mu)
CAVEAT = ("SIMULATION AND RECORDS ONLY: comparative, not predictive. "
          "Forces and stiffnesses are uncalibrated; the deflection and "
          "strength levels are shown, never scored.")
LETTER = {k: v[0] for k, v in report.REASON_STYLE.items()}


def mkey(style, yaw):
    return f"{style}_yaw{int(yaw)}"


def hold(task, fingers, dm_mm):
    out = {}
    for f in fingers:
        for style, yaw in MOUNTS:
            for n in NS:
                a = Arrangement(f, task, n, style, yaw)
                c = a.cup
                w = c.weight_demand()
                out[(f.id, mkey(style, yaw), n)] = soft_hold_passive(
                    c.points, c.normals, c.com, w, c.mu, c.L, a.B,
                    a.passive(dm_mm * 1e-3), a.F, a.D)
    return out


def active(task, fingers):
    out = {}
    for f in fingers:
        for style, yaw in MOUNTS:
            for n in NS:
                a = Arrangement(f, task, n, style, yaw)
                c = a.cup
                out[(f.id, mkey(style, yaw), n)] = soft_hold(
                    c.points, c.normals, c.com, c.weight_demand(), c.mu, c.L,
                    a.B, a.F)["reason"]
    return out


def b_angle(f, task, style, yaw):
    if f.b is None:
        return None
    a = Arrangement(f, task, 1, style, yaw)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(
        np.dot(a.B[0], a.cup.normals[0]))))))


def score(task, ttask, fingers, h, act, turn):
    half = math.degrees(math.atan(float(task["mu"])))
    scored = [f for f in fingers if not f.quality]
    stiff = [f for f in scored if f.k.any()]
    none = [f for f in scored if not f.k.any()]
    held = PASSIVE_HELD
    P, D = {}, {}
    bad = []
    for f in none:
        for style, yaw in MOUNTS:
            ang = b_angle(f, task, style, yaw)
            for n in NS:
                k = (f.id, mkey(style, yaw), n)
                if h[k]["reason"] != act[k] and not (
                        ang is not None and abs(ang - half) <= BAND_DEG):
                    bad.append((*k, act[k], h[k]["reason"]))
    P["Q1-P1"], D["Q1-P1"] = not bad, bad[:10]
    bad = [(f.id, m, n, h[(f.id, m, n)]["reason"]) for f in stiff
           for m in ("aimed_yaw0", "aimed_yaw90") for n in NS
           if (n == 1 and h[(f.id, m, n)]["reason"] != "direction")
           or (n >= 2 and h[(f.id, m, n)]["reason"] not in held)]
    P["Q1-P2"], D["Q1-P2"] = not bad, bad[:10]

    def palm0(ids, want):
        out = []
        for rid in ids:
            for n in NS[1:]:
                r = h[(rid, "palm_yaw0", n)]["reason"]
                if not (r in held if want == "held" else r == want):
                    out.append((rid, n, r))
        return out
    walls = [f.id for f in stiff if f.id.startswith("wall_")]
    b3 = palm0(walls, "held")
    P["Q1-P3"], D["Q1-P3"] = not b3, b3[:10]
    b4 = palm0(["mckibben_v2_20deg", "mckibben_v2_30deg",
                "mckibben_v2_40deg"], "held") + palm0(["cable_finger"],
                                                      "friction")
    P["Q1-P4"], D["Q1-P4"] = not b4, b4[:10]
    b5 = palm0(["bellows1"], "held")
    P["Q1-P5"], D["Q1-P5"] = not b5, b5[:10]
    b6 = [(f.id, duty, n, turn[(f.id, duty, n)]) for f in stiff
          for duty, n in turn_keys(ttask)
          if turn[(f.id, duty, n)] != (duty == ttask["wave"]["duty"]
                                       and n >= 4)]
    P["Q1-P6"], D["Q1-P6"] = not b6, b6[:10]
    return P, D


def turn_keys(task):
    duties = [task["wave"]["duty"]] + list(task["wave"]["duty_sensitivity"])
    return [(d, n) for d in duties for n in NS]


def write_hold(task, fingers, h, dm, txt_extra):
    lines = [f"Demo 3 - cup_hold with passive stiffness, budget {dm:g} mm "
             "(addendum section 13)", CAVEAT, "",
             "letters: " + ", ".join(f"{v[0]} {k}" for k, v in
                                     report.REASON_STYLE.items()), ""]
    head = f"{'':32s}" + "".join(f"{n:>3d}" for n in NS)
    for style, yaw in MOUNTS:
        m = mkey(style, yaw)
        lines += [f"mount {m}", head]
        for f in fingers:
            note = []
            if not f.k.any():
                note.append("no stiffness")
            note += f.flags()
            lines.append(f"{f.id[:32]:32s}" + "".join(
                f"{LETTER[h[(f.id, m, n)]['reason']]:>3s}" for n in NS)
                + (f"  [{'; '.join(note)}]" if note else ""))
        lines.append("")
    lines += txt_extra
    txt = "\n".join(lines) + "\n"
    base = os.path.join(ROOT, "outputs", f"cup_hold_passive_{dm:g}mm")
    with open(base + ".txt", "w") as fo:
        fo.write(txt)
    with open(base + ".json", "w") as fo:
        json.dump({"budget_mm": dm, "caveat": CAVEAT,
                   "soft": {"|".join(map(str, k)): v for k, v in h.items()}},
                  fo, indent=1, default=str)
    body = [report.legend_reasons(),
            f'<p class="note">Each finger pushes along its record\'s blocked '
            f"force and also resists as a spring (the record's tip stiffness, "
            f"minimum over probed inputs) within &plusmn;{dm:g} mm per axis. "
            "Records without stiffness data are marked &#9675;.</p>"]
    for style, yaw in MOUNTS:
        m = mkey(style, yaw)
        body.append(f"<h2>mount {report.esc(m)}</h2>")

        def cell(i, j, m=m):
            f = fingers[i]
            r = h[(f.id, m, NS[j])]
            fill, letter, ink = report.reason_cell(r["reason"])
            tip = (f"{f.id}, {m}, N={NS[j]}, budget {dm:g} mm: {r['reason']}; "
                   f"k (x,y,z) = {', '.join(f'{k:.4g}' for k in f.k)} N/m")
            if f.k_notes:
                tip += "; " + "; ".join(f.k_notes)
            return fill, letter, tip, ink
        labels = [f.id + ("" if f.k.any() else " ○")
                  + (" ⚑" if f.quality else "") for f in fingers]
        body.append('<div class="card">' + report.grid_svg(
            labels, [f"N={n}" for n in NS], cell) + "</div>")
    body.append("<details><summary>Table view and prediction scores"
                f'</summary><div class="card"><pre>{report.esc(txt)}</pre>'
                "</div></details>")
    with open(base + ".html", "w") as fo:
        fo.write(report.page(f"Cup hold with passive stiffness ({dm:g} mm)",
                             CAVEAT, "".join(body)))
    return txt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--record", action="append", default=[],
                    help="add your own actuator record (JSON); repeatable")
    a = ap.parse_args()
    task = load_task("cup_hold")
    ttask = load_task("cup_turn")
    fingers = all_fingers(a.record)
    os.makedirs(os.path.join(ROOT, "outputs"), exist_ok=True)
    act = active(task, fingers)
    holds = {dm: hold(task, fingers, dm) for dm in BUDGETS_MM}
    stiff = [f for f in fingers if f.k.any()]
    turn = {}
    tl = [f"Demo 3 - cup_turn with passive stiffness, budget {HEADLINE_MM:g} "
          "mm, mount aimed_yaw90: fraction of ticks held", CAVEAT, "",
          f"{'':32s}{'duty':>6s}" + "".join(f"{n:>7d}" for n in NS)]
    for f in stiff:
        for duty, n in turn_keys(ttask):
            r = run(Arrangement(f, ttask, n, "aimed", 90.0), duty,
                    delta_max_m=HEADLINE_MM * 1e-3)
            turn[(f.id, duty, n)] = r["soft_held_all"]
            turn[(f.id, duty, n, "frac")] = r["soft_held_frac"]
        for duty in sorted({d for d, _ in turn_keys(ttask)}, reverse=True):
            tl.append(f"{f.id[:32]:32s}{duty:6.2f}" + "".join(
                f"{turn[(f.id, duty, n, 'frac')]:7.2f}" for n in NS))
    P, D = score(task, ttask, fingers, holds[HEADLINE_MM], act, turn)
    ptxt = ["predictions (addendum section 13.5; 2 mm):"] + [
        f"  {k}: {'HIT' if v else 'MISS'}" + (f"  {D[k]}" if not v else "")
        for k, v in P.items()]
    tl += [""] + ptxt
    ttxt = "\n".join(tl) + "\n"
    with open(os.path.join(ROOT, "outputs", "cup_turn_passive_2mm.txt"),
              "w") as fo:
        fo.write(ttxt)
    with open(os.path.join(ROOT, "outputs", "cup_turn_passive_2mm.json"),
              "w") as fo:
        json.dump({"budget_mm": HEADLINE_MM, "caveat": CAVEAT,
                   "held_frac": {f"{k[0]}|{k[1]}|{k[2]}": v
                                 for k, v in turn.items() if len(k) == 4},
                   "predictions": P, "prediction_detail": D},
                  fo, indent=1, default=str)
    for dm in BUDGETS_MM:
        txt = write_hold(task, fingers, holds[dm], dm,
                         ptxt if dm == HEADLINE_MM else [])
        if dm == HEADLINE_MM:
            print(txt)
    print(ttxt)


if __name__ == "__main__":
    main()
