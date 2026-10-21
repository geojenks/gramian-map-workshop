"""Demo 1: can N copies of your actuator hold a ball in a cup?

    python demos/cup_hold.py

For every record, mount and N = 1..8 fingers on W20 step 4's cup ring, the
reason ladder (registration section 5): no contact, direction, friction,
cone edge, strength (comparative, never scored), met. The top row is the
ideal bracket: any hand touching at the same points. Writes
outputs/cup_hold.{txt,json,html} and scores predictions 1-4.
"""
import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import report  # noqa: E402
from gramian_workshop.arrange import Arrangement  # noqa: E402
from gramian_workshop.finger import all_fingers  # noqa: E402
from gramian_workshop.readout import HELD, ideal_hold, soft_hold  # noqa: E402
from gramian_workshop.task import Cup, load_task  # noqa: E402

NS = list(range(1, 9))
MOUNTS = [("aimed", 0.0), ("aimed", 90.0), ("palm", 0.0), ("palm", 90.0)]
CAVEAT = ("SIMULATION AND RECORDS ONLY: comparative, not predictive. "
          "Forces are uncalibrated; strength is shown, never scored.")
LETTER = {k: v[0] for k, v in report.REASON_STYLE.items()}


def mkey(style, yaw):
    return f"{style}_yaw{int(yaw)}"


def sweep(task, extra=()):
    ideal = {}
    for n in NS:
        c = Cup(task, n)
        ideal[n] = ideal_hold(c.points, c.normals, c.com, c.weight_demand(),
                              c.mu, c.L)
    soft = {}
    fingers = all_fingers(extra)
    for f in fingers:
        for style, yaw in MOUNTS:
            for n in NS:
                a = Arrangement(f, task, n, style, yaw)
                c = a.cup
                soft[(f.id, mkey(style, yaw), n)] = soft_hold(
                    c.points, c.normals, c.com, c.weight_demand(), c.mu, c.L,
                    a.B, a.F, a.D, want_margin=True)
    return fingers, ideal, soft


def score(fingers, ideal, soft):
    with_b = [f for f in fingers if f.b is not None and not f.quality]
    P = {}
    P["1"] = (ideal[1]["reason"] == "direction"
              and all(ideal[n]["reason"] == "met" and ideal[n]["met_mu0"]
                      for n in NS[1:]))
    bad2 = [(f.id, m, n) for f in with_b for m in ("aimed_yaw0", "aimed_yaw90")
            for n in NS
            if (soft[(f.id, m, n)]["reason"] == "direction") != (n == 1)
            or (n >= 2 and soft[(f.id, m, n)]["reason"] not in HELD)]
    P["2"] = not bad2
    contract = {f.id for f in with_b if f.b[0] < 0}
    expect3 = {}
    for f in with_b:
        if f.b_rule == "force_N":
            expect3[f.id] = "direction"
        elif f.id in contract:
            expect3[f.id] = "friction"
        elif f.id == "mckibben_v2_60deg":
            expect3[f.id] = "held"
        else:
            expect3[f.id] = "cone_edge"
    bad3 = []
    for f in with_b:
        for n in NS[1:]:
            r = soft[(f.id, "palm_yaw0", n)]["reason"]
            e = expect3[f.id]
            if not (r in HELD if e == "held" else r == e):
                bad3.append((f.id, n, r, e))
    if any(f.b_rule == "force_N" and soft[(f.id, "palm_yaw0", 1)]["reason"]
           != "direction" for f in with_b):
        bad3.append(("force_N at N=1", "not direction"))
    P["3"] = not bad3
    bad4 = [(f.id, m, n, soft[(f.id, m, n)]["input_count"])
            for f in fingers if f.d is not None and not f.quality
            for m in [mkey(*x) for x in MOUNTS] for n in NS
            if soft[(f.id, m, n)]["input_count"] != (1 if n <= 2 else 0)]
    P["4"] = not bad4
    detail = {"2": bad2[:12], "3": bad3[:20], "4": bad4[:12]}
    strength_min = {}
    for f in with_b:
        ok = [n for n in NS if soft[(f.id, "aimed_yaw0", n)]["reason"] == "met"]
        strength_min[f.id] = ok[0] if ok else None
    return P, detail, strength_min


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--record", action="append", default=[],
                    help="add your own actuator record (JSON, gramian_map "
                         "schema); repeatable")
    a = ap.parse_args()
    task = load_task("cup_hold")
    fingers, ideal, soft = sweep(task, a.record)
    P, detail, smin = score(fingers, ideal, soft)
    os.makedirs(os.path.join(ROOT, "outputs"), exist_ok=True)

    rows_txt = ["Demo 1 - cup_hold (W20 step 6 readout; registration section 5)",
                CAVEAT, "",
                f"task: ball R {task['object']['radius_m']} m, mass "
                f"{task['object']['mass_kg']} kg, mu {task['mu']}, ring altitude "
                f"{task['contact_ring']['altitude_deg']:.3f} deg",
                "letters: " + ", ".join(f"{v[0]} {k}" for k, v in
                                         report.REASON_STYLE.items()), ""]
    head = f"{'':32s}" + "".join(f"{n:>3d}" for n in NS)
    rows_txt.append("ideal contacts (any hand at these points)")
    rows_txt.append(head)
    rows_txt.append(f"{'ideal':32s}" + "".join(
        f"{LETTER[ideal[n]['reason']]:>3s}" for n in NS))
    for style, yaw in MOUNTS:
        m = mkey(style, yaw)
        rows_txt += ["", f"mount {m}", head]
        for f in fingers:
            flag = " [" + " ".join(f.flags()) + "]" if f.flags() else ""
            rows_txt.append(f"{f.id[:32]:32s}" + "".join(
                f"{LETTER[soft[(f.id, m, n)]['reason']]:>3s}" for n in NS)
                + flag)
    rows_txt += ["", "smallest N met with comparative strength (aimed, yaw 0):",
                 "  " + ", ".join(f"{k} {v}" for k, v in smin.items()),
                 "", "predictions (registration section 10):"]
    for k, v in P.items():
        rows_txt.append(f"  P{k}: {'HIT' if v else 'MISS'}"
                        + (f"  {detail.get(k)}" if not v and detail.get(k)
                           else ""))
    txt = "\n".join(rows_txt) + "\n"
    with open(os.path.join(ROOT, "outputs", "cup_hold.txt"), "w") as fo:
        fo.write(txt)

    js = {"task": "cup_hold", "caveat": CAVEAT,
          "ideal": {str(n): ideal[n] for n in NS},
          "soft": {f"{k[0]}|{k[1]}|{k[2]}": v for k, v in soft.items()},
          "strength_min_N_aimed_yaw0": smin,
          "predictions": P, "prediction_detail": detail}
    with open(os.path.join(ROOT, "outputs", "cup_hold.json"), "w") as fo:
        json.dump(js, fo, indent=1, default=str)

    body = [report.legend_reasons()]
    body.append("<h2>Ideal contacts (any hand touching at the ring points)</h2>")

    def icell(i, j):
        r = ideal[NS[j]]
        fill, letter, ink = report.reason_cell(r["reason"])
        return fill, letter, (f"ideal contacts, N={NS[j]}: {r['reason']}"
                              f"{'; met with normals alone' if r['met_mu0'] else ''}"), ink
    body.append('<div class="card">' + report.grid_svg(
        ["ideal contacts"], [f"N={n}" for n in NS], icell) + "</div>")
    for style, yaw in MOUNTS:
        m = mkey(style, yaw)
        body.append(f"<h2>Soft fingers from records: mount {report.esc(m)}</h2>")

        def cell(i, j, m=m):
            f = fingers[i]
            r = soft[(f.id, m, NS[j])]
            fill, letter, ink = report.reason_cell(r["reason"])
            tip = (f"{f.id}, {m}, N={NS[j]}: {r['reason']}; input count "
                   f"{r['input_count']}")
            if r.get("margin") is not None:
                tip += f"; comparative scale margin {r['margin']:.3g}"
            if f.flags():
                tip += "; flags: " + ", ".join(f.flags())
            return fill, letter, tip, ink
        labels = [f.id + (" ⚑" if f.quality else "") for f in fingers]
        body.append('<div class="card">' + report.grid_svg(
            labels, [f"N={n}" for n in NS], cell) + "</div>")
    body.append('<p class="note">⚑ = trust flag carried from gramian_map '
                "(floor noise); reported, not scored. Hover a cell for its "
                "reason, input count and comparative margin.</p>")
    body.append("<details><summary>Table view and prediction scores</summary>"
                f'<div class="card"><pre>{report.esc(txt)}</pre></div></details>')
    with open(os.path.join(ROOT, "outputs", "cup_hold.html"), "w") as fo:
        fo.write(report.page("Cup hold readout", CAVEAT, "".join(body)))
    print(txt)


if __name__ == "__main__":
    main()
