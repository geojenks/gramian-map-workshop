"""Backwards lookup: which records point their tip the way the task needs?

    python demos/backwards_lookup.py [--task cup_turn] [--n 4]
                                     [--mount aimed] [--yaw 90] [--drafts]
                                     [--worked-out-lines] [--tag NAME]

Starts from a task the hand cannot do and works back to the records. For
each record with a free tip curve, the share of its tip motion along the
direction the task needs (the azimuth at contact 0, in the finger's own
frame), asked over the first third of its input range, the first half, the
whole sweep, and the best run at least a third long. A record with two
inputs gets one row per measured line (one input swept, the other held), so
a line that starts with the other input already applied is ranked too. See
gramian_workshop/lookup.py for the calculation.

Prints the ranked table and writes outputs/backwards_lookup.{txt,json}.
With --drafts it also reads the draft records under data/draft_records/
(digitised from published figures, not reviewed, not in the manifest), each
flagged as a draft plus any flags in data/draft_records/FLAGS.json, and
writes outputs/backwards_lookup_with_drafts.{txt,json} instead.
With --worked-out-lines a two-input record with a declared rule in
data/map_two_input_rules.json also gets a row for every line across the
surface the map export works out from its two single-input lines (one input
swept, the other held at each of its logged levels). Those rows are marked
as worked out, not simulated, with the number of simulated points that lie
on the line. Where a worked-out line is also logged in the record, only the
logged one is ranked and the two are compared in a table underneath. Where
a worked-out line is not logged but at least MIN_LEVELS simulated points
lie on it, the shares over just those levels are compared too.
Where outputs/cup_turn.json holds a stored turner verdict for the same
setting, it also says whether each share separates the stored turners from
the rest.

Records only: comparative, not predictive. Nothing is simulated.
"""
import argparse
import glob
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from gramian_workshop.finger import load_manifest  # noqa: E402
from gramian_workshop.lookup import MIN_LEVELS, WINDOWS, free_lines, rank  # noqa: E402
from gramian_workshop.task import load_task  # noqa: E402

CAVEAT = ("SIMULATION AND RECORDS ONLY: comparative, not predictive. Shares "
          "are read from each record's free tip curve; nothing is simulated.")
CAVEAT_WORKED_OUT = (
    "SIMULATION AND RECORDS ONLY: comparative, not predictive. Shares are "
    "read from each record's free tip curve, except on the rows marked "
    "'worked out, not simulated', which are read from lines worked out "
    "between a two-input record's logged lines. Nothing is simulated here.")
# The stored turner verdict (outputs/cup_turn.json, prediction 7) was made by
# demos/cup_turn.py at this setting: mount aimed_yaw90, N = 4, first duty.
VERDICT_TASK, VERDICT_STYLE, VERDICT_YAW, VERDICT_N = "cup_turn", "aimed", 90.0, 4
WINDOW_WORDS = {"share_first_third": "first third",
                "share_first_half": "first half",
                "share_whole": "whole sweep",
                "best_window_share": "best window"}


def draft_records():
    """{record file path: flags} for every record under data/draft_records/.
    A file counts as a record if it holds schema_version and id."""
    base = os.path.join(ROOT, "data", "draft_records")
    with open(os.path.join(base, "FLAGS.json"), encoding="utf-8") as fo:
        more = json.load(fo)["flags"]
    out = {}
    for path in sorted(glob.glob(os.path.join(base, "*", "*.json"))):
        with open(path, encoding="utf-8") as fo:
            rec = json.load(fo)
        if isinstance(rec, dict) and "schema_version" in rec and "id" in rec:
            out[path] = {"source": "draft_from_published_figures",
                         **more.get(rec["id"], {})}
    return out


def worked_out_lines(known_ids):
    """A function for rank's `more_lines`: for a two-input record with a
    declared rule, one line per input and per logged level of the other
    input, read off the surface that tools/export_map_paths.py works out
    (the same call the map page's data comes from). Lines with the other
    input at zero are the record's own single-input lines and are not made
    again. Each line's about["worked_out"] holds the rule, its basis, how
    many simulated points with both inputs on lie on the line, the worst
    gap in tip position between the working out and the simulation at
    those points (the export's position_gap_share; None where there are
    none) and "also_logged": whether the record logs this very line.

    Where a line is not logged but, with its start (logged, the other
    input alone), at least MIN_LEVELS simulated points lie on it, two more
    lines over just those levels are returned, marked "check": "simulated"
    and "check": "worked_out". They are for comparing, not for ranking."""
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    import export_map_paths as ex
    declared = ex.load_two_input_rules(ex.TWO_INPUT_RULES, known_ids)
    extras = ex.load_extra_checks(ex.EXTRA_CHECKS, known_ids)

    def lines(rec):
        s = ex.surface_of(rec, declared.get(rec["id"]),
                          extra=extras.get(rec["id"]))
        if s is None:
            return []
        logged, _ = free_lines(rec)
        logged = {(ab["swept_input"], ab["held"][0]["at"])
                  for ab, _, _ in logged or [] if ab["held"]}
        tested = list(s["tests"])
        tested += (s["extra_checks"] or {}).get("points", [])
        ins, g = s["inputs"], s["values"]
        out = []
        for c in (0, 1):
            o = 1 - c
            u = ins[c]["values"]
            if len(u) < MIN_LEVELS:
                continue
            for k, at in enumerate(ins[o]["values"]):
                if at == 0.0:
                    continue

                def cell(m):
                    i, j = (m, k) if c == 0 else (k, m)
                    return [g[key][i][j] for key in ex.XYZ]
                on = {t["inputs"][c]: t for t in tested
                      if t["inputs"][o] == at and t["inputs"][c] != 0.0}
                gaps = [t["position_gap_share"] for t in on.values()
                        if t["position_gap_share"] is not None]
                about = {
                    "swept_input": ins[c]["name"], "swept_unit": ins[c]["unit"],
                    "held": [{"input": ins[o]["name"], "at": at,
                              "unit": ins[o]["unit"]}],
                    "worked_out": {
                        "rule": s["rule"], "basis": s["basis"],
                        "simulated_points_on_this_line": len(on),
                        "worst_position_gap_share_at_them":
                            max(gaps) if gaps else None,
                        "also_logged": (ins[c]["name"], at) in logged}}
                out.append((about, u, [cell(m) for m in range(len(u))]))
                if about["worked_out"]["also_logged"] or \
                        len(on) + 1 < MIN_LEVELS or u[0] != 0.0:
                    continue
                lv = sorted(on)
                for which, y in (
                        ("simulated", [cell(0)] + [[on[v]["measured"][key]
                                                    for key in ex.XYZ]
                                                   for v in lv]),
                        ("worked_out", [cell(0)] + [cell(u.index(v))
                                                    for v in lv])):
                    out.append(({**about, "worked_out": {
                        **about["worked_out"], "check": which}},
                        [0.0] + lv, y))
        return out
    return lines


def line_key(r):
    return (r["id"], r["swept_input"], r["held"][0]["at"] if r["held"] else None)


def u_range(lo_hi, unit):
    return f"{lo_hi[0]:.4g} to {lo_hi[1]:.4g} {unit or '(no unit)'}"


def share(v):
    return "   -  " if v is None or math.isnan(v) else f"{v:6.3f}"


def stored_verdict(task_id, task, n, style, yaw):
    """(stored turner ids, ids with a stored verdict, None) or
    (None, None, why there is nothing to compare with)."""
    if (task_id, style, float(yaw), n) != (VERDICT_TASK, VERDICT_STYLE,
                                           VERDICT_YAW, VERDICT_N):
        return None, None, (
            f"the stored turner verdict in outputs/cup_turn.json is for task "
            f"{VERDICT_TASK}, mount {VERDICT_STYLE}, yaw {VERDICT_YAW:g} deg, "
            f"N={VERDICT_N}; this run asks something else, so it is not "
            "compared")
    path = os.path.join(ROOT, "outputs", "cup_turn.json")
    if not os.path.exists(path):
        return None, None, "outputs/cup_turn.json not found (run demos/cup_turn.py)"
    with open(path, encoding="utf-8") as fo:
        stored = json.load(fo)
    duty = task["wave"]["duty"]
    key = f"|{style}_yaw{int(yaw)}|{duty}|{n}"
    have = {k[:-len(key)] for k in stored["summary"] if k.endswith(key)}
    return set(stored["prediction_detail"]["7"]["turners"]), have, None


def separation(rows, turners, have):
    """Per window: lowest stored-turner share against highest share among
    the rest. Compared: records with a stored verdict and no flags at all
    (force data, a motion direction, no quality flags)."""
    comp = [r for r in rows if r["id"] in have and not r["flags"]]
    out = {}
    for key in WINDOWS:
        t = [(r[key], r["id"]) for r in comp
             if r["id"] in turners and not math.isnan(r[key])]
        o = [(r[key], r["id"]) for r in comp
             if r["id"] not in turners and not math.isnan(r[key])]
        if not t or not o:
            out[key] = None
            continue
        lo, hi = min(t), max(o)
        out[key] = {"lowest_turner": lo[0], "lowest_turner_id": lo[1],
                    "highest_other": hi[0], "highest_other_id": hi[1],
                    "separated": lo[0] > hi[0]}
    return comp, out


def no_nan(x):
    if isinstance(x, float) and math.isnan(x):
        return None
    if isinstance(x, dict):
        return {k: no_nan(v) for k, v in x.items()}
    if isinstance(x, list):
        return [no_nan(v) for v in x]
    return x


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--task", default="cup_turn")
    ap.add_argument("--n", type=int, default=4, help="number of fingers")
    ap.add_argument("--mount", default="aimed", choices=["aimed", "palm"])
    ap.add_argument("--yaw", type=float, default=90.0, help="mount yaw, deg")
    ap.add_argument("--drafts", action="store_true",
                    help="also read the draft records under data/draft_records/")
    ap.add_argument("--worked-out-lines", action="store_true",
                    help="also rank the lines worked out across a two-input "
                         "record's surface by its declared rule (needs --tag)")
    ap.add_argument("--tag", default="",
                    help="added to the output file names; a tagged run never "
                         "overwrites")
    a = ap.parse_args(argv)
    task = load_task(a.task)
    drafts = draft_records() if a.drafts else None
    more = None
    if a.worked_out_lines:
        if not a.tag:
            print("--worked-out-lines needs --tag, so the stored files stay "
                  "as they are")
            return 1
        known = set(load_manifest()["records"])
        for path in draft_records():     # the rules file names draft records
            with open(path, encoding="utf-8") as fo:
                known.add(json.load(fo)["id"])
        more = worked_out_lines(known)
    rows, left_out = rank(task, a.n, a.mount, a.yaw, drafts, more)
    # a worked-out line that the record also logs is not ranked: it is set
    # beside the logged one below, as a check on the working out
    def wo(r):
        return r.get("worked_out") or {}
    both = [r for r in rows if wo(r).get("also_logged")]
    pairs = [r for r in rows if wo(r).get("check")]
    rows = [r for r in rows
            if not wo(r).get("also_logged") and not wo(r).get("check")]
    by_line = {line_key(r): r for r in rows if "worked_out" not in r}
    keys = WINDOWS + ("normal_whole", "other_tangent_whole")

    def compare(m, w):
        return {
            "id": w["id"], "swept_input": w["swept_input"], "held": w["held"],
            "levels_logged": m["levels"], "levels_worked_out": w["levels"],
            "swept_range": w["whole_u"],
            "logged": {k: m[k] for k in keys},
            "worked_out": {k: w[k] for k in keys},
            "largest_share_difference": max(
                (abs(m[k] - w[k]) for k in keys
                 if not math.isnan(m[k]) and not math.isnan(w[k])),
                default=None)}
    checks = [compare(by_line[line_key(w)], w) for w in both]
    sim = {line_key(r): r for r in pairs if wo(r)["check"] == "simulated"}
    checks_unlogged = [compare(sim[line_key(w)], w) for w in pairs
                       if wo(w)["check"] == "worked_out"]
    stem = "backwards_lookup_with_drafts" if a.drafts else "backwards_lookup"
    if a.tag:
        stem += "_" + a.tag
        for ext in (".txt", ".json"):
            if os.path.exists(os.path.join(ROOT, "outputs", stem + ext)):
                print("REFUSING to overwrite:", stem + ext)
                return 1
    turners, have, why_not = stored_verdict(a.task, task, a.n, a.mount, a.yaw)

    caveat = CAVEAT_WORKED_OUT if a.worked_out_lines else CAVEAT
    lines = [f"Backwards lookup - {a.task}, N={a.n}, mount {a.mount}, "
             f"yaw {a.yaw:g} deg", caveat, "",
             "The turn needs each fingertip to move along the azimuth at its "
             "contact. At contact 0 that direction is turned into the "
             "finger's own frame by the mount rotation.",
             "For each record, the steps of its free tip curve are summed as "
             "dy dy^T / |du| over part of its input range; the share is the "
             "part of that sum along the needed direction (1 = all of it).",
             "Parts of the range: the first third of the steps, the first "
             "half, the whole sweep, and the best run of steps at least a "
             "third of the sweep long.",
             "Along normal and other tangent: whole-sweep shares along the "
             "contact normal and the remaining tangent; with the whole-sweep "
             "share they add to one.",
             "A record with more than one input has one row per measured "
             "line: one input swept with the other held at the value shown "
             "after the row. Where that value is not zero the line starts "
             "away from rest. Only lines logged in the record are ranked.",
             "Ranked by the first-third share, highest first.", ""]
    if a.worked_out_lines:
        lines[-3:-2] = [
            "A record with more than one input has one row per line: one "
            "input swept with the other held at the value shown after the "
            "row. Where that value is not zero the line starts away from "
            "rest.",
            "Rows marked 'worked out, not simulated' are not logged in the "
            "record. They are read off the surface the map export works out "
            "from the record's two single-input lines by the rule declared "
            "in data/map_two_input_rules.json. 'simulated points on it' "
            "counts the simulated points with both inputs on that lie on "
            "that line; 0 means nothing on that line has been simulated. "
            "'worst gap' is the largest gap in tip position between the "
            "working out and the simulation at those points, as the map "
            "export gives it (position_gap_share). A "
            "share is built from the steps between levels, and a small gap "
            "in tip position does not by itself bound the error of a share: "
            "see the comparison under the table."]
    if a.drafts:
        lines[-1:-1] = [
            "Draft records (flag source:draft_from_published_figures) are "
            "digitised from published figures, not reviewed and not in the "
            "manifest. Flag frame:shared_three_chamber_frame_not_canonical: "
            "the record is not in the canonical frame, so its share reflects "
            "the frame, not the actuator. Flag "
            "stroke:read_under_preload_tension_not_free: the tip curve was "
            "read with the actuator under a steady load."]
    head = (f"{'record':34s} {'levels':>6s} {'first':>6s} {'first':>6s} "
            f"{'whole':>6s} {'best':>6s}  {'best window input range':28s} "
            f"{'along':>6s} {'other':>7s}")
    head2 = (f"{'':34s} {'':>6s} {'third':>6s} {'half':>6s} {'sweep':>6s} "
             f"{'window':>6s}  {'':28s} {'normal':>6s} {'tangent':>7s}")
    if turners is not None:
        head += "  stored"
        head2 += "  turner"
    lines += [head, head2]
    for r in rows:
        ln = (f"{r['id'][:34]:34s} {r['levels']:6d} "
              f"{share(r['share_first_third'])} {share(r['share_first_half'])} "
              f"{share(r['share_whole'])} {share(r['best_window_share'])}  "
              f"{u_range(r['best_window_u'], r['input_unit']):28s} "
              f"{share(r['normal_whole'])} {share(r['other_tangent_whole']):>7s}")
        if turners is not None:
            ln += ("  " + ("yes" if r["id"] in turners else
                           "no" if r["id"] in have and not r["flags"]
                           else "-")).ljust(8)
        if r["flags"]:
            ln += "  [" + " ".join(r["flags"]) + "]"
        if r["held"]:
            ln += "  {sweeps " + str(r["swept_input"]) + "; " + "; ".join(
                f"{h['input']} held at {h['at']:g} {h['unit'] or ''}".rstrip()
                for h in r["held"]) + "}"
        if "worked_out" in r:
            w = r["worked_out"]
            ln += (f"  {{worked out, not simulated: {w['rule']} rule; "
                   "simulated points on it: "
                   f"{w['simulated_points_on_this_line']}")
            g = w["worst_position_gap_share_at_them"]
            ln += (f", worst gap {g:.2g}}}" if g is not None else "}")
        lines.append(ln.rstrip())
    if turners is not None:
        lines += ["", "stored turner: yes or no from outputs/cup_turn.json; "
                  "- where the record has no stored verdict there, or carries "
                  "a flag, so it is not compared."]
    lines += ["", f"left out ({len(left_out)} of "
              f"{len({r['id'] for r in rows}) + len(left_out)} records):"]
    lines += [f"  {lo['id']}: {lo['reason']}" for lo in left_out]

    if a.worked_out_lines:
        lines += ["", "Working out checked against lines the record also "
                  "logs (the logged line is the one ranked above):"]
        if not checks:
            lines.append("  no worked-out line is also logged")
        for c in checks:
            lines.append(
                f"  {c['id']}, sweeps {c['swept_input']}, "
                f"{c['held'][0]['input']} held at {c['held'][0]['at']:g}: "
                f"levels {c['levels_logged']} logged, "
                f"{c['levels_worked_out']} worked out")
            for word, k in (("logged    ", "logged"), ("worked out", "worked_out")):
                lines.append(
                    f"    {word}: first third {share(c[k]['share_first_third'])}"
                    f"  first half {share(c[k]['share_first_half'])}"
                    f"  whole {share(c[k]['share_whole'])}"
                    f"  best {share(c[k]['best_window_share'])}"
                    f"  normal {share(c[k]['normal_whole'])}"
                    f"  other tangent {share(c[k]['other_tangent_whole'])}")
            lines.append(f"    largest difference in a share: "
                         f"{c['largest_share_difference']:.2g}")
        lines += ["", "Working out checked against simulated points on "
                  "lines the record does not log. Shares here are over just "
                  "the levels that were simulated (the start, with the swept "
                  "input at zero, is logged), so they are coarser than the "
                  "ranked rows and are not the same numbers:"]
        if not checks_unlogged:
            lines.append("  no such line has enough simulated points")
        for c in checks_unlogged:
            lines.append(
                f"  {c['id']}, sweeps {c['swept_input']}, "
                f"{c['held'][0]['input']} held at {c['held'][0]['at']:g}: "
                f"{c['levels_logged']} levels, swept from "
                f"{c['swept_range'][0]:g} to {c['swept_range'][1]:g}")
            for word, k in (("simulated ", "logged"), ("worked out", "worked_out")):
                lines.append(
                    f"    {word}: first third {share(c[k]['share_first_third'])}"
                    f"  first half {share(c[k]['share_first_half'])}"
                    f"  whole {share(c[k]['share_whole'])}"
                    f"  best {share(c[k]['best_window_share'])}"
                    f"  normal {share(c[k]['normal_whole'])}"
                    f"  other tangent {share(c[k]['other_tangent_whole'])}")
            lines.append(f"    largest difference in a share: "
                         f"{c['largest_share_difference']:.2g}")
    sep_js = None
    lines.append("")
    if turners is None:
        lines.append(f"Separation from the stored verdict: not checked; {why_not}.")
    else:
        comp, sep = separation(rows, turners, have)
        nt = sum(r["id"] in turners for r in comp)
        lines.append(
            "Does the lookup separate the stored turners from the rest? "
            f"Compared: the {len(comp)} ranked records with a stored verdict "
            "and no flags at all (so force data, a motion direction and no "
            f"quality flags), {nt} of them stored turners.")
        for key in WINDOWS:
            s = sep[key]
            if s is None:
                lines.append(f"  {WINDOW_WORDS[key]}: nothing to compare")
                continue
            lines.append(
                f"  {WINDOW_WORDS[key]:12s}: lowest stored turner "
                f"{s['lowest_turner']:.3f} ({s['lowest_turner_id']}), highest "
                f"other {s['highest_other']:.3f} ({s['highest_other_id']}) -> "
                f"{'separated' if s['separated'] else 'NOT separated'}")
        sep_js = {"compared": [r["id"] for r in comp],
                  "stored_turners": sorted(turners & {r["id"] for r in comp}),
                  "by_window": sep}
    txt = "\n".join(lines) + "\n"

    os.makedirs(os.path.join(ROOT, "outputs"), exist_ok=True)
    with open(os.path.join(ROOT, "outputs", stem + ".txt"), "w",
              encoding="utf-8") as fo:
        fo.write(txt)
    js = {"task": a.task, "caveat": caveat, "n": a.n, "mount": a.mount,
          "yaw_deg": a.yaw, "made_by": "demos/backwards_lookup.py",
          "rows": rows, "left_out": left_out,
          "separation": sep_js,
          "separation_not_checked": why_not}
    if a.worked_out_lines:
        js["worked_out_lines_checked_against_logged"] = checks
        js["worked_out_lines_checked_against_simulated_points"] = [
            {("simulated" if k == "logged" else
              "levels" if k == "levels_logged" else k): v
             for k, v in c.items() if k != "levels_worked_out"}
            for c in checks_unlogged]
    with open(os.path.join(ROOT, "outputs", stem + ".json"), "w",
              encoding="utf-8") as fo:
        json.dump(no_nan(js), fo, indent=1)
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
