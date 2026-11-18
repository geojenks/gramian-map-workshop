"""Backwards lookup: which records point their tip the way the task needs?

    python demos/backwards_lookup.py [--task cup_turn] [--n 4]
                                     [--mount aimed] [--yaw 90] [--drafts]

Starts from a task the hand cannot do and works back to the records. For
each record with a free tip curve, the share of its tip motion along the
direction the task needs (the azimuth at contact 0, in the finger's own
frame), asked over the first third of its input range, the first half, the
whole sweep, and the best run at least a third long. See
gramian_workshop/lookup.py for the calculation.

Prints the ranked table and writes outputs/backwards_lookup.{txt,json}.
With --drafts it also reads the draft records under data/draft_records/
(digitised from published figures, not reviewed, not in the manifest), each
flagged as a draft plus any flags in data/draft_records/FLAGS.json, and
writes outputs/backwards_lookup_with_drafts.{txt,json} instead.
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

from gramian_workshop.lookup import WINDOWS, rank  # noqa: E402
from gramian_workshop.task import load_task  # noqa: E402

CAVEAT = ("SIMULATION AND RECORDS ONLY: comparative, not predictive. Shares "
          "are read from each record's free tip curve; nothing is simulated.")
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
    a = ap.parse_args(argv)
    task = load_task(a.task)
    rows, left_out = rank(task, a.n, a.mount, a.yaw,
                          draft_records() if a.drafts else None)
    stem = "backwards_lookup_with_drafts" if a.drafts else "backwards_lookup"
    turners, have, why_not = stored_verdict(a.task, task, a.n, a.mount, a.yaw)

    lines = [f"Backwards lookup - {a.task}, N={a.n}, mount {a.mount}, "
             f"yaw {a.yaw:g} deg", CAVEAT, "",
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
             "Ranked by the first-third share, highest first.", ""]
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
        lines.append(ln.rstrip())
    if turners is not None:
        lines += ["", "stored turner: yes or no from outputs/cup_turn.json; "
                  "- where the record has no stored verdict there, or carries "
                  "a flag, so it is not compared."]
    lines += ["", f"left out ({len(left_out)} of {len(rows) + len(left_out)} "
              "records):"]
    lines += [f"  {lo['id']}: {lo['reason']}" for lo in left_out]

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
    js = {"task": a.task, "caveat": CAVEAT, "n": a.n, "mount": a.mount,
          "yaw_deg": a.yaw, "made_by": "demos/backwards_lookup.py",
          "rows": rows, "left_out": left_out,
          "separation": sep_js,
          "separation_not_checked": why_not}
    with open(os.path.join(ROOT, "outputs", stem + ".json"), "w",
              encoding="utf-8") as fo:
        json.dump(no_nan(js), fo, indent=1)
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
