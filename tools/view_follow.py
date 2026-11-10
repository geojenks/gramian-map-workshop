"""One page for the follow-the-leader results: per hand, the share of moments
green / amber / red, a strip per finger through the leader's recording, and
why the red moments are red.

    python tools/view_follow.py outputs/follow_*.json   ->  outputs/follow.html
    python tools/view_follow.py RESULTS... --out PAGE.html [--table TABLE.md]

Where a result carries them (tools/run_follow.py --margins), the page also
shows its scale margin and disturbance margin beside each verdict: per
hand, the smallest over the frames; per frame, on hover and in a table.
--table writes both margins per hand as a Markdown table.
"""
import argparse
import collections
import html
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.report import CSS, page  # noqa: E402

COL = {"green": "var(--st-good)", "amber": "var(--st-warn)", "red": "var(--st-critical)",
       "off": "var(--st-none)"}
INK = {"green": "var(--st-good-ink)", "amber": "var(--st-warn-ink)",
       "red": "var(--st-critical-ink)", "off": "var(--st-none-ink)"}
LETTER = {"green": "✓", "amber": "~", "red": "✕", "off": "·"}
REASON = {"direction": "the touching fingers cannot push the ball that way",
          "friction": "they could, but would slip",
          "torque": "too weak at the joints", "no_contact": "no finger in place",
          "reach": "a finger cannot get there", "facing": "a finger cannot face its pad to the ball"}


# one line each, from the docstrings of follow._scale_margin and
# follow._disturbance_margin
MEANING = {
    "scale_margin": "how many times larger the leader's load on the object could be and the "
                    "contacts the hand makes still carry it (below 1 they cannot carry the "
                    "leader's own load; 0 means the leader's own load already fails)",
    "disturbance_margin": "the smallest extra push or twist on the object, on top of the "
                          "leader's load and as a multiple of that load's size, that the "
                          "contacts made could no longer resist; tried along 12 directions "
                          "(a push and a twist each way along each axis), so the true "
                          "smallest may be lower",
}
NAME = {"scale_margin": "Scale margin", "disturbance_margin": "Disturbance margin"}
KEYS = ("scale_margin", "disturbance_margin")
MARGIN_NOTE = ("Each hand's figure is the smallest over the frames checked, and, where "
               "some frames are red, also the smallest over the frames that pass. "
               "\"At least\" marks a search that stopped at its cap (1024); a direction "
               "such as +fy is the worst push (f) or twist (t) found. "
               "Neither margin changes the verdict.")


def esc(s):
    return html.escape(str(s))


def fmt_margin(v):
    """A frame's margin entry as text: None (not computed: no contact or no
    torque checked) or its number, "at least" when the search stopped at
    its cap, with the worst direction where there is one."""
    if v is None:
        return "not computed"
    m = v["margin"]
    t = f"{m:.0f}" if m >= 100 else f"{m:.3g}"
    if v.get("capped"):
        t = "at least " + t
    if v.get("direction"):
        t += f" ({v['direction']})"
    if v.get("contiguous") is False:
        t += ", feasible scales not one run"
    return t


def smallest(frames, key):
    """The frame whose `key` margin is smallest, as (entry, k), among the
    frames that carry a number; (None, None) if frames carry the key but
    none has a number; None if no frame carries the key at all."""
    if not any(key in f for f in frames):
        return None
    got = [(f[key], f["k"]) for f in frames if isinstance(f.get(key), dict)]
    if not got:
        return None, None
    return min(got, key=lambda g: g[0]["margin"])


def hand_margin_text(frames, key):
    """The smallest margin over the frames and, when some frames are red and
    some pass, the smallest over the frames that pass (green or amber)."""
    sm = smallest(frames, key)
    if sm is None:
        return "not stored"
    v, k = sm
    if v is None:
        return "not computed (no contact or no torque checked)"
    t = f"{fmt_margin(v)}, at frame {k}"
    passing = [f for f in frames if f["verdict"] in ("green", "amber")]
    if passing and len(passing) < len(frames):
        sp = smallest(passing, key)
        if sp is not None and sp[0] is not None:
            t += f"; over the frames that pass, {fmt_margin(sp[0])}, at frame {sp[1]}"
    return t


def margin_table(results, paths):
    """Both margins per hand as a Markdown table: the smallest over each
    result's frames, with the frame it occurs at."""
    out = ["| Hand | Leader | Result | Verdicts | Scale margin | Disturbance margin |",
           "|---|---|---|---|---|---|"]
    for r, p in zip(results, paths):
        v = collections.Counter(f["verdict"] for f in r["frames"])
        n = len(r["frames"])
        verd = ", ".join(f"{s} {v[s]}/{n}" for s in ("green", "amber", "red") if v.get(s))
        out.append(f"| {r['hand']} | `{r['leader']}` | `{os.path.basename(p)}` | {verd} | "
                   + " | ".join(hand_margin_text(r["frames"], key) for key in KEYS) + " |")
    return "\n".join(out)


def strip(states, labels, title, w=760):
    n = len(states)
    cw = w / max(n, 1)
    rows = len(labels)
    h = 18 * rows + 24
    out = [f'<svg width="{w + 90}" height="{h}" role="img" aria-label="{esc(title)}">']
    for r, lab in enumerate(labels):
        y = 18 * r + 4
        out.append(f'<text class="lab" x="84" y="{y + 12}" text-anchor="end">{esc(lab)}</text>')
        for k, st in enumerate(states):
            s, tip = st[r]
            out.append(f'<rect class="c" x="{90 + k * cw}" y="{y}" width="{max(cw, 1)}" height="16" '
                       f'rx="2" style="fill:{COL[s]}"><title>{esc(tip)}</title></rect>')
    out.append("</svg>")
    return "".join(out)


def card(r):
    fr = r["frames"]
    v = collections.Counter(f["verdict"] for f in fr)
    n = len(fr)
    # red moments, by their root cause: a finger that cannot get there or
    # cannot face its pad to the ball, else what the redistribution lacked
    why = collections.Counter()
    for f in fr:
        if f["verdict"] != "red":
            continue
        bad = [x for x in f["fingers"] if x.get("reason")]
        if bad:
            for x in bad:
                why[f"{x['finger']}: {REASON.get(x['reason'], x['reason'])}"] += 1
        elif f["reason"]:
            why[REASON.get(f["reason"], f["reason"])] += 1
    fingers = [x["finger"] for x in fr[0]["fingers"]]
    states = []
    for f in fr:
        col = []
        for x in f["fingers"]:
            st = x["state"]
            tip = f"frame {f['k']}: {x['finger']} on station {x['station']}: {st}"
            if "reach_mm" in x:
                tip += f"; {x['reach_mm']:.1f} mm off, pad {x['facing_deg']:.0f}° from facing, {x.get('phalanx', '')} phalanx"
            if x.get("reason"):
                tip += f"; {REASON.get(x['reason'], x['reason'])}"
            col.append((st if st in COL else "red", tip))
        vt = f"frame {f['k']}: {f['verdict']}" + (f" ({REASON.get(f['reason'], f['reason'])})" if f["reason"] else "")
        for key in KEYS:
            if key in f:
                vt += f"; {NAME[key].lower()} {fmt_margin(f[key])}"
        col.append((f["verdict"], vt))
        states.append(col)
    bar = "".join(f'<span style="display:inline-block;height:14px;width:{100 * v.get(s, 0) / n:.1f}%;'
                  f'background:{COL[s]}" title="{s} {v.get(s, 0)}/{n}"></span>'
                  for s in ("green", "amber", "red"))
    summary = ", ".join(f"{s} {v.get(s, 0)}/{n}" for s in ("green", "amber", "red"))
    reasons = "; ".join(f"{k} ({c} moments)" for k, c in why.most_common()) or "none"
    margins, margins_table = "", ""
    if any(key in f for f in fr for key in KEYS):
        margins = ("<br>" + "; ".join(f"{NAME[key]} (smallest over the frames): "
                                      f"{esc(hand_margin_text(fr, key))}" for key in KEYS) + ".")
        rows = "".join(f"<tr><td>{f['k']}</td><td>{esc(f['verdict'])}</td>"
                       + "".join(f"<td>{esc(fmt_margin(f[key]) if key in f else 'not stored')}</td>"
                                 for key in KEYS) + "</tr>" for f in fr)
        margins_table = ('<details><summary>Margins frame by frame</summary>'
                         '<table class="mt"><tr><th>Frame</th><th>Verdict</th>'
                         + "".join(f"<th>{NAME[key]}</th>" for key in KEYS) + "</tr>"
                         + rows + "</table></details>")
    caveat = f'<br>Caveat: {esc(r["caveat"])}' if r.get("caveat") else ""
    return (f'<div class="card"><h2>{esc(r["hand_name"])}</h2>'
            f'<p class="note">Leader: {esc(r["leader"])}</p>'
            f'<div style="width:100%;max-width:760px;border-radius:4px;overflow:hidden;display:flex">{bar}</div>'
            f'<p class="note">{esc(summary)} of the sampled moments. Red because: {esc(reasons)}.<br>'
            f'Fingers to stations: {esc(", ".join(f"{k} → {v}" for k, v in r["assignment"].items()))}. '
            f'Placement fit: {r["placement_cost_mm"]:.1f} mm mean reach error.'
            f'{margins}{caveat}</p>'
            + strip(states, fingers + ["verdict"], r["hand_name"]) + margins_table + "</div>")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+", help="follow result files")
    ap.add_argument("--out", default=os.path.join(ROOT, "outputs", "follow.html"),
                    help="the page to write (default outputs/follow.html)")
    ap.add_argument("--table", default=None,
                    help="also write both margins per hand as a Markdown table here")
    a = ap.parse_args()
    rs = []
    for p in a.paths:
        with open(p, encoding="utf-8") as f:
            rs.append(json.load(f))
    legend = ('<div class="legend">' + "".join(
        f'<span><span class="sw" style="background:{COL[s]};color:{INK[s]}">{LETTER[s]}</span>{t}</span>'
        for s, t in [("green", "follows the leader as it is"),
                     ("amber", "close, or a different force split does it"),
                     ("red", "cannot (hover for why)"),
                     ("off", "this finger's station is lifted off")]) + "</div>")
    if all(r["leader"].startswith("w10_cup") for r in rs):
        task = ("Each hand follows a leader with one station per finger: the "
                "slowed w10 cup (contacts 52° below the equator, about 40°/s, soft landing and "
                "takeoff). ")
    else:
        task = "Each hand follows the leader named on its card, one station per finger. "
    margin_note = ""
    if any(key in f for r in rs for f in r["frames"] for key in KEYS):
        margin_note = ('<p class="note">' + "<br>".join(
            f"<b>{NAME[key]}</b>: {esc(MEANING[key])}." for key in KEYS)
            + "<br>" + esc(MARGIN_NOTE) + "</p>")
    body = (legend + '<p class="note">' + task
            + "The leader is placed where the hand reaches it best, each finger takes "
            "one station, and contact can be anywhere on the palm side of the last or middle "
            "phalanx. The wrist (Shadow) is held still. "
            "Hover a cell for the details.</p>" + margin_note
            + "<style>table.mt{border-collapse:collapse;margin:8px 0;font-size:13px}"
              "table.mt td,table.mt th{border-bottom:1px solid var(--ring);padding:2px 10px;"
              "text-align:left}</style>"
            + "".join(card(r) for r in rs))
    out = a.out
    with open(out, "w", encoding="utf-8") as f:
        f.write(page("Can this hand follow the leader?",
                     "SIMULATION ONLY: comparative, not predictive. Allegro and LEAP "
                     "models carry no torque limits, so strength is not checked for them.",
                     body))
    print("wrote", out)
    if a.table:
        with open(a.table, "w", encoding="utf-8", newline="\r\n") as f:
            f.write("# Both margins per hand\n\nWritten by `tools/view_follow.py --table` from the "
                    "results named in the table.\n\n"
                    + "".join(f"- {NAME[key]}: {MEANING[key]}.\n" for key in KEYS)
                    + "\n" + MARGIN_NOTE + "\n\n" + margin_table(rs, a.paths) + "\n")
        print("wrote", a.table)


if __name__ == "__main__":
    main()
