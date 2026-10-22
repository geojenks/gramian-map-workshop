"""One page for the follow-the-leader results: per hand, the share of moments
green / amber / red, a strip per finger through the leader's recording, and
why the red moments are red.

    python tools/view_follow.py outputs/follow_*.json   ->  outputs/follow.html
"""
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


def esc(s):
    return html.escape(str(s))


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
        col.append((f["verdict"], vt))
        states.append(col)
    bar = "".join(f'<span style="display:inline-block;height:14px;width:{100 * v.get(s, 0) / n:.1f}%;'
                  f'background:{COL[s]}" title="{s} {v.get(s, 0)}/{n}"></span>'
                  for s in ("green", "amber", "red"))
    summary = ", ".join(f"{s} {v.get(s, 0)}/{n}" for s in ("green", "amber", "red"))
    reasons = "; ".join(f"{k} ({c} moments)" for k, c in why.most_common()) or "none"
    return (f'<div class="card"><h2>{esc(r["hand_name"])}</h2>'
            f'<div style="width:100%;max-width:760px;border-radius:4px;overflow:hidden;display:flex">{bar}</div>'
            f'<p class="note">{esc(summary)} of the sampled moments. Red because: {esc(reasons)}.<br>'
            f'Fingers to stations: {esc(", ".join(f"{k} → {v}" for k, v in r["assignment"].items()))}. '
            f'Placement fit: {r["placement_cost_mm"]:.1f} mm mean reach error.</p>'
            + strip(states, fingers + ["verdict"], r["hand_name"]) + "</div>")


def main():
    rs = [json.load(open(p)) for p in sys.argv[1:]]
    legend = ('<div class="legend">' + "".join(
        f'<span><span class="sw" style="background:{COL[s]};color:{INK[s]}">{LETTER[s]}</span>{t}</span>'
        for s, t in [("green", "follows the leader as it is"),
                     ("amber", "close, or a different force split does it"),
                     ("red", "cannot (hover for why)"),
                     ("off", "this finger's station is lifted off")]) + "</div>")
    body = (legend + '<p class="note">Each hand follows a leader with one station per finger: the '
            "slowed w10 cup (contacts 52° below the equator, about 40°/s, soft landing and "
            "takeoff). The leader is placed where the hand reaches it best, each finger takes "
            "one station, and contact can be anywhere on the palm side of the last or middle "
            "phalanx. The wrist (Shadow) is held still. "
            "Hover a cell for the details.</p>" + "".join(card(r) for r in rs))
    out = os.path.join(ROOT, "outputs", "follow.html")
    with open(out, "w") as f:
        f.write(page("Can this hand follow the leader?",
                     "SIMULATION ONLY: comparative, not predictive. Allegro and LEAP "
                     "models carry no torque limits, so strength is not checked for them.",
                     body))
    print("wrote", out)


if __name__ == "__main__":
    main()
