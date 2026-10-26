"""Draw chosen moments of a follow-the-leader result: the hand in its solved
pose, the ball, each finger coloured by its verdict, its pad's outward normal
and the station it should touch, from two sides.

    python tools/render_follow.py outputs/follow_<hand>__<leader>__....json [--amber 4 --green 2 --red 2]
        ->  outputs/render_<hand>.html

Re-evaluates the stored placement (follow(placement=...), same stiffness) to
recover each finger's pose; no new search. Orthographic views in the hand's
frame, painter's order by depth. SIMULATION ONLY.
"""
import argparse
import html
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.follow import FingerIK, follow  # noqa: E402
from gramian_workshop.hands import Hand, _mj  # noqa: E402

COL = {"green": "#2e9e5b", "amber": "#e0a100", "red": "#d1453b", "off": "#9aa3ad"}


def capsules(hand, fr):
    """[(a, b, radius, finger label, state)] for every capsule in the pose."""
    mj = _mj()
    m = hand.model
    d = mj.MjData(m)
    q = np.array(hand.base_q)
    states = {}
    for f, x in zip(hand.fingers, fr["fingers"]):
        states[f.label] = x["state"]
        if "q" in x:
            ik = FingerIK(hand, f)
            q[ik.qadr] = x["q"]
    d.qpos[:] = hand.apply_coupling(q)
    mj.mj_kinematics(m, d)
    body_finger = {}
    for f in hand.fingers:
        b = f.tip_body
        while b > 0 and b not in body_finger:
            body_finger[b] = f.label
            if any(m.jnt_bodyid[j] == b for j in f.joints[:1]):
                break
            b = int(m.body_parentid[b])
    out = []
    for g in range(m.ngeom):
        if m.geom_type[g] != mj.mjtGeom.mjGEOM_CAPSULE:
            continue
        c, R = d.geom_xpos[g], d.geom_xmat[g].reshape(3, 3)
        h, r = m.geom_size[g][1], m.geom_size[g][0]
        lab = body_finger.get(int(m.geom_bodyid[g]))
        out.append((c - R[:, 2] * h, c + R[:, 2] * h, r, lab, states.get(lab, "off")))
    return out


def view(caps, fr, R_ball, axes, size=300, title=""):
    """SVG of one orthographic view; axes = (screen x, screen y, depth) as
    (index, sign) pairs of the hand frame."""
    pts = [p for a, b, *_ in caps for p in (a, b)] + [np.array(fr["ball_center"])]
    P = lambda v: np.array([axes[0][1] * v[axes[0][0]], axes[1][1] * v[axes[1][0]]])
    Z = lambda v: axes[2][1] * v[axes[2][0]]
    xy = np.array([P(p) for p in pts])
    lo, hi = xy.min(0) - 0.03, xy.max(0) + 0.03
    sc = (size - 20) / float(max(hi - lo))
    S = lambda v: ((P(v)[0] - lo[0]) * sc + 10, size - ((P(v)[1] - lo[1]) * sc + 10))
    items = []
    for a, b, r, lab, st in caps:
        (x1, y1), (x2, y2) = S(a), S(b)
        items.append(((Z(a) + Z(b)) / 2, f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
                      f'stroke="{COL.get(st, "#9aa3ad") if lab else "#c9ced4"}" stroke-width="{2 * r * sc:.1f}" '
                      f'stroke-linecap="round" opacity="{0.92 if st != "off" else 0.3}"/>'))
    c = np.array(fr["ball_center"])
    cx, cy = S(c)
    items.append((Z(c), f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{R_ball * sc:.1f}" fill="#dfe8f3" '
                  f'stroke="#6b7c93" stroke-width="1.2" opacity="0.85"/>'))
    for x in fr["fingers"]:
        if "target" not in x:
            continue
        t, pw, nw = map(np.array, (x["target"], x["pad_world"], x["pad_normal"]))
        (tx, ty), (px, py), (nx, ny) = S(t), S(pw), S(pw + 0.012 * nw)
        col = COL.get(x["state"], "#9aa3ad")
        items.append((max(Z(t), Z(pw)) + 1, f'<line x1="{px:.1f}" y1="{py:.1f}" x2="{nx:.1f}" y2="{ny:.1f}" '
                      f'stroke="#222" stroke-width="1.6"/><circle cx="{px:.1f}" cy="{py:.1f}" r="2.6" fill="#222"/>'
                      f'<path d="M{tx - 4:.1f},{ty - 4:.1f}L{tx + 4:.1f},{ty + 4:.1f}M{tx - 4:.1f},{ty + 4:.1f}'
                      f'L{tx + 4:.1f},{ty - 4:.1f}" stroke="{col}" stroke-width="2"/>'))
    body = "".join(s for _, s in sorted(items, key=lambda t: t[0]))
    return (f'<svg width="{size}" height="{size}" viewBox="0 0 {size} {size}" role="img" '
            f'aria-label="{html.escape(title)}"><rect width="{size}" height="{size}" fill="#fff"/>'
            f'{body}<text x="8" y="16" font-size="11" fill="#555">{html.escape(title)}</text></svg>')


def pick(frames, n_amber, n_green, n_red):
    out = []
    for v, n in (("red", n_red), ("amber", n_amber), ("green", n_green)):
        fs = [f for f in frames if f["verdict"] == v]
        if fs and n:
            idx = np.linspace(0, len(fs) - 1, min(n, len(fs))).astype(int)
            out += [fs[i] for i in idx]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("result")
    ap.add_argument("--amber", type=int, default=4)
    ap.add_argument("--green", type=int, default=2)
    ap.add_argument("--red", type=int, default=2)
    a = ap.parse_args()
    src = json.load(open(a.result))
    L = json.load(open(os.path.join(ROOT, "data", "leaders", src["leader"] + ".json")))
    kq = src.get("settings", {}).get("joint_stiffness_Nm_per_rad", 0.0)
    r = follow(src["hand"], L, n_eval=len(src["frames"]), placement=src, joint_stiffness=kq)
    hand = Hand(src["hand"])
    R_ball = float(L["object"]["radius_m"])
    cards = []
    for fr in pick(r["frames"], a.amber, a.green, a.red):
        caps = capsules(hand, fr)
        palm = view(caps, fr, R_ball, ((0, 1), (1, 1), (2, 1)), title="palm side (thumb side to the right)")
        side = view(caps, fr, R_ball, ((2, 1), (1, 1), (0, 1)), title="from the thumb side (palm to the right)")
        rows = "".join(
            f'<tr><td><span class="sw" style="background:{COL.get(x["state"], "#9aa3ad")}"></span>{html.escape(x["finger"])}</td>'
            f'<td>{html.escape(x["state"])}</td>'
            + (f'<td>{x["reach_mm"]:.1f} mm</td><td>{x["facing_deg"]:.0f}&deg;</td><td>{x["phalanx"]}</td>'
               if "reach_mm" in x else '<td colspan="3">lifted off</td>')
            + f'<td>{html.escape(str(x.get("reason") or ""))}</td></tr>' for x in fr["fingers"])
        cards.append(f'<section class="card"><h2>moment {fr["k"]}: <span style="color:{COL[fr["verdict"]]}">'
                     f'{fr["verdict"]}</span>{(" (" + html.escape(fr["reason"]) + ")") if fr["reason"] else ""}</h2>'
                     f'<div class="views">{palm}{side}</div>'
                     f'<table><tr><th>finger</th><th>state</th><th>off target</th><th>pad facing</th>'
                     f'<th>phalanx</th><th>reason</th></tr>{rows}</table></section>')
    page = f"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Follow renders</title><style>
body{{font-family:system-ui,sans-serif;margin:16px;color:#222;background:#fafafa}}
.card{{background:#fff;border:1px solid #ddd;border-radius:8px;padding:12px;margin:12px 0}}
.views{{display:flex;flex-wrap:wrap;gap:8px}} svg{{max-width:100%;height:auto;border:1px solid #eee}}
table{{border-collapse:collapse;font-size:13px;margin-top:8px}} td,th{{padding:3px 8px;border-bottom:1px solid #eee;text-align:left}}
.sw{{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:6px}} h2{{font-size:16px;margin:4px 0}}
p{{max-width:760px}}</style></head><body>
<h1 style="font-size:20px">{html.escape(r["hand_name"])} following {html.escape(r["leader"])}</h1>
<p>Each moment drawn from two sides. Fingers are coloured by their verdict (grey: lifted off). The black dot is the
contact point the solver chose on the finger, with its outward normal; the coloured cross is the leader's contact
on the ball. Green needs the dot within 2 mm of the cross and the normal within 30&deg; of pointing into the ball. Lifted-off fingers are drawn faded in the model's rest pose (collisions are not checked, so they may cross the ball).
SIMULATION ONLY.</p>{''.join(cards)}</body></html>"""
    out = os.path.join(ROOT, "outputs", f"render_{src['hand']}.html")
    open(out, "w").write(page)
    print("wrote", out)


if __name__ == "__main__":
    main()
