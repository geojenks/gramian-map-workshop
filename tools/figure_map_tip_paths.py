"""One picture of the roster: where each actuator's tip goes.

    python tools/figure_map_tip_paths.py [--drafts] [--out outputs/map_tip_paths]
                                         [--tag v2] [--arrow FROM_ID,TO_ID]

--tag adds a suffix to the output names (outputs are never overwritten).
--arrow draws an arrow from one record's mark to another's, e.g. from the
finger that failed the task to the record the backwards lookup picks.

Each record is one mark, placed by two numbers read from its free tip curve:

  across  share of the tip motion that is sideways (at right angles to the
          base-to-tip axis X): 1 - W_xx / trace(W), with
          W = sum_k dy_k dy_k^T / |du_k| over the whole sweep. 0 = moves only
          along its length, 1 = moves only sideways. It does not depend on
          which way round the axis the record's Y and Z point.
  up      free stroke, mm (scalars.free_stroke_m), log scale.

Centred on each mark is its own tip path, scaled to a fixed size, with a
small bar at the end the sweep finishes on: up the
page is along the actuator (tip moving away from the base is up), across the
page is sideways distance from the axis. The path shows what the two numbers
cannot: a tip that starts one way and ends another.

Colour is the actuation method (metadata.actuation_physics). A filled mark
has force data the tool can read; a hollow one has motion only. Draft
records (--drafts) are drawn with a dashed outline.

Records with no free tip curve are placed from scalars.free_direction and
free_stroke_m if they hold both (mark only, no path); otherwise left out with
the reason. Records are read, never edited. Refuses to overwrite its outputs.
Simulated magnitudes are comparative, not predictive.
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "demos"))

from gramian_workshop.finger import Finger, all_fingers, load_record  # noqa: E402
from gramian_workshop.lookup import free_curve  # noqa: E402

COLOURS = {"pneumatic": "#2B6CB0", "tendon": "#B8531A", "thermal": "#C53030",
           "electrostatic": "#6B46C1", "hydraulic": "#2C7A7B",
           "rigid": "#4A5568"}
OTHER = "#718096"
GLYPH = 0.045            # half-size of a tip path, in axes fractions


def place(f, rec):
    """(row, None) or (None, reason)."""
    u, y, why = free_curve(rec)
    s = rec.get("scalars") or {}
    stroke = s.get("free_stroke_m")
    path = None
    if why is None:
        dy, du = np.diff(y, axis=0), np.diff(u)
        W = np.einsum("k,ki,kj->ij", 1.0 / np.abs(du), dy, dy)
        tr = float(np.trace(W))
        if tr <= 0:
            return None, "free tip curve does not move"
        sideways = 1.0 - float(W[0, 0]) / tr
        rel = y - y[0]
        path = np.column_stack([np.hypot(rel[:, 1], rel[:, 2]), rel[:, 0]])
        basis = "free tip curve"
    elif f.d is not None:
        sideways = 1.0 - float(f.d[0]) ** 2
        basis = "scalars.free_direction (no free tip curve)"
    else:
        return None, why + "; no motion direction"
    if not stroke:
        return None, "no free stroke"
    return {"id": f.id, "physics": f.physics, "sideways_share": sideways,
            "stroke_mm": 1e3 * float(stroke), "has_force": f.b is not None,
            "flags": f.flags(), "placed_from": basis,
            "path": None if path is None else path.tolist()}, None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--drafts", action="store_true")
    ap.add_argument("--out", default=os.path.join("outputs", "map_tip_paths"))
    ap.add_argument("--tag", default="")
    ap.add_argument("--arrow", default="")
    a = ap.parse_args(argv)
    stem = os.path.join(ROOT, a.out, "map_tip_paths"
                        + ("_with_drafts" if a.drafts else "")
                        + ("_" + a.tag if a.tag else ""))
    for ext in (".png", ".json"):
        if os.path.exists(stem + ext):
            print("REFUSING to overwrite:", stem + ext)
            return 1

    pairs = [(f, load_record(f.id), False) for f in all_fingers()]
    if a.drafts:
        from backwards_lookup import draft_records
        for path, q in draft_records().items():
            rec = load_record(path)
            pairs.append((Finger(rec, q), rec, True))
    rows, left_out = [], []
    for f, rec, draft in pairs:
        row, why = place(f, rec)
        if row is None:
            left_out.append({"id": f.id, "reason": why})
        else:
            row["draft"] = draft
            rows.append(row)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(16, 9), dpi=120)
    ax.set_xlim(-0.08, 1.08)
    ax.set_yscale("log")
    lo = min(r["stroke_mm"] for r in rows)
    hi = max(r["stroke_mm"] for r in rows)
    ax.set_ylim(lo / 2.2, hi * 2.2)
    to_axes = ax.transData + ax.transAxes.inverted()
    aspect = 9 / 16
    pos = {}
    for r in sorted(rows, key=lambda r: r["draft"]):
        c = COLOURS.get(r["physics"], OTHER)
        cx, cy = to_axes.transform((r["sideways_share"], r["stroke_mm"]))
        pos[r["id"]] = (cx, cy)
        if r["path"] is not None:
            p = np.asarray(r["path"])
            p = p - (p.max(axis=0) + p.min(axis=0)) / 2
            p = p / max(np.abs(p).max(), 1e-12)
            px, py = cx + GLYPH * aspect * p[:, 0], cy + GLYPH * p[:, 1]
            ax.plot(px, py, color=c, lw=1.6, alpha=0.85,
                    transform=ax.transAxes, solid_capstyle="round", zorder=2)
            ax.plot(px[-1:], py[-1:], marker="_", ms=7, mew=1.8, color=c,
                    transform=ax.transAxes, zorder=2)
        ax.plot([cx], [cy], marker="o", ms=8, mew=1.6, mec=c,
                mfc=c if r["has_force"] else "white", transform=ax.transAxes,
                zorder=3, ls="none")
        if r["draft"]:
            ax.plot([cx], [cy], marker="o", ms=14, mew=1.0, mec=c, mfc="none",
                    transform=ax.transAxes, zorder=3, ls="none")
    # labels: to the right of each mark (left for marks near the right edge),
    # pushed apart up the page where they would collide, with a leader line
    GAP, WIDE = 0.019, 0.17
    placed = []
    for r in sorted(rows, key=lambda r: pos[r["id"]][1]):
        cx, cy = pos[r["id"]]
        left = cx > 0.8
        lx = cx - 0.035 if left else cx + 0.035
        ly = cy
        for qx, qy in sorted(placed, key=lambda q: q[1]):
            if abs(qx - lx) < WIDE and abs(qy - ly) < GAP:
                ly = qy + GAP
        placed.append((lx, ly))
        ax.plot([cx, lx], [cy, ly], color="#A0AEC0", lw=0.5,
                transform=ax.transAxes, zorder=1)
        ax.text(lx, ly, r["id"], transform=ax.transAxes, fontsize=7,
                color="#16233B", va="center", ha="right" if left else "left",
                bbox=dict(fc="white", ec="none", pad=0.6, alpha=0.8), zorder=4)
    if a.arrow:
        a_from, a_to = a.arrow.split(",")
        ax.annotate("", pos[a_to], pos[a_from], xycoords="axes fraction",
                    textcoords="axes fraction", zorder=5,
                    arrowprops=dict(arrowstyle="-|>", color="#B8531A", lw=2.2,
                                    shrinkA=9, shrinkB=9,
                                    connectionstyle="arc3,rad=-0.15"))
    ax.set_xlabel("share of tip motion that is sideways   "
                  "(0 = along its length, 1 = across it)", fontsize=13)
    ax.set_ylabel("free stroke, mm (log)", fontsize=13)
    ax.set_title("Where each actuator's tip goes: one mark per record, its "
                 "own tip path drawn through it", fontsize=15, loc="left")
    seen = sorted({r["physics"] for r in rows}, key=str)
    hands = [plt.Line2D([], [], marker="o", ls="none", ms=8, mec=COLOURS.get(p, OTHER),
                        mfc=COLOURS.get(p, OTHER), label=str(p)) for p in seen]
    hands += [plt.Line2D([], [], marker="o", ls="none", ms=8, mec="#16233B",
                         mfc="white", label="motion only (no force data)")]
    if a.drafts:
        hands += [plt.Line2D([], [], marker="o", ls="none", ms=14, mec="#16233B",
                             mfc="none", label="draft, not yet reviewed")]
    ax.legend(handles=hands, loc="lower right", fontsize=10, frameon=False)
    fig.text(0.01, 0.01, "Records only: comparative, not predictive. Tip path: "
             "up the page is away from the base, across is distance from the "
             "axis; each path is scaled to the same size.", fontsize=9,
             color="#4A5568")
    ax.grid(True, which="major", color="#D9D3C5", lw=0.6)
    fig.tight_layout(rect=(0, 0.025, 1, 1))
    os.makedirs(os.path.dirname(stem), exist_ok=True)
    fig.savefig(stem + ".png")
    with open(stem + ".json", "w", encoding="utf-8") as fo:
        json.dump({"made_by": "tools/figure_map_tip_paths.py", "drafts": a.drafts, "arrow": a.arrow or None,
                   "rows": rows, "left_out": left_out}, fo, indent=1)
    print("wrote", stem + ".png")
    print("wrote", stem + ".json")
    for lo_ in left_out:
        print("left out:", lo_["id"], "-", lo_["reason"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
