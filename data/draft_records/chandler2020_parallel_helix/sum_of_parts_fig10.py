"""Does the combined-input tip move (Fig 10 start dots) equal the sum of the single-chamber moves (Fig 7C) at the
same chamber volumes?  leftover = combined - (C1 alone + C2 alone + C3 alone), all in the paper frame (mm).

A single-chamber move is used only where Fig 7C resolves it: between that chamber's lowest and highest counted
marker volume (chandler2020_fig7C_a0_tip_C{n}.csv, status ok, V_ml set). Below that range the Fig 7C markers are
merged at the origin and we do NOT interpolate from zero. If any chamber is unresolved the leftover is reported as
not obtainable, with a bound that needs one extra assumption (stated in the output): a chamber's move magnitude
does not exceed its move at its lowest counted volume when its volume is lower still.

combined_minus_resolved_parts_mm is NOT the leftover when a chamber is unresolved: it still contains the
unresolved chambers' moves. A clean test would need a point where the inputs all sit in the resolved range,
e.g. one chamber at zero and the other two at 0.75 of peak (resolved for peaks of 5.5 ml and up); Fig 10 marks
only the cycle start with a dot and the traces carry no step markers, so such points cannot be read.

Writes chandler2020_fig10_sum_of_parts.csv. Run:  python -I sum_of_parts_fig10.py
"""
import csv
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def read(fname):
    with open(os.path.join(HERE, fname), encoding="utf-8") as f:
        return list(csv.DictReader(ln for ln in f if not ln.startswith("#")))


def single(ch):
    rows = [r for r in read(f"chandler2020_fig7C_a0_tip_C{ch}.csv") if r["status"] == "ok" and r["V_ml"]]
    rows.sort(key=lambda r: float(r["V_ml"]))
    V = np.array([float(r["V_ml"]) for r in rows])
    P = np.array([[float(r["x_mm"]), float(r["y_mm"]), float(r["z_mm"])] for r in rows])
    return V, P


def norm(v):
    return float(math.sqrt(sum(c * c for c in v)))


def main():
    sing = {ch: single(ch) for ch in (1, 2, 3)}
    for ch, (V, P) in sing.items():
        print(f"C{ch} alone resolved for {V.min():.1f}-{V.max():.1f} ml; move at lowest counted volume "
              f"{norm(P[0]):.1f} mm")
    out = []
    for d in read("chandler2020_fig10CD_start_dots.csv"):
        if d["status"] != "ok":
            continue
        comb = [float(d["X_mm"]), float(d["Y_mm"]), float(d["Z_mm"])]
        parts, unresolved, bound = [0.0, 0.0, 0.0], [], 0.0
        r = {"V_peak_ml": d["V_peak_ml"], "combined_x_mm": comb[0], "combined_y_mm": comb[1],
             "combined_z_mm": comb[2], "combined_move_mm": round(norm(comb), 2)}
        for ch in (1, 2, 3):
            v = float(d[f"V_C{ch}_ml"])
            V, P = sing[ch]
            if V.min() <= v <= V.max():
                p = [float(np.interp(v, V, P[:, k])) for k in range(3)]
                parts = [a + b for a, b in zip(parts, p)]
                r[f"C{ch}_alone_mm"] = " ".join(f"{c:.2f}" for c in p)
            else:
                unresolved.append(ch)
                bound += norm(P[0])
                r[f"C{ch}_alone_mm"] = f"not resolved at {v:.2f} ml (Fig 7C resolves {V.min():.1f}-{V.max():.1f})"
        rest = [a - b for a, b in zip(comb, parts)]
        if unresolved:
            lo, hi = max(0.0, norm(rest) - bound), norm(rest) + bound
            r.update({"leftover_mm": "not obtainable",
                      "leftover_fraction": "not obtainable",
                      "combined_minus_resolved_parts_mm": round(norm(rest), 2),
                      "leftover_bound_mm": f"{lo:.1f} to {hi:.1f} (assumes monotone single-chamber magnitude)",
                      "unresolved_chambers": " ".join(f"C{c}" for c in unresolved)})
        else:
            r.update({"leftover_mm": " ".join(f"{c:.2f}" for c in rest),
                      "leftover_fraction": round(norm(rest) / norm(comb), 3),
                      "combined_minus_resolved_parts_mm": round(norm(rest), 2),
                      "leftover_bound_mm": "", "unresolved_chambers": ""})
        out.append(r)
        print(f"V {r['V_peak_ml']} ml: combined {r['combined_move_mm']} mm; C1 {r['C1_alone_mm']}; "
              f"C2 {r['C2_alone_mm']}; C3 {r['C3_alone_mm']}; leftover {r['leftover_mm']}; "
              f"combined minus resolved parts {r['combined_minus_resolved_parts_mm']} mm; bound {r['leftover_bound_mm']}")
    cols = list(out[0].keys())
    p = os.path.join(HERE, "chandler2020_fig10_sum_of_parts.csv")
    with open(p, "w", newline="", encoding="utf-8") as f:
        f.write("# Sum-of-parts test: Fig 10 start dots (combined inputs) vs Fig 7C single-chamber moves. Paper frame, mm.\n"
                "# Assumes Fig 7 and Fig 10 share one frame (same axis labels and orientation). Fig 7 is a 1.6 ml/s\n"
                "# up-sweep; Fig 10 is quasi-static (2 s settle), so rate effects are mixed into any leftover.\n")
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(out)
    print("wrote", p)


if __name__ == "__main__":
    main()
