"""Q1 pre-check (registration addendum section 13): records only, nothing
solved. Per record, the passive stiffness the addendum's rule reads (per-axis
minimum over probed levels, negatives and missing axes as 0) and the passive
force it gives at each deflection budget, beside the per-finger weight share.

    python tools/precheck_q1.py [--out outputs/precheck_q1]
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gramian_workshop.finger import all_fingers  # noqa: E402
from gramian_workshop.task import load_task  # noqa: E402

BUDGETS_MM = (1.0, 2.0, 4.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="outputs/precheck_q1")
    a = ap.parse_args()
    t = load_task("cup_hold")
    W = float(t["object"]["mass_kg"]) * float(t["gravity_m_s2"])
    rows = []
    lines = ["Q1 pre-check (records only; nothing solved)",
             f"weight {W:.4f} N; budgets {BUDGETS_MM} mm per axis", "",
             f"{'record':30s} {'k_x':>9s} {'k_y':>9s} {'k_z':>9s}   "
             + "  ".join(f"K*{b:g}mm (x,y,z) N" for b in BUDGETS_MM) + "   notes"]
    for f in all_fingers():
        cap = {str(b): [float(k) * b * 1e-3 for k in f.k] for b in BUDGETS_MM}
        rows.append({"id": f.id, "k_N_per_m": f.k.tolist(), "notes": f.k_notes,
                     "passive_N": cap, "F_N_comparative": f.F})
        lines.append(f"{f.id[:30]:30s} " + " ".join(f"{k:9.3f}" for k in f.k)
                     + "   " + "  ".join("(" + ",".join(f"{v:.4f}" for v in cap[str(b)])
                                         + ")" for b in BUDGETS_MM)
                     + "   " + "; ".join(f.k_notes))
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    with open(a.out + ".json", "w") as fo:
        json.dump({"weight_N": W, "budgets_mm": BUDGETS_MM, "records": rows},
                  fo, indent=1)
    txt = "\n".join(lines) + "\n"
    with open(a.out + ".txt", "w") as fo:
        fo.write(txt)
    print(txt)


if __name__ == "__main__":
    main()
