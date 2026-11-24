"""Your actuator as a formula: write its test curves from an analytical model.

    python examples/analytical_model_to_curves.py --out my_model_curves.csv
    python tools/curves_to_record.py my_model_curves.csv --out my_model.json
    python tools/check_record.py my_model.json

The file it writes is the plain curve file of docs/TEST_CURVE_FORMAT.md, the
same file a bench test or a simulation would fill. To use your own model,
replace the two functions `free_tip_displacement_m` and `blocked_force_N`
with your formulas and set the details in `DETAILS`. Nothing else changes.

The model here is a textbook one, kept short on purpose: a bending finger of
length L with one pressure chamber offset from its stiff layer.

  bending moment        M = p * A * e        (pressure, chamber area, offset)
  free shape            constant curvature   kappa = M / EI
  blocked force         the tip is held sideways and the base is clamped:
                        F = 3 M / (2 L), pushing towards the side it bends to

The numbers below are for illustration and describe no real actuator.
A model is not a measurement: the record says `simulation` and names the
model in its `detail` line.
"""
import argparse
import math
import os
import sys

# --- the model's numbers: replace with your own ------------------------------
LENGTH_M = 0.100            # L, base to tip
CHAMBER_AREA_M2 = 1.0e-4    # A, cross-section the pressure acts on
OFFSET_M = 0.005            # e, chamber centre to the stiff layer
BENDING_STIFFNESS_NM2 = 0.002   # EI
PRESSURES_KPA = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]

DETAILS = [
    ("id", "example_constant_curvature_bender"),
    ("name", "Example: constant-curvature bending finger (analytical model)"),
    ("source", "simulation"),
    ("detail", "analytical model, not a measurement: constant curvature "
               "kappa = p A e / EI; blocked force 3 p A e / (2 L); "
               "examples/analytical_model_to_curves.py"),
    ("actuation", "pneumatic"),
    ("input_name", "chamber pressure"),
    ("motion", "bends"),
    ("blocked_method", "analytical: clamped base, tip held sideways"),
    ("notes", "Illustrative numbers; describes no real actuator."),
]


def moment_Nm(pressure_Pa):
    return pressure_Pa * CHAMBER_AREA_M2 * OFFSET_M


def free_tip_displacement_m(pressure_Pa):
    """Tip displacement from rest (dx, dy) in the actuator's frame: X from
    base to tip, Y the side it bends to."""
    kappa = moment_Nm(pressure_Pa) / BENDING_STIFFNESS_NM2
    if abs(kappa) < 1e-12:
        return 0.0, 0.0
    x = math.sin(kappa * LENGTH_M) / kappa
    y = (1.0 - math.cos(kappa * LENGTH_M)) / kappa
    return x - LENGTH_M, y


def blocked_force_N(pressure_Pa):
    """Force the held tip makes (fx, fy), same frame."""
    return 0.0, 3.0 * moment_Nm(pressure_Pa) / (2.0 * LENGTH_M)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default="my_model_curves.csv")
    args = ap.parse_args()
    if os.path.exists(args.out):
        sys.exit(f"{args.out} exists; choose another --out (nothing is overwritten)")

    lines = ["# Test curves from an analytical model: docs/TEST_CURVE_FORMAT.md. "
             "Written by examples/analytical_model_to_curves.py."]
    for key, value in DETAILS:
        lines.append(f'{key},"{value}"' if "," in value else f"{key},{value}")
    lines.append("")
    lines.append("curve,input_kPa,tip_dx_mm,tip_dy_mm,force_x_N,force_y_N")
    for p_kPa in PRESSURES_KPA:
        dx, dy = free_tip_displacement_m(p_kPa * 1e3)
        lines.append(f"free,{p_kPa:g},{dx * 1e3:.4f},{dy * 1e3:.4f},,")
    for p_kPa in PRESSURES_KPA:
        fx, fy = blocked_force_N(p_kPa * 1e3)
        lines.append(f"blocked,{p_kPa:g},,,{fx:.5f},{fy:.5f}")
    with open(args.out, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {args.out}: {len(PRESSURES_KPA)} free and "
          f"{len(PRESSURES_KPA)} blocked rows")


if __name__ == "__main__":
    main()
