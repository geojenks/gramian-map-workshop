"""Check an actuator record and show what the workshop tool reads from it.

    python tools/check_record.py my_actuator.json [more.json ...]

1. Validates against the record schema (gramian_map's own validator,
   vendored unchanged in gramian_workshop/vendor/actuator_record.py).
2. Prints what the tool uses: push direction and force, motion direction
   and stroke, passive stiffness, each with the rule that produced it, so
   you can see whether the numbers say what you meant.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gramian_workshop.finger import Finger  # noqa: E402
from gramian_workshop.vendor import actuator_record as ar  # noqa: E402

RULES = {
    "force_N": "blocked.force_N (full force vector), highest input level",
    "axial_transverse": "blocked.force_axial_N + force_transverse_N "
                        "(transverse taken along +Y), highest input level",
    "polarity": "scalars.blocked_force_N along +X or -X (polarity_axial)",
    "none": "no force data: this actuator cannot push in the tool",
}


def vec(v):
    return "-" if v is None else "[" + ", ".join(f"{x:+.3f}" for x in v) + "]"


def check(path):
    rec = ar.load(path)
    errors, warnings = ar.validate(rec)
    print(f"\n== {path}")
    print(f"   id {rec.get('id')}  name {rec.get('name')!r}  tier "
          f"{ar.tier(rec) if not errors else '?'}")
    for e in errors:
        print(f"   ERROR  {e}")
    for w in warnings:
        print(f"   warn   {w}")
    if errors:
        print("   -> fix the errors above; the tool will not load this record")
        return False
    f = Finger(rec)
    print("   what the tool reads (canonical frame: X base->tip, +Y main "
          "sideways output):")
    print(f"     push direction  {vec(f.b)}   from {RULES[f.b_rule]}")
    print(f"     push force      "
          f"{'-' if f.F is None else f'{f.F:.4g} N'}   (comparative unless "
          "calibrated)")
    print(f"     motion          {vec(f.d)}   from scalars.free_direction")
    print(f"     stroke          "
          f"{'-' if f.S is None else f'{f.S * 1e3:.3g} mm'}")
    print(f"     stiffness       {vec(f.k)} N/m (x, y, z)   "
          + ("; ".join(f.k_notes) or "minimum over probed inputs"))
    if f.b is not None and f.d is not None:
        import numpy as np
        c = float(np.dot(f.b, f.d))
        print(f"     push vs motion  {np.degrees(np.arccos(max(-1, min(1, c)))):.0f}"
              " deg apart (a bender often pushes along X but moves along Y)")
    return True


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    ok = all([check(p) for p in sys.argv[1:]])
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
