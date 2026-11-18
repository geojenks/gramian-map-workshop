"""build_kubler2023_records.py -- DRAFT v0.2 literature records for the three
40 mm vine-robot actuators of Kubler et al. 2023 (arXiv:2305.00967v3), built
only from the digitised Fig 7 CSVs in this folder (no hand-typed numbers).

Pattern: follows gramian_map/build_record_from_digitized.py (Mosadegh 2014
records) -- provenance block with CSV hashes, data_completeness
"curves_only", kPa->Pa conversion here, one raw CSV row asserted to
round-trip, validation before any write, refuse to overwrite.

Writes into THIS folder only:
  kubler2023_pouch_W40xL40.json, kubler2023_cPAM_W40xL40.json,
  kubler2023_fPAM_W40.json             the draft records
  kubler2023_fig7_reading_error.json   reading-error estimate per panel

Free-motion conversion (stated in every record's notes)
------------------------------------------------------
The paper reports bending angle per nominal length, b (deg/cm), measured
from markers along the actuated part. We assume the actuated length bends
as one circular arc of constant curvature b over L_ACTUATED_M, with an
inextensible centreline, and the rest of the vine body (the remaining
60 mm of the 420 mm body) ignored. Then the total turn is
    theta = b * (L_ACTUATED_M in cm)       (converted to radians)
and the displacement of the arc's end from its straight position, in the
canonical frame (X base->tip, bending toward +Y), is
    dx = L (sin(theta)/theta - 1),   dy = L (1 - cos(theta))/theta.

Run (from anywhere):
    python -I build_kubler2023_records.py            # writes + validates
    python -I build_kubler2023_records.py --dry-run  # build + assert only
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WORKSHOP = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, WORKSHOP)
from gramian_workshop.vendor import actuator_record as ar  # noqa: E402

KPA_TO_PA = 1000.0
# Kubler et al. 2023 Section 3 and Table 1: L_vine = 420 mm, actuator line
# length L_actuated = 360 mm, D_vine = 80 mm. Only L_actuated enters the
# conversion; the other two go into the notes text.
L_ACTUATED_M = 0.360
L_VINE_MM, D_VINE_MM = 420, 80
VINE_PRESSURE_KPA = 1.75          # Section 4: vine body pressure in all tests
DEFAULT_UNCERTAINTY_FRACTION = 0.1   # as the Mosadegh records
READ_PX = 1.0     # pick-off error of a line centre, px (lines are 5-7 px)

DATA_DIR = "data/draft_records/kubler2023_vine_actuators"
DIGITIZER = "digitize_kubler2023_fig7.py"
CITATION = ("Kubler, A. M., du Pasquier, C., Low, A., Djambazi, B., Aymon, N., "
            "Forster, J., Agharese, N., Siegwart, R. and Okamura, A. M. "
            "\"A Comparison of Pneumatic Actuators for Soft Growing Vine "
            "Robots.\" Soft Robotics 11(5), 857-868, 2024. "
            "doi:10.1089/soro.2023.0169 (journal version not checked). "
            "Digitised from the arXiv preprint arXiv:2305.00967v3 "
            "(15 Sep 2023), Fig 7.")

TYPES = {
    "pouch_W40xL40": dict(
        name="pouch motor W40 x L40",
        materials="70 denier ripstop nylon with one-sided TPU coating; a line "
                  "of welded rectangular pouches taped onto the vine body"),
    "cPAM_W40xL40": dict(
        name="cPAM W40 x L40",
        materials="70 denier ripstop nylon with one-sided TPU coating; "
                  "cylindrical pneumatic artificial muscles welded into the "
                  "vine body"),
    "fPAM_W40": dict(
        name="fPAM W40",
        materials="30 denier ripstop nylon with two-sided silicone coating, "
                  "pre-stretched, one cylindrical tube glued along the vine "
                  "body with SilPoxy (fPAMs vary by width only)"),
}


# --------------------------------------------------------------- utilities

def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def read_csv_rows(path):
    with open(path, encoding="utf-8") as f:
        lines = [ln for ln in f if not ln.lstrip().startswith("#")]
    return list(csv.DictReader(lines))


def csv_ref(filename):
    return {"file": filename, "sha256": sha256_file(os.path.join(HERE, filename))}


def unit_direction(p0, p1):
    v = [b - a for a, b in zip(p0, p1)]
    n = math.sqrt(sum(c * c for c in v))
    if n == 0.0:
        return [0.0, 0.0, 0.0]
    return [c / n for c in v]


def assert_anchor(label, expected, actual, tol=1e-9):
    if abs(expected - actual) > tol * max(1.0, abs(expected)):
        raise AssertionError(f"anchor mismatch [{label}]: expected "
                             f"{expected!r}, got {actual!r}")
    print(f"  anchor OK [{label}]: {actual!r}")


def arc_tip(bend_deg_per_cm, length_m=L_ACTUATED_M):
    """Constant-curvature arc: tip displacement [dx, dy, 0] (m)."""
    theta = math.radians(bend_deg_per_cm * length_m * 100.0)
    if theta == 0.0:
        return [0.0, 0.0, 0.0]
    return [length_m * (math.sin(theta) / theta - 1.0),
            length_m * (1.0 - math.cos(theta)) / theta, 0.0]


# ---------------------------------------------------------- reading error

def reading_error():
    """Per panel: calibration residual and pick-off error in data units,
    plus (force) the largest disagreement between bar centre and the dashed
    mean line. Written to kubler2023_fig7_reading_error.json."""
    with open(os.path.join(HERE, "kubler2023_fig7_calibration.json"),
              encoding="utf-8") as f:
        cal = json.load(f)
    out = {"method": (
        f"per-point reading error = sqrt(y calibration max residual^2 + "
        f"{READ_PX} px pick-off^2) x |units per px|; for force panels the "
        f"largest |line_check_N| (bar centre vs dashed mean line) is also "
        f"given as an empirical check"), "panels": {}}
    for key, c in cal["panels"].items():
        upp = abs(c["y_fit_unit_per_px"])
        kpp = abs(c["x_fit_kPa_per_px"])
        e_px = math.hypot(c["y_max_residual_px"], READ_PX)
        ex_px = math.hypot(c["x_max_residual_px"], READ_PX)
        row, typ = key.split("_", 1)
        unit = "deg_per_cm" if row == "bend" else "N"
        ent = {"units_per_px": upp, "kPa_per_px": kpp,
               f"reading_error_{unit}": e_px * upp,
               "reading_error_kPa": ex_px * kpp}
        if row == "force":
            rows = read_csv_rows(os.path.join(HERE, f"kubler2023_fig7_force_{typ}.csv"))
            lc = [abs(float(r["line_check_N"])) for r in rows if r["line_check_N"]]
            ent["max_abs_line_check_N"] = max(lc) if lc else None
        if row == "bend":
            ent["reading_error_tip_y_m_at_end"] = None   # filled below
        out["panels"][key] = ent
    return out


# --------------------------------------------------------------- builder

def build(typ, rerr):
    meta = TYPES[typ]
    bend_csv = f"kubler2023_fig7_bend_{typ}.csv"
    force_csv = f"kubler2023_fig7_force_{typ}.csv"
    bend = sorted(read_csv_rows(os.path.join(HERE, bend_csv)),
                  key=lambda r: float(r["pressure_kPa"]))
    force = sorted(read_csv_rows(os.path.join(HERE, force_csv)),
                   key=lambda r: float(r["pressure_kPa"]))

    free_u = [[float(r["pressure_kPa"]) * KPA_TO_PA] for r in bend]
    free_disp = [arc_tip(float(r["bend_deg_per_cm"])) for r in bend]
    blocked_u = [[float(r["pressure_kPa"]) * KPA_TO_PA] for r in force]
    blocked_f = [float(r["force_N"]) for r in force]

    # Small-signal direction: from rest (zero displacement at zero pressure,
    # by definition of the reference pose) to the lowest digitised level.
    free_dir = unit_direction([0.0, 0.0, 0.0], free_disp[0])
    assert free_dir != [0.0, 0.0, 0.0], typ
    stroke = math.sqrt(sum(c * c for c in free_disp[-1]))

    b_end = float(bend[-1]["bend_deg_per_cm"])
    theta_end = b_end * L_ACTUATED_M * 100.0
    e_b = rerr["panels"][f"bend_{typ}"]["reading_error_deg_per_cm"]
    dy_err = abs(arc_tip(b_end + e_b)[1] - arc_tip(b_end)[1])
    rerr["panels"][f"bend_{typ}"]["reading_error_tip_y_m_at_end"] = dy_err
    e_f = rerr["panels"][f"force_{typ}"]["reading_error_N"]

    notes = (
        f"DRAFT -- not reviewed, not in the manifest. {meta['name']} mm actuator "
        f"line on a soft growing (everting) vine robot. Measured MOUNTED ON AN "
        f"INFLATED VINE BODY (diameter {D_VINE_MM} mm, body length "
        f"{L_VINE_MM} mm, body pressure {VINE_PRESSURE_KPA} kPa), not as a "
        f"free-standing actuator: the curves describe actuator plus inflated "
        f"beam. Materials: {meta['materials']}. "
        f"Free motion: the paper reports bending angle per nominal length "
        f"(deg/cm; data mean of five runs, horizontal configuration, Fig 7 "
        f"top row). Converted here to tip displacement ASSUMING one circular "
        f"arc of constant curvature over the actuated length "
        f"L = {L_ACTUATED_M * 1000:.0f} mm (paper Table 1), inextensible "
        f"centreline, the remaining {L_VINE_MM - L_ACTUATED_M * 1000:.0f} mm "
        f"of body ignored. That unactuated part is at the tip end (paper Fig 3 "
        f"side view), so the port here is the end of the actuated length: the "
        f"vine's physical tip lies a further "
        f"{L_VINE_MM - L_ACTUATED_M * 1000:.0f} mm along the final tangent "
        f"and moves more than this port, and the force sensor held that "
        f"physical tip. Frame X base->tip, bending toward +Y. At the last "
        f"level ({float(bend[-1]['pressure_kPa']):.1f} kPa) b = {b_end:.3f} "
        f"deg/cm, i.e. {theta_end:.1f} deg total turn. Free levels start at "
        f"{float(bend[0]['pressure_kPa']):.0f} kPa: below that the curve "
        f"runs under other series and could not be traced. free_direction is "
        f"rest -> lowest digitised level. "
        f"Blocked force: the front (tip) of the vine robot was fixed to an "
        f"ATI Nano-17 six-axis force-torque sensor and the lateral force "
        f"(perpendicular to the growing axis) reported, mean of five runs at "
        f"ten pressures (Fig 7 bottom row); recorded as force_transverse_N. "
        f"The paper does not report the axial component, so force_axial_N is "
        f"null; the workshop loader then reads no push force from this "
        f"record (same as the Mosadegh records). "
        f"Reading error (kubler2023_fig7_reading_error.json): about "
        f"{e_b:.4f} deg/cm on bend (about {dy_err * 1000:.2f} mm of tip Y at "
        f"the last level) and {e_f:.4f} N on force; the paper's own run-to-run "
        f"SD is larger (CSV columns band_lo/band_hi and force_sd_N). "
        f"Polarity_axial null: bender, not an axial actuator.")

    rec = {
        "schema_version": "0.2",
        "id": f"kubler2023_{typ}",
        "name": f"DRAFT Kubler 2023 vine robot {meta['name']}",
        "source": {
            "kind": "literature",
            "detail": (f"digitized: Kubler et al. 2023 (arXiv:2305.00967v3) "
                       f"Fig 7, {meta['name']}: top row quasi-static bending "
                       f"(data mean) + bottom row lateral force with tip fixed"),
            "date": "2026-11-12",
            "reference": "arXiv:2305.00967v3",
        },
        "metadata": {
            "actuation_physics": "pneumatic",
            "length_m": L_ACTUATED_M,
            "mass_kg": None,
            "material_volume_m3": None,
            "materials": {},
            "notes": notes,
        },
        "input": {
            "channels": [
                {"name": "actuator pressure", "unit": "Pa",
                 "range": [0.0, max(free_u[-1][0], blocked_u[-1][0])],
                 "rated": None},
            ],
            "energy_convention": "mechanical work delivered at the input port (J)",
            "work_J": None,
        },
        "port": {"name": "tip", "description":
                 "end of the actuated length (vine robot tip region)"},
        "free": {"u": free_u, "tip_displacement_m": free_disp,
                 "uncertainty": None},
        "blocked": {
            "u": blocked_u,
            "force_axial_N": None,
            "force_transverse_N": blocked_f,
            "method": ("digitized Kubler 2023 Fig 7 bottom row: front of the "
                       "vine robot fixed to an ATI Nano-17 6-axis force-torque "
                       "sensor, lateral force (perpendicular to the growing "
                       "axis) reported; actuator mounted on the inflated vine "
                       f"body at {VINE_PRESSURE_KPA} kPa; mean of five runs"),
            "uncertainty": None,
        },
        "stiffness": None,
        "scalars": {
            "free_stroke_m": stroke,
            "free_direction": free_dir,
            "polarity_axial": None,
            "blocked_force_N": max(abs(f) for f in blocked_f),
            "stiffness_N_per_m": None,
            "effective_dof": None,
        },
        "shape": None,
        "uncertainty": {"default_fraction": DEFAULT_UNCERTAINTY_FRACTION},
        "data_completeness": "curves_only",
        "provenance": {
            "data_dir": DATA_DIR,
            "digitizer_script": DIGITIZER,
            "source_csvs": [csv_ref(bend_csv), csv_ref(force_csv)],
            "citation": CITATION,
        },
    }
    return rec, bend, force


def main():
    dry = "--dry-run" in sys.argv
    rerr = reading_error()
    built = []
    for typ in TYPES:
        rec, bend, force = build(typ, rerr)
        built.append(rec)
        a = force[-1]
        i = rec["blocked"]["u"].index([float(a["pressure_kPa"]) * KPA_TO_PA])
        assert_anchor(f"{typ} blocked @ {a['pressure_kPa']} kPa",
                      float(a["force_N"]), rec["blocked"]["force_transverse_N"][i])
        a = bend[-1]
        i = rec["free"]["u"].index([float(a["pressure_kPa"]) * KPA_TO_PA])
        assert_anchor(f"{typ} free tip y @ {a['pressure_kPa']} kPa",
                      arc_tip(float(a["bend_deg_per_cm"]))[1],
                      rec["free"]["tip_displacement_m"][i][1])
        s = rec["scalars"]
        print(f"  {rec['id']}: {len(rec['free']['u'])} free levels, stroke "
              f"{s['free_stroke_m'] * 1000:.1f} mm, dir "
              f"[{', '.join(f'{c:+.3f}' for c in s['free_direction'])}], "
              f"{len(rec['blocked']['u'])} blocked levels, max force "
              f"{s['blocked_force_N']:.3f} N")

    n_err = 0
    for rec in built:
        errors, warnings = ar.validate(rec)
        for e in errors:
            print(f"{rec['id']}: ERROR: {e}")
        for w in warnings:
            print(f"{rec['id']}: warning: {w}")
        n_err += len(errors)
    if n_err:
        print(f"{n_err} validation errors -- refusing to write")
        return 1
    print("all records validate")
    for k, v in rerr["panels"].items():
        print(f"  reading error {k}: " + ", ".join(
            f"{kk} {vv:.4g}" for kk, vv in v.items() if vv is not None))
    if dry:
        print("--dry-run: nothing written")
        return 0

    targets = [os.path.join(HERE, f"{r['id']}.json") for r in built]
    targets.append(os.path.join(HERE, "kubler2023_fig7_reading_error.json"))
    clash = [t for t in targets if os.path.exists(t)]
    if clash:
        print("REFUSING to overwrite: " + ", ".join(clash))
        return 1
    with open(targets[-1], "w", encoding="utf-8") as f:
        json.dump(rerr, f, indent=1)
    print(f"wrote {targets[-1]}")
    for rec, t in zip(built, targets):
        ar.save(rec, t)
        print(f"wrote {t}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
