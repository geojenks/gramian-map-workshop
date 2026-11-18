"""build_yang2016_records.py -- DRAFT v0.2 literature records for the two
vacuum-actuated muscle-inspired pneumatic actuators (VAMPs) of Yang et al.
2016 (Adv. Mater. Technol., doi:10.1002/admt.201600055), built only from the
digitised Fig 2 CSVs and the sourced text-values CSV in this folder (no
hand-typed numbers).

Pattern: follows ../kubler2023_vine_actuators/build_kubler2023_records.py --
provenance block with CSV hashes, data_completeness "curves_only", kPa->Pa
conversion here, one raw CSV row asserted to round-trip, validation before
any write, refuse to overwrite.

Writes into THIS folder only:
  yang2016_vamp_ecoflex.json, yang2016_vamp_elastosil.json   the draft records
  yang2016_fig2_reading_error.json                            reading error

Input convention (as data/records/abboodi2026_ossa5_vacuum_contract.json)
-------------------------------------------------------------------------
The paper's input is the differential pressure dP = P_ext - P_int (vacuum
depth, positive). The record stores u = -dP in Pa (negative gauge pressure),
ordered by increasing vacuum. Free motion is contraction along the length:
tip displacement [dL, 0, 0] with dL < 0, read off the figure's top axis
(L(dP,0) - L(0,0), mm). polarity_axial = -1.

Blocked force: null in both records
-----------------------------------
The paper gives no blocked-force value in the text. Fig 2C/D hang weights on
the actuator at dP = 90 kPa (caption). The paper's strain under load is
s = (L(dP,sigma) - L(0,sigma)) / L(0,0) (Eq 3): zero means the vacuum makes
no change to the length the actuator has under that same load, which is
stretched from rest by an amount this copy does not give. The schema's
blocked block means the port held at rest (x ~= 0), so no point fits. The
heaviest load, and whether its strain is zero within the published SD plus
our reading error, goes into the notes and the reading-error JSON only;
nothing is extrapolated.

Run (from anywhere):
    python -I build_yang2016_records.py            # writes + validates
    python -I build_yang2016_records.py --dry-run  # build + assert only
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
MM_TO_M = 1e-3
DEFAULT_UNCERTAINTY_FRACTION = 0.1   # as the Mosadegh and Kubler records
READ_PX = 1.0     # pick-off error of a marker centre, px (markers are 13-14 px)
MOVE_FRACTION = 0.02   # free_direction: first level moving > 2% of the stroke

DATA_DIR = "data/draft_records/yang2016_vacuum_buckling"
DIGITIZER = "digitize_yang2016_fig2.py"
TEXT_CSV = "yang2016_text_values.csv"
CAL_JSON = "yang2016_fig2_calibration.json"
REFERENCE = ("Harvard DASH accepted manuscript, "
             "http://nrs.harvard.edu/urn-3:HUL.InstRepos:29951794")
CITATION = ("Yang, D., Verma, M. S., So, J.-H., Mosadegh, B., Keplinger, C., "
            "Lee, B., Khashai, F., Lossner, E., Suo, Z. and Whitesides, G. M. "
            "\"Buckling Pneumatic Linear Actuators Inspired by Muscle.\" "
            "Advanced Materials Technologies 1(3), 1600055, 2016. "
            "doi:10.1002/admt.201600055. Digitised from the Harvard DASH "
            "accepted manuscript (" + REFERENCE.split(", ")[1] + "), Fig 2; "
            "the published version was not checked.")

TYPES = {
    "ecoflex": dict(label="Ecoflex 00-30", e_key="E_ecoflex",
                    free_panel="A", load_panel="D"),
    "elastosil": dict(label="Elastosil M4601", e_key="E_elastosil",
                      free_panel="B", load_panel="C"),
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


def fnum(s):
    return None if s in (None, "") else float(s)


def text_values():
    rows = read_csv_rows(os.path.join(HERE, TEXT_CSV))
    return {r["quantity"]: (float(r["value"]), r["unit"]) for r in rows}


def assert_anchor(label, expected, actual, tol=1e-9):
    if abs(expected - actual) > tol * max(1.0, abs(expected)):
        raise AssertionError(f"anchor mismatch [{label}]: expected "
                             f"{expected!r}, got {actual!r}")
    print(f"  anchor OK [{label}]: {actual!r}")


# ---------------------------------------------------------- reading error

def reading_error():
    """Per panel and axis: sqrt(calibration max residual^2 + READ_PX^2) x
    |units per px|. Written to yang2016_fig2_reading_error.json."""
    with open(os.path.join(HERE, CAL_JSON), encoding="utf-8") as f:
        cal = json.load(f)
    out = {"method": (
        f"per-point reading error on each axis = sqrt(axis calibration max "
        f"residual^2 + {READ_PX} px pick-off^2) x |axis units per px|. Axis "
        f"units: bottom = strain, top = mm, left = kPa, right = N. This is "
        f"our reading error only; the paper's sample-to-sample SD (n = 7) is "
        f"in the CSV column strain_sd and is larger."),
        "panels": {}}
    for key, c in cal["panels"].items():
        ent = {}
        for side, a in c["axes"].items():
            upp = abs(a["slope_per_px"])
            ent[f"{side}_units_per_px"] = upp
            ent[f"{side}_reading_error"] = math.hypot(a["max_residual_px"],
                                                      READ_PX) * upp
        out["panels"][key] = ent
    return out


# --------------------------------------------------------------- builder

def build(typ, rerr, tv):
    meta = TYPES[typ]
    pf, pl = meta["free_panel"], meta["load_panel"]
    free_csv = f"yang2016_fig2{pf}_free_{typ}.csv"
    load_csv = f"yang2016_fig2{pl}_load_{typ}.csv"
    free = sorted(read_csv_rows(os.path.join(HERE, free_csv)),
                  key=lambda r: float(r["dP_kPa"]))
    load = sorted(read_csv_rows(os.path.join(HERE, load_csv)),
                  key=lambda r: float(r["T_N"]))

    length_mm, u_len = tv["device_length"]
    width_mm, _ = tv["device_width"]
    depth_mm, _ = tv["device_depth"]
    assert u_len == "mm"
    dp_load_kpa, u_dp = tv["load_test_dP"]
    assert u_dp == "kPa"
    e_kpa, _ = tv[meta["e_key"]]
    n_samples = int(tv["n_samples_static"][0])

    free_u = [[-float(r["dP_kPa"]) * KPA_TO_PA] for r in free]
    free_disp = [[float(r["dL_mm"]) * MM_TO_M, 0.0, 0.0] for r in free]
    norms = [abs(d[0]) for d in free_disp]
    stroke = max(norms)
    i_move = next(i for i, n in enumerate(norms) if n > MOVE_FRACTION * stroke)
    free_dir = [math.copysign(1.0, free_disp[i_move][0]), 0.0, 0.0]

    # ---- load test: heaviest hung load at dP = 90 kPa. Not a blocked force
    # in the schema's sense (port held at rest, x ~= 0): the paper's strain
    # is measured from the passively loaded length L(0,T), whose stretch is
    # not in this copy. Recorded in the notes only; blocked stays null.
    eL = rerr["panels"][pl]
    heavy = load[-1]
    s_h = float(heavy["strain"])
    status = heavy["sd_status"]
    if status == "read":
        sd_h, sd_src = float(heavy["strain_sd"]), "published SD"
    elif status == "hidden_in_marker":
        sd_h = float(heavy["sd_upper_bound_strain"])
        sd_src = "SD bar hidden in the marker; half a marker used as its bound"
    else:
        sd_h, sd_src = 0.0, "SD unreadable; reading error only"
    tol_h = sd_h + eL["bottom_reading_error"]
    T_h = float(heavy["T_N"])
    at_zero = abs(s_h) <= tol_h
    rerr["panels"][pl]["heaviest_load_test"] = {
        "panel": pl, "heaviest_load_T_N": T_h, "strain_at_heaviest": s_h,
        "strain_tolerance": tol_h, "tolerance_source": sd_src,
        "actuation_strain_zero_within_tolerance": at_zero,
        "used_as_blocked_force": False}

    e_mm = rerr["panels"][pf]["top_reading_error"]
    e_kpa_rd = rerr["panels"][pf]["left_reading_error"]
    e_T = eL["right_reading_error"]
    plateau = [float(r["dL_mm"]) for r in free]
    nonmono = any(b > a for a, b in zip(plateau, plateau[1:]))

    blocked = None
    blocked_force = None
    blocked_txt = (
        f"Blocked force: none. The paper quotes no blocked force for these "
        f"devices. Fig 2{pl} hangs weights T on the actuator at dP = "
        f"{dp_load_kpa:g} kPa (Fig 2 caption) and reports the actuation strain "
        f"(L(dP,T) - L(0,T))/L(0,0) (paper Eq 3), measured from the length "
        f"under the same load with no vacuum, not from the rest length; the "
        f"passive stretch L(0,T) - L(0,0) is not given in this copy (the SI is "
        f"missing). So no point of Fig 2{pl} has the tip held at its rest "
        f"position, and blocked is left null. ")
    if at_zero:
        blocked_txt += (
            f"At the heaviest load, T = {T_h:.3f} N, the actuation strain is "
            f"{s_h:+.4f}, zero within SD plus reading error ({tol_h:.4f}): "
            f"this is the largest load the vacuum can still lift at "
            f"{dp_load_kpa:g} kPa (the text, printed page 7, calls it the "
            f"maximum loading stress). It is not the force with the tip held "
            f"at rest length, which is likely lower. ")
    else:
        blocked_txt += (
            f"At the heaviest load, T = {T_h:.3f} N, the actuation strain is "
            f"still {s_h:+.4f} (beyond SD plus reading error, {tol_h:.4f}): "
            f"the vacuum still lifts it, so the largest liftable load at "
            f"{dp_load_kpa:g} kPa is above {T_h:.3f} N and was not reached. "
            f"How this relates to the force with the tip held at rest length "
            f"is unknown. ")
    blocked_txt += (f"The load-strain curve (CSV {load_csv}) has no place in "
                    f"schema v0.2 and is kept in the CSV only. ")

    notes = (
        f"DRAFT -- not reviewed, not in the manifest. Vacuum-actuated "
        f"muscle-inspired pneumatic actuator (VAMP) cast in {meta['label']} "
        f"(E = {e_kpa:g} kPa, paper text): an elastomer block of "
        f"{width_mm:g} x {depth_mm:g} x {length_mm:g} mm with rows of "
        f"internal cavities whose beams buckle under vacuum, so the actuator "
        f"contracts along its length. Input: the paper's differential "
        f"pressure dP = P_ext - P_int; stored as u = -dP (Pa, negative gauge "
        f"pressure, as the abboodi2026 vacuum-contract record). Free motion: "
        f"Fig 2{pf}, no load, mean of n = {n_samples} samples; tip displacement "
        f"= L(dP,0) - L(0,0) read off the figure's top axis (mm), along -X "
        f"(frame X base->tip; the base is taken as fixed and the free end as "
        f"the tip). The first level is the digitised rest point as read, not "
        f"forced to zero. "
        + ("The digitised curve is not monotonic on the plateau (kept as read, "
           "not smoothed). " if nonmono else "")
        + f"free_direction is the first level that moves more than "
        f"{MOVE_FRACTION * 100:.0f}% of the stroke. "
        + blocked_txt
        + f"Reading error (yang2016_fig2_reading_error.json): about "
        f"{e_mm:.3f} mm on displacement and {e_kpa_rd:.4f} kPa on dP; "
        f"{e_T:.4f} N on load. The paper's SD between samples (CSV column "
        f"strain_sd) is larger. No stiffness data.")

    rec = {
        "schema_version": "0.2",
        "id": f"yang2016_vamp_{typ}",
        "name": f"DRAFT Yang 2016 VAMP {meta['label']}",
        "source": {
            "kind": "literature",
            "detail": (f"digitized: Yang et al. 2016 (doi:10.1002/admt.201600055) "
                       f"Fig 2{pf} quasi-static free contraction vs vacuum + "
                       f"Fig 2{pl} hung-weight loading at dP {dp_load_kpa:g} kPa, "
                       f"{meta['label']}"),
            "date": "2026-11-12",
            "reference": REFERENCE,
        },
        "metadata": {
            "actuation_physics": "pneumatic",
            "length_m": length_mm * MM_TO_M,
            "mass_kg": None,
            "material_volume_m3": None,
            "materials": {},
            "notes": notes,
        },
        "input": {
            "channels": [
                {"name": "chamber vacuum", "unit": "Pa",
                 "range": [min([free_u[-1][0]]
                               + ([blocked["u"][0][0]] if blocked else [])),
                           0.0],
                 "rated": None},
            ],
            "energy_convention": "mechanical work delivered at the input port (J)",
            "work_J": None,
        },
        "port": {"name": "tip", "description":
                 "free end of the actuator (end face opposite the fixed base)"},
        "free": {"u": free_u, "tip_displacement_m": free_disp,
                 "uncertainty": None},
        "blocked": blocked,
        "stiffness": None,
        "scalars": {
            "free_stroke_m": stroke,
            "free_direction": free_dir,
            "polarity_axial": -1,
            "blocked_force_N": blocked_force,
            "stiffness_N_per_m": None,
            "effective_dof": None,
        },
        "shape": None,
        "uncertainty": {"default_fraction": DEFAULT_UNCERTAINTY_FRACTION},
        "data_completeness": "curves_only",
        "provenance": {
            "data_dir": DATA_DIR,
            "digitizer_script": DIGITIZER,
            "source_csvs": [csv_ref(free_csv), csv_ref(load_csv),
                            csv_ref(TEXT_CSV)],
            "citation": CITATION,
        },
    }
    return rec, free, load


def main():
    dry = "--dry-run" in sys.argv
    tv = text_values()
    rerr = reading_error()
    built = []
    for typ in TYPES:
        rec, free, load = build(typ, rerr, tv)
        built.append(rec)
        a = free[-1]
        i = rec["free"]["u"].index([-float(a["dP_kPa"]) * KPA_TO_PA])
        assert_anchor(f"{typ} free dL @ {a['dP_kPa']} kPa",
                      float(a["dL_mm"]) * MM_TO_M,
                      rec["free"]["tip_displacement_m"][i][0])
        assert rec["blocked"] is None and rec["scalars"]["blocked_force_N"] is None
        s = rec["scalars"]
        bt = rerr["panels"][TYPES[typ]["load_panel"]]["heaviest_load_test"]
        assert f"T = {float(load[-1]['T_N']):.3f} N" in rec["metadata"]["notes"]
        print(f"  {rec['id']}: {len(rec['free']['u'])} free levels, stroke "
              f"{s['free_stroke_m'] * 1000:.2f} mm, dir {s['free_direction']}, "
              f"heaviest load {bt['heaviest_load_T_N']:.3f} N at strain "
              f"{bt['strain_at_heaviest']:+.4f} (tol {bt['strain_tolerance']:.4f},"
              f" zero within tol: {bt['actuation_strain_zero_within_tolerance']})"
              f" -> blocked null")

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
            f"{kk} {vv:.4g}" for kk, vv in v.items()
            if kk.endswith("reading_error")))
    if dry:
        print("--dry-run: nothing written")
        return 0

    targets = [os.path.join(HERE, f"{r['id']}.json") for r in built]
    targets.append(os.path.join(HERE, "yang2016_fig2_reading_error.json"))
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
