"""build_cherubini2015_record.py -- DRAFT v0.2 literature record for the
"Reference Coil" (RC) of Cherubini et al. 2015 (AIP Advances 5:067158), a
thermally driven coiled nylon 6 fishing-line actuator, built only from the
digitised CSVs and the quoted-values CSV in this folder (no hand-typed
numbers).

Pattern: follows build_kubler2023_records.py -- provenance block with CSV
hashes, data_completeness "curves_only", one raw CSV row asserted to
round-trip, validation before any write, refuse to overwrite.

Writes into THIS folder only:
  cherubini2015_RC_datasetC.json        the draft record
  cherubini2015_figs_reading_error.json      reading error and the checks below

Reference state
---------------
The coil is held at dataset C's elongation (20 mm, 28.6 % of L0 = 70 mm)
at T_REF, under the stress the isometric line gives there:
    sigma0 = linear fit of the digitised rising-temperature points of
             dataset C (Fig 7), evaluated at T_REF.
T_REF is the paper's ambient isothermal temperature, 25 degC (quoted CSV);
the isometric cycles started at 24 degC. The fit is extrapolated about
11 degC below the first plotted point (about 36 degC).

Input: chamber air temperature rise above T_REF (degC), unit null, as
tools/curves_to_record.py does for an input outside its unit list.

Blocked force (Fig 7, dataset C, rising temperature): the length is held,
so the record's blocked force at temperature T is the rise in tension
    force_axial_N = -(sigma(T) - sigma0) * A
with A the cross-section of the untwisted precursor fibre (the paper's
definition of nominal stress), A = pi d^2 / 4, d = 0.5 mm (quoted). Sign:
a contractor pulls toward the base, so negative along X, as in the
mckibben_v2 records.

Free motion (Fig 9 right, isothermal unloading curves): at constant
tension sigma0, heating from T_REF to T shortens the coil from
eps_25(sigma0) to eps_T(sigma0). The tip displacement along X is
    dx = (eps_T(sigma0) - eps_25(sigma0)) * L0
which is negative. eps_T(sigma0) is a straight-line fit of strain against
stress over the traced rows within +/- FIT_HALF_MPA of sigma0. The
unloading curves are used because the paper says the isometric data agree
much better with them than with the loading/unloading average; the average
family (Fig 9 left) is computed as a comparison only.

Run (from anywhere):
    python -I build_cherubini2015_record.py            # writes + validates
    python -I build_cherubini2015_record.py --dry-run  # build + assert only
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WORKSHOP = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, WORKSHOP)
from gramian_workshop.vendor import actuator_record as ar  # noqa: E402

MM = 1e-3
MPA = 1e6
MG = 1e-6
STANDARD_GRAVITY = 9.80665        # m/s^2, only for the 25 MPa cross-check
DEFAULT_UNCERTAINTY_FRACTION = 0.1   # as the Mosadegh and Kubler records
READ_PX = 1.0                     # pick-off error of a marker or line centre
FIT_HALF_MPA = 1.5                # half-window of the eps(sigma) line fit
DATASET = "C"
FREE_FAMILY = "unloading"

DATA_DIR = "data/draft_records/cherubini2015_coiled_nylon"
DIGITIZER = "digitize_cherubini2015_figs.py"
CITATION = ("Cherubini, A., Moretti, G., Vertechy, R. and Fontana, M. "
            "\"Experimental characterization of thermally-activated artificial "
            "muscles based on coiled nylon fishing lines.\" AIP Advances 5, "
            "067158 (2015). doi:10.1063/1.4923315. CC BY 3.0. Digitised from "
            "Fig 7 (isometric stress against temperature) and Fig 9 right "
            "(isothermal unloading curves).")


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


def assert_anchor(label, expected, actual, tol=1e-9):
    if abs(expected - actual) > tol * max(1.0, abs(expected)):
        raise AssertionError(f"anchor mismatch [{label}]: expected "
                             f"{expected!r}, got {actual!r}")
    print(f"  anchor OK [{label}]: {actual!r}")


def quoted():
    rows = read_csv_rows(os.path.join(HERE, "cherubini2015_quoted_values.csv"))
    return {r["key"]: float(r["value"]) for r in rows}


def linfit(x, y):
    """Slope, intercept, and the standard error of the line at x0 (callable)."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    p = np.polyfit(x, y, 1)
    res = y - np.polyval(p, x)
    n = len(x)
    s2 = float(res @ res) / (n - 2)
    xm, sxx = x.mean(), float(((x - x.mean()) ** 2).sum())

    def se(x0):
        return math.sqrt(s2 * (1.0 / n + (x0 - xm) ** 2 / sxx))
    return float(p[0]), float(p[1]), se, float(np.abs(res).max())


def strain_at(curve_rows, T, sigma):
    """eps_T(sigma): straight-line fit of strain on stress over the traced
    rows within +/- FIT_HALF_MPA of sigma; also the bracketing-row value and
    the gap between the bracketing rows."""
    rows = [r for r in curve_rows if float(r["T_degC"]) == T]
    s = np.array([float(r["stress_MPa"]) for r in rows])
    e = np.array([float(r["strain"]) for r in rows])
    w = np.abs(s - sigma) <= FIT_HALF_MPA
    below, above = s[w] < sigma, s[w] >= sigma
    assert below.sum() >= 3 and above.sum() >= 3, (T, sigma, below.sum(), above.sum())
    slope, icpt, se, maxres = linfit(s[w], e[w])
    lo = int(np.argmax(np.where(s < sigma, s, -np.inf)))
    hi = int(np.argmin(np.where(s >= sigma, s, np.inf)))
    t = (sigma - s[lo]) / (s[hi] - s[lo])
    bracket = float(e[lo] + t * (e[hi] - e[lo]))
    return {"T_degC": T, "strain": slope * sigma + icpt,
            "strain_bracket": bracket, "bracket_gap_MPa": float(s[hi] - s[lo]),
            "rows_used": int(w.sum()), "dstrain_dstress_per_MPa": slope,
            "fit_se_strain": se(sigma), "fit_max_residual_strain": maxres}


# ---------------------------------------------------------- the analysis

def analyse():
    q = quoted()
    with open(os.path.join(HERE, "cherubini2015_figs_calibration.json"),
              encoding="utf-8") as f:
        cal = json.load(f)
    iso = read_csv_rows(os.path.join(HERE, "cherubini2015_fig7_isometric.csv"))
    curves = {k: read_csv_rows(os.path.join(HERE, f"cherubini2015_fig9_curves_{k}.csv"))
              for k in ("unloading", "average")}
    tris = read_csv_rows(os.path.join(HERE, "cherubini2015_fig9_triangles.csv"))

    # cross-section from the quoted diameter, checked against the paper's
    # own 0.5 kg <-> 25 MPa statement
    d = q["precursor_diameter"] * MM
    area = math.pi * d * d / 4.0
    s_fab = q["fabrication_load"] * STANDARD_GRAVITY / area / MPA
    assert abs(s_fab - q["fabrication_nominal_stress"]) < 0.5, s_fab
    L0 = q["nominal_length_L0"] * MM
    mass = q["mass"] * MG
    T_ref = q["isothermal_temperature_1"]
    temps = [q[f"isothermal_temperature_{i}"] for i in (1, 2, 3, 4)]
    eps_C = q[f"isometric_deformation_{DATASET}"] / 100.0
    assert abs(q[f"isometric_elongation_{DATASET}"] / q["nominal_length_L0"]
               - eps_C) < 0.001

    # Fig 7: dataset C, rising temperature
    rise = [r for r in iso if r["dataset"] == DATASET and r["series"] == "rising"]
    desc = [r for r in iso if r["dataset"] == DATASET and r["series"] == "descending"]
    rise.sort(key=lambda r: float(r["T_degC"]))
    Tr = [float(r["T_degC"]) for r in rise]
    Sr = [float(r["stress_MPa"]) for r in rise]
    a, b, se, maxres = linfit(Tr, Sr)
    sigma0 = a * T_ref + b
    a_t, b_t = q[f"tableI_alpha_{DATASET}"], q[f"tableI_beta_{DATASET}"]
    # digitised fit must reproduce Table I (fit of all their points)
    assert abs(a - a_t) < 0.01, (a, a_t)
    assert abs(sigma0 - (a_t * T_ref + b_t)) < 0.5, (sigma0, a_t * T_ref + b_t)
    ad, bd, _se, _m = linfit([float(r["T_degC"]) for r in desc],
                             [float(r["stress_MPa"]) for r in desc])

    # Fig 9: strain at sigma0 on each isothermal curve
    free = {}
    for fam, rows in curves.items():
        free[fam] = [strain_at(rows, T, sigma0) for T in temps]
    e25 = free[FREE_FAMILY][0]["strain"]
    # the 25 degC unloading curve should sit near dataset C's held strain
    assert abs(e25 - eps_C) < 0.05, (e25, eps_C)
    for fam in free:
        strains = [x["strain"] for x in free[fam]]
        assert all(s1 < s0 for s0, s1 in zip(strains, strains[1:])), (fam, strains)

    tri_C = {(r["panel"], float(r["T_degC"])): float(r["stress_MPa"])
             for r in tris if abs(float(r["strain"]) - eps_C) < 0.01}
    return dict(q=q, cal=cal, area=area, s_fab=s_fab, L0=L0, mass=mass,
                T_ref=T_ref, temps=temps, eps_C=eps_C, rise=rise, Tr=Tr, Sr=Sr,
                fit=(a, b, se(T_ref), maxres), fit_desc=(ad, bd),
                tableI=(a_t, b_t), sigma0=sigma0, free=free, tri_C=tri_C)


def reading_error(A):
    cal = A["cal"]["panels"]
    out = {"method": (
        f"per-point reading error = sqrt(calibration max residual^2 + "
        f"{READ_PX} px pick-off^2) x |units per px|, per axis. For the free "
        f"strain at sigma0 the stress error is carried through the curve "
        f"slope; the displacement error is that of a difference of two "
        f"strains, times L0. These are lower bounds: the paper states "
        f"+/-5 degC on temperature."), "panels": {}}
    for key, c in cal.items():
        ex = math.hypot(c["x_max_residual_px"], READ_PX) * abs(c["x_fit_unit_per_px"])
        ey = math.hypot(c["y_max_residual_px"], READ_PX) * abs(c["y_fit_unit_per_px"])
        out["panels"][key] = {"x_units_per_px": abs(c["x_fit_unit_per_px"]),
                              "y_units_per_px": abs(c["y_fit_unit_per_px"]),
                              "reading_error_x": ex, "reading_error_y": ey}
    p7 = out["panels"]["fig7"]
    p9 = out["panels"]["Fig 9 right (unloading)"]
    out["blocked_force_N"] = p7["reading_error_y"] * MPA * A["area"]
    out["temperature_degC"] = p7["reading_error_x"]
    a, b, se0, maxres = A["fit"]
    out["sigma0_fit_standard_error_MPa"] = se0
    out["sigma0_force_equivalent_N"] = se0 * MPA * A["area"]
    fr = []
    e_ref = A["free"][FREE_FAMILY][0]
    for x in A["free"][FREE_FAMILY]:
        e_str = math.hypot(p9["reading_error_x"],
                           x["dstrain_dstress_per_MPa"] * p9["reading_error_y"])
        e_dx = math.hypot(e_str, math.hypot(
            p9["reading_error_x"], e_ref["dstrain_dstress_per_MPa"] * p9["reading_error_y"]))
        fr.append({"T_degC": x["T_degC"], "strain_reading_error": e_str,
                   "displacement_reading_error_m": e_dx * A["L0"]
                   if x is not e_ref else 0.0,
                   # line fit against bracketing-row value: a method spread,
                   # from row scatter and the stair-step of the traced line
                   "method_spread_m": abs(x["strain"] - x["strain_bracket"]) * A["L0"]})
    out["free_displacement"] = fr
    out["checks"] = {
        "cross_section_m2": A["area"],
        "fabrication_stress_from_area_MPa": A["s_fab"],
        "fig7_rising_C_fit_slope_MPa_per_degC": a,
        "fig7_rising_C_fit_intercept_MPa": b,
        "fig7_rising_C_fit_max_residual_MPa": maxres,
        "fig7_descending_C_fit_slope_MPa_per_degC": A["fit_desc"][0],
        "fig7_descending_C_fit_intercept_MPa": A["fit_desc"][1],
        "tableI_C_alpha_beta": list(A["tableI"]),
        "T_ref_degC": A["T_ref"],
        "sigma0_MPa": A["sigma0"],
        "sigma0_tableI_MPa": A["tableI"][0] * A["T_ref"] + A["tableI"][1],
        "fig9_triangles_at_dataset_C_strain_MPa": {
            f"{p} {T:g} degC": s for (p, T), s in sorted(A["tri_C"].items())},
        "held_strain_C": A["eps_C"],
        "strain_at_sigma0": {fam: [{k: v for k, v in x.items()} for x in rows]
                             for fam, rows in A["free"].items()},
        "free_dx_mm_by_family": {
            fam: [(x["strain"] - rows[0]["strain"]) * A["L0"] / MM for x in rows]
            for fam, rows in A["free"].items()},
        "free_dx_mm_if_referenced_to_held_strain": [
            (x["strain"] - A["eps_C"]) * A["L0"] / MM
            for x in A["free"][FREE_FAMILY]],
    }
    return out


# --------------------------------------------------------------- builder

def build(A, rerr):
    q, T_ref, area, L0 = A["q"], A["T_ref"], A["area"], A["L0"]
    sigma0 = A["sigma0"]
    blocked_u = [[T - T_ref] for T in A["Tr"]]
    force = [-(s - sigma0) * MPA * area for s in A["Sr"]]
    fam = A["free"][FREE_FAMILY]
    e25 = fam[0]["strain"]
    free_u = [[x["T_degC"] - T_ref] for x in fam[1:]]
    disp = [[(x["strain"] - e25) * L0, 0.0, 0.0] for x in fam[1:]]
    mags = [abs(d[0]) for d in disp]
    stroke = max(mags)
    direction = next([math.copysign(1.0, d[0]), 0.0, 0.0]
                     for d, m in zip(disp, mags) if m > 0.02 * stroke)
    us = [u[0] for u in free_u + blocked_u]
    chk = rerr["checks"]
    fr = rerr["free_displacement"]
    avg = chk["free_dx_mm_by_family"]["average"]
    held = chk["free_dx_mm_if_referenced_to_held_strain"]
    tri = chk["fig9_triangles_at_dataset_C_strain_MPa"]
    tri25 = [v for k, v in tri.items() if k.startswith("Fig 9 right") and " 25 degC" in k]
    notes = (
        f"DRAFT -- not reviewed, not in the manifest. Coiled nylon 6 fishing "
        f"line (homochiral 'Reference Coil'; precursor monofilament d = "
        f"{q['precursor_diameter']:g} mm; made under {q['fabrication_load']:g} kg; "
        f"coil index {q['coil_index']:g}, bias angle {q['bias_angle']:g} deg; "
        f"annealed at {q['anneal_temperature']:g} degC), heated by hot air in "
        f"a test chamber. One specimen. "
        f"REFERENCE STATE: the coil held at dataset {DATASET}'s elongation "
        f"({q[f'isometric_elongation_{DATASET}']:g} mm, "
        f"{q[f'isometric_deformation_{DATASET}']:g} % of L0 = "
        f"{q['nominal_length_L0']:g} mm) at {T_ref:g} degC under "
        f"sigma0 = {sigma0:.2f} MPa (our linear fit of the digitised "
        f"rising-temperature points of Fig 7 dataset {DATASET}, extrapolated "
        f"from the first plotted point at {A['Tr'][0]:.1f} degC; Table I gives "
        f"{chk['sigma0_tableI_MPa']:.2f} MPa). At zero load the coil is "
        f"coil-bound and barely shortens on heating (paper: 'preload knee'), "
        f"so this record describes the pre-loaded operating point, not the "
        f"unloaded coil. "
        f"INPUT: chamber air temperature RISE above {T_ref:g} degC (the "
        f"paper's ambient isothermal test); the isometric cycles started at "
        f"{q['isometric_ambient_temperature']:g} degC. Bench temperature "
        f"uncertainty +/-{q['temperature_uncertainty']:g} degC. "
        f"BLOCKED: Fig 7 dataset {DATASET}, rising temperature (red "
        f"asterisks), second thermal cycle, length held; force_axial_N = "
        f"-(sigma(T) - sigma0) x A, the rise in tension above the reference "
        f"preload, negative = pulls toward the base. A = pi d^2/4 = "
        f"{area:.4e} m^2 from the quoted precursor diameter, which is the "
        f"paper's own definition of nominal stress (check: {q['fabrication_load']:g} "
        f"kg gives {A['s_fab']:.2f} MPa; paper says "
        f"{q['fabrication_nominal_stress']:g}). The paper gives no area "
        f"directly. The descending-temperature circles agree (fit slope "
        f"{A['fit_desc'][0]:.4f} against {A['fit'][0]:.4f} MPa/degC rising). "
        f"FREE: strain change at constant tension sigma0 read across the "
        f"isothermal UNLOADING curves (Fig 9 right) at "
        f"{', '.join(f'{T:g}' for T in A['temps'])} degC, times L0; contraction "
        f"along -X, frame X base->tip. These are separate isothermal tensile "
        f"tests, not an isotonic heating test; isotonic tests are not in this "
        f"paper (it cites its Ref. 11 for them). The 60 degC curve has a gap "
        f"in the trace right at sigma0, where a Fig 9 triangle was masked "
        f"out. The 25 degC unloading curve reaches sigma0 at strain "
        f"{e25:.4f}, not the held {A['eps_C']:.3f}; the isometric triangle at "
        f"{A['eps_C']:.3f} and 25 degC in Fig 9 sits at "
        f"{', '.join(f'{v:.1f}' for v in tri25)} MPa. Referenced to the held "
        f"strain instead, the free displacements would be "
        f"{', '.join(f'{v:.2f}' for v in held[1:])} mm; read from the "
        f"loading/unloading AVERAGE curves (Fig 9 left) they would be "
        f"{', '.join(f'{v:.2f}' for v in avg[1:])} mm. "
        f"Reading error (cherubini2015_figs_reading_error.json): about "
        f"{rerr['blocked_force_N']:.3f} N on force, "
        f"{rerr['temperature_degC']:.2f} degC on temperature, "
        f"{max(x['displacement_reading_error_m'] for x in fr) / MM:.2f} mm on "
        f"free displacement, plus up to "
        f"{max(x['method_spread_m'] for x in fr) / MM:.2f} mm between the "
        f"line-fit and nearest-row readings; sigma0 fit standard error "
        f"{rerr['sigma0_fit_standard_error_MPa']:.2f} MPa "
        f"({rerr['sigma0_force_equivalent_N']:.3f} N). "
        f"length_m is L0 (unloaded, ambient); the coil works at "
        f"{1 + A['eps_C']:.3f} L0. Polarity_axial -1: contracts on heating.")
    rec = {
        "schema_version": "0.2",
        "id": f"cherubini2015_RC_dataset{DATASET}",
        "name": f"DRAFT Cherubini 2015 coiled nylon Reference Coil, held at "
                f"dataset {DATASET} stretch",
        "source": {
            "kind": "literature",
            "detail": (f"digitized: Cherubini et al. 2015 (AIP Advances "
                       f"5:067158) Fig 7 dataset {DATASET} rising temperature "
                       f"(isometric) + Fig 9 right isothermal unloading curves "
                       f"read at the reference stress"),
            "date": "2026-11-12",
            "reference": "doi:10.1063/1.4923315",
        },
        "metadata": {
            "actuation_physics": "thermal",
            "length_m": L0,
            "mass_kg": A["mass"],
            "material_volume_m3": None,
            "materials": {},
            "notes": notes,
        },
        "input": {
            "channels": [{
                "name": f"chamber air temperature rise above {T_ref:g} degC (degC)",
                "unit": None,
                "range": [min(0.0, min(us)), max(0.0, max(us))],
                "rated": None,
            }],
            "energy_convention": "mechanical work delivered at the input port (J)",
            "work_J": None,
        },
        "port": {"name": "tip",
                 "description": "moving end of the coil; the other end fixed"},
        "free": {"u": free_u, "tip_displacement_m": disp, "uncertainty": None},
        "blocked": {
            "u": blocked_u,
            "force_axial_N": force,
            "force_transverse_N": None,
            "method": (f"digitized Cherubini 2015 Fig 7: isometric test, coil "
                       f"held at {q[f'isometric_elongation_{DATASET}']:g} mm "
                       f"elongation, load cell, rising chamber temperature, "
                       f"second thermal cycle; force = rise in nominal stress "
                       f"above sigma0 at {T_ref:g} degC times the precursor "
                       f"cross-section"),
            "uncertainty": None,
        },
        "stiffness": None,
        "scalars": {
            "free_stroke_m": stroke,
            "free_direction": direction,
            "polarity_axial": -1,
            "blocked_force_N": max(abs(f) for f in force),
            "stiffness_N_per_m": None,
            "effective_dof": None,
        },
        "shape": None,
        "uncertainty": {"default_fraction": DEFAULT_UNCERTAINTY_FRACTION},
        "data_completeness": "curves_only",
        "provenance": {
            "data_dir": DATA_DIR,
            "digitizer_script": DIGITIZER,
            "source_csvs": [csv_ref("cherubini2015_fig7_isometric.csv"),
                            csv_ref(f"cherubini2015_fig9_curves_{FREE_FAMILY}.csv"),
                            csv_ref("cherubini2015_quoted_values.csv")],
            "citation": CITATION,
        },
    }
    return rec


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    dry = "--dry-run" in sys.argv
    A = analyse()
    rerr = reading_error(A)
    rec = build(A, rerr)

    # anchors: raw CSV rows round-trip into the record
    last = A["rise"][-1]
    i = rec["blocked"]["u"].index([float(last["T_degC"]) - A["T_ref"]])
    assert_anchor(f"blocked @ {last['T_degC']} degC",
                  -(float(last["stress_MPa"]) - A["sigma0"]) * MPA * A["area"],
                  rec["blocked"]["force_axial_N"][i])
    fam = A["free"][FREE_FAMILY]
    assert_anchor("free dx @ last level",
                  (fam[-1]["strain"] - fam[0]["strain"]) * A["L0"],
                  rec["free"]["tip_displacement_m"][-1][0])

    c = rerr["checks"]
    print(f"  A = {A['area']:.5e} m^2 (0.5 kg -> {A['s_fab']:.3f} MPa)")
    print(f"  Fig 7 C rising fit: {c['fig7_rising_C_fit_slope_MPa_per_degC']:.4f} "
          f"MPa/degC, {c['fig7_rising_C_fit_intercept_MPa']:.3f} MPa (Table I "
          f"{c['tableI_C_alpha_beta']}); descending "
          f"{c['fig7_descending_C_fit_slope_MPa_per_degC']:.4f}, "
          f"{c['fig7_descending_C_fit_intercept_MPa']:.3f}")
    print(f"  sigma0 @ {A['T_ref']:g} degC = {A['sigma0']:.3f} MPa (Table I "
          f"{c['sigma0_tableI_MPa']:.3f}); fit s.e. "
          f"{rerr['sigma0_fit_standard_error_MPa']:.3f} MPa")
    for fam_name, rows in A["free"].items():
        print(f"  strain at sigma0, {fam_name}: " + ", ".join(
            f"{x['T_degC']:g}C {x['strain']:.4f} (bracket {x['strain_bracket']:.4f}, "
            f"gap {x['bracket_gap_MPa']:.2f} MPa, n {x['rows_used']})" for x in rows))
        print(f"    dx mm: " + ", ".join(f"{v:.2f}" for v in c["free_dx_mm_by_family"][fam_name]))
    print(f"  dx mm referenced to held strain {A['eps_C']}: " + ", ".join(
        f"{v:.2f}" for v in c["free_dx_mm_if_referenced_to_held_strain"]))
    print(f"  triangles at held strain: {c['fig9_triangles_at_dataset_C_strain_MPa']}")
    s = rec["scalars"]
    print(f"  {rec['id']}: {len(rec['free']['u'])} free levels, stroke "
          f"{s['free_stroke_m'] * 1000:.2f} mm, dir {s['free_direction']}, "
          f"{len(rec['blocked']['u'])} blocked levels, max force "
          f"{s['blocked_force_N']:.3f} N")

    errors, warnings = ar.validate(rec)
    for e in errors:
        print(f"{rec['id']}: ERROR: {e}")
    for w in warnings:
        print(f"{rec['id']}: warning: {w}")
    if errors:
        print(f"{len(errors)} validation errors -- refusing to write")
        return 1
    print("record validates")
    print(f"  reading error: force {rerr['blocked_force_N']:.4f} N, temperature "
          f"{rerr['temperature_degC']:.3f} degC, free dx " + ", ".join(
              f"{x['T_degC']:g}C {x['displacement_reading_error_m'] / MM:.3f} mm"
              for x in rerr["free_displacement"]))
    if dry:
        print("--dry-run: nothing written")
        return 0

    targets = [os.path.join(HERE, f"{rec['id']}.json"),
               os.path.join(HERE, "cherubini2015_figs_reading_error.json")]
    clash = [t for t in targets if os.path.exists(t)]
    if clash:
        print("REFUSING to overwrite: " + ", ".join(clash))
        return 1
    with open(targets[1], "w", encoding="utf-8") as f:
        json.dump(rerr, f, indent=1)
    print(f"wrote {targets[1]}")
    ar.save(rec, targets[0])
    print(f"wrote {targets[0]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
