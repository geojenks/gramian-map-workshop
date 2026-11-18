"""Build one DRAFT actuator record per chamber of the Chandler et al. 2020 parallel helix actuator (alpha = 0).

Follows the pattern of Gramain_extraction/gramian_map/build_record_from_digitized.py (schema v0.2, literature
provenance block, data_completeness "curves_only", anchor asserts, validate before write, refuse to overwrite).

Every value comes from chandler2020_fig7C_a0_tip_C{n}.csv (written by digitize_chandler2020_fig7.py).
Unit conversion happens here: volume ml -> m^3, displacement mm -> m.

Frame (one common actuator frame for all three chambers, right-handed):
    record X = -z_paper  (base -> tip; the paper's Fig 7 z is the tip's rise from rest back toward the base)
    record Y = +y_paper
    record Z = +x_paper  (x_paper is the inset's "up" direction, 0 deg; det of the map = +1)
So each chamber bends in its own direction in the record Y-Z plane; no chamber is rotated onto +Y.

Rows used: status "ok" with a counted volume, plus the rest row (zero volume, zero displacement by definition).
Poorly read points are kept and declared through free.uncertainty, as the schema asks, not filtered out. Run:  python -I build_chandler2020_draft_records.py
"""
import csv
import hashlib
import importlib.util
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WORKSHOP = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
GRAMIAN_MAP_VALIDATOR = r"C:\Users\geo\Documents\Projects\ARIA\Gramain_extraction\gramian_map\actuator_record.py"
DATA_DIR = "data/draft_records/chandler2020_parallel_helix"
DIGITIZER = "digitize_chandler2020_fig7.py"
ML_TO_M3 = 1e-6
MM_TO_M = 1e-3
DEFAULT_UNCERTAINTY_FRACTION = 0.1
CITATION = ("Chandler, J. H., Chauhan, M., Garbin, N., Obstein, K. L., and Valdastri, P. \"Parallel Helix Actuators "
            "for Soft Robotic Applications.\" Front. Robot. AI 7:119, 2020. doi:10.3389/frobt.2020.00119 (CC BY 4.0)")
COLOUR = {1: "red circles", 2: "green triangles", 3: "blue squares"}


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


def read_csv_rows(path):
    with open(path, encoding="utf-8") as f:
        lines = [ln for ln in f if not ln.startswith("#")]
    return list(csv.DictReader(lines))


def csv_ref(fname):
    return {"file": fname, "sha256": sha256_file(os.path.join(HERE, fname))}


def to_record_frame(x_mm, y_mm, z_mm):
    return [-z_mm * MM_TO_M, y_mm * MM_TO_M, x_mm * MM_TO_M]


def unit_direction(p0, p1):
    d = [b - a for a, b in zip(p0, p1)]
    n = math.sqrt(sum(c * c for c in d))
    return [c / n for c in d]


def assert_anchor(label, expected, actual, tol=1e-12):
    if abs(expected - actual) > tol:
        raise AssertionError(f"anchor {label}: expected {expected}, got {actual}")


def load_validator(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def build(ch, src_name):
    rows = read_csv_rows(os.path.join(HERE, src_name))
    used, dropped = [], []
    for r in rows:
        if r["status"] == "rest_by_definition":
            used.append(r)
        elif r["status"] == "ok" and r["V_ml"]:
            used.append(r)
        else:
            dropped.append(r)
    used.sort(key=lambda r: float(r["V_ml"]))
    u = [[float(r["V_ml"]) * ML_TO_M3] for r in used]
    disp = [to_record_frame(float(r["x_mm"]), float(r["y_mm"]), float(r["z_mm"])) for r in used]

    # B3 anchor: the last raw row round-trips into the record
    raw = used[-1]
    assert_anchor("V", float(raw["V_ml"]) * 1e-6, u[-1][0])
    assert_anchor("X=-z", -float(raw["z_mm"]) / 1000.0, disp[-1][0])
    assert_anchor("Y=y", float(raw["y_mm"]) / 1000.0, disp[-1][1])
    assert_anchor("Z=x", float(raw["x_mm"]) / 1000.0, disp[-1][2])

    xy_err = [float(r["xy_err_mm"]) for r in used if r["xy_err_mm"]]
    z_err = [float(r["z_err_mm"]) for r in used if r["z_err_mm"]]
    stroke = math.sqrt(sum(c * c for c in disp[-1]))
    # Curve uncertainty (schema: fractional, 1-sigma-ish; "sparse literature points get wide tolerances by
    # declaring large uncertainty, not by special-casing"): worst 3D reading error over the used rows / free stroke.
    err3d_mm = max(math.hypot(float(r["xy_err_mm"]), float(r["z_err_mm"])) for r in used if r["xy_err_mm"])
    free_unc = err3d_mm * MM_TO_M / stroke
    rid = f"chandler2020_pha_a0_C{ch}"
    rec = {
        "schema_version": "0.2",
        "id": rid,
        "name": f"Chandler 2020 parallel helix actuator, thread angle 0 deg, chamber C{ch} alone (DRAFT)",
        "source": {
            "kind": "literature",
            "detail": (f"digitized: Chandler et al. 2020 Fig 7C (3D tip trajectory, {COLOUR[ch]}) with its x-y polar "
                       f"inset; volume per marker counted back from the end of the 8 ml up-sweep in 0.2 ml steps "
                       f"(assumption, see PROVENANCE.md). DRAFT: not in the manifest."),
            "date": "2026-11-12",
            "reference": "doi:10.3389/frobt.2020.00119",
        },
        "metadata": {
            "actuation_physics": "pneumatic",
            "length_m": None,
            "mass_kg": None,
            "material_volume_m3": None,
            "materials": {},
            "notes": (
                "Three-chamber parallel helix actuator (helical chambers round a core), single silicone material; "
                "this record is one chamber driven alone, the other two at zero injected volume. Geometry and material "
                "are given in the paper's Table 1 and Fig 4 but are not entered here (no hand-typed numbers). "
                "INPUT: syringe-displaced air volume (compressible), not chamber volume; stored in m^3 with unit "
                "null because the schema's SI unit list has no volume unit. Up-sweep of a linear volume ramp to 8 ml; "
                "the paper reports up/down hysteresis that grows with volume rate (its Fig 9). VOLUME UNCERTAINTY: "
                "about +-0.3 ml from 5 ml up, up to about 1.3 ml below 5 ml (C1 check, PROVENANCE.md). FRAME: one common frame for all three chamber "
                "records: X base->tip (= -z of the paper's Fig 7, which plots the rise of the tip toward the base), "
                "Y = paper y, Z = paper x (inset 0 deg); right-handed. Each chamber bends in its own direction "
                "in Y-Z (paper Table 3 gives the measured bending planes); no chamber was rotated onto +Y. "
                f"READING ERROR (from the digitiser): x/y up to {max(xy_err):.1f} mm (inset stroke half-width), "
                f"z up to {max(z_err):.1f} mm; free.uncertainty = worst 3D reading error / free stroke "
                f"= {free_unc:.2f}. Per-point errors are in the source CSV (xy_err_mm, z_err_mm). VOLUMES are inferred, not read off an axis: Fig 7C carries no "
                "volume; see PROVENANCE.md for the counting rule and its check against the paper's Eq 11 fit. "
                "Low-volume markers merged at the origin are omitted. Bending device: polarity_axial left null. "
                "BLOCKED: null -- the paper has no blocked-force or tip-force measurement; its Fig 11 shows only "
                "the workspace with small tip masses (2.5-7.5 g) hung on, which is not a blocked force."),
        },
        "input": {
            "channels": [{
                "name": f"injected air volume, chamber C{ch} (m^3; syringe displacement)",
                "unit": None,
                "range": [u[0][0], u[-1][0]],
                "rated": None,
            }],
            "energy_convention": "mechanical work delivered at the input port (J)",
            "work_J": None,
        },
        "port": {"name": "tip", "description": "free-end centroid"},
        "free": {"u": u, "tip_displacement_m": disp, "uncertainty": free_unc},
        "blocked": None,
        "stiffness": None,
        "scalars": {
            "free_stroke_m": stroke,
            "free_direction": unit_direction(disp[0], disp[1]),
            "polarity_axial": None,
            "blocked_force_N": None,
            "stiffness_N_per_m": None,
            "effective_dof": None,
        },
        "shape": None,
        "uncertainty": {"default_fraction": DEFAULT_UNCERTAINTY_FRACTION},
        "data_completeness": "curves_only",
        "provenance": {
            "data_dir": DATA_DIR,
            "digitizer_script": DIGITIZER,
            "source_csvs": [csv_ref(src_name), csv_ref("chandler2020_fig7C_a0_calibration.csv")],
            "citation": CITATION,
        },
    }
    return rid, rec, len(used), len(dropped)


def main():
    vend = load_validator(os.path.join(WORKSHOP, "gramian_workshop", "vendor", "actuator_record.py"), "ar_vendor")
    gm = load_validator(GRAMIAN_MAP_VALIDATOR, "ar_gm")
    outs = []
    for ch in (1, 2, 3):
        rid, rec, n_used, n_drop = build(ch, f"chandler2020_fig7C_a0_tip_C{ch}.csv")
        for label, mod in (("workshop vendored v0.1", vend), ("gramian_map v0.2", gm)):
            errs, warns = mod.validate(rec)
            print(f"{rid} [{label}] errors={errs} warnings={warns}")
            if errs:
                raise SystemExit(f"{rid}: validation errors, not written")
        path = os.path.join(HERE, f"{rid}.json")
        if os.path.exists(path):
            raise SystemExit(f"{path} exists; refusing to overwrite (additive only)")
        outs.append((path, rec, n_used, n_drop))
    for path, rec, n_used, n_drop in outs:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(rec, f, indent=1)
            f.write("\n")
        d = rec["free"]["tip_displacement_m"][-1]
        print(f"wrote {path}: {n_used} levels ({n_drop} digitised rows not used), "
              f"stroke {rec['scalars']['free_stroke_m'] * 1000:.1f} mm, end displacement (mm) "
              f"[{d[0] * 1000:.1f}, {d[1] * 1000:.1f}, {d[2] * 1000:.1f}]")


if __name__ == "__main__":
    main()
