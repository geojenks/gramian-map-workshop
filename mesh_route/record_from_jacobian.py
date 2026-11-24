"""record_from_jacobian.py — SOFA pipeline results → characterization record.

Converts a jacobian.npz (plus optional CAD-dir metadata and either a legacy
blocked_force_probe log or structured blocked_moment_probe CSV) into a
RECORD_SCHEMA.md v0.1 JSON record. This is the Tier-2 producer: free-motion
curve + blocked-wrench curve + spatial mode fingerprint, with geometry pulled
from actuator.npz when available.

No re-simulation: everything comes from artifacts that already exist.

Usage:
    python record_from_jacobian.py experiments/mckibben/braid_angle/v2_30deg/free_expansion/jacobian.npz \
        --cad cads/mckibben/v2_30deg \
        --bforce experiments/mckibben/braid_angle/v2_30deg/blocked_force/probe.log \
        --out records/mckibben_v2_30deg.json
"""

from __future__ import annotations

import argparse
import csv
import glob
import hashlib
import json
import os
import re
import sys

import numpy as np

import actuator_record
from jacobian import rest_spine_from_jacobian

N_MODES_STORED = 6
DEFAULT_UNCERTAINTY = 0.1          # fractional, simulation source


def tet_volume_sum(points: np.ndarray, tets: np.ndarray) -> float:
    a = points[tets[:, 1]] - points[tets[:, 0]]
    b = points[tets[:, 2]] - points[tets[:, 0]]
    c = points[tets[:, 3]] - points[tets[:, 0]]
    return float(np.abs(np.einsum("ij,ij->i", np.cross(a, b), c)).sum() / 6.0)


def parse_bforce_log(path: str, expect_cad: str | None):
    """blocked_force_probe.py log → (u_Pa, F_axial_N, F_transverse_N)."""
    cad_seen = None
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = re.search(r"CAD=(\S+)", line)
            if m:
                cad_seen = m.group(1)
            m = re.match(r"\s*([\d.eE+-]+)\s+([\d.eE+-]+)\s+([\d.eE+-]+)\s+"
                         r"([\d.eE+-]+)\s+([\d.eE+-]+)\s+(EXTEND|CONTRACT|"
                         r"NEUTRAL)", line)
            if m:
                rows.append((float(m.group(1)), float(m.group(2)),
                             float(m.group(3))))
    if expect_cad and cad_seen and cad_seen != expect_cad:
        print(f"WARNING: {path} says CAD={cad_seen}, expected {expect_cad}",
              file=sys.stderr)
    if not rows:
        raise SystemExit(f"ERROR: no force rows parsed from {path}")
    rows.sort(key=lambda r: r[0])
    u = [r[0] for r in rows]
    return u, [r[1] for r in rows], [r[2] for r in rows]


UNACCEPTABLE_SETTLE_REASONS = frozenset({"diverged", "max_steps"})


def parse_blocked_moment_csv(path: str,
                             rotation: np.ndarray | None = None):
    """Map blocked_moment_probe.py CSV rows to canonical record fields.

    ``rotation`` is the same source-to-canonical matrix applied to free motion
    and modes. Full force and base-moment vectors rotate with it; the tip twist
    scalar is already defined about the longitudinal axis and is unchanged.

    New probe CSVs carry ``settle_reason`` so numerically incomplete levels are
    rejected before a record can be produced.  Older C2 CSVs without convergence
    columns remain readable because those files predate the shared settle helper.
    """
    with open(path, "r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"no blocked-moment rows parsed from {path}")

    required = {
        "p_Pa", "Fx_N", "Fy_N", "Fz_N",
        "Mx_Nm", "My_Nm", "Mz_Nm", "M_twist_tip_Nm",
    }
    missing = sorted(required - set(rows[0]))
    if missing:
        raise ValueError(
            f"blocked-moment CSV missing columns: {', '.join(missing)}")

    failed = []
    for row in rows:
        reason = (row.get("settle_reason") or row.get("reason") or "").strip()
        if reason in UNACCEPTABLE_SETTLE_REASONS:
            failed.append(f"p={row.get('p_Pa')}:{reason}")
    if failed:
        raise ValueError("blocked-moment CSV has unacceptable convergence: "
                         + ", ".join(failed))

    def finite_value(row, name):
        value = float(row[name])
        if not np.isfinite(value):
            raise ValueError(
                f"blocked-moment CSV has non-finite {name} at p={row.get('p_Pa')}")
        return value

    u = []
    forces = []
    moments = []
    twists = []
    for row in rows:
        p = finite_value(row, "p_Pa")
        force = [finite_value(row, name)
                 for name in ("Fx_N", "Fy_N", "Fz_N")]
        moment = [finite_value(row, name)
                  for name in ("Mx_Nm", "My_Nm", "Mz_Nm")]
        twist = finite_value(row, "M_twist_tip_Nm")
        u.append([p])
        forces.append(force)
        moments.append(moment)
        twists.append(twist)

    force_array = np.asarray(forces, dtype=float)
    moment_array = np.asarray(moments, dtype=float)
    if rotation is not None:
        rotation = np.asarray(rotation, dtype=float)
        if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
            raise ValueError("blocked-moment rotation must be a finite 3x3 matrix")
        force_array = force_array @ rotation.T
        moment_array = moment_array @ rotation.T
    blocked = {
        "u": u,
        "force_axial_N": force_array[:, 0].tolist(),
        "force_transverse_N": np.linalg.norm(
            force_array[:, 1:3], axis=1).tolist(),
        "force_N": force_array.tolist(),
        "moment_Nm": moment_array.tolist(),
        "moment_method":
            "tip spring reaction x lever arm (blocked_moment_probe.py)",
        "twist_tip_Nm": twists,
        "method": "held-tip reaction springs (blocked_moment_probe.py)",
        "uncertainty": None,
    }
    blocked_force_max = float(np.linalg.norm(force_array, axis=1).max())
    return blocked, blocked_force_max


def canonical_rotation_matrix(tip_disp: np.ndarray, small_index: int | None,
                              fixed_azimuth_deg: float | None = None
                              ) -> tuple[np.ndarray, str]:
    """Return the source-to-canonical rotation about the longitudinal X axis.

    With ``fixed_azimuth_deg`` omitted, retain the legacy behaviour: infer the
    transverse azimuth from measurable small-signal motion.  A fixed value
    declares where the intended canonical +Y axis lies in the source frame;
    zero therefore preserves generated +Y even when near-axisymmetric lateral
    motion is only numerical noise.
    """
    if fixed_azimuth_deg is not None:
        theta = float(np.deg2rad(fixed_azimuth_deg))
        source = f"fixed:{fixed_azimuth_deg:g}deg"
    else:
        small = tip_disp[small_index] if small_index is not None else None
        if small is None or np.hypot(small[1], small[2]) <= 0.1 * \
                np.linalg.norm(small):
            return np.eye(3), "auto:no_resolved_transverse_motion"
        theta = float(np.arctan2(small[2], small[1]))
        source = f"auto:{np.rad2deg(theta):g}deg"
    c, s = np.cos(-theta), np.sin(-theta)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, c, -s],
                     [0.0, s, c]]), source


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def provenance_sidecars(cad_dir: str) -> dict[str, str]:
    """Hash established NPZ sidecars plus geometry.json when present."""
    paths = [p for p in sorted(glob.glob(os.path.join(cad_dir, "*.npz")))
             if os.path.basename(p) != "actuator.npz"]
    geometry = os.path.join(cad_dir, "geometry.json")
    if os.path.isfile(geometry):
        paths.append(geometry)
    return {os.path.basename(path): sha256_file(path) for path in paths}


def parse_stiffness(path):
    """Parse a stiffness CSV into the record's stiffness block + baseline k.

    Handles both producers via the CSV's `mode` column: tip_stiffness_probe.py
    (mode='pressure', u = chamber pressure) and stiffness_probe.py
    (mode='displacement'/'force', u = cable co-contraction) — plus the old
    pre-3-axis `u_co_m` format. Labels adapt so a pressure probe is never
    mislabelled as co-contraction.
    """
    import csv as _csv
    with open(path, "r", encoding="utf-8") as f:
        rd = list(_csv.DictReader(f))
    if not rd:
        return None, None
    old_fmt = "u_co_m" in rd[0]
    mode = "displacement" if old_fmt else rd[0].get("mode", "displacement")
    if mode == "pressure":
        method = ("two-sided tip force probe (tip_stiffness_probe.py); "
                  "u = chamber pressure (Pa); input-holding condition: "
                  "pressure-controlled")
    else:
        method = ("two-sided tip force probe (stiffness_probe.py); "
                  "u = co-contraction level, all drive channels equal; "
                  f"input-holding condition: {mode}-controlled tendons")
    stiffness = {
        "u": [[float(r["u_co_m" if old_fmt else "level"])] for r in rd],
        "k_N_per_m": [float(r["k_transverse_N_per_m" if old_fmt
                             else "k_y_N_per_m"]) for r in rd],
        "k_out_of_plane_N_per_m": (None if old_fmt
                                   else [float(r["k_z_N_per_m"]) for r in rd]),
        "k_axial_N_per_m": [float(r["k_axial_N_per_m" if old_fmt
                                    else "k_x_N_per_m"]) for r in rd],
        "direction": "transverse (+Y canonical)",
        "method": method,
        "uncertainty": None,
    }
    return stiffness, stiffness["k_N_per_m"][0]


def rest_relative_free_motion(data):
    """Return finite free-motion levels referenced to the explicit true rest."""
    pressures = np.asarray(data["pressures"], dtype=float)
    spine = np.asarray(data["spine_positions"], dtype=float)
    n = min(len(pressures), len(spine))
    pressures, spine = pressures[:n], spine[:n]
    rest_spine, rest_index, rest_source = rest_spine_from_jacobian(
        data, spine=spine, pressures=pressures)
    original_indices = np.arange(n)
    ok = np.isfinite(spine).all(axis=(1, 2)) & np.isfinite(pressures)
    pressures, spine, original_indices = pressures[ok], spine[ok], \
        original_indices[ok]
    if len(pressures) < 2:
        raise ValueError("fewer than 2 finite sweep levels")
    filtered_rest_index = None
    if rest_index is not None:
        hits = np.flatnonzero(original_indices == rest_index)
        if len(hits) != 1:
            raise ValueError("finite filtering removed the explicit rest level")
        filtered_rest_index = int(hits[0])
    tip_disp = spine[:, -1, :] - rest_spine[-1, :]
    return (pressures, spine, rest_spine, tip_disp, filtered_rest_index,
            rest_source)


def select_free_motion_indices(pressures: np.ndarray, tip_disp: np.ndarray,
                               rest_index: int | None,
                               measurable_m: float = 1e-6) -> tuple[int | None, int]:
    """Select order-independent small-signal and stroke levels.

    Small-signal direction uses the measurable non-rest level nearest zero by
    absolute input.  An exact +/- tie selects the numerically lower input
    (negative before positive), then the lower original index if duplicate
    inputs somehow survive.  Stroke uses the largest displacement norm; equal
    norms use the same absolute-input/signed-input tie rule.
    """
    u = np.asarray(pressures, float)
    disp = np.asarray(tip_disp, float)
    if len(u) != len(disp):
        raise ValueError("pressures and tip displacement lengths differ")
    nonrest = np.ones(len(u), dtype=bool)
    if rest_index is not None:
        nonrest[rest_index] = False
    norms = np.linalg.norm(disp, axis=1)

    def deterministic_first(candidates: np.ndarray) -> int:
        order = np.lexsort((candidates, u[candidates], np.abs(u[candidates])))
        return int(candidates[order[0]])

    measurable = np.flatnonzero(nonrest & (norms > measurable_m))
    small_index = deterministic_first(measurable) if len(measurable) else None
    candidates = np.flatnonzero(nonrest)
    if not len(candidates):
        raise ValueError("no non-rest free-motion levels")
    max_norm = float(norms[candidates].max())
    max_candidates = candidates[np.isclose(
        norms[candidates], max_norm, rtol=1e-12, atol=1e-15)]
    stroke_index = deterministic_first(max_candidates)
    return small_index, stroke_index


def free_motion_scalars(pressures: np.ndarray, tip_disp: np.ndarray,
                        rest_index: int | None, length_m: float) -> dict:
    """Compute direction, maximum stroke, and polarity without row-order bias."""
    small_index, stroke_index = select_free_motion_indices(
        pressures, tip_disp, rest_index)
    stroke_vec = np.asarray(tip_disp[stroke_index], float)
    stroke = float(np.linalg.norm(stroke_vec))
    if small_index is None:
        direction = [0.0, 0.0, 0.0]
    else:
        small_vec = np.asarray(tip_disp[small_index], float)
        direction = (small_vec / np.linalg.norm(small_vec)).tolist()
    axial = float(stroke_vec[0])
    significant = (abs(axial) >= 0.2 * max(stroke, 1e-12)
                   and abs(axial) >= 0.005 * length_m)
    polarity = (1 if axial > 0 else -1) if significant else 0
    return {
        "small_index": small_index,
        "stroke_index": stroke_index,
        "stroke_m": stroke,
        "direction": direction,
        "polarity_axial": polarity,
    }


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("jacobian", nargs="?", help="jacobian.npz from jacobian.py")
    ap.add_argument("--out", required=True, help="output record .json")
    ap.add_argument("--cad", default=None,
                    help="CAD dir (cads/<name>) for actuator.npz geometry "
                         "and materials.json pass-through")
    blocked_inputs = ap.add_mutually_exclusive_group()
    blocked_inputs.add_argument(
        "--bforce", default=None,
        help="legacy blocked_force_probe.py log for the blocked block")
    blocked_inputs.add_argument(
        "--blocked-moment-csv", "--blocked-moment",
        dest="blocked_moment_csv", default=None,
        help="structured blocked_moment_probe.py CSV; maps full force, base "
             "moment, and tip torsional moment into existing blocked fields")
    ap.add_argument("--stiffness", default=None,
                    help="stiffness CSV (stiffness_probe.py or "
                         "tip_stiffness_probe.py) for the stiffness block")
    ap.add_argument("--into", default=None,
                    help="attach the --stiffness block to an EXISTING record "
                         ".json non-destructively (preserves id/name/blocked/"
                         "etc.) and write to --out. No jacobian needed.")
    ap.add_argument("--id", dest="rec_id", default=None,
                    help="record id (default: CAD dir name or npz stem)")
    ap.add_argument("--name", default=None)
    ap.add_argument("--physics", default="pneumatic")
    ap.add_argument("--input_name", default="chamber pressure",
                    help="input channel name (e.g. 'tendon displacement')")
    ap.add_argument("--input_unit", default="Pa",
                    help="SI unit string of the input (e.g. 'm' for cable "
                         "displacement)")
    ap.add_argument("--source_detail",
                    default="SOFA v24.06, gramian_map pipeline")
    ap.add_argument("--canonical-transverse-azimuth-deg",
                    "--transverse-azimuth-deg",
                    dest="canonical_transverse_azimuth_deg", type=float,
                    default=None,
                    help="fixed source-frame azimuth of canonical +Y about "
                         "+X (degrees); use 0 for generated +Y. Omit to "
                         "retain legacy small-signal auto-detection.")
    ap.add_argument("--date", default=None)
    ap.add_argument("--generator_command", default=None,
                    help="verbatim CAD reproduce command; with --cad, emits "
                         "the provenance block (RECORD_SCHEMA.md) at build "
                         "time — generator name, mesh/npz/sidecar sha256")
    return ap


def main(argv=None) -> int:
    args = build_arg_parser().parse_args(argv)

    # Non-destructive merge: attach only the stiffness block to an existing
    # record (no jacobian rebuild -> id/name/blocked/etc. are preserved).
    if args.into:
        if not args.stiffness:
            print("--into requires --stiffness", file=sys.stderr)
            return 2
        rec = json.load(open(args.into, encoding="utf-8"))
        sk, base = parse_stiffness(args.stiffness)
        rec["stiffness"] = sk
        rec.setdefault("scalars", {})["stiffness_N_per_m"] = base
        json.dump(rec, open(args.out, "w", encoding="utf-8"), indent=1)
        print(f"Merged stiffness into {args.out} "
              f"(baseline k={base:.2f} N/m, {len(sk['u'])} levels)")
        return 0

    if not args.jacobian:
        print("jacobian.npz required (unless using --into)", file=sys.stderr)
        return 2

    d = np.load(args.jacobian)
    try:
        (pressures, spine, rest_spine, tip_disp, rest_index,
         rest_source) = rest_relative_free_motion(d)
        small_index, stroke_index = select_free_motion_indices(
            pressures, tip_disp, rest_index)
    except ValueError as exc:
        raise SystemExit(f"ERROR: cannot identify true rest spine: {exc}")
    print(f"Rest spine source: {rest_source}")

    # --- canonical mounting frame -----------------------------------
    # Transverse azimuth is a MOUNTING convention (rotating the actuator
    # about its long axis is free), so records are stored canonically:
    # X = base->tip, dominant small-signal transverse output along +Y.
    # Without this, a +y bender and a -y bender (cable finger vs pneunet)
    # read as maximally different directions. Axial sign is intrinsic
    # (extend vs contract) and is NOT touched.
    R_canon, azimuth_source = canonical_rotation_matrix(
        tip_disp, small_index, args.canonical_transverse_azimuth_deg)
    print(f"Canonical transverse azimuth: {azimuth_source}")
    spine = spine @ R_canon.T
    rest_spine = rest_spine @ R_canon.T
    tip_disp = tip_disp @ R_canon.T

    cad_name = os.path.basename(os.path.normpath(args.cad)) if args.cad \
        else None
    rec_id = args.rec_id or cad_name or \
        os.path.basename(args.jacobian).replace("jacobian_", "") \
        .replace(".npz", "")

    # --- geometry / materials from the CAD dir ---
    length_m = float(rest_spine[:, 0].max() - rest_spine[:, 0].min())
    material_volume = None
    materials = {}
    if args.cad:
        npz_path = os.path.join(args.cad, "actuator.npz")
        if os.path.exists(npz_path):
            a = np.load(npz_path, allow_pickle=True)
            pts = np.asarray(a["points"], dtype=float)
            length_m = float(pts[:, 0].max() - pts[:, 0].min())
            if "tet_nodes" in a.files:
                material_volume = tet_volume_sum(
                    pts, np.asarray(a["tet_nodes"], dtype=int))
        mat_path = os.path.join(args.cad, "materials.json")
        if os.path.exists(mat_path):
            with open(mat_path, "r", encoding="utf-8") as f:
                materials = json.load(f)

    # --- blocked block ---
    blocked = None
    blocked_force_max = None
    if args.bforce:
        bu, bfx, bft = parse_bforce_log(args.bforce, cad_name)
        blocked = {
            "u": [[v] for v in bu],
            "force_axial_N": bfx,
            "force_transverse_N": bft,
            "method": "tip spring k=1e6 N/m (blocked_force_probe.py)",
            "uncertainty": None,
        }
        blocked_force_max = float(max(np.hypot(np.array(bfx),
                                               np.array(bft))))
    elif args.blocked_moment_csv:
        try:
            blocked, blocked_force_max = parse_blocked_moment_csv(
                args.blocked_moment_csv, rotation=R_canon)
        except ValueError as exc:
            raise SystemExit(f"ERROR: {exc}")

    # --- stiffness block ---
    stiffness, stiffness_baseline = (parse_stiffness(args.stiffness)
                                     if args.stiffness else (None, None))

    # --- scalars ---
    # Stroke and polarity come from the largest displacement anywhere in the
    # sampled envelope, not the final array row (mixed-sign branch ordering is
    # not a physical convention).  Direction is the measurable non-rest level
    # nearest zero input; select_free_motion_indices documents the deterministic
    # negative-before-positive tie rule.
    free_scalars = free_motion_scalars(
        pressures, tip_disp, rest_index, length_m)
    stroke = free_scalars["stroke_m"]
    direction = free_scalars["direction"]
    polarity = free_scalars["polarity_axial"]

    # --- shape fingerprint ---
    shape = None
    if "vec_c" in d.files:
        eig = np.asarray(d["eig_c"], dtype=float)
        vec = np.asarray(d["vec_c"], dtype=float)
        order = np.argsort(eig)[::-1]
        eig = np.clip(eig[order], 0.0, None)
        vec = vec[:, order]
        norms = np.linalg.norm(vec, axis=0, keepdims=True)
        norms[norms < 1e-30] = 1.0
        vec = vec / norms
        # Rotate mode vectors into the canonical mounting frame (they are
        # point-major xyz fields on the spine, so rotate per point).
        P = vec.shape[0] // 3
        vec = (vec.T.reshape(-1, P, 3) @ R_canon.T).reshape(-1, P * 3).T
        k = min(N_MODES_STORED, vec.shape[1])
        shape = {
            "convention": "spine centroids, point-major xyz; polarity-blind;"
                          " read jointly with authority",
            "spine_rest_m": rest_spine.tolist(),
            "mode_eigenvalues": eig[:k].tolist(),
            "mode_vectors": vec[:, :k].tolist(),
            "authority": float(np.sqrt(eig.sum())),
        }

    u_all = list(pressures) + ([v[0] for v in blocked["u"]] if blocked
                               else [])
    record = {
        "schema_version": actuator_record.SCHEMA_VERSION,
        "id": rec_id,
        "name": args.name or rec_id,
        "source": {
            "kind": "simulation",
            "detail": args.source_detail,
            "date": args.date,
            "reference": None,
        },
        "metadata": {
            "actuation_physics": args.physics,
            "length_m": length_m,
            "mass_kg": None,
            "material_volume_m3": material_volume,
            "materials": materials,
            "notes": (f"converted from {os.path.basename(args.jacobian)}; "
                      f"canonical transverse azimuth {azimuth_source}"),
        },
        "input": {
            "channels": [{
                "name": args.input_name, "unit": args.input_unit,
                "range": [float(min(min(u_all), 0.0)), float(max(u_all))],
                "rated": None,
            }],
            "energy_convention":
                "mechanical work delivered at the input port (J)",
            "work_J": None,
        },
        "port": {"name": "tip",
                 "description": "last spine centroid (free end)"},
        "free": {
            "u": [[float(p)] for p in pressures],
            "tip_displacement_m": tip_disp.tolist(),
            "uncertainty": None,
        },
        "blocked": blocked,
        "stiffness": stiffness,
        "scalars": {
            "free_stroke_m": stroke,
            "free_direction": direction,
            "polarity_axial": polarity,
            "blocked_force_N": blocked_force_max,
            "stiffness_N_per_m": stiffness_baseline,
            "effective_dof": int(d["rank_c"]) if "rank_c" in d.files
            else None,
        },
        "shape": shape,
        "uncertainty": {"default_fraction": DEFAULT_UNCERTAINTY},
    }

    # Provenance pin at build time (no "backfilled" field — captured live).
    if args.generator_command and args.cad:
        cad_rel = os.path.relpath(args.cad,
                                  os.path.dirname(os.path.abspath(__file__))
                                  ).replace("\\", "/")
        mesh = os.path.join(args.cad, "mesh.msh")
        npz = os.path.join(args.cad, "actuator.npz")
        prov = {
            "generator": args.generator_command.split()[1]
            if args.generator_command.startswith("python ")
            else args.generator_command.split()[0],
            "generator_command": args.generator_command,
            "cad_dir": cad_rel,
            "mesh_file": "mesh.msh",
            "mesh_sha256": sha256_file(mesh) if os.path.isfile(mesh) else None,
            "actuator_npz_sha256": sha256_file(npz)
            if os.path.isfile(npz) else None,
            "scene": "parametricGripper.py",
            "provenance_doc": "/".join(cad_rel.split("/")[:2])
            + "/PROVENANCE.md",
        }
        sidecars = provenance_sidecars(args.cad)
        if sidecars:
            prov["sidecars_sha256"] = sidecars
        record["provenance"] = prov

    errors, warnings_ = actuator_record.validate(record)
    for w in warnings_:
        print(f"warning: {w}")
    if errors:
        for e in errors:
            print(f"ERROR: {e}", file=sys.stderr)
        return 1
    actuator_record.save(record, args.out)
    print(f"Wrote {args.out} (tier {actuator_record.tier(record)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
