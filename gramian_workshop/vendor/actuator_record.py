"""actuator_record.py — reference implementation of RECORD_SCHEMA.md v0.1.

Load / save / validate actuator characterization records (one JSON per
actuator), derive their tier, and print a summary table. Everything
downstream of the SOFA pipeline should consume records through this
module rather than reading jacobian.npz directly.

CLI:
    python actuator_record.py records/*.json     # validate + summary table
"""

from __future__ import annotations

import glob
import json
import math
import os
import sys

SCHEMA_VERSION = "0.1"

REQUIRED_TOP = ["schema_version", "id", "source", "metadata", "input",
                "port", "scalars"]
SOURCE_KINDS = {"simulation", "experiment", "literature"}

# unit strings conforming writers must use for known quantities
SI_UNITS = {"Pa", "N", "m", "kg", "J", "s", "N/m", "m/s"}


# ---------------------------------------------------------------- I/O

def load(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save(record: dict, path: str) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(record, f, indent=1)
        f.write("\n")


# ---------------------------------------------------------------- tier

def _curve_levels(block) -> int:
    if not isinstance(block, dict) or "u" not in block:
        return 0
    return len(block["u"])


def tier(record: dict) -> int:
    shape = record.get("shape")
    if isinstance(shape, dict) and shape.get("mode_vectors"):
        return 2
    for name in ("free", "blocked", "stiffness"):
        if _curve_levels(record.get(name)) >= 3:
            return 1
    return 0


# ---------------------------------------------------------------- validate

def _finite(x) -> bool:
    if isinstance(x, (int, float)):
        return math.isfinite(x)
    if isinstance(x, list):
        return all(_finite(v) for v in x)
    return True                                  # None / str are fine here


def validate(record: dict) -> tuple[list[str], list[str]]:
    """Returns (errors, warnings)."""
    errors, warnings = [], []

    for key in REQUIRED_TOP:
        if key not in record:
            errors.append(f"missing required field '{key}'")
    if errors:
        return errors, warnings

    if record["schema_version"] != SCHEMA_VERSION:
        warnings.append(f"schema_version {record['schema_version']} != "
                        f"reference {SCHEMA_VERSION}")
    kind = record["source"].get("kind")
    if kind not in SOURCE_KINDS:
        errors.append(f"source.kind '{kind}' not in {sorted(SOURCE_KINDS)}")

    channels = record["input"].get("channels") or []
    if not channels:
        errors.append("input.channels is empty")
    for ch in channels:
        u = ch.get("unit")
        if u and u not in SI_UNITS:
            errors.append(f"channel '{ch.get('name')}' unit '{u}' is not a "
                          f"recognized SI unit string {sorted(SI_UNITS)}")

    # curve blocks: internal consistency
    for name, cols in (("free", ["tip_displacement_m"]),
                       ("blocked", ["force_axial_N", "force_transverse_N"])):
        block = record.get(name)
        if not isinstance(block, dict):
            continue
        u = block.get("u")
        if u is None:
            errors.append(f"{name}.u missing")
            continue
        n = len(u)
        for row in u:
            if len(row) != len(channels):
                errors.append(f"{name}.u row width {len(row)} != "
                              f"{len(channels)} channels")
                break
        for col in cols:
            v = block.get(col)
            if v is not None and len(v) != n:
                errors.append(f"{name}.{col} length {len(v)} != "
                              f"{name}.u length {n}")
        if not _finite(u) or any(not _finite(block.get(c) or [])
                                 for c in cols):
            errors.append(f"{name} block contains NaN/inf")
        # C2 optional vector fields (additive, backward-compatible)
        if name == "blocked":
            for vec3_col in ("force_N", "moment_Nm"):
                v = block.get(vec3_col)
                if v is not None:
                    if len(v) != n:
                        errors.append(
                            f"blocked.{vec3_col} length {len(v)} != "
                            f"blocked.u length {n}")
                    elif any(len(row) != 3 for row in v
                             if isinstance(row, (list, tuple))):
                        errors.append(
                            f"blocked.{vec3_col} rows must be length 3")
                    if not _finite(v):
                        errors.append(
                            f"blocked.{vec3_col} contains NaN/inf")
            twist = block.get("twist_tip_Nm")
            if twist is not None:
                if len(twist) != n:
                    errors.append(
                        f"blocked.twist_tip_Nm length {len(twist)} != "
                        f"blocked.u length {n}")
                if not _finite(twist):
                    errors.append(
                        "blocked.twist_tip_Nm contains NaN/inf")
        for row in u:
            for val, ch in zip(row, channels):
                lo, hi = (ch.get("range") or [None, None])
                if lo is not None and (val < lo - 1e-12 or val > hi + 1e-12):
                    warnings.append(f"{name}.u value {val} outside declared "
                                    f"range {ch.get('range')} of "
                                    f"'{ch.get('name')}'")
                    break

    shape = record.get("shape")
    if isinstance(shape, dict):
        eig = shape.get("mode_eigenvalues") or []
        vec = shape.get("mode_vectors") or []
        if vec and eig and len(vec[0]) != len(eig):
            errors.append("shape.mode_vectors column count != "
                          "len(mode_eigenvalues)")
        spine = shape.get("spine_rest_m") or []
        if vec and spine and len(vec) != 3 * len(spine):
            errors.append(f"shape.mode_vectors rows {len(vec)} != "
                          f"3 * {len(spine)} spine points")
        if shape.get("authority") is None:
            warnings.append("shape block without authority — shape "
                            "distances are uninterpretable at low authority")
        if not _finite(eig) or not _finite(vec):
            errors.append("shape block contains NaN/inf")

    t = tier(record)
    if t == 0 and kind == "simulation":
        warnings.append("tier 0 from a simulation source — sims can always "
                        "produce at least curves")
    unc = record.get("uncertainty") or {}
    if unc.get("default_fraction") is None:
        warnings.append("no uncertainty.default_fraction — metric "
                        "tolerances will fall back to global defaults")

    return errors, warnings


# ---------------------------------------------------------------- summary

def summarize(record: dict) -> dict:
    s = record.get("scalars") or {}
    md = record.get("metadata") or {}
    stroke = s.get("free_stroke_m")
    length = md.get("length_m")
    strain = (stroke / length) if (stroke and length) else None
    shape = record.get("shape") or {}
    return {
        "id": record.get("id", "?"),
        "tier": tier(record),
        "kind": (record.get("source") or {}).get("kind", "?"),
        "physics": md.get("actuation_physics", "?"),
        "stroke_mm": None if stroke is None else stroke * 1e3,
        "strain_pct": None if strain is None else strain * 100.0,
        "polarity": s.get("polarity_axial"),
        "blocked_N": s.get("blocked_force_N"),
        "dof": s.get("effective_dof"),
        "authority": shape.get("authority"),
    }


def _fmt(v, spec) -> str:
    return "-" if v is None else format(v, spec)


def main() -> int:
    paths = []
    for arg in sys.argv[1:]:
        paths.extend(sorted(glob.glob(arg)))
    if not paths:
        print(__doc__)
        return 2

    rows, n_err = [], 0
    for p in paths:
        try:
            rec = load(p)
        except (json.JSONDecodeError, OSError) as e:
            print(f"{p}: UNREADABLE — {e}")
            n_err += 1
            continue
        errors, warnings = validate(rec)
        for e in errors:
            print(f"{p}: ERROR: {e}")
        for w in warnings:
            print(f"{p}: warning: {w}")
        n_err += len(errors)
        rows.append(summarize(rec))

    hdr = (f"{'id':<28} {'tier':>4} {'kind':<11} {'physics':<10} "
           f"{'stroke_mm':>9} {'strain%':>8} {'pol':>4} {'blocked_N':>10} "
           f"{'dof':>4} {'authority':>10}")
    print("\n" + hdr)
    print("-" * len(hdr))
    for r in rows:
        print(f"{r['id']:<28} {r['tier']:>4} {r['kind']:<11} "
              f"{r['physics']:<10} {_fmt(r['stroke_mm'], '9.2f')} "
              f"{_fmt(r['strain_pct'], '8.2f')} {_fmt(r['polarity'], '4d')} "
              f"{_fmt(r['blocked_N'], '10.3g')} {_fmt(r['dof'], '4d')} "
              f"{_fmt(r['authority'], '10.3g')}")
    print(f"\n{len(rows)} records, {n_err} errors")
    return 1 if n_err else 0


if __name__ == "__main__":
    raise SystemExit(main())
