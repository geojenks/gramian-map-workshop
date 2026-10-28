"""Turn a filled test-curve file into an actuator record.

    python tools/curves_to_record.py my_curves.csv --out my_actuator.json
    python tools/curves_to_record.py my_curves.csv --out my_actuator.json \\
        --compare data/records/mosadegh2014_fPN.json
    python tools/curves_to_record.py --from-record data/records/X.json \\
        --out X_curves.csv

The file format is in docs/TEST_CURVE_FORMAT.md: a details block of
`key,value` rows, then one table whose header row starts with `curve`.
Every unit is named in its column (`input_kPa`, `tip_dx_mm`, `force_N`)
and converted to SI here, by the same factors as forms/record_form.html.

The record is validated with the schema's own checker before it is
written. Nothing is overwritten: an existing --out file is refused.

--compare prints, curve by curve, how the new record differs from an
existing one. --from-record goes the other way: it writes a filled file
from a record's free tip path and blocked force, so a round trip can be
checked.
"""
import argparse
import csv
import hashlib
import io
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gramian_workshop.vendor import actuator_record as ar  # noqa: E402

# Unit factors to SI. The same values as forms/record_form.html (UNITS).
INPUT_UNITS = {"kPa": (1e3, "Pa"), "Pa": (1.0, "Pa"), "bar": (1e5, "Pa"),
               "psi": (6894.757, "Pa"), "mm": (1e-3, "m"), "m": (1.0, "m"),
               "N": (1.0, "N"), "V": (1.0, None), "A": (1.0, None)}
LENGTH_UNITS = {"mm": 1e-3, "m": 1.0, "cm": 1e-2, "um": 1e-6}
FORCE_UNITS = {"N": 1.0, "mN": 1e-3, "gf": 9.80665e-3, "kgf": 9.80665}
ANGLE_UNITS = {"deg": math.pi / 180.0, "rad": 1.0}

# Table columns: name prefix -> (unit table, what it is)
COLUMNS = {
    "input": (INPUT_UNITS, "input level"),
    "tip_dx": (LENGTH_UNITS, "free tip displacement along X"),
    "tip_dy": (LENGTH_UNITS, "free tip displacement along Y"),
    "tip_dz": (LENGTH_UNITS, "free tip displacement along Z"),
    "angle": (ANGLE_UNITS, "free bending angle"),
    "x": (LENGTH_UNITS, "backbone point x"),
    "y": (LENGTH_UNITS, "backbone point y"),
    "z": (LENGTH_UNITS, "backbone point z"),
    "force_x": (FORCE_UNITS, "blocked force along X"),
    "force_y": (FORCE_UNITS, "blocked force along Y"),
    "force_z": (FORCE_UNITS, "blocked force along Z"),
    "force": (FORCE_UNITS, "blocked force size"),
}
CURVES = ("free", "blocked", "backbone")
SOURCES = ("experiment", "literature", "simulation")
MOTIONS = ("bends", "extends", "contracts")
FORCE_DIRECTIONS = ("axial", "sideways")
DETAIL_KEYS = ("id", "name", "source", "detail", "reference", "date",
               "actuation", "input_name", "motion", "force_direction",
               "blocked_method", "uncertainty", "notes")
REQUIRED_DETAILS = ("id", "name", "source", "detail", "motion")
# The workshop's reading rule for the motion direction (docs/RECORDS.md):
# the first level that moves more than this share of the full stroke.
DIRECTION_FLOOR = 0.02


class CurveFileError(ValueError):
    """The file does not follow docs/TEST_CURVE_FORMAT.md."""


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


# ----------------------------------------------------------------------
# reading the file
# ----------------------------------------------------------------------
def _split_column(name):
    """'tip_dx_mm' -> ('tip_dx', 'mm', factor); None if not a known
    quantity. Raises if the quantity is known but the unit is not."""
    best = None
    for q in COLUMNS:
        if name == q or name.startswith(q + "_"):
            if best is None or len(q) > len(best):
                best = q
    if best is None:
        return None
    unit = name[len(best) + 1:]
    table = COLUMNS[best][0]
    if unit not in table:
        if best in ("x", "y", "z") and unit == "":
            raise CurveFileError(f"column '{name}': give a unit, e.g. "
                                 f"'{best}_mm'")
        raise CurveFileError(
            f"column '{name}': unit '{unit}' is not one of "
            f"{', '.join(table)} for the {COLUMNS[best][1]}")
    f = table[unit]
    return best, unit, f[0] if isinstance(f, tuple) else f


def _number(s, where):
    try:
        v = float(s)
    except ValueError:
        raise CurveFileError(f"{where}: '{s}' is not a number") from None
    if not math.isfinite(v):
        raise CurveFileError(f"{where}: '{s}' is not a finite number")
    return v


def read_curve_file(path):
    """(details dict, rows list of dicts in SI, column notes). Rows carry
    'curve', 'u' and whichever quantities the file gives, converted."""
    with open(path, encoding="utf-8-sig", newline="") as f:
        lines = list(csv.reader(f))
    details, header, rows, ignored = {}, None, [], []
    input_unit = None
    for n, raw in enumerate(lines, start=1):
        cells = [c.strip() for c in raw]
        while cells and cells[-1] == "":
            cells.pop()
        if not cells or cells[0].startswith("#"):
            continue
        if header is None:
            if cells[0].lower() == "curve":
                header = [c for c in cells]
                cols = {}
                for i, name in enumerate(header[1:], start=1):
                    if name == "point":
                        cols[i] = ("point", None, 1.0)
                        continue
                    sp = _split_column(name)
                    if sp is None:
                        ignored.append(name)
                        continue
                    if sp[0] in [c[0] for c in cols.values()]:
                        raise CurveFileError(f"line {n}: two columns give "
                                             f"the {COLUMNS[sp[0]][1]}")
                    cols[i] = sp
                    if sp[0] == "input":
                        input_unit = sp[1]
                if input_unit is None:
                    raise CurveFileError(
                        f"line {n}: the table needs an input column, e.g. "
                        f"'input_kPa' (units: {', '.join(INPUT_UNITS)})")
                continue
            key = cells[0]
            if key not in DETAIL_KEYS:
                raise CurveFileError(
                    f"line {n}: '{key}' is not a detail this format knows "
                    f"({', '.join(DETAIL_KEYS)}) and no table header "
                    f"('curve,input_...') has been seen yet")
            if key in details:
                raise CurveFileError(f"line {n}: '{key}' given twice")
            details[key] = ",".join(cells[1:]).strip()
            continue
        curve = cells[0].lower()
        if curve not in CURVES:
            raise CurveFileError(f"line {n}: curve '{cells[0]}' is not one "
                                 f"of {', '.join(CURVES)}")
        row = {"curve": curve, "line": n}
        for i, (q, unit, factor) in cols.items():
            s = cells[i] if i < len(cells) else ""
            if s == "":
                continue
            v = _number(s, f"line {n}, column '{header[i]}'")
            row["u" if q == "input" else q] = v * factor
        if "u" not in row:
            raise CurveFileError(f"line {n}: no input value")
        rows.append(row)
    if header is None:
        raise CurveFileError("no table found: the table's header row must "
                             "start with 'curve'")
    if not rows:
        raise CurveFileError("the table has no rows")
    missing = [k for k in REQUIRED_DETAILS if not details.get(k)]
    if missing:
        raise CurveFileError(f"missing details: {', '.join(missing)}")
    return details, rows, {"input_unit": input_unit, "ignored": ignored}


# ----------------------------------------------------------------------
# building the record
# ----------------------------------------------------------------------
def _sorted_unique(rows, curve):
    rows = sorted(rows, key=lambda r: r["u"])
    us = [r["u"] for r in rows]
    if len(set(us)) != len(us):
        dup = sorted({u for u in us if us.count(u) > 1})
        raise CurveFileError(
            f"{curve}: more than one row at input {dup} (SI); average "
            f"repeats first and give their spread as 'uncertainty'")
    return rows


def _all_or_none(rows, keys, curve, what):
    has = [all(k in r for k in keys) for r in rows]
    some = [any(k in r for k in keys) for r in rows]
    if any(some) and not all(has):
        bad = [r["line"] for r, h in zip(rows, has) if not h]
        raise CurveFileError(f"{curve}: {what} must be given on every "
                             f"{curve} row (missing on lines {bad})")
    return all(has) and bool(rows)


def _free_block(free_rows, bb_rows, notes):
    """(free block or None, length from the rest backbone or None)."""
    free_rows = _sorted_unique(free_rows, "free") if free_rows else []
    has_tip = _all_or_none(free_rows, ("tip_dx", "tip_dy"), "free",
                           "tip_dx and tip_dy")
    has_angle = _all_or_none(free_rows, ("angle",), "free", "angle")
    if free_rows and not (has_tip or has_angle):
        raise CurveFileError("free rows give neither a tip displacement "
                             "(tip_dx, tip_dy) nor an angle")
    _all_or_none(free_rows, ("tip_dz",), "free", "tip_dz")
    length = None
    if bb_rows:
        if has_tip:
            raise CurveFileError("the tip path is given twice: as free "
                                 "tip_dx/tip_dy and as backbone points; "
                                 "give one")
        levels = {}
        for r in bb_rows:
            for k in ("point", "x", "y"):
                if k not in r:
                    raise CurveFileError(f"backbone line {r['line']}: "
                                         f"needs point, x and y")
            levels.setdefault(r["u"], []).append(r)
        if 0.0 not in levels:
            raise CurveFileError("backbone: give the rest shape at input 0, "
                                 "so tip displacements can be measured "
                                 "from it")
        counts = {len(v) for v in levels.values()}
        if len(counts) != 1 or counts.pop() < 2:
            raise CurveFileError("backbone: every input level needs the "
                                 "same number of points (at least 2)")
        pts = {}
        for u, rs in levels.items():
            rs = sorted(rs, key=lambda r: r["point"])
            idx = [r["point"] for r in rs]
            if len(set(idx)) != len(idx):
                raise CurveFileError(f"backbone: point numbers repeat at "
                                     f"input {u} (SI)")
            pts[u] = np.array([[r["x"], r["y"], r.get("z", 0.0)]
                               for r in rs])
        us = sorted(pts)
        rest = pts[0.0]
        length = float(np.linalg.norm(np.diff(rest, axis=0), axis=1).sum())
        tipdir = rest[-1] - rest[0]
        ang = math.degrees(math.atan2(math.hypot(tipdir[1], tipdir[2]),
                                      tipdir[0]))
        if ang > 10.0:
            notes.append(f"warning: the rest backbone runs {ang:.0f} deg "
                         f"off +X; the format asks for X from base to tip")
        disp = [(pts[u][-1] - rest[-1]).tolist() for u in us]
        block = {"u": [[u] for u in us], "tip_displacement_m": disp}
        if has_angle:
            fu = [r["u"] for r in free_rows]
            if fu != us:
                raise CurveFileError("free angle rows and backbone levels "
                                     "must be at the same inputs")
            block["bend_angle_rad"] = [r["angle"] for r in free_rows]
        block["uncertainty"] = None
        notes.append(f"tip path from the last backbone point at "
                     f"{len(us)} levels; rest length {length:.4g} m is the "
                     f"rest backbone's arc length")
        return block, length
    if not free_rows:
        return None, None
    block = {"u": [[r["u"]] for r in free_rows]}
    if has_tip:
        block["tip_displacement_m"] = [
            [r["tip_dx"], r["tip_dy"], r.get("tip_dz", 0.0)]
            for r in free_rows]
    if has_angle:
        block["bend_angle_rad"] = [r["angle"] for r in free_rows]
    block["uncertainty"] = None
    return block, None


def _blocked_block(rows, details):
    if not rows:
        return None
    rows = _sorted_unique(rows, "blocked")
    has_xy = _all_or_none(rows, ("force_x", "force_y"), "blocked",
                          "force_x and force_y")
    has_size = _all_or_none(rows, ("force",), "blocked", "force")
    _all_or_none(rows, ("force_z",), "blocked", "force_z")
    if has_xy and has_size:
        raise CurveFileError("blocked: give force_x/force_y or force, "
                             "not both")
    if not (has_xy or has_size):
        if any("force_z" in r for r in rows):
            raise CurveFileError("blocked: force_z needs force_x and "
                                 "force_y beside it")
        raise CurveFileError("blocked rows give no force")
    method = details.get("blocked_method") or details["detail"]
    block = {"u": [[r["u"]] for r in rows]}
    if has_xy:
        vec = [[r["force_x"], r["force_y"], r.get("force_z", 0.0)]
               for r in rows]
        block["force_axial_N"] = [v[0] for v in vec]
        block["force_transverse_N"] = [math.hypot(v[1], v[2]) for v in vec]
        block["force_N"] = vec
    else:
        fd = details.get("force_direction")
        if fd not in FORCE_DIRECTIONS:
            raise CurveFileError(
                "blocked: a force size alone needs the detail "
                "'force_direction' set to 'axial' (signed along X: + "
                "pushes out, - pulls in) or 'sideways' (size across the "
                "finger, direction not stated)")
        sizes = [r["force"] for r in rows]
        if fd == "axial":
            block["force_axial_N"] = sizes
            block["force_transverse_N"] = None
        else:
            block["force_axial_N"] = None
            block["force_transverse_N"] = sizes
    block["method"] = method
    block["uncertainty"] = None
    return block


def _scalars(details, free, blocked):
    """Headline numbers by the workshop's reading rules (docs/RECORDS.md;
    gramian_map's record_from_experiment.py), except polarity, which is
    the stated motion: a bender's polarity is left null, as in the
    published-data records."""
    motion = details["motion"]
    stroke = direction = None
    if free and free.get("tip_displacement_m"):
        d = np.asarray(free["tip_displacement_m"], float)
        mags = np.linalg.norm(d, axis=1)
        stroke = float(mags.max())
        for v, m in zip(d, mags):
            if m > DIRECTION_FLOOR * stroke:
                direction = (v / m).tolist()
                break
    elif free and free.get("bend_angle_rad") and motion == "bends":
        a = np.asarray(free["bend_angle_rad"], float)
        top = float(np.abs(a).max())
        for v in a:
            if abs(v) > DIRECTION_FLOOR * top:
                direction = [0.0, float(np.sign(v)), 0.0]
                break
    elif motion in ("extends", "contracts") and free:
        direction = [1.0 if motion == "extends" else -1.0, 0.0, 0.0]
    force = None
    if blocked:
        if blocked.get("force_N"):
            force = float(np.linalg.norm(blocked["force_N"], axis=1).max())
        else:
            col = blocked.get("force_axial_N") or blocked["force_transverse_N"]
            force = float(np.abs(col).max())
    return {
        "free_stroke_m": stroke,
        "free_direction": direction,
        "polarity_axial": {"bends": None, "extends": 1,
                           "contracts": -1}[motion],
        "blocked_force_N": force,
        "stiffness_N_per_m": None,
        "effective_dof": None,
    }


def build_record(path):
    """(record, notes). Raises CurveFileError on a malformed file."""
    details, rows, info = read_curve_file(path)
    notes = []
    if info["ignored"]:
        notes.append("ignored columns: " + ", ".join(info["ignored"]))
    if not details["id"].replace("_", "").replace("-", "").isalnum():
        raise CurveFileError(f"id '{details['id']}': use letters, digits, "
                             f"_ or - only (it becomes a file name)")
    if details["source"] not in SOURCES:
        raise CurveFileError(f"source '{details['source']}' is not one of "
                             f"{', '.join(SOURCES)}")
    if details["motion"] not in MOTIONS:
        raise CurveFileError(f"motion '{details['motion']}' is not one of "
                             f"{', '.join(MOTIONS)}")
    by = {c: [r for r in rows if r["curve"] == c] for c in CURVES}
    free, bb_length = _free_block(by["free"], by["backbone"], notes)
    blocked = _blocked_block(by["blocked"], details)
    if free is None and blocked is None:
        raise CurveFileError("no free or blocked rows")
    length = bb_length
    unc = details.get("uncertainty")
    unc = 0.1 if not unc else _number(unc, "detail 'uncertainty'")
    factor, si_unit = INPUT_UNITS[info["input_unit"]]
    name = details.get("input_name") or "chamber pressure"
    if si_unit is None:
        name = f"{name} ({info['input_unit']})"
    us = [u[0] for b in (free, blocked) if b for u in b["u"]]
    src_note = (f"made with tools/curves_to_record.py from "
                f"{os.path.basename(path)} (sha256 {sha256_file(path)})")
    rec = {
        "schema_version": ar.SCHEMA_VERSION,
        "id": details["id"],
        "name": details["name"],
        "source": {"kind": details["source"], "detail": details["detail"],
                   "date": details.get("date") or None,
                   "reference": details.get("reference") or None},
        "metadata": {
            "actuation_physics": details.get("actuation") or "pneumatic",
            "length_m": length, "mass_kg": None,
            "material_volume_m3": None, "materials": {},
            "notes": " ".join(x for x in (details.get("notes"), src_note)
                              if x)},
        "input": {
            "channels": [{"name": name, "unit": si_unit,
                          "range": [min(0.0, min(us)), max(0.0, max(us))],
                          "rated": None}],
            "energy_convention":
                "mechanical work delivered at the input port (J)",
            "work_J": None},
        "port": {"name": "tip", "description": "free end, as measured"},
        "free": free,
        "blocked": blocked,
        "stiffness": None,
        "scalars": _scalars(details, free, blocked),
        "shape": None,
        "uncertainty": {"default_fraction": unc},
    }
    return rec, notes


# ----------------------------------------------------------------------
# a filled file from a record (for examples and round trips)
# ----------------------------------------------------------------------
def _fmt(x):
    """Text for a value divided back out of SI: 12 significant figures,
    which drops the round-off of the division and keeps the data."""
    return repr(float(f"{x:.12g}"))


def record_to_curves(rec):
    """CSV text of a record's free tip path and blocked force, in kPa, mm
    and N. Only what the format can carry back: no stiffness, no shape."""
    ch = rec["input"]["channels"][0]
    if ch.get("unit") != "Pa":
        raise CurveFileError("--from-record writes kPa; this record's "
                             f"input unit is {ch.get('unit')!r}")
    free = rec.get("free") or {}
    blk = rec.get("blocked") or {}
    s = rec.get("scalars") or {}
    pol = s.get("polarity_axial")
    motion = {1: "extends", -1: "contracts"}.get(pol, "bends")
    det = [("id", rec["id"] + "_from_curves"),
           ("name", rec.get("name", rec["id"])),
           ("source", rec["source"]["kind"]),
           ("detail", rec["source"].get("detail") or ""),
           ("reference", rec["source"].get("reference") or ""),
           ("date", rec["source"].get("date") or ""),
           ("actuation", rec["metadata"].get("actuation_physics") or ""),
           ("input_name", ch.get("name") or ""),
           ("motion", motion)]
    size_col = None
    if blk:
        if blk.get("force_N"):
            size_col = None
        elif blk.get("force_axial_N") is not None:
            size_col = "force_axial_N"
            det.append(("force_direction", "axial"))
        else:
            size_col = "force_transverse_N"
            det.append(("force_direction", "sideways"))
        det.append(("blocked_method", blk.get("method") or ""))
    det.append(("uncertainty",
                str((rec.get("uncertainty") or {}).get("default_fraction")
                    or 0.1)))
    det.append(("notes", (rec["metadata"].get("notes") or "")))
    vec = bool(blk.get("force_N"))
    head = ["curve", "input_kPa", "tip_dx_mm", "tip_dy_mm", "tip_dz_mm"]
    head += ["force_x_N", "force_y_N", "force_z_N"] if vec else ["force_N"]
    out = io.StringIO()
    w = csv.writer(out, lineterminator="\n")
    w.writerow([f"# Test curves for one actuator: docs/TEST_CURVE_FORMAT.md. "
                f"Written by tools/curves_to_record.py --from-record from "
                f"record {rec['id']}."])
    for k, v in det:
        w.writerow([k, v])
    w.writerow([])
    w.writerow(head)
    blank = [""] * (len(head) - 5)
    for u, d in zip(free.get("u") or [], free.get("tip_displacement_m")
                    or []):
        w.writerow(["free", _fmt(u[0] / 1e3)] +
                   [_fmt(c / 1e-3) for c in d] + blank)
    for i, u in enumerate(blk.get("u") or []):
        if vec:
            f = [_fmt(c) for c in blk["force_N"][i]]
        else:
            f = [_fmt(blk[size_col][i])]
        w.writerow(["blocked", _fmt(u[0] / 1e3), "", "", ""] + f)
    return out.getvalue()


# ----------------------------------------------------------------------
# comparing with an existing record
# ----------------------------------------------------------------------
CURVE_FIELDS = (("input.channels[0].range", lambda r: r["input"]["channels"][0]["range"]),
                ("free.u", lambda r: (r.get("free") or {}).get("u")),
                ("free.tip_displacement_m",
                 lambda r: (r.get("free") or {}).get("tip_displacement_m")),
                ("blocked.u", lambda r: (r.get("blocked") or {}).get("u")),
                ("blocked.force_axial_N",
                 lambda r: (r.get("blocked") or {}).get("force_axial_N")),
                ("blocked.force_transverse_N",
                 lambda r: (r.get("blocked") or {}).get("force_transverse_N")),
                ("blocked.force_N",
                 lambda r: (r.get("blocked") or {}).get("force_N")))
SCALAR_FIELDS = ("free_stroke_m", "free_direction", "polarity_axial",
                 "blocked_force_N")


def compare(new, old):
    """Per curve field: 'same (both absent)', 'identical', or the largest
    difference; then the headline numbers side by side."""
    out = {"curves": {}, "scalars": {}}
    for name, get in CURVE_FIELDS:
        a, b = get(new), get(old)
        if a is None and b is None:
            out["curves"][name] = {"status": "both absent"}
            continue
        if a is None or b is None:
            out["curves"][name] = {"status": "only in "
                                   + ("old" if a is None else "new")}
            continue
        A, B = np.asarray(a, float), np.asarray(b, float)
        if A.shape != B.shape:
            out["curves"][name] = {"status": "shapes differ",
                                   "new": list(A.shape), "old": list(B.shape)}
            continue
        diff = float(np.abs(A - B).max()) if A.size else 0.0
        out["curves"][name] = {
            "status": "identical" if diff == 0.0 else "differs",
            "max_abs_diff": diff,
            "max_rel_diff": float((np.abs(A - B)
                                   / np.maximum(np.abs(B), 1e-300)).max())
            if A.size else 0.0}
    for k in SCALAR_FIELDS:
        out["scalars"][k] = {"new": new["scalars"].get(k),
                             "old": old["scalars"].get(k)}
    return out


def _compare_text(cmp, new_id, old_path):
    L = [f"round trip: {new_id} (new) against {old_path} (old)",
         "curves:"]
    for k, v in cmp["curves"].items():
        extra = ""
        if v["status"] == "differs":
            extra = (f": largest difference {v['max_abs_diff']:.3g} "
                     f"(relative {v['max_rel_diff']:.3g})")
        elif v["status"] == "shapes differ":
            extra = f": new {v['new']}, old {v['old']}"
        L.append(f"  {k:<28} {v['status']}{extra}")
    L.append("headline numbers (scalars):")
    for k, v in cmp["scalars"].items():
        L.append(f"  {k:<18} new {v['new']!r}")
        L.append(f"  {'':<18} old {v['old']!r}")
    return "\n".join(L)


# ----------------------------------------------------------------------
def _refuse_existing(path):
    if os.path.exists(path):
        raise SystemExit(f"refusing to overwrite {path}; choose another "
                         f"--out")


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("curves", nargs="?", help="filled test-curve file (CSV)")
    ap.add_argument("--out", required=True,
                    help="record (.json) to write; with --from-record, the "
                         "curve file (.csv) to write")
    ap.add_argument("--compare", metavar="RECORD",
                    help="print how the new record's curves and headline "
                         "numbers differ from this record's")
    ap.add_argument("--compare-out", metavar="FILE",
                    help="also write the comparison as JSON and text "
                         "(FILE.json, FILE.txt)")
    ap.add_argument("--from-record", metavar="RECORD",
                    help="write a filled curve file from this record instead")
    a = ap.parse_args(argv)

    if a.from_record:
        _refuse_existing(a.out)
        text = record_to_curves(ar.load(a.from_record))
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, "w", encoding="utf-8", newline="") as f:
            f.write(text)
        print(f"wrote {a.out} from {a.from_record}")
        return 0
    if not a.curves:
        ap.error("give a curve file, or --from-record")
    _refuse_existing(a.out)
    try:
        rec, notes = build_record(a.curves)
    except CurveFileError as e:
        print(f"ERROR: {a.curves}: {e}")
        return 1
    for n in notes:
        print(f"note: {n}")
    errors, warnings = ar.validate(rec)
    for w in warnings:
        print(f"warn: {w}")
    if errors:
        for e in errors:
            print(f"ERROR: {e}")
        print("not written: the record does not pass the schema check")
        return 1
    ar.save(rec, a.out)
    print(f"wrote {a.out} (tier {ar.tier(rec)}); check it with "
          f"python tools/check_record.py {a.out}")
    if a.compare:
        cmp = compare(rec, ar.load(a.compare))
        text = _compare_text(cmp, rec["id"], a.compare)
        print(text)
        if a.compare_out:
            for ext in (".json", ".txt"):
                _refuse_existing(a.compare_out + ext)
            with open(a.compare_out + ".json", "w", encoding="utf-8") as f:
                json.dump(cmp, f, indent=1)
                f.write("\n")
            with open(a.compare_out + ".txt", "w", encoding="utf-8") as f:
                f.write(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
