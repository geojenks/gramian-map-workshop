"""Turn a per-joint table of a jointed finger into an actuator record.

    python tools/joint_table_to_record.py my_finger.csv --out my_finger.json
    python tools/joint_table_to_record.py my_finger.csv --out my_finger.json \\
        --compare data/records/shadow_hand_finger.json \\
        --compare-out outputs/joint_table/my_finger_vs_stored.txt

The table is the analytical route into the map (ruling of 28 Oct on
GitHub issue 16): for each joint, its axis, where it sits, its range, its
torque limit and its spring stiffness. The file format is in
docs/JOINT_TABLE_FORMAT.md: a details block of `key,value` rows, then one
table whose header row starts with `joint`. Every unit is named in its
column (`pos_x_mm`, `max_deg`, `torque_Nm`) and converted to SI here.

The Jacobian at any pose comes from the table: for a hinge, its column is
the joint axis crossed with the vector from the joint to the tip. The
record is built from it:

  free       the tip path as the one input drives every joint from rest
             towards the upper end of its range, in proportion;
  blocked    with the tip pinned at rest, the largest push along the first
             motion direction that every joint holds within its torque
             limit, scaled by the input (only when every torque is known);
  stiffness  the transverse (+Y) tip stiffness from the joint springs at
             each free level (only when every stiffness is known).

The record is validated with the schema's own checker before it is
written. Nothing is overwritten: an existing --out file is refused.
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

# Unit factors to SI.
LENGTH_UNITS = {"mm": 1e-3, "cm": 1e-2, "m": 1.0, "um": 1e-6}
ANGLE_UNITS = {"deg": math.pi / 180.0, "rad": 1.0}
TORQUE_UNITS = {"Nm": 1.0, "Nmm": 1e-3, "mNm": 1e-3}
STIFFNESS_UNITS = {"Nm_per_rad": 1.0, "Nmm_per_rad": 1e-3,
                   "Nm_per_deg": 180.0 / math.pi,
                   "Nmm_per_deg": 1e-3 * 180.0 / math.pi}
# Table columns: prefix -> (unit table or None for unitless, what it is)
COLUMNS = {
    "axis_x": (None, "joint axis, x part"),
    "axis_y": (None, "joint axis, y part"),
    "axis_z": (None, "joint axis, z part"),
    "pos_x": (LENGTH_UNITS, "joint position, x"),
    "pos_y": (LENGTH_UNITS, "joint position, y"),
    "pos_z": (LENGTH_UNITS, "joint position, z"),
    "min": (ANGLE_UNITS, "lower end of the joint's range"),
    "max": (ANGLE_UNITS, "upper end of the joint's range"),
    "torque": (TORQUE_UNITS, "torque limit"),
    "stiffness": (STIFFNESS_UNITS, "spring stiffness"),
}
JOINT_NEEDS = tuple(COLUMNS)
TIP_NEEDS = ("pos_x", "pos_y", "pos_z")
MAY_BE_UNKNOWN = ("torque", "stiffness")
SOURCES = ("experiment", "literature", "simulation")
DETAIL_KEYS = ("id", "name", "source", "detail", "reference", "date",
               "actuation", "levels", "uncertainty", "notes")
REQUIRED_DETAILS = ("id", "name", "source", "detail")
DEFAULT_LEVELS = 21
# The workshop's reading rule for the motion direction (docs/RECORDS.md,
# as in tools/curves_to_record.py): the first level that moves more than
# this share of the full stroke.
DIRECTION_FLOOR = 0.02
# Polarity rule of the stored Shadow record's generator
# (gramian_map/generate_literature_records.py): sign of the axial part of
# the motion direction beyond this size, else 0.
POLARITY_FLOOR = 0.1
# A lever (|J column . direction|) below this, in metres, is no lever.
LEVER_FLOOR = 1e-12


class JointTableError(ValueError):
    """The file does not follow docs/JOINT_TABLE_FORMAT.md."""


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def _split_column(name):
    """'pos_x_mm' -> ('pos_x', 'mm'); 'axis_z' -> ('axis_z', None);
    an unknown column -> (None, None). A known quantity with a missing or
    unknown unit raises."""
    for prefix in sorted(COLUMNS, key=len, reverse=True):
        units, what = COLUMNS[prefix]
        if name == prefix:
            if units is None:
                return prefix, None
            raise JointTableError(
                f"column '{name}' ({what}) has no unit; write it as "
                + " or ".join(f"'{prefix}_{u}'" for u in units))
        if name.startswith(prefix + "_"):
            unit = name[len(prefix) + 1:]
            if units is None:
                raise JointTableError(
                    f"column '{name}': {what} has no unit; write '{prefix}'")
            if unit not in units:
                raise JointTableError(
                    f"column '{name}': unit '{unit}' for {what} is not clear; "
                    f"use one of {', '.join(units)}")
            return prefix, unit
    return None, None


def _number(s, where):
    try:
        v = float(s)
    except ValueError:
        raise JointTableError(f"{where}: '{s}' is not a number") from None
    if not math.isfinite(v):
        raise JointTableError(f"{where}: '{s}' is not a finite number")
    return v


def read_joint_table(path):
    """(details, joints, tip, info). Each joint is a dict of SI values
    (axis unit vector, pos (3,), lo, hi, torque or None, stiffness or
    None); tip is (3,). Raises JointTableError on a malformed file."""
    with open(path, encoding="utf-8-sig", newline="") as f:
        text = f.read()
    details, header, rows = {}, None, []
    for n, row in enumerate(csv.reader(io.StringIO(text)), start=1):
        cells = [c.strip() for c in row]
        if not any(cells) or cells[0].startswith("#"):
            continue
        if header is None:
            if cells[0] == "joint":
                header = cells
                continue
            key = cells[0]
            if key not in DETAIL_KEYS:
                raise JointTableError(
                    f"line {n}: unknown detail '{key}' (known: "
                    f"{', '.join(DETAIL_KEYS)}); is the table header, "
                    f"starting with 'joint', missing?")
            if key in details:
                raise JointTableError(f"line {n}: detail '{key}' given twice")
            details[key] = cells[1] if len(cells) > 1 else ""
            continue
        rows.append((n, cells))
    for key in REQUIRED_DETAILS:
        if not details.get(key):
            raise JointTableError(f"detail '{key}' is missing")
    if header is None:
        raise JointTableError("no table: a header row starting with 'joint' "
                              "is missing")
    cols, ignored, seen = {}, [], {}
    for i, name in enumerate(header[1:], start=1):
        prefix, unit = _split_column(name)
        if prefix is None:
            ignored.append(name)
            continue
        if prefix in seen:
            raise JointTableError(f"columns '{seen[prefix]}' and '{name}' "
                                  f"both give the {COLUMNS[prefix][1]}")
        seen[prefix] = name
        cols[prefix] = (i, unit)
    for prefix in JOINT_NEEDS:
        if prefix not in cols:
            units = COLUMNS[prefix][0]
            hint = prefix if units is None else f"{prefix}_<unit>"
            raise JointTableError(f"column '{hint}' ({COLUMNS[prefix][1]}) "
                                  f"is missing")
    joints, tip, names = [], None, set()
    for n, cells in rows:
        name = cells[0]
        if tip is not None:
            raise JointTableError(f"line {n}: rows after the 'tip' row; the "
                                  f"tip comes last")

        def get(prefix):
            i, unit = cols[prefix]
            s = cells[i] if i < len(cells) else ""
            where = f"line {n} ({name}), column '{seen[prefix]}'"
            if s == "":
                if prefix in MAY_BE_UNKNOWN:
                    raise JointTableError(
                        f"{where} is empty; give a value, or write 'unknown' "
                        f"if it is not known")
                raise JointTableError(f"{where} is empty")
            if s.lower() == "unknown":
                if prefix in MAY_BE_UNKNOWN:
                    return None
                raise JointTableError(f"{where}: only torque and stiffness "
                                      f"may be 'unknown'")
            v = _number(s, where)
            units = COLUMNS[prefix][0]
            return v if units is None else v * units[unit]

        if name == "tip":
            tip = np.array([get(p) for p in TIP_NEEDS])
            continue
        if not name:
            raise JointTableError(f"line {n}: the joint has no name")
        if name in names:
            raise JointTableError(f"line {n}: joint '{name}' listed twice")
        names.add(name)
        axis = np.array([get(p) for p in ("axis_x", "axis_y", "axis_z")])
        na = float(np.linalg.norm(axis))
        if na == 0.0:
            raise JointTableError(f"line {n} ({name}): the axis is zero")
        lo, hi = get("min"), get("max")
        if not lo < hi:
            raise JointTableError(f"line {n} ({name}): the range's lower end "
                                  f"is not below its upper end")
        if not lo <= 0.0 <= hi:
            raise JointTableError(
                f"line {n} ({name}): the range does not contain 0; the table "
                f"gives the finger at rest, where every joint is at 0")
        torque, stiff = get("torque"), get("stiffness")
        if torque is not None and torque <= 0.0:
            raise JointTableError(f"line {n} ({name}): the torque limit must "
                                  f"be above zero")
        if stiff is not None and stiff < 0.0:
            raise JointTableError(f"line {n} ({name}): the stiffness is "
                                  f"negative")
        joints.append({"name": name, "axis": axis / na,
                       "pos": np.array([get(p) for p in TIP_NEEDS]),
                       "lo": lo, "hi": hi, "torque": torque,
                       "stiffness": stiff})
    if not joints:
        raise JointTableError("no joint rows")
    if tip is None:
        raise JointTableError("the 'tip' row is missing: give the tip point "
                              "at rest after the last joint")
    return details, joints, tip, {"ignored": ignored}


# ----------------------------------------------------------------------
# kinematics (numpy only)
# ----------------------------------------------------------------------
def _rotation(axis, angle):
    """Rodrigues: rotation by `angle` about the unit `axis`."""
    k = np.array([[0.0, -axis[2], axis[1]],
                  [axis[2], 0.0, -axis[0]],
                  [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * k @ k


def forward(joints, tip, q):
    """(tip position (3,), axes (n, 3), joint positions (n, 3)) at pose q.
    A serial chain in table order; each joint turns everything after it
    about its own axis through its own point (positions given at rest)."""
    R, t = np.eye(3), np.zeros(3)
    axes, pos = [], []
    for j, qi in zip(joints, q):
        p = R @ j["pos"] + t
        a = R @ j["axis"]
        axes.append(a)
        pos.append(p)
        Ri = _rotation(a, qi)
        R, t = Ri @ R, Ri @ (t - p) + p
    return R @ tip + t, np.array(axes), np.array(pos)


def jacobian(joints, tip, q):
    """(3, n) tip Jacobian: column i is axis_i x (tip - joint_i)."""
    x, axes, pos = forward(joints, tip, q)
    return np.cross(axes, x - pos).T


def flex_pose(joints, u):
    """The pose at input u: every joint at fraction u of its upper end."""
    return np.array([u * j["hi"] for j in joints])


# ----------------------------------------------------------------------
# the record
# ----------------------------------------------------------------------
def _free_block(joints, tip, levels):
    us = np.linspace(0.0, 1.0, levels)
    rest = forward(joints, tip, np.zeros(len(joints)))[0]
    disp = [forward(joints, tip, flex_pose(joints, u))[0] - rest for u in us]
    return {"u": [[float(u)] for u in us],
            "tip_displacement_m": [[float(c) for c in d] for d in disp],
            "uncertainty": None}


def first_motion(joints, tip):
    """Unit direction the tip starts to move in at rest: J(0) . q_max."""
    J = jacobian(joints, tip, np.zeros(len(joints)))
    v = J @ np.array([j["hi"] for j in joints])
    n = float(np.linalg.norm(v))
    return None if n == 0.0 else v / n


def _blocked_block(joints, tip, us):
    """Tip pinned at rest. At full input each joint i can carry up to its
    torque limit; a tip force s e loads joint i with s (J_i . e). The push
    is the largest s that no joint exceeds, scaled by the input."""
    if any(j["torque"] is None for j in joints):
        return None, None
    e = first_motion(joints, tip)
    if e is None:
        return None, None
    J = jacobian(joints, tip, np.zeros(len(joints)))
    levers = J.T @ e
    caps = [j["torque"] / abs(c) for j, c in zip(joints, levers)
            if abs(c) > LEVER_FLOOR]
    if not caps:
        return None, None
    s_max = min(caps)
    binding = [j["name"] for j, c in zip(joints, levers)
               if abs(c) > LEVER_FLOOR and j["torque"] / abs(c) == s_max]
    force = [[float(u * s_max * c) for c in e] for u in us]
    block = {
        "u": [[float(u)] for u in us],
        "force_axial_N": [f[0] for f in force],
        "force_transverse_N": [float(math.hypot(f[1], f[2])) for f in force],
        "force_N": force,
        "method": ("from the joint table: tip pinned at rest; the largest "
                   "push along the first motion direction that every joint "
                   "holds within its torque limit, scaled by the input "
                   "(tools/joint_table_to_record.py)"),
        "uncertainty": None,
    }
    return block, binding


def _stiffness_block(joints, tip, us):
    """Transverse tip stiffness at each free level: 1 / C_yy with the tip
    compliance C = J K^-1 J^T, so C_yy = sum_i J_iy^2 / k_i. A joint with
    no spring that moves the tip along Y makes it 0."""
    if any(j["stiffness"] is None for j in joints):
        return None
    ks = []
    for u in us:
        Jy = jacobian(joints, tip, flex_pose(joints, u))[1]
        c, free_joint = 0.0, False
        for j, jy in zip(joints, Jy):
            if abs(jy) <= LEVER_FLOOR:
                continue
            if j["stiffness"] == 0.0:
                free_joint = True
                break
            c += jy * jy / j["stiffness"]
        if free_joint:
            ks.append(0.0)
        elif c == 0.0:
            return None
        else:
            ks.append(1.0 / c)
    return {"u": [[float(u)] for u in us],
            "k_N_per_m": ks,
            "direction": "transverse (+Y canonical)",
            "method": ("from the joint table: 1 / C_yy, tip compliance "
                       "C = J K^-1 J^T at each free level "
                       "(tools/joint_table_to_record.py)"),
            "uncertainty": None}


def _scalars(free, blocked, stiffness, n_joints):
    d = np.asarray(free["tip_displacement_m"], float)
    mags = np.linalg.norm(d, axis=1)
    stroke = float(mags.max())
    direction = None
    for v, m in zip(d, mags):
        if m > DIRECTION_FLOOR * stroke:
            direction = (v / m).tolist()
            break
    pol = 0
    if direction is not None:
        if direction[0] < -POLARITY_FLOOR:
            pol = -1
        elif direction[0] > POLARITY_FLOOR:
            pol = 1
    force = None
    if blocked:
        force = float(np.linalg.norm(blocked["force_N"], axis=1).max())
    k = None
    if stiffness:
        k = float(min(stiffness["k_N_per_m"]))
    return {"free_stroke_m": stroke,
            "free_direction": direction,
            "polarity_axial": pol,
            "blocked_force_N": force,
            "stiffness_N_per_m": k,
            "effective_dof": n_joints}


def build_record(path):
    """(record, notes). Raises JointTableError on a malformed file."""
    details, joints, tip, info = read_joint_table(path)
    notes = []
    if info["ignored"]:
        notes.append("ignored columns: " + ", ".join(info["ignored"]))
    if not details["id"].replace("_", "").replace("-", "").isalnum():
        raise JointTableError(f"id '{details['id']}': use letters, digits, "
                              f"_ or - only (it becomes a file name)")
    if details["source"] not in SOURCES:
        raise JointTableError(f"source '{details['source']}' is not one of "
                              f"{', '.join(SOURCES)}")
    levels = DEFAULT_LEVELS
    if details.get("levels"):
        lv = _number(details["levels"], "detail 'levels'")
        if lv != int(lv) or lv < 3:
            raise JointTableError("detail 'levels' must be a whole number, "
                                  "3 or more")
        levels = int(lv)
    unc = details.get("uncertainty")
    unc = 0.1 if not unc else _number(unc, "detail 'uncertainty'")
    free = _free_block(joints, tip, levels)
    if max(np.linalg.norm(free["tip_displacement_m"], axis=1)) == 0.0:
        raise JointTableError("the tip does not move: no joint moves it as "
                              "the joints go towards their upper ends")
    us = [u[0] for u in free["u"]]
    blocked, binding = _blocked_block(joints, tip, us)
    if blocked is None:
        notes.append("no blocked force: a torque limit is unknown, or no "
                     "joint pushes the pinned tip")
    else:
        notes.append("blocked force limited by joint "
                     + ", ".join(binding))
    stiffness = _stiffness_block(joints, tip, us)
    if stiffness is None:
        notes.append("no stiffness: a joint stiffness is unknown")
    rest_tip = forward(joints, tip, np.zeros(len(joints)))[0]
    length = float(np.linalg.norm(rest_tip - joints[0]["pos"]))
    table = "; ".join(
        f"{j['name']} at {[round(float(c), 6) for c in j['pos']]} m, axis "
        f"{[round(float(c), 6) for c in j['axis']]}, range "
        f"[{j['lo']:.6g}, {j['hi']:.6g}] rad, torque "
        f"{'unknown' if j['torque'] is None else format(j['torque'], '.6g') + ' N.m'}"
        f", stiffness "
        f"{'unknown' if j['stiffness'] is None else format(j['stiffness'], '.6g') + ' N.m/rad'}"
        for j in joints)
    src_note = (f"Made with tools/joint_table_to_record.py from "
                f"{os.path.basename(path)} (sha256 {sha256_file(path)}). "
                f"Joints: {table}; tip at "
                f"{[round(float(c), 6) for c in tip]} m.")
    rec = {
        "schema_version": ar.SCHEMA_VERSION,
        "id": details["id"],
        "name": details["name"],
        "source": {"kind": details["source"], "detail": details["detail"],
                   "date": details.get("date") or None,
                   "reference": details.get("reference") or None},
        "metadata": {
            "actuation_physics": details.get("actuation") or "electric",
            "length_m": length, "mass_kg": None,
            "material_volume_m3": None, "materials": {},
            "notes": " ".join(x for x in (details.get("notes"), src_note)
                              if x)},
        "input": {
            "channels": [{"name": "drive level (fraction of each joint's "
                                  "upper range end and of its torque limit)",
                          "unit": None, "range": [0.0, 1.0], "rated": 1.0}],
            "energy_convention":
                "mechanical work delivered at the input port (J)",
            "work_J": None},
        "port": {"name": "tip", "description": "the table's tip point"},
        "free": free,
        "blocked": blocked,
        "stiffness": stiffness,
        "scalars": _scalars(free, blocked, stiffness, len(joints)),
        "shape": None,
        "uncertainty": {"default_fraction": unc},
    }
    return rec, notes


# ----------------------------------------------------------------------
# comparison with an existing record
# ----------------------------------------------------------------------
COMPARED = (("metadata", "length_m"), ("metadata", "actuation_physics"),
            ("scalars", "free_stroke_m"), ("scalars", "free_direction"),
            ("scalars", "polarity_axial"), ("scalars", "blocked_force_N"),
            ("scalars", "stiffness_N_per_m"), ("scalars", "effective_dof"),
            ("input", "channels"))


def _differ(a, b, tol):
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) \
            and not isinstance(a, bool) and not isinstance(b, bool):
        return abs(float(a) - float(b)) > tol
    if isinstance(a, list) and isinstance(b, list) and len(a) == len(b) \
            and all(isinstance(x, (int, float)) for x in a + b):
        return max(abs(float(x) - float(y)) for x, y in zip(a, b)) > tol
    return a != b


def compare(new, old, tol):
    """[(field, new value, old value, agrees)] for the headline fields,
    then which curve blocks each record has."""
    out = []
    for block, key in COMPARED:
        a = (new.get(block) or {}).get(key)
        b = (old.get(block) or {}).get(key)
        out.append((f"{block}.{key}", a, b, not _differ(a, b, tol)))
    for block in ("free", "blocked", "stiffness", "shape"):
        out.append((f"{block} block", new.get(block) is not None,
                    old.get(block) is not None,
                    (new.get(block) is None) == (old.get(block) is None)))
    return out


def compare_text(rows, new_id, old_path, tol):
    lines = [f"{new_id} (from the joint table) against {old_path}",
             f"numbers agree when within {tol:g} of each other", ""]
    for field, a, b, ok in rows:
        lines.append(f"{'same' if ok else 'DIFFERS'}  {field}")
        if not ok or isinstance(a, (int, float, list)):
            lines.append(f"    table:  {json.dumps(a)}")
            lines.append(f"    stored: {json.dumps(b)}")
    return "\n".join(lines) + "\n"


def _refuse_existing(path):
    if os.path.exists(path):
        raise SystemExit(f"refusing to overwrite {path}; give a new name")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("table", help="the filled joint table (CSV)")
    ap.add_argument("--out", required=True, help="record to write (JSON)")
    ap.add_argument("--compare", help="an existing record to compare with")
    ap.add_argument("--compare-out", help="also write the comparison here "
                                          "(.txt)")
    ap.add_argument("--tol", type=float, default=1e-6,
                    help="numbers closer than this agree (default 1e-6, the "
                         "stored Shadow record's rounding)")
    a = ap.parse_args(argv)
    _refuse_existing(a.out)
    if a.compare_out:
        _refuse_existing(a.compare_out)
    try:
        rec, notes = build_record(a.table)
    except JointTableError as e:
        print(f"{a.table}: cannot convert: {e}", file=sys.stderr)
        return 1
    errors, warnings = ar.validate(rec)
    for w in warnings:
        print(f"warn   {w}")
    if errors:
        for e in errors:
            print(f"ERROR  {e}", file=sys.stderr)
        return 1
    for n in notes:
        print(f"note   {n}")
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w", encoding="utf-8", newline="\n") as f:
        json.dump(rec, f, indent=1)
        f.write("\n")
    print(f"wrote {a.out} (tier {ar.tier(rec)})")
    if a.compare:
        with open(a.compare, encoding="utf-8") as f:
            old = json.load(f)
        text = compare_text(compare(rec, old, a.tol), rec["id"],
                            a.compare.replace("\\", "/"), a.tol)
        print(text, end="")
        if a.compare_out:
            os.makedirs(os.path.dirname(os.path.abspath(a.compare_out)),
                        exist_ok=True)
            with open(a.compare_out, "w", encoding="utf-8",
                      newline="\n") as f:
                f.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
