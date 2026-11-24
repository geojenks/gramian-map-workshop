"""Backwards lookup: from a task the hand cannot do, back to the records.

Given the direction a task needs the fingertip to move, rank every record by
how much of its tip motion points that way, asked per part of its input
range. The turn check in turn.py reads one number per record (the
small-signal motion direction); this reads the whole free tip curve instead.

For a record with a free tip curve (free.u, free.tip_displacement_m, at
least MIN_LEVELS levels), take the step between levels, dy_k and du_k. For a
window of steps form

    W = sum over the window of dy_k dy_k^T / |du_k|

(the tip Gramian of that stretch of the sweep, in the record's own frame).
The share along a unit direction e is e^T W e / trace(W).

The needed direction is the azimuth at contact 0, e_phi, rotated into the
finger's frame by the mount rotation: R[0]^T e_phi. The contact normal is
taken the same way, and the other tangent is the unit vector at right angles
to both, so the three shares add to one.

Reported per record: the share over the first third of the steps, the first
half, the whole sweep, and the best contiguous window at least a third long
(with its input range); and, over the whole sweep, the shares along the
contact normal and the other tangent. Records with no usable free curve are
left out with the reason.

A record with more than one input (free.u holds several values per level)
has no single sweep. It is split into its measured lines: one input swept
with every other input held at one value, kept if it has at least
MIN_LEVELS levels and no repeated level. Each line gets its own row, with
the swept input and the held inputs named. A line whose held inputs are not
all zero starts away from rest; the share is built from the steps between
levels, so it is the same calculation wherever the line starts. Only lines
logged in the record are used: nothing is worked out between them here.

The caller may hand in further lines for a record (`more_lines`), for
example lines worked out from a two-input record's single-input lines by a
declared rule. They are ranked by the same calculation. This module does not
make them or judge them: whatever the caller puts under "worked_out" in a
line's `about` is carried onto its row unchanged, and a row without that key
is a logged line.

Records are read, never edited. Simulated magnitudes are comparative, not
predictive; a share is a ratio and carries no magnitude.
"""
import math

import numpy as np

from .arrange import Arrangement
from .finger import Finger, all_fingers, load_record

MIN_LEVELS = 3                  # a free curve needs this many levels for two steps
WINDOWS = ("share_first_third", "share_first_half", "share_whole",
           "best_window_share")


def _held(rec):
    held = [b for b in ("free", "blocked", "stiffness", "shape") if rec.get(b)]
    return ", ".join(f"{b} curve" for b in held) if held else "headline numbers only"


def free_curve(rec):
    """(u (L,), y (L, 3), None) for a usable free tip curve, else
    (None, None, reason)."""
    free = rec.get("free") or {}
    u = np.asarray(free.get("u", []), float).ravel()
    y = np.asarray(free.get("tip_displacement_m", []), float)
    if len(u) == 0 and y.size == 0:
        return None, None, f"no free tip curve (holds: {_held(rec)})"
    if y.ndim != 2 or y.shape[1] != 3:
        return None, None, f"free tip curve is not one 3D tip position per level (shape {list(y.shape)})"
    if len(u) != len(y):
        return None, None, (f"free tip curve has {len(u)} input levels but "
                            f"{len(y)} tip positions")
    if len(y) < MIN_LEVELS:
        return None, None, (f"free tip curve has {len(y)} levels "
                            f"(needs at least {MIN_LEVELS})")
    if np.any(np.diff(u) == 0):
        return None, None, "free tip curve repeats an input level"
    return u, y, None


def free_lines(rec):
    """The measured lines of a record's free tip curve, as
    ([(about, u (L,), y (L, 3))], None), or (None, reason). A single-input
    record gives one line (free_curve's). A record with several inputs
    gives one line per input and per setting of all the other inputs, the
    levels ordered by the size of the swept input, kept if it has at least
    MIN_LEVELS levels and no two of the same size. `about` holds
    "swept_input", "swept_unit" and "held" ([{"input", "at", "unit"}], empty
    for a single-input record)."""
    chans = (rec.get("input") or {}).get("channels") or []
    free = rec.get("free") or {}
    try:
        U = np.asarray(free.get("u", []), float)
    except ValueError:
        U = np.zeros(0)
    if U.ndim != 2 or U.shape[1] < 2:
        u, y, reason = free_curve(rec)
        if reason:
            return None, reason
        ch0 = chans[0] if chans else {}
        return [({"swept_input": ch0.get("name"), "swept_unit": ch0.get("unit"),
                  "held": []}, u, y)], None
    L, C = U.shape
    y = np.asarray(free.get("tip_displacement_m", []), float)
    if y.ndim != 2 or y.shape != (L, 3):
        return None, (f"free tip curve has {L} input levels but tip positions "
                      f"of shape {list(y.shape)}")

    def ch(j, key):
        return chans[j].get(key) if j < len(chans) else None

    out = []
    for c in range(C):
        others = [j for j in range(C) if j != c]
        groups = {}
        for i in range(L):
            groups.setdefault(tuple(float(U[i, j]) for j in others), []).append(i)
        for setting in sorted(groups):
            rows = sorted(groups[setting], key=lambda i: abs(float(U[i, c])))
            sizes = [abs(float(U[i, c])) for i in rows]
            if len(rows) < MIN_LEVELS or len(set(sizes)) != len(sizes):
                continue
            out.append(({"swept_input": ch(c, "name"), "swept_unit": ch(c, "unit"),
                         "held": [{"input": ch(j, "name"), "at": v,
                                   "unit": ch(j, "unit")}
                                  for j, v in zip(others, setting)]},
                        U[rows, c], y[rows]))
    if not out:
        return None, (f"free tip curve has {C} inputs but no line of at least "
                      f"{MIN_LEVELS} levels with one input swept and the "
                      "others held")
    return out, None


def window_share(dy, du, a, b, e):
    """Share along e of W over steps a to b-1 (nan if the tip does not move)."""
    W = np.einsum("k,ki,kj->ij", 1.0 / np.abs(du[a:b]), dy[a:b], dy[a:b])
    tr = float(np.trace(W))
    return float(e @ W @ e) / tr if tr > 0 else float("nan")


def directions(finger, task, n, style, yaw_deg):
    """The needed direction (azimuth), the contact normal and the other
    tangent at contact 0, in the finger's own frame."""
    arr = Arrangement(finger, task, n, style, yaw_deg)
    R, nrm, ephi = arr.R[0], arr.cup.normals[0], arr.cup.e_phi[0]
    e, nn = R.T @ ephi, R.T @ nrm
    t = np.cross(nn, e)
    return e, nn, t / np.linalg.norm(t)


def lookup(finger, rec, task, n=4, style="aimed", yaw_deg=90.0, more=None):
    """One finger against one task. Returns (rows, None), one row per
    measured line of the record's free tip curve (one for a single-input
    record), or (None, reason) when the record has no usable line. `more`
    is a list of further lines (about, u, y) for this record, ranked after
    its measured ones; see the module docstring."""
    found, reason = free_lines(rec)
    if reason:
        return None, reason
    e, nn, t = directions(finger, task, n, style, yaw_deg)
    return [_row(finger, about, np.asarray(u, float), np.asarray(y, float),
                 e, nn, t)
            for about, u, y in list(found) + list(more or [])], None


def _row(finger, about, u, y, e, nn, t):
    dy, du = np.diff(y, axis=0), np.diff(u)
    K = len(du)
    third = max(K // 3, 1)
    half = max(K // 2, 1)
    best = max(((window_share(dy, du, a, b, e), a, b)
                for a in range(K) for b in range(a + third, K + 1)),
               key=lambda s: -math.inf if math.isnan(s[0]) else s[0])
    W = np.einsum("k,ki,kj->ij", 1.0 / np.abs(du), dy, dy)
    tr = float(np.trace(W))

    def whole(v):
        return float(v @ W @ v) / tr if tr > 0 else float("nan")

    row = {
        "id": finger.id,
        "name": finger.name,
        "flags": finger.flags(),
        "levels": int(len(u)),
        "input_unit": about["swept_unit"],
        "swept_input": about["swept_input"],
        "held": about["held"],
        "share_first_third": window_share(dy, du, 0, third, e),
        "first_third_u": [float(u[0]), float(u[third])],
        "share_first_half": window_share(dy, du, 0, half, e),
        "first_half_u": [float(u[0]), float(u[half])],
        "share_whole": whole(e),
        "whole_u": [float(u[0]), float(u[-1])],
        "best_window_share": best[0],
        "best_window_u": [float(u[best[1]]), float(u[best[2]])],
        "normal_whole": whole(nn),
        "other_tangent_whole": whole(t),
        "needed_direction_in_finger_frame": e.tolist(),
    }
    if "worked_out" in about:
        row["worked_out"] = about["worked_out"]
    return row


def _sort_key(row):
    s = row["share_first_third"]
    return (math.isnan(s), -s if not math.isnan(s) else 0.0)


def rank(task, n=4, style="aimed", yaw_deg=90.0, extra=None, more_lines=None):
    """Every manifest record looked up against the task, then any extra
    record files. `extra` maps a record file path to the flags to carry
    for it (e.g. {"source": "draft ..."}); those records are read where
    they lie and are not added to the manifest. Returns (rows sorted by
    first-third share, highest first, with the record's flags carried
    through; left_out as [{"id", "reason"}]). A record with several inputs
    gives one row per measured line, all with the record's id; "held" tells
    them apart. `more_lines` is called with each record and returns further
    lines (about, u, y) to rank for it, or nothing."""
    rows, left_out = [], []
    pairs = [(f, load_record(f.id)) for f in all_fingers()]
    for path, quality in (extra or {}).items():
        rec = load_record(path)
        pairs.append((Finger(rec, quality), rec))
    for f, rec in pairs:
        found, reason = lookup(f, rec, task, n, style, yaw_deg,
                               more_lines(rec) if more_lines else None)
        if found is None:
            left_out.append({"id": f.id, "flags": f.flags(), "reason": reason})
        else:
            rows.extend(found)
    rows.sort(key=_sort_key)
    return rows, left_out
