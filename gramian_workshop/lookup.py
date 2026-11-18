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


def lookup(finger, rec, task, n=4, style="aimed", yaw_deg=90.0):
    """One finger against one task. Returns (row, None), or (None, reason)
    when the record has no usable free tip curve."""
    u, y, reason = free_curve(rec)
    if reason:
        return None, reason
    e, nn, t = directions(finger, task, n, style, yaw_deg)
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

    ch = ((rec.get("input") or {}).get("channels") or [{}])[0]
    return {
        "id": finger.id,
        "name": finger.name,
        "flags": finger.flags(),
        "levels": int(len(u)),
        "input_unit": ch.get("unit"),
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
    }, None


def _sort_key(row):
    s = row["share_first_third"]
    return (math.isnan(s), -s if not math.isnan(s) else 0.0)


def rank(task, n=4, style="aimed", yaw_deg=90.0, extra=None):
    """Every manifest record looked up against the task, then any extra
    record files. `extra` maps a record file path to the flags to carry
    for it (e.g. {"source": "draft ..."}); those records are read where
    they lie and are not added to the manifest. Returns (rows sorted by
    first-third share, highest first, with the record's flags carried
    through; left_out as [{"id", "reason"}])."""
    rows, left_out = [], []
    pairs = [(f, load_record(f.id)) for f in all_fingers()]
    for path, quality in (extra or {}).items():
        rec = load_record(path)
        pairs.append((Finger(rec, quality), rec))
    for f, rec in pairs:
        row, reason = lookup(f, rec, task, n, style, yaw_deg)
        if row is None:
            left_out.append({"id": f.id, "flags": f.flags(), "reason": reason})
        else:
            rows.append(row)
    rows.sort(key=_sort_key)
    return rows, left_out
