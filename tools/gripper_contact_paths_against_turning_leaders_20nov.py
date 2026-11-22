"""How close do the hanging gripper's contact paths come to a banked turning record?

Reads, and changes nothing in:
  - leader files (data/leaders/*.json): frame by frame, the object's pose and,
    per contact station, whether it touches and where
  - the gripper's contact logs written by
    tools/pneunet_gripper_turn_ball_mujoco_20nov.py (contacts_*.csv)

The same measures are taken on both sides, all as angles or fractions so the
ball's size and the speed do not matter:

  stroke     one station (or finger) from touch-down to lift-off. Strokes cut
             by the start or end of the record are dropped.
  per stroke the object's turn about its turning axis from touch-down to
             lift-off (net) and the largest turn reached on the way (peak);
             how far round the axis the contact point moved in the world, and
             how far round it moved on the object (zero for a contact that
             sticks and is carried round with the object); the contact's
             angle above (+) or below (-) the object's equator, taken about
             the turning axis.
  schedule   how many contacts touch at each moment; how many others still
             touch when one lifts off; the object's turn over the whole
             record; the turn while nothing touches.

The turning axis is the direction of the summed frame-to-frame rotation of
the object (leaders), or the ball's own vertical axis (gripper; the log gives
twist about the vertical, and the contact point in the ball's own frame).
Sliding and rolling of the contact over the object cannot be told apart from
contact points alone, so "moved on the object" is both together.
Forces are not compared: the gripper log holds the normal force only.

SIMULATION ONLY -- comparative, not predictive.

Run from the repository root:
  python tools/gripper_contact_paths_against_turning_leaders_20nov.py
"""
import argparse
import csv
import json
import os

import numpy as np

OUT = "outputs/gripper_against_turning_leaders_20nov"
GRIP = "outputs/pneunet_gripper_turn_ball_20nov"
LEADERS = [
    "data/leaders/w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4__contactfix.json",
    "data/leaders/w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n5__contactfix.json",
    "data/leaders/tennis_new_cup_take3_derived__nframesfix.json",
]
GRIPPER = [
    GRIP + "/contacts_wall_0.0020m_tips_level_turned_+60deg.csv",
    GRIP + "/contacts_wall_0.0020m_tips_level_turned_+60deg_ramp_40s.csv",
    GRIP + "/contacts_wall_0.0020m_tips_level_turned_+45deg.csv",
    GRIP + "/contacts_wall_0.0020m_tips_low_turned_+60deg.csv",
    GRIP + "/contacts_wall_0.0015m_tips_low_turned_+60deg.csv",
    GRIP + "/contacts_wall_0.0015m_tips_low_turned_+60deg_ramp_40s.csv",
]


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qrot(q, v):
    return qmul(qmul(q, np.r_[0.0, v]), qconj(q))[1:]


def rotvec(q):
    """Rotation vector (radians) of a unit quaternion, shortest way round."""
    q = q / np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    s = np.linalg.norm(q[1:])
    if s < 1e-12:
        return np.zeros(3)
    return q[1:] / s * 2.0 * np.arctan2(s, q[0])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def load_leader(path):
    """-> t, turn (rad, about the turning axis), touch[n,k], az_world, az_ball, lat (rad)."""
    d = json.load(open(path))
    fr = d["frames"]
    n, k = len(fr), d["n_stations"]
    t = np.array([f["t"] for f in fr])
    q = np.array([f["obj_quat"] for f in fr], float)  # w, x, y, z (MuJoCo order)
    steps = np.array([rotvec(qmul(q[i + 1], qconj(q[i]))) for i in range(n - 1)])
    total = steps.sum(0)
    axis = total / np.linalg.norm(total)
    turn = np.r_[0.0, np.cumsum(steps @ axis)]
    off_axis = np.abs(steps - np.outer(steps @ axis, axis)).sum() / max(np.abs(steps @ axis).sum(), 1e-12)
    # two directions square to the axis, fixed in the world
    e1 = np.cross(axis, [1.0, 0.0, 0.0])
    if np.linalg.norm(e1) < 0.3:
        e1 = np.cross(axis, [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(axis, e1)
    touch = np.zeros((n, k), bool)
    az_w = np.full((n, k), np.nan)
    az_b = np.full((n, k), np.nan)
    lat = np.full((n, k), np.nan)
    for i, f in enumerate(fr):
        c = np.array(f["obj_pos"], float)
        for j in range(k):
            cp = f["contact_point"][j]
            if not f["contact"][j] or cp is None:
                continue
            r = np.array(cp, float) - c
            if not np.all(np.isfinite(r)):
                continue
            touch[i, j] = True
            az_w[i, j] = np.arctan2(r @ e2, r @ e1)
            lat[i, j] = np.arcsin(np.clip(r @ axis / np.linalg.norm(r), -1, 1))
            az_b[i, j] = az_w[i, j] - turn[i]  # the same point seen from the turning object
    info = {"file": path, "kind": d.get("kind"), "object_radius_m": d["object"]["radius_m"],
            "object_mass_kg": d["object"]["mass_kg"], "contacts": k,
            "turning_axis_world": axis.tolist(),
            "rotation_off_the_turning_axis_fraction": float(off_axis)}
    return info, t, turn, touch, az_w, az_b, lat


def load_gripper(path):
    rows = list(csv.reader(open(path)))
    h = rows[0]
    a = np.array([[float(x) for x in r] for r in rows[1:]])
    t = a[:, h.index("time_s")]
    turn = np.radians(a[:, h.index("ball_twist_deg")])
    pa = a[:, h.index("input_Pa")]
    xs = [i for i, name in enumerate(h) if name.endswith("x_on_ball_m")]
    k = len(xs)
    n = len(t)
    touch = np.zeros((n, k), bool)
    az_w = np.full((n, k), np.nan)
    az_b = np.full((n, k), np.nan)
    lat = np.full((n, k), np.nan)
    for j, i0 in enumerate(xs):
        p = a[:, i0:i0 + 3]
        ok = np.all(np.isfinite(p), 1)
        touch[:, j] = ok
        az_b[ok, j] = np.arctan2(p[ok, 1], p[ok, 0])
        lat[ok, j] = np.arcsin(np.clip(p[ok, 2] / np.linalg.norm(p[ok], axis=1), -1, 1))
        az_w[ok, j] = az_b[ok, j] + turn[ok]
    info = {"file": path, "kind": "hanging three-finger gripper, MuJoCo", "contacts": k,
            "input_peak_Pa": float(pa.max())}
    return info, t, turn, touch, az_w, az_b, lat, pa


def swept(az):
    """Net and peak change of an angle along a run, unwrapped."""
    u = np.unwrap(az)
    return u[-1] - u[0], np.abs(u - u[0]).max()


def measure(t, turn, touch, az_w, az_b, lat):
    n, k = touch.shape
    strokes = []
    for j in range(k):
        i = 0
        while i < n:
            if not touch[i, j]:
                i += 1
                continue
            s = i
            while i < n and touch[i, j]:
                i += 1
            e = i - 1
            if s == 0 or e == n - 1:
                continue  # cut by the record's ends
            if e - s < 2:
                continue  # one or two frames: a graze, not a stroke
            dt = turn[s:e + 1] - turn[s]
            wn, wp = swept(az_w[s:e + 1, j])
            bn, bp = swept(az_b[s:e + 1, j])
            others = int(touch[min(e + 1, n - 1)].sum() - touch[min(e + 1, n - 1), j])
            strokes.append({
                "contact": j, "t_start": float(t[s]), "t_end": float(t[e]),
                "turn_net_deg": float(np.degrees(dt[-1])),
                "turn_peak_deg": float(np.degrees(dt[np.abs(dt).argmax()])),
                "contact_round_axis_in_world_peak_deg": float(np.degrees(wp)),
                "contact_round_axis_on_object_peak_deg": float(np.degrees(bp)),
                "contact_round_axis_on_object_net_deg": float(np.degrees(bn)),
                "latitude_mean_deg": float(np.degrees(np.nanmean(lat[s:e + 1, j]))),
                "latitude_min_deg": float(np.degrees(np.nanmin(lat[s:e + 1, j]))),
                "latitude_max_deg": float(np.degrees(np.nanmax(lat[s:e + 1, j]))),
                "others_touching_at_lift_off": others,
            })
    cnt = touch.sum(1)
    d = np.diff(turn)
    none = cnt[:-1] == 0
    out = {
        "record_length_s": float(t[-1] - t[0]),
        "turn_over_record_deg": float(np.degrees(turn[-1] - turn[0])),
        "largest_turn_from_start_deg": float(np.degrees(turn[np.abs(turn - turn[0]).argmax()] - turn[0])),
        "touching_at_once_min": int(cnt.min()), "touching_at_once_mean": float(cnt.mean()),
        "touching_at_once_max": int(cnt.max()),
        "share_of_time_nothing_touches": float((cnt == 0).mean()),
        "turn_while_nothing_touches_deg": float(np.degrees(d[none].sum())),
        "strokes": strokes, "n_strokes": len(strokes),
    }
    if strokes:
        def med(key, absolute=False):
            v = np.array([s[key] for s in strokes])
            return float(np.median(np.abs(v) if absolute else v))
        out["median_stroke"] = {
            "turn_net_deg": med("turn_net_deg"), "turn_peak_deg": med("turn_peak_deg"),
            "contact_round_axis_in_world_peak_deg": med("contact_round_axis_in_world_peak_deg"),
            "contact_round_axis_on_object_peak_deg": med("contact_round_axis_on_object_peak_deg"),
            "latitude_mean_deg": med("latitude_mean_deg"),
            "others_touching_at_lift_off_min": int(min(s["others_touching_at_lift_off"] for s in strokes)),
        }
        # of the object's surface motion past a contact, the share the contact did not follow
        pk = np.array([abs(s["turn_peak_deg"]) for s in strokes])
        ob = np.array([s["contact_round_axis_on_object_peak_deg"] for s in strokes])
        big = pk > 1.0
        out["contact_moved_on_object_per_degree_turned_median"] = (
            float(np.median(ob[big] / pk[big])) if big.any() else None)
    return out


def line(label, m):
    s = m.get("median_stroke")
    a = "%-58s %2d  %5.1f %5.1f" % (label[:58], m["touching_at_once_min"], m["touching_at_once_mean"],
                                     100 * m["share_of_time_nothing_touches"])
    a += "  %8.1f %8.1f" % (m["turn_over_record_deg"], m["turn_while_nothing_touches_deg"])
    if s:
        r = m["contact_moved_on_object_per_degree_turned_median"]
        a += "  %3d %7.1f %7.1f %7.1f %7.1f %7.1f %3d  %s" % (
            m["n_strokes"], s["turn_net_deg"], s["turn_peak_deg"],
            s["contact_round_axis_in_world_peak_deg"], s["contact_round_axis_on_object_peak_deg"],
            s["latitude_mean_deg"], s["others_touching_at_lift_off_min"],
            "  n/a" if r is None else "%5.2f" % r)
    else:
        a += "    0  (no whole stroke)"
    return a


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--leaders", nargs="*", default=LEADERS)
    ap.add_argument("--gripper", nargs="*", default=GRIPPER)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    res = {"leaders": {}, "gripper": {}}
    txt = ["Gripper contact paths against banked turning records",
           "SIMULATION ONLY -- comparative, not predictive.",
           "",
           "Columns: fewest contacts touching at once | mean touching | % of time nothing touches |",
           "  object's turn over the record (deg) | of that, turn while nothing touches (deg) |",
           "  whole strokes | then medians over strokes: turn touch-down to lift-off (deg) |",
           "  largest turn within the stroke (deg) | contact moved round the axis in the world (deg) |",
           "  contact moved round the axis on the object (deg) | contact latitude about the turning axis (deg; + is above the equator for the authored waves and the gripper, whose axis points up; sign not fixed for the human record) |",
           "  fewest other contacts still touching at a lift-off |",
           "  contact's movement on the object per degree the object turned (0 = carried with it, 1 = object turns past it)",
           ""]
    txt.append("Banked turning records")
    for p in a.leaders:
        info, t, turn, touch, aw, ab, lat = load_leader(p)
        m = measure(t, turn, touch, aw, ab, lat)
        m.update(info)
        res["leaders"][os.path.basename(p)] = m
        txt.append(line(os.path.basename(p).replace(".json", ""), m))
        txt.append("    %s; object radius %.1f mm; %d stations; record %.1f s; rotation off the turning axis %.1f%% of the turn"
                   % (info["kind"], 1e3 * info["object_radius_m"], info["contacts"], m["record_length_s"],
                      100 * info["rotation_off_the_turning_axis_fraction"]))
    txt.append("")
    txt.append("Hanging three-finger gripper (two pressure cycles each)")
    for p in a.gripper:
        info, t, turn, touch, aw, ab, lat, pa = load_gripper(p)
        m = measure(t, turn, touch, aw, ab, lat)
        m.update(info)
        # turn at each return of the input to zero after a peak
        up = pa > 0.5 * pa.max()
        ends = [i for i in range(1, len(pa)) if pa[i] == 0 and pa[i - 1] > 0]
        m["turn_at_each_release_deg"] = [float(np.degrees(turn[i])) for i in ends]
        m["turn_at_end_deg"] = float(np.degrees(turn[-1]))
        res["gripper"][os.path.basename(p)] = m
        txt.append(line(os.path.basename(p).replace("contacts_", "").replace(".csv", ""), m))
        txt.append("    turn when the input is back at zero, each cycle (deg): %s; at the end of the log: %.1f; largest turn from the start: %.1f"
                   % (", ".join("%.1f" % v for v in m["turn_at_each_release_deg"]), m["turn_at_end_deg"],
                      m["largest_turn_from_start_deg"]))
    txt.append("")
    open(OUT + "/gripper_against_turning_leaders.txt", "w", encoding="utf-8", newline="\n").write("\n".join(txt) + "\n")
    json.dump(res, open(OUT + "/gripper_against_turning_leaders.json", "w"), indent=1)
    print("\n".join(txt))
    print("wrote", OUT + "/gripper_against_turning_leaders.txt")
    print("wrote", OUT + "/gripper_against_turning_leaders.json")


if __name__ == "__main__":
    main()
