"""Build a MuJoCo hand from actuator records (docs/RECORD_FINGER_DESIGN.md).

    python tools/build_record_hand.py data/hands/<name>/hand.json

Reads the hand file, fits a stand-in finger to each record with
gramian_workshop.record_fit, and writes beside the hand file:

  hand.xml         the MuJoCo model (register it in gramian_workshop/hands.py)
  fit_report.json  each finger's fit and fit check, and the round-trip check
                   below

The hand file (JSON):

  name, palm {size_m: [x, y, z] half-sizes, pos_m}, fingers: [
    {label, record, hinges (default 3), slide (null: the fitter chooses),
     contact_radius_m (null: radius of a cylinder holding the record's
       material volume over its length),
     input_range (null: 0 to the record's last free level),
     mount {pos_m, length_axis, bend_axis}}]

length_axis is where the record's canonical X (base -> tip) points on the
palm, bend_axis where its +Y (the free bending side) points; they must be
at right angles.

How the fit is written into the model, so that the free equilibrium is the
fitted one, q = r a(u):
  joints   optional slide along the finger's X, then one hinge about Z at the
           start of each of N equal links; stiffness K_j about 0
  drive    one fixed tendon per finger with coef_j = K_j r_j and one motor on
           it. The motor's force is the drive coordinate a(u), not newtons:
           joint torque is K_j r_j a. Its range is the span of a(u) over the
           input range, with 0.
  ranges   each joint's span of r_j a(u) over the input range, with 0 (the
           free motion; how far contact may push a joint beyond it is left
           to the spring term in follow.py)
  mass     the records give none: MuJoCo's default density applies.

Round-trip check (needs mujoco): the written model is loaded, set to
q = r a(u) at every free level, and its tip is compared with the record's
free path, in the bending plane (should equal the fit's own error) and with
the record's out-of-plane drift added; the drive and spring forces must
balance there.

follow.py reads each joint's stiffness and springref from the model and
adds the spring torque at each solved pose to the joint balance (issue 21;
follow.py's docstring, "Joint springs"). Strength verdicts stay provisional
while the drive is in the record's units (hands.py gives the hand a caveat,
which every result carries).
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop.finger import load_record           # noqa: E402
from gramian_workshop.record_fit import (Chain, a_of_u,   # noqa: E402
                                         fit_record)


def _f(x):
    return f"{float(x):.9g}"


def _vec(v):
    return " ".join(_f(x) for x in v)


def contact_radius(rec):
    md = rec.get("metadata") or {}
    V, L = md.get("material_volume_m3"), md.get("length_m")
    if not V or not L:
        return None, "record gives no material volume and length"
    return float(np.sqrt(V / (np.pi * L))), \
        "cylinder of the record's material volume over its length"


def fit_finger(spec):
    rec = load_record(spec["record"])
    fit = fit_record(spec["record"], spec.get("hinges", 3), spec.get("slide"))
    if fit["status"] != "fitted":
        return None, fit
    f = fit["fit"]
    u_lo, u_hi = spec.get("input_range") or (0.0, float(max(f["u"])))
    knots = [u for u in f["u"] if u_lo < u < u_hi]
    a = np.array([a_of_u(f, u) for u in [u_lo, u_hi] + knots] + [0.0])
    r = np.asarray(f["ratios"], float)
    K = np.asarray(fit["springs"]["k"], float)
    q = np.outer(a, r)
    rad, rad_src = (spec["contact_radius_m"], "hand file") \
        if spec.get("contact_radius_m") else contact_radius(rec)
    if rad is None:
        return None, dict(fit, status="refused",
                          reason="no contact radius: " + rad_src)
    return {"rec": rec, "fit": fit, "r": r, "K": K, "u_range": [u_lo, u_hi],
            "a_range": [float(a.min()), float(a.max())],
            "q_lo": q.min(0), "q_hi": q.max(0), "radius": float(rad),
            "radius_source": rad_src}, fit


def finger_xml(spec, ff):
    lab, f = spec["label"], ff["fit"]["fit"]
    ch = Chain(f["n_hinges"], f["slide"], f["length_m"])
    X = np.asarray(spec["mount"]["length_axis"], float)
    Y = np.asarray(spec["mount"]["bend_axis"], float)
    X, Y = X / np.linalg.norm(X), Y / np.linalg.norm(Y)
    if abs(X @ Y) > 1e-9:
        raise ValueError(f"{lab}: length_axis and bend_axis are not at right angles")
    names, lines, depth = [], [], 0
    lines.append(f'  <body name="{lab}_mount" pos="{_vec(spec["mount"]["pos_m"])}" '
                 f'xyaxes="{_vec(X)} {_vec(Y)}">')
    j = 0
    for i in range(ch.n_h):
        body = f"{lab}_distal" if i == ch.n_h - 1 else f"{lab}_link{i}"
        pos = "0 0 0" if i == 0 else f"{_f(ch.l)} 0 0"
        lines.append("  " * (depth + 2) + f'<body name="{body}" pos="{pos}">')
        if i == 0 and ch.slide:
            names.append(f"{lab}_slide")
            lines.append("  " * (depth + 3) + _joint(names[-1], "slide", ff, j))
            j += 1
        names.append(f"{lab}_hinge{i}")
        lines.append("  " * (depth + 3) + _joint(names[-1], "hinge", ff, j))
        j += 1
        lines.append("  " * (depth + 3) + f'<geom type="capsule" fromto="0 0 0 {_f(ch.l)} 0 0" '
                     f'size="{_f(ff["radius"])}"/>')
        depth += 1
    for d in range(depth, 0, -1):
        lines.append("  " * (d + 1) + "</body>")
    lines.append("  </body>")
    coef = ff["K"] * ff["r"]
    tendon = (f'<fixed name="{lab}_drive">' + "".join(
        f'<joint joint="{n}" coef="{_f(c)}"/>' for n, c in zip(names, coef))
        + "</fixed>")
    lo, hi = ff["a_range"]
    motor = (f'<motor name="{lab}_drive" tendon="{lab}_drive" gear="1" '
             f'ctrlrange="{_f(lo)} {_f(hi)}" ctrllimited="true" '
             f'forcerange="{_f(lo)} {_f(hi)}" forcelimited="true"/>')
    return "\n".join(lines), tendon, motor, names


def _joint(name, kind, ff, j):
    axis = "1 0 0" if kind == "slide" else "0 0 1"
    return (f'<joint name="{name}" type="{kind}" axis="{axis}" '
            f'range="{_f(ff["q_lo"][j])} {_f(ff["q_hi"][j])}" limited="true" '
            f'stiffness="{_f(ff["K"][j])}" springref="0"/>')


def build(hand_file):
    with open(hand_file, encoding="utf-8") as fh:
        hf = json.load(fh)
    out_dir = os.path.dirname(os.path.abspath(hand_file))
    report = {"hand": hf["name"], "hand_file": os.path.relpath(hand_file, ROOT),
              "fingers": []}
    bodies, tendons, motors, fitted = [], [], [], []
    for spec in hf["fingers"]:
        ff, fit = fit_finger(spec)
        if ff is None:
            report["status"] = "refused"
            report["reason"] = f"{spec['label']} ({spec['record']}): {fit.get('reason')}"
            _write(out_dir, "fit_report.json", report)
            raise SystemExit("cannot build: " + report["reason"])
        b, t, m, names = finger_xml(spec, ff)
        bodies.append(b)
        tendons.append(t)
        motors.append(m)
        fitted.append((spec, ff, names))
        report["fingers"].append({
            "label": spec["label"], "record": spec["record"],
            "fit_check": "PASS" if fit["pass"] else "FLAGGED",
            "checks": fit["checks"], "input_range": ff["u_range"],
            "drive_range": ff["a_range"], "contact_radius_m": ff["radius"],
            "contact_radius_source": ff["radius_source"],
            "length_m": fit["fit"]["length_m"],
            "length_source": fit["length_source"],
            "ratios": ff["r"].tolist(), "springs": fit["springs"],
            "joints": names})
    p = hf["palm"]
    xml = "\n".join([
        f'<mujoco model="{hf["name"]}">',
        f"<!-- generated by tools/build_record_hand.py from {report['hand_file']}."
        " Each finger is a stand-in fitted to its record. A motor's force is the"
        " drive coordinate a(u), not newtons; joint torque is coef x a. No"
        " masses in the records: MuJoCo's default density applies. -->",
        '<compiler angle="radian"/>',
        '<option gravity="0 0 0"/>',
        '<default><joint damping="0.02" armature="1e-5"/></default>',
        "<worldbody>",
        ' <body name="palm">',
        f'  <geom type="box" pos="{_vec(p["pos_m"])}" size="{_vec(p["size_m"])}"/>',
        *bodies,
        " </body>",
        "</worldbody>",
        "<tendon>" + "".join(tendons) + "</tendon>",
        "<actuator>" + "".join(motors) + "</actuator>",
        "</mujoco>", ""])
    xml_path = os.path.join(out_dir, "hand.xml")
    with open(xml_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(xml)
    report["model"] = os.path.relpath(xml_path, ROOT)
    report["round_trip"] = round_trip(xml_path, fitted)
    report["status"] = "built"
    _write(out_dir, "fit_report.json", report)
    return report


def round_trip(xml_path, fitted):
    """The written model against the fit and the record's free path."""
    try:
        import mujoco as mj
    except ImportError:
        return {"skipped": "mujoco not installed"}
    m = mj.MjModel.from_xml_path(xml_path)
    d = mj.MjData(m)
    out = {}
    for spec, ff, names in fitted:
        lab, f = spec["label"], ff["fit"]["fit"]
        ch = Chain(f["n_hinges"], f["slide"], f["length_m"])
        dof = [m.jnt_dofadr[m.joint(n).id] for n in names]
        qadr = [m.jnt_qposadr[m.joint(n).id] for n in names]
        act = m.actuator(f"{lab}_drive").id
        mount = m.body(f"{lab}_mount").id
        distal = m.body(f"{lab}_distal").id

        def tip_local():
            p = d.xpos[distal] + d.xmat[distal].reshape(3, 3) @ [ch.l, 0, 0]
            Rm = d.xmat[mount].reshape(3, 3)
            return Rm.T @ (p - d.xpos[mount])

        d.qpos[:] = m.qpos0
        d.ctrl[:] = 0
        mj.mj_forward(m, d)
        rest = tip_local()
        free = ff["rec"]["free"]
        u = np.asarray(free["u"], float).ravel()
        rec_d = np.asarray(free["tip_displacement_m"], float)
        tip_err, tip_err3, bal_err, in_range = [], [], [], []
        for k, uk in enumerate(u):
            if not ff["u_range"][0] <= uk <= ff["u_range"][1]:
                continue
            a = a_of_u(f, uk)
            d.qpos[:] = m.qpos0
            d.qpos[qadr] = ff["r"] * a
            d.ctrl[:] = 0
            d.ctrl[act] = a
            mj.mj_forward(m, d)
            e = tip_local() - rest - rec_d[k]
            tip_err.append(float(np.linalg.norm(e[:2])))
            tip_err3.append(float(np.linalg.norm(e)))
            bal_err.append(float(np.abs(d.qfrc_actuator[dof]
                                        + d.qfrc_passive[dof]).max()))
            in_range.append(k)
        stroke = float(ff["rec"]["scalars"]["free_stroke_m"])
        # the loader's fingertip rule (hands.Finger) against the chain's tip
        tip_rule = _tip_rule_point(m, distal)
        out[lab] = {
            "free_levels_checked": len(in_range),
            "tip_err_max_m": max(tip_err),
            "fit_free_err_max_m": f["free_err_max_m"],
            "tip_err_with_out_of_plane_max_m": max(tip_err3),
            "tip_err_with_out_of_plane_share_of_stroke": max(tip_err3) / stroke,
            "tip_err_note": "tip_err is in the bending plane, as the fit "
                            "check measures it, so it should equal the fit's "
                            "error; the second figure adds the record's "
                            "out-of-plane drift, which a planar chain cannot "
                            "follow",
            "drive_spring_balance_max": max(bal_err),
            "loader_tip_offset_m": float(np.linalg.norm(tip_rule - [ch.l, 0, 0])),
            "loader_tip_note": "hands.Finger takes the farthest bounding-box "
                               "corner of the distal capsule, so its fingertip "
                               "sits about a capsule radius (and a corner) past "
                               "the chain's tip"}
    return out


def _tip_rule_point(m, body):
    """hands.Finger._tip_point's rule for capsule geoms, in the body frame."""
    import mujoco as mj
    best, bestd = None, -1.0
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != body:
            continue
        Rm = np.zeros(9)
        mj.mju_quat2Mat(Rm, m.geom_quat[g])
        s = m.geom_size[g]
        ext = np.array([s[0], s[0], s[1] + s[0]])
        corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1)
                            for z in (-1, 1)]) * ext
        pts = corners @ Rm.reshape(3, 3).T + m.geom_pos[g]
        dist = np.linalg.norm(pts, axis=1)
        i = int(np.argmax(dist))
        if dist[i] > bestd:
            best, bestd = pts[i], float(dist[i])
    return best


def _write(out_dir, name, obj):
    with open(os.path.join(out_dir, name), "w", encoding="utf-8", newline="\n") as fh:
        json.dump(obj, fh, indent=1, default=float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("hand_file")
    rep = build(ap.parse_args().hand_file)
    for fg in rep["fingers"]:
        print(f"{fg['label']}: {fg['record']}, fit check {fg['fit_check']}, "
              f"radius {fg['contact_radius_m'] * 1e3:.2f} mm "
              f"({fg['contact_radius_source']})")
    for lab, rt in rep["round_trip"].items():
        print(f"{lab} round trip: {json.dumps(rt)}" if isinstance(rt, dict)
              else f"round trip: {rt}")
    print("wrote", rep["model"])


if __name__ == "__main__":
    main()
