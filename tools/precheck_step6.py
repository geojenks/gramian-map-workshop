"""W20 step 6 pre-check: records and task geometry only, nothing solved.

    python tools/precheck_step6.py [--out outputs/precheck_step6]

Registration section 7 (pre-check). For every record: which rule gives its
push direction b, b and the motion direction d in the canonical frame, the
comparative force F and stroke S, and its trust flags. For the cup ring: the
angle between each mount's world push direction and the contact normal,
against the friction cone's half-angle atan(mu); and, for the turn, how
much of the finger's tangent-plane motion lies along the azimuth. These are
single dot products on the record vectors; no feasibility test runs here.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gramian_workshop.finger import all_fingers, mount_rotation  # noqa: E402
from gramian_workshop.task import Cup, load_task  # noqa: E402


def ang(a, b):
    c = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def tangent_split(dw, n, e_phi):
    """(azimuthal share, off-axis fraction) of d's tangent-plane part."""
    pt = dw - float(np.dot(dw, n)) * n
    npt = float(np.linalg.norm(pt))
    if npt == 0.0:
        return 0.0, 1.0
    along = float(np.dot(pt, e_phi))
    off = float(np.linalg.norm(pt - along * e_phi)) / npt
    return along, off


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="outputs/precheck_step6")
    a = ap.parse_args()
    task = load_task("cup_hold")
    cup = Cup(task, 4)
    i = 0
    n, e_r, e_phi = cup.normals[i], cup.e_r[i], cup.e_phi[i]
    mu = cup.mu
    half = math.degrees(math.atan(mu))
    R = cup.R
    W = float(task["object"]["mass_kg"]) * float(task["gravity_m_s2"])
    nz = float(n[2])
    rows = []
    for f in all_fingers():
        r = {"id": f.id, "physics": f.physics, "b_rule": f.b_rule,
             "b_can": None if f.b is None else [round(x, 4) for x in f.b],
             "F_N_comparative": f.F, "d_can": None if f.d is None
             else [round(x, 4) for x in f.d], "S_m": f.S, "flags": f.flags()}
        for style in ("aimed", "palm"):
            for yaw in (0.0, 90.0):
                Rm = mount_rotation(f, style, n, e_r, e_phi,
                                    math.radians(yaw))
                key = f"{style}_yaw{int(yaw)}"
                if f.b is not None:
                    bw = Rm @ f.b
                    r[key + "_b_to_normal_deg"] = round(ang(bw, n), 3)
                    r[key + "_b_in_cone"] = ang(bw, n) <= half
                if f.d is not None:
                    along, off = tangent_split(Rm @ f.d, n, e_phi)
                    r[key + "_d_azimuth"] = round(along, 4)
                    r[key + "_d_offaxis"] = round(off, 4)
                    if f.S:
                        r[key + "_turn_per_stroke_deg"] = round(math.degrees(
                            f.S * abs(along) / (R * math.cos(cup.alt))), 2)
        if f.S is not None:
            r["S_over_R"] = round(f.S / R, 3)
        if f.F:
            r["N_needed_aimed_comparative"] = math.ceil(W / (nz * f.F)
                                                        - 1e-12)
        rows.append(r)
    out = {"task": "cup_hold", "mu": mu, "cone_half_angle_deg": half,
           "normal_to_vertical_deg": ang(n, np.array([0, 0, 1.0])),
           "weight_N": W, "normal_vertical_component": nz,
           "per_finger_push_needed_aimed_N": {
               str(k): W / (k * nz) for k in range(1, 9)},
           "records": rows}
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    with open(a.out + ".json", "w", encoding="utf-8") as fo:
        json.dump(out, fo, indent=1)
    lines = [
        "W20 step 6 pre-check (records and task geometry only; nothing solved)",
        f"cup: R {R*1e3:.2f} mm, altitude {task['contact_ring']['altitude_deg']:.3f} deg, "
        f"mu {mu}, cone half-angle {half:.3f} deg, normal {out['normal_to_vertical_deg']:.3f} deg "
        f"from vertical, weight {W:.4f} N",
        "per-finger push for the weight, aimed mount (normals): " + ", ".join(
            f"N={k}: {v:.4f}" for k, v in out["per_finger_push_needed_aimed_N"].items()),
        "",
        f"{'record':30s} {'rule':16s} {'F(cmp)':>9s} {'S mm':>7s} "
        f"{'palm0 b-n':>9s} {'palm90':>7s} {'aim90 az':>8s} {'aim90 off':>9s} "
        f"{'turn/str':>8s} {'N(cmp)':>6s} flags"]
    for r in rows:
        def g(k, fmt):
            v = r.get(k)
            return format(v, fmt) if isinstance(v, (int, float)) else "-"
        lines.append(
            f"{r['id'][:30]:30s} {r['b_rule']:16s} {g('F_N_comparative', '9.4g')} "
            f"{(r['S_m'] or 0)*1e3:7.2f} {g('palm_yaw0_b_to_normal_deg', '9.2f')} "
            f"{g('palm_yaw90_b_to_normal_deg', '7.2f')} {g('aimed_yaw90_d_azimuth', '8.3f')} "
            f"{g('aimed_yaw90_d_offaxis', '9.4f')} {g('aimed_yaw90_turn_per_stroke_deg', '8.1f')} "
            f"{g('N_needed_aimed_comparative', '6d')} {' '.join(r['flags'])}")
    txt = "\n".join(lines) + "\n"
    with open(a.out + ".txt", "w", encoding="utf-8") as fo:
        fo.write(txt)
    print(txt)


if __name__ == "__main__":
    main()
