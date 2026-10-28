"""Check a record-built hand against the cup-ring demos (design doc
docs/RECORD_FINGER_DESIGN.md, "Checks before results are shown", check 2):
do copies of one stand-in push the way the demos' copies of the same record
push, and as hard, at the rest pose?

    python tools/record_hand_vs_demo.py [--hand wall_x3_ring]

For each finger of the hand (all stand-ins of one record), at the record's
last blocked input level:
  stand-in  the MuJoCo model at rest, tip (the end of the chain) held in
            the bending plane, drive at a(u); the push on the holder from
            the small-motion balance K q - g a = J^T lam, J q = 0 (as
            gramian_workshop/record_fit.py predicts it in the record's
            plane)
  demo      the record's own push as the demos read it (finger.py: b and F
            from blocked.force_N at that level), turned into the world by the
            demos' "palm" mount at yaw 0 at the finger's azimuth (canonical X
            up, +Y toward the ring's axis), which is how the hand file mounts
            the stand-ins
Then the demos' cup_hold readout (readout.soft_hold) for N copies on the
palm_yaw0 mount, once with the record's push (as outputs/cup_hold.json) and
once with the record's push less its out-of-plane part (which a planar
stand-in cannot make), and once with the stand-in's push in its place, so
any difference in the reason can be traced to the push.

Writes outputs/record_hand_vs_demo__<hand>.json. Needs mujoco. The drive is
the record's unitless drive level, and the simulated record's forces are
comparative, so the strengths compared here are comparative too.
"""
import argparse
import copy
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop.arrange import Arrangement            # noqa: E402
from gramian_workshop.finger import Finger                  # noqa: E402
from gramian_workshop.follow import model_springs, torque_limits   # noqa: E402
from gramian_workshop.hands import HANDS, Hand, _mj         # noqa: E402
from gramian_workshop.readout import soft_hold              # noqa: E402
from gramian_workshop.record_fit import a_of_u              # noqa: E402
from gramian_workshop.task import load_task                 # noqa: E402


def _angle_deg(a, b):
    c = float(a @ b) / (np.linalg.norm(a) * np.linalg.norm(b))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def chain_end(m, body):
    """The far end of the body's capsule (the chain's tip), body frame."""
    mj = _mj()
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == body and int(m.geom_type[g]) == int(mj.mjtGeom.mjGEOM_CAPSULE):
            R = np.zeros(9)
            mj.mju_quat2Mat(R, m.geom_quat[g])
            ends = [m.geom_pos[g] + s * R.reshape(3, 3)[:, 2] * m.geom_size[g][1] for s in (-1, 1)]
            return max(ends, key=lambda e: float(np.linalg.norm(e)))
    raise ValueError("no capsule on the distal body")


def blocked_at_rest(hand, finger, a, ex, ey):
    """Push (world) on a holder pinning the chain's tip in the plane (ex, ey)
    at the rest pose, with the drive at a."""
    mj = _mj()
    m = hand.model
    d = mj.MjData(m)
    d.qpos[:] = m.qpos0
    mj.mj_kinematics(m, d)
    mj.mj_comPos(m, d)
    p = d.xpos[finger.tip_body] + d.xmat[finger.tip_body].reshape(3, 3) @ chain_end(m, finger.tip_body)
    jacp = np.zeros((3, m.nv))
    mj.mj_jac(m, d, jacp, None, p, finger.tip_body)
    J = np.vstack([ex @ jacp[:, finger.dof], ey @ jacp[:, finger.dof]])
    k, ref = model_springs(hand, finger)
    M, lim = torque_limits(hand, finger)
    tau = M[0] * a
    n = len(k)
    KKT = np.block([[np.diag(k), -J.T], [J, np.zeros((2, 2))]])
    lam = np.linalg.solve(KKT, np.concatenate([tau + np.diag(k) @ ref, np.zeros(2)]))[n:]
    return -(lam[0] * ex + lam[1] * ey)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hand", default="wall_x3_ring")
    a_ = ap.parse_args()
    spec = HANDS[a_.hand]
    hdir = os.path.dirname(os.path.join(ROOT, "data", "hands", spec["file"]))
    with open(os.path.join(hdir, "hand.json"), encoding="utf-8") as fh:
        hfile = json.load(fh)
    recs = {f["record"] for f in hfile["fingers"]}
    if len(recs) != 1:
        raise SystemExit("the check compares copies of one record; this hand mixes records")
    rid = recs.pop()
    fit_path = os.path.join(ROOT, "outputs", "record_fit", rid + ".json")
    with open(fit_path, encoding="utf-8") as fh:
        fitrep = json.load(fh)
    rec_finger = Finger.from_id(rid)
    blk = fitrep["blocked"][-1]
    u = float(blk["u"])
    a = a_of_u(fitrep["fit"], u)
    hand = Hand(a_.hand)
    up = np.array([0.0, 0.0, 1.0])
    rows = []
    for f, fs in zip(hand.fingers, hfile["fingers"]):
        mt = fs["mount"]
        pos = np.asarray(mt["pos_m"], float)
        phi = math.atan2(pos[1], pos[0])
        e_r = np.array([math.cos(phi), math.sin(phi), 0.0])
        ex, ey = np.asarray(mt["length_axis"], float), np.asarray(mt["bend_axis"], float)
        ex, ey = ex / np.linalg.norm(ex), ey / np.linalg.norm(ey)
        # the demos' palm mount at yaw 0: canonical X up, +Y toward the axis
        W = np.column_stack([up, -e_r, np.cross(up, -e_r)])
        demo = W @ rec_finger.b
        push = blocked_at_rest(hand, f, a, ex, ey)
        rows.append({
            "finger": f.label, "azimuth_deg": math.degrees(phi),
            "mount_vs_demo_frame_deg": {"length_axis": _angle_deg(ex, up),
                                        "bend_axis": _angle_deg(ey, -e_r)},
            "stand_in_push": push.tolist(), "stand_in_push_N": float(np.linalg.norm(push)),
            "demo_push_dir": demo.tolist(), "demo_F_N": float(rec_finger.F),
            "direction_diff_deg": _angle_deg(push, demo),
            "strength_ratio_stand_in_over_demo": float(np.linalg.norm(push) / rec_finger.F),
            "fit_report_pred_mag_N": float(blk["pred_mag_N"])})

    # the demos' cup_hold readout, N copies on palm_yaw0: the record's push,
    # then the stand-in's (its canonical push from the fit report)
    task = load_task("cup_hold")
    n = len(hand.fingers)
    alt = copy.copy(rec_finger)
    p2 = np.asarray(blk["pred_push_N"], float)
    alt.b = np.array([p2[0], p2[1], 0.0]) / np.linalg.norm(p2)
    alt.F = float(np.linalg.norm(p2))
    # the record's push with its out-of-plane part (canonical Z) dropped,
    # which a planar stand-in cannot make
    flat = copy.copy(rec_finger)
    flat.b = np.array([rec_finger.b[0], rec_finger.b[1], 0.0]) / np.linalg.norm(rec_finger.b[:2])
    readout = {}
    for lab, fg in (("record_push", rec_finger), ("record_push_in_plane", flat),
                    ("stand_in_push", alt)):
        arr = Arrangement(fg, task, n, "palm", 0.0)
        c = arr.cup
        r = soft_hold(c.points, c.normals, c.com, c.weight_demand(), c.mu, c.L,
                      arr.B, arr.F, arr.D, want_margin=True)
        readout[lab] = {"canonical_push_dir": [float(x) for x in fg.b], "F_N": float(fg.F),
                        **{k: r[k] for k in ("reason", "margin", "input_count")}}
    with open(os.path.join(ROOT, "outputs", "cup_hold.json"), encoding="utf-8") as fh:
        stored = json.load(fh)["soft"].get(f"{rid}|palm_yaw0|{n}")
    out = {"hand": a_.hand, "record": rid,
           "caveat": spec.get("caveat"),
           "input_u": u, "drive_a": a, "drive_top": float(torque_limits(hand, hand.fingers[0])[1][0, 1]),
           "fit_report": os.path.relpath(fit_path, ROOT).replace("\\", "/"),
           "fit_report_extrapolated_above_u": fitrep.get("extrapolated_above_u"),
           "fingers": rows,
           "cup_hold_readout_palm_yaw0": {"n": n, **readout,
                                          "stored_outputs_cup_hold": stored}}
    path = os.path.join(ROOT, "outputs", f"record_hand_vs_demo__{a_.hand}.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=1)
    for r in rows:
        print(f"{r['finger']}: push {r['stand_in_push_N']:.4g} vs demo F {r['demo_F_N']:.4g} "
              f"(ratio {r['strength_ratio_stand_in_over_demo']:.4g}); direction differs by "
              f"{r['direction_diff_deg']:.3g} deg; mount vs demo frame {r['mount_vs_demo_frame_deg']}")
    for lab, r in readout.items():
        print(f"cup_hold, {n} copies, palm_yaw0, {lab}: {r['reason']} (margin {r['margin']})")
    print("stored outputs/cup_hold.json:", stored)
    print("wrote", path)


if __name__ == "__main__":
    main()
