"""Does the Valero-Cuevas 1998 finger, built as a physical model, behave?

    python tools/vc_finger_mujoco.py [--column adjusted]  ->  outputs/vc1998_mujoco_checks.json

A MuJoCo index finger: metacarpal fixed, MCP ad/abduction and flexion, PIP
and DIP flexion, phalanges 50 / 31 / 16 mm (Table 2); seven muscles as fixed
tendons whose joint coefficients are the moment arms of
tools/valero_cuevas_finger.py (so tendon force T gives joint torque r T),
each driven by a motor of maximal force F0 (Table 1). Not from the paper
(assumptions, stated): phalanx masses and radii, joint damping, joint ranges,
no gravity, no passive joint stiffness.

Three tests:
  1  isometric: the fingertip pinned at the 1998 test posture by a point
     constraint (a frictionless point contact that cannot slip), each of the
     five maximal-force excitation patterns applied; is the measured tip
     force what the matrix predicts, and does the finger hold its posture?
  2  the same with an excitation pattern that leaves distal-phalanx torque
     unbalanced: the paper's constraint says the finger cannot hold; does it
     roll?
  3  free: each muscle alone at 5 % from the test posture, no contact; which
     way does each joint start to move, against the textbook actions of
     each muscle?
  4  statics and stability, no dynamics: MuJoCo's own kinematics and tendon
     transmission give the tip force J^-T tau (the matrix check), and the
     pinned finger's one spare motion (tip fixed, DIP moved, the rest
     solved) is stable if the muscles' potential -tau.q rises along it
     (constant moment arms, no muscle stiffness: the paper's model taken
     literally). Test 1 runs dynamically through a soft pin, so its forces
     carry a transient; test 4 is the statics check.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import valero_cuevas_finger as vc  # noqa: E402

JOINTS = ["mcp_add", "mcp_flex", "pip", "dip"]
# assumptions (not in the paper)
MASS = (0.012, 0.006, 0.004)        # kg, proximal / middle / distal
RADIUS = (0.009, 0.008, 0.007)      # m
DAMPING = 0.02                      # N.m.s/rad
RANGE_DEG = {"mcp_add": (-30, 30), "mcp_flex": (-30, 90), "pip": (0, 110), "dip": (-10, 90)}
# textbook actions at MCP flex, PIP, DIP (+ flex, - extend, 0 none) and MCP
# adduction (+ add toward the middle finger, - abduct): FDP flexes all three,
# FDS flexes MCP and PIP, EIP/EDC extend all three, LUM flexes MCP and extends
# the IP joints, DI abducts, PI adducts
TEXTBOOK = {"FDP": {"mcp_flex": 1, "pip": 1, "dip": 1}, "FDS": {"mcp_flex": 1, "pip": 1},
            "EIP": {"mcp_flex": -1, "pip": -1, "dip": -1}, "EDC": {"mcp_flex": -1, "pip": -1, "dip": -1},
            "LUM": {"mcp_flex": 1, "pip": -1, "dip": -1}, "DI": {"mcp_add": -1}, "PI": {"mcp_add": 1}}


def mjcf(R_m, F0):
    lo = {k: math.radians(v[0]) for k, v in RANGE_DEG.items()}
    hi = {k: math.radians(v[1]) for k, v in RANGE_DEG.items()}
    seg = lambda L, r, m: (f'<geom type="capsule" fromto="0 0 0 0 {L} 0" size="{r}" mass="{m}" '
                           f'contype="0" conaffinity="0"/>')
    tendons = "".join(
        f'<fixed name="t_{mu}">' + "".join(
            f'<joint joint="{j}" coef="{R_m[i, k]:.6g}"/>' for i, j in enumerate(JOINTS) if R_m[i, k] != 0)
        + "</fixed>" for k, mu in enumerate(vc.MUSCLES))
    acts = "".join(f'<motor name="{mu}" tendon="t_{mu}" gear="{F0[k]:.6g}" ctrlrange="0 1" ctrllimited="true"/>'
                   for k, mu in enumerate(vc.MUSCLES))
    rng = lambda j: f'range="{lo[j]} {hi[j]}" limited="true"'
    return f"""<mujoco><compiler angle="radian"/><option gravity="0 0 0" timestep="0.0005"/>
<default><joint damping="{DAMPING}" armature="1e-5"/></default>
<worldbody><body name="prox">
  <joint name="mcp_add" axis="0 0 1" {rng('mcp_add')}/><joint name="mcp_flex" axis="1 0 0" {rng('mcp_flex')}/>
  {seg(vc.L_PROX, RADIUS[0], MASS[0])}
  <body name="mid" pos="0 {vc.L_PROX} 0"><joint name="pip" axis="1 0 0" {rng('pip')}/>
    {seg(vc.L_MID, RADIUS[1], MASS[1])}
    <body name="dist" pos="0 {vc.L_MID} 0"><joint name="dip" axis="1 0 0" {rng('dip')}/>
      {seg(vc.L_DIST, RADIUS[2], MASS[2])}<site name="tip" pos="0 {vc.L_DIST} 0"/>
    </body></body></body></worldbody>
<tendon>{tendons}</tendon><actuator>{acts}</actuator>
<equality><connect name="pin" body1="dist" anchor="0 {vc.L_DIST} 0" active="false"/></equality>
</mujoco>"""


def posture():
    P = vc.POSTURE
    return np.array([P["MCP_add"], P["MCP_flex"], P["PIP"], P["DIP"]])


def settle(mj, m, d, e, pinned, T=0.5):
    q0 = posture()
    mj.mj_resetData(m, d)
    d.qpos[:] = q0
    mj.mj_forward(m, d)
    if pinned:
        m.eq_active0[0] = 1
        d.eq_active[0] = 1
        # anchor on the world side: the tip's position at the posture
        m.eq_data[0, 3:6] = d.site_xpos[0]
    else:
        m.eq_active0[0] = 0
        d.eq_active[0] = 0
    d.ctrl[:] = e
    n = int(T / m.opt.timestep)
    for _ in range(n):
        mj.mj_step(m, d)
    mj.mj_forward(m, d)
    return q0


def tip_force(mj, m, d):
    """Force the fingertip applies to the pin, in the distal-phalanx frame
    (x lateral, y distal, z palmar), from the constraint's joint forces."""
    jacp = np.zeros((3, m.nv))
    mj.mj_jacSite(m, d, jacp, None, 0)
    f_on_finger = np.linalg.lstsq(jacp.T, d.qfrc_constraint, rcond=None)[0]
    Rd = d.xmat[m.body("dist").id].reshape(3, 3)
    return Rd.T @ (-f_on_finger)


def _tip(q):
    q1, q2, q3, q4 = q
    R1 = vc._rz(q1)
    p = np.zeros(3)
    for ang, L in ((q2, vc.L_PROX), (q2 + q3, vc.L_MID), (q2 + q3 + q4, vc.L_DIST)):
        p = p + R1 @ vc._rx(ang) @ np.array([0, L, 0])
    return p


def selfmotion(s):
    """The pinned finger's spare motion: DIP moved by s, the other joints
    solved to keep the tip where it is at the posture."""
    from scipy.optimize import fsolve
    q0 = posture()
    p0 = _tip(q0)
    x = fsolve(lambda x: _tip(np.array([x[0], x[1], x[2], q0[3] + s])) - p0, q0[:3], xtol=1e-14)
    return np.array([x[0], x[1], x[2], q0[3] + s])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--column", default="adjusted", choices=["nominal", "adjusted"])
    a = ap.parse_args()
    import mujoco as mj
    R_mm, F0, _ = vc.build(a.column)
    m = mj.MjModel.from_xml_string(mjcf(R_mm / 1e3, F0))
    d = mj.MjData(m)
    J, M = vc.model(posture(), a.column)
    out = {"column": a.column, "assumptions": {"mass_kg": MASS, "radius_m": RADIUS, "damping": DAMPING,
                                               "range_deg": RANGE_DEG, "gravity": 0}}
    print(f"== {a.column} model as a MuJoCo finger")
    print("Test 1, pinned tip, the maximal-force excitations (N, [lateral, distal, palmar]):")
    t1 = {}
    for k, dvec in vc.DIRS.items():
        s, e = vc.max_force(M, dvec)
        q0 = settle(mj, m, d, e, True)
        f = tip_force(mj, m, d)
        pred = M[:3] @ e
        drift = np.degrees(np.abs(d.qpos - q0)).max()
        t1[k] = {"predicted_N": pred.tolist(), "simulated_N": f.tolist(),
                 "error_N": float(np.linalg.norm(f - pred)), "posture_drift_deg": float(drift)}
        print(f"   {k:8s} predicted {np.round(pred, 2)}  simulated {np.round(f, 2)}  "
              f"error {np.linalg.norm(f - pred):.3f} N  posture drift {drift:.3f} deg")
    out["test1_pinned_max_force"] = t1
    print("Test 2, pinned tip, every muscle at 30 % (distal-phalanx torque "
          f"{(M[3] @ np.full(7, 0.3)) * 1e3:.1f} N.mm, not balanced):")
    e = np.full(7, 0.3)
    q0 = settle(mj, m, d, e, True, T=0.3)
    drift = np.degrees(d.qpos - q0)
    print(f"   joints moved {dict(zip(JOINTS, np.round(drift, 1)))} deg in 0.3 s")
    out["test2_unbalanced"] = {"tx_Nmm": float(M[3] @ e * 1e3), "joint_change_deg": dict(zip(JOINTS, drift.tolist()))}
    print("Test 3, free, each muscle alone at 5 %: joint motion after 20 ms (deg), against textbook action")
    t3, agree, total = {}, 0, 0
    for k, mu in enumerate(vc.MUSCLES):
        e = np.zeros(7)
        e[k] = 0.05
        q0 = settle(mj, m, d, e, False, T=0.02)
        dq = np.degrees(d.qpos - q0)
        exp = TEXTBOOK[mu]
        marks = []
        for j, v in exp.items():
            ok = np.sign(dq[JOINTS.index(j)]) == v
            agree += ok
            total += 1
            marks.append(f"{j} {'ok' if ok else 'WRONG'}")
        t3[mu] = {"joint_change_deg": dict(zip(JOINTS, dq.tolist())), "textbook": exp}
        print(f"   {mu:4s} {dict(zip(JOINTS, np.round(dq, 2)))}  {', '.join(marks)}")
    print(f"   textbook actions reproduced: {agree}/{total}")
    out["test3_free"] = t3
    out["test3_agree"] = [int(agree), int(total)]
    print("Test 4, statics (MuJoCo kinematics and transmission) and stability of the pinned posture:")
    d.qpos[:] = posture()
    mj.mj_forward(m, d)
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    mj.mj_jacSite(m, d, jp, jr, 0)
    Rd = d.xmat[m.body("dist").id].reshape(3, 3)
    Jm = np.vstack([Rd.T @ jp, (Rd.T @ jr)[0:1]])
    t4 = {"jacobian_max_diff": float(np.abs(Jm - J).max())}
    h = math.radians(0.5)
    qs = [selfmotion(-h), posture(), selfmotion(h)]
    for k, dvec in vc.DIRS.items():
        s, e = vc.max_force(M, dvec)
        d.ctrl[:] = e
        mj.mj_forward(m, d)
        f = np.linalg.solve(Jm.T, d.qfrc_actuator)
        tau = (R_mm / 1e3) @ (F0 * e)
        V = [-tau @ q for q in qs]
        curv = (V[0] - 2 * V[1] + V[2]) / h ** 2
        state = "stable" if curv > 1e-6 else ("UNSTABLE" if curv < -1e-6 else "neutral")
        t4[k] = {"static_tip_force_N": f[:3].tolist(), "matrix_N": (M[:3] @ e).tolist(),
                 "distal_torque_Nmm": float(f[3] * 1e3), "curvature_Nm_per_rad2": float(curv),
                 "stability": state}
        print(f"   {k:8s} static {np.round(f[:3], 2)} vs matrix {np.round(M[:3] @ e, 2)}; "
              f"self-motion curvature {curv * 1e3:8.2f} N.mm/rad^2: {state}")
    out["test4_statics_stability"] = t4
    with open(os.path.join(ROOT, "outputs", f"vc1998_mujoco_checks_{a.column}.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    print(f"wrote outputs/vc1998_mujoco_checks_{a.column}.json")


if __name__ == "__main__":
    main()
