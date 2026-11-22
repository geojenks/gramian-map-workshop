"""The Shadow Hand follows the recorded learned turning gait: MuJoCo with
contact, a free cylinder, and a three-column clip (21 Nov 2026).

    python tools/shadow_follows_hora_gait_mujoco_21nov.py [--tag b] [--render-only]

SIMULATION ONLY. Comparative, not predictive. Open loop: no hand here knows
where the cylinder is. The recorded gait came from a policy that did know, so
an open-loop miss is NOT evidence that the real hand cannot do the task.

WHAT IS RUN. One 10 s stretch of the recording (data/rl_logs/hora_seed42_30s*,
from T0), three hands in the same scene with the same cylinder:

  allegro        REFERENCE. The Allegro model (MuJoCo Menagerie) is sent the
                 recording's own joint commands. If this does not turn the
                 cylinder about as logged, a Shadow miss says nothing about
                 the Shadow.
  shadow         The Shadow Hand model as it is: on each finger one tendon
                 actuator fixes only the SUM of the last two joint angles.
  shadow tied    The same, with the last two joints of each finger held equal
                 by a joint equality (a new model file; the Menagerie file is
                 not edited). Which of the two is nearer the real hand is not
                 verified here.

Each with the cylinder free (the test) and with its weight carried (it slides
and turns freely but cannot fall or change height), and the Shadow once with no cylinder
and no press (how well it tracks its targets in free space; a check).

ASSUMED, all fixed before the first run and not tuned afterwards:
  - Allegro base: placed by a rigid fit of the model's link positions to the
    logged link positions. The residual is printed.
  - Allegro fingertip: the recording's hand has a ball fingertip (radius and
    position read from its URDF, the copy in Wave_manip
    experiment_log/models/allegro_internal_hora.urdf). The Menagerie model's
    own fingertip ends about 20 mm short of it, so its fingertip contact is
    switched off and that ball is added to the last link.
  - Allegro drive: the recording's controller, torque = 3 x (command - angle)
    - 0.1 x speed, at most 0.5 N.m (gains as in Wave_manip w16s2_hora_driver).
    The model's own joint damping, 0.1, stands for the speed term. The log
    holds torque, not command, so command = angle + (torque + 0.1 x speed)/3;
    where the logged torque sits at 0.5 the true command was further out.
    Nothing is added to it.
  - Finger pairing: Allegro index, middle, ring, thumb to Shadow first,
    middle, ring fingers and thumb. The little finger is held straight and
    back. (The stored 7 Nov snapshot check used another pairing, found for
    five frozen moments against a ball; this run is not a re-run of it.)
  - Shadow fingertip: the rounded end of the last link is treated as a ball
    of the link's half width (8 mm; thumb 9.6 mm) centred that far back from
    the tip of the mesh. Its target is the centre of the Allegro's fingertip
    ball, moved by the difference in ball radius towards the logged cylinder
    surface, so both fingertips sit at the same distance from the surface.
  - Shadow base: six numbers, fitted to make the summed squared reach miss of
    the four fingertips, plus the squared depth by which any other part of
    the hand cuts into the logged cylinder, smallest over 30 moments.
  - Shadow joint targets: inverse kinematics on fingertip position only, last
    two joints equal, each moment started from the one before. Joint limits
    kept.
  - Press (Shadow only): the model's own position actuators and gains. To
    each target is added (torque to make the LOGGED normal contact force at the fingertip)
    / (actuator gain). No shear force is added; friction comes from the
    moving targets. Faded in over the first 0.2 s.
  - Cylinder: logged size and mass; friction 0.7212 on every surface (the
    log's value for both hand and cylinder). Starts at its logged pose and
    velocity.
  - Real time (not slowed). Each model's own contact settings and time step.
  - Changed after the first run (21 Nov, never completed, folder left in
    place): that run used the Menagerie Allegro fingertip and sent the logged
    angles through the Menagerie gain; no fingertip reached the cylinder and
    it fell at once in every column. The two Allegro items above are the
    fix. The no-cylinder check then also had the press added, which it
    should not; removed. No Shadow assumption was changed, but its fingertip
    targets move with the corrected Allegro fingertip.
  - Changed after the second run (21 Nov, folder ..._b, left in place; its
    Allegro lost the cylinder at 0.16 s, so it says nothing about the Shadow):
    1. The main test is now "cylinder moved as recorded": every step the
       cylinder is put at its logged place with its logged speed, and only
       the hand is simulated. A policy's commands replayed without its
       feedback cannot hold a friction grip in another engine, so the free
       cylinder is kept only as one Allegro row, reported as it is.
    2. Allegro: its weight is switched off, as in the recording (Wave_manip
       w16s2_hora_driver.py line 581), and it starts with the logged joint
       speeds.
    3. Shadow: to every command is added what holds the hand's own weight at
       the target pose (bias force / gain), since the model's wrist otherwise
       droops and carries the fingertips about 10 mm off. Joint targets may
       move at most MAX_REF_SPEED; they jumped before. In placing the base,
       other hand parts cutting into the cylinder now weigh CUT_WEIGHT times
       a fingertip miss.
    An "axle" support (cylinder on a fixed hinge) was tried and is not used.
  - Lost: the cylinder's centre more than 30 mm from where the log has it.
    Stopped: a not-a-number, or the cylinder faster than 5 m/s (falling
    away counts).

Writes a new folder outputs/shadow_follows_hora_gait_mujoco_21nov[_tag]/ with
report.md, results.json, the three model .xml files, runs.npz, the clip and a
still. Refuses to overwrite.
"""
import argparse
import json
import os
import re
import subprocess
import sys

import mujoco
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MEN = os.path.join(ROOT, "external", "mujoco_menagerie")
LOGS = os.path.join(ROOT, "data", "rl_logs")
CYL_URDF_RADIUS_M, CYL_URDF_HALFLEN_M = 0.04, 0.032   # Wave_manip w16s1_hora_scout.py lines 96-97
T0, DUR = 2.0, 10.0
RAMP_S = 0.2
LOST_M, UNSTABLE_MPS = 0.030, 5.0
FIT_MOMENTS = 30
MAX_REF_SPEED = 6.0         # rad/s, Shadow joint targets
CUT_WEIGHT = 10.0
ALLEGRO_WEIGHTLESS = True   # the recording's hand had gravity off (Wave_manip w16s2_hora_driver.py, line 581)
HORA_KP, HORA_KD, HORA_TAU_MAX = 3.0, 0.1, 0.5   # Wave_manip w16s2_hora_driver.py
HORA_URDF = os.path.join(os.path.dirname(ROOT), "ARIA", "Wave_manip", "experiment_log",
                         "models", "allegro_internal_hora.urdf")
W, H, FPS = 640, 600, 30

ALLEGRO = {"file": "wonik_allegro/right_hand.xml", "root": "palm",
           # log column -> model joint (log order: index, thumb, middle, ring)
           "joints": ["ffj0", "ffj1", "ffj2", "ffj3", "thj0", "thj1", "thj2", "thj3",
                      "mfj0", "mfj1", "mfj2", "mfj3", "rfj0", "rfj1", "rfj2", "rfj3"],
           "log_bodies": ["palm", "ff_base", "ff_proximal", "ff_medial", "ff_distal",
                          "th_base", "th_proximal", "th_medial", "th_distal",
                          "mf_base", "mf_proximal", "mf_medial", "mf_distal",
                          "rf_base", "rf_proximal", "rf_medial", "rf_distal"],
           # fingers in the force log's order: index, middle, ring, thumb
           "tips": None}       # filled from the recording hand's URDF by hora_pads()


def hora_pads():
    """Ball fingertips of the recording's hand: (last-link body, centre in that
    link's frame, radius), fingers in the force log's order. Parsed as in
    Wave_manip w16s1_hora_scout.tip_pad_from_urdf."""
    urdf = open(HORA_URDF).read()
    out = []
    for body, link in (("ff_distal", "link_3.0"), ("mf_distal", "link_7.0"),
                       ("rf_distal", "link_11.0"), ("th_distal", "link_15.0")):
        jm = next(j for j in re.finditer(r"<joint[^>]*>(.*?)</joint>", urdf, re.S)
                  if f'<parent link="{link}"/>' in j.group(1))
        j_xyz = [float(v) for v in re.search(r'<origin[^>]*?xyz="([^"]+)"', jm.group(1)).group(1).split()]
        lm = re.search(r'<link name="%s_tip">(.*?)</link>' % re.escape(link), urdf, re.S)
        col = re.search(r"<collision>(.*?)</collision>", lm.group(1), re.S).group(1)
        r = float(re.search(r'<sphere radius="([^"]+)"', col).group(1))
        s_xyz = [float(v) for v in re.search(r'<origin[^>]*?xyz="([^"]+)"', col).group(1).split()]
        out.append((body, [a + b for a, b in zip(j_xyz, s_xyz)], r))
    return out
SHADOW = {"file": "shadow_hand/right_hand.xml", "root": "rh_forearm",
          "fingers": [
              {"body": "rh_ffdistal", "loc": [0, 0, 0.024], "r": 0.008,
               "groups": [["rh_FFJ4"], ["rh_FFJ3"], ["rh_FFJ2", "rh_FFJ1"]],
               "acts": ["rh_A_FFJ4", "rh_A_FFJ3", "rh_A_FFJ0"]},
              {"body": "rh_mfdistal", "loc": [0, 0, 0.024], "r": 0.008,
               "groups": [["rh_MFJ4"], ["rh_MFJ3"], ["rh_MFJ2", "rh_MFJ1"]],
               "acts": ["rh_A_MFJ4", "rh_A_MFJ3", "rh_A_MFJ0"]},
              {"body": "rh_rfdistal", "loc": [0, 0, 0.024], "r": 0.008,
               "groups": [["rh_RFJ4"], ["rh_RFJ3"], ["rh_RFJ2", "rh_RFJ1"]],
               "acts": ["rh_A_RFJ4", "rh_A_RFJ3", "rh_A_RFJ0"]},
              {"body": "rh_thdistal", "loc": [0, 0, 0.0262], "r": 0.0096,
               "groups": [["rh_THJ5"], ["rh_THJ4"], ["rh_THJ3"], ["rh_THJ2"], ["rh_THJ1"]],
               "acts": ["rh_A_THJ5", "rh_A_THJ4", "rh_A_THJ3", "rh_A_THJ2", "rh_A_THJ1"]}],
          "tied": [("rh_FFJ1", "rh_FFJ2"), ("rh_MFJ1", "rh_MFJ2"),
                   ("rh_RFJ1", "rh_RFJ2"), ("rh_LFJ1", "rh_LFJ2")]}


# ---------------------------------------------------------------- small maths
def q_wxyz(q_xyzw):
    q = np.asarray(q_xyzw, float)
    return np.concatenate([q[..., 3:4], q[..., :3]], -1)


def qmat(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, float))
    return R.reshape(3, 3)


def matq(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).ravel())
    return q


def rodrigues(w):
    q = np.zeros(4)
    a = np.linalg.norm(w)
    mujoco.mju_axisAngle2Quat(q, w / a if a > 0 else np.array([0, 0, 1.0]), a)
    return qmat(q)


def rigid_fit(A, B):
    """R, t with R A + t ~ B (least squares)."""
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cb - R @ ca


def cyl_sdf(x, R, half):
    dr, dz = np.hypot(x[0], x[1]) - R, abs(x[2]) - half
    return np.hypot(max(dr, 0), max(dz, 0)) + min(max(dr, dz), 0)


def cyl_out(x, R, half, h=1e-5):
    g = np.array([cyl_sdf(x + h * e, R, half) - cyl_sdf(x - h * e, R, half) for e in np.eye(3)])
    n = np.linalg.norm(g)
    return g / n if n > 0 else np.array([0, 0, 1.0])


def own_axis_turn(quats):
    """Turn about the body's own z axis, summed sample to sample (rad)."""
    out = np.zeros(len(quats))
    for i in range(1, len(quats)):
        dR = qmat(quats[i - 1]).T @ qmat(quats[i])
        out[i] = out[i - 1] + np.arctan2(dR[1, 0] - dR[0, 1], dR[0, 0] + dR[1, 1])
    return out


def nelder_mead(f, x0, step, iters):
    n = len(x0)
    S = [np.array(x0, float)] + [np.array(x0, float) + step[i] * np.eye(n)[i] for i in range(n)]
    F = [f(s) for s in S]
    for _ in range(iters):
        o = np.argsort(F)
        S, F = [S[i] for i in o], [F[i] for i in o]
        c = np.mean(S[:-1], 0)
        xr = c + (c - S[-1]); fr = f(xr)
        if fr < F[0]:
            xe = c + 2 * (c - S[-1]); fe = f(xe)
            S[-1], F[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < F[-2]:
            S[-1], F[-1] = xr, fr
        else:
            xc = c + 0.5 * (S[-1] - c); fc = f(xc)
            if fc < F[-1]:
                S[-1], F[-1] = xc, fc
            else:
                S = [S[0]] + [S[0] + 0.5 * (s - S[0]) for s in S[1:]]
                F = [F[0]] + [f(s) for s in S[1:]]
    o = int(np.argmin(F))
    return S[o], F[o]


# ---------------------------------------------------------------- the scene
def build(hand, base_R, base_t, support, tied, L, xml_path=None):
    """support: 'free', 'carried' or 'none' (no cylinder)."""
    spec = mujoco.MjSpec.from_file(os.path.join(MEN, hand["file"]))
    spec.meshdir = os.path.join(MEN, os.path.dirname(hand["file"]), "assets")
    for k in list(spec.keys):
        spec.delete(k)
    root = spec.body(hand["root"])
    root.pos, root.quat = base_t, matq(base_R)
    # 22 Nov: the DEX-EE model file hangs its fingers on the world, beside the base, so they
    # are moved with it. No other hand lists "moves_with_root"; nothing changes for them.
    for name in hand.get("moves_with_root", []):
        b = spec.body(name)
        b.pos, b.quat = base_t + base_R @ np.array(b.pos), matq(base_R @ qmat(np.array(b.quat)))
    if hand is ALLEGRO:
        if ALLEGRO_WEIGHTLESS:
            for b in spec.bodies[1:]:
                b.gravcomp = 1.0
        for tip in ("ff_tip", "mf_tip", "rf_tip", "th_tip"):
            for g in spec.body(tip).geoms:
                g.contype, g.conaffinity = 0, 0
        for body, loc, r in ALLEGRO["tips"]:
            spec.body(body).add_geom(name=body + "_ball", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                                     size=[r, 0, 0], pos=loc, mass=0, rgba=[0.9, 0.9, 0.9, 1])
        for a in spec.actuators:
            a.gainprm[0], a.biasprm[1] = HORA_KP, -HORA_KP
            a.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
            a.forcerange = [-HORA_TAU_MAX, HORA_TAU_MAX]
            a.ctrllimited = mujoco.mjtLimited.mjLIMITED_FALSE
    for g in spec.geoms:
        g.friction[0] = L["mu"]
    if tied:
        for a, b in hand["tied"]:
            e = spec.add_equality()
            e.type, e.objtype = mujoco.mjtEq.mjEQ_JOINT, mujoco.mjtObj.mjOBJ_JOINT
            e.name1, e.name2 = a, b
            e.data[:5] = [0, 1, 0, 0, 0]
    if support != "none":
        b = spec.worldbody.add_body(name="obj")
        if support in ("free", "led"):
            b.add_freejoint(name="obj_free")
            if support == "led":
                b.gravcomp = 1.0
        elif support == "axle":
            b.pos, b.quat = L["axle_pos"], L["axle_quat"]
            b.add_joint(name="obj_turn", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1])
        else:
            b.pos = [0, 0, L["op"][0][2]]
            b.add_joint(name="obj_x", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[1, 0, 0])
            b.add_joint(name="obj_y", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 1, 0])
            b.add_joint(name="obj_ball", type=mujoco.mjtJoint.mjJNT_BALL)
        b.add_geom(name="obj", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[L["R"], L["half"], 0],
                   mass=L["mass"], friction=[L["mu"], 0.005, 0.0001], rgba=[0.85, 0.75, 0.45, 1])
        for k, ang in enumerate((0.0, np.pi / 2)):      # marks, drawn only
            b.add_geom(name=f"mark{k}", type=mujoco.mjtGeom.mjGEOM_BOX,
                       size=[L["R"] * 1.01, 0.003, L["half"] * 1.02], mass=0,
                       quat=matq(rodrigues(np.array([0, 0, ang]))) if ang else [1, 0, 0, 0],
                       contype=0, conaffinity=0,
                       rgba=[0.75, 0.2, 0.15, 1] if k == 0 else [0.2, 0.3, 0.6, 1])
    m = spec.compile()
    if xml_path:
        with open(xml_path, "x", encoding="utf-8") as fo:
            fo.write(spec.to_xml())
    return m


def set_obj(m, d, support, pos, quat, vel=None):
    if support in ("free", "led"):
        a = m.joint("obj_free").qposadr[0]
        d.qpos[a:a + 3], d.qpos[a + 3:a + 7] = pos, quat
        if vel is not None:
            v = m.joint("obj_free").dofadr[0]
            d.qvel[v:v + 3] = vel[:3]
            d.qvel[v + 3:v + 6] = qmat(quat).T @ vel[3:]
    elif support == "axle":
        d.qpos[m.joint("obj_turn").qposadr[0]] = 0.0
        if vel is not None:
            d.qvel[m.joint("obj_turn").dofadr[0]] = (qmat(quat).T @ vel[3:])[2]
    elif support == "carried":
        d.qpos[m.joint("obj_x").qposadr[0]] = pos[0]
        d.qpos[m.joint("obj_y").qposadr[0]] = pos[1]
        a = m.joint("obj_ball").qposadr[0]
        d.qpos[a:a + 4] = quat


def point_jac(m, d, body, world_point):
    J = np.zeros((3, m.nv))
    mujoco.mj_jac(m, d, J, None, world_point, body)
    return J


# ---------------------------------------------------------------- the recording
def load():
    F = np.load(os.path.join(LOGS, "hora_seed42_30s_forces.npz"), allow_pickle=True)
    T = np.load(os.path.join(LOGS, "hora_seed42_30s.npz"), allow_pickle=True)
    Z = np.load(os.path.join(LOGS, "hora_seed42_30s_physics.npz"), allow_pickle=True)
    t = F["t"].astype(float)
    k = np.where((t >= T0) & (t <= T0 + DUR))[0]
    scale = float(T["obj_scale"])
    return {"t": t[k] - t[k[0]], "q": Z["q"][k].astype(float), "qd": Z["qd"][k].astype(float),
            "tau": Z["tau"][k].astype(float), "body": Z["body"][k].astype(float),
            "op": F["obj_pos"][k].astype(float), "oq": q_wxyz(F["obj_quat"][k]),
            "ovel": Z["obj"][k][:, 7:13].astype(float),
            "N": F["normal_N"][k].astype(float), "nrm": F["normal_dir"][k].astype(float),
            "P": F["point"][k].astype(float), "names": [str(x) for x in F["fingers"]],
            "R": CYL_URDF_RADIUS_M * scale, "half": CYL_URDF_HALFLEN_M * scale,
            "mass": float(Z["obj_mass"]), "mu": float(np.ravel(Z["obj_fric"])[0]),
            "hand_mu": float(np.ravel(Z["hand_fric"])[0])}


# ---------------------------------------------------------------- Allegro reference
def allegro_reference(L, say):
    m = build(ALLEGRO, np.eye(3), np.zeros(3), "none", False, L)
    d = mujoco.MjData(m)
    adr = [m.joint(j).qposadr[0] for j in ALLEGRO["joints"]]
    bid = [m.body(b).id for b in ALLEGRO["log_bodies"]]
    idx = np.linspace(0, len(L["t"]) - 1, 40).astype(int)
    A, B = [], []
    for i in idx:
        d.qpos[adr] = L["q"][i]
        mujoco.mj_kinematics(m, d)
        A.append(d.xpos[bid].copy()); B.append(L["body"][i, :, :3])
    A, B = np.concatenate(A), np.concatenate(B)
    R, tr = rigid_fit(A, B)
    res = np.linalg.norm(A @ R.T + tr - B, axis=1)
    say(f"Allegro base fit to the logged link positions: mean miss {1e3 * res.mean():.2f} mm, "
        f"largest {1e3 * res.max():.2f} mm ({len(idx)} moments, 17 links)")
    # fingertip ball centres in the world, feed-forward, at every sample
    m = build(ALLEGRO, R, tr, "none", False, L)
    d = mujoco.MjData(m)
    n = len(L["t"])
    tips = np.zeros((n, 4, 3)); ctrl = np.zeros((n, m.nu))
    act = [m.actuator("%sa%d" % (j[:2], int(j[3]))).id for j in ALLEGRO["joints"]]
    gap = []
    for i in range(n):
        d.qpos[adr] = L["q"][i]
        mujoco.mj_kinematics(m, d)
        Ro = qmat(L["oq"][i])
        for k, (b, loc, r) in enumerate(ALLEGRO["tips"]):
            B_ = m.body(b).id
            tips[i, k] = d.xpos[B_] + d.xmat[B_].reshape(3, 3) @ np.array(loc)
            if L["N"][i, k] > 0:
                gap.append(cyl_sdf(Ro.T @ (tips[i, k] - L["op"][i]), L["R"], L["half"]) - r)
    gap = 1e3 * np.array(gap)
    say(f"Allegro ball fingertip against the logged cylinder, whenever the log has that finger touching: "
        f"gap median {np.median(gap):+.2f} mm, from {gap.min():+.2f} to {gap.max():+.2f} mm "
        f"({len(gap)} samples; negative is overlap)")
    ctrl[:, act] = L["q"] + (L["tau"] + HORA_KD * L["qd"]) / HORA_KP
    capped = float(np.mean(np.abs(L["tau"]) >= HORA_TAU_MAX - 1e-6))
    say(f"Allegro commands rebuilt from logged torque; logged torque at its {HORA_TAU_MAX} N.m cap in "
        f"{100 * capped:.1f} % of joint samples")
    plain = ctrl
    q0 = np.zeros(m.nq); q0[adr] = L["q"][0]
    qd0 = np.zeros(m.nv); qd0[[m.joint(j).dofadr[0] for j in ALLEGRO["joints"]]] = L["qd"][0]
    return {"R": R, "t": tr, "tips": tips, "ctrl": ctrl, "plain": plain, "q0": q0, "qd0": qd0,
            "fit_mm": [float(1e3 * res.mean()), float(1e3 * res.max())],
            "ball_gap_mm_median_min_max": [float(np.median(gap)), float(gap.min()), float(gap.max())],
            "share_of_logged_torque_at_cap": capped}


# ---------------------------------------------------------------- Shadow targets
class ShadowIK:
    def __init__(self, L):
        self.L = L
        self.m = build(SHADOW, np.eye(3), np.zeros(3), "free", False, L)
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.f = []
        for f in SHADOW["fingers"]:
            g = [[m.joint(j).id for j in grp] for grp in f["groups"]]
            self.f.append({"body": m.body(f["body"]).id, "loc": np.array(f["loc"]), "r": f["r"],
                           "qadr": [[m.jnt_qposadr[j] for j in grp] for grp in g],
                           "dadr": [[m.jnt_dofadr[j] for j in grp] for grp in g],
                           "lo": np.array([max(m.jnt_range[j][0] for j in grp) for grp in g]),
                           "hi": np.array([min(m.jnt_range[j][1] for j in grp) for grp in g])})
        self.distal = {f["body"] for f in self.f}
        self.obj_geom = m.geom("obj").id
        self.lf3 = m.joint("rh_LFJ3").qposadr[0]
        self.lf3_lo = m.jnt_range[m.joint("rh_LFJ3").id][0]

    def tip(self, k):
        f, d = self.f[k], self.d
        return d.xpos[f["body"]] + d.xmat[f["body"]].reshape(3, 3) @ f["loc"]

    def solve(self, k, target, x0, iters=30):
        f, m, d = self.f[k], self.m, self.d
        x = np.clip(np.array(x0, float), f["lo"], f["hi"])
        for _ in range(iters):
            for a, v in zip(f["qadr"], x):
                d.qpos[a] = v
            mujoco.mj_kinematics(m, d); mujoco.mj_comPos(m, d)
            p = self.tip(k)
            e = target - p
            if np.linalg.norm(e) < 1e-6:
                break
            J = point_jac(m, d, f["body"], p)
            Jg = np.column_stack([J[:, a].sum(1) for a in f["dadr"]])
            x = np.clip(x + Jg.T @ np.linalg.solve(Jg @ Jg.T + 1e-6 * np.eye(3), e), f["lo"], f["hi"])
        for a, v in zip(f["qadr"], x):
            d.qpos[a] = v
        mujoco.mj_kinematics(m, d)
        return x, float(np.linalg.norm(target - self.tip(k)))

    def neutral(self, k):
        f = self.f[k]
        return f["lo"] + 0.4 * (f["hi"] - f["lo"])

    def track(self, R, tr, targets, idx, cut=False):
        """Targets (world) -> joint values per finger; reach miss; depth by
        which other hand parts cut into the logged cylinder."""
        L, m, d = self.L, self.m, self.d
        d.qpos[:] = 0
        d.qpos[self.lf3] = self.lf3_lo
        x = [self.neutral(k) for k in range(4)]
        step = MAX_REF_SPEED * float(np.mean(np.diff(L["t"]))) if len(idx) == len(L["t"]) else None
        X = [np.zeros((len(idx), len(x[k]))) for k in range(4)]
        miss = np.zeros((len(idx), 4)); depth = np.zeros(len(idx))
        for n, i in enumerate(idx):
            for k in range(4):
                new, miss[n, k] = self.solve(k, R.T @ (targets[i, k] - tr), x[k])
                if step is not None and n > 0:
                    new = np.clip(new, x[k] - step, x[k] + step)
                    for a_, v_ in zip(self.f[k]["qadr"], new):
                        d.qpos[a_] = v_
                    mujoco.mj_kinematics(m, d)
                    miss[n, k] = float(np.linalg.norm(R.T @ (targets[i, k] - tr) - self.tip(k)))
                x[k] = new
                X[k][n] = x[k]
            if cut:
                set_obj(m, d, "free", R.T @ (L["op"][i] - tr), matq(R.T @ qmat(L["oq"][i])))
                mujoco.mj_kinematics(m, d); mujoco.mj_collision(m, d)
                for c in d.contact[:d.ncon]:
                    o = c.geom2 if c.geom1 == self.obj_geom else c.geom1 if c.geom2 == self.obj_geom else -1
                    if o >= 0 and m.geom_bodyid[o] not in self.distal and c.dist < 0:
                        depth[n] = max(depth[n], -c.dist)
        return X, miss, depth


def shadow_reference(L, al, say):
    ik = ShadowIK(L)
    n = len(L["t"])
    # targets: the Allegro's fingertip ball centre, moved towards the logged surface
    targets = np.zeros((n, 4, 3)); out = np.zeros((n, 4, 3))
    for i in range(n):
        Ro = qmat(L["oq"][i])
        for k in range(4):
            out[i, k] = Ro @ cyl_out(Ro.T @ (al["tips"][i, k] - L["op"][i]), L["R"], L["half"])
            targets[i, k] = al["tips"][i, k] + (ik.f[k]["r"] - ALLEGRO["tips"][k][2]) * out[i, k]
    # first guess for the base: knuckles on the Allegro's finger bases, tips on mean targets
    m, d = ik.m, ik.d
    d.qpos[:] = 0
    for k in range(4):
        for a, v in zip(ik.f[k]["qadr"], ik.neutral(k)):
            d.qpos[a] = v
    mujoco.mj_kinematics(m, d)
    A = [d.xanchor[m.joint(j).id].copy() for j in ("rh_FFJ3", "rh_MFJ3", "rh_RFJ3", "rh_THJ5")]
    A += [ik.tip(k).copy() for k in range(4)]
    bases = L["body"][:, [1, 9, 13, 5], :3].mean(0)          # index, middle, ring, thumb bases
    B = list(bases) + list(targets.mean(0))
    R0, t0 = rigid_fit(np.array(A), np.array(B))
    idx = np.linspace(0, n - 1, FIT_MOMENTS).astype(int)

    def cost(p):
        R = rodrigues(p[:3]) @ R0
        _, miss, depth = ik.track(R, t0 + p[3:], targets, idx, cut=True)
        return float((miss ** 2).sum() + CUT_WEIGHT * (depth ** 2).sum())

    c0 = cost(np.zeros(6))
    p, c = nelder_mead(cost, np.zeros(6), [0.15] * 3 + [0.02] * 3, 350)
    R, tr = rodrigues(p[:3]) @ R0, t0 + p[3:]
    say(f"Shadow base fit: cost {c0:.3e} at the first guess, {c:.3e} after the search (m^2, "
        f"{FIT_MOMENTS} moments)")
    X, miss, depth = ik.track(R, tr, targets, np.arange(n), cut=True)
    names = ["first", "middle", "ring", "thumb"]
    reach = {}
    for k in range(4):
        f = ik.f[k]
        at = float(np.mean(np.any((X[k] <= f["lo"] + 1e-4) | (X[k] >= f["hi"] - 1e-4), axis=1)))
        touching = L["N"][:, k] > 0
        reach[names[k]] = {"mean_mm": float(1e3 * miss[:, k].mean()), "max_mm": float(1e3 * miss[:, k].max()),
                           "mean_mm_while_logged_touching": float(1e3 * miss[touching, k].mean()) if touching.any() else None,
                           "share_of_time_at_a_joint_limit": at}
        say(f"  reach, Shadow {names[k]} finger: mean miss {reach[names[k]]['mean_mm']:.1f} mm, largest "
            f"{reach[names[k]]['max_mm']:.1f} mm; at a joint limit {100 * at:.0f} % of the time")
    say(f"  other hand parts cut into the logged cylinder by at most {1e3 * depth.max():.1f} mm "
        f"(mean {1e3 * depth.mean():.1f} mm)")
    # controls, with the base in place
    m = build(SHADOW, R, tr, "none", False, L)
    d = mujoco.MjData(m)
    ctrl = np.zeros((n, m.nu)); plain = np.zeros((n, m.nu)); qref = np.zeros((n, m.nq))
    ctrl[:, m.actuator("rh_A_LFJ3").id] = plain[:, m.actuator("rh_A_LFJ3").id] = ik.lf3_lo
    qref[:, m.joint("rh_LFJ3").qposadr[0]] = ik.lf3_lo
    tip_target = targets
    for i in range(n):
        for k, f in enumerate(SHADOW["fingers"]):
            for grp, v in zip(f["groups"], X[k][i]):
                for j in grp:
                    qref[i, m.joint(j).qposadr[0]] = v
        d.qpos[:] = qref[i]; d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        hold = np.zeros(m.nu)
        for a in range(m.nu):
            if m.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT:
                dofs = [m.jnt_dofadr[m.actuator_trnid[a, 0]]]
            else:
                tn = m.actuator_trnid[a, 0]      # fixed tendon: its joints
                dofs = [m.jnt_dofadr[m.wrap_objid[w]]
                        for w in range(m.tendon_adr[tn], m.tendon_adr[tn] + m.tendon_num[tn])]
            hold[a] = d.qfrc_bias[dofs].mean() / m.actuator_gainprm[a, 0]
        for k, f in enumerate(SHADOW["fingers"]):
            b = m.body(f["body"]).id
            tau = np.zeros(m.nv)
            if L["N"][i, k] > 0:
                c = d.xpos[b] + d.xmat[b].reshape(3, 3) @ np.array(f["loc"])
                tau = point_jac(m, d, b, c - f["r"] * out[i, k]).T @ L["f"][i, k]
            for grp, a, v in zip(f["groups"], f["acts"], X[k][i]):
                a = m.actuator(a).id
                dofs = [m.joint(j).dofadr[0] for j in grp]
                plain[i, a] = v * len(grp)
                ctrl[i, a] = plain[i, a] + tau[dofs].mean() / m.actuator_gainprm[a, 0]
        plain[i] += hold; ctrl[i] += hold
    return {"R": R, "t": tr, "ctrl": ctrl, "plain": plain, "q0": qref[0], "qref": qref,
            "targets": tip_target, "reach": reach,
            "cut_mm": [float(1e3 * depth.mean()), float(1e3 * depth.max())],
            "fit_cost": [c0, c]}


# ---------------------------------------------------------------- one run
def obj_at(L, t):
    i = min(max(int(np.searchsorted(L["t"], t)) - 1, 0), len(L["t"]) - 2)
    w = (t - L["t"][i]) / (L["t"][i + 1] - L["t"][i])
    q0, q1 = L["oq"][i], L["oq"][i + 1]
    if q0 @ q1 < 0:
        q1 = -q1
    q = (1 - w) * q0 + w * q1
    return ((1 - w) * L["op"][i] + w * L["op"][i + 1], q / np.linalg.norm(q),
            (1 - w) * L["ovel"][i] + w * L["ovel"][i + 1])


def run(hand, ref, L, support, tied, tips, xml_path=None):
    m = build(hand, ref["R"], ref["t"], support, tied, L, xml_path)
    d = mujoco.MjData(m)
    nh = len(ref["q0"])
    d.qpos[:nh] = ref["q0"]
    if "qd0" in ref:
        d.qvel[:nh] = ref["qd0"]
    if support != "none":
        set_obj(m, d, support, L["op"][0], L["oq"][0], L["ovel"][0])
    mujoco.mj_forward(m, d)
    steps = int(round(DUR / m.opt.timestep))
    every = max(1, int(round(1 / (120 * m.opt.timestep))))
    ob = m.body("obj").id if support != "none" else -1
    tb = [(m.body(b).id, np.array(loc)) for b, loc in tips]
    lo, hi = m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
    rec = {"t": [], "qpos": [], "op": [], "oq": [], "tips": [], "sat": [], "clip": [],
           "touch": [], "N": [], "other": [], "Fz": [], "act": []}
    unstable = None
    og = m.geom("obj").id if support != "none" else -1
    tip_bodies = [b for b, _ in tb]
    fr_ = m.actuator_forcerange[:, 1]
    for s in range(steps + 1):
        t = s * m.opt.timestep
        if support == "led":
            set_obj(m, d, "led", *obj_at(L, t))
            if s % every == 0:      # contacts as left by the step before
                touch = np.zeros(4, bool); N = np.zeros(4); other = False; Fz = 0.0
                for ci in range(d.ncon):
                    c = d.contact[ci]
                    if og not in (c.geom1, c.geom2):
                        continue
                    f6 = np.zeros(6); mujoco.mj_contactForce(m, d, ci, f6)
                    o = c.geom1 if c.geom2 == og else c.geom2
                    fw = c.frame.reshape(3, 3).T @ f6[:3]
                    Fz += fw[2] if c.geom2 == og else -fw[2]
                    ob_ = m.geom_bodyid[o]
                    if ob_ in tip_bodies:
                        k_ = tip_bodies.index(ob_); touch[k_] = True; N[k_] += f6[0]
                    elif f6[0] > 0:
                        other = True
                rec["touch"].append(touch); rec["N"].append(N); rec["other"].append(other); rec["Fz"].append(Fz)
                rec["act"].append(np.abs(d.actuator_force) / np.where(fr_ > 0, fr_, np.inf))
        w = min(1.0, t / RAMP_S)
        tgt = ref["plain"] if support == "none" else ref["ctrl"]
        c = np.array([np.interp(t, L["t"], (1 - w) * ref["plain"][:, a] + w * tgt[:, a])
                      for a in range(m.nu)])
        d.ctrl[:] = c
        if s % every == 0:
            rec["t"].append(t); rec["qpos"].append(d.qpos.copy())
            rec["tips"].append([d.xpos[b] + d.xmat[b].reshape(3, 3) @ loc for b, loc in tb])
            cl = m.actuator_ctrllimited.astype(bool)
            rec["clip"].append(float(np.mean((c[cl] < lo[cl] - 1e-9) | (c[cl] > hi[cl] + 1e-9))) if cl.any() else 0.0)
            fr = m.actuator_forcerange
            lim = fr[:, 1] > fr[:, 0]
            rec["sat"].append(float(np.mean(np.abs(d.actuator_force[lim]) >= fr[lim, 1] - 1e-6)) if lim.any() else 0.0)
            if ob >= 0:
                rec["op"].append(d.xpos[ob].copy()); rec["oq"].append(d.xquat[ob].copy())
        mujoco.mj_step(m, d)
        bad = not np.all(np.isfinite(d.qpos))
        if ob >= 0 and not bad and support != "led":
            bad = np.linalg.norm(d.cvel[ob][3:]) > UNSTABLE_MPS
        if bad:
            unstable = t
            break
    return m, {k: np.array(v) for k, v in rec.items()}, unstable


def measure(rec, unstable, L, target_tips, m=None, led=False):
    t = rec["t"]
    out = {"unstable_at_s": unstable, "ran_s": float(t[-1]),
           "share_of_commands_outside_control_range": float(rec["clip"].mean()),
           "share_of_actuator_samples_at_force_limit": float(rec["sat"].mean())}
    tt = np.array([[np.interp(t, L["t"], target_tips[:, k, a]) for a in range(3)] for k in range(4)])
    err = np.linalg.norm(rec["tips"] - tt.transpose(2, 0, 1), axis=2)
    out["fingertip_from_target_mm"] = {"mean": float(1e3 * err.mean()), "max": float(1e3 * err.max()),
                                       "per_finger_mean": [float(1e3 * x) for x in err.mean(0)]}
    if led:
        names = ["first", "middle", "ring", "thumb"]
        logN = np.column_stack([np.interp(t, L["t"], L["N"][:, k]) for k in range(4)])
        logtouch = np.column_stack([np.interp(t, L["t"], (L["N"][:, k] > 0).astype(float)) > 0.5 for k in range(4)])
        out["fingers"] = {names[k]: {
            "touching_share": float(rec["touch"][:, k].mean()), "logged_touching_share": float(logtouch[:, k].mean()),
            "touch_agrees_with_log_share": float((rec["touch"][:, k] == logtouch[:, k]).mean()),
            "mean_normal_force_N": float(rec["N"][:, k].mean()), "logged_mean_normal_force_N": float(logN[:, k].mean()),
            "largest_normal_force_N": float(rec["N"][:, k].max()), "logged_largest_normal_force_N": float(L["N"][:, k].max())}
            for k in range(4)}
        out["other_hand_parts_pressing_on_cylinder_share"] = float(rec["other"].mean())
        out["share_of_cylinder_weight_carried_time_average"] = float(rec["Fz"].mean() / (L["mass"] * 9.81))
        lim = m.actuator_forcerange[:, 1] > 0
        out["actuators"] = {m.actuator(a).name: {"largest_share_of_force_range": float(rec["act"][:, a].max()),
                                                 "mean_share_of_force_range": float(rec["act"][:, a].mean()),
                                                 "share_of_time_at_limit": float((rec["act"][:, a] >= 1 - 1e-6).mean())}
                            for a in range(m.nu) if lim[a]}
        return out
    if len(rec["op"]):
        logp = np.column_stack([np.interp(t, L["t"], L["op"][:, a]) for a in range(3)])
        off = np.linalg.norm(rec["op"] - logp, axis=1)
        lost = np.where(off > LOST_M)[0]
        e = int(lost[0]) if len(lost) else len(t) - 1
        turn = own_axis_turn(rec["oq"])
        logturn = np.interp(t, L["t"], L["turn"])
        tilt = np.degrees(np.arccos(np.clip([abs(qmat(q)[2, 2]) for q in rec["oq"]], 0, 1)))
        out.update({"lost_at_s": float(t[e]) if len(lost) else None,
                    "turn_deg_until_lost_or_end": float(np.degrees(turn[e])),
                    "logged_turn_deg_same_time": float(np.degrees(logturn[e])),
                    "turn_deg_whole_run": float(np.degrees(turn[-1])),
                    "logged_turn_deg_whole_run": float(np.degrees(logturn[-1])),
                    "centre_from_logged_mm_max_until_lost": float(1e3 * off[:e + 1].max()),
                    "centre_from_logged_mm_end": float(1e3 * off[-1]),
                    "tilt_deg_max_until_lost": float(tilt[:e + 1].max()),
                    "lowest_height_m": float(rec["op"][:, 2].min())})
        rec["turn"] = turn
    return out


# ---------------------------------------------------------------- the clip
def film(out, L, runs, res):
    from PIL import Image, ImageDraw, ImageFont
    cols = []
    for key, title in (("allegro", "Allegro, the recording's commands (check)"),
                       ("shadow", "Shadow Hand, model as it is"),
                       ("shadow_tied", "Shadow Hand, last two joints tied")):
        m = mujoco.MjModel.from_xml_path(os.path.join(out, f"model_{key}.xml"))
        m.vis.global_.offwidth, m.vis.global_.offheight = W, H
        m.vis.headlight.ambient[:] = 0.5
        m.vis.headlight.diffuse[:] = 0.7
        cols.append((key, title, m, mujoco.MjData(m), mujoco.Renderer(m, H, W), mujoco.Renderer(m, H, W)))
        cols[-1][5].enable_segmentation_rendering()
    cam = mujoco.MjvCamera()
    cam.lookat[:] = L["op"].mean(0) + np.array([0, 0, -0.02])
    cam.distance, cam.azimuth, cam.elevation = 0.42, 135, -40
    font, small = ImageFont.truetype("arial.ttf", 20), ImageFont.truetype("arial.ttf", 15)
    mp4 = os.path.join(out, "shadow_follows_hora_gait_21nov.mp4")
    still = os.path.join(out, "shadow_follows_hora_gait_21nov_still.png")
    p = subprocess.Popen(["ffmpeg", "-n", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                          "-s", f"{3 * W}x{H}", "-r", str(FPS), "-i", "-", "-pix_fmt", "yuv420p",
                          "-crf", "20", mp4], stdin=subprocess.PIPE)
    nf = int(DUR * FPS)
    for fi in range(nf + 1):
        t = fi / FPS
        row = []
        for key, title, m, d, r, seg in cols:
            rec, ms = runs[key], res[key + ", cylinder moved as recorded"]
            i = min(int(np.searchsorted(rec["t"], t)), len(rec["t"]) - 1)
            d.qpos[:] = rec["qpos"][i]
            mujoco.mj_forward(m, d)
            r.update_scene(d, cam); rgb = r.render().copy()
            seg.update_scene(d, cam); rgb[seg.render()[:, :, 0] < 0] = (236, 236, 232)
            im = Image.fromarray(rgb); dr = ImageDraw.Draw(im)
            dr.text((12, 10), title, fill=(30, 30, 30), font=font)
            dr.text((12, 38), f"t = {t:4.1f} s   fingertips touching: "
                              + " ".join(n_ for n_, on in zip(("first", "middle", "ring", "thumb"), rec["touch"][i]) if on),
                    fill=(30, 30, 30), font=small)
            dr.text((12, H - 46), "the cylinder is moved as recorded; only the hand is simulated",
                    fill=(90, 90, 90), font=small)
            dr.text((12, H - 26), "simulation only: comparative, not predictive",
                    fill=(90, 90, 90), font=small)
            row.append(np.asarray(im))
        f = np.hstack(row)
        p.stdin.write(f.tobytes())
        if fi == nf // 2:
            Image.fromarray(f).save(still)
    p.stdin.close(); p.wait()
    return mp4, still


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="")
    ap.add_argument("--no-film", action="store_true")
    a = ap.parse_args()
    out = os.path.join(ROOT, "outputs", "shadow_follows_hora_gait_mujoco_21nov" + ("_" + a.tag if a.tag else ""))
    if os.path.exists(out):
        print("REFUSING to overwrite:", out)
        return 1
    os.makedirs(out)
    lines = []

    def say(s):
        print(s, flush=True)
        lines.append(s)

    L = load()
    # force of each fingertip ON the cylinder: the logged normal force, pointing into it
    sgn = np.sign(sum(float(np.einsum("ij,ij->", L["nrm"][:, k] * L["N"][:, k:k + 1],
                                      L["P"][:, k] - L["op"])) for k in range(4)))
    L["f"] = -sgn * L["nrm"] * L["N"][:, :, None]
    L["turn"] = own_axis_turn(L["oq"])
    L["axle_pos"], L["axle_quat"] = L["op"].mean(0), L["oq"][0]
    ALLEGRO["tips"] = hora_pads()
    say("Recording hand's ball fingertips (from its URDF): " + "; ".join(
        f"{b} centre {1e3 * loc[2]:.1f} mm along the last link, radius {1e3 * r:.2f} mm"
        for b, loc, r in ALLEGRO["tips"]))
    say(f"Recording: {T0:.1f} to {T0 + DUR:.1f} s; cylinder radius {1e3 * L['R']:.1f} mm, half length "
        f"{1e3 * L['half']:.1f} mm, mass {1e3 * L['mass']:.1f} g; friction {L['mu']:.4f} (hand {L['hand_mu']:.4f}); "
        f"logged turn about its own axis {np.degrees(L['turn'][-1]):+.0f} deg; logged normal forces up to "
        f"{L['N'].max():.2f} N; normal-direction sign {sgn:+.0f}")
    al = allegro_reference(L, say)
    sh = shadow_reference(L, al, say)
    sh_tips = [(f["body"], f["loc"]) for f in SHADOW["fingers"]]
    al_tips = [(b, loc) for b, loc, _ in ALLEGRO["tips"]]
    cases = [("allegro", ALLEGRO, al, False, al_tips, al["tips"]),
             ("shadow", SHADOW, sh, False, sh_tips, sh["targets"]),
             ("shadow_tied", SHADOW, sh, True, sh_tips, sh["targets"])]
    res, keep = {}, {}
    for key, hand, ref, tied, tips, tgt in cases:
        for support, label in (("none", "no cylinder, no press"), ("led", "cylinder moved as recorded"), ("free", "cylinder free")):
            if (support == "none") == (hand is ALLEGRO) and support != "led":
                continue
            xml = os.path.join(out, f"model_{key}.xml") if support == "led" else None
            m, rec, unstable = run(hand, ref, L, support, tied, tips, xml)
            ms = measure(rec, unstable, L, tgt, m, support == "led")
            if hand is SHADOW:
                gap = [rec["qpos"][:, m.joint(a).qposadr[0]] - rec["qpos"][:, m.joint(b).qposadr[0]]
                       for a, b in SHADOW["tied"][:3]]
                ms["end_joint_minus_middle_joint_deg"] = {
                    "mean_abs": float(np.degrees(np.mean(np.abs(gap)))),
                    "max_abs": float(np.degrees(np.max(np.abs(gap))))}
            res[f"{key}, {label}"] = ms
            if support == "led":
                keep[key] = rec
            s = f"{key}, {label}: fingertips {ms['fingertip_from_target_mm']['mean']:.1f} mm from target on average"
            if support == "led":
                say(s + f"; other hand parts press on the cylinder {100 * ms['other_hand_parts_pressing_on_cylinder_share']:.0f} % of the time"
                        f"; fingers carry {100 * ms['share_of_cylinder_weight_carried_time_average']:.0f} % of its weight on average")
                for n_, v in ms["fingers"].items():
                    say(f"    {n_}: touching {100 * v['touching_share']:.0f} % of the time (log {100 * v['logged_touching_share']:.0f} %), "
                        f"agrees with the log {100 * v['touch_agrees_with_log_share']:.0f} %; mean force {v['mean_normal_force_N']:.2f} N "
                        f"(log {v['logged_mean_normal_force_N']:.2f}); largest {v['largest_normal_force_N']:.1f} N (log {v['logged_largest_normal_force_N']:.1f})")
                worst = sorted(ms["actuators"].items(), key=lambda kv: -kv[1]["largest_share_of_force_range"])[:4]
                say("    actuators nearest their force limit: " + "; ".join(
                    f"{a_} largest {100 * v['largest_share_of_force_range']:.0f} % of range, at the limit {100 * v['share_of_time_at_limit']:.1f} % of the time, mean {100 * v['mean_share_of_force_range']:.0f} % of range"
                    for a_, v in worst))
                continue
            if support != "none":
                s += (f"; turned {ms['turn_deg_until_lost_or_end']:+.0f} deg (log {ms['logged_turn_deg_same_time']:+.0f})"
                      f"; lost at {ms['lost_at_s']} s; stopped at {ms['unstable_at_s']} s")
            say(s)
    np.savez(os.path.join(out, "runs.npz"), **{f"{k}_{n}": v for k, r in keep.items() for n, v in r.items()})
    results = {"made_by": "tools/shadow_follows_hora_gait_mujoco_21nov.py", "window_s": [T0, T0 + DUR],
               "allegro_base_fit_mm_mean_max": al["fit_mm"],
               "allegro_ball_gap_mm_median_min_max": al["ball_gap_mm_median_min_max"],
               "allegro_share_of_logged_torque_at_cap": al["share_of_logged_torque_at_cap"], "shadow_reach": sh["reach"],
               "shadow_other_parts_cut_into_cylinder_mm_mean_max": sh["cut_mm"],
               "shadow_base_fit_cost_first_guess_and_final": sh["fit_cost"],
               "shadow_base_rotation": sh["R"].tolist(), "shadow_base_translation_m": sh["t"].tolist(),
               "runs": res}
    with open(os.path.join(out, "results.json"), "x", encoding="utf-8") as fo:
        json.dump(results, fo, indent=1, default=lambda o: np.asarray(o).tolist())
    with open(os.path.join(out, "report.md"), "x", encoding="utf-8") as fo:
        fo.write("# The Shadow Hand follows the recorded turning gait (21 Nov 2026)\n\n"
                 "Simulation only: comparative, not predictive. The cylinder is moved as recorded; only the hands are simulated. Printed by "
                 "`tools/shadow_follows_hora_gait_mujoco_21nov.py`; what is assumed is in its header.\n\n"
                 + "\n".join("- " + s.strip() for s in lines) + "\n")
    print("wrote", os.path.join(out, "results.json"))
    print("wrote", os.path.join(out, "report.md"))
    if not a.no_film:
        for f in film(out, L, keep, res):
            print("wrote", f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
