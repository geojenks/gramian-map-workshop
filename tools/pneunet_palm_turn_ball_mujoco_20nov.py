"""Pneu-net fingers on a palm turning a ball, in MuJoCo with real contact.
SIMULATION ONLY, comparative, not predictive.

Follows tools/pneunet_palm_turn_ball_20nov.py (kinematic, slip ignored) and
tools/pneunet_palm_two_input_cycle_20nov.py. Same records, same mounts (read
from their output files), same ball. This is a different model: the finger
is the fitted hinge-chain stand-in of tools/build_record_hand.py with its
joint springs, the ball is a free body, and contact with friction is solved
by MuJoCo. Its numbers need not agree with the kinematic ones.

What differs from tools/build_record_hand.py (that file is not changed):
  - the fingers lie flat on the palm (hinge axes vertical), at the mounts of
    the kinematic scripts;
  - joint limits are switched off, so contact can push a finger off its free
    path against its springs (the builder limits each joint to its free
    span, which would lock the finger against the ball);
  - each joint's damping is its critical damping, 2 sqrt(K I), from its
    spring and the inertia MuJoCo computes (the builder's one default number
    is not used);
  - links of one finger do not collide with each other; the palm is drawn
    only;
  - gravity is on; the ball is a free 50 g body on a rest.

Two rests under the ball, both run:
  smooth  a flat rest with no friction (the ball turns freely on it);
  seat    three fixed points 10 mm from the axis with the task's friction
          (an assumed seat: the rest friction depends on that radius).

Checks made before any contact run, per hand:
  - mount geometry: with the model posed at q = r a(u) at every input level,
    the tip's distance from the ball's axis less the ring radius must equal
    the kinematic script's;
  - no finger touches the ball at zero input;
  - the input ramp is slow enough: with the ball's contact off, the driven
    tip stays on its posed path.
The seat runs are repeated at half and a quarter of the ramp speed.

A run is called unstable (decided before running) if any of: a MuJoCo
warning, a non-finite state, ball speed over 1 m/s, a contact overlap over
half the tip radius, or the ball's centre moving more than its radius.

Run from the repository root, with a Python that has mujoco:
    python tools/pneunet_palm_turn_ball_mujoco_20nov.py
Writes into outputs/pneunet_palm_turn_ball_20nov/mujoco/: mujoco_report.txt,
mujoco_results.json, one model .xml and one log .csv per run, clips (.mp4)
and stills (.png). Needs ffmpeg for the clips.
"""
import json
import os
import shutil
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
import mujoco as mj                                            # noqa: E402
import build_record_hand as BH                                 # noqa: E402
import pneunet_palm_turn_ball_20nov as M                       # noqa: E402
from gramian_workshop.record_fit import Chain, a_of_u          # noqa: E402

OUT = os.path.join(M.OUT, "mujoco")
SEAT_RADIUS = 0.010       # assumed: seat points this far from the axis
SEAT_POINT = 0.002        # radius of each seat point
DT = 0.0005
RAMP = 10.0               # seconds for one ramp of an input
HOLD = 1.0
FPS, SPEED = 30, 4        # clip plays 4 times faster than simulated time
W, H = 1280, 720

HANDS = [
    {"key": "wall_0.0020m_x3", "record": "wall_0.0020m", "n": 3,
     "inputs": 1, "what": "three 2 mm wall fingers, one shared input"},
    {"key": "wall_0.0010m_x4", "record": "wall_0.0010m", "n": 4,
     "inputs": 1, "what": "four 1 mm wall fingers, one shared input"},
    {"key": "wall_0.0005m_2x3", "record": "wall_0.0005m", "n": 3,
     "inputs": 2, "what": "two sets of three 0.5 mm wall fingers, "
                           "two inputs"},
]


def rot3(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def mount_of(hand):
    """Mount and kinematic prediction from the kinematic scripts' outputs."""
    if hand["inputs"] == 1:
        res = json.load(open(os.path.join(M.OUT, "results.json")))
        ch = res["records"][hand["record"]]["hands"][str(hand["n"])][
            "chosen"]["record depth"]
        return ch, ch["turn_deg"]
    res = json.load(open(os.path.join(M.OUT, "two_input_cycle.json")))
    ch = res[hand["record"]][str(hand["n"])]
    return ch, ch["net_deg"]


def build(hand, task, rest):
    R = task["object"]["radius_m"]
    mass = task["object"]["mass_kg"]
    mu = task["mu"]
    alt = abs(np.radians(task["contact_ring"]["altitude_deg"]))
    ch, kin_deg = mount_of(hand)
    spec0 = {"label": "f", "record": f"data/records/{hand['record']}.json"}
    ff, fit = BH.fit_finger(spec0)
    rc = ff["radius"]
    rho = (R + rc) * np.cos(alt)
    z_ring = R - (R + rc) * np.sin(alt)        # rest top is z = 0
    c = np.array(ch["axis_in_finger_frame_m"])
    n, n_tot = hand["n"], hand["n"] * hand["inputs"]
    bodies, tendons, motors, fingers = [], [], [], []
    for k in range(n_tot):
        s, m_ = divmod(k, n) if hand["inputs"] == 2 else (0, k)
        phi = 2 * np.pi * m_ / n + s * np.pi / n
        Rz = rot3(phi)
        pos = -Rz @ np.array([c[0], c[1], 0.0]) + [0, 0, z_ring]
        spec = dict(spec0, label=f"s{s}f{m_}",
                    mount={"pos_m": pos.tolist(),
                           "length_axis": Rz[:, 0].tolist(),
                           "bend_axis": Rz[:, 1].tolist()})
        b, t, mo, names = BH.finger_xml(spec, ff)
        bodies.append(b)
        tendons.append(t)
        motors.append(mo)
        fingers.append({"label": spec["label"], "set": s, "joints": names})
    f = fit["fit"]
    chain = Chain(f["n_hinges"], f["slide"], f["length_m"])
    excl = []
    for fg in fingers:
        lk = [f"{fg['label']}_link{i}" for i in range(chain.n_h - 1)] \
            + [f"{fg['label']}_distal"]
        excl += [f'<exclude body1="{lk[a]}" body2="{lk[b]}"/>'
                 for a in range(len(lk)) for b in range(a + 1, len(lk))]
    reach = float(np.linalg.norm(c)) + 0.02
    if rest == "smooth":
        rest_geoms = ('<geom name="rest0" type="cylinder" pos="0 0 -0.004" '
                      f'size="{SEAT_RADIUS} 0.004" rgba="0.5 0.5 0.55 1" '
                      'contype="0" conaffinity="0"/>')
        pairs = '<pair geom1="ball" geom2="rest0" condim="1"/>'
        z_ball = R
    else:
        dz = np.sqrt((R + SEAT_POINT) ** 2 - SEAT_RADIUS ** 2)
        z_ball = R                               # seat points set to fit
        rest_geoms, pairs = "", ""
        for i in range(3):
            a = 2 * np.pi * i / 3 + 0.3
            rest_geoms += (
                f'<geom name="rest{i}" type="sphere" size="{SEAT_POINT}" '
                f'pos="{SEAT_RADIUS * np.cos(a):.6f} '
                f'{SEAT_RADIUS * np.sin(a):.6f} {z_ball - dz:.6f}" '
                'rgba="0.5 0.5 0.55 1" contype="0" conaffinity="0"/>')
            pairs += (f'<pair geom1="ball" geom2="rest{i}" condim="3" '
                      f'friction="{mu} {mu} 0.005 0.0001 0.0001"/>')
    xml = "\n".join([
        f'<mujoco model="pneunet_palm_{hand["key"]}_{rest}">',
        "<!-- generated by tools/pneunet_palm_turn_ball_mujoco_20nov.py."
        " SIMULATION ONLY. A motor's force is the drive coordinate a(u), not"
        " newtons. Joint limits and damping are reset after loading by that"
        " script. -->",
        '<compiler angle="radian"/>',
        f'<option gravity="0 0 -9.81" timestep="{DT}" integrator="implicitfast"/>',
        f'<visual><global offwidth="{W}" offheight="{H}"/>'
        '<headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6"/></visual>',
        '<default><joint armature="1e-5"/>'
        f'<geom friction="{mu} 0.005 0.0001" rgba="0.2 0.5 0.8 1"/></default>',
        '<asset><texture type="skybox" builtin="flat" rgb1="1 1 1" '
        'rgb2="1 1 1" width="32" height="32"/>'
        '<texture name="ball" type="cube" builtin="checker" '
        'rgb1="0.95 0.8 0.3" rgb2="0.75 0.3 0.15" width="64" height="64"/>'
        '<material name="ball" texture="ball" texrepeat="3 3"/></asset>',
        "<worldbody>",
        f' <geom name="palm" type="cylinder" pos="0 0 {z_ring - rc - 0.003:.6f}" '
        f'size="{reach:.4f} 0.003" rgba="0.82 0.8 0.75 1" contype="0" '
        'conaffinity="0"/>',
        " " + rest_geoms,
        f' <body name="ball" pos="0 0 {z_ball:.6f}"><freejoint name="ball"/>'
        f'<geom name="ball" type="sphere" size="{R}" mass="{mass}" '
        'material="ball"/></body>',
        *bodies,
        "</worldbody>",
        "<contact>" + pairs + "".join(excl) + "</contact>",
        "<tendon>" + "".join(tendons) + "</tendon>",
        "<actuator>" + "".join(motors) + "</actuator>",
        "</mujoco>", ""])
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"model_{hand['key']}_{rest}.xml")
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(xml)
    m = mj.MjModel.from_xml_path(path)
    # joint limits off; critical damping per joint
    d = mj.MjData(m)
    mj.mj_forward(m, d)
    Mfull = np.zeros((m.nv, m.nv))
    mj.mj_fullM(m, Mfull, d.qM)
    for fg in fingers:
        fg["jid"] = [m.joint(nm).id for nm in fg["joints"]]
        fg["qadr"] = [int(m.jnt_qposadr[j]) for j in fg["jid"]]
        fg["dof"] = [int(m.jnt_dofadr[j]) for j in fg["jid"]]
        fg["act"] = m.actuator(f"{fg['label']}_drive").id
        fg["distal"] = m.body(f"{fg['label']}_distal").id
        for j, dof in zip(fg["jid"], fg["dof"]):
            m.jnt_limited[j] = 0
            m.dof_damping[dof] = 2 * np.sqrt(m.jnt_stiffness[j]
                                             * Mfull[dof, dof])
    u_hi = ff["u_range"][1]
    # table of the drive coordinate; the record's own levels are included
    # so that the posed model matches the kinematic script at each of them
    ug = np.unique(np.concatenate([
        np.linspace(0, u_hi, 2001),
        [uu for uu in np.asarray(ff["rec"]["free"]["u"], float).ravel()
         if 0 <= uu <= u_hi]]))
    ag = np.array([a_of_u(f, uu) for uu in ug])
    return {"m": m, "path": path, "fingers": fingers, "ff": ff, "fit": f,
            "chain": chain, "R": R, "rc": rc, "rho": rho, "mu": mu,
            "mass": mass, "c": c, "ch": ch, "kin_deg": kin_deg,
            "u_hi": u_hi, "ug": ug, "ag": ag, "z_ball": z_ball,
            "ball_geom": m.geom("ball").id, "ball_body": m.body("ball").id,
            "finger_bodies": {b for b in range(m.nbody)
                              if m.body(b).name.startswith("s")}}


def tip_xy(B, d, fg):
    p = d.xpos[fg["distal"]] + d.xmat[fg["distal"]].reshape(3, 3) @ \
        [B["chain"].l, 0, 0]
    return p


def pose(B, d, u_sets):
    m = B["m"]
    d.qpos[:] = m.qpos0
    d.qvel[:] = 0
    for fg in B["fingers"]:
        a = np.interp(u_sets[fg["set"]], B["ug"], B["ag"])
        d.qpos[fg["qadr"]] = B["ff"]["r"] * a
        d.ctrl[fg["act"]] = a
    mj.mj_forward(m, d)


def geometry_check(B, hand):
    """Posed model against the kinematic script, level by level."""
    fgk = M.load_finger(hand["record"])
    ch = B["ch"]
    ev = M.evaluate(fgk, ch["level_i"], ch["level_j"], ch["side"], B["rho"])
    assert np.allclose(ev["c"], B["c"], atol=1e-9), "mount does not match"
    rb_tip = np.linalg.norm(fgk["B"][:, -1] - ev["c"], axis=1) - B["rho"]
    d = mj.MjData(B["m"])
    err_chain, err_rec, touching0 = 0.0, 0.0, 0
    for k, uk in enumerate(fgk["u"]):
        pose(B, d, [uk, uk])
        if k == 0:
            touching0 = sum(1 for c_ in d.contact[:d.ncon]
                            if B["ball_geom"] in (c_.geom1, c_.geom2)
                            and (B["m"].geom_bodyid[c_.geom1] in B["finger_bodies"]
                                 or B["m"].geom_bodyid[c_.geom2] in B["finger_bodies"]))
            ff_touch0 = sum(1 for c_ in d.contact[:d.ncon]
                            if B["m"].geom_bodyid[c_.geom1] in B["finger_bodies"]
                            and B["m"].geom_bodyid[c_.geom2] in B["finger_bodies"])
        for fg in B["fingers"]:
            p = tip_xy(B, d, fg)
            g = np.hypot(p[0], p[1]) - B["rho"]
            err_chain = max(err_chain, abs(g - rb_tip[k]))
            err_rec = max(err_rec, abs(g - ev["gap"][k]))
    # sense of sweep in the world, first finger
    pose(B, d, [fgk["u"][ch["level_i"]]] * 2)
    p0 = tip_xy(B, d, B["fingers"][0])
    pose(B, d, [fgk["u"][ch["level_j"]]] * 2)
    p1 = tip_xy(B, d, B["fingers"][0])
    sw = np.arctan2(p0[0] * p1[1] - p0[1] * p1[0], p0[0] * p1[0] + p0[1] * p1[1])
    return {"tip_gap_err_against_kinematic_chain_m": float(err_chain),
            "tip_gap_err_against_record_path_m": float(err_rec),
            "gap_before_contact_max_m": float(ev["gap"][:ch["level_i"] + 1].max()),
            "gap_deepest_m": float(ev["gap"].min()),
            "finger_ball_contacts_at_zero_input": int(touching0),
            "finger_finger_contacts_at_zero_input": int(ff_touch0),
            "sense_world": float(np.sign(sw)),
            "sense_kinematic": float(ev["sense"]),
            "u_levels": fgk["u"], "psi_deg": np.degrees(np.concatenate([
                np.zeros(ch["level_i"]), np.abs(ev["theta"] - ev["theta"][0]),
                np.full(len(fgk["u"]) - ch["level_j"] - 1,
                        abs(ev["theta"][-1] - ev["theta"][0]))])),
            "u_contact": [float(fgk["u"][ch["level_i"]]),
                          float(fgk["u"][ch["level_j"]])]}


def schedule(hand, B, ramp, cycles):
    """Knots (time, input of set 0, input of set 1)."""
    top = B["u_hi"]
    t, kn = 0.0, [(0.0, 0.0, 0.0)]

    def to(a, b, dur):
        nonlocal t
        t += dur
        kn.append((t, a, b))
    to(0, 0, HOLD)
    for _ in range(cycles):
        if hand["inputs"] == 1:
            to(top, top, ramp)
            to(top, top, HOLD)
            to(0, 0, ramp)
            to(0, 0, HOLD)
        else:
            to(top, 0, ramp)
            to(top, 0, HOLD)
            to(top, top, ramp)
            to(top, top, HOLD)
            to(0, 0, ramp)
            to(0, 0, HOLD)
    return np.array(kn)


def run(B, hand, kn, ball_contact=True, frames=False):
    m = B["m"]
    d = mj.MjData(m)
    save = (m.geom_contype[B["ball_geom"]], m.geom_conaffinity[B["ball_geom"]])
    if not ball_contact:
        m.geom_contype[B["ball_geom"]] = 0
        m.geom_conaffinity[B["ball_geom"]] = 0
    mj.mj_resetData(m, d)
    mj.mj_forward(m, d)
    nstep = int(round(kn[-1, 0] / DT))
    every = int(round(0.01 / DT))
    fevery = int(round(SPEED / FPS / DT))
    log, qf = [], []
    twist_prev, twist = 0.0, 0.0
    tip_prev = [None] * len(B["fingers"])
    tip_az = np.zeros(len(B["fingers"]))
    az_contact = np.zeros(len(B["fingers"]))
    unstable, why = False, ""
    pen_max, speed_max, path_err = 0.0, 0.0, 0.0
    dp = mj.MjData(m)
    for s in range(nstep + 1):
        t = s * DT
        u0 = np.interp(t, kn[:, 0], kn[:, 1])
        u1 = np.interp(t, kn[:, 0], kn[:, 2])
        for fg in B["fingers"]:
            d.ctrl[fg["act"]] = np.interp((u0, u1)[fg["set"]], B["ug"], B["ag"])
        mj.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)):
            unstable, why = True, "non-finite state"
            break
        # ball twist about the vertical, unwrapped
        q = d.qpos[3:7]
        tw = 2 * np.arctan2(q[3], q[0])
        dtw = (tw - twist_prev + np.pi) % (2 * np.pi) - np.pi
        twist += dtw
        twist_prev = tw
        touching = np.zeros(len(B["fingers"]), bool)
        pen = 0.0
        for c_ in d.contact[:d.ncon]:
            pen = max(pen, -c_.dist)
            if B["ball_geom"] in (c_.geom1, c_.geom2):
                other = c_.geom2 if c_.geom1 == B["ball_geom"] else c_.geom1
                nm = m.body(m.geom_bodyid[other]).name
                for i, fg in enumerate(B["fingers"]):
                    if nm.startswith(fg["label"] + "_"):
                        touching[i] = True
        pen_max = max(pen_max, pen)
        for i, fg in enumerate(B["fingers"]):
            p = tip_xy(B, d, fg)
            if tip_prev[i] is not None:
                dz = np.arctan2(tip_prev[i][0] * p[1] - tip_prev[i][1] * p[0],
                                tip_prev[i][0] * p[0] + tip_prev[i][1] * p[1])
                tip_az[i] += dz
                if touching[i]:
                    az_contact[i] += dz
            tip_prev[i] = p.copy()
        sp = float(np.linalg.norm(d.qvel[0:3]))
        speed_max = max(speed_max, sp)
        drift = float(np.hypot(d.qpos[0], d.qpos[1]))
        if int(d.warning.number.sum()) > 0:
            unstable, why = True, "MuJoCo warning"
        elif sp > 1.0:
            unstable, why = True, "ball speed over 1 m/s"
        elif pen > 0.5 * B["rc"]:
            unstable, why = True, "contact overlap over half the tip radius"
        elif drift > B["R"] or abs(d.qpos[2] - B["z_ball"]) > B["R"]:
            unstable, why = True, "ball centre moved more than its radius"
        if s % every == 0 or unstable:
            if not ball_contact:
                pose(B, dp, [u0, u1])
                for fg in B["fingers"]:
                    path_err = max(path_err, float(np.linalg.norm(
                        tip_xy(B, d, fg) - tip_xy(B, dp, fg))))
            tilt = np.arccos(np.clip(1 - 2 * (q[1] ** 2 + q[2] ** 2), -1, 1))
            log.append([t, u0, u1, np.degrees(twist), np.degrees(tilt),
                        drift, d.qpos[2] - B["z_ball"], int(touching.sum()),
                        pen, np.degrees(az_contact.mean())])
        if frames and s % fevery == 0:
            qf.append((t, u0, u1, np.degrees(twist), d.qpos.copy()))
        if unstable:
            break
    m.geom_contype[B["ball_geom"]], m.geom_conaffinity[B["ball_geom"]] = save
    return {"log": np.array(log), "frames": qf, "unstable": unstable,
            "why": why, "t_end": float(s * DT), "pen_max_m": float(pen_max),
            "ball_speed_max_m_s": float(speed_max),
            "tip_path_err_max_m": float(path_err),
            "warnings": int(d.warning.number.sum())}


def summarise(hand, B, kn, r):
    """Ball twist at the schedule's knots."""
    lg = r["log"]
    at = lambda t: float(np.interp(t, lg[:, 0], lg[:, 3]))   # noqa: E731
    out = {"twist_at_knots_deg": [[float(t), float(a), float(b), at(t)]
                                  for t, a, b in kn if t <= r["t_end"]],
           "twist_end_deg": float(lg[-1, 3]),
           "twist_max_abs_deg": float(np.abs(lg[:, 3]).max()),
           "tilt_max_deg": float(lg[:, 4].max()),
           "drift_max_m": float(lg[:, 5].max()),
           "lift_max_m": float(lg[:, 6].max()),
           "drop_max_m": float(-lg[:, 6].min()),
           "fingers_touching_max": int(lg[:, 7].max()),
           "tip_sweep_while_touching_deg": float(lg[-1, 9])}
    return out


def render(B, hand, rest, r, title):
    if not r["frames"]:
        return []
    from PIL import Image, ImageDraw, ImageFont
    m = B["m"]
    d = mj.MjData(m)
    ren = mj.Renderer(m, H, W)
    cam = mj.MjvCamera()
    cam.lookat[:] = [0, 0, 0.012]
    cam.distance, cam.azimuth, cam.elevation = 0.30, 90, -60
    try:
        font = ImageFont.truetype("arial.ttf", 24)
    except OSError:
        font = ImageFont.load_default()
    base = os.path.join(OUT, f"mujoco_{hand['key']}_{rest}_rest")
    ff = shutil.which("ffmpeg") or r"C:\Program Files\ffmpeg\bin\ffmpeg.exe"
    proc = subprocess.Popen(
        [ff, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt",
         "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-pix_fmt",
         "yuv420p", "-vcodec", "libx264", base + ".mp4"],
        stdin=subprocess.PIPE)
    best, still = -1.0, None
    for t, u0, u1, tw, qpos in r["frames"]:
        d.qpos[:] = qpos
        mj.mj_forward(m, d)
        ren.update_scene(d, cam)
        im = Image.fromarray(ren.render())
        dr = ImageDraw.Draw(im)
        dr.text((20, 14), title, fill=(0, 0, 0), font=font)
        inp = f"input {u0:.0f} Pa" if hand["inputs"] == 1 else \
            f"inputs {u0:.0f} and {u1:.0f} Pa"
        dr.text((20, 46), f"{inp} (simulation units)   ball turned "
                f"{tw:+.1f} deg   time {t:.1f} s", fill=(0, 0, 0), font=font)
        dr.text((20, H - 40), "SIMULATION ONLY. MuJoCo, fitted stand-in "
                "fingers. Comparative, not predictive.", fill=(0, 0, 0),
                font=font)
        proc.stdin.write(np.asarray(im).tobytes())
        if abs(tw) > best:
            best, still = abs(tw), im.copy()
    proc.stdin.close()
    proc.wait()
    still.save(base + ".png")
    ren.close()
    return [base + ".mp4", base + ".png"]


def main():
    task = json.load(open(M.TASK))
    lines, results = [], {}
    say = lines.append
    say("Pneu-net fingers on a palm turning a ball, MuJoCo with contact and "
        "friction. SIMULATION ONLY, comparative. A different model from the "
        "kinematic scripts: fitted hinge-chain fingers with joint springs, "
        "free ball.")
    say("script: tools/pneunet_palm_turn_ball_mujoco_20nov.py")
    say(f"mujoco {mj.__version__}; timestep {DT * 1e3:.1f} ms; one input ramp "
        f"{RAMP:.0f} s, hold {HOLD:.0f} s; joint limits off; damping critical "
        f"per joint; seat points {SEAT_RADIUS * 1e3:.0f} mm from the axis "
        "(assumed)")
    for hand in HANDS:
        say("")
        say(f"=== {hand['key']}: {hand['what']} ===")
        res = results.setdefault(hand["key"], {"what": hand["what"]})
        cycles = 2 if hand["inputs"] == 1 else 3
        first = True
        for rest in ("smooth", "seat"):
            B = build(hand, task, rest)
            kn = schedule(hand, B, RAMP, cycles)
            if first:
                g = geometry_check(B, hand)
                res["geometry_check"] = {k: v for k, v in g.items()
                                         if k not in ("u_levels", "psi_deg")}
                K = B["ff"]["K"]
                say(f"  ball radius {B['R'] * 1e3:.1f} mm, {B['mass'] * 1e3:.0f} g, "
                    f"friction {B['mu']}; tip radius {B['rc'] * 1e3:.2f} mm; "
                    f"joint springs {', '.join(f'{x:.4g}' for x in K)} "
                    "(slide N/m, hinges N m/rad)")
                say(f"  mount check, posed model against the kinematic "
                    f"script at every level: tip gap differs by at most "
                    f"{g['tip_gap_err_against_kinematic_chain_m'] * 1e6:.3f} um "
                    f"from the kinematic stand-in and "
                    f"{g['tip_gap_err_against_record_path_m'] * 1e3:.2f} mm from "
                    f"the record path (the fit error); kinematic gap "
                    f"{g['gap_before_contact_max_m'] * 1e3:+.2f} mm before "
                    f"contact, {g['gap_deepest_m'] * 1e3:+.2f} mm deepest; "
                    f"sweep sense {g['sense_world']:+.0f} (kinematic "
                    f"{g['sense_kinematic']:+.0f})")
                say(f"  at zero input: {g['finger_ball_contacts_at_zero_input']} "
                    f"finger-ball contacts, "
                    f"{g['finger_finger_contacts_at_zero_input']} "
                    "finger-finger contacts")
                if g["tip_gap_err_against_kinematic_chain_m"] > 1e-6:
                    say("  MOUNT CHECK FAILED: stopping this hand")
                    break
                r0 = run(B, hand, kn, ball_contact=False)
                res["ramp_check"] = {"tip_path_err_max_m": r0["tip_path_err_max_m"]}
                say(f"  ramp check, ball contact off: driven tip stays within "
                    f"{r0['tip_path_err_max_m'] * 1e3:.3f} mm of its posed path"
                    + (f"; UNSTABLE ({r0['why']})" if r0["unstable"] else ""))
                if hand["inputs"] == 1:
                    say(f"  kinematic script, slip ignored: "
                        f"{B['kin_deg']:.1f} deg in one stroke, 0.0 left after "
                        f"the return; contact from {g['u_contact'][0]:.0f} to "
                        f"{g['u_contact'][1]:.0f} Pa")
                else:
                    say(f"  kinematic script, slip ignored: net "
                        f"{B['kin_deg']:.1f} deg per cycle "
                        f"({cycles * B['kin_deg']:.1f} over {cycles} cycles); "
                        f"contact from {g['u_contact'][0]:.0f} to "
                        f"{g['u_contact'][1]:.0f} Pa")
            variants = [("", RAMP)] + ([("_half_speed", 2 * RAMP),
                                        ("_quarter_speed", 4 * RAMP)]
                                       if rest == "seat" else [])
            for tag, ramp in variants:
                knv = schedule(hand, B, ramp, cycles)
                if tag:
                    r0 = run(B, hand, knv, ball_contact=False)
                    res["ramp_check" + tag] = {
                        "tip_path_err_max_m": r0["tip_path_err_max_m"]}
                    say(f"  ramp check at {ramp:.0f} s per ramp, ball contact "
                        f"off: driven tip stays within "
                        f"{r0['tip_path_err_max_m'] * 1e3:.3f} mm of its "
                        "posed path")
                r = run(B, hand, knv, frames=(tag == ""))
                sm = summarise(hand, B, knv, r)
                name = f"{rest}{tag}"
                np.savetxt(os.path.join(OUT, f"log_{hand['key']}_{name}.csv"),
                           r["log"], delimiter=",", fmt="%.6g",
                           header="time_s,input_set0_Pa,input_set1_Pa,"
                           "ball_twist_deg,ball_tilt_deg,ball_drift_m,"
                           "ball_height_change_m,fingers_touching,"
                           "deepest_overlap_m,tip_sweep_while_touching_deg",
                           comments="")
                media = []
                if tag == "":
                    media = render(B, hand, rest, r,
                                   f"{hand['what']}; {rest} rest")
                res[name] = {**sm, **{k: v for k, v in r.items()
                                      if k not in ("log", "frames")},
                             "ramp_s": ramp, "media": [
                                 os.path.relpath(p, ROOT) for p in media],
                             "model": os.path.relpath(B["path"], ROOT)}
                say(f"  -- rest: {rest}{', ' + tag[1:].replace('_', ' ') + ' (' + format(ramp, '.0f') + ' s per ramp)' if tag else ''} --")
                if r["unstable"]:
                    say(f"     UNSTABLE at {r['t_end']:.2f} s: {r['why']}")
                else:
                    say("     stable to the end: no MuJoCo warning, deepest "
                        f"contact overlap {r['pen_max_m'] * 1e3:.3f} mm, ball "
                        f"speed at most {r['ball_speed_max_m_s'] * 1e3:.1f} mm/s")
                kk = sm["twist_at_knots_deg"]
                say("     ball turned (deg) at the end of each ramp or hold: "
                    + ", ".join(f"{x[3]:+.1f}" for x in kk[1:]))
                say(f"     largest turn {sm['twist_max_abs_deg']:.1f} deg; left "
                    f"at the end {sm['twist_end_deg']:+.1f} deg; tips swept "
                    f"{sm['tip_sweep_while_touching_deg']:+.1f} deg round the "
                    "axis while touching the ball (net)")
                say(f"     ball tilt at most {sm['tilt_max_deg']:.1f} deg, "
                    f"sideways {sm['drift_max_m'] * 1e3:.2f} mm, lifted "
                    f"{sm['lift_max_m'] * 1e3:.2f} mm, dropped "
                    f"{sm['drop_max_m'] * 1e3:.2f} mm; most fingers touching "
                    f"at once {sm['fingers_touching_max']}")
                for p in media:
                    say(f"     wrote {os.path.relpath(p, ROOT)}")
            first = False
    with open(os.path.join(OUT, "mujoco_results.json"), "w") as fh:
        json.dump(results, fh, indent=1, default=float)
    with open(os.path.join(OUT, "mujoco_report.txt"), "w",
              encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
