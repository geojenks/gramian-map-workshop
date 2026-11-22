"""Pneu-net fingers hanging from a plate, turning a ball: MuJoCo (20 Nov 2026).

SIMULATION ONLY. Comparative, not predictive.

The layout is the traditional soft gripper: three finger bases on one plate
above the ball, fingers hanging straight down round it, ball on a rest
below. Each finger is turned about its own long axis by a set angle, so its
bend is partly round the ball's vertical axis and not only towards it.

The finger is a ten-link stand-in (ten is the number of spine points the
record gives). Each link has a hinge in the bending plane, a hinge out of
that plane and a stretch along its length:

  free shape   every link bends by the same angle and stretches by the same
               amount (an even bend: ASSUMED, the record holds no bent body
               shape). The two numbers are solved at every input level so
               the tip is on the record's free path exactly.
  drive        the input moves the springs' rest values to that shape.
  springs      from the record's tip stiffness at zero input: in the
               bending plane, out of the plane and along the length.
               Stiffness in twist about the long axis is not in the record
               and is not modelled (rigid).
  mass         the records give none. Each finger weighs its record's
               material volume at an ASSUMED 1000 kg/m^3, shared equally
               between the links.
  contact      MuJoCo's soft contact with its time constant shortened from
               the default 20 ms to 5 ms (ASSUMED): with links this light
               the default lets the ball sink 3 to 4 mm into a finger.
               One capsule per link, radius from the record's material
               volume over its length. The ribbed blocks drawn are for the
               eye only.

Assumed layout numbers, stated in the report: the gap between a hanging
finger and the ball, the height of the tips, the angle each finger is
turned by, and the radius of the rest's three contact points.

Run from the repository root with a Python that has mujoco (needs ffmpeg):

    python tools/pneunet_gripper_turn_ball_mujoco_20nov.py

Writes outputs/pneunet_gripper_turn_ball_20nov/: gripper_report.txt,
gripper_results.json, one model .xml and one log .csv per run, and clips
(.mp4) with stills (.png).
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
from gramian_workshop.finger import load_record                # noqa: E402

OUT = os.path.join(ROOT, "outputs", "pneunet_gripper_turn_ball_20nov")
TASK = os.path.join(ROOT, "data", "tasks", "cup_turn.json")
N_LINKS = 10              # the record's rest shape has ten spine points
N_FINGERS = 3
CONTACT_TIME = 0.005      # assumed, s: MuJoCo contact time constant
DENSITY = 1000.0          # assumed, kg/m^3: the records give no mass
GAP = 0.002               # assumed: hanging finger to ball, at the equator
SEAT_RADIUS = 0.010       # assumed: rest's contact points, from the axis
SEAT_POINT = 0.002
DT = 0.0005
RAMP = 10.0
HOLD = 1.0
CYCLES = 2
FPS, SPEED = 30, 4
W, H = 1280, 720
RECORDS = ["wall_0.0020m", "wall_0.0015m"]
TURNS_DEG = [0.0, 30.0, -30.0, 45.0, 50.0, 55.0, 60.0, -60.0, 65.0, 70.0, 75.0]   # each finger about its own long axis
CLIP_TURN = 30.0          # the hand shown with no ball
CLIP_TURNS = [30.0, 60.0]
SLOW_RAMPS = [40.0]       # repeats of the clip cases, slower


def finger_table(rid):
    rec = load_record(f"data/records/{rid}.json")
    L = float(rec["metadata"]["length_m"])
    l = L / N_LINKS
    u = np.concatenate([[0.0], np.asarray(rec["free"]["u"], float).ravel()])
    d = np.vstack([[0, 0, 0], np.asarray(rec["free"]["tip_displacement_m"], float)])
    P = d[:, :2] + [L, 0.0]
    i = np.arange(1, N_LINKS + 1)
    ang = np.unwrap(np.arctan2(P[:, 1], P[:, 0]))
    th = 2 * ang / (N_LINKS + 1)
    mag = np.array([np.hypot(np.cos(i * t).sum(), np.sin(i * t).sum())
                    for t in th])
    e = np.hypot(P[:, 0], P[:, 1]) / mag - l
    st = rec["stiffness"]
    i0 = int(np.where(np.asarray(st["u"], float).ravel() == 0.0)[0][0])
    k_y = float(st["k_N_per_m"][i0])
    k_z = float(st["k_out_of_plane_N_per_m"][i0])
    k_x = float(st["k_axial_N_per_m"][i0])
    arms2 = float(((L - np.arange(N_LINKS) * l) ** 2).sum())
    rc, rc_src = BH.contact_radius(rec)
    link_mass = float(rec["metadata"]["material_volume_m3"]) * DENSITY / N_LINKS
    return {"id": rid, "L": L, "l": l, "u": u, "P": P, "th": th, "e": e,
            "k_rec": [k_x, k_y, k_z], "link_mass": link_mass, "k_bend": k_y * arms2,
            "k_out": k_z * arms2, "k_stretch": k_x * N_LINKS, "rc": rc,
            "rc_src": rc_src, "dz_max": float(np.abs(d[:, 2]).max()),
            "stroke": float(rec["scalars"]["free_stroke_m"])}


def build(ft, task, turn_deg, tip_z, key, with_ball=True):
    R = task["object"]["radius_m"]
    mass = task["object"]["mass_kg"]
    mu = task["mu"]
    rc, l = ft["rc"], ft["l"]
    Rb = R + rc + GAP
    z_base = tip_z + ft["L"]
    b = np.radians(turn_deg)
    bodies, excl, fingers = [], [], []
    vis = 'contype="0" conaffinity="0" mass="0"'
    for k in range(N_FINGERS):
        phi = 2 * np.pi * k / N_FINGERS
        er = np.array([np.cos(phi), np.sin(phi), 0.0])
        et = np.array([-np.sin(phi), np.cos(phi), 0.0])
        X = np.array([0.0, 0.0, -1.0])
        Y = -np.cos(b) * er + np.sin(b) * et
        pos = Rb * er + [0, 0, z_base]
        lines = [f'<body name="f{k}_mount" pos="{BH._vec(pos)}" '
                 f'xyaxes="{BH._vec(X)} {BH._vec(Y)}">',
                 f'<geom type="box" size="0.005 {1.15 * rc:.5f} {1.15 * rc:.5f}" '
                 f'pos="-0.005 0 0" rgba="0.3 0.4 0.6 1" {vis}/>']
        jb, jo, js = [], [], []
        for i in range(N_LINKS):
            lines.append(f'<body name="f{k}_l{i}" pos="{0 if i == 0 else l:.9g} 0 0">')
            for nm, kind, axis, kk, store in (
                    ("b", "hinge", "0 0 1", ft["k_bend"], jb),
                    ("o", "hinge", "0 1 0", ft["k_out"], jo),
                    ("s", "slide", "1 0 0", ft["k_stretch"], js)):
                store.append(f"f{k}_{nm}{i}")
                lines.append(f'<joint name="{store[-1]}" type="{kind}" '
                             f'axis="{axis}" stiffness="{kk:.9g}"/>')
            lines.append(f'<geom name="f{k}_c{i}" type="capsule" '
                         f'fromto="0 0 0 {l:.9g} 0 0" size="{rc:.9g}" mass="{ft['link_mass']:.9g}" group="3"/>')
            lines.append(f'<geom type="box" size="{0.36 * l:.6f} {0.78 * rc:.6f} '
                         f'{rc:.6f}" pos="{0.5 * l:.6f} {-0.1 * rc:.6f} 0" '
                         f'rgba="0.80 0.82 0.86 1" {vis}/>')
            lines.append(f'<geom type="box" size="{0.5 * l:.6f} {0.12 * rc:.6f} '
                         f'{rc:.6f}" pos="{0.5 * l:.6f} {0.85 * rc:.6f} 0" '
                         f'rgba="0.45 0.5 0.6 1" {vis}/>')
        lines += ["</body>"] * (N_LINKS + 1)
        bodies.append("\n".join(lines))
        excl += [f'<exclude body1="f{k}_l{a}" body2="f{k}_l{c}"/>'
                 for a in range(N_LINKS) for c in range(a + 1, N_LINKS)]
        fingers.append({"k": k, "jb": jb, "jo": jo, "js": js,
                        "X": X, "Y": Y, "Z": np.cross(X, Y)})
    rest_geoms, pairs, ball = "", "", ""
    if with_ball:
        dz = np.sqrt((R + SEAT_POINT) ** 2 - SEAT_RADIUS ** 2)
        for i in range(3):
            a = 2 * np.pi * i / 3 + 0.3
            rest_geoms += (
                f'<geom name="rest{i}" type="sphere" size="{SEAT_POINT}" '
                f'pos="{SEAT_RADIUS * np.cos(a):.6f} '
                f'{SEAT_RADIUS * np.sin(a):.6f} {R - dz:.6f}" '
                'rgba="0.4 0.4 0.45 1" contype="0" conaffinity="0"/>')
            pairs += (f'<pair geom1="ball" geom2="rest{i}" condim="3" '
                      f'friction="{mu} {mu} 0.005 0.0001 0.0001"/>')
        pairs += (f'<pair geom1="ball" geom2="plate" condim="3" '
                  f'friction="{mu} {mu} 0.005 0.0001 0.0001"/>')
        ball = (f'<body name="ball" pos="0 0 {R:.6f}"><freejoint name="ball"/>'
                f'<geom name="ball" type="sphere" size="{R}" mass="{mass}" '
                'material="ball"/></body>')
    xml = "\n".join([
        f'<mujoco model="pneunet_gripper_{key}">',
        "<!-- generated by tools/pneunet_gripper_turn_ball_mujoco_20nov.py."
        " SIMULATION ONLY. Joint damping and the springs' rest values are set"
        " after loading by that script. -->",
        '<compiler angle="radian"/>',
        f'<option gravity="0 0 -9.81" timestep="{DT}" integrator="implicitfast"/>',
        f'<visual><global offwidth="{W}" offheight="{H}"/>'
        '<headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6"/></visual>',
        '<default><joint armature="1e-5"/>'
        f'<geom friction="{mu} 0.005 0.0001" solref="{CONTACT_TIME} 1" rgba="0.2 0.5 0.8 1"/></default>',
        '<asset><texture type="skybox" builtin="flat" rgb1="1 1 1" '
        'rgb2="1 1 1" width="32" height="32"/>'
        '<texture name="ball" type="cube" builtin="checker" '
        'rgb1="0.95 0.8 0.3" rgb2="0.75 0.3 0.15" width="64" height="64"/>'
        '<material name="ball" texture="ball" texrepeat="3 3" rgba="1 1 1 1"/></asset>',
        "<worldbody>",
        f'<geom name="plate" type="cylinder" pos="0 0 {z_base + 0.012:.6f}" '
        f'size="{Rb + 0.02:.4f} 0.002" rgba="0.3 0.4 0.6 0.5" contype="0" '
        'conaffinity="0"/>',
        f'<geom name="table" type="cylinder" pos="0 0 {R - np.sqrt((R + SEAT_POINT) ** 2 - SEAT_RADIUS ** 2) - SEAT_POINT - 0.002:.6f}" '
        f'size="{Rb + 0.03:.4f} 0.002" rgba="0.85 0.83 0.78 1" contype="0" '
        'conaffinity="0"/>',
        rest_geoms, ball, *bodies,
        "</worldbody>",
        "<contact>" + pairs + "".join(excl) + "</contact>",
        "</mujoco>", ""])
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"model_{key}.xml")
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(xml)
    m = mj.MjModel.from_xml_path(path)
    d = mj.MjData(m)
    mj.mj_forward(m, d)
    Mfull = np.zeros((m.nv, m.nv))
    mj.mj_fullM(m, Mfull, d.qM)
    geom_finger = -np.ones(m.ngeom, int)
    for fg in fingers:
        for nm in ("jb", "jo", "js"):
            ids = [m.joint(j).id for j in fg[nm]]
            fg[nm + "_q"] = [int(m.jnt_qposadr[j]) for j in ids]
            fg[nm + "_v"] = [int(m.jnt_dofadr[j]) for j in ids]
            for j in ids:
                dof = int(m.jnt_dofadr[j])
                m.dof_damping[dof] = 2 * np.sqrt(m.jnt_stiffness[j]
                                                 * Mfull[dof, dof])
        fg["tip_body"] = m.body(f"f{fg['k']}_l{N_LINKS - 1}").id
        fg["mount_pos"] = m.body(f"f{fg['k']}_mount").pos.copy()
        for i in range(N_LINKS):
            geom_finger[m.geom(f"f{fg['k']}_c{i}").id] = fg["k"]
    B = {"m": m, "path": path, "fingers": fingers, "ft": ft, "R": R,
         "rc": rc, "mu": mu, "Rb": Rb, "z_base": z_base, "tip_z": tip_z,
         "turn_deg": turn_deg, "geom_finger": geom_finger,
         "with_ball": with_ball, "key": key, "u_hi": float(ft["u"][-1])}
    if with_ball:
        B["ball_geom"] = m.geom("ball").id
        B["ball_body"] = m.body("ball").id
        B["plate_geom"] = m.geom("plate").id
        B["ball_q"] = int(m.jnt_qposadr[m.joint("ball").id])
        B["ball_v"] = int(m.jnt_dofadr[m.joint("ball").id])
    return B


def set_input(B, m, u, d=None):
    """Move the springs' rest values to the free shape at input u."""
    ft = B["ft"]
    th = np.interp(u, ft["u"], ft["th"])
    e = np.interp(u, ft["u"], ft["e"])
    for fg in B["fingers"]:
        m.qpos_spring[fg["jb_q"]] = th
        m.qpos_spring[fg["js_q"]] = e
        if d is not None:
            d.qpos[fg["jb_q"]] = th
            d.qpos[fg["js_q"]] = e
            d.qpos[fg["jo_q"]] = 0.0


def tip_world(B, d, fg):
    return d.xpos[fg["tip_body"]] + d.xmat[fg["tip_body"]].reshape(3, 3) @ \
        [B["ft"]["l"], 0, 0]


def tip_in_finger(B, d, fg):
    p = tip_world(B, d, fg) - fg["mount_pos"]
    return np.array([p @ fg["X"], p @ fg["Y"], p @ fg["Z"]])


def static_checks(B):
    """Posed tip against the record's free path; tip stiffness of the built
    model against the record; contacts at zero input."""
    m, ft = B["m"], B["ft"]
    d = mj.MjData(m)
    err = 0.0
    for k, uk in enumerate(ft["u"]):
        d.qpos[:] = m.qpos0
        set_input(B, m, uk, d)
        mj.mj_forward(m, d)
        for fg in B["fingers"]:
            p = tip_in_finger(B, d, fg)
            err = max(err, float(np.hypot(p[0] - ft["P"][k, 0],
                                          p[1] - ft["P"][k, 1])), abs(p[2]))
        if k == 0:
            n0 = sum(1 for c in d.contact[:d.ncon]
                     if B["with_ball"] and B["ball_geom"] in (c.geom1, c.geom2)
                     and max(B["geom_finger"][c.geom1],
                             B["geom_finger"][c.geom2]) >= 0)
            fg = B["fingers"][0]
            jp = np.zeros((3, m.nv))
            mj.mj_jac(m, d, jp, None, tip_world(B, d, fg), fg["tip_body"])
            dofs = fg["jb_v"] + fg["jo_v"] + fg["js_v"]
            Kd = np.array([m.jnt_stiffness[m.dof_jntid[v]] for v in dofs])
            Jf = np.vstack([fg["X"], fg["Y"], fg["Z"]]) @ jp[:, dofs]
            C = (Jf / Kd) @ Jf.T
            k_model = [float(1 / C[a, a]) for a in range(3)]
    set_input(B, m, 0.0)
    return {"posed_tip_err_max_m": err, "finger_ball_contacts_at_zero": n0,
            "k_model_axial_inplane_outofplane_N_per_m": k_model,
            "k_record_axial_inplane_outofplane_N_per_m": ft["k_rec"]}


def schedule(top, ramp, cycles):
    t, kn = 0.0, [(0.0, 0.0)]
    for dur, a in [(HOLD, 0.0)] + [(ramp, top), (HOLD, top), (ramp, 0.0),
                                   (HOLD, 0.0)] * cycles:
        t += dur
        kn.append((t, a))
    return np.array(kn)


def run(B, kn, frames=False, gravity=True):
    m = B["m"]
    m.opt.gravity[2] = -9.81 if gravity else 0.0
    d = mj.MjData(m)
    dp = mj.MjData(m)
    set_input(B, m, 0.0)
    mj.mj_forward(m, d)
    nstep = int(round(kn[-1, 0] / DT))
    every = int(round(0.01 / DT))
    fevery = int(round(SPEED / FPS / DT))
    nf = len(B["fingers"])
    wb = B["with_ball"]
    log, qf, clog = [], [], []
    twist_prev, twist = 0.0, 0.0
    slide, travel = np.zeros(nf), np.zeros(nf)
    jp = np.zeros((3, m.nv))
    unstable, why = False, ""
    pen_max, path_err, plate_touch, fmax = 0.0, 0.0, False, 0.0
    f6 = np.zeros(6)
    for s in range(nstep + 1):
        t = s * DT
        u = float(np.interp(t, kn[:, 0], kn[:, 1]))
        set_input(B, m, u)
        mj.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)):
            unstable, why = True, "non-finite state"
            break
        touching = np.zeros(nf, bool)
        deep = np.full(nf, -1.0)
        vt_k, vb_k = np.zeros(nf), np.zeros(nf)
        cp, fk = np.full((nf, 3), np.nan), np.zeros(nf)
        pen, fn = 0.0, 0.0
        tw_deg = tilt = drift = lift = 0.0
        if wb:
            q = d.qpos[B["ball_q"] + 3:B["ball_q"] + 7]
            tw = 2 * np.arctan2(q[3], q[0])
            twist += (tw - twist_prev + np.pi) % (2 * np.pi) - np.pi
            twist_prev = tw
            for ci in range(d.ncon):
                c = d.contact[ci]
                pen = max(pen, -c.dist)
                if B["ball_geom"] not in (c.geom1, c.geom2):
                    continue
                other = c.geom2 if c.geom1 == B["ball_geom"] else c.geom1
                if other == B["plate_geom"]:
                    plate_touch = True
                k = B["geom_finger"][other]
                if k < 0:
                    continue
                touching[k] = True
                mj.mj_contactForce(m, d, ci, f6)
                fn += f6[0]
                fk[k] += f6[0]
                if -c.dist > deep[k]:
                    deep[k] = -c.dist
                    cp[k] = c.pos
                    n = np.array(c.frame[:3])
                    mj.mj_jac(m, d, jp, None, c.pos, B["ball_body"])
                    vb = jp @ d.qvel
                    mj.mj_jac(m, d, jp, None, c.pos, m.geom_bodyid[other])
                    vr = vb - jp @ d.qvel
                    vt_k[k] = np.linalg.norm(vr - (vr @ n) * n)
                    vb_k[k] = np.linalg.norm(vb - (vb @ n) * n)
            slide += vt_k * DT
            travel += vb_k * DT
            pen_max, fmax = max(pen_max, pen), max(fmax, fn)
            zax = 1 - 2 * (q[1] ** 2 + q[2] ** 2)
            tilt = float(np.degrees(np.arccos(np.clip(zax, -1, 1))))
            bp = d.qpos[B["ball_q"]:B["ball_q"] + 3]
            drift, lift = float(np.hypot(bp[0], bp[1])), float(bp[2] - B["R"])
            tw_deg = float(np.degrees(twist))
            sp = float(np.linalg.norm(d.qvel[B["ball_v"]:B["ball_v"] + 3]))
            if sp > 1.0:
                unstable, why = True, "ball speed over 1 m/s"
            elif pen > 0.5 * B["rc"]:
                unstable, why = True, "contact overlap over half the finger radius"
            elif drift > B["R"] or abs(lift) > 2 * B["R"]:
                unstable, why = True, "ball left the hand"
        if int(d.warning.number.sum()) > 0:
            unstable, why = True, "MuJoCo warning"
        if s % every == 0 or unstable:
            dp.qpos[:] = m.qpos0
            set_input(B, m, u, dp)
            mj.mj_kinematics(m, dp)
            te = max(float(np.linalg.norm(tip_world(B, d, fg)
                                          - tip_world(B, dp, fg)))
                     for fg in B["fingers"])
            path_err = max(path_err, te)
            log.append([t, u, tw_deg, tilt, drift, lift, int(touching.sum()),
                        pen, fn, 1e3 * slide.mean(), 1e3 * travel.mean(), te])
            if wb:
                Rm = d.xmat[B["ball_body"]].reshape(3, 3)
                row = [t, u, tw_deg]
                for k in range(nf):
                    row += [*cp[k], *(Rm.T @ (cp[k] - d.xpos[B["ball_body"]])),
                            fk[k]]
                clog.append(row)
        if frames and s % fevery == 0:
            qf.append((t, u, tw_deg, d.qpos.copy()))
        if unstable:
            break
    m.opt.gravity[2] = -9.81
    return {"log": np.array(log), "contacts": np.array(clog), "frames": qf, "unstable": unstable,
            "why": why, "pen_max": pen_max, "tip_off_posed_max_m": path_err,
            "plate_touch": plate_touch, "normal_force_max_N": fmax,
            "t_end": t}


def at(lg, t, col):
    return float(lg[np.argmin(np.abs(lg[:, 0] - t)), col])


def summarise(kn, r):
    lg = r["log"]
    ends = [at(lg, t, 2) for t in kn[:, 0] if t <= r["t_end"] + 1e-9]
    out = {"unstable": bool(r["unstable"]), "why": r["why"],
           "ball_turn_deg_at_knots": ends,
           "deepest_overlap_m": r["pen_max"],
           "ball_tilt_max_deg": float(lg[:, 3].max()),
           "ball_drift_max_m": float(lg[:, 4].max()),
           "ball_lift_max_m": float(lg[:, 5].max()),
           "ball_drop_max_m": float(-lg[:, 5].min()),
           "fingers_touching_max": int(lg[:, 6].max()),
           "normal_force_total_max_N": r["normal_force_max_N"],
           "ball_touched_plate": bool(r["plate_touch"])}
    if len(kn) > 2 and r["t_end"] >= kn[2, 0]:
        t0, t1 = kn[1, 0], kn[2, 0]
        out["first_stroke"] = {
            "ball_turn_deg": at(lg, t1, 2) - at(lg, t0, 2),
            "sliding_at_contact_mm": at(lg, t1, 9) - at(lg, t0, 9),
            "ball_surface_travel_at_contact_mm": at(lg, t1, 10) - at(lg, t0, 10)}
    return out


def render(B, frames, title, base, show_turn=True):
    from PIL import Image, ImageDraw, ImageFont
    m = B["m"]
    d = mj.MjData(m)
    ren = mj.Renderer(m, H, W)
    cam = mj.MjvCamera()
    cam.lookat[:] = [0, 0, 0.5 * B["z_base"]]
    cam.distance, cam.azimuth, cam.elevation = 0.30, 20, -22
    try:
        font = ImageFont.truetype("arial.ttf", 24)
    except OSError:
        font = ImageFont.load_default()
    ff = shutil.which("ffmpeg") or r"C:\Program Files\ffmpeg\bin\ffmpeg.exe"
    proc = subprocess.Popen(
        [ff, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt",
         "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-pix_fmt",
         "yuv420p", "-vcodec", "libx264", base + ".mp4"],
        stdin=subprocess.PIPE)
    best, still = -1.0, None
    for t, u, tw, qpos in frames:
        d.qpos[:] = qpos
        mj.mj_forward(m, d)
        ren.update_scene(d, cam)
        im = Image.fromarray(ren.render())
        dr = ImageDraw.Draw(im)
        dr.text((20, 14), title, fill=(0, 0, 0), font=font)
        line = f"input {u:.0f} Pa (simulation units)   time {t:.1f} s"
        if show_turn:
            line += f"   ball turned {tw:+.1f} deg"
        dr.text((20, 46), line, fill=(0, 0, 0), font=font)
        dr.text((20, H - 40), "SIMULATION ONLY. MuJoCo, ten-link stand-in "
                "fingers, even bend assumed. Comparative, not predictive.",
                fill=(0, 0, 0), font=font)
        proc.stdin.write(np.asarray(im).tobytes())
        score = abs(tw) if show_turn else u
        if score > best:
            best, still = score, im.copy()
    proc.stdin.close()
    proc.wait()
    still.save(base + ".png")
    ren.close()
    return [base + ".mp4", base + ".png"]


LOG_HEADER = ("time_s,input_Pa,ball_twist_deg,ball_tilt_deg,ball_drift_m,"
              "ball_height_change_m,fingers_touching,deepest_overlap_m,"
              "normal_force_total_N,sliding_at_contact_mm_cumulative,"
              "ball_surface_travel_at_contact_mm_cumulative,"
              "tip_distance_from_free_shape_m")


CONTACT_HEADER = "time_s,input_Pa,ball_twist_deg," + ",".join(
    f"finger{k}_{c}" for k in range(N_FINGERS) for c in (
        "x_world_m", "y_world_m", "z_world_m", "x_on_ball_m", "y_on_ball_m",
        "z_on_ball_m", "normal_force_N"))
# one row per 10 ms; the point is the finger's deepest contact with the
# ball (empty while not touching), in the world frame (origin under the
# ball on its lowest point's level, z up) and in the ball's own frame.


def main():
    task = json.load(open(TASK))
    R = task["object"]["radius_m"]
    alt = abs(np.radians(task["contact_ring"]["altitude_deg"]))
    rep = ["Pneu-net fingers hanging from a plate, turning a ball. MuJoCo "
           "with contact and friction. SIMULATION ONLY, comparative.",
           "script: tools/pneunet_gripper_turn_ball_mujoco_20nov.py",
           f"mujoco {mj.__version__}; timestep {DT * 1e3:g} ms; one input "
           f"ramp {RAMP:g} s, hold {HOLD:g} s, {CYCLES} out-and-back cycles",
           f"{N_FINGERS} fingers, one shared input; {N_LINKS} links each, "
           "even bend and stretch along the finger ASSUMED",
           f"assumed layout: gap {GAP * 1e3:g} mm between a hanging finger "
           f"and the ball; rest points {SEAT_RADIUS * 1e3:g} mm from the axis; "
           f"contact time constant {CONTACT_TIME * 1e3:g} ms (MuJoCo default 20 ms)",
           "angle: each finger turned about its own long axis; 0 bends "
           "straight at the ball's axis; positive is anticlockwise from above",
           ""]
    results = {}
    files = []
    for rid in RECORDS:
        ft = finger_table(rid)
        heights = [("tips_low", "tips below the "
                    "ball's centre", R - (R + ft["rc"]) * np.sin(alt)),
                   ("tips_level", "tips level with the ball's centre", R)]
        rep += [f"=== {rid} ===",
                f"  length {ft['L'] * 1e3:.1f} mm, contact radius "
                f"{ft['rc'] * 1e3:.2f} mm ({ft['rc_src']}); free stroke "
                f"{ft['stroke'] * 1e3:.1f} mm",
                f"  free shape at full input: each link bends "
                f"{np.degrees(ft['th'][-1]):.1f} deg (total "
                f"{np.degrees(ft['th'][-1]) * N_LINKS:.0f} deg) and stretches "
                f"{ft['e'][-1] * 1e3:.2f} mm (most {ft['e'].max() * 1e3:.2f} mm "
                f"of {ft['l'] * 1e3:.2f} mm)",
                f"  springs: bend {ft['k_bend']:.4g}, out of plane "
                f"{ft['k_out']:.4g} N m/rad, stretch {ft['k_stretch']:.4g} N/m",
                f"  finger mass {ft['link_mass'] * N_LINKS * 1e3:.2f} g (record's "
                f"material volume at an assumed {DENSITY:g} kg/m^3)"]
        results[rid] = {}
        kn = schedule(float(ft["u"][-1]), RAMP, CYCLES)
        # the hand with no ball: free motion, for the eye and the ramp check
        key = f"{rid}_free_turned_{CLIP_TURN:+.0f}deg"
        B = build(ft, task, CLIP_TURN, heights[0][2], key, with_ball=False)
        sc = static_checks(B)
        km, kr = sc["k_model_axial_inplane_outofplane_N_per_m"], ft["k_rec"]
        rep += [f"  posed model against the record's free path, every level: "
                f"tip off by at most {sc['posed_tip_err_max_m'] * 1e6:.3f} um "
                f"(record's own out-of-plane drift, not modelled: "
                f"{ft['dz_max'] * 1e3:.3f} mm)",
                "  tip stiffness of the built model at zero input, N/m "
                f"(record): along {km[0]:.1f} ({kr[0]:.1f}), in the bending "
                f"plane {km[1]:.2f} ({kr[1]:.2f}), out of plane {km[2]:.1f} "
                f"({kr[2]:.1f})"]
        free = {}
        for ramp in [RAMP] + SLOW_RAMPS:
            knr = schedule(float(ft["u"][-1]), ramp, 1)
            r0 = run(B, knr, gravity=False)
            r1 = run(B, knr, frames=(ramp == RAMP), gravity=True)
            sag0 = at(r1["log"], knr[1, 0], 11)
            sag1 = at(r1["log"], knr[3, 0], 11)
            free[f"{ramp:g}"] = {
                "no_gravity_tip_off_free_shape_max_m": r0["tip_off_posed_max_m"],
                "gravity_tip_off_free_shape_max_m": r1["tip_off_posed_max_m"],
                "sag_at_zero_input_m": sag0, "sag_at_full_input_m": sag1,
                "unstable": bool(r0["unstable"] or r1["unstable"])}
            rep.append(
                f"  no ball, {ramp:g} s ramp: tip stays within "
                f"{r0['tip_off_posed_max_m'] * 1e3:.3f} mm of the free shape "
                f"with gravity off, {r1['tip_off_posed_max_m'] * 1e3:.3f} mm "
                f"with gravity on (sag {sag0 * 1e3:.3f} mm at zero input, "
                f"{sag1 * 1e3:.3f} mm held at full input, "
                f"{100 * sag1 / ft['stroke']:.1f}% of the free stroke; the "
                "repository's tolerance for a free path is 5%)"
                + ("  UNSTABLE" if free[f"{ramp:g}"]["unstable"] else ""))
            if ramp == RAMP:
                fs = render(B, r1["frames"], f"three {rid.split('_')[1]} wall "
                            f"fingers, no ball, each turned {CLIP_TURN:+.0f} deg "
                            "about its own axis", os.path.join(
                                OUT, f"gripper_{key}"), show_turn=False)
                files += fs
                rep += [f"     wrote {f}" for f in fs]
        results[rid]["checks"] = dict(sc, free_ramps=free)
        for hkey, hwhat, tip_z in heights:
            rep.append(f"  -- {hwhat} (tip centre {tip_z * 1e3:.1f} mm above "
                       f"the ball's lowest point; bases {(tip_z + ft['L']) * 1e3:.1f} mm) --")
            results[rid][hkey] = {}
            for turn in TURNS_DEG:
                key = f"{rid}_{hkey}_turned_{turn:+.0f}deg"
                B = build(ft, task, turn, tip_z, key)
                sc = static_checks(B)
                ramps = [RAMP] + (SLOW_RAMPS if turn in CLIP_TURNS else [])
                for ramp in ramps:
                    knr = schedule(float(ft["u"][-1]), ramp, CYCLES)
                    clip = ramp == RAMP and turn in CLIP_TURNS
                    r = run(B, knr, frames=clip)
                    sm = summarise(knr, r)
                    sm["finger_ball_contacts_at_zero"] = sc["finger_ball_contacts_at_zero"]
                    tag = key + ("" if ramp == RAMP else f"_ramp_{ramp:g}s")
                    results[rid][hkey][f"{turn:+.0f}deg_ramp_{ramp:g}s"] = sm
                    np.savetxt(os.path.join(OUT, f"log_{tag}.csv"), r["log"],
                               delimiter=",", fmt="%.6g", header=LOG_HEADER,
                               comments="")
                    np.savetxt(os.path.join(OUT, f"contacts_{tag}.csv"),
                               r["contacts"], delimiter=",", fmt="%.6g",
                               header=CONTACT_HEADER, comments="")
                    e = sm["ball_turn_deg_at_knots"]
                    fsx = sm.get("first_stroke", {})
                    rep += [
                        f"     turned {turn:+.0f} deg, {ramp:g} s ramp: "
                        + (f"UNSTABLE ({sm['why']}) at {r['t_end']:.2f} s"
                           if sm["unstable"] else "stable to the end")
                        + f"; contacts at zero input {sc['finger_ball_contacts_at_zero']}",
                        "        ball turned (deg) at the end of each ramp or "
                        "hold: " + ", ".join(f"{x:+.1f}" for x in e),
                        f"        first stroke: ball {fsx.get('ball_turn_deg', float('nan')):+.1f} deg; "
                        f"sliding at the contacts {fsx.get('sliding_at_contact_mm', float('nan')):.1f} mm "
                        f"while the ball's surface there moved "
                        f"{fsx.get('ball_surface_travel_at_contact_mm', float('nan')):.1f} mm (mean over fingers)",
                        f"        ball lifted {sm['ball_lift_max_m'] * 1e3:.2f} mm, "
                        f"dropped {sm['ball_drop_max_m'] * 1e3:.2f} mm, tilt at most "
                        f"{sm['ball_tilt_max_deg']:.1f} deg, sideways "
                        f"{sm['ball_drift_max_m'] * 1e3:.2f} mm; most fingers "
                        f"touching {sm['fingers_touching_max']}; deepest overlap "
                        f"{sm['deepest_overlap_m'] * 1e3:.3f} mm; total normal "
                        f"force at most {sm['normal_force_total_max_N']:.3f} N; "
                        f"ball touched the plate: {'yes' if sm['ball_touched_plate'] else 'no'}"]
                    if clip:
                        fs = render(B, r["frames"],
                                    f"three {rid.split('_')[1]} wall fingers, "
                                    f"each turned {turn:+.0f} deg about its own "
                                    f"axis; {hwhat}",
                                    os.path.join(OUT, f"gripper_{key}"))
                        files += fs
                        rep += [f"        wrote {f}" for f in fs]
                    print("\n".join(rep[-4:]), flush=True)
        rep.append("")
        with open(os.path.join(OUT, "gripper_report.txt"), "w",
                  encoding="utf-8") as fh:
            fh.write("\n".join(rep) + "\n")
        with open(os.path.join(OUT, "gripper_results.json"), "w",
                  encoding="utf-8") as fh:
            json.dump(results, fh, indent=1)
    print("\n".join(rep))


if __name__ == "__main__":
    main()
