"""One stroke of a recorded learned rotation, replicated in part by an
arrangement of pneu-net fingers: MuJoCo (20 Nov 2026).

SIMULATION ONLY. Comparative, not predictive.

The recording is the learned policy on a four-finger hand turning a cylinder
(data/rl_logs/hora_seed42_30s*.npz). This script

  1. finds sections of the recording in which the same three fingertips
     touch throughout, the cylinder stays near upright and its centre
     moves little;
  2. for each section and each touching fingertip, places one pneu-net
     finger (one input) so that its tip follows the logged contact path:
     the finger bends in the plane square to the cylinder's axis, at the
     height of the logged contact; its base position, its direction, which
     way it bends and the first and last input levels are fitted (the input
     rises or falls evenly between them). No simulation is used in the fit;
  3. keeps the section and finger record with the smallest worst-finger
     misfit, builds that arrangement round the logged cylinder at its logged
     pose, and simulates the stroke;
  4. measures both the recording and the simulated arrangement the same way
     and writes a clip: the recorded section, then the arrangement.

The finger is the ten-link stand-in of
tools/pneunet_gripper_turn_ball_mujoco_20nov.py (springs from the record's
tip stiffness, tip on the record's free path, mass ASSUMED).

ASSUMED, and stated in the report: the squeeze each pneu-net is placed to
give (twice what friction needs to carry the cylinder's weight, shared
equally); friction between finger and cylinder (the log's value for the
cylinder); the duration of the simulated stroke; the thresholds that pick
the sections.

Run from the repository root with a Python that has mujoco (needs ffmpeg):

    python tools/pneunet_arrangement_replicates_rl_stroke_20nov.py --wave-manip <Wave_manip folder>

Writes outputs/pneunet_arrangement_replicates_rl_stroke_20nov/.
"""
import argparse
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
import pneunet_gripper_turn_ball_mujoco_20nov as G             # noqa: E402

OUT = os.path.join(ROOT, "outputs", "pneunet_arrangement_replicates_rl_stroke_20nov")
LOGS = os.path.join(ROOT, "data", "rl_logs")
N = G.N_LINKS
DT = G.DT
RECORDS = ["wall_0.0020m", "wall_0.0015m", "wall_0.0010m"]
TOUCH_N = 0.02            # a fingertip counts as touching above this normal force, N
MIN_SECTION_S = 0.3       # shortest section considered
MAX_TILT_DEG = 15.0       # mean tilt of the cylinder's axis from vertical
MAX_TILT_RANGE_DEG = 5.0
MAX_DRIFT_M = 0.006       # movement of the cylinder's centre over the section
N_SECTIONS = 6            # sections fitted, largest turn first
SQUEEZE_FACTOR = 2.0      # assumed: squeeze = this x what friction needs for the weight
M_FIT = 9                 # points of each contact path used in the fit
RETRACT = 0.030           # fingers start this far back along the contact normal
T_IN0, T_IN1, T_SETTLE, T_FREE, T_STROKE, T_HOLD = 0.5, 2.5, 4.0, 6.0, 10.0, 12.0
IMPRATIO = 10             # with the elliptic friction cone: less slow slip at a held contact
DROP_STOP = 0.03
FPS, W, H = 30, 1280, 720
RL_SLOW = 5.0             # the recorded section is shown this many times slower
COL = {"index": (0.85, 0.2, 0.2), "middle": (0.1, 0.6, 0.2),
       "ring": (0.15, 0.35, 0.9), "thumb": (0.85, 0.55, 0.0)}


# ---------- quaternions, x y z w ----------
def rot(q, v):
    return v + 2 * np.cross(q[..., :3], np.cross(q[..., :3], v) + q[..., 3:4] * v)


def inv(q):
    return np.concatenate([-q[..., :3], q[..., 3:4]], -1)


def qmul(a, b):
    av, aw, bv, bw = a[..., :3], a[..., 3:], b[..., :3], b[..., 3:]
    return np.concatenate([aw * bv + bw * av + np.cross(av, bv),
                           aw * bw - (av * bv).sum(-1, keepdims=True)], -1)


def own_axis_turn(q):
    """Turn about the object's own axis, summed from step to step, rad."""
    dq = qmul(inv(q[:-1]), q[1:])
    dq = dq * np.sign(dq[:, 3:])
    return np.concatenate([[0.0], np.cumsum(2 * dq[:, 2])])


# ---------- the recording ----------
def load_recording():
    F = np.load(os.path.join(LOGS, "hora_seed42_30s_forces.npz"), allow_pickle=True)
    T = np.load(os.path.join(LOGS, "hora_seed42_30s.npz"), allow_pickle=True)
    Z = np.load(os.path.join(LOGS, "hora_seed42_30s_physics.npz"), allow_pickle=True)
    scale = float(T["obj_scale"])
    q = F["obj_quat"].astype(float)
    ax = rot(q, np.array([0, 0, 1.0]))
    return {"t": F["t"].astype(float), "lam": F["normal_N"].astype(float),
            "P": F["point"].astype(float), "nrm": F["normal_dir"].astype(float),
            "op": F["obj_pos"].astype(float), "q": q, "axis": ax,
            "tilt": np.degrees(np.arccos(np.clip(np.abs(ax[:, 2]), 0, 1))),
            "turn": own_axis_turn(q), "names": [str(x) for x in F["fingers"]],
            "scale": scale, "mass": float(Z["obj_mass"]), "mu": float(np.ravel(Z["obj_fric"])[0]), "com": np.asarray(Z["obj_com"], float),
            "I": np.asarray(Z["obj_I"], float), "T": T, "Z": Z}


def sections(L, R_urdf):
    t, touch = L["t"], L["lam"] > TOUCH_N
    key = (touch * [1, 2, 4, 8]).sum(1)
    out, i = [], 0
    while i < len(t):
        s = i
        while i < len(t) and key[i] == key[s]:
            i += 1
        e = i - 1
        if t[e] - t[s] < MIN_SECTION_S or touch[s].sum() < 3 or s == 0 or e == len(t) - 1:
            continue
        sec = {"s": s, "e": e, "t0": float(t[s]), "t1": float(t[e]),
               "fingers": [j for j in range(4) if touch[s, j]],
               "turn_deg": float(np.degrees(L["turn"][e] - L["turn"][s])),
               "tilt_mean_deg": float(L["tilt"][s:e + 1].mean()),
               "tilt_range_deg": float(np.ptp(L["tilt"][s:e + 1])),
               "drift_m": float(np.linalg.norm(L["op"][s:e + 1] - L["op"][s], axis=1).max())}
        if (sec["tilt_mean_deg"] < MAX_TILT_DEG and sec["tilt_range_deg"] < MAX_TILT_RANGE_DEG
                and sec["drift_m"] < MAX_DRIFT_M):
            out.append(sec)
    out.sort(key=lambda x: -abs(x["turn_deg"]))
    return out


def section_frame(L, sec):
    s, e = sec["s"], sec["e"]
    a = L["axis"][s:e + 1].mean(0)
    a /= np.linalg.norm(a)
    e1 = np.array([1.0, 0, 0]) - a[0] * a
    e1 /= np.linalg.norm(e1)
    return {"c": L["op"][s:e + 1].mean(0), "a": a, "e1": e1, "e2": np.cross(a, e1)}


# ---------- the finger in its bending plane ----------
def chain_tables(ft, ng=121):
    """Joint positions, tip and tip compliance of the ten-link finger's free
    shape, on an even grid of input levels."""
    ug = np.linspace(0, ft["u"][-1], ng)
    th = np.interp(ug, ft["u"], ft["th"])
    e = np.interp(ug, ft["u"], ft["e"])
    ang = th[:, None] * np.arange(1, N + 1)
    d = np.stack([np.cos(ang), np.sin(ang)], -1)
    J = np.concatenate([np.zeros((ng, 1, 2)),
                        np.cumsum((ft["l"] + e)[:, None, None] * d, 1)], 1)
    C = np.zeros((ng, 2, 2))
    for n in range(N):
        r = J[:, -1] - J[:, n]
        jb = np.stack([-r[:, 1], r[:, 0]], -1)
        C += jb[:, :, None] * jb[:, None, :] / ft["k_bend"]
        C += d[:, n, :, None] * d[:, n, None, :] / ft["k_stretch"]
    return J, C


def fit_parts(x, tab, Nk, mirror, squeeze, lin):
    J, C = tab
    ng = J.shape[0] - 1
    b, psi = x[:2] * 0.05, x[2]
    uf = x[3] + (x[4] - x[3]) * lin
    f = np.clip(uf, 0, 1) * ng
    i0 = np.minimum(f.astype(int), ng - 1)
    w = (f - i0)[:, None, None]
    Jm = J[i0] * (1 - w) + J[i0 + 1] * w
    Cm = C[i0] * (1 - w) + C[i0 + 1] * w
    Rm = np.array([[np.cos(psi), -np.sin(psi)], [np.sin(psi), np.cos(psi)]])
    Sm = np.array([1.0, mirror])
    fl = ((-squeeze * Nk) @ Rm) * Sm
    tip = Jm[:, -1] + np.einsum("mij,mj->mi", Cm, fl)
    pred = b + (tip * Sm) @ Rm.T
    joints = b + (Jm[[0, -1], :N - 1] * Sm) @ Rm.T
    return pred, joints, uf


def nelder_mead(f, x0, step, iters):
    n = len(x0)
    X = np.vstack([x0] + [x0 + step * np.eye(n)[i] for i in range(n)])
    Fv = np.array([f(x) for x in X])
    for _ in range(iters):
        o = np.argsort(Fv)
        X, Fv = X[o], Fv[o]
        xo = X[:-1].mean(0)
        xr = xo + (xo - X[-1])
        fr = f(xr)
        if fr < Fv[0]:
            xe = xo + 2 * (xo - X[-1])
            fe = f(xe)
            X[-1], Fv[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < Fv[-2]:
            X[-1], Fv[-1] = xr, fr
        else:
            xc = xo + 0.5 * (X[-1] - xo)
            fc = f(xc)
            if fc < Fv[-1]:
                X[-1], Fv[-1] = xc, fc
            else:
                X[1:] = X[0] + 0.5 * (X[1:] - X[0])
                Fv[1:] = [f(x) for x in X[1:]]
    o = np.argmin(Fv)
    return X[o], Fv[o]


def fit_finger(L, sec, fr, j, ft, tab, R, squeeze):
    """Place one finger so its loaded tip follows fingertip j's contact path."""
    s, e = sec["s"], sec["e"]
    ks = np.round(np.linspace(s, e, M_FIT)).astype(int)
    n3 = L["nrm"][ks, j]                       # points into the cylinder
    g3 = L["P"][ks, j] - ft["rc"] * n3 - fr["c"]   # where the tip's centre must be
    h = float((g3 @ fr["a"]).mean())
    Gk = np.stack([g3 @ fr["e1"], g3 @ fr["e2"]], -1)
    Nk = np.stack([n3 @ fr["e1"], n3 @ fr["e2"]], -1)
    lin = np.linspace(0, 1, M_FIT)
    clear = R + ft["rc"]
    best = None
    for mirror in (1.0, -1.0):
        def cost(x):
            pred, joints, uf = fit_parts(x, tab, Nk, mirror, squeeze, lin)
            r = pred - Gk
            pen = np.maximum(0, clear - np.linalg.norm(joints, axis=-1))
            out = np.maximum(0, np.abs(uf - 0.5) - 0.5)
            return (r * r).sum(1).mean() + 10 * (pen * pen).mean() + (out * out).sum()
        for psi in np.arange(8) * np.pi / 4:
            for ua, ub in ((0.3, 0.6), (0.6, 0.3)):
                x0 = np.array([0, 0, psi, ua, ub])
                pred, _, _ = fit_parts(x0, tab, Nk, mirror, squeeze, lin)
                x0[:2] = (Gk.mean(0) - pred.mean(0)) / 0.05
                x, fv = nelder_mead(cost, x0, 0.1, 350)
                if best is None or fv < best[1]:
                    best = (x, fv, mirror)
    x, fv, mirror = best
    pred, joints, uf = fit_parts(x, tab, Nk, mirror, squeeze, lin)
    r = np.linalg.norm(pred - Gk, axis=1)
    pen = np.maximum(0, clear - np.linalg.norm(joints, axis=-1))
    # tip stiffness along the contact normal and along the cylinder's surface, at the first input level
    Jt, Ct = tab
    C0 = Ct[int(round(np.clip(x[3], 0, 1) * (Jt.shape[0] - 1)))]
    Rm = np.array([[np.cos(x[2]), -np.sin(x[2])], [np.sin(x[2]), np.cos(x[2])]])
    A = Rm * np.array([1.0, mirror])
    Cw = A @ C0 @ A.T
    nn = Nk[0] / np.linalg.norm(Nk[0])
    tt = np.array([-nn[1], nn[0]])
    k_n, k_t = 1 / (nn @ Cw @ nn), 1 / (tt @ Cw @ tt)
    return {"finger": L["names"][j], "j": j, "k_normal_N_per_m": float(k_n),
            "k_along_surface_N_per_m": float(k_t),
            "friction_can_hold_back_tip_m": float(L["mu"] * squeeze / k_t), "b": (x[:2] * 0.05).tolist(), "psi": float(x[2]),
            "mirror": mirror, "u0": float(np.clip(x[3], 0, 1) * ft["u"][-1]),
            "u1": float(np.clip(x[4], 0, 1) * ft["u"][-1]), "h": h,
            "misfit_rms_m": float(np.sqrt((r * r).mean())), "misfit_max_m": float(r.max()),
            "off_plane_rms_m": float(np.sqrt((((g3 @ fr["a"]) - h) ** 2).mean())),
            "path_length_m": float(np.linalg.norm(np.diff(Gk, axis=0), axis=1).sum()),
            "body_inside_cylinder_max_m": float(pen.max()),
            "normal_mean": (-n3.mean(0) / np.linalg.norm(n3.mean(0))).tolist()}


def fingers_clash(fits, ft, tab, fr):
    """Smallest gap between the bodies of different fingers, free shapes at
    the first and last input level (centre lines, less two contact radii)."""
    J, _ = tab
    ng = J.shape[0] - 1
    pts = []
    for f in fits:
        Rm = np.array([[np.cos(f["psi"]), -np.sin(f["psi"])], [np.sin(f["psi"]), np.cos(f["psi"])]])
        p = []
        for u in (f["u0"], f["u1"]):
            Jm = J[int(round(u / ft["u"][-1] * ng))]
            xy = np.array(f["b"]) + (Jm * [1.0, f["mirror"]]) @ Rm.T
            p.append(np.column_stack([xy, np.full(len(xy), f["h"])]))
        pts.append(np.vstack(p))
    gap = np.inf
    for a in range(len(pts)):
        for b in range(a + 1, len(pts)):
            d = np.linalg.norm(pts[a][:, None] - pts[b][None], axis=-1).min()
            gap = min(gap, d - 2 * ft["rc"])
    return float(gap)


# ---------- the arrangement in MuJoCo ----------
def build(L, sec, fr, fits, ft, R, half, key, support):
    """support: 'free' (nothing but the fingers holds the cylinder),
    'weight_carried' (its height is fixed; sliding, tilting and turning are
    free) or 'driven' (the cylinder is moved along the logged motion)."""
    mu = L["mu"]
    rc, l = ft["rc"], ft["l"]
    vis = 'contype="0" conaffinity="0" mass="0"'
    bodies, excl, fingers = [], [], []
    for k, f in enumerate(fits):
        X = np.cos(f["psi"]) * fr["e1"] + np.sin(f["psi"]) * fr["e2"]
        Y = f["mirror"] * (-np.sin(f["psi"]) * fr["e1"] + np.cos(f["psi"]) * fr["e2"])
        pos = fr["c"] + f["b"][0] * fr["e1"] + f["b"][1] * fr["e2"] + f["h"] * fr["a"]
        back = np.array(f["normal_mean"])
        back = back - (back @ fr["a"]) * fr["a"]
        back /= np.linalg.norm(back)
        c = COL[f["finger"]]
        lines = [f'<body name="f{k}_mount" mocap="true" pos="{BH._vec(pos + RETRACT * back)}" '
                 f'xyaxes="{BH._vec(X)} {BH._vec(Y)}">',
                 f'<geom type="box" size="0.005 {1.15 * rc:.5f} {1.15 * rc:.5f}" '
                 f'pos="-0.005 0 0" rgba="{c[0]} {c[1]} {c[2]} 1" {vis}/>']
        jb, jo, js = [], [], []
        for i in range(N):
            lines.append(f'<body name="f{k}_l{i}" pos="{0 if i == 0 else l:.9g} 0 0">')
            for nm, kind, axis, kk, store in (
                    ("b", "hinge", "0 0 1", ft["k_bend"], jb),
                    ("o", "hinge", "0 1 0", ft["k_out"], jo),
                    ("s", "slide", "1 0 0", ft["k_stretch"], js)):
                store.append(f"f{k}_{nm}{i}")
                lines.append(f'<joint name="{store[-1]}" type="{kind}" '
                             f'axis="{axis}" stiffness="{kk:.9g}"/>')
            lines.append(f'<geom name="f{k}_c{i}" type="capsule" fromto="0 0 0 {l:.9g} 0 0" '
                         f'size="{rc:.9g}" mass="{ft["link_mass"]:.9g}" group="3"/>')
            lines.append(f'<geom type="box" size="{0.36 * l:.6f} {0.78 * rc:.6f} {rc:.6f}" '
                         f'pos="{0.5 * l:.6f} {-0.1 * rc:.6f} 0" rgba="0.80 0.82 0.86 1" {vis}/>')
            lines.append(f'<geom type="box" size="{0.5 * l:.6f} {0.12 * rc:.6f} {rc:.6f}" '
                         f'pos="{0.5 * l:.6f} {0.85 * rc:.6f} 0" '
                         f'rgba="{0.6 * c[0]:.2f} {0.6 * c[1]:.2f} {0.6 * c[2]:.2f} 1" {vis}/>')
        lines.append(f'<geom type="sphere" size="{rc:.6f}" pos="{l:.6f} 0 0" '
                     f'rgba="{c[0]} {c[1]} {c[2]} 1" {vis}/>')
        lines += ["</body>"] * (N + 1)
        bodies.append("\n".join(lines))
        excl += [f'<exclude body1="f{k}_l{a}" body2="f{k}_l{b}"/>'
                 for a in range(N) for b in range(a + 1, N)]
        fingers.append({"k": k, "jb": jb, "jo": jo, "js": js, "fit": f,
                        "home": pos, "back": back})
    s = sec["s"]
    q0 = L["q"][s]
    Id = np.diag(L["I"]) if L["I"].ndim == 2 else np.ravel(L["I"])[:3]
    if not np.allclose(Id, Id[0]) or not np.allclose(L["I"], np.diag(Id)):
        raise SystemExit("logged inertia is not the same about every axis; the body below assumes it is")
    # the body is unrotated (so its sliding joints lie along the world axes);
    # the cylinder's shape carries the logged starting orientation
    gq = f'quat="{q0[3]:.9g} {q0[0]:.9g} {q0[1]:.9g} {q0[2]:.9g}"'
    geom = f'<geom name="obj" type="cylinder" size="{R:.9g} {half:.9g}" {gq} rgba="0.95 0.75 0.3 0.55"/>'
    if support == "driven":
        obj = f'<body name="obj" mocap="true" pos="{BH._vec(L["op"][s])}">{geom}</body>'
        weld = ""
    else:
        axes = ("1 0 0", "0 1 0", "0 0 1")[:3 if support == "free" else 2]
        obj = (f'<body name="obj" pos="{BH._vec(L["op"][s])}">'
               + "".join(f'<joint name="obj_s{i}" type="slide" axis="{a}" armature="0"/>' for i, a in enumerate(axes))
               + '<joint name="obj_ball" type="ball" armature="0"/>'
               f'<inertial pos="{BH._vec(rot(q0, L["com"]))}" mass="{L["mass"]:.9g}" '
               f'diaginertia="{Id[0]:.9g} {Id[1]:.9g} {Id[2]:.9g}"/>{geom}</body>')
        weld = '<equality><weld name="hold" body1="obj"/></equality>'
    key = f"{key}_{support}"
    xml = "\n".join([
        f'<mujoco model="pneunet_arrangement_{key}">',
        "<!-- generated by tools/pneunet_arrangement_replicates_rl_stroke_20nov.py."
        " SIMULATION ONLY. Joint damping and the springs' rest values are set"
        " after loading by that script. -->",
        '<compiler angle="radian"/>',
        f'<option gravity="0 0 -9.81" timestep="{DT}" integrator="implicitfast" cone="elliptic" '
        f'impratio="{IMPRATIO}"/>',
        f'<visual><global offwidth="{W}" offheight="{H}"/>'
        '<headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6"/></visual>',
        '<default><joint armature="1e-5"/>'
        f'<geom friction="{mu} 0.005 0.0001" solref="{G.CONTACT_TIME} 1" rgba="0.2 0.5 0.8 1"/></default>',
        '<asset><texture type="skybox" builtin="flat" rgb1="1 1 1" rgb2="1 1 1" width="32" height="32"/>'
        '<texture name="obj" type="cube" builtin="checker" rgb1="0.95 0.8 0.3" '
        'rgb2="0.75 0.3 0.15" width="64" height="64"/>'
        '<material name="obj" texture="obj" texrepeat="3 3" rgba="1 1 1 0.75"/></asset>',
        "<worldbody>", obj, *bodies, "</worldbody>",
        "<contact>" + "".join(excl) + "</contact>", weld, "</mujoco>", ""])
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
    geom_link = -np.ones(m.ngeom, int)
    for fg in fingers:
        for nm in ("jb", "jo", "js"):
            ids = [m.joint(j).id for j in fg[nm]]
            fg[nm + "_q"] = [int(m.jnt_qposadr[j]) for j in ids]
            for j in ids:
                dof = int(m.jnt_dofadr[j])
                m.dof_damping[dof] = 2 * np.sqrt(m.jnt_stiffness[j] * Mfull[dof, dof])
        fg["mocap"] = int(m.body_mocapid[m.body(f"f{fg['k']}_mount").id])
        for i in range(N):
            gid = m.geom(f"f{fg['k']}_c{i}").id
            geom_finger[gid], geom_link[gid] = fg["k"], i
    B = {"m": m, "path": path, "fingers": fingers, "ft": ft, "geom_finger": geom_finger,
         "geom_link": geom_link, "obj_geom": m.geom("obj").id, "obj_body": m.body("obj").id,
         "support": support, "q0": q0.copy(), "p0": L["op"][s].copy(), "sec": sec, "L": L}
    if support == "driven":
        B["omocap"] = int(m.body_mocapid[B["obj_body"]])
    else:
        B["nslide"] = 3 if support == "free" else 2
        B["sq"] = int(m.jnt_qposadr[m.joint("obj_s0").id])
        B["bq"] = int(m.jnt_qposadr[m.joint("obj_ball").id])
    return B


def object_pose(B, d):
    """Cylinder centre and orientation (x y z w) in the world."""
    if B["support"] == "driven":
        p, w = d.mocap_pos[B["omocap"]].copy(), d.mocap_quat[B["omocap"]]
    else:
        p = B["p0"].copy()
        p[:B["nslide"]] += d.qpos[B["sq"]:B["sq"] + B["nslide"]]
        w = d.qpos[B["bq"]:B["bq"] + 4]
    return p, qmul(w[[1, 2, 3, 0]], B["q0"])


def set_inputs(B, m, us, d=None):
    ft = B["ft"]
    for fg, u in zip(B["fingers"], us):
        th = np.interp(u, ft["u"], ft["th"])
        e = np.interp(u, ft["u"], ft["e"])
        m.qpos_spring[fg["jb_q"]] = th
        m.qpos_spring[fg["js_q"]] = e
        if d is not None:
            d.qpos[fg["jb_q"]] = th
            d.qpos[fg["js_q"]] = e


LOG_HEADER = ("t,part,turn_deg,tilt_from_start_deg,centre_from_start_mm,centre_drop_mm,"
              "ox,oy,oz,qx,qy,qz,qw," + ",".join(
                  f"f{k}_input,f{k}_touching,f{k}_normal_N,f{k}_px,f{k}_py,f{k}_pz,f{k}_link"
                  for k in range(3)))


def run(B, vary):
    """part 0: fingers brought in, cylinder held in place. 1: cylinder let go
    (or, if driven, kept at the logged starting pose), inputs constant.
    2: the stroke, inputs go evenly from first to last level (vary) or stay
    at the first level (the control). 3: inputs constant."""
    m, L, sec = B["m"], B["L"], B["sec"]
    d = mj.MjData(m)
    fs = B["fingers"]
    driven = B["support"] == "driven"
    u0 = [f["fit"]["u0"] for f in fs]
    u1 = [f["fit"]["u1"] for f in fs] if vary else u0
    set_inputs(B, m, u0, d)
    mj.mj_forward(m, d)
    contacts0 = int(d.ncon)
    every, fevery = int(round(0.01 / DT)), int(round(1 / FPS / DT))
    log, frames = [], []
    p_start, q_start = object_pose(B, d)
    quat_prev = q_start.copy()
    ax0 = rot(q_start, np.array([0, 0, 1.0]))
    turn, stopped = 0.0, None
    f6 = np.zeros(6)
    for s in range(int(round(T_HOLD / DT)) + 1):
        t = s * DT
        part = 0 if t < T_SETTLE else 1 if t < T_FREE else 2 if t < T_STROKE else 3
        a = float(np.clip((t - T_IN0) / (T_IN1 - T_IN0), 0, 1))
        x = float(np.clip((t - T_FREE) / (T_STROKE - T_FREE), 0, 1))
        us = [p + x * (q - p) for p, q in zip(u0, u1)]
        set_inputs(B, m, us)
        for fg in fs:
            d.mocap_pos[fg["mocap"]] = fg["home"] + (1 - a) * RETRACT * fg["back"]
        if driven:
            idx = sec["s"] + x * (sec["e"] - sec["s"])
            i0 = min(int(idx), sec["e"] - 1)
            w = idx - i0
            qa, qb = L["q"][i0], L["q"][i0 + 1]
            qi = qa * (1 - w) + (qb if qa @ qb >= 0 else -qb) * w
            rel = qmul(qi / np.linalg.norm(qi), inv(B["q0"]))
            d.mocap_pos[B["omocap"]] = L["op"][i0] * (1 - w) + L["op"][i0 + 1] * w
            d.mocap_quat[B["omocap"]] = rel[[3, 0, 1, 2]]
        else:
            d.eq_active[0] = part == 0
        mj.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)):
            stopped = ("non-finite state", t)
            break
        p, q = object_pose(B, d)
        dq = qmul(inv(quat_prev), q)
        turn += 2 * dq[2] * np.sign(dq[3])
        quat_prev = q
        if s % every == 0 or s % fevery == 0:
            row = []
            for k in range(3):
                Fn, P, lk, big = 0.0, np.zeros(3), -1, 0.0
                for ci in range(d.ncon):
                    c = d.contact[ci]
                    g = c.geom1 if c.geom2 == B["obj_geom"] else c.geom2 if c.geom1 == B["obj_geom"] else -1
                    if g < 0 or B["geom_finger"][g] != k:
                        continue
                    mj.mj_contactForce(m, d, ci, f6)
                    if f6[0] > big:
                        big, lk = f6[0], B["geom_link"][g]
                    Fn += f6[0]
                    P += f6[0] * c.pos
                row += [us[k], float(Fn > TOUCH_N), Fn, *(P / Fn if Fn > 0 else P), lk]
            ax = rot(q, np.array([0, 0, 1.0]))
            rec = [t, part, np.degrees(turn), np.degrees(np.arccos(np.clip(ax0 @ ax, -1, 1))),
                   1e3 * np.linalg.norm(p - p_start), 1e3 * (p_start[2] - p[2]), *p, *q, *row]
            if s % every == 0:
                log.append(rec)
            if s % fevery == 0:
                frames.append((rec, d.qpos.copy(), d.mocap_pos.copy(), d.mocap_quat.copy()))
        if B["support"] == "free" and part > 0 and p_start[2] - p[2] > DROP_STOP:
            stopped = (f"the cylinder had dropped {DROP_STOP * 1e3:.0f} mm", t)
            break
    return {"log": np.array(log), "frames": frames, "stopped": stopped, "contacts_at_start": contacts0}


# ---------- the same measures for the recording and the arrangement ----------
def recorded_measures(L, sec):
    s, e = sec["s"], sec["e"]
    sl = slice(s, e + 1)
    turn = np.degrees(L["turn"][sl] - L["turn"][s])
    out = {"turn_deg": float(turn[-1]), "duration_s": float(L["t"][e] - L["t"][s]),
           "tilt_change_max_deg": float(np.degrees(np.arccos(np.clip(L["axis"][sl] @ L["axis"][s], -1, 1))).max()),
           "centre_moved_max_mm": float(1e3 * np.linalg.norm(L["op"][sl] - L["op"][s], axis=1).max()),
           "fingers": {}}
    for j in sec["fingers"]:
        pb = rot(inv(L["q"][sl]), L["P"][sl, j] - L["op"][sl])
        out["fingers"][L["names"][j]] = {
            "normal_force_mean_N": float(L["lam"][sl, j].mean()),
            "contact_moved_in_world_mm": float(1e3 * np.linalg.norm(L["P"][e, j] - L["P"][s, j])),
            "contact_moved_on_object_mm": float(1e3 * np.linalg.norm(pb[-1] - pb[0]))}
    return out, turn


def part_change(lg, part, col):
    i = np.where(lg[:, 1] == part)[0]
    return float(lg[i[-1], col] - lg[i[0], col]) if len(i) > 1 else float("nan")


def arrangement_measures(L, sec, B, r, rec):
    """Contacts are compared with the recording at the same fraction of the
    stroke's duration (not at equal turn: the turn may be small or reversed)."""
    lg = r["log"]
    s, e = sec["s"], sec["e"]
    i_str = np.where(lg[:, 1] == 2)[0]
    i_free = np.where(lg[:, 1] >= 1)[0]
    out = {"stopped": r["stopped"], "contacts_before_fingers_brought_in": r["contacts_at_start"],
           "turn_let_go_deg": part_change(lg, 1, 2), "turn_stroke_deg": part_change(lg, 2, 2),
           "turn_final_hold_deg": part_change(lg, 3, 2), "fingers": {}}
    if len(i_free):
        out.update({"tilt_change_max_deg": float(lg[i_free, 3].max()),
                    "centre_moved_max_mm": float(lg[i_free, 4].max()),
                    "centre_drop_max_mm": float(lg[i_free, 5].max())})
    if len(i_str) < 2:
        return out
    out["centre_drop_at_stroke_start_mm"] = float(lg[i_str[0], 5])
    out["share_of_recorded_turn"] = out["turn_stroke_deg"] / rec["turn_deg"]
    rows = lg[i_str]
    frac = np.clip((rows[:, 0] - T_FREE) / (T_STROKE - T_FREE), 0, 1)
    idx = s + frac * (e - s)
    i0 = np.clip(np.floor(idx).astype(int), s, e - 1)
    w = (idx - i0)[:, None]
    for k, fg in enumerate(B["fingers"]):
        j = fg["fit"]["j"]
        c = 13 + 7 * k
        touching = rows[:, c + 1] > 0
        Pw = L["P"][i0, j] * (1 - w) + L["P"][i0 + 1, j] * w
        ew = np.linalg.norm(rows[:, c + 3:c + 6] - Pw, axis=1)
        pb = rot(inv(rows[:, 9:13]), rows[:, c + 3:c + 6] - rows[:, 6:9])
        pbr = rot(inv(L["q"][i0]), Pw - L["op"][i0])
        eo = np.linalg.norm(pb - pbr, axis=1)
        tip = rows[:, c + 6] == N - 1
        d = {"share_of_stroke_touching": float(touching.mean()),
             "share_of_touching_on_the_tip_link": float(tip[touching].mean()) if touching.any() else float("nan"),
             "normal_force_mean_N": float(rows[touching, c + 2].mean()) if touching.any() else 0.0,
             "normal_force_start_end_N": [float(rows[0, c + 2]), float(rows[-1, c + 2])],
             "input_first_last": [fg["fit"]["u0"], fg["fit"]["u1"]],
             "recorded_contact_moved_in_world_mm": rec["fingers"][fg["fit"]["finger"]]["contact_moved_in_world_mm"]}
        if touching.any():
            d.update({"contact_off_recorded_at_stroke_start_mm": float(1e3 * ew[touching][0]),
                      "contact_off_recorded_at_stroke_end_mm": float(1e3 * ew[touching][-1]),
                      "contact_off_recorded_in_world_median_mm": float(1e3 * np.median(ew[touching])),
                      "contact_off_recorded_in_world_max_mm": float(1e3 * ew[touching].max()),
                      "contact_off_recorded_on_object_median_mm": float(1e3 * np.median(eo[touching])),
                      "contact_off_recorded_on_object_max_mm": float(1e3 * eo[touching].max()),
                      "contact_moved_in_world_mm": float(1e3 * np.linalg.norm(
                          rows[touching, c + 3:c + 6][-1] - rows[touching, c + 3:c + 6][0])),
                      "contact_moved_on_object_mm": float(1e3 * np.linalg.norm(pb[touching][-1] - pb[touching][0]))})
        out["fingers"][fg["fit"]["finger"]] = d
    return out


# ---------- pictures ----------
def add_sphere(scene, pos, radius, rgba):
    if scene.ngeom >= scene.maxgeom:
        return
    mj.mjv_initGeom(scene.geoms[scene.ngeom], mj.mjtGeom.mjGEOM_SPHERE, np.array([radius, 0, 0.0]),
                    np.asarray(pos, float), np.eye(3).ravel(), np.asarray(rgba, np.float32))
    scene.ngeom += 1


def camera(fr):
    cam = mj.MjvCamera()
    cam.lookat[:] = fr["c"] - [0, 0, 0.01]
    cam.distance, cam.azimuth, cam.elevation = 0.33, 35, -50
    return cam


def recorded_scene(wave_manip, L, R, half):
    for sub in ("a3h_sandbox", os.path.join("experiment_log", "scripts")):
        sys.path.insert(0, os.path.join(wave_manip, sub))
    from w16s1_hora_scout import URDF_INTERNAL, FINGERS, ig_to_mj_body, tip_pad_from_urdf
    from w16s2_driver import fit_initial_pose
    from w16s4_replay_hora import build_replay_scene
    T, Z = L["T"], L["Z"]
    palm_pos, palm_quat, _, _ = fit_initial_pose(T, 0.0, R, half)
    m = build_replay_scene(R, half, palm_pos, palm_quat, tip_pad_from_urdf(URDF_INTERNAL), present=True)
    m.vis.global_.offwidth, m.vis.global_.offheight = W, H
    m.vis.headlight.active = 1
    m.vis.headlight.ambient[:] = .05
    m.vis.headlight.diffuse[:] = .15
    adr = []
    for nm in Z["dof_names"]:
        i = int(str(nm).split("_")[1].split(".")[0])
        adr.append(m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, f"{FINGERS[i // 4]}j{i % 4}")])
    oj = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "obj_free")
    ob = m.jnt_bodyid[oj]
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == ob:
            m.geom_matid[g] = -1
            m.geom_rgba[g] = (0.95, 0.75, 0.3, 0.55)
    pairs = []
    for i, n in enumerate(Z["body_names"]):
        b = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, ig_to_mj_body(str(n)))
        if b > 0 and "base_link" not in str(n):
            pairs.append((b, i))
    return {"m": m, "adr": adr, "oadr": int(m.jnt_qposadr[oj]), "pairs": pairs}


def pose_recorded(S, d, Z, k):
    d.qpos[S["adr"]] = Z["q"][k]
    o = Z["obj"][k]
    d.qpos[S["oadr"]:S["oadr"] + 3] = o[:3]
    d.qpos[S["oadr"] + 3:S["oadr"] + 7] = [o[6], o[3], o[4], o[5]]
    mj.mj_kinematics(S["m"], d)
    mj.mj_camlight(S["m"], d)


def film(L, sec, fr, B, r, S, rec, base, what, speed_note):
    from PIL import Image, ImageDraw, ImageFont
    try:
        font = ImageFont.truetype("arial.ttf", 24)
        small = ImageFont.truetype("arial.ttf", 20)
    except OSError:
        font = small = ImageFont.load_default()
    s, e = sec["s"], sec["e"]
    names = [L["names"][j] for j in sec["fingers"]]
    path = {L["names"][j]: L["P"][s:e + 1:3, j] for j in sec["fingers"]}
    legend = ("dots: logged contact paths (" + ", ".join(names)
              + "); black line: fixed to the cylinder, starts at the same place in both")
    foot = "SIMULATION ONLY. Comparative, not predictive."
    ff = shutil.which("ffmpeg") or r"C:\Program Files\ffmpeg\bin\ffmpeg.exe"
    proc = subprocess.Popen(
        [ff, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
         "-r", str(FPS), "-i", "-", "-pix_fmt", "yuv420p", "-vcodec", "libx264", base + ".mp4"],
        stdin=subprocess.PIPE)
    files = [base + ".mp4"]
    cam = camera(fr)

    def emit(ren, title, line, still=None, repeat=1, sub=None):
        rgb = ren.render().copy()
        ren.enable_depth_rendering()
        far = ren.render() > 5.0          # nothing in the scene is this far: background
        ren.disable_depth_rendering()
        rgb[far] = 255
        im = Image.fromarray(rgb)
        dr = ImageDraw.Draw(im)
        dr.text((20, 14), title, fill=(0, 0, 0), font=font)
        dr.text((20, 46), line, fill=(0, 0, 0), font=font)
        if sub:
            dr.text((20, 78), sub, fill=(0, 0, 0), font=small)
        dr.text((20, H - 66), legend, fill=(0, 0, 0), font=small)
        dr.text((20, H - 38), foot, fill=(0, 0, 0), font=small)
        for _ in range(repeat):
            proc.stdin.write(np.asarray(im).tobytes())
        if still:
            im.save(still)
            files.append(still)

    Rc, hc = (float(v) for v in B["m"].geom_size[B["obj_geom"]][:2])

    def dots(scene, current=None, pose=None):
        if pose is not None and scene.ngeom < scene.maxgeom:
            # a line on the cylinder's top face, fixed to the cylinder, to show its turn
            a = pose[0] + rot(pose[1], np.array([0, 0, hc + 0.001]))
            b = pose[0] + rot(pose[1], np.array([Rc, 0, hc + 0.001]))
            g = scene.geoms[scene.ngeom]
            mj.mjv_initGeom(g, mj.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).ravel(),
                            np.array([0.1, 0.1, 0.1, 1], np.float32))
            mj.mjv_connector(g, mj.mjtGeom.mjGEOM_CAPSULE, 0.0015, a, b)
            scene.ngeom += 1
        for n, pts in path.items():
            for p in pts:
                add_sphere(scene, p, 0.0016, (*COL[n], 1))
        for n, p in (current or {}).items():
            add_sphere(scene, p, 0.0035, (*COL[n], 1))

    # the recorded section
    m = S["m"]
    d = mj.MjData(m)
    ren = mj.Renderer(m, H, W)
    Z = L["Z"]
    dt = float(np.median(np.diff(L["t"])))
    pre, post = int(round(0.3 / dt)), int(round(0.2 / dt))
    nfr = int((e - s + pre + post) * dt * RL_SLOW * FPS)
    title = "Recorded: learned policy, four-finger hand turning a cylinder"
    for n in range(nfr + 1):
        k = int(round(s - pre + n / (FPS * RL_SLOW * dt)))
        pose_recorded(S, d, Z, k)
        ren.update_scene(d, cam)
        cur = {L["names"][j]: L["P"][k, j] for j in sec["fingers"] if L["lam"][k, j] > TOUCH_N}
        dots(ren.scene, cur, (Z["obj"][k][:3].astype(float), Z["obj"][k][3:7].astype(float)))
        inside = s <= k <= e
        tw = np.degrees(L["turn"][min(max(k, s), e)] - L["turn"][s])
        line = (f"log time {L['t'][k]:.2f} s, shown {RL_SLOW:g} times slower   "
                + (f"THE STROKE: cylinder turned {tw:+.1f} deg" if inside else
                   "before the stroke" if k < s else f"after the stroke ({tw:+.1f} deg)"))
        last = n == nfr
        emit(ren, title, line, still=base + "_recorded.png" if k == e else None,
             repeat=FPS if last else 1)
    ren.close()
    # the arrangement
    m = B["m"]
    d = mj.MjData(m)
    ren = mj.Renderer(m, H, W)
    title = "Arrangement of three pneu-net fingers, one input each"
    sub = what[0].upper() + what[1:]
    part_txt = ["fingers brought into place, cylinder held",
                "inputs constant" if B["support"] == "driven" else "cylinder let go, inputs constant",
                "THE STROKE", "inputs constant"]
    fr_list = r["frames"]
    for n, (row, qpos, mocap, mocap_q) in enumerate(fr_list):
        part = int(row[1])
        if part == 0 and n % 3:
            continue
        d.qpos[:] = qpos
        d.mocap_pos[:] = mocap
        d.mocap_quat[:] = mocap_q
        mj.mj_forward(m, d)
        ren.update_scene(d, cam)
        cur = {}
        for k, fg in enumerate(B["fingers"]):
            c = 13 + 7 * k
            if row[c + 1] > 0:
                cur[fg["fit"]["finger"]] = row[c + 3:c + 6]
        dots(ren.scene, cur, (np.array(row[6:9]), np.array(row[9:13])))
        tw = row[2] - rec["turn0"]
        line = (f"{part_txt[part]}   " + (f"cylinder turned {tw:+.1f} deg (recorded stroke: "
                                          f"{rec['turn_deg']:+.1f} deg)   {speed_note}" if part >= 2 else
                                          "shown 3 times faster" if part == 0 else ""))
        last = n == len(fr_list) - 1
        emit(ren, title, line, still=base + "_arrangement.png" if last else None,
             repeat=FPS if last else 1, sub=sub)
    ren.close()
    proc.stdin.close()
    proc.wait()
    return files


RUNS = [  # name, support, inputs vary, what it is
    ("cylinder_free", "free", True, "nothing but the fingers holds the cylinder"),
    ("weight_carried", "weight_carried", True,
     "the cylinder's height is fixed; it is free to slide, tilt and turn"),
    ("weight_carried_inputs_constant", "weight_carried", False,
     "control: as the last, but the inputs stay at their first levels"),
    ("cylinder_driven_along_logged_motion", "driven", True,
     "diagnostic, not a replication: the cylinder is moved along the logged motion"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", default=os.path.join(ROOT, "..", "ARIA", "Wave_manip"))
    ap.add_argument("--no-film", action="store_true")
    A = ap.parse_args()
    for sub in ("a3h_sandbox", os.path.join("experiment_log", "scripts")):
        sys.path.insert(0, os.path.join(A.wave_manip, sub))
    from w16s1_hora_scout import CYL_URDF_HALFLEN_M, CYL_URDF_RADIUS_M
    os.makedirs(OUT, exist_ok=True)
    L = load_recording()
    R, half = CYL_URDF_RADIUS_M * L["scale"], CYL_URDF_HALFLEN_M * L["scale"]
    secs = sections(L, R)
    squeeze = SQUEEZE_FACTOR * L["mass"] * 9.81 / (L["mu"] * 3)
    rep = ["One stroke of a recorded learned rotation, replicated in part by pneu-net fingers",
           "SIMULATION ONLY. Comparative, not predictive.", "",
           f"Recording: cylinder radius {R * 1e3:.1f} mm, half length {half * 1e3:.1f} mm, mass "
           f"{L['mass'] * 1e3:.1f} g, friction {L['mu']:.2f}; {len(L['t'])} samples over {L['t'][-1]:.1f} s; "
           f"turn about its own axis over the log {np.degrees(L['turn'][-1]):.0f} deg.",
           f"Sections with the same three fingertips touching (normal force over {TOUCH_N} N) for at least "
           f"{MIN_SECTION_S} s, mean tilt under {MAX_TILT_DEG:g} deg, tilt range under {MAX_TILT_RANGE_DEG:g} deg, "
           f"centre moving under {MAX_DRIFT_M * 1e3:g} mm: {len(secs)}. Up to {N_SECTIONS}, largest turn first, are fitted.",
           f"Squeeze each pneu-net is placed to give (ASSUMED): {squeeze:.3f} N, {SQUEEZE_FACTOR:g} times what "
           "friction needs to carry the weight, shared between three.",
           f"Contact model: elliptic friction cone, impratio {IMPRATIO}, time step {DT} s.", ""]
    fts = {rid: G.finger_table(rid) for rid in RECORDS}
    tabs = {rid: chain_tables(fts[rid]) for rid in RECORDS}
    for rid in RECORDS:
        ft = fts[rid]
        _, C = tabs[rid]
        rep.append(f"Finger {rid}: length {ft['L'] * 1e3:.0f} mm, contact radius {ft['rc'] * 1e3:.2f} mm, free stroke "
                   f"{ft['stroke'] * 1e3:.1f} mm; tip stiffness at zero input along / in the bending plane "
                   f"{1 / C[0, 0, 0]:.1f} / {1 / C[0, 1, 1]:.2f} N/m (record {ft['k_rec'][0]:.1f} / {ft['k_rec'][1]:.2f}).")
    rep += ["", "Fit of each finger's loaded tip path to the logged contact path (no simulation):",
            "  section (log s)  fingertips          recorded turn  finger record   misfit rms per finger (mm)   "
            "worst (mm)  gap between finger bodies (mm)"]
    cands = []
    for sec in secs[:N_SECTIONS]:
        fr = section_frame(L, sec)
        for rid in RECORDS:
            fits = [fit_finger(L, sec, fr, j, fts[rid], tabs[rid], R, squeeze) for j in sec["fingers"]]
            gap = fingers_clash(fits, fts[rid], tabs[rid], fr)
            worst = max(f["misfit_rms_m"] for f in fits)
            cands.append({"sec": sec, "fr": fr, "rid": rid, "fits": fits, "gap": gap, "worst": worst})
            rep.append(f"  {sec['t0']:.2f} to {sec['t1']:.2f}   {'+'.join(f['finger'] for f in fits):<19s} "
                       f"{sec['turn_deg']:+7.1f} deg    {rid:<14s}  "
                       + "  ".join(f"{f['misfit_rms_m'] * 1e3:5.2f}" for f in fits)
                       + f"          {worst * 1e3:5.2f}      {gap * 1e3:+6.1f}")
            print(rep[-1], flush=True)
    ok = [c for c in cands if c["gap"] > 0]
    rep.append(f"  Arrangements whose finger bodies do not overlap: {len(ok)} of {len(cands)}.")
    pick = min(ok or cands, key=lambda c: c["worst"])
    sec, fr, rid, fits = pick["sec"], pick["fr"], pick["rid"], pick["fits"]
    ft = fts[rid]
    rec, _ = recorded_measures(L, sec)
    rep += ["", f"Chosen (smallest worst-finger misfit among those that do not overlap): log {sec['t0']:.2f} to "
            f"{sec['t1']:.2f} s, {rid}.",
            f"  section: mean tilt {sec['tilt_mean_deg']:.1f} deg, tilt range {sec['tilt_range_deg']:.1f} deg, "
            f"centre moves {sec['drift_m'] * 1e3:.1f} mm."]
    for f in fits:
        rep.append(f"  {f['finger']:<6s}: logged contact path {f['path_length_m'] * 1e3:.1f} mm long; misfit rms "
                   f"{f['misfit_rms_m'] * 1e3:.2f} mm, largest {f['misfit_max_m'] * 1e3:.2f} mm; path leaves the "
                   f"finger's plane by {f['off_plane_rms_m'] * 1e3:.2f} mm rms; input {f['u0']:.0f} to {f['u1']:.0f} "
                   f"of {ft['u'][-1]:.0f} (simulation units); body inside the cylinder by at most "
                   f"{f['body_inside_cylinder_max_m'] * 1e3:.2f} mm in the free shape.")
        rep.append(f"          tip stiffness at the first input level: {f['k_normal_N_per_m']:.1f} N/m along the "
                   f"contact normal, {f['k_along_surface_N_per_m']:.1f} N/m along the cylinder's surface; friction at "
                   f"the assumed squeeze can hold the tip back by {f['friction_can_hold_back_tip_m'] * 1e3:.1f} mm; "
                   f"the recorded {rec['fingers'][f['finger']]['normal_force_mean_N']:.2f} N would push the tip back "
                   f"{rec['fingers'][f['finger']]['normal_force_mean_N'] / f['k_normal_N_per_m'] * 1e3:.0f} mm.")
    key = f"log_{sec['t0']:.2f}s_{rid}"
    results = {"recording": rec, "section": {k: v for k, v in sec.items() if k != "fingers"},
               "finger_record": rid, "squeeze_assumed_N": squeeze, "fits": fits, "runs": {},
               "candidates": [{"t0": c["sec"]["t0"], "t1": c["sec"]["t1"], "record": c["rid"],
                               "worst_misfit_rms_m": c["worst"], "gap_m": c["gap"]} for c in cands]}
    rep += ["", f"Recorded stroke: cylinder turns {rec['turn_deg']:+.1f} deg in {rec['duration_s']:.2f} s; tilt changes "
            f"by at most {rec['tilt_change_max_deg']:.1f} deg; centre moves at most {rec['centre_moved_max_mm']:.1f} mm."]
    for n, v in rec["fingers"].items():
        rep.append(f"  {n:<6s}: normal force {v['normal_force_mean_N']:.2f} N; contact moves "
                   f"{v['contact_moved_in_world_mm']:.1f} mm in the world, {v['contact_moved_on_object_mm']:.1f} mm on the cylinder.")
    rep += ["", f"Simulated timeline (ASSUMED): fingers brought in {T_IN0:g} to {T_IN1:g} s with the cylinder held; "
            f"let go at {T_SETTLE:g} s; stroke {T_FREE:g} to {T_STROKE:g} s; inputs constant to {T_HOLD:g} s.",
            "Contacts are compared with the recording at the same fraction of the stroke's duration."]
    runs, builds = {}, {}
    for name, support, vary, what in RUNS:
        B = builds.get(support) or build(L, sec, fr, fits, ft, R, half, key, support)
        builds[support] = B
        r = run(B, vary)
        ms = arrangement_measures(L, sec, B, r, rec)
        runs[name] = (B, r, what)
        results["runs"][name] = ms
        np.savetxt(os.path.join(OUT, f"log_{key}_{name}.csv"), r["log"], delimiter=",", fmt="%.6g",
                   header=LOG_HEADER, comments="")
        rep += ["", f"Arrangement, {name.replace('_', ' ')} ({what}): "
                + (f"STOPPED, {r['stopped'][0]} at {r['stopped'][1]:.2f} s" if r["stopped"] else "ran to the end")
                + f"; contacts before the fingers are brought in: {ms['contacts_before_fingers_brought_in']}.",
                f"  cylinder turn: {ms['turn_let_go_deg']:+.2f} deg in the {T_FREE - T_SETTLE:g} s after being let go, "
                f"{ms['turn_stroke_deg']:+.2f} deg in the stroke, {ms['turn_final_hold_deg']:+.2f} deg in the final "
                f"{T_HOLD - T_STROKE:g} s (recorded stroke {rec['turn_deg']:+.1f} deg)."]
        if "tilt_change_max_deg" in ms:
            rep.append(f"  tilt changed by at most {ms['tilt_change_max_deg']:.1f} deg; centre moved at most "
                       f"{ms['centre_moved_max_mm']:.1f} mm, dropped at most {ms['centre_drop_max_mm']:.1f} mm"
                       + (f" ({ms['centre_drop_at_stroke_start_mm']:.1f} mm by the start of the stroke)."
                          if "centre_drop_at_stroke_start_mm" in ms else " (it fell before the stroke began)."))
        for n, v in ms["fingers"].items():
            if "contact_off_recorded_in_world_median_mm" not in v:
                rep.append(f"  {n:<6s}: never touched in the stroke.")
                continue
            rep.append(f"  {n:<6s}: touching {100 * v['share_of_stroke_touching']:.0f}% of the stroke "
                       f"({100 * v['share_of_touching_on_the_tip_link']:.0f}% of that on the tip link); normal force "
                       f"{v['normal_force_start_end_N'][0]:.2f} N at the start, {v['normal_force_start_end_N'][1]:.2f} N "
                       f"at the end; contact moved {v['contact_moved_in_world_mm']:.1f} mm in the world (recorded "
                       f"{v['recorded_contact_moved_in_world_mm']:.1f} mm); off the recorded contact by "
                       f"{v['contact_off_recorded_at_stroke_start_mm']:.1f} mm at the start, "
                       f"{v['contact_off_recorded_at_stroke_end_mm']:.1f} mm at the end, median "
                       f"{v['contact_off_recorded_in_world_median_mm']:.1f} mm, largest "
                       f"{v['contact_off_recorded_in_world_max_mm']:.1f} mm in the world; median "
                       f"{v['contact_off_recorded_on_object_median_mm']:.1f} mm on the cylinder.")
    a, b = results["runs"]["weight_carried"], results["runs"]["weight_carried_inputs_constant"]
    net = a["turn_stroke_deg"] - b["turn_stroke_deg"]
    results["weight_carried_turn_less_control_deg"] = net
    rep += ["", f"Weight carried, turn in the stroke less the control's turn over the same time: {net:+.2f} deg, "
            f"{100 * net / rec['turn_deg']:.0f}% of the recorded {rec['turn_deg']:+.1f} deg."]

    def save():
        with open(os.path.join(OUT, "arrangement_report.txt"), "w", encoding="utf-8") as fh:
            fh.write("\n".join(rep) + "\n")
        with open(os.path.join(OUT, "arrangement_results.json"), "w", encoding="utf-8") as fh:
            json.dump(results, fh, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    save()
    print("\n".join(rep), flush=True)
    if not A.no_film:
        n0 = len(rep)
        S = recorded_scene(A.wave_manip, L, R, half)
        d = mj.MjData(S["m"])
        errs = []
        for k in range(sec["s"], sec["e"] + 1, 5):
            pose_recorded(S, d, L["Z"], k)
            errs += [np.linalg.norm(d.xpos[b] - L["Z"]["body"][k, i, :3]) for b, i in S["pairs"]]
        rep += ["", f"Replay of the recording (Wave_manip's hand model posed from the logged joint angles): links off "
                f"the logged link positions by median {1e3 * np.median(errs):.2f} mm, at most {1e3 * np.max(errs):.2f} mm "
                "over the section."]
        note = f"stroke takes {T_STROKE - T_FREE:g} s here, {rec['duration_s']:.2f} s in the recording"
        for name, (B, r, what) in runs.items():
            if name == "weight_carried_inputs_constant":
                continue
            lg = r["log"]
            i2 = np.where(lg[:, 1] >= 2)[0]
            rec["turn0"] = float(lg[i2[0], 2]) if len(i2) else 0.0
            fs = film(L, sec, fr, B, r, S, rec, os.path.join(OUT, f"recorded_then_arrangement_{name}"), what, note)
            rep += [f"  wrote {f}" for f in fs]
        save()
        print("\n".join(rep[n0:]))


if __name__ == "__main__":
    main()
