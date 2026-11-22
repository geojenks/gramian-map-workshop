"""One stroke of a recorded learned rotation, re-enacted by three pneu-net
fingers standing upright on a palm: MuJoCo (21 Nov 2026).

SIMULATION ONLY. Comparative, not predictive.

This replaces the layout of
tools/pneunet_arrangement_replicates_rl_stroke_20nov.py (fingers lying flat
round the cylinder) and reuses that script's recording loader, finger
stand-in, simulation loop and measures by import.

  1. Section of the recording: the same three fingertips touch throughout
     (that script's thresholds), all three contacts are on the cylinder's
     side wall, and the three contacts are as near as the log offers to an
     equilateral triangle round the cylinder (gaps between them near 120 deg).
  2. One pneu-net finger (one input) per contact, standing on a palm below
     the cylinder. Each finger stands beside the cylinder and bends in a
     plane that touches the cylinder's side, so its tip sweeps along the
     surface. The squeeze is carried by the finger bending sideways out of
     that plane. It is the smaller of the logged normal force and a share of
     an estimated limit (see squeeze_limit). Fitted per finger, with no
     simulation: where the plane touches, how far out it stands and how much
     it leans outward, the base's place in the plane, the lean within the
     plane, which way it bends, and the first and last input level.
  3. The arrangement is simulated with that script's timeline and cases.
  4. One clip, three columns playing together at the same fraction of the
     stroke: the recording, the extracted contact points, the re-enactment.

ASSUMED, and stated in the report: the share of the estimated squeeze limit;
friction between finger and cylinder (the
log's value for the cylinder); the duration of the simulated stroke; the
thresholds that pick the section; the height of the mounts above the palm
plate (drawn only).

Run from the repository root with a Python that has mujoco (needs ffmpeg):

    python tools/pneunet_upright_arrangement_replicates_rl_stroke_21nov.py --wave-manip <Wave_manip folder>

Writes outputs/pneunet_upright_arrangement_replicates_rl_stroke_21nov/.
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
import mujoco as mj                                                  # noqa: E402
import build_record_hand as BH                                       # noqa: E402
import pneunet_arrangement_replicates_rl_stroke_20nov as A           # noqa: E402

G = A.G
N = A.N
OUT = os.path.join(ROOT, "outputs", "pneunet_upright_arrangement_replicates_rl_stroke_21nov")
RECORDS = A.RECORDS
SIDE_NORMAL_MAX = 0.3     # a contact is on the side wall if its normal is this near square to the axis
TRIANGLE_BAND_DEG = 5.0   # sections this near the best triangle are tied; the largest turn wins
CLEAR = 0.0005            # links below the tip are kept this clear of the cylinder in the fit
MOUNT = 0.012             # assumed: palm plate this far below the lowest finger base (drawn only)
BETA_MAX, ALPHA_MAX = 0.09, 0.5
SQUEEZE_SHARE = 0.5       # assumed: squeeze as a share of the largest steady one (see squeeze_limit)
SQUARE_WEIGHT = 0.03      # m: how hard the fit holds the bending plane square to the contact normal
CW, CH, HEAD, FOOT = 640, 720, 96, 92
FPS = A.FPS
ELEVATION = -28.0       # camera elevation, deg
STILLS_ONLY = False
COL = A.COL


# ---------- the section ----------
def triangle_sections(L, R, half):
    out = []
    for sec in A.sections(L, R):
        if len(sec["fingers"]) != 3:
            continue
        sl = slice(sec["s"], sec["e"] + 1)
        fr = A.section_frame(L, sec)
        az, hs, nz = [], [], []
        for j in sec["fingers"]:
            r = L["P"][sl, j] - L["op"][sl]
            az.append(np.degrees(np.arctan2((r @ fr["e2"]).mean(), (r @ fr["e1"]).mean())))
            hs.append(r @ L["axis"][sl].mean(0) / np.linalg.norm(L["axis"][sl].mean(0)))
            nz.append(float(np.abs((L["nrm"][sl, j] * L["axis"][sl]).sum(1)).mean()))
        o = np.sort(np.mod(az, 360))
        gaps = np.diff(np.concatenate([o, [o[0] + 360]]))
        sec = dict(sec, gaps_deg=gaps.tolist(), off_triangle_deg=float(np.abs(gaps - 120).max()),
                   contact_height_mm=[float(1e3 * h.mean()) for h in hs],
                   normal_along_axis=nz,
                   on_side=bool(max(nz) < SIDE_NORMAL_MAX and max(np.abs(h).max() for h in hs) < half))
        out.append(sec)
    out.sort(key=lambda x: x["off_triangle_deg"])
    return out


# ---------- the finger: free shape, and how far a tip load moves the tip ----------
def tables(ft, ng=121):
    J, C = A.chain_tables(ft, ng)
    ug = np.linspace(0, ft["u"][-1], ng)
    th = np.interp(ug, ft["u"], ft["th"])
    ang = th[:, None] * np.arange(1, N + 1)
    d = np.stack([np.cos(ang), np.sin(ang)], -1)
    arm = ((J[:, -1:, :] - J[:, :-1, :]) * d).sum(-1)       # along each link, joint to tip
    Co = (arm * arm).sum(1) / ft["k_out"]                   # tip compliance out of the bending plane
    return J, C, Co


def axes(x, mirror, fr):
    phi, rho, beta, sb, hb, alpha = x[0], x[1] * 0.05, x[2], x[3] * 0.05, x[4] * 0.05, x[5]
    nr = np.cos(phi) * fr["e1"] + np.sin(phi) * fr["e2"]
    tau = -np.sin(phi) * fr["e1"] + np.cos(phi) * fr["e2"]
    w = np.cos(beta) * fr["a"] - np.sin(beta) * nr          # up the plane; it leans in towards the top
    o = np.cos(beta) * nr + np.sin(beta) * fr["a"]          # square to the plane, outward
    X = np.cos(alpha) * w + np.sin(alpha) * tau             # the finger's rest axis
    Y = mirror * (-np.sin(alpha) * w + np.cos(alpha) * tau)  # the way it bends
    return rho * nr + sb * tau + hb * w, X, Y, o, tau, w


def fit_parts(x, tab, mirror, fr, load, lin, ft):
    J, C, Co = tab
    ng = J.shape[0] - 1
    base, X, Y, o, _, _ = axes(x, mirror, fr)
    uf = x[6] + (x[7] - x[6]) * lin
    f = np.clip(uf, 0, 1) * ng
    i0 = np.minimum(f.astype(int), ng - 1)
    wt = f - i0
    Jm = J[i0] * (1 - wt)[:, None, None] + J[i0 + 1] * wt[:, None, None]
    Cm = C[i0] * (1 - wt)[:, None, None] + C[i0 + 1] * wt[:, None, None]
    Com = Co[i0] * (1 - wt) + Co[i0 + 1] * wt
    # The contact is taken to stick, so friction carries whatever acts along the surface and the
    # tip's load is square to the bending plane, of the size that gives the logged normal force.
    nl = np.linalg.norm(load, axis=1)
    do = Com * nl * nl / np.maximum(load @ o, 0.7 * nl)
    # the finger's own weight bends it within its plane: moment at each joint from the links beyond
    wg = ft["link_mass"] * 9.81 * np.array([-X[2], -Y[2]])
    cen = 0.5 * (Jm[:, :-1] + Jm[:, 1:])
    d2 = np.zeros((len(lin), 2))
    for n in range(N):
        arm = cen[:, n:] - Jm[:, n:n + 1]
        dth = (arm[..., 0] * wg[1] - arm[..., 1] * wg[0]).sum(1) / ft["k_bend"]
        tv = Jm[:, -1] - Jm[:, n]
        d2 += dth[:, None] * np.stack([-tv[:, 1], tv[:, 0]], -1)
    tip = (base + (Jm[:, -1, 0] + d2[:, 0])[:, None] * X + (Jm[:, -1, 1] + d2[:, 1])[:, None] * Y
           + do[:, None] * o)
    # links below the tip, pushed out by the load in rough proportion (square of the way along)
    share = (np.arange(N) / N) ** 2
    jo = (base + Jm[:, :N, 0, None] * X + Jm[:, :N, 1, None] * Y
          + (do[:, None] * share)[:, :, None] * o)
    return tip, jo, uf, do


def squeeze_limit(ft, tab, R):
    """An estimate, not a measurement. A tip pressed on a round object by a finger that is soft along the surface slides off to one
    side once the squeeze exceeds the finger's stiffness along the surface times the distance from
    the axis to the tip's centre. Stiffness: the straight finger's, at the tip, in the bending plane."""
    return (R + ft["rc"]) / tab[1][0][1, 1]


def fit_finger(L, sec, fr, j, ft, tab, R, half):
    s, e = sec["s"], sec["e"]
    ks = np.round(np.linspace(s, e, A.M_FIT)).astype(int)
    n3 = L["nrm"][ks, j]                                    # points into the cylinder
    g3 = L["P"][ks, j] - ft["rc"] * n3 - fr["c"]            # where the tip's centre must be
    logged = float(L["lam"][s:e + 1, j].mean())
    squeeze = min(logged, SQUEEZE_SHARE * squeeze_limit(ft, tab, R))
    load = -squeeze * n3                                    # the cylinder's push on the tip
    lin = np.linspace(0, 1, A.M_FIT)
    phi0 = float(np.arctan2(g3.mean(0) @ fr["e2"], g3.mean(0) @ fr["e1"]))
    best = None
    for mirror in (1.0, -1.0):
        def cost(x):
            tip, jo, uf, _ = fit_parts(x, tab, mirror, fr, load, lin, ft)
            r = tip - g3
            hh = jo @ fr["a"]
            rad = np.linalg.norm(jo - hh[..., None] * fr["a"], axis=-1)
            pen = np.where(np.abs(hh) < half + ft["rc"], np.maximum(0, R + ft["rc"] + CLEAR - rad), 0)[:, :N - 1]
            out = np.maximum(0, np.abs(uf - 0.5) - 0.5)
            lim = max(0, -x[2]) + max(0, x[2] - BETA_MAX) + max(0, abs(x[5]) - ALPHA_MAX)
            along = float((n3 @ axes(x, mirror, fr)[4]).mean())   # the normal's mean lean along the surface
            return ((r * r).sum(1).mean() + 10 * (pen * pen).mean() + (out * out).sum() + lim * lim
                    + (SQUARE_WEIGHT * along) ** 2)
        for alpha0 in (-0.2, 0.0, 0.2):
            for ua, ub in ((0.3, 0.6), (0.6, 0.3)):
                x0 = np.array([phi0, (R + ft["rc"]) / 0.05, 0.1, 0, 0, alpha0, ua, ub])
                tip, _, _, _ = fit_parts(x0, tab, mirror, fr, load, lin, ft)
                _, _, _, _, tau, w = axes(x0, mirror, fr)
                dm = (g3 - tip).mean(0)
                x0[3], x0[4] = dm @ tau / 0.05, dm @ w / 0.05
                x, fv = A.nelder_mead(cost, x0, 0.1, 700)
                x, fv = A.nelder_mead(cost, x, 0.02, 400)
                if best is None or fv < best[1]:
                    best = (x, fv, mirror)
    x, fv, mirror = best
    tip, jo, uf, do = fit_parts(x, tab, mirror, fr, load, lin, ft)
    base, X, Y, o, tau, w = axes(x, mirror, fr)
    own = tip - fit_parts(x, tab, mirror, fr, load, lin, dict(ft, link_mass=0.0))[0]
    r = np.linalg.norm(tip - g3, axis=1)
    hh = jo @ fr["a"]
    rad = np.linalg.norm(jo - hh[..., None] * fr["a"], axis=-1)
    beside = np.abs(hh) < half + ft["rc"]
    clear = np.where(beside, rad - R - ft["rc"], np.inf)[:, :N - 1]
    J, C, Co = tab
    i0 = int(round(np.clip(x[6], 0, 1) * (J.shape[0] - 1)))
    A2 = np.stack([X, Y], -1)                               # plane coordinates to world
    Cw = A2 @ C[i0] @ A2.T
    return {"finger": L["names"][j], "j": j, "squeeze_N": squeeze, "logged_normal_force_N": logged, "squeeze_limit_N": squeeze_limit(ft, tab, R), "mirror": mirror,
            "plane_touches_at_deg": float(np.degrees(x[0])), "plane_stands_out_m": float(x[1] * 0.05),
            "plane_leans_out_deg": float(np.degrees(x[2])), "lean_in_plane_deg": float(np.degrees(x[5])),
            "base": (fr["c"] + base).tolist(), "X": X.tolist(), "Y": Y.tolist(), "o": o.tolist(),
            "back": (np.cos(x[0]) * fr["e1"] + np.sin(x[0]) * fr["e2"]).tolist(),
            "base_height_m": float(base @ fr["a"]),
            "u0": float(np.clip(x[6], 0, 1) * ft["u"][-1]), "u1": float(np.clip(x[7], 0, 1) * ft["u"][-1]),
            "misfit_rms_m": float(np.sqrt((r * r).mean())), "misfit_max_m": float(r.max()),
            "path_length_m": float(np.linalg.norm(np.diff(g3, axis=0), axis=1).sum()),
            "path_height_range_m": float(np.ptp(g3 @ fr["a"])),
            "tip_pushed_out_m": float(np.abs(do).mean()),
            "normal_off_square_to_plane_deg": [float(np.degrees(np.arcsin(v))) for v in (n3 @ tau)[[0, len(n3) // 2, -1]]], "own_weight_moves_tip_m": float(np.linalg.norm(own, axis=1).mean()),
            "k_out_of_plane_N_per_m": float(1 / Co[i0]),
            "k_along_surface_N_per_m": float(1 / (tau @ Cw @ tau)),
            "links_below_tip_clear_min_m": float(clear.min()),
            "joints": (fr["c"] + jo[[0, -1]]).tolist(), "tips": (fr["c"] + tip[[0, -1]]).tolist()}


def fingers_gap(fits, ft):
    pts = [np.vstack([np.vstack([np.array(f["joints"][i]), np.array(f["tips"][i])[None]]) for i in (0, 1)])
           for f in fits]
    gap = np.inf
    for a in range(len(pts)):
        for b in range(a + 1, len(pts)):
            gap = min(gap, np.linalg.norm(pts[a][:, None] - pts[b][None], axis=-1).min() - 2 * ft["rc"])
    return float(gap)


# ---------- the arrangement in MuJoCo ----------
def build(L, sec, fr, fits, ft, R, half, key, support):
    """support as in the 20 Nov script: 'free', 'weight_carried' or 'driven'."""
    mu, rc, l = L["mu"], ft["rc"], ft["l"]
    vis = 'contype="0" conaffinity="0" mass="0"'
    h_plate = min(f["base_height_m"] for f in fits) - MOUNT
    r_plate = max(np.linalg.norm(np.array(f["base"]) - fr["c"] - f["base_height_m"] * fr["a"]) for f in fits) + 0.02
    zq = np.zeros(4)
    mj.mju_quatZ2Vec(zq, fr["a"])
    plate = (f'<geom type="cylinder" size="{r_plate:.5f} 0.002" pos="{BH._vec(fr["c"] + (h_plate - 0.002) * fr["a"])}" '
             f'quat="{zq[0]:.9g} {zq[1]:.9g} {zq[2]:.9g} {zq[3]:.9g}" rgba="0.55 0.57 0.62 1" {vis}/>')
    bodies, excl, fingers = [], [], []
    for k, f in enumerate(fits):
        X, Y, pos, back = np.array(f["X"]), np.array(f["Y"]), np.array(f["base"]), np.array(f["back"])
        Z = np.cross(X, Y)
        post = -(f["base_height_m"] - h_plate) * np.array([fr["a"] @ X, fr["a"] @ Y, fr["a"] @ Z])
        c = COL[f["finger"]]
        lines = [f'<body name="f{k}_mount" mocap="true" pos="{BH._vec(pos + A.RETRACT * back)}" '
                 f'xyaxes="{BH._vec(X)} {BH._vec(Y)}">',
                 f'<geom type="box" size="0.005 {1.15 * rc:.5f} {1.15 * rc:.5f}" '
                 f'pos="-0.005 0 0" rgba="{c[0]} {c[1]} {c[2]} 1" {vis}/>',
                 f'<geom type="cylinder" fromto="0 0 0 {BH._vec(post)}" size="{0.8 * rc:.5f}" '
                 f'rgba="0.55 0.57 0.62 1" {vis}/>']
        jb, jo, js = [], [], []
        for i in range(N):
            lines.append(f'<body name="f{k}_l{i}" pos="{0 if i == 0 else l:.9g} 0 0">')
            for nm, kind, axis, kk, store in (
                    ("b", "hinge", "0 0 1", ft["k_bend"], jb),
                    ("o", "hinge", "0 1 0", ft["k_out"], jo),
                    ("s", "slide", "1 0 0", ft["k_stretch"], js)):
                store.append(f"f{k}_{nm}{i}")
                lines.append(f'<joint name="{store[-1]}" type="{kind}" axis="{axis}" stiffness="{kk:.9g}"/>')
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
        excl += [f'<exclude body1="f{k}_l{a}" body2="f{k}_l{b}"/>' for a in range(N) for b in range(a + 1, N)]
        fingers.append({"k": k, "jb": jb, "jo": jo, "js": js, "fit": f, "home": pos, "back": back})
    s = sec["s"]
    q0 = L["q"][s]
    Id = np.diag(L["I"]) if L["I"].ndim == 2 else np.ravel(L["I"])[:3]
    if not np.allclose(Id, Id[0]) or not np.allclose(L["I"], np.diag(Id)):
        raise SystemExit("logged inertia is not the same about every axis; the body below assumes it is")
    gq = f'quat="{q0[3]:.9g} {q0[0]:.9g} {q0[1]:.9g} {q0[2]:.9g}"'
    geom = f'<geom name="obj" type="cylinder" size="{R:.9g} {half:.9g}" {gq} rgba="0.95 0.75 0.3 0.55"/>'
    if support == "driven":
        obj = f'<body name="obj" mocap="true" pos="{BH._vec(L["op"][s])}">{geom}</body>'
        weld = ""
    else:
        ax = ("1 0 0", "0 1 0", "0 0 1")[:3 if support == "free" else 2]
        obj = (f'<body name="obj" pos="{BH._vec(L["op"][s])}">'
               + "".join(f'<joint name="obj_s{i}" type="slide" axis="{a}" armature="0"/>' for i, a in enumerate(ax))
               + '<joint name="obj_ball" type="ball" armature="0"/>'
               f'<inertial pos="{BH._vec(A.rot(q0, L["com"]))}" mass="{L["mass"]:.9g}" '
               f'diaginertia="{Id[0]:.9g} {Id[1]:.9g} {Id[2]:.9g}"/>{geom}</body>')
        weld = '<equality><weld name="hold" body1="obj"/></equality>'
    key = f"{key}_{support}"
    xml = "\n".join([
        f'<mujoco model="pneunet_upright_arrangement_{key}">',
        "<!-- generated by tools/pneunet_upright_arrangement_replicates_rl_stroke_21nov.py."
        " SIMULATION ONLY. Joint damping and the springs' rest values are set"
        " after loading by that script. The palm plate and posts are drawn only. -->",
        '<compiler angle="radian"/>',
        f'<option gravity="0 0 -9.81" timestep="{A.DT}" integrator="implicitfast" cone="elliptic" '
        f'impratio="{A.IMPRATIO}"/>',
        f'<visual><global offwidth="{CW}" offheight="{CH}"/>'
        '<headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6"/></visual>',
        '<default><joint armature="1e-5"/>'
        f'<geom friction="{mu} 0.005 0.0001" solref="{G.CONTACT_TIME} 1" rgba="0.2 0.5 0.8 1"/></default>',
        '<asset><texture type="skybox" builtin="flat" rgb1="1 1 1" rgb2="1 1 1" width="32" height="32"/></asset>',
        "<worldbody>", plate, obj, *bodies, "</worldbody>",
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
         "support": support, "q0": q0.copy(), "p0": L["op"][s].copy(), "sec": sec, "L": L,
         "h_plate": h_plate}
    if support == "driven":
        B["omocap"] = int(m.body_mocapid[B["obj_body"]])
    else:
        B["nslide"] = 3 if support == "free" else 2
        B["sq"] = int(m.jnt_qposadr[m.joint("obj_s0").id])
        B["bq"] = int(m.jnt_qposadr[m.joint("obj_ball").id])
    return B


# ---------- pictures ----------
def contacts_scene(L, sec, R, half):
    """The logged cylinder alone, for the column of extracted contact points."""
    q0 = L["q"][sec["s"]]
    xml = (f'<mujoco><visual><global offwidth="{CW}" offheight="{CH}"/>'
           '<headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6"/></visual>'
           '<asset><texture type="skybox" builtin="flat" rgb1="1 1 1" rgb2="1 1 1" width="32" height="32"/></asset>'
           f'<worldbody><body name="obj" mocap="true" pos="{BH._vec(L["op"][sec["s"]])}" '
           f'quat="{q0[3]:.9g} {q0[0]:.9g} {q0[1]:.9g} {q0[2]:.9g}">'
           f'<geom type="cylinder" size="{R:.9g} {half:.9g}" rgba="0.95 0.75 0.3 0.4"/></body></worldbody></mujoco>')
    return mj.MjModel.from_xml_string(xml)


def connector(scene, kind, width, a, b, rgba):
    if scene.ngeom >= scene.maxgeom:
        return
    g = scene.geoms[scene.ngeom]
    mj.mjv_initGeom(g, kind, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.asarray(rgba, np.float32))
    mj.mjv_connector(g, kind, width, np.asarray(a, float), np.asarray(b, float))
    scene.ngeom += 1


def camera(fr, h_plate, azimuth):
    cam = mj.MjvCamera()
    cam.lookat[:] = fr["c"] + 0.45 * h_plate * fr["a"]
    cam.distance, cam.azimuth, cam.elevation = 0.40, azimuth, ELEVATION
    return cam


def film(L, sec, fr, B, r, S, rec, R, half, base, what, azimuth):
    from PIL import Image, ImageDraw, ImageFont
    try:
        font = ImageFont.truetype("arial.ttf", 24)
        small = ImageFont.truetype("arial.ttf", 19)
    except OSError:
        font = small = ImageFont.load_default()
    s, e = sec["s"], sec["e"]
    js = sec["fingers"]
    names = [L["names"][j] for j in js]
    Wt, Ht = 3 * CW, HEAD + CH + FOOT
    ff = shutil.which("ffmpeg") or r"C:\Program Files\ffmpeg\bin\ffmpeg.exe"
    proc = subprocess.Popen(
        [ff, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{Wt}x{Ht}",
         "-r", str(FPS), "-i", "-", "-pix_fmt", "yuv420p", "-vcodec", "libx264", base + ".mp4"],
        stdin=subprocess.PIPE)
    files = [base + ".mp4"]
    cam = camera(fr, B["h_plate"], azimuth)
    mC = contacts_scene(L, sec, R, half)
    models = [S["m"], mC, B["m"]]
    S["m"].vis.global_.offwidth, S["m"].vis.global_.offheight = CW, CH
    datas = [mj.MjData(m) for m in models]
    rens = [mj.Renderer(m, CH, CW) for m in models]
    Z = L["Z"]
    stroke_s = A.T_STROKE - A.T_FREE
    slow = stroke_s / rec["duration_s"]
    titles = ["Recorded: learned policy, four-finger hand",
              "Extracted: the three contact points",
              "Re-enactment: three one-input pneu-nets"]
    foot = [f"All three play together at the same fraction of the stroke. The recorded stroke takes "
            f"{rec['duration_s']:.2f} s and is shown {slow:.1f} times slower; the re-enactment takes {stroke_s:g} s. "
            f"Fingertips: {', '.join(names)}.",
            "Small dots: logged contact path. Large dots: contact now. Arrows: logged normal force. Black line: "
            "fixed to the cylinder.",
            f"Re-enactment: {what}. SIMULATION ONLY. Comparative, not predictive."]

    def shot(i, extra):
        ren, d = rens[i], datas[i]
        ren.update_scene(d, cam)
        extra(ren.scene)
        rgb = ren.render().copy()
        ren.enable_depth_rendering()
        far = ren.render() > 5.0
        ren.disable_depth_rendering()
        rgb[far] = 255
        return rgb

    def turn_line(scene, p, q):
        connector(scene, mj.mjtGeom.mjGEOM_CAPSULE, 0.0015, p + A.rot(q, np.array([0, 0, half + 0.001])),
                  p + A.rot(q, np.array([R, 0, half + 0.001])), (0.1, 0.1, 0.1, 1))

    def path_dots(scene, k):
        for j in js:
            for p in L["P"][s:k + 1:3, j]:
                A.add_sphere(scene, p, 0.0016, (*COL[L["names"][j]], 1))

    def frame(x, row, qpos, mocap, mocap_q, state, still=None, repeat=1):
        k = int(round(s + x * (e - s)))
        pose = (L["op"][k], L["q"][k])
        now = {j: L["P"][k, j] for j in js if L["lam"][k, j] > A.TOUCH_N}
        # column 1: the recording
        A.pose_recorded(S, datas[0], Z, k)

        def ex0(scene):
            turn_line(scene, Z["obj"][k][:3].astype(float), Z["obj"][k][3:7].astype(float))
            path_dots(scene, k)
            for j, p in now.items():
                A.add_sphere(scene, p, 0.0035, (*COL[L["names"][j]], 1))
        im0 = shot(0, ex0)
        # column 2: the extracted contacts
        datas[1].mocap_pos[0] = pose[0]
        datas[1].mocap_quat[0] = pose[1][[3, 0, 1, 2]]
        mj.mj_forward(mC, datas[1])

        def ex1(scene):
            turn_line(scene, *pose)
            path_dots(scene, k)
            pts = list(now.values())
            for a in range(len(pts)):
                for b in range(a + 1, len(pts)):
                    connector(scene, mj.mjtGeom.mjGEOM_CAPSULE, 0.0008, pts[a], pts[b], (0.2, 0.2, 0.2, 1))
            for j, p in now.items():
                c = (*COL[L["names"][j]], 1)
                A.add_sphere(scene, p, 0.0035, c)
                connector(scene, mj.mjtGeom.mjGEOM_ARROW, 0.004, p - 0.03 * L["lam"][k, j] * L["nrm"][k, j], p, c)
        im1 = shot(1, ex1)
        # column 3: the re-enactment
        d = datas[2]
        d.qpos[:] = qpos
        d.mocap_pos[:] = mocap
        d.mocap_quat[:] = mocap_q
        mj.mj_forward(B["m"], d)

        def ex2(scene):
            turn_line(scene, np.array(row[6:9]), np.array(row[9:13]))
            path_dots(scene, k)
            for kk, fg in enumerate(B["fingers"]):
                c = 13 + 7 * kk
                if row[c + 1] > 0:
                    A.add_sphere(scene, row[c + 3:c + 6], 0.0035, (*COL[fg["fit"]["finger"]], 1))
        im2 = shot(2, ex2)
        im = Image.new("RGB", (Wt, Ht), (255, 255, 255))
        im.paste(Image.fromarray(np.hstack([im0, im1, im2])), (0, HEAD))
        dr = ImageDraw.Draw(im)
        tw_rec = np.degrees(L["turn"][k] - L["turn"][s])
        tw = row[2] - rec["turn0"]
        seconds = [f"log time {L['t'][k]:.2f} s   cylinder turned {tw_rec:+.1f} deg",
                   "triangle joins the contacts; arrows: 30 mm per N",
                   f"{state}   cylinder turned {tw:+.1f} deg"]
        for i in range(3):
            dr.text((i * CW + 16, 12), titles[i], fill=(0, 0, 0), font=font)
            dr.text((i * CW + 16, 48), seconds[i], fill=(0, 0, 0), font=small)
            if i:
                dr.line([(i * CW, HEAD), (i * CW, HEAD + CH)], fill=(200, 200, 200), width=1)
        dr.text((16, HEAD + CH + 6), foot[0], fill=(0, 0, 0), font=small)
        dr.text((16, HEAD + CH + 32), foot[1], fill=(0, 0, 0), font=small)
        dr.text((16, HEAD + CH + 58), foot[2], fill=(0, 0, 0), font=small)
        buf = np.asarray(im).tobytes()
        for _ in range(repeat):
            proc.stdin.write(buf)
        if still:
            im.save(still)
            files.append(still)

    fl = [f for f in r["frames"] if f[0][1] >= 2]
    stroke = [f for f in fl if f[0][1] == 2]
    for n, (row, qpos, mocap, mocap_q) in enumerate(fl):
        part = int(row[1])
        x = float(np.clip((row[0] - A.T_FREE) / stroke_s, 0, 1))
        still = (base + "_stroke_start.png" if n == 0 else base + "_stroke_middle.png" if n == len(stroke) // 2
                 else base + "_stroke_end.png" if n == len(stroke) - 1 else None)
        if STILLS_ONLY and still is None:
            continue
        frame(x, row, qpos, mocap, mocap_q,
              "THE STROKE" if part == 2 else "after the stroke, inputs constant", still=still,
              repeat=FPS if n in (0, len(fl) - 1) else 1)
    for ren in rens:
        ren.close()
    proc.stdin.close()
    proc.wait()
    return files


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", default=os.path.join(ROOT, "..", "ARIA", "Wave_manip"))
    ap.add_argument("--no-film", action="store_true")
    ap.add_argument("--azimuth", type=float, default=None, help="camera azimuth, deg (default: chosen from the fit)")
    ap.add_argument("--elevation", type=float, default=ELEVATION, help="camera elevation, deg")
    ap.add_argument("--tag", default="", help="added to the clips' file names")
    ap.add_argument("--stills-only", action="store_true", help="render the three stills of each clip only")
    args = ap.parse_args()
    globals().update(ELEVATION=args.elevation, STILLS_ONLY=args.stills_only)
    for sub in ("a3h_sandbox", os.path.join("experiment_log", "scripts")):
        sys.path.insert(0, os.path.join(args.wave_manip, sub))
    from w16s1_hora_scout import CYL_URDF_HALFLEN_M, CYL_URDF_RADIUS_M
    os.makedirs(OUT, exist_ok=True)
    L = A.load_recording()
    R, half = CYL_URDF_RADIUS_M * L["scale"], CYL_URDF_HALFLEN_M * L["scale"]
    secs = triangle_sections(L, R, half)
    side = [x for x in secs if x["on_side"]]
    if not side:
        raise SystemExit("no section has all three contacts on the side wall")
    band = [x for x in side if x["off_triangle_deg"] <= side[0]["off_triangle_deg"] + TRIANGLE_BAND_DEG]
    sec = max(band, key=lambda x: abs(x["turn_deg"]))
    fr = A.section_frame(L, sec)
    rep = ["One stroke of a recorded learned rotation, re-enacted by three pneu-net fingers standing on a palm",
           "SIMULATION ONLY. Comparative, not predictive.", "",
           f"Recording: cylinder radius {R * 1e3:.1f} mm, half length {half * 1e3:.1f} mm, mass "
           f"{L['mass'] * 1e3:.1f} g, friction {L['mu']:.2f}; {len(L['t'])} samples over {L['t'][-1]:.1f} s; "
           f"turn about its own axis over the log {np.degrees(L['turn'][-1]):.0f} deg.",
           f"Sections with the same three fingertips touching (normal force over {A.TOUCH_N} N) for at least "
           f"{A.MIN_SECTION_S} s, mean tilt under {A.MAX_TILT_DEG:g} deg, tilt range under "
           f"{A.MAX_TILT_RANGE_DEG:g} deg, centre moving under {A.MAX_DRIFT_M * 1e3:g} mm: {len(secs)}.",
           f"Of those, all three contacts on the side wall (normal within {SIDE_NORMAL_MAX} of square to the axis, "
           f"logged contact height within the half length): {len(side)}.",
           "  section (log s)  fingertips          turn      gaps between contacts round the cylinder (deg)  "
           "furthest gap from 120   contact heights from the centre (mm)   on the side wall"]
    for x in secs:
        rep.append(f"  {x['t0']:.2f} to {x['t1']:.2f}   {'+'.join(L['names'][j] for j in x['fingers']):<19s}"
                   f"{x['turn_deg']:+6.1f} deg   " + "  ".join(f"{g:5.0f}" for g in x["gaps_deg"])
                   + f"                              {x['off_triangle_deg']:5.1f}                "
                   + "  ".join(f"{h:+6.1f}" for h in x["contact_height_mm"])
                   + f"              {'yes' if x['on_side'] else 'no'}")
    rep += [f"Chosen: log {sec['t0']:.2f} to {sec['t1']:.2f} s. Rule: on the side wall; within {TRIANGLE_BAND_DEG:g} deg "
            "of the nearest to an equilateral triangle; of those, the largest turn.", ""]
    print("\n".join(rep), flush=True)
    fts = {rid: G.finger_table(rid) for rid in RECORDS}
    tabs = {rid: tables(fts[rid]) for rid in RECORDS}
    rep += ["Fit of each finger's loaded tip path to the logged contact path (no simulation). The tip load is the "
            f"smaller of the logged mean normal force of that fingertip and {SQUEEZE_SHARE:g} (ASSUMED) of the largest "
            "squeeze the finger can hold steadily on a round object (its stiffness along the surface times the "
            "distance from the axis to the tip's centre). It is taken to act square to the bending plane "
            "(the contact sticks). The bend from the finger's own weight is included.",
            "  finger record   misfit rms per finger (mm)   worst (mm)   gap between finger bodies (mm)   "
            "links below the tip clear of the cylinder by at least (mm)"]
    cands = []
    for rid in RECORDS:
        fits = [fit_finger(L, sec, fr, j, fts[rid], tabs[rid], R, half) for j in sec["fingers"]]
        c = {"rid": rid, "fits": fits, "gap": fingers_gap(fits, fts[rid]),
             "worst": max(f["misfit_rms_m"] for f in fits),
             "clear": min(f["links_below_tip_clear_min_m"] for f in fits)}
        cands.append(c)
        rep.append(f"  {rid:<14s}  " + "  ".join(f"{f['misfit_rms_m'] * 1e3:5.2f}" for f in fits)
                   + f"          {c['worst'] * 1e3:5.2f}        {c['gap'] * 1e3:+6.1f}"
                   f"                           {c['clear'] * 1e3:+6.2f}")
        print(rep[-1], flush=True)
    ok = [c for c in cands if c["gap"] > 0]
    pick = min(ok or cands, key=lambda c: c["worst"])
    rid, fits = pick["rid"], pick["fits"]
    ft = fts[rid]
    rec, _ = A.recorded_measures(L, sec)
    h_plate = min(f["base_height_m"] for f in fits) - MOUNT
    rep += ["", f"Chosen finger record (smallest worst-finger misfit among those whose bodies do not overlap): {rid}. "
            f"Length {ft['L'] * 1e3:.0f} mm, contact radius {ft['rc'] * 1e3:.2f} mm.",
            f"Palm plate: square to the cylinder's mean axis over the section (tilted {sec['tilt_mean_deg']:.1f} deg "
            f"from vertical), {-(h_plate + half) * 1e3:.1f} mm below the cylinder's bottom face; each finger stands on a "
            f"post; the plate is {MOUNT * 1e3:g} mm below the lowest base (ASSUMED, drawn only)."]
    for f in fits:
        rep.append(f"  {f['finger']:<6s}: plane touches the cylinder at {f['plane_touches_at_deg']:+.1f} deg round it, "
                   f"stands {f['plane_stands_out_m'] * 1e3:.1f} mm from the axis at the centre's height (cylinder radius "
                   f"plus contact radius is {(R + ft['rc']) * 1e3:.1f} mm), leans outward {f['plane_leans_out_deg']:.1f} deg; "
                   f"finger leans {f['lean_in_plane_deg']:+.1f} deg within the plane; base {-f['base_height_m'] * 1e3:.1f} mm "
                   f"below the cylinder's centre, post {(f['base_height_m'] - h_plate) * 1e3:.1f} mm; input "
                   f"{f['u0']:.0f} to {f['u1']:.0f} of {ft['u'][-1]:.0f} (simulation units).")
        rep.append(f"          logged contact path {f['path_length_m'] * 1e3:.1f} mm long, {f['path_height_range_m'] * 1e3:.1f} mm "
                   f"in height; misfit rms {f['misfit_rms_m'] * 1e3:.2f} mm, largest {f['misfit_max_m'] * 1e3:.2f} mm; "
                   f"squeeze {f['squeeze_N']:.2f} N (logged {f['logged_normal_force_N']:.2f} N, "
                   f"steady up to {f['squeeze_limit_N']:.2f} N) pushes the tip {f['tip_pushed_out_m'] * 1e3:.1f} mm out of its plane "
                   f"(tip stiffness there {f['k_out_of_plane_N_per_m']:.0f} N/m; along the surface "
                   f"{f['k_along_surface_N_per_m']:.1f} N/m); its own weight moves the tip "
                   f"{f['own_weight_moves_tip_m'] * 1e3:.1f} mm; the logged normal is off square to the plane by "
                   + ", ".join(f"{v:+.0f}" for v in f["normal_off_square_to_plane_deg"])
                   + " deg at the start, middle and end.")
    key = f"log_{sec['t0']:.2f}s_{rid}"
    results = {"recording": rec, "section": sec, "finger_record": rid, "fits": fits, "runs": {},
               "palm_plate_below_cylinder_bottom_m": -(h_plate + half),
               "candidates": [{"record": c["rid"], "worst_misfit_rms_m": c["worst"], "gap_m": c["gap"],
                               "links_clear_m": c["clear"]} for c in cands],
               "sections": secs}
    rep += ["", f"Recorded stroke: cylinder turns {rec['turn_deg']:+.1f} deg in {rec['duration_s']:.2f} s; tilt changes "
            f"by at most {rec['tilt_change_max_deg']:.1f} deg; centre moves at most {rec['centre_moved_max_mm']:.1f} mm."]
    for n, v in rec["fingers"].items():
        rep.append(f"  {n:<6s}: normal force {v['normal_force_mean_N']:.2f} N; contact moves "
                   f"{v['contact_moved_in_world_mm']:.1f} mm in the world, {v['contact_moved_on_object_mm']:.1f} mm on the cylinder.")
    need = L["mass"] * 9.81 / (3 * L["mu"])
    results["squeeze_friction_needs_per_finger_N"] = need
    asked = ", ".join(f"{f['squeeze_N']:.2f}" for f in fits)
    rep += ["", f"Squeeze that friction needs to carry the cylinder's weight, shared evenly by three fingers: "
            f"{need:.3f} N each. The fit asks for {asked} N. The limit of a steady squeeze is our estimate, "
            f"not a measurement."]
    rep += ["", f"Simulated timeline (ASSUMED): fingers brought in {A.T_IN0:g} to {A.T_IN1:g} s with the cylinder held; "
            f"let go at {A.T_SETTLE:g} s; stroke {A.T_FREE:g} to {A.T_STROKE:g} s; inputs constant to {A.T_HOLD:g} s.",
            f"Contact model: elliptic friction cone, impratio {A.IMPRATIO}, time step {A.DT} s. "
            "Contacts are compared with the recording at the same fraction of the stroke's duration."]
    runs, builds = {}, {}
    for name, support, vary, what in A.RUNS:
        B = builds.get(support) or build(L, sec, fr, fits, ft, R, half, key, support)
        builds[support] = B
        r = A.run(B, vary)
        ms = A.arrangement_measures(L, sec, B, r, rec)
        runs[name] = (B, r, what)
        results["runs"][name] = ms
        np.savetxt(os.path.join(OUT, f"log_{key}_{name}.csv"), r["log"], delimiter=",", fmt="%.6g",
                   header=A.LOG_HEADER, comments="")
        rep += ["", f"Arrangement, {name.replace('_', ' ')} ({what}): "
                + (f"STOPPED, {r['stopped'][0]} at {r['stopped'][1]:.2f} s" if r["stopped"] else "ran to the end")
                + f"; contacts before the fingers are brought in: {ms['contacts_before_fingers_brought_in']}.",
                f"  cylinder turn: {ms['turn_let_go_deg']:+.2f} deg in the {A.T_FREE - A.T_SETTLE:g} s after being let go, "
                f"{ms['turn_stroke_deg']:+.2f} deg in the stroke, {ms['turn_final_hold_deg']:+.2f} deg in the final "
                f"{A.T_HOLD - A.T_STROKE:g} s (recorded stroke {rec['turn_deg']:+.1f} deg)."]
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
                       f"mean {v['normal_force_mean_N']:.2f} N ({v['normal_force_start_end_N'][0]:.2f} N at the start, "
                       f"{v['normal_force_start_end_N'][1]:.2f} N at the end; recorded "
                       f"{rec['fingers'][n]['normal_force_mean_N']:.2f} N); contact moved "
                       f"{v['contact_moved_in_world_mm']:.1f} mm in the world (recorded "
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
    if not args.no_film:
        n0 = len(rep)
        # camera: look in through the widest gap between the fingers
        az = np.sort(np.mod([f["plane_touches_at_deg"] for f in fits], 360))
        gaps = np.diff(np.concatenate([az, [az[0] + 360]]))
        i = int(np.argmax(gaps))
        mid = np.radians(az[i] + gaps[i] / 2)
        v = np.cos(mid) * fr["e1"] + np.sin(mid) * fr["e2"]      # from the axis out through that gap
        azimuth = args.azimuth if args.azimuth is not None else float(np.degrees(np.arctan2(-v[1], -v[0])))
        S = A.recorded_scene(args.wave_manip, L, R, half)
        rep += ["", f"Clips: three columns of {CW} x {CH}, same camera (azimuth {azimuth:.0f} deg, elevation {ELEVATION:.0f} deg, looking in through "
                "the widest gap between the pneu-net fingers)."]
        for name, (B, r, what) in runs.items():
            if name not in ("weight_carried", "cylinder_free"):
                continue
            lg = r["log"]
            i2 = np.where(lg[:, 1] >= 2)[0]
            if not len(i2):
                rep.append(f"  no clip for {name}: the run stopped before the stroke.")
                continue
            rec["turn0"] = float(lg[i2[0], 2])
            fs = film(L, sec, fr, B, r, S, rec, R, half,
                      os.path.join(OUT, f"recorded_contacts_reenactment_{name}{args.tag}"), what, azimuth)
            rep += [f"  wrote {f}" for f in fs]
        save()
        print("\n".join(rep[n0:]))


if __name__ == "__main__":
    main()
