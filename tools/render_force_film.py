"""Short film: the forces a manipulation needs, split into normal and shear for each finger,
saved as snapshots.

    python tools/render_force_film.py hora --wave-manip <Wave_manip folder> --out hora.mp4
    python tools/render_force_film.py hora ... --stills 0 2 5   (test pictures, no video)

The Hora part replays data/rl_logs/hora_seed42_30s*.npz (tools/hora_physics_log.py,
tools/hora_force_split.py) on the Allegro model, from the log's own joint angles. No physics
is stepped. Every number on screen is read from those files.

The human part plays the recorded video of a person turning a ball. At each snapshot it stops, and
draws on the frozen frame the tracked hand and ball and the forces derived from the ball's motion.
A hand from a stored follow result then closes onto the snapshots. follow.py checks each frozen
frame on its own: no hand is simulated doing the task. Hands are drawn as the models' own meshes,
in the check's schematic poses.

With --checks <folder> (a tools/position_turning_check.py result) the human part sets no per-finger
force targets. At each snapshot it draws the tracked hand and ball, the ball's measured turn and
where the fingers touch, then shows the fitted hand's position check and turning check.

    python tools/render_force_film.py human --video --wave-manip <Wave_manip folder> --runs follow_shadow.json ... --out human.mp4
    (without --video: the earlier cut, the palm-frame wireframe playing live)

Needs mujoco, numpy, PIL and ffmpeg on PATH.
"""
import argparse, json, math, os, subprocess, sys, textwrap
import numpy as np, mujoco as mj
from PIL import Image, ImageDraw, ImageFont
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from render_force_snapshots import add, link, View, shoot, Rig, RGB

BLUE = (.06, .36, .78, 1); ORANGE = (.93, .42, .05, 1)
BLUE8 = (15, 92, 199); ORANGE8 = (225, 100, 10)
BG = (250, 250, 248); INK = (30, 30, 30); SOFT = (105, 108, 112); DISC = (232, 234, 238); RIM = (150, 154, 160)
W, H, P = 1600, 900, 900
FOV = 30
SMOOTH_S = .1       # arrows and panel show forces averaged over this window
SHOW_MIN_N = .02    # below this mean normal force a finger is drawn as lifted off


def font(name, size): return ImageFont.truetype(name, size)
BIG, MID, SMALL, BOLD = font("arialbd.ttf", 36), font("arial.ttf", 25), font("arial.ttf", 19), font("arialbd.ttf", 24)


class Forces:
    """One finger set's forces over time, as a sequence of per-frame means."""
    def __init__(s, t, names, normal, nrm_dir, point, shear, mu, axis):
        s.t, s.names, s.normal, s.nrm, s.point, s.shear, s.mu, s.axis = t, names, normal, nrm_dir, point, shear, mu, axis

    def window(s, tc):
        """Mean force vectors about time tc: lifted-off steps count as zero force."""
        m = np.abs(s.t - tc) <= SMOOTH_S / 2; out = []
        if not m.any(): m = np.arange(len(s.t)) == np.argmin(np.abs(s.t - tc))   # a gap in the recording: hold the nearest frame
        for j in range(len(s.names)):
            lam = s.normal[m, j]; on = lam > 0
            if lam.mean() < SHOW_MIN_N or not on.any(): out.append(None); continue
            n = (lam[:, None] * s.nrm[m, j]).mean(0); nl = np.linalg.norm(n)
            out.append(dict(point=s.point[m, j][on].mean(0), normal=n, shear=s.shear[m, j].mean(0), press=float(nl), dir=n / nl))
        return out

    def exact(s, k):
        """The forces at frame k itself, unaveraged: the targets the check was given at that frame."""
        out = []
        for j in range(len(s.names)):
            lam = float(s.normal[k, j])
            if lam < SHOW_MIN_N: out.append(None); continue
            out.append(dict(point=s.point[k, j].copy(), normal=lam * s.nrm[k, j], shear=s.shear[k, j].copy(), press=lam, dir=s.nrm[k, j].copy()))
        return out

    def plane(s, f):
        """Shear in the contact's own plane: x along the turn, y along the turning axis."""
        r = f["point"] - s.centre_at; along = np.cross(s.axis, r); along -= (along @ f["dir"]) * f["dir"]; along /= np.linalg.norm(along)
        return float(f["shear"] @ along), float(f["shear"] @ np.cross(f["dir"], along))


class Panel:
    """The right of the frame: heading, key, a friction disc per finger, snapshot tiles."""
    def __init__(c, names, px_per_N):
        c.names, c.k = names, px_per_N; c.tiles = []

    def disc(c, dr, x, y, h, name, f, mu, plane):
        rmax = min(88, h / 2 - 3); cx, cy = x + 8 + rmax, y + h / 2; tx = cx + rmax + 18
        dr.text((tx, cy - 54), name.capitalize(), fill=INK, font=BOLD)
        if f is None:
            dr.text((tx, cy - 24), "Lifted off", fill=SOFT, font=SMALL)
            dr.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], fill=RIM); return
        r = max(3, min(rmax, mu * f["press"] * c.k))
        dr.ellipse([cx - r, cy - r, cx + r, cy + r], fill=DISC, outline=RIM, width=2)
        sx, sy = plane(f); L = math.hypot(sx, sy) * c.k; cut = min(1, rmax / L) if L else 1
        ex, ey = cx + sx * c.k * cut, cy - sy * c.k * cut
        dr.line([cx, cy, ex, ey], fill=ORANGE8, width=5)
        L = math.hypot(ex - cx, ey - cy)
        if L > 6:
            ux, uy = (ex - cx) / L, (ey - cy) / L
            dr.polygon([(ex + ux * 8, ey + uy * 8), (ex - uy * 7, ey + ux * 7), (ex + uy * 7, ey - ux * 7)], fill=ORANGE8)
        dr.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], fill=BLUE8)
        sh = float(np.linalg.norm(f["shear"]))
        dr.text((tx, cy - 24), f"Normal {f['press']:.2f} N", fill=BLUE8, font=SMALL)
        dr.text((tx, cy + 2), f"Shear {sh:.2f} N", fill=ORANGE8, font=SMALL)
        dr.text((tx, cy + 28), f"{100 * sh / (mu * f['press']):.0f}% of limit", fill=SOFT, font=SMALL)

    def frame(c, pic, head, lines, fingers, mu, plane, tag, note=None, active=None, footer="Simulated: comparative, not predictive.", inset=None, borders=None, strips=None, marks=None, bars=None):
        im = Image.new("RGB", (W, H), BG); im.paste(Image.fromarray(pic), (0, 0)); dr = ImageDraw.Draw(im)
        if inset is not None: im.paste(inset, (12, P - inset.height - 12))
        x, y = P + 20, 30
        for ln in textwrap.wrap(head, 36):
            dr.text((x, y), ln, fill=INK, font=BIG); y += 44
        y += 6
        for ln in lines:
            col = None
            if isinstance(ln, tuple): col, ln = ln
            if col:
                dr.rectangle([x, y + 5, x + 16, y + 21], fill=col)
            for w in textwrap.wrap(ln, 62):
                dr.text((x + (26 if col else 0), y), w, fill=INK, font=SMALL); y += 26
        if strips:
            y += 18
            for name, res in strips:
                cnt = {v: sum(f["verdict"] == v for f in res) for v in RGB}
                dr.text((x, y), f"{name}: {cnt['green']} green, {cnt['amber']} amber, {cnt['red']} red of {len(res)}", fill=INK, font=SMALL); y += 26
                tw = max(2, min(13, (W - x - 30) // len(res) - 1))
                for i, f in enumerate(res):
                    dr.rectangle([x + i * (tw + 1), y, x + i * (tw + 1) + tw - 1, y + 24], fill=RGB[f["verdict"]])
                y += 42
        elif bars:
            y += 14
            for name, rows_ in bars:
                dr.text((x, y), name, fill=INK, font=BOLD); y += 30
                for what_, res in rows_:
                    cnt = {v: res.count(v) for v in RGB}
                    dr.text((x, y), f"{what_}: {cnt['green']} green, {cnt['amber']} amber, {cnt['red']} red of {len(res)}", fill=INK, font=SMALL); y += 24
                    tw = max(2, min(13, (W - x - 30) // len(res) - 1))
                    for i, v_ in enumerate(res):
                        dr.rectangle([x + i * (tw + 1), y, x + i * (tw + 1) + tw - 1, y + 16], fill=RGB[v_])
                    y += 24
                y += 8
        elif fingers is not None:
            n = len(c.names); rows = (n + 1) // 2; top = y + 6; h = (660 - top) // rows
            for i, (nm, f) in enumerate(zip(c.names, fingers)):
                c.disc(dr, x + (i % 2) * 335, top + (i // 2) * h, h, nm, f, mu, plane)
        if note:
            ls_ = textwrap.wrap(note, 52)
            for i_, w_ in enumerate(ls_): dr.text((x, 664 - 29 * (len(ls_) - 1 - i_)), w_, fill=INK, font=BOLD)
        ty, tw = 702, 118
        for i, t in enumerate(c.tiles):
            tx = x + i * (tw + 8); im.paste(t, (tx, ty))
            bc = (borders or {}).get(i)
            if bc or active == i: dr.rectangle([tx - 3, ty - 3, tx + tw + 2, ty + tw + 2], outline=bc or INK, width=4)
            if marks and i in marks:
                for j_, col_ in enumerate(marks[i]):
                    dr.rectangle([tx + j_ * (tw // 2 + 1), ty + tw + 5, tx + j_ * (tw // 2 + 1) + tw // 2 - 1, ty + tw + 21], fill=col_)
        dr.text((x, H - 46), footer, fill=SOFT, font=SMALL)
        if tag:
            b = dr.textbbox((20, 18), tag, font=MID); dr.rectangle([b[0] - 8, b[1] - 6, b[2] + 8, b[3] + 6], fill=BG); dr.text((20, 18), tag, fill=INK, font=MID)
        return np.asarray(im, np.uint8)


class HoraScene:
    """The Allegro model and cylinder from Wave_manip's replay, posed from the log's joint angles."""
    def __init__(s, wave_manip, az, el, dist, look_below, alpha):
        for sub in ("a3h_sandbox", os.path.join("experiment_log", "scripts")):
            sys.path.insert(0, os.path.join(wave_manip, sub))
        from w16s1_hora_scout import CYL_URDF_HALFLEN_M, CYL_URDF_RADIUS_M, URDF_INTERNAL, FINGERS, ig_to_mj_body, tip_pad_from_urdf
        from w16s2_driver import fit_initial_pose
        from w16s4_replay_hora import build_replay_scene
        d = os.path.join(ROOT, "data", "rl_logs")
        s.T = np.load(os.path.join(d, "hora_seed42_30s.npz"), allow_pickle=True)
        s.Z = np.load(os.path.join(d, "hora_seed42_30s_physics.npz"), allow_pickle=True)
        s.Fz = np.load(os.path.join(d, "hora_seed42_30s_forces.npz"), allow_pickle=True)
        scale = float(s.T["obj_scale"]); s.R = CYL_URDF_RADIUS_M * scale; s.half = CYL_URDF_HALFLEN_M * scale
        palm_pos, palm_quat, q0, _ = fit_initial_pose(s.T, 0.0, s.R, s.half)
        s.m = build_replay_scene(s.R, s.half, palm_pos, palm_quat, tip_pad_from_urdf(URDF_INTERNAL), present=True)
        s.m.vis.global_.offwidth = W; s.m.vis.global_.offheight = H; s.m.vis.global_.fovy = FOV
        s.m.vis.headlight.active = 1; s.m.vis.headlight.ambient[:] = .5; s.m.vis.headlight.diffuse[:] = .55; s.m.vis.headlight.specular[:] = 0
        obj_body = s.m.jnt_bodyid[mj.mj_name2id(s.m, mj.mjtObj.mjOBJ_JOINT, "obj_free")]
        s.obj_geoms = [g for g in range(s.m.ngeom) if s.m.geom_bodyid[g] == obj_body]; s.alpha = alpha
        s.d = mj.MjData(s.m)
        s.adr = []
        for nm in s.Z["dof_names"]:
            i = int(str(nm).split("_")[1].split(".")[0])
            s.adr.append(s.m.jnt_qposadr[mj.mj_name2id(s.m, mj.mjtObj.mjOBJ_JOINT, f"{FINGERS[i // 4]}j{i % 4}")])
        s.oadr = s.m.jnt_qposadr[mj.mj_name2id(s.m, mj.mjtObj.mjOBJ_JOINT, "obj_free")]
        s.pairs = []
        for i, n in enumerate(s.Z["body_names"]):
            b = mj.mj_name2id(s.m, mj.mjtObj.mjOBJ_BODY, ig_to_mj_body(str(n)))
            if b > 0 and "base_link" not in str(n): s.pairs.append((b, i))
        s.r = mj.Renderer(s.m, P, P); s.cam = mj.MjvCamera(); s.hide = mj.MjvOption(); s.hide.geomgroup[:] = 0
        c0 = s.Fz["obj_pos"].mean(0)
        s.v = View(c0 - [0, 0, look_below], az, el, dist)

    def pose(s, k):
        s.d.qpos[s.adr] = s.Z["q"][k]
        o = s.Z["obj"][k]; s.d.qpos[s.oadr:s.oadr + 3] = o[:3]; s.d.qpos[s.oadr + 3:s.oadr + 7] = [o[6], o[3], o[4], o[5]]
        mj.mj_kinematics(s.m, s.d)

    def residual(s, ks):
        e = []
        for k in ks:
            s.pose(k); e += [np.linalg.norm(s.d.xpos[b] - s.Z["body"][k, i, :3]) for b, i in s.pairs]
        return float(np.median(e)), float(np.max(e))

    def picture(s, k, fingers, arrow):
        # The object is mixed over a picture taken without it, so fingers behind it stay visible.
        s.pose(k)
        s.m.geom_rgba[s.obj_geoms, 3] = 1; s.r.update_scene(s.d, s.cam); s.v.apply(s.r.scene); full, fd = shoot(s.r)
        s.m.geom_rgba[s.obj_geoms, 3] = 0; s.r.update_scene(s.d, s.cam); s.v.apply(s.r.scene); rgb, dep = shoot(s.r)
        s.m.geom_rgba[s.obj_geoms, 3] = 1
        rgb = rgb.copy(); rgb[np.isinf(dep)] = BG; o = np.isfinite(fd) & (fd < dep - 1e-5)
        rgb[o] = (s.alpha * full[o] + (1 - s.alpha) * rgb[o]).astype(np.uint8)
        s.r.update_scene(s.d, s.cam, s.hide); g = s.r.scene
        for f in fingers:
            if f is None: continue
            link(g, mj.mjtGeom.mjGEOM_ARROW, .0024, f["point"], f["point"] + arrow * f["normal"], BLUE)
            if np.linalg.norm(f["shear"]) * arrow > .002:
                link(g, mj.mjtGeom.mjGEOM_ARROW, .0024, f["point"], f["point"] + arrow * f["shear"], ORANGE)
        s.v.apply(g); top, td = shoot(s.r); m = np.isfinite(td); rgb[m] = top[m]
        return rgb


class HoraFitStage:
    """The frozen cylinder with another hand drawn round it; also what Rig needs of a stage."""
    def __init__(s, scene, L):
        s.sc, s.L, s.W, s.H, s.v = scene, L, P, P, scene.v; s.c0 = np.array(L["frames"][0]["obj_pos"])
        for g in range(scene.m.ngeom):
            if g not in scene.obj_geoms: scene.m.geom_matid[g] = -1; scene.m.geom_rgba[g, 3] = 0

    def inside(s, k):
        """Points in the log's frame to depth inside the cylinder at step k, positive inside."""
        c = s.sc; g = [i for i in c.obj_geoms if c.m.geom_type[i] == mj.mjtGeom.mjGEOM_CYLINDER][0]; r, h = c.m.geom_size[g][:2]
        c.pose(k); o, M = c.d.geom_xpos[g].copy(), c.d.geom_xmat[g].reshape(3, 3).copy()
        def depth(X):
            x = (X - o) @ M; return np.minimum(r - np.hypot(x[:, 0], x[:, 1]), h - np.abs(x[:, 2]))
        return depth

    def compose(s, k, hand, fingers, arrow, pads, missing):
        c = s.sc; c.pose(k); c.r.update_scene(c.d, c.cam); s.v.apply(c.r.scene); obj, od = shoot(c.r)
        rgb, dep = hand[0].copy(), hand[1]; rgb[np.isinf(dep)] = BG
        o = np.isfinite(od) & (od < dep - 1e-5); rgb[o] = (c.alpha * obj[o] + (1 - c.alpha) * rgb[o]).astype(np.uint8)
        c.r.update_scene(c.d, c.cam, c.hide); g = c.r.scene
        for f in fingers:
            if f is None: continue
            link(g, mj.mjtGeom.mjGEOM_ARROW, .0024, f["point"], f["point"] + arrow * f["normal"], BLUE)
            if np.linalg.norm(f["shear"]) * arrow > .002:
                link(g, mj.mjtGeom.mjGEOM_ARROW, .0024, f["point"], f["point"] + arrow * f["shear"], ORANGE)
        for p_, col in pads: add(g, mj.mjtGeom.mjGEOM_SPHERE, [.0048, 0, 0], p_, col)
        for p_ in missing: add(g, mj.mjtGeom.mjGEOM_SPHERE, [.0062, 0, 0], p_, (.82, .1, .1, 1))
        s.v.apply(g); top, td = shoot(c.r); m = np.isfinite(td); rgb[m] = top[m]
        return rgb


LABEL = {"leap": "LEAP", "allegro": "Allegro", "shadow": "Shadow"}
TURN = (.55, .12, .71, 1); TURN8 = (140, 30, 180)


def turn_path(c, om, R, fwd):
    """The turn as an axis through the ball and an arc round it, drawn the way the ball goes (right-hand rule about om).
    The arc ends where a viewer looking along fwd sees its motion most clearly."""
    a = np.array(om, float) / np.linalg.norm(om); h = [0, 0, 1.] if abs(a[2]) < .9 else [1., 0, 0]
    e1 = np.cross(a, h); e1 /= np.linalg.norm(e1); e2 = np.cross(a, e1); rho = 1.3 * R
    th = np.linspace(0, 2 * np.pi, 72, endpoint=False)
    tang = -np.outer(np.sin(th), e1) + np.outer(np.cos(th), e2); rad = np.outer(np.cos(th), e1) + np.outer(np.sin(th), e2)
    seen = np.linalg.norm(tang - np.outer(tang @ fwd, fwd), axis=1) - .3 * (rad @ fwd)
    th_ = th[int(np.argmax(seen))] - np.linspace(1.5 * np.pi, 0, 55)
    return c - 1.7 * R * a, c + 1.7 * R * a, c + rho * (np.outer(np.cos(th_), e1) + np.outer(np.sin(th_), e2))


def turn_colour(x):
    """The turning check's colour: red, the touching fingers cannot hold the ball with torque purely about the turn;
    amber, only with torque against it; green, with torque to spare along it."""
    if x["on_axis"] == "ok": return "green" if x["spare_on_axis_mNm"] > 0 else "amber"
    assert x["on_axis"] == "cannot", x["on_axis"]
    return "red"


class Video:
    def __init__(v, path, fps):
        v.p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(fps), "-i", "-",
                                "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", path], stdin=subprocess.PIPE)
        v.n = 0

    def put(v, a):
        v.p.stdin.write(a.tobytes()); v.n += 1
        if v.n % 100 == 0: print("frames", v.n, flush=True)

    def close(v):
        v.p.stdin.close(); v.p.wait()


def hora(A):
    sc = HoraScene(A.wave_manip, A.az, A.el, A.dist, A.look_below, A.alpha); Z = sc.Fz
    t = Z["t"]; names = [str(x) for x in Z["fingers"]]; mu = float(Z["mu"])
    spin = Z["obj_angvel"].mean(0); axis = spin / np.linalg.norm(spin)
    fr = Forces(t, names, Z["normal_N"], Z["normal_dir"], Z["point"], Z["shear_N"], mu, axis)
    fps, t0, dur = A.fps, A.start, A.seconds
    med, mx = sc.residual(np.arange(int(t0 / t[1]), int((t0 + dur) / t[1]), 24))
    rate = -np.degrees(Z["obj_angvel"][(t >= t0) & (t <= t0 + dur), 2].mean())
    print(f"log joint angles on the model: link position error median {med * 1e3:.2f} mm, max {mx * 1e3:.2f} mm; turning {rate:.1f} deg/s", flush=True)
    seq = [fr.window(t0 + j / fps) for j in range(int(dur * fps) + 1)]
    press = [f["press"] for w in seq for f in w if f]; shear = [np.linalg.norm(f["shear"]) for w in seq for f in w if f]
    px = 88 / (mu * max(press)); arrow = A.arrow_R * sc.R / max(press)
    print(f"over this window: longest normal {max(press):.2f} N, longest shear {max(shear):.2f} N; arrows {arrow * 1e3:.0f} mm per N; normal p98 {np.percentile(press, 98):.2f} N", flush=True)
    pan = Panel(names, px)
    head = "What the turning asks of each finger"
    lines = [f"Hora, a published learned policy on an Allegro hand, turning a cylinder in simulation at {abs(rate):.0f} degrees per second. Replayed from its log.",
             (BLUE8, "Normal force: read from the simulator's contacts."),
             (ORANGE8, "Shear: derived from the object's motion and shared between fingers. One consistent split, not the only one."),
             "Each disc is that finger's friction limit. Right is the way the object turns."]
    def plane(f):
        fr.centre_at = sc.Z["obj"][np.searchsorted(t, cur[0]), :3]; return fr.plane(f)
    cur = [t0]
    if A.stills:
        for tt in A.stills:
            cur[0] = t0 + tt; w = fr.window(cur[0]); k = int(np.searchsorted(t, cur[0]))
            pic = sc.picture(k, w, arrow)
            Image.fromarray(pan.frame(pic, head, lines, w, mu, plane, f"Hora replay, t = {tt:.1f} s")).save(f"{A.out}_{tt:g}.png")
        print("stills written"); os._exit(0)
    vid = Video(A.out, fps); saved = []; note = None; snaps = list(np.arange(A.every, dur + 1e-6, A.every)); snap_t = [t0 + m for m in snaps]
    if A.fit_only:
        for tc in snap_t:
            cur[0] = tc; pic = sc.picture(int(np.searchsorted(t, tc)), fr.window(tc), arrow); cut = int(P * .18)
            pan.tiles.append(Image.fromarray(pic[cut // 2:P - cut - cut // 2, cut:P - cut]).resize((118, 118), Image.LANCZOS))
    for j in range(0 if A.fit_only else int(dur * fps) + 1):
        tt = j / fps; cur[0] = t0 + tt; w = seq[j]; k = int(np.searchsorted(t, cur[0])); pic = sc.picture(k, w, arrow)
        tag = f"Hora replay, t = {tt:.1f} s"
        if snaps and tt >= snaps[0] - 1e-6:
            snaps.pop(0); cut = int(P * .18)
            pan.tiles.append(Image.fromarray(pic[cut // 2:P - cut - cut // 2, cut:P - cut]).resize((118, 118), Image.LANCZOS))
            note = f"Snapshot {len(pan.tiles)} saved: positions and forces, as targets."
            saved.append(dict(t_s=round(float(cur[0]), 4), obj_pos=sc.Z["obj"][k, :3].tolist(), obj_quat_xyzw=sc.Z["obj"][k, 3:7].tolist(), fingers={
                n: None if f is None else dict(point_m=f["point"].tolist(), normal_N=f["normal"].tolist(), shear_N=f["shear"].tolist()) for n, f in zip(names, w)}))
            for _ in range(int(A.hold * fps)): vid.put(pan.frame(pic, head, lines, w, mu, plane, tag, note, len(pan.tiles) - 1))
            continue
        vid.put(pan.frame(pic, head, lines, w, mu, plane, tag, note))
    for _ in range(0 if A.fit_only else int(A.end * fps)):
        vid.put(pan.frame(pic, head, lines, w, mu, plane, tag, f"{len(pan.tiles)} snapshots: what a hand must supply."))
    if A.run:
        # Another hand against the five frozen snapshots, a moment each.
        L = json.load(open(A.leader_json, encoding="utf-8")); sc.v = View(sc.v.pos + A.dist * sc.v.fwd, A.az if A.fit_az is None else A.fit_az, A.el if A.fit_el is None else A.fit_el, A.fit_dist); stage = HoraFitStage(sc, L); r = Rig(A.run, stage, full=not A.sticks)
        assert np.allclose([f["t"] for f in L["frames"]], snap_t, atol=1e-3), "the run is for different snapshots"
        nm = LABEL.get(r.D["hand"], r.D["hand"]); borders = {}; close = int(A.fit_close * fps); total = int(A.fit_seconds * fps)
        what = ["Each snapshot is checked on its own, frozen, against the targets just saved. No hand is simulated doing the turn.",
                "Each dot is a finger's result: green passes, amber is close, red does not pass by this method at this pose." + (" A larger red dot marks a contact with no finger." if any(f.get("unserved") for f in r.D["frames"]) else ""),
                "Discs: the targets, named by Hora's fingers.",
                "The pose is schematic. Each contact is tested on the cylinder's surface there, but the rest of the hand only on a sphere, and not against itself, so parts can pass through the cylinder or each other."]
        print(nm, "at snapshots:", [f["verdict"] for f in r.D["frames"]], flush=True)
        if r.full:
            for i, tc in enumerate(snap_t):
                dep, own = r.overlaps(i, stage.inside(int(np.searchsorted(t, tc))))
                print(f"  snapshot {i + 1}: deepest in the cylinder, mm: " + ", ".join(f"{a} {b * 1e3:.1f}" for a, b in dep.items()) + f"; hand into itself: {own and (own[0], own[1], round(own[2] * 1e3, 1))}", flush=True)
        if A.fit_stills:
            for i, tc in enumerate(snap_t):
                cur[0] = tc; w = fr.window(tc); k = int(np.searchsorted(t, tc)); lay, pads, f_ = r.layer(i, 1.0)
                im = stage.compose(k, lay, w, arrow, pads, [])
                Image.fromarray(pan.frame(im, f"Can {nm} supply these targets?", what, w, mu, plane, f"Snapshot {i + 1}, frozen", f"Snapshot {i + 1}: {f_['verdict']}.", i)).save(f"{A.out}_fit{i + 1}.png")
            print("stills written", flush=True); os._exit(0)
        for i, tc in enumerate(snap_t):
            cur[0] = tc; w = fr.window(tc); k = int(np.searchsorted(t, tc)); fi = L["frames"][i]
            for q_ in range(total):
                a = min(1.0, q_ / max(1, close)); a = a * a * (3 - 2 * a)
                lay, pads, f_ = r.layer(i, a)
                miss = [np.array(fi["contact_point"][u]) for u in f_.get("unserved", []) if fi["contact_point"][u]] if a >= .999 else []
                im = stage.compose(k, lay, w, arrow, pads, miss)
                if a >= .999: borders[i] = RGB[f_["verdict"]]
                vid.put(pan.frame(im, f"Can {nm} supply these targets?", what, w, mu, plane, f"Snapshot {i + 1}, frozen", f"Snapshot {i + 1}: {f_['verdict']}." if a >= .999 else f"Snapshot {i + 1}: fitting.", i, borders=borders))
    vid.close()
    if not A.fit_only:
        json.dump(dict(source="data/rl_logs/hora_seed42_30s_forces.npz", averaged_over_s=SMOOTH_S, snapshots=saved), open(os.path.splitext(A.out)[0] + "_snapshots.json", "w"), indent=1)
    print("wrote", A.out, "frames", vid.n, "seconds", round(vid.n / fps, 1), flush=True)
    os._exit(0)   # GL contexts can hang on exit


def kabsch(a, b):
    ac, bc = a.mean(0), b.mean(0); U, _, Vt = np.linalg.svd((a - ac).T @ (b - bc))
    return Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T


def qmat(q):
    m = np.zeros(9); mj.mju_quat2Mat(m, np.array(q, float)); return m.reshape(3, 3)


def spin_in_palm(F, B):
    """The ball's orientation in the palm frame. The board-frame leader has it in the board's
    frame; the fit between the two files' points at each frame gives the palm's rotation."""
    by_t = {round(b["t"], 5): b for b in B}; Rob = []
    for f in F:
        b = by_t[round(f["t"], 5)]
        ok = [i for i in range(len(f["station_pos"])) if f["station_pos"][i] is not None and b["station_pos"][i] is not None]
        R = kabsch(np.array([b["station_pos"][i] for i in ok] + [b["obj_pos"]]), np.array([f["station_pos"][i] for i in ok] + [f["obj_pos"]]))
        Rob.append(R @ qmat(b["obj_quat"]))
        q = np.zeros(4); mj.mju_mat2Quat(q, Rob[-1].ravel()); f["obj_quat"] = q.tolist()
    return np.array(Rob)


BONES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20)]
BONE = (.45, .4, .36, 1)


class HumanStage:
    """The ball and the person's hand in the palm frame; also what Rig needs of a stage."""
    def __init__(s, L, az, el, dist, alpha, look_back):
        s.L, s.F, s.W, s.H, s.alpha = L, L["frames"], P, P, alpha
        s.R = L["object"]["radius_m"]; s.c0 = np.array(s.F[0]["obj_pos"]); s.n = L["n_stations"]
        s.look = np.median([f["obj_pos"] for f in s.F], axis=0) - [0, look_back, 0]
        s.v = View(s.look, az, el, dist)
        xml = f"""<mujoco><visual><global offwidth="{P}" offheight="{P}" fovy="{FOV}"/>
        <headlight ambient=".5 .5 .5" diffuse=".55 .55 .55" specular="0 0 0"/></visual>
        <asset><texture name="t" type="cube" builtin="checker" rgb1=".88 .92 .38" rgb2=".66 .74 .2" width="256" height="256"/>
        <material name="m" texture="t"/></asset><worldbody><body mocap="true">
        <geom type="sphere" size="{s.R}" material="m" contype="0" conaffinity="0"/></body></worldbody></mujoco>"""
        s.m = mj.MjModel.from_xml_string(xml); s.d = mj.MjData(s.m); s.r = mj.Renderer(s.m, P, P); s.cam = mj.MjvCamera()
        s.hide = mj.MjvOption(); s.hide.geomgroup[:] = 0

    def ball(s, k):
        f = s.F[k]; s.d.mocap_pos[0] = f["obj_pos"]; s.d.mocap_quat[0] = f["obj_quat"]; mj.mj_forward(s.m, s.d)
        s.r.update_scene(s.d, s.cam); s.v.apply(s.r.scene); return shoot(s.r)

    def person(s, k):
        s.r.update_scene(s.d, s.cam, s.hide); g = s.r.scene; sk = s.F[k]["skeleton"]
        for a, b in BONES:
            if sk[a] is not None and sk[b] is not None: link(g, mj.mjtGeom.mjGEOM_CAPSULE, .0052, sk[a], sk[b], BONE)
        for p in sk:
            if p is not None: add(g, mj.mjtGeom.mjGEOM_SPHERE, [.0072, 0, 0], p, BONE)
        s.v.apply(g); return shoot(s.r)

    def overlay(s, fingers, arrow, cap, pads=(), missing=(), turn=None):
        s.r.update_scene(s.d, s.cam, s.hide); g = s.r.scene
        if turn is not None:
            # MuJoCo 3.6 draws an arrow at half the length it is given (a capsule draws in full), so the axis is
            # given twice its length to run through the whole ball, as in the video. Other arrows keep their look.
            a_, b_, arc = turn; link(g, mj.mjtGeom.mjGEOM_ARROW, .0022, a_, a_ + 2 * (np.array(b_) - a_), TURN)
            for u, w in zip(arc[:-2], arc[1:-1]): link(g, mj.mjtGeom.mjGEOM_CAPSULE, .0016, u, w, TURN)
            link(g, mj.mjtGeom.mjGEOM_ARROW, .0026, arc[-6], arc[-1], TURN)
        def draw(p, v, col):
            L = np.linalg.norm(v)
            if L * arrow > .002: link(g, mj.mjtGeom.mjGEOM_ARROW, .0024, p, p + arrow * v * min(1, cap / L), col)
        for f in fingers:
            if f is not None:
                draw(f["point"], f["normal"], BLUE); draw(f["point"], f["shear"], ORANGE)
        for p, c in pads: add(g, mj.mjtGeom.mjGEOM_SPHERE, [.0048, 0, 0], p, c)
        for p in missing: add(g, mj.mjtGeom.mjGEOM_SPHERE, [.0062, 0, 0], p, (.82, .1, .1, 1))
        s.v.apply(g); return shoot(s.r)

    def compose(s, k, hand_layers, top, turn=None):
        ball, bd = s.ball(k); rgb = np.full((P, P, 3), BG, np.uint8); dep = np.full((P, P), np.inf)
        for c, d in hand_layers:
            m = d < dep; rgb[m] = c[m]; dep[m] = d[m]
        o = np.isfinite(bd) & (bd < dep)
        rgb[o] = (s.alpha * ball[o] + (1 - s.alpha) * rgb[o]).astype(np.uint8)
        if turn is not None:    # the turn's axis and arc: full where nothing is in front, faint where the ball or hand hides them
            m = np.isfinite(turn[1]); near = m & (turn[1] < np.minimum(dep, bd))
            rgb[m] = (.3 * turn[0][m] + .7 * rgb[m]).astype(np.uint8); rgb[near] = turn[0][near]
        m = np.isfinite(top[1]); rgb[m] = top[0][m]
        return rgb


class Footage:
    """The recorded take: phone video, cameras, tracked joints, from Wave_manip's human_demo folder."""
    DIR = ("data_ext/takes_2026-10-23/phone_tennis_new_cup_20261023_150117.mp4", "derived/takes_2026-10-23/tennis_new_cup/take3_tennis_ball_sam2_rfree.npz",
           "derived/leaders/tennis_new_cup_take3_cameras.json", "derived/leaders/tennis_new_cup_take3_keypoints.json")

    def __init__(s, wave_manip):
        import cv2; s.cv = cv2
        d = os.path.join(wave_manip, "human_demo"); clip, rows, cams, kp = [os.path.join(d, x) for x in s.DIR]
        z = np.load(rows, allow_pickle=True); keep = z["held"] & np.isfinite(z["centre"]).all(1); i = np.flatnonzero(keep)
        s.tl = np.round(z["t"][i] - z["t"][i[0]], 5); s.pf = z["phone_frame"][i].astype(float)
        c = json.load(open(cams, encoding="utf-8"))["cameras"]["phone"]
        s.K, s.dist, s.rv, s.tv = np.array(c["K"]), np.array(c["dist"]), cv2.Rodrigues(np.array(c["R_world_to_cam"]))[0], np.array(c["t_world_to_cam_m"])
        s.eye = -np.array(c["R_world_to_cam"]).T @ s.tv; s.sw = c["image_size_wh"][0]; s.Rc = np.array(c["R_world_to_cam"])
        s.joints = {round(f["t"], 5): f["joints"] for f in json.load(open(kp, encoding="utf-8"))["frames"]}
        s.cap = cv2.VideoCapture(clip); s.cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0); s.at = -1; s.last = None; s.crop = (0, 0, 1080)

    def to_px(s, X):
        """World points to picture pixels: raw sensor, turned a quarter, cropped, scaled."""
        u = s.cv.projectPoints(np.array(X, float).reshape(-1, 3), s.rv, s.tv, s.K, s.dist)[0].reshape(-1, 2)
        x0, y0, S = s.crop
        return np.c_[u[:, 1] - x0, s.sw - 1 - u[:, 0] - y0] * P / S

    def outline(s, c, R):
        """The ball's silhouette as seen by the camera."""
        d = np.array(c) - s.eye; L = np.linalg.norm(d); d = d / L
        a = np.cross(d, [0, 0, 1.]); a /= np.linalg.norm(a); b = np.cross(d, a); th = np.linspace(0, 2 * np.pi, 72)
        return s.to_px(np.array(c) + R * (-(R / L) * d + math.sqrt(1 - (R / L) ** 2) * (np.outer(np.cos(th), a) + np.outer(np.sin(th), b))))

    def fit_crop(s, ts, B, R, margin):
        """The smallest square, in the turned picture, holding the tracked hand and ball over these times."""
        s.crop = (0, 0, P); pts = []
        for tt in ts:
            pts += [p for p in s.joints[round(tt, 5)] if p is not None]
        q = s.to_px(pts); o = np.vstack([s.outline(B[round(tt, 5)]["obj_pos"], R) for tt in ts]); q = np.vstack([q, o])
        lo, hi = q.min(0) - margin, q.max(0) + margin; S = min(1080, int(max(hi - lo)))
        x0, y0 = [int(np.clip(c - S / 2, 0, m - S)) for c, m in zip((lo + hi) / 2, (1080, s.sw))]
        s.crop = (x0, y0, S); print("video crop in the turned picture: x, y, side", s.crop, flush=True)

    def picture(s, tt):
        """The video frame at take time tt, cropped, scaled."""
        n = int(round(np.interp(tt, s.tl, s.pf)))
        if n == s.at: return s.last.copy()
        if n < s.at or n > s.at + 40: s.cap.set(s.cv.CAP_PROP_POS_FRAMES, n); s.at = n - 1
        while s.at < n - 1: s.cap.grab(); s.at += 1
        ok, im = s.cap.read(); s.at = n; assert ok and im.shape[1] == s.sw, "the video did not come back as raw sensor frames"
        im = s.cv.rotate(im, s.cv.ROTATE_90_COUNTERCLOCKWISE); x0, y0, S = s.crop
        s.last = s.cv.cvtColor(s.cv.resize(im[y0:y0 + S, x0:x0 + S], (P, P), interpolation=s.cv.INTER_AREA), s.cv.COLOR_BGR2RGB)
        return s.last.copy()

    def skeleton(s, im, tt, centre, R):
        cv = s.cv; im = im.copy(); J = s.joints[round(tt, 5)]; ok = [p is not None for p in J]
        u = s.to_px([p if p is not None else [0, 0, .3] for p in J])
        ball = s.outline(centre, R).astype(np.int32)
        cv.polylines(im, [ball], True, (20, 20, 20), 6, cv.LINE_AA); cv.polylines(im, [ball], True, (255, 255, 255), 3, cv.LINE_AA)
        for a, b in BONES:
            if ok[a] and ok[b]: cv.line(im, tuple(int(x) for x in u[a]), tuple(int(x) for x in u[b]), (20, 20, 20), 8, cv.LINE_AA)
        for a, b in BONES:
            if ok[a] and ok[b]: cv.line(im, tuple(int(x) for x in u[a]), tuple(int(x) for x in u[b]), (120, 235, 255), 4, cv.LINE_AA)
        for i, g in enumerate(ok):
            if g:
                c = tuple(int(x) for x in u[i]); cv.circle(im, c, 7, (20, 20, 20), -1, cv.LINE_AA); cv.circle(im, c, 5, (255, 255, 255), -1, cv.LINE_AA)
        return im

    def hidden(s, X, c, R):
        """Which world points the ball hides from the phone: inside it, or behind its near surface."""
        d = np.array(X, float) - s.eye; L = np.linalg.norm(d, axis=1); u = d / L[:, None]; tc = u @ (np.array(c) - s.eye)
        q2 = np.sum((np.array(c) - s.eye) ** 2) - tc ** 2
        return (q2 < R * R) & (L > tc - np.sqrt(np.maximum(R * R - q2, 0)))

    def turn_marks(s, im, c, om, R, dots, behind=False):
        """The ball's turn and the touching points over the frame: all in world metres.
        With behind, what the ball hides of the axis and arc is drawn faint."""
        cv = s.cv; c = np.array(c); fwd = (c - s.eye) / np.linalg.norm(c - s.eye); a, b, arc = turn_path(c, om, R, fwd)
        base = im; im = s.arrows(im, [(a, b, TURN8)]); u = s.to_px(arc).astype(np.int32)
        cv.polylines(im, [u[:-3]], False, (255, 255, 255), 8, cv.LINE_AA); cv.polylines(im, [u[:-3]], False, TURN8, 4, cv.LINE_AA)
        im = s.arrows(im, [(arc[-6], arc[-1], TURN8)])
        if behind:
            im = (.3 * im + .7 * base).astype(np.uint8)
            ln = a + np.outer(np.linspace(0, 1, 120), b - a)
            for pts, hid, wide in ((ln, s.hidden(ln, c, R), True), (arc[:-2], s.hidden(arc[:-2], c, R), False)):
                q = s.to_px(pts).astype(np.int32); j = 0
                while j < len(pts):
                    if hid[j]: j += 1; continue
                    e = j
                    while e + 1 < len(pts) and not hid[e + 1]: e += 1
                    if e > j:
                        cv.polylines(im, [q[j:e + 1]], False, (255, 255, 255), 9 if wide else 8, cv.LINE_AA); cv.polylines(im, [q[j:e + 1]], False, TURN8, 5 if wide else 4, cv.LINE_AA)
                    j = e + 1
            if not s.hidden([b], c, R)[0]: im = s.arrows(im, [(a + .9 * (b - a), b, TURN8)])
            if not s.hidden([arc[-1]], c, R)[0]: im = s.arrows(im, [(arc[-6], arc[-1], TURN8)])
        for p in dots:
            q = tuple(int(x) for x in s.to_px([p])[0]); cv.circle(im, q, 11, (255, 255, 255), -1, cv.LINE_AA); cv.circle(im, q, 8, BLUE8, -1, cv.LINE_AA)
        return im

    def arrows(s, im, segs):
        """Force arrows as seen by the camera: each (start, end, colour) in world metres."""
        cv = s.cv; im = im.copy()
        for halo in (True, False):
            for a, b, col in segs:
                (ax, ay), (bx, by) = s.to_px([a, b]); L = math.hypot(bx - ax, by - ay)
                if L < 5: continue
                ux, uy = (bx - ax) / L, (by - ay) / L; tip = np.array([[bx + ux * 13, by + uy * 13], [bx - uy * 10, by + ux * 10], [bx + uy * 10, by - ux * 10]], np.int32)
                c = (255, 255, 255) if halo else col; w = 9 if halo else 5
                cv.line(im, (int(ax), int(ay)), (int(bx), int(by)), c, w, cv.LINE_AA)
                if halo: cv.polylines(im, [tip], True, c, 5, cv.LINE_AA)
                else: cv.fillConvexPoly(im, tip, c, cv.LINE_AA)
        return im


def human_from_video(A, L, F, t, fr, st, pan, rigs, snaps, arrow, cap, mu, names, rate, Rob_fit, B, plane, cur):
    fps = A.fps; t0, dur = A.start, A.seconds; v = Footage(A.wave_manip); R = L["object"]["radius_m"]
    r_all = [float(t[k]) for k in range(len(t)) if t0 <= t[k] <= t0 + dur]
    v.fit_crop(r_all, B, R, A.crop_margin)
    foot = "Forces derived from tracked motion: comparative, not predictive."
    head = "Can a hand do what a human can?"
    lines = [f"A person turning a tennis ball in the palm, about {rate:.0f} degrees per second on average. Recorded video, at normal speed.",
             (BLUE8, "Normal force."), (ORANGE8, "Shear force."), "Both derived from the ball's tracked motion, with an assumed mass and friction. Not measured by a sensor.",
             "Disc: friction limit. Right is the way the ball turns.", "Wireframe: the joints the cameras tracked. Hidden joints are left out."]
    rig = [r for r in rigs if r.D["hand"] == A.fit_hand][0]; nm = LABEL.get(rig.D["hand"], rig.D["hand"])
    def inside(k):
        c = np.array(F[k]["obj_pos"])
        return lambda X: R - np.linalg.norm(X - c, axis=1)
    if rig.full:
        for i, k in enumerate(snaps):
            dep, own = rig.overlaps(k, inside(k))
            print(f"  snapshot {i + 1}: deepest in the ball, mm: " + ", ".join(f"{a} {b * 1e3:.1f}" for a, b in dep.items()) + f"; hand into itself: {own and (own[0], own[1], round(own[2] * 1e3, 1))}", flush=True)
    Rc = v.cv.Rodrigues(v.rv)[0]
    side = View(st.look, A.az, A.el, A.fit_dist); side.pos = side.pos + side.up * A.fit_lift
    def phone_side(k):
        """Which side of the ball each palm is on, seen from the phone: positive is the far side, as the person's is."""
        po = np.array(F[k]["obj_pos"]); d = np.array(B[round(float(t[k]), 5)]["obj_pos"]) - v.eye; fwd = Rob_fit[k] @ (d / np.linalg.norm(d))
        person = np.mean([F[k]["skeleton"][j] for j in (0, 5, 9, 13, 17) if F[k]["skeleton"][j] is not None], axis=0)
        fr_ = rig.D["frames"][rig.idx[k]]; rig.pose(fr_, 1.0); m_ = rig.m
        pal = np.mean([rig.d.geom_xpos[i] for i in range(m_.ngeom) if m_.geom_type[i] == mj.mjtGeom.mjGEOM_MESH and m_.geom_group[i] == rig.vis_group and int(m_.geom_bodyid[i]) not in rig.owner], axis=0)
        return float((person - po) @ fwd), float((rig.to_leader(pal) - po) @ fwd)
    for i, k in enumerate(snaps):
        a_, b_ = phone_side(k); print(f"  snapshot {i + 1}: palm behind the ball as the phone sees it, mm: person {a_ * 1e3:.0f}, {nm} {b_ * 1e3:.0f}", flush=True)
    def frozen(k):
        """The frozen frame three ways: bare, with the tracked hand and ball, and with the forces."""
        tt = float(t[k]); bare = v.picture(tt); b = B[round(tt, 5)]; c = np.array(b["obj_pos"]); sk = v.skeleton(bare, tt, c, R)
        Rp = Rob_fit[k]; po = np.array(F[k]["obj_pos"]); segs = []
        for f in fr.exact(k):
            if f is None: continue
            a = Rp.T @ (f["point"] - po) + c
            for vec, col in ((f["normal"], BLUE8), (f["shear"], ORANGE8)):
                n_ = np.linalg.norm(vec)
                if n_ * arrow > .002: segs.append((a, a + arrow * Rp.T @ vec * min(1, cap / n_), col))
        return bare, sk, v.arrows(sk, segs)
    what = ["Each snapshot is checked on its own, frozen, against a moving task's forces. The hand is not simulated doing the turn.",
            "Each dot is a finger's result: green passes, amber is close, red does not pass by this method at this pose." + (" A larger red dot marks a contact with no finger." if any(rig.D["frames"][rig.idx[k]].get("unserved") for k in snaps) else ""),
            "The pose is schematic: parts of the hand can pass through the ball or each other."]
    def fit_frame(k, a):
        cur[0] = float(t[k]); w_ = fr.exact(k); st.v = side; lay, pads, f_ = rig.layer(k, a)
        miss = [np.array(F[k]["contact_point"][u]) for u in f_.get("unserved", []) if F[k]["contact_point"][u]] if a >= .999 else []
        return st.compose(k, [lay], st.overlay(w_, arrow, cap, pads, miss)), w_, f_
    if A.video_stills:
        for i, k in enumerate(snaps):
            cur[0] = float(t[k]); bare, sk, ar = frozen(k); w_ = fr.exact(k)
            Image.fromarray(pan.frame(ar, head, lines, w_, mu, plane, f"Video, t = {cur[0] - t0:.1f} s, paused", f"Snapshot {i + 1} saved: positions and forces, as targets.", None, foot)).save(f"{A.out}_snap{i + 1}.png")
            ins = Image.fromarray(ar).resize((250, 250), Image.LANCZOS)
            im, w_, f_ = fit_frame(k, 1.0)
            Image.fromarray(pan.frame(im, f"Can {nm} supply these targets?", what, w_, mu, plane, None, f"Snapshot {i + 1}: {f_['verdict']}.", None, foot, ins)).save(f"{A.out}_fit{i + 1}.png")
        print("stills written", flush=True); os._exit(0)
    vid = Video(A.out, fps); saved = []; note = None; todo = list(snaps); shown = None; insets = []
    for j in range(int(dur * fps) + 1):
        cur[0] = t0 + j / fps; tag = f"Video, t = {j / fps:.1f} s"
        if todo and cur[0] >= t[todo[0]]:
            k = todo.pop(0); cur[0] = float(t[k]); w_ = fr.exact(k); bare, sk, ar = frozen(k); shown = w_; tag = f"Video, t = {cur[0] - t0:.1f} s, paused"
            pan.tiles.append(Image.fromarray(ar).resize((118, 118), Image.LANCZOS)); insets.append(Image.fromarray(ar).resize((250, 250), Image.LANCZOS)); i = len(pan.tiles) - 1
            saved.append(dict(t_s=round(float(t[k]), 4), frame=k, obj_pos=F[k]["obj_pos"], fingers={
                nm_: None if f is None else dict(point_m=f["point"].tolist(), normal_N=f["normal"].tolist(), shear_N=f["shear"].tolist()) for nm_, f in zip(names, w_)}))
            T = int(A.hold * fps); fade = int(.3 * fps); late = int(.8 * fps)
            for q in range(T):
                if q < late: a = min(1, q / fade); im = (a * sk + (1 - a) * bare).astype(np.uint8); txt, show = f"Snapshot {i + 1}: the tracked hand and ball.", None
                else: a = min(1, (q - late) / fade); im = (a * ar + (1 - a) * sk).astype(np.uint8); txt, show = f"Snapshot {i + 1} saved: positions and forces, as targets.", w_
                vid.put(pan.frame(im, head, lines, show, mu, plane, tag, txt, i, foot))
            note = f"Discs show snapshot {i + 1}."
            continue
        vid.put(pan.frame(v.picture(cur[0]), head, lines, shown, mu, plane, tag, note, None, foot))
    borders = {}
    for i, k in enumerate(snaps):
        for q_ in range(int(A.fit_seconds * fps)):
            a = min(1.0, q_ / max(1, A.fit_close * fps)); a = a * a * (3 - 2 * a)
            im, w_, f_ = fit_frame(k, a)
            if a >= .999: borders[i] = RGB[f_["verdict"]]
            vid.put(pan.frame(im, f"Can {nm} supply these targets?", what, w_, mu, plane, None, f"Snapshot {i + 1}: {f_['verdict']}." if a >= .999 else f"Snapshot {i + 1}: fitting.", i, foot, insets[i], borders))
    strips = [(LABEL.get(r.D["hand"], r.D["hand"]), r.D["frames"]) for r in rigs]; n_f = len(rigs[0].D["frames"]); _, _, ar = frozen(snaps[-1])
    for _ in range(int(A.end * fps)):
        vid.put(pan.frame(ar, "Every frame the check looked at", [f"{n_f} frames across the whole take, each checked on its own. Red means not by this method at this pose, not that the hand can never do the task."], None, mu, plane, None, None, None, foot, None, None, strips))
    vid.close()
    json.dump(dict(source=A.leader, averaged_over_s=0, snapshots=saved), open(os.path.splitext(A.out)[0] + "_snapshots.json", "w"), indent=1)
    print("wrote", A.out, "frames", vid.n, "seconds", round(vid.n / fps, 1), flush=True)
    os._exit(0)


def human_checks(A):
    """The human take with no per-finger force targets: the position check and the turning check on the fitted hand."""
    def load(n): return json.load(open(os.path.join(ROOT, "data", "leaders", n + ".json"), encoding="utf-8"))
    L = load(A.leader); F = L["frames"]; board = load(A.leader.replace("handframe", "derived"))["frames"]; spin_in_palm(F, board)
    names = [str(x) for x in L["stations"]]; t = np.array([f["t"] for f in F]); fps, t0, dur = A.fps, A.start, A.seconds; R = L["object"]["radius_m"]
    C = {h: json.load(open(os.path.join(A.checks, f"position_turning_{h}.json"), encoding="utf-8")) for h in A.check_hands}
    C_ = {x["k"]: x for x in C[A.fit_hand]["frames"]}
    L["id"] = C[A.fit_hand]["position_run"]["leader"]    # the poses drawn are the position run's
    st = HumanStage(L, A.az, A.el, A.dist, A.alpha, A.look_back); pan = Panel(names, 1)
    rig = Rig(os.path.join(A.checks, f"follow_{A.fit_hand}__{L['id']}.json"), st, full=True); nm = LABEL.get(A.fit_hand, A.fit_hand)
    ev = np.array(sorted(k for k in C_ if t0 <= t[k] <= t0 + dur))
    snaps = [int(ev[np.argmin(np.abs(t[ev] - (t0 + m)))]) for m in np.arange(A.every, dur + 1e-6, A.every)]
    print("snapshots at", [round(float(t[k] - t0), 2) for k in snaps], "frames", snaps, flush=True)
    B = {round(b["t"], 5): b for b in board}; fits = []
    for f in F:     # the palm's rotation at each frame, from the two files' points
        b = B[round(f["t"], 5)]; ok = [i for i in range(len(f["station_pos"])) if f["station_pos"][i] is not None and b["station_pos"][i] is not None]
        fits.append(kabsch(np.array([b["station_pos"][i] for i in ok] + [b["obj_pos"]]), np.array([f["station_pos"][i] for i in ok] + [f["obj_pos"]])))
    v = Footage(A.wave_manip); v.fit_crop([float(t[k]) for k in range(len(t)) if t0 <= t[k] <= t0 + dur], B, R, A.crop_margin)
    rate = float(np.median([C_[k]["turn_deg_s"] for k in ev]))
    side = View(st.look, A.az, A.el, A.fit_dist); side.pos = side.pos + side.up * A.fit_lift
    views = {}
    if A.phone_view:    # each fit seen from where the phone was at that snapshot, carried into the palm frame
        for k in snaps:
            c_ = np.array(B[round(float(t[k]), 5)]["obj_pos"]); f_ = fits[k] @ ((c_ - v.eye) / np.linalg.norm(c_ - v.eye)) * (1 if A.phone_view == "near" else -1)
            u_ = fits[k] @ v.Rc.T @ np.array([1., 0, 0]); u_ = u_ - (u_ @ f_) * f_
            vw = View(F[k]["obj_pos"], 0, 0, 1); vw.fwd, vw.up = f_, u_ / np.linalg.norm(u_); vw.pos = np.array(F[k]["obj_pos"]) - A.fit_dist * f_ + vw.up * A.fit_lift; views[k] = vw
            up_px = v.to_px([c_, c_ + .05 * (v.Rc.T @ np.array([1., 0, 0]))]); print(f"  phone view, frame {k}: picture up check, pixels {up_px[1][1] - up_px[0][1]:.0f} (negative is up)", flush=True)
    def inside(k):
        c = np.array(F[k]["obj_pos"])
        return lambda X: R - np.linalg.norm(X - c, axis=1)
    sides = []
    for i, k in enumerate(snaps):
        dep, own = rig.overlaps(k, inside(k))
        print(f"  snapshot {i + 1}: deepest in the ball, mm: " + ", ".join(f"{a} {b * 1e3:.1f}" for a, b in dep.items()) + f"; hand into itself: {own and (own[0], own[1], round(own[2] * 1e3, 1))}", flush=True)
        po = np.array(F[k]["obj_pos"]); d = np.array(B[round(float(t[k]), 5)]["obj_pos"]) - v.eye; fwd = fits[k] @ (d / np.linalg.norm(d))
        person = np.mean([F[k]["skeleton"][j] for j in (0, 5, 9, 13, 17) if F[k]["skeleton"][j] is not None], axis=0)
        rig.pose(rig.D["frames"][rig.idx[k]], 1.0); m_ = rig.m
        pal = np.mean([rig.d.geom_xpos[g] for g in range(m_.ngeom) if m_.geom_type[g] == mj.mjtGeom.mjGEOM_MESH and m_.geom_group[g] == rig.vis_group and int(m_.geom_bodyid[g]) not in rig.owner], axis=0)
        sides.append(((person - po) @ fwd, (rig.to_leader(pal) - po) @ fwd))
        print(f"  snapshot {i + 1}: palm behind the ball as the phone sees it, mm: person {sides[-1][0] * 1e3:.0f}, {nm} {sides[-1][1] * 1e3:.0f}", flush=True)
    opposite = all(a_ * b_ < 0 for a_, b_ in sides)
    foot = "Mass and friction assumed. Comparative, not predictive."
    head = "Can a hand do what a human can?"
    lines = [f"A person turning a tennis ball in the palm, about {rate:.0f} degrees per second. Recorded video, at normal speed.",
             "No sensor measured the forces here, and forces worked out from the ball's motion show the weight shared out, not the turn. So the targets are where the fingers touch and how the ball turns.",
             "Wireframe: the joints the cameras tracked. Hidden joints are left out.",
             (TURN8, "Axis and arc: the ball's turn relative to the palm, measured from the tracked ball."),
             (BLUE8, "Dots: where the person's fingers touch.")]
    def frozen(k):
        tt = float(t[k]); bare = v.picture(tt); b = B[round(tt, 5)]; c = np.array(b["obj_pos"]); sk = v.skeleton(bare, tt, c, R)
        Rp = fits[k]; po = np.array(F[k]["obj_pos"])
        dots = [Rp.T @ (np.array(F[k]["contact_point"][i]) - po) + c for i in range(len(names)) if F[k]["contact"][i] and F[k]["contact_point"][i]]
        return bare, sk, v.turn_marks(sk, c, Rp.T @ np.array(C_[k]["omega_palm_rad_s"]), R, dots, A.hide_turn_behind_ball)
    def axis_words(k):
        """Where the turn's axis points, as the phone sees it: computed, as drawn."""
        if not A.hide_turn_behind_ball: return []
        ax = fits[k].T @ np.array(C_[k]["omega_palm_rad_s"]); ax = ax / np.linalg.norm(ax)
        d = np.array(B[round(float(t[k]), 5)]["obj_pos"]) - v.eye; d = d / np.linalg.norm(d)
        return [f"The turn's axis points {'into' if ax @ d > 0 else 'out of'} the picture, {np.degrees(np.arccos(abs(ax @ d))):.0f} degrees from straight on; the ball hides its far end."]
    def listed(xs): return ", ".join(xs[:-1]) + " and " + xs[-1] if len(xs) > 1 else xs[0]
    def dropped(h):
        """Contacts left out of a hand's checks because it has no finger of that name: tools/run_follow_named_pairing.py
        marks the take it checked with "__no_<finger>" at the end of its id."""
        lid = str(C[h]["leader"]); return lid.rsplit("__no_", 1)[1].split("_") if "__no_" in lid else []
    def who(k): return [names[i] for i in range(len(names)) if F[k]["contact"][i] and F[k]["contact_point"][i]]
    def said(k): return f"Saved: where the ball is, which fingers touch, and how it turns: {C_[k]['turn_deg_s']:.0f} degrees per second."
    def fit_lines(k):
        x = C_[k]; pc = x["position"]; tc = turn_colour(x); un = [names[u] for u in x["unserved"]]; tch = listed(x["touching"]).capitalize()
        dr = dropped(A.fit_hand)
        pos = f"Position check: {pc}. " + (("Every other contact reached." if dr else "Every contact reached.") if pc == "green" else "")
        if un: pos += f"The check pairs fingers by position, not by name, and the person's {listed(un)} has no finger to match: the red dot. The other contacts are reached."
        elif pc == "amber": pos += "Every contact has a finger, but at least one is only close."
        if tc == "green": turn_ = f"Turning check: green. {tch} can hold the ball inside friction and joint strength, with {x['spare_on_axis_mNm']:.0f} mN m to spare about its turn. The turn itself needs {abs(x['need_torque_along_turn_mNm']):.2f} mN m."
        elif tc == "amber": turn_ = f"Turning check: amber. {tch} can hold the ball only with torque against the turn: {x['spare_on_axis_mNm']:.0f} mN m. If torque off the turn's axis is left free, {x['spare_off_axis_free_mNm']:.0f} mN m."
        else: turn_ = f"Turning check: red. {tch} cannot hold the ball with torque purely about the turn."
        sh = [q for q in x["follow_share"].values() if q is not None]
        out = ["Two separate checks on each frozen snapshot. No hand is simulated doing the turn."]
        if dr: out.append(f"{nm} has no {listed(dr)} finger, so the person's {listed(dr)}-finger contact is left out of both checks. Each other finger is paired with the person's finger of the same name.")
        out += [(RGB[pc], pos), (RGB[tc], turn_)]
        if sh: out.append("The touching fingers' joints can follow the ball's surface motion at their contacts." if min(sh) >= .995 else f"The touching fingers' joints can follow at least {100 * min(sh):.0f}% of the ball's surface motion at their contacts.")
        if tc != "red": out.append("Torque to spare can be overstated: the model sets no limit on the load a joint's end stop carries.")
        if A.phone_view == "far": out.append("Seen along the phone's line of sight from the far side, so the picture is a mirror image of the video.")
        out += [(TURN8, "The ball's turn."),
                "The pose is schematic: parts of the hand can pass through the ball or each other." + (" The check places the hand freely: here it holds the ball from the opposite side to the person." if opposite else ""),
                "Squares under each snapshot: position check left, turning check right."]
        return out
    cache = {}
    def fit_frame(k, a):
        if a >= .999 and k in cache: return cache[k]
        st.v = views.get(k, side); lay, pads, f_ = rig.layer(k, a)
        miss = [np.array(F[k]["contact_point"][u]) for u in f_.get("unserved", []) if F[k]["contact_point"][u]] if a >= .999 else []
        c = np.array(F[k]["obj_pos"]); turn = turn_path(c, C_[k]["omega_palm_rad_s"], R, st.v.fwd)
        if A.hide_turn_behind_ball: im = st.compose(k, [lay], st.overlay([], 0, 1, pads, miss), st.overlay([], 0, 1, (), (), turn))
        else: im = st.compose(k, [lay], st.overlay([], 0, 1, pads, miss, turn))
        if a >= .999: cache[k] = im
        return im
    def end_frame(ar):
        allh = C[A.check_hands[0]]["frames"]; need = float(np.median([abs(x["need_torque_along_turn_mNm"]) for x in allh]))
        bars = [(LABEL.get(h, h), [("Position check", [x["position"] for x in C[h]["frames"]]), ("Turning check", [turn_colour(x) for x in C[h]["frames"]])]) for h in A.check_hands]
        dh = [h for h in A.check_hands if dropped(h)]
        left_out = (f"{listed([LABEL.get(h, h) for h in dh])}: no {listed(dropped(dh[0]))} finger, so the person's {listed(dropped(dh[0]))}-finger contact is left out."
                    if dh and all(dropped(h) == dropped(dh[0]) for h in dh) else "Contacts with no finger are left out of the turning check.")
        return pan.frame(ar, "Every frame the checks looked at", [
            f"{len(allh)} frames across the whole take, each frozen and checked on its own. {left_out} Red means not by this method at this pose, not that the hand can never do the task.",
            f"Green in the turning check is a low bar: the turn itself needs little torque, median {need:.2f} mN m, and the model sets no limit on end-stop load. Built for this film, not the banked method."], None, 1, None, None, None, None, foot, bars=bars)
    marks = {}
    if A.video_stills:
        for i, k in enumerate(snaps):
            bare, sk, ar = frozen(k); pan.tiles.append(Image.fromarray(ar).resize((118, 118), Image.LANCZOS))
            Image.fromarray(pan.frame(ar, head, lines + [f"Touching at this moment: {listed(who(k))}."] + axis_words(k), None, 1, None, f"Video, t = {t[k] - t0:.1f} s, paused", f"Snapshot {i + 1}. {said(k)}", i, foot)).save(f"{A.out}_snap{i + 1}.png")
            marks[i] = (RGB[C_[k]["position"]], RGB[turn_colour(C_[k])])
            Image.fromarray(pan.frame(fit_frame(k, 1.0), f"Can {nm} do the same?", fit_lines(k), None, 1, None, None, None, i, foot, Image.fromarray(ar).resize((250, 250), Image.LANCZOS), marks=marks)).save(f"{A.out}_fit{i + 1}.png")
        Image.fromarray(end_frame(ar)).save(f"{A.out}_end.png"); print("stills written", flush=True); os._exit(0)
    vid = Video(A.out, fps); todo = list(snaps); insets = []
    for j in range(int(dur * fps) + 1):
        tt = t0 + j / fps; tag = f"Video, t = {j / fps:.1f} s"
        if todo and tt >= t[todo[0]]:
            k = todo.pop(0); bare, sk, ar = frozen(k); tag = f"Video, t = {t[k] - t0:.1f} s, paused"
            pan.tiles.append(Image.fromarray(ar).resize((118, 118), Image.LANCZOS)); insets.append(Image.fromarray(ar).resize((250, 250), Image.LANCZOS)); i = len(pan.tiles) - 1
            T_ = int(A.hold * fps); fade = int(.3 * fps); late = int(.9 * fps); extra = [f"Touching at this moment: {listed(who(k))}."] + axis_words(k)
            for q in range(T_):
                if q < late: a = min(1, q / fade); im = (a * sk + (1 - a) * bare).astype(np.uint8); txt = f"Snapshot {i + 1}: the tracked hand and ball."
                else: a = min(1, (q - late) / fade); im = (a * ar + (1 - a) * sk).astype(np.uint8); txt = f"Snapshot {i + 1}. {said(k)}"
                vid.put(pan.frame(im, head, lines + extra, None, 1, None, tag, txt, i, foot))
            continue
        vid.put(pan.frame(v.picture(tt), head, lines, None, 1, None, tag, None, None, foot))
    for i, k in enumerate(snaps):
        for q_ in range(int(A.fit_seconds * fps)):
            a = min(1.0, q_ / max(1, A.fit_close * fps)); a = a * a * (3 - 2 * a); im = fit_frame(k, a)
            if a >= .999: marks[i] = (RGB[C_[k]["position"]], RGB[turn_colour(C_[k])])
            vid.put(pan.frame(im, f"Can {nm} do the same?", fit_lines(k) if a >= .999 else ["Two separate checks on each frozen snapshot. No hand is simulated doing the turn."], None, 1, None, None, None if a >= .999 else f"Snapshot {i + 1}: fitting.", i, foot, insets[i], marks=marks))
    e_ = end_frame(frozen(snaps[-1])[2])
    for _ in range(int(A.end * fps)): vid.put(e_)
    vid.close(); print("wrote", A.out, "frames", vid.n, "seconds", round(vid.n / fps, 1), flush=True)
    os._exit(0)


def human(A):
    if A.checks: human_checks(A)
    def load(n): return json.load(open(os.path.join(ROOT, "data", "leaders", n + ".json"), encoding="utf-8"))
    L = load(A.leader); F = L["frames"]; board = load(A.leader.replace("handframe", "derived"))["frames"]; Rob = spin_in_palm(F, board)
    names = [str(x) for x in L["stations"]]; mu = float(L.get("mu", 1.0))   # as follow.py reads it
    t = np.array([f["t"] for f in F]); n = len(names); N = len(F)
    lam, nrm, pnt, she = np.zeros((N, n)), np.zeros((N, n, 3)), np.zeros((N, n, 3)), np.zeros((N, n, 3))
    for k, f in enumerate(F):
        for i in range(n):
            if f["contact"][i] and f["contact_normal"][i] and f["force_on_object_N"][i]:
                u = np.array(f["contact_normal"][i]); v = np.array(f["force_on_object_N"][i])
                lam[k, i] = v @ u; nrm[k, i] = u; pnt[k, i] = f["contact_point"][i]; she[k, i] = v - lam[k, i] * u
    t0, dur, fps = A.start, A.seconds, A.fps; sel = np.nonzero((t >= t0) & (t <= t0 + dur))[0]
    w = []
    for i, j in zip(sel[:-1], sel[1:]):
        R = Rob[i].T @ Rob[j]; q = np.zeros(4); mj.mju_mat2Quat(q, R.ravel()); ang = 2 * np.arctan2(np.linalg.norm(q[1:]), q[0])
        w.append(Rob[j] @ (q[1:] / max(np.linalg.norm(q[1:]), 1e-9) * ang / (t[j] - t[i])))
    spin = np.mean(w, 0); rate = np.degrees(np.linalg.norm(spin)); axis = spin / np.linalg.norm(spin)
    fr = Forces(t, names, lam, nrm, pnt, she, mu, axis)
    seq = [fr.window(t0 + j / fps) for j in range(int(dur * fps) + 1)]
    press = np.array([f["press"] for w_ in seq for f in w_ if f]); shear = np.array([np.linalg.norm(f["shear"]) for w_ in seq for f in w_ if f])
    cap = float(np.percentile(press, 99)); arrow = A.arrow_R * L["object"]["radius_m"] / cap
    gaps = np.diff(t[sel]); print(f"window {t0}-{t0 + dur} s: {len(sel)} frames, longest gap {gaps.max():.2f} s; mu {mu}; turning {rate:.0f} deg/s about {axis.round(2)}", flush=True)
    print(f"shown forces: normal median {np.median(press):.2f}, p99 {cap:.2f}, max {press.max():.2f} N; shear median {np.median(shear):.2f}, max {shear.max():.2f} N; over scale {(press > cap).mean() * 100:.1f}% of finger-frames", flush=True)
    st = HumanStage(L, A.az, A.el, A.dist, A.alpha, A.look_back); pan = Panel(names, 60 / (mu * cap))
    head = "Can a hand do what a human can?"
    lines = [f"A person turning a tennis ball in the palm, about {rate:.0f} degrees per second on average. Shown in the palm's own frame.",
             (BLUE8, "Normal force."), (ORANGE8, "Shear force."), "Both derived from the ball's tracked motion, with an assumed mass and friction. Not measured by a sensor.",
             "Disc: friction limit. Right is the way the ball turns."]
    cur = [t0]
    def plane(f):
        fr.centre_at = np.array(F[min(int(np.searchsorted(t, cur[0])), N - 1)]["obj_pos"]); return fr.plane(f)
    stills = None
    if A.stills_dir and os.path.isdir(A.stills_dir):
        idx = json.load(open(os.path.join(A.stills_dir, "index.json"), encoding="utf-8"))["stills"]; st_t = np.array([x["t"] for x in idx]); cache = {}
        def stills(tt):
            x = idx[max(0, int(np.searchsorted(st_t, tt, "right")) - 1)]
            if x["file"] not in cache:
                im = Image.open(os.path.join(A.stills_dir, x["file"])).convert("RGB"); cache[x["file"]] = im.resize((250, int(250 * im.height / im.width)), Image.LANCZOS)
            return cache[x["file"]]
    foot = "Forces derived from tracked motion: comparative, not predictive."
    def pic(k, w_): return st.compose(k, [st.person(k)], st.overlay(w_, arrow, cap))
    if A.stills is not None:
        for tt in A.stills:
            cur[0] = t0 + tt; w_ = fr.window(cur[0]); k = min(int(np.searchsorted(t, cur[0])), N - 1)
            Image.fromarray(pan.frame(pic(k, w_), head, lines, w_, mu, plane, f"Human take, t = {tt:.1f} s", inset=stills(cur[0]) if stills else None, footer=foot)).save(f"{A.out}_{tt:g}.png")
        print("stills written", flush=True); os._exit(0)
    rigs = [Rig(p, st, full=A.video and json.load(open(p, encoding="utf-8"))["hand"] == A.fit_hand) for p in A.runs]
    ev = sorted(set.intersection(*[set(r.idx) for r in rigs])); ev = np.array([k for k in ev if t0 <= t[k] <= t0 + dur])
    big = np.array([max(np.linalg.norm(v) for v in F[k]["force_on_object_N"] if v) for k in ev]); ev_ok = ev[big < cap * 2]
    print(f"frames checked in this window: {len(ev)}; free of force spikes: {len(ev_ok)}", flush=True)
    snaps = [int(ev_ok[np.argmin(np.abs(t[ev_ok] - (t0 + m)))]) for m in np.arange(A.every, dur + 1e-6, A.every)]
    print("snapshots at", [round(float(t[k] - t0), 2) for k in snaps], "frames", snaps, flush=True)
    for r in rigs: print(r.D["hand"], "at snapshots:", [r.D["frames"][r.idx[k]]["verdict"] for k in snaps], flush=True)
    if A.video:
        B = {round(b["t"], 5): b for b in board}; fits = []
        for f in F:     # the palm's rotation at each frame, from the two files' points
            b = B[round(f["t"], 5)]; ok = [i for i in range(len(f["station_pos"])) if f["station_pos"][i] is not None and b["station_pos"][i] is not None]
            fits.append(kabsch(np.array([b["station_pos"][i] for i in ok] + [b["obj_pos"]]), np.array([f["station_pos"][i] for i in ok] + [f["obj_pos"]])))
        for k in snaps:     # the mapping must carry the palm file's tracked points onto the board file's. Contact points differ on purpose: the palm file re-makes them from the skeleton (its notes)
            b, f = B[round(float(t[k]), 5)], F[k]
            e = [np.linalg.norm(fits[k].T @ (np.array(f["station_pos"][i]) - f["obj_pos"]) + b["obj_pos"] - b["station_pos"][i]) for i in range(n) if f["station_pos"][i] and b["station_pos"][i]]
            print(f"frame {k}: palm-to-board mapping misses the tracked finger points by at most {max(e) * 1e3:.2f} mm", flush=True)
            assert max(e) < .001, "the palm-to-board mapping does not hold"
        human_from_video(A, L, F, t, fr, st, pan, rigs, snaps, arrow, cap, mu, names, rate, fits, B, plane, cur)
    vid = Video(A.out, fps); saved = []; note = None; todo = list(snaps)
    for j in range(int(dur * fps) + 1):
        cur[0] = t0 + j / fps; tt = j / fps; w_ = seq[j]; k = min(int(np.searchsorted(t, cur[0])), N - 1)
        tag = f"Human take, t = {tt:.1f} s"; ins = stills(cur[0]) if stills else None
        if todo and t[k] >= t[todo[0]]:
            k = todo.pop(0); cur[0] = float(t[k]); w_ = fr.window(cur[0]); im = pic(k, w_); cut = int(P * .14)
            pan.tiles.append(Image.fromarray(im[cut:P - cut, cut:P - cut]).resize((118, 118), Image.LANCZOS))
            note = f"Snapshot {len(pan.tiles)} saved: positions and forces, as targets."
            saved.append(dict(t_s=round(float(t[k]), 4), frame=k, obj_pos=F[k]["obj_pos"], fingers={
                nm: None if f is None else dict(point_m=f["point"].tolist(), normal_N=f["normal"].tolist(), shear_N=f["shear"].tolist()) for nm, f in zip(names, w_)}))
            for _ in range(int(A.hold * fps)): vid.put(pan.frame(im, head, lines, w_, mu, plane, tag, note, len(pan.tiles) - 1, foot, ins))
            continue
        vid.put(pan.frame(pic(k, w_), head, lines, w_, mu, plane, tag, note, None, foot, ins))
    # Each hand against the frozen snapshots.
    what = ["Each snapshot is checked on its own, frozen, against a moving task's forces. The hand is not simulated doing the turn.",
            "Green passes, amber is close, red does not pass by this method at this pose. A red dot marks a contact with no finger.",
            "Hand poses are schematic: the check tests each finger against the ball, not against the other fingers, so fingers can pass through each other."]
    label = {"leap": "LEAP", "allegro": "Allegro", "shadow": "Shadow"}
    for r in rigs[:A.fit_hands]:
        nm = label.get(r.D["hand"], r.D["hand"]); borders = {}; close, hold = A.fit_close, A.fit_seconds - A.fit_close
        for i, k in enumerate(snaps):
            cur[0] = float(t[k]); w_ = fr.window(cur[0])
            for q_ in range(int((close + hold) * fps)):
                a = min(1.0, q_ / max(1, close * fps)); a = a * a * (3 - 2 * a)
                lay, pads, f_ = r.layer(k, a)
                miss = [np.array(F[k]["contact_point"][u]) for u in f_.get("unserved", []) if F[k]["contact_point"][u]] if a >= .999 else []
                im = st.compose(k, [lay], st.overlay(w_, arrow, cap, pads, miss))
                if a >= .999: borders[i] = RGB[f_["verdict"]]
                vid.put(pan.frame(im, f"Can {nm} supply these targets?", what, w_, mu, plane, None, f"Snapshot {i + 1}: {f_['verdict']}." if a >= .999 else f"Snapshot {i + 1}: fitting.", i, foot, None, borders))
    strips = [(label.get(r.D["hand"], r.D["hand"]), r.D["frames"]) for r in rigs]; n_f = len(rigs[0].D["frames"])
    k = snaps[-1]; cur[0] = float(t[k]); w_ = fr.window(cur[0]); im = st.compose(k, [st.person(k)], st.overlay(w_, arrow, cap))
    for _ in range(int(A.end * fps)):
        vid.put(pan.frame(im, "Every frame the check looked at", [f"{n_f} frames across the whole take, each checked on its own. Red means not by this method at this pose, not that the hand can never do the task."], None, mu, plane, None, None, None, foot, None, None, strips))
    vid.close()
    json.dump(dict(source=A.leader, averaged_over_s=SMOOTH_S, snapshots=saved), open(os.path.splitext(A.out)[0] + "_snapshots.json", "w"), indent=1)
    print("wrote", A.out, "frames", vid.n, "seconds", round(vid.n / fps, 1), flush=True)
    os._exit(0)


def main():
    a = argparse.ArgumentParser(); sub = a.add_subparsers(dest="part", required=True)
    h = sub.add_parser("hora")
    h.add_argument("--wave-manip", required=True); h.add_argument("--out", required=True)
    h.add_argument("--start", type=float, default=3.0); h.add_argument("--seconds", type=float, default=10.0)
    h.add_argument("--every", type=float, default=2.0); h.add_argument("--hold", type=float, default=1.6); h.add_argument("--end", type=float, default=2.5)
    h.add_argument("--fps", type=int, default=30); h.add_argument("--stills", type=float, nargs="*")
    h.add_argument("--az", type=float, default=120); h.add_argument("--el", type=float, default=-50)
    h.add_argument("--dist", type=float, default=.34); h.add_argument("--look-below", type=float, default=.015)
    h.add_argument("--alpha", type=float, default=.6); h.add_argument("--arrow-R", type=float, default=2.0)
    h.add_argument("--run", help="a tools/run_follow_snapshots.py result: draw that hand against the snapshots after the replay")
    h.add_argument("--leader-json", help="the leader file that run wrote"); h.add_argument("--fit-only", action="store_true", help="only the fitted-hand shot")
    h.add_argument("--fit-az", type=float, default=None); h.add_argument("--fit-el", type=float, default=None)
    h.add_argument("--fit-dist", type=float, default=.56); h.add_argument("--fit-seconds", type=float, default=2.0); h.add_argument("--fit-close", type=float, default=.8)
    h.add_argument("--sticks", action="store_true", help="draw the fitted hand as sticks, not its own meshes"); h.add_argument("--fit-stills", action="store_true", help="test pictures of the fitted hand, no video")
    u = sub.add_parser("human")
    u.add_argument("--video", action="store_true", help="play the recorded video, with the extraction on each frozen frame"); u.add_argument("--wave-manip")
    u.add_argument("--video-stills", action="store_true", help="test pictures, no video"); u.add_argument("--phone-view", choices=["near", "far"], help="fit shot along the phone's line of sight: from the phone's side, or from the far side (a mirror image)"); u.add_argument("--hide-turn-behind-ball", action="store_true", help="draw faint what the ball hides of the turn's axis and arc"); u.add_argument("--fit-hand", default="leap"); u.add_argument("--crop-margin", type=int, default=70); u.add_argument("--fit-dist", type=float, default=.46); u.add_argument("--fit-lift", type=float, default=.04)
    u.add_argument("--leader", default="tennis_new_cup_take3_handframe__nframesfix"); u.add_argument("--runs", nargs="*", default=[]); u.add_argument("--out", required=True)
    u.add_argument("--start", type=float, default=11.0); u.add_argument("--seconds", type=float, default=10.0)
    u.add_argument("--every", type=float, default=2.0); u.add_argument("--hold", type=float, default=1.3); u.add_argument("--end", type=float, default=5.0)
    u.add_argument("--fps", type=int, default=30); u.add_argument("--stills", type=float, nargs="*"); u.add_argument("--stills-dir")
    u.add_argument("--az", type=float, default=90); u.add_argument("--el", type=float, default=-35); u.add_argument("--dist", type=float, default=.56)
    u.add_argument("--alpha", type=float, default=.75); u.add_argument("--arrow-R", type=float, default=2.0); u.add_argument("--look-back", type=float, default=.04)
    u.add_argument("--checks", help="a tools/position_turning_check.py output folder: no force targets; position and turning checks instead"); u.add_argument("--check-hands", nargs="*", default=["allegro", "leap", "shadow"])
    u.add_argument("--fit-hands", type=int, default=1); u.add_argument("--fit-seconds", type=float, default=2.0); u.add_argument("--fit-close", type=float, default=.8)
    A = a.parse_args()
    if A.part == "human" and A.video: A.hold, A.end, A.arrow_R, A.alpha = (2.2 if A.hold == 1.3 else A.hold), (4.0 if A.end == 5.0 else A.end), (1.2 if A.arrow_R == 2.0 else A.arrow_R), (.55 if A.alpha == .75 else A.alpha)
    {"hora": hora, "human": human}[A.part](A)


if __name__ == "__main__":
    main()
