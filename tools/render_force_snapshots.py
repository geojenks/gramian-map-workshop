"""Short video: the forces a simulated manipulation needs, saved as snapshots, then
hands checked against those snapshots.

Shot 1 plays a recorded leader file with its logged contact forces. At each chosen
frame a snapshot is saved. Later shots hold the ball at each snapshot and show a
stick drawing of a hand closing onto it, in the pose stored by the follow check.

Every number on screen is read from the leader file or the follow result files.
The hand poses come from a static check of each frozen frame: the closing motion
is illustration, from the hand's rest pose, and no hand is simulated doing the task.
Hands are drawn as sticks because the check does not test fingers against each
other, and solved fingers can pass through each other.

Run with an interpreter that has mujoco, numpy and PIL, with ffmpeg on PATH:
    python tools/render_force_snapshots.py --leader <id> --pass-run a.json [b.json]
        --fail-run c.json --inset finger.gif --out out.mp4
"""
import argparse, json, math, os, subprocess, sys, textwrap
import numpy as np, mujoco as mj
from PIL import Image, ImageDraw, ImageFont, ImageSequence
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.follow import FingerIK
from gramian_workshop.hands import Hand

COL = {"green": (.15, .66, .36, 1), "amber": (.92, .63, 0, 1), "red": (.82, .22, .2, 1)}
RGB = {"green": (38, 168, 92), "amber": (235, 160, 0), "red": (209, 56, 51)}
ORANGE, RED = (.93, .36, .05, 1), (.82, .1, .1, 1)
BG = (250, 250, 248); INK = (30, 30, 30); SOFT = (105, 108, 112)
STICK, IDLE = (.3, .32, .36, 1), (.78, .79, .81, 1)
FOV = 30
LOOK_BELOW, AZ, EL, DIST = .02, 120, -15, .34   # camera, relative to the ball's start


def add(s, t, sz, p, rgba):
    g = s.geoms[s.ngeom]
    mj.mjv_initGeom(g, t, np.array(sz, float), np.array(p, float), np.eye(3).ravel(), np.array(rgba, np.float32))
    s.ngeom += 1


def link(s, t, w, a, b, rgba):
    g = s.geoms[s.ngeom]
    mj.mjv_initGeom(g, t, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.array(rgba, np.float32))
    s.ngeom += 1
    mj.mjv_connector(g, t, w, np.array(a, float), np.array(b, float))


class View:
    """A camera given in the recording's frame, applied to any model through its placement."""
    def __init__(v, look, az, el, dist):
        a, e = math.radians(az), math.radians(el)
        v.fwd = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
        v.up = np.array([-math.sin(e) * math.cos(a), -math.sin(e) * math.sin(a), math.cos(e)])
        v.pos = np.array(look) - dist * v.fwd

    def apply(v, scn, Rw=np.eye(3), mapped=None):
        pos = v.pos if mapped is None else mapped(v.pos)
        for c in scn.camera:
            c.pos[:] = pos; c.forward[:] = Rw @ v.fwd; c.up[:] = Rw @ v.up
        for i in range(scn.nlight):
            if scn.lights[i].headlight:
                scn.lights[i].pos[:] = pos; scn.lights[i].dir[:] = Rw @ v.fwd


def shoot(r):
    rgb = r.render().copy()
    r.enable_depth_rendering(); dep = r.render().copy(); r.disable_depth_rendering()
    dep[dep >= dep.max() * .999] = np.inf
    return rgb, dep


def nearer(a, b):
    out = a[0].copy(); m = b[1] < a[1]; out[m] = b[0][m]
    return out, np.minimum(a[1], b[1])


class Stage:
    """The ball, the recording's contact stations and the force arrows, in the recording's frame."""
    def __init__(s, L, W, H):
        s.L, s.F, s.W, s.H = L, L["frames"], W, H
        s.R = L["object"]["radius_m"]; s.c0 = np.array(s.F[0]["obj_pos"]); s.n = L["n_stations"]
        s.fmax = max(np.linalg.norm(a) for f in s.F for a in f["force_on_object_N"])
        s.arrow = 1.5 * s.R / s.fmax
        s.tip = float(np.median([np.linalg.norm(np.array(f["contact_point"][i]) - f["station_pos"][i])
                                 for f in s.F for i in range(s.n) if f["contact"][i] and f["contact_point"][i]]))
        s.v = View(s.c0 - [0, 0, LOOK_BELOW], AZ, EL, DIST)
        xml = f"""<mujoco><visual><global offwidth="{W}" offheight="{H}" fovy="{FOV}"/>
        <headlight ambient=".5 .5 .5" diffuse=".55 .55 .55" specular="0 0 0"/></visual>
        <asset><texture name="t" type="cube" builtin="checker" rgb1=".96 .94 .86" rgb2=".32 .45 .62" width="256" height="256"/>
        <material name="m" texture="t"/></asset><worldbody><body mocap="true">
        <geom type="sphere" size="{s.R}" material="m" contype="0" conaffinity="0"/></body></worldbody></mujoco>"""
        s.m = mj.MjModel.from_xml_string(xml); s.d = mj.MjData(s.m)
        s.r = mj.Renderer(s.m, H, W); s.cam = mj.MjvCamera()
        s.hide = mj.MjvOption(); s.hide.geomgroup[:] = 0
        s.cache = {}

    def ball(s, k, stations):
        key = (k, stations)
        if key in s.cache: return s.cache[key]
        f = s.F[k]
        s.d.mocap_pos[0] = f["obj_pos"]; s.d.mocap_quat[0] = f["obj_quat"]; mj.mj_forward(s.m, s.d)
        s.r.update_scene(s.d, s.cam); g = s.r.scene
        if stations:
            for i in range(s.n):
                p = np.array(f["station_pos"][i]); c = (.36, .38, .42, 1) if f["contact"][i] else (.74, .76, .78, 1)
                add(g, mj.mjtGeom.mjGEOM_SPHERE, [s.tip, 0, 0], p, c)
                link(g, mj.mjtGeom.mjGEOM_CAPSULE, s.tip * .55, p, p - [0, 0, .035], c)
        s.v.apply(g); out = shoot(s.r)
        if not stations: s.cache[key] = out
        return out

    def overlay(s, k, pads=(), missing=()):
        """Arrows and fingertip dots, drawn over everything so nothing hides them."""
        s.r.update_scene(s.d, s.cam, s.hide); g = s.r.scene; f = s.F[k]
        for i in range(s.n):
            if f["contact"][i] and f["contact_point"][i]:
                cp = np.array(f["contact_point"][i])
                link(g, mj.mjtGeom.mjGEOM_ARROW, .0032, cp, cp + s.arrow * np.array(f["force_on_object_N"][i]), RED if i in missing else ORANGE)
                if i in missing: add(g, mj.mjtGeom.mjGEOM_SPHERE, [.0062, 0, 0], cp, RED)
        for p, c in pads: add(g, mj.mjtGeom.mjGEOM_SPHERE, [.0048, 0, 0], p, c)
        s.v.apply(g); return shoot(s.r)

    def picture(s, layers, top):
        rgb, dep = layers[0]
        for l in layers[1:]: rgb, dep = nearer((rgb, dep), l)
        rgb = rgb.copy(); rgb[np.isinf(dep)] = BG
        m = np.isfinite(top[1]); rgb[m] = top[0][m]
        return rgb


class Rig:
    """One hand and its stored follow result, drawn as a stick hand, or with full=True as the
    model's own visual meshes."""
    def __init__(g, path, stage, full=False):
        g.D = json.load(open(path, encoding="utf-8")); g.st = stage; g.full = full
        assert g.D["leader"] == stage.L["id"], "result is for a different leader"
        g.Rw = np.array(g.D["placement"]["rotation_matrix"]); g.tw = np.array(g.D["placement"]["translation_m"])
        g.h = Hand(g.D["hand"]); m = g.m = g.h.model; g.d = mj.MjData(m)
        m.vis.global_.offwidth = stage.W; m.vis.global_.offheight = stage.H; m.vis.global_.fovy = FOV
        m.vis.headlight.active = 1; m.vis.headlight.ambient[:] = .5; m.vis.headlight.diffuse[:] = .55; m.vis.headlight.specular[:] = 0
        g.idx = {f["k"]: i for i, f in enumerate(g.D["frames"])}
        g.ik = {f.label: FingerIK(g.h, f) for f in g.h.fingers}
        owner, g.chain = {}, {}
        for f in g.h.fingers:
            first = m.jnt_bodyid[f.joints[0]]; b = f.tip_body; ch = []
            while b > 0:
                owner[b] = f.label; ch.append(b)
                if b == first: break
                b = int(m.body_parentid[b])
            g.chain[f.label] = [int(m.body_parentid[ch[-1]])] + ch[::-1]
        g.owner = owner
        if not full:
            for i in range(m.ngeom):
                m.geom_matid[i] = -1
                m.geom_rgba[i] = (0, 0, 0, 0) if int(m.geom_bodyid[i]) in owner else (.74, .72, .69, 1)
        g.o = mj.MjvOption(); g.o.geomgroup[:] = 0
        if full:   # the menagerie models keep their looks in a group above 0 (LEAP 1; Shadow, Allegro 2)
            g.vis_group = min(int(m.geom_group[i]) for i in range(m.ngeom) if m.geom_type[i] == mj.mjtGeom.mjGEOM_MESH and m.geom_group[i] > 0)
            for i in range(m.ngeom):   # LEAP's fingertip pads sit only in its contact group: without this the film draws its fingers with no pads
                if m.geom_type[i] == mj.mjtGeom.mjGEOM_MESH and m.geom_group[i] != g.vis_group and mj.mj_id2name(m, mj.mjtObj.mjOBJ_MESH, int(m.geom_dataid[i])).endswith('tip'):
                    print('drawing fingertip pad', mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, i)); m.geom_group[i] = g.vis_group
            g.o.geomgroup[g.vis_group] = 1
            m.mat_rgba[:, :3] = .36 + .64 * m.mat_rgba[:, :3]; m.geom_rgba[:, :3] = .36 + .64 * m.geom_rgba[:, :3]   # these hands are near black: lifted so their shape reads on screen
            m.vis.headlight.specular[:] = .12
        else:
            g.o.geomgroup[1 if any(m.geom_group[i] == 1 for i in range(m.ngeom)) else 0] = 1
        g.r = mj.Renderer(m, stage.H, stage.W); g.cam = mj.MjvCamera(); g.attach = {}

    def to_leader(g, p): return g.Rw.T @ (np.array(p) - g.tw) + g.st.c0
    def to_hand(g, p): return g.Rw @ (np.array(p) - g.st.c0) + g.tw

    def pose(g, fr, a):
        q = np.array(g.h.base_q, float)
        for f, x in zip(g.h.fingers, fr["fingers"]):
            if "q" in x:
                ad = g.ik[f.label].qadr; q[ad] = q[ad] + a * (np.array(x["q"]) - q[ad])
        g.d.qpos[:] = g.h.apply_coupling(q); mj.mj_kinematics(g.m, g.d)

    def anchor(g, fr, f, x):
        """Where on the finger the stored contact sits: nearest link, held in that link's frame."""
        key = (fr["k"], f.label)
        if key not in g.attach:
            g.pose(fr, 1.0); p = np.array(x["pad_world"]); ch = g.chain[f.label][1:]
            b = min(ch, key=lambda b: np.linalg.norm(g.d.xpos[b] - p)) if x.get("phalanx") == "middle" else ch[-1]
            g.attach[key] = (b, g.d.xmat[b].reshape(3, 3).T @ (p - g.d.xpos[b]))
        return g.attach[key]

    def overlaps(g, k, inside):
        """At frame k, fully closed: how far the drawn meshes sink into the object (inside: points
        in the recording's frame to depth, positive inside), part by part, and how far the hand's
        own collision shapes sink into each other."""
        from gramian_workshop.finger_actuator import self_collisions
        fr = g.D["frames"][g.idx[k]]; g.pose(fr, 1.0); m = g.m; out = {}
        for i in range(m.ngeom):
            if m.geom_type[i] != mj.mjtGeom.mjGEOM_MESH or m.geom_group[i] != g.vis_group: continue
            a, n = m.mesh_vertadr[m.geom_dataid[i]], m.mesh_vertnum[m.geom_dataid[i]]
            X = g.d.geom_xpos[i] + m.mesh_vert[a:a + n] @ g.d.geom_xmat[i].reshape(3, 3).T
            part = g.owner.get(int(m.geom_bodyid[i]), "palm")
            out[part] = max(out.get(part, -np.inf), float(inside((X - g.tw) @ g.Rw + g.st.c0).max()))
        own = self_collisions(m, g.d.qpos.copy())
        return out, max(own, key=lambda c: c[2]) if own else None

    def layer(g, k, a):
        fr = g.D["frames"][g.idx[k]]
        anchors = [g.anchor(fr, f, x) if x["state"] != "off" else None for f, x in zip(g.h.fingers, fr["fingers"])]
        g.pose(fr, a); g.r.update_scene(g.d, g.cam, g.o); s = g.r.scene; pads = []
        for f, x, an in zip(g.h.fingers, fr["fingers"], anchors):
            on = x["state"] != "off"; c = STICK if on else IDLE
            if not g.full:
                pts = [g.d.xpos[b].copy() for b in g.chain[f.label]]
                for u, w in zip(pts, pts[1:]): link(s, mj.mjtGeom.mjGEOM_CAPSULE, .0026, u, w, c)
                for u in pts: add(s, mj.mjtGeom.mjGEOM_SPHERE, [.0042, 0, 0], u, c)
            if on:
                b, loc = an; p = g.d.xpos[b] + g.d.xmat[b].reshape(3, 3) @ loc
                if not g.full: link(s, mj.mjtGeom.mjGEOM_CAPSULE, .0026, g.d.xpos[b], p, c)
                if a >= .999: pads.append((g.to_leader(p), COL[x["state"]]))
        g.st.v.apply(s, g.Rw, g.to_hand)
        return shoot(g.r), pads, fr


class Sheet:
    """The whole frame: picture on the left, words and snapshot tiles on the right."""
    def __init__(c, scale):
        c.s = scale; c.W, c.H = int(1600 * scale) // 2 * 2, int(900 * scale) // 2 * 2
        c.P = int(900 * scale) // 2 * 2
        c.big = ImageFont.truetype("arialbd.ttf", int(36 * scale)); c.mid = ImageFont.truetype("arial.ttf", int(25 * scale))
        c.small = ImageFont.truetype("arial.ttf", int(19 * scale))
        c.tiles, c.borders = [], {}

    def frame(c, pic, head, body, tag=None, active=None, strips=None, inset=None, flash=0.0):
        im = Image.new("RGB", (c.W, c.H), BG); im.paste(Image.fromarray(pic), (0, 0)); dr = ImageDraw.Draw(im); s = c.s
        x, y = c.P + int(20 * s), int(44 * s)
        for ln in textwrap.wrap(head, 34):
            dr.text((x, y), ln, fill=INK, font=c.big); y += int(46 * s)
        y += int(10 * s)
        for ln in sum((textwrap.wrap(b, 50) for b in body), []):
            dr.text((x, y), ln, fill=INK, font=c.mid); y += int(34 * s)
        if inset is not None:
            im.paste(inset, (x, c.H - int(450 * s)))
        if strips: c.draw_strips(dr, x, int(430 * s), strips)
        ty = c.H - int(205 * s); tw = int(124 * s)
        for i, t in enumerate(c.tiles):
            tx = x + i * (tw + int(8 * s))
            im.paste(t, (tx, ty))
            col = c.borders.get(i)
            if col or active == i:
                dr.rectangle([tx - 2, ty - 2, tx + tw + 1, ty + tw + 1], outline=col or INK, width=max(2, int(5 * s)))
        dr.text((x, c.H - int(44 * s)), "Simulated: comparative, not predictive.", fill=SOFT, font=c.small)
        if tag: dr.text((int(20 * s), int(18 * s)), tag, fill=INK, font=c.mid)
        a = np.asarray(im, np.uint8)
        return (a * (1 - flash) + 255 * flash).astype(np.uint8) if flash else a

    def draw_strips(c, dr, x, y, strips):
        s = c.s; tw = max(2, int(13 * s))
        for name, res in strips:
            cnt = {v: sum(f["verdict"] == v for f in res) for v in RGB}
            dr.text((x, y), f"{name}:  {cnt['green']} green, {cnt['amber']} amber, {cnt['red']} red of {len(res)}", fill=INK, font=c.small); y += int(26 * s)
            for i, f in enumerate(res):
                dr.rectangle([x + i * (tw + 1), y, x + i * (tw + 1) + tw - 1, y + int(26 * s)], fill=RGB[f["verdict"]])
            y += int(44 * s)


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--leader", required=True); a.add_argument("--pass-run", nargs="+", required=True)
    a.add_argument("--fail-run"); a.add_argument("--inset"); a.add_argument("--out", required=True)
    a.add_argument("--snapshots", type=int, nargs="+", default=[60, 121, 182, 258, 296])
    a.add_argument("--scale", type=float, default=1.0); a.add_argument("--fps", type=int, default=30)
    A = a.parse_args()
    L = json.load(open(os.path.join(ROOT, "data", "leaders", A.leader + ".json"), encoding="utf-8"))
    c = Sheet(A.scale); st = Stage(L, c.P, c.H); F = L["frames"]; fps = A.fps
    t0 = F[0]["t"]; T = lambda k: F[k]["t"] - t0
    print(f"largest force {st.fmax:.3f} N, arrow {st.arrow:.4f} m/N, station tip {st.tip*1e3:.1f} mm, snapshots at", [round(T(k), 2) for k in A.snapshots], flush=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{c.W}x{c.H}", "-r", str(fps), "-i", "-",
                           "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", A.out], stdin=subprocess.PIPE)
    n = [0]
    def put(a):
        ff.stdin.write(a.tobytes()); n[0] += 1
        if n[0] % 100 == 0: print("frames", n[0], flush=True)
    def rec_pic(k): return st.picture([st.ball(k, True)], st.overlay(k))
    # Shot: the recording, saving snapshots as it passes them.
    head = "The forces a manipulation needs"
    body = [f"A simulated ball, {2*st.R*1e3:.0f} mm across, turned by a wave of {st.n} contacts.",
            f"Orange arrows: the force each contact puts on the ball. The longest in this recording is {st.fmax:.2f} N."]
    todo = list(A.snapshots)
    times = [f["t"] - t0 for f in F]
    for j in range(int(T(len(F) - 1) * fps) + 1):
        k = min(int(np.searchsorted(times, j / fps)), len(F) - 1)
        tag = f"Recording, t = {T(k):.2f} s"
        if todo and k >= todo[0]:
            k = todo.pop(0); pic = rec_pic(k)
            cut = int(c.P * .2)   # tile: the ball and contacts, not the empty margin
            c.tiles.append(Image.fromarray(pic[cut // 2:c.P - cut - cut // 2, cut:c.P - cut]).resize((int(124 * A.scale),) * 2, Image.LANCZOS))
            for w in range(int(.7 * fps)):
                put(c.frame(pic, head, body + [f"Snapshot {len(c.tiles)}: positions and forces saved as targets."], f"Recording, t = {T(k):.2f} s", active=len(c.tiles) - 1, flash=max(0, .7 - w / (.25 * fps)) if w < .25 * fps else 0))
            continue
        put(c.frame(rec_pic(k), head, body, tag))
    # Shots: each hand against the frozen snapshots.
    def fit(rig, head, body, close, hold, inset_frames=None):
        c.borders = {}; u = 0
        for i, k in enumerate(A.snapshots):
            base = st.ball(k, False)
            for w in range(int((close + hold) * fps)):
                a = min(1.0, w / max(1, close * fps)); a = a * a * (3 - 2 * a)
                lay, pads, fr = rig.layer(k, a)
                miss = fr.get("unserved", []) if a >= .999 else []
                pic = st.picture([base, lay], st.overlay(k, pads, miss))
                if a >= .999: c.borders[i] = RGB[fr["verdict"]]
                ins = None
                if inset_frames: ins = inset_frames[u % len(inset_frames)]; u += 1
                put(c.frame(pic, head, body + ([f"Snapshot {i+1}: {fr['verdict']}."] if a >= .999 else [f"Snapshot {i+1}: fitting."]), active=i, inset=ins))
    runs = [Rig(p, st) for p in A.pass_run]
    what = "Each snapshot is checked on its own, frozen. The hand is not simulated doing the task. Green passes, amber is close, red does not pass."
    label = {"leap": "LEAP", "allegro": "Allegro"}
    for i, r in enumerate(runs):
        nm = label.get(r.D["hand"], r.D["hand_name"])
        fit(r, f"Can {nm} supply these targets?" if i == 0 else f"The same targets, {nm}", [what] if i == 0 else [], .8 if i == 0 else .5, .8 if i == 0 else .6)
    strips = [(label.get(r.D["hand"], r.D["hand_name"]) + " (fresh run)", r.D["frames"]) for r in runs]
    if A.fail_run:
        w = Rig(A.fail_run, st); nf = len(w.h.fingers)
        inset = None
        if A.inset:
            g = Image.open(A.inset); side = int(235 * A.scale)
            box = (int(.16 * g.width), int(.07 * g.height), int(.84 * g.width), int(.93 * g.height))   # the actuator, without the plot's margins
            inset = [fr.convert("RGB").crop(box).resize((side, int(side * (box[3] - box[1]) / (box[2] - box[0]))), Image.LANCZOS) for fr in ImageSequence.Iterator(g)]
        fit(w, f"A {nf}-finger hand from our actuator records", [f"An example, on a {st.n}-contact wave. Red marks a contact with no finger. Below: the simulated actuator its fingers stand in for."], .8, .8, inset)
        d = sum(f["reason"] == "direction" for f in w.D["frames"]); e = sum(f["reason"] == "torque" for f in w.D["frames"])
        strips.append((f"{nf}-finger example", w.D["frames"]))
    # Shot: the whole check, every frame it looked at.
    k = A.snapshots[-1]; pic = st.picture([st.ball(k, False)], st.overlay(k))
    c.borders = {}
    body = ["A hand passes a frame if it can supply that frame's targets. Red means not by this method at this pose, not that the hand can never do the task."]
    if A.fail_run:
        body.append(f"With a contact left without a finger, the {nf}-finger example cannot push the ball the way the wave needs in {d} of {len(w.D['frames'])} frames. Its other {e} reds are provisional.")
    for _ in range(int(6 * fps)):
        put(c.frame(pic, "Every frame the check looked at", body, strips=strips))
    ff.stdin.close(); ff.wait()
    print("wrote", A.out, "frames", n[0], "seconds", round(n[0] / fps, 1), flush=True)
    os._exit(0)   # GL contexts can hang on exit


if __name__ == "__main__":
    main()
