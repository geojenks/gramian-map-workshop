"""Clip: four two-pneu-net fingers, mounted like the fingers of a hand and
drawn in their real settled shapes, following the contact points of one
round of a recorded learned rotation. GEOMETRY ONLY: nothing is simulated
here. The cylinder is the recorded one, replayed; each finger is drawn in
the settled free shape its two inputs give, with its tip face put on the
logged contact point. No forces, no contact mechanics: the clip shows where
the fingers can be, not that they would turn the cylinder.

Inputs:
  - placements and input pairs: reach_check_results.json of
    tools/two_pneunet_fingers_human_like_mounting_reach_check_21nov.py (the
    mounting that tool chose by its fixed rule, or --mounting "<name>");
  - shapes: finger_shapes.npz of tools/two_pneunet_finger_shapes_export_21nov.py
    (settled shapes of saved SOFA runs, through the 20 Nov shape model);
  - between touching stretches the inputs follow the return route the reach
    check judged (the earlier clip tool's route code, reused), at even tip
    speed along it; between tabulated input pairs the skin is interpolated
    linearly.

Three columns, one camera: the recording; the logged contacts; the pneu-net
fingers on the replayed cylinder. Small dots in the third column are the
logged contact points the tip faces should sit on.

MEASURES, fixed before the first run, taken on the frames of the clip:
  1. per finger, distance of the tip face's centre from the logged contact
     point at the frames where the log says touching (median, largest);
  2. deepest cut of any skin node into the replayed cylinder, per finger,
     over all frames (touching and returning);
  3. smallest distance between skin nodes of different fingers over all
     frames (nodes are about a millimetre apart, so under about 1 mm means
     touching or overlapping; this does not detect one finger inside another);
  4. lowest skin node against the palm plate's top (the plate is drawn 2 mm
     below the lowest clamped end);
  5. (added after the first clip, before this was computed) measure 1 split
     into the first, middle and last third of each touching stretch, pooled
     per finger, to show where along the stroke a miss sits.

Drawn over the third column, always on top (the fingers would hide them):
a black ring at each logged contact point being touched now, a filled dot in
the finger's colour at its tip face's centre, and a line between the two. The
length of that line is measure 1. The frame is padded to 1920 x 1080.

Run from the repository root (needs mujoco, ffmpeg):
    python tools/two_pneunet_fingers_human_like_mounting_clip_21nov.py \
        --wave-manip <Wave_manip> --gramian-map <gramian_map>
Writes into outputs/two_pneunet_fingers_human_like_mounting_21nov/; refuses to
overwrite a clip that exists (use --tag for another view).
--smoke: draw one still of the four fingers at upright starting placements,
without the fit's results, to check the drawing.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import mujoco as mj                                                            # noqa: E402
import pneunet_arrangement_replicates_rl_stroke_20nov as A                     # noqa: E402
import pneunet_upright_arrangement_replicates_rl_stroke_21nov as U             # noqa: E402
import two_pneunet_fingers_follow_rl_gait_reach_check_21nov as RC              # noqa: E402
import two_pneunet_fingers_follow_rl_gait_clip_21nov as C                      # noqa: E402
import two_pneunet_fingers_human_like_mounting_reach_check_21nov as HM         # noqa: E402

OUT = HM.OUT
CLIP_SLOW = 3.0           # the clip shows the recording this many times slower
CW, CH, HEAD, FOOT = 640, 720, 96, 92
FPS = A.FPS
COL = A.COL


def flat(v_tri):
    """Per-corner normals of unshared triangles (n, 3, 3) -> (3n, 3)."""
    n = np.cross(v_tri[:, 1] - v_tri[:, 0], v_tri[:, 2] - v_tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    return np.repeat(n, 3, axis=0)


class Skins:
    """The four fingers as meshes in one MuJoCo model with the replayed cylinder; vertices rewritten per frame."""

    def __init__(self, L, s0, R, half, tri, rest_world, plate_z):
        self.tri = tri
        q0 = L["q"][s0]
        nt = len(tri)
        face = " ".join(map(str, range(3 * nt)))
        assets, geoms = [], []
        for k, (name, V) in enumerate(rest_world):
            c = 0.45 * np.array(COL[name]) + 0.55 * np.array([0.86, 0.9, 0.95])
            assets.append(f'<mesh name="f{k}" vertex="{" ".join(f"{x:.7g}" for x in V[tri].reshape(-1))}" face="{face}"/>')
            geoms.append(f'<geom name="f{k}" type="mesh" mesh="f{k}" rgba="{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} 1" '
                         'contype="0" conaffinity="0"/>')
        xml = (f'<mujoco><visual><global offwidth="{CW}" offheight="{CH}"/>'
               '<headlight ambient="0.42 0.42 0.42" diffuse="0.62 0.62 0.62" specular="0.05 0.05 0.05"/></visual>'
               '<asset><texture type="skybox" builtin="flat" rgb1="1 1 1" rgb2="1 1 1" width="32" height="32"/>'
               + "".join(assets) + "</asset><worldbody>"
               f'<geom type="cylinder" size="0.075 0.002" pos="{L["op"][s0][0]:.6g} {L["op"][s0][1]:.6g} {plate_z - 0.002:.6g}" '
               'rgba="0.55 0.57 0.62 1" contype="0" conaffinity="0"/>'
               f'<body name="obj" mocap="true" pos="{C.vec(L["op"][s0])}" quat="{q0[3]:.9g} {q0[0]:.9g} {q0[1]:.9g} {q0[2]:.9g}">'
               f'<geom type="cylinder" size="{R:.9g} {half:.9g}" rgba="0.95 0.75 0.3 0.55" contype="0" conaffinity="0"/></body>'
               + "".join(geoms) + "</worldbody></mujoco>")
        self.m = m = mj.MjModel.from_xml_string(xml)
        self.d = d = mj.MjData(m)
        mj.mj_forward(m, d)
        self.ren = mj.Renderer(m, CH, CW)
        self.ids, self.check = [], []
        for k, (name, V) in enumerate(rest_world):
            g = m.geom(f"f{k}").id
            mid = int(m.geom_dataid[g])
            va, vn = int(m.mesh_vertadr[mid]), int(m.mesh_vertnum[mid])
            na, nn = int(m.mesh_normaladr[mid]), int(m.mesh_normalnum[mid])
            fa, fn = int(m.mesh_faceadr[mid]), int(m.mesh_facenum[mid])
            assert vn == 3 * nt and fn == nt, f"MuJoCo changed the mesh: {vn} vertices, {fn} faces for {3 * nt}, {nt}"
            assert nn == vn and np.array_equal(m.mesh_facenormal[fa:fa + fn], m.mesh_face[fa:fa + fn]), "normals are not per vertex"
            assert np.array_equal(m.mesh_face[fa:fa + fn].reshape(-1), np.arange(3 * nt)), "MuJoCo reordered the faces"
            Rg, pg = d.geom_xmat[g].reshape(3, 3).copy(), d.geom_xpos[g].copy()
            back = m.mesh_vert[va:va + vn] @ Rg.T + pg
            self.check.append(float(np.abs(back - V[tri].reshape(-1, 3)).max()))
            self.ids.append((mid, va, vn, na, Rg, pg))

    def set(self, k, V):
        """Write finger k's skin (world coordinates, shared nodes) into its mesh and send it to the renderer."""
        mid, va, vn, na, Rg, pg = self.ids[k]
        vt = V[self.tri]
        self.m.mesh_vert[va:va + vn] = (vt.reshape(-1, 3) - pg) @ Rg
        self.m.mesh_normal[na:na + vn] = flat(vt) @ Rg
        self.ren._gl_context.make_current()
        mj.mjr_uploadMesh(self.m, self.ren._mjr_context, mid)


class Shape:
    def __init__(self, sh, x):
        self.skin, self.lev = sh["skin"], sh["levels"]
        self.pad_tab = sh["pad"].astype(float)
        self.x, self.Rm = np.array(x[:3]), RC.rodrigues(np.array(x[3:6]))

    def _w(self, p1, p2):
        a = self.lev
        u = np.clip((p1 - a[0]) / (a[1] - a[0]), 0, len(a) - 1 - 1e-9)
        v = np.clip((p2 - a[0]) / (a[1] - a[0]), 0, len(a) - 1 - 1e-9)
        i, j = int(u), int(v)
        u, v = u - i, v - j
        return ((i, j, (1 - u) * (1 - v)), (i + 1, j, u * (1 - v)), (i, j + 1, (1 - u) * v), (i + 1, j + 1, u * v))

    def world(self, p1, p2):
        V = sum(w * self.skin[i, j].astype(float) for i, j, w in self._w(p1, p2))
        return V @ self.Rm.T + self.x

    def pad(self, p1, p2):
        return sum(w * self.pad_tab[i, j] for i, j, w in self._w(p1, p2)) @ self.Rm.T + self.x


def min_dist(a, b):
    """Smallest distance between two node sets (coarse grid of b first to keep it quick)."""
    best = np.inf
    for s in range(0, len(a), 400):
        d = np.linalg.norm(a[s:s + 400, None, :] - b[None, :, :], axis=-1)
        best = min(best, float(d.min()))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", required=True)
    ap.add_argument("--gramian-map", required=True)
    ap.add_argument("--mounting", default=None)
    ap.add_argument("--azimuth", type=float, default=35.0)
    ap.add_argument("--elevation", type=float, default=-25.0)
    ap.add_argument("--tag", default="")
    ap.add_argument("--smoke", action="store_true")
    args = ap.parse_args()
    from PIL import Image, ImageDraw, ImageFont
    W = HM.setup(args.wave_manip, args.gramian_map)
    L, R, half, tab, sh, idx = W["L"], W["R"], W["half"], W["tab"], W["sh"], W["idx"]
    s0, s1 = W["s0"], W["s1"]
    t = L["t"]
    tri = sh["tri"]
    C.FINGER = lambda L_, idx_, j_, tab_, R_, half_: HM.Mounted(L_, idx_, j_, tab_, R_, half_, None)   # noqa: E731

    if args.smoke:
        shapes = []
        for j in range(4):
            F = HM.Mounted(L, idx, j, tab, R, half, 15.0)
            shapes.append(Shape(sh, F.starts(np.random.default_rng(0))[1]))
        plate = min(S.x[2] for S in shapes) - 0.002
        sk = Skins(L, s0, R, half, tri, [(L["names"][j], shapes[j].world(0, 0)) for j in range(4)], plate)
        print("largest difference between the supplied rest skin and what MuJoCo holds, m:", sk.check)
        cam = mj.MjvCamera()
        cam.lookat[:] = L["op"][s0] + [0, 0, -0.03]
        cam.distance, cam.azimuth, cam.elevation = 0.42, args.azimuth, args.elevation
        ims = []
        for p1, p2 in ((0, 0), (60, 90), (120, 120)):
            for j in range(4):
                sk.set(j, shapes[j].world(p1, p2))
            sk.ren.update_scene(sk.d, cam)
            ims.append(sk.ren.render().copy())
        f = os.path.join(OUT, "drawing_check_upright_starts_not_fitted.png")
        Image.fromarray(np.hstack(ims)).save(f)
        print("wrote", f)
        return

    res = json.load(open(os.path.join(OUT, "reach_check_results.json")))
    name = args.mounting or res["chosen_for_the_clips"]
    fits = res["mountings"][name]
    plans = [C.Plan(L, idx, j, tab, R, half, fits[j], None) for j in range(4)]
    shapes = [Shape(sh, fits[j]["x"]) for j in range(4)]
    plate = min(S.x[2] for S in shapes) - 0.002
    slug = name.replace(",", "").replace(" ", "_").replace("-", "_") + (("_" + args.tag) if args.tag else "")
    base = os.path.join(OUT, f"recorded_contacts_pneunet_fingers_{slug}")
    assert not os.path.exists(base + ".mp4"), "this clip exists already; use --tag"
    sk = Skins(L, s0, R, half, tri, [(L["names"][j], shapes[j].world(0, 0)) for j in range(4)], plate)
    assert max(sk.check) < 1e-6, f"MuJoCo's mesh differs from the supplied skin: {sk.check}"

    try:
        font = ImageFont.truetype("arial.ttf", 24)
        small = ImageFont.truetype("arial.ttf", 19)
    except OSError:
        font = small = ImageFont.load_default()
    Wt, Ht = 3 * CW, 1080
    TOP = (Ht - (HEAD + CH + FOOT)) // 2
    ff = shutil.which("ffmpeg") or r"C:\Program Files\ffmpeg\bin\ffmpeg.exe"
    proc = subprocess.Popen(
        [ff, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{Wt}x{Ht}",
         "-r", str(FPS), "-i", "-", "-pix_fmt", "yuv420p", "-vcodec", "libx264", base + ".mp4"], stdin=subprocess.PIPE)
    files = [base + ".mp4"]
    cam = mj.MjvCamera()
    cam.lookat[:] = L["op"][s0:s1 + 1].mean(0) + [0, 0, 0.45 * (plate - L["op"][s0][2])]
    cam.distance, cam.azimuth, cam.elevation = 0.42, args.azimuth, args.elevation
    S = A.recorded_scene(args.wave_manip, L, R, half)
    S["m"].vis.global_.offwidth, S["m"].vis.global_.offheight = CW, CH
    mC = U.contacts_scene(L, {"s": s0}, R, half)
    models = [S["m"], mC]
    datas = [mj.MjData(m) for m in models]
    rens = [mj.Renderer(m, CH, CW) for m in models] + [sk.ren]
    datas.append(sk.d)
    models.append(sk.m)
    Z = L["Z"]
    titles = ["Recorded: learned policy, four-finger hand", "Extracted: contacts as they make and break",
              "Four two-pneu-net fingers placed on those contacts"]
    foot = [f"All three play together, {CLIP_SLOW:g} times slower than the recording. Log {t[s0]:.2f} to {t[s1]:.2f} s.",
            "Small dots: logged contact paths. Large dots: touching now. Arrows: logged normal force. Black line: fixed to the cylinder.",
            "Right: GEOMETRY ONLY, nothing simulated. Recorded cylinder replayed; fingers drawn in their settled free shapes "
            "(saved simulations). Comparative, not predictive."]

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
        U.connector(scene, mj.mjtGeom.mjGEOM_CAPSULE, 0.0015, p + A.rot(q, np.array([0, 0, half + 0.001])),
                    p + A.rot(q, np.array([R, 0, half + 0.001])), (0.1, 0.1, 0.1, 1))

    n_frames = int(round((t[s1] - t[s0]) * CLIP_SLOW * FPS)) + 1
    err = [[] for _ in range(4)]
    err_n = [[] for _ in range(4)]
    cut = [0.0] * 4
    gap_between, low = np.inf, np.inf
    stills = {0: "_start.png", n_frames // 2: "_middle.png", n_frames - 1: "_end.png"}
    for n in range(n_frames):
        tt = t[s0] + n / (CLIP_SLOW * FPS)
        k = int(np.clip(np.searchsorted(t, tt), s0, s1))
        pose = (L["op"][k], L["q"][k])
        now = {j: L["P"][k, j] for j in range(4) if L["lam"][k, j] > A.TOUCH_N}
        A.pose_recorded(S, datas[0], Z, k)

        def ex0(scene):
            turn_line(scene, Z["obj"][k][:3].astype(float), Z["obj"][k][3:7].astype(float))
            for j, p in now.items():
                A.add_sphere(scene, p, 0.0035, (*COL[L["names"][j]], 1))
        im0 = shot(0, ex0)
        for d_, m_ in ((datas[1], mC), (sk.d, sk.m)):
            d_.mocap_pos[0] = pose[0]
            d_.mocap_quat[0] = pose[1][[3, 0, 1, 2]]
            mj.mj_forward(m_, d_)

        def ex1(scene):
            turn_line(scene, *pose)
            for j in range(4):
                for kk in range(max(s0, k - 90), k + 1, 3):
                    if L["lam"][kk, j] > A.TOUCH_N:
                        A.add_sphere(scene, L["P"][kk, j], 0.0016, (*COL[L["names"][j]], 1))
            for j, p in now.items():
                c = (*COL[L["names"][j]], 1)
                A.add_sphere(scene, p, 0.0035, c)
                U.connector(scene, mj.mjtGeom.mjGEOM_ARROW, 0.004, p - 0.03 * L["lam"][k, j] * L["nrm"][k, j], p, c)
        im1 = shot(1, ex1)
        Vs, pads = [], {}
        for j, (P, Sh) in enumerate(zip(plans, shapes)):
            p1, p2 = P.inputs(t[k])
            V = Sh.world(p1, p2)
            Vs.append(V)
            sk.set(j, V)
            cut[j] = max(cut[j], -float(RC.cyl_sdf(V, L["op"][k], L["axis"][k], R, half).min()))
            low = min(low, float(V[:, 2].min()) - plate)
            if j in now and P.F.touch[int(np.argmin(np.abs(P.F.t - t[k])))]:
                pads[j] = Sh.pad(p1, p2)
                err[j].append(float(np.linalg.norm(pads[j] - now[j])))
                err_n[j].append(n)
        if n % 3 == 0:
            for a in range(4):
                for b in range(a + 1, 4):
                    gap_between = min(gap_between, min_dist(Vs[a][::3], Vs[b][::3]))

        def ex2(scene):
            turn_line(scene, *pose)
            for j, p in now.items():
                A.add_sphere(scene, p, 0.0022, (0, 0, 0, 1))
        im2 = shot(2, ex2)
        im = Image.new("RGB", (Wt, Ht), (255, 255, 255))
        im.paste(Image.fromarray(np.hstack([im0, im1, im2])), (0, TOP + HEAD))
        dr = ImageDraw.Draw(im)
        cams = sk.ren.scene.camera
        cp = 0.5 * (np.array(cams[0].pos) + np.array(cams[1].pos))
        fw, up = np.array(cams[0].forward, float), np.array(cams[0].up, float)
        rt = np.cross(fw, up)
        foc = 0.5 * CH / np.tan(np.radians(sk.m.vis.global_.fovy) / 2)

        def px(p):
            r = np.asarray(p, float) - cp
            return (2 * CW + CW / 2 + foc * (r @ rt) / (r @ fw), TOP + HEAD + CH / 2 - foc * (r @ up) / (r @ fw))
        for j, pd in pads.items():
            a, b = px(now[j]), px(pd)
            c = tuple(int(255 * x) for x in COL[L["names"][j]])
            dr.line([a, b], fill=(0, 0, 0), width=3)
            dr.ellipse([b[0] - 6, b[1] - 6, b[0] + 6, b[1] + 6], fill=c, outline=(0, 0, 0))
            dr.ellipse([a[0] - 7, a[1] - 7, a[0] + 7, a[1] + 7], outline=(0, 0, 0), width=3)
        tw = np.degrees(L["turn"][k] - L["turn"][s0])
        tch = " ".join(L["names"][j] for j in now)
        seconds = [f"log time {t[k]:.2f} s   cylinder turned {tw:+.1f} deg", f"touching: {tch}",
                   "ring: logged contact now. dot: tip face centre"]
        for i in range(3):
            dr.text((i * CW + 16, TOP + 12), titles[i], fill=(0, 0, 0), font=font)
            dr.text((i * CW + 16, TOP + 48), seconds[i], fill=(0, 0, 0), font=small)
            if i:
                dr.line([(i * CW, TOP + HEAD), (i * CW, TOP + HEAD + CH)], fill=(200, 200, 200), width=1)
        for i in range(3):
            dr.text((16, TOP + HEAD + CH + 6 + 26 * i), foot[i], fill=(0, 0, 0), font=small)
        buf = np.asarray(im).tobytes()
        for _ in range(FPS if n in (0, n_frames - 1) else 1):
            proc.stdin.write(buf)
        if n in stills:
            im.save(base + stills[n])
            files.append(base + stills[n])
    proc.stdin.close()
    proc.wait()
    for ren in rens:
        ren.close()
    rep = [f"Clip of four two-pneu-net fingers on the recorded contacts: mounting \"{name}\"",
           "GEOMETRY ONLY, nothing simulated. Comparative, not predictive.", "",
           f"Camera azimuth {args.azimuth:g} deg, elevation {args.elevation:g} deg; {n_frames} frames, log {t[s0]:.2f} to {t[s1]:.2f} s, "
           f"shown {CLIP_SLOW:g} times slower.",
           f"Drawing check: MuJoCo's mesh against the supplied skin, largest difference {max(sk.check) * 1e6:.3f} micrometres.", "",
           "Measures on the frames of the clip:"]
    out = {"mounting": name, "frames": n_frames, "fingers": {}}
    for j, P in enumerate(plans):
        e = np.array(err[j]) * 1e3
        rep.append(f"  {P.name:6s} tip face centre from the logged contact while touching: median {np.median(e):.2f} mm, largest "
                   f"{e.max():.2f} mm ({len(e)} frames); deepest cut of any skin node into the replayed cylinder {cut[j] * 1e3:.2f} mm")
        nn, th = np.array(err_n[j]), [[], [], []]
        for run in np.split(np.arange(len(nn)), np.where(np.diff(nn) > 1)[0] + 1):
            for q, part in enumerate(np.array_split(run, 3)):
                th[q] += list(e[part])
        th = [float(np.median(x)) for x in th]
        rep.append(f"         by third of each touching stretch (first, middle, last), median: {th[0]:.2f}, {th[1]:.2f}, {th[2]:.2f} mm")
        out["fingers"][P.name] = {"contact_error_median_by_third_of_stretch_mm": th,"contact_error_median_mm": float(np.median(e)), "contact_error_max_mm": float(e.max()),
                                  "frames_touching": len(e), "deepest_skin_cut_mm": cut[j] * 1e3}
    rep += [f"  Smallest distance between skin nodes of different fingers (every third node, every third frame): {gap_between * 1e3:.2f} mm.",
            f"  Lowest skin node above the palm plate's top: {low * 1e3:.2f} mm.", "", "Files:"] + ["  " + os.path.relpath(f, ROOT) for f in files]
    out.update(smallest_distance_between_fingers_mm=gap_between * 1e3, lowest_skin_node_above_plate_mm=low * 1e3,
               files=[os.path.relpath(f, ROOT) for f in files])
    with open(base + "_report.txt", "w", encoding="utf-8") as fh:
        fh.write("\n".join(rep) + "\n")
    with open(base + "_measures.json", "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=1)
    print("\n".join(rep))


if __name__ == "__main__":
    main()
