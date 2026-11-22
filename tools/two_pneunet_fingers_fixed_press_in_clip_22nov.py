"""Clip of four two-pneu-net fingers, tips aimed a fixed depth inside the
surface, turning a cylinder beside the recording. DRAWS saved frames; runs
no simulation.

COPY of two_pneunet_fingers_turn_cylinder_on_axle_clip_22nov.py with the
titles and the lines under the picture changed for the 22 Nov fixed
press-in runs (--support axle: frames from
two_pneunet_fingers_turn_cylinder_on_axle_fixed_press_in_22nov.py;
--support free: frames from
two_pneunet_fingers_turn_free_cylinder_lighter_22nov.py, where the cylinder
is NOT on an axle). The drawing is unchanged. The text below is the
original tool's, kept for what is drawn.

Left: the recorded hand (learned policy) and its cylinder, replayed.
Right: the frames saved by tools/two_pneunet_fingers_turn_cylinder_on_axle_21nov.py
(a contact simulation): the simulated cylinder, and the four fingers drawn
in their settled free shapes for the inputs commanded at that moment.

What is simplified, and said on the clip:
  - The cylinder is on an axle: its centre and axis direction follow the
    recording; only its turn about its own axis is free.
  - In the simulation each finger acts through a tip ball on a spring. The
    ball is not drawn. The drawn skin is the finger's free shape, so it can
    cut into the cylinder where the ball does not touch; the simulation
    does not see that.
  - A coloured dot marks a tip ball pressing now, from the simulated normal
    force (above the same threshold as the simulation's report), at the
    point of the ball nearest the cylinder's axis.

The numbers on the clip are read from the saved frames. Nothing is tuned
here.

Run from the repository root:
    python tools/two_pneunet_fingers_turn_cylinder_on_axle_clip_22nov.py \
        --wave-manip <path to Wave_manip> --gramian-map <path to gramian_map> \
        --run-folder outputs/<folder written by the axle tool> [--key two_inputs_main]
Writes clip_<key>[_tag].mp4, three stills and a short report into the run
folder (new names only; refuses to overwrite: use --tag). --stills-only
writes the three stills and no clip.
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
import two_pneunet_fingers_human_like_mounting_reach_check_21nov as HM         # noqa: E402
import two_pneunet_fingers_human_like_mounting_clip_21nov as K                 # noqa: E402

CW, CH, HEAD, FOOT = 960, 830, 110, 140
K.CW, K.CH = CW, CH                       # the drawing classes of the 21 Nov clip tool, at this size
FPS = A.FPS
COL = A.COL
C0, NB = 13, 11                           # layout of the saved frames (see the axle tool's HEADER)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", required=True)
    ap.add_argument("--gramian-map", required=True)
    ap.add_argument("--run-folder", required=True)
    ap.add_argument("--key", default="two_inputs_main")
    ap.add_argument("--mounting-results", default=os.path.join(HM.OUT, "reach_check_results.json"))
    ap.add_argument("--mounting", default=None)
    ap.add_argument("--azimuth", type=float, default=35.0)
    ap.add_argument("--elevation", type=float, default=-25.0)
    ap.add_argument("--distance", type=float, default=0.33)
    ap.add_argument("--tag", default="")
    ap.add_argument("--support", choices=("axle", "free"), required=True)
    ap.add_argument("--stills-only", action="store_true")
    args = ap.parse_args()
    from PIL import Image, ImageDraw, ImageFont
    W = HM.setup(args.wave_manip, args.gramian_map)
    L, R, half, sh = W["L"], W["R"], W["half"], W["sh"]
    s0, s1 = W["s0"], W["s1"]
    t = L["t"]
    folder = args.run_folder
    results = json.load(open(os.path.join(folder, "axle_results.json")))
    radius = results["tip_ball_radius_m"]
    run = results["runs"][args.key]
    fr = np.loadtxt(os.path.join(folder, f"frames_{args.key}.csv"), delimiter=",", skiprows=1)
    assert fr.shape[1] == C0 + 4 * NB, "the saved frames do not have the expected columns"
    res = json.load(open(args.mounting_results))
    name = args.mounting or res["chosen_for_the_clips"]
    fits = res["mountings"][name]
    shapes = [K.Shape(sh, fits[j]["x"]) for j in range(4)]
    plate = min(S.x[2] for S in shapes) - 0.002
    base = os.path.join(folder, f"clip_{args.key}" + (("_" + args.tag) if args.tag else ""))
    assert not os.path.exists(base + ".mp4") and not os.path.exists(base + "_start.png"), "these files exist already; use --tag"
    sk = K.Skins(L, s0, R, half, sh["tri"], [(L["names"][j], shapes[j].world(0, 0)) for j in range(4)], plate)
    assert max(sk.check) < 1e-6, f"MuJoCo's mesh differs from the supplied skin: {sk.check}"
    try:
        font = ImageFont.truetype("arial.ttf", 30)
        small = ImageFont.truetype("arial.ttf", 22)
    except OSError:
        font = small = ImageFont.load_default()
    Wt, Ht = 2 * CW, 1080
    assert HEAD + CH + FOOT == Ht
    cam = mj.MjvCamera()
    cam.lookat[:] = L["op"][s0:s1 + 1].mean(0) + [0, 0, 0.45 * (plate - L["op"][s0][2])]
    cam.distance, cam.azimuth, cam.elevation = args.distance, args.azimuth, args.elevation
    S = A.recorded_scene(args.wave_manip, L, R, half)
    S["m"].vis.global_.offwidth, S["m"].vis.global_.offheight = CW, CH
    dS = mj.MjData(S["m"])
    renS = mj.Renderer(S["m"], CH, CW)
    Z = L["Z"]
    rec_total = results["recorded_turn_deg"]
    slow = (fr[-1, 0] - fr[0, 0]) / max(fr[-1, 2] - fr[0, 2], 1e-9) if fr[-1, 1] < 2.5 else None
    gait = fr[fr[:, 1] == 2]
    sim_slow = (gait[-1, 0] - gait[0, 0]) / (gait[-1, 2] - gait[0, 2])
    clip_slow = (len(gait) - 1) / FPS / (gait[-1, 2] - gait[0, 2])
    del slow
    depth = results["press_in_depth_m"] * 1e3
    press = (f"While the recording has a finger touching, its tip ball is aimed {depth:.1f} mm inside the cylinder's side "
             "(ASSUMED depth); the ball is not drawn.")
    if args.support == "axle":
        titles = ["Recorded: learned policy, four-finger hand", "Simulated: four two-pneu-net fingers, cylinder on an axle"]
        foot = [f"Both play together, {clip_slow:.1f} times slower than the recording (the simulation itself ran {sim_slow:.1f} times slower). "
                "Black line: fixed to the cylinder.",
                "Right: SIMULATION. The cylinder is on an axle: its centre and axis direction follow the recording; only its turn is free.",
                press,
                "Drawn skin: the finger's free shape for the commanded inputs, not pressed in. Coloured dots: a tip pressing now. "
                "Comparative, not predictive."]
    else:
        mass = run["mass_kg"] * 1e3
        titles = ["Recorded: learned policy, four-finger hand",
                  f"Simulated: four two-pneu-net fingers, free cylinder of {mass:.1f} g"]
        foot = [f"Both play together, {clip_slow:.1f} times slower than the recording (the simulation itself ran {sim_slow:.1f} times slower). "
                "Black line: fixed to the cylinder.",
                f"Right: SIMULATION. Nothing holds the cylinder but the tips; no palm. Its mass is {mass:.1f} g, the recorded one "
                f"{results['recorded_mass_kg'] * 1e3:.1f} g" + ("." if run["gravity"] else "; NO GRAVITY in this run."),
                press,
                "Drawn skin: the finger's free shape for the commanded inputs, not pressed in. Coloured dots: a tip pressing now. "
                "Comparative, not predictive."]

    def shot(ren, m, d, extra):
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

    n_frames = len(fr)
    stills = {0: "_start.png", int(np.argmin(np.abs(fr[:, 2] - 0.5 * (gait[0, 2] + gait[-1, 2])))): "_middle.png",
              n_frames - 1: "_end.png"}
    proc = None
    files = []
    if not args.stills_only:
        ff = shutil.which("ffmpeg") or r"C:\Program Files\ffmpeg\bin\ffmpeg.exe"
        proc = subprocess.Popen(
            [ff, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{Wt}x{Ht}",
             "-r", str(FPS), "-i", "-", "-pix_fmt", "yuv420p", "-vcodec", "libx264", base + ".mp4"], stdin=subprocess.PIPE)
        files.append(base + ".mp4")
    turn0 = fr[0, 3]
    pressed_frames = np.zeros(4, int)
    for n in (range(n_frames) if not args.stills_only else sorted(stills)):
        row = fr[n]
        lt = row[2]
        k = int(np.clip(np.searchsorted(t, lt), s0, s1))
        A.pose_recorded(S, dS, Z, k)
        now = {j: L["P"][k, j] for j in range(4) if L["lam"][k, j] > A.TOUCH_N}

        def ex0(scene):
            turn_line(scene, Z["obj"][k][:3].astype(float), Z["obj"][k][3:7].astype(float))
            for j, p in now.items():
                A.add_sphere(scene, p, 0.0035, (*COL[L["names"][j]], 1))
        im0 = shot(renS, S["m"], dS, ex0)
        pos, q = row[6:9], row[9:13]
        sk.d.mocap_pos[0] = pos
        sk.d.mocap_quat[0] = q[[3, 0, 1, 2]]
        mj.mj_forward(sk.m, sk.d)
        axis = A.rot(q, np.array([0.0, 0.0, 1.0]))
        dots = {}
        for j in range(4):
            b = row[C0 + NB * j:C0 + NB * (j + 1)]
            sk.set(j, shapes[j].world(b[0], b[1]))
            if b[3] > A.TOUCH_N:
                r = b[8:11] - pos
                r -= (r @ axis) * axis
                dots[j] = b[8:11] - radius * r / np.linalg.norm(r)
                pressed_frames[j] += 1

        def ex1(scene):
            turn_line(scene, pos, q)
            for j, p in dots.items():
                A.add_sphere(scene, p, 0.0045, (*COL[L["names"][j]], 1))
        im1 = shot(sk.ren, sk.m, sk.d, ex1)
        im = Image.new("RGB", (Wt, Ht), (255, 255, 255))
        im.paste(Image.fromarray(np.hstack([im0, im1])), (0, HEAD))
        dr = ImageDraw.Draw(im)
        rec_now = np.degrees(L["turn"][k] - L["turn"][s0])
        seconds = [f"log time {t[k]:.2f} s   cylinder turned {rec_now:+.1f} deg   touching: " + " ".join(L["names"][j] for j in now),
                   f"cylinder turned {row[3] - turn0:+.1f} deg (recorded {rec_now:+.1f})   pressing: "
                   + " ".join(L["names"][j] for j in dots)]
        for i in range(2):
            dr.text((i * CW + 20, 14), titles[i], fill=(0, 0, 0), font=font)
            dr.text((i * CW + 20, 60), seconds[i], fill=(0, 0, 0), font=small)
        dr.line([(CW, HEAD), (CW, HEAD + CH)], fill=(200, 200, 200), width=1)
        for i, line in enumerate(foot):
            dr.text((20, HEAD + CH + 8 + 31 * i), line, fill=(0, 0, 0), font=small)
        if proc is not None:
            buf = np.asarray(im).tobytes()
            for _ in range(FPS if n in (0, n_frames - 1) else 1):
                proc.stdin.write(buf)
        if n in stills:
            im.save(base + stills[n])
            files.append(base + stills[n])
    if proc is not None:
        proc.stdin.close()
        proc.wait()
    renS.close()
    sk.ren.close()
    seconds_long = (n_frames + 2 * (FPS - 1)) / FPS
    rep = [f"Clip of four two-pneu-net fingers, tips aimed a fixed depth inside the surface, turning a cylinder "
           f"({'on an axle' if args.support == 'axle' else 'free, off the axle'}), beside the recording: run '{args.key}'",
           "Right-hand side: SIMULATION frames from " + os.path.relpath(os.path.join(folder, f"frames_{args.key}.csv"), ROOT)
           + ". Comparative, not predictive.", "",
           f"Mounting drawn: '{name}' from {os.path.relpath(args.mounting_results, ROOT)}.",
           f"{n_frames} frames at {FPS} per second, {clip_slow:.2f} times slower than the recording"
           + ("" if args.stills_only else f"; with the first and last frame held one second each the clip lasts {seconds_long:.1f} s") + ".",
           f"Simulated turn over the gait {run['turn_gait_deg']:+.1f} deg; recorded {rec_total:+.1f} deg.",
           "Frames with each tip marked as pressing: " + ", ".join(f"{L['names'][j]} {pressed_frames[j]}" for j in range(4))
           + (" (stills only)" if args.stills_only else f" of {n_frames}") + ".",
           f"Drawing check: MuJoCo's mesh against the supplied skin, largest difference {max(sk.check) * 1e6:.3f} micrometres.",
           "", "Files:"] + ["  " + os.path.relpath(f, ROOT) for f in files]
    with open(base + ("_stills_report.txt" if args.stills_only else "_report.txt"), "x", encoding="utf-8") as fh:
        fh.write("\n".join(rep) + "\n")
    print("\n".join(rep))


if __name__ == "__main__":
    main()
