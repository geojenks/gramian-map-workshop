"""Replay a stored follow result in MuJoCo (issue 14): drive the hand along
the joint path the check solved and see whether the object follows the
leader. gramian_workshop/replay.py says what is simulated and measured.

    python tools/run_replay.py <result.json> [--drive servo|kinematic]
                               [--pads leader|allowance|default|given] [--pad-k K]
                               [--feedforward all|standing]
                               [--lift leader|rest] [--film] [--no-windows]
                               [--no-frozen] [--no-track] [--no-control]
                               [--out-dir outputs/replay]

The leader is read from data/leaders/<result["leader"]>.json. The record
goes to <out-dir>/replay_<result stem>__<drive>[__ffstanding]__pads<pads>[K]__<lift>.json,
and with --film a short video of the free run (hand, object, and the
leader's object as a blue ghost) to the same stem .mp4. Existing files are
never overwritten: a numbered suffix is added. --pads is what takes up the
check's allowed sink (replay.py, "Pads"): leader (the headline, the
leader's own recorded contact stiffness), allowance, default (MuJoCo's own
contact), or given with
--pad-k K in N/m; all but leader are sensitivities. --feedforward standing
(servo drive) feeds forward the contact load and weight only, not the
motion and damping torques (replay.py, "The drive"): the other sensitivity.

Set GMW_MENAGERIE / GMW_WAVE_MANIP as for tools/run_follow.py.
"""
import argparse
import json
import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop import replay as rp   # noqa: E402

W, H = 960, 720
FPS = 25


def fresh(path):
    """path, or path with a numbered suffix if it exists: nothing stored is
    overwritten."""
    if not os.path.exists(path):
        return path
    stem, ext = os.path.splitext(path)
    i = 2
    while os.path.exists(f"{stem}__{i}{ext}"):
        i += 1
    return f"{stem}__{i}{ext}"


class Film:
    """Frames of the free run through mujoco.Renderer into ffmpeg. The camera
    looks at the object's first centre from a fixed offset in the hand's
    frame; geom groups 0-2 are drawn (the hand's visual meshes sit in
    group 2 for Allegro, 1 for LEAP)."""
    def __init__(self, path, centre, fps=FPS):
        import mujoco as mj
        self.mj = mj
        self.path = path
        self.r = None
        self.cam = mj.MjvCamera()
        self.cam.type = mj.mjtCamera.mjCAMERA_FREE
        self.cam.lookat[:] = centre
        self.cam.distance = 0.32
        self.cam.azimuth = 120
        self.cam.elevation = -20
        self.opt = mj.MjvOption()
        self.opt.geomgroup[:] = 0
        self.opt.geomgroup[0] = self.opt.geomgroup[1] = self.opt.geomgroup[2] = 1
        self.p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
                                   "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(fps), "-i", "-",
                                   "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", path],
                                  stdin=subprocess.PIPE)
        self.n = 0

    def __call__(self, k, model, data):
        mj = self.mj
        if self.r is None:
            # the offscreen framebuffer defaults to 640 x 480
            model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), W)
            model.vis.global_.offheight = max(int(model.vis.global_.offheight), H)
            self.r = mj.Renderer(model, H, W)
        self.r.update_scene(data, self.cam, self.opt)
        img = self.r.render()
        self.p.stdin.write(np.ascontiguousarray(img).tobytes())
        self.n += 1

    def close(self):
        self.p.stdin.close()
        self.p.wait()
        if self.r is not None:
            self.r.close()
        return self.n


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("result")
    ap.add_argument("--drive", choices=["servo", "kinematic"], default="servo")
    ap.add_argument("--pads", choices=["leader", "allowance", "default", "given"], default="leader")
    ap.add_argument("--pad-k", type=float, default=None,
                    help="--pads given: the pad stiffness in N/m")
    ap.add_argument("--feedforward", choices=["all", "standing"], default="all",
                    help="servo drive: which of the check's torques are fed forward")
    ap.add_argument("--lift", choices=["leader", "rest"], default="leader")
    ap.add_argument("--film", action="store_true")
    ap.add_argument("--no-windows", action="store_true")
    ap.add_argument("--no-frozen", action="store_true")
    ap.add_argument("--no-track", action="store_true")
    ap.add_argument("--no-control", action="store_true")
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "outputs", "replay"))
    a = ap.parse_args()
    if (a.pad_k is not None) != (a.pads == "given"):
        ap.error("--pad-k goes with --pads given, and only there")
    with open(a.result, encoding="utf-8") as f:
        result = json.load(f)
    with open(os.path.join(ROOT, "data", "leaders", result["leader"] + ".json"), encoding="utf-8") as f:
        leader_json = json.load(f)
    os.makedirs(a.out_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(a.result))[0]
    if stem.startswith("follow_"):
        stem = stem[len("follow_"):]
    if a.feedforward != "all" and a.drive != "servo":
        ap.error("--feedforward goes with --drive servo")
    tag = (a.drive + ("__ff" + a.feedforward if a.feedforward != "all" else "")
           + "__pads" + a.pads + (f"{a.pad_k:g}" if a.pad_k is not None else ""))
    out = fresh(os.path.join(a.out_dir, f"replay_{stem}__{tag}__{a.lift}.json"))
    film = None
    if a.film:
        Rw = np.array(result["placement"]["rotation_matrix"], float)
        t = np.array(result["placement"]["translation_m"], float)
        c0 = Rw @ (np.zeros(3)) + t     # the object's first centre, placed (Leader.center(0))
        film = Film(os.path.splitext(out)[0] + ".mp4", c0)
    rec = rp.replay(result, leader_json, lift=a.lift, drive=a.drive, pads=a.pads, pad_k=a.pad_k,
                    feedforward=a.feedforward,
                    windows=not a.no_windows, frozen=not a.no_frozen, track=not a.no_track,
                    control=not a.no_control, on_frame=film)
    rec["result_file"] = os.path.relpath(a.result, ROOT).replace("\\", "/")
    if film is not None:
        n = film.close()
        rec["film"] = {"file": os.path.relpath(film.path, ROOT).replace("\\", "/"),
                       "frames": n, "fps": FPS, "size": [W, H]}
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rec, f, indent=1)
    print(rp.brief(rec))
    print("wrote", os.path.relpath(out, ROOT))
    if film is not None:
        print("wrote", rec["film"]["file"], f"({rec['film']['frames']} frames)")


if __name__ == "__main__":
    main()
