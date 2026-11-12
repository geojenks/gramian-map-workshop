"""Pictures for the mock-up of the landing page (tools/make_landing_mockup.py).

    python tools/render_landing_pictures.py --out outputs/NEW_FOLDER
    python tools/render_landing_pictures.py --out outputs/NEW_FOLDER --atlas PAGE.html --browser MSEDGE_OR_CHROME

Writes one picture per hand of the hands table, hand_<key>.png: the hand's
model in its rest pose, drawn off screen by MuJoCo, with a clear background
so that it sits on a light or a dark page. Hands whose key or name holds a
word the mock-up leaves off are skipped, and so is a hand whose model file
is not on this machine.

With --atlas it also writes actuator_map.png: a picture of that page taken
by the browser given (a Chromium browser, run headless). The page meant is
the M-11 project's actuator atlas (presentation/actuator_atlas.html, made by
make_atlas.py there). --atlas-note is written beside it in pictures.json,
to say which copy was pictured.

pictures.json lists what was written. No picture is ever written over: the
folder must be new. Needs MuJoCo and numpy (the hands extra).
"""
import argparse
import io
import json
import os
import struct
import subprocess
import sys
import zlib

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
from gramian_workshop.hands import HANDS, model_path  # noqa: E402
from make_landing_mockup import left_out  # noqa: E402

W, H = 960, 720          # drawn at this size, then cut down to the hand
PAD = 12                 # clear pixels kept round the hand


def png(rgba):
    """An H x W x 4 uint8 array as PNG bytes."""
    h, w = rgba.shape[:2]
    raw = b"".join(b"\x00" + rgba[y].tobytes() for y in range(h))

    def chunk(kind, body):
        return (struct.pack(">I", len(body)) + kind + body
                + struct.pack(">I", zlib.crc32(kind + body) & 0xffffffff))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def hand_picture(path):
    """The model at `path` in its rest pose, seen from the front and above:
    RGBA, clear where there is no body, cut down to the hand and halved."""
    import mujoco
    m = mujoco.MjModel.from_xml_path(path)
    m.vis.global_.offwidth = max(m.vis.global_.offwidth, W)
    m.vis.global_.offheight = max(m.vis.global_.offheight, H)
    m.vis.headlight.ambient[:] = 0.6
    m.vis.headlight.diffuse[:] = 0.8
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = m.stat.center
    cam.distance = 1.6 * m.stat.extent
    cam.azimuth, cam.elevation = 140.0, -25.0
    r = mujoco.Renderer(m, H, W)
    r.update_scene(d, cam)
    rgb = r.render().copy()
    r.enable_segmentation_rendering()
    r.update_scene(d, cam)
    seg = r.render()
    r.close()
    body = seg[:, :, 0] >= 0
    if not body.any():
        raise SystemExit("nothing of %s is in view" % path)
    rgba = np.dstack([rgb, np.where(body, 255, 0).astype(np.uint8)])
    ys, xs = np.where(body.any(axis=1))[0], np.where(body.any(axis=0))[0]
    y0, y1 = max(ys[0] - PAD, 0), min(ys[-1] + PAD + 1, H)
    x0, x1 = max(xs[0] - PAD, 0), min(xs[-1] + PAD + 1, W)
    cut = rgba[y0:y1 - (y1 - y0) % 2, x0:x1 - (x1 - x0) % 2].astype(np.uint16)
    half = (cut[0::2, 0::2] + cut[0::2, 1::2] + cut[1::2, 0::2] + cut[1::2, 1::2]) // 4
    return half.astype(np.uint8)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True, help="a new folder for the pictures")
    ap.add_argument("--atlas", help="the actuator atlas page to picture")
    ap.add_argument("--atlas-note", default="", help="which copy of the atlas that is")
    ap.add_argument("--browser", help="a Chromium browser to picture the atlas with")
    ap.add_argument("--atlas-window", default="1200,1150",
                    help="the browser window for the atlas picture, as WIDTH,HEIGHT")
    a = ap.parse_args(argv)
    os.makedirs(a.out)                       # fails if it is there: nothing is written over
    listed = {"hands": {}, "skipped": {}}
    for key, spec in HANDS.items():
        if left_out(key, spec["name"]):
            continue
        path = model_path(key)
        if not os.path.isfile(path):
            listed["skipped"][key] = "no model file on this machine"
            continue
        name = "hand_%s.png" % key
        with io.open(os.path.join(a.out, name), "xb") as f:
            f.write(png(hand_picture(path)))
        listed["hands"][key] = name
        print("wrote", name)
    if a.atlas:
        if not a.browser:
            raise SystemExit("--atlas needs --browser")
        shot = os.path.abspath(os.path.join(a.out, "actuator_map.png"))
        subprocess.run([a.browser, "--headless=new", "--disable-gpu", "--hide-scrollbars",
                        "--window-size=" + a.atlas_window, "--virtual-time-budget=4000",
                        "--screenshot=" + shot,
                        "file:///" + os.path.abspath(a.atlas).replace(os.sep, "/")],
                       check=True, timeout=180)
        if not os.path.isfile(shot):
            raise SystemExit("the browser wrote no picture")
        listed["atlas"] = {"file": "actuator_map.png", "note": a.atlas_note}
        print("wrote actuator_map.png")
    with io.open(os.path.join(a.out, "pictures.json"), "x", encoding="utf-8", newline="\n") as f:
        json.dump(listed, f, indent=1, sort_keys=True)
        f.write("\n")


if __name__ == "__main__":
    main()
