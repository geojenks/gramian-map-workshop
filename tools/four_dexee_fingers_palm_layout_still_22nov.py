"""Still of the four-DEX-EE-finger palm layout, seen whole (the film crops the palm).

Simulation only. Not a product hand: the layout is ours. Reads the model and one
moment of a saved run from an output folder of
tools/four_dexee_fingers_on_a_palm_turn_free_cylinder_22nov.py; draws two views
(from the side and from above). The palm plate is drawn only. Nothing is
overwritten.

Use: python tools/four_dexee_fingers_palm_layout_still_22nov.py <output folder> [seconds]
"""
import os
import sys

import mujoco
import numpy as np
from PIL import Image

out = sys.argv[1]
t_show = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
m = mujoco.MjModel.from_xml_path(os.path.join(out, "model_four_dexee_fingers.xml"))
d = mujoco.MjData(m)
Z = np.load(os.path.join(out, "runs.npz"))
key = "four dexee fingers, feedback, seed 0"
i = int(np.argmin(np.abs(Z[key + "|t"] - t_show)))
d.qpos[:] = Z[key + "|qpos"][i]
mujoco.mj_forward(m, d)
m.vis.global_.offwidth, m.vis.global_.offheight = 1000, 1000
r = mujoco.Renderer(m, 1000, 1000)
centre = (d.body("palm_plate").xpos + d.body("obj").xpos) / 2
ims = []
for az, el, c, dist in ((40, -15, centre, 0.75), (90, -89, d.body("palm_plate").xpos, 0.75)):
    cam = mujoco.MjvCamera()
    cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = c, dist, az, el
    r.update_scene(d, cam)
    ims.append(r.render().copy())
dst = os.path.join(out, "four_dexee_fingers_palm_layout_side_and_top_22nov.png")
with open(dst, "xb") as fo:
    Image.fromarray(np.hstack(ims)).save(fo, format="PNG")
print("wrote", os.path.abspath(dst), f"(moment shown: {Z[key + '|t'][i]:.2f} s, feedback, seed 0)")
