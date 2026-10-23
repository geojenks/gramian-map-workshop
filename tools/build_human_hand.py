"""A human-finger hand: four offset copies of the Valero-Cuevas 1998 index
finger and a thumb made from the same finger with more abduction and an
opposed base (George, 23 Oct: "the 4 fingers can be offset copies ... can we
get away with just increasing [the thumb's] abduction and offsetting its
starting position?").

    python tools/build_human_hand.py [--column adjusted] [--arch]
        ->  data/hands/human_vc1998/hand.xml (or human_vc1998_arch/ with --arch)

--arch adds a transverse palmar arch (ASSUMED values, not measured): the
ring and little MCPs sit ARCH_PALMAR_M palmar of the index and middle, and
their fingers' curl planes are turned toward the thumb by ARCH_ROLL_DEG
(about each finger's distal axis).

Each finger: MCP ad/abduction and flexion, PIP and DIP flexion, phalanges
50 / 31 / 16 mm, seven muscles as fixed tendons with the paper's moment arms
(tools/valero_cuevas_finger.build) and motors that only pull, 0 to F0
(forcerange, so the follow check reads them as one-sided inputs). Constant
moment arms, as in the paper.

Not from the paper (assumptions, stated): MCP spacing 20 mm across the palm
(index radial-most), every finger the index's size; thumb base 35 mm radial
and 20 mm distal of the palm's proximal edge, its distal axis pointing
radial, distal and palmar and its curl facing the fingers, its ad/abduction
range +/-60 deg (fingers +/-30); masses, radii and joint ranges as in
tools/vc_finger_mujoco.py; palm a 90 x 80 x 20 mm box.
"""
import argparse
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import valero_cuevas_finger as vc  # noqa: E402
import vc_finger_mujoco as fm  # noqa: E402

OUT = os.path.join(ROOT, "data", "hands", "human_vc1998", "hand.xml")
# hand frame: x radial (toward the thumb), y distal, z palmar
FINGERS = {"index": 0.030, "middle": 0.010, "ring": -0.010, "little": -0.030}
MCP_Y = 0.080
THUMB_BASE = np.array([0.040, 0.020, 0.008])
THUMB_DISTAL = np.array([0.6, 0.6, 0.5])          # its y axis
THUMB_CURL = np.array([-0.8, 0.2, 0.6])           # its z (palmar) axis, before orthogonalising
THUMB_ABD_DEG = 60.0
ARCH_PALMAR_M = {"index": 0.0, "middle": 0.0, "ring": 0.004, "little": 0.008}
ARCH_ROLL_DEG = {"index": 0.0, "middle": 0.0, "ring": 8.0, "little": 16.0}


def _quat(R):
    """MuJoCo quaternion (w x y z) of a rotation matrix."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return f"{w:.6f} {x:.6f} {y:.6f} {z:.6f}"


def thumb_frame():
    y = THUMB_DISTAL / np.linalg.norm(THUMB_DISTAL)
    z = THUMB_CURL - (THUMB_CURL @ y) * y
    z /= np.linalg.norm(z)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def finger_xml(name, pos, quat, abd_deg):
    lo = {k: math.radians(v[0]) for k, v in fm.RANGE_DEG.items()}
    hi = {k: math.radians(v[1]) for k, v in fm.RANGE_DEG.items()}
    lo["mcp_add"], hi["mcp_add"] = -math.radians(abd_deg), math.radians(abd_deg)
    rng = lambda j: f'range="{lo[j]:.6f} {hi[j]:.6f}" limited="true"'
    seg = lambda L, r, m: f'<geom type="capsule" fromto="0 0 0 0 {L} 0" size="{r}" mass="{m}"/>'
    return f"""
  <body name="{name}_prox" pos="{pos[0]} {pos[1]} {pos[2]}" quat="{quat}">
    <joint name="{name}_mcp_add" axis="0 0 1" {rng('mcp_add')}/>
    <joint name="{name}_mcp_flex" axis="1 0 0" {rng('mcp_flex')}/>
    {seg(vc.L_PROX, fm.RADIUS[0], fm.MASS[0])}
    <body name="{name}_mid" pos="0 {vc.L_PROX} 0"><joint name="{name}_pip" axis="1 0 0" {rng('pip')}/>
      {seg(vc.L_MID, fm.RADIUS[1], fm.MASS[1])}
      <body name="{name}_distal" pos="0 {vc.L_MID} 0"><joint name="{name}_dip" axis="1 0 0" {rng('dip')}/>
        {seg(vc.L_DIST, fm.RADIUS[2], fm.MASS[2])}
      </body></body></body>"""


def _roll(deg):
    """Rotation about the distal (y) axis taking palmar (z) toward radial (x)."""
    a = math.radians(deg)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def build(column, arch=False):
    R_mm, F0, _ = vc.build(column)
    R_m = R_mm / 1e3
    joints = ["mcp_add", "mcp_flex", "pip", "dip"]
    names = list(FINGERS) + ["thumb"]
    if arch:
        bodies = "".join(finger_xml(n, (x, MCP_Y, ARCH_PALMAR_M[n]), _quat(_roll(ARCH_ROLL_DEG[n])), 30.0)
                         for n, x in FINGERS.items())
    else:
        bodies = "".join(finger_xml(n, (x, MCP_Y, 0.0), "1 0 0 0", 30.0) for n, x in FINGERS.items())
    bodies += finger_xml("thumb", THUMB_BASE, _quat(thumb_frame()), THUMB_ABD_DEG)
    tendons, acts = [], []
    for n in names:
        for k, mu in enumerate(vc.MUSCLES):
            tendons.append(f'<fixed name="{n}_{mu}">' + "".join(
                f'<joint joint="{n}_{j}" coef="{R_m[i, k]:.6g}"/>' for i, j in enumerate(joints)
                if R_m[i, k] != 0) + "</fixed>")
            acts.append(f'<motor name="{n}_{mu}" tendon="{n}_{mu}" gear="1" ctrlrange="0 {F0[k]:.4g}" '
                        f'ctrllimited="true" forcerange="0 {F0[k]:.4g}" forcelimited="true"/>')
    xml = f"""<mujoco model="human_vc1998_{column}{'_arch' if arch else ''}">
<!-- generated by tools/build_human_hand.py ({column} column of Valero-Cuevas et al. 1998, Tables 1-2) -->
<compiler angle="radian"/>
<option gravity="0 0 0"/>
<default><joint damping="{fm.DAMPING}" armature="1e-5"/></default>
<worldbody>
 <body name="palm">
  <geom type="box" pos="0 0.040 -0.010" size="0.045 0.040 0.010" mass="0.3"/>
  {bodies}
 </body>
</worldbody>
<tendon>{''.join(tendons)}</tendon>
<actuator>{''.join(acts)}</actuator>
</mujoco>
"""
    out = OUT.replace("human_vc1998", "human_vc1998_arch") if arch else OUT
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        f.write(xml)
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--column", default="adjusted", choices=["nominal", "adjusted"])
    ap.add_argument("--arch", action="store_true")
    a = ap.parse_args()
    print("wrote", build(a.column, a.arch))
