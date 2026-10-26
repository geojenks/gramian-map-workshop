"""A human-finger hand: four offset copies of the Valero-Cuevas 1998 index
finger and a thumb made from the same finger with more abduction and an
opposed base (George, 23 Oct: "the 4 fingers can be offset copies ... can we
get away with just increasing [the thumb's] abduction and offsetting its
starting position?").

    python tools/build_human_hand.py [--column adjusted] [--arch] [--proportions]
        ->  data/hands/human_vc1998[_arch][_prop]/hand.xml

--measured PATH builds a person's hand from measured joint-centre lengths
(a hand_dimensions JSON from three-camera keypoints, e.g. data/hands/measured/
george_right_2026-10-25.json): each segment its measured length (the last
ends at the nail end, so the tip capsule stops there); knuckle positions a
least-squares fit to the wrist-to-knuckle distances, the knuckle gaps and
the palm width; the thumb base at its measured distance from the wrist, in a
direction ASSUMED (THUMB_CMC_DEG radial of the middle finger's axis, since
the file gives distance only); each finger's radius the contact model's pad
offset for its last segment (+0.5 mm middle, +1 mm proximal). Muscles as
the index's. Writes data/hands/measured_<name>/hand.xml (with --arch, the
assumed arch below).

--thumb PATH [--thumb-clip CLIP] (with --measured) places the thumb's base
and points its metacarpal from measured keypoints (a thumb_base JSON, e.g.
data/hands/measured/george_right_thumb_base_2026-10-26.json; default clip
tennis_new_cup, the clip nearest the tennis-ball cup leader): the base offset
from the index knuckle and the metacarpal direction of the held (gripping)
frames. The thumb's curl direction stays ASSUMED (THUMB_CURL).

--thumb-keypoints PATH (with --measured) takes the thumb's base, metacarpal
and curl direction from the demonstrator's keypoints in the model frame
(tools/demo_handframe.py writes data/hands/measured/george_thumb_from_keypoints.json):
nothing about the thumb is then assumed but its joint ranges.

--proportions gives each digit measured human proportions instead of copies
of the index: Buryanov & Kotiuk 2010 (Int. J. Morphol. 28(3):755-758, Table
I, X-ray bone lengths, 66 adults). Bone lengths are not the joint-to-joint
lengths of Valero-Cuevas's index (50/31/16 mm, An et al. 1979), so each
segment is the index's scaled by that segment's ratio to the index's in
Table I (the index is unchanged); the thumb's metacarpal, proximal and
distal phalanx map onto the chain's three segments. Knuckle positions along
the hand follow the metacarpal lengths with the metacarpal bases level
(ASSUMED). Moment arms and muscle forces stay the index's in every digit.

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
# Buryanov & Kotiuk 2010, Table I (mm): distal, medial, proximal phalanx, metacarpal
BK2010 = {"thumb": (21.67, None, 31.57, 46.22), "index": (15.82, 22.38, 39.78, 68.12),
          "middle": (17.40, 26.33, 44.63, 64.60), "ring": (17.30, 25.65, 41.37, 58.00),
          "little": (15.96, 18.11, 32.74, 53.69)}


# joint ranges for a measured hand (deg), widened 26 Oct to what George's
# tennis-ball cup uses (tools/skeleton_fit.py with the limits removed: thumb
# MCP to -10, index PIP to -5, ring DIP to -7) plus a margin: hyperextension
# allowed. None = the index model's (tools/vc_finger_mujoco.RANGE_DEG).
RANGE_MEASURED_DEG = {"finger": {"mcp_add": (-30, 30), "mcp_flex": (-30, 90), "pip": (-10, 110), "dip": (-30, 90)},
                      "thumb": {"mcp_add": (-60, 60), "mcp_flex": (-30, 90), "pip": (-20, 90), "dip": (-30, 90)}}
PALM_RADIUS_M = 0.009         # ASSUMED: half the palm's thickness along the metacarpals
THUMB_CMC_DEG = 32.0          # ASSUMED: direction of the thumb's base from the wrist


def _from_keypoint_frame(v):
    """Keypoint hand frame (x wrist->middle knuckle, y toward the thumb,
    z back of the hand) -> this model's (x toward the thumb, y distal, z palmar)."""
    return np.array([v[1], v[0], -v[2]], float)


def measured_thumb(path, clip):
    """(base offset from the index knuckle m, metacarpal direction) from a
    thumb_base JSON (three-camera keypoints, held frames of one clip)."""
    import json
    c = json.load(open(path))["per_clip"][clip]
    return _from_keypoint_frame(c["cmc_from_index_mcp_mm"]) / 1e3, _from_keypoint_frame(c["metacarpal_dir"])


def measured_geometry(path, thumb=None):
    """From a hand_dimensions JSON: {finger: (knuckle xyz m, (Lp, Lm, Ld) m,
    radius m)} in the hand frame (wrist at the origin)."""
    import json
    from scipy.optimize import least_squares
    J = json.load(open(path))
    g = lambda k: J["segments"][k]["median_mm"] / 1e3
    four = ["index", "middle", "ring", "little"]

    def res(v):
        P = {"index": v[0:2], "middle": np.array([0.0, v[2]]), "ring": v[3:5], "little": v[5:7]}
        r = [np.linalg.norm(P[f]) - g(f"{f} wrist-MCP") for f in four]
        r += [np.linalg.norm(P["index"] - P["middle"]) - g("palm index MCP-middle MCP"),
              np.linalg.norm(P["middle"] - P["ring"]) - g("palm middle MCP-ring MCP"),
              np.linalg.norm(P["ring"] - P["little"]) - g("palm ring MCP-little MCP"),
              np.linalg.norm(P["index"] - P["little"]) - g("palm width index MCP-little MCP")]
        return r
    v = least_squares(res, [0.026, 0.108, 0.118, -0.026, 0.105, -0.048, 0.088]).x
    xy = {"index": v[0:2], "middle": np.array([0.0, v[2]]), "ring": v[3:5], "little": v[5:7]}
    a = math.radians(THUMB_CMC_DEG)
    xy["thumb"] = g("thumb wrist-CMC") * np.array([math.sin(a), math.cos(a)])
    thumb_base = None
    if thumb is not None:
        off = thumb[0]
        thumb_base = (np.array(off) if (len(thumb) > 2 and thumb[2].get("absolute"))
                      else np.array([xy["index"][0], xy["index"][1], 0.0]) + off)
    pads = {f: x / 1e3 for f, x in J["pad_offset_used_mm"].items()}
    out = {}
    for f in four + ["thumb"]:
        names = ("CMC-MCP", "MCP-IP", "IP-tip") if f == "thumb" else ("MCP-PIP", "PIP-DIP", "DIP-tip")
        L = tuple(g(f"{f} {n}") for n in names)
        z = THUMB_BASE[2] if f == "thumb" else 0.0
        p0 = thumb_base if (f == "thumb" and thumb_base is not None) else np.array([xy[f][0], xy[f][1], z])
        out[f] = (p0, L, pads[f])
    return out


def segment_lengths(name, proportions):
    """(proximal, middle, distal) chain segment lengths (m)."""
    if not proportions:
        return vc.L_PROX, vc.L_MID, vc.L_DIST
    ix = BK2010["index"]
    k = (vc.L_PROX / ix[2], vc.L_MID / ix[1], vc.L_DIST / ix[0])
    d, mm, pp, mc = BK2010[name]
    bones = (mc, pp, d) if name == "thumb" else (pp, mm, d)
    return tuple(b * f for b, f in zip(bones, k))


def mcp_y(name, proportions):
    """Knuckle position along the hand: the index's at MCP_Y, the others
    shifted by their metacarpal's length difference (bases level)."""
    if not proportions:
        return MCP_Y
    return MCP_Y + (BK2010[name][3] - BK2010["index"][3]) / 1e3


def _quat(R):
    """MuJoCo quaternion (w x y z) of a rotation matrix."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return f"{w:.6f} {x:.6f} {y:.6f} {z:.6f}"


def thumb_frame(distal=None, curl=None):
    y = THUMB_DISTAL if distal is None else np.asarray(distal, float)
    y = y / np.linalg.norm(y)
    cu = THUMB_CURL if curl is None else np.asarray(curl, float)
    z = cu - (cu @ y) * y
    z /= np.linalg.norm(z)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def finger_xml(name, pos, quat, abd_deg, lengths=None, radius=None, tip_at_end=False, ranges=None):
    Lp, Lm, Ld = lengths or (vc.L_PROX, vc.L_MID, vc.L_DIST)
    rad = (radius + 0.001, radius + 0.0005, radius) if radius else fm.RADIUS
    Ldg = Ld - rad[2] if tip_at_end else Ld       # capsule surface ends at the tip
    lo = {k: math.radians(v[0]) for k, v in fm.RANGE_DEG.items()}
    hi = {k: math.radians(v[1]) for k, v in fm.RANGE_DEG.items()}
    lo["mcp_add"], hi["mcp_add"] = -math.radians(abd_deg), math.radians(abd_deg)
    if ranges:
        for k_, (l_, h_) in ranges.items():
            lo[k_], hi[k_] = math.radians(l_), math.radians(h_)
    rng = lambda j: f'range="{lo[j]:.6f} {hi[j]:.6f}" limited="true"'
    seg = lambda L, r, m: f'<geom type="capsule" fromto="0 0 0 0 {L} 0" size="{r}" mass="{m}"/>'
    return f"""
  <body name="{name}_prox" pos="{pos[0]} {pos[1]} {pos[2]}" quat="{quat}">
    <joint name="{name}_mcp_add" axis="0 0 1" {rng('mcp_add')}/>
    <joint name="{name}_mcp_flex" axis="1 0 0" {rng('mcp_flex')}/>
    {seg(Lp, rad[0], fm.MASS[0])}
    <body name="{name}_mid" pos="0 {Lp:.5f} 0"><joint name="{name}_pip" axis="1 0 0" {rng('pip')}/>
      {seg(Lm, rad[1], fm.MASS[1])}
      <body name="{name}_distal" pos="0 {Lm:.5f} 0"><joint name="{name}_dip" axis="1 0 0" {rng('dip')}/>
        {seg(Ldg, rad[2], fm.MASS[2])}
      </body></body></body>"""


def _roll(deg):
    """Rotation about the distal (y) axis taking palmar (z) toward radial (x)."""
    a = math.radians(deg)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def build(column, arch=False, proportions=False, measured=None, thumb=None, wide=False):
    R_mm, F0, _ = vc.build(column)
    R_m = R_mm / 1e3
    joints = ["mcp_add", "mcp_flex", "pip", "dip"]
    names = list(FINGERS) + ["thumb"]
    bodies = ""
    if measured:
        geo = measured_geometry(measured, thumb)
        for n in FINGERS:
            p0, L, r = geo[n]
            pos = (p0[0], p0[1], ARCH_PALMAR_M[n] if arch else 0.0)
            quat = _quat(_roll(ARCH_ROLL_DEG[n])) if arch else "1 0 0 0"
            bodies += finger_xml(n, pos, quat, 30.0, L, r, tip_at_end=True,
                                 ranges=RANGE_MEASURED_DEG["finger"] if wide else None)
        p0, L, r = geo["thumb"]
        bodies += finger_xml("thumb", p0, _quat(thumb_frame(thumb[1] if thumb else None,
                                                            thumb[2].get("curl") if thumb and len(thumb) > 2 else None)),
                             THUMB_ABD_DEG, L, r, tip_at_end=True,
                             ranges=RANGE_MEASURED_DEG["thumb"] if wide else None)
        # the palm as the skeleton has it (26 Oct, George: "why does the model
        # hand have a big block for a palm instead of the skeleton structure"):
        # a capsule from the wrist to each knuckle and a bar across the
        # knuckles, PALM_RADIUS_M thick (ASSUMED)
        kn = {n: np.array([geo[n][0][0], geo[n][0][1], ARCH_PALMAR_M[n] if arch else 0.0]) for n in FINGERS}
        segs_ = [(np.zeros(3), kn[n]) for n in FINGERS] + [(kn["index"], kn["middle"]), (kn["middle"], kn["ring"]),
                                                          (kn["ring"], kn["little"])]
        segs_.append((np.zeros(3), np.array(geo["thumb"][0])))   # wrist to the thumb's base, as the skeleton has it
        palm = "".join(f'<geom type="capsule" fromto="{a[0]:.5f} {a[1]:.5f} {a[2]:.5f} {b[0]:.5f} {b[1]:.5f} {b[2]:.5f}" '
                       f'size="{PALM_RADIUS_M}" mass="0.04"/>' for a, b in segs_)
    else:
        for n, x in FINGERS.items():
            pos = (x, mcp_y(n, proportions), ARCH_PALMAR_M[n] if arch else 0.0)
            quat = _quat(_roll(ARCH_ROLL_DEG[n])) if arch else "1 0 0 0"
            bodies += finger_xml(n, pos, quat, 30.0, segment_lengths(n, proportions))
        bodies += finger_xml("thumb", THUMB_BASE, _quat(thumb_frame()), THUMB_ABD_DEG,
                             segment_lengths("thumb", proportions))
        palm = '<geom type="box" pos="0 0.040 -0.010" size="0.045 0.040 0.010" mass="0.3"/>'
    tendons, acts = [], []
    for n in names:
        for k, mu in enumerate(vc.MUSCLES):
            tendons.append(f'<fixed name="{n}_{mu}">' + "".join(
                f'<joint joint="{n}_{j}" coef="{R_m[i, k]:.6g}"/>' for i, j in enumerate(joints)
                if R_m[i, k] != 0) + "</fixed>")
            acts.append(f'<motor name="{n}_{mu}" tendon="{n}_{mu}" gear="1" ctrlrange="0 {F0[k]:.4g}" '
                        f'ctrllimited="true" forcerange="0 {F0[k]:.4g}" forcelimited="true"/>')
    tag = ("_arch" if arch else "") + ("_prop" if proportions else "")
    if measured:
        tag = ("_measured_" + os.path.splitext(os.path.basename(measured))[0].split("_20")[0] + ("_arch" if arch else "")
               + (("_thumbkp" if len(thumb) > 2 else "_thumb") if thumb else "") + ("_wide" if wide else ""))
    xml = f"""<mujoco model="human_vc1998_{column}{tag}">
<!-- generated by tools/build_human_hand.py ({column} column of Valero-Cuevas et al. 1998, Tables 1-2) -->
<compiler angle="radian"/>
<option gravity="0 0 0"/>
<default><joint damping="{fm.DAMPING}" armature="1e-5"/></default>
<worldbody>
 <body name="palm">
  {palm}
  {bodies}
 </body>
</worldbody>
<tendon>{''.join(tendons)}</tendon>
<actuator>{''.join(acts)}</actuator>
</mujoco>
"""
    out = OUT.replace("human_vc1998", "human_vc1998" + tag)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        f.write(xml)
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--column", default="adjusted", choices=["nominal", "adjusted"])
    ap.add_argument("--arch", action="store_true")
    ap.add_argument("--proportions", action="store_true")
    ap.add_argument("--measured", default=None, help="hand_dimensions JSON")
    ap.add_argument("--thumb", default=None, help="thumb_base JSON (with --measured)")
    ap.add_argument("--thumb-clip", default="tennis_new_cup")
    ap.add_argument("--wide-ranges", action="store_true",
                    help="RANGE_MEASURED_DEG (hyperextension allowed) instead of the index model's ranges")
    ap.add_argument("--thumb-keypoints", default=None,
                    help="thumb base, metacarpal and curl from tools/demo_handframe.py (model frame)")
    a = ap.parse_args()
    th = measured_thumb(a.thumb, a.thumb_clip) if a.thumb else None
    if a.thumb_keypoints:
        import json as _j
        T = _j.load(open(a.thumb_keypoints))
        th = (np.array(T["cmc_m"]), np.array(T["metacarpal_dir"]), {"absolute": True, "curl": T["curl_dir"]})
    print("wrote", build(a.column, a.arch, a.proportions, a.measured, th, a.wide_ranges))
    if th is not None:
        print(f"   thumb base from the index knuckle {np.round(th[0] * 1e3, 1)} mm, metacarpal {np.round(th[1], 3)}")
    if a.measured:
        for n, (p0, L, r) in measured_geometry(a.measured, th).items():
            print(f"   {n:7s} base {np.round(p0 * 1e3, 1)} mm, segments {[round(x * 1e3, 1) for x in L]} mm, "
                  f"pad radius {r * 1e3:.1f} mm")
        raise SystemExit
    for n in list(FINGERS) + ["thumb"]:
        print(f"   {n:7s} segments {[round(x * 1e3, 1) for x in segment_lengths(n, a.proportions)]} mm, "
              f"knuckle y {mcp_y(n, a.proportions) * 1e3:.1f} mm" if n != "thumb" else
              f"   {n:7s} segments {[round(x * 1e3, 1) for x in segment_lengths(n, a.proportions)]} mm")
