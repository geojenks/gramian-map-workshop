"""Joint tables and draft records for two Shadow fingers, from model files.

    python build_shadow_finger_drafts.py tables     (needs mujoco)
    python build_shadow_finger_drafts.py records    (numpy only)

Two fingers, both read from MuJoCo Menagerie at the commit
tools/fetch_hands.py pins:

  shadow_e3m5_first_finger  the first finger of the Shadow Hand E3M5
                            (external/mujoco_menagerie/shadow_hand/
                            right_hand.xml), with its abduction joint;
  shadow_dexee_finger       finger F0 of the Shadow DEX-EE hand
                            (source/shadow_dexee.xml, copied here from the
                            same commit; the three fingers are the same).

`tables` writes one joint table per finger under data/joint_tables/, in the
format of docs/JOINT_TABLE_FORMAT.md. Every number in a table is read from
the model: joint axes and anchor points at the zero pose (turned into the
finger's own frame), ranges, each actuator's force range, joint spring
stiffness. It then reads the table back and checks its tip position against
MuJoCo's at random poses.

`records` writes one draft record per independent input of each finger: that
input goes from 0 to the upper end of its range and every other joint is held
at 0. The rules are the ones of tools/joint_table_to_record.py, whose
functions are used here unchanged. (The whole-finger record, every joint
driven together, comes from running that tool itself on the table.)

Nothing is overwritten: an existing output file is refused.
"""
import copy
import csv
import hashlib
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
import joint_table_to_record as jt  # noqa: E402
from gramian_workshop.vendor import actuator_record as ar  # noqa: E402

MENAGERIE = "https://github.com/google-deepmind/mujoco_menagerie"
TABLE_DIR = os.path.join(ROOT, "data", "joint_tables")
DATE = "2026-11-21"
EXTRA_COLUMNS = ("reverse_torque_Nm", "driven_by")
CHECK_POSES = 200
CHECK_TOL_M = 1e-6

FINGERS = {
    "shadow_e3m5_first_finger": {
        "label": "Shadow Hand E3M5 first finger",
        "xml": os.path.join(ROOT, "external", "mujoco_menagerie",
                            "shadow_hand", "right_hand.xml"),
        "xml_in_menagerie": "shadow_hand/right_hand.xml",
        "strip": False,
        "joints": ["rh_FFJ4", "rh_FFJ3", "rh_FFJ2", "rh_FFJ1"],
        "bend_joint": "rh_FFJ3",
        # The model has no fingertip site: the tip point is the far end of
        # the last link's mesh, on the line through the joints.
        "tip": ("mesh_end", "rh_ffdistal"),
        "actuation": "cable",
        "actuation_why": ("actuation 'cable' as the stored "
                          "shadow_hand_finger record has it"),
    },
    "shadow_dexee_finger": {
        "label": "Shadow DEX-EE finger",
        "xml": os.path.join(HERE, "source", "shadow_dexee.xml"),
        "xml_in_menagerie": "shadow_dexee/shadow_dexee.xml",
        # Its meshes and the mujoco.pid actuator plugin are not needed for
        # joint geometry, so they are taken out before MuJoCo builds it.
        "strip": True,
        "joints": ["F0/J0", "F0/J1", "F0/J2", "F0/J3"],
        "bend_joint": "F0/J1",
        "tip": ("site", "F0/fingertip_site"),
        "actuation": "electric",
        "actuation_why": ("actuation 'electric': the model gives each joint "
                          "a position actuator and does not say how the "
                          "real hand transmits it"),
    },
}


def sha256_file(path):
    return jt.sha256_file(path)


def refuse_existing(path):
    if os.path.exists(path):
        raise SystemExit(f"refusing to overwrite {path}; give a new name")


def menagerie_commit():
    """The commit tools/fetch_hands.py pins."""
    import fetch_hands
    return fetch_hands.MENAGERIE_COMMIT


# ----------------------------------------------------------------------
# tables (MuJoCo)
# ----------------------------------------------------------------------
def load_model(spec):
    import mujoco
    if not spec["strip"]:
        return mujoco.MjModel.from_xml_path(spec["xml"])
    root = ET.parse(spec["xml"]).getroot()
    for tag in ("asset", "extension", "actuator", "sensor", "contact",
                "keyframe"):
        for el in root.findall(tag):
            root.remove(el)
    for body in root.iter("body"):
        for g in body.findall("geom"):
            body.remove(g)
        # A moving body needs some mass to build; it changes no geometry.
        if body.find("inertial") is None:
            ET.SubElement(body, "inertial", pos="0 0 0", mass="0.01",
                          diaginertia="1e-6 1e-6 1e-6")
    wb = root.find("worldbody")
    for g in wb.findall("geom"):
        wb.remove(g)
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def torque_limits_from_model(m, joint_names):
    """{joint: (actuator, torque towards max, torque towards min)} from the
    built model's actuators, through a fixed tendon where there is one."""
    import mujoco
    out = {}
    for a in range(m.nu):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
        lo, hi = m.actuator_forcerange[a]
        gear = m.actuator_gear[a][0]
        trn, obj = m.actuator_trntype[a], m.actuator_trnid[a][0]
        if trn == mujoco.mjtTrn.mjTRN_JOINT:
            parts = [(obj, 1.0)]
        elif trn == mujoco.mjtTrn.mjTRN_TENDON:
            adr, num = m.tendon_adr[obj], m.tendon_num[obj]
            parts = [(m.wrap_objid[i], m.wrap_prm[i])
                     for i in range(adr, adr + num)
                     if m.wrap_type[i] == mujoco.mjtWrap.mjWRAP_JOINT]
        else:
            continue
        for jid, coef in parts:
            jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if jn in joint_names:
                ends = [lo * gear * coef, hi * gear * coef]
                out[jn] = (name, max(ends), -min(ends))
    return out


def torque_limits_from_xml(path, joint_names):
    """The same, read from the actuator elements of the model file (used
    where the actuators are taken out before building)."""
    out = {}
    for el in ET.parse(path).getroot().find("actuator"):
        jn = el.get("joint")
        if jn in joint_names:
            lo, hi = (float(v) for v in el.get("forcerange").split())
            out[jn] = (el.get("name"), hi, -lo)
    return out


def tip_local(m, d, spec, x_w, last_anchor):
    """(body id, tip point in that body's frame, what it is)."""
    import mujoco
    kind, name = spec["tip"]
    if kind == "site":
        sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, name)
        return (m.site_bodyid[sid], m.site_pos[sid].copy(),
                f"the model's site {name}")
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
    far, mesh = -math.inf, None
    for g in range(m.ngeom):
        mid = m.geom_dataid[g]
        if m.geom_bodyid[g] != bid or mid < 0:
            continue
        v = m.mesh_vert[m.mesh_vertadr[mid]:
                        m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        w = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        reach = float(((w - last_anchor) @ x_w).max())
        if reach > far:
            far = reach
            mesh = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, mid)
    tip_w = last_anchor + far * x_w
    R = d.xmat[bid].reshape(3, 3)
    return (bid, R.T @ (tip_w - d.xpos[bid]),
            f"the far end of mesh {mesh} of body {name} along the line "
            f"through the joints ({1e3 * far:.4f} mm past the last joint); "
            f"the model has no fingertip site")


def make_table(fid, spec, commit):
    import mujoco
    m = load_model(spec)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    jids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
            for n in spec["joints"]]
    if min(jids) < 0:
        raise SystemExit(f"{fid}: a joint is not in the model")
    anchors = np.array([d.xanchor[j] for j in jids])
    axes = np.array([d.xaxis[j] for j in jids])
    x_w = anchors[-1] - anchors[0]
    x_w /= np.linalg.norm(x_w)
    bid, tip_b, tip_what = tip_local(m, d, spec, x_w, anchors[-1])

    def tip_world():
        return d.xpos[bid] + d.xmat[bid].reshape(3, 3) @ tip_b

    tip_w = tip_world()
    k = spec["joints"].index(spec["bend_joint"])
    v = np.cross(axes[k], tip_w - anchors[k])
    y_w = v - (v @ x_w) * x_w
    y_w /= np.linalg.norm(y_w)
    R = np.column_stack([x_w, y_w, np.cross(x_w, y_w)])   # finger -> world
    origin = anchors[0]

    if spec["strip"]:
        limits = torque_limits_from_xml(spec["xml"], spec["joints"])
    else:
        limits = torque_limits_from_model(m, spec["joints"])
    rows = []
    for n, j, p, a in zip(spec["joints"], jids, anchors, axes):
        act, t_max, t_min = limits[n]
        lo, hi = m.jnt_range[j]
        pf, af = R.T @ (p - origin), R.T @ a
        rows.append([n.replace("/", "_")]
                    + [f"{c:.6f}" for c in af]
                    + [f"{1e3 * c:.4f}" for c in pf]
                    + [repr(float(lo)), repr(float(hi)),
                       repr(float(t_max)), repr(float(m.jnt_stiffness[j])),
                       repr(float(t_min)), act.replace("/", "_")])
    tip_f = R.T @ (tip_w - origin)
    rows.append(["tip", "", "", ""] + [f"{1e3 * c:.4f}" for c in tip_f]
                + ["", "", "", "", "", ""])

    xml_sha = sha256_file(spec["xml"])
    src = (f"MuJoCo Menagerie {spec['xml_in_menagerie']} at commit {commit} "
           f"(sha256 {xml_sha}), Apache-2.0")
    comments = [
        "Joint table for one jointed finger: docs/JOINT_TABLE_FORMAT.md. "
        "Written by data/draft_records/shadow_menagerie_fingers/"
        "build_shadow_finger_drafts.py; do not edit by hand.",
        f"Every number is read from {src}.",
        "Axes and positions: each joint's axis and anchor at the model's "
        "zero pose, in the finger's own frame: origin at the first joint, "
        "X along the line from the first joint to the last, Y the side the "
        f"tip moves to when {spec['bend_joint']} turns positive, Z "
        "right-handed.",
        f"Tip: {tip_what}.",
        "Ranges: the model's joint ranges. torque_Nm: the upper end of the "
        "force range of the joint's actuator (named in driven_by), which "
        "turns the joint towards max; reverse_torque_Nm: the size of the "
        "lower end. Joints with the same driven_by share one actuator "
        "through a tendon with coefficient 1 each.",
        "stiffness: the joint's spring stiffness in the model (0: the model "
        "gives the joints no spring; the stiffness a position controller "
        "adds is not in the table).",
        "tools/joint_table_to_record.py ignores the last two columns.",
    ]
    details = [
        ("id", fid + "_all_joints_together"),
        ("name", f"{spec['label']}, every joint driven together"),
        ("source", "literature"),
        ("detail", f"joint table read from {src}. One input drives every "
                   f"joint to the upper end of its range in proportion, "
                   f"which is one arbitrary line through a finger that has "
                   f"several independent inputs. DRAFT: not in the "
                   f"manifest."),
        ("reference", f"{MENAGERIE} at {commit}, "
                      f"{spec['xml_in_menagerie']}"),
        ("date", DATE),
        ("actuation", spec["actuation"]),
        ("levels", "21"),
        ("notes", f"Rigid jointed finger; {spec['actuation_why']}."),
    ]
    out = os.path.join(TABLE_DIR, fid + ".csv")
    refuse_existing(out)
    with open(out, "w", encoding="utf-8", newline="") as f:
        for c in comments:
            f.write("# " + c + "\n")
        w = csv.writer(f, lineterminator="\n")
        w.writerows(details)
        f.write("\n")
        w.writerow(["joint", "axis_x", "axis_y", "axis_z", "pos_x_mm",
                    "pos_y_mm", "pos_z_mm", "min_rad", "max_rad",
                    "torque_Nm", "stiffness_Nm_per_rad", *EXTRA_COLUMNS])
        w.writerows(rows)

    # Read the written table back and compare its tip with MuJoCo's.
    _, joints, tip, _ = jt.read_joint_table(out)
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(CHECK_POSES):
        q = np.array([rng.uniform(j["lo"], j["hi"]) for j in joints])
        d.qpos[:] = m.qpos0
        for j, qi in zip(jids, q):
            d.qpos[m.jnt_qposadr[j]] = qi
        mujoco.mj_kinematics(m, d)
        ours = jt.forward(joints, tip, q)[0]
        theirs = R.T @ (tip_world() - origin)
        worst = max(worst, float(np.linalg.norm(ours - theirs)))
    ok = worst <= CHECK_TOL_M
    line = (f"{fid}: table tip against MuJoCo {mujoco.__version__} at "
            f"{CHECK_POSES} random poses within the joint ranges: largest "
            f"difference {worst:.3e} m ({'pass' if ok else 'FAIL'}, limit "
            f"{CHECK_TOL_M:g} m); table sha256 {sha256_file(out)}")
    print(f"wrote {out}")
    print(line)
    return line, ok


def cmd_tables():
    commit = menagerie_commit()
    check = os.path.join(HERE, "tables_against_mujoco.txt")
    refuse_existing(check)
    for fid in FINGERS:
        refuse_existing(os.path.join(TABLE_DIR, fid + ".csv"))
    lines, good = [], True
    for fid, spec in FINGERS.items():
        line, ok = make_table(fid, spec, commit)
        lines.append(line)
        good = good and ok
    with open(check, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {check}")
    return 0 if good else 1


# ----------------------------------------------------------------------
# one record per independent input (numpy only)
# ----------------------------------------------------------------------
def read_extra(path):
    """{joint: {column: text}} for the two columns the tool ignores."""
    with open(path, encoding="utf-8-sig", newline="") as f:
        rows = [r for r in csv.reader(f) if r and not r[0].startswith("#")]
    start = next(i for i, r in enumerate(rows) if r[0] == "joint")
    header = rows[start]
    idx = {c: header.index(c) for c in EXTRA_COLUMNS}
    return {r[0]: {c: r[i] for c, i in idx.items()}
            for r in rows[start + 1:] if r[0] != "tip"}


def motion_words(e):
    """Plain words for a first motion direction in the finger frame."""
    part = int(np.argmax(np.abs(e)))
    return ("along the finger", "bending, in the bending plane",
            "side swing, across the bending plane")[part]


def input_record(fid, spec, table, details, joints, tip, extra, actuator):
    driven = [j["name"] for j in joints
              if extra[j["name"]]["driven_by"] == actuator]
    held = [j["name"] for j in joints if j["name"] not in driven]
    # Same table, with the other joints held at 0 and rigid.
    js = copy.deepcopy(joints)
    for j in js:
        if j["name"] in held:
            j["hi"] = 0.0
    levels = int(details.get("levels") or jt.DEFAULT_LEVELS)
    free = jt._free_block(js, tip, levels)
    us = [u[0] for u in free["u"]]
    e = jt.first_motion(js, tip)
    # Each joint's torque limit on the side that pushes along e.
    levers = jt.jacobian(js, tip, np.zeros(len(js))).T @ e
    for j, c in zip(js, levers):
        if c < 0.0:
            j["torque"] = float(extra[j["name"]]["reverse_torque_Nm"])
    blocked, binding = jt._blocked_block(js, tip, us)
    blocked["method"] = (
        "from the joint table: tip pinned at rest; the largest push along "
        "this input's first motion direction that every joint of the finger "
        "holds within the torque limit of its own actuator, scaled by the "
        "input; joints that share an actuator are each read on their own "
        "(the rule of tools/joint_table_to_record.py)")
    for j in js:
        if j["name"] in held:
            j["stiffness"] = math.inf
    stiffness = jt._stiffness_block(js, tip, us)
    if stiffness is not None:
        stiffness["method"] = (
            "from the joint table: 1 / C_yy from the spring of the driven "
            "joint alone, the held joints rigid; the model gives the joint "
            "no spring, so this is 0 (the tip has no passive stiffness; a "
            "position controller's stiffness is not in the record)")
    hi = {j["name"]: j["hi"] for j in joints}
    drive = ", ".join(f"{n} 0 to {hi[n]:.6g} rad" for n in driven)
    words = motion_words(e)
    table_rel = os.path.relpath(table, ROOT).replace("\\", "/")
    shared = ("" if len(driven) == 1 else
              " The model fixes only the sum of these joints' angles; the "
              "equal split along the way is an assumption (the end, every "
              "one at its upper limit, is not).")
    rec = {
        "schema_version": ar.SCHEMA_VERSION,
        "id": f"{fid}_{actuator}_alone",
        "name": f"{spec['label']}, {actuator} alone ({words})",
        "source": {
            "kind": details["source"],
            "detail": (f"one input of the joint table {table_rel}: actuator "
                       f"{actuator} drives {drive}; "
                       f"{', '.join(held)} held at 0. DRAFT: not in the "
                       f"manifest."),
            "date": details.get("date") or None,
            "reference": details.get("reference") or None},
        "metadata": {
            "actuation_physics": details.get("actuation") or "electric",
            "length_m": float(np.linalg.norm(tip - joints[0]["pos"])),
            "mass_kg": None, "material_volume_m3": None, "materials": {},
            "notes": (f"{details.get('notes', '')} One of the finger's "
                      f"{len({x['driven_by'] for x in extra.values()})} "
                      f"independent inputs, taken alone." + shared +
                      f" Blocked force limited by joint "
                      f"{', '.join(binding)}. Made with data/draft_records/"
                      f"shadow_menagerie_fingers/"
                      f"build_shadow_finger_drafts.py from {table_rel} "
                      f"(sha256 {sha256_file(table)}).")},
        "input": {
            "channels": [{"name": f"{actuator} (fraction of the upper range "
                                  f"end of {' and '.join(driven)}, and of "
                                  f"the torque limits)",
                          "unit": None, "range": [0.0, 1.0], "rated": 1.0}],
            "energy_convention":
                "mechanical work delivered at the input port (J)",
            "work_J": None},
        "port": {"name": "tip", "description": "the table's tip point"},
        "free": free,
        "blocked": blocked,
        "stiffness": stiffness,
        "scalars": jt._scalars(free, blocked, stiffness, 1),
        "shape": None,
        "uncertainty": {"default_fraction":
                        float(details.get("uncertainty") or 0.1)},
    }
    return rec


def cmd_records():
    todo = []
    for fid, spec in FINGERS.items():
        table = os.path.join(TABLE_DIR, fid + ".csv")
        details, joints, tip, _ = jt.read_joint_table(table)
        extra = read_extra(table)
        actuators = list(dict.fromkeys(extra[j["name"]]["driven_by"]
                                       for j in joints))
        for act in actuators:
            rec = input_record(fid, spec, table, details, joints, tip,
                               extra, act)
            out = os.path.join(HERE, rec["id"] + ".json")
            refuse_existing(out)
            errors, warnings = ar.validate(rec)
            for w in warnings:
                print(f"warn   {rec['id']}: {w}")
            if errors:
                for e in errors:
                    print(f"ERROR  {rec['id']}: {e}", file=sys.stderr)
                return 1
            todo.append((out, rec))
    for out, rec in todo:
        with open(out, "w", encoding="utf-8", newline="\n") as f:
            json.dump(rec, f, indent=1)
            f.write("\n")
        s = rec["scalars"]
        print(f"wrote {os.path.basename(out)} (tier {ar.tier(rec)}): stroke "
              f"{1e3 * s['free_stroke_m']:.2f} mm, direction "
              f"{[round(c, 3) for c in s['free_direction']]}, blocked "
              f"{s['blocked_force_N']:.3f} N, stiffness "
              f"{s['stiffness_N_per_m']}")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in ("tables", "records"):
        sys.exit(__doc__)
    sys.exit(cmd_tables() if sys.argv[1] == "tables" else cmd_records())
