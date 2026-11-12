"""Add a hand from a model file we did not prepare (a URDF or a MuJoCo XML),
so the other tools find it by name. The steps are in docs/BRING_YOUR_HAND.md.

    python tools/add_hand.py MODEL                 # list the model's bodies and print a starter description
    python tools/add_hand.py MODEL DESCRIPTION     # check the pair and add the hand
    python tools/add_hand.py MODEL DESCRIPTION --check-only
    python tools/add_hand.py --list                # the hands added on this machine
    python tools/add_hand.py --remove NAME         # take one out again

MODEL        a .urdf file, or a MuJoCo .xml file. It is read where it is and
             never changed. A URDF is built into a MuJoCo model first, with
             its <mimic> couplings kept as joint couplings (as
             tools/fetch_hands.py builds l20_public).
DESCRIPTION  a small JSON file saying what the model cannot say for itself:

    {"schema": "gramian-workshop/hand-description@1",
     "key": "my_hand",                       the name the tools will know it by
     "name": "My hand, version 2",           a title for results
     "fingers": {"thumb": "thumb_tip_link",  one line per finger: a label, and
                 "index": "index_tip_link"}, the body that carries the fingertip
     "datasheet_torque_Nm": [0.7, "where it was published"],   optional
     "datasheet_no_load_speed": [81, "rev/min", "where published"],   optional
     "caveat": "..."}                                          optional

Everything else is worked out from the model by the same rules as for every
other hand (gramian_workshop/hands.py): a finger's joints are those between
its fingertip body and the base, less the joints every finger shares (a
wrist); the palm is every body that belongs to no finger; the fingertip
point and the contact pad come from the fingertip body's collision geometry.

The pair is checked by building the hand as the hold maker and the check
build it. Anything missing or wrong is said in plain words and nothing is
added. A hand that passes is written to external/added_hands/<key>/ (or the
folder named by GMW_ADDED_HANDS), where gramian_workshop/hands.py finds it.

Needs mujoco (pip install -e .[hands]).
"""
import argparse
import json
import os
import re
import shutil
import sys
import traceback
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gramian_workshop import hands  # noqa: E402

DESCRIPTION_SCHEMA = "gramian-workshop/hand-description@1"
KEYS = {"schema", "key", "name", "fingers", "datasheet_torque_Nm", "datasheet_no_load_speed",
        "caveat"}
BUILT_MODEL = "hand.xml"          # a URDF built into a MuJoCo model, beside the entry
CHECK_KEY = "__add_hand_check__"  # the hand's name in the table while it is checked
PACKAGE_UP = 5                    # folders above a URDF searched for a package:// mesh


class Problems(Exception):
    """What is missing or wrong, one plain sentence per item."""

    def __init__(self, items):
        self.items = [items] if isinstance(items, str) else list(items)
        super().__init__("; ".join(self.items))


# ----------------------------------------------------------------------
# the description file (no mujoco needed)
# ----------------------------------------------------------------------
def read_description(path):
    """The description file as a dict. Problems if it cannot be read or is
    not what the header describes."""
    if not os.path.isfile(path):
        raise Problems(f"The description file {path} does not exist.")
    try:
        with open(path, encoding="utf-8-sig") as fh:
            d = json.load(fh)
    except ValueError as e:
        raise Problems(f"The description file {path} is not valid JSON: {e}. "
                       "Check for a missing comma, quote or bracket near that place.") from None
    except OSError as e:
        raise Problems(f"The description file {path} cannot be read: {e}.") from None
    bad = check_description(d)
    if bad:
        raise Problems(bad)
    return d


def check_description(d):
    """Every problem with a description (already parsed), as sentences."""
    if not isinstance(d, dict):
        return ["The description must be one JSON object: { ... }."]
    out = []
    if d.get("schema") != DESCRIPTION_SCHEMA:
        out.append(f'"schema" must be "{DESCRIPTION_SCHEMA}"'
                   + (f' (it is {json.dumps(d["schema"])}).' if "schema" in d else " (it is missing)."))
    for k in sorted(set(d) - KEYS):
        out.append(f'"{k}" is not something a description holds. It may hold: '
                   + ", ".join(sorted(KEYS)) + ".")
    key = d.get("key")
    if not isinstance(key, str) or not re.fullmatch(r"[a-z][a-z0-9_]*", key or ""):
        out.append('"key" is the name the tools will know the hand by. It must be given, start '
                   "with a lower-case letter and hold only lower-case letters, digits and "
                   'underscores, for example "my_hand".')
    elif key in hands.BUILT_IN:
        out.append(f'"key" is "{key}", which a hand in the table already has. Choose another name.')
    if "name" in d and not (isinstance(d["name"], str) and d["name"].strip()):
        out.append('"name" must be a title in words (or leave it out, and the key is used).')
    f = d.get("fingers")
    if not isinstance(f, dict) or not f:
        out.append('"fingers" is missing. Give one line per finger: a label and the name of the '
                   'body that carries the fingertip, for example {"thumb": "thumb_distal", '
                   '"index": "index_distal"}.')
    else:
        for lab, body in f.items():
            if not isinstance(body, str) or not body:
                out.append(f'Finger "{lab}": the fingertip body must be a body name in quotes.')
        if len(f) < 2:
            out.append("Only one finger is given. The check needs two or more: a joint that "
                       "every finger shares is read as a wrist, so with one finger every joint "
                       "would be read as a wrist.")
        twice = sorted({b for b in f.values() if isinstance(b, str) and list(f.values()).count(b) > 1})
        for b in twice:
            out.append(f'The body "{b}" is given as the fingertip of more than one finger.')
    ds = d.get("datasheet_torque_Nm")
    if ds is not None:
        ok = (isinstance(ds, list) and len(ds) == 2 and isinstance(ds[0], (int, float))
              and not isinstance(ds[0], bool) and ds[0] > 0 and isinstance(ds[1], str) and ds[1].strip())
        if not ok:
            out.append('"datasheet_torque_Nm" must be a list of two: the largest joint torque in '
                       'N.m (a number above 0) and where it was published, for example '
                       '[0.7, "maker\'s data sheet, 2024"]. Leave it out if no figure is published.')
    sp = d.get("datasheet_no_load_speed")
    if sp is not None:
        ok = (isinstance(sp, list) and len(sp) == 3 and isinstance(sp[0], (int, float))
              and not isinstance(sp[0], bool) and sp[0] > 0 and sp[1] in hands.SPEED_UNITS
              and isinstance(sp[2], str) and sp[2].strip())
        if not ok:
            out.append('"datasheet_no_load_speed" must be a list of three: the joint motor\'s '
                       'no-load speed (a number above 0), its unit (one of '
                       + ", ".join(sorted(hands.SPEED_UNITS)) + ') and where it was published, '
                       'for example [81, "rev/min", "maker\'s data sheet, 2024"]. It is used only '
                       'by the torque-at-speed check. Leave it out if no figure is published.')
    if "caveat" in d and not isinstance(d["caveat"], str):
        out.append('"caveat" must be a sentence in quotes.')
    return out


# ----------------------------------------------------------------------
# the model file
# ----------------------------------------------------------------------
def _mj():
    try:
        import mujoco
    except ImportError:
        raise Problems("MuJoCo is not installed in this Python, and a hand model cannot be read "
                       "without it. Install it with: pip install -e .[hands]") from None
    return mujoco


def _kind(model):
    """"urdf" or "mjcf", from the file's root element."""
    if not os.path.isfile(model):
        raise Problems(f"The model file {model} does not exist.")
    try:
        root = ET.parse(model).getroot()
    except ET.ParseError as e:
        raise Problems(f"The model file {model} is not well-formed XML: {e}.") from None
    except OSError as e:
        raise Problems(f"The model file {model} cannot be read: {e}.") from None
    if root.tag == "robot":
        return "urdf", root
    if root.tag == "mujoco":
        return "mjcf", root
    raise Problems(f"The model file {model} is neither a URDF (its first element would be "
                   f"<robot>) nor a MuJoCo model (<mujoco>): its first element is <{root.tag}>. "
                   "A xacro file must be expanded to a URDF first.")


def _find_package_mesh(uri, src):
    """A package://<package>/<rest> mesh as a file: looked for beside the
    URDF and up to PACKAGE_UP folders above it, as <folder>/<package>/<rest>
    and <folder>/<rest>. None if it is not found."""
    pkg, _, rest = uri[len("package://"):].partition("/")
    at = src
    for _ in range(PACKAGE_UP + 1):
        for cand in (os.path.join(at, pkg, rest), os.path.join(at, rest)):
            if os.path.isfile(cand):
                return os.path.abspath(cand)
        up = os.path.dirname(at)
        if up == at:
            break
        at = up
    return None


def urdf_fixed_links(root):
    """{child link: parent link} for the URDF's fixed joints. MuJoCo merges
    such a child into its parent, so the child is not a body of the model."""
    return {j.find("child").get("link"): j.find("parent").get("link")
            for j in root.findall("joint")
            if j.get("type") == "fixed" and j.find("child") is not None and j.find("parent") is not None}


def build_urdf(model, out_xml):
    """A URDF built into a MuJoCo model at out_xml. The model is what the
    URDF says, nothing tuned: joint ranges and efforts as given, collision
    meshes as given (read from where they are), and each <mimic> as a
    linear joint coupling (dependent = offset + multiplier * master).
    Returns the number of couplings taken from <mimic> tags."""
    mj = _mj()
    kind, root = _kind(model)
    src = os.path.dirname(os.path.abspath(model))
    with open(model, encoding="utf-8-sig") as fh:
        text = fh.read()
    lost = []
    for uri in sorted({m.get("filename") for m in root.iter("mesh") if m.get("filename")}):
        if uri.startswith("package://"):
            found = _find_package_mesh(uri, src)
            if found is None:
                lost.append(uri)
            else:
                text = text.replace(f'"{uri}"', '"' + found.replace("\\", "/") + '"')
        elif uri.startswith("file://"):
            text = text.replace(f'"{uri}"', '"' + uri[len("file://"):] + '"')
    if lost:
        raise Problems([f'The URDF names the mesh "{u}", which was not found beside the URDF or '
                        f"in the {PACKAGE_UP} folders above it. Put the meshes where that path "
                        "points from the URDF's folder, or change the path in the URDF to one "
                        "relative to the URDF." for u in lost])
    mimic = {}
    for j in root.findall("joint"):
        mm = j.find("mimic")
        if mm is not None:
            mimic[j.get("name")] = (mm.get("joint"), float(mm.get("offset", 0)),
                                    float(mm.get("multiplier", 1)))
    comp = ('<compiler balanceinertia="true" discardvisual="true" strippath="false" meshdir="'
            + src.replace("\\", "/") + '"/>')
    if root.find("mujoco") is not None:
        raise Problems("The URDF already holds a <mujoco> element. Load it into MuJoCo yourself, "
                       "save the model as MuJoCo XML, and give that file as the model.")
    text = re.sub(r"(<robot\b[^>]*>)", lambda m: m.group(1) + "<mujoco>" + comp + "</mujoco>",
                  text, count=1)
    try:
        spec = mj.MjSpec.from_string(text)
        have = {(e.name1, e.name2): list(e.data[:5]) for e in spec.equalities
                if e.type == mj.mjtEq.mjEQ_JOINT}
        for dep, (master, off, mul) in sorted(mimic.items()):
            if (dep, master) in have:           # newer MuJoCo imports <mimic> itself
                if not all(abs(x - y) < 1e-9 for x, y in zip(have[dep, master], [off, mul, 0, 0, 0])):
                    raise Problems(f"MuJoCo read the coupling of joint {dep} differently from "
                                   "what the URDF's <mimic> says.")
                continue
            e = spec.add_equality()
            e.type = mj.mjtEq.mjEQ_JOINT
            e.name1, e.name2 = dep, master
            e.name = f"{dep}_coupling"
            e.data[:5] = [off, mul, 0, 0, 0]
        spec.compile()
        xml = spec.to_xml()
    except Problems:
        raise
    except Exception as e:                                     # noqa: BLE001
        raise Problems(f"MuJoCo could not build a model from the URDF {model}. It said: "
                       f"{_one_line(e)}") from None
    os.makedirs(os.path.dirname(out_xml), exist_ok=True)
    with open(out_xml, "w", encoding="utf-8") as fo:
        fo.write(f"<!-- Built by tools/add_hand.py from {os.path.abspath(model)}; "
                 "couplings from its <mimic> tags; meshes read from beside it. -->\n")
        fo.write(xml)
    return len(mimic)


def _one_line(e):
    return " ".join(str(e).split()) or type(e).__name__


def load_model(path):
    mj = _mj()
    try:
        return mj.MjModel.from_xml_path(path)
    except Exception as e:                                     # noqa: BLE001
        raise Problems(f"MuJoCo could not load the model {path}. It said: {_one_line(e)}") from None


def _has_collision(m, b):
    return any(m.geom_bodyid[g] == b and not (m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0)
               for g in range(m.ngeom))


def body_table(m):
    """One row per body: name, parent, its joints, whether it has collision
    geometry, whether it is the last body of a chain."""
    mj = _mj()
    kids = {int(m.body_parentid[b]) for b in range(1, m.nbody)}
    rows = []
    for b in range(1, m.nbody):
        jn = [m.joint(j).name or f"(joint {j})"
              for j in range(m.body_jntadr[b], m.body_jntadr[b] + m.body_jntnum[b])]
        depth, x = 0, int(m.body_parentid[b])
        while x > 0:
            depth, x = depth + 1, int(m.body_parentid[x])
        rows.append({"body": m.body(b).name or f"(body {b})", "depth": depth, "joints": jn,
                     "collision": _has_collision(m, b), "last": b not in kids,
                     "parent": mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, int(m.body_parentid[b])) or "world"})
    return rows


def print_bodies(m, out=print):
    out("Bodies in the model (indented under their parents):")
    for r in body_table(m):
        out("  " + "  " * r["depth"] + r["body"]
            + (f"   joints: {', '.join(r['joints'])}" if r["joints"] else "")
            + ("" if r["collision"] else "   [no collision geometry]")
            + ("   <- last body of its chain" if r["last"] else ""))


def starter(m, model):
    """A description to start from: the last body of every chain as a
    fingertip, labelled finger_1, finger_2, ... for the visitor to rename."""
    last = [r["body"] for r in body_table(m) if r["last"]]
    stem = re.sub(r"[^a-z0-9_]+", "_", os.path.splitext(os.path.basename(model))[0].lower()).strip("_")
    return {"schema": DESCRIPTION_SCHEMA, "key": ("h_" + stem) if not stem[:1].isalpha() else stem,
            "name": "(a title for this hand)",
            "fingers": {f"finger_{i + 1}": b for i, b in enumerate(last)}}


# ----------------------------------------------------------------------
# the pair, checked as the other tools will build it
# ----------------------------------------------------------------------
def _chain(m, b):
    """(hinge and slide joints, other joints) from body b back to the base."""
    mj = _mj()
    ok, other = [], []
    while b > 0:
        for j in range(m.body_jntadr[b], m.body_jntadr[b] + m.body_jntnum[b]):
            (ok if int(m.jnt_type[j]) in (int(mj.mjtJoint.mjJNT_HINGE), int(mj.mjtJoint.mjJNT_SLIDE))
             else other).append(j)
        b = int(m.body_parentid[b])
    return sorted(ok), sorted(other)


def check_structure(m, desc, fixed_links=None):
    """Problems with the fingers as the description names them, found from
    the model alone (before the hand is built)."""
    mj = _mj()
    fixed_links = fixed_links or {}
    jname = lambda j: m.joint(j).name or f"(joint {j})"            # noqa: E731
    out, chains = [], {}
    names = [m.body(b).name for b in range(1, m.nbody)]
    last = [r["body"] for r in body_table(m) if r["last"]]
    for lab, body in desc["fingers"].items():
        b = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, body)
        if b < 0:
            if body in fixed_links:
                par = fixed_links[body]
                while par in fixed_links:
                    par = fixed_links[par]
                out.append(f'Finger "{lab}": "{body}" is a link of the URDF but not a body of '
                           f'the model, because a fixed joint holds it to "{par}" and MuJoCo '
                           f'merges the two. Name "{par}" instead; its collision geometry now '
                           f'includes that of "{body}".')
            else:
                near = [n for n in names if body.lower() in n.lower() or n.lower() in body.lower()]
                out.append(f'Finger "{lab}": the model has no body named "{body}". '
                           + (f"Close names: {', '.join(near)}. " if near else "")
                           + f"The last bodies of the model's chains are: {', '.join(last)}.")
            continue
        chains[lab], other = _chain(m, b)
        if other:
            out.append(f'Finger "{lab}": the joints {", ".join(jname(j) for j in other)} between '
                       f'"{body}" and the base are free or ball joints. The check reads hinge and '
                       "slide joints only. Fix the base of the hand in the model (remove the "
                       "free joint) and give ball joints as hinges.")
        if b <= 0 or not _has_collision(m, b):
            out.append(f'Finger "{lab}": the body "{body}" has no collision geometry, so there is '
                       "nothing for the object to touch. In MuJoCo XML a geom with contype and "
                       "conaffinity both 0 is for display only; in a URDF the link needs a "
                       "<collision> element.")
    if out or len(chains) < 2:
        return out
    shared = set.intersection(*[set(c) for c in chains.values()])
    labs = list(chains)
    for lab in labs:
        own = [j for j in chains[lab] if j not in shared]
        if not own:
            out.append(f'Finger "{lab}": it has no joint of its own. Every joint between '
                       f'"{desc["fingers"][lab]}" and the base is also on the way to every other '
                       "fingertip, so they are read as a wrist. Check that each fingertip body "
                       "is on a different finger.")
        for j in own:
            lo, hi = m.jnt_range[j]
            if not m.jnt_limited[j] or not hi > lo:
                out.append(f'Finger "{lab}": joint {jname(j)} has no range of motion in the '
                           "model. The hold is posed from the joint ranges, so each finger joint "
                           'needs one (range="lo hi" in MuJoCo XML, <limit lower= upper=> in a URDF).')
    for i, a in enumerate(labs):
        for b_ in labs[i + 1:]:
            both = sorted((set(chains[a]) & set(chains[b_])) - shared)
            if both:
                out.append(f'Fingers "{a}" and "{b_}" share the joints '
                           f'{", ".join(jname(j) for j in both)}, which the other fingers do '
                           "not. Two fingers must not move each other. If one fingertip body "
                           "sits on the other's finger, name one body per finger; if the two "
                           "branch from one moving link, give them as one finger.")
    return out


def check_pair(model_xml, desc, fixed_links=None):
    """Build the hand from the model and the description, as the hold maker
    and the check build it. Returns (summary dict, warnings). Problems if
    anything is missing or wrong."""
    mj = _mj()
    m = load_model(model_xml)
    bad = check_structure(m, desc, fixed_links)
    if bad:
        raise Problems(bad)
    spec = {"name": desc.get("name") or desc["key"], "base": "local",
            "file": os.path.abspath(model_xml), "fingers": dict(desc["fingers"]), "added": True}
    if desc.get("datasheet_torque_Nm"):
        spec["datasheet_torque_Nm"] = tuple(desc["datasheet_torque_Nm"])
    if desc.get("datasheet_no_load_speed"):
        spec["datasheet_no_load_speed"] = tuple(desc["datasheet_no_load_speed"])
    from gramian_workshop import follow as fo
    import make_hold_leader as mh
    hands.HANDS[CHECK_KEY] = spec
    try:
        hand = hands.Hand(CHECK_KEY)
        hand.key = desc["key"]                 # so any message names the visitor's hand
        d = mj.MjData(hand.model)
        d.qpos[:] = hand.apply_coupling(np_array(hand.base_q))
        mj.mj_kinematics(hand.model, d)
        fingers, warn, bad, no_limit = [], [], [], []
        for f in hand.fingers:
            ik = fo.FingerIK(hand, f)
            body = hand.model.body(f.tip_body).name
            if f.tip_body not in ik.proxies(middle=True):
                bad.append(f'Finger "{f.label}": no contact pad could be made from the collision '
                           f'geometry of "{body}".')
                continue
            a, e, r, side, _, _ = mh.capsule(hand, f, d, ik)
            n_in, lim, src = hand.inputs(f)
            M, lims = fo.torque_limits(hand, f)
            fingers.append({
                "finger": f.label, "tip_body": body,
                "joints": [hand.model.joint(j).name for j in f.joints],
                "coupled_joints": [hand.model.joint(j).name for j in f.joints if j in hand.coupled],
                "inputs": int(n_in), "inputs_source": src,
                "torque_limits": ("none in the model: strength not checked" if M is None else
                                  "data sheet figure" if spec.get("datasheet_torque_Nm")
                                  and lim is None else lim),
                "pad_radius_mm": round(1e3 * float(r), 2),
                "pad_length_mm": round(1e3 * float(np_norm(e - a)), 2)})
            if M is None:
                no_limit.append(f.label)
            if n_in == 0:
                bad.append(f'Finger "{f.label}": none of the model\'s actuators drives its joints, '
                           "so it has no inputs.")
        if bad:
            raise Problems(bad)
        if no_limit:
            warn.append("The model gives no torque or force limit for the inputs of "
                        + ", ".join(no_limit) + ", so the verdict will not test their "
                        "strength. Add limits to the model, or a published figure as "
                        '"datasheet_torque_Nm".')
        palm = [hand.model.body(b).name for b in fo.palm_shapes(hand)]
        if not palm:
            warn.append("No body outside the fingers has collision geometry, so the palm is "
                        "not tested against the object. (The first link of a URDF is merged "
                        "into the fixed world, which the palm test does not read.)")
        summary = {"key": desc["key"], "name": spec["name"], "model": os.path.abspath(model_xml),
                   "n_joints": int(hand.model.njnt), "n_actuators": int(hand.model.nu),
                   "couplings": len(hand.coupled),
                   "shared_joints": [hand.model.joint(j).name for j in hand.shared_joints],
                   "palm_bodies": palm, "fingers": fingers}
        return summary, warn
    except Problems:
        raise
    except Exception as e:                                     # noqa: BLE001
        if os.environ.get("GMW_DEBUG"):
            traceback.print_exc()
        raise Problems(f"The hand could not be built from this pair. The step that stopped "
                       f"said: {type(e).__name__}: {_one_line(e)}. (Set GMW_DEBUG=1 to see where.)") from None
    finally:
        hands.HANDS.pop(CHECK_KEY, None)


def np_array(x):
    import numpy as np
    return np.array(x)


def np_norm(x):
    import numpy as np
    return np.linalg.norm(x)


def print_summary(s, warn, out=print):
    out(f"{s['key']}: {s['name']}")
    out(f"  model: {s['model']}")
    out(f"  {s['n_joints']} joints, {s['n_actuators']} actuators, {s['couplings']} joint couplings")
    out("  joints every finger shares (read as a wrist, held at rest): "
        + (", ".join(s["shared_joints"]) or "none"))
    out("  palm (bodies of no finger that have collision geometry): " + (", ".join(s["palm_bodies"]) or "none"))
    for f in s["fingers"]:
        out(f"  finger {f['finger']}: fingertip body {f['tip_body']}; joints {', '.join(f['joints'])}"
            + (f" (coupled to another: {', '.join(f['coupled_joints'])})" if f["coupled_joints"] else "")
            + f"; {f['inputs']} inputs ({f['inputs_source']}); torque limits: {f['torque_limits']}; "
            f"contact pad {f['pad_length_mm']} mm long, radius {f['pad_radius_mm']} mm")
    for w in warn:
        out("  note: " + w)


# ----------------------------------------------------------------------
# adding, listing, removing
# ----------------------------------------------------------------------
def add(model, description, registry=None, replace=False, check_only=False, out=print):
    """Check the pair and, unless check_only, write the entry. Returns the
    summary. Problems if anything is missing or wrong (nothing is then
    added)."""
    desc = read_description(description)
    kind, root = _kind(model)
    registry = registry or hands.added_dir()
    folder = os.path.join(registry, desc["key"])
    entry = os.path.join(folder, hands.ADDED_ENTRY)
    if os.path.exists(entry) and not (replace or check_only):
        raise Problems(f'A hand named "{desc["key"]}" was already added ({entry}). Choose another '
                       '"key", or pass --replace to add this one in its place.')
    fixed, built = None, None
    work = folder + ".checking"
    try:
        if kind == "urdf":
            fixed = urdf_fixed_links(root)
            built = os.path.join(work, BUILT_MODEL)
            n_mimic = build_urdf(model, built)
            out(f"Built a MuJoCo model from the URDF ({n_mimic} couplings from <mimic> tags).")
        summary, warn = check_pair(built or model, desc, fixed)
        summary["model"] = os.path.abspath(model)
        print_summary(summary, warn, out)
        if check_only:
            out("The pair is sound. Nothing was added (--check-only).")
            return summary
        os.makedirs(folder, exist_ok=True)
        if built:
            os.replace(built, os.path.join(folder, BUILT_MODEL))
        e = {"schema": hands.ADDED_SCHEMA, "key": desc["key"], "name": summary["name"],
             "model": BUILT_MODEL if built else os.path.abspath(model),
             "fingers": desc["fingers"],
             "source_model": os.path.abspath(model),
             "description_file": os.path.abspath(description),
             "written_by": "tools/add_hand.py"}
        for k in ("datasheet_torque_Nm", "datasheet_no_load_speed", "caveat"):
            if desc.get(k):
                e[k] = desc[k]
        tmp = entry + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(e, fh, indent=1)
        os.replace(tmp, entry)                 # the entry appears whole or not at all
    finally:
        shutil.rmtree(work, ignore_errors=True)
    hands.load_added(registry if registry != hands.added_dir() else None, warn=False)
    out(f'Added. The tools now know this hand as "{desc["key"]}" ({entry}).')
    if not built:
        out("The model is read from where it is: if you move it, add the hand again.")
    else:
        out("The meshes are read from beside the URDF: if you move them, add the hand again.")
    k = desc["key"]
    out("Next, a hold this hand can do by construction, and the check on it:")
    out(f"  python tools/make_hold_leader.py {k}")
    out(f"  python tools/run_follow.py --hands {k} --leader-id hold_{k}__self__selfoverlapfix "
        f"--placement-from outputs/hold_placement_{k}__selfoverlapfix.json --tag fixed")
    return summary


def listed(registry=None):
    registry = registry or hands.added_dir()
    rows = []
    for sub in sorted(os.listdir(registry)) if os.path.isdir(registry) else []:
        p = os.path.join(registry, sub, hands.ADDED_ENTRY)
        if os.path.isfile(p):
            try:
                key, spec = hands.added_entry(p)
                rows.append((key, spec["name"], hands.os.path.normpath(
                    os.path.join(ROOT, "data", "hands", spec["file"])), None))
            except ValueError as e:
                rows.append((sub, None, p, str(e)))
    return rows


def remove(key, registry=None):
    registry = registry or hands.added_dir()
    folder = os.path.join(registry, key)
    entry = os.path.join(folder, hands.ADDED_ENTRY)
    if not re.fullmatch(r"[a-z][a-z0-9_]*", key) or not os.path.isfile(entry):
        raise Problems(f'No added hand is named "{key}". --list shows the ones there are.')
    for name in (hands.ADDED_ENTRY, BUILT_MODEL):
        p = os.path.join(folder, name)
        if os.path.isfile(p):
            os.remove(p)
    try:
        os.rmdir(folder)                       # only if nothing else was put there
    except OSError:
        pass


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 epilog="The steps are in docs/BRING_YOUR_HAND.md.")
    ap.add_argument("model", nargs="?", help="a .urdf file or a MuJoCo .xml file")
    ap.add_argument("description", nargs="?", help="the description file (JSON)")
    ap.add_argument("--check-only", action="store_true", help="check the pair, add nothing")
    ap.add_argument("--replace", action="store_true",
                    help="add in place of a hand already added under the same key")
    ap.add_argument("--list", action="store_true", help="list the hands added on this machine")
    ap.add_argument("--remove", metavar="NAME", help="take an added hand out again")
    ap.add_argument("--registry", default=None,
                    help="folder of added hands (default: GMW_ADDED_HANDS, else external/added_hands)")
    a = ap.parse_args(argv)
    try:
        if a.list:
            rows = listed(a.registry)
            if not rows:
                print("No hands have been added.")
            for key, name, where, bad in rows:
                print(f"{key}: {name}  ({where})" if bad is None else f"{key}: cannot be used: {where} {bad}")
            return 0
        if a.remove:
            remove(a.remove, a.registry)
            print(f'Removed "{a.remove}". Its leaders and results, if any, were left where they are.')
            return 0
        if not a.model:
            ap.print_usage()
            print("Give a model file. The steps are in docs/BRING_YOUR_HAND.md.")
            return 2
        if not a.description:
            kind, _ = _kind(a.model)
            tmp = None
            try:
                if kind == "urdf":
                    import tempfile
                    tmp = tempfile.mkdtemp()
                    build_urdf(a.model, os.path.join(tmp, BUILT_MODEL))
                m = load_model(os.path.join(tmp, BUILT_MODEL) if tmp else a.model)
            finally:
                if tmp:
                    shutil.rmtree(tmp, ignore_errors=True)
            print_bodies(m)
            print("\nA description to start from (save it as a .json file, keep one line per "
                  "finger, and give each finger its own label; \"thumb\" is the one label with "
                  "a meaning):\n")
            print(json.dumps(starter(m, a.model), indent=1))
            return 0
        add(a.model, a.description, a.registry, a.replace, a.check_only)
        return 0
    except Problems as p:
        print("Not added. " + ("This needs putting right:" if len(p.items) == 1 else
                               "These need putting right:"))
        for i, item in enumerate(p.items, 1):
            print(f"  {i}. {item}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
