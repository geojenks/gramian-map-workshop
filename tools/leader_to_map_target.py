"""A simulation's task file as a target for the map page.

    python tools/leader_to_map_target.py my_task.json --tag mine

reads a task file in the form of docs/TASK_FILE_FORMAT.md (a leader file)
and writes outputs/map_targets/map_target_mine.json. On the map page, choose
that file under "Task". The page draws one line for each contact station:
the path its contact point takes over the object, on the map's three
tip-travel axes. The page only draws the target beside the actuators'
paths. It does not test any actuator against it.

How a contact path becomes a line on the map's axes:

* A station makes and breaks contact. Each unbroken stretch of contact is
  one path. By default the longest stretch of each station is used (the
  first, where two are as long); --all-stretches writes every stretch of
  two frames or more.
* The path is the contact point's position in each frame of the stretch,
  less its position in the stretch's first frame. It is taken in the file's
  own frame: the object's motion is not taken off.
* It is then turned into the frame of a finger standing on a palm beside
  the object, the placement the map page opens with. The file's +z is up.
  The radial direction is the level part of the line from the object's
  centre to the contact point, in the stretch's first frame. The finger's
  frame is gramian_workshop.finger.mount_rotation's "palm" frame at the yaw
  stored in data/map_task_cones_palm_mount.json, fixed where the stretch
  starts. Its three parts are the map's axial travel and its two sideways
  travels.

Nothing is written into data/, and an existing file is never overwritten.
"""
import argparse
import hashlib
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.finger import mount_rotation  # noqa: E402

KIND = "target for the map page"
MADE_BY = "tools/leader_to_map_target.py"
MOUNT = "palm"
CONE_FILE = os.path.join(ROOT, "data", "map_task_cones_palm_mount.json")
AXES = ("axial_m", "side_y_m", "side_z_m")
OUT_DIR = os.path.join("outputs", "map_targets")
LEVEL_TOL_M = 1e-9      # a contact this close to the z axis has no radial direction
DECIMALS = 7            # the task files hold six


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fo:
        for block in iter(lambda: fo.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def palm_yaw_deg(cone_file=CONE_FILE):
    """The yaw of the palm placement the map page is made with."""
    with open(cone_file, encoding="utf-8") as fo:
        cones = json.load(fo)["cones"]
    yaws = {float(c["yaw_deg"]) for c in cones if c.get("mount") == MOUNT}
    if len(yaws) != 1:
        raise ValueError(f"{cone_file}: expected one yaw for the palm "
                         f"placement, found {sorted(yaws)}")
    return yaws.pop()


def stretches(touching):
    """[(first, last)] frame indexes of each unbroken stretch of contact."""
    out, start = [], None
    for i, on in enumerate(list(touching) + [False]):
        if on and start is None:
            start = i
        elif not on and start is not None:
            out.append((start, i - 1))
            start = None
    return out


def finger_frame(centre, point, yaw_deg):
    """World-from-finger rotation for a finger standing on a palm beside the
    object, at the contact `point`. Raises ValueError where the contact has
    no radial direction."""
    r = np.asarray(point, float) - np.asarray(centre, float)
    r[2] = 0.0
    size = float(np.linalg.norm(r))
    if size < LEVEL_TOL_M:
        raise ValueError("its contact starts on the vertical line through "
                         "the object's centre, so it has no radial direction "
                         "to stand a finger by")
    e_r = r / size
    e_phi = np.cross([0.0, 0.0, 1.0], e_r)
    return mount_rotation(None, MOUNT, None, e_r, e_phi,
                          math.radians(yaw_deg))


def _points(frames, station, first, last, path):
    out = []
    for i in range(first, last + 1):
        try:
            p = frames[i]["contact_point"][station]
        except (KeyError, IndexError, TypeError):
            p = None
        if (not isinstance(p, (list, tuple)) or len(p) != 3
                or not all(isinstance(v, (int, float))
                           and not isinstance(v, bool)
                           and math.isfinite(v) for v in p)):
            raise ValueError(f"{path}: frame {i}, station {station} is in "
                             "contact but has no usable 'contact_point'")
        out.append(p)
    return np.asarray(out, float)


def convert(path, all_stretches=False, yaw_deg=None, name=None):
    """The target document for the leader file at `path`. `name` is the
    words the map page lists it by; the task file's own name where none is
    given."""
    with open(path, encoding="utf-8") as fo:
        doc = json.load(fo)
    frames = doc.get("frames") if isinstance(doc, dict) else None
    if not isinstance(frames, list) or not frames:
        raise ValueError(f"{path}: not a task file with frames; see "
                         "docs/TASK_FILE_FORMAT.md")
    try:
        n = len(frames[0]["contact"])
        touching = np.array([[bool(c) for c in f["contact"]]
                             for f in frames])
        centres = np.array([f["obj_pos"] for f in frames], float)
        times = [float(f["t"]) for f in frames]
    except (KeyError, TypeError, ValueError) as e:
        raise ValueError(f"{path}: a frame lacks 't', 'obj_pos' or "
                         f"'contact' ({e}); see docs/TASK_FILE_FORMAT.md")
    if touching.ndim != 2 or touching.shape[1] != n or centres.shape != (
            len(frames), 3):
        raise ValueError(f"{path}: the frames do not all hold the same "
                         "number of stations and a three-part 'obj_pos'")
    if yaw_deg is None:
        yaw_deg = palm_yaw_deg()

    targets, left_out = [], []
    for s in range(n):
        runs = [r for r in stretches(touching[:, s]) if r[1] > r[0]]
        if not runs:
            left_out.append({"station": s, "reason":
                             "it is never in contact for two frames running"})
            continue
        if not all_stretches:
            runs = [max(runs, key=lambda r: r[1] - r[0])]   # first of equals
        for first, last in runs:
            pts = _points(frames, s, first, last, path)
            try:
                R = finger_frame(centres[first], pts[0], yaw_deg)
            except ValueError as e:
                left_out.append({"station": s, "frames": [first, last],
                                 "reason": str(e)})
                continue
            moved = (pts - pts[0]) @ R            # rows: finger-frame travel
            targets.append({
                "name": f"Station {s}, {times[first]:.3f} s to "
                        f"{times[last]:.3f} s",
                "station": s, "frames": [first, last],
                "along": {k: [round(float(v), DECIMALS) + 0.0
                              for v in moved[:, j]]
                          for j, k in enumerate(AXES)}})
    if not targets:
        raise ValueError(f"{path}: no station gives a contact path: "
                         + "; ".join(f"station {x['station']}: {x['reason']}"
                                     for x in left_out))
    if name is None:
        name = "Contact paths of %s" % (
            doc.get("name") or doc.get("id") or os.path.basename(path))
    return {
        "kind": KIND, "made_by": MADE_BY,
        "id": str(doc.get("id") or os.path.splitext(
            os.path.basename(path))[0]),
        "name": str(name),
        "note": (
            "One line for each contact station of the task file: the path "
            "its contact point takes, measured from where the stretch of "
            "contact starts. "
            + ("Every unbroken stretch of contact of two frames or more is "
               "a line. " if all_stretches else
               "Only each station's longest unbroken stretch of contact is "
               "drawn (the first, where two are as long). ")
            + "The path is taken in the task file's own frame, so the "
            "object's motion is not taken off. It is turned into the frame "
            "of a finger standing on a palm beside the object: the file's "
            "+z is up, and the radial direction is the level part of the "
            "line from the object's centre to the contact point where the "
            "stretch starts. That frame is fixed for the stretch. The "
            "finger's frame is the palm placement of "
            "gramian_workshop.finger.mount_rotation at the yaw below. "
            "The map page draws these lines beside the actuators' paths "
            "and does not test any actuator against them."),
        "placement": {"mount": MOUNT, "yaw_deg": yaw_deg,
                      "yaw_from": "data/map_task_cones_palm_mount.json"},
        "show": list(AXES),
        "source": {"file": os.path.basename(path), "sha256": sha256(path),
                   "frames": len(frames), "stations": n},
        "left_out": left_out,
        "targets": targets}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("leader", help="a task file in the form of "
                                   "docs/TASK_FILE_FORMAT.md")
    ap.add_argument("--tag", default=None,
                    help="the output is map_target_<tag>.json; the task "
                         "file's own name where none is given")
    ap.add_argument("--out", default=OUT_DIR,
                    help="output folder, relative to the repository")
    ap.add_argument("--all-stretches", action="store_true",
                    help="a line for every unbroken stretch of contact of "
                         "two frames or more, not only each station's "
                         "longest")
    ap.add_argument("--name", default=None,
                    help="the words the map page lists the target by; "
                         "\"Contact paths of\" and the task file's own "
                         "name where none is given")
    a = ap.parse_args(argv)
    if not os.path.isfile(a.leader):
        print("ERROR: %s: no such file" % a.leader)
        return 1
    tag = a.tag or os.path.splitext(os.path.basename(a.leader))[0]
    out = os.path.join(ROOT, a.out, "map_target_%s.json" % tag)
    if os.path.exists(out):
        print("REFUSING to overwrite:", out, "- give another --tag")
        return 1
    try:
        doc = convert(a.leader, a.all_stretches, name=a.name)
    except ValueError as e:
        print("ERROR:", e)
        return 1
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as fo:
        json.dump(doc, fo, indent=1, allow_nan=False)
        fo.write("\n")
    print("wrote", out)
    for t in doc["targets"]:
        end = [t["along"][k][-1] * 1e3 for k in AXES]
        print("  %s: %d frames, ends at axial %.2f mm, sideways y %.2f mm, "
              "sideways z %.2f mm" % (t["name"], len(t["along"][AXES[0]]),
                                      *end))
    for x in doc["left_out"]:
        print("  left out, station %d: %s" % (x["station"], x["reason"]))
    print('On the map page, choose this file under "Task".')
    return 0


if __name__ == "__main__":
    sys.exit(main())
