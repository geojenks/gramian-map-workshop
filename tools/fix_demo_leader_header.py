"""Write a corrected copy of a demonstration leader's header, beside it, as
data/leaders/<id>__nframesfix.json (issue 22). The stored leader is not
changed.

    python tools/fix_demo_leader_header.py \
        data/leaders/tennis_new_cup_take3_derived.json \
        data/leaders/tennis_new_cup_take3_handframe.json

What changes, and nothing else:
  n_frames   set to the file's own frame count. The source recording's
             count, which the file carried, is kept as n_frames_source.
  notes      the notes carried from the source recording are tested
             against this file. One that no longer holds is kept, word for
             word, behind "OUT OF DATE in this file: <why>.". A note this
             tool has no test for is left as it is.
  added      id gains "__nframesfix"; "supersedes" names the stored leader;
             "copied_from" gives its path, sha256, this tool and the
             workshop commit; a last note says what was done.
Frames, forces, contacts and every other field are copied unchanged.
"""
import json
import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from _provenance import sha256  # noqa: E402

SUFFIX = "__nframesfix"
RADIAL_TOL_M = 5e-4   # a contact point this far from the radial rule breaks it


def _forces(L):
    n = sum(1 for f in L["frames"]
            if any(v is not None and any(v) for v in f["force_on_object_N"]))
    if n:
        return (f"{n} of {len(L['frames'])} frames carry derived forces "
                "(see derivation)")
    return None


def _obj_quat(L):
    if all(f.get("obj_quat") is None for f in L["frames"]):
        return "obj_quat is null in every frame"
    return None


def _contact_point(L):
    """The note: contact_point = ball centre + radius x unit(tip - centre).
    station_pos is the tip moved toward the centre, so it lies on the same
    ray; test each contact against that ray."""
    R = float(L["object"]["radius_m"])
    off = total = 0
    for f in L["frames"]:
        c = np.array(f["obj_pos"], float)
        for i, cp in enumerate(f["contact_point"]):
            sp = (f.get("station_pos") or [None] * len(f["contact_point"]))[i]
            if cp is None or sp is None:
                continue
            u = np.array(sp, float) - c
            total += 1
            off += np.linalg.norm(c + R * u / np.linalg.norm(u) - cp) > RADIAL_TOL_M
    if off:
        why = (f"{off} of {total} contact points lie more than "
               f"{RADIAL_TOL_M * 1e3:g} mm from that point")
        if "handframe" in L:
            why += (". Here contact_point is re-made from the hand skeleton "
                    f"({L['handframe'].get('script')}): the point on the ball "
                    "nearest the digit's finger segments or its pad point. "
                    "contact_normal is still into the ball")
        return why
    return None


TESTS = (("force_on_object_N:", _forces),
         ("obj_quat ", _obj_quat),
         ("contact_point = ball centre", _contact_point))


def fix(path):
    with open(path, encoding="utf-8") as f:
        L = json.load(f)
    old_id = L["id"]
    out = dict(L)
    out["id"] = old_id + SUFFIX
    n_src = L.get("n_frames")
    out["n_frames"] = len(L["frames"])
    out["n_frames_source"] = n_src
    notes, marked = [], 0
    for note in L.get("notes") or []:
        why = None
        for head, test in TESTS:
            if note.startswith(head):
                why = test(L)
        if why:
            notes.append(f"OUT OF DATE in this file: {why}. "
                         f"Carried from the source: {note}")
            marked += 1
        else:
            notes.append(note)
    rel = os.path.relpath(os.path.abspath(path), ROOT).replace(os.sep, "/")
    notes.append(
        f"Copy of {old_id} with n_frames set to this file's frame count "
        f"(n_frames_source is the source recording's) and {marked} out-of-date "
        "carried notes marked; every other field unchanged. "
        "n_held_frames_without_ball_centre_dropped and "
        "n_frames_without_tip_per_station count frames of the source "
        "recording, not of this file.")
    out["notes"] = notes
    out["supersedes"] = old_id
    head = subprocess.run(["git", "-C", ROOT, "rev-parse", "HEAD"],
                          capture_output=True, text=True).stdout.strip()
    out["copied_from"] = {"file": rel, "sha256": sha256(path),
                          "tool": "tools/fix_demo_leader_header.py",
                          "workshop_commit": head}
    p = os.path.join(os.path.dirname(os.path.abspath(path)),
                     out["id"] + ".json")
    if os.path.exists(p):
        raise SystemExit(f"{p} exists; not overwriting it")
    with open(p, "w", encoding="utf-8") as f:
        json.dump(out, f, separators=(",", ":"))
    print(f"wrote {p}: n_frames {n_src} -> {out['n_frames']}, "
          f"{marked} notes marked out of date")


if __name__ == "__main__":
    for a in sys.argv[1:]:
        fix(a)
