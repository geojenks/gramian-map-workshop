"""tools/run_follow_snapshots.py with LEAP's fingertip contacts placed on its pads.

LEAP's fingertip link carries a soft pad (the "*_tip" mesh) over a rigid core of boxes. follow.py's
contact candidates spread over every collision shape on the link's palm side, so the check can touch
with the rigid core at the base of the fingertip, behind the pad. Here, for LEAP only, the candidate
points on the fingertip link come from the pad alone; the middle link is unchanged, and every other
test (reach, facing, overlap of the whole link, strength) is as before. follow.py itself is unchanged:
its candidate function is wrapped in this process only.

    python tools/run_follow_snapshots_leap_pads.py --snapshots <..._snapshots.json> --out-dir <dir>
        [--no-middle]    LEAP touches with its fingertip pads only, not its middle links

Writes follow_leap__<leader id>__pads.json (or __pads_tips_only.json). Never overwrites.
"""
import json, os, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, "tools"))
import numpy as np  # noqa: E402
from gramian_workshop import follow as F  # noqa: E402
import run_follow_snapshots as S  # noqa: E402

_orig_cands, _orig_verts = F.pad_candidates, F._link_vertices


def _pad_vertices(m, body):
    """The pad mesh on this body, in the body's frame; None if it has none."""
    import mujoco as mj
    pts = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != body or m.geom_type[g] != mj.mjtGeom.mjGEOM_MESH or not m.geom(g).name.endswith("_tip"):
            continue
        if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0:
            continue
        R = np.zeros(9); mj.mju_quat2Mat(R, m.geom_quat[g]); mid = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        pts.append(v @ R.reshape(3, 3).T + m.geom_pos[g])
    return np.vstack(pts) if pts else None


def main():
    no_middle = "--no-middle" in sys.argv
    if no_middle:
        sys.argv.remove("--no-middle")

    def cands(hand, finger, middle=False):
        if hand.key != "leap":
            return _orig_cands(hand, finger, middle)
        if no_middle:
            middle = False
        F._link_vertices = lambda m, b: (_pad_vertices(m, b) if b == finger.tip_body else _orig_verts(m, b))
        try:
            out = _orig_cands(hand, finger, middle)
        finally:
            F._link_vertices = _orig_verts
        tip = [x for x in out if x[0] == finger.tip_body]
        assert tip, f"{finger.label}: no pad candidates"
        print(f"  {finger.label}: {len(tip)} candidate points on the pad, {len(out) - len(tip)} on the middle link", flush=True)
        return out
    F.pad_candidates = cands

    tag = "__pads_tips_only" if no_middle else "__pads"
    a = sys.argv
    lid = a[a.index("--leader-id") + 1] if "--leader-id" in a else "hora_seed42_five_snapshots"
    out_dir = a[a.index("--out-dir") + 1]
    target = os.path.join(out_dir, f"follow_leap__{lid}{tag}.json")
    assert not os.path.exists(target), f"{target} exists: not overwriting"
    if "--hands" not in a:
        sys.argv += ["--hands", "leap"]
    sys.argv += ["--leader-id", lid] if "--leader-id" not in a else []
    S.main()
    src = os.path.join(out_dir, f"follow_leap__{lid}.json")
    r = json.load(open(src, encoding="utf-8"))
    r["contact_candidates"] = ("LEAP fingertip contacts on the pad mesh only (tools/run_follow_snapshots_leap_pads.py)"
                               + ("; no middle-link contacts" if no_middle else "; middle links as in follow.py"))
    json.dump(r, open(target, "w", encoding="utf-8"), indent=1, default=float)
    os.replace(src, os.path.join(out_dir, f"_unlabelled_follow_leap__{lid}{tag}.json"))   # the same result before labelling; kept, not deleted
    print("wrote", target, flush=True)


if __name__ == "__main__":
    main()
