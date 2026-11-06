"""A known-good task for one hand: a static hold the hand can do by
construction, written as a leader file, so the follow check has a case
whose answer is known before anyone trusts it on a real task.

    python tools/make_hold_leader.py allegro            # writes the leader and the fixed placement
    python tools/make_hold_leader.py allegro --run      # ... and runs the follow check on it

How the hold is made (nothing is chosen by hand):
  pose      each finger's joints at one fraction of their ranges toward the
            end they curl to, the fraction chosen per finger (below); shared
            (wrist) joints at the model's rest pose; couplings applied
  contact   the follow check's own tangent contact (follow.FingerIK.proxies):
            each distal link is a capsule (axis a-e, radius r) and the object
            touches it where the sphere's centre is nearest the axis, at
            whatever point along the link that falls. The contact normal is
            the direction from that axis point to the centre; "facing" is its
            angle to the link's palm-side direction (0 = the middle of the pad)
  object    a sphere of a given radius R (--radius, default 30 mm) whose
            centre is R + r_i from every link's axis, by Gauss-Newton on
            distance - r_i - R over the fingers; the leftover |distance - r_i - R|
            is the contact miss and is stored, as is the facing
  curl      each finger's fraction is picked from a grid (--curl, default 0
            to 1 in steps of 0.05) by coordinate descent on the score
            max miss + FACE_WEIGHT_M * max facing (the check's own weight,
            metres per radian) + PALM_WEIGHT * the palm's penetration into
            the ball (follow.palm_penetration, the check's own measure, at
            the hand's base pose) + PALM_WEIGHT * the deepest overlap of a
            finger's other links (every link from the tip back to the first
            joint's body, as the check reads them: FingerIK.shapes) with the
            ball + PALM_WEIGHT * the hand's overlap with itself (below):
            fingers in turn, each set to the grid value
            that scores best given the others, until nothing changes; started
            from every grid value (all fingers alike) and, with the term for
            the hand against itself, also from the pose the score without
            that term chooses (from a uniform curl the only single-finger
            moves that part two crossed fingers also take that finger off
            the ball); the best kept (source.sphere_fit.search_start says
            which start won; source.curl_start is that curl, or null when
            the extra start won)
  itself    the hand's overlap with itself at the whole-hand pose: the
            pairs of the hand's own bodies whose collision geoms penetrate
            in MuJoCo (tools/hold_pose_self_overlap.py: pairs_at), each
            pair's depth less its depth at the rest pose, the deepest pair
            taken. Without it each finger is curled against the ball alone
            and two fingers can be given the same space. MuJoCo leaves out
            a body against its parent and pairs the model excludes, so this
            is a lower bound. --no-self-term leaves it out, which
            reproduces the superseded hold_<hand>__self leaders
  forces    the same inward normal force at every station (--force, N).
            Their sum is not zero on a cup of fingers, so the object is given
            the mass that sum carries, with gravity along it ("frame" says
            so); the check never reads gravity, only the forces
The leader is hold_<hand>__self__selfoverlapfix (it supersedes
hold_<hand>__self, which had no term for the hand against itself and is
kept). It is in the hand's own frame, so the check needs no placement
search: the fixed placement (identity, translated to the object centre,
finger i serving station i) is written beside it
(outputs/hold_placement_<hand>__selfoverlapfix.json). With --run, the check
runs with that fixed placement, with margins (the known answer: every
frame green at zero placement cost); with --search it then runs again with
its own placement search at the coarse budget (does the search find it?).
"""
import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gramian_workshop.hands import HANDS, Hand, model_path              # noqa: E402
from gramian_workshop.follow import (FingerIK, follow, PLACE_COARSE,     # noqa: E402
                                     FACE_WEIGHT_M, palm_penetration, _mj)
from hold_pose_self_overlap import owners, pairs_at, net_of_rest         # noqa: E402

G = 9.81
SCHEMA = "gramian-workshop/leader@1"
RADIUS_M = 0.030         # default ball: about the wave-manipulation cup task's (33.5 mm)
PALM_WEIGHT = 3.0        # metres of score per metre of overlap (palm or other links in the
                         # ball, the hand in itself)
GN_ITERS = 50
SELF_SUFFIX = "__selfoverlapfix"   # leaders whose curl score has the hand-against-itself term


def _git():
    try:
        sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
        dirty = bool(subprocess.check_output(["git", "status", "--porcelain", "tools/make_hold_leader.py"],
                                            cwd=ROOT, text=True).strip())
        return sha + ("-dirty" if dirty else "")
    except Exception:                                   # noqa: BLE001
        return None


def pose(hand, curl):
    """qpos with each finger's joints at curl[i] of their ranges toward the
    end they curl to (the larger end in magnitude, as follow._palm_side
    reads it)."""
    q = np.array(hand.base_q)
    for f, t in zip(hand.fingers, curl):
        lo, hi = f.ranges[:, 0], f.ranges[:, 1]
        big = np.where(np.abs(hi) >= np.abs(lo), hi, lo)
        small = np.where(np.abs(hi) >= np.abs(lo), lo, hi)
        q[f.qadr] = small + t * (big - small)
    return hand.apply_coupling(q)


def capsule(hand, finger, data, ik):
    """The distal link's contact capsule in the world at `data`'s pose:
    (a, e, r, side, local, others) with a, e the axis ends and side the
    palm-side unit direction, from the follow check's own model
    (FingerIK.proxies); local = (a, e, side) in the link's frame; others =
    the finger's other links' collision shapes in the world, as the check
    reads them (FingerIK.shapes, every link from the tip back to the first
    joint's body, less the tip): ("seg", a, e, r) capsules and ("pts", V)
    surface points, which must stay clear of the ball."""
    body = finger.tip_body
    a, e, r, side = ik.proxies(middle=True)[body]
    R = data.xmat[body].reshape(3, 3)
    p = data.xpos[body]
    others = []
    for b, shp in ik.shapes.items():
        if b == body:
            continue
        R2, p2 = data.xmat[b].reshape(3, 3), data.xpos[b]
        for s_ in shp:
            if s_[0] == "seg":
                others.append(("seg", p2 + R2 @ s_[1], p2 + R2 @ s_[2], float(s_[3])))
            else:
                others.append(("pts", p2 + s_[1] @ R2.T))
    return p + R @ a, p + R @ e, float(r), R @ side, (a, e, side), others


def touch(c, caps):
    """For a centre c: per finger the nearest axis point, the unit direction
    from it to c, the distance, and the facing angle (deg) to the palm side."""
    X, U, D, F = [], [], [], []
    for a, e, r, side, *_ in caps:
        ae = e - a
        t = float(np.clip((c - a) @ ae / max(ae @ ae, 1e-18), 0.0, 1.0))
        x = a + t * ae
        d = np.linalg.norm(c - x)
        u = (c - x) / d if d > 0 else side
        X.append(x), U.append(u), D.append(d)
        F.append(np.degrees(np.arccos(np.clip(u @ side, -1, 1))))
    return np.array(X), np.array(U), np.array(D), np.array(F)


def fit_ball(caps, c0, R):
    """Centre c with |c - axis_i| = R + r_i for every finger in the
    least-squares sense (Gauss-Newton from c0). Returns c, miss per finger
    (m), facing (deg), the axis points and the unit directions to c."""
    c = np.array(c0, float)
    r = np.array([cp[2] for cp in caps])
    for _ in range(GN_ITERS):
        X, U, D, F = touch(c, caps)
        res = D - r - R
        dc = np.linalg.lstsq(U, -res, rcond=None)[0]   # d res_i / d c = u_i
        c = c + dc
        if np.linalg.norm(dc) < 1e-9:
            break
    X, U, D, F = touch(c, caps)
    return c, np.abs(D - r - R), F, X, U


def other_overlap(c, caps, R):
    """Deepest overlap (m) of the fingers' other links with the ball (c, R),
    measured as the check does (follow._deepest): for a capsule R + r minus
    the distance from c to its axis segment, for surface points R minus the
    nearest point's distance; negative is clear."""
    worst = -np.inf
    for cp in caps:
        for s_ in cp[5]:
            if s_[0] == "seg":
                _, a, e, r = s_
                ae = e - a
                t = float(np.clip((c - a) @ ae / max(ae @ ae, 1e-18), 0.0, 1.0))
                worst = max(worst, R + r - float(np.linalg.norm(c - (a + t * ae))))
            else:
                worst = max(worst, R - float(np.linalg.norm(s_[1] - c, axis=1).min()))
    return worst


def score(caps, c0, R, hand, self_m=0.0):
    """self_m: the hand's overlap with itself at this pose (m), 0 when that
    term is left out."""
    c, miss, face, X, U = fit_ball(caps, c0, R)
    pen = max(0.0, palm_penetration(hand, c, R))
    ov = max(0.0, other_overlap(c, caps, R))
    return (miss.max() + FACE_WEIGHT_M * np.radians(face.max()) + PALM_WEIGHT * (pen + ov + self_m),
            c, miss, face, X, U, pen, ov)


def _start_centre(cs, R):
    """The palm-side normals' meeting point (least squares of
    c - R n_i = p_i over the pad middles): the Gauss-Newton start."""
    P = np.array([0.5 * (a + e) + r * s for a, e, r, s, *_ in cs])
    N = np.array([s for a, e, r, s, *_ in cs])
    A = np.zeros((3 * len(cs), 4))
    b = np.zeros(3 * len(cs))
    for i in range(len(cs)):
        A[3 * i:3 * i + 3, :3] = np.eye(3)
        A[3 * i:3 * i + 3, 3] = -N[i]
        b[3 * i:3 * i + 3] = P[i]
    x = np.linalg.lstsq(A, b, rcond=None)[0]
    return x[:3] if 0 < x[3] < 0.2 else P.mean(0) + R * N.mean(0)


def make(key, curls, force_N, n_frames, R, self_term=True):
    mj = _mj()
    hand = Hand(key)
    n = len(hand.fingers)
    grid = np.array(sorted(set(curls)))
    iks = [FingerIK(hand, f) for f in hand.fingers]
    # the hand against itself at a whole-hand pose (one grid index per
    # finger): the deepest pair of its own bodies, net of the rest pose
    own = owners(hand.model, hand)
    rest = pairs_at(mj, hand.model, -1, hand.apply_coupling(np.array(hand.base_q)), own) if self_term else []
    seen = {}

    def self_at(idx):
        if not self_term:
            return 0.0, None
        k_ = tuple(idx)
        if k_ not in seen:
            mm, who = net_of_rest(pairs_at(mj, hand.model, -1, pose(hand, [grid[j] for j in k_]), own), rest)
            seen[k_] = (1e-3 * mm, who)
        return seen[k_]
    # each finger's capsule at every grid value: the fingers are independent
    # chains, so one pose per grid value gives every finger's
    caps = {}
    for k, t in enumerate(grid):
        d = mj.MjData(hand.model)
        d.qpos[:] = pose(hand, [t] * n)
        mj.mj_kinematics(hand.model, d)
        for i, f in enumerate(hand.fingers):
            caps[i, k] = capsule(hand, f, d, iks[i])
    def own_m(idx, with_self):
        return self_at(idx)[0] if with_self else 0.0

    def descend(idx, with_self):
        """Coordinate descent from one whole-hand pose: one finger at a time
        to the grid value that lowers the score, until none does."""
        idx = list(idx)
        cur = [caps[i, idx[i]] for i in range(n)]
        s, c, *_ = score(cur, _start_centre(cur, R), R, hand, own_m(idx, with_self))
        changed = True
        while changed:
            changed = False
            for i in range(n):
                for k in range(len(grid)):
                    if k == idx[i]:
                        continue
                    trial = list(cur)
                    trial[i] = caps[i, k]
                    s2, c2, *_ = score(trial, c, R, hand, own_m(idx[:i] + [k] + idx[i + 1:], with_self))
                    if s2 < s - 1e-12:
                        s, c, idx[i], cur, changed = s2, c2, k, trial, True
        return s, c, idx, cur

    def search(starts, with_self):
        """The best of the descents. The sphere fit inside a descent carries
        on from the last accepted centre, so a descent's own score is not
        quite the score of its final pose: a descent that beats the best so
        far is re-scored from its final centre, and that is what is kept
        (and what the leader records). With the term for the hand against
        itself the descent crosses poses no sphere fits well, from which the
        fit can wander, so there the pose is also re-scored from the start
        centre and the lower is kept; without the term the one re-score
        stands, so the superseded leaders are reproduced exactly."""
        best = None
        for name, idx0 in starts:
            s, c, idx, cur = descend(idx0, with_self)
            if best is None or s < best["score"]:
                fits = [score(cur, c, R, hand, own_m(idx, with_self))]
                if with_self:
                    fits.append(score(cur, _start_centre(cur, R), R, hand, own_m(idx, with_self)))
                s, c, miss, face, X, U, pen, ov = min(fits, key=lambda f: f[0])
                best = dict(score=s, c=c, idx=idx, caps=cur, start=name, miss=miss, face=face, X=X, U=U,
                            palm=pen, other=ov)
        return best

    uniform = [(float(grid[k0]), [k0] * n) for k0 in range(len(grid))]
    starts = list(uniform)
    if self_term:
        # the hand against itself makes the score rugged: from a uniform curl
        # the only single-finger moves that part two crossed fingers also
        # take that finger off the ball, so the descent can settle on a pose
        # with no overlap and no ball. One more start: the pose the score
        # without the term chooses (the superseded leader's pose).
        starts.append(("without the term", search(uniform, False)["idx"]))
    best = search(starts, self_term)
    curl = [float(grid[j]) for j in best["idx"]]
    best.update(curl=curl, q=pose(hand, curl), R=R, self=self_at(best["idx"]))
    c, U, X = best["c"], best["U"], best["X"]
    r = np.array([cp[2] for cp in best["caps"]])
    normal = U                                            # into the object
    contact = c - R * U                                   # on the sphere
    pad = X + r[:, None] * U                              # on the finger
    F = force_N * U.sum(0)                                # net force on the object
    mass = float(np.linalg.norm(F) / G)
    g_dir = (F / np.linalg.norm(F)).tolist() if mass > 0 else None
    frames = []
    for k in range(n_frames):
        frames.append({"t": round(k / max(n_frames - 1, 1), 6),
                       "obj_pos": c.tolist(), "obj_quat": [1.0, 0.0, 0.0, 0.0],
                       "station_pos": contact.tolist(),
                       "contact": [True] * n,
                       "force_on_object_N": (force_N * normal).tolist(),
                       "contact_point": contact.tolist(),
                       "contact_normal": normal.tolist()})
    lid = f"hold_{key}__self" + (SELF_SUFFIX if self_term else "")
    leader = {
        "schema": SCHEMA, "id": lid,
        "name": f"{hand.spec['name']}: static hold of a {1e3 * R:.1f} mm sphere on its distal pads "
                f"({force_N:g} N per finger)" + (", curls scored against the hand's own overlap" if self_term else ""),
        "kind": "self-generated static hold (the hand's own forward kinematics)",
        "source": {"script": "tools/make_hold_leader.py", "commit": _git(), "hand": key,
                   "model": os.path.relpath(model_path(key), ROOT).replace("\\", "/"),
                   "curl_fraction": {f.label: best["curl"][i] for i, f in enumerate(hand.fingers)},
                   "curl_grid": [float(t) for t in grid],
                   "curl_start": best["start"] if isinstance(best["start"], float) else None,
                   "force_per_station_N": force_N,
                   "pose_qpos": {f.label: best["q"][f.qadr].tolist() for f in hand.fingers},
                   "contact_capsule_local": {
                       f.label: {"body": hand.model.body(f.tip_body).name,
                                 "axis_m": [best["caps"][i][4][0].tolist(), best["caps"][i][4][1].tolist()],
                                 "radius_m": float(r[i]), "palm_side": best["caps"][i][4][2].tolist()}
                       for i, f in enumerate(hand.fingers)},
                   "pad_point_world_m": {f.label: pad[i].tolist() for i, f in enumerate(hand.fingers)},
                   "sphere_fit": {"method": "Gauss-Newton on |c - axis_i| = R + r_i over the distal "
                                            "capsules, R given",
                                  "miss_m": best["miss"].tolist(), "miss_max_m": float(best["miss"].max()),
                                  "facing_deg": best["face"].tolist(),
                                  "facing_max_deg": float(best["face"].max()),
                                  "palm_penetration_m": float(best["palm"]),
                                  "other_link_overlap_m": float(best["other"]),
                                  "score_m": float(best["score"])}},
        "frame": {"units": "m", "note": "the hand model's own frame; no placement search is needed",
                  "gravity": (f"{G} m/s^2 along {np.round(g_dir, 4).tolist()} (the direction of the "
                              "stations' net force, which the object's weight balances)") if g_dir else
                  "none: the stations' forces sum to zero"},
        "object": {"shape": "sphere", "radius_m": R, "mass_kg": mass},
        "n_stations": n,
        "stations": [f.label for f in hand.fingers],
        "frames": frames,
        "caveat": "SELF-GENERATED: a consistency case for the follow check (the answer is known: "
                  "every frame green at zero placement cost), not a task anyone asked for",
        "notes": [f"{n_frames} identical frames over 1 s: a static hold, no motion",
                  "each station's force is the same inward normal force; the object's mass is "
                  "whatever their sum carries (see frame.gravity)",
                  "the contact miss (source.sphere_fit) is how far each distal link's capsule is "
                  "from touching the sphere at this pose; the check's reach should match it, "
                  "and its facing the stored facing"],
    }
    if self_term:
        sm, who = best["self"]
        leader["supersedes"] = f"hold_{key}__self"
        leader["source"]["sphere_fit"]["self_overlap_m"] = float(sm)
        leader["source"]["sphere_fit"]["self_overlap_pair"] = None if who is None else [who["body1"], who["body2"]]
        leader["source"]["sphere_fit"]["search_start"] = (
            f"uniform curl {best['start']}" if isinstance(best["start"], float) else
            "the pose the score without the term chooses (source.curl_start is null)")
        leader["source"]["sphere_fit"]["self_overlap_measure"] = (
            "deepest pair of the hand's own bodies whose collision geoms penetrate in MuJoCo at "
            "source.pose_qpos, each pair less its depth at the rest pose "
            "(tools/hold_pose_self_overlap.py: pairs_at, net_of_rest); a term of the curl score")
        leader["notes"].append(
            f"supersedes hold_{key}__self, whose curl score had no term for the hand against itself: "
            "each finger was curled against the ball alone, so two fingers could be given the same "
            "space. Here the score also carries the hand's overlap with itself "
            "(source.sphere_fit.self_overlap_m is what is left of it)")
    fixed = {"placement": {"rotation_matrix": np.eye(3).tolist(), "translation_m": c.tolist()},
             "assignment": {f.label: i for i, f in enumerate(hand.fingers)},
             "respace_deg": [0.0] * n, "placement_cost_mm": 0.0,
             "note": "the hand's own frame, finger i at station i: no placement search"}
    return leader, fixed, best


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("hand", choices=list(HANDS))
    ap.add_argument("--curl", type=float, nargs="+", default=list(np.round(np.linspace(0, 1, 21), 2)),
                    help="grid of curl fractions each finger may take (default 0, 0.05, ..., 1)")
    ap.add_argument("--radius", type=float, default=RADIUS_M, help="the ball's radius, m (default 0.03)")
    ap.add_argument("--force", type=float, default=0.1, help="inward normal force per station, N")
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "data", "leaders"),
                    help="where the leader goes (default data/leaders)")
    ap.add_argument("--results-dir", default=os.path.join(ROOT, "outputs"),
                    help="where the fixed placement and --run results go (default outputs)")
    ap.add_argument("--run", action="store_true", help="run the follow check with the fixed placement")
    ap.add_argument("--search", action="store_true",
                    help="with --run: also let the check search for the placement (coarse budget; slow)")
    ap.add_argument("--no-self-term", action="store_true",
                    help="leave the hand's overlap with itself out of the curl score: reproduces the "
                         "superseded hold_<hand>__self leader and hold_placement_<hand>.json")
    ap.add_argument("--overwrite", action="store_true")
    a = ap.parse_args()

    leader, fixed, best = make(a.hand, a.curl, a.force, a.frames, a.radius, self_term=not a.no_self_term)
    os.makedirs(a.out_dir, exist_ok=True)
    os.makedirs(a.results_dir, exist_ok=True)
    p_lead = os.path.join(a.out_dir, leader["id"] + ".json")
    p_fix = os.path.join(a.results_dir, f"hold_placement_{a.hand}{'' if a.no_self_term else SELF_SUFFIX}.json")
    for p in (p_lead, p_fix):
        if os.path.exists(p) and not a.overwrite:
            raise SystemExit(f"{p} exists; pass --overwrite or another --out-dir/--results-dir")
    with open(p_lead, "w", encoding="utf-8") as fh:
        json.dump(leader, fh, indent=1)
    with open(p_fix, "w", encoding="utf-8") as fh:
        json.dump(fixed, fh, indent=1)
    print(f"{a.hand}: curl {best['curl']}, sphere R = {1e3 * best['R']:.2f} mm at "
          f"{np.round(1e3 * best['c'], 2).tolist()} mm, contact miss per finger "
          f"{np.round(1e3 * best['miss'], 3).tolist()} mm, facing "
          f"{np.round(best['face'], 1).tolist()} deg, palm penetration {1e3 * best['palm']:.2f} mm, "
          f"other-link overlap {1e3 * best['other']:.2f} mm, "
          + ("" if a.no_self_term else
             f"overlap with itself {1e3 * best['self'][0]:.2f} mm"
             + (f" ({best['self'][1]['body1']} / {best['self'][1]['body2']})" if best['self'][1] else "") + ", ")
          + f"object mass {1e3 * leader['object']['mass_kg']:.2f} g")
    print(f"wrote {os.path.relpath(p_lead, ROOT)} and {os.path.relpath(p_fix, ROOT)}")
    if not a.run:
        return
    runs = [("fixed", {"placement": fixed, "margin": True})]
    if a.search:
        runs.append(("search", {"place": PLACE_COARSE}))
    for tag, kw in runs:
        t0 = time.time()
        r = follow(a.hand, leader, n_eval=a.frames, **kw)
        r["elapsed_s"] = round(time.time() - t0, 1)
        out = os.path.join(a.results_dir, f"follow_{a.hand}__{leader['id']}__{tag}.json")
        with open(out, "w", encoding="utf-8") as fh:
            json.dump(r, fh, indent=1, default=float)
        verd = [fr["verdict"] for fr in r["frames"]]
        why = sorted({fr["reason"] for fr in r["frames"] if fr["reason"]})
        fin = {(x["finger"], x["state"]) for fr in r["frames"] for x in fr["fingers"]}
        marg = [fr.get("scale_margin") for fr in r["frames"] if "scale_margin" in fr]
        print(f"[{tag}] {a.hand}: verdicts {verd}; reasons {why}; placement cost "
              f"{r['placement_cost_mm']:.2f} mm; assignment {r['assignment']}; "
              f"{r['elapsed_s']} s" + (f"; scale margins {marg}" if marg else ""))
        print(f"   per finger: {sorted(fin)}")
        for fr in r["frames"][:1]:
            for x in fr["fingers"]:
                print("   ", {k: (round(v, 3) if isinstance(v, float) else v) for k, v in x.items()})
        print(f"   wrote {os.path.relpath(out, ROOT)}")


if __name__ == "__main__":
    main()
