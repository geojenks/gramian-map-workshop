"""The whole-motion check (follow(trajectory=True)) over the dense cases.

    python tools/trajectory_sweep.py [workers]   # default 5; needs the [trajectory] extra

One stored placement per robot hand on the 10-step cup wave (every frame)
and the four human hands on the tennis-ball demonstration (n_eval=300),
with lags of 0.03, 0.1 and 0.3 s and rate limits of 1 and 3 per second
(illustrative, unsourced). follow's own margins are off. Last, per-frame
trajectory margins on tennis at lag 0.1 s (slow). Writes one json per job
to outputs/trajectory/, skipping jobs already there; summarise with
tools/trajectory_summary.py.
"""
import json
import os
import sys
import time
from multiprocessing import Pool

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "outputs", "trajectory")
TENNIS = "tennis_new_cup_take3_handframe"
W10 = "w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5"
SETTINGS = [(f"lag{v}", None, v) for v in (0.03, 0.1, 0.3)] + [(f"rate{v}", v, None) for v in (1.0, 3.0)]


def out_path(name):
    return os.path.join(ROOT, "outputs", name)


def human_tennis():
    return [out_path(f"follow_{h}__{TENNIS}.json") for h in ("george_clip", "george_kp", "george_kp_wide")] + \
           [out_path("follow_george_measured_thumb__tennis_new_cup_take3_derived__tan.json")]


def dense_files():
    return [out_path(f"follow_{h}__{W10}_n{n}.json") for h, n in
            (("allegro", 4), ("leap", 4), ("l20", 5), ("shadow", 5))] + human_tennis()


def name_of(fn, label, per_frame):
    base = os.path.splitext(os.path.basename(fn))[0]
    return f"traj__{base}__{label}{'__per_frame' if per_frame else ''}.json"


def one(job):
    sys.path.insert(0, ROOT)
    from gramian_workshop import follow as F
    fn, label, per_frame = job
    t0 = time.time()
    old = json.load(open(fn, encoding="utf-8"))
    s = old["settings"]
    L = json.load(open(os.path.join(ROOT, "data", "leaders", old["leader"] + ".json"), encoding="utf-8"))
    _, rate, lag = next(x for x in SETTINGS if x[0] == label)
    tennis = "tennis" in old["leader"]
    if per_frame:
        from gramian_workshop import trajectory as T
        _check = T.check
        T.check = lambda recs, *a, **k: _check(recs, *a, **{**k, "per_frame": True})
    r = json.loads(json.dumps(F.follow(
        old["hand"], L, middle=s["middle"], ik_iters=s.get("ik_iters"), wrist=bool(s.get("wrist")),
        respace=bool(s.get("respace")), joint_stiffness=s.get("joint_stiffness_Nm_per_rad"),
        placement=old, n_eval=300 if tennis else None, margin=False, disturbance=False,
        rate_limit=rate, lag_s=lag, trajectory=True)))
    res = {"file": os.path.basename(fn), "hand": old["hand"], "leader": old["leader"],
           "setting": label, "rate_limit_per_s": rate, "lag_s": lag, "per_frame": per_frame,
           "n_eval": 300 if tennis else None,
           "frame_k": [fr["k"] for fr in r["frames"]],
           "frame_t": [float(L["frames"][fr["k"]]["t"]) for fr in r["frames"]],
           "trajectory": r["trajectory"], "wall_s": time.time() - t0}
    json.dump(res, open(os.path.join(OUT, name_of(fn, label, per_frame)), "w", encoding="utf-8"), indent=1)
    return name_of(fn, label, per_frame), round(res["wall_s"])


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    jobs = [(f, s[0], False) for f in human_tennis() for s in SETTINGS] + \
           [(f, s[0], False) for f in dense_files()[:4] for s in SETTINGS] + \
           [(f, "lag0.1", True) for f in human_tennis()]
    jobs = [j for j in jobs if not os.path.exists(os.path.join(OUT, name_of(*j)))]
    print("jobs", len(jobs), flush=True)
    with Pool(int(sys.argv[1]) if len(sys.argv) > 1 else 5, maxtasksperchild=1) as p:
        for r in p.imap_unordered(one, jobs):
            print("done", *r, time.strftime("%H:%M:%S"), flush=True)
    print("[complete]", flush=True)
