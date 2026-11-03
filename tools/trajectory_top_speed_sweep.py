"""tools/trajectory_top_speed.py over the dense cases of the whole-motion
sweep (tools/trajectory_sweep.py), at its settings, with and without the
fingers' motion torque.

    python tools/trajectory_top_speed_sweep.py [workers]   # default 5; needs the [trajectory] extra

The cases: one stored placement per robot hand on the 10-step cup wave
(every frame) and the four human hands on the tennis-ball demonstration
(300 frames spread over it); lags of 0.03, 0.1 and 0.3 s and rate limits
of 1 and 3 per second, one at a time (illustrative, unsourced). Each job
runs the tool as its own process, so its readout lands in
outputs/top_speed/ and any retimed leader in outputs/retimed_leaders/;
jobs whose readout is already there are skipped. Summarise with
tools/trajectory_top_speed_summary.py.
"""
import os
import subprocess
import sys
import time
from multiprocessing import Pool

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from trajectory_sweep import SETTINGS, dense_files, human_tennis  # noqa: E402
from trajectory_top_speed import OUT, setting_label  # noqa: E402

TOOL = os.path.join(ROOT, "tools", "trajectory_top_speed.py")


def jobs():
    out = []
    for fn in dense_files():
        n_eval = 300 if fn in human_tennis() else None
        for _, rate, lag in SETTINGS:
            for inertia in (False, True):
                out.append((fn, rate, lag, inertia, n_eval))
    return out


def readout_name(fn, rate, lag, inertia):
    base = os.path.splitext(os.path.basename(fn))[0]
    return f"topspeed__{base}__{setting_label(rate, lag)}" + ("__inertia" if inertia else "") + ".json"


def one(job):
    fn, rate, lag, inertia, n_eval = job
    args = [sys.executable, TOOL, fn]
    if rate is not None:
        args += ["--rate", str(rate)]
    if lag is not None:
        args += ["--lag", str(lag)]
    if inertia:
        args.append("--inertia")
    if n_eval is not None:
        args += ["--n-eval", str(n_eval)]
    t0 = time.time()
    p = subprocess.run(args, capture_output=True, text=True, cwd=ROOT)
    name = readout_name(fn, rate, lag, inertia)
    log = os.path.join(OUT, name[:-5] + ".log")
    with open(log, "w", encoding="utf-8") as f:
        f.write(p.stdout + ("\n[stderr]\n" + p.stderr if p.stderr else ""))
    return name, p.returncode, round(time.time() - t0)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    todo = [j for j in jobs() if not os.path.exists(os.path.join(OUT, readout_name(*j[:4])))]
    print("jobs", len(todo), flush=True)
    with Pool(int(sys.argv[1]) if len(sys.argv) > 1 else 5, maxtasksperchild=1) as p:
        for r in p.imap_unordered(one, todo):
            print("done", *r, time.strftime("%H:%M:%S"), flush=True)
    print("[complete]", flush=True)
