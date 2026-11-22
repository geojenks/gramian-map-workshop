"""Slip check on the MuJoCo logs of pneu-net fingers turning a ball (20 Nov 2026).

SIMULATION ONLY, comparative. Reads the logs written by
tools/pneunet_palm_turn_ball_mujoco_20nov.py; runs no simulation.
For each ramp of an input it compares how far the ball turned with how far
the tips swept round the axis while touching (mean over all fingers).
Writes outputs/pneunet_palm_turn_ball_20nov/mujoco/mujoco_slip_check.txt
Run from the repository root: python tools/pneunet_palm_turn_ball_mujoco_slip_check_20nov.py
"""
import glob, os
import numpy as np
OUT = os.path.join("outputs", "pneunet_palm_turn_ball_20nov", "mujoco")
lines = ["Slip check on the MuJoCo logs. SIMULATION ONLY, comparative.",
         "Per ramp: ball turn, mean tip sweep round the axis while touching, "
         "fingers touching (fewest to most).", ""]
for f in sorted(glob.glob(os.path.join(OUT, "log_*.csv"))):
    a = np.loadtxt(f, delimiter=",", skiprows=1)
    u = a[:, 1:3]
    moving = np.any(np.abs(np.diff(u, axis=0)) > 0, axis=1)
    edges = np.flatnonzero(np.diff(np.r_[0, moving.astype(int), 0]))
    lines.append(os.path.basename(f))
    for s, e in zip(edges[0::2], edges[1::2]):
        which = "+".join(str(k) for k in range(2) if u[e, k] != u[s, k])
        sense = "out" if u[e].sum() > u[s].sum() else "back"
        lines.append(f"  {a[s,0]:6.1f} to {a[e,0]:6.1f} s, input set {which} {sense}: "
                     f"ball {a[e,3]-a[s,3]:+7.1f} deg, tips while touching "
                     f"{a[e,9]-a[s,9]:+7.1f} deg, touching "
                     f"{int(a[s:e+1,7].min())} to {int(a[s:e+1,7].max())}")
    lines.append("")
open(os.path.join(OUT, "mujoco_slip_check.txt"), "w", encoding="utf-8").write("\n".join(lines))
print("\n".join(lines))
