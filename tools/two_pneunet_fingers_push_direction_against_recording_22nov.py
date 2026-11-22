"""Push direction and size: each simulated two-pneu-net tip against the recorded finger (22 Nov).

Reads logs of runs already made; nothing is simulated. SIMULATION OUTPUT against the recording;
comparative, not predictive.

What each side holds
  Recording (data/rl_logs/hora_seed42_30s_forces.npz): per finger, the logged normal force and its
  direction, and a RECOVERED friction force (tools/hora_force_split.py: the simulator's friction
  fields read zero, so friction is what the cylinder's motion needs beyond the normal forces,
  shared between fingers by least squares; one consistent split, not the only one).
  Simulated run (log_<key>.csv): per tip, the size of the normal force and the size of the friction
  force, the ball's centre and the cylinder's pose. The friction force's DIRECTION WAS NOT LOGGED.
  The normal push direction is rebuilt here: from the ball's centre to the nearest point of the
  cylinder (on the side this is straight towards the axis).

Measures, fixed here before the first run of this script, over the gait's samples at which the
simulated tip presses and the recorded finger presses (both above the touch threshold) at the same
log time:
  1. angle between the simulated normal push and the recorded normal push;
  2. angle between the simulated normal push and the recorded whole force (normal + recovered friction);
  3. simulated normal force over recorded normal force;
  4. simulated whole force size (normal and friction sizes combined) over recorded whole force size;
  5. friction over normal, each side.
Each as median with the 10th and 90th percentiles.

Usage: python <this> --wave-manip <path> --run <folder> <key> [--run <folder> <key> ...] --out <report path>
"""
import argparse
import csv
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import pneunet_arrangement_replicates_rl_stroke_20nov as A   # noqa: E402

NAMES = ["index", "middle", "ring", "thumb"]


def rot(q, v):          # q as x, y, z, w
    return v + 2 * np.cross(q[..., :3], np.cross(q[..., :3], v) + q[..., 3:4] * v)


def unit(v):
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-12)


def angle(a, b):
    return np.degrees(np.arccos(np.clip((unit(a) * unit(b)).sum(-1), -1, 1)))


def mps(x):
    return f"median {np.median(x):.2f} (10th to 90th percentile {np.percentile(x, 10):.2f} to {np.percentile(x, 90):.2f})"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", required=True)
    ap.add_argument("--run", nargs=2, action="append", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    for sub in ("a3h_sandbox", os.path.join("experiment_log", "scripts")):
        sys.path.insert(0, os.path.join(a.wave_manip, sub))
    from w16s1_hora_scout import CYL_URDF_HALFLEN_M, CYL_URDF_RADIUS_M
    L = A.load_recording()
    R, half = CYL_URDF_RADIUS_M * L["scale"], CYL_URDF_HALFLEN_M * L["scale"]
    Fz = np.load(os.path.join(A.LOGS, "hora_seed42_30s_forces.npz"), allow_pickle=True)
    shear = Fz["shear_N"].astype(float)
    # recorded normal direction signed to push INTO the cylinder (towards its centre)
    inward = L["op"][:, None, :] - L["P"]
    sgn = np.sign((L["nrm"] * inward).sum(-1, keepdims=True))
    n_rec = L["nrm"] * sgn
    lines = [__doc__.strip().splitlines()[0], "Nothing simulated here; logs read only. Comparative, not predictive.",
             f"Cylinder radius {R * 1e3:.1f} mm, half length {half * 1e3:.1f} mm. Touch threshold {A.TOUCH_N} N.",
             "Recorded whole force = logged normal force + RECOVERED friction (one consistent split, not the only one).",
             "Simulated friction direction was not logged: angles use the simulated NORMAL push only.", ""]
    # checks that the recovered friction lines up with the arrays the recording loader returns
    press = L["lam"] > A.TOUCH_N
    along = np.abs((shear * n_rec).sum(-1))[press] / np.maximum(np.linalg.norm(shear, axis=-1)[press], 1e-12)
    lines += ["Checks on the recording's arrays:",
              f"  rows: loader {len(L['t'])}, force file {len(Fz['t'])}; times equal: {bool(len(L['t']) == len(Fz['t']) and np.allclose(L['t'], Fz['t']))}",
              f"  normal force equal to the file's: {bool(np.allclose(L['lam'], Fz['normal_N']))}; normal direction equal to the file's: {bool(np.allclose(L['nrm'], Fz['normal_dir']))}",
              f"  finger order: loader {[str(x) for x in L['names']]}, file {[str(x) for x in Fz['fingers']]}, used here {NAMES}",
              f"  recovered friction's part along the normal, over its size, where pressing: median {np.median(along):.4f}, largest {along.max():.4f} (0 means it lies in the surface)",
              f"  share of pressing samples whose logged normal direction was flipped to point into the cylinder: {100 * (sgn[..., 0][press] < 0).mean():.1f}%",
              "Angles are taken in the world frame; each side uses its own cylinder pose.", ""]
    for folder, key in a.run:
        rows = list(csv.DictReader(open(os.path.join(folder, f"log_{key}.csv"), encoding="utf-8")))
        rows = [r for r in rows if r["part"] in ("2", "2.0")]
        g = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
        o = np.stack([g["ox"], g["oy"], g["oz"]], 1)
        q = np.stack([g["qx"], g["qy"], g["qz"], g["qw"]], 1)
        ax = rot(q, np.array([0, 0, 1.0]))
        i = np.clip(np.searchsorted(L["t"], g["log_t"]), 1, len(L["t"]) - 1)
        i = np.where(np.abs(L["t"][i - 1] - g["log_t"]) < np.abs(L["t"][i] - g["log_t"]), i - 1, i)
        lines.append(f"Run '{key}' in {folder} ({len(rows)} samples of the gait):")
        for k, name in enumerate(NAMES):
            cx = f"f{k}_tip_x" if f"f{k}_tip_x" in g else f"f{k}_cx"
            c = np.stack([g[cx], g[cx[:-1] + "y"], g[cx[:-1] + "z"]], 1)
            d = c - o
            z = (d * ax).sum(1, keepdims=True)
            rho = d - z * ax
            rn = np.linalg.norm(rho, axis=1, keepdims=True)
            p = o + np.clip(z, -half, half) * ax + np.minimum(rn, R) * unit(rho)
            n_sim = unit(p - c)
            lam = L["lam"][i, k]
            both = (g[f"f{k}_touching"] > 0.5) & (g[f"f{k}_normal_N"] > A.TOUCH_N) & (lam > A.TOUCH_N)
            if both.sum() < 5:
                lines.append(f"  {name}: fewer than 5 samples with both pressing.")
                continue
            ns, ts = g[f"f{k}_normal_N"][both], g[f"f{k}_tangential_N"][both]
            nr, sr = n_rec[i, k][both], shear[i, k][both]
            whole = lam[both, None] * nr + sr
            on_side = (np.abs(z[both, 0]) <= half).mean()
            extra = ""
            if f"f{k}_rest_x" in g:
                rz = np.abs(((np.stack([g[f"f{k}_rest_x"], g[f"f{k}_rest_y"], g[f"f{k}_rest_z"]], 1) - o) * ax).sum(1))
                tch = g[f"f{k}_touching"] > 0.5
                extra = (f" The finger's own rest position is past a flat end in {100 * (rz[both] > half).mean():.0f}% of them"
                         f" ({100 * (rz[tch] > half).mean():.0f}% of all its touching samples).")
            lines += [
                f"  {name}: {int(both.sum())} samples with both pressing; simulated ball beside the cylinder's side (not past a flat end) in {100 * on_side:.0f}% of them." + extra,
                f"    angle, simulated normal push to recorded normal push, deg: {mps(angle(n_sim[both], nr))}",
                f"    angle, simulated normal push to recorded whole force, deg: {mps(angle(n_sim[both], whole))}",
                f"    normal force, simulated over recorded: {mps(ns / lam[both])}",
                f"    whole force size, simulated over recorded: {mps(np.hypot(ns, ts) / np.linalg.norm(whole, axis=1))}",
                f"    friction over normal: simulated {mps(ts / ns)}; recorded {mps(np.linalg.norm(sr, axis=1) / lam[both])}",
                f"    forces, N: simulated normal median {np.median(ns):.3f}, friction {np.median(ts):.3f}; recorded normal {np.median(lam[both]):.3f}, recovered friction {np.median(np.linalg.norm(sr, axis=1)):.3f}"]
        lines.append("")
    lines.append("Not measured: the angle between the simulated and recorded friction forces, and so between the whole forces. "
                 "It needs the runs repeated with the tip's force vector logged.")
    open(a.out, "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    print("Written:", a.out)


main()
