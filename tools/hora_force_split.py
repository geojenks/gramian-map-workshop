"""The force each fingertip puts on the object in the Hora log, split into normal and shear.

Isaac Gym's friction fields read zero in this log, and its net contact force lies along
the contact normal, so friction has to be recovered. The normal force at each contact is
logged. The friction the object needs is what its motion asks for, less what the normal
forces already supply:
    needed wrench = (m(a - g), I alpha + w x I w) - sum over contacts (lam n, r x lam n)
That wrench is shared between the fingers by least squares, with each finger's shear held
inside its friction limit (mu lam) and weakly guided by the shear its joint torques imply.
Six equations and eight unknowns, so the split is one consistent split and not the only
one; the total is what the motion fixes.

    python tools/hora_force_split.py [--physics data/rl_logs/hora_seed42_30s_physics.npz]

Writes <physics stem>_forces.npz, one row per physics step, fingers in FINGERS order. Needs scipy.
"""
import argparse, os
import numpy as np
from scipy.optimize import lsq_linear

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FINGERS = [("index", "link_3"), ("middle", "link_7"), ("ring", "link_11"), ("thumb", "link_15")]
ARMATURE = 0.001       # in the hand's asset, absent from the logged mass matrix
GUIDE = 0.03           # weight of the joint-torque guide against the wrench rows
TORQUE_LENGTH = 0.03   # m, scales torque rows to match force rows
SMALL_LAM = 1e-4       # N, contacts below this are ignored


def rot(q, v):
    return v + 2 * np.cross(q[..., :3], np.cross(q[..., :3], v) + q[..., 3:4] * v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--physics", default=os.path.join(ROOT, "data", "rl_logs", "hora_seed42_30s_physics.npz"))
    a = ap.parse_args()
    z = np.load(a.physics, allow_pickle=True)
    dt, m, com, I0 = float(z["dt"]), float(z["obj_mass"]), z["obj_com"], z["obj_I"]
    C, obj, B, body, jac, mm, tau, qd, step = (z[k] for k in ("contacts", "obj", "obj_body", "body", "jac", "mm", "tau", "qd", "step"))
    B = int(B); mu = float(z["obj_fric"][0]); N = len(obj); g = np.array([0, 0, -9.81])
    names = [str(n).split(".")[0] for n in z["body_names"]]
    tip = {names.index(l): j for j, (_, l) in enumerate(FINGERS)}
    p, q, v, w = obj[:, :3], obj[:, 3:7], obj[:, 7:10], obj[:, 10:13]
    pc = p + rot(q, com)
    # Per step and finger: normal force, its direction, and the force-weighted contact point.
    lam = np.zeros((N, 4)); nrm = np.zeros((N, 4, 3)); pnt = np.zeros((N, 4, 3)); other = 0
    for c in C:
        k, b0, b1 = int(c[0]), int(c[1]), int(c[2])
        lo, o, n = (c[7:10], b1, c[4:7]) if b0 == B else (c[10:13], b0, -c[4:7])
        if c[3] <= SMALL_LAM or o < 0: continue
        if o not in tip: other += 1; continue
        j = tip[o]; lam[k, j] += c[3]; nrm[k, j] += c[3] * n; pnt[k, j] += c[3] * (p[k] + rot(q[k], lo))
    on = lam > 0
    nrm[on] /= np.linalg.norm(nrm[on], axis=1, keepdims=True); pnt[on] /= lam[on][:, None]
    print("contacts on links other than the four tips:", other)
    dofs = {o: np.nonzero(np.abs(jac[N // 2, o - 1, :6]).sum(0) > 1e-6)[0] for o in tip}
    qdd = np.zeros_like(qd); qdd[1:] = (qd[1:] - qd[:-1]) / dt
    shear = np.zeros((N, 4, 3)); guide = np.zeros((N, 4, 3)); need = np.full((N, 6), np.nan); miss = np.full((N, 6), np.nan)
    for k in range(13, N):        # the first steps come before the log's first differences settle
        if not on[k].any(): continue
        acc = (v[k] - v[k - 1]) / dt; al = (w[k] - w[k - 1]) / dt
        R = np.stack([rot(q[k], np.eye(3)[i]) for i in range(3)], -1); Iw = R @ I0 @ R.T
        b = np.r_[m * (acc - g), Iw @ al + np.cross(w[k], Iw @ w[k])]
        js = np.nonzero(on[k])[0]; W = np.zeros((6, 2 * len(js))); pr = np.zeros(2 * len(js)); lim = np.zeros(2 * len(js)); axes = []
        for i, j in enumerate(js):
            o = [t for t, jj in tip.items() if jj == j][0]; n, x, l = nrm[k, j], pnt[k, j], lam[k, j]; r = x - pc[k]
            b -= np.r_[l * n, np.cross(r, l * n)]
            u = np.cross(n, [0, 0, 1.]); u = u / np.linalg.norm(u) if np.linalg.norm(u) > 1e-6 else np.array([1., 0, 0]); t = np.cross(n, u)
            axes.append((u, t)); W[:3, 2 * i] = u; W[:3, 2 * i + 1] = t; W[3:, 2 * i] = np.cross(r, u); W[3:, 2 * i + 1] = np.cross(r, t)
            d = dofs[o]; J = jac[k, o - 1, :3][:, d] + np.cross(jac[k, o - 1, 3:6][:, d].T, x - body[k, o, :3]).T
            F = -np.linalg.lstsq(J.T, mm[k][np.ix_(d, d)] @ qdd[k][d] + ARMATURE * qdd[k][d] - tau[k][d], rcond=None)[0]
            guide[k, j] = F - (F @ n) * n; pr[2 * i], pr[2 * i + 1] = F @ u, F @ t; lim[2 * i:2 * i + 2] = mu * l
        s = np.r_[np.ones(3), np.ones(3) / TORQUE_LENGTH]
        f = lsq_linear(np.vstack([W * s[:, None], GUIDE * np.eye(2 * len(js))]), np.r_[b * s, GUIDE * np.clip(pr, -lim, lim)], bounds=(-lim, lim)).x
        need[k] = b; miss[k] = W @ f - b
        for i, j in enumerate(js):
            shear[k, j] = f[2 * i] * axes[i][0] + f[2 * i + 1] * axes[i][1]
    ok = np.isfinite(need[:, 0]); mag = np.linalg.norm(shear, axis=2); sel = on & ok[:, None]
    print(f"friction force needed: median {np.median(np.linalg.norm(need[ok, :3], axis=1)):.3f} N, split misses it by median {np.median(np.linalg.norm(miss[ok, :3], axis=1)):.5f} N")
    print(f"friction torque needed: median {np.median(np.linalg.norm(need[ok, 3:], axis=1)) * 1e3:.2f} mN m, split misses it by median {np.median(np.linalg.norm(miss[ok, 3:], axis=1)) * 1e3:.3f} mN m")
    print(f"normal median {np.median(lam[sel]):.3f} N; shear median {np.median(mag[sel]):.3f} N, p95 {np.percentile(mag[sel], 95):.3f} N; share of friction limit median {np.median(mag[sel] / (mu * lam[sel])):.2f}")
    for j, (nm, _) in enumerate(FINGERS):
        s_ = sel[:, j]; print(f"  {nm}: in contact {s_.mean():.2f} of steps, normal median {np.median(lam[s_, j]):.3f} N, shear median {np.median(mag[s_, j]):.3f} N")
    out = a.physics.replace("_physics.npz", "_forces.npz")
    np.savez_compressed(out, t=np.arange(N) * dt, normal_N=lam, normal_dir=nrm, point=pnt, shear_N=shear, joint_torque_shear_N=guide, need=need, miss=miss,
                        mu=mu, obj_pos=p, obj_quat=q, obj_angvel=w, step=step, fingers=[f for f, _ in FINGERS], body=body, body_names=names)
    print("wrote", out)


if __name__ == "__main__":
    main()
