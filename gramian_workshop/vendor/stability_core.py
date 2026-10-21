"""VENDORED VERBATIM -- do not edit by hand; regenerate with
tools/vendor_w20.py.

Source: geojenks/Wave_manip unified_v6/bridge/stability.py
Source commit (last to touch the file): 217746b6812c9bd14409a7bb6dfea1ec3eeabf01
Source sha256: be483b1b3cd671fed5ea13441c73b5bcfadfecec861fe1f123de2d0c4124d0d4

Only the named top-level definitions are copied, by source text.
"""
import math

import numpy as np


def nnls(A, b, tol=1e-12, max_iter=None):
    """Lawson-Hanson nonnegative least squares: argmin_{x>=0} ||A x - b||.

    Small, exact, numpy-only. Used to test cone membership (residual ~ 0 =>
    the target wrench is reachable) and to read off the min-force solution.
    """
    A = np.asarray(A, float)
    b = np.asarray(b, float)
    m, n = A.shape
    if max_iter is None:
        max_iter = 3 * n + 10
    x = np.zeros(n)
    P = np.zeros(n, bool)          # passive (positive) set
    w = A.T @ (b - A @ x)
    it = 0
    while (~P).any() and w[~P].max(initial=-np.inf) > tol and it < max_iter:
        it += 1
        # bring the most-violated coordinate into the passive set
        cand = np.where(~P)[0]
        j = cand[np.argmax(w[cand])]
        P[j] = True
        while True:
            it += 1
            if it > max_iter:
                break
            idx = np.where(P)[0]
            z_p, *_ = np.linalg.lstsq(A[:, idx], b, rcond=None)
            if (z_p > 0).all():
                x[idx] = z_p
                break
            # some passive coord went non-positive: step to the boundary
            zp_full = np.zeros(n)
            zp_full[idx] = z_p
            neg = idx[z_p <= 0]
            alpha = np.min(x[neg] / (x[neg] - zp_full[neg]))
            x = x + alpha * (zp_full - x)
            P[(x <= tol) & P] = False
            x[~P] = 0.0
        w = A.T @ (b - A @ x)
    return x


def _tangent_basis(n):
    """Two orthonormal vectors spanning the plane perpendicular to unit n."""
    a = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    t1 = a - n * (a @ n)
    t1 /= np.linalg.norm(t1)
    t2 = np.cross(n, t1)
    return t1, t2


def cone_generators(points, normals, com, mu, edges):
    """6xG matrix of wrench generators for the linearized friction cones.

    Column = a cone-edge force applied at a contact, stacked as
    (force, torque-about-com). Each contact contributes `edges` columns (1 if
    mu==0). Non-negative combinations reachable with total normal force
    sum(coeff) -- so coeff sums read as the grip-force budget.
    """
    cols = []
    com = np.asarray(com, float)
    for p, n in zip(points, normals):
        p = np.asarray(p, float)
        n = np.asarray(n, float)
        r = p - com
        if mu <= 0.0 or edges < 2:
            f = n
            cols.append(np.concatenate([f, np.cross(r, f)]))
        else:
            t1, t2 = _tangent_basis(n)
            for k in range(edges):
                ang = 2.0 * math.pi * k / edges
                f = n + mu * (math.cos(ang) * t1 + math.sin(ang) * t2)
                cols.append(np.concatenate([f, np.cross(r, f)]))
    return np.array(cols).T if cols else np.zeros((6, 0))
