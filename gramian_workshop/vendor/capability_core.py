"""VENDORED VERBATIM -- do not edit by hand; regenerate with
tools/vendor_w20.py.

Source: geojenks/Wave_manip a3h_sandbox/w20_capability.py
Source commit (last to touch the file): 217746b6812c9bd14409a7bb6dfea1ec3eeabf01
Source sha256: 3d8c9f047b5c87058fa853293c7ddbffae578f773454f39967c8d832e3fa0875

Only the named top-level definitions are copied, by source text.
"""
import math

import numpy as np

from . import stability_core as stability


CONE_EDGES = 8


OUTER_FACTOR = 1.0 / math.cos(math.pi / CONE_EDGES)


KAPPA_OLD = 0.7


FEAS_REL = 1e-9


RANK_REL = 1e-6


BISECT_TOL = 1e-4


SCALE_STEP = 0.25


SCALE_MAX = 16.0


SCALE_GRID = np.arange(0.0, SCALE_MAX + SCALE_STEP / 2, SCALE_STEP)


NNLS_ITER_MULT = 10


NNLS_RECHECK_MULT = 100


REASONS = ["no_contact", "direction", "friction", "cone_edge", "torque",
           "met"]


R_NOCON, R_DIR, R_FRIC, R_EDGE, R_TORQ, R_MET = range(6)


def nnls_iter(n, mult=NNLS_ITER_MULT):
    return int(mult * (3 * n + 10))


def rank(M, rel=RANK_REL):
    if M.size == 0:
        return 0
    s = np.linalg.svd(M, compute_uv=False)
    if s.size == 0 or s[0] == 0.0:
        return 0
    return int(np.sum(s > rel * s[0]))


def scale_wrench(w, L):
    w = np.asarray(w, float).copy()
    w[3:] /= L
    return w


def direction_basis(points, normals, com, L):
    """6 x 3n: for each contact the wrenches of n, t1, t2 (torque / L)."""
    cols = []
    com = np.asarray(com, float)
    for p, n in zip(points, normals):
        n = np.asarray(n, float)
        t1, t2 = stability._tangent_basis(n)
        r = np.asarray(p, float) - com
        for d in (n, t1, t2):
            cols.append(np.concatenate([d, np.cross(r, d) / L]))
    return np.array(cols).T if cols else np.zeros((6, 0))


def generators(points, normals, com, mu, L, edges=CONE_EDGES):
    """Scaled 6xG cone generators (stability.cone_generators, torque / L)
    and, per column, its contact index and its force (3,)."""
    G = stability.cone_generators(list(points), list(normals), com, mu,
                                  edges)
    per = 1 if (mu <= 0.0 or edges < 2) else edges
    cidx = np.repeat(np.arange(len(points)), per)
    F = G[:3].copy()
    Gs = G.copy()
    Gs[3:] /= L
    return Gs, cidx, F


def torque_matrix(cidx, F, J, B):
    """A (nact x G): actuator force per unit generator weight.
    J: (ncon, 3, ndof) contact-point Jacobians on the hand dofs."""
    tau = np.zeros((J.shape[2], F.shape[1]))
    for g in range(F.shape[1]):
        tau[:, g] = J[cidx[g]].T @ F[:, g]
    return B @ tau


def solve(A, b, mult=NNLS_ITER_MULT):
    x = stability.nnls(A, b, max_iter=nnls_iter(A.shape[1], mult))
    return x, float(np.linalg.norm(A @ x - b))


def direction_test(D, ws, rows=None):
    r = slice(None) if rows is None else rows
    if D.shape[1] == 0:
        return False, float(np.linalg.norm(ws[r]))
    x, *_ = np.linalg.lstsq(D[r], ws[r], rcond=None)
    res = float(np.linalg.norm(D[r] @ x - ws[r]))
    return res <= FEAS_REL * float(np.linalg.norm(ws[r])), res


def friction_test(Gs, ws, rows=None, recheck=None):
    """NNLS on the scaled generators.  Returns (ok, resid, lam, flipped):
    `flipped` is True when an infeasible verdict turns feasible under the
    larger recheck cap (and the rechecked answer is returned)."""
    r = slice(None) if rows is None else rows
    A, b = Gs[r], ws[r]
    bar = FEAS_REL * float(np.linalg.norm(b))
    x, res = solve(A, b)
    ok = res <= bar
    flipped = False
    if not ok and recheck is not None:
        x2, res2 = solve(A, b, recheck)
        if res2 <= bar:
            ok, x, res, flipped = True, x2, res2, True
    return ok, res, x, flipped


def torque_system(Gs, As, u0s, ws, rows=None):
    """Stacked NNLS system for the torque test (all rows scaled):
    As = A / u_max (row-wise), u0s = u0 / u_max."""
    r = slice(None) if rows is None else rows
    Gr = Gs[r]
    na, ng = As.shape
    I = np.eye(na)
    Z = np.zeros((na, na))
    M = np.block([[Gr, np.zeros((Gr.shape[0], 2 * na))],
                  [As, I, Z],
                  [-As, Z, I]])
    rhs = np.concatenate([ws[r], 1.0 - u0s, 1.0 + u0s])
    return M, rhs


def torque_test(Gs, As, u0s, ws, rows=None, recheck=None):
    M, rhs = torque_system(Gs, As, u0s, ws, rows)
    r = slice(None) if rows is None else rows
    # the bar is FEAS_REL x the norm of the scaled DEMAND (not of the
    # stacked right-hand side, whose limit rows would loosen it).  Only for
    # a zero demand (the s = 0 point of the scale grid: can the joints hold
    # the bias torque alone?) is there no demand scale; there the stacked
    # right-hand side is the only scale left.
    dn = float(np.linalg.norm(ws[r]))
    bar = FEAS_REL * (dn if dn > 0 else float(np.linalg.norm(rhs)))
    x, res = solve(M, rhs)
    ok = res <= bar
    flipped = False
    if not ok and recheck is not None:
        x2, res2 = solve(M, rhs, recheck)
        if res2 <= bar:
            ok, x, res, flipped = True, x2, res2, True
    lam = x[:Gs.shape[1]]
    return ok, res, lam, flipped


def scale_margin(Gs, As, u0s, ws, rows=None):
    """Largest s with s * demand feasible (torque-limited), by the full grid
    then bisection to BISECT_TOL between the largest feasible grid point and
    the next.  Returns dict: grid (bool per SCALE_GRID), margin, capped,
    contiguous (feasible grid points form one run), lo (smallest feasible
    grid point, nan if none)."""
    grid = np.zeros(len(SCALE_GRID), bool)
    for i, s in enumerate(SCALE_GRID):
        grid[i] = torque_test(Gs, As, u0s, s * ws, rows)[0]
    idx = np.where(grid)[0]
    out = {"grid": grid, "capped": False, "contiguous": True,
           "lo": float("nan"), "margin": float("nan")}
    if len(idx) == 0:
        out["margin"] = float("nan")       # no feasible grid point
        return out
    out["contiguous"] = bool(idx[-1] - idx[0] + 1 == len(idx))
    out["lo"] = float(SCALE_GRID[idx[0]])
    top = idx[-1]
    if top == len(SCALE_GRID) - 1:
        out["capped"] = True
        out["margin"] = float(SCALE_GRID[top])
        return out
    a, b = float(SCALE_GRID[top]), float(SCALE_GRID[top + 1])
    while b - a > BISECT_TOL:
        c = 0.5 * (a + b)
        if torque_test(Gs, As, u0s, c * ws, rows)[0]:
            a = c
        else:
            b = c
    out["margin"] = a
    return out


def skew(r):
    return np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]],
                     [-r[1], r[0], 0.0]])


def input_count(points, com, J, L):
    """dim(range(G^T) intersect range(J_hand)), SVD tol RANK_REL.
    G^T (3n x 6): object twist (v, L*omega) -> contact point velocity,
    rows [I, -skew(r)/L] per contact (the transpose of the scaled grasp
    map).  J (n, 3, ndof): hand point Jacobians, stacked to 3n x ndof."""
    n = len(points)
    if n == 0:
        return 0
    com = np.asarray(com, float)
    Gt = np.vstack([np.hstack([np.eye(3), -skew(np.asarray(p) - com) / L])
                    for p in points])
    Jh = np.vstack([J[i] for i in range(n)])
    return rank(Gt) + rank(Jh) - rank(np.hstack([Gt, Jh]))
