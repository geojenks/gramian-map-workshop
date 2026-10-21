"""Can these contacts carry this demand, and if not, why (W20 step 6).

Registration: Wave_manip experiment_log/2026-10-21_w20s6_soft_finger_registration.md
section 5. Every test is the vendored W20 core (vendor/capability_core.py),
called unchanged; this module only builds its inputs.

ideal_hold  any hand touching at these points, unlimited force in its full
            cones: W20 readout (a). Reasons: no_contact, direction, friction,
            cone_edge, met (plus met_mu0: met with normal forces alone).
soft_hold   single-input fingers from records, each pushing only along its
            world push direction b_i. Reasons, first failed: no_contact,
            direction, friction, cone_edge, strength (COMPARATIVE, never
            scored), met.

Simulation and records only; comparative, not predictive.
"""
import math

import numpy as np

from .vendor import capability_core as cap

REASONS = ["no_contact", "direction", "friction", "cone_edge", "strength",
           "met"]
HELD = ("met", "strength")       # friction-feasible; strength is not scored


def _closure(G, lam, ws):
    return float(np.linalg.norm(G @ lam - ws))


def ideal_hold(points, normals, com, demand, mu, L):
    out = {"reason": None, "met_mu0": False, "closure": None}
    if len(points) == 0:
        out["reason"] = "no_contact"
        return out
    ws = cap.scale_wrench(demand, L)
    D = cap.direction_basis(points, normals, com, L)
    ok, _ = cap.direction_test(D, ws)
    if not ok:
        out["reason"] = "direction"
        return out
    G0 = cap.generators(points, normals, com, 0.0, L)[0]
    out["met_mu0"] = bool(cap.friction_test(
        G0, ws, recheck=cap.NNLS_RECHECK_MULT)[0])
    Gi = cap.generators(points, normals, com, mu, L)[0]
    ok_i, _, lam, _ = cap.friction_test(Gi, ws,
                                        recheck=cap.NNLS_RECHECK_MULT)
    if ok_i:
        out["reason"] = "met"
        out["closure"] = _closure(Gi, lam, ws) / float(np.linalg.norm(ws))
        return out
    Go = cap.generators(points, normals, com, mu * cap.OUTER_FACTOR, L)[0]
    ok_o = cap.friction_test(Go, ws, recheck=cap.NNLS_RECHECK_MULT)[0]
    out["reason"] = "cone_edge" if ok_o else "friction"
    return out


def push_columns(points, com, B, L):
    """6 x m scaled wrench of a unit push along each b_i at p_i."""
    com = np.asarray(com, float)
    return np.array([np.concatenate([b, np.cross(np.asarray(p) - com, b) / L])
                     for p, b in zip(points, B)]).T


def soft_hold(points, normals, com, demand, mu, L, B, F, D=None,
              want_margin=False):
    """B: world push direction per contact (None = cannot push); F: the
    comparative blocked force per contact; D: world motion direction per
    contact (for the input count; None entries count as no motion)."""
    n = len(points)
    out = {"reason": None, "input_count": None, "margin": None,
           "closure": None, "edge_fingers": 0, "out_fingers": 0}
    if D is not None and n:
        J = np.zeros((n, 3, n))
        for i, d in enumerate(D):
            if d is not None:
                J[i][:, i] = d
        out["input_count"] = int(cap.input_count(points, com, J, L))
    idx = [i for i in range(n) if B[i] is not None]
    if not idx:
        out["reason"] = "no_contact"
        return out
    P = np.asarray(points, float)[idx]
    Nn = np.asarray(normals, float)[idx]
    Bs = np.asarray([B[i] for i in idx], float)
    Fs = np.asarray([F[i] for i in idx], float)
    ws = cap.scale_wrench(demand, L)
    G = push_columns(P, com, Bs, L)
    ok, _ = cap.direction_test(G, ws)
    if not ok:
        out["reason"] = "direction"
        return out
    c = np.einsum("ij,ij->i", Bs, Nn) / np.linalg.norm(Bs, axis=1)
    cos_in = 1.0 / math.sqrt(1.0 + mu * mu)
    cos_out = 1.0 / math.sqrt(1.0 + (mu * cap.OUTER_FACTOR) ** 2)
    inner = c >= cos_in
    edge = (c >= cos_out) & ~inner
    out["edge_fingers"] = int(edge.sum())
    out["out_fingers"] = int((~inner & ~edge).sum())
    ok_i = False
    if inner.any():
        ok_i, _, lam, _ = cap.friction_test(G[:, inner], ws,
                                            recheck=cap.NNLS_RECHECK_MULT)
    if not ok_i:
        ok_o = False
        if (inner | edge).any():
            ok_o = cap.friction_test(G[:, inner | edge], ws,
                                     recheck=cap.NNLS_RECHECK_MULT)[0]
        out["reason"] = "cone_edge" if ok_o else "friction"
        return out
    out["closure"] = _closure(G[:, inner], lam, ws) / float(np.linalg.norm(ws))
    Gi = G[:, inner]
    As = np.diag(1.0 / Fs[inner])
    u0s = np.zeros(int(inner.sum()))
    ok_s = cap.torque_test(Gi, As, u0s, ws,
                           recheck=cap.NNLS_RECHECK_MULT)[0]
    out["reason"] = "met" if ok_s else "strength"
    if want_margin:
        out["margin"] = cap.scale_margin(Gi, As, u0s, ws)["margin"]
    return out
