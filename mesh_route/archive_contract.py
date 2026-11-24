"""Pure helpers for aligned, zero-inclusive sweep archive payloads."""
from __future__ import annotations

import numpy as np


def build_zero_inclusive_archive(
        levels,
        rest_positions,
        positions,
        tet_nodes,
        substeps,
        ke_final,
        settle_reason,
        *,
        braid_rest=None,
        braid_positions=None,
        braid_edges=None,
        **extra,
) -> dict:
    """Validate and build the common snapshots.npz payload.

    ``positions`` and all convergence arrays must already contain settled rest
    as row zero.  ``levels`` may include later planned levels after an early
    divergence; it is truncated to the number of recorded aligned states.
    Optional braid arrays obey exactly the same row contract.
    """
    rest = np.asarray(rest_positions)
    pos = np.asarray(positions)
    n = len(pos)
    if n < 1:
        raise ValueError("archive needs at least the settled rest state")
    u = np.asarray(levels, dtype=float)[:n]
    sub = np.asarray(substeps, dtype=np.int32)
    ke = np.asarray(ke_final, dtype=np.float64)
    reasons = np.asarray(settle_reason)
    if len(u) != n:
        raise ValueError("planned levels are shorter than recorded states")
    if len(sub) != n or len(ke) != n or len(reasons) != n:
        raise ValueError("convergence arrays are not aligned with positions")
    if not np.isclose(u[0], 0.0, rtol=0.0, atol=1e-12):
        raise ValueError("archive row zero must have zero input")
    if pos.shape[1:] != rest.shape or not np.allclose(
            pos[0], rest, rtol=1e-7, atol=1e-12, equal_nan=False):
        raise ValueError("positions[0] must equal settled rest_positions")

    payload = {
        "pressures": u,
        "rest_positions": rest,
        "positions": pos,
        "tet_nodes": np.asarray(tet_nodes),
        "substeps": sub,
        "ke_final": ke,
        "settle_reason": reasons,
        "rest_index": np.array(0, dtype=np.int64),
        "rest_substeps": np.array(sub[0], dtype=np.int32),
        "rest_ke_final": np.array(ke[0], dtype=np.float64),
        "rest_settle_reason": np.array(reasons[0]),
    }

    braid_values = (braid_rest, braid_positions, braid_edges)
    if any(v is not None for v in braid_values):
        if not all(v is not None for v in braid_values):
            raise ValueError("braid_rest, braid_positions, and braid_edges travel together")
        b_rest = np.asarray(braid_rest)
        b_pos = np.asarray(braid_positions)
        if len(b_pos) != n:
            raise ValueError("braid_positions are not aligned with body positions")
        if b_pos.shape[1:] != b_rest.shape or not np.allclose(
                b_pos[0], b_rest, rtol=1e-7, atol=1e-12, equal_nan=False):
            raise ValueError("braid_positions[0] must equal braid_rest")
        payload.update({
            "braid_rest": b_rest,
            "braid_positions": b_pos,
            "braid_edges": np.asarray(braid_edges),
        })

    payload.update(extra)
    return payload
