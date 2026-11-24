"""
jacobian.py — Phase 2: controllability characterization from snapshot data.

Reads a snapshots.npz from run_sweep.py and extracts:

  Spine sampling. K node-groups at evenly-spaced X positions along the rest
  finger. Each group's centroid traces a spine point through the sweep.

  Per-snapshot observable y(p):
      y = [spine_positions (3K)  ||  spine_tangents (3K)]    in R^{6K}
  Tangents are central-difference of consecutive spine points (normalized).

  Per-level Jacobian J(p):
      J(p_i) = (y(p_{i+1}) - y(p_{i-1})) / (p_{i+1} - p_{i-1})    in R^{6K x 1}
  Forward/backward difference at the endpoints.

  Singular value sigma(p):
      Single-input case: sigma(p) = ||J(p)||_2
      Multi-input (later): SVD of stacked-column J matrix.

  Outputs:
    jacobian.npz     — pressures, spine_indices, spine_positions per level,
                        y per level, J per level, sigma per level
    jacobian.png     — sigma vs p plot + spine deformation overlay

Usage:
    python jacobian.py snapshots.npz --n_spine 10 --out jacobian.npz \\
        --plot jacobian.png

Multi-chamber comes later (Phase 4): then we sweep each chamber independently,
stack the per-input J columns, and SVD for full controllability rank /
condition number / dominant directions.
"""

from __future__ import annotations

import argparse
import os
import sys
import warnings

import numpy as np


def find_spine_indices(rest_positions: np.ndarray, n_points: int,
                       x_margin: float = 0.05) -> list[np.ndarray]:
    """Return one list of node indices per spine point.

    Spine points sit at `n_points` evenly-spaced X positions between
    [x_min + margin, x_max - margin]. Each point captures all nodes within
    a narrow X-band around its target.
    """
    x = rest_positions[:, 0]
    x_min, x_max = float(x.min()), float(x.max())
    span = x_max - x_min
    x_targets = np.linspace(x_min + x_margin * span, x_max - x_margin * span,
                            n_points)
    half_width = span / (2.5 * n_points)
    return [np.where(np.abs(x - tx) < half_width)[0] for tx in x_targets]


def spine_positions(positions: np.ndarray,
                    spine_indices: list[np.ndarray]) -> np.ndarray:
    """Centroids per spine group, shape (K, 3)."""
    return np.array([positions[idx].mean(axis=0) for idx in spine_indices])


def spine_tangents(spine_pos: np.ndarray) -> np.ndarray:
    """Unit tangent at each spine point (central differences, end-padded)."""
    n = len(spine_pos)
    t = np.zeros_like(spine_pos)
    t[1:-1] = spine_pos[2:] - spine_pos[:-2]
    t[0] = spine_pos[1] - spine_pos[0]
    t[-1] = spine_pos[-1] - spine_pos[-2]
    norms = np.linalg.norm(t, axis=1, keepdims=True)
    return t / np.maximum(norms, 1e-12)


def feature_vector(positions: np.ndarray,
                   spine_indices: list[np.ndarray]) -> np.ndarray:
    sp = spine_positions(positions, spine_indices)
    tg = spine_tangents(sp)
    return np.concatenate([sp.ravel(), tg.ravel()])


def normalize_archive_rest_state(
        pressures: np.ndarray,
        positions: np.ndarray,
        rest_positions: np.ndarray,
        zero_atol: float = 1e-12,
) -> tuple[np.ndarray, np.ndarray, int, str]:
    """Return a zero-inclusive, ordered state sequence and explicit rest index.

    Legacy pressure/tendon archives commonly saved ``rest_positions`` but began
    ``positions`` at the first positive input.  Such strictly increasing
    positive-only branches gain one leading zero/rest state.  Existing unique
    zero levels are retained (including an internal zero on a monotonic
    mixed-sign branch), and their state is made exactly equal to the separately
    saved settled rest geometry.

    Non-monotonic, repeated-input, multi-zero, and positive-only descending
    sequences are ambiguous for finite differences and are rejected rather
    than silently reordered.
    """
    p = np.asarray(pressures, dtype=float).reshape(-1).copy()
    pos = np.asarray(positions, dtype=float).copy()
    rest = np.asarray(rest_positions, dtype=float)
    if pos.ndim < 2 or pos.shape[0] != len(p):
        raise ValueError(
            f"pressures/positions length mismatch: {len(p)} vs {pos.shape[:1]}")
    if pos.shape[1:] != rest.shape:
        raise ValueError(
            f"rest shape {rest.shape} does not match state shape {pos.shape[1:]}")
    if len(p) == 0 or not np.isfinite(p).all():
        raise ValueError("pressure/input sequence must be non-empty and finite")

    zero_hits = np.flatnonzero(np.isclose(p, 0.0, rtol=0.0, atol=zero_atol))
    if len(zero_hits) > 1:
        raise ValueError("ambiguous archive ordering: multiple zero-input rows")

    if len(p) > 1:
        dp = np.diff(p)
        increasing = bool(np.all(dp > 0.0))
        decreasing = bool(np.all(dp < 0.0))
        if not (increasing or decreasing):
            raise ValueError(
                "ambiguous archive ordering: inputs must be strictly monotonic")
    else:
        increasing = decreasing = True

    if len(zero_hits) == 1:
        rest_index = int(zero_hits[0])
        p[rest_index] = 0.0
        if not np.allclose(pos[rest_index], rest, rtol=1e-7, atol=1e-12,
                           equal_nan=False):
            warnings.warn(
                "zero-input state differs from saved rest_positions; using the "
                "saved settled rest geometry", RuntimeWarning, stacklevel=2)
            pos[rest_index] = rest
        mixed = bool(np.any(p < -zero_atol) and np.any(p > zero_atol))
        if mixed:
            kind = "mixed_sign_internal_zero"
        elif rest_index == 0:
            kind = "zero_inclusive"
        else:
            kind = "zero_inclusive_rest_at_end"
        return p, pos, rest_index, kind

    positive_forward = bool(np.all(p > zero_atol) and increasing)
    negative_forward = bool(np.all(p < -zero_atol) and decreasing)
    if not (positive_forward or negative_forward):
        raise ValueError(
            "ambiguous archive ordering: no zero row and branch does not move "
            "monotonically away from rest")

    p = np.concatenate(([0.0], p))
    pos = np.concatenate((rest[None, ...], pos), axis=0)
    kind = ("prepended_positive_only_rest" if positive_forward
            else "prepended_negative_only_rest")
    return p, pos, 0, kind


def _archive_has(data, key: str) -> bool:
    files = getattr(data, "files", None)
    return key in files if files is not None else key in data


def rest_spine_from_jacobian(
        data,
        spine: np.ndarray | None = None,
        pressures: np.ndarray | None = None,
) -> tuple[np.ndarray, int | None, str]:
    """Extract the explicit settled rest spine from any Jacobian generation.

    New files carry both ``rest_spine`` and ``rest_index``.  Older files prefer
    their saved ``rest_feature_vector`` (the true settled rest even when a zero
    row is approximate), while an explicit or uniquely inferred rest index is
    retained for skipping that level.  The zero-input row is geometry fallback
    only when no separately saved rest exists; arbitrary row zero is never used.
    """
    S = np.asarray(data["spine_positions"] if spine is None else spine,
                   dtype=float)
    u = np.asarray(data["pressures"] if pressures is None else pressures,
                   dtype=float)

    rest_index = None
    if _archive_has(data, "rest_index"):
        rest_index = int(np.asarray(data["rest_index"]).reshape(()))
        if not 0 <= rest_index < len(S):
            raise ValueError(f"rest_index {rest_index} outside {len(S)} levels")
    zero_hits = np.flatnonzero(np.isclose(u, 0.0, rtol=0.0, atol=1e-12))
    if rest_index is None and len(zero_hits) == 1:
        rest_index = int(zero_hits[0])
    if _archive_has(data, "rest_spine"):
        rest = np.asarray(data["rest_spine"], dtype=float)
        if rest.shape != S.shape[1:]:
            raise ValueError(
                f"rest_spine shape {rest.shape} vs levels {S.shape[1:]}")
        return rest, rest_index, "rest_spine"
    if _archive_has(data, "rest_feature_vector"):
        flat = np.asarray(data["rest_feature_vector"], dtype=float).reshape(-1)
        n_position = int(np.prod(S.shape[1:]))
        if len(flat) < n_position:
            raise ValueError("rest_feature_vector is too short for spine positions")
        return flat[:n_position].reshape(S.shape[1:]), rest_index, \
            "rest_feature_vector"
    if rest_index is not None:
        # Geometry fallback only: prefer separately saved rest_spine or the
        # legacy rest_feature_vector whenever either is available.
        return S[rest_index].copy(), rest_index, "zero_row_fallback"
    raise ValueError(
        "jacobian has no explicit rest_spine/rest_index, saved rest feature, "
        "or unique zero-input row")


def central_diff_jacobians(pressures: np.ndarray,
                           Y: np.ndarray) -> np.ndarray:
    """For a sequence of (p_i, y_i), return J_i = dy/dp at each i.

    Forward diff at i=0, backward at i=-1, central elsewhere.
    Y has shape (N, D); returns (N, D).
    """
    N, D = Y.shape
    J = np.zeros((N, D))
    J[0] = (Y[1] - Y[0]) / (pressures[1] - pressures[0])
    J[-1] = (Y[-1] - Y[-2]) / (pressures[-1] - pressures[-2])
    for i in range(1, N - 1):
        J[i] = (Y[i + 1] - Y[i - 1]) / (pressures[i + 1] - pressures[i - 1])
    return J


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("snapshots_npz",
                        help="path to snapshots.npz from run_sweep.py")
    parser.add_argument("--n_spine", type=int, default=10,
                        help="number of spine sample points (default 10)")
    parser.add_argument("--x_margin", type=float, default=0.05,
                        help="fractional margin from each end (default 0.05)")
    parser.add_argument("--target_point", default="tip",
                        help="which spine point to characterize "
                             "(integer 0..K-1, or 'tip' / 'base' / 'mid'; "
                             "default 'tip' = K-1)")
    parser.add_argument("--p_max", type=float, default=None,
                        help="ignore snapshots above this pressure (Pa); "
                             "use to trim a contaminated high-pressure tail "
                             "(e.g. contact pop-through) before differencing")
    parser.add_argument("--out", default="jacobian.npz",
                        help="output .npz path")
    parser.add_argument("--plot", default=None,
                        help="if set, also save a PNG plot to this path")
    args = parser.parse_args()

    def resolve_target(name: str, K: int) -> int:
        if name == "tip":
            return K - 1
        if name == "base":
            return 0
        if name == "mid":
            return K // 2
        try:
            n = int(name)
            if not 0 <= n < K:
                raise ValueError
            return n
        except ValueError:
            print(f"ERROR: --target_point must be 'tip', 'base', 'mid', or "
                  f"an integer in [0, {K - 1}]", file=sys.stderr)
            sys.exit(2)

    data = np.load(args.snapshots_npz, allow_pickle=False)
    pressures = np.asarray(data["pressures"], dtype=float)
    rest = np.asarray(data["rest_positions"], dtype=float)
    positions = np.asarray(data["positions"], dtype=float)  # (N, P, 3)

    # Convergence info if present.  It is normalized alongside the state rows
    # so the explicit rest level remains aligned for highlighting and audit.
    settle_reason = (np.asarray(data["settle_reason"])
                     if "settle_reason" in data.files else None)

    # Older run_sweep.py saved the FULL planned pressures vector even when
    # the sweep stopped early (NaN / divergence), leaving pressures longer
    # than positions. Trim all per-level arrays before rest normalization.
    if len(pressures) != len(positions):
        n_have = min(len(positions), len(pressures))
        print(f"  pressures ({len(pressures)}) and positions "
              f"({len(positions)}) differ; truncating to {n_have} "
              f"recorded aligned levels.")
        pressures = pressures[:n_have]
        positions = positions[:n_have]
        if settle_reason is not None:
            settle_reason = settle_reason[:n_have]

    try:
        pressures, positions, rest_index, normalization_kind = \
            normalize_archive_rest_state(pressures, positions, rest)
    except ValueError as exc:
        print(f"ERROR: cannot normalize rest state: {exc}", file=sys.stderr)
        return 2
    prepended_rest = normalization_kind.startswith("prepended_")
    if settle_reason is not None and prepended_rest:
        rest_reason = (str(np.asarray(data["rest_settle_reason"]).reshape(()))
                       if "rest_settle_reason" in data.files else "rest_saved")
        settle_reason = np.concatenate(([rest_reason], settle_reason))
    print(f"  rest normalization: {normalization_kind}; "
          f"rest_index={rest_index}")
    if settle_reason is not None:
        from collections import Counter
        counts = Counter(str(r) for r in settle_reason)
        print(f"  convergence: {dict(counts)}")
    else:
        print("  convergence: settle_reason field NOT in this snapshots.npz "
              "(max-step highlighting will be skipped)")
    rest_flags = np.arange(len(pressures)) == rest_index

    # Drop divergent snapshots (NaN/Inf) so the differential makes sense.
    finite_mask = np.isfinite(positions).all(axis=(1, 2))
    n_dropped = int((~finite_mask).sum())
    if n_dropped:
        print(f"  dropping {n_dropped} non-finite snapshots "
              f"(first at p={pressures[~finite_mask][0]:.3f} Pa)")
        positions = positions[finite_mask]
        pressures = pressures[finite_mask]
        rest_flags = rest_flags[finite_mask]
        if settle_reason is not None:
            settle_reason = settle_reason[finite_mask]

    if args.p_max is not None:
        keep = pressures <= args.p_max + 1e-9
        if keep.sum() < len(pressures):
            print(f"  capping at p_max={args.p_max:g} Pa: keeping "
                  f"{int(keep.sum())}/{len(pressures)} levels")
            positions = positions[keep]
            pressures = pressures[keep]
            rest_flags = rest_flags[keep]
            if settle_reason is not None:
                settle_reason = settle_reason[keep]

    if int(rest_flags.sum()) != 1:
        print("ERROR: filtering removed or duplicated the explicit rest level",
              file=sys.stderr)
        return 2
    rest_index = int(np.flatnonzero(rest_flags)[0])

    if len(pressures) < 3:
        print("ERROR: need at least 3 finite snapshots for central differences",
              file=sys.stderr)
        return 2

    print(f"Loaded {args.snapshots_npz}: {len(pressures)} valid snapshots, "
          f"p in [{pressures.min():.3f}, {pressures.max():.3f}] Pa")

    spine_idx = find_spine_indices(rest, args.n_spine, args.x_margin)
    print(f"Spine: {args.n_spine} sample points, "
          f"{[len(idx) for idx in spine_idx]} nodes per group")

    # Build per-snapshot feature vectors y_i (shape (N, 6K)).
    K = args.n_spine
    D = 6 * K
    Y = np.zeros((len(pressures), D))
    spine_pos_per = np.zeros((len(pressures), K, 3))
    for i, pos in enumerate(positions):
        sp = spine_positions(pos, spine_idx)
        spine_pos_per[i] = sp
        Y[i, :3 * K] = sp.ravel()
        Y[i, 3 * K:] = spine_tangents(sp).ravel()

    rest_y = feature_vector(rest, spine_idx)
    Y_disp = Y - rest_y[None, :]

    target_idx = resolve_target(args.target_point, K)
    target_label = (f"point {target_idx} (tip)" if target_idx == K - 1
                    else f"point {target_idx} (base)" if target_idx == 0
                    else f"point {target_idx}")

    J = central_diff_jacobians(pressures, Y)              # (N, 6K)
    J_pos = J[:, :3 * K]                                  # (N, 3K) — position part only
    sigma_pos = np.linalg.norm(J_pos, axis=1)             # m/Pa, clean units
    sigma_full = np.linalg.norm(J, axis=1)                # legacy, mixed-units

    # Per-point sensitivity (3-vector dy_point/dp, magnitude, unit direction)
    J_point = J_pos[:, 3 * target_idx:3 * (target_idx + 1)]   # (N, 3)
    sigma_point = np.linalg.norm(J_point, axis=1)
    direction_point = J_point / np.maximum(sigma_point[:, None], 1e-30)

    # ------------------------------------------------------------------
    # Empirical Gramians (position-only — mixed-units would distort
    # eigenvalue magnitudes).
    #   W_c  = sum_k |dp_k|  J_k J_k^T          (controllability)
    #   W_o  = sum_k |dp_k| (y_k - y0)(y_k - y0)^T  (observability)
    # The eigendecomposition gives the principal output-space directions
    # the actuator visits; the spectrum's effective rank is a single
    # number characterizing actuator complexity.
    # ------------------------------------------------------------------
    Y_pos = Y[:, :3 * K]
    dp = np.zeros_like(pressures)
    if len(pressures) >= 2:
        dp[0] = pressures[1] - pressures[0]
        dp[-1] = pressures[-1] - pressures[-2]
        dp[1:-1] = 0.5 * (pressures[2:] - pressures[:-2])
    abs_dp = np.abs(dp)
    W_c = np.einsum("k,ki,kj->ij", abs_dp, J_pos, J_pos)
    rest_pos_flat = rest_y[:3 * K]
    Y_dev = Y_pos - rest_pos_flat[None, :]
    W_o = np.einsum("k,ki,kj->ij", abs_dp, Y_dev, Y_dev)

    eig_c_all, vec_c = np.linalg.eigh(W_c)
    order_c = np.argsort(eig_c_all)[::-1]
    eig_c = eig_c_all[order_c]
    vec_c = vec_c[:, order_c]

    eig_o_all, vec_o = np.linalg.eigh(W_o)
    order_o = np.argsort(eig_o_all)[::-1]
    eig_o = eig_o_all[order_o]
    vec_o = vec_o[:, order_o]

    def effective_rank(eigs: np.ndarray, threshold: float = 0.99) -> int:
        total = float(eigs.sum())
        if total <= 0:
            return 0
        return int(np.searchsorted(np.cumsum(eigs) / total, threshold) + 1)

    rank_c = effective_rank(eig_c)
    rank_o = effective_rank(eig_o)
    ratio_c = (float(eig_c[0] / eig_c[1])
               if len(eig_c) > 1 and eig_c[1] > 0 else float("inf"))

    # Per-step displacement of the chosen point (relative to rest)
    rest_spine = spine_positions(rest, spine_idx)
    target_disp_mm = (np.linalg.norm(spine_pos_per[:, target_idx]
                                     - rest_spine[target_idx], axis=1) * 1000.0)

    print(f"\n  target: {target_label}\n")
    print(f"  {'p (Pa)':>8} {'sigma_pos':>12} {'sigma_point':>14} "
          f"{'point_disp_mm':>14} {'dx_hat':>8} {'dy_hat':>8} {'dz_hat':>8}")
    print(f"  {'(spine)':>8} {'(m/Pa)':>12} {'(m/Pa)':>14} "
          f"{'(mm)':>14}")
    for i in range(len(pressures)):
        print(f"  {pressures[i]:>8.3f} {sigma_pos[i]:>12.4e} "
              f"{sigma_point[i]:>14.4e} {target_disp_mm[i]:>14.3f} "
              f"{direction_point[i, 0]:>8.3f} "
              f"{direction_point[i, 1]:>8.3f} "
              f"{direction_point[i, 2]:>8.3f}")

    print(f"\n  Gramian summary (position-only output, {3 * K} dims):")
    print(f"    W_c top 5 eigenvalues: "
          f"{', '.join(f'{e:.3e}' for e in eig_c[:5])}")
    print(f"    W_o top 5 eigenvalues: "
          f"{', '.join(f'{e:.3e}' for e in eig_o[:5])}")
    print(f"    effective rank (W_c, 99% energy): {rank_c}")
    print(f"    effective rank (W_o, 99% energy): {rank_o}")
    print(f"    lambda_1 / lambda_2 (W_c): {ratio_c:.2f}  "
          f"(=> {'1D-dominated' if ratio_c > 100 else 'multi-modal'})")

    np.savez(
        args.out,
        pressures=pressures,
        spine_node_indices=np.array(
            [np.pad(idx, (0, max(len(s) for s in spine_idx) - len(idx)),
                    constant_values=-1) for idx in spine_idx],
            dtype=np.int64,
        ),
        spine_positions=spine_pos_per,
        rest_index=np.array(rest_index, dtype=np.int64),
        rest_spine=rest_spine,
        rest_normalization=np.array(normalization_kind),
        feature_vectors=Y,
        rest_feature_vector=rest_y,
        jacobian=J,
        sigma=sigma_full,                # kept for backward compat
        sigma_pos=sigma_pos,              # position-only norm (m/Pa)
        sigma_point=sigma_point,          # |dy_point/dp| at chosen spine point
        direction=J / np.maximum(sigma_full[:, None], 1e-30),  # legacy
        direction_point=direction_point,  # unit direction of motion at the point
        target_point=target_idx,
        # Gramian outputs (position-only, 3K x 3K)
        W_c=W_c,
        W_o=W_o,
        eig_c=eig_c,
        eig_o=eig_o,
        vec_c=vec_c,
        vec_o=vec_o,
        rank_c=rank_c,
        rank_o=rank_o,
        lambda_ratio_c=ratio_c,
    )
    print(f"\nWrote {args.out}")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        # Pressures of levels that hit max_substeps. We draw both a faint
        # vertical line at each non-converged pressure AND ring the data
        # markers in red on each panel.
        unsettled_mask = np.zeros(len(pressures), dtype=bool)
        if settle_reason is not None:
            unsettled_mask = np.array(
                [str(r) == "max_steps" for r in settle_reason]
            )
            n_unsettled = int(unsettled_mask.sum())
            if n_unsettled:
                print(f"  {n_unsettled} of {len(pressures)} levels hit "
                      f"max_substeps (red rings + faint red lines)")
            else:
                print(f"  all {len(pressures)} levels settled cleanly "
                      f"(nothing to highlight)")
        unsettled_p = pressures[unsettled_mask]

        def shade_unsettled(ax):
            for p in unsettled_p:
                ax.axvline(p, color="tab:red", alpha=0.25, linewidth=0.8,
                           zorder=0)

        def ring_unsettled(ax, ys):
            if unsettled_mask.any():
                ax.scatter(pressures[unsettled_mask], np.asarray(ys)[unsettled_mask],
                           s=80, facecolors="none", edgecolors="tab:red",
                           linewidths=1.5, zorder=5, label="hit max_substeps")

        fig, axes = plt.subplots(3, 2, figsize=(13, 14))

        # (1) Per-point sensitivity vs p — clean units, single point
        ax = axes[0, 0]
        shade_unsettled(ax)
        ax.plot(pressures, sigma_point * 1000.0, "o-", color="tab:blue",
                label=f"|d(y_{target_idx})/dp|  ({target_label})")
        # Also overlay the whole-spine position norm as a secondary signal
        ax.plot(pressures, sigma_pos * 1000.0, "-", color="0.6", alpha=0.7,
                label="||J_positions|| (whole spine)")
        ring_unsettled(ax, sigma_point * 1000.0)
        ax.set_xlabel("pressure (Pa)")
        ax.set_ylabel("sensitivity (mm / Pa)")
        ax.set_title("How much does the chosen point move per Pa?")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.4)

        # (2) Displacement of the chosen point vs p
        ax = axes[0, 1]
        shade_unsettled(ax)
        ax.plot(pressures, target_disp_mm, "o-", color="tab:orange")
        ring_unsettled(ax, target_disp_mm)
        ax.set_xlabel("pressure (Pa)")
        ax.set_ylabel("displacement (mm)")
        ax.set_title(f"Displacement of {target_label} from rest")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.4)

        # (3) Spine shape every Nth snapshot, with chosen point marked
        ax = axes[1, 0]
        step = max(1, len(pressures) // 10)
        cmap = plt.get_cmap("viridis")
        for i in range(0, len(pressures), step):
            sp = spine_pos_per[i]
            ax.plot(sp[:, 0] * 1000, sp[:, 1] * 1000,
                    color=cmap(i / max(1, len(pressures) - 1)),
                    marker=".", label=f"{pressures[i]:.2f} Pa")
            ax.plot(sp[target_idx, 0] * 1000, sp[target_idx, 1] * 1000,
                    color=cmap(i / max(1, len(pressures) - 1)),
                    marker="o", markersize=8, markerfacecolor="none",
                    markeredgewidth=1.5)
        ax.plot(rest_spine[:, 0] * 1000, rest_spine[:, 1] * 1000,
                color="black", linestyle="--", label="rest")
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_title("Spine shapes (chosen point marked by hollow circle)")
        ax.legend(fontsize=7, loc="best")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.4)

        # (4) Unit direction of motion at the chosen point
        ax = axes[1, 1]
        shade_unsettled(ax)
        ax.plot(pressures, direction_point[:, 0], label="x̂", color="tab:red")
        ax.plot(pressures, direction_point[:, 1], label="ŷ", color="tab:green")
        ax.plot(pressures, direction_point[:, 2], label="ẑ", color="tab:blue")
        ax.axhline(0, color="0.7", linewidth=0.5)
        # Ring all three components at unsettled pressures
        if unsettled_mask.any():
            for col, vals in zip(("tab:red", "tab:green", "tab:blue"),
                                  (direction_point[:, 0],
                                   direction_point[:, 1],
                                   direction_point[:, 2])):
                ax.scatter(pressures[unsettled_mask], vals[unsettled_mask],
                           s=80, facecolors="none", edgecolors=col,
                           linewidths=1.2, zorder=5)
        ax.set_xlabel("pressure (Pa)")
        ax.set_ylabel("unit direction component")
        ax.set_ylim(-1.05, 1.05)
        ax.set_title(f"Unit direction the {target_label} moves per +dp")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.4)

        # (5) Gramian eigenvalue spectra (log scale)
        ax = axes[2, 0]
        idx = np.arange(1, len(eig_c) + 1)
        floor = max(eig_c.max(), eig_o.max(), 1.0) * 1e-16
        ax.semilogy(idx, np.maximum(eig_c, floor), "o-",
                    color="tab:blue", label=r"$W_c$ (controllability)")
        ax.semilogy(idx, np.maximum(eig_o, floor), "s-",
                    color="tab:orange", label=r"$W_o$ (observability)")
        # Mark 99% energy threshold on the controllability spectrum
        ax.axvline(rank_c, color="tab:blue", alpha=0.3, linestyle="--",
                   label=f"rank(W_c, 99%) = {rank_c}")
        ax.set_xlabel("mode index")
        ax.set_ylabel("eigenvalue")
        ax.set_title("Gramian spectra (effective rank = how many "
                     "independent modes)")
        ax.legend(fontsize=8)
        ax.grid(True, which="both", alpha=0.3)

        # (6) Top 3 controllability modes drawn on the spine (X-Y plane)
        ax = axes[2, 1]
        rest_xy_mm = rest_spine[:, :2] * 1000.0
        span_mm = float(np.linalg.norm(rest_xy_mm.max(0) - rest_xy_mm.min(0)))
        scale_mm = 0.25 * span_mm
        mode_colors = ("tab:red", "tab:green", "tab:purple")
        ax.plot(rest_xy_mm[:, 0], rest_xy_mm[:, 1], "k.-",
                label="rest spine", linewidth=1.5)
        for mi in range(min(3, vec_c.shape[1])):
            mode = vec_c[:, mi].reshape(K, 3)
            mode_xy = mode[:, :2]
            n = float(np.linalg.norm(mode_xy))
            if n < 1e-12:
                continue
            disp_mm = mode_xy * (scale_mm / n)
            ax.plot(rest_xy_mm[:, 0] + disp_mm[:, 0],
                    rest_xy_mm[:, 1] + disp_mm[:, 1],
                    "+-", color=mode_colors[mi],
                    label=f"mode {mi + 1} (λ={eig_c[mi]:.2e})")
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_title("Top 3 W_c modes on spine (mode-magnitude exaggerated)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        fig.suptitle(
            f"Controllability characterization — {target_label}  "
            f"|  rank(W_c) = {rank_c},  λ₁/λ₂ = {ratio_c:.1f}",
            fontsize=12,
        )
        fig.tight_layout()
        fig.savefig(args.plot, dpi=120)
        plt.close(fig)
        print(f"Wrote {args.plot}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
