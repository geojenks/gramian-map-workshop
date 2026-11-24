"""
run_sweep.py — headless pressure-sweep runner.

Loads parametricGripper.py without GUI, ramps the cavity pressure from 0
to a max in N levels, settles each level toward quasi-equilibrium, and
logs the tip pose + simple bending metrics to CSV + console.

Run from this folder (mesh_route/) with the SOFA Python interpreter that has Sofa
bindings on the path:

    python run_sweep.py
    python run_sweep.py --p_max 200 --n_steps 40 --out sweep_low.csv
    python run_sweep.py --p_max 20000 --n_steps 30 --max_substeps 200

What it captures per pressure level:
  pressure_Pa     command pressure (positive number; sign handled internally)
  tip_x,y,z       tip ROI centroid in world coords (m)
  disp_x,y,z      tip displacement from rest (m)
  disp_norm_mm    Euclidean tip displacement magnitude (mm)
  tilt_deg        atan2(disp_y, disp_x) — coarse bending angle proxy
  substeps        sim steps taken to reach equilibrium
  ke_final        final kinetic energy (J-equivalent)

CSV columns are in the same order. Console prints a one-line summary
per level so you can eyeball monotonicity / blow-up at a glance.

Returns nonzero exit if any pressure level failed to settle (ke stays
above threshold after max_substeps) — useful for headless iteration.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys

import numpy as np

from archive_contract import build_zero_inclusive_archive


HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def _bootstrap_sofa() -> None:
    """Ensure `import Sofa` works under plain `python`. SOFA's bindings live
    under <SOFA_ROOT>/plugins/SofaPython3/lib/python3/site-packages/ and the
    .pyd modules are tagged cp310 — so we need:
      (1) that path on sys.path
      (2) DLL search paths set (handled by Sofa/__init__.py once it's found)
      (3) Python 3.10
    """
    try:
        import Sofa  # noqa: F401
        return
    except ImportError:
        pass

    sofa_root = os.environ.get("SOFA_ROOT")
    if not sofa_root:
        print("ERROR: SOFA_ROOT is not set. Set SOFA_ROOT to your SOFA 24.06 "
              "install root (optionally SOFAPYTHON3_ROOT to its SofaPython3 "
              "plugin folder) and run under Python 3.10.", file=sys.stderr)
        sys.exit(2)

    sp3_root = os.environ.get(
        "SOFAPYTHON3_ROOT",
        os.path.join(sofa_root, "plugins", "SofaPython3"),
    )
    os.environ.setdefault("SOFAPYTHON3_ROOT", sp3_root)
    site_pkgs = os.path.join(sp3_root, "lib", "python3", "site-packages")
    if not os.path.isdir(site_pkgs):
        print(f"ERROR: SOFA python site-packages not found at {site_pkgs}",
              file=sys.stderr)
        sys.exit(2)
    sys.path.insert(0, site_pkgs)


_bootstrap_sofa()


def kinetic_energy(mo, total_mass: float) -> float:
    v = np.array(mo.velocity.value)
    n = len(v)
    if n == 0:
        return 0.0
    m_per_node = total_mass / n
    return 0.5 * m_per_node * float(np.sum(v ** 2))


def total_kinetic_energy(bodies: list[tuple]) -> float:
    """Sum of KE over (mechanical_object, total_mass) pairs. Used so the
    settle detector also waits for the braid body (two-body McKibben) to
    stop ringing, not just the bladder."""
    return sum(kinetic_energy(mo, m) for mo, m in bodies)


def centroid(mo, indices: np.ndarray) -> np.ndarray:
    pos = np.array(mo.position.value)
    return pos[indices].mean(axis=0)


def step_to_equilibrium(root, bodies: list[tuple],
                        ke_threshold: float, max_substeps: int,
                        progress_every: int = 10,
                        plateau_window: int = 15,
                        plateau_rel_tol: float = 0.03,
                        disp_tol: float = 1e-7,
                        ) -> tuple[int, float, str]:
    """Step until one of:
      - KE goes NaN/Inf (numerical divergence)
      - KE < ke_threshold (true static equilibrium)
      - geometry settled: the fastest-moving node moves < `disp_tol` m per
        step for a full `plateau_window` (robust to KE chatter from contact
        noise, which keeps the KE-spread test from firing even though the
        actuator has physically stopped — this is the usual early-out)
      - KE plateau detected (NaN-safe; spread of last `plateau_window`
        finite samples is less than `plateau_rel_tol`)
      - max_substeps hit (timeout)
    `bodies` is a list of (mechanical_object, total_mass) pairs whose
    summed KE drives the detector.
    Returns (substeps_used, final_ke, reason)."""
    dt = root.dt.value
    import Sofa.Simulation
    history: list[float] = []
    disp_hist: list[float] = []
    prev_pos = [np.array(mo.position.value) for mo, _ in bodies]
    for i in range(max_substeps):
        Sofa.Simulation.animate(root, dt)
        ke = total_kinetic_energy(bodies)
        history.append(ke)
        # Max per-step nodal displacement across all tracked bodies — a
        # direct "has anything moved" measure that ignores KE ringing.
        max_disp = 0.0
        for k, (mo, _) in enumerate(bodies):
            cur = np.array(mo.position.value)
            if cur.shape == prev_pos[k].shape and cur.size:
                max_disp = max(max_disp, float(np.abs(cur - prev_pos[k]).max()))
            prev_pos[k] = cur
        disp_hist.append(max_disp)
        if progress_every and (i + 1) % progress_every == 0:
            print(f"    step {i+1}/{max_substeps}  ke={ke:.3e}  "
                  f"dmax={max_disp:.2e}", flush=True)
        # Hard divergence — abort this level immediately. The caller will
        # decide whether to stop the whole sweep.
        if not np.isfinite(ke):
            return i + 1, ke, "diverged"
        if i >= 5 and ke < ke_threshold:
            return i + 1, ke, "below_threshold"
        if i >= 5 and len(disp_hist) >= plateau_window:
            if max(disp_hist[-plateau_window:]) < disp_tol:
                return i + 1, ke, "settled_disp"
        if len(history) >= plateau_window:
            window = np.asarray(history[-plateau_window:])
            if np.isfinite(window).all():
                lo, hi = float(window.min()), float(window.max())
                if hi > 0 and (hi - lo) / hi < plateau_rel_tol:
                    return i + 1, ke, "plateau"
    return max_substeps, history[-1] if history else 0.0, "max_steps"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--p_max", type=float, default=None,
                        help="max pressure (Pa); default = PRESSURE_MAX from the scene")
    parser.add_argument("--n_steps", type=int, default=20,
                        help="number of nonzero pressure increments; the "
                             "settled zero state is saved in addition")
    parser.add_argument("--out", default=os.path.join(HERE, "sweep.csv"),
                        help="output CSV path")
    parser.add_argument("--ke_threshold", type=float, default=1e-9,
                        help="KE threshold for quasi-equilibrium")
    parser.add_argument("--disp_tol", type=float, default=1e-7,
                        help="geometry-settled early-out: stop a level once "
                             "the fastest-moving node moves < this (m) per "
                             "step for a full window. Robust to KE chatter "
                             "from contact; this is the usual early-out and "
                             "clips the long plateau tail. Default 1e-7 m.")
    parser.add_argument("--max_substeps", type=int, default=300,
                        help="max sim steps per pressure level")
    parser.add_argument("--settle_steps", type=int, default=50,
                        help="initial settle steps at p=0 before the sweep")
    parser.add_argument("--snapshots", default=None,
                        help="if set, path to write snapshots.npz with body "
                             "positions at each equilibrium (for replay.py)")
    parser.add_argument("--dt", type=float, default=None,
                        help="override the scene's time step (s). Stiff "
                             "braid springs in two-body McKibbens may need "
                             "dt below the scene default of 0.01.")
    parser.add_argument("--ramp_substeps", type=int, default=50,
                        help="substeps over which the pressure ramps "
                             "linearly from the previous level to the new "
                             "target before the settle phase begins "
                             "(default 50; 0 = instant step). Instant "
                             "multi-kPa steps shock-load the body and can "
                             "diverge a scene that is perfectly stable "
                             "under quasi-static loading.")
    args = parser.parse_args()

    # SOFA imports happen here so the script's --help doesn't require them.
    import Sofa
    import Sofa.Core
    import Sofa.Simulation
    import SofaRuntime

    # Make sure the relevant plugin set is loaded. We mirror what
    # parametricGripper.py declares via RequiredPlugin.
    for plugin in (
        "SofaPython3",
        "SoftRobots",
        "Sofa.Component.AnimationLoop",
        "Sofa.Component.Constraint.Lagrangian.Correction",
        "Sofa.Component.Constraint.Lagrangian.Solver",
        "Sofa.Component.Engine.Select",
        "Sofa.Component.IO.Mesh",
        "Sofa.Component.LinearSolver.Direct",
        "Sofa.Component.Mapping.Linear",
        "Sofa.Component.Mass",
        "Sofa.Component.ODESolver.Backward",
        "Sofa.Component.Setting",
        "Sofa.Component.SolidMechanics.FEM.HyperElastic",
        "Sofa.Component.SolidMechanics.Spring",
        "Sofa.Component.StateContainer",
        "Sofa.Component.Topology.Container.Dynamic",
        "Sofa.Component.Visual",
        "Sofa.GL.Component.Rendering2D",
        "Sofa.GL.Component.Rendering3D",
    ):
        try:
            SofaRuntime.importPlugin(plugin)
        except Exception as exc:
            print(f"  (plugin {plugin}: {exc})", file=sys.stderr)

    import parametricGripper as pg

    p_max = args.p_max if args.p_max is not None else pg.PRESSURE_MAX

    root = Sofa.Core.Node("root")
    pg.createScene(root)
    if args.dt is not None:
        root.dt.value = args.dt
        print(f"# dt override: {args.dt} s (scene default was {pg.DT})")
    Sofa.Simulation.init(root)

    finger = root.Finger1
    mo = finger.tetras

    inlet_idx = np.array(finger.inletROI.indices.value, dtype=int)
    tip_idx = np.array(finger.tipROI.indices.value, dtype=int)
    if len(inlet_idx) == 0 or len(tip_idx) == 0:
        print(f"ERROR: empty ROI (inlet={len(inlet_idx)}, tip={len(tip_idx)})",
              file=sys.stderr)
        return 2

    total_mass = (pg.DENSITY * pg.MESH_VOLUME_M3
                  if pg.MESH_VOLUME_M3 is not None else 0.04)
    sign = -1.0 if pg.SIGN_FLIP else 1.0
    cavity_constraint = finger.Cavity.SurfacePressureConstraint

    # Two-body McKibben: include the braid in the settle detector and
    # capture its positions alongside the bladder's.
    braid_mo = None
    ke_bodies = [(mo, total_mass)]
    if hasattr(finger, "Braid"):
        braid_mo = finger.Braid.braidMO
        ke_bodies.append((braid_mo, float(getattr(pg, "BRAID_TOTAL_MASS", 0.0))))
        print(f"# braid body detected: {len(braid_mo.position.value)} nodes "
              f"(KE included in settle detection; snapshots include braid)")

    # ---- Rest geometry (after a brief settle at p=0) -------------------
    cavity_constraint.value = [0.0]
    settle_substeps, settle_ke, settle_reason = step_to_equilibrium(
        root, ke_bodies, args.ke_threshold, args.settle_steps,
        disp_tol=args.disp_tol,
    )
    rest_tip = centroid(mo, tip_idx)
    rest_inlet = centroid(mo, inlet_idx)
    rest_vec = rest_tip - rest_inlet
    rest_len = float(np.linalg.norm(rest_vec))

    # Snapshot bookkeeping. We capture rest positions + tet connectivity once,
    # then append a position array per equilibrium plus convergence metadata
    # (so downstream tools can flag levels that hit max_substeps).
    snapshot_positions: list[np.ndarray] | None = None
    snapshot_substeps: list[int] | None = None
    snapshot_ke: list[float] | None = None
    snapshot_reason: list[str] | None = None
    braid_snapshot_positions: list[np.ndarray] | None = None
    braid_rest = None
    braid_edges = None
    rest_positions = None
    tet_nodes = None
    if args.snapshots:
        rest_positions = np.array(mo.position.value).copy()
        tet_nodes = np.array(finger.container.tetrahedra.value).copy()
        snapshot_positions = [rest_positions.copy()]
        snapshot_substeps = [settle_substeps]
        snapshot_ke = [settle_ke]
        snapshot_reason = [settle_reason]
        if braid_mo is not None:
            braid_rest = np.array(braid_mo.position.value).copy()
            braid_edges = np.array(finger.Braid.braidTopo.edges.value).copy()
            braid_snapshot_positions = [braid_rest.copy()]
        print(f"# snapshots: enabled, writing to {args.snapshots} "
              f"({rest_positions.shape[0]} nodes, {tet_nodes.shape[0]} tets)")
    print(f"# scene: numFingers={pg.numFingers}, USE_CUSTOM_CAD={pg.USE_CUSTOM_CAD}, "
          f"USE_TWO_MATERIAL={pg.USE_TWO_MATERIAL}")
    print(f"# rest_inlet = {rest_inlet}")
    print(f"# rest_tip   = {rest_tip}")
    print(f"# rest_len   = {rest_len*1000:.3f} mm "
          f"(settled in {settle_substeps} steps, KE={settle_ke:.2e})")
    print(f"# sweep:     0 -> {p_max} Pa in {args.n_steps} levels, "
          f"ke_threshold={args.ke_threshold}, max_substeps={args.max_substeps}")
    print(f"# {'p(Pa)':>10} {'tipx':>10} {'tipy':>10} {'tipz':>10} "
          f"{'|d| mm':>10} {'tilt_deg':>9} {'subs':>5} {'ke':>10}")

    # n_steps is the number of NONZERO increments.  The settled p=0 state is
    # an explicit archive level, so a request for 8 increments saves 9 states.
    pressures = np.linspace(0.0, p_max, args.n_steps + 1)
    nonzero_pressures = pressures[1:]
    failed_to_settle = 0

    # Line-buffered so the CSV is up-to-date on disk after each row even
    # if the process is killed mid-sweep.
    with open(args.out, "w", newline="", buffering=1) as f:
        writer = csv.writer(f)
        writer.writerow([
            "pressure_Pa",
            "tip_x", "tip_y", "tip_z",
            "disp_x", "disp_y", "disp_z",
            "disp_norm_mm",
            "tilt_deg",
            "substeps",
            "ke_final",
            "settle_reason",
        ])
        writer.writerow([
            "0.0000",
            *[f"{v:.6f}" for v in rest_tip],
            "0.000000", "0.000000", "0.000000",
            "0.0000", "0.000",
            settle_substeps, f"{settle_ke:.3e}", settle_reason,
        ])
        f.flush()

        diverged_at = None
        p_prev = 0.0
        for level_idx, p in enumerate(nonzero_pressures, 1):
            print(f"\n[{level_idx}/{len(nonzero_pressures)}] p = {p:.4f} Pa  "
                  f"(ramp {args.ramp_substeps} + settle "
                  f"max {args.max_substeps} substeps)", flush=True)
            # Quasi-static ramp: ease the pressure from the previous level
            # to the new target so the body never sees an impulsive load.
            ramp_used = 0
            ramp_diverged = False
            if args.ramp_substeps > 0:
                for k in range(args.ramp_substeps):
                    frac = (k + 1) / args.ramp_substeps
                    cavity_constraint.value = [
                        sign * (p_prev + frac * (p - p_prev))
                    ]
                    Sofa.Simulation.animate(root, root.dt.value)
                    ramp_used += 1
                    if (k + 1) % 10 == 0 or k == args.ramp_substeps - 1:
                        ke_now = total_kinetic_energy(ke_bodies)
                        if not np.isfinite(ke_now):
                            ramp_diverged = True
                            break
            p_prev = p
            if ramp_diverged:
                substeps, ke, reason = ramp_used, float("nan"), "diverged"
            else:
                cavity_constraint.value = [sign * p]
                substeps, ke, reason = step_to_equilibrium(
                    root, ke_bodies, args.ke_threshold, args.max_substeps,
                    disp_tol=args.disp_tol,
                )
                substeps += ramp_used
            if reason == "max_steps":
                failed_to_settle += 1
            if reason == "diverged" and diverged_at is None:
                diverged_at = p

            tip = centroid(mo, tip_idx)
            disp = tip - rest_tip
            disp_norm_mm = float(np.linalg.norm(disp)) * 1000.0
            tilt_deg = math.degrees(math.atan2(disp[1], disp[0]))

            writer.writerow([
                f"{p:.4f}",
                *[f"{v:.6f}" for v in tip],
                *[f"{v:.6f}" for v in disp],
                f"{disp_norm_mm:.4f}",
                f"{tilt_deg:.3f}",
                substeps,
                f"{ke:.3e}",
                reason,
            ])
            f.flush()
            print(f"  {p:>10.4f} {tip[0]:>10.5f} {tip[1]:>10.5f} {tip[2]:>10.5f} "
                  f"{disp_norm_mm:>10.3f} {tilt_deg:>8.2f} "
                  f"{substeps:>5} {ke:>10.2e}  ({reason})", flush=True)

            if snapshot_positions is not None:
                snapshot_positions.append(np.array(mo.position.value).copy())
                snapshot_substeps.append(substeps)
                snapshot_ke.append(ke)
                snapshot_reason.append(reason)
                if braid_snapshot_positions is not None:
                    braid_snapshot_positions.append(
                        np.array(braid_mo.position.value).copy()
                    )

            if reason == "diverged":
                print(f"\n  DIVERGENCE at p={p:.4f} Pa — aborting sweep "
                      f"(further levels would all be NaN). "
                      f"{level_idx} of {len(nonzero_pressures)} nonzero levels recorded.",
                      flush=True)
                break

    if args.snapshots and snapshot_positions:
        # The pure contract helper validates every body/convergence/braid row
        # and truncates planned levels after an early divergence.
        archive = build_zero_inclusive_archive(
            pressures,
            rest_positions,
            np.stack(snapshot_positions),
            tet_nodes,
            snapshot_substeps,
            snapshot_ke,
            snapshot_reason,
            braid_rest=braid_rest,
            braid_positions=(np.stack(braid_snapshot_positions)
                             if braid_snapshot_positions is not None else None),
            braid_edges=braid_edges,
        )
        np.savez(args.snapshots, **archive)
        n_max = sum(1 for r in snapshot_reason if r == "max_steps")
        print(f"\nWrote {args.snapshots} "
              f"({len(snapshot_positions)} snapshots, "
              f"{rest_positions.shape[0]} nodes each; "
              f"{n_max} of {len(snapshot_reason)} hit max_substeps)")

    print(f"\nWrote {args.out}")
    if failed_to_settle:
        print(f"NOTE: {failed_to_settle}/{len(nonzero_pressures)} nonzero levels did not "
              f"reach KE < {args.ke_threshold} within {args.max_substeps} "
              f"substeps (data still recorded; analyze for plateau noise)",
              file=sys.stderr)
    if diverged_at is not None:
        print(f"NOTE: sweep diverged at p={diverged_at:.4f} Pa "
              f"(cliff found; remaining pressure levels skipped)",
              file=sys.stderr)
    # Always return 0 on a successful run, regardless of partial settling
    # or divergence. The caller (e.g. run_experiment.py) can still proceed
    # to other variants and run jacobian.py on whatever data we collected.
    return 0


if __name__ == "__main__":
    # SOFA's DLL teardown deadlocks at interpreter shutdown on Windows —
    # background sweeps would otherwise finish their work and then hang
    # forever (no exit → no job notification). Exit hard on EVERY path,
    # including exceptions.
    try:
        rc = main()
    except BaseException:
        import traceback
        traceback.print_exc()
        rc = 1
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)
