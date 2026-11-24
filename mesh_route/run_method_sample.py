"""run_method_sample.py - free sweep of one crude actuation-method sample.

One runner for the actuation methods that run_sweep.py (positive pressure)
and run_cable_sweep.py (tendon pull) do not cover. The scene is the
unchanged parametricGripper.py (CAD chosen by the CAD_NAME env var); this
script only changes what drives it.

  --method pressure   cavity pressure in scene units; negative levels = vacuum.
                      Goes through SurfacePressureConstraint, so it carries
                      the scene's known value/dt quirk (see
                      check_pressure_force_scale.py).
  --method volume     pumped volume: the same constraint switched to
                      valueType='volumeGrowth'; levels are the commanded
                      cavity volume change (m^3). The enclosed volume is
                      also measured from the cavity triangles at each level.
  --method moment     motor torque (N m) about +z on the tip nodes, applied
                      as nodal forces recomputed every step so the net
                      moment about the tip centroid stays at the level and
                      the net force stays zero. Bends the beam.
  --method twist      as moment, but about the beam axis +x.
  --method magnetic   uniform field B (tesla) along +y on a body magnetised
                      along +x at rest (--magnetisation, A/m). Ideal
                      hard-magnetic model: energy -(F M).B per rest volume,
                      which gives constant nodal forces V (gradN.M) B.
  --method electric   Maxwell stress from a field through the thickness (y)
                      of the active layer (tets with centroid y > 0). Level
                      = permittivity x field^2 (Pa). Applied as the Maxwell
                      stress tensor p*diag(-1/2, +1/2, -1/2) held in the
                      material frame (field fixed in the body, not updated
                      for thinning).
  --method strain     imposed actuation strain in the active layer: uniaxial
                      contraction of the level value along x, volume
                      conserving. Applied as the equivalent stress
                      2 mu * diag(a, -a/2, -a/2) in the material frame.

The last five are plain nodal forces (a ConstantForceField, physical
newtons, no constraint), because SOFA has no native model for them.

Settle: the project's step_to_equilibrium, then --drift_steps more steps;
tip movement over those is logged as tip_drift_mm so a false plateau shows.

Run with the SOFA python from mesh_route/ (CAD_NAME set).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run_sweep import _bootstrap_sofa, centroid, step_to_equilibrium  # noqa: E402
from archive_contract import build_zero_inclusive_archive  # noqa: E402

UNITS = {
    "pressure": "scene pressure units (acts as value/dt in this scene)",
    "volume": "m^3 commanded cavity volume change",
    "moment": "N m about +z at the tip",
    "twist": "N m about +x at the tip",
    "magnetic": "tesla along +y (body magnetised along +x)",
    "electric": "Pa (permittivity x field^2) in the active layer",
    "strain": "contraction strain along x in the active layer (pure number)",
}


def rigid_rotation(rest: np.ndarray, cur: np.ndarray) -> np.ndarray:
    """Best-fit rotation taking rest points to current points, as a
    rotation vector in degrees (Kabsch)."""
    a = rest - rest.mean(0)
    b = cur - cur.mean(0)
    u, _, vt = np.linalg.svd(a.T @ b)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    ang = math.acos(max(-1.0, min(1.0, (np.trace(r) - 1.0) / 2.0)))
    if ang < 1e-12:
        return np.zeros(3)
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    axis /= (2.0 * math.sin(ang))
    return np.degrees(ang) * axis


def enclosed_volume(pos: np.ndarray, tris: np.ndarray) -> float:
    a, b, c = pos[tris[:, 0]], pos[tris[:, 1]], pos[tris[:, 2]]
    return abs(float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum()) / 6.0)


class TetLoads:
    """Rest-frame tet data and the nodal forces of a material-frame stress."""

    def __init__(self, rest: np.ndarray, tets: np.ndarray):
        self.tets = tets
        x = rest[tets]                                   # (T,4,3)
        dm = np.stack([x[:, 1] - x[:, 0], x[:, 2] - x[:, 0],
                       x[:, 3] - x[:, 0]], axis=2)       # columns = edges
        det = np.linalg.det(dm)
        self.vol = np.abs(det) / 6.0
        self.dm_inv = np.linalg.inv(dm)
        g = np.empty((len(tets), 4, 3))
        g[:, 1:, :] = self.dm_inv                        # rows = gradN_1..3
        g[:, 0, :] = -self.dm_inv.sum(axis=1)
        self.grad = g
        self.centroid = x.mean(axis=1)
        self.n_nodes = len(rest)

    def deformation(self, pos: np.ndarray) -> np.ndarray:
        x = pos[self.tets]
        ds = np.stack([x[:, 1] - x[:, 0], x[:, 2] - x[:, 0],
                       x[:, 3] - x[:, 0]], axis=2)
        return ds @ self.dm_inv

    def active_stress_forces(self, pos, s_star, mask) -> np.ndarray:
        """Nodal forces of an added second Piola-Kirchhoff stress s_star
        (3x3, material frame) in the masked tets: f_a = -V F S* gradN_a."""
        f = np.zeros((self.n_nodes, 3))
        fdef = self.deformation(pos)[mask]
        p = fdef @ s_star                                # first Piola (T,3,3)
        fa = -self.vol[mask, None, None] * np.einsum(
            "tij,taj->tai", p, self.grad[mask])          # (T,4,3)
        np.add.at(f, self.tets[mask].ravel(), fa.reshape(-1, 3))
        return f

    def magnetic_forces(self, m_vec, b_vec) -> np.ndarray:
        """f_a = sum_tets V (gradN_a . M) B  (constant in the deformation)."""
        f = np.zeros((self.n_nodes, 3))
        w = self.vol[:, None] * (self.grad @ m_vec)      # (T,4)
        fa = w[:, :, None] * np.asarray(b_vec)[None, None, :]
        np.add.at(f, self.tets.ravel(), fa.reshape(-1, 3))
        return f


def couple_forces(pos, tip_idx, torque_vec, n_nodes) -> np.ndarray:
    """Nodal forces on the tip nodes with zero net force and net moment
    about the tip centroid equal to torque_vec along its own direction."""
    f = np.zeros((n_nodes, 3))
    t = float(np.linalg.norm(torque_vec))
    if t == 0.0:
        return f
    that = torque_vec / t
    r = pos[tip_idx] - pos[tip_idx].mean(0)
    raw = np.cross(that, r)
    m = float(np.cross(r, raw).sum(0) @ that)
    f[tip_idx] = raw * (t / m) * np.sign(torque_vec @ that)
    return f


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--method", required=True, choices=sorted(UNITS))
    ap.add_argument("--levels", required=True,
                    help="comma-separated nonzero input levels, in order")
    ap.add_argument("--out_dir", required=True)
    ap.add_argument("--magnetisation", type=float, default=1.0e5,
                    help="A/m along +x at rest (magnetic only)")
    ap.add_argument("--ke_threshold", type=float, default=1e-9)
    ap.add_argument("--max_substeps", type=int, default=400)
    ap.add_argument("--settle_steps", type=int, default=50)
    ap.add_argument("--ramp_substeps", type=int, default=50)
    ap.add_argument("--drift_steps", type=int, default=20)
    ap.add_argument("--disp_tol", type=float, default=1e-7)
    args = ap.parse_args()
    levels_nz = [float(v) for v in args.levels.split(",") if v.strip()]
    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, "sweep.csv")
    if os.path.exists(csv_path):
        print(f"ERROR: {csv_path} exists; additive only, pick a new folder",
              file=sys.stderr)
        return 2

    import Sofa
    import Sofa.Core
    import Sofa.Simulation
    import SofaRuntime
    for plugin in ("SofaPython3", "SoftRobots",
                   "Sofa.Component.MechanicalLoad"):
        try:
            SofaRuntime.importPlugin(plugin)
        except Exception as exc:
            print(f"  (plugin {plugin}: {exc})", file=sys.stderr)
    import parametricGripper as pg

    root = Sofa.Core.Node("root")
    pg.createScene(root)
    finger = root.Finger1
    mo = finger.tetras
    nodal = args.method in ("moment", "twist", "magnetic", "electric", "strain")
    npz = np.load(pg.ACTUATOR_NPZ, allow_pickle=True)
    n_npz = len(npz["points"])

    cff = None
    if nodal:
        cff = finger.addObject("ConstantForceField", name="methodLoad",
                               indices=list(range(n_npz)),
                               forces=[[0.0, 0.0, 0.0]] * n_npz)
    constraint = None
    if args.method in ("pressure", "volume"):
        constraint = finger.Cavity.SurfacePressureConstraint
        if args.method == "volume":
            constraint.valueType.value = "volumeGrowth"
            # With this scene's cavity orientation a positive constraint
            # pressure shrinks the cavity, so volume control is a runaway
            # (measured: error doubles every step) unless the constraint's
            # normals are flipped. Set here at run time; the scene file is
            # unchanged and pressure-mode runs are not affected.
            constraint.flipNormal.value = True
    Sofa.Simulation.init(root)

    inlet_idx = np.array(finger.inletROI.indices.value, dtype=int)
    tip_idx = np.array(finger.tipROI.indices.value, dtype=int)
    if len(inlet_idx) == 0 or len(tip_idx) == 0:
        print("ERROR: empty ROI", file=sys.stderr)
        return 2
    n_nodes = len(mo.position.value)
    if nodal and n_nodes != n_npz:
        print(f"ERROR: scene has {n_nodes} nodes, actuator.npz {n_npz}",
              file=sys.stderr)
        return 2
    total_mass = (pg.DENSITY * pg.MESH_VOLUME_M3
                  if pg.MESH_VOLUME_M3 is not None else 0.04)
    bodies = [(mo, total_mass)]
    sign = -1.0 if pg.SIGN_FLIP else 1.0
    dt = float(root.dt.value)

    e_body = float(pg.YOUNG_MODULUS_BODY)
    mat_path = os.path.join(os.path.dirname(pg.ACTUATOR_NPZ), "materials.json")
    if os.path.exists(mat_path):
        with open(mat_path) as f:
            e_body = float(json.load(f).get("young_modulus_body", e_body))
    nu = float(pg.POISSON_RATIO)
    mu = e_body / (2.0 * (1.0 + nu))

    state = {"level": 0.0}
    tl = None
    active = None
    mag_unit = None

    def loads_for(level: float) -> np.ndarray:
        pos = np.array(mo.position.value)
        if args.method == "moment":
            return couple_forces(pos, tip_idx, np.array([0, 0, level]), n_nodes)
        if args.method == "twist":
            return couple_forces(pos, tip_idx, np.array([level, 0, 0]), n_nodes)
        if args.method == "magnetic":
            return mag_unit * level
        if args.method == "electric":
            s = level * np.diag([-0.5, 0.5, -0.5])
            return tl.active_stress_forces(pos, s, active)
        if args.method == "strain":
            s = 2.0 * mu * level * np.diag([1.0, -0.5, -0.5])
            return tl.active_stress_forces(pos, s, active)
        raise ValueError(args.method)

    def apply(level: float) -> None:
        state["level"] = level
        if constraint is not None:
            constraint.value = [sign * level if args.method == "pressure"
                                else level]
        elif abs(level) > 0.0 or True:
            cff.forces.value = loads_for(level).tolist()

    if nodal:
        class LoadUpdater(Sofa.Core.Controller):
            """Refresh the nodal forces from the current shape every step."""

            def onAnimateBeginEvent(self, _event):
                if tl is None:      # rest phase: no load yet
                    return
                cff.forces.value = loads_for(state["level"]).tolist()

        root.addObject(LoadUpdater(name="loadUpdater"))

    # ---- rest ---------------------------------------------------------
    if constraint is not None:
        constraint.value = [0.0]
    sub0, ke0, why0 = step_to_equilibrium(root, bodies, args.ke_threshold,
                                          args.settle_steps,
                                          disp_tol=args.disp_tol)
    rest = np.array(mo.position.value).copy()
    tets = np.array(finger.container.tetrahedra.value).copy()
    rest_tip = centroid(mo, tip_idx)
    rest_inlet = centroid(mo, inlet_idx)
    body_len = float(rest[:, 0].max() - rest[:, 0].min())
    if nodal:
        tl = TetLoads(rest, tets)
        active = tl.centroid[:, 1] > 0.0
        mag_unit = tl.magnetic_forces(
            np.array([args.magnetisation, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]))

    cav_tris = None
    cav_mo = None
    if constraint is not None:
        cav_mo = finger.Cavity.cavity
        cav_tris = np.array(finger.Cavity.topo.triangles.value, dtype=int)
        v_rest = enclosed_volume(np.array(cav_mo.position.value), cav_tris)
    else:
        v_rest = float("nan")

    def readout(name):
        try:
            v = constraint.getData(name).value
            return float(np.ravel(v)[0])
        except Exception:
            return float("nan")

    info = {
        "method": args.method, "input_units": UNITS[args.method],
        "cad_name": pg.CAD_NAME, "levels_nonzero": levels_nz,
        "dt_s": dt, "body_length_m": body_len,
        "body_young_modulus_Pa": e_body, "poisson_ratio": nu,
        "n_nodes": int(n_nodes), "n_tets": int(len(tets)),
        "n_tip_nodes": int(len(tip_idx)), "n_clamped_nodes": int(len(inlet_idx)),
        "rest_tip_m": rest_tip.tolist(), "rest_inlet_m": rest_inlet.tolist(),
        "rest_cavity_volume_m3": v_rest,
        "settle": {"ke_threshold": args.ke_threshold,
                   "max_substeps": args.max_substeps,
                   "ramp_substeps": args.ramp_substeps,
                   "drift_steps": args.drift_steps, "disp_tol": args.disp_tol},
        "argv": sys.argv, "started": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    if nodal:
        info["active_layer_tets"] = int(active.sum())
        info["active_layer_volume_m3"] = float(tl.vol[active].sum())
        info["body_volume_m3"] = float(tl.vol.sum())
        info["shear_modulus_Pa"] = mu
    if args.method == "magnetic":
        info["magnetisation_A_per_m"] = args.magnetisation
        info["net_force_per_tesla_N"] = mag_unit.sum(0).tolist()
    with open(os.path.join(args.out_dir, "run_info.json"), "w") as f:
        json.dump(info, f, indent=2)
    print(f"# {args.method}: body length {body_len*1e3:.2f} mm, E {e_body:.3g} "
          f"Pa, {n_nodes} nodes, {len(tets)} tets, tip nodes {len(tip_idx)}",
          flush=True)

    snap_pos, snap_sub, snap_ke, snap_why = [rest.copy()], [sub0], [ke0], [why0]
    cols = ["input", "tip_x", "tip_y", "tip_z", "disp_x", "disp_y", "disp_z",
            "disp_norm_mm", "tilt_deg", "rot_x_deg", "rot_y_deg", "rot_z_deg",
            "rot_angle_deg", "substeps", "ke_final", "settle_reason",
            "tip_drift_mm", "cavity_volume_change_m3",
            "constraint_pressure_readout", "constraint_volume_growth_readout",
            "net_load_N", "applied_tip_moment_Nm"]
    rc = 0
    with open(csv_path, "w", newline="", buffering=1) as f:
        w = csv.writer(f)
        w.writerow(cols)
        w.writerow(["0", *[f"{v:.6f}" for v in rest_tip], "0", "0", "0", "0",
                    "0", "0", "0", "0", "0", sub0, f"{ke0:.3e}", why0, "0",
                    "0" if constraint is not None else "nan",
                    "nan", "nan", "0", "0"])
        prev = 0.0
        for k, lv in enumerate(levels_nz, 1):
            diverged = False
            for j in range(args.ramp_substeps):
                apply(prev + (j + 1) / args.ramp_substeps * (lv - prev))
                Sofa.Simulation.animate(root, dt)
                if not np.all(np.isfinite(np.array(mo.position.value))):
                    diverged = True
                    break
            prev = lv
            if diverged:
                sub, ke, why, drift = args.ramp_substeps, float("nan"), "diverged", float("nan")
            else:
                apply(lv)
                sub, ke, why = step_to_equilibrium(
                    root, bodies, args.ke_threshold, args.max_substeps,
                    disp_tol=args.disp_tol)
                sub += args.ramp_substeps
                tip_a = centroid(mo, tip_idx)
                for _ in range(args.drift_steps):
                    Sofa.Simulation.animate(root, dt)
                drift = float(np.linalg.norm(centroid(mo, tip_idx) - tip_a)) * 1e3
                sub += args.drift_steps
            pos = np.array(mo.position.value).copy()
            tip = pos[tip_idx].mean(0)
            disp = tip - rest_tip
            rot = rigid_rotation(rest[tip_idx], pos[tip_idx]) if why != "diverged" else np.full(3, np.nan)
            dvol = (enclosed_volume(np.array(cav_mo.position.value), cav_tris)
                    - v_rest) if constraint is not None else float("nan")
            p_read = readout("pressure") if constraint is not None else float("nan")
            v_read = readout("volumeGrowth") if constraint is not None else float("nan")
            if nodal and why != "diverged":
                fl = loads_for(lv)
                net = float(np.linalg.norm(fl.sum(0)))
                r = pos[tip_idx] - tip
                mom = float(np.linalg.norm(np.cross(r, fl[tip_idx]).sum(0)))
            else:
                net, mom = float("nan"), float("nan")
            w.writerow([f"{lv:.9g}", *[f"{v:.6f}" for v in tip],
                        *[f"{v:.6f}" for v in disp],
                        f"{np.linalg.norm(disp)*1e3:.4f}",
                        f"{math.degrees(math.atan2(disp[1], disp[0])):.3f}",
                        *[f"{v:.3f}" for v in rot],
                        f"{np.linalg.norm(rot):.3f}", sub, f"{ke:.3e}", why,
                        f"{drift:.5f}", f"{dvol:.6e}", f"{p_read:.6e}",
                        f"{v_read:.6e}", f"{net:.4e}", f"{mom:.4e}"])
            print(f"[{k}/{len(levels_nz)}] input {lv:.6g}: tip disp "
                  f"{np.linalg.norm(disp)*1e3:.3f} mm ({disp[0]*1e3:.2f}, "
                  f"{disp[1]*1e3:.2f}, {disp[2]*1e3:.2f}), rot "
                  f"{np.linalg.norm(rot):.2f} deg, {sub} steps, {why}, drift "
                  f"{drift:.5f} mm, dV {dvol:.3e}, p_read {p_read:.4e}",
                  flush=True)
            snap_pos.append(pos)
            snap_sub.append(sub)
            snap_ke.append(ke)
            snap_why.append(why)
            if why == "diverged":
                rc = 3
                break

    archive = build_zero_inclusive_archive(
        [0.0] + levels_nz, rest, np.stack(snap_pos), tets, snap_sub, snap_ke,
        snap_why)
    archive["tip_indices"] = tip_idx
    archive["inlet_indices"] = inlet_idx
    np.savez(os.path.join(args.out_dir, "snapshots.npz"), **archive)
    print("[complete]", flush=True)
    return rc


if __name__ == "__main__":
    try:
        rc = main()
    except BaseException:
        import traceback
        traceback.print_exc()
        rc = 1
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)
