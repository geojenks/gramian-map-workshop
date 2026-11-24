"""run_shared_body_drive.py - one body, many drives: free shape, tip stiffness
and blocked force for each actuation method applied to the SAME mesh.

The actuation-method samples so far put five of the drives on a plain solid
beam. This runner puts every drive on one shared body (the pneunet mesh for
bending, the bellows mesh for stretching or shortening), so the drives are
compared with the geometry held fixed. A new file beside run_method_sample.py
and run_method_sample_tip_load.py; those, and the scene, are unchanged. The
load models are imported from run_method_sample.py.

  --drive none            nothing driven (the bare body, for the baseline)
  --drive pressure        chamber pressure set (scene units, value/dt quirk)
  --drive volume          chamber volume set (m^3)
  --drive moment          couple about +z on the tip nodes (N m)
  --drive magnetic        body magnetised along +x; field along --field_axis
                          (y bends, x stretches or shortens) (tesla)
  --drive electric        Maxwell stress level (Pa) in the active layer; field
                          along --field_axis (y, or radial = through the wall
                          of a body of revolution about x)
  --drive strain          actuation strain along x in the active layer
  --drive cable_tension   tendon tension set (scene units), CABLE_MODE=force
  --drive cable_length    tendon pull length set (m), CABLE_MODE=displacement

The active layer is the tets whose rest centroid lies inside --layer_x and
--layer_y (both "lo,hi" in metres; default everything). The tet count and
volume are logged.

Without --hold_tip: for each level ramp, settle and save the free state;
unless --no_push, then with the drive held push the tip with +F and -F along
each of --push_axes and settle each time (stiffness = 2F / travel).
With --hold_tip: the scene's tip spring (BLOCK_TIP=1) holds the tip nodes at
rest; the force and moment of the body on the hold are logged per level.

Writes <out_dir>/drive_states.csv, run_info.json, states.npz. Refuses to
overwrite. Run with the SOFA python from mesh_route/ (CAD_NAME set).
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

DRIVES = ["none", "pressure", "volume", "moment", "magnetic", "electric",
          "strain", "cable_tension", "cable_length"]
NODAL = ("moment", "magnetic", "electric", "strain")
AXIS = {"x": 0, "y": 1, "z": 2}


def lohi(text):
    a, b = (float(v) for v in text.split(","))
    return a, b


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--drive", required=True, choices=DRIVES)
    ap.add_argument("--levels", required=True, help="comma list; 0 allowed")
    ap.add_argument("--out_dir", required=True)
    ap.add_argument("--field_axis", default="y", choices=["x", "y", "radial"])
    ap.add_argument("--magnetisation", type=float, default=8.1e4)
    ap.add_argument("--layer_x", default="-1,1")
    ap.add_argument("--layer_y", default="-1,1")
    ap.add_argument("--cables", default="1", help="tendons driven, e.g. 1,2,3,4")
    ap.add_argument("--push_axes", default="yz")
    ap.add_argument("--no_push", action="store_true")
    ap.add_argument("--hold_tip", action="store_true")
    ap.add_argument("--hold_stiffness", type=float, default=1e6)
    ap.add_argument("--hold_line", choices=["y", "z"], default=None,
                    help="hold only the end-face nodes on the line through "
                         "the tip centre across this axis' normal (y: the "
                         "line y = centre), so the tip is held in place but "
                         "free to rotate, as when pressed on a wall")
    ap.add_argument("--hold_line_tol", type=float, default=0.0005)
    ap.add_argument("--hold_sideways", choices=["y", "z"], default=None,
                    help="stop the same mid-height line of end-face nodes in "
                         "this one direction only, as a tip pressing on a "
                         "flat frictionless surface: it may slide along the "
                         "body's length and across, and rotate")
    ap.add_argument("--f_test", type=float, default=0.01)
    ap.add_argument("--ke_threshold", type=float, default=1e-14)
    ap.add_argument("--max_substeps", type=int, default=1500)
    ap.add_argument("--settle_steps", type=int, default=50)
    ap.add_argument("--ramp_substeps", type=int, default=50)
    ap.add_argument("--drift_steps", type=int, default=20)
    ap.add_argument("--disp_tol", type=float, default=1e-7)
    ap.add_argument("--plateau_rel_tol", type=float, default=0.03,
                    help="stop when kinetic energy varies by less than this "
                         "fraction over 15 steps; 0 turns that stop off, for "
                         "a body that creeps slowly at steady speed")
    args = ap.parse_args()

    levels = [float(v) for v in args.levels.split(",") if v.strip()]
    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, "drive_states.csv")
    if os.path.exists(csv_path):
        print(f"ERROR: {csv_path} exists; additive only", file=sys.stderr)
        return 2
    os.environ["BLOCK_TIP"] = "1" if args.hold_tip else "0"
    if args.hold_tip:
        os.environ["TIP_BLOCK_STIFFNESS"] = repr(args.hold_stiffness)
    if args.drive == "cable_tension":
        os.environ["CABLE_MODE"] = "force"
    elif args.drive == "cable_length":
        os.environ["CABLE_MODE"] = "displacement"

    from run_sweep import _bootstrap_sofa, centroid, step_to_equilibrium
    from run_method_sample import (TetLoads, couple_forces, enclosed_volume,
                                   rigid_rotation)
    _bootstrap_sofa()
    import Sofa
    import Sofa.Core
    import Sofa.Simulation
    import SofaRuntime
    for plugin in ("SofaPython3", "SoftRobots", "Sofa.Component.MechanicalLoad"):
        try:
            SofaRuntime.importPlugin(plugin)
        except Exception as exc:
            print(f"  (plugin {plugin}: {exc})", file=sys.stderr)
    import parametricGripper as pg

    root = Sofa.Core.Node("root")
    pg.createScene(root)
    finger = root.Finger1
    mo = finger.tetras
    drive = args.drive
    nodal = drive in NODAL
    n_npz = len(np.load(pg.ACTUATOR_NPZ, allow_pickle=True)["points"])

    test_cff = None
    if sum(bool(v) for v in (args.hold_tip, args.hold_line,
                             args.hold_sideways)) > 1:
        print("ERROR: give one of --hold_tip, --hold_line, --hold_sideways",
              file=sys.stderr)
        return 2
    line_axis = args.hold_line or args.hold_sideways
    held = args.hold_tip or line_axis is not None
    hold_dirs = np.ones(3)
    if args.hold_sideways:
        hold_dirs = np.zeros(3)
        hold_dirs[AXIS[args.hold_sideways]] = 1.0
    line_pts = None
    if line_axis:
        pts0 = np.load(pg.ACTUATOR_NPZ, allow_pickle=True)["points"]
        x_end = pts0[:, 0].max()
        face = np.where(pts0[:, 0] > x_end - 1e-6)[0]
        k = AXIS[line_axis]
        mid = 0.5 * (pts0[face, k].min() + pts0[face, k].max())
        line_pts = face[np.abs(pts0[face, k] - mid) < args.hold_line_tol]
        if len(line_pts) < 2:
            print("ERROR: fewer than 2 nodes on the hold line", file=sys.stderr)
            return 2
        # 0/1, not True/False: SOFA cannot read Python booleans here and
        # then silently holds all three directions
        tip_line_hold = finger.addObject(
            "RestShapeSpringsForceField", name="tipLineHold",
            points=[int(v) for v in line_pts],
            stiffness=args.hold_stiffness,
            activeDirections=[int(v) for v in hold_dirs])
    if not held and not args.no_push:
        test_cff = finger.addObject("ConstantForceField", name="tipTestForce",
                                    indices="@tipROI.indices",
                                    totalForce=[0.0, 0.0, 0.0])
    load_cff = None
    if nodal:
        load_cff = finger.addObject("ConstantForceField", name="methodLoad",
                                    indices=list(range(n_npz)),
                                    forces=[[0.0, 0.0, 0.0]] * n_npz)
    chamber = None
    try:
        chamber = finger.Cavity.SurfacePressureConstraint
    except Exception:
        chamber = None
    if drive in ("pressure", "volume") and chamber is None:
        print("ERROR: no chamber in this scene", file=sys.stderr)
        return 2
    if drive == "volume":
        chamber.valueType.value = "volumeGrowth"
        chamber.flipNormal.value = True

    cables = {}
    for k in range(1, 9):
        node = finger.getChild(f"DriveCable{k}")
        if node is not None:
            cables[k] = node.CableConstraint
    driven = [int(v) for v in args.cables.split(",")] if drive.startswith("cable") else []
    for k in driven:
        if k not in cables:
            print(f"ERROR: no DriveCable{k} in scene", file=sys.stderr)
            return 2
    if cables and not drive.startswith("cable"):
        # A tendon mesh run with another drive: the tendons must be slack,
        # which only the tension-set mode gives at value 0.
        print("ERROR: this CAD has tendons; use a cable drive", file=sys.stderr)
        return 2

    Sofa.Simulation.init(root)
    if line_axis:
        got = [bool(v) for v in tip_line_hold.activeDirections.value]
        if got != [bool(v) for v in hold_dirs]:
            print(f"ERROR: hold directions read as {got}, wanted "
                  f"{[bool(v) for v in hold_dirs]}", file=sys.stderr)
            return 2
        print(f"# hold directions (x, y, z) as SOFA read them: {got}")

    tip_idx = np.array(finger.tipROI.indices.value, dtype=int)
    inlet_idx = np.array(finger.inletROI.indices.value, dtype=int)
    n_nodes = len(mo.position.value)
    if nodal and n_nodes != n_npz:
        print(f"ERROR: scene has {n_nodes} nodes, npz {n_npz}", file=sys.stderr)
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
    mu = e_body / (2.0 * (1.0 + float(pg.POISSON_RATIO)))

    st = {"level": 0.0, "tl": None, "mask": None, "mag": None, "stress": None}

    def loads_for(level):
        pos = np.array(mo.position.value)
        if drive == "moment":
            return couple_forces(pos, tip_idx, np.array([0.0, 0.0, level]), n_nodes)
        if drive == "magnetic":
            return st["mag"] * level
        return st["tl"].active_stress_forces(pos, level * st["stress"], st["mask"])

    def apply(level):
        st["level"] = level
        if drive == "pressure":
            chamber.value = [sign * level]
        elif drive == "volume":
            chamber.value = [level]
        elif drive.startswith("cable"):
            for k in driven:
                cables[k].value = [level]
        elif nodal and st["tl"] is not None:
            load_cff.forces.value = loads_for(level).tolist()

    if nodal:
        class LoadUpdater(Sofa.Core.Controller):
            def onAnimateBeginEvent(self, _event):
                if st["tl"] is not None:
                    load_cff.forces.value = loads_for(st["level"]).tolist()

        root.addObject(LoadUpdater(name="loadUpdater"))

    def settle():
        sub, ke, why = step_to_equilibrium(root, bodies, args.ke_threshold,
                                           args.max_substeps,
                                           disp_tol=args.disp_tol,
                        plateau_rel_tol=args.plateau_rel_tol)
        a = centroid(mo, tip_idx)
        for _ in range(args.drift_steps):
            Sofa.Simulation.animate(root, dt)
        drift = float(np.linalg.norm(centroid(mo, tip_idx) - a)) * 1e3
        return sub + args.drift_steps, why, drift

    def read(obj, name):
        try:
            return float(np.ravel(obj.getData(name).value)[0])
        except Exception:
            return float("nan")

    # ---- rest ---------------------------------------------------------
    if chamber is not None:
        chamber.value = [0.0]
    for c in cables.values():
        c.value = [0.0]
    step_to_equilibrium(root, bodies, args.ke_threshold, args.settle_steps,
                        disp_tol=args.disp_tol,
                        plateau_rel_tol=args.plateau_rel_tol)
    rest = np.array(mo.position.value).copy()
    tets = np.array(finger.container.tetrahedra.value).copy()
    rest_tip = rest[tip_idx].mean(0)
    cav_mo = cav_tris = None
    v_rest = float("nan")
    if chamber is not None:
        cav_mo = finger.Cavity.cavity
        cav_tris = np.array(finger.Cavity.topo.triangles.value, dtype=int)
        v_rest = enclosed_volume(np.array(cav_mo.position.value), cav_tris)

    hold_idx = line_pts if line_pts is not None else tip_idx
    info = {"drive": drive, "hold_tip": bool(args.hold_tip),
            "hold_line": args.hold_line or "",
            "hold_sideways": args.hold_sideways or "",
            "hold_directions_xyz": [bool(v) for v in hold_dirs] if held else [],
            "n_hold_nodes": int(len(hold_idx)) if held else 0,
            "hold_stiffness_N_per_m_per_node": args.hold_stiffness,
            "f_test_N": args.f_test, "push_axes": args.push_axes,
            "cad_name": pg.CAD_NAME, "levels": levels, "dt_s": dt,
            "n_nodes": int(n_nodes), "n_tets": int(len(tets)),
            "n_tip_nodes": int(len(tip_idx)),
            "n_clamped_nodes": int(len(inlet_idx)),
            "rest_tip_m": rest_tip.tolist(),
            "body_length_m": float(np.ptp(rest[:, 0])),
            "body_young_modulus_Pa": e_body, "shear_modulus_Pa": mu,
            "rest_cavity_volume_m3": v_rest, "cables_driven": driven,
            "cable_mode": os.environ.get("CABLE_MODE", ""),
            "settle": {"ke_threshold": args.ke_threshold,
                       "max_substeps": args.max_substeps,
                       "drift_steps": args.drift_steps,
                       "disp_tol": args.disp_tol,
                       "plateau_rel_tol": args.plateau_rel_tol},
            "argv": sys.argv, "started": time.strftime("%Y-%m-%d %H:%M:%S")}
    if nodal:
        tl = TetLoads(rest, tets)
        c = tl.centroid
        (x0, x1), (y0, y1) = lohi(args.layer_x), lohi(args.layer_y)
        mask = (c[:, 0] >= x0) & (c[:, 0] <= x1) & (c[:, 1] >= y0) & (c[:, 1] <= y1)
        st["mask"] = mask
        info.update({"body_volume_m3": float(tl.vol.sum()),
                     "active_layer_tets": int(mask.sum()),
                     "active_layer_volume_m3": float(tl.vol[mask].sum()),
                     "layer_x_m": [x0, x1], "layer_y_m": [y0, y1],
                     "field_axis": args.field_axis})
        if drive == "magnetic":
            b = np.zeros(3)
            b[AXIS[args.field_axis]] = 1.0
            st["mag"] = tl.magnetic_forces(
                np.array([args.magnetisation, 0.0, 0.0]), b)
            info["magnetisation_A_per_m"] = args.magnetisation
        elif drive == "electric":
            if args.field_axis == "radial":
                n = c[mask].copy()
                n[:, 0] = 0.0
                n /= np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]
            else:
                n = np.zeros((int(mask.sum()), 3))
                n[:, AXIS[args.field_axis]] = 1.0
            # Maxwell stress of a field along n, as in run_method_sample.py:
            # n n^T - I/2 (pulls the faces together, spreads in the plane).
            st["stress"] = np.einsum("ti,tj->tij", n, n) - 0.5 * np.eye(3)
        elif drive == "strain":
            st["stress"] = 2.0 * mu * np.diag([1.0, -0.5, -0.5])
        st["tl"] = tl
    with open(os.path.join(args.out_dir, "run_info.json"), "w") as f:
        json.dump(info, f, indent=2)
    print(f"# {drive} on {pg.CAD_NAME}: {n_nodes} nodes, {len(tets)} tets, "
          f"tip nodes {len(tip_idx)}, active tets {info.get('active_layer_tets')}",
          flush=True)

    cols = ["level", "state", "test_force_x_N", "test_force_y_N",
            "test_force_z_N", "disp_x_mm", "disp_y_mm", "disp_z_mm",
            "rot_x_deg", "rot_y_deg", "rot_z_deg", "substeps", "settle_reason",
            "tip_drift_mm", "cavity_volume_change_m3",
            "chamber_pressure_readout", "chamber_volume_growth_readout",
            "cable_force_readout", "cable_displacement_readout_m",
            "hold_force_x_N", "hold_force_y_N", "hold_force_z_N",
            "hold_moment_x_Nm", "hold_moment_y_Nm", "hold_moment_z_Nm",
            "net_drive_load_N"]
    names, states = [], []
    rc = 0
    with open(csv_path, "w", newline="", buffering=1) as f:
        w = csv.writer(f)
        w.writerow(cols)

        def record(level, state, force):
            sub, why, drift = settle()
            pos = np.array(mo.position.value).copy()
            ok = bool(np.all(np.isfinite(pos)))
            d = (pos[tip_idx].mean(0) - rest_tip) * 1e3
            rot = rigid_rotation(rest[tip_idx], pos[tip_idx]) if ok else np.full(3, np.nan)
            dvol = (enclosed_volume(np.array(cav_mo.position.value), cav_tris)
                    - v_rest) if chamber is not None else float("nan")
            hf = hm = np.full(3, np.nan)
            if held:
                fn = (args.hold_stiffness * hold_dirs
                      * (pos[hold_idx] - rest[hold_idx]))
                hf = fn.sum(0)
                hm = np.cross(rest[hold_idx] - rest_tip, fn).sum(0)
            cab = cables[driven[0]] if driven else None
            net = (float(np.linalg.norm(loads_for(level).sum(0)))
                   if nodal and ok else float("nan"))
            w.writerow([f"{level:.9g}", state, *[f"{v:.6g}" for v in force],
                        *[f"{v:.5f}" for v in d], *[f"{v:.4f}" for v in rot],
                        sub, why if ok else "diverged", f"{drift:.5f}",
                        f"{dvol:.6e}", f"{read(chamber, 'pressure'):.6e}",
                        f"{read(chamber, 'volumeGrowth'):.6e}",
                        f"{read(cab, 'force'):.6e}",
                        f"{read(cab, 'displacement'):.6e}",
                        *[f"{v:.6e}" for v in hf], *[f"{v:.6e}" for v in hm],
                        f"{net:.4e}"])
            names.append(f"{level:.9g}|{state}")
            states.append(pos)
            print(f"  level {level:g} {state}: tip ({d[0]:.4f}, {d[1]:.4f}, "
                  f"{d[2]:.4f}) mm, rot ({rot[0]:.2f}, {rot[1]:.2f}, {rot[2]:.2f}) "
                  f"deg, {sub} steps, {why}, drift {drift:.5f} mm, hold F "
                  f"({hf[0]:.4e}, {hf[1]:.4e}, {hf[2]:.4e}) N, hold M z "
                  f"{hm[2]:.4e}, cable F {read(cab, 'force'):.4e} pull "
                  f"{read(cab, 'displacement'):.4e}, p {read(chamber, 'pressure'):.4e}",
                  flush=True)
            return ok

        pushes = []
        if test_cff is not None:
            for ax in args.push_axes:
                for sgn, word in ((1.0, "plus"), (-1.0, "minus")):
                    fv = [0.0, 0.0, 0.0]
                    fv[AXIS[ax]] = sgn * args.f_test
                    pushes.append((f"push_{word}_{ax}", tuple(fv)))
            pushes.append(("free_again", (0.0, 0.0, 0.0)))
        prev = 0.0
        for lv in levels:
            for j in range(args.ramp_substeps):
                apply(prev + (j + 1) / args.ramp_substeps * (lv - prev))
                Sofa.Simulation.animate(root, dt)
            prev = lv
            apply(lv)
            if not record(lv, ("held_sideways_only" if args.hold_sideways
                               else "held_on_line" if args.hold_line
                               else "held")
                          if held else "free", (0, 0, 0)):
                rc = 1
                break
            for state, force in pushes:
                test_cff.totalForce.value = list(map(float, force))
                if not record(lv, state, force):
                    rc = 1
                    break
            if rc:
                break
    np.savez(os.path.join(args.out_dir, "states.npz"), names=np.array(names),
             positions=np.array(states), rest_positions=rest, tet_nodes=tets,
             tip_indices=tip_idx, inlet_indices=inlet_idx)
    print("[complete]", flush=True)
    return rc


if __name__ == "__main__":
    rc = 1
    try:
        rc = main()
    except BaseException:
        import traceback
        traceback.print_exc()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)
