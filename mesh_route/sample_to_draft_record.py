"""sample_to_draft_record.py - turn one finished actuator sample into a DRAFT
record the map can read. No SOFA.

A sample is one body driven one way by run_shared_body_drive.py. Its folder
holds some of these runs:

    free_levels/         several drive levels, tip free      (needed)
    tip_pushes*/         drive held, tip pushed both ways    (stiffness)
    tip_held_on_line/    tip place held, free to rotate      (blocked force)
    tip_held/            whole end face pinned               (blocked force,
                         used when there is no tip_held_on_line)
    ../bare_body/tip_pushes*/   the same pushes with no drive

Steps, all with the map's own unchanged scripts:
  1. pool the settled free levels into one snapshot file,
  2. jacobian.py on it (Jacobian, Gramians, modes),
  3. record_from_jacobian.py (free motion, shape, canonical frame),
  4. add the blocked and stiffness blocks from the held and pushed runs,
  5. check the result with actuator_record.validate.

Writes <out_root>/<id>.json and the working files under <out_root>/<id>/.
DRAFT ONLY: refuses to write into records/. A draft becomes a record only
with sign-off.

Inputs are stored in the units the scene took them in. Scene pressure and
cable values act as value/dt and their physical unit is not ruled on, so the
record's notes say so. The schema's unit list has no tesla, volt per metre or
plain strain; those samples carry the unit in the channel name and null in
the unit field.

    ..\\.venv\\Scripts\\python.exe sample_to_draft_record.py ^
        --sample experiments/actuation_methods/two_layer_strip/thermal_strain_in_top_layer ^
        --cad cads/actuation_methods/two_layer_strip --id two_layer_strip_thermal ^
        --physics thermal --input_name "strain in the top layer (no unit)"
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys

import numpy as np

import actuator_record
import record_from_jacobian as rfj

HERE = os.path.dirname(os.path.abspath(__file__))
SETTLED = ("below_threshold", "settled_disp")
AX = {"x": 0, "y": 1, "z": 2}


def read_csv(path):
    with open(path, "r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def complete_run(folder, prefix):
    """The finished run (it has states.npz) whose name starts with prefix.
    If a run was repeated under a longer name, the longest name wins."""
    if not os.path.isdir(folder):
        return None
    hits = [d for d in sorted(os.listdir(folder), key=len)
            if d.startswith(prefix) and not d.startswith(prefix + "_on_line")
            and os.path.exists(os.path.join(folder, d, "states.npz"))]
    return os.path.join(folder, hits[-1]) if hits else None


def pooled_free_levels(sample, runs):
    rest = tets = None
    lv, pos, why = [], [], []
    for run in runs:
        d = os.path.join(sample, run)
        a = np.load(os.path.join(d, "states.npz"), allow_pickle=True)
        rows = read_csv(os.path.join(d, "drive_states.csv"))
        if rest is None:
            rest, tets = a["rest_positions"], a["tet_nodes"]
        elif not (np.allclose(rest, a["rest_positions"], rtol=0, atol=1e-12)
                  and np.array_equal(tets, a["tet_nodes"])):
            raise SystemExit(f"{d}: mesh differs from the first run")
        if len(rows) != len(a["names"]):
            raise SystemExit(f"{d}: csv rows and saved states differ in number")
        for k, r in enumerate(rows):
            u = float(r["level"])
            if (r["state"] != "free" or u == 0.0
                    or r["settle_reason"] not in SETTLED
                    or not np.all(np.isfinite(a["positions"][k]))):
                continue
            lv.append(u)
            pos.append(a["positions"][k])
            why.append(r["settle_reason"])
    if len(lv) < 2:
        raise SystemExit(f"{sample}: fewer than 2 settled free levels")
    order = np.argsort(np.abs(lv))
    lv, pos, why = np.array(lv)[order], np.array(pos)[order], np.array(why)[order]
    keep = np.concatenate(([True], np.abs(np.diff(lv)) > 1e-9 * np.abs(lv).max()))
    lv, pos, why = lv[keep], pos[keep], why[keep]
    return (np.concatenate(([0.0], lv)), np.concatenate((rest[None], pos)),
            rest, tets, np.concatenate((["rest_saved"], why)))


def push_stiffness(run):
    """Level and k (N/m) per pushed axis from one tip_pushes run:
    k = 2 F / (travel between the plus and minus pushes)."""
    rows = read_csv(os.path.join(run, "drive_states.csv"))
    out = {}
    for ax in "xyz":
        p = [r for r in rows if r["state"] == f"push_plus_{ax}"]
        m = [r for r in rows if r["state"] == f"push_minus_{ax}"]
        if not p or not m:
            continue
        if p[-1]["settle_reason"] not in SETTLED or m[-1]["settle_reason"] not in SETTLED:
            continue
        f = abs(float(p[-1][f"test_force_{ax}_N"]))
        travel = (float(p[-1][f"disp_{ax}_mm"]) - float(m[-1][f"disp_{ax}_mm"])) * 1e-3
        out[ax] = 2.0 * f / travel
    return float(rows[0]["level"]), out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sample", required=True, help="the drive's folder")
    ap.add_argument("--cad", required=True)
    ap.add_argument("--id", required=True)
    ap.add_argument("--name", default=None)
    ap.add_argument("--physics", required=True)
    ap.add_argument("--input_name", required=True)
    ap.add_argument("--input_unit", default="",
                    help="one of the schema's unit strings, or leave out")
    ap.add_argument("--level_runs", default="free_levels",
                    help="comma list of runs, relative to --sample, whose "
                         "settled free levels are pooled")
    ap.add_argument("--held_run", default=None,
                    help="name of the held run to read, relative to --sample "
                         "(default tip_held_on_line, else tip_held)")
    ap.add_argument("--azimuth_deg", type=float, default=None,
                    help="fix where the record's +Y lies in the scene frame; "
                         "give 0 for a sample that does not bend")
    ap.add_argument("--out_root", default=os.path.join(
        "experiments", "actuation_methods", "draft_records"))
    args = ap.parse_args()

    out_root = os.path.abspath(args.out_root)
    canon = os.path.abspath(os.path.join(HERE, "records"))
    same_drive = (os.path.splitdrive(out_root)[0].lower()
                  == os.path.splitdrive(canon)[0].lower())
    if same_drive and os.path.normcase(os.path.commonpath(
            [out_root, canon])) == os.path.normcase(canon):
        raise SystemExit("drafts never go into records/")
    work = os.path.join(out_root, args.id)
    final = os.path.join(out_root, args.id + ".json")
    if os.path.exists(work) or os.path.exists(final):
        raise SystemExit(f"{args.id} already exists under {out_root}; "
                         "additive only, pick a new --id or --out_root")
    os.makedirs(work)

    # 1. settled free levels -> one snapshot file in jacobian.py's layout
    lv, pos, rest, tets, why = pooled_free_levels(
        args.sample, [r for r in args.level_runs.split(",") if r])
    snap = os.path.join(work, "settled_levels_snapshots.npz")
    np.savez(snap, pressures=lv, positions=pos, rest_positions=rest,
             tet_nodes=tets, settle_reason=why)

    # 2. the map's Jacobian and Gramian step
    jac = os.path.join(work, "jacobian.npz")
    r = subprocess.run([sys.executable, os.path.join(HERE, "jacobian.py"), snap,
                        "--out", jac, "--plot", os.path.join(work, "jacobian.png")],
                       capture_output=True, text=True, encoding="utf-8",
                       errors="replace")
    with open(os.path.join(work, "jacobian_log.txt"), "w", encoding="utf-8") as f:
        f.write(r.stdout + "\n--- stderr ---\n" + r.stderr)
    if r.returncode != 0 or not os.path.exists(jac):
        raise SystemExit(f"jacobian.py failed; see {work}/jacobian_log.txt")

    # 3. the map's record builder (free motion, shape, canonical frame)
    base = os.path.join(work, "record_free_and_shape_only.json")
    cmd = [jac, "--out", base, "--cad", args.cad, "--id", args.id,
           "--name", args.name or args.id, "--physics", args.physics,
           "--input_name", args.input_name, "--input_unit", args.input_unit,
           "--source_detail", "SOFA v24.06, run_shared_body_drive.py "
           "(actuation-method sample)"]
    if args.azimuth_deg is not None:
        cmd += ["--canonical-transverse-azimuth-deg", str(args.azimuth_deg)]
    if rfj.main(cmd) != 0:
        raise SystemExit("record_from_jacobian.py refused the sample")
    with open(base, "r", encoding="utf-8") as f:
        rec = json.load(f)
    if not args.input_unit:
        rec["input"]["channels"][0]["unit"] = None

    # the same scene-to-record rotation record_from_jacobian.py used
    d = np.load(jac)
    pressures, _, _, tip_disp, rest_index, _ = rfj.rest_relative_free_motion(d)
    small_index, _ = rfj.select_free_motion_indices(pressures, tip_disp, rest_index)
    R, az = rfj.canonical_rotation_matrix(tip_disp, small_index, args.azimuth_deg)
    # within about 2 degrees of a multiple of 90 about the length
    quarter = np.allclose(np.abs(R), np.round(np.abs(R)), atol=0.035)

    # 4a. blocked force: what the actuator pushes with when its tip is held
    held_dir = os.path.join(args.sample, "tip_held_on_line")
    how = ("tip held in place on a line of end-face nodes, free to rotate "
           "(run_shared_body_drive.py --hold_line)")
    pinned = ("every end-face node held, so rotation is blocked too "
              "(run_shared_body_drive.py --hold_tip)")
    if args.held_run:
        held_dir = os.path.join(args.sample, args.held_run)
        if not os.path.exists(os.path.join(held_dir, "drive_states.csv")):
            raise SystemExit(f"no held run at {held_dir}")
        if "on_line" not in args.held_run:
            how = pinned
    elif not os.path.exists(os.path.join(held_dir, "drive_states.csv")):
        held_dir = os.path.join(args.sample, "tip_held")
        how = pinned
    notes = []
    if os.path.exists(os.path.join(held_dir, "drive_states.csv")):
        with open(os.path.join(held_dir, "run_info.json")) as f:
            k_hold = json.load(f)["hold_stiffness_N_per_m_per_node"]
        rows = [x for x in read_csv(os.path.join(held_dir, "drive_states.csv"))
                if x["state"].startswith("held") and x["settle_reason"] in SETTLED]
        if rows:
            F = np.array([[float(x[f"hold_force_{a}_N"]) for a in "xyz"]
                          for x in rows]) @ R.T
            M = np.array([[float(x[f"hold_moment_{a}_Nm"]) for a in "xyz"]
                          for x in rows]) @ R.T
            rec["blocked"] = {
                "u": [[float(x["level"])] for x in rows],
                "force_axial_N": F[:, 0].tolist(),
                "force_transverse_N": np.linalg.norm(F[:, 1:], axis=1).tolist(),
                "force_N": F.tolist(),
                "moment_Nm": M.tolist(),
                "moment_method": "held-node forces x lever arm about the tip "
                                 "centre (not about the inlet)",
                "method": f"holding springs {k_hold:g} N/m per node; force the "
                          "actuator puts on the hold; " + how,
                "uncertainty": None}
            rec["scalars"]["blocked_force_N"] = float(
                np.linalg.norm(F, axis=1).max())
        else:
            notes.append("held run did not settle; no blocked block")
    else:
        notes.append("no held run; no blocked block")

    # 4b. stiffness with the drive held, and of the undriven body
    levels, ks = [], []
    bare = complete_run(os.path.join(os.path.dirname(
        os.path.normpath(args.sample)), "bare_body"), "tip_pushes")
    driven = complete_run(args.sample, "tip_pushes")
    for run in (bare, driven):
        if run:
            u, k = push_stiffness(run)
            if k:
                levels.append(u)
                ks.append(k)
    if ks and quarter:
        # scene y and z keep their roles when the rotation is a multiple of
        # 90 degrees about the length; which is which follows R
        y_src = "yz"[int(np.argmax(np.abs(R[1, 1:])))]
        z_src = "z" if y_src == "y" else "y"
        rec["stiffness"] = {
            "u": [[u] for u in levels],
            "k_N_per_m": [k.get(y_src) for k in ks],
            "k_out_of_plane_N_per_m": [k.get(z_src) for k in ks],
            "k_axial_N_per_m": [k.get("x") for k in ks],
            "direction": "transverse (+Y canonical)",
            "method": "two-sided tip push with the drive held "
                      "(run_shared_body_drive.py); k = 2 F / travel between "
                      "the two pushes; an axis not pushed is left out; "
                      "u = 0 is the undriven body",
            "uncertainty": None}
        # an axis that was not pushed in every run is left out (the map
        # tool reads a missing axis as no data; it cannot read a null)
        for key in ("k_N_per_m", "k_out_of_plane_N_per_m", "k_axial_N_per_m"):
            if any(v is None for v in rec["stiffness"][key]):
                del rec["stiffness"][key]
        first = rec["stiffness"].get("k_N_per_m", [None])[0]
        rec["scalars"]["stiffness_N_per_m"] = (
            first if first is not None else rec["stiffness"]["k_axial_N_per_m"][0])
    elif ks:
        notes.append(f"record frame is turned {az} from the scene frame, so "
                     "the pushed axes do not line up; no stiffness block")
    else:
        notes.append("no finished push run; no stiffness block")

    rec["metadata"]["notes"] = (
        "DRAFT, not canon. " + rec["metadata"].get("notes", "") +
        ". Inputs are in the units the scene took them in; scene pressure and "
        "cable values act as value/dt and their physical unit is not ruled on. "
        "Gravity off. Simulated magnitudes are comparative." +
        ("".join(" " + n + "." for n in notes)))

    # 5. check and save
    errors, warnings = actuator_record.validate(rec)
    for w in warnings:
        print("warning:", w)
    if errors:
        for e in errors:
            print("ERROR:", e, file=sys.stderr)
        return 1
    actuator_record.save(rec, final)
    print(f"Wrote {final} (tier {actuator_record.tier(rec)}); "
          f"{len(lv) - 1} free levels; frame {az}; "
          f"blocked {'yes' if rec.get('blocked') else 'no'}; "
          f"stiffness {'yes' if rec.get('stiffness') else 'no'}")
    for n in notes:
        print("note:", n)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
