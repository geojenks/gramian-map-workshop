"""Record a Wave_manip preset as a leader file (data/leaders/<name>.json).

    python tools/record_leader.py --wave-manip ../wave_manip \
        --preset w12c_cup_analytic_lock --duration 7

Runs the preset headless on Wave_manip's canonical bridge (unified_v6,
loaded through its own regression harness's `fresh_bridge`, read-only) and
logs, every control tick: the object's pose, each station's position, whether
it is in contact, and the contact force it puts on the object (from MuJoCo's
contacts between the station geoms and the object). Needs Wave_manip, its
sim/ submodule and mujoco.

A leader file is the general format: any source that can say where the
contacts are, when, and with what force (an authored gait, a human demo,
another policy) can write one.
"""
import argparse
import hashlib
import importlib.util
import io
import json
import os
import subprocess
from contextlib import redirect_stdout

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "data", "leaders")
EVERY = 4            # keep every 4th control tick (the bridge ticks at 8 ms)


def load(path, name):
    s = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(s)
    s.loader.exec_module(m)
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", required=True)
    ap.add_argument("--preset", required=True)
    ap.add_argument("--duration", type=float, default=7.0)
    ap.add_argument("--speed-scale", type=float, default=1.0,
                    help="scale every primitive's wave speed (wave.c) and "
                         "flow rate (flows[].omega) together: same gait, slower")
    ap.add_argument("--amp", type=float, default=None,
                    help="override each wave's stroke amplitude (wave.amp, "
                         "manifold units; 0.15 is the standard 15 mm)")
    ap.add_argument("--soft-landing", type=float, default=None,
                    help="Wave_manip's softLanding fraction for every cup wave "
                         "(the share of the break given to a velocity-matched "
                         "touchdown; needs Wave_manip with soft landing)")
    ap.add_argument("--takeoff", action="store_true",
                    help="with --soft-landing: smooth the lift-off as well")
    ap.add_argument("--ball-radius", type=float, default=None,
                    help="override the sim's ball radius (m); sim/run_wave_"
                         "unified.py's module constant, not a bridge option")
    ap.add_argument("--ball-mass", type=float, default=None,
                    help="override the sim's ball mass (kg), likewise")
    ap.add_argument("--station-r", type=float, default=None,
                    help="scene station_r (m), the station ring's radius")
    ap.add_argument("--stations", type=int, default=None,
                    help="number of contact stations (scene n_contacts and "
                         "each wave's nWave), e.g. the hand's finger count")
    ap.add_argument("--settle", type=float, default=0.5,
                    help="seconds dropped from the start")
    a = ap.parse_args()
    import mujoco
    wm = os.path.abspath(a.wave_manip)
    cwd = os.getcwd()
    os.chdir(wm)
    rr = load(os.path.join(wm, "experiment_log", "scripts", "run_regression.py"),
              "_rr")
    ppath = os.path.join(wm, "unified_v6", "examples", "presets",
                         a.preset + ".json")
    with open(ppath) as f:
        preset = json.load(f)
    if a.speed_scale != 1.0:
        for prim in preset["control"]["primitives"]:
            w = prim.get("wave") or {}
            if "c" in w:
                w["c"] = w["c"] * a.speed_scale
            for fl in prim.get("flows") or []:
                if "omega" in fl:
                    fl["omega"] = fl["omega"] * a.speed_scale
    if a.amp is not None:
        for prim in preset["control"]["primitives"]:
            if "wave" in prim:
                prim["wave"]["amp"] = a.amp
    if a.soft_landing is not None:
        for prim in preset["control"]["primitives"]:
            if "wave" in prim:
                prim["wave"]["softLanding"] = {"fraction": a.soft_landing,
                                               "takeoff": bool(a.takeoff)}
    if a.station_r is not None:
        preset.setdefault("scene", {})["station_r"] = a.station_r
    if a.ball_radius is not None or a.ball_mass is not None:
        # the bridge imports run_wave_unified inside run_session, so the
        # same module object (from sys.modules) carries these values
        import sys as _sys
        _sys.path.insert(0, os.path.join(wm, "sim"))
        import run_wave_unified as _rwu
        if a.ball_radius is not None:
            _rwu.ball_radius = a.ball_radius
        if a.ball_mass is not None:
            _rwu.ball_mass = a.ball_mass
    if a.stations:
        preset.setdefault("scene", {})["n_contacts"] = a.stations
        for prim in preset["control"]["primitives"]:
            if "wave" in prim and "nWave" in prim["wave"]:
                prim["wave"]["nWave"] = a.stations
    lid = (a.preset + ("" if a.speed_scale == 1.0 else f"_x{a.speed_scale:g}")
           + ("" if a.amp is None else f"_a{a.amp:g}")
           + ("" if a.soft_landing is None else
              f"_sl{a.soft_landing:g}" + ("t" if a.takeoff else ""))
           + ("" if a.ball_radius is None else f"_R{a.ball_radius * 1e3:g}")
           + ("" if not a.stations else f"_n{a.stations}"))
    buf = io.StringIO()
    with redirect_stdout(buf):
        srv = rr.fresh_bridge()
        for k, v in (preset.get("scene") or {}).items():
            if k in srv.SCENE_DEFAULTS:
                srv.SCENE_DEFAULTS[k] = v
        live = srv.Live()
        for k, v in (preset.get("scene") or {}).items():
            live.scene[k] = v
        live.update_control(preset["control"])

    rows, meta = [], {}
    f6 = np.zeros(6)

    def sink(m, d):
        fb = live.get_feedback()
        if fb is None or not fb.get("stations"):
            return
        meta.setdefault("tick", 0)
        meta["tick"] += 1
        if meta["tick"] % EVERY or d.time < a.settle:
            return
        if "ball_body" not in meta:
            jb = int(m.jnt_bodyid[0])
            meta["ball_body"] = jb
            meta["ball_geoms"] = {g for g in range(m.ngeom) if m.geom_bodyid[g] == jb}
            meta["mass"] = float(m.body_mass[jb])
            meta["frame"] = fb["frame"]
        st = fb["stations"]
        pos = np.array(st["pos"])
        # station body = the body nearest each reported station position
        if "station_bodies" not in meta:
            cand = [b for b in range(m.nbody) if b != meta["ball_body"]]
            meta["station_bodies"] = [min(cand, key=lambda b: np.linalg.norm(d.xpos[b] - p))
                                      for p in pos]
        sb = {b: i for i, b in enumerate(meta["station_bodies"])}
        n = len(pos)
        force, cpt, cnorm = np.zeros((n, 3)), [None] * n, [None] * n
        gf = (fb.get("grip_force") or {}).get("contacts") or []
        if gf:
            # the bridge's own contact report (analytic and native routes)
            meta["force_source"] = "bridge grip_force.contacts"
            for c in gf:
                i = int(c["station"])
                force[i] = c["force_n"]
                cpt[i], cnorm[i] = c["pos"], c["normal"]
        else:
            meta["force_source"] = "MuJoCo contacts (mj_contactForce)"
            for c in range(d.ncon):
                con = d.contact[c]
                g1, g2 = int(con.geom1), int(con.geom2)
                b1, b2 = int(m.geom_bodyid[g1]), int(m.geom_bodyid[g2])
                if g1 in meta["ball_geoms"] and b2 in sb:
                    i, sign = sb[b2], -1.0   # force on geom2 is +frame; on ball -
                elif g2 in meta["ball_geoms"] and b1 in sb:
                    i, sign = sb[b1], 1.0
                else:
                    continue
                mujoco.mj_contactForce(m, d, c, f6)
                force[i] += sign * (con.frame.reshape(3, 3).T @ f6[:3])
                cpt[i] = np.round(con.pos, 6).tolist()
        # contact points on the object's surface (a sphere): the analytic
        # route reports the station centre, a station radius outside it
        Rb = float(meta["frame"]["ball_radius"])
        oc = np.array(d.xpos[meta["ball_body"]])
        for i in range(n):
            if cpt[i] is not None:
                v = np.array(cpt[i]) - oc
                cpt[i] = np.round(oc + Rb * v / np.linalg.norm(v), 6).tolist()
        rows.append({
            "t": round(float(d.time), 4),
            "obj_pos": np.round(d.xpos[meta["ball_body"]], 6).tolist(),
            "obj_quat": np.round(d.xquat[meta["ball_body"]], 6).tolist(),
            "station_pos": np.round(pos, 6).tolist(),
            "contact": [bool(x) for x in st["contact"]],
            "force_on_object_N": np.round(force, 5).tolist(),
            "contact_point": cpt,
            "contact_normal": cnorm,
        })

    with redirect_stdout(buf):
        srv.run_session(live, headless=True, duration=a.duration,
                        _frame_sink=sink)
    os.chdir(cwd)
    head = subprocess.run(["git", "-C", wm, "rev-parse", "HEAD"],
                          capture_output=True, text=True).stdout.strip()
    with open(ppath, "rb") as f:
        psha = hashlib.sha256(f.read()).hexdigest()
    leader = {
        "schema": "gramian-workshop/leader@1",
        "id": lid,
        "name": preset.get("title", a.preset) + (
            "" if a.speed_scale == 1.0 else f" -- slowed x{a.speed_scale:g}"),
        "kind": "authored wave (Wave_manip unified_v6 preset)",
        "source": {"repo": "geojenks/Wave_manip", "commit": head,
                   "preset": os.path.relpath(ppath, wm), "preset_sha256": psha,
                   "recorder": "tools/record_leader.py",
                   "speed_scale": a.speed_scale,
                   "stations_override": a.stations,
                   "amp_override": a.amp,
                   "ball_radius_override_m": a.ball_radius,
                   "ball_mass_override_kg": a.ball_mass,
                   "station_r_override_m": a.station_r,
                   "soft_landing": (None if a.soft_landing is None else
                                    {"fraction": a.soft_landing,
                                     "takeoff": bool(a.takeoff)}),
                   "duration_s": a.duration, "settle_dropped_s": a.settle,
                   "every_nth_tick": EVERY,
                   "force_source": meta.get("force_source")},
        "object": {"shape": "sphere",
                   "radius_m": float(meta["frame"]["ball_radius"]),
                   "mass_kg": meta["mass"]},
        "n_stations": len(rows[0]["station_pos"]) if rows else 0,
        "frames": rows,
        "caveat": "SIMULATION ONLY -- comparative, not predictive",
    }
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, lid + ".json")
    with open(p, "w", encoding="utf-8") as f:
        json.dump(leader, f, separators=(",", ":"))
    n_con = [sum(r["contact"]) for r in rows]
    print(f"wrote {p}: {len(rows)} frames, {leader['n_stations']} stations, "
          f"contacts per frame {min(n_con)}-{max(n_con)}, ball R "
          f"{leader['object']['radius_m'] * 1e3:.1f} mm, mass {meta['mass']:.4f} kg")


if __name__ == "__main__":
    main()
