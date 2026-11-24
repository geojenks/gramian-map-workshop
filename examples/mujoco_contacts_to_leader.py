"""Your own MuJoCo run as a task file: write its contacts as a leader.

    python examples/mujoco_contacts_to_leader.py               # 4 pads
    python examples/mujoco_contacts_to_leader.py --pads 3 --out outputs/my_mujoco_task_3pads.json
    python tools/check_leader.py outputs/my_mujoco_task.json

To use your own model: load it in place of `build_scene`, pass your
object's body id (a free body with one sphere geom) and, per station, the
geom id (or list of geom ids) of that fingertip, pad or link; call
`station_contacts` once per frame you keep, after `mj_forward`; and fill
`object`, `source` and `stations` from your model. Nothing else changes.
docs/TASK_FROM_MUJOCO.md walks through it; docs/TASK_FILE_FORMAT.md is the
format.

The scene here is a ball resting under gravity on a ring of fixed pads
below its centre. Its numbers are for illustration only.
Needs mujoco and numpy only.
"""
import argparse
import json
import math
import os

import mujoco
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCHEMA = "gramian-workshop/leader@1"

# --- the example scene's numbers: replace with your own -----------------------
BALL_RADIUS_M = 0.03
BALL_MASS_KG = 0.05
PAD_RADIUS_M = 0.008
CONTACT_BELOW_DEG = 40.0    # each contact sits this far below the ball's equator
FRICTION = 1.0
TIMESTEP_S = 0.002
BALL_CENTRE_M = (0.0, 0.0, 0.1)
DROP_M = 0.002              # the ball starts this far above its touching height
REST_SPEED = 1e-4           # m/s and rad/s: below this the ball counts as at rest
REST_HOLD_S = 0.2           # ... for this long
SETTLE_MAX_S = 10.0


def build_scene(n_pads):
    """A free ball above a ring of n_pads fixed spherical pads, z up.
    Returns (model, data, ball body id, list of pad geom ids)."""
    R, r = BALL_RADIUS_M, PAD_RADIUS_M
    el = math.radians(CONTACT_BELOW_DEG)
    c = np.array(BALL_CENTRE_M)
    pads = []
    for i in range(n_pads):
        phi = 2.0 * math.pi * i / n_pads
        u = np.array([math.cos(el) * math.cos(phi), math.cos(el) * math.sin(phi),
                      -math.sin(el)])
        p = c + (R + r) * u
        pads.append(f'<geom name="pad_{i}" type="sphere" size="{r}" '
                    f'pos="{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}" rgba=".8 .3 .3 1"/>')
    xml = f"""
<mujoco model="ball_on_pads">
  <option timestep="{TIMESTEP_S}" gravity="0 0 -9.81"/>
  <default><geom friction="{FRICTION} 0.005 0.0001" condim="3"/></default>
  <worldbody>
    {''.join(pads)}
    <body name="ball" pos="{c[0]} {c[1]} {c[2] + DROP_M}">
      <freejoint/>
      <geom name="ball" type="sphere" size="{R}" mass="{BALL_MASS_KG}"/>
    </body>
  </worldbody>
</mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    ball = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ball")
    pad_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"pad_{i}")
               for i in range(n_pads)]
    return model, data, ball, pad_ids


def object_sphere_radius(model, object_body_id):
    """Radius of the object's sphere geom. The format takes a sphere only."""
    r = [float(model.geom_size[g][0]) for g in range(model.ngeom)
         if model.geom_bodyid[g] == object_body_id
         and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE]
    if len(r) != 1:
        raise SystemExit(f"the object body {object_body_id} has {len(r)} sphere geoms; "
                         "the task file needs exactly one sphere")
    return r[0]


def station_contacts(model, data, object_body_id, station_geom_ids, stats=None):
    """One frame's contacts, in the task-file fields, from MuJoCo's contacts.

    model, data       call mj_forward first, so the contacts, the forces and
                      the object's position all belong to the same instant
    object_body_id    the object's body (a free body with one sphere geom)
    station_geom_ids  one entry per station: a geom id, or a list of geom
                      ids that together are that station
    stats             optional dict, for checking: it gains counts of the
                      object's contacts with no station and of which geom
                      of each pair the object was, and the largest overlap
                      and raw contact-point distance off the sphere

    Returns a dict with the frame's contact, contact_point,
    force_on_object_N and contact_normal lists, one entry per station.
    """
    n = len(station_geom_ids)
    owner = {}
    for i, g in enumerate(station_geom_ids):
        for gg in (g if isinstance(g, (list, tuple)) else [g]):
            owner[int(gg)] = i
    R = object_sphere_radius(model, object_body_id)
    centre = np.array(data.xpos[object_body_id])
    force = np.zeros((n, 3))
    w_point = np.zeros((n, 3))     # contact points weighted by normal force
    w_normal = np.zeros((n, 3))
    weight = np.zeros(n)
    hits = [[] for _ in range(n)]  # unweighted, for a station with zero force
    f6 = np.zeros(6)
    for c in range(data.ncon):
        con = data.contact[c]
        g1, g2 = int(con.geom1), int(con.geom2)
        on1 = model.geom_bodyid[g1] == object_body_id
        on2 = model.geom_bodyid[g2] == object_body_id
        if not (on1 or on2):
            continue                       # a contact not on the object
        other = g2 if on1 else g1
        if other not in owner:
            # TRAP: the object touches something that is not a station (a
            # table, a palm, another link). That contact carries load the
            # file will not show, so the forces will not balance.
            if stats is not None:
                stats["object_contacts_not_on_a_station"] = (
                    stats.get("object_contacts_not_on_a_station", 0) + 1)
            continue
        i = owner[other]
        # TRAP: mj_contactForce gives the force in the contact's own frame:
        # f6[0] along the normal, f6[1:3] along the two tangents. The rows
        # of contact.frame are those three axes in world coordinates, so
        # frame.T rotates the force into the world.
        mujoco.mj_contactForce(model, data, c, f6)
        frame = np.array(con.frame).reshape(3, 3)
        f_world = frame.T @ f6[:3]
        # TRAP: that force, and the frame's normal (frame[0]), point from
        # geom1 to geom2: it is the force on geom2. The file wants the force
        # ON the object, and the normal INTO the object, so flip both when
        # the object is geom1. Which geom comes first is MuJoCo's choice.
        sign = 1.0 if on2 else -1.0
        f_on_obj = sign * f_world
        n_into = sign * frame[0]
        # TRAP: a station can have several contacts with the object (a
        # mesh fingertip, a box pad): sum their forces.
        force[i] += f_on_obj
        wn = max(float(f6[0]), 0.0)
        w_point[i] += wn * np.array(con.pos)
        w_normal[i] += wn * n_into
        weight[i] += wn
        hits[i].append((np.array(con.pos), n_into))
        if stats is not None:
            key = "object_was_geom2" if on2 else "object_was_geom1"
            stats[key] = stats.get(key, 0) + 1
            # contact.pos lies midway through the overlap, not on the sphere
            off = abs(float(np.linalg.norm(np.array(con.pos) - centre)) - R)
            stats["raw_point_off_surface_max_m"] = max(
                stats.get("raw_point_off_surface_max_m", 0.0), off)
            stats["overlap_max_m"] = max(stats.get("overlap_max_m", 0.0),
                                         max(0.0, -float(con.dist)))
    out = {"contact": [], "contact_point": [], "force_on_object_N": [],
           "contact_normal": []}
    for i in range(n):
        if not hits[i]:
            out["contact"].append(False)
            out["contact_point"].append(None)
            out["force_on_object_N"].append([0.0, 0.0, 0.0])
            out["contact_normal"].append(None)
            continue
        if weight[i] > 0:
            p, nn = w_point[i] / weight[i], w_normal[i] / weight[i]
        else:                              # touching with zero force
            p = np.mean([h[0] for h in hits[i]], axis=0)
            nn = np.mean([h[1] for h in hits[i]], axis=0)
        # TRAP: the format's contact point is on the sphere's surface. Put
        # the (force-weighted) contact point there, along the line from the
        # centre. With several contacts spread over a pad this is an average.
        v = p - centre
        p_surf = centre + R * v / np.linalg.norm(v)
        out["contact"].append(True)
        out["contact_point"].append([round(float(x), 6) for x in p_surf])
        out["force_on_object_N"].append([round(float(x), 6) for x in force[i]])
        out["contact_normal"].append([round(float(x), 6) for x in nn / np.linalg.norm(nn)])
    # TRAP: the weight. The follow check adds no gravity; the forces must
    # carry it. MuJoCo's contact forces on a free object already do (and,
    # in a moving run, its acceleration too): write them as they are. If
    # your object hangs on a joint, a weld, a tendon or a mocap body
    # instead, that carries load the contacts do not show.
    return out


def settle(model, data):
    """Step until the ball is at rest; stop with a message if it never is."""
    still = 0.0
    while data.time < SETTLE_MAX_S:
        mujoco.mj_step(model, data)
        still = still + model.opt.timestep if np.max(np.abs(data.qvel)) < REST_SPEED else 0.0
        if still >= REST_HOLD_S:
            return float(data.time)
    raise SystemExit(f"the ball did not come to rest within {SETTLE_MAX_S} s "
                     f"(speed {np.max(np.abs(data.qvel)):.3g}); nothing written")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--pads", type=int, default=4, help="number of pads (3 or more)")
    ap.add_argument("--frames", type=int, default=40, help="frames recorded")
    ap.add_argument("--duration", type=float, default=1.0,
                    help="seconds recorded, after the ball is at rest")
    ap.add_argument("--out", default=os.path.join(ROOT, "outputs", "my_mujoco_task.json"),
                    help="the task file to write; its name is the leader's id")
    a = ap.parse_args()
    if a.pads < 3:
        ap.error("--pads must be 3 or more (fewer cannot hold the ball)")
    if a.frames < 2 or a.duration <= 0:
        ap.error("--frames must be 2 or more and --duration above 0")
    if os.path.exists(a.out):
        ap.error(f"{a.out} exists; choose another --out (nothing is overwritten)")

    model, data, ball, pad_ids = build_scene(a.pads)
    t_rest = settle(model, data)
    t0 = float(data.time)
    frames, stats = [], {}
    for k in range(a.frames):
        t_want = t0 + a.duration * k / (a.frames - 1)
        while data.time < t_want - 1e-9:
            mujoco.mj_step(model, data)
        # after mj_step the contacts and body positions are still those from
        # before the step; mj_forward makes them match data.time
        mujoco.mj_forward(model, data)
        fr = station_contacts(model, data, ball, pad_ids, stats)
        frames.append({
            "t": round(float(data.time), 6),
            "obj_pos": [round(float(x), 6) for x in data.xpos[ball]],
            "obj_quat": [round(float(x), 6) for x in data.xquat[ball]],   # w, x, y, z
            "contact": fr["contact"],
            "contact_point": fr["contact_point"],
            "force_on_object_N": fr["force_on_object_N"],
            "station_pos": [[round(float(x), 6) for x in data.geom_xpos[g]] for g in pad_ids],
            "contact_normal": fr["contact_normal"],
        })

    lid = os.path.splitext(os.path.basename(a.out))[0]
    R = object_sphere_radius(model, ball)
    mass = float(model.body_mass[ball])
    leader = {
        "schema": SCHEMA,
        "id": lid,
        "name": f"Ball resting on {a.pads} fixed pads (MuJoCo example)",
        "kind": "simulation (MuJoCo, examples/mujoco_contacts_to_leader.py)",
        "source": {
            "script": "examples/mujoco_contacts_to_leader.py",
            "mujoco_version": mujoco.__version__,
            "force_source": "MuJoCo contacts (mj_contactForce), summed per station",
            "settings": {
                "pads": a.pads, "frames": a.frames, "duration_s": a.duration,
                "ball_radius_m": BALL_RADIUS_M, "ball_mass_kg": BALL_MASS_KG,
                "pad_radius_m": PAD_RADIUS_M, "contact_below_equator_deg": CONTACT_BELOW_DEG,
                "friction": FRICTION, "timestep_s": TIMESTEP_S, "drop_m": DROP_M,
                "rest_speed": REST_SPEED, "rest_hold_s": REST_HOLD_S,
                "solref_solimp": "MuJoCo defaults",
            },
            "settled_after_s": round(t_rest, 6),
        },
        "object": {"shape": "sphere", "radius_m": R, "mass_kg": mass},
        "n_stations": a.pads,
        "stations": [f"pad_{i}" for i in range(a.pads)],
        "frame": {"units": "m", "gravity": [0.0, 0.0, -1.0],
                  "note": "MuJoCo world frame, z up; the pads go round z"},
        "frames": frames,
        "caveat": "SIMULATION ONLY -- comparative, not predictive",
    }
    out = os.path.abspath(a.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(leader, f, indent=1)

    # what to look at: the net force should point up and equal m g, and by
    # symmetry each pad should carry m g / N of it upwards
    g = float(-model.opt.gravity[2])
    nets = np.array([np.sum(fr["force_on_object_N"], axis=0) for fr in frames])
    up = np.array([[f[2] for f in fr["force_on_object_N"]] for fr in frames])
    print(f"wrote {out}")
    print(f"  {a.pads} pads, {len(frames)} frames over {a.duration:g} s; ball at rest "
          f"after {t_rest:.3f} s; ball R {R * 1e3:g} mm, mass {mass:g} kg")
    print(f"  net force on the ball / m g: vertical {np.median(nets[:, 2]) / (mass * g):.4f}"
          f" (median), sideways at most {np.max(np.abs(nets[:, :2])) / (mass * g):.2e}")
    print("  each pad's upward force / (m g / pads), median: "
          + ", ".join(f"{x:.4f}" for x in np.median(up, axis=0) / (mass * g / a.pads)))
    print(f"  raw MuJoCo contact points lay up to "
          f"{stats.get('raw_point_off_surface_max_m', 0) * 1e3:.3f} mm off the sphere "
          f"(overlap up to {stats.get('overlap_max_m', 0) * 1e3:.3f} mm); written on it")
    print(f"  pair order: the ball was geom2 in {stats.get('object_was_geom2', 0)} "
          f"contacts, geom1 in {stats.get('object_was_geom1', 0)}")
    if stats.get("object_contacts_not_on_a_station"):
        print(f"  WARNING: {stats['object_contacts_not_on_a_station']} contacts on the "
              "ball were with geoms that are not stations; their load is not in the file")


if __name__ == "__main__":
    main()
