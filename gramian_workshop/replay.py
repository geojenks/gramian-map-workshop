"""Replay a stored follow result in MuJoCo (issue 14; docs/WORKSHOP_PLAN.md
step 4, item 6): drive the hand along the joint path the check solved and
watch whether the object follows the leader.

What the check says
-------------------
follow.py checks each leader frame on its own: it poses the hand's fingers
on the object (one joint vector "q" per touching finger, stored in the
result) and asks whether the stored contact forces can be made from there.
Nothing moves. The replay makes the check's assumption literal: the hand
is moved through those poses, in order, at the leader's own times, and
the object is left free. If the check is right, the object should move as
the leader's object did; if the poses do not carry it, it falls, drifts
or stays still.

The model
---------
One MuJoCo model: the hand's own file (its collision shapes, the ones the
check's overlap test reads: every geom with contype or conaffinity set),
plus a free sphere with the leader's radius and mass, placed by the
result's placement (rotation Rw, translation t, about the leader's first
object centre; follow.Leader), plus a ghost sphere (a mocap body, no
collision) that draws where the leader's object is. Gravity is the
leader's, carried into the hand's frame by Rw (follow._leader_gravity);
a leader whose gravity cannot be read is refused. Friction is the
leader's mu (follow.py's default 1.0 when the file states none), set on
the sphere with priority so the pair uses it.

Only hand-object pairs collide (the hand is kept from colliding with
itself, as the check does not test that). The contact solver settings are
the hand file's own (the Allegro file sets an elliptic cone and impratio
10, not MuJoCo's defaults); the record keeps them (contact_options).

The drive
---------
"servo" (default): the hand is driven as a hand is, through the file's
own joint position actuators (Allegro: force = kp (target - q), kp 1
N m/rad), told the path's joint angles as targets at every step, and
MuJoCo's contact model is left as the file sets it. A position servo
alone sags under load by torque / kp (0.03 N m of contact load is 1.7
degrees of Allegro joint, a few millimetres at the tip), so the torques
the check itself computed are fed forward (Feedforward, through
qfrc_applied on the hand's joints): the contact load J^T f at each checked
frame (follow's own Jacobian, mj_jac at the stored pad point on the stored
phalanx with the hand at the stored q, times the leader's force on the
object), linear in time between checked frames and zero where the finger
is off; the hand's weight at the path's pose (mj_rne at rest, follow's own
weight term, so at a checked frame it equals the stored weight_Nm); the
stored motion torque (follow's fitted M qdd + C qd, where it was fitted);
and the file's own joint damping times the path's joint velocity (the
hand file's dof_damping stands for the servo's velocity loop, and a
position servo alone trails its path by damping x speed / kp, for Allegro
0.1 s of path, which is 27 mm at a fingertip on this leader; the velocity
term is computed from the path alone). Nothing is tuned: the drive is the
check's own plan applied, and the object's response is the test. The
feedforward is open loop: it is the torque for a hand already moving at
the path's speed, applied from the first step to a hand at rest, so at the
start the damping and motion parts press the fingers into the object
(seen: the first frame's contact forces about three times the leader's
with all parts, about the leader's with the standing parts alone). That
is reported, not smoothed: replay(feedforward="standing") feeds forward
the contact load and weight only, the parts that do not depend on the
hand moving, and is the named sensitivity (run_replay.py --feedforward
standing). Before the object is put in, the hand alone is driven along
the path (track_check) and its joint and fingertip errors recorded: a
drive that cannot follow its own path says nothing about the object.

"kinematic": the hand is moved as if it could not be pushed. Its joints
are given a large armature (ARMATURE, kg m^2), its links' weight is
cancelled (gravity compensation), its actuators are made inert, and at
every step the hand's joint positions and velocities are set from the
path. A position-prescribed finger's contact force is then penetration
times contact stiffness alone, so this drive's outcome rests wholly on the
pad setting below. It is kept as a named sensitivity (run_replay.py
--drive kinematic), not the demonstration.

Pads
----
The check lets a posed finger sink into the object by the task's allowed
overlap (3 mm by default, follow.overlap_allowance), standing for pad
compliance; here the touching fingers sit sunk by up to that at the
checked frames (geometry_check). A stiff contact turns that sink into a
squeeze: with the object pinned to the leader's path (tried), the servo
fingers pressed like the leader's stations between checked frames but by
newtons at them. So the hand-object contact is given a stiffness through
MuJoCo's soft constraint (pad_contact: critically damped, constant
impedance, calibrated on a sphere of the object's mass on a plane so the
record holds the stiffness reached). Which stiffness (replay(pads=...)):
  leader     the headline: what the leader's own contacts showed,
             leader_pad_stiffness, the median over its recorded contacts
             of force over the station's sink (pad radius + object radius
             - |station - centre|). Measured data of the leader the check
             was fed, chosen before any replay was run with it.
  allowance  pad_stiffness, the leader's median contact force over the
             allowed overlap: the softest pad consistent with the check
  default    MuJoCo's default contact (solref 0.02 s), whose stiffness
             scales with the object's mass, so it is not rigid for a
             light object: the record holds what it reached
  given      a stiffness in N/m (--pad-k)
All but the headline are sensitivities to run and report beside it, never
a correction: a replay is not re-run with another pad to change its
verdict.

The joint path
--------------
q(t) is the hand's full joint vector: the rest pose (Hand.base_q) with
each touching finger's stored q written in at its joints, linear in time
between the checked frames (the result holds every n-th leader frame),
held before the first and after the last. A finger the check left off
(no stored q) is a free choice; the "lift" option names it:

  "leader"  (default) the finger moves from the pose where it left the
            object to the pose where it lands again, linearly in joint
            space, and is kept clear of the object by the leader's own
            lift: how far the leader's station for that finger backs off
            the object while it is off (|station_pos - obj_pos| - R,
            less the station's distance while on), read from the leader
            per frame. The clearance is enforced by Jacobian steps on the
            finger's nearest collision shape (clear_finger). A finger that
            is off at the start starts at its first landing pose; one off
            at the end holds its last pose, lifted.
  "rest"    the finger goes to the rest pose, as the renderers draw it.

Measures
--------
At every leader frame time in the run (follow.Leader.center for the
centre; the leader's obj_quat, placed by Rw, for the turn):
  centre_err_m   distance between the simulated object's centre and the
                 leader's, placed
  orient_err_deg angle between the object's rotation since the start and
                 the leader's rotation since the start (placed)
  turn_sim_deg / turn_leader_deg
                 the rotation about the leader's turning axis (the mean
                 axis of its frame-to-frame rotations, carried by Rw),
                 summed from the start: signed, so a ball turning the
                 wrong way counts negative
  contacts       the fingers touching the object (via its collision
                 shapes), and the net contact force on the object
                 (sum of mj_contactForce, world frame) beside the
                 leader's own net force (placed)
A run's verdict (summary()): "follows" when the centre stays within
centre_tol_m (default: the task's allowed overlap, follow.overlap_allowance)
at every frame and the turn achieved is at least TURN_SHARE_MIN of the
leader's; "held, not turned" when the centre stays but the turn is below
that share (the "stays still" case issue 25 rules out); "lost" when the
centre leaves the tolerance, with the time it first does; "dropped" when
it leaves by more than the object's radius. A leader whose turn is below
TURN_MIN_DEG is judged on the centre alone.

Every row also holds each finger's own contact force on the object
beside the leader's force at the station the finger follows (the check's
input), so a hold can be read finger by finger.

The runs (replay()), in the order they gate each other:
  track     the hand alone (object collisions off) driven along the path:
            joint and fingertip error at every leader frame
  frozen    the hand held at its first checked pose, the object seated
            there at rest: the control. It should hold the object (if the
            first pose is a grasp) but not turn it, which shows the turn
            measure tells the two apart; its per-finger forces against the
            leader's say whether the pose makes the check's forces
  windows   issue 25's "short window": from every checked frame, the
            object re-seated on the leader (its pose and velocity there)
            and the hand driven for WINDOW_S; the errors at the window's
            end, kept with that frame's check verdict, so a frame the
            check called green can be read against one it called amber
  free      the whole leader, object free from the first frame
  (the check's own path jumps, path_jumps: where a finger's stored joints
  change IK branch between checked frames, or move faster than the hand's
  published no-load speed, are recorded beside these, and each window
  says whether it spans one; the replay drives the path as stored, so
  what the object does across a jump is the jump's consequence)
  stations  the positive control, no hand: the leader's own stations
            carry the object through this harness. Spheres of radius
            STATION_PAD_RADIUS_M (Wave_manip's finger_geom_radius) are
            servoed to the recorded station positions, placed by Rw, t,
            with Wave_manip's own "native" station servo (STATION_SERVO:
            kp 5000 N/m, 5 g, critically damped, 50 N cap), and MuJoCo
            friction mu stands in for Wave_manip's analytic friction kernel
            (the leader's stations were frictionless condim=1 mocap contacts
            plus that kernel, so its recorded forces are the normal parts).
            The stations are servoed, not mocap, because a mocap body has
            no velocity in MuJoCo and friction with it holds the object
            still instead of turning it (tried: the object's centre was
            carried to 0.4 mm but it turned -0.7 of 386 degrees). If the
            leader's own stations cannot carry the object here, no "does
            not follow" verdict about the hand stands.

Every number here is comparative, as the leaders are: the sim's contact
model stands in for pads, and the drive is MuJoCo's servo, not the
hand's. The centre error is against the leader's centre; a grasp whose
pads sink into the object by the check's allowance seats the object a few
millimetres from there once and then holds it, so the summary also gives
the mean offset vector and the error about that mean (offset_mean_m,
centre_err_about_mean_max_m): a constant offset and a slip read
differently there.
"""
import math
import time

import numpy as np

from . import follow as fo
from .hands import Hand, _mj

ARMATURE = 1e2            # the kinematic drive's joint armature (kg m^2) on every hand joint
STATION_PAD_RADIUS_M = 0.008   # Wave_manip sim/run_wave_unified.py finger_geom_radius: the
                               # leader's mocap finger spheres (condim 1 there)
STATION_SERVO = {"kp_N_m": 5000.0, "mass_kg": 0.005, "force_limit_N": 50.0,
                 "damping": "critical"}   # Wave_manip unified_v6/bridge/server.py SCENE_DEFAULTS
                                          # servo_kp, station_mass, force_limit; servo_kv None =
                                          # critical (its "native" station route)
STATION_INTEGRATOR = "implicit"   # Wave_manip sim/run_wave_unified.py integrator="implicit" at
                                  # timestep 0.002: a 5 g station on a 5000 N/m servo (1000 rad/s)
                                  # is unstable under Euler at that step (tried: 47 mm off by the
                                  # first frame, 425 m by the end), stable with the servo's
                                  # velocity term integrated implicitly
TURN_SHARE_MIN = 0.9      # "follows" needs at least this share of the leader's turn
TURN_MIN_DEG = 5.0        # a leader turning less than this is judged on the centre alone
WINDOW_S = 0.32           # issue 25's short window (ten frames of the w10 leaders)
CLEAR_ITERS = 12          # Jacobian steps that lift a free finger clear of the object
CLEAR_TOL_M = 1e-4


# ----------------------------------------------------------------- rotations

def quat_to_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(R):
    mj = _mj()
    q = np.zeros(4)
    mj.mju_mat2Quat(q, np.asarray(R, float).reshape(9))
    return q


def rotvec(R):
    """The rotation vector (axis times angle, rad) of rotation matrix R."""
    mj = _mj()
    q = mat_to_quat(R)
    v = np.zeros(3)
    mj.mju_quat2Vel(v, q, 1.0)
    return v


def angle_between(Ra, Rb):
    """Angle (rad) of the rotation taking Ra to Rb."""
    return float(np.linalg.norm(rotvec(np.asarray(Rb) @ np.asarray(Ra).T)))


def leader_rotations(leader_json, Rw):
    """R_k: the leader's object rotation at each frame, placed (Rw R_obj),
    or None when the leader has no quaternions."""
    Q = [f.get("obj_quat") for f in leader_json["frames"]]
    if any(q is None for q in Q):
        return None
    Rw = np.asarray(Rw)
    return [Rw @ quat_to_mat(np.asarray(q, float)) for q in Q]


def turning_axis(Rs):
    """The unit mean axis of the frame-to-frame rotations of Rs (placed), or
    None when they do not turn."""
    v = np.zeros(3)
    for a, b in zip(Rs[:-1], Rs[1:]):
        v += rotvec(b @ a.T)
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else None


def signed_turns(Rs, axis):
    """Cumulative signed turn (rad) about `axis` of the rotation sequence
    Rs, zero at the first."""
    out = [0.0]
    for a, b in zip(Rs[:-1], Rs[1:]):
        out.append(out[-1] + float(rotvec(b @ a.T) @ axis))
    return out


# -------------------------------------------------------------------- model

def hand_bodies(model):
    """Body ids of the hand: every body that is not the world, the object or
    the ghost."""
    mj = _mj()
    skip = {mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, n) for n in ("object", "leader_ghost")}
    return [b for b in range(1, model.nbody) if b not in skip]


def finger_owner(hand):
    """body id -> finger label, for the bodies from each finger's first
    joint to its tip (as the renderers chain them)."""
    m = hand.model
    owner = {}
    for f in hand.fingers:
        first = int(m.jnt_bodyid[f.joints[0]])
        b = int(f.tip_body)
        while b > 0:
            owner[b] = f.label
            if b == first:
                break
            b = int(m.body_parentid[b])
    return owner


def leader_gravity_hand(leader_json, Rw):
    """Gravity in the hand's frame (m/s^2), from the leader
    (follow._leader_gravity carried by Rw), with its record; raises when the
    leader gives no single direction."""
    grav = fo._leader_gravity(leader_json)
    if grav["dir"] is None:
        raise ValueError(f"the leader's gravity cannot be read: {grav['source']}")
    g = grav["g"] * (np.asarray(Rw) @ np.asarray(grav["dir"], float))
    return g, grav


def build_model(hand, leader, Rw, t, mu, g_hand, ghost=True, pole=True):
    """The replay model: the hand's file plus the free sphere (and the ghost).
    Returns (model, info); info names the object's body/geom/qpos/dof
    indices, the ghost's mocap index and the hand's qpos/dof indices. The
    hand's joint addresses are checked unchanged."""
    mj = _mj()
    spec = mj.MjSpec.from_file(hand.path)
    spec.option.gravity = [float(x) for x in g_hand]
    R, mass = float(leader.R_obj), float(leader.mass)
    c0 = leader.center(0, Rw, t)
    body = spec.worldbody.add_body(name="object", pos=[float(x) for x in c0])
    body.add_freejoint(name="object_free")
    body.add_geom(name="object_geom", type=mj.mjtGeom.mjGEOM_SPHERE, size=[R, 0, 0],
                  mass=mass, friction=[mu, 0.005, 0.0001], condim=3, priority=1,
                  rgba=[0.93, 0.42, 0.05, 1.0], group=0)
    if pole:   # a stripe from the centre out, so the turn can be seen
        body.add_geom(name="object_pole", type=mj.mjtGeom.mjGEOM_CAPSULE,
                      fromto=[0, 0, 0, 0, 0, R * 1.02], size=[R * 0.12, 0, 0], mass=0,
                      contype=0, conaffinity=0, rgba=[0.1, 0.1, 0.1, 1.0], group=0)
    if ghost:
        gh = spec.worldbody.add_body(name="leader_ghost", mocap=True, pos=[float(x) for x in c0])
        gh.add_geom(name="ghost_geom", type=mj.mjtGeom.mjGEOM_SPHERE, size=[R, 0, 0], mass=0,
                    contype=0, conaffinity=0, rgba=[0.06, 0.36, 0.78, 0.25], group=0)
        if pole:
            gh.add_geom(name="ghost_pole", type=mj.mjtGeom.mjGEOM_CAPSULE,
                        fromto=[0, 0, 0, 0, 0, R * 1.3], size=[R * 0.06, 0, 0], mass=0,
                        contype=0, conaffinity=0, rgba=[0.06, 0.36, 0.78, 0.9], group=0)
    model = spec.compile()
    hm = hand.model
    for j in range(hm.njnt):
        name = mj.mj_id2name(hm, mj.mjtObj.mjOBJ_JOINT, j)
        j2 = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
        if (j2 != j or model.jnt_qposadr[j2] != hm.jnt_qposadr[j]
                or model.jnt_dofadr[j2] != hm.jnt_dofadr[j]):
            raise RuntimeError(f"joint {name} moved when the object was added")
    ob = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "object")
    og = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "object_geom")
    oj = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "object_free")
    hb = hand_bodies(model)
    info = {"object_body": ob, "object_geom": og,
            "object_qpos": int(model.jnt_qposadr[oj]), "object_dof": int(model.jnt_dofadr[oj]),
            "hand_qpos": np.arange(hm.nq), "hand_dof": np.arange(hm.nv),
            "ghost_mocap": (int(model.body_mocapid[mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY,
                                                                 "leader_ghost")])
                            if ghost else None),
            "hand_bodies": hb}
    hbs = set(hb)
    info["hand_collision_geoms"] = [g for g in range(model.ngeom)
                                    if int(model.geom_bodyid[g]) in hbs
                                    and (model.geom_contype[g] or model.geom_conaffinity[g])]
    return model, info


def kinematic_drive(model, info):
    """Make the hand immovable for the kinematic drive: joint armature
    ARMATURE, weight cancelled, actuators inert, and only hand-object pairs
    colliding. Returns what was done, for the record."""
    hb = info["hand_bodies"]
    model.body_gravcomp[hb] = 1.0
    model.dof_armature[info["hand_dof"]] += ARMATURE
    if model.nu:
        model.actuator_gainprm[:, :] = 0.0
        model.actuator_biasprm[:, :] = 0.0
    for g in info["hand_collision_geoms"]:
        model.geom_contype[g], model.geom_conaffinity[g] = 1, 0
    og = info["object_geom"]
    model.geom_contype[og], model.geom_conaffinity[og] = 0, 1
    return {"drive": "kinematic", "armature_kg_m2": ARMATURE, "gravity_compensation": 1.0,
            "actuators": "inert",
            "collisions": "hand-object pairs only",
            "hand_collision_geoms": len(info["hand_collision_geoms"])}


def hand_object_only(model, info):
    """Collision masks so that only hand-object pairs collide."""
    for g in info["hand_collision_geoms"]:
        model.geom_contype[g], model.geom_conaffinity[g] = 1, 0
    og = info["object_geom"]
    model.geom_contype[og], model.geom_conaffinity[og] = 0, 1


def contact_options(model):
    """The contact solver settings in force (the hand file's own, which for
    Allegro are not MuJoCo's defaults), for the record."""
    mj = _mj()
    o = model.opt
    return {"cone": mj.mjtCone(int(o.cone)).name, "impratio": float(o.impratio),
            "noslip_iterations": int(o.noslip_iterations),
            "solver": mj.mjtSolver(int(o.solver)).name, "iterations": int(o.iterations),
            "integrator": mj.mjtIntegrator(int(o.integrator)).name,
            "timestep_s": float(o.timestep)}


def servo_drive(model, info):
    """The servo drive (module docstring): the hand file's own joint
    position actuators, untouched; only the collision masks change.
    Returns the record, with "ctrl_qpos": for each actuator the qpos
    address of the joint it serves (so a target q gives the ctrl vector).
    Refused when the hand has no actuators or one is not a plain joint
    position servo (force = kp (ctrl - q))."""
    mj = _mj()
    if model.nu == 0:
        raise ValueError("the servo drive needs the hand file's actuators; this hand has none")
    ctrl_qpos, kp = [], []
    for a in range(model.nu):
        ok = (model.actuator_trntype[a] == mj.mjtTrn.mjTRN_JOINT
              and model.actuator_gaintype[a] == mj.mjtGain.mjGAIN_FIXED
              and model.actuator_biastype[a] == mj.mjtBias.mjBIAS_AFFINE
              and model.actuator_gainprm[a, 0] > 0
              and abs(model.actuator_biasprm[a, 1] + model.actuator_gainprm[a, 0]) < 1e-9
              and abs(model.actuator_biasprm[a, 0]) < 1e-12)
        if not ok:
            name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, a)
            raise ValueError(f"the servo drive needs joint position servos; actuator {name!r} "
                             "is not one (use --drive kinematic)")
        ctrl_qpos.append(int(model.jnt_qposadr[int(model.actuator_trnid[a, 0])]))
        kp.append(float(model.actuator_gainprm[a, 0]))
    served = set(ctrl_qpos)
    unserved = [int(q) for q in info["hand_qpos"] if int(q) not in served]
    hand_object_only(model, info)
    return {"drive": "servo", "actuators": int(model.nu), "kp_Nm_per_rad": kp,
            "kv_Nm_s_per_rad": [float(-model.actuator_biasprm[a, 2]) for a in range(model.nu)],
            "joint_damping_Nm_s_per_rad": model.dof_damping[info["hand_dof"]].tolist(),
            "ctrl_qpos": ctrl_qpos, "unserved_joints": unserved,
            "armature_added_kg_m2": 0.0, "gravity_compensation": 0.0,
            "feedforward": "the check's contact load J^T f (linear between checked frames), "
                           "the hand's weight at the path's pose (mj_rne at rest), the stored "
                           "motion torque where fitted, the file's joint damping times the "
                           "path's joint velocity; through qfrc_applied",
            "collisions": "hand-object pairs only",
            "hand_collision_geoms": len(info["hand_collision_geoms"])}


class Feedforward:
    """The check's torques along the path (module docstring, "The drive").
    contact_Nm[j] is the contact-load torque on every hand dof at checked
    frame j: for each touching finger, J^T f with J follow's Jacobian
    (mj_jac at the stored pad_world on the stored phalanx, hand at the
    frame's path pose) and f the leader's force on the object at the
    finger's station, in the hand's frame. motion_Nm[j] is the stored
    motion torque (inertia.motion_Nm, zero where not fitted or off); for a
    hand with coupled joints the stored motion is over the free joints
    only and is left out (noted). Both are linear in time between checked
    frames and held outside. weight(q) is mj_rne at (q, 0, 0) on the hand's
    dofs under the model's gravity (the leader's). damping(t) is the
    model's dof_damping times the path's joint velocity."""
    def __init__(self, model, info, hand, result, leader, path, Rw):
        mj = _mj()
        self.mj, self.model, self.info, self.path = mj, model, info, path
        self.hq, self.hv = info["hand_qpos"], info["hand_dof"]
        self.data = mj.MjData(model)
        self.dof_damping = np.asarray(model.dof_damping[self.hv], float)
        frames = result["frames"]
        self.T = path.T
        nvh = len(self.hv)
        self.contact_Nm = np.zeros((len(frames), nvh))
        self.motion_Nm = np.zeros((len(frames), nvh))
        self.notes = {"contact_items": 0, "motion_items": 0, "motion_left_out": 0,
                      "motion_note": None}
        for j, fr in enumerate(frames):
            k = fr["k"]
            self.data.qpos[self.hq] = path.q(float(self.T[j]))[:len(self.hq)]
            self.data.qvel[:] = 0.0
            mj.mj_forward(model, self.data)
            for f, x in zip(hand.fingers, fr["fingers"]):
                if "q" not in x:
                    continue
                b = (int(f.tip_body) if x.get("phalanx", "distal") == "distal"
                     else int(model.body_parentid[f.tip_body]))
                pw = np.asarray(x["pad_world"], float)
                fv = np.asarray(leader.force(k, int(x["station"]), Rw), float)
                jacp = np.zeros((3, model.nv))
                mj.mj_jac(model, self.data, jacp, None, pw, b)
                self.contact_Nm[j, f.dof] += jacp[:, f.dof].T @ fv
                self.notes["contact_items"] += 1
                mo = (x.get("inertia") or {}).get("motion_Nm")
                if mo is not None:
                    if hand.coupled:
                        self.notes["motion_left_out"] += 1
                        self.notes["motion_note"] = ("stored motion torque is over the free "
                                                     "joints of a coupled hand; not fed forward")
                    else:
                        self.motion_Nm[j, f.dof] += np.asarray(mo, float)
                        self.notes["motion_items"] += 1
        self.notes["contact_Nm_max"] = float(np.abs(self.contact_Nm).max()) if len(frames) else 0.0
        self.notes["motion_Nm_max"] = float(np.abs(self.motion_Nm).max()) if len(frames) else 0.0

    def _interp(self, A, t):
        T = self.T
        if t <= T[0]:
            return A[0]
        if t >= T[-1]:
            return A[-1]
        j = int(np.searchsorted(T, t, side="right")) - 1
        s_ = (t - T[j]) / (T[j + 1] - T[j])
        return A[j] + s_ * (A[j + 1] - A[j])

    def weight(self, q):
        mj, d = self.mj, self.data
        d.qpos[self.hq] = q[:len(self.hq)]
        d.qvel[:] = 0.0
        d.qacc[:] = 0.0
        mj.mj_kinematics(self.model, d)
        mj.mj_comPos(self.model, d)
        mj.mj_comVel(self.model, d)
        r = np.zeros(self.model.nv)
        mj.mj_rne(self.model, d, 1, r)
        return r[self.hv]

    def damping(self, t):
        return self.dof_damping * self.path.qdot(t)[:len(self.hv)]

    def tau(self, t, contact=True, motion=True, weight=True, damping=True):
        """The torque fed forward at time t (on the hand's dofs), for the
        path's pose and velocity there."""
        out = np.zeros(len(self.hv))
        if contact:
            out += self._interp(self.contact_Nm, t)
        if motion:
            out += self._interp(self.motion_Nm, t)
        if weight:
            out += self.weight(self.path.q(t))
        if damping:
            out += self.damping(t)
        return out


class KinematicDriver:
    """Sets the hand's joint positions and velocities from the path at every
    step (the kinematic drive)."""
    name = "kinematic"

    def __init__(self, path, info):
        self.path, self.hq, self.hv = path, info["hand_qpos"], info["hand_dof"]

    def start(self, data, t):
        data.qpos[self.hq] = self.path.q(t)[:len(self.hq)]
        data.qvel[self.hv] = 0.0

    def step(self, data, t):
        data.qpos[self.hq] = self.path.q(t)[:len(self.hq)]
        data.qvel[self.hv] = self.path.qdot(t)[:len(self.hv)]

    def hold(self, data, t):
        """The pose at t, standing still."""
        data.qpos[self.hq] = self.path.q(t)[:len(self.hq)]
        data.qvel[self.hv] = 0.0


class ServoDriver:
    """Tells the actuators the path's targets and feeds the check's torques
    forward (the servo drive). The hand starts on the path, at rest."""
    name = "servo"

    def __init__(self, path, info, ff, ctrl_qpos, contact=True, motion=True, weight=True,
                 damping=True):
        self.path, self.ff, self.ctrl_qpos = path, ff, np.asarray(ctrl_qpos, int)
        self.hq, self.hv = info["hand_qpos"], info["hand_dof"]
        self.parts = dict(contact=contact, motion=motion, weight=weight, damping=damping)

    def start(self, data, t):
        data.qpos[self.hq] = self.path.q(t)[:len(self.hq)]
        data.qvel[self.hv] = 0.0
        self.step(data, t)

    def step(self, data, t):
        q = self.path.q(t)
        data.ctrl[:] = q[self.ctrl_qpos]
        data.qfrc_applied[self.hv] = self.ff.tau(t, **self.parts)

    def hold(self, data, t):
        """The targets at t with the standing parts of the feedforward only
        (contact load and weight; no motion or damping, since the hand is
        not moving)."""
        q = self.path.q(t)
        data.ctrl[:] = q[self.ctrl_qpos]
        parts = dict(self.parts, motion=False, damping=False)
        data.qfrc_applied[self.hv] = self.ff.tau(t, **parts)


class Frozen:
    """A driver held at one time: the control. The hand stands still there
    (driver.hold), so nothing meant for a moving hand is applied."""
    def __init__(self, driver, t0):
        self.driver, self.t0 = driver, t0
        self.name = driver.name + " (frozen)"

    def start(self, data, t):
        self.driver.start(data, self.t0)
        self.driver.hold(data, self.t0)

    def step(self, data, t):
        self.driver.hold(data, self.t0)


class StationDriver:
    """Servoes the leader's stations (bodies on three slide joints with
    position actuators, stations_model) to their recorded positions, placed
    by Rw, t, linear in time between leader frames (the positive control).
    A station with no recorded position is parked PARK_M away; when it
    reappears it is put there directly, at rest, not flown back."""
    name = "stations"
    PARK_M = 10.0

    def __init__(self, leader, Rw, t, qpos_adr, dof_adr, ctrl_adr):
        self.T = np.array([float(f["t"]) for f in leader.frames])
        n = len(qpos_adr)
        P = np.full((len(leader.frames), n, 3), np.nan)
        for k, f in enumerate(leader.frames):
            for i in range(n):
                sp = f["station_pos"][i]
                if sp is not None:
                    P[k, i] = Rw @ (np.asarray(sp, float) - leader.c0) + t
        self.P = P
        self.qpos_adr = [int(x) for x in qpos_adr]
        self.dof_adr = [int(x) for x in dof_adr]
        self.ctrl_adr = [int(x) for x in ctrl_adr]
        self.parked = None

    def pos(self, t):
        """Targets (n, 3) and which stations are parked at time t."""
        T, P = self.T, self.P
        if t <= T[0]:
            out = P[0].copy()
        elif t >= T[-1]:
            out = P[-1].copy()
        else:
            j = int(np.searchsorted(T, t, side="right")) - 1
            s_ = (t - T[j]) / (T[j + 1] - T[j])
            out = P[j] + s_ * (P[j + 1] - P[j])
        bad = np.isnan(out).any(axis=1)
        out[bad] = [0.0, 0.0, self.PARK_M]
        return out, bad

    def start(self, data, t):
        out, bad = self.pos(t)
        for i, (q, v) in enumerate(zip(self.qpos_adr, self.dof_adr)):
            data.qpos[q:q + 3] = out[i]
            data.qvel[v:v + 3] = 0.0
            data.ctrl[self.ctrl_adr[i]:self.ctrl_adr[i] + 3] = out[i]
        self.parked = bad

    def step(self, data, t):
        out, bad = self.pos(t)
        for i, (q, v) in enumerate(zip(self.qpos_adr, self.dof_adr)):
            if self.parked[i] != bad[i]:
                data.qpos[q:q + 3] = out[i]
                data.qvel[v:v + 3] = 0.0
            data.ctrl[self.ctrl_adr[i]:self.ctrl_adr[i] + 3] = out[i]
        self.parked = bad


PAD_IMPEDANCE = 0.95      # MuJoCo constraint impedance of the object's contacts (constant)
PAD_DAMPING_RATIO = 1.0   # critical


def pad_stiffness(leader_json, allowed_overlap_m):
    """The "allowance" pad stiffness (N/m): the leader's median contact
    force size over the frames and stations in contact, divided by the
    task's allowed overlap: the softest pad consistent with the check's
    allowance. Returns (K, median force)."""
    F = [float(np.linalg.norm(f["force_on_object_N"][i]))
         for f in leader_json["frames"] for i, on in enumerate(f["contact"]) if on]
    if not F or allowed_overlap_m <= 0:
        raise ValueError("pad stiffness needs contact forces and a positive allowed overlap")
    med = float(np.median(F))
    return med / float(allowed_overlap_m), med


def leader_pad_stiffness(leader_json, pad_radius=STATION_PAD_RADIUS_M):
    """The "leader" pad stiffness (N/m): what the leader's own contacts
    showed. At every frame and station in contact with a recorded station
    position, the force size over the station's sink into the object
    (pad_radius + object radius - |station - centre|); the median of those
    ratios. Returns (K, record). ValueError when no contact has a positive
    sink (then the leader's data gives no stiffness)."""
    R = float(leader_json["object"]["radius_m"])
    ratios, sinks, forces = [], [], []
    for f in leader_json["frames"]:
        c = np.asarray(f["obj_pos"], float)
        for i, on in enumerate(f["contact"]):
            sp = f["station_pos"][i]
            if not on or sp is None:
                continue
            sink = pad_radius + R - float(np.linalg.norm(np.asarray(sp, float) - c))
            F = float(np.linalg.norm(f["force_on_object_N"][i]))
            if sink > 0 and F > 0:
                ratios.append(F / sink)
                sinks.append(sink)
                forces.append(F)
    if not ratios:
        raise ValueError("the leader's contacts show no positive sink with force")
    return float(np.median(ratios)), {
        "contacts": len(ratios), "sink_median_m": float(np.median(sinks)),
        "force_median_N": float(np.median(forces)),
        "pad_radius_m": pad_radius,
        "stations_at_surface_share": float(np.mean(np.abs(np.asarray(sinks) - pad_radius) < 1e-6)),
        "stations_at_surface_note": "share of the contacts whose station lies on the object's "
                                    "surface (sink = the whole pad radius): a leader that recorded "
                                    "contact points, not pad centres, for which this stiffness "
                                    "is force over the pad radius",
        "derivation": "median over the leader's contacts of force / (pad radius + object radius "
                      "- |station - centre|), the leader's recorded station positions and forces"}


def _measure_stiffness(k, b, D, mass, g, radius, steps=2000):
    """A sphere of `mass` resting on a plane under gravity g with the given
    constraint setting (k None: MuJoCo's defaults): (force / penetration,
    N/m) once settled."""
    mj = _mj()
    setting = (f'solref="{-k} {-b}" solimp="{D} {D} 0.001 0.5 2"' if k is not None else "")
    xml = (f'<mujoco><option gravity="0 0 -{g}" timestep="0.002"/><worldbody>'
           f'<geom type="plane" size="1 1 .1"/>'
           f'<body pos="0 0 {radius}"><freejoint/><geom type="sphere" size="{radius}" mass="{mass}" '
           f'priority="1" {setting}/></body>'
           f'</worldbody></mujoco>')
    M = mj.MjModel.from_xml_string(xml)
    d = mj.MjData(M)
    for _ in range(steps):
        mj.mj_step(M, d)
    f6 = np.zeros(6)
    F = 0.0
    for i in range(d.ncon):
        mj.mj_contactForce(M, d, i, f6)
        F += f6[0]
    pen = radius - d.qpos[2]
    return F / pen if pen > 0 else float("inf")


def pad_contact(model, info, K, mass, g, radius):
    """Set the object's contacts to stiffness K (N/m), critically damped,
    through MuJoCo's direct (negative) solref with a constant impedance.
    The first setting follows the solver's formula (stiffness =
    d/(1-d) mass k), then is corrected once from a measurement on a sphere
    of this mass on a plane and measured again. Returns the record."""
    D = PAD_IMPEDANCE
    scale = D / (1 - D) * mass
    k0 = K / scale
    b = PAD_DAMPING_RATIO * 2 * math.sqrt(K * mass) / scale
    K0 = _measure_stiffness(k0, b, D, mass, g, radius)
    k = k0 * K / K0
    K1 = _measure_stiffness(k, b, D, mass, g, radius)
    og = info["object_geom"]
    model.geom_solref[og] = [-k, -b]
    model.geom_solimp[og] = [D, D, 0.001, 0.5, 2]
    return {"asked_N_m": K, "reached_N_m": K1, "first_setting_N_m": K0,
            "solref": [-k, -b], "solimp": [D, D, 0.001, 0.5, 2],
            "damping_ratio": PAD_DAMPING_RATIO, "impedance": D,
            "calibration": "sphere of the object's mass on a plane under the leader's g"}


# --------------------------------------------------------------------- path

def station_lift(leader, i, k, d_on):
    """How far the leader's station i backs off the object at frame k while
    off (m): its distance from the surface less d_on, its distance while
    on; 0 when on or unknown."""
    f = leader.frames[k]
    if f["contact"][i] or f["station_pos"][i] is None:
        return 0.0
    d = float(np.linalg.norm(np.asarray(f["station_pos"][i], float)
                             - np.asarray(f["obj_pos"], float))) - leader.R_obj
    return max(0.0, d - d_on)


def station_on_distance(leader, i):
    """The median distance of station i from the object's surface over the
    frames it touches in (m), or 0 if it never does."""
    d = [float(np.linalg.norm(np.asarray(f["station_pos"][i], float)
                              - np.asarray(f["obj_pos"], float))) - leader.R_obj
         for f in leader.frames if f["contact"][i] and f["station_pos"][i] is not None]
    return float(np.median(d)) if d else 0.0


class Path:
    """The hand's joint vector over time: Q[j] at times T[j] (the checked
    frames), linear between, held outside. Coupling applied."""
    def __init__(self, hand, T, Q):
        self.hand, self.T, self.Q = hand, np.asarray(T, float), np.asarray(Q, float)

    def q(self, t):
        T, Q = self.T, self.Q
        if t <= T[0]:
            return Q[0].copy()
        if t >= T[-1]:
            return Q[-1].copy()
        j = int(np.searchsorted(T, t, side="right")) - 1
        s = (t - T[j]) / (T[j + 1] - T[j])
        return self.hand.apply_coupling(Q[j] + s * (Q[j + 1] - Q[j]))

    def qdot(self, t, h=1e-4):
        return (self.q(t + h) - self.q(t - h)) / (2 * h)


def stored_path(hand, result, leader, lift="leader", model=None, info=None, Rw=None, t=None):
    """Path from the result's stored finger poses (module docstring, "The
    joint path"), with the lift policy for fingers the check left off.
    Returns (path, notes): notes counts the off frames and the lifts made."""
    frames = result["frames"]
    ks = [fr["k"] for fr in frames]
    T = [float(leader.frames[k]["t"]) for k in ks]
    base = np.array(hand.base_q, float)
    Q = np.tile(base, (len(frames), 1))
    on = np.zeros((len(frames), len(hand.fingers)), bool)
    for j, fr in enumerate(frames):
        for fi, (f, x) in enumerate(zip(hand.fingers, fr["fingers"])):
            if "q" in x:
                Q[j, f.qadr] = x["q"]
                on[j, fi] = True
    notes = {"lift": lift, "checked_frames": len(frames),
             "off_frames": {f.label: int((~on[:, fi]).sum()) for fi, f in enumerate(hand.fingers)},
             "lifts_m": {}}
    if lift == "rest":
        return Path(hand, T, Q), notes
    if lift != "leader":
        raise ValueError(f"unknown lift policy {lift!r}")
    assignment = result["assignment"]
    idx = np.arange(len(frames))
    for fi, f in enumerate(hand.fingers):
        col = on[:, fi]
        if col.all():
            continue
        if not col.any():
            notes["lifts_m"][f.label] = "never touches: rest pose"
            continue
        i = assignment[f.label]
        d_on = station_on_distance(leader, i)
        # joint-space line from the leaving pose to the landing pose
        for j in idx[~col]:
            prev = idx[:j][col[:j]]
            nxt = idx[j + 1:][col[j + 1:]]
            if len(prev) and len(nxt):
                a, b = prev[-1], nxt[0]
                s = (j - a) / (b - a)
                Q[j, f.qadr] = Q[a, f.qadr] + s * (Q[b, f.qadr] - Q[a, f.qadr])
            elif len(nxt):
                Q[j, f.qadr] = Q[nxt[0], f.qadr]
            else:
                Q[j, f.qadr] = Q[prev[-1], f.qadr]
        lifts = []
        if model is not None:
            for j in idx[~col]:
                want = station_lift(leader, i, ks[j], d_on)
                c = leader.center(ks[j], Rw, t)
                Q[j], got = clear_finger(model, info, hand, f, Q[j], c, want)
                lifts.append((ks[j], want, got))
        notes["lifts_m"][f.label] = {"station": int(i), "station_on_distance_m": d_on,
                                     "frames": [(int(k), round(w, 5), round(g, 5))
                                                for k, w, g in lifts]}
    return Path(hand, T, Q), notes


def finger_clearance(model, data, info, hand, finger, q, centre):
    """(signed distance, witness point on the finger, body id) of the finger's
    nearest collision shape to the object's surface, with the hand at q and
    the object at `centre` (positions only)."""
    mj = _mj()
    hq = info["hand_qpos"]
    data.qpos[hq] = q[:len(hq)]
    oq = info["object_qpos"]
    data.qpos[oq:oq + 3] = centre
    data.qpos[oq + 3:oq + 7] = [1, 0, 0, 0]
    mj.mj_forward(model, data)      # xpos, xmat and the com frames the Jacobian needs
    own = finger_owner(hand)
    best = (np.inf, None, None)
    fromto = np.zeros(6)
    for g in info["hand_collision_geoms"]:
        if own.get(int(model.geom_bodyid[g])) != finger.label:
            continue
        d = mj.mj_geomDistance(model, data, g, info["object_geom"], 0.2, fromto)
        if d < best[0]:
            best = (float(d), fromto[:3].copy(), int(model.geom_bodyid[g]))
    return best


def clear_finger(model, info, hand, finger, q, centre, want):
    """Lift the finger's nearest collision shape to at least `want` (m)
    clear of the object by Jacobian steps along the outward radial
    direction, within joint ranges. Returns (q, clearance reached)."""
    mj = _mj()
    data = mj.MjData(model)
    q = np.array(q, float)
    d, p, b = finger_clearance(model, data, info, hand, finger, q, centre)
    rng = np.asarray(finger.ranges, float)
    for _ in range(CLEAR_ITERS):
        if d >= want - CLEAR_TOL_M:
            break
        out = p - centre
        out /= np.linalg.norm(out)
        jacp = np.zeros((3, model.nv))
        mj.mj_jac(model, data, jacp, None, p, b)
        J = jacp[:, finger.dof]
        dq = np.linalg.pinv(J, rcond=1e-3) @ (out * (want - d))
        q[finger.qadr] = np.clip(q[finger.qadr] + dq, rng[:, 0], rng[:, 1])
        d, p, b = finger_clearance(model, data, info, hand, finger, q, centre)
    return q, d


# ---------------------------------------------------------------------- runs

def leader_state(leader, k, Rw, t, Rs):
    """The leader's object pose at frame k, placed: (centre, rotation,
    linear velocity, angular velocity) by central differences."""
    c = leader.center(k, Rw, t)
    km, kp = max(k - 1, 0), min(k + 1, len(leader.frames) - 1)
    dt = float(leader.frames[kp]["t"]) - float(leader.frames[km]["t"])
    v = (leader.center(kp, Rw, t) - leader.center(km, Rw, t)) / dt if dt > 0 else np.zeros(3)
    if Rs is None:
        return c, np.eye(3), v, np.zeros(3)
    w = rotvec(Rs[kp] @ Rs[km].T) / dt if dt > 0 else np.zeros(3)
    return c, Rs[k], v, w


def net_contact_force(model, data, info, owner=None):
    """(net force on the object, world frame; set of labels touching it;
    number of contacts; force on the object per label)."""
    mj = _mj()
    og = info["object_geom"]
    F = np.zeros(3)
    who = set()
    per = {}
    n = 0
    f6 = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        if c.geom1 != og and c.geom2 != og:
            continue
        mj.mj_contactForce(model, data, i, f6)
        fw = c.frame.reshape(3, 3).T @ f6[:3]      # on geom2, along the normal from geom1
        fo_ = fw if c.geom2 == og else -fw
        F += fo_
        other = c.geom1 if c.geom2 == og else c.geom2
        lab = owner.get(int(model.geom_bodyid[other]), "other") if owner is not None else "other"
        who.add(lab)
        per[lab] = per.get(lab, np.zeros(3)) + fo_
        n += 1
    return F, who, n, {k: v.tolist() for k, v in per.items()}


def seat_object(data, info, c, R, v, w):
    oq, od = info["object_qpos"], info["object_dof"]
    data.qpos[oq:oq + 3] = c
    data.qpos[oq + 3:oq + 7] = mat_to_quat(R)
    data.qvel[od:od + 3] = v
    data.qvel[od + 3:od + 6] = w


def simulate(model, info, driver, leader, Rw, t, Rs, k0, k1, owner, follows=None,
             at_rest=False, on_frame=None):
    """Run the driver from leader frame k0 to k1 (inclusive) with the object
    seated on the leader at k0 (its pose and velocity there; at rest with
    at_rest), and read the object at every leader frame. owner maps body
    ids to labels (fingers, or stations); follows maps a label to the
    leader station it follows, for the per-label force comparison. Returns
    the rows (one per frame). on_frame(k, model, data) is called at each
    frame read (films)."""
    mj = _mj()
    data = mj.MjData(model)
    T = [float(f["t"]) for f in leader.frames]
    t0 = T[k0]
    driver.start(data, t0)
    c, R, v, w = leader_state(leader, k0, Rw, t, Rs)
    if at_rest:
        v, w = np.zeros(3), np.zeros(3)
    seat_object(data, info, c, R, v, w)
    gm = info.get("ghost_mocap")
    if gm is not None:
        data.mocap_pos[gm] = c
        data.mocap_quat[gm] = mat_to_quat(R)
    mj.mj_forward(model, data)
    dt = model.opt.timestep
    axis = info.get("turn_axis")
    rows = []
    oq = info["object_qpos"]
    R_sim0 = quat_to_mat(data.qpos[oq + 3:oq + 7])
    R_prev, turn_sim = R_sim0, 0.0
    R_lead0 = Rs[k0] if Rs is not None else np.eye(3)
    turn_lead = 0.0
    sim_t = t0
    follows = follows or {}
    for k in range(k0, k1 + 1):
        while sim_t + dt / 2 < T[k]:
            driver.step(data, sim_t)
            mj.mj_step(model, data)
            sim_t += dt
        driver.step(data, sim_t)
        c, R, v, w = leader_state(leader, k, Rw, t, Rs)
        if gm is not None:
            data.mocap_pos[gm] = c
            data.mocap_quat[gm] = mat_to_quat(R)
        mj.mj_forward(model, data)
        cs = data.qpos[oq:oq + 3].copy()
        R_sim = quat_to_mat(data.qpos[oq + 3:oq + 7])
        if axis is not None:
            turn_sim += float(rotvec(R_sim @ R_prev.T) @ axis)
            if k > k0 and Rs is not None:
                turn_lead += float(rotvec(Rs[k] @ Rs[k - 1].T) @ axis)
        R_prev = R_sim
        F, who, n, per = net_contact_force(model, data, info, owner)
        fl = np.zeros(3)
        for i, onn in enumerate(leader.frames[k]["contact"]):
            if onn:
                fl += leader.force(k, i, Rw)
        per_leader = {}
        for lab, i in follows.items():
            per_leader[lab] = (leader.force(k, int(i), Rw).tolist()
                               if leader.frames[k]["contact"][int(i)] else [0.0, 0.0, 0.0])
        rows.append({"k": k, "t": T[k], "centre_sim_m": cs.tolist(), "centre_leader_m": c.tolist(),
                     "centre_err_m": float(np.linalg.norm(cs - c)),
                     "orient_err_deg": (math.degrees(angle_between(R_sim @ R_sim0.T, R @ R_lead0.T))
                                        if Rs is not None else None),
                     "turn_sim_deg": math.degrees(turn_sim) if axis is not None else None,
                     "turn_leader_deg": math.degrees(turn_lead) if axis is not None else None,
                     "contacts": sorted(who), "n_contacts": n,
                     "net_force_sim_N": F.tolist(), "net_force_leader_N": fl.tolist(),
                     "force_sim_N": per, "force_leader_N": per_leader,
                     "leader_on": [i for i, x in enumerate(leader.frames[k]["contact"]) if x]})
        if on_frame is not None:
            on_frame(k, model, data)
    return rows


def force_comparison(rows):
    """Per label, over the rows where the leader's station is on: the median
    of |sim| / |leader| and of the angle between them (degrees), and the
    share of rows where the sim has any contact there."""
    out = {}
    labs = sorted({lab for r in rows for lab in r["force_leader_N"]})
    for lab in labs:
        ratios, angles, touched, n_on = [], [], 0, 0
        for r in rows:
            fl = np.asarray(r["force_leader_N"][lab], float)
            if np.linalg.norm(fl) <= 0:
                continue
            n_on += 1
            fs = np.asarray(r["force_sim_N"].get(lab, [0.0, 0.0, 0.0]), float)
            ratios.append(float(np.linalg.norm(fs) / np.linalg.norm(fl)))
            if np.linalg.norm(fs) > 0:
                touched += 1
                angles.append(math.degrees(math.acos(max(-1.0, min(1.0, float(
                    fs @ fl / (np.linalg.norm(fs) * np.linalg.norm(fl))))))))
        out[lab] = {"frames_leader_on": n_on,
                    "ratio_median": float(np.median(ratios)) if ratios else None,
                    "ratio_max": float(max(ratios)) if ratios else None,
                    "angle_median_deg": float(np.median(angles)) if angles else None,
                    "touching_share": (touched / n_on) if n_on else None}
    return out


def summary(rows, leader, centre_tol_m, turn_share_min=TURN_SHARE_MIN, turn_min_deg=TURN_MIN_DEG):
    """The run's verdict and headline numbers (module docstring,
    "Measures")."""
    errs = np.array([r["centre_err_m"] for r in rows])
    off = np.array([np.asarray(r["centre_sim_m"]) - np.asarray(r["centre_leader_m"]) for r in rows])
    off_mean = off.mean(axis=0)
    about = np.linalg.norm(off - off_mean, axis=1)
    over = [r for r in rows if r["centre_err_m"] > centre_tol_m]
    dropped = [r for r in rows if r["centre_err_m"] > leader.R_obj]
    turn_s = rows[-1]["turn_sim_deg"]
    turn_l = rows[-1]["turn_leader_deg"]
    share = (turn_s / turn_l) if (turn_l is not None and abs(turn_l) > 1e-9) else None
    turned_enough = (share is None or abs(turn_l) < turn_min_deg or share >= turn_share_min)
    if dropped:
        verdict = "dropped"
    elif over:
        verdict = "lost"
    elif turned_enough:
        verdict = "follows"
    else:
        verdict = "held, not turned"
    return {"verdict": verdict, "centre_tol_m": centre_tol_m, "frames": len(rows),
            "centre_err_max_m": float(errs.max()), "centre_err_end_m": float(errs[-1]),
            "offset_mean_m": off_mean.tolist(), "offset_mean_size_m": float(np.linalg.norm(off_mean)),
            "centre_err_about_mean_max_m": float(about.max()),
            "centre_err_about_mean_end_m": float(about[-1]),
            "first_over_tol_t": over[0]["t"] if over else None,
            "first_over_tol_k": over[0]["k"] if over else None,
            "dropped_t": dropped[0]["t"] if dropped else None,
            "turn_sim_deg": turn_s, "turn_leader_deg": turn_l, "turn_share": share,
            "turn_share_min": turn_share_min, "turn_min_deg": turn_min_deg,
            "orient_err_end_deg": rows[-1]["orient_err_deg"],
            "orient_err_max_deg": (max(r["orient_err_deg"] for r in rows)
                                   if rows[0]["orient_err_deg"] is not None else None),
            "frames_with_no_contact": int(sum(r["n_contacts"] == 0 for r in rows)),
            "mean_contacts": float(np.mean([r["n_contacts"] for r in rows])),
            "forces_vs_leader": force_comparison(rows)}


def geometry_check(model, info, hand, path, result, leader, Rw, t):
    """At each checked frame, no stepping: the stored ball_center against
    the leader's placed centre, each touching finger's stored pad_world
    against the model's nearest point to the object, and the finger's
    sinking into the object (negative clearance) against the check's
    allowance. Largest discrepancies."""
    mj = _mj()
    data = mj.MjData(model)
    bc, pad, sink, lifted = [], [], [], []
    for fr in result["frames"]:
        k = fr["k"]
        c = leader.center(k, Rw, t)
        bc.append(float(np.linalg.norm(np.asarray(fr["ball_center"], float) - c)))
        q = path.q(float(leader.frames[k]["t"]))
        for f, x in zip(hand.fingers, fr["fingers"]):
            d, p, _ = finger_clearance(model, data, info, hand, f, q, c)
            if "q" in x:
                pad.append(float(np.linalg.norm(np.asarray(x["pad_world"], float) - p)))
                sink.append(-d)
            else:
                lifted.append(d)
    return {"ball_center_vs_leader_max_m": max(bc) if bc else None,
            "pad_world_vs_model_nearest_max_m": max(pad) if pad else None,
            "pad_world_vs_model_nearest_median_m": float(np.median(pad)) if pad else None,
            "touching_sink_max_m": max(sink) if sink else None,
            "touching_sink_median_m": float(np.median(sink)) if sink else None,
            "touching_sink_min_m": min(sink) if sink else None,
            "lifted_clearance_min_m": min(lifted) if lifted else None,
            "lifted_clearance_median_m": float(np.median(lifted)) if lifted else None}


def track_check(model, info, hand, driver, path, leader, k0, k1):
    """The hand alone along the path (the object's collisions off for the
    run): at every leader frame the joint error and each fingertip's
    position error against the path. Returns the summary."""
    mj = _mj()
    og = info["object_geom"]
    masks = (int(model.geom_contype[og]), int(model.geom_conaffinity[og]))
    model.geom_contype[og] = model.geom_conaffinity[og] = 0
    try:
        data, ref = mj.MjData(model), mj.MjData(model)
        hq = info["hand_qpos"]
        T = [float(f["t"]) for f in leader.frames]
        driver.start(data, T[k0])
        mj.mj_forward(model, data)
        dt = model.opt.timestep
        sim_t = T[k0]
        jerr, terr = [], {f.label: [] for f in hand.fingers}
        for k in range(k0, k1 + 1):
            while sim_t + dt / 2 < T[k]:
                driver.step(data, sim_t)
                mj.mj_step(model, data)
                sim_t += dt
            mj.mj_forward(model, data)
            q = path.q(sim_t)
            ref.qpos[hq] = q[:len(hq)]
            mj.mj_kinematics(model, ref)
            jerr.append(float(np.abs(data.qpos[hq] - q[:len(hq)]).max()))
            for f in hand.fingers:
                terr[f.label].append(float(np.linalg.norm(f.tip_pos(data) - f.tip_pos(ref))))
    finally:
        model.geom_contype[og], model.geom_conaffinity[og] = masks
    allt = [e for v in terr.values() for e in v]
    return {"frames": len(jerr), "joint_err_max_rad": max(jerr),
            "joint_err_median_rad": float(np.median(jerr)),
            "tip_err_max_m": max(allt), "tip_err_median_m": float(np.median(allt)),
            "tip_err_max_by_finger_m": {lab: max(v) for lab, v in terr.items()}}


def stations_model(leader, Rw, t, mu, g_hand, pad_radius=STATION_PAD_RADIUS_M, timestep=None):
    """The positive control's model: the object (and ghost) as build_model
    makes them, and one sphere of pad_radius per leader station on three
    slide joints with position actuators (STATION_SERVO), colliding with
    the object alone under MuJoCo's default contact settings. Returns
    (model, info)."""
    mj = _mj()
    spec = mj.MjSpec()
    spec.option.gravity = [float(x) for x in g_hand]
    if timestep is not None:
        spec.option.timestep = float(timestep)
    spec.option.integrator = getattr(mj.mjtIntegrator, "mjINT_" + STATION_INTEGRATOR.upper())
    R, mass = float(leader.R_obj), float(leader.mass)
    c0 = leader.center(0, Rw, t)
    body = spec.worldbody.add_body(name="object", pos=[float(x) for x in c0])
    body.add_freejoint(name="object_free")
    body.add_geom(name="object_geom", type=mj.mjtGeom.mjGEOM_SPHERE, size=[R, 0, 0],
                  mass=mass, friction=[mu, 0.005, 0.0001], condim=3, priority=1,
                  contype=0, conaffinity=1, rgba=[0.93, 0.42, 0.05, 1.0], group=0)
    body.add_geom(name="object_pole", type=mj.mjtGeom.mjGEOM_CAPSULE,
                  fromto=[0, 0, 0, 0, 0, R * 1.02], size=[R * 0.12, 0, 0], mass=0,
                  contype=0, conaffinity=0, rgba=[0.1, 0.1, 0.1, 1.0], group=0)
    gh = spec.worldbody.add_body(name="leader_ghost", mocap=True, pos=[float(x) for x in c0])
    gh.add_geom(name="ghost_geom", type=mj.mjtGeom.mjGEOM_SPHERE, size=[R, 0, 0], mass=0,
                contype=0, conaffinity=0, rgba=[0.06, 0.36, 0.78, 0.25], group=0)
    n = len(leader.frames[0]["station_pos"])
    kp, ms, cap = STATION_SERVO["kp_N_m"], STATION_SERVO["mass_kg"], STATION_SERVO["force_limit_N"]
    kv = 2.0 * math.sqrt(kp * ms)
    for i in range(n):
        sb = spec.worldbody.add_body(name=f"station{i}", pos=[0.0, 0.0, 0.0])
        for ax, axis in (("x", [1, 0, 0]), ("y", [0, 1, 0]), ("z", [0, 0, 1])):
            sb.add_joint(name=f"station{i}_{ax}", type=mj.mjtJoint.mjJNT_SLIDE, axis=axis)
        sb.add_geom(name=f"station{i}_geom", type=mj.mjtGeom.mjGEOM_SPHERE,
                    size=[pad_radius, 0, 0], mass=ms, friction=[mu, 0.005, 0.0001], condim=3,
                    contype=1, conaffinity=0, rgba=[0.35, 0.35, 0.4, 1.0], group=0)
        for ax in "xyz":
            act = spec.add_actuator(name=f"station{i}_{ax}_servo", target=f"station{i}_{ax}",
                                    trntype=mj.mjtTrn.mjTRN_JOINT)
            act.gainprm[0] = kp
            act.biasprm[0], act.biasprm[1], act.biasprm[2] = 0.0, -kp, -kv
            act.biastype = mj.mjtBias.mjBIAS_AFFINE
            act.forcerange[0], act.forcerange[1] = -cap, cap
            act.forcelimited = mj.mjtLimited.mjLIMITED_TRUE
    model = spec.compile()
    oj = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "object_free")
    sbodies = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, f"station{i}") for i in range(n)]
    sj = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, f"station{i}_x") for i in range(n)]
    sa = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, f"station{i}_x_servo") for i in range(n)]
    info = {"object_body": mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "object"),
            "object_geom": mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "object_geom"),
            "object_qpos": int(model.jnt_qposadr[oj]), "object_dof": int(model.jnt_dofadr[oj]),
            "hand_qpos": np.arange(0), "hand_dof": np.arange(0),
            "ghost_mocap": int(model.body_mocapid[mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY,
                                                                "leader_ghost")]),
            "station_bodies": sbodies,
            "station_qpos": [int(model.jnt_qposadr[j]) for j in sj],
            "station_dof": [int(model.jnt_dofadr[j]) for j in sj],
            "station_ctrl": sa,
            "station_servo": {"kp_N_m": kp, "kv_N_s_m": kv, "mass_kg": ms, "force_limit_N": cap},
            "owner": {b: f"station{i}" for i, b in enumerate(sbodies)},
            "follows": {f"station{i}": i for i in range(n)}}
    return model, info


def stations_control(leader, Rw, t, mu, g_hand, Rs, axis, k0, k1, centre_tol_m,
                     pad_radius=STATION_PAD_RADIUS_M, timestep=None, on_frame=None):
    """The positive control (module docstring, "stations"): the leader's own
    stations carry the object through this harness. Returns the record
    (summary, rows, settings)."""
    model, info = stations_model(leader, Rw, t, mu, g_hand, pad_radius, timestep)
    info["turn_axis"] = axis
    drv = StationDriver(leader, Rw, t, info["station_qpos"], info["station_dof"],
                        info["station_ctrl"])
    rows = simulate(model, info, drv, leader, Rw, t, Rs, k0, k1, info["owner"],
                    follows=info["follows"], on_frame=on_frame)
    n = len(info["station_bodies"])
    d_on = [station_on_distance(leader, i) for i in range(n)]
    return {"summary": summary(rows, leader, centre_tol_m),
            "settings": {"pad_radius_m": pad_radius,
                         "pad_radius_source": ("Wave_manip sim/run_wave_unified.py "
                                               "finger_geom_radius, the leader's mocap finger "
                                               "spheres"),
                         "station_servo": info["station_servo"],
                         "station_servo_source": ("Wave_manip unified_v6/bridge/server.py "
                                                  "SCENE_DEFAULTS servo_kp, station_mass, "
                                                  "force_limit; critically damped (servo_kv "
                                                  "None); the leader itself used mocap "
                                                  "stations, which cannot turn an object here"),
                         "leader_station_surface_distance_m": d_on,
                         "stations_at_surface": bool(all(abs(d) < 1e-6 for d in d_on)),
                         "stations_at_surface_note": ("the leader's stations lie on the object's "
                                                     "surface (contact points, not pad centres): "
                                                     "the pad spheres start buried by their radius "
                                                     "and this control does not apply to it"
                                                     if all(abs(d) < 1e-6 for d in d_on) else None),
                         "leader_station_surface_distance_note":
                             "median |station - object centre| - R while on; the pad radius "
                             "less the leader's own penetration",
                         "friction": "MuJoCo contact friction mu (condim 3) in place of the "
                                     "leader's analytic kernel; its stations were condim 1",
                         "contact_options": contact_options(model),
                         "integrator_source": ("Wave_manip sim/run_wave_unified.py "
                                               'integrator="implicit" at timestep 0.002; the '
                                               "station servo is unstable under Euler at that "
                                               "step (STATION_INTEGRATOR)"),
                         "object_contact": "MuJoCo defaults (solref 0.02 s)"},
            "rows": rows}


FEEDFORWARD = {"all": dict(contact=True, motion=True, weight=True, damping=True),
               "standing": dict(contact=True, motion=False, weight=True, damping=False)}


def replay(result, leader_json, lift="leader", drive="servo", centre_tol_m=None,
           window_s=WINDOW_S, windows=True, frozen=True, track=True, control=True,
           pads="leader", pad_k=None, feedforward="all", on_frame=None, hand=None):
    """The whole demonstration for one stored result (module docstring, "The
    runs"): the hand alone on the path (track), the frozen control, the
    re-seated windows, the free run and the stations positive control.
    drive is "servo" or "kinematic".
    pads says what takes up the check's allowed sink of the pads into the
    object (module docstring, "Pads"): "leader" (leader_pad_stiffness, the
    stiffness the leader's own contacts showed: the headline), "allowance"
    (pad_stiffness, the softest pad consistent with the check's allowed
    overlap), "default" (MuJoCo's default contact), or "given" with pad_k in
    N/m. feedforward (servo drive) is "all" (every torque the check
    computed: the headline) or "standing" (contact load and weight only;
    module docstring, "The drive"). All but the headlines are
    sensitivities. Returns the record to save."""
    t_start = time.time()
    if result["leader"] != leader_json["id"]:
        raise ValueError(f"the result is for leader {result['leader']}, not {leader_json['id']}")
    hand = hand or Hand(result["hand"])
    leader = fo.Leader(leader_json)
    if result.get("respace_deg"):
        leader.phi = np.radians(np.asarray(result["respace_deg"], float))
    Rw = np.array(result["placement"]["rotation_matrix"], float)
    t = np.array(result["placement"]["translation_m"], float)
    mu = float(leader_json.get("mu", 1.0))
    g_hand, grav = leader_gravity_hand(leader_json, Rw)
    allow = fo.overlap_allowance(leader_json)
    if centre_tol_m is None:
        centre_tol_m = float(allow["contact_m"])
    model, info = build_model(hand, leader, Rw, t, mu, g_hand)
    Rs = leader_rotations(leader_json, Rw)
    axis = turning_axis(Rs) if Rs is not None else None
    info["turn_axis"] = axis
    path, notes = stored_path(hand, result, leader, lift=lift, model=model, info=info, Rw=Rw, t=t)
    ks = [fr["k"] for fr in result["frames"]]
    k0, k1 = ks[0], ks[-1]
    geo = geometry_check(model, info, hand, path, result, leader, Rw, t)
    jumps = path_jumps(result, hand)
    k_link = sorted({f["k"] for f in jumps["link_changes"]})
    k_fast = sorted({f["k"] for f in jumps["above_no_load"]})
    if (pad_k is not None) != (pads == "given"):
        raise ValueError('pad_k goes with pads="given", and only there')
    if pads == "leader":
        K, src = leader_pad_stiffness(leader_json)
    elif pads == "allowance":
        K, f_med = pad_stiffness(leader_json, allow["contact_m"])
        src = (f"the leader's median contact force {f_med:.4f} N over the allowed "
               f"overlap {allow['contact_m']} m ({allow['source']['contact']})")
    elif pads == "given":
        K, src = float(pad_k), "given (--pad-k)"
    elif pads == "default":
        K, src = None, ("MuJoCo's default contact (solref 0.02 s), whose stiffness scales "
                        "with the object's mass; reached_N_m is what it gave this object")
    else:
        raise ValueError(f"unknown pads {pads!r}")
    if K is not None:
        pad = pad_contact(model, info, K, float(leader.mass), float(np.linalg.norm(g_hand)),
                          float(leader.R_obj))
    else:
        pad = {"reached_N_m": _measure_stiffness(None, None, None, float(leader.mass),
                                                 float(np.linalg.norm(g_hand)),
                                                 float(leader.R_obj))}
    pad["pads"] = pads
    pad["derivation"] = src
    if drive == "servo":
        drive_rec = servo_drive(model, info)
        ff = Feedforward(model, info, hand, result, leader, path, Rw)
        drive_rec["feedforward_notes"] = ff.notes
        if feedforward not in FEEDFORWARD:
            raise ValueError(f"unknown feedforward {feedforward!r}")
        drive_rec["feedforward"] = {"setting": feedforward, "parts": FEEDFORWARD[feedforward]}
        driver = ServoDriver(path, info, ff, drive_rec["ctrl_qpos"], **FEEDFORWARD[feedforward])
    elif drive == "kinematic":
        drive_rec = kinematic_drive(model, info)
        driver = KinematicDriver(path, info)
    else:
        raise ValueError(f"unknown drive {drive!r}")
    owner = finger_owner(hand)
    follows = {lab: int(i) for lab, i in result["assignment"].items()}
    og = info["object_geom"]
    rec = {"schema": "gramian-workshop/replay@1",
           "hand": result["hand"], "leader": leader_json["id"],
           "result_frames": len(ks), "leader_frames": len(leader.frames),
           "placement": result["placement"], "assignment": result["assignment"],
           "settings": {"drive": drive_rec, "lift": notes, "centre_tol_m": centre_tol_m,
                        "contact_options": contact_options(model),
                        "centre_tol_source": ("the task's allowed overlap at the contact "
                                              f"({allow['source']['contact']})"),
                        "turn_share_min": TURN_SHARE_MIN, "turn_min_deg": TURN_MIN_DEG,
                        "window_s": window_s, "timestep_s": float(model.opt.timestep),
                        "integrator": int(model.opt.integrator),
                        "object": {"radius_m": float(leader.R_obj), "mass_kg": float(leader.mass),
                                   "friction_mu": mu,
                                   "friction_source": ("leader file" if "mu" in leader_json
                                                       else "follow.py default"),
                                   "condim": 3, "pad_contact": pad},
                        "gravity": {"hand_frame_m_s2": g_hand.tolist(), "leader": grav},
                        "turn_axis_hand": axis.tolist() if axis is not None else None,
                        "leader_turn_total_deg": (math.degrees(signed_turns(Rs, axis)[-1])
                                                  if axis is not None else None)},
           "geometry_check": geo, "path_jumps": jumps}
    if track:
        if drive == "servo":
            tracker = ServoDriver(path, info, ff, drive_rec["ctrl_qpos"],
                                  **dict(FEEDFORWARD[feedforward], contact=False))
            rec["track"] = track_check(model, info, hand, tracker, path, leader, k0, k1)
        else:
            rec["track"] = {"note": "the kinematic drive sets the path exactly; not run"}
    if frozen:
        T0 = float(leader.frames[k0]["t"])
        rows_f = simulate(model, info, Frozen(driver, T0), leader, Rw, t, Rs, k0, k1, owner,
                          follows=follows, at_rest=True)
        rec["frozen"] = {"summary": summary(rows_f, leader, centre_tol_m), "rows": rows_f,
                         "seated": "at rest"}
    if windows:
        T = [float(f["t"]) for f in leader.frames]
        out = []
        verdict = {fr["k"]: fr["verdict"] for fr in result["frames"]}
        for k in ks:
            kend = k
            while kend + 1 < len(T) and T[kend + 1] <= T[k] + window_s + 1e-9:
                kend += 1
            if kend == k:
                continue
            r = simulate(model, info, driver, leader, Rw, t, Rs, k, kend, owner, follows=follows)
            out.append({"k": k, "t": T[k], "k_end": kend, "t_end": T[kend], "verdict": verdict[k],
                        "spans_link_change": _spans(k_link, k, kend),
                        "spans_above_no_load": _spans(k_fast, k, kend),
                        "centre_err_end_m": r[-1]["centre_err_m"],
                        "centre_err_max_m": max(x["centre_err_m"] for x in r),
                        "orient_err_end_deg": r[-1]["orient_err_deg"],
                        "turn_sim_deg": r[-1]["turn_sim_deg"],
                        "turn_leader_deg": r[-1]["turn_leader_deg"],
                        "contacts_end": r[-1]["contacts"]})
        by = {}
        for w in out:
            by.setdefault(w["verdict"], []).append(w)
        byc = {}
        for w in out:
            byc.setdefault("spans a branch change" if w["spans_link_change"]
                           else "no branch change", []).append(w)
        rec["windows"] = {"window_s": window_s, "n": len(out),
                          "by_verdict": {v: _window_stats(ws, centre_tol_m) for v, ws in by.items()},
                          "by_path_continuity": {v: _window_stats(ws, centre_tol_m)
                                                 for v, ws in byc.items()},
                          "rows": out}
    rows = simulate(model, info, driver, leader, Rw, t, Rs, k0, k1, owner, follows=follows,
                    on_frame=on_frame)
    rec["free"] = {"summary": summary(rows, leader, centre_tol_m), "rows": rows}
    if control:
        rec["stations_control"] = stations_control(leader, Rw, t, mu, g_hand, Rs, axis, k0, k1,
                                                   centre_tol_m, timestep=float(model.opt.timestep))
    rec["elapsed_s"] = round(time.time() - t_start, 1)
    return rec


def path_jumps(result, hand):
    """Where the check's own path jumps: at every checked frame, the fastest
    joint rate between it and the frame before (the check's continuity
    readout, "jump"), whether the finger changed IK branch there
    (link_changed), and the hand's published no-load speed to read the
    rates against. The replay drives the path as stored, jumps included:
    a jump the hand would have to make within one leader step is the
    check's finding, and what the replay then does with the object is its
    consequence, reported, not repaired. Returns the record."""
    from .hands import no_load_rad_s
    nl = no_load_rad_s(hand.spec)
    w0 = nl[0] if nl is not None else None
    frames = []
    for fr in result["frames"]:
        if not fr.get("fingers"):
            continue
        for x in fr["fingers"]:
            c = x.get("continuity") or {}
            jumps = c.get("jump") or []
            if not jumps:
                continue
            j = max(jumps, key=lambda e: abs(e["rate"]))
            frames.append({"k": fr["k"], "finger": x["finger"], "from_frame": c.get("from_frame"),
                           "dt_s": c.get("dt_s"), "joint": j["joint"], "dq": j["dq"],
                           "rate_rad_s": j["rate"], "link_changed": bool(c.get("link_changed")),
                           "spans_off": bool(c.get("spans_off"))})
    T = {fr["k"]: fr for fr in result["frames"]}
    link = [f for f in frames if f["link_changed"]]
    fast = [f for f in frames if w0 is not None and abs(f["rate_rad_s"]) > w0]
    return {"no_load_speed": None if nl is None else nl[1],
            "n_checked_with_jump_readout": len(frames),
            "fastest_rate_rad_s": max((abs(f["rate_rad_s"]) for f in frames), default=None),
            "link_changes": link,
            "above_no_load": fast,
            "first_link_change_k": link[0]["k"] if link else None,
            "first_above_no_load_k": fast[0]["k"] if fast else None,
            "note": "rates are the stored joint change over the leader step between checked "
                    "frames (follow's continuity readout); the replay drives them as stored"}


def _spans(ks_jump, k, kend):
    """Whether a checked frame in (k, kend] is a jump frame."""
    return any(k < kj <= kend for kj in ks_jump)


def _window_stats(ws, centre_tol_m):
    ends = [w["centre_err_end_m"] for w in ws]
    turns = [w["turn_sim_deg"] / w["turn_leader_deg"] for w in ws
             if w["turn_leader_deg"] is not None and abs(w["turn_leader_deg"]) > 1e-6]
    orient = [w["orient_err_end_deg"] for w in ws if w["orient_err_end_deg"] is not None]
    return {"n": len(ws),
            "centre_err_end_median_m": float(np.median(ends)),
            "centre_err_end_max_m": max(ends),
            "within_tol": int(sum(e <= centre_tol_m for e in ends)),
            "orient_err_end_median_deg": float(np.median(orient)) if orient else None,
            "turn_share_median": float(np.median(turns)) if turns else None}


def brief(rec):
    """A few printed lines on a replay record."""
    s = rec["free"]["summary"]
    drv = rec["settings"]["drive"]["drive"]
    out = [f"{rec['hand']} on {rec['leader']} ({drv} drive): free run {s['verdict']}; centre error max "
           f"{s['centre_err_max_m'] * 1e3:.2f} mm (tolerance {s['centre_tol_m'] * 1e3:g} mm); mean "
           f"offset {s['offset_mean_size_m'] * 1e3:.2f} mm, error about it max "
           f"{s['centre_err_about_mean_max_m'] * 1e3:.2f} mm"
           + (f"; first over at t={s['first_over_tol_t']:.3f} s"
              if s["first_over_tol_t"] is not None else "")
           + (f"; turn {s['turn_sim_deg']:.1f} of the leader's {s['turn_leader_deg']:.1f} deg "
              f"(share {s['turn_share']:.2f}); orientation error at the end "
              f"{s['orient_err_end_deg']:.1f} deg"
              if s["turn_share"] is not None else "")
           + f"; frames with no contact {s['frames_with_no_contact']} of {s['frames']}"]
    j = rec.get("path_jumps")
    if j:
        nl = j["no_load_speed"]
        out.append("   the check's path: fastest stored joint rate "
                   + (f"{j['fastest_rate_rad_s']:.1f} rad/s" if j["fastest_rate_rad_s"] is not None
                      else "none recorded")
                   + (f" (published no-load {nl['rad_s']:.1f} rad/s; {len(j['above_no_load'])} "
                      f"checked finger-frames above it)" if nl else " (no published no-load speed)")
                   + f"; IK branch changes at {len(j['link_changes'])} checked finger-frames"
                   + (f", first at k={j['first_link_change_k']}" if j["link_changes"] else ""))
    if "track" in rec and "joint_err_max_rad" in rec["track"]:
        tr = rec["track"]
        out.append(f"   hand alone on the path: joint error max {math.degrees(tr['joint_err_max_rad']):.2f} "
                   f"deg, fingertip error max {tr['tip_err_max_m'] * 1e3:.2f} mm, median "
                   f"{tr['tip_err_median_m'] * 1e3:.2f} mm")
    if "frozen" in rec:
        f = rec["frozen"]["summary"]
        fv = f.get("forces_vs_leader", {})
        out.append(f"   frozen hand (control, object seated at rest): {f['verdict']}; centre error max "
                   f"{f['centre_err_max_m'] * 1e3:.2f} mm, about its mean {f['centre_err_about_mean_max_m'] * 1e3:.2f} mm"
                   + (f"; turn {f['turn_sim_deg']:.1f} deg" if f["turn_sim_deg"] is not None else "")
                   + f"; frames with no contact {f['frames_with_no_contact']} of {f['frames']}"
                   + "; finger force / leader's: "
                   + ", ".join(f"{lab} {v['ratio_median']:.2f}" for lab, v in fv.items()
                               if v["ratio_median"] is not None))
    if "stations_control" in rec:
        c = rec["stations_control"]["summary"]
        cs = rec["stations_control"]["settings"]
        out.append(f"   leader's own stations (positive control): {c['verdict']}; centre error max "
                   f"{c['centre_err_max_m'] * 1e3:.2f} mm"
                   + (f"; turn {c['turn_sim_deg']:.1f} of {c['turn_leader_deg']:.1f} deg"
                      if c["turn_sim_deg"] is not None else "")
                   + ("; " + cs["stations_at_surface_note"] if cs.get("stations_at_surface_note") else ""))
    pad = rec["settings"]["object"]["pad_contact"]
    out.append(f"   pads: {pad['pads']}"
               + (f", asked {pad['asked_N_m']:.1f} N/m" if "asked_N_m" in pad else "")
               + f", reached {pad['reached_N_m']:.1f} N/m"
               + (f"; the leader's stations lie on the object's surface at "
                  f"{pad['derivation']['stations_at_surface_share']:.0%} of its contacts"
                  if isinstance(pad.get("derivation"), dict)
                  and pad["derivation"].get("stations_at_surface_share") else ""))
    if "windows" in rec:
        w = rec["windows"]
        for v, b in sorted(w.get("by_path_continuity", {}).items()):
            out.append(f"   {w['window_s']:g} s windows, {v}: {b['n']}; centre error at the end "
                       f"median {b['centre_err_end_median_m'] * 1e3:.2f} mm, within tolerance "
                       f"{b['within_tol']} of {b['n']}")
        for v, b in sorted(w["by_verdict"].items()):
            out.append(f"   {w['window_s']:g} s windows from {v} frames: {b['n']}; centre error at "
                       f"the end median {b['centre_err_end_median_m'] * 1e3:.2f} mm, max "
                       f"{b['centre_err_end_max_m'] * 1e3:.2f} mm, within tolerance "
                       f"{b['within_tol']} of {b['n']}"
                       + (f"; turn share median {b['turn_share_median']:.2f}"
                          if b["turn_share_median"] is not None else ""))
    g = rec["geometry_check"]
    out.append("   geometry at the checked frames: stored ball centre vs leader max "
               f"{(g['ball_center_vs_leader_max_m'] or 0) * 1e3:.3f} mm; stored pad point vs model "
               f"nearest point median {(g['pad_world_vs_model_nearest_median_m'] or 0) * 1e3:.2f} mm; "
               f"touching fingers sink median {(g['touching_sink_median_m'] or 0) * 1e3:.2f} mm, max "
               f"{(g['touching_sink_max_m'] or 0) * 1e3:.2f} mm"
               + (f"; lifted fingers clear min {g['lifted_clearance_min_m'] * 1e3:.2f} mm"
                  if g["lifted_clearance_min_m"] is not None else ""))
    out.append(f"   {rec['elapsed_s']} s")
    return "\n".join(out)
