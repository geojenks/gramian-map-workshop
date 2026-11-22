"""Four Shadow DEX-EE fingers arranged on a palm, turning a free cylinder.

Simulation only: comparative, not predictive. The controller knows the full
state and plans with the same model.

Same task, same controller, same settings and same pass test as
tools/hands_turn_free_cylinder_lookahead_21nov.py (read its header first);
its run, measure and film are used as they are. The three-finger DEX-EE run is
tools/dexee_turns_free_cylinder_lookahead_22nov.py. Only the hand is new.

The hand: not a product, the layout is ours
-------------------------------------------
The DEX-EE finger is a self-contained module: the MuJoCo Menagerie model
(shadow_dexee/shadow_dexee.xml, not edited) hangs three identical fingers on
the world, each by a mount that is a position on the base plane and a turn
about the base normal. Here four of those fingers stand on one flat palm, each
with a mount of that same form (x, y on the palm plane, turn about the palm
normal). Three are the model's own fingers, moved; the fourth is a copy of the
first, attached from a fresh read of the same file. The tool checks, before
anything else, that the copy has the same joint, actuator, body and contact
numbers as the stock finger and follows one command to the same joint angles.

The stock three-finger base body is removed (wrong shape for four). In its
place a round plate is drawn on the palm plane. The plate is drawn only: it
does not collide. It sits about 0.2 m below the cylinder, under the finger
bases. Finger bases are fixed to the world, as in the stock model; the model
is weightless as shipped and has no wrist.

Each finger follows one of the recording's four fingertips (first, middle,
ring, thumb); the fingers are identical, so which module takes which is
arbitrary.

Layout rule, fixed before any run
---------------------------------
One search over 15 numbers: the palm's tilt (two angles) and height, and each
finger's mount (x, y, turn). The palm stays under the cylinder's mean
position. Cost, over moments spread through the recording: fingertip miss
squared, plus CUT_WEIGHT times the squares of how far inside CLEAR (2 mm) any
part other than a fingertip body comes to the logged cylinder, how far inside
CLEAR any two fingers come to each other, and how far inside CLEAR any two
finger bases come. MuJoCo makes no contacts between world-fixed bodies, so
base overlap would go unnoticed in the run: the tool measures base-to-base
distance directly and refuses to run if two bases overlap.

Fingertip ball, targets, joint-speed limit and reference commands are made as
in the three-finger tool.

What to expect, written before the first run
--------------------------------------------
With all four fingertips followed, the contacts match the recording (three or
more fingers touch all the time), so the three-finger worry about holding on
two contacts does not apply. But the three-finger run did not fail that way:
it kept the cylinder 9 to 10 s and failed because fingers were blocked far
from their reference with actuators at their force limit, for a reason not
yet identified. Four fingers this large crowd more than three, so the same
may recur. Our guess: the cylinder is kept and turned, and the run fails the
force-limit line on at least one seed. This controller also fails narrowly on
the Allegro itself (and still does with the Allegro's torque cap lifted
tenfold, tools/allegro_turn_torque_cap_lifted_22nov.py). So a pass is evidence
that the gait ports to four DEX-EE fingers on a palm of our layout; a fail is
not evidence that it cannot.

The pass test is the one in the other tool's header, unchanged.
Nothing is overwritten: the output folder must not exist.
"""
import argparse
import json
import os
import pickle
import sys
import time

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shadow_follows_hora_gait_mujoco_21nov as G  # noqa: E402
import hands_turn_free_cylinder_lookahead_21nov as K  # noqa: E402
import dexee_turns_free_cylinder_lookahead_22nov as D  # noqa: E402

DEX_PATH = os.path.join(G.MEN, D.DEX_FILE)
WRAP = ["F0/", "F1/", "F2/", "F3/"]              # the body that carries each finger's mount
PRE = ["F0/", "F1/", "F2/", "F3/F0/"]            # name prefix of each finger's bodies, joints and actuators
NAMES = ["first", "middle", "ring", "thumb"]     # the log's order; finger k follows log finger k
CLEAR = 0.002                                    # m, see the layout rule
REF_SPEED = D.REF_SPEED
CUT_WEIGHT = G.CUT_WEIGHT
STARTS = (0.0, 0.6, -0.6)                        # rad, every finger turned this far from facing the cylinder
SCOUT_ITERS, SCOUT_MOMENTS, FIT_ITERS, FIT_MOMENTS = 400, 12, 800, 30
STEP = [0.1, 0.1, 0.02] + [0.02, 0.02, 0.3] * 4
PLATE_EDGE = 0.06                                # m, the drawn plate reaches this far past the outermost mount
EARLIER = D.EARLIER
THREE = os.path.join(G.ROOT, "outputs", "dexee_turns_free_cylinder_lookahead_22nov_three_seeds")
STEM = "four_dexee_fingers_on_a_palm_22nov"
N, LOG, WORD = 4, [0, 1, 2, 3], "four"            # finger count; the log finger each follows
TOOL = "four_dexee_fingers_on_a_palm_turn_free_cylinder_22nov.py"
OUT_STEM = "four_dexee_fingers_on_a_palm_turn_free_cylinder_22nov_"


def set_fingers(log, word, tool, out_stem, stem):
    """Added after the four-finger result (see tools/three_dexee_fingers_on_a_palm_turn_free_cylinder_22nov.py):
    the same tool with fewer fingers, each following the log finger listed. Not called for four."""
    global N, LOG, WORD, TOOL, OUT_STEM, STEM, WRAP, PRE, NAMES, STEP
    all_names = ["first", "middle", "ring", "thumb"]
    N, LOG, WORD, TOOL, OUT_STEM, STEM = len(log), list(log), word, tool, out_stem, stem
    WRAP, PRE, NAMES = WRAP[:N], PRE[:N], [all_names[j] for j in log]
    STEP = [0.1, 0.1, 0.02] + [0.02, 0.02, 0.3] * N


def rot(w):
    w = np.asarray(w, float)
    return G.rodrigues(w) if np.linalg.norm(w) > 1e-12 else np.eye(3)


def palm_hand(mounts):
    """mounts: four (x, y, turn) on the palm plane, or None for the layout-search model."""
    return {"file": D.DEX_FILE, "mounts": mounts, "tied": [],
            "fingers": [{"body": p + "finger_distal", "prefix": p, "groups": [[p + f"J{j}"] for j in range(4)],
                         "acts": [p + f"J{j}_actuator" for j in range(4)]} for p in PRE]}


def palm_build(hand, base_R, base_t, support, tied, L, xml_path=None):
    """The model: four fingers, no stock base. With mounts None the four mounts are left movable
    (mocap bodies) for the layout search; otherwise they are fixed where the mounts say."""
    spec = mujoco.MjSpec.from_file(DEX_PATH)
    spec.meshdir = os.path.join(G.MEN, os.path.dirname(D.DEX_FILE), "assets")
    for k in list(spec.keys):
        spec.delete(k)
    spec.delete(spec.body("hand_base"))
    child = mujoco.MjSpec.from_file(DEX_PATH)          # kept alive until compile
    if N == 4:
        spec.worldbody.add_body(name="F3/").add_frame().attach_body(child.body("F0/finger_base"), "F3/", "")
    mounts = hand["mounts"]
    for k, name in enumerate(WRAP):
        b = spec.body(name)
        if mounts is None:
            b.pos, b.quat, b.mocap = [0, 0, 0], [1, 0, 0, 0], True
        else:
            x, y, yaw = mounts[k]
            b.pos, b.quat = base_t + base_R @ np.array([x, y, 0.0]), G.matq(base_R @ rot([0, 0, yaw]))
    if mounts is not None:
        rad = max(float(np.hypot(x, y)) for x, y, _ in mounts) + PLATE_EDGE
        p = spec.worldbody.add_body(name="palm_plate", pos=base_t, quat=G.matq(base_R))
        p.add_geom(name="palm_plate", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[rad, 0.004, 0], pos=[0, 0, -0.004],
                   mass=0, contype=0, conaffinity=0, rgba=[0.55, 0.57, 0.6, 1])
    for g in spec.geoms:
        g.friction[0] = L["mu"]
    if support != "none":                              # the cylinder, as G.build makes it
        b = spec.worldbody.add_body(name="obj")
        b.add_freejoint(name="obj_free")
        b.add_geom(name="obj", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[L["R"], L["half"], 0],
                   mass=L["mass"], friction=[L["mu"], 0.005, 0.0001], rgba=[0.85, 0.75, 0.45, 1])
        for k, ang in enumerate((0.0, np.pi / 2)):      # marks, drawn only
            b.add_geom(name=f"mark{k}", type=mujoco.mjtGeom.mjGEOM_BOX,
                       size=[L["R"] * 1.01, 0.003, L["half"] * 1.02], mass=0,
                       quat=G.matq(G.rodrigues(np.array([0, 0, ang]))) if ang else [1, 0, 0, 0],
                       contype=0, conaffinity=0,
                       rgba=[0.75, 0.2, 0.15, 1] if k == 0 else [0.2, 0.3, 0.6, 1])
    m = spec.compile()
    del child
    if support != "none":
        assert m.joint("obj_free").qposadr[0] == 4 * N, "finger joints must come first"
    if xml_path:
        with open(xml_path, "x", encoding="utf-8") as fo:
            fo.write(spec.to_xml())
    return m


_stock_build = G.build


def _build(hand, *a, **k):
    return palm_build(hand, *a, **k) if "mounts" in hand else _stock_build(hand, *a, **k)


G.build = _build          # K.run and G's own code call G.build; other hands go to the stock builder unchanged


def copy_check(L, say):
    """Is every finger of the four-finger model the stock finger? Numbers, then one command stepped
    on each with contacts off (the fingers stand in different places, so contacts would differ)."""
    stock = mujoco.MjModel.from_xml_path(DEX_PATH)
    four = palm_build(palm_hand([[0.1 * np.cos(a), 0.1 * np.sin(a), a] for a in np.arange(N) * 2 * np.pi / N]),
                      np.eye(3), np.zeros(3), "none", False, L)
    mu0 = stock.geom_friction[:, 0].copy()

    def fields(m, pre):
        out = []
        for j in range(4):
            jn, a = m.joint(pre + f"J{j}"), m.actuator(pre + f"J{j}_actuator")
            dof = jn.dofadr[0]
            out += [*jn.range, m.dof_damping[dof], m.dof_armature[dof], m.dof_frictionloss[dof], *a.ctrlrange,
                    *a.forcerange, *a.gainprm[:3], *a.biasprm[:3], a.actnum[0], float(m.actuator_plugin[a.id] >= 0)]
        bs = [b for b in range(m.nbody) if m.body(b).name.startswith(pre + "finger_")]
        for b in bs:
            out += [m.body_mass[b], m.body_gravcomp[b], *m.body_inertia[b]]
        for g in range(m.ngeom):
            if m.geom_bodyid[g] in bs:
                out += [m.geom_contype[g], m.geom_conaffinity[g], m.geom_condim[g], *m.geom_friction[g, 1:],
                        *m.geom_solref[g], m.geom_margin[g]]
        return np.array(out, float)

    def trace(m, pre):
        m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
        d = mujoco.MjData(m)
        ids = [m.actuator(pre + f"J{j}_actuator").id for j in range(4)]
        qa = [m.joint(pre + f"J{j}").qposadr[0] for j in range(4)]
        T = []
        for s in range(1500):
            t = s * m.opt.timestep
            d.ctrl[ids] = [0.3 * np.sin(3 * t), -0.4 * np.sin(2 * t), 0.6 + 0.5 * np.sin(4 * t), 0.4 * np.sin(5 * t)]
            mujoco.mj_step(m, d)
            T.append(np.concatenate([d.qpos[qa], d.actuator_force[ids]]))
        return np.array(T)

    f0, t0 = fields(stock, "F0/"), trace(stock, "F0/")
    same = all(len(fields(four, p)) == len(f0) and np.array_equal(fields(four, p), f0) for p in PRE)
    worst = max(float(np.abs(trace(four, p) - t0).max()) for p in PRE)
    ok = bool(same and worst < 1e-9 and four.nu == 4 * N and four.na == 8 * N
              and np.allclose(four.geom_friction[:, 0], L["mu"]) and len(mu0))
    say(f"Copy check: {WORD}-finger model has {four.nu} inputs and {four.na} controller states (stock {stock.nu} and "
        f"{stock.na}); joint, actuator, body and contact numbers of every finger "
        f"{'equal' if same else 'DIFFER FROM'} the stock finger's; one 3 s command, contacts off, largest "
        f"difference from the stock finger in joint angle (rad) or actuator force (N.m) {worst:.1e}; swing of the "
        f"joints in that test {np.ptp(t0[:, :4], 0).min():.2f} to {np.ptp(t0[:, :4], 0).max():.2f} rad: "
        f"{'PASS' if ok else 'FAIL'}")
    return ok


class PalmIK(G.ShadowIK):
    def __init__(self, L):
        self.L = L
        self.hand = hand = palm_hand(None)
        self.m = m = G.build(hand, np.eye(3), np.zeros(3), "free", False, L)
        m.geom_margin[:] = CLEAR / 2                   # so the collision check reports anything nearer than CLEAR
        self.d = mujoco.MjData(m)
        self.mid = [int(m.body_mocapid[m.body(w).id]) for w in WRAP]
        self.f, self.module = [], {}
        for k, f in enumerate(hand["fingers"]):
            g = [[m.joint(j).id for j in grp] for grp in f["groups"]]
            b = m.body(f["body"]).id
            gid = m.geom(f["prefix"] + "distal_geom_tip_col").id
            mid = m.geom_dataid[gid]
            V = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            Vb = V @ G.qmat(m.geom_quat[gid]).T + m.geom_pos[gid]
            lo, hi = Vb.min(0), Vb.max(0)
            r = float(min(hi[0] - lo[0], hi[1] - lo[1]) / 2)
            loc = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, hi[2] - r])
            f["loc"], f["r"] = loc.tolist(), r
            self.f.append({"body": b, "loc": loc, "r": r,
                           "qadr": [[m.jnt_qposadr[j] for j in grp] for grp in g],
                           "dadr": [[m.jnt_dofadr[j] for j in grp] for grp in g],
                           "lo": np.array([m.jnt_range[grp[0]][0] for grp in g]),
                           "hi": np.array([m.jnt_range[grp[0]][1] for grp in g])})
            for bb in range(m.nbody):
                if m.body(bb).name.startswith(f["prefix"]):
                    self.module[bb] = k
        self.distal = {f["body"] for f in self.f}
        self.obj_geom = m.geom("obj").id
        self.base_geom = [m.geom(p + "base_geom_col").id for p in PRE]
        self.col = [[g for g in range(m.ngeom) if self.module.get(int(m.geom_bodyid[g])) == k and m.geom_contype[g]]
                    for k in range(N)]
        self.c0 = np.array([*L["op"].mean(0)[:2], 0.0])

    def pose(self, p):
        """Palm rotation and position, and the four mounts, from the 15 numbers."""
        return rot([p[0], p[1], 0.0]), self.c0 + np.array([0, 0, p[2]]), np.reshape(p[3:], (N, 3))

    def place(self, p):
        R, t, mounts = self.pose(p)
        for k, (x, y, yaw) in enumerate(mounts):
            self.d.mocap_pos[self.mid[k]] = t + R @ np.array([x, y, 0.0])
            self.d.mocap_quat[self.mid[k]] = G.matq(R @ rot([0, 0, yaw]))

    def base_gap(self):
        mujoco.mj_kinematics(self.m, self.d)
        return min(mujoco.mj_geomDistance(self.m, self.d, self.base_geom[a], self.base_geom[b], 0.05, None)
                   for a in range(N) for b in range(a + 1, N))

    def track(self, targets, idx):
        """Follow the targets at the moments idx. Returns joint targets, fingertip miss, and how far
        inside CLEAR other parts come to the cylinder and fingers come to each other."""
        L, m, d = self.L, self.m, self.d
        d.qpos[:] = 0
        x = [self.neutral(k) for k in range(N)]
        step = REF_SPEED * float(np.mean(np.diff(L["t"]))) if len(idx) == len(L["t"]) else None
        X = [np.zeros((len(idx), 4)) for _ in range(N)]
        miss = np.zeros((len(idx), N)); near_obj = np.zeros(len(idx)); near_ff = np.zeros(len(idx))
        for n, i in enumerate(idx):
            for k in range(N):
                new, miss[n, k] = self.solve(k, targets[i, k], x[k])
                if step is not None and n > 0:
                    new = np.clip(new, x[k] - step, x[k] + step)
                    for a_, v_ in zip(self.f[k]["qadr"], new):
                        d.qpos[a_] = v_
                    mujoco.mj_kinematics(m, d)
                    miss[n, k] = float(np.linalg.norm(targets[i, k] - self.tip(k)))
                x[k] = new
                X[k][n] = x[k]
            G.set_obj(m, d, "free", L["op"][i], L["oq"][i])
            mujoco.mj_kinematics(m, d); mujoco.mj_collision(m, d)
            for c in d.contact[:d.ncon]:
                b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
                if self.obj_geom in (c.geom1, c.geom2):
                    o = b2 if c.geom1 == self.obj_geom else b1
                    if o not in self.distal:
                        near_obj[n] = max(near_obj[n], CLEAR - c.dist)
                elif self.module.get(b1) != self.module.get(b2):
                    near_ff[n] = max(near_ff[n], CLEAR - c.dist)
        return X, miss, near_obj, near_ff

    def cost(self, p, targets, idx):
        self.place(p)
        _, miss, near_obj, near_ff = self.track(targets, idx)
        base = max(0.0, CLEAR - self.base_gap())
        return float((miss ** 2).mean() + CUT_WEIGHT * ((near_obj ** 2).mean() + (near_ff ** 2).mean() + base ** 2))

    def distances(self, X, idx):
        """True smallest distances along the track: other parts to the cylinder, finger to finger."""
        # Read from MuJoCo's own collision check with a wide margin (it lists every pair nearer than
        # 50 mm). mj_geomDistance on these mesh pairs returned exactly 0 at about a sixth of the
        # samples where the collision check, and a repeat call, gave 30 mm or more, so it is not used
        # here. Base to base is not covered (fixed bodies): that is base_gap.
        L, m, d = self.L, self.m, self.d
        to_obj, ff = 0.05, 0.05
        keep = m.geom_margin.copy()
        m.geom_margin[:] = 0.025
        for n, i in enumerate(idx):
            for k in range(N):
                for a_, v_ in zip(self.f[k]["qadr"], X[k][i]):
                    d.qpos[a_] = v_
            G.set_obj(m, d, "free", L["op"][i], L["oq"][i])
            mujoco.mj_kinematics(m, d); mujoco.mj_collision(m, d)
            for c in d.contact[:d.ncon]:
                b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
                if self.obj_geom in (c.geom1, c.geom2):
                    o = b2 if c.geom1 == self.obj_geom else b1
                    if o not in self.distal:
                        to_obj = min(to_obj, c.dist)
                elif self.module.get(b1) != self.module.get(b2):
                    ff = min(ff, c.dist)
        m.geom_margin[:] = keep
        return float(to_obj), float(ff)


def palm_reference(L, al, say):
    n = len(L["t"])
    kp = D.gains()
    say("DEX-EE proportional gains read from the model file, joints 0 to 3: " + ", ".join(f"{v:g}" for v in kp))
    ik = PalmIK(L)
    m, d = ik.m, ik.d
    out = np.zeros((n, N, 3)); targets = np.zeros((n, N, 3))
    for i in range(n):
        Ro = G.qmat(L["oq"][i])
        for k in range(N):
            out[i, k] = Ro @ G.cyl_out(Ro.T @ (al["tips"][i, LOG[k]] - L["op"][i]), L["R"], L["half"])
            targets[i, k] = al["tips"][i, LOG[k]] + (ik.f[k]["r"] - G.ALLEGRO["tips"][LOG[k]][2]) * out[i, k]
    # fingertip of a finger standing on an unturned mount at the origin, joints at their neutral angles
    d.qpos[:] = 0
    d.mocap_pos[:] = 0; d.mocap_quat[:] = [1, 0, 0, 0]
    for k in range(N):
        for a, v in zip(ik.f[k]["qadr"], ik.neutral(k)):
            d.qpos[a] = v
    mujoco.mj_kinematics(m, d)
    tip0 = ik.tip(0).copy()
    goal = targets.mean(0)
    idx_s = np.linspace(0, n - 1, SCOUT_MOMENTS).astype(int)
    idx_f = np.linspace(0, n - 1, FIT_MOMENTS).astype(int)
    best = None
    for turn in STARTS:
        # palm level; each finger placed so its neutral fingertip is on its mean target, curling
        # toward the cylinder's axis (the finger curls toward its own -y), then turned by `turn`
        p0 = [0.0, 0.0, float(goal[:, 2].mean() - tip0[2])]
        for k in range(N):
            u = goal[k, :2] - ik.c0[:2]
            yaw = float(np.arctan2(u[1], u[0]) - np.pi / 2 + turn)
            xy = goal[k, :2] - ik.c0[:2] - (rot([0, 0, yaw]) @ tip0)[:2]
            p0 += [float(xy[0]), float(xy[1]), yaw]
        p, c = G.nelder_mead(lambda q: ik.cost(q, targets, idx_s), np.array(p0), STEP, SCOUT_ITERS)
        say(f"  layout search, every finger started turned {np.degrees(turn):+.0f} deg from facing the cylinder: "
            f"cost {c:.3e}")
        if best is None or c < best[0]:
            best = (c, p)
    c_scout, p = best
    p, c = G.nelder_mead(lambda q: ik.cost(q, targets, idx_f), p, [s / 3 for s in STEP], FIT_ITERS)
    R, tr, mounts = ik.pose(p)
    ik.place(p)
    gap = float(ik.base_gap())
    say(f"Palm layout: cost {c_scout:.3e} after the scouting search, {c:.3e} after the fine one ({FIT_MOMENTS} "
        f"moments); palm tilted {np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))):.1f} deg from level, "
        f"{1e3 * (L['op'].mean(0)[2] - tr[2]):.0f} mm below the cylinder's mean centre; fingertip ball radius "
        f"{1e3 * ik.f[0]['r']:.1f} mm")
    for k in range(N):
        say(f"  mount of the finger following the {NAMES[k]} finger: {1e3 * mounts[k][0]:+.0f}, "
            f"{1e3 * mounts[k][1]:+.0f} mm on the palm, turned {np.degrees(mounts[k][2]) % 360:.0f} deg")
    X, miss, near_obj, near_ff = ik.track(targets, np.arange(n))
    to_obj, ff = ik.distances(X, np.arange(0, n, 10))
    say(f"  smallest distance between two finger bases {1e3 * gap:+.1f} mm (negative is overlap)")
    say(f"  along the reference, every tenth sample: other hand parts come no nearer than {1e3 * to_obj:+.1f} mm to "
        f"the logged cylinder; two fingers come no nearer than {1e3 * ff:+.1f} mm to each other (distances beyond "
        f"50 mm print as 50)")
    reach = {}
    for k in range(N):
        f = ik.f[k]
        at = float(np.mean(np.any((X[k] <= f["lo"] + 1e-4) | (X[k] >= f["hi"] - 1e-4), axis=1)))
        reach[NAMES[k]] = {"mean_mm": float(1e3 * miss[:, k].mean()), "max_mm": float(1e3 * miss[:, k].max()),
                           "share_of_time_at_a_joint_limit": at}
        say(f"  reach, finger following the {NAMES[k]} finger: mean miss {reach[NAMES[k]]['mean_mm']:.1f} mm, "
            f"largest {reach[NAMES[k]]['max_mm']:.1f} mm; at a joint limit {100 * at:.0f} % of the time")
    hand = palm_hand([[float(v) for v in mt] for mt in mounts])
    for f, g in zip(hand["fingers"], ik.hand["fingers"]):
        f["loc"], f["r"] = g["loc"], g["r"]
    m = G.build(hand, R, tr, "none", False, L)
    d = mujoco.MjData(m)
    ctrl = np.zeros((n, m.nu)); plain = np.zeros((n, m.nu)); qref = np.zeros((n, m.nq))
    for i in range(n):
        for k, f in enumerate(hand["fingers"]):
            for grp, v in zip(f["groups"], X[k][i]):
                qref[i, m.joint(grp[0]).qposadr[0]] = v
        d.qpos[:] = qref[i]; d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        for k, f in enumerate(hand["fingers"]):
            b = m.body(f["body"]).id
            tau = np.zeros(m.nv)
            if L["N"][i, LOG[k]] > 0:
                cpt = d.xpos[b] + d.xmat[b].reshape(3, 3) @ np.array(f["loc"])
                tau = G.point_jac(m, d, b, cpt - f["r"] * out[i, k]).T @ L["f"][i, LOG[k]]
            if i == 0:                                 # the fixed-mount model puts the fingertip where the search model did
                tip = d.xpos[b] + d.xmat[b].reshape(3, 3) @ np.array(f["loc"])
                assert np.linalg.norm(tip - targets[0, k]) < miss[0, k] + 1e-6, "fixed-mount model differs from the search model"
            for j, (grp, a, v) in enumerate(zip(f["groups"], f["acts"], X[k][i])):
                a = m.actuator(a).id
                plain[i, a] = v
                ctrl[i, a] = v + tau[m.joint(grp[0]).dofadr[0]] / kp[j]
    lo, hi = m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
    say(f"  reference commands outside the input range (clipped when played): "
        f"{100 * np.mean((ctrl < lo) | (ctrl > hi)):.1f} % of input samples; "
        f"largest push beyond the joint target {np.abs(ctrl - plain).max():.2f} rad")
    return {"R": R, "t": tr, "ctrl": ctrl, "plain": plain, "q0": qref[0], "qref": qref, "targets": targets,
            "reach": reach, "hand": hand, "kp": kp, "mounts": hand["mounts"], "layout_numbers": [float(v) for v in p],
            "base_gap_mm": 1e3 * gap, "other_parts_to_cylinder_mm_min": 1e3 * to_obj, "finger_to_finger_mm_min": 1e3 * ff,
            "near_mm": [float(1e3 * near_obj.max()), float(1e3 * near_ff.max())]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--seeds", default="0,1,2")
    ap.add_argument("--seconds", type=float, default=G.DUR)
    ap.add_argument("--refs", default="")                    # a references.pkl from an earlier run of this tool
    ap.add_argument("--no-film", action="store_true")
    a = ap.parse_args()
    out = os.path.join(G.ROOT, "outputs", OUT_STEM + a.tag)
    if os.path.exists(out):
        print("REFUSING to overwrite:", out)
        return 1
    os.makedirs(out)
    lines = []

    def say(s):
        print(s, flush=True)
        lines.append(s)

    def report():
        with open(os.path.join(out, "report.md"), "x", encoding="utf-8") as fo:
            fo.write(f"# {WORD.capitalize()} Shadow DEX-EE fingers on a palm turning a free cylinder (22 Nov 2026)\n\n"
                     "Simulation only: comparative, not predictive. Not a product hand: the layout is ours. The "
                     "controller knows the full state and plans with the same model. Printed by "
                     f"`tools/{TOOL}`; what is assumed, the layout "
                     "rule and what was expected before the run are in its header, and the pass test is in the "
                     "header of `tools/hands_turn_free_cylinder_lookahead_21nov.py`.\n\n"
                     + "\n".join("- " + s.strip() for s in lines) + "\n")

    L = G.load()
    sgn = np.sign(sum(float(np.einsum("ij,ij->", L["nrm"][:, k] * L["N"][:, k:k + 1],
                                      L["P"][:, k] - L["op"])) for k in range(4)))
    L["f"] = -sgn * L["nrm"] * L["N"][:, :, None]
    L["turn"] = G.own_axis_turn(L["oq"])
    L["axle_pos"], L["axle_quat"] = L["op"].mean(0), L["oq"][0]
    G.ALLEGRO["tips"] = G.hora_pads()
    say(f"Score settings: turn {K.TURN_TOL} rad, centre {K.POS_TOL} m, tilt {K.TILT_TOL} rad, change weight "
        f"{K.CHANGE_WEIGHT}, speed {K.SPEED_TOL} m/s, wobble {K.WOBBLE_TOL} rad/s")
    say(f"Look-ahead {K.HORIZON_S} s, new choice every {K.REPLAN_S} s, {K.SAMPLES} sampled changes, offsets spread "
        f"{K.SPREAD_RAD} rad, {K.KNOTS} knots; hold {K.HOLD_S} s before release; {a.seconds} s run; logged turn in "
        f"that time {np.degrees(np.interp(a.seconds, L['t'], L['turn'])):+.0f} deg")
    c4 = (L["N"] > 0).sum(1)
    say(f"Recording: all four fingers touch {100 * (c4 == 4).mean():.0f} % of the time, three "
        f"{100 * (c4 == 3).mean():.0f} %, fewer {100 * (c4 < 3).mean():.1f} %")
    if N < 4:
        cn = (L["N"][:, LOG] > 0).sum(1)
        say(f"Of the {N} log fingers followed ({', '.join(NAMES)}), all touch {100 * (cn == N).mean():.0f} % of "
            f"the time, all but one {100 * (cn == N - 1).mean():.0f} %, fewer {100 * (cn < N - 1).mean():.1f} %")
    if not copy_check(L, say):
        say("Stopped: the copy check failed.")
        report()
        return 1
    if a.refs:
        with open(a.refs, "rb") as fo:
            ref = pickle.load(fo)
        say(f"reference read from {os.path.relpath(a.refs, G.ROOT)}")
    else:
        al = G.allegro_reference(L, say)
        t0 = time.time()
        ref = palm_reference(L, al, say)
        say(f"  reference took {time.time() - t0:.0f} s to make")
    with open(os.path.join(out, "references.pkl"), "xb") as fo:
        pickle.dump(ref, fo)
    if ref["base_gap_mm"] < 0:
        say("Stopped: two finger bases overlap, so this layout could not be built. No run made.")
        report()
        return 1
    hand = ref["hand"]
    tips = [(f["body"], f["loc"]) for f in hand["fingers"]]
    xml = os.path.join(out, f"model_{WORD}_dexee_fingers.xml")
    res, panels, keep = {}, [], {}
    for label, fb, seeds in (("no feedback", False, [0]), ("feedback", True, [int(x) for x in a.seeds.split(",")])):
        for seed in seeds:
            t0 = time.time()
            m, rec, stopped, p0, ax0 = K.run(hand, ref, L, False, tips, seed, fb,
                                             None if os.path.exists(xml) else xml, a.seconds)
            ms = K.measure(m, rec, stopped, p0, ax0, L, a.seconds, NAMES)
            ms["wall_s"] = time.time() - t0
            early = rec["t"] <= 0.1
            v = np.linalg.norm(np.diff(rec["op"][early], axis=0), axis=1) / np.diff(rec["t"][early])
            ms["centre_speed_first_0p1_s_max_m_s"] = float(v.max()) if len(v) else None
            ms["largest_share_of_force_range_at_release"] = float(rec["act"][0].max())
            key = f"{WORD} dexee fingers, {label}" + (f", seed {seed}" if fb else "")
            res[key] = ms; keep[key] = rec
            say(f"{key}: {'PASS' if ms['passes'] else 'FAIL'}; held until {ms['held_until_s']:.2f} s; turned "
                f"{ms['turn_deg']:+.0f} deg (recording {ms['logged_turn_deg_same_time']:+.0f}); centre moved at most "
                f"{ms['centre_from_release_mm_max']:.0f} mm, axis tilted at most {ms['axis_tilt_deg_max']:.0f} deg; "
                f"most time any actuator spent at its limit {100 * ms['largest_share_of_time_any_actuator_at_limit']:.1f} %; "
                f"other hand parts pressing {100 * ms['other_hand_parts_pressing_share']:.0f} % of the time; "
                f"controller's mean change to the commands {ms['controller_offset_rad_mean_abs']:.3f} rad; "
                f"took {ms['wall_s']:.0f} s to compute")
            say("    failed: " + "; ".join(k for k, v_ in ms["pass_test"].items() if not v_)) if not ms["passes"] else None
            say("    hardest-worked actuators: " + "; ".join(
                f"{n} largest {100 * v_['largest_share_of_force_range']:.0f} %, mean {100 * v_['mean_share_of_force_range']:.0f} % of range"
                for n, v_ in ms["actuators_hardest_worked"].items()))
            say("    fingertips touching, share of time: " + ", ".join(
                f"{n} {100 * s_:.0f} %" for n, s_ in ms["fingertips_touching_share"].items())
                + f"; cylinder's largest speed in the first 0.1 s {ms['centre_speed_first_0p1_s_max_m_s']:.3f} m/s; "
                f"hardest-pressed actuator at release {100 * ms['largest_share_of_force_range_at_release']:.0f} % of range")
            if (not fb) or seed == seeds[0]:
                panels.append((f"{WORD.capitalize()} DEX-EE fingers on a palm" + (", no feedback" if not fb else ", with feedback"), xml, rec,
                               "no feedback: the reference commands as they are" if not fb else f"feedback, sampler seed {seed}"))
    np.savez(os.path.join(out, "runs.npz"), **{f"{k}|{n}": v for k, r in keep.items() for n, v in r.items()})
    with open(os.path.join(out, "results.json"), "x", encoding="utf-8") as fo:
        json.dump({"made_by": "tools/" + TOOL,
                   "settings": {"hold_s": K.HOLD_S, "horizon_s": K.HORIZON_S, "replan_s": K.REPLAN_S, "knots": K.KNOTS,
                                "samples": K.SAMPLES, "spread_rad": K.SPREAD_RAD, "turn_tol_rad": K.TURN_TOL,
                                "pos_tol_m": K.POS_TOL, "tilt_tol_rad": K.TILT_TOL, "change_weight": K.CHANGE_WEIGHT,
                                "speed_tol_m_s": K.SPEED_TOL, "wobble_tol_rad_s": K.WOBBLE_TOL,
                                "ref_speed_rad_s": REF_SPEED, "clear_m": CLEAR},
                   "reference": {k: ref[k] for k in ("mounts", "layout_numbers", "reach", "kp", "base_gap_mm",
                                                     "other_parts_to_cylinder_mm_min", "finger_to_finger_mm_min")},
                   "pass_test": K.PASS, "runs": res}, fo, indent=1, default=lambda o: np.asarray(o).tolist())
    report()
    print("wrote", os.path.join(out, "results.json"))
    if not a.no_film:
        earlier = []
        if a.seconds == G.DUR:
            for folder, key, model, title in (
                    (EARLIER, "allegro, feedback, seed 0", "model_allegro.xml", "Allegro, with feedback"),
                    (THREE, "dexee, feedback, seed 0", "model_dexee.xml", "Shadow DEX-EE, three fingers, with feedback")):
                if os.path.exists(os.path.join(folder, "runs.npz")):
                    Z = np.load(os.path.join(folder, "runs.npz"))
                    earlier.append((title, os.path.join(folder, model),
                                    {n: Z[f"{key}|{n}"] for n in ("t", "qpos", "turn")}, "feedback, sampler seed 0"))
        for f in K.film(out, L, earlier + panels, a.seconds, STEM):
            print("wrote", f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
