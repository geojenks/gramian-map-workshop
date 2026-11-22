"""Four two-pneu-net fingers, mounted like the fingers of a hand, turn a
cylinder that is carried on an axle: simulation with contact, numbers only
(the clip is drawn by a separate tool from the frames saved here).
SIMULATION ONLY (MuJoCo). Comparative, not predictive. Runs no SOFA.

Why: in the 21 Nov contact simulation the cylinder was free to tip and slide
(only its weight was carried) and it tipped up to 47 degrees, so the fingers
missed it. Here the question is narrower: if the cylinder is kept where the
recording had it, do the fingers' strokes, lifts and returns add up to the
recorded turn?

What is simulated, and what is simplified (say all of this with the clip):
  - The cylinder is on an axle. Its centre and the direction of its axis
    follow the recording exactly (with the recording's velocities, so the
    tips feel the carried motion); only the turn about its own axis is free.
    The axle has no friction and nothing else brakes the cylinder, so it
    coasts when no tip touches it. Its weight is carried by the axle.
  - Each finger is a tip ball on a spring, as in the 21 Nov contact
    simulation: radius half the finger's depth, TIP_MASS, critical damping,
    stiffness ASSUMED (the three values of that simulation, taken from the
    one-pneu-net stand-in; the two-pneu-net finger's own is not measured).
    The spring's rest position is where the free finger's tip would be for
    the commanded inputs: the ball sits just inside the tip face, its
    surface touching the centre of the face (centre of the face minus one
    radius along the face's outward normal; ASSUMED). Only the ball touches.
  - Where the fingers stand and their inputs: the hand-like mounting with
    roll up to 15 degrees, from the 21 Nov human-like-mounting reach check,
    unchanged. No tuning here.
  - REFERENCE run, not a finger: the springs' rest positions put the ball's
    surface on the logged contact points themselves (lifted between
    contacts as in the 21 Nov simulation). It says how much of the turn this
    stand-in and axle can pass on at best.
  - Timeline, friction, contact settings: as the 21 Nov simulation
    (two_pneunet_fingers_follow_rl_gait_clip_21nov.py). The gait is run
    SLOW times slower than the recording.

Measures and the rule, FIXED BEFORE THE FIRST RUN:
  1. Turn of the cylinder about its own axis over the gait, by the same
     running sum as the 21 Nov simulation, and as a share of the recorded
     turn. The axle's own angle is given beside it.
  2. Coasting: degrees turned during the gait in time steps when no tip
     pressed on the cylinder (normal force above the touch threshold), and
     degrees turned when at least one did.
  3. Per finger: share of the gait where touching agrees with the log;
     whole lifts; mean forces; how far the ball sat from its rest position
     (median, largest).
  4. Drawn against simulated: deepest cut of the finger's real skin, at its
     free shape for the commanded inputs, into the cylinder (the ball may
     be pushed back; the drawn skin is not).
  RULE for "the fingers turn the object on an axle": at the main stiffness,
  the two-input run turns the same way as the recording and by a larger
  share of the recorded turn than the 21 Nov contact simulation's two-input
  run at the main stiffness (read from its results file). Otherwise the
  21 Nov clip stands. The reference run is reported beside it either way.

FIXED ON 22 NOV, BEFORE THE REFIT WITH CONTACT ANYWHERE ON THE TIP HAD
FINISHED (its results were not seen when this was written):
  a. Where the ball goes for the refitted inputs. The refit says, for each
     logged contact sample, which skin node of the tip touches. The ball is
     put one radius behind that node along the skin's outward normal at
     that node (area-weighted mean of the triangles that share it), at the
     free shape for the commanded inputs (ASSUMED, as for the face centre
     above). Where the log has no contact for that finger, the node of the
     contact sample nearest in time is kept.
  b. Which run goes on the clip. The refitted run, if the refit passes its
     own reach-check rules and its two-input run at the main stiffness
     turns the same way as the recording and by a larger share than the
     fixed-carry two-input run with the face-centre ball. Otherwise the
     fixed-carry face-centre run, if it meets the RULE above. Otherwise the
     21 Nov clip stands.

CARRY_TIME (ASSUMED): the cylinder's centre and axis direction are held to
the recording by joint constraints of this time constant, whose targets
move with the recording. The first version overwrote those five
coordinates every step instead; a check run with the tips held back showed
that this turned the axle by itself (+12.1 deg over the gait), so it was
replaced before any result was reported. The check run (--tips-held-back)
is the test that the carry alone does not turn the cylinder.
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import mujoco as mj                                                            # noqa: E402
import pneunet_arrangement_replicates_rl_stroke_20nov as A                     # noqa: E402
import two_pneunet_fingers_follow_rl_gait_reach_check_21nov as RC              # noqa: E402
import two_pneunet_fingers_follow_rl_gait_clip_21nov as C                      # noqa: E402
import two_pneunet_fingers_human_like_mounting_reach_check_21nov as HM         # noqa: E402

OUT = os.path.join(ROOT, "outputs", "two_pneunet_fingers_turn_cylinder_on_axle_21nov")
EARLIER = os.path.join(ROOT, "outputs", "two_pneunet_fingers_follow_rl_gait_clip_bodies_clear_21nov", "gait_clip_results.json")
MOUNTING = "hand-like, roll up to 15 deg"
CARRY_TIME = 0.002        # s, ASSUMED, see the header
DT, SLOW, TIP_MASS, RETRACT = C.DT, C.SLOW, C.TIP_MASS, C.RETRACT
T_IN0, T_IN1, T_LET_GO, T_GAIT, T_AFTER = C.T_IN0, C.T_IN1, C.T_LET_GO, C.T_GAIT, C.T_AFTER
CLIP_SLOW, FPS = C.CLIP_SLOW, C.FPS
vec = C.vec


class Grid:
    """The finger's tip face (centre and outward normal) and skin, for any input pair, in the world."""
    def __init__(self, sh, x):
        self.lev = sh["levels"].astype(float)
        self.pad_tab, self.nrm_tab, self.skin = sh["pad"].astype(float), sh["pad_normal"].astype(float), sh["skin"]
        self.x, self.Rm = np.array(x[:3]), RC.rodrigues(np.array(x[3:6]))

    def _w(self, p1, p2):
        a = self.lev
        u = np.clip((p1 - a[0]) / (a[1] - a[0]), 0, len(a) - 1 - 1e-9)
        v = np.clip((p2 - a[0]) / (a[1] - a[0]), 0, len(a) - 1 - 1e-9)
        i, j = int(u), int(v)
        u, v = u - i, v - j
        return ((i, j, (1 - u) * (1 - v)), (i + 1, j, u * (1 - v)), (i, j + 1, (1 - u) * v), (i + 1, j + 1, u * v))

    def pad(self, p1, p2):
        return sum(w * self.pad_tab[i, j] for i, j, w in self._w(p1, p2)) @ self.Rm.T + self.x

    def normal(self, p1, p2):
        n = sum(w * self.nrm_tab[i, j] for i, j, w in self._w(p1, p2)) @ self.Rm.T
        return n / np.linalg.norm(n)

    def world(self, p1, p2):
        V = sum(w * self.skin[i, j].astype(float) for i, j, w in self._w(p1, p2))
        return V @ self.Rm.T + self.x


class Carry:
    """The logged centre and axis direction of the cylinder as the axle's five carried coordinates."""
    def __init__(self, L, s0, s1):
        self.t = L["t"][s0:s1 + 1]
        self.q0 = L["q"][s0]
        qi = A.inv(self.q0)[None]
        self.c, self.u = L["op"][s0:s1 + 1], L["axis"][s0:s1 + 1]
        ul = A.rot(qi, self.u)
        assert np.linalg.norm(ul[0] - [0, 0, 1]) < 1e-6, "the log's axis is not the cylinder's own z"
        s = A.rot(qi, self.c - self.c[0])
        self.tab = np.column_stack([s, np.arctan2(-ul[:, 1], ul[:, 2]), np.arcsin(np.clip(ul[:, 0], -1, 1))])

    def at(self, lt):
        return np.array([np.interp(lt, self.t, self.tab[:, i]) for i in range(5)])

    def pose(self, lt):
        return (np.array([np.interp(lt, self.t, self.c[:, i]) for i in range(3)]),
                np.array([np.interp(lt, self.t, self.u[:, i]) for i in range(3)]))


def build(L, s0, R, half, radius, homes, k_tip, key, out):
    mu, q0 = L["mu"], L["q"][s0]
    Id = np.diag(L["I"]) if L["I"].ndim == 2 else np.ravel(L["I"])[:3]
    ax = ("1 0 0", "0 1 0", "0 0 1")
    obj = (f'<body name="obj" pos="{vec(L["op"][s0])}" quat="{q0[3]:.9g} {q0[0]:.9g} {q0[1]:.9g} {q0[2]:.9g}" gravcomp="1">'
           + "".join(f'<joint name="carry_s{i}" type="slide" axis="{a}"/>' for i, a in enumerate(ax))
           + "".join(f'<joint name="carry_h{i}" type="hinge" axis="{a}"/>' for i, a in enumerate(ax[:2]))
           + '<joint name="axle" type="hinge" axis="0 0 1" armature="0" damping="0" frictionloss="0"/>'
           f'<inertial pos="{vec(L["com"])}" mass="{L["mass"]:.9g}" diaginertia="{vec(Id)}"/>'
           f'<geom name="obj" type="cylinder" size="{R:.9g} {half:.9g}" rgba="0.95 0.75 0.3 0.55"/></body>')
    tips = []
    for k, (name, h) in enumerate(homes):
        c = C.COL[name]
        tips.append(f'<body name="tip{k}" pos="{vec(h)}" gravcomp="1">'
                    + "".join(f'<joint name="tip{k}_{a}" type="slide" axis="{v}" stiffness="{k_tip:.9g}" '
                              f'damping="{2 * np.sqrt(k_tip * TIP_MASS):.9g}"/>' for a, v in zip("xyz", ax))
                    + f'<geom name="tip{k}" type="sphere" size="{radius:.9g}" mass="{TIP_MASS}" '
                    f'rgba="{c[0]} {c[1]} {c[2]} 1"/></body>')
    xml = "\n".join([
        f'<mujoco model="two_pneunet_fingers_cylinder_on_axle_{key}">',
        "<!-- generated by tools/two_pneunet_fingers_turn_cylinder_on_axle_21nov.py. SIMULATION ONLY."
        " The cylinder's centre and axis direction are held to a recording by joint constraints whose targets that script moves every step; only the joint 'axle' is free."
        " Each finger is a tip ball on a spring whose rest position is moved by that script. -->",
        f'<option gravity="0 0 -9.81" timestep="{DT}" integrator="implicitfast" cone="elliptic" impratio="{A.IMPRATIO}"/>',
        f'<default><geom friction="{mu} 0.005 0.0001" solref="{A.G.CONTACT_TIME} 1"/></default>',
        "<worldbody>", obj, *tips, "</worldbody>",
        "<equality>" + "".join(f'<joint name="hold_{n}" joint1="carry_{n}" polycoef="0 0 0 0 0" solref="{CARRY_TIME} 1"/>'
                               for n in ("s0", "s1", "s2", "h0", "h1")) + "</equality>", "</mujoco>", ""])
    path = os.path.join(out, f"model_{key}.xml")
    with open(path, "x", encoding="utf-8", newline="\n") as fh:
        fh.write(xml)
    m = mj.MjModel.from_xml_path(path)
    j = m.joint("carry_s0").id
    return {"m": m, "cq": int(m.jnt_qposadr[j]), "cv": int(m.jnt_dofadr[j]), "eq": [m.equality(f"hold_{n}").id for n in ("s0", "s1", "s2", "h0", "h1")],
            "aq": int(m.jnt_qposadr[m.joint("axle").id]), "av": int(m.jnt_dofadr[m.joint("axle").id]),
            "obj": m.body("obj").id, "obj_geom": m.geom("obj").id, "tip_geom": [m.geom(f"tip{k}").id for k in range(len(homes))],
            "tq": [int(m.jnt_qposadr[m.joint(f"tip{k}_x").id]) for k in range(len(homes))],
            "homes": [np.array(h) for _, h in homes], "path": path}


HEADER = ("t,part,log_t,turn_deg,axle_deg,any_tip_pressing,ox,oy,oz,qx,qy,qz,qw,"
          + ",".join(f"f{k}_input_first,f{k}_input_second,f{k}_touching,f{k}_normal_N,f{k}_tangential_N,"
                     f"f{k}_rest_x,f{k}_rest_y,f{k}_rest_z,f{k}_tip_x,f{k}_tip_y,f{k}_tip_z" for k in range(4)))
C0, NB = 13, 11


HELD_BACK = False         # --tips-held-back: a check that the carry alone does not turn the axle


def run(B, plans, centre_of, carry, t0, t1):
    """centre_of(k, lt) gives the ball's rest position for finger k at log time lt."""
    m = B["m"]
    d = mj.MjData(m)
    T_END = T_GAIT + (t1 - t0) * SLOW
    c0, u0 = carry.pose(t0)
    first = [centre_of(k, t0) for k in range(len(plans))]
    out = []
    for p in first:
        r = p - c0
        r -= (r @ u0) * u0
        out.append(r / np.linalg.norm(r))
    for k, (p, o) in enumerate(zip(first, out)):
        d.qpos[B["tq"][k]:B["tq"][k] + 3] = p + RETRACT * o - B["homes"][k]
    every, fevery = int(round(0.01 / DT)), int(round(SLOW / CLIP_SLOW / FPS / DT))
    log, frames = [], []
    mj.mj_forward(m, d)
    quat_prev = d.xquat[B["obj"]][[1, 2, 3, 0]].copy()
    turn, with_tip, without_tip, stopped, tiptip = 0.0, 0.0, 0.0, None, 0
    f6 = np.zeros(6)
    carry_off = np.zeros(5)
    nk = len(plans)
    for s in range(int(round((T_END + T_AFTER) / DT)) + 1):
        t = s * DT
        part = 0 if t < T_LET_GO else 1 if t < T_GAIT else 2 if t < T_END else 3
        a = 0.0 if HELD_BACK else float(np.clip((t - T_IN0) / (T_IN1 - T_IN0), 0, 1))
        lt = t0 + float(np.clip((t - T_GAIT) / SLOW, 0, t1 - t0))
        lt2 = t0 + float(np.clip((t + DT - T_GAIT) / SLOW, 0, t1 - t0))
        tg = [centre_of(k, lt) + (1 - a) * RETRACT * o for k, o in enumerate(out)]
        for k, p in enumerate(tg):
            m.qpos_spring[B["tq"][k]:B["tq"][k] + 3] = p - B["homes"][k]
        v0, v1 = carry.at(lt), carry.at(lt2)
        if s == 0:
            d.qpos[B["cq"]:B["cq"] + 5] = v0
            d.qvel[B["cv"]:B["cv"] + 5] = (v1 - v0) / DT
        m.eq_data[B["eq"], 0] = v1
        off = np.abs(d.qpos[B["cq"]:B["cq"] + 5] - v0)
        carry_off = np.maximum(carry_off, off)
        if part == 0:
            d.qpos[B["aq"]], d.qvel[B["av"]] = 0.0, 0.0
        mj.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)):
            stopped = ("non-finite state", t)
            break
        q = d.xquat[B["obj"]][[1, 2, 3, 0]].copy()
        dq = A.qmul(A.inv(quat_prev), q)
        dturn = 2 * dq[2] * np.sign(dq[3])
        turn += dturn
        quat_prev = q
        Fn, Ft = np.zeros(nk), np.zeros(nk)
        for ci in range(d.ncon):
            c = d.contact[ci]
            pair = {int(c.geom1), int(c.geom2)}
            for k in range(nk):
                if B["tip_geom"][k] in pair:
                    if B["obj_geom"] not in pair:
                        tiptip += 1
                        continue
                    mj.mj_contactForce(m, d, ci, f6)
                    Fn[k] += f6[0]
                    Ft[k] += float(np.hypot(f6[1], f6[2]))
        pressing = bool((Fn > A.TOUCH_N).any())
        if part == 2:
            if pressing:
                with_tip += dturn
            else:
                without_tip += dturn
        if s % every == 0 or s % fevery == 0:
            row = []
            for k, P in enumerate(plans):
                tip = B["homes"][k] + d.qpos[B["tq"][k]:B["tq"][k] + 3]
                row += [*P.inputs(lt), float(Fn[k] > A.TOUCH_N), Fn[k], Ft[k], *tg[k], *tip]
            rec = [t, part, lt, np.degrees(turn), np.degrees(d.qpos[B["aq"]]), float(pressing), *d.xpos[B["obj"]], *q, *row]
            if s % every == 0:
                log.append(rec)
            if s % fevery == 0 and t >= T_GAIT - 1e-9:
                frames.append(rec)
    return {"log": np.array(log), "frames": np.array(frames), "stopped": stopped, "tip_tip_contacts_logged": tiptip,
            "carry_off_target_largest_mm": float(carry_off[:3].max() * 1e3), "carry_off_target_largest_deg": float(np.degrees(carry_off[3:].max())),
            "turn_with_a_tip_pressing_deg": float(np.degrees(with_tip)), "turn_with_no_tip_pressing_deg": float(np.degrees(without_tip))}


def measures(L, plans, r, rec_turn, grids, carry, R, half):
    lg = r["log"]
    g = lg[lg[:, 1] == 2]
    wait = lg[lg[:, 1] == 1]
    after = lg[lg[:, 1] == 3]
    out = {"stopped": r["stopped"], "tip_tip_contacts_logged": r["tip_tip_contacts_logged"],
           "turn_in_wait_deg": float(wait[-1, 3] - wait[0, 3]),
           "turn_gait_deg": float(g[-1, 3] - g[0, 3]), "axle_angle_change_gait_deg": float(g[-1, 4] - g[0, 4]),
           "turn_after_the_gait_deg": float(after[-1, 3] - after[0, 3]) if len(after) else float("nan"),
           "turn_with_a_tip_pressing_deg": r["turn_with_a_tip_pressing_deg"],
           "turn_with_no_tip_pressing_deg": r["turn_with_no_tip_pressing_deg"],
           "carry_off_target_largest_mm": r["carry_off_target_largest_mm"],
           "carry_off_target_largest_deg": r["carry_off_target_largest_deg"],
           "share_of_gait_time_with_no_tip_pressing": float((g[:, 5] < 0.5).mean()), "fingers": {}}
    out["share_of_recorded_turn"] = out["turn_gait_deg"] / rec_turn
    for k, P in enumerate(plans):
        c = C0 + NB * k
        F = P.F
        log_touch = np.interp(g[:, 2], F.t, F.touch.astype(float)) > 0.5
        sim = g[:, c + 2] > 0.5
        lam = L["lam"][F.idx, F.j]
        off = 1e3 * np.linalg.norm(g[:, c + 5:c + 8] - g[:, c + 8:c + 11], axis=1)
        fo = {"share_of_gait_agreeing_with_log": float((sim == log_touch).mean()),
              "share_touching": float(sim.mean()), "log_share_touching": float(log_touch.mean()),
              "whole_lifts": C.lifts(sim), "log_whole_lifts": C.lifts(log_touch),
              "normal_force_mean_N": float(g[sim, c + 3].mean()) if sim.any() else 0.0,
              "tangential_force_mean_N": float(g[sim, c + 4].mean()) if sim.any() else 0.0,
              "log_normal_force_mean_N": float(lam[F.touch & (lam > A.TOUCH_N)].mean()),
              "ball_from_rest_position_median_mm": float(np.median(off)), "ball_from_rest_position_max_mm": float(off.max())}
        if grids is not None:
            cut = []
            for row in g[::10]:
                cc, uu = carry.pose(row[2])
                cut.append(-RC.cyl_sdf(grids[k].world(row[c], row[c + 1]), cc, uu, R, half).min())
            fo["drawn_skin_deepest_cut_mm"] = float(1e3 * max(cut))
            fo["drawn_skin_share_of_gait_cutting_in"] = float((np.array(cut) > 0).mean())
        out["fingers"][P.name] = fo
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", required=True)
    ap.add_argument("--gramian-map", required=True)
    ap.add_argument("--tag", default="")
    ap.add_argument("--tips-held-back", action="store_true",
                    help="check run: the tips stay drawn back all through, main stiffness only; the axle should not turn")
    args = ap.parse_args()
    global HELD_BACK
    HELD_BACK = args.tips_held_back
    out = OUT + (("_" + args.tag) if args.tag else "")
    os.makedirs(out, exist_ok=False)
    W = HM.setup(args.wave_manip, args.gramian_map)
    L, R, half, tab, sh, idx, s0, s1 = W["L"], W["R"], W["half"], W["tab"], W["sh"], W["idx"], W["s0"], W["s1"]
    t = L["t"]
    t0, t1 = float(t[idx[0]]), float(t[idx[-1]])
    radius = 0.5 * RC.tip_table(args.gramian_map)["depth"]
    C.FINGER = lambda L_, idx_, j_, tab_, R_, half_: HM.Mounted(L_, idx_, j_, tab_, R_, half_, None)   # noqa: E731
    res = json.load(open(os.path.join(HM.OUT, "reach_check_results.json")))
    fits = res["mountings"][MOUNTING]
    plans = [C.Plan(L, idx, j, tab, R, half, fits[j], None) for j in range(4)]
    grids = [Grid(sh, fits[j]["x"]) for j in range(4)]
    refs = [C.Reference(L, idx, j, tab, R, half) for j in range(4)]
    carry = Carry(L, s0, int(idx[-1]))
    # build checks
    worst = 0.0
    for lt in np.linspace(t0, t1, 9):
        for P, G in zip(plans, grids):
            p1, p2 = P.inputs(lt)
            worst = max(worst, float(np.linalg.norm(P.target(lt) - G.pad(p1, p2))))
    assert worst < 1e-6, f"the plan's tip position is not the tip face's centre: {worst}"

    def finger_centre(k, lt):
        p1, p2 = plans[k].inputs(lt)
        return grids[k].pad(p1, p2) - radius * grids[k].normal(p1, p2)

    def ref_centre(k, lt):
        p = refs[k].target(lt)
        cc, uu = carry.pose(lt)
        r = p - cc
        r -= (r @ uu) * uu
        return p + radius * r / np.linalg.norm(r)

    up = json.load(open(C.UPRIGHT))
    ks = (float(np.mean([f["k_along_surface_N_per_m"] for f in up["fits"]])),
          float(np.mean([f["k_out_of_plane_N_per_m"] for f in up["fits"]])))
    stiff = {"main": float(np.sqrt(ks[0] * ks[1])), "soft": ks[0], "stiff": ks[1]}
    rec_turn = float(np.degrees(L["turn"][idx[-1]] - L["turn"][idx[0]]))
    earlier = json.load(open(EARLIER))
    e_turn = earlier["runs"]["two_inputs_main_weight_carried"]["turn_gait_deg"]
    e_share = e_turn / earlier["recorded_turn_deg"]
    # how fast the carried motion is, against the turn (why the carry needs real velocities)
    ii = np.arange(s0, int(idx[-1]) + 1)
    dtt = np.diff(t[ii])
    v_c = np.linalg.norm(np.diff(L["op"][ii], axis=0), axis=1) / dtt
    v_t = R * np.abs(np.diff(L["turn"][ii])) / dtt
    names = [L["names"][j] for j in range(4)]
    sets = {"two_inputs": (plans, finger_centre, grids), "logged_path": (refs, ref_centre, None)}
    what = {"two_inputs": "four two-pneu-net fingers, both inputs driven",
            "logged_path": "REFERENCE, not a finger: the ball's surface on the logged contacts themselves"}
    rep = ["Four two-pneu-net fingers, mounted like the fingers of a hand, turn a cylinder carried on an axle: simulation with contact",
           "SIMULATION ONLY. Comparative, not predictive.", "",
           f"Where the fingers stand and their inputs: {os.path.relpath(HM.OUT, ROOT).replace(os.sep, '/')}/reach_check_results.json, "
           f"mounting '{MOUNTING}', unchanged.",
           f"Recording: cylinder radius {R * 1e3:.1f} mm, half length {half * 1e3:.1f} mm, mass {L['mass'] * 1e3:.1f} g, "
           f"friction {L['mu']:.2f}. Window: log {t0:.2f} to {t1:.2f} s; the cylinder turns {rec_turn:+.1f} deg in it.",
           "The cylinder is on an axle: its centre and axis direction follow the recording, with the recording's velocities; only "
           "the turn about its own axis is free. The axle has no friction, so the cylinder coasts when nothing touches it. "
           f"In the recording the centre moves at a median {np.median(v_c) * 1e3:.1f} mm/s and the surface turns at a median "
           f"{np.median(v_t) * 1e3:.1f} mm/s, so the carried motion is not small beside the turn.",
           f"Finger stand-in: a tip ball of radius {radius * 1e3:.1f} mm and {TIP_MASS * 1e3:g} g (ASSUMED) on a spring. Its rest "
           "position puts the ball's surface on the centre of the free finger's tip face for the commanded inputs. Only the ball "
           f"touches. Check: the plan's tip position and the tip face's centre differ by at most {worst * 1e3:.2g} mm.",
           "Tip stiffness (ASSUMED, same in all directions; from the one-pneu-net stand-in of the 21 Nov upright arrangement): "
           + ", ".join(f"{k} {v:.1f} N/m" for k, v in stiff.items()) + ".",
           f"Timeline (ASSUMED): tips brought in {T_IN0:g} to {T_IN1:g} s with the turn held; let go at {T_LET_GO:g} s; "
           f"gait from {T_GAIT:g} s, run {SLOW:g} times slower than the recording ({(t1 - t0) * SLOW:.1f} s); then {T_AFTER:g} s "
           "with the last targets held.",
           f"Friction tip to cylinder {L['mu']:.2f} (ASSUMED: the log's value). Elliptic cone, impratio {A.IMPRATIO}, time step {DT} s.",
           f"Earlier, for the rule: the 21 Nov contact simulation (cylinder free to tip and slide), two inputs, main stiffness, "
           f"turned {e_turn:+.1f} deg, {100 * e_share:.0f}% of the recorded turn.", ""]
    results = {"window_log_s": [t0, t1], "recorded_turn_deg": rec_turn, "stiffness_N_per_m": stiff, "tip_ball_radius_m": radius,
               "mounting": MOUNTING, "earlier_two_input_share": e_share, "runs": {}}
    if HELD_BACK:
        rep += ["CHECK RUN: the tips are held drawn back from the cylinder all through, so nothing should turn it. "
                "The rule line at the end does not apply to this run.", ""]
    for sname, k_tip in ({"main": stiff["main"]} if HELD_BACK else stiff).items():
        for name, (pl, centre, gr) in sets.items():
            key = f"{name}_{sname}"
            homes = [(names[k], centre(k, t0)) for k in range(4)]
            B = build(L, s0, R, half, radius, homes, k_tip, key, out)
            r = run(B, pl, centre, carry, t0, t1)
            np.savetxt(os.path.join(out, f"log_{key}.csv"), r["log"], delimiter=",", header=HEADER, comments="", fmt="%.7g")
            np.savetxt(os.path.join(out, f"frames_{key}.csv"), r["frames"], delimiter=",", header=HEADER, comments="", fmt="%.9g")
            ms = measures(L, pl, r, rec_turn, gr, carry, R, half)
            results["runs"][key] = ms
            rep += [f"{what[name]}; stiffness {sname} ({k_tip:.1f} N/m):",
                    f"  Turn over the gait {ms['turn_gait_deg']:+.1f} deg, {100 * ms['share_of_recorded_turn']:.0f}% of the recorded "
                    f"{rec_turn:+.1f} deg (the axle's own angle changed {ms['axle_angle_change_gait_deg']:+.1f} deg). In the wait "
                    f"before the gait {ms['turn_in_wait_deg']:+.1f} deg; in the {T_AFTER:g} s after it {ms['turn_after_the_gait_deg']:+.1f} deg.",
                    f"  Coasting: {ms['turn_with_a_tip_pressing_deg']:+.1f} deg turned while at least one tip pressed, "
                    f"{ms['turn_with_no_tip_pressing_deg']:+.1f} deg while none did "
                    f"({100 * ms['share_of_gait_time_with_no_tip_pressing']:.0f}% of the gait's time had no tip pressing above "
                    f"{A.TOUCH_N:g} N). The carried cylinder is off the recording by at most {ms['carry_off_target_largest_mm']:.3f} mm "
                    f"and {ms['carry_off_target_largest_deg']:.3f} deg."
                    + (f" STOPPED: {ms['stopped']}." if ms["stopped"] else "")]
            for fn, fo in ms["fingers"].items():
                rep.append(f"    {fn}: touching agrees with the log {100 * fo['share_of_gait_agreeing_with_log']:.0f}% of the gait "
                           f"(touching {100 * fo['share_touching']:.0f}%, log {100 * fo['log_share_touching']:.0f}%); whole lifts "
                           f"{fo['whole_lifts']} (log {fo['log_whole_lifts']}); normal force {fo['normal_force_mean_N']:.3f} N "
                           f"(log {fo['log_normal_force_mean_N']:.3f} N), tangential {fo['tangential_force_mean_N']:.3f} N; ball from "
                           f"its rest position median {fo['ball_from_rest_position_median_mm']:.1f} mm, largest "
                           f"{fo['ball_from_rest_position_max_mm']:.1f} mm"
                           + (f"; drawn skin cuts into the cylinder at most {fo['drawn_skin_deepest_cut_mm']:.1f} mm "
                              f"({100 * fo['drawn_skin_share_of_gait_cutting_in']:.0f}% of the gait)" if "drawn_skin_deepest_cut_mm" in fo
                              else "") + ".")
            rep.append("")
            print("\n".join(rep[-7:]), flush=True)
    m2 = results["runs"]["two_inputs_main"]
    ok = bool(np.sign(m2["turn_gait_deg"]) == np.sign(rec_turn) and m2["share_of_recorded_turn"] > e_share)
    results["rule_met"] = ok
    rep += ["RULE (fixed in the header before the first run): at the main stiffness the two-input run turns the same way as the "
            "recording and by a larger share than the 21 Nov contact simulation's.",
            f"  {'MET' if ok else 'NOT MET'}: {m2['turn_gait_deg']:+.1f} deg, {100 * m2['share_of_recorded_turn']:.0f}% against "
            f"{100 * e_share:.0f}%. The reference on the same axle: "
            f"{results['runs']['logged_path_main']['turn_gait_deg']:+.1f} deg, "
            f"{100 * results['runs']['logged_path_main']['share_of_recorded_turn']:.0f}%.",
            "", "Not modelled: the finger's body and its bending under load (only a tip ball on a spring touches); the palm; "
            "anything holding the cylinder other than the axle; pressure dynamics."]
    with open(os.path.join(out, "axle_report.txt"), "x", encoding="utf-8") as fh:
        fh.write("\n".join(rep) + "\n")
    with open(os.path.join(out, "axle_results.json"), "x", encoding="utf-8") as fh:
        json.dump(results, fh, indent=1)
    print("\n".join(rep[-5:]))


if __name__ == "__main__":
    main()
