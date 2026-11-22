"""Four two-pneu-net fingers follow a recorded learned gait, lifting and
landing: MuJoCo with contact, and a three-column clip (21 Nov 2026).

SIMULATION ONLY. Comparative, not predictive.

Follows tools/two_pneunet_fingers_follow_rl_gait_reach_check_21nov.py (geometry
only) and reads its saved result: where each finger stands and which input
pair it takes at each touching sample. Here the same fingers are run against
a cylinder that is free to turn.

THE FINGER STAND-IN. No force measurement exists for the two-pneu-net finger,
so the finger is NOT simulated as a body. Each finger is:
  - a tip ball, radius half the finger's depth (as in the reach check), on
    three sliding joints, pulled by a spring towards a target;
  - the target is the measured free tip position for the commanded input
    pair (the 20 Nov table of settled free shapes), placed in the world by
    the reach check's six numbers. It is open-loop: it does not know where
    the cylinder is.
  - only the tip ball touches anything. The finger body is DRAWN, as two
    straight pieces from the clamped end through the measured junction to
    the tip. A separate check below says how far those pieces, at the
    finger's half depth, would cut into the cylinder and each other.
The spring moves its rest position (it is not a driven body), so the tip has
a real velocity and friction can drag the cylinder. A check at the start
(one tip dragging a block) confirms that.

THE INPUTS. While the log says a fingertip touches: the reach check's input
pair at each fourth log sample, joined by straight lines in time. While it
is off: the return the reach check judged (tip climbs steadily away, travels
at the best clearance, descends steadily), rebuilt here with its route, timed
evenly along the tip's path. The largest input change between samples is
printed; nothing is rate-limited.

ASSUMED, all fixed before the first run and not tuned afterwards:
  - tip stiffness, the same in every direction. Three values, read from the
    21 Nov upright-arrangement results for the ONE-pneu-net stand-in: its tip
    stiffness in its bending plane, square to that plane, and the geometric
    mean of the two. The mean is the main case and the clip. The
    two-pneu-net finger is twice as long, so it is probably softer.
  - tip ball mass TIP_MASS; its weight is not applied (the table of free
    shapes already holds the finger's own sag in the pose it was measured).
  - friction between tip and cylinder: the log's value for the cylinder.
  - the gait is run SLOW times slower than the recording.
  - support: "weight carried" (the cylinder slides and turns freely but
    cannot fall; a palm would carry it) is the main case; "cylinder free"
    is also run.
  - contact model as in the earlier clips (elliptic cone, same time step).

RUNS, all reported whatever they give:
  two inputs    the reach check's fit with both inputs free.
  one input     the reach check's control: one input held constant.
  logged path   REFERENCE, not a finger: the same tip balls and springs with
                targets at the logged contacts themselves, lifted
                REF_LIFT_MM straight out from the cylinder's axis between
                lift-off and landing. It shows what this stand-in does with
                no reach limit, so a low turn can be put down to reach or
                to the stand-in.
Each at the three stiffnesses with the weight carried, and at the main
stiffness with the cylinder free.

MEASURES, fixed here before the first run:
  1. cylinder turn about its own axis over the gait, against the recorded
     turn over the same window, and over the first round of the gait;
  2. turn in the wait before the gait (drift);
  3. largest change of tilt, largest movement of the centre, largest drop;
  4. per finger: share of the gait in which touching / not touching agrees
     with the log; number of whole lifts made against the log's; mean
     normal force while touching against the log's.
  5. finger bodies: deepest cut of the drawn pieces (half depth) into the
     cylinder, more than one depth back from the tip, on the logged motion
     and in each run; smallest gap between different fingers' pieces.

Run from the repository root (needs mujoco, ffmpeg):

    python tools/two_pneunet_fingers_follow_rl_gait_clip_21nov.py --wave-manip <Wave_manip> --gramian-map <gramian_map>

Writes outputs/two_pneunet_fingers_follow_rl_gait_clip_21nov/.

Option --bodies-clear (added later on 21 Nov, after the first clips showed the
drawn bodies cutting into the cylinder): reads instead the placements of
tools/two_pneunet_fingers_follow_rl_gait_reach_check_bodies_clear_21nov.py,
which refits where the fingers stand with the drawn body kept out of the
logged cylinder, uses that tool's rule for which input pair is taken and
which pairs a return may pass through, and writes
outputs/two_pneunet_fingers_follow_rl_gait_clip_bodies_clear_21nov/. Nothing
else differs: same stand-in, stiffnesses, timeline, friction and measures.
Without the option the tool does what it did before.
"""
import argparse
import csv
import heapq
import json
import os
import shutil
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
import mujoco as mj                                                          # noqa: E402
import pneunet_arrangement_replicates_rl_stroke_20nov as A                   # noqa: E402
import pneunet_upright_arrangement_replicates_rl_stroke_21nov as U           # noqa: E402
import two_pneunet_fingers_follow_rl_gait_reach_check_21nov as RC            # noqa: E402

OUT = os.path.join(ROOT, "outputs", "two_pneunet_fingers_follow_rl_gait_clip_21nov")
REACH = os.path.join(ROOT, "outputs", "two_pneunet_fingers_follow_rl_gait_reach_check_21nov", "reach_check_results.json")
UPRIGHT = os.path.join(ROOT, "outputs", "pneunet_upright_arrangement_replicates_rl_stroke_21nov", "arrangement_results.json")
DT = A.DT
SLOW = A.RL_SLOW          # ASSUMED: the gait is run this many times slower than the recording
TIP_MASS = 0.005          # ASSUMED, kg
RETRACT = 0.030           # tips start this far out from the cylinder and are brought in
T_IN0, T_IN1, T_LET_GO, T_GAIT = 0.2, 1.0, 1.5, 2.5   # s of simulated time; ASSUMED
T_AFTER = 1.0             # s held after the gait
REF_LIFT_MM = 10.0        # ASSUMED: the reference run's lift between lift-off and landing
CLIP_SLOW = 3.0           # the clip shows the recording this many times slower
CW, CH, HEAD, FOOT = 640, 720, 96, 92
FPS = A.FPS
COL = A.COL
NB = 13                   # log columns per finger
FINGER = RC.Finger        # replaced under --bodies-clear


def vec(v):
    return " ".join(f"{x:.9g}" for x in np.asarray(v, float))


# ---------- the return route, as the reach check judged it ----------
def climb_route(clear, a):
    """As RC.climb, keeping how each input pair was reached."""
    seen, stack = {a: None}, [a]
    n1, n2 = clear.shape
    while stack:
        i, j = stack.pop()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            k = (i + di, j + dj)
            if 0 <= k[0] < n1 and 0 <= k[1] < n2 and k not in seen and clear[k] >= clear[i, j]:
                seen[k] = (i, j)
                stack.append(k)
    return seen


def back(pred, k):
    out = [k]
    while pred[out[-1]] is not None:
        out.append(pred[out[-1]])
    return out[::-1]


def lift_and_land_route(clear, a, b):
    """The route of RC.lift_and_land: list of input pairs (table indices) from a to b, and its clearance."""
    n1, n2 = clear.shape
    every = {k: None for k in zip(*np.where(np.isfinite(clear)))}
    Aset = climb_route(clear, a) if a is not None else every
    Bset = climb_route(clear, b) if b is not None else every
    best = np.full(clear.shape, -np.inf)
    pred = {}
    h = []
    for k in Aset:
        k = (int(k[0]), int(k[1]))
        best[k] = clear[k]
        pred[k] = None
        h.append((-best[k], k))
    heapq.heapify(h)
    while h:
        w, (i, j) = heapq.heappop(h)
        w = -w
        if w < best[i, j]:
            continue
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            k = (i + di, j + dj)
            if 0 <= k[0] < n1 and 0 <= k[1] < n2:
                v = min(w, clear[k])
                if v > best[k]:
                    best[k] = v
                    pred[k] = (i, j)
                    heapq.heappush(h, (-v, k))
    e = max(((int(k[0]), int(k[1])) for k in Bset), key=lambda k: best[k])
    mid = back(pred, e)
    up = back({(int(k[0]), int(k[1])): v for k, v in Aset.items()}, mid[0]) if a is not None else [mid[0]]
    down = back({(int(k[0]), int(k[1])): v for k, v in Bset.items()}, e)[::-1] if b is not None else [e]
    return up[:-1] + mid + down[1:], float(best[e])


# ---------- one finger: where it stands, its inputs in time, its drawn body ----------
class Plan:
    def __init__(self, L, idx, j, tab, R, half, res, junction):
        self.name = L["names"][j]
        self.x = np.array(res["x"])
        self.F = F = FINGER(L, idx, j, tab, R, half)
        self.tab, self.junction = tab, junction
        self.Rm = RC.rodrigues(self.x[3:6])
        self.W = F.world(self.x)
        n1, n2, _ = self.W.shape
        a1, a2 = tab["p1"], tab["p2"]
        mask = None
        r1, r2 = res["input_proximal_range"], res["input_distal_range"]
        self.held = res.get("held")
        if self.held:
            mask = np.zeros((n1, n2), bool)
            if r1[0] == r1[1]:
                mask[int(np.argmin(np.abs(a1 - r1[0]))), :] = True
            else:
                mask[:, int(np.argmin(np.abs(a2 - r2[0])))] = True
        on, d, arg, off, cl = F.parts(self.x, mask)
        assert np.allclose(F.t[on], res["t_on"]) and np.allclose(a1[arg // n2], res["p1_on"]) \
            and np.allclose(a2[arg % n2], res["p2_on"]), "the saved reach check does not reproduce"
        pair = {int(s): (int(a) // n2, int(a) % n2) for s, a in zip(on, arg)}
        row = {int(s): k for k, s in enumerate(off)}
        kt, kp, self.routes = [], [], []
        for touching, s, e in F.spans:
            if touching:
                for k in range(s, e + 1):
                    if k in pair:
                        kt.append(F.t[k])
                        kp.append(pair[k])
                continue
            node = cl[[row[k] for k in range(s, e + 1)]].min(0)
            a, b = pair.get(s - 1), pair.get(e + 1)
            route, clear = lift_and_land_route(node, a, b)
            P = np.array([self.W[k] for k in route])
            arc = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
            t0 = F.t[s - 1] if a is not None else F.t[s]
            t1 = F.t[e + 1] if b is not None else F.t[e]
            tt = t0 + (t1 - t0) * (arc / arc[-1] if arc[-1] > 0 else np.linspace(0, 1, len(route)))
            i0, i1 = (1 if a is not None else 0), (len(route) - 1 if b is not None else len(route))
            kt += list(tt[i0:i1])
            kp += route[i0:i1]
            self.routes.append({"from_s": float(t0), "to_s": float(t1), "pairs": len(route),
                                "tip_path_mm": float(arc[-1] * 1e3), "clearance_mm": clear * 1e3})
        o = np.argsort(kt, kind="stable")
        self.kt = np.array(kt)[o]
        self.k1 = a1[np.array([p[0] for p in kp])[o]]
        self.k2 = a2[np.array([p[1] for p in kp])[o]]
        ton = np.array(res["t_on"])
        step = np.diff(ton) < 1.5 * (F.t[1] - F.t[0])
        self.jump = (float(np.abs(np.diff(res["p1_on"]))[step].max(initial=0)),
                     float(np.abs(np.diff(res["p2_on"]))[step].max(initial=0)))
        dtk = np.maximum(np.diff(self.kt), 1e-9)
        self.rate = (float(np.abs(np.diff(self.k1) / dtk).max()), float(np.abs(np.diff(self.k2) / dtk).max()))

    def inputs(self, t):
        return float(np.interp(t, self.kt, self.k1)), float(np.interp(t, self.kt, self.k2))

    def tip(self, p1, p2):
        a1, a2, W = self.tab["p1"], self.tab["p2"], self.W
        u = np.clip((p1 - a1[0]) / (a1[1] - a1[0]), 0, len(a1) - 1 - 1e-9)
        v = np.clip((p2 - a2[0]) / (a2[1] - a2[0]), 0, len(a2) - 1 - 1e-9)
        i, j = int(u), int(v)
        u, v = u - i, v - j
        return (W[i, j] * (1 - u) * (1 - v) + W[i + 1, j] * u * (1 - v) + W[i, j + 1] * (1 - u) * v + W[i + 1, j + 1] * u * v)

    def target(self, t):
        return self.tip(*self.inputs(t))

    def body(self, p1, tip):
        """Clamped end, measured junction, tip: the drawn finger."""
        jn = self.junction["rest"] + np.array([np.interp(p1, self.junction["p1"], self.junction["d"][:, i]) for i in range(3)])
        return np.array([self.x[:3], self.Rm @ jn + self.x[:3], tip])


class Reference:
    """Targets at the logged contacts themselves; no finger behind them."""
    def __init__(self, L, idx, j, tab, R, half):
        self.name = L["names"][j]
        F = self.F = RC.Finger(L, idx, j, tab, R, half)
        self.held, self.routes, self.jump, self.rate = None, [], (0.0, 0.0), (0.0, 0.0)
        kt, kx = [], []
        for touching, s, e in F.spans:
            if touching:
                for k in range(s, e + 1):
                    if F.on[k]:
                        kt.append(F.t[k])
                        kx.append(F.target[k])
                continue
            a = F.target[s - 1] if s > 0 else None
            b = F.target[e + 1] if e + 1 < len(F.t) else None
            a, b = (b if a is None else a), (a if b is None else b)
            for k in range(s, e + 1):
                w = (k - s + 1) / (e - s + 2)
                p = a * (1 - w) + b * w
                r = p - F.c[k]
                r -= (r @ F.u[k]) * F.u[k]
                lift = REF_LIFT_MM * 1e-3 * np.sin(np.pi * w) if (s > 0 and e + 1 < len(F.t)) else REF_LIFT_MM * 1e-3
                kt.append(F.t[k])
                kx.append(p + lift * r / np.linalg.norm(r))
        self.kt, self.kx = np.array(kt), np.array(kx)

    def inputs(self, t):
        return float("nan"), float("nan")

    def target(self, t):
        return np.array([np.interp(t, self.kt, self.kx[:, i]) for i in range(3)])


def junction_table(gm):
    d = os.path.join(gm, "experiments", "pneunet_tandem", "knuckle_analysis_20nov")
    s = json.load(open(os.path.join(d, "summary.json")))
    rows = list(csv.DictReader(open(os.path.join(d, "knuckle_levels_from_rest.csv"), newline="")))
    p1 = np.array([0.0] + [float(r["p1"]) for r in rows])
    dd = np.array([[0, 0, 0.0]] + [[float(r[f"junction_d{c}_mm"]) for c in "xyz"] for r in rows]) * 1e-3
    rest = np.array([s["knuckle_from_rest"]["pivot_x_mm"] + s["lever_split"]["along_finger_mm"],
                     s["lever_split"]["junction_centroid_rest_y_mm"], 0.0]) * 1e-3
    return {"p1": p1, "d": dd, "rest": rest}


# ---------- finger bodies against the cylinder and each other ----------
def body_points(B3, depth):
    """Points along the drawn finger more than one depth back from the tip (path length)."""
    seg = np.linalg.norm(np.diff(B3, axis=0), axis=1)
    keep = max(seg.sum() - depth, 0.0)
    s = np.linspace(0, keep, 12)
    cum = np.concatenate([[0], np.cumsum(seg)])
    return np.array([np.array([np.interp(v, cum, B3[:, i]) for i in range(3)]) for v in s])


def body_check(plans, times, centres, axes, tips, R, half, depth):
    """Deepest cut of each finger's drawn pieces into the cylinder (mm, positive = cutting in), the share of
    times it cuts in, and the smallest gap between different fingers' pieces (mm)."""
    cut = np.zeros((len(plans), len(times)))
    gap = np.inf
    for n, t in enumerate(times):
        pts = []
        for k, P in enumerate(plans):
            p1, _ = P.inputs(t)
            pts.append(body_points(P.body(p1, tips[k][n]), depth))
            cut[k, n] = -(RC.cyl_sdf(pts[-1], centres[n], axes[n], R, half) - 0.5 * depth).min()
        for a in range(len(plans)):
            for b in range(a + 1, len(plans)):
                gap = min(gap, np.linalg.norm(pts[a][:, None] - pts[b][None], axis=-1).min() - depth)
    return {"deepest_cut_mm": (cut.max(1) * 1e3).tolist(), "share_of_time_cutting_in": (cut > 0).mean(1).tolist(),
            "smallest_gap_between_fingers_mm": float(gap * 1e3)}


# ---------- the model ----------
def build(L, s0, R, half, homes, k_tip, support, key, base_low):
    mu = L["mu"]
    q0 = L["q"][s0]
    Id = np.diag(L["I"]) if L["I"].ndim == 2 else np.ravel(L["I"])[:3]
    gq = f'quat="{q0[3]:.9g} {q0[0]:.9g} {q0[1]:.9g} {q0[2]:.9g}"'
    ax = ("1 0 0", "0 1 0", "0 0 1")[:3 if support == "free" else 2]
    obj = (f'<body name="obj" pos="{vec(L["op"][s0])}">'
           + "".join(f'<joint name="obj_s{i}" type="slide" axis="{a}" armature="0"/>' for i, a in enumerate(ax))
           + '<joint name="obj_ball" type="ball" armature="0"/>'
           f'<inertial pos="{vec(A.rot(q0, L["com"]))}" mass="{L["mass"]:.9g}" diaginertia="{vec(Id)}"/>'
           f'<geom name="obj" type="cylinder" size="{R:.9g} {half:.9g}" {gq} rgba="0.95 0.75 0.3 0.55"/></body>')
    tips = []
    for k, (name, h) in enumerate(homes):
        c = COL[name]
        tips.append(f'<body name="tip{k}" pos="{vec(h)}" gravcomp="1">'
                    + "".join(f'<joint name="tip{k}_{a}" type="slide" axis="{v}" stiffness="{k_tip:.9g}" '
                              f'damping="{2 * np.sqrt(k_tip * TIP_MASS):.9g}"/>'
                              for a, v in zip("xyz", ("1 0 0", "0 1 0", "0 0 1")))
                    + f'<geom name="tip{k}" type="sphere" size="{RADIUS:.9g}" mass="{TIP_MASS}" '
                    f'rgba="{c[0]} {c[1]} {c[2]} 1"/></body>')
    plate = (f'<geom type="cylinder" size="0.075 0.002" pos="{vec([L["op"][s0][0], L["op"][s0][1], base_low - 0.002])}" '
             'rgba="0.55 0.57 0.62 1" contype="0" conaffinity="0"/>')
    xml = "\n".join([
        f'<mujoco model="two_pneunet_fingers_{key}">',
        "<!-- generated by tools/two_pneunet_fingers_follow_rl_gait_clip_21nov.py. SIMULATION ONLY."
        " Each finger is a tip ball on a spring; the springs' rest positions are moved by that script."
        " The plate is drawn only. -->",
        f'<option gravity="0 0 -9.81" timestep="{DT}" integrator="implicitfast" cone="elliptic" impratio="{A.IMPRATIO}"/>',
        f'<visual><global offwidth="{CW}" offheight="{CH}"/><headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6"/></visual>',
        f'<default><geom friction="{mu} 0.005 0.0001" solref="{A.G.CONTACT_TIME} 1"/></default>',
        '<asset><texture type="skybox" builtin="flat" rgb1="1 1 1" rgb2="1 1 1" width="32" height="32"/></asset>',
        "<worldbody>", plate, obj, *tips, "</worldbody>",
        '<equality><weld name="hold" body1="obj"/></equality>', "</mujoco>", ""])
    path = os.path.join(OUT, f"model_{key}.xml")
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(xml)
    m = mj.MjModel.from_xml_path(path)
    B = {"m": m, "support": support, "q0": q0.copy(), "p0": L["op"][s0].copy(), "nslide": len(ax),
         "sq": int(m.jnt_qposadr[m.joint("obj_s0").id]), "bq": int(m.jnt_qposadr[m.joint("obj_ball").id]),
         "obj_geom": m.geom("obj").id, "tip_geom": [m.geom(f"tip{k}").id for k in range(len(homes))],
         "tq": [int(m.jnt_qposadr[m.joint(f"tip{k}_x").id]) for k in range(len(homes))],
         "homes": [np.array(h) for _, h in homes], "path": path}
    return B


LOG_HEADER = ("t,part,log_t,turn_deg,tilt_from_start_deg,centre_from_start_mm,centre_drop_mm,ox,oy,oz,qx,qy,qz,qw,"
              + ",".join(f"f{k}_input_first,f{k}_input_second,f{k}_touching,f{k}_normal_N,f{k}_tangential_N,"
                         f"f{k}_cx,f{k}_cy,f{k}_cz,f{k}_target_x,f{k}_target_y,f{k}_target_z,f{k}_tip_x,f{k}_tip_y,f{k}_tip_z"
                         for k in range(4)))
NB = 14
C0 = 14


def run(B, plans, t0, t1, c0, u0):
    m = B["m"]
    d = mj.MjData(m)
    T_END = T_GAIT + (t1 - t0) * SLOW
    first = [P.target(t0) for P in plans]
    out = []
    for p in first:
        r = p - c0
        r -= (r @ u0) * u0
        out.append(r / np.linalg.norm(r))
    for k, (p, o) in enumerate(zip(first, out)):
        d.qpos[B["tq"][k]:B["tq"][k] + 3] = p + RETRACT * o - B["homes"][k]
    every, fevery = int(round(0.01 / DT)), int(round(SLOW / CLIP_SLOW / FPS / DT))
    log, frames = [], []
    B2 = dict(B, support="x")
    p_start, q_start = A.object_pose(B2, d)
    quat_prev = q_start.copy()
    ax0 = A.rot(q_start, np.array([0, 0, 1.0]))
    turn, stopped, tiptip = 0.0, None, 0
    f6 = np.zeros(6)
    for s in range(int(round((T_END + T_AFTER) / DT)) + 1):
        t = s * DT
        part = 0 if t < T_LET_GO else 1 if t < T_GAIT else 2 if t < T_END else 3
        a = float(np.clip((t - T_IN0) / (T_IN1 - T_IN0), 0, 1))
        lt = t0 + float(np.clip((t - T_GAIT) / SLOW, 0, t1 - t0))
        tg = [P.target(lt) + (1 - a) * RETRACT * o for P, o in zip(plans, out)]
        for k, p in enumerate(tg):
            m.qpos_spring[B["tq"][k]:B["tq"][k] + 3] = p - B["homes"][k]
        d.eq_active[0] = part == 0
        mj.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)):
            stopped = ("non-finite state", t)
            break
        p, q = A.object_pose(B2, d)
        dq = A.qmul(A.inv(quat_prev), q)
        turn += 2 * dq[2] * np.sign(dq[3])
        quat_prev = q
        if s % every == 0 or s % fevery == 0:
            row = []
            for k, P in enumerate(plans):
                Fn, Ft, Pc = 0.0, 0.0, np.zeros(3)
                for ci in range(d.ncon):
                    c = d.contact[ci]
                    pair = {int(c.geom1), int(c.geom2)}
                    if B["tip_geom"][k] not in pair:
                        continue
                    if B["obj_geom"] not in pair:
                        tiptip += 1
                        continue
                    mj.mj_contactForce(m, d, ci, f6)
                    Fn, Ft, Pc = Fn + f6[0], Ft + float(np.hypot(f6[1], f6[2])), Pc + f6[0] * c.pos
                tip = B["homes"][k] + d.qpos[B["tq"][k]:B["tq"][k] + 3]
                row += [*P.inputs(lt), float(Fn > A.TOUCH_N), Fn, Ft, *(Pc / Fn if Fn > 0 else Pc), *tg[k], *tip]
            ax = A.rot(q, np.array([0, 0, 1.0]))
            rec = [t, part, lt, np.degrees(turn), np.degrees(np.arccos(np.clip(ax0 @ ax, -1, 1))),
                   1e3 * np.linalg.norm(p - p_start), 1e3 * (p_start[2] - p[2]), *p, *q, *row]
            if s % every == 0:
                log.append(rec)
            if s % fevery == 0 and t >= T_GAIT - 1e-9:
                frames.append((rec, d.qpos.copy()))
        if B["support"] == "free" and part > 0 and p_start[2] - p[2] > A.DROP_STOP:
            stopped = (f"the cylinder had dropped {A.DROP_STOP * 1e3:.0f} mm", t)
            break
    return {"log": np.array(log), "frames": frames, "stopped": stopped, "tip_tip_contacts_logged": tiptip}


def lifts(touch):
    """Whole stretches off between two stretches on."""
    e = np.diff(touch.astype(int))
    offs, ons = np.where(e == -1)[0], np.where(e == 1)[0]
    return int(sum(1 for o in offs if (ons > o).any()))


def measures(L, plans, r, t0, t1, t_round, R, half, depth, bodies):
    lg = r["log"]
    g = lg[lg[:, 1] == 2]
    out = {"stopped": r["stopped"], "tip_tip_contacts_logged": r["tip_tip_contacts_logged"]}
    if not len(g):
        return out
    wait = lg[lg[:, 1] == 1]
    out["turn_in_wait_deg"] = float(wait[-1, 3] - wait[0, 3]) if len(wait) else float("nan")
    out["turn_gait_deg"] = float(g[-1, 3] - g[0, 3])
    out["gait_completed_to_log_s"] = float(g[-1, 2])
    i = int(np.argmin(np.abs(g[:, 2] - t_round)))
    out["turn_first_round_deg"] = float(g[i, 3] - g[0, 3])
    out["tilt_change_max_deg"] = float(g[:, 4].max())
    out["centre_moved_max_mm"] = float(g[:, 5].max())
    out["centre_drop_max_mm"] = float(g[:, 6].max())
    out["fingers"] = {}
    for k, P in enumerate(plans):
        c = C0 + NB * k
        F = P.F
        log_touch = np.interp(g[:, 2], F.t, F.touch.astype(float)) > 0.5
        sim = g[:, c + 2] > 0.5
        lam = L["lam"][F.idx, F.j]
        out["fingers"][P.name] = {
            "share_of_gait_agreeing_with_log": float((sim == log_touch).mean()),
            "share_touching": float(sim.mean()), "log_share_touching": float(log_touch.mean()),
            "whole_lifts": lifts(sim), "log_whole_lifts": lifts(log_touch),
            "normal_force_mean_N": float(g[sim, c + 3].mean()) if sim.any() else 0.0,
            "tangential_force_mean_N": float(g[sim, c + 4].mean()) if sim.any() else 0.0,
            "log_normal_force_mean_N": float(lam[F.touch & (lam > A.TOUCH_N)].mean()),
            "tip_from_target_max_mm": float(1e3 * np.linalg.norm(g[:, c + 8:c + 11] - g[:, c + 11:c + 14], axis=1).max())}
    if bodies:
        n = g[::10]
        axes = np.array([A.rot(q, np.array([0, 0, 1.0])) for q in n[:, 10:14]])
        out["bodies"] = body_check(plans, n[:, 2], n[:, 7:10], axes,
                                   [n[:, C0 + NB * k + 11:C0 + NB * k + 14] for k in range(len(plans))], R, half, depth)
    return out


# ---------- the check that a tip can drag something ----------
def drag_check(mu, k_tip, mass):
    press, travel = 0.003, 0.020
    xml = (f'<mujoco><option gravity="0 0 -9.81" timestep="{DT}" integrator="implicitfast" cone="elliptic" '
           f'impratio="{A.IMPRATIO}"/><default><geom friction="{mu} 0.005 0.0001" solref="{A.G.CONTACT_TIME} 1"/></default>'
           f'<worldbody><body name="block"><joint name="bx" type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/>'
           f'<geom name="block" type="box" size="0.05 0.05 0.01" mass="{mass}"/></body>'
           f'<body name="tip" pos="0 0 {0.01 + RADIUS + 0.002}" gravcomp="1">'
           + "".join(f'<joint name="t{a}" type="slide" axis="{v}" stiffness="{k_tip}" damping="{2 * np.sqrt(k_tip * TIP_MASS)}"/>'
                     for a, v in zip("xyz", ("1 0 0", "0 1 0", "0 0 1")))
           + f'<geom type="sphere" size="{RADIUS}" mass="{TIP_MASS}"/></body></worldbody></mujoco>')
    m = mj.MjModel.from_xml_string(xml)
    d = mj.MjData(m)
    for s in range(int(3.0 / DT)):
        t = s * DT
        m.qpos_spring[2:5] = [travel * np.clip((t - 1) / 1.5, 0, 1), 0, -(0.002 + press) * np.clip(t / 0.5, 0, 1)]
        mj.mj_step(m, d)
    return float(d.qpos[0] / travel), press * k_tip


# ---------- the clip ----------
def film(L, idx, plans, B, r, S, R, half, base, title3, note3, azimuth, elevation, base_low, bodies):
    from PIL import Image, ImageDraw, ImageFont
    try:
        font = ImageFont.truetype("arial.ttf", 24)
        small = ImageFont.truetype("arial.ttf", 19)
    except OSError:
        font = small = ImageFont.load_default()
    s0, s1 = int(idx[0]), int(idx[-1])
    t0 = L["t"][s0]
    Wt, Ht = 3 * CW, HEAD + CH + FOOT
    ff = shutil.which("ffmpeg") or r"C:\Program Files\ffmpeg\bin\ffmpeg.exe"
    proc = subprocess.Popen(
        [ff, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{Wt}x{Ht}",
         "-r", str(FPS), "-i", "-", "-pix_fmt", "yuv420p", "-vcodec", "libx264", base + ".mp4"], stdin=subprocess.PIPE)
    files = [base + ".mp4"]
    cam = mj.MjvCamera()
    cam.lookat[:] = L["op"][s0:s1 + 1].mean(0) + [0, 0, 0.45 * (base_low - L["op"][s0][2])]
    cam.distance, cam.azimuth, cam.elevation = 0.42, azimuth, elevation
    mC = U.contacts_scene(L, {"s": s0}, R, half)
    models = [S["m"], mC, B["m"]]
    S["m"].vis.global_.offwidth, S["m"].vis.global_.offheight = CW, CH
    datas = [mj.MjData(m) for m in models]
    rens = [mj.Renderer(m, CH, CW) for m in models]
    Z = L["Z"]
    titles = ["Recorded: learned policy, four-finger hand", "Extracted: contacts as they make and break", title3]
    foot = [f"All three play together. Recording shown {CLIP_SLOW:g} times slower; the simulated gait runs {SLOW:g} times "
            f"slower than the recording. Log {t0:.2f} to {L['t'][s1]:.2f} s.",
            "Small dots: logged contact paths. Large dots: touching now. Arrows: logged normal force. Black line: fixed to the cylinder.",
            note3 + " SIMULATION ONLY. Comparative, not predictive."]

    def shot(i, extra):
        ren, d = rens[i], datas[i]
        ren.update_scene(d, cam)
        extra(ren.scene)
        rgb = ren.render().copy()
        ren.enable_depth_rendering()
        far = ren.render() > 5.0
        ren.disable_depth_rendering()
        rgb[far] = 255
        return rgb

    def turn_line(scene, p, q):
        U.connector(scene, mj.mjtGeom.mjGEOM_CAPSULE, 0.0015, p + A.rot(q, np.array([0, 0, half + 0.001])),
                    p + A.rot(q, np.array([R, 0, half + 0.001])), (0.1, 0.1, 0.1, 1))

    def path_dots(scene, k):
        for j in range(4):
            for kk in range(max(s0, k - 90), k + 1, 3):
                if L["lam"][kk, j] > A.TOUCH_N:
                    A.add_sphere(scene, L["P"][kk, j], 0.0016, (*COL[L["names"][j]], 1))

    def frame(row, qpos, still=None, repeat=1):
        k = int(np.clip(np.searchsorted(L["t"], row[2]), s0, s1))
        pose = (L["op"][k], L["q"][k])
        now = {j: L["P"][k, j] for j in range(4) if L["lam"][k, j] > A.TOUCH_N}
        A.pose_recorded(S, datas[0], Z, k)

        def ex0(scene):
            turn_line(scene, Z["obj"][k][:3].astype(float), Z["obj"][k][3:7].astype(float))
            for j, p in now.items():
                A.add_sphere(scene, p, 0.0035, (*COL[L["names"][j]], 1))
        im0 = shot(0, ex0)
        datas[1].mocap_pos[0] = pose[0]
        datas[1].mocap_quat[0] = pose[1][[3, 0, 1, 2]]
        mj.mj_forward(mC, datas[1])

        def ex1(scene):
            turn_line(scene, *pose)
            path_dots(scene, k)
            for j, p in now.items():
                c = (*COL[L["names"][j]], 1)
                A.add_sphere(scene, p, 0.0035, c)
                U.connector(scene, mj.mjtGeom.mjGEOM_ARROW, 0.004, p - 0.03 * L["lam"][k, j] * L["nrm"][k, j], p, c)
        im1 = shot(1, ex1)
        d = datas[2]
        d.qpos[:] = qpos
        mj.mj_forward(B["m"], d)

        def ex2(scene):
            turn_line(scene, np.array(row[7:10]), np.array(row[10:14]))
            for kk, P in enumerate(plans):
                c = C0 + NB * kk
                if bodies:
                    b3 = P.body(row[c], np.array(row[c + 11:c + 14]))
                    for a, b in zip(b3[:-1], b3[1:]):
                        U.connector(scene, mj.mjtGeom.mjGEOM_CAPSULE, 0.5 * P.tab["depth"], a, b, (0.80, 0.82, 0.86, 1))
                    A.add_sphere(scene, b3[0], 0.6 * P.tab["depth"], (0.45, 0.47, 0.52, 1))
                if row[c + 2] > 0:
                    A.add_sphere(scene, row[c + 5:c + 8], 0.0035, (0, 0, 0, 1))
        im2 = shot(2, ex2)
        im = Image.new("RGB", (Wt, Ht), (255, 255, 255))
        im.paste(Image.fromarray(np.hstack([im0, im1, im2])), (0, HEAD))
        dr = ImageDraw.Draw(im)
        tw_rec = np.degrees(L["turn"][k] - L["turn"][s0])
        tw = row[3] - turn0
        tch = " ".join(L["names"][j] for j in now)
        sim_t = " ".join(P.name for kk, P in enumerate(plans) if row[C0 + NB * kk + 2] > 0)
        seconds = [f"log time {L['t'][k]:.2f} s   cylinder turned {tw_rec:+.1f} deg",
                   f"touching: {tch}",
                   f"cylinder turned {tw:+.1f} deg   touching: {sim_t}"]
        for i in range(3):
            dr.text((i * CW + 16, 12), titles[i], fill=(0, 0, 0), font=font)
            dr.text((i * CW + 16, 48), seconds[i], fill=(0, 0, 0), font=small)
            if i:
                dr.line([(i * CW, HEAD), (i * CW, HEAD + CH)], fill=(200, 200, 200), width=1)
        for i in range(3):
            dr.text((16, HEAD + CH + 6 + 26 * i), foot[i], fill=(0, 0, 0), font=small)
        buf = np.asarray(im).tobytes()
        for _ in range(repeat):
            proc.stdin.write(buf)
        if still:
            im.save(still)
            files.append(still)

    fl = [f for f in r["frames"] if f[0][1] >= 2]
    gait = [f for f in fl if f[0][1] == 2]
    turn0 = gait[0][0][3]
    for n, (row, qpos) in enumerate(fl):
        still = (base + "_start.png" if n == 0 else base + "_middle.png" if n == len(gait) // 2
                 else base + "_end.png" if n == len(gait) - 1 else None)
        frame(row, qpos, still=still, repeat=FPS if n in (0, len(fl) - 1) else 1)
    for ren in rens:
        ren.close()
    proc.stdin.close()
    proc.wait()
    return files


RADIUS = None


def main():
    global RADIUS, OUT, REACH, FINGER
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", required=True)
    ap.add_argument("--gramian-map", required=True)
    ap.add_argument("--no-film", action="store_true")
    ap.add_argument("--bodies-clear", action="store_true",
                    help="use the placements refitted with the drawn finger bodies kept clear; writes a separate folder")
    ap.add_argument("--azimuth", type=float, default=35.0)
    ap.add_argument("--elevation", type=float, default=-25.0)
    args = ap.parse_args()
    for sub in ("a3h_sandbox", os.path.join("experiment_log", "scripts")):
        sys.path.insert(0, os.path.join(args.wave_manip, sub))
    from w16s1_hora_scout import CYL_URDF_HALFLEN_M, CYL_URDF_RADIUS_M
    tab = RC.tip_table(args.gramian_map)
    if args.bodies_clear:
        import two_pneunet_fingers_follow_rl_gait_reach_check_bodies_clear_21nov as BC
        OUT = os.path.join(ROOT, "outputs", "two_pneunet_fingers_follow_rl_gait_clip_bodies_clear_21nov")
        REACH = os.path.join(BC.OUT, "reach_check_results.json")
        FINGER = BC.Finger
        tab = BC.add_body(tab, args.gramian_map)
    os.makedirs(OUT, exist_ok=True)
    L = A.load_recording()
    R, half = CYL_URDF_RADIUS_M * L["scale"], CYL_URDF_HALFLEN_M * L["scale"]
    RADIUS = 0.5 * tab["depth"]
    junction = junction_table(args.gramian_map)
    reach = json.load(open(REACH))
    t = L["t"]
    s0, s1 = (int(np.argmin(np.abs(t - v))) for v in reach["round"])
    idx = np.arange(s0, s1 + 1, RC.STEP)
    t0, t1 = float(t[idx[0]]), float(t[idx[-1]])
    up = json.load(open(UPRIGHT))
    ks = (float(np.mean([f["k_along_surface_N_per_m"] for f in up["fits"]])),
          float(np.mean([f["k_out_of_plane_N_per_m"] for f in up["fits"]])))
    stiff = {"soft": ks[0], "main": float(np.sqrt(ks[0] * ks[1])), "stiff": ks[1]}
    # the end of the first round of the gait, as the reach check found it
    touch = L["lam"] > A.TOUCH_N
    key = (touch * [1, 2, 4, 8]).sum(1)
    i, n = s0, 0
    while i < len(t):
        s = i
        while i < len(t) and key[i] == key[s]:
            i += 1
        n += 1
        if n > 1 and key[s] == key[s0] and t[i - 1] - t[s] >= RC.MIN_STROKE_S:
            break
    t_round = float(t[s])
    sets = {"two_inputs": [Plan(L, idx, j, tab, R, half, reach["two_inputs"][j], junction) for j in range(4)],
            "one_input": [Plan(L, idx, j, tab, R, half, reach["one_input"][j], junction) for j in range(4)],
            "logged_path": [Reference(L, idx, j, tab, R, half) for j in range(4)]}
    what = {"two_inputs": "four two-pneu-net fingers, both inputs driven",
            "one_input": "the same fingers with one input held constant (control)",
            "logged_path": "REFERENCE, not a finger: tip targets at the logged contacts themselves"}
    rec_turn = float(np.degrees(L["turn"][idx[-1]] - L["turn"][idx[0]]))
    rec_round = float(np.degrees(L["turn"][int(np.argmin(np.abs(t - t_round)))] - L["turn"][idx[0]]))
    rep = ["Four two-pneu-net fingers follow a recorded learned gait, lifting and landing: simulation with contact",
           "SIMULATION ONLY. Comparative, not predictive.", "",
           "Where the fingers stand and their inputs: " + os.path.relpath(REACH, ROOT).replace(os.sep, "/")
           + (" (the refit with the drawn finger bodies kept out of the logged cylinder)." if args.bodies_clear else "."),
           f"Recording: cylinder radius {R * 1e3:.1f} mm, half length {half * 1e3:.1f} mm, mass {L['mass'] * 1e3:.1f} g, "
           f"friction {L['mu']:.2f}. Window: log {t0:.2f} to {t1:.2f} s; the cylinder turns {rec_turn:+.1f} deg in it, "
           f"{rec_round:+.1f} deg in the first round of the gait (to {t_round:.2f} s).",
           f"Finger stand-in: a tip ball of radius {RADIUS * 1e3:.1f} mm and {TIP_MASS * 1e3:g} g (ASSUMED) on a spring to the "
           "measured free tip position for the commanded inputs. Only the tip ball touches. See the script's header.",
           "Tip stiffness (ASSUMED, same in all directions; from the one-pneu-net stand-in of the 21 Nov upright arrangement): "
           + ", ".join(f"{k} {v:.1f} N/m" for k, v in stiff.items()) + ". The two-pneu-net finger's own has not been measured.",
           f"Timeline (ASSUMED): tips brought in {T_IN0:g} to {T_IN1:g} s with the cylinder held; let go at {T_LET_GO:g} s; "
           f"gait from {T_GAIT:g} s, run {SLOW:g} times slower than the recording ({(t1 - t0) * SLOW:.1f} s); then {T_AFTER:g} s held.",
           f"Friction tip to cylinder {L['mu']:.2f} (ASSUMED: the log's value). Elliptic cone, impratio {A.IMPRATIO}, time step {DT} s.", ""]
    dc = {k: drag_check(L["mu"], v, L["mass"]) for k, v in stiff.items()}
    rep += ["Check that a tip on a moving spring drags what it presses on (a block of the cylinder's mass, free to slide; "
            "the spring's rest position moves 20 mm sideways while pressing 3 mm in):"]
    rep += [f"  {k}: block follows {100 * v[0]:.0f}% of the 20 mm (press force {v[1]:.2f} N)" for k, v in dc.items()]
    rep += ["", "Inputs: largest change between consecutive touching samples (log step "
            f"{(t[idx[1]] - t[idx[0]]) * 1e3:.0f} ms; full range is 120), and largest rate anywhere including lifts "
            "(per second of log time; the simulation is run slower):"]
    for name in ("two_inputs", "one_input"):
        for P in sets[name]:
            rep.append(f"  {name.replace('_', ' ')}, {P.name:<6s}: first input {P.jump[0]:.0f}, second {P.jump[1]:.0f} per sample; "
                       f"rates {P.rate[0]:.0f} and {P.rate[1]:.0f} per s; returns: "
                       + "; ".join(f"{x['from_s']:.2f}-{x['to_s']:.2f} s through {x['pairs']} input pairs, tip path "
                                   f"{x['tip_path_mm']:.0f} mm, clearance {x['clearance_mm']:.1f} mm" for x in P.routes))
    results = {"window_log_s": [t0, t1], "first_round_ends_log_s": t_round, "recorded_turn_deg": rec_turn,
               "recorded_turn_first_round_deg": rec_round, "stiffness_N_per_m": stiff, "drag_check": dc, "runs": {}}
    # finger bodies on the logged motion, tips at their targets
    rep += ["", "Finger bodies, drawn pieces at the finger's half depth, more than one depth back from the tip, against the "
            "LOGGED cylinder motion with the tips at their targets (no simulation). Positive = cutting into the cylinder:"]
    for name in ("two_inputs", "one_input"):
        tt = t[idx]
        bc = body_check(sets[name], tt, L["op"][idx], L["axis"][idx],
                        [np.array([P.target(v) for v in tt]) for P in sets[name]], R, half, tab["depth"])
        results[f"bodies_on_logged_motion_{name}"] = bc
        rep.append(f"  {name.replace('_', ' ')}: deepest cut "
                   + ", ".join(f"{P.name} {c:+.1f} mm ({100 * sh:.0f}% of the time)" for P, c, sh in
                               zip(sets[name], bc["deepest_cut_mm"], bc["share_of_time_cutting_in"]))
                   + f"; smallest gap between different fingers' pieces {bc['smallest_gap_between_fingers_mm']:+.1f} mm "
                   "(negative = overlapping).")
    rep += ["  The junction's height in the finger's own frame is taken as zero (ASSUMED); the pieces are straight, the real "
            "finger curves.", ""]

    def save():
        with open(os.path.join(OUT, "gait_clip_report.txt"), "w", encoding="utf-8") as fh:
            fh.write("\n".join(rep) + "\n")
        with open(os.path.join(OUT, "gait_clip_results.json"), "w", encoding="utf-8") as fh:
            json.dump(results, fh, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    print("\n".join(rep), flush=True)
    c0, u0 = L["op"][s0], L["axis"][s0]
    base_low = min(P.x[2] for P in sets["two_inputs"]) - 0.006
    kept = {}
    cases = [(n, s, "weight_carried") for s in ("main", "soft", "stiff") for n in sets] + [(n, "main", "free") for n in sets]
    for name, sk, support in cases:
        plans = sets[name]
        homes = [(P.name, P.target(t0)) for P in plans]
        tag = f"{name}_{sk}_{support}"
        B = build(L, s0, R, half, homes, stiff[sk], support, tag, base_low)
        r = run(B, plans, t0, t1, c0, u0)
        ms = measures(L, plans, r, t0, t1, t_round, R, half, tab["depth"], name != "logged_path")
        results["runs"][tag] = ms
        np.savetxt(os.path.join(OUT, f"log_{tag}.csv"), r["log"], delimiter=",", fmt="%.6g", header=LOG_HEADER, comments="")
        n0 = len(rep)
        rep += [f"{what[name]}; tip stiffness {stiff[sk]:.1f} N/m; {'weight carried' if support != 'free' else 'cylinder free'}: "
                + (f"STOPPED, {r['stopped'][0]} at {r['stopped'][1]:.2f} s" if r["stopped"] else "ran to the end")]
        if "turn_gait_deg" in ms:
            rep.append(f"  cylinder turn in the gait {ms['turn_gait_deg']:+.1f} deg (recorded {rec_turn:+.1f}; "
                       f"{100 * ms['turn_gait_deg'] / rec_turn:.0f}% of it"
                       + (f"; reached only log {ms['gait_completed_to_log_s']:.2f} s" if r["stopped"] else "")
                       + f"); first round {ms['turn_first_round_deg']:+.1f} deg (recorded {rec_round:+.1f}); "
                       f"in the wait before {ms['turn_in_wait_deg']:+.2f} deg. Tilt changed by at most "
                       f"{ms['tilt_change_max_deg']:.1f} deg; centre moved at most {ms['centre_moved_max_mm']:.1f} mm, dropped at most "
                       f"{ms['centre_drop_max_mm']:.1f} mm. Tip-to-tip contacts logged: {ms['tip_tip_contacts_logged']}.")
            for n, v in ms["fingers"].items():
                rep.append(f"  {n:<6s}: touching or not agrees with the log {100 * v['share_of_gait_agreeing_with_log']:.0f}% of the "
                           f"gait (touching {100 * v['share_touching']:.0f}%, log {100 * v['log_share_touching']:.0f}%); whole lifts "
                           f"{v['whole_lifts']} (log {v['log_whole_lifts']}); normal force {v['normal_force_mean_N']:.3f} N "
                           f"(log {v['log_normal_force_mean_N']:.3f} N), along the surface {v['tangential_force_mean_N']:.3f} N; "
                           f"tip at most {v['tip_from_target_max_mm']:.1f} mm from its target.")
            if "bodies" in ms:
                bc = ms["bodies"]
                rep.append("  drawn finger pieces against the simulated cylinder: deepest cut "
                           + ", ".join(f"{P.name} {c:+.1f} mm ({100 * sh:.0f}%)" for P, c, sh in
                                       zip(plans, bc["deepest_cut_mm"], bc["share_of_time_cutting_in"]))
                           + f"; smallest gap between fingers {bc['smallest_gap_between_fingers_mm']:+.1f} mm.")
        rep.append("")
        print("\n".join(rep[n0:]), flush=True)
        save()
        if sk == "main" and support == "weight_carried":
            kept[name] = (B, r, plans)
    if not args.no_film:
        S = A.recorded_scene(args.wave_manip, L, R, half)
        n0 = len(rep)
        rep += [f"Clips: three columns of {CW} x {CH}, same camera (azimuth {args.azimuth:.0f} deg, elevation {args.elevation:.0f} deg). "
                f"Main stiffness, weight carried."]
        cuts = results["runs"]["two_inputs_main_weight_carried"].get("bodies", {}).get("deepest_cut_mm", [])
        body_note = ("no body measure for this run" if not cuts else
                     f"it cuts up to {max(cuts):.1f} mm into the cylinder (see report)" if max(cuts) > 0 else
                     "it stays clear of the cylinder in this run")
        clips = {"two_inputs": ("Simulation: four two-pneu-net fingers",
                                "Coloured balls: tips, the only parts that touch. Grey: finger drawn through its measured "
                                "junction and tip, not simulated; " + body_note + "."),
                 "one_input": ("Control: same fingers, one input held",
                               "One input per finger held constant. Coloured balls: tips (only they touch). Grey: finger drawn, "
                               "not simulated. Black dots: touching now."),
                 "logged_path": ("Reference: tips sent to the logged contacts",
                                 "Not a finger: tip balls on the same springs, sent to the logged contact places. "
                                 "Black dots: touching now.")}
        for name, (B, r, plans) in kept.items():
            if not any(f[0][1] == 2 for f in r["frames"]):
                rep.append(f"  no clip for {name}: the run stopped before the gait.")
                continue
            fs = film(L, idx, plans, B, r, S, R, half, os.path.join(OUT, f"recorded_contacts_simulation_{name}"),
                      *clips[name], args.azimuth, args.elevation, base_low, name != "logged_path")
            rep += [f"  wrote {f}" for f in fs]
        save()
        print("\n".join(rep[n0:]))


if __name__ == "__main__":
    main()
