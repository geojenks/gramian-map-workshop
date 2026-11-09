"""Two checks on a hand against a human demonstration, with no per-finger force targets.

The human take's per-finger forces are derived from the ball's motion, and they turn out to be the
ball's weight shared out, with no sign of the turn in them. So they make poor targets. Here:

1. Position check. follow.py itself, unchanged, on a copy of the leader with every per-finger force
   set to zero, at each hand's stored placement. Reach, facing and overlap decide the verdict.

2. Turning check, built for this film; not the banked method. At each pose from (1), for the fingers
   that touch: can they, inside friction and inside what their joints can supply, hold the net
   force the ball's motion needs and also push the ball round its measured turn? One linear
   programme per frame:
       forces in the friction cones, joint balance within each finger's input columns,
       net force = the frame's net force,  net torque = I alpha + s * axis,   maximise s.
   s is the spare torque about the turn, with the turn's own sign. Read again with the torque
   off the axis left free. Both use follow.py's strength columns, stops and cone generators, and
   the first is cross-checked against follow.py's own stacked system, solved with no tolerance.

   Also: the share of the ball's surface motion at the contact each finger's joints can follow at
   that pose, with joints at a limit moving only away from it. This is what sticking contact needs.

Frozen poses, each frame on its own. No hand is simulated doing the turn. Mass and friction are the
leader's assumed values. Forces on the object, in the placed frame.

    python tools/position_turning_check.py --leader <palm file> --derived <board file>
        --runs follow_*.json --out-dir <dir>

Needs scipy, mujoco and numpy together in one Python.
"""
import argparse, collections, copy, json, os, sys, time
import numpy as np
from scipy.optimize import linprog, lsq_linear

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop import follow as F  # noqa: E402
from gramian_workshop.hands import Hand  # noqa: E402
from gramian_workshop.vendor import stability_core as stability  # noqa: E402

CONE_EDGES = 8     # as the derivation of the file's forces (tools/import_demo_leader.py)
TURN_FIT_S = 0.1   # half-width for the turn's rate, as the derivation of the file's forces
TIGHT_TOL = 1e-4   # for the cross-check. follow.py's own 5% of the wrench lets a big wrench through well above
                   # the top, because the same tolerance also covers the input-range rows; the verdicts here do not use it


def kabsch(a, b):
    ac, bc = a.mean(0), b.mean(0)
    U, _, Vt = np.linalg.svd((a - ac).T @ (b - bc))
    return Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T


def qmat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def rotvec(R):
    a = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if a < 1e-9:
        return np.zeros(3)
    return a / (2 * np.sin(a)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


class Motion:
    """The ball's turn in the palm frame, from the board file's tracked rotation and the two files' shared finger points."""
    def __init__(s, L, B):
        s.F = L["frames"]; s.t = np.array([f["t"] for f in s.F]); s.n = len(s.F)
        by = {round(b["t"], 5): b for b in B["frames"]}
        s.R = []
        for f in s.F:
            b = by[round(f["t"], 5)]
            ok = [i for i in range(L["n_stations"]) if f["station_pos"][i] is not None and b["station_pos"][i] is not None]
            Rp = kabsch(np.array([b["station_pos"][i] for i in ok] + [b["obj_pos"]]), np.array([f["station_pos"][i] for i in ok] + [f["obj_pos"]]))
            s.R.append(Rp @ qmat(b["obj_quat"]))
        s.R = np.array(s.R)

    def _span(s, tk):
        a = int(np.searchsorted(s.t, tk - TURN_FIT_S)); b = min(s.n - 1, int(np.searchsorted(s.t, tk + TURN_FIT_S)))
        return a, b

    def omega_at(s, tk):
        a, b = s._span(tk)
        return rotvec(s.R[b] @ s.R[a].T) / max(s.t[b] - s.t[a], 1e-3)

    def omega(s, k):
        return s.omega_at(s.t[k])

    def alpha(s, k):
        a, b = s._span(s.t[k])
        return (s.omega_at(s.t[b]) - s.omega_at(s.t[a])) / max(s.t[b] - s.t[a], 1e-3)

    def centre_velocity(s, k):
        a, b = s._span(s.t[k])
        return (np.array(s.F[b]["obj_pos"]) - s.F[a]["obj_pos"]) / max(s.t[b] - s.t[a], 1e-3)


def position_leader(L):
    """A copy with every per-finger force zero. follow.py then tests geometry only."""
    Z = copy.deepcopy(L); Z["id"] = L["id"] + "__position_only"
    Z["position_only_note"] = ("Made by tools/position_turning_check.py: every force_on_object_N set to zero so follow.py tests "
                               "reach, facing and overlap only. The net force here is not the object's weight on purpose.")
    for f in Z["frames"]:
        f["force_on_object_N"] = [None if x is None else [0.0, 0.0, 0.0] for x in f["force_on_object_N"]]
    return Z


def net_force(L, k, Rw):
    """The frame's net force: the sum of the file's derived forces, which carry m(a - g). Only the sum is used, not the share-out."""
    f = L["frames"][k]; v = np.zeros(3)
    for i, x in enumerate(f["contact"]):
        if x and f["force_on_object_N"][i] is not None:
            v += np.array(f["force_on_object_N"][i])
    return Rw @ v


def turning_lp(pts, nrms, c, Js, taus, stops, W, axis, mu, R, off_axis_free):
    """(status, s): status "ok", "no_bound" or "cannot". off_axis_free drops the torque rows and maximises torque about the axis."""
    Gs, Fc = [], []
    for p, n in zip(pts, nrms):
        g = stability.cone_generators([p], [n], c, mu, CONE_EDGES)
        Gs.append(g); Fc.append(g[:3])
    G = np.hstack(Gs); ng = G.shape[1]
    nx = sum(j[1].shape[1] for j in Js); nt = sum(j[0].shape[0] for j in Js)
    scale = 1.0 / R
    A = np.zeros((6 + nt, ng + nx + 1)); b = np.zeros(6 + nt)
    A[:3, :ng] = G[:3]; b[:3] = W[:3]
    A[3:6, :ng] = G[3:] * scale; b[3:6] = W[3:] * scale
    A[3:6, ng + nx] = -axis * scale
    if off_axis_free:
        # the torque along the axis is the only torque row: its value minus the need is s
        A[3:6] = 0; b[3:6] = 0
        A[3, :ng] = (axis @ G[3:]) * scale; A[3, ng + nx] = -scale; b[3] = (axis @ W[3:]) * scale
    r0, x0, g0, bx = 6, ng, 0, []
    for (Tr, C), tau, fc, st in zip(Js, taus, Fc, stops):
        A[r0:r0 + Tr.shape[0], g0:g0 + fc.shape[1]] = Tr @ fc
        A[r0:r0 + Tr.shape[0], x0:x0 + C.shape[1]] = -C
        if tau is not None:
            b[r0:r0 + Tr.shape[0]] = -tau
        l, h = F._stop_box(C.shape[1], st)
        bx += [(float(a_), None if not np.isfinite(h_) else float(h_)) for a_, h_ in zip(l, h)]
        r0 += Tr.shape[0]; x0 += C.shape[1]; g0 += fc.shape[1]
    cost = np.zeros(ng + nx + 1); cost[-1] = -1.0
    keep = np.any(A != 0, axis=1) | (b != 0)
    res = linprog(cost, A_eq=A[keep], b_eq=b[keep], bounds=[(0, None)] * ng + bx + [(None, None)], method="highs")
    if res.status == 3:
        return "no_bound", None
    if res.status == 2 or not res.success:
        return "cannot", None
    return "ok", float(res.x[-1])


def follow_system_top(pts, nrms, c, Js, taus, W, axis, mu, R, bounds):
    """The largest s on follow.py's own torque stack (_torque_stack, _stack_rhs), solved exactly. Should equal turning_lp's s."""
    Gs, cidx, Fc = F.cap.generators(np.array(pts), np.array(nrms), c, mu, R)
    Mx, nt, nx, _ = F._torque_stack(Gs, cidx, Fc, Js, R, bounds)
    if bounds is None:
        jr, span = (np.zeros(nt) if taus is None else F._joint_rhs(Js, taus, R)), np.ones(nx)
    else:
        jr, span, _ = F._stack_rhs(Js, taus, bounds, R)
    rhs = np.concatenate([F.cap.scale_wrench(W, R), jr, span])
    d = np.concatenate([F.cap.scale_wrench(np.r_[0, 0, 0, axis], R), np.zeros(len(rhs) - 6)])
    cost = np.zeros(Mx.shape[1] + 1); cost[-1] = -1
    res = linprog(cost, A_eq=np.c_[Mx, -d], b_eq=rhs, bounds=[(0, None)] * Mx.shape[1] + [(None, None)], method="highs")
    return float(res.x[-1]) if res.status == 0 else float("nan")


def motion_share(hand, f, ik, Jf, v):
    """The share of the surface motion v the finger's joints can make at this pose, each free joint at a limit moving only away from it."""
    n = np.linalg.norm(v)
    if n < 1e-9:
        return None
    m = hand.model; E = F._free_map(hand, f, ik); A = Jf @ E
    lo, hi = np.full(A.shape[1], -np.inf), np.full(A.shape[1], np.inf)
    for c_, j in enumerate(ik.free):
        if m.jnt_limited[j]:
            q = float(ik.d.qpos[m.jnt_qposadr[j]])
            if q <= m.jnt_range[j][0] + F.STOP_TOL: lo[c_] = 0.0
            if q >= m.jnt_range[j][1] - F.STOP_TOL: hi[c_] = 0.0
    x = lsq_linear(A, v, bounds=(lo, hi)).x
    made = A @ x
    return float(np.clip((made @ v) / (n * n), 0.0, 1.0))


def run_hand(hk, stored, L, Z, mo):
    t0 = time.time()
    keys = [fr["k"] for fr in stored["frames"]]
    r0 = F.follow(hk, Z, n_eval=len(keys), placement=stored)
    assert [fr["k"] for fr in r0["frames"]] == keys, "the checked frames moved"
    mj = F._mj(); hand = Hand(hk); m = hand.model
    allow = F.overlap_allowance(Z); iks = {f.label: F.FingerIK(hand, f, allow) for f in hand.fingers}
    fingers = {f.label: f for f in hand.fingers}; lims = {f.label: F.torque_limits(hand, f) for f in hand.fingers}
    springs = {f.label: F.model_springs(hand, f) for f in hand.fingers}
    Rw = np.array(stored["placement"]["rotation_matrix"]); mu = float(L.get("mu", 1.0))
    mass, R = float(L["object"]["mass_kg"]), float(L["object"]["radius_m"]); I = 2 / 3 * mass * R * R
    out = []
    for fr, st in zip(r0["frames"], stored["frames"]):
        k, c = fr["k"], np.array(fr["ball_center"])
        om = Rw @ mo.omega(k); speed = float(np.linalg.norm(om)); axis = om / max(speed, 1e-9)
        W = np.concatenate([net_force(L, k, Rw), I * (Rw @ mo.alpha(k))])
        vc = Rw @ mo.centre_velocity(k)
        pts, nrms, Js, taus, stops, shares = [], [], [], [], [], {}
        for x in fr["fingers"]:
            if x["state"] not in ("green", "amber"):
                continue
            lab = x["finger"]; f, ik = fingers[lab], iks[lab]; M, lim = lims[lab]
            q, pw = np.array(x["q"]), np.array(x["pad_world"])
            ik.d.qpos[:] = hand.apply_coupling(F._q_full(m, ik, q)); mj.mj_kinematics(m, ik.d); mj.mj_comPos(m, ik.d)
            b = f.tip_body if x["phalanx"] == "distal" else int(m.body_parentid[f.tip_body])
            jacp = np.zeros((3, m.nv)); mj.mj_jac(m, ik.d, jacp, None, pw, b); Jf = jacp[:, f.dof]
            v_ = pw - c; cp = c + R * v_ / np.linalg.norm(v_); n_in = (c - cp) / R
            st_ = F._stops(hand, f, ik) if F.STOP_REACTIONS and M is not None else []
            if M is None:
                raise RuntimeError(f"{hk} {lab}: no strength data; the turning check needs it")
            cols = F._finger_columns(hand, f, ik, M, lim, Jf, pw, F.JOINT_STIFFNESS, st_)
            tau = F._spring_item(hand, f, ik, M, lim, Jf, pw, springs[lab])[1] if springs[lab] is not None else None
            pts.append(cp); nrms.append(n_in); Js.append(cols); taus.append(tau); stops.append(len(st_))
            shares[lab] = motion_share(hand, f, ik, Jf, vc + np.cross(om, cp - c))
        rec = dict(k=k, t=float(mo.t[k]), turn_deg_s=float(np.degrees(speed)), omega_palm_rad_s=mo.omega(k).tolist(), position=fr["verdict"], position_reason=fr["reason"],
                   touching=[x["finger"] for x in fr["fingers"] if x["state"] in ("green", "amber")],
                   states={x["finger"]: x["state"] for x in fr["fingers"]}, reach_mm={x["finger"]: x.get("reach_mm") for x in fr["fingers"]},
                   unserved=fr["unserved"], need_torque_along_turn_mNm=float(W[3:] @ axis) * 1e3, net_force_N=float(np.linalg.norm(W[:3])),
                   follow_share=shares)
        if pts:
            st1, s1 = turning_lp(pts, nrms, c, Js, taus, stops, W, axis, mu, R, False)
            st2, s2 = turning_lp(pts, nrms, c, Js, taus, stops, W, axis, mu, R, True)
        else:
            st1, s1, st2, s2 = "cannot", None, "cannot", None
        rec.update(on_axis=st1, spare_on_axis_mNm=None if s1 is None else s1 * 1e3, off_axis_free=st2, spare_off_axis_free_mNm=None if s2 is None else s2 * 1e3)
        if st1 == "ok":     # cross-check: the top of follow.py's own stacked system, with no tolerance
            bnds = [F._stop_box(j[1].shape[1], n_) if n_ else None for j, n_ in zip(Js, stops)]
            tt = taus if any(t_ is not None for t_ in taus) else None; bb = bnds if any(b_ is not None for b_ in bnds) else None
            rec["follow_system_top_mNm"] = follow_system_top(pts, nrms, c, Js, tt, W, axis, mu, R, bb) * 1e3
            Wc = W.copy(); Wc[3:] += axis * (s1 + max(.05 * abs(s1), .002))     # and its solver, tolerance tightened, refuses 5% above the top
            rec["follow_solver_refuses_above"] = not F._redistribute(pts, nrms, Js, c, Wc, mu, R, TIGHT_TOL, tt, bb, {})[0]
        out.append(rec)
    # the reach under no forces should be no worse than with them; report any finger where it is
    worse = [(fr["k"], x["finger"], x.get("reach_mm"), y.get("reach_mm"))
             for fr, st in zip(r0["frames"], stored["frames"]) for x, y in zip(fr["fingers"], st["fingers"])
             if x.get("reach_mm") is not None and y.get("reach_mm") is not None and x["reach_mm"] > y["reach_mm"] + .05]
    return dict(hand=hk, leader=L["id"], placement=stored["placement"], assignment=stored["assignment"], placement_key=stored.get("placement_key"),
                respace_deg=stored.get("respace_deg"), placement_cost_mm=stored.get("placement_cost_mm"),
                note="Frozen poses. Turning check built for this film, not the banked method. Mass and friction are the leader's assumed values.",
                frames=out, position_run=r0, reach_worse_than_with_forces=worse, elapsed_s=round(time.time() - t0, 1))


def summary(r, stored):
    fr = r["frames"]; n = len(fr); C = collections.Counter
    print(f"\n{r['hand']}: {n} frames, {r['elapsed_s']} s")
    print("  with force targets, stored:", dict(C(x["verdict"] for x in stored["frames"])), dict(C(x["reason"] for x in stored["frames"] if x["reason"])))
    print("  position only:", dict(C(x["position"] for x in fr)), dict(C(x["position_reason"] for x in fr if x["position_reason"])))
    print("  touching fingers per frame:", dict(sorted(C(len(x["touching"]) for x in fr).items())), "; frames with a contact no finger has:", sum(bool(x["unserved"]) for x in fr))
    print("  on the axis:", dict(C(x["on_axis"] for x in fr)), " off-axis free:", dict(C(x["off_axis_free"] for x in fr)))
    for key, lab in (("spare_on_axis_mNm", "spare torque, on the axis"), ("spare_off_axis_free_mNm", "spare torque, off-axis free")):
        v = np.array([x[key] for x in fr if x[key] is not None])
        if len(v):
            print(f"  {lab}, mN m: min {v.min():.2f}, median {np.median(v):.2f}, max {v.max():.1f}; above zero {int((v > 0).sum())} of {len(v)}")
    need = np.array([abs(x["need_torque_along_turn_mNm"]) for x in fr]); print(f"  torque the turn itself needs (I alpha), mN m: abs median {np.median(need):.3f}, max {need.max():.2f}")
    sh = [v for x in fr for v in x["follow_share"].values() if v is not None]
    if sh:
        print(f"  share of the surface motion a touching finger can follow: median {np.median(sh):.2f}, below 0.9 in {sum(v < .9 for v in sh)} of {len(sh)}")
    e = np.array([abs(x["follow_system_top_mNm"] - x["spare_on_axis_mNm"]) for x in fr if "follow_system_top_mNm" in x])
    print(f"  difference from the top of follow.py's own system, solved exactly, mN m: largest {e.max() if len(e) else 0:.5f} over {len(e)} frames")
    ab = [x["follow_solver_refuses_above"] for x in fr if "follow_solver_refuses_above" in x]
    print(f"  follow.py's solver, tolerance tightened, refuses 5% above the top: {sum(ab)} of {len(ab)}")
    print(f"  fingers with reach worse than with forces: {len(r['reach_worse_than_with_forces'])}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--leader", required=True); ap.add_argument("--derived", required=True)
    ap.add_argument("--runs", nargs="+", required=True); ap.add_argument("--out-dir", required=True)
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    L = json.load(open(a.leader, encoding="utf-8")); B = json.load(open(a.derived, encoding="utf-8"))
    Z = position_leader(L); pz = os.path.join(a.out_dir, Z["id"] + ".json")
    if not os.path.exists(pz):
        json.dump(Z, open(pz, "w", encoding="utf-8"), indent=1)
    mo = Motion(L, B)
    for p in a.runs:
        stored = json.load(open(p, encoding="utf-8"))
        r = run_hand(stored["hand"], stored, L, Z, mo)
        q = os.path.join(a.out_dir, f"position_turning_{stored['hand']}.json")
        assert not os.path.exists(q), f"{q} exists: not overwriting"
        json.dump(r, open(q, "w", encoding="utf-8"), indent=1, default=float)
        q2 = os.path.join(a.out_dir, f"follow_{stored['hand']}__{Z['id']}.json")     # the position run alone, in follow.py's own format
        assert not os.path.exists(q2), f"{q2} exists: not overwriting"
        json.dump(r["position_run"], open(q2, "w", encoding="utf-8"), indent=1, default=float)
        summary(r, stored)
        print("  wrote", q, flush=True)


if __name__ == "__main__":
    main()
