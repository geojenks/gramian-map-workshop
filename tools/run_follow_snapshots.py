"""Check other hands against frozen snapshots taken from a non-spherical object's log.

follow.py assumes a sphere: one centre per frame sets each contact's normal. On the Hora
cylinder that tilts the normal by up to about 40 degrees, so its answer would be wrong. This
script keeps follow.py's tests and limits, called from follow.py itself, and changes one thing:
each finger is posed against a sphere of the object's radius touching the logged contact point
with the logged normal, so the local tangent plane is the real one. Links other than the
touching one, and the palm, are therefore tested against spheres, not the whole cylinder.
The object's wrench, its share-out between fingers and the palm test use the true object centre.

Each snapshot is checked frozen. No hand is simulated doing the motion, and the poses are
schematic (a finger may sit inside another finger; outputs/hold_pose_self_overlap.md).

    python tools/run_follow_snapshots.py --snapshots <..._snapshots.json> --out-dir <dir>
        [--hands allegro leap shadow] [--mu 0.7212] [--mass 0.02394] [--radius 0.0329]
        [--plain-check]
    python tools/run_follow_snapshots.py --restate follow_*.json --leader-json <leader file>

--plain-check runs this script's loop with the object as a plain sphere, and compares it frame by
frame with follow.follow at the same placement: they should agree exactly.
"""
import argparse, collections, json, math, os, sys, time
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop import follow as F  # noqa: E402
from gramian_workshop.hands import Hand  # noqa: E402


def leader_from_snapshots(path, mu, mass, radius, leader_id):
    """One leader frame per snapshot. A fifth station that never touches lets a five-fingered
    hand be paired up (the pairing needs a station for every finger)."""
    D = json.load(open(path, encoding="utf-8"))
    names = list(D["snapshots"][0]["fingers"])
    frames = []
    for s in D["snapshots"]:
        fs = [s["fingers"][n] for n in names]
        O = s["obj_pos"]
        frames.append(dict(
            t=s["t_s"], obj_pos=O,
            contact=[f is not None for f in fs] + [False],
            contact_point=[f["point_m"] if f else None for f in fs] + [None],
            station_pos=[f["point_m"] if f else O for f in fs] + [O],
            force_on_object_N=[(np.array(f["normal_N"]) + np.array(f["shear_N"])).tolist() if f else [0., 0., 0.] for f in fs] + [[0., 0., 0.]],
            normal_in=[(np.array(f["normal_N"]) / np.linalg.norm(f["normal_N"])).tolist() if f else None for f in fs] + [None]))
    return dict(id=leader_id, kind="snapshots from a simulated log", source=D["source"], averaged_over_s=D["averaged_over_s"],
                caveat="shear is derived from the object's motion, and its split between fingers is one consistent split, not the only one",
                object=dict(radius_m=radius, mass_kg=mass), mu=mu, n_stations=len(names) + 1,
                stations=names + ["never touches"], frames=frames)


class ContactLeader(F.Leader):
    """station's centre is the touching sphere's. center stays the true object centre."""
    def station(self, k, i, Rw, t, raw=False):
        f = self.frames[k]
        p = Rw @ (np.array(f["contact_point"][i]) - self.c0) + t
        n = Rw @ np.array(f["normal_in"][i])
        return p, n, p + self.R_obj * n


def check(hand_key, L, placement=None, plain=False, seed=0):
    """follow._follow's per-frame loop at its default options, with a centre per contact."""
    allow = F.overlap_allowance(L)
    a_c, a_o = allow["contact_m"], allow["other_m"]
    hand = Hand(hand_key)
    leader = F.Leader(L) if plain else ContactLeader(L)
    cands = [F.pad_candidates(hand, f, True) for f in hand.fingers]
    iks = [F.FingerIK(hand, f, allow) for f in hand.fingers]
    mu = float(L["mu"])
    for ik in iks:
        ik.mu = mu
    if placement is None:
        cost, Rw, t, assign, _ = F.placement_search(hand, leader, cands, iks, seed=seed, **F.PLACE_FINE)
    else:
        Rw, t = np.array(placement["placement"]["rotation_matrix"]), np.array(placement["placement"]["translation_m"])
        assign = [int(placement["assignment"][f.label]) for f in hand.fingers]
        cost = placement["placement_cost_mm"] / 1e3
    lims = [F.torque_limits(hand, f) for f in hand.fingers]
    springs = [F.model_springs(hand, f) for f in hand.fingers]
    use_taus = any(s is not None for s in springs)
    slide = int(F._mj().mjtJoint.mjJNT_SLIDE)
    kinds = []
    for ik in iks:
        kd = ["slide" if int(hand.model.jnt_type[j]) == slide else "hinge" for j in ik.free]
        kinds.append(kd if len(set(kd)) > 1 else None)
    mj, m, kq, R = F._mj(), hand.model, F.JOINT_STIFFNESS, leader.R_obj
    frames = []
    for k in range(len(leader.frames)):
        made = leader.made(k)
        c = leader.center(k, Rw, t)
        W = np.zeros(6)
        for i in made:
            p = leader.station(k, i, Rw, t, raw=True)[0]
            fv = leader.force(k, i, Rw, raw=True)
            W += np.concatenate([fv, np.cross(p - c, fv)])
        per, pts, nrms, Js, taus, bounds = [], [], [], [], [], []
        for a, f in enumerate(hand.fingers):
            i = assign[a]
            if i not in made:
                per.append({"finger": f.label, "station": i, "state": "off"})
                continue
            p, _, cl = leader.station(k, i, Rw, t)
            (b, pl, nl), r_ = iks[a].best_tangent(p, cl, R, middle=True, force=leader.force(k, i, Rw))
            q, res, face, pw, nw, pen, pc, po = r_[:8]
            r_link = iks[a].proxies(True)[b][2]
            patch = math.sqrt(2.0 * (R * r_link / (R + r_link)) * a_c)
            o_rank = F._overlap_rank(pc, po, allow)
            near = res <= F.REACH_AMBER_M + patch and face <= F.SIDE_AMBER_DEG and o_rank <= 1
            exact = res <= F.REACH_GREEN_M + patch and face <= F.SIDE_GREEN_DEG and o_rank == 0
            item = {"finger": f.label, "station": i, "reach_mm": res * 1e3, "facing_deg": face,
                    "phalanx": "distal" if b == f.tip_body else "middle", "q": [float(x) for x in q], "target": p.tolist(),
                    "pad_world": pw.tolist(), "pad_normal": nw.tolist(), "penetration_mm": max(0.0, pen) * 1e3,
                    "side_deg": face, "patch_mm": patch * 1e3}
            if not near:
                item["state"] = "red"
                item["reason"] = "reach" if res > F.REACH_AMBER_M + patch else "nail side" if face > F.SIDE_AMBER_DEG else "penetrates"
                if not (-F.TOUCH_TOL_M <= pen and o_rank <= 1 and face <= F.SIDE_AMBER_DEG):
                    per.append(item)
                    continue
                item["used_in_force_check"] = True
            v = pw - cl
            cp = cl + R * v / np.linalg.norm(v)
            n_in = (cl - cp) / np.linalg.norm(cl - cp)
            M, lim = lims[a]
            iks[a].d.qpos[:] = hand.apply_coupling(F._q_full(m, iks[a], q))
            mj.mj_kinematics(m, iks[a].d)
            mj.mj_comPos(m, iks[a].d)
            jacp = np.zeros((3, m.nv))
            mj.mj_jac(m, iks[a].d, jacp, None, pw, b)
            Jf = jacp[:, f.dof]
            fv = leader.force(k, i, Rw)
            in_cone = fv @ n_in >= math.cos(math.atan(mu)) * np.linalg.norm(fv) - 1e-12
            torque_ok, cols, tau, box = True, None, None, None
            stops = F._stops(hand, f, iks[a]) if F.STOP_REACTIONS and M is not None else []
            if M is not None:
                cols = F._finger_columns(hand, f, iks[a], M, lim, Jf, pw, kq, stops)
                if springs[a] is not None:
                    item["spring"], tau = F._spring_item(hand, f, iks[a], M, lim, Jf, pw, springs[a])
                if stops:
                    box = F._stop_box(cols[1].shape[1], len(stops), box)
                torque_ok, un, _ = F._hold_solve(cols, fv, tau, box, kinds[a])
                item["unbalanced_Nm"] = un
            if item.get("state") != "red":
                item["state"] = "green" if (exact and in_cone and torque_ok) else "amber?"
            item.update(in_cone=bool(in_cone), torque_ok=bool(torque_ok), torque_checked=M is not None)
            per.append(item); pts.append(cp); nrms.append(n_in); Js.append(cols); taus.append(tau); bounds.append(box)
        unserved = [i for i in made if i not in assign]
        need = bool(unserved) or any(x.get("state") in ("amber?", "red") for x in per)
        all_green = not unserved and all(x["state"] in ("green", "off") for x in per)
        verdict, reason = "green", None
        if need:
            ok, reason = F._redistribute(pts, nrms, Js, c, W, mu, R, None, taus if use_taus else None,
                                         bounds if any(b is not None for b in bounds) else None, {})
            verdict = ("green" if all_green else "amber") if ok else "red"
            for x in per:
                if x.get("state") == "amber?":
                    x["state"] = "amber" if ok else "red"
        palm_raw = F.palm_penetration(hand, c, R)
        if palm_raw > a_o + a_c:
            verdict, reason = "red", "palm penetrates"
        elif palm_raw > a_o and verdict == "green":
            verdict = "amber"
        frames.append({"k": k, "verdict": verdict, "reason": reason, "ball_center": c.tolist(),
                       "palm_penetration_mm": max(0.0, palm_raw) * 1e3, "unserved": unserved, "fingers": per})
    return {"hand": hand_key, "hand_name": hand.spec["name"], "leader": L["id"], "placement_cost_mm": cost * 1e3,
            "method": "plain sphere" if plain else "follow.py's tests, each finger against a sphere touching the logged contact with the logged normal; other links and the palm on spheres",
            "assignment": {hand.fingers[a].label: int(assign[a]) for a in range(len(hand.fingers))},
            "respace_deg": [0.0] * leader.n, "placement": {"rotation_matrix": Rw.tolist(), "translation_m": t.tolist()}, "frames": frames}


def cost_over_touching(r, L):
    """Placement cost over the stations that touch at some frame. follow.py's search charges 1 m
    for pairing a finger with a station that never touches, shared over the hand's fingers. A
    five-fingered hand must take that one station, so the charge is the same for all its pairings:
    the search's choice does not change, only the number it reports."""
    touches = [any(f["contact"][i] for f in L["frames"]) for i in range(L["n_stations"])]
    nf = len(r["assignment"]); idle = sum(not touches[i] for i in r["assignment"].values())
    a_o = F.overlap_allowance(L)["other_m"] * 1e3
    assert all(fr["palm_penetration_mm"] <= a_o for fr in r["frames"]), "the cost holds a palm term as well: restate it from the search"
    return (r["placement_cost_mm"] - idle * 1e3 / nf) * nf / (nf - idle)


def show(r, L):
    print(f"{r['hand']}: placement cost over touching stations {cost_over_touching(r, L):.2f} mm (search's own figure {r['placement_cost_mm']:.2f} mm), pairing {r['assignment']}", flush=True)
    for fr in r["frames"]:
        print(f"  snapshot {fr['k']}: {fr['verdict']} ({fr['reason']}), palm {fr['palm_penetration_mm']:.1f} mm; " + ", ".join(
            f"{x['finger']} {x['state']}" + (f" reach {x['reach_mm']:.1f} side {x['side_deg']:.0f} cone {x.get('in_cone')} torque {x.get('torque_ok')}" if "reach_mm" in x else "")
            for x in fr["fingers"]), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshots")
    ap.add_argument("--out-dir")
    ap.add_argument("--restate", nargs="+", help="stored result files: print each one's cost over touching stations, then stop")
    ap.add_argument("--leader-json", help="with --restate, the leader file those runs wrote")
    ap.add_argument("--hands", nargs="*", default=["allegro", "leap", "shadow"])
    ap.add_argument("--mu", type=float, default=0.7212)
    ap.add_argument("--mass", type=float, default=0.02394)
    ap.add_argument("--radius", type=float, default=0.0329)
    ap.add_argument("--leader-id", default="hora_seed42_five_snapshots")
    ap.add_argument("--plain-check", action="store_true")
    a = ap.parse_args()
    if a.restate:
        L = json.load(open(a.leader_json, encoding="utf-8"))
        for p in a.restate:
            r = json.load(open(p, encoding="utf-8"))
            print(f"{r['hand']}: over touching stations {cost_over_touching(r, L):.3f} mm; search's own figure {r['placement_cost_mm']:.3f} mm; pairing {r['assignment']}")
        return
    os.makedirs(a.out_dir, exist_ok=True)
    L = leader_from_snapshots(a.snapshots, a.mu, a.mass, a.radius, a.leader_id)
    with open(os.path.join(a.out_dir, a.leader_id + ".json"), "w", encoding="utf-8") as f:
        json.dump(L, f, indent=1)
    for hk in a.hands:
        t0 = time.time()
        if a.plain_check:
            mine = check(hk, L, plain=True)
            ref = F.follow(hk, L, placement=mine)
            same = all(x["verdict"] == y["verdict"] and x["reason"] == y["reason"] and
                       [u["state"] for u in x["fingers"]] == [u["state"] for u in y["fingers"]] and
                       all(np.allclose(u["q"], v["q"]) for u, v in zip(x["fingers"], y["fingers"]) if "q" in u)
                       for x, y in zip(mine["frames"], ref["frames"]))
            print(f"{hk}: this loop on a plain sphere {'matches' if same else 'DIFFERS FROM'} follow.follow at the same placement "
                  f"({collections.Counter(x['verdict'] for x in mine['frames'])} against {collections.Counter(x['verdict'] for x in ref['frames'])}), {time.time() - t0:.0f} s", flush=True)
            if not same:
                show(mine, L); show(ref, L)
            continue
        r = check(hk, L)
        r["placement_cost_touching_mm"] = cost_over_touching(r, L)
        r["elapsed_s"] = round(time.time() - t0, 1)
        with open(os.path.join(a.out_dir, f"follow_{hk}__{a.leader_id}.json"), "w", encoding="utf-8") as f:
            json.dump(r, f, indent=1, default=float)
        show(r, L)
        print(f"   {r['elapsed_s']} s", flush=True)


if __name__ == "__main__":
    main()
