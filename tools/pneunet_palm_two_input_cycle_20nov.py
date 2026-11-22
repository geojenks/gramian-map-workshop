"""Pneu-net fingers on a palm turning a ball: two sets of fingers on two
separate inputs, stepped round a closed cycle of the inputs. SIMULATION ONLY,
kinematic, comparative.

Follows tools/pneunet_palm_turn_ball_20nov.py (same finger model, same mount
conditions, same ball). That script shows one shared input gives no net turn.
This one asks: with two inputs, does a closed cycle of the inputs leave the
ball turned?

Cycle (both sets use the same mount, the second set half a pitch round):
  1. first set goes out alone, second set waits at zero input;
  2. second set goes out alone, first set waits at its last input level;
  3. both sets come back together, level for level.
Slip ignored: while a tip is pressed into the ball the ball turns with it.

Checked at every input level of the cycle, not on a coarse grid:
  - the waiting set's tips and bodies are clear of the ball;
  - bodies of the two sets, and of neighbours within a set, stay at least
    two tip radii apart.

Also prints, for the hands chosen by the first script, how much further the
ball's ring moves than the tip travels in contact (read from results.json).

Run from the repository root:
    python tools/pneunet_palm_two_input_cycle_20nov.py
Writes outputs/pneunet_palm_turn_ball_20nov/two_input_cycle_report.txt and
two_input_cycle.json.
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pneunet_palm_turn_ball_20nov as M  # noqa: E402


def min_dist(X, Y):
    return float(np.sqrt(((X[:, None, :] - Y[None, :, :]) ** 2).sum(-1)).min())


def cycle(fg, m_, n, rho):
    """Step the three-phase cycle at every input level. Returns a dict."""
    B, c = fg["B"], m_["c"]
    L = len(fg["u"]) - 1
    i, j = m_["i"], m_["j"]
    gap = m_["gap"]
    psi = np.zeros(L + 1)                       # ball angle a set gives
    psi[i:j + 1] = np.abs(m_["theta"] - m_["theta"][0])
    psi[j + 1:] = psi[j]
    rb = np.linalg.norm(B - c, axis=2)          # body to axis, per level
    up = list(range(L + 1))
    sched = ([(k, 0) for k in up] + [(L, k) for k in up]
             + [(k, k) for k in up[::-1]])
    Rhalf_p, Rhalf_m = M.rot2(np.pi / n), M.rot2(-np.pi / n)
    Rfull = M.rot2(2 * np.pi / n)
    ball, angle = [0.0], 0.0
    both_in = 0
    gap_sets, gap_same = np.inf, np.inf
    wait_tip, wait_body = np.inf, np.inf
    for s in range(1, len(sched)):
        (a0, b0), (a1, b1) = sched[s - 1], sched[s]
        da, db = psi[a1] - psi[a0], psi[b1] - psi[b0]
        a_in = i <= min(a0, a1) and max(a0, a1) <= j and a0 != a1
        b_in = i <= min(b0, b1) and max(b0, b1) <= j and b0 != b1
        if a_in and b_in:
            both_in += 1
            assert abs(da - db) < 1e-12        # same mount, same level
            angle += da
        else:
            angle += da + db
        ball.append(angle)
        XA = B[a1] - c
        XB = B[b1] - c
        gap_sets = min(gap_sets, min_dist(XA, XB @ Rhalf_p.T),
                       min_dist(XA, XB @ Rhalf_m.T))
        gap_same = min(gap_same, min_dist(XA, XA @ Rfull.T),
                       min_dist(XB, XB @ Rfull.T))
        # a set that is not driving the ball must be clear of it
        for k_move, k_wait in ((a1, b1), (b1, a1)):
            if not (i <= k_wait <= j) and (i <= k_move <= j):
                wait_tip = min(wait_tip, gap[k_wait])
                wait_body = min(wait_body, rb[k_wait].min() - rho)
    return {"ball_deg": np.degrees(ball).tolist(),
            "net_deg": float(np.degrees(angle)),
            "one_set_deg": float(np.degrees(psi[-1])),
            "steps_with_both_sets_in_contact": int(both_in),
            "bodies_two_sets_min_m": gap_sets,
            "bodies_same_set_min_m": gap_same,
            "waiting_tip_gap_min_m": float(wait_tip),
            "waiting_body_gap_min_m": float(wait_body),
            "schedule": sched}


def main():
    task = json.load(open(M.TASK))
    R = task["object"]["radius_m"]
    alt = abs(np.radians(task["contact_ring"]["altitude_deg"]))
    res_first = json.load(open(os.path.join(M.OUT, "results.json")))
    lines = []
    say = lines.append
    say("Pneu-net fingers on a palm turning a ball: two sets of fingers on "
        "two inputs, one closed cycle of the inputs. SIMULATION ONLY, "
        "comparative. Slip ignored.")
    say("script: tools/pneunet_palm_two_input_cycle_20nov.py")
    say("cycle: first set out alone; second set out alone; both back "
        "together")
    out = {}
    for rid in M.RECORDS:
        fg = M.load_finger(rid)
        u = fg["u"]
        rc = fg["tip_r"]
        rho = (R + rc) * np.cos(alt)
        idx = np.unique(np.linspace(0, len(u) - 1, M.GRID).round().astype(int))
        cands = []
        for a_ in range(len(idx)):
            for b_ in range(a_ + 1, len(idx)):
                for side in (+1, -1):
                    m_ = M.evaluate(fg, idx[a_], idx[b_], side, rho)
                    if m_ is None:
                        continue
                    if not (m_["approach_clear"] and m_["tip_deepest"]):
                        continue
                    if m_["i"] == 0 or m_["j"] >= len(u) - 1 \
                            or not m_["after_clear"]:
                        continue
                    cands.append(m_)
        cands.sort(key=lambda m_: -m_["turn_rad"])
        say("")
        say(f"=== {rid} ===  ({len(cands)} mounts whose tip enters and "
            f"leaves contact clear of the ball; need bodies "
            f"{2 * rc * 1e3:.1f} mm apart)")
        out[rid] = {}
        # ring travel against tip travel, hands of the first script
        for n in M.N_FINGERS:
            ch = res_first["records"][rid]["hands"][str(n)]["chosen"][
                "record depth"]
            ratio = ch["ring_arc_m"] / ch["tip_travel_in_contact_m"]
            say(f"  one shared input, {n} fingers: ring moves "
                f"{ch['ring_arc_m'] * 1e3:.1f} mm for "
                f"{ch['tip_travel_in_contact_m'] * 1e3:.1f} mm of tip travel "
                f"in contact, {100 * (ratio - 1):.0f}% more")
        for n in M.N_FINGERS:
            found, tried, why = None, 0, {"two sets": 0, "same set": 0,
                                           "waiting set touches": 0}
            for m_ in cands:
                tried += 1
                # quick first filter on a coarse grid of level pairs (the
                # first script's check); the full cycle is stepped only for
                # mounts that pass it
                if M.neighbour_gap(fg, m_["c"], 2 * n, 0, len(u) - 1, m=12,
                                   other=(0, len(u) - 1)) < 2 * rc:
                    why["two sets"] += 1
                    continue
                r = cycle(fg, m_, n, rho)
                if r["bodies_two_sets_min_m"] < 2 * rc:
                    why["two sets"] += 1
                    continue
                if r["bodies_same_set_min_m"] < 2 * rc:
                    why["same set"] += 1
                    continue
                if r["waiting_tip_gap_min_m"] < 0 \
                        or r["waiting_body_gap_min_m"] < -1e-9:
                    why["waiting set touches"] += 1
                    continue
                found = (m_, r)
                break
            if found is None:
                say(f"  two sets of {n}: no mount passes the cycle "
                    f"({tried} tried; failed on bodies of the two sets "
                    f"{why['two sets']}, bodies within a set "
                    f"{why['same set']}, waiting set touching the ball "
                    f"{why['waiting set touches']})")
                out[rid][str(n)] = {"found": False, "tried": tried,
                                    "failed_on": why}
                continue
            m_, r = found
            # one shared input on the same mount, for comparison
            psi_end = r["one_set_deg"]
            say(f"  two sets of {n} ({2 * n} fingers, 2 inputs): mount "
                f"number {tried} by size of turn; contact from "
                f"{u[m_['i']]:.0f} to {u[m_['j']]:.0f} Pa, deepest push-out "
                f"{m_['depth_max'] * 1e3:.1f} mm")
            say(f"     first set out +{psi_end:.1f} deg, second set out "
                f"+{psi_end:.1f} deg, both back -{psi_end:.1f} deg: net "
                f"{r['net_deg']:.1f} deg per cycle of the inputs "
                f"(3 cycles {3 * r['net_deg']:.1f} deg); the same mount on "
                f"one shared input: net 0.0 deg")
            say(f"     checked at every level: bodies of the two sets at "
                f"least {r['bodies_two_sets_min_m'] * 1e3:.1f} mm apart, "
                f"within a set {r['bodies_same_set_min_m'] * 1e3:.1f} mm; "
                f"the waiting set's tip at least "
                f"{r['waiting_tip_gap_min_m'] * 1e3:.2f} mm clear of the "
                f"ball and its body {r['waiting_body_gap_min_m'] * 1e3:.2f} "
                f"mm clear")
            out[rid][str(n)] = {
                "found": True, "mount_rank": tried,
                "level_i": m_["i"], "level_j": m_["j"], "side": m_["side"],
                "axis_in_finger_frame_m": m_["c"].tolist(),
                "contact_from_Pa": float(u[m_["i"]]),
                "contact_to_Pa": float(u[m_["j"]]),
                "depth_max_m": m_["depth_max"],
                **{k: v for k, v in r.items() if k != "schedule"}}
    with open(os.path.join(M.OUT, "two_input_cycle.json"), "w") as f:
        json.dump(out, f, indent=1)
    with open(os.path.join(M.OUT, "two_input_cycle_report.txt"), "w",
              encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
