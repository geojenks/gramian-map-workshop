"""Hands of three or more pneu-net fingers on a palm, turning a ball: how far
does one stroke turn it, and can the fingers come back for a second stroke?

    python tools/pneunet_palm_turn_ball_20nov.py
      -> outputs/pneunet_palm_turn_ball_20nov/results.json
         outputs/pneunet_palm_turn_ball_20nov/report.txt

Kinematic only, numpy only, records read and never edited. SIMULATION ONLY:
magnitudes are comparative, not predictive.

What is modelled
  ball      sphere, radius, mass, friction and contact ring altitude from
            data/tasks/cup_turn.json; it turns about the vertical through
            its centre.
  finger    one pneu-net record. Its tip follows the record's free path
            (free.tip_displacement_m, one point per input level): one input,
            so one curve. The tip is a ball of the radius
            tools/build_record_hand.py gives a stand-in (cylinder of the
            record's material volume over its length).
  palm      a flat plate under the ball. Every finger lies flat on it with
            its bending plane parallel to the plate, at the height where a
            tip touching the ball touches it on the task's contact ring. The
            N fingers are copies turned by 360/N about the ball's axis and
            driven together.
  contact   the tip is on the ball while its free path lies inside the
            sphere of (ball radius + tip radius). The soft finger gives way
            in its bending plane, so the tip sits on the ring, pushed out by
            the depth the free path is inside. No band is chosen: contact
            starts and ends where the free path crosses that sphere.
  turn      slip ignored, the ball turns with the tip: the angle is the
            azimuth the tip sweeps about the ball's axis between the two
            crossings.
  mount     fixed by the two input levels where contact starts and ends and
            by which side of the path the ball sits. All pairs on a grid of
            input levels are tried.

A mount is kept only if
  - the path between the two crossings is inside throughout and the azimuth
    is monotonic (the ball turns one way),
  - the finger (fitted stand-in body, free shape) is clear of the ball until
    the tip makes contact,
  - while in contact the tip is the part of the finger nearest the axis,
  - neighbouring fingers' bodies stay at least two tip radii apart from rest
    to the end of contact.
The body is the jointed stand-in fitted by gramian_workshop.record_fit
(the record holds no body shape under input), in its free shape.

How deep the path is set into the ball (the push-out depth) is a design
choice, so the turn is reported against the largest depth allowed, as a
curve. Two depths come from the record and the task, not from us:
  hold depth    N fingers carry the ball's weight when N k depth (tan(alt)
                + mu) >= m g, with k the record's tip stiffness at zero
                input (in-plane transverse);
  record depth  the record's largest blocked force over that stiffness.
"""
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
from gramian_workshop.finger import load_record  # noqa: E402
from gramian_workshop.record_fit import Chain, a_of_u, fit_record  # noqa: E402
from build_record_hand import contact_radius  # noqa: E402

OUT = os.path.join(ROOT, "outputs", "pneunet_palm_turn_ball_20nov")
TASK = os.path.join(ROOT, "data", "tasks", "cup_turn.json")
RECORDS = ["wall_0.0005m", "wall_0.0010m", "wall_0.0015m", "wall_0.0020m"]
N_FINGERS = [3, 4, 6]
GRID = 64                 # input levels tried for contact start and end
PTS_PER_LINK = 10         # body points per stand-in link
DEPTH_GRID_MM = np.arange(1.0, 30.5, 1.0)


def rot2(a):
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def load_finger(rid):
    rec = load_record(rid)
    u = np.concatenate([[0.0], np.asarray(rec["free"]["u"], float).ravel()])
    d = np.asarray(rec["free"]["tip_displacement_m"], float)
    fit = fit_record(rid)
    f = fit["fit"]
    ch = Chain(f["n_hinges"], f["slide"], f["length_m"])
    r = np.asarray(f["ratios"], float)
    P = np.vstack([[0.0, 0.0], d[:, :2]]) + np.array([f["length_m"], 0.0])
    # stand-in body points (free shape) at every input level, finger frame
    B = np.zeros((len(u), ch.n_h * PTS_PER_LINK + 1, 2))
    t = np.linspace(0, 1, PTS_PER_LINK, endpoint=False)
    for k, uk in enumerate(u):
        q = r * a_of_u(f, uk)
        s = q[0] if ch.slide else 0.0
        phi = np.cumsum(q[int(ch.slide):])
        j = np.array([s, 0.0])
        row = []
        for h in range(ch.n_h):
            e = np.array([np.cos(phi[h]), np.sin(phi[h])])
            row += [j + ch.l * tt * e for tt in t]
            j = j + ch.l * e
        row.append(j)
        B[k] = np.array(row)
    rad, rad_src = contact_radius(rec)
    st = rec["stiffness"]
    return {"id": rid, "rec": rec, "u": u, "P": P, "B": B, "tip_r": rad,
            "tip_r_source": rad_src, "k": float(st["k_N_per_m"][0]),
            "k_out_of_plane": float(st["k_out_of_plane_N_per_m"][0]),
            "F_blocked": float(rec["scalars"]["blocked_force_N"]),
            "stroke": float(rec["scalars"]["free_stroke_m"]),
            "fit_pass": bool(fit["pass"]),
            "fit_free_err_max_m": float(f["free_err_max_m"]),
            "out_of_plane_max_m": float(np.abs(d[:, 2]).max())}


def evaluate(fg, i, j, side, rho):
    """Mount with contact from level i to level j, ball axis on `side` of
    the chord. Returns dict or None (not a mount)."""
    P, B = fg["P"], fg["B"]
    a, b = P[i], P[j]
    ch = np.linalg.norm(b - a)
    if ch <= 0 or ch >= 2 * rho:
        return None
    nrm = np.array([-(b - a)[1], (b - a)[0]]) / ch
    c = 0.5 * (a + b) + side * np.sqrt(rho ** 2 - (ch / 2) ** 2) * nrm
    rt = np.linalg.norm(P - c, axis=1)                  # tip to axis
    gap = rt - rho                                      # < 0: pressed in
    if not np.all(gap[i + 1:j] < 0):
        return None
    th = np.unwrap(np.arctan2(P[i:j + 1, 1] - c[1], P[i:j + 1, 0] - c[0]))
    dth = np.diff(th)
    if not (np.all(dth > 0) or np.all(dth < 0)):
        return None
    rb = np.linalg.norm(B - c, axis=2)                  # body to axis
    tol = 1e-9
    approach_clear = bool(np.all(rb[:i] >= rho - tol)) if i > 0 else True
    tip_deepest = bool(np.all(rb[i:j + 1].min(axis=1)
                              >= rb[i:j + 1, -1] - tol))
    after_clear = bool(np.all(rb[j + 1:] >= rho - tol)) \
        if j + 1 < len(P) else None
    return {"i": int(i), "j": int(j), "side": int(side), "c": c,
            "turn_rad": float(abs(th[-1] - th[0])),
            "sense": float(np.sign(th[-1] - th[0])),
            "depth_max": float(-gap[i:j + 1].min()), "gap": gap,
            "theta": th, "approach_clear": approach_clear,
            "tip_deepest": tip_deepest, "after_clear": after_clear}


def neighbour_gap(fg, c, n, k_lo, k_hi, m=14, other=None):
    """Least distance between the body points of one finger and of its
    neighbour 360/n round the axis. Same input level in both, or, with
    other=(lo, hi), every pairing of levels on an m x m grid."""
    B = fg["B"]
    Rn = rot2(2 * np.pi / n)
    ks = np.unique(np.linspace(k_lo, k_hi, m).round().astype(int))
    best = np.inf
    if other is None:
        pairs = [(k, k) for k in ks]
    else:
        ko = np.unique(np.linspace(other[0], other[1], m).round().astype(int))
        pairs = [(k, kk) for k in ks for kk in ko]
    for k, kk in pairs:
        X = B[k] - c
        Y = (B[kk] - c) @ Rn.T
        dmin = np.sqrt(((X[:, None, :] - Y[None, :, :]) ** 2).sum(-1)).min()
        best = min(best, dmin)
    return float(best)


def path_length(P):
    return float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())


def held_range(m_, depth_hold):
    """Ball angle (rad, from contact start) over which the push-out depth is
    at least depth_hold: (start, end) or None."""
    depth = -m_["gap"][m_["i"]:m_["j"] + 1]
    psi = np.abs(m_["theta"] - m_["theta"][0])
    ok = depth >= depth_hold
    if not ok.any():
        return None
    w = np.where(ok)[0]
    return float(psi[w[0]]), float(psi[w[-1]])


def main():
    os.makedirs(OUT, exist_ok=True)
    task = json.load(open(TASK))
    R = task["object"]["radius_m"]
    mass = task["object"]["mass_kg"]
    g = task["gravity_m_s2"]
    mu = task["mu"]
    alt = abs(np.radians(task["contact_ring"]["altitude_deg"]))
    lines = []
    say = lines.append
    say("Pneu-net fingers on a palm turning a ball: one stroke, and the "
        "return. SIMULATION ONLY, comparative.")
    say(f"script: tools/pneunet_palm_turn_ball_20nov.py   task file: "
        f"data/tasks/cup_turn.json")
    say(f"ball radius {R * 1e3:.1f} mm, mass {mass * 1e3:.0f} g, friction "
        f"{mu:g}, contact ring {np.degrees(alt):.2f} deg below the centre")
    res = {"task": {"radius_m": R, "mass_kg": mass, "mu": mu,
                    "ring_altitude_below_centre_deg": float(np.degrees(alt))},
           "grid_levels": GRID, "records": {}}

    for rid in RECORDS:
        fg = load_finger(rid)
        u, P = fg["u"], fg["P"]
        rc = fg["tip_r"]
        rho = (R + rc) * np.cos(alt)        # tip centre to axis, on the ring
        z0 = -(R + rc) * np.sin(alt)        # tip centre height
        plen = path_length(P)
        idx = np.unique(np.linspace(0, len(u) - 1, GRID).round().astype(int))
        cands, geo = [], []
        for a_ in range(len(idx)):
            for b_ in range(a_ + 1, len(idx)):
                for side in (+1, -1):
                    m_ = evaluate(fg, idx[a_], idx[b_], side, rho)
                    if m_ is None:
                        continue
                    geo.append(m_)
                    if not (m_["approach_clear"] and m_["tip_deepest"]):
                        continue
                    m_["nb"] = {n: neighbour_gap(fg, m_["c"], n, 0, m_["j"])
                                for n in N_FINGERS}
                    cands.append(m_)
        depth_rec = fg["F_blocked"] / fg["k"]
        say("")
        say(f"=== {rid} ===")
        say(f"  free path: {len(u) - 1} input levels to {u[-1]:.0f} Pa, "
            f"length along the path {plen * 1e3:.1f} mm, straight-line "
            f"stroke {fg['stroke'] * 1e3:.1f} mm, out of plane at most "
            f"{fg['out_of_plane_max_m'] * 1e3:.2f} mm")
        say(f"  tip radius {rc * 1e3:.2f} mm ({fg['tip_r_source']}); tip "
            f"centre on the ring is {rho * 1e3:.1f} mm from the axis, "
            f"{-z0 * 1e3:.1f} mm below the ball centre")
        say(f"  stand-in body fit: {'PASS' if fg['fit_pass'] else 'FAIL'}, "
            f"worst free-path error {fg['fit_free_err_max_m'] * 1e3:.2f} mm")
        say(f"  tip stiffness at zero input {fg['k']:.2f} N/m in plane "
            f"({fg['k_out_of_plane']:.0f} N/m out of plane); largest "
            f"blocked force in the record {fg['F_blocked']:.3f} N -> record "
            f"depth {depth_rec * 1e3:.1f} mm")
        say(f"  if the whole path lay on the ring: {plen * 1e3:.1f} mm / "
            f"{rho * 1e3:.1f} mm = {np.degrees(plen / rho):.1f} deg")
        say(f"  mounts kept before the neighbour check: {len(cands)} of "
            f"{len(idx) * (len(idx) - 1)} tried")
        def top(rows):
            return max(rows, key=lambda m_: m_["turn_rad"]) if rows else None
        t_all = top(geo)
        t_body = top([m_ for m_ in geo if m_["approach_clear"]])
        t_kept = top(cands)
        say(f"  widest turn by condition: tip path alone "
            f"{np.degrees(t_all['turn_rad']):.1f} deg (push-out "
            f"{t_all['depth_max'] * 1e3:.1f} mm); body clear of the ball on "
            f"the approach {np.degrees(t_body['turn_rad']):.1f} deg; and the "
            f"tip nearest the axis in contact "
            f"{np.degrees(t_kept['turn_rad']):.1f} deg")
        rr = {"widest_turn_deg": {
                  "tip_path_alone": float(np.degrees(t_all["turn_rad"])),
                  "tip_path_alone_push_out_m": t_all["depth_max"],
                  "body_clear_on_approach": float(np.degrees(t_body["turn_rad"])),
                  "all_single_finger_conditions": float(np.degrees(t_kept["turn_rad"]))},
              "tip_radius_m": rc, "ring_radius_tip_centre_m": float(rho),
              "tip_height_m": float(z0), "path_length_m": plen,
              "stroke_m": fg["stroke"], "k_N_per_m": fg["k"],
              "record_depth_m": depth_rec, "n_mounts_kept": len(cands),
              "whole_path_on_ring_deg": float(np.degrees(plen / rho)),
              "hands": {}}

        for n in N_FINGERS:
            ok = [m_ for m_ in cands if m_["nb"][n] >= 2 * rc]
            depth_hold = mass * g / (n * fg["k"] * (np.tan(alt) + mu))
            say(f"  -- {n} fingers: {len(ok)} mounts with neighbours clear; "
                f"hold depth {depth_hold * 1e3:.1f} mm")
            hand = {"n_mounts": len(ok), "hold_depth_m": float(depth_hold),
                    "curve": []}
            if not ok:
                say("     no mount passes: this hand cannot make the stroke")
                rr["hands"][str(n)] = hand
                continue
            # turn against the largest depth allowed
            for dm in DEPTH_GRID_MM:
                sub = [m_ for m_ in ok if m_["depth_max"] <= dm * 1e-3]
                if sub:
                    b = max(sub, key=lambda m_: m_["turn_rad"])
                    hand["curve"].append(
                        {"depth_cap_mm": float(dm),
                         "turn_deg": float(np.degrees(b["turn_rad"]))})
            say("     turn against the largest push-out allowed: " + ", ".join(
                f"{c_['depth_cap_mm']:.0f} mm {c_['turn_deg']:.0f} deg"
                for c_ in hand["curve"] if c_["depth_cap_mm"] in
                (1, 2, 3, 5, 8, 12, 16, 20)))
            caps = {"record depth": depth_rec, "no cap": np.inf}
            hand["chosen"] = {}
            prev = None
            for name, cap in caps.items():
                sub = [m_ for m_ in ok if m_["depth_max"] <= cap]
                if sub and prev is max(sub, key=lambda m_: m_["turn_rad"]):
                    say(f"     {name}: the same mount")
                    hand["chosen"][name] = "same as record depth"
                    continue
                prev = max(sub, key=lambda m_: m_["turn_rad"]) if sub else None
                if not sub:
                    say(f"     {name}: no mount")
                    continue
                b = max(sub, key=lambda m_: m_["turn_rad"])
                i, j = b["i"], b["j"]
                arc = path_length(P[i:j + 1])
                hr = held_range(b, depth_hold)
                # what binds: the same search with one condition dropped
                limits = []
                if j == len(u) - 1:
                    limits.append("contact ends at the last input level")
                if i == 0:
                    limits.append("contact starts at rest")
                if np.isfinite(cap) and b["depth_max"] > 0.9 * cap:
                    limits.append("the depth cap")
                loose = [m_ for m_ in cands if m_["depth_max"] <= cap]
                bl = max(loose, key=lambda m_: m_["turn_rad"])
                if bl["turn_rad"] > b["turn_rad"] + 1e-9:
                    limits.append(
                        "neighbouring fingers' bodies (without that check "
                        f"{np.degrees(bl['turn_rad']):.1f} deg)")
                base = np.linalg.norm(fg["B"][0, 0] - b["c"])
                ch = {"contact_from_Pa": float(u[i]),
                      "contact_to_Pa": float(u[j]),
                      "level_i": i, "level_j": j, "side": b["side"],
                      "axis_in_finger_frame_m": b["c"].tolist(),
                      "turn_deg": float(np.degrees(b["turn_rad"])),
                      "sense": b["sense"],
                      "depth_max_m": b["depth_max"],
                      "tip_travel_in_contact_m": arc,
                      "share_of_path_in_contact": arc / plen,
                      "ring_arc_m": float(b["turn_rad"] * rho),
                      "neighbour_gap_m": b["nb"][n],
                      "base_to_axis_m": float(base),
                      "clear_after_contact_ends": b["after_clear"],
                      "held_ball_angle_deg":
                          None if hr is None else
                          [float(np.degrees(hr[0])), float(np.degrees(hr[1]))],
                      "limits": limits,
                      # return check: gap along the one path
                      "gap_m": b["gap"].tolist(),
                      "gap_min_between_crossings_m":
                          float(b["gap"][i + 1:j].min()) if j > i + 1 else 0.0,
                      "gap_max_between_crossings_m":
                          float(b["gap"][i + 1:j].max()) if j > i + 1 else 0.0,
                      "gap_max_before_m":
                          float(b["gap"][:i].max()) if i > 0 else None,
                      "gap_max_after_m":
                          float(b["gap"][j + 1:].max())
                          if j + 1 < len(u) else None}
                hand["chosen"][name] = ch
                say(f"     {name} ({'none' if not np.isfinite(cap) else f'{cap * 1e3:.1f} mm'}): "
                    f"one stroke turns the ball {ch['turn_deg']:.1f} deg")
                say(f"        contact from {u[i]:.0f} to {u[j]:.0f} Pa; tip "
                    f"travels {arc * 1e3:.1f} mm of its {plen * 1e3:.1f} mm "
                    f"path in contact ({100 * arc / plen:.0f}%); the ring "
                    f"moves {ch['ring_arc_m'] * 1e3:.1f} mm; deepest "
                    f"push-out {b['depth_max'] * 1e3:.1f} mm")
                say(f"        bases {base * 1e3:.1f} mm from the axis; "
                    f"neighbouring bodies at least {b['nb'][n] * 1e3:.1f} mm "
                    f"apart (need {2 * rc * 1e3:.1f}); limited by: "
                    f"{'; '.join(limits) if limits else 'nothing found'}")
                if hr is None:
                    say(f"        weight: never carried (needs "
                        f"{depth_hold * 1e3:.1f} mm push-out, has at most "
                        f"{b['depth_max'] * 1e3:.1f})")
                else:
                    say(f"        weight carried only between "
                        f"{np.degrees(hr[0]):.1f} and {np.degrees(hr[1]):.1f} "
                        f"deg of the stroke "
                        f"({np.degrees(hr[1] - hr[0]):.1f} deg)")
                # the return
                g_in = ch["gap_max_between_crossings_m"]
                say(f"        return: between the two crossings the tip's "
                    f"gap to the ball is {ch['gap_min_between_crossings_m'] * 1e3:.2f}"
                    f" to {g_in * 1e3:.2f} mm (negative = pressed in) at "
                    f"every level, whichever way the input moves; a return "
                    f"clear of the ball needs a gap above 0 at each of "
                    f"those levels: "
                    f"{'none exists' if g_in < 0 else 'EXISTS'}")
                ga, gb = ch["gap_max_after_m"], ch["gap_max_before_m"]
                say(f"        breaking contact: past the far crossing the "
                    f"tip is "
                    + ("at the last level, no room" if ga is None else
                       f"up to {ga * 1e3:.1f} mm clear"
                       + ("" if b["after_clear"] else
                          " (but the stand-in body is inside the ball there)"))
                    + "; before the near crossing "
                    + ("it starts at rest in contact" if gb is None else
                       f"up to {gb * 1e3:.1f} mm clear"))

            # relay: a second set of n fingers half a pitch round, taking
            # over when the first set leaves by its far crossing
            sub = [m_ for m_ in ok if m_["i"] > 0 and m_["after_clear"]
                   and m_["j"] < len(u) - 1]
            relay = {"n_mounts_that_enter_and_leave_clear": len(sub)}
            if sub:
                sub2 = []
                for m_ in sorted(sub, key=lambda m_: -m_["turn_rad"]):
                    gp = neighbour_gap(fg, m_["c"], 2 * n, 0, len(u) - 1,
                                       m=12, other=(0, len(u) - 1))
                    if gp >= 2 * rc:
                        m_["relay_gap"] = gp
                        sub2.append(m_)
                        break
                relay["found_interleaved"] = bool(sub2)
                if sub2:
                    b = sub2[0]
                    dh = mass * g / (n * fg["k"] * (np.tan(alt) + mu))
                    psi = np.abs(b["theta"] - b["theta"][0])
                    dep = -b["gap"][b["i"]:b["j"] + 1]
                    sup = dep / dh              # share of the weight one set carries
                    D = psi[-1]
                    hr = held_range(b, dh)
                    best_s = None
                    if hr is not None:
                        xs = np.linspace(0, 2 * D, 2001)
                        for s in np.linspace(D, 0, 401):
                            tot = (np.interp(xs, psi, sup, left=0, right=0)
                                   + np.interp(xs - s, psi, sup, left=0, right=0))
                            w = (xs >= hr[0]) & (xs <= s + hr[1])
                            if np.all(tot[w] >= 1.0):
                                best_s = float(s)
                                break
                    relay.update({
                        "turn_one_set_deg": float(np.degrees(D)),
                        "turn_two_sets_no_overlap_deg": float(np.degrees(2 * D)),
                        "bodies_gap_m": b["relay_gap"],
                        "held_one_set_deg": None if hr is None else
                            float(np.degrees(hr[1] - hr[0])),
                        "held_relay_deg": None if best_s is None else
                            float(np.degrees(best_s + hr[1] - hr[0])),
                        "contact_from_Pa": float(u[b["i"]]),
                        "contact_to_Pa": float(u[b["j"]])})
                    say(f"     relay, two sets of {n} ({2 * n} fingers): one "
                        f"set turns {np.degrees(D):.1f} deg and leaves by "
                        f"its far crossing, the second takes over: "
                        f"{np.degrees(2 * D):.1f} deg in one pass with the "
                        f"ball untouched at the hand-over; with the weight "
                        f"carried throughout: "
                        + ("not possible" if best_s is None else
                           f"{relay['held_relay_deg']:.1f} deg")
                        + ". To go again every finger must come back "
                          "through its own contact stretch, which turns the "
                          "ball back.")
                else:
                    say(f"     relay, two sets of {n}: no mount found whose "
                        f"{2 * n} bodies stay clear of each other at all "
                        f"pairs of input levels ({len(sub)} tried)")
            else:
                say(f"     relay, two sets of {n}: no mount both enters and "
                    f"leaves contact clear of the ball")
            hand["relay"] = relay
            rr["hands"][str(n)] = hand
        res["records"][rid] = rr

    with open(os.path.join(OUT, "results.json"), "w") as f:
        json.dump(res, f, indent=1)
    with open(os.path.join(OUT, "report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
