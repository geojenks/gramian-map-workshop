"""A longer stretch of a recorded learned rotation, beside four one-input
pneu-net fingers fitted to the same contacts: the recorded hand lifts each
finger off, brings it back clear of the object and touches again; a one-input
pneu-net has a single path for its tip, so its only way back is the way it
came. GEOMETRY ONLY, no simulation. Comparative, not predictive.

EVERYTHING BELOW WAS FIXED BEFORE THE FIRST RUN OF THIS TOOL:
  - Recording and window: data/rl_logs/hora_seed42_30s*.npz, the window of
    the 21 Nov reach check (found by its own code), all four fingertips.
    Touching, lift-offs and landings are that reach check's (every fourth
    sample; its merging of short gaps).
  - Finger: the one-input pneu-net record chosen by
    tools/pneunet_upright_arrangement_replicates_rl_stroke_21nov.py (read
    from its results file), as that tool's ten-link stand-in. Only its FREE
    shapes are used: no squeeze, no weight, no contact forces.
  - One finger per recorded fingertip, upright on a palm, placed with that
    tool's parametrisation and limits (bending plane beside the cylinder,
    leaning out by 0 to BETA_MAX, leaning within the plane by up to
    ALPHA_MAX). Fitted so that the tip's single path passes as near as it
    can to the logged contact on that fingertip's touching samples (the tip
    centre one contact radius outside the logged point), with the links
    below the tip kept out of the logged cylinder on those samples.
    Starts and iterations as that tool's fit.
  - Drive: on a touching sample, the input level whose tip is nearest the
    logged contact. Between a lift-off and the next landing the input goes
    evenly from the level at lift-off to the level at landing: with one
    input there is no other way from one to the other. Before the first and
    after the last touching sample of the window the level is held.
  - Measures: per finger, distance from the tip's surface point nearest the
    logged contact to that contact (median, largest) on touching samples.
    On every return, the clearance from the logged cylinder of (a) the
    recorded fingertip and (b) the pneu-net's tip. (a) is the distance of a
    point fixed in the recorded hand's last link of that finger: the mean,
    in that link, of its logged contact points in the window. (b) is the
    tip centre's distance less the contact radius. A pneu-net is counted as
    "going back along the surface" on a return sample where (b) is under
    2.06 mm, the reach check's limit for a return.
  - No turn is claimed for the pneu-nets: nothing here moves the object.
    The cylinder in both pictures is the logged one.
  - Clip: 1920 x 1080, two columns playing together, one frame per logged
    sample at 30 frames a second.

ADDED AFTER THE FIRST NUMBERS-ONLY RUN, to explain it; no rule above was
changed: the report also gives how far the logged cylinder wanders over the
window, the spread of the tip's clearance while touching, the largest jump
in input between two touching samples next to each other, and which
mounting limits each fit ends on. The clip draws each tip's single path as
a thin line. The wording of the titles was changed from "placed on" to
"fitted to" because the fit turned out poor.

NOT checked: forces, friction, whether the object would be held or dragged
back, speed, the palm, the pneu-net bodies against each other beyond the
gap reported.

Run from the repository root (needs mujoco and ffmpeg):
    python tools/one_input_pneunets_cannot_reset_beside_recording_21nov.py \
        --wave-manip <Wave_manip folder> --gramian-map <gramian_map folder>
Writes outputs/one_input_pneunets_cannot_reset_beside_recording_21nov/ and
refuses to overwrite; use --tag for a rerun.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
import mujoco as mj                                                             # noqa: E402
import pneunet_arrangement_replicates_rl_stroke_20nov as A                      # noqa: E402
import pneunet_upright_arrangement_replicates_rl_stroke_21nov as U              # noqa: E402
import two_pneunet_fingers_follow_rl_gait_reach_check_21nov as RC               # noqa: E402
import two_pneunet_fingers_human_like_mounting_reach_check_21nov as HM          # noqa: E402

G, N = A.G, A.N
OUT = os.path.join(ROOT, "outputs", "one_input_pneunets_cannot_reset_beside_recording_21nov")
UPRIGHT_RESULTS = os.path.join(U.OUT, "arrangement_results.json")
NG = 121
NEAR_MM = RC.MIN_CLEAR_MM
CW, CH, HEAD, FOOT = 960, 850, 120, 110
FPS = A.FPS
HOLD_FRAMES = 15          # the first and last frames are held this long
COL = A.COL
TIP_LINKS = ["link_3", "link_7", "link_11", "link_15"]


def plane_tables(ft):
    J, _ = A.chain_tables(ft, NG)
    return J, np.linspace(0, ft["u"][-1], NG)


def place(x, mirror, fr, J):
    """Joints (levels, N + 1, 3) in the world, the last being the tip centre."""
    base, X, Y, o, tau, w = U.axes(np.concatenate([x, [0, 0]]), mirror, fr)
    return fr["c"] + base + J[..., 0, None] * X + J[..., 1, None] * Y, (base, X, Y)


def fit_finger(F, fr, ft, J, R, half):
    on = np.where(F.on)[0]
    g = F.P[on] + ft["rc"] * F.n[on]                     # where the tip's centre should be
    c, u = F.c[on], F.u[on]
    rows = np.arange(len(on))

    def parts(x, mirror):
        W, _ = place(x, mirror, fr, J)
        D = np.linalg.norm(g[:, None] - W[None, :, -1], axis=-1)
        k = D.argmin(1)
        low = W[k][:, :N]                                # joints below the tip, at the level used
        cut = np.maximum(0, ft["rc"] + U.CLEAR - RC.cyl_sdf(low, c[:, None], u[:, None], R, half))
        return D[rows, k], k, cut

    best = None
    for mirror in (1.0, -1.0):
        def cost(x):
            d, _, cut = parts(x, mirror)
            lim = max(0, -x[2]) + max(0, x[2] - U.BETA_MAX) + max(0, abs(x[5]) - U.ALPHA_MAX)
            return (d * d).mean() + 10 * (cut * cut).mean() + lim * lim
        gm = g.mean(0) - fr["c"]
        phi0 = float(np.arctan2(gm @ fr["e2"], gm @ fr["e1"]))
        for alpha0 in (-0.2, 0.0, 0.2):
            for lv in (0.3, 0.6):
                x0 = np.array([phi0, (R + ft["rc"]) / 0.05, 0.05, 0, 0, alpha0])
                W, (_, _, _) = place(x0, mirror, fr, J)
                _, _, _, _, tau, w = U.axes(np.concatenate([x0, [0, 0]]), mirror, fr)
                dm = g.mean(0) - W[int(lv * (NG - 1)), -1]
                x0[3], x0[4] = dm @ tau / 0.05, dm @ w / 0.05
                x, fv = A.nelder_mead(cost, x0, 0.1, 700)
                x, fv = A.nelder_mead(cost, x, 0.02, 400)
                if best is None or fv < best[1]:
                    best = (x, fv, mirror)
    x, fv, mirror = best
    d, k, cut = parts(x, mirror)
    W, (base, X, Y) = place(x, mirror, fr, J)
    back = np.cos(x[0]) * fr["e1"] + np.sin(x[0]) * fr["e2"]
    return {"finger": F.name, "x": x.tolist(), "mirror": mirror, "base": (fr["c"] + base).tolist(), "X": X.tolist(),
            "Y": Y.tolist(), "back": back.tolist(), "base_height_m": float(base @ fr["a"]),
            "plane_touches_at_deg": float(np.degrees(x[0])), "plane_leans_out_deg": float(np.degrees(x[2])),
            "lean_in_plane_deg": float(np.degrees(x[5])), "cost": float(fv),
            "links_below_tip_cut_max_mm": float(cut.max() * 1e3)}, W, on, k, d


def patch_point(F, body, names, j_idx):
    """The point fixed in the recorded hand's last link of this finger that its logged contacts average to."""
    on = np.where(F.on)[0]
    best = None
    for nm in TIP_LINKS:
        b = body[j_idx[on], names.index(nm)].astype(float)
        loc = A.rot(A.inv(b[:, 3:7]), F.P[on] - b[:, :3])
        sc = float(np.linalg.norm(loc - loc.mean(0), axis=1).mean())
        if best is None or sc < best[1]:
            best = (nm, sc, loc.mean(0))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", required=True)
    ap.add_argument("--gramian-map", required=True)
    ap.add_argument("--azimuth", type=float, default=None)
    ap.add_argument("--elevation", type=float, default=U.ELEVATION, help="camera elevation, deg")
    ap.add_argument("--view-gap", type=int, default=0, help="look through the widest gap between fingers (0), the next widest (1), ...")
    ap.add_argument("--tag", default="")
    ap.add_argument("--no-film", action="store_true")
    ap.add_argument("--stills-only", action="store_true", help="render the stills but no clip")
    args = ap.parse_args()
    base = os.path.join(OUT, "one_input_pneunets_beside_recording" + args.tag)
    f_rep, f_json = base + "_report.txt", base + "_measures.json"
    for f in (f_rep, f_json, base + ".mp4", os.path.join(OUT, f"model_one_input_pneunets{args.tag}_driven.xml")):
        if os.path.exists(f):
            raise SystemExit(f"{f} exists; use --tag")
    os.makedirs(OUT, exist_ok=True)
    W0 = HM.setup(args.wave_manip, args.gramian_map)
    L, R, half, s0, s1, idx = W0["L"], W0["R"], W0["half"], W0["s0"], W0["s1"], W0["idx"]
    rid = json.load(open(UPRIGHT_RESULTS))["finger_record"]
    ft = G.finger_table(rid)
    J, ug = plane_tables(ft)
    a = L["axis"][idx].mean(0)
    a /= np.linalg.norm(a)
    e1 = np.array([1.0, 0, 0]) - a[0] * a
    e1 /= np.linalg.norm(e1)
    fr = {"c": L["op"][idx].mean(0), "a": a, "e1": e1, "e2": np.cross(a, e1)}
    Fz = np.load(os.path.join(A.LOGS, "hora_seed42_30s_forces.npz"), allow_pickle=True)
    body, bnames = Fz["body"], [str(v) for v in Fz["body_names"]]
    t = L["t"]
    turn = float(np.degrees(L["turn"][s1] - L["turn"][s0]))
    cr = np.ptp(L["op"][idx], axis=0) * 1e3
    tilt = float(np.degrees(np.arccos(np.clip(L["axis"][idx] @ a, -1, 1))).max())
    rep = ["A recorded learned rotation beside four one-input pneu-net fingers fitted to the same contacts",
           "GEOMETRY ONLY, no simulation. Comparative, not predictive.", "",
           f"Recording: data/rl_logs/hora_seed42_30s*.npz. Cylinder radius {R * 1e3:.1f} mm, half length {half * 1e3:.1f} mm. "
           f"Window: log {t[s0]:.2f} to {t[s1]:.2f} s ({s1 - s0 + 1} samples; touching judged on every {RC.STEP}th, "
           f"{len(idx)} samples). Recorded turn of the cylinder about its own axis over the window: {turn:+.1f} deg.",
           f"Finger: one-input pneu-net record {rid}, length {ft['L'] * 1e3:.0f} mm, contact radius {ft['rc'] * 1e3:.2f} mm, "
           f"input 0 to {ft['u'][-1]:.0f} (simulation units), free shapes only.",
           f"A pneu-net counts as going back along the surface on a return sample where its tip is under {NEAR_MM} mm from the "
           "logged cylinder (the 21 Nov reach check's limit for a return).",
           f"Over the window the logged cylinder's centre ranges over {cr[0]:.1f} x {cr[1]:.1f} x {cr[2]:.1f} mm (x, y, z) and its axis "
           f"tilts up to {tilt:.1f} deg from its mean. The pneu-nets stand on a palm that does not move.", ""]
    fits, fingers, results = [], [], {"window_s": [float(t[s0]), float(t[s1])], "recorded_turn_deg": turn,
                                      "cylinder_centre_range_mm": cr.tolist(), "cylinder_axis_tilt_from_mean_max_deg": tilt,
                                      "finger_record": rid, "near_limit_mm": NEAR_MM, "fingers": {}}
    for j in range(4):
        F = RC.Finger(L, idx, j, W0["tab"], R, half)
        F.name = L["names"][j]
        fit, Wj, on, k, d = fit_finger(F, fr, ft, J, R, half)
        lev = np.interp(np.arange(len(idx)), on, k.astype(float))        # level (grid units) at every judged sample
        tipc = np.stack([np.array([np.interp(v, np.arange(NG), Wj[:, -1, i]) for i in range(3)]) for v in lev])
        clear = (RC.cyl_sdf(tipc, F.c, F.u, R, half) - ft["rc"]) * 1e3
        link, scatter, loc = patch_point(F, body, bnames, idx)
        b = body[idx, bnames.index(link)].astype(float)
        rec_clear = RC.cyl_sdf(b[:, :3] + A.rot(b[:, 3:7], loc), F.c, F.u, R, half) * 1e3
        err = d * 1e3                                    # tip centre against its place on the logged contact
        m = {"contact_error_median_mm": float(np.median(err)), "contact_error_max_mm": float(err.max()),
             "touching_samples": int(len(on)), "input_range": [float(ug[k.min()]), float(ug[k.max()])],
             "recorded_fingertip_point_in": link, "recorded_contacts_scatter_about_that_point_mm": scatter * 1e3,
             "recorded_fingertip_clearance_while_touching_median_mm": float(np.median(rec_clear[on])),
             "pneunet_tip_clearance_while_touching_median_mm": float(np.median(clear[on])),
             "pneunet_tip_clearance_while_touching_least_mm": float(clear[on].min()),
             "pneunet_tip_clearance_while_touching_largest_mm": float(clear[on].max()),
             "largest_input_jump_between_touching_samples_next_to_each_other": float(np.abs(np.diff(ug[k]))[np.diff(on) == 1].max()),
             "fit_ends_on_outward_lean_limit": bool(fit["x"][2] > U.BETA_MAX - 1e-3 or fit["x"][2] < 1e-3),
             "fit_ends_on_in_plane_lean_limit": bool(abs(fit["x"][5]) > U.ALPHA_MAX - 1e-2),
             "links_below_tip_cut_max_mm": fit["links_below_tip_cut_max_mm"], "returns": []}
        rep += [f"{F.name}: plane touches the cylinder at {fit['plane_touches_at_deg']:+.1f} deg round it, leans outward "
                f"{fit['plane_leans_out_deg']:.1f} deg, finger leans {fit['lean_in_plane_deg']:+.1f} deg within the plane; input used "
                f"{m['input_range'][0]:.0f} to {m['input_range'][1]:.0f}.",
                f"  while touching ({len(on)} samples): tip centre off its place on the logged contact by median "
                f"{m['contact_error_median_mm']:.2f} mm, largest {m['contact_error_max_mm']:.2f} mm; links below the tip cut into "
                f"the cylinder by at most {m['links_below_tip_cut_max_mm']:.2f} mm; tip clearance median "
                f"{m['pneunet_tip_clearance_while_touching_median_mm']:+.2f} mm (least "
                f"{m['pneunet_tip_clearance_while_touching_least_mm']:+.2f}, largest {m['pneunet_tip_clearance_while_touching_largest_mm']:+.2f}).",
                f"  largest jump in input between two touching samples next to each other: "
                f"{m['largest_input_jump_between_touching_samples_next_to_each_other']:.0f} of {ft['u'][-1]:.0f}; fit ends on the outward-lean "
                f"limit: {'yes' if m['fit_ends_on_outward_lean_limit'] else 'no'}; on the in-plane lean limit: "
                f"{'yes' if m['fit_ends_on_in_plane_lean_limit'] else 'no'}.",
                f"  recorded fingertip point: fixed in {link}; the logged contacts lie a mean {scatter * 1e3:.1f} mm from it; its "
                f"clearance while touching, median {m['recorded_fingertip_clearance_while_touching_median_mm']:+.2f} mm."]
        back = np.zeros(len(idx), bool)
        for n, (touching, sa, sb) in enumerate(F.spans):
            if touching:
                continue
            whole = 0 < n < len(F.spans) - 1
            sl = slice(sa, sb + 1)
            near = clear[sl] < NEAR_MM
            back[sl] = near
            travel = float(np.linalg.norm(np.diff(tipc[max(sa - 1, 0):min(sb + 2, len(idx))], axis=0), axis=1).sum() * 1e3)
            r = {"from_s": float(F.t[sa]), "to_s": float(F.t[sb]), "complete": bool(whole),
                 "recorded_clearance_max_mm": float(rec_clear[sl].max()), "recorded_clearance_median_mm": float(np.median(rec_clear[sl])),
                 "pneunet_clearance_max_mm": float(clear[sl].max()), "pneunet_clearance_median_mm": float(np.median(clear[sl])),
                 "pneunet_clearance_min_mm": float(clear[sl].min()), "share_of_return_under_limit": float(near.mean()),
                 "pneunet_tip_travel_mm": travel,
                 "input_from_to": [float(np.interp(lev[max(sa - 1, 0)], np.arange(NG), ug)),
                                   float(np.interp(lev[min(sb + 1, len(idx) - 1)], np.arange(NG), ug))]}
            m["returns"].append(r)
            rep.append(f"  return {r['from_s']:.2f} to {r['to_s']:.2f} s{'' if whole else ' (cut by the window)'}: recorded fingertip "
                       f"clears the cylinder by up to {r['recorded_clearance_max_mm']:.1f} mm (median {r['recorded_clearance_median_mm']:.1f}); "
                       f"pneu-net tip by up to {r['pneunet_clearance_max_mm']:.1f} mm (median {r['pneunet_clearance_median_mm']:.1f}, "
                       f"least {r['pneunet_clearance_min_mm']:+.1f}); under {NEAR_MM} mm for {100 * r['share_of_return_under_limit']:.0f}% "
                       f"of the return; its tip travels {travel:.1f} mm, input {r['input_from_to'][0]:.0f} to {r['input_from_to'][1]:.0f}.")
        whole = [r for r in m["returns"] if r["complete"]]
        m["complete_returns"] = len(whole)
        m["complete_returns_mostly_under_limit"] = int(sum(r["share_of_return_under_limit"] > 0.5 for r in whole))
        rep.append(f"  complete returns: {len(whole)}; of those, the pneu-net tip is under the limit for more than half the return in "
                   f"{m['complete_returns_mostly_under_limit']}.")
        results["fingers"][F.name] = m
        fits.append(fit)
        fingers.append({"F": F, "W": Wj, "lev": lev, "touch": F.touch.copy(), "back": back, "clear": clear})
    pts = [f["W"][np.round(f["lev"]).astype(int)] for f in fingers]      # joints of each finger at every judged sample
    gap = min(np.linalg.norm(pts[p][:, :, None] - pts[q][:, None], axis=-1).min() - 2 * ft["rc"]
              for p in range(4) for q in range(p + 1, 4))
    results["least_gap_between_pneunet_bodies_mm"] = float(gap * 1e3)
    results["fits"] = fits
    whole = [(nm, r) for nm, m in results["fingers"].items() for r in m["returns"] if r["complete"]]
    n_under = sum(r["share_of_return_under_limit"] > 0.5 for _, r in whole)
    med = [m["contact_error_median_mm"] for m in results["fingers"].values()]
    results["complete_returns"], results["complete_returns_mostly_under_limit"] = len(whole), int(n_under)
    rep[7:7] = ["RESULT: the fit is poor and the measure does not show the pneu-net tips staying on the surface on the way back. "
                f"While touching, the tip centres are a median {min(med):.1f} to {max(med):.1f} mm off their place on the logged contacts. "
                f"Of {len(whole)} complete returns, the pneu-net tip is under {NEAR_MM} mm from the cylinder for more than half the return in "
                f"{n_under}. The recorded fingertip point itself reads "
                + ", ".join(f"{m['recorded_fingertip_clearance_while_touching_median_mm']:+.1f}" for m in results["fingers"].values())
                + " mm (median) while touching, which is the zero error of that measure.", ""]
    rep += ["", f"Least gap between two pneu-net bodies at the same moment: {gap * 1e3:+.1f} mm"
            + (" (they overlap; not corrected)." if gap < 0 else "."),
            "", "Simplified: free shapes only (no squeeze, weight or contact force); the cylinder is the logged one in both pictures, "
            "so nothing here says what the pneu-nets would do to it; the finger is drawn as a ten-link stand-in; the palm plate and "
            "posts are drawn only.",
            "No turn is claimed for the pneu-nets."]

    def save():
        with open(f_rep, "w", encoding="utf-8") as fh:
            fh.write("\n".join(rep) + "\n")
        with open(f_json, "w", encoding="utf-8") as fh:
            json.dump(results, fh, indent=1)
    save()
    print("\n".join(rep), flush=True)
    if args.no_film:
        return
    files = film(args, L, R, half, s0, s1, idx, fr, ft, ug, fits, fingers, base, turn, rep, (min(med), max(med), cr.max()))
    rep += ["", "Files:"] + ["  " + os.path.relpath(f, ROOT) for f in files]
    save()
    print("\n".join(rep[-len(files) - 3:]))


def film(args, L, R, half, s0, s1, idx, fr, ft, ug, fits, fingers, base, turn, rep, fitnote):
    from PIL import Image, ImageDraw, ImageFont
    try:
        font, small = ImageFont.truetype("arial.ttf", 30), ImageFont.truetype("arial.ttf", 22)
        bold = ImageFont.truetype("arialbd.ttf", 22)
    except OSError:
        font = small = bold = ImageFont.load_default()
    U.OUT, U.CW, U.CH = OUT, CW, CH
    B = U.build(L, {"s": s0}, fr, fits, ft, R, half, "one_input_pneunets" + args.tag, "driven")
    m = B["m"]
    az = np.sort(np.mod([f["plane_touches_at_deg"] for f in fits], 360))
    gaps = np.diff(np.concatenate([az, [az[0] + 360]]))
    i = int(np.argsort(-gaps)[args.view_gap])
    mid = np.radians(az[i] + gaps[i] / 2)
    v = np.cos(mid) * fr["e1"] + np.sin(mid) * fr["e2"]
    azimuth = args.azimuth if args.azimuth is not None else float(np.degrees(np.arctan2(-v[1], -v[0])))
    cam = U.camera(fr, B["h_plate"], azimuth)
    cam.elevation = args.elevation
    S = A.recorded_scene(args.wave_manip, L, R, half)
    S["m"].vis.global_.offwidth, S["m"].vis.global_.offheight = CW, CH
    models = [S["m"], m]
    datas = [mj.MjData(x) for x in models]
    rens = [mj.Renderer(x, CH, CW) for x in models]
    Z, t = L["Z"], L["t"]
    frames = np.arange(s0, s1 + 1)
    pos = (frames - s0) / RC.STEP                           # place among the judged samples
    near = np.clip(np.round(pos).astype(int), 0, len(idx) - 1)
    lv = [np.interp(pos, np.arange(len(idx)), f["lev"]) for f in fingers]
    us = [np.interp(x, np.arange(NG), ug) for x in lv]
    touch = [f["touch"][near] for f in fingers]
    back = [f["back"][near] for f in fingers]
    slow = FPS * (t[s1] - t[s0]) / (len(frames) - 1)
    slow = 1 / slow
    Wt, Ht = 2 * CW, HEAD + CH + FOOT
    ff = shutil.which("ffmpeg") or r"C:\Program Files\ffmpeg\bin\ffmpeg.exe"
    proc = None if args.stills_only else subprocess.Popen([ff, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{Wt}x{Ht}",
                             "-r", str(FPS), "-i", "-", "-pix_fmt", "yuv420p", "-vcodec", "libx264", base + ".mp4"],
                            stdin=subprocess.PIPE)
    files = [] if args.stills_only else [base + ".mp4"]
    titles = ["Recorded: learned policy, four-finger hand", "Four one-input pneu-nets fitted to the same contacts"]
    foot = [f"Both play together, {slow:.1f} times slower than recorded. The recorded hand lifts a finger off, brings it back clear "
            "of the cylinder and touches again, so the turn adds up.",
            f"A one-input pneu-net has one path for its tip (thin line): its only way back is along that same path. Red ball: a tip going "
            f"back within {NEAR_MM} mm of the cylinder.",
            f"Right: GEOMETRY ONLY, no forces, no simulation; the cylinder is the recorded one. The fit is poor (tips a median "
            f"{fitnote[0]:.0f} to {fitnote[1]:.0f} mm off the contacts; the cylinder wanders {fitnote[2]:.0f} mm). Comparative only."]
    n_back = np.sum(back, axis=0)
    off = np.sum(~np.array(touch), axis=0)
    marks = {0: "_start", len(frames) - 1: "_end"}
    most = np.where(off == off.max())[0]
    marks.setdefault(int(most[len(most) // 2]), "_most_recorded_fingers_off_the_cylinder")
    if n_back.max() > 0:
        marks.setdefault(int(np.argmax(n_back)), "_most_pneunet_tips_going_back_near_the_cylinder")
    paths = [f["W"][::4, -1] for f in fingers]
    q0 = B["q0"]
    drawn_off = 0.0

    def shot(i, extra):
        ren = rens[i]
        ren.update_scene(datas[i], cam)
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

    def now(k):
        return {j: L["P"][k, j] for j in range(4) if L["lam"][k, j] > A.TOUCH_N}

    for n, k in enumerate(frames):
        if args.stills_only and n not in marks:
            continue
        A.pose_recorded(S, datas[0], Z, k)

        def ex0(scene):
            turn_line(scene, Z["obj"][k][:3].astype(float), Z["obj"][k][3:7].astype(float))
            for j, p in now(k).items():
                A.add_sphere(scene, p, 0.0035, (*COL[L["names"][j]], 1))
        im0 = shot(0, ex0)
        d = datas[1]
        d.mocap_pos[B["omocap"]] = L["op"][k]
        d.mocap_quat[B["omocap"]] = A.qmul(L["q"][k], A.inv(q0))[[3, 0, 1, 2]]
        for fg in B["fingers"]:
            d.mocap_pos[fg["mocap"]] = fg["home"]
        A.set_inputs(B, m, [u[n] for u in us], d)
        mj.mj_forward(m, d)
        tips = []
        for jj, fg in enumerate(B["fingers"]):
            bid = m.body(f"f{fg['k']}_l{N - 1}").id
            tips.append(d.xpos[bid] + d.xmat[bid].reshape(3, 3) @ np.array([ft["l"], 0, 0]))
            want = np.array([np.interp(lv[jj][n], np.arange(NG), fingers[jj]["W"][:, -1, i]) for i in range(3)])
            drawn_off = max(drawn_off, float(np.linalg.norm(tips[-1] - want)))

        def ex1(scene):
            turn_line(scene, L["op"][k], L["q"][k])
            for j, p in now(k).items():
                A.add_sphere(scene, p, 0.0035, (*COL[L["names"][j]], 1))
            for jj in range(4):
                for pa, pb in zip(paths[jj][:-1], paths[jj][1:]):
                    U.connector(scene, mj.mjtGeom.mjGEOM_CAPSULE, 0.0006, pa, pb, (*COL[L["names"][jj]], 1))
                if back[jj][n]:
                    A.add_sphere(scene, tips[jj], 1.7 * ft["rc"], (1, 0, 0, 0.35))
        im1 = shot(1, ex1)
        im = Image.new("RGB", (Wt, Ht), (255, 255, 255))
        im.paste(Image.fromarray(np.hstack([im0, im1])), (0, HEAD))
        dr = ImageDraw.Draw(im)
        dr.line([(CW, HEAD), (CW, HEAD + CH)], fill=(200, 200, 200), width=1)
        for i in range(2):
            dr.text((i * CW + 20, 12), titles[i], fill=(0, 0, 0), font=font)
        dr.text((20, 56), f"log time {t[k]:.2f} s   cylinder turned {np.degrees(L['turn'][k] - L['turn'][s0]):+.1f} deg", fill=(0, 0, 0), font=small)
        dr.text((20, 86), "dots: logged contacts now; black line: fixed to the cylinder", fill=(0, 0, 0), font=small)
        for jj in range(4):
            nm = L["names"][jj]
            c = tuple(int(255 * v) for v in COL[nm])
            state = ("nearest it gets to the contact" if touch[jj][n] else
                     "going back, NEAR THE CYLINDER" if back[jj][n] else "going back along its path")
            x, y = CW + 20 + 470 * (jj % 2), 56 + 30 * (jj // 2)
            dr.text((x, y), nm + ":", fill=c, font=bold)
            x += dr.textlength(nm + ": ", font=bold)
            dr.text((x, y), state, fill=(200, 0, 0) if back[jj][n] else (0, 0, 0), font=bold if back[jj][n] else small)
        for i, s in enumerate(foot):
            dr.text((20, HEAD + CH + 8 + 32 * i), s, fill=(0, 0, 0), font=small)
        buf = np.asarray(im).tobytes()
        for _ in range((HOLD_FRAMES if n in (0, len(frames) - 1) else 1) if proc else 0):
            proc.stdin.write(buf)
        if n in marks:
            im.save(base + marks[n] + ".png")
            files.append(base + marks[n] + ".png")
    for ren in rens:
        ren.close()
    if proc:
        proc.stdin.close()
        proc.wait()
    rep += ["", f"{'Stills only' if args.stills_only else 'Clip'}: {Wt} x {Ht}, {FPS} frames a second, {(len(frames) + 2 * HOLD_FRAMES - 2) / FPS:.1f} s, {slow:.1f} times slower "
            f"than recorded; camera azimuth {azimuth:.0f} deg, elevation {args.elevation:.0f} deg, the same in both columns.",
            f"The drawn tip is within {drawn_off * 1e3:.2f} mm of the computed one in every frame.",
            f"Largest number of pneu-net tips going back within the limit of the cylinder at once: {int(n_back.max())}; share of frames with at least "
            f"one: {100 * float((n_back > 0).mean()):.0f}%."]
    return files


if __name__ == "__main__":
    main()
