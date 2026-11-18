"""Digitise Chandler et al. 2020 (Front. Robot. AI 7:119) Fig 7C: single-chamber tip trajectories, alpha = 0.

Fig 7C is a MATLAB 3D view of the tip position (X, Y, Z in mm) for each chamber driven alone, with an x-y polar inset.
Neither view carries the injected volume. This script:

1. Calibrates the 3D view. The tick marks on the three box edges are found by image processing; the view is treated as
   an orthographic (affine) projection u = ax*X + ay*Y + cu, v = bx*X + by*Y + bz*Z + cv (vertical edges are vertical in
   the image, so Z does not move u). The box limits are measured, not assumed.
2. Calibrates the polar inset: ring centre and radius from a circle fit to the grey rings; the labelled 25 mm ring sets
   the scale; 0 deg is up (+x) and angles run counter-clockwise (+y to the left), as the angle labels show.
3. Finds the marker centres of each chamber's series in the 3D view (distance-transform peaks of the colour mask) and
   solves each marker in 3D: u fixes a line in the x-y plane; that line is intersected with the chamber's inset trace
   to give (x, y); v then gives z. Markers where the inset band is ambiguous or empty are dropped and counted.
4. Gives each marker an estimated volume. The bend angle is computed from the tip position with the constant-curvature
   relation rho/h = (1 - cos t)/(t - sin t) (independent of arc length), then the paper's own Gaussian fit of bend
   angle against up-sweep volume (Table 3, Eq 11) is inverted. The paper reports this constant-curvature fit to its
   tip data within 2 mm RMS. Neighbouring resolved markers are then checked for a constant volume step (Fig 7D
   samples every 0.2 ml); see PROVENANCE.md for the outcome.

Outputs (beside this script): chandler2020_fig7C_a0_tip_C{1,2,3}.csv, chandler2020_fig7C_a0_calibration.csv,
verify_fig7C_a0_*.png. Run with the Gramain_extraction venv python:
    python -I digitize_chandler2020_fig7.py
"""
import csv
import hashlib
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage as ndi
from scipy import optimize

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "source", "chandler2020_fig7.jpeg")
TABLES = os.path.join(HERE, "chandler2020_tables_transcribed.csv")

# --- image regions (full-resolution pixels of chandler2020_fig7.jpeg), found by inspection of gridded crops ---
BOX = (440, 2350, 1600, 3330)            # 3D view interior searched for markers
INSET_REGION = (1500, 2000, 2080, 2620)  # polar inset
INSET_GUESS = (1792.0, 2305.0)           # rough inset centre, refined by circle fit
INSET_EXCLUDE_R = 300                    # inset disc removed from the 3D marker search
EDGE_GUESS = {"Y": [0.2475, 3213.0], "X": [-0.4957, 4157.0]}  # rough bottom-edge lines y = m x + c
LEFT_EDGE_X = 434.6                      # left vertical box edge (X=-40, Y=+40), checked below
Z_TICK_SEARCH = (2380, 3320)

COL = {  # colour masks: C1 red circles, C2 green triangles, C3 blue squares
    1: lambda R, G, B: (R > 150) & (G < 110) & (B < 110),
    2: lambda R, G, B: (G > 150) & (R < 140) & (B < 120),
    3: lambda R, G, B: (B > 140) & (R < 110) & (G < 110),
}


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


def read_tables():
    rows = []
    with open(TABLES, encoding="utf-8") as f:
        lines = [ln for ln in f if not ln.startswith("#")]
    for r in csv.DictReader(lines):
        rows.append(r)
    return rows


# ---------------------------------------------------------------- 3D view calibration
def fit_edge(dark, p, x0, x1, w=8):
    for _ in range(4):
        xs, ys = [], []
        for x in range(x0, x1):
            yc = np.polyval(p, x)
            lo = int(yc - w)
            r = np.where(dark[lo:lo + 2 * w + 1, x])[0]
            if 2 <= len(r) <= 10:
                xs.append(x)
                ys.append(lo + r.mean())
        p = np.polyfit(xs, ys, 1)
        res = np.array(ys) - np.polyval(p, xs)
        w = max(4, int(3 * res.std() + 2))
    return p, float(res.std())


def edge_ticks(dark, p, x0, x1, direction):
    d = np.array(direction, float)
    d /= np.hypot(*d)
    hits = []
    for x in np.arange(x0, x1, 0.5):
        y = np.polyval(p, x)
        c = sum(dark[int(round(y + d[1] * s)), int(round(x + d[0] * s))] for s in range(6, 14))
        if c >= 6:
            hits.append(x)
    clusters, cur = [], []
    for h in hits:
        if cur and h - cur[-1] > 3:
            clusters.append(float(np.mean(cur)))
            cur = []
        cur.append(h)
    if cur:
        clusters.append(float(np.mean(cur)))
    return clusters


def z_ticks(dark, x_edge):
    """Ticks on the left vertical edge stick out to the left of it."""
    xe = int(round(x_edge))
    ys = [y for y in range(*Z_TICK_SEARCH) if dark[y, xe - 13:xe - 5].sum() >= 6]
    clusters, cur = [], []
    for y in ys:
        if cur and y - cur[-1] > 2:
            clusters.append(float(np.mean(cur)))
            cur = []
        cur.append(y)
    if cur:
        clusters.append(float(np.mean(cur)))
    return clusters


def calibrate_3d(gray):
    dark = gray < 120
    # left vertical edge x: column with longest dark run near the guess
    cols = {x: dark[2400:3300, x].sum() for x in range(int(LEFT_EDGE_X) - 6, int(LEFT_EDGE_X) + 7)}
    xs = np.array(sorted(cols))
    wts = np.array([cols[x] for x in xs], float)
    wts = np.where(wts > 0.5 * wts.max(), wts, 0)
    x_left = float((xs * wts).sum() / wts.sum())
    pY, sY = fit_edge(dark, EDGE_GUESS["Y"], 480, 1200)
    pX, sX = fit_edge(dark, EDGE_GUESS["X"], 1300, 1820)
    # front corner = intersection of the two bottom edges; left/right corners on the vertical edges
    xf = (pX[1] - pY[1]) / (pY[0] - pX[0])
    front = (xf, np.polyval(pY, xf))
    left = (x_left, np.polyval(pY, x_left))
    # right vertical edge: strongest dark column near the X-edge end
    cols = {x: dark[3000:3230, x].sum() for x in range(1835, 1870)}
    x_right = float(max(cols, key=cols.get))
    right = (x_right, np.polyval(pX, x_right))
    tY = edge_ticks(dark, pY, int(left[0]) + 10, int(front[0]) - 8, (-583, 286))
    tX = edge_ticks(dark, pX, int(front[0]) + 10, int(right[0]) - 8, (835, 199))
    tZ = z_ticks(dark, x_left)
    # Tick values: labels read on the figure (Y: 20, 0, -20 left to right; X: -20, 0, 20 left to right;
    # Z: 60, 40, 20, 0 top to bottom). Box limits follow from the tick spacing (measured, then rounded check).
    assert len(tY) == 3 and len(tX) == 3, (tY, tX)
    sy = (tY[2] - tY[0]) / 40.0
    sx = (tX[2] - tX[0]) / 40.0
    Y_left = 20 + (tY[0] - left[0]) / sy
    Y_front = -20 - (front[0] - tY[2]) / sy
    X_front = -20 - (tX[0] - front[0]) / sx
    X_right = 20 + (right[0] - tX[2]) / sx
    zt = [t for t in tZ if t < left[1] - 15]   # drop the corner itself
    zt = zt[-4:]
    assert len(zt) == 4, tZ
    sz = (zt[-1] - zt[0]) / 60.0
    Z_floor = -(left[1] - zt[-1]) / sz
    lim = dict(Y_left=Y_left, Y_front=Y_front, X_front=X_front, X_right=X_right, Z_floor=Z_floor)
    for k in ("Y_left", "Y_front", "X_front", "X_right"):
        assert abs(abs(lim[k]) - 40) < 1.5, lim
    # least-squares affine fit; floor points use Z = Zf (unknown, solved as d = bz*Zf)
    U, Arow, V, Brow = [], [], [], []
    pts = []  # (X, Y, Z or None, u, v)
    for val, x in zip((20, 0, -20), tY):
        pts.append((-40, val, None, x, np.polyval(pY, x)))
    for val, x in zip((-20, 0, 20), tX):
        pts.append((val, -40, None, x, np.polyval(pX, x)))
    for val, y in zip((60, 40, 20, 0), zt):
        pts.append((-40, 40, val, x_left, y))
    pts += [(-40, 40, None, *left), (-40, -40, None, *front), (40, -40, None, *right)]
    for X, Y, Z, u, v in pts:
        Arow.append([X, Y, 1])
        U.append(u)
        Brow.append([X, Y, 0 if Z is None else Z, 1, 1 if Z is None else 0])
        V.append(v)
    a, *_ = np.linalg.lstsq(np.array(Arow, float), np.array(U), rcond=None)
    b, *_ = np.linalg.lstsq(np.array(Brow, float), np.array(V), rcond=None)
    ru = np.array(U) - np.array(Arow, float) @ a
    rv = np.array(V) - np.array(Brow, float) @ b
    cal = dict(ax=a[0], ay=a[1], cu=a[2], bx=b[0], by=b[1], bz=b[2], cv=b[3], Zf=b[4] / b[2],
               res_u_px=float(np.sqrt(np.mean(ru ** 2))), res_v_px=float(np.sqrt(np.mean(rv ** 2))),
               edge_fit_sd_px=max(sY, sX), x_left=x_left, x_right=x_right, **lim)
    return cal, pts


# ---------------------------------------------------------------- inset calibration
def calibrate_inset(rgb):
    x0, y0, x1, y1 = INSET_REGION
    s = rgb[y0:y1, x0:x1]
    mx, mn = s.max(2), s.min(2)
    grey = (mx - mn < 20) & (mx > 170) & (mx < 240)
    yy, xx = np.nonzero(grey)
    xx = xx + x0
    yy = yy + y0
    c = np.array(INSET_GUESS)
    rings = []
    for _ in range(3):
        r = np.hypot(xx - c[0], yy - c[1])
        h, e = np.histogram(r, bins=np.arange(50, 320, 2))
        pk = sorted(set(int(round((e[i] + 1) / 6) * 6) for i in np.argsort(h)[-6:]))
        rings = []
        for R0 in pk:
            sel = np.abs(r - R0) < 6
            if sel.sum() < 80:
                continue
            f = lambda p: np.hypot(xx[sel] - p[0], yy[sel] - p[1]) - p[2]
            rings.append(optimize.least_squares(f, [c[0], c[1], R0], loss="soft_l1", f_scale=2).x)
        c = np.mean([q[:2] for q in rings], axis=0)
    radii = sorted(q[2] for q in rings)
    inner = float(np.mean([q for q in radii if q < 200]))
    outer = float(np.mean([q for q in radii if q > 200]))
    return dict(cx=float(c[0]), cy=float(c[1]), r25_px=inner, outer_px=outer,
                mm_per_px=25.0 / inner, outer_mm=25.0 * outer / inner,
                centre_spread_px=float(np.ptp([q[0] for q in rings]) + np.ptp([q[1] for q in rings])))


def inset_xy(ins, px, py):
    """Inset pixel -> paper-frame (x, y) mm. Up = +x, left = +y."""
    k = ins["mm_per_px"]
    return (ins["cy"] - py) * k, (ins["cx"] - px) * k


# ---------------------------------------------------------------- markers
def markers(mask, x0, y0, cal_mask_keep, min_sep=14):
    m = ndi.binary_fill_holes(mask & cal_mask_keep)
    dt = ndi.distance_transform_edt(m)
    mx = ndi.maximum_filter(dt, size=15)
    pk = (dt == mx) & (dt >= 8)
    lab, n = ndi.label(pk)
    cms = ndi.center_of_mass(pk, lab, range(1, n + 1))
    pts = [(c[1] + x0, c[0] + y0) for c in cms]
    # merge peaks closer than min_sep (one marker can give two plateau peaks)
    merged = []
    for p in sorted(pts, key=lambda q: q[1]):
        for i, q in enumerate(merged):
            if math.hypot(p[0] - q[0][0], p[1] - q[0][1]) < min_sep:
                q[1].append(p)
                merged[i] = (tuple(np.mean(q[1], axis=0)), q[1])
                break
        else:
            merged.append((p, [p]))
    return [q[0] for q in merged]


def order_chain(pts, start):
    pts = list(pts)
    out = []
    cur = start
    while pts:
        i = int(np.argmin([math.hypot(p[0] - cur[0], p[1] - cur[1]) for p in pts]))
        cur = pts.pop(i)
        out.append(cur)
    return out


# ---------------------------------------------------------------- volume estimate
def theta_cc(rho, h):
    """Bend angle from tip radial offset rho and rise h (constant curvature, length-free)."""
    if h <= 0.5 or rho <= 0.2:
        return float("nan")
    q = rho / h
    f = lambda t: (1 - math.cos(t)) / (t - math.sin(t)) - q
    lo, hi = 1e-3, 2 * math.pi - 1e-3
    if f(lo) * f(hi) > 0:
        return float("nan")
    return optimize.brentq(f, lo, hi)


def gauss_theta(v, c):
    return c["A1"] * math.exp(-((v - c["B1"]) / c["E1"]) ** 2) + c["A2"] * math.exp(-((v - c["B2"]) / c["E2"]) ** 2)


def volume_from_theta(t, c):
    if not math.isfinite(t):
        return float("nan")
    grid = np.linspace(0, 8, 8001)
    th = np.array([gauss_theta(v, c) for v in grid])
    if np.any(np.diff(th) <= 0):
        raise SystemExit("Gaussian fit not monotonic on 0-8 ml; cannot invert")
    if t < th[0] or t > th[-1]:
        return float("nan")
    return float(np.interp(t, th, grid))


V_END_ML = 8.0       # paper: linear injection up to 8 ml
V_STEP_ML = 0.2      # Fig 7D marker spacing (read from its volume axis), assumed for Fig 7C too
GAP_RATIO = 1.6      # image spacing above 1.6x the local median = hidden (merged) marker(s)
STOP_RATIO = 2.5     # above 2.5x: cannot count across; stop
ORIGIN_MERGED_PX = 50  # markers this close to the plot origin sit in the merged start blob


def assign_volumes(rows, origin_uv):
    """Count markers back from the last one (8.0 ml) in 0.2 ml steps, allowing for merged markers.

    Walks from the end of the trajectory toward the origin. Where the image spacing to the previous marker is more
    than GAP_RATIO times the local median spacing, round(ratio) - 1 hidden markers are assumed; above STOP_RATIO,
    or once a marker sits in the merged blob at the origin, counting stops and earlier markers get no volume.
    """
    ok = [r for r in rows if r["status"] == "ok"]
    sp = [math.hypot(ok[i + 1]["u_px"] - ok[i]["u_px"], ok[i + 1]["v_px"] - ok[i]["v_px"]) for i in range(len(ok) - 1)]
    k = 0
    stopped = False
    for i in range(len(ok) - 1, -1, -1):
        r = ok[i]
        near_origin = math.hypot(r["u_px"] - origin_uv[0], r["v_px"] - origin_uv[1]) < ORIGIN_MERGED_PX
        if stopped or near_origin:
            r["count_note"] = "before_count_stop" if stopped else "in_origin_blob"
            stopped = True
            continue
        r["k_from_end"] = k
        r["V_ml"] = round(V_END_ML - V_STEP_ML * k, 6)
        if i == 0:
            break
        s = sp[i - 1]
        loc = [sp[j] for j in range(max(0, i - 4), min(len(sp), i + 3)) if j != i - 1]
        ratio = s / float(np.median(loc))
        r_prev = ok[i - 1]
        if ratio > STOP_RATIO:
            stopped = True
            r_prev["count_note"] = f"gap_ratio_{ratio:.1f}_stop"
            continue
        hidden = int(round(ratio)) - 1 if ratio > GAP_RATIO else 0
        if hidden:
            r_prev["count_note"] = f"after_{hidden}_hidden_marker(s)_gap_ratio_{ratio:.1f}"
        k += 1 + hidden


def main():
    for path in (FIG, TABLES):
        if not os.path.exists(path):
            raise SystemExit(f"missing {path}")
    img = Image.open(FIG).convert("RGB")
    rgb = np.asarray(img).astype(int)
    gray = np.asarray(img.convert("L")).astype(float)
    cal, cal_pts = calibrate_3d(gray)
    ins = calibrate_inset(rgb)
    tables = read_tables()
    t3 = {int(r["chamber"]): {k: float(r[k]) for k in ("A1", "B1", "E1", "A2", "B2", "E2", "l_mm", "phi_deg")}
          for r in tables if r["table"] == "3"}
    t2 = {int(r["chamber"]): float(r["max_theta_deg"]) for r in tables if r["table"] == "2" and r["alpha_deg"] == "0"}

    # pixel <-> mm along u for the inset/3D link
    R, G, B = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    x0, y0, x1, y1 = BOX
    yy, xx = np.mgrid[y0:y1, x0:x1]
    keep = (xx - ins["cx"]) ** 2 + (yy - ins["cy"]) ** 2 > INSET_EXCLUDE_R ** 2
    ix0, iy0, ix1, iy1 = INSET_REGION
    iyy, ixx = np.mgrid[iy0:iy1, ix0:ix1]
    in_disc = (ixx - ins["cx"]) ** 2 + (iyy - ins["cy"]) ** 2 < (ins["outer_px"] - 3) ** 2

    origin_uv = (cal["cu"], cal["cv"])  # (0,0,0)
    ov3d = img.crop((x0 - 20, y0 - 60, x1 + 20, y1 + 20)).copy()
    d3 = ImageDraw.Draw(ov3d)
    ovin = img.crop(INSET_REGION).copy().resize(((ix1 - ix0) * 2, (iy1 - iy0) * 2), Image.NEAREST)
    dI = ImageDraw.Draw(ovin)
    summary = []
    for ch in (1, 2, 3):
        mask = COL[ch](R, G, B)
        mk = markers(mask[y0:y1, x0:x1], x0, y0, keep)
        mk = order_chain(mk, origin_uv)
        # inset trace pixels for this chamber, in mm
        im_mask = COL[ch](R[iy0:iy1, ix0:ix1], G[iy0:iy1, ix0:ix1], B[iy0:iy1, ix0:ix1]) & in_disc
        py, px = np.nonzero(im_mask)
        px = px + ix0
        py = py + iy0
        ixy = np.array([inset_xy(ins, a, b) for a, b in zip(px, py)])
        iu = cal["ax"] * ixy[:, 0] + cal["ay"] * ixy[:, 1] + cal["cu"]
        band = 1.0  # half-width of the u band, px
        rows = []
        prev = np.array([0.0, 0.0])
        for (u, v) in mk:
            sel = np.abs(iu - u) < band
            if sel.sum() < 3:
                rows.append(dict(u_px=u, v_px=v, status="no_inset_match"))
                continue
            cand = ixy[sel]
            # split into clusters along the band; keep the one nearest the previous solution
            order = np.argsort(cand[:, 0] * cal["ay"] - cand[:, 1] * cal["ax"])  # along the band direction
            cs = cand[order]
            gaps = np.where(np.hypot(*np.diff(cs, axis=0).T) > 1.5)[0]
            groups = np.split(cs, gaps + 1)
            cents = [g.mean(axis=0) for g in groups]
            j = int(np.argmin([np.hypot(*(c - prev)) for c in cents]))
            xy = cents[j]
            amb = len(groups) > 1 and sorted(np.hypot(*(c - prev)) for c in cents)[1] < 3.0
            halfw = float(np.ptp(groups[j] @ np.array([cal["ay"], -cal["ax"]]) / math.hypot(cal["ax"], cal["ay"]))) / 2
            z = (v - cal["bx"] * xy[0] - cal["by"] * xy[1] - cal["cv"]) / cal["bz"]
            rho = math.hypot(*xy)
            t = theta_cc(rho, z)
            V = volume_from_theta(t, t3[ch])
            rows.append(dict(u_px=u, v_px=v, x_mm=xy[0], y_mm=xy[1], z_mm=z, rho_mm=rho,
                             inset_halfwidth_mm=halfw, n_inset_groups=len(groups),
                             theta_cc_deg=math.degrees(t) if math.isfinite(t) else float("nan"),
                             V_est_ml=V, status="ambiguous_inset" if amb else "ok"))
            prev = xy
        # reading error: x, y are uncertain along the u-band direction by the inset stroke half-width; that
        # uncertainty carries into z through v. Calibration residuals add < 0.1 mm.
        nb = np.array([cal["ay"], -cal["ax"]]) / math.hypot(cal["ax"], cal["ay"])
        dv_dmm = abs(cal["bx"] * nb[0] + cal["by"] * nb[1])
        for r in rows:
            if r["status"] == "ok":
                r["xy_err_mm"] = r["inset_halfwidth_mm"]
                r["z_err_mm"] = r["inset_halfwidth_mm"] * dv_dmm / abs(cal["bz"]) + cal["res_v_px"] / abs(cal["bz"])
        assign_volumes(rows, origin_uv)
        # draw overlays: reproject solved points into both views
        colr = {1: (0, 0, 0), 2: (0, 0, 0), 3: (0, 0, 0)}[ch]
        for r in rows:
            u, v = r["u_px"], r["v_px"]
            cx, cy = u - (x0 - 20), v - (y0 - 60)
            if r["status"] == "ok":
                d3.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], outline=colr, width=2)
                pu = cal["ax"] * r["x_mm"] + cal["ay"] * r["y_mm"] + cal["cu"]
                pv = cal["bx"] * r["x_mm"] + cal["by"] * r["y_mm"] + cal["bz"] * r["z_mm"] + cal["cv"]
                d3.line([cx, cy, pu - (x0 - 20), pv - (y0 - 60)], fill=(255, 0, 255), width=1)
                qx = (ins["cx"] - r["y_mm"] / ins["mm_per_px"] - ix0) * 2
                qy = (ins["cy"] - r["x_mm"] / ins["mm_per_px"] - iy0) * 2
                dI.ellipse([qx - 3, qy - 3, qx + 3, qy + 3], outline=(0, 0, 0), width=1)
            else:
                d3.line([cx - 6, cy - 6, cx + 6, cy + 6], fill=(255, 0, 255), width=2)
                d3.line([cx - 6, cy + 6, cx + 6, cy - 6], fill=(255, 0, 255), width=2)
        out = os.path.join(HERE, f"chandler2020_fig7C_a0_tip_C{ch}.csv")
        swap = {1: 1, 2: 3, 3: 2}
        for r in rows:
            if "V_ml" in r:
                r["theta_eq11_deg"] = math.degrees(gauss_theta(r["V_ml"], t3[ch]))
                r["theta_eq11_rows23swapped_deg"] = math.degrees(gauss_theta(r["V_ml"], t3[swap[ch]]))
        rows.insert(0, dict(V_ml=0.0, x_mm=0.0, y_mm=0.0, z_mm=0.0, rho_mm=0.0, status="rest_by_definition",
                            count_note="plot origin; tip displacement is zero at zero injected volume"))
        cols = ["marker", "V_ml", "x_mm", "y_mm", "z_mm", "xy_err_mm", "z_err_mm", "status", "count_note",
                "k_from_end", "u_px", "v_px", "rho_mm", "inset_halfwidth_mm", "n_inset_groups", "theta_cc_deg",
                "theta_eq11_deg", "theta_eq11_rows23swapped_deg", "V_est_ml"]
        with open(out, "w", newline="", encoding="utf-8") as f:
            f.write(f"# Digitized from Chandler et al. 2020, Front. Robot. AI 7:119, Fig 7C (alpha = 0), chamber C{ch}.\n")
            f.write(f"# Source image source/chandler2020_fig7.jpeg sha256 {sha256(FIG)}\n")
            f.write("# Paper frame: z = rise of the tip from rest toward the base; x, y in the cross-section "
                    "(inset: x up, y left).\n")
            f.write("# Markers ordered from the origin outward along the trajectory. V_est_ml is a model-based "
                    "estimate (constant curvature + paper Eq 11 fit), see PROVENANCE.md.\n")
            f.write("# Digitized data -- error bounded by pixel resolution of source figure; these are published "
                    "measurements, not our measurements\n")
            w = csv.writer(f)
            w.writerow(cols)
            for i, r in enumerate(rows):
                w.writerow([i] + [("" if (k not in r or (isinstance(r[k], float) and not math.isfinite(r[k])))
                                   else (f"{r[k]:.3f}" if isinstance(r[k], float) else r[k])) for k in cols[1:]])
        ok = [r for r in rows if r["status"] == "ok"]
        withV = [r for r in ok if "V_ml" in r and math.isfinite(r["theta_cc_deg"])]
        off1 = np.array([r["theta_cc_deg"] - r["theta_eq11_deg"] for r in withV])
        off2 = np.array([r["theta_cc_deg"] - r["theta_eq11_rows23swapped_deg"] for r in withV])
        summary.append((ch, len(rows) - 1, len(ok), len(withV), min(r["V_ml"] for r in withV),
                        max(r["z_mm"] for r in ok),
                        math.degrees(math.atan2(np.mean([r["y_mm"] for r in ok[-15:]]),
                                                np.mean([r["x_mm"] for r in ok[-15:]]))),
                        max(r["theta_cc_deg"] for r in ok if math.isfinite(r["theta_cc_deg"])),
                        np.median(off1), np.percentile(off1, 25), np.percentile(off1, 75),
                        np.median(off2), np.percentile(off2, 25), np.percentile(off2, 75)))
        mu = [r["u_px"] for r in rows if "u_px" in r]
        far = int(np.argmax(np.abs(np.array(mu) - cal["cu"])))
        sgn = np.sign(mu[far] - cal["cu"])
        print(f"C{ch}: wrote {out}")
        print(f"   inset-scale check: furthest marker u - u0 = {mu[far] - cal['cu']:.1f} px; inset trace reaches "
              f"{(iu.max() if sgn > 0 else iu.min()) - cal['cu']:.1f} px (should exceed it by about the inset stroke "
              f"half-width; a 25 mm label on the outer ring instead would shrink the trace by a factor "
              f"{ins['outer_px'] / ins['r25_px']:.2f})")
    ov3d.save(os.path.join(HERE, "verify_fig7C_a0_3dview.png"))
    ovin.save(os.path.join(HERE, "verify_fig7C_a0_inset.png"))

    with open(os.path.join(HERE, "chandler2020_fig7C_a0_calibration.csv"), "w", newline="", encoding="utf-8") as f:
        f.write("# Calibration of Fig 7C found by digitize_chandler2020_fig7.py (pixels of source/chandler2020_fig7.jpeg)\n")
        w = csv.writer(f)
        w.writerow(["quantity", "value"])
        for k, v in list(cal.items()) + [("inset_" + k, v) for k, v in ins.items()]:
            w.writerow([k, f"{v:.5f}"])
        for X, Y, Z, u, v in cal_pts:
            w.writerow([f"calpoint X={X} Y={Y} Z={'floor' if Z is None else Z}", f"{u:.2f};{v:.2f}"])
    print("3D calibration:", {k: round(v, 4) for k, v in cal.items()})
    print("inset:", {k: round(v, 4) for k, v in ins.items()})
    print("chamber | markers found, solved, given a volume, lowest volume (ml) | max z (mm) | azimuth of last 15 (deg)"
          " | max theta_cc (deg) | theta_cc - Eq11 at counted volume, median [IQR] (deg): Table 3 rows as printed;"
          " rows 2 and 3 swapped")
    for s in summary:
        print(f"C{s[0]} | {s[1]} {s[2]} {s[3]} {s[4]:.1f} | {s[5]:.1f} | {s[6]:.1f} (Table 3: {t3[s[0]]['phi_deg']})"
              f" | {s[7]:.0f} (Table 2: {t2[s[0]]}) | {s[8]:+.0f} [{s[9]:+.0f}, {s[10]:+.0f}]; "
              f"{s[11]:+.0f} [{s[12]:+.0f}, {s[13]:+.0f}]")


if __name__ == "__main__":
    main()
