"""digitize_kubler2023_fig7.py -- digitise the 40 mm members of Fig 7 of
Kubler et al. 2023, "A Comparison of Pneumatic Actuators for Soft Growing
Vine Robots" (arXiv:2305.00967v3).

What it reads: the single embedded raster image of Fig 7 (page index 9 of
the arXiv PDF in source/), no rendering or resampling.

What it writes (next to this script):
  kubler2023_fig7.png                       the figure image, as embedded
  kubler2023_fig7_bend_<type>.csv           free bending (row 1), mean curve
  kubler2023_fig7_force_<type>.csv          lateral force, tip held (row 3)
  kubler2023_fig7_calibration.json          tick pixels, fits, residuals
  verify_<row>_<type>.png                   digitised points over the panel

<type> is pouch_W40xL40, cPAM_W40xL40, fPAM_W40.

Method
------
Calibration: per panel, the axis lines are found as the longest dark-grey
runs, then the inward tick marks are detected as short dark neutral marks
just inside each axis. Tick values are assigned from the axis origin and
the uniform tick spacing (x: 0-40 kPa every 10; y: 0-2.5 deg/cm every 0.5
in row 1, 0-5 N every 1 in row 3; read off the figure). A straight line is
fitted per axis and the maximum residual is asserted below 1.5 px.

Colour: every 40 mm member is drawn in the same mint colour in its panel
(legend sample ~RGB 142,231,212, hue ~167 deg). Pixels are classed by hue
(158-178 deg), which separates it from the neighbouring teal W40xL20
(hue ~187), light-blue W60xL20 (~215), lavender W40xL60 (~227) and the
dark-blue/black members.

Row 1 (bending): model (solid) and data (dashed + shaded standard
deviation band) share the colour. In each pixel column the mint pixels are
split into vertical runs; candidate data runs are those carrying pale
(band) pixels, and the mean is taken as the midpoint of the run (the band
is mean +/- 1 SD); the band edges are kept as the SD. Tracing starts at the
high-pressure end of the curve (where the band is unambiguous) and walks
left, accepting only a run within 6 px of the previous one; after 1 kPa of
columns with no such run it stops, so where the curve disappears under
other series at low pressure there are no rows rather than wrong ones.
The column estimates
are then summarised at every 2 kPa (median of columns within +/-0.5 kPa)
and at the end of the curve (median of the last 0.5 kPa of columns).

Row 3 (force): each plotted mean carries a vertical error bar with caps.
Error bars are found as columns where the mint run is much taller than the
dashed line (gaps up to 10 px bridged, so a stem crossed by another
series' line stays whole); the run's centre is the mean and its
half-height the SD. As a check, the dashed mean line either side of each
bar is fitted and compared with the bar centre (CSV column line_check_N).
The paper states ten pressures per actuator; the count is asserted.

Run:
    python -I digitize_kubler2023_fig7.py
"""

from __future__ import annotations

import hashlib
import json
import os
import sys

import numpy as np
import pymupdf
from matplotlib import colors as mcolors
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
PDF = os.path.join(HERE, "source", "kubler2023_arxiv2305.00967.pdf")
PDF_SHA256 = "495c299ad6b08341e0127616caa6bd894de3fec171c0cfc95e8834ab7cadca51"
FIG_PAGE_INDEX = 9          # 0-based page carrying Fig 7

# Panel search windows in the embedded image (columns split in thirds; rows
# hold the first and third figure rows). Only used to localise the search.
ROW_WINDOWS = {"bend": (0, 900), "force": (1550, 2390)}
TYPES = ["pouch_W40xL40", "cPAM_W40xL40", "fPAM_W40"]
TYPE_LABEL = {"pouch_W40xL40": "pouch motor W40 x L40 mm",
              "cPAM_W40xL40": "cPAM W40 x L40 mm",
              "fPAM_W40": "fPAM W40 mm"}
X_TICKS = (0.0, 10.0)       # first value, step (kPa)
Y_TICKS = {"bend": (0.0, 0.5), "force": (0.0, 1.0)}
Y_TOP = {"bend": 2.5, "force": 5.0}

HUE_LO, HUE_HI = 158 / 360, 178 / 360
SAT_MIN = 0.05
SAT_LINE = 0.30             # above: full line; below: pale band / antialias


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def runs(mask_1d, gap=0):
    """Contiguous True runs as (start, end) inclusive, merging gaps <= gap."""
    idx = np.flatnonzero(mask_1d)
    if idx.size == 0:
        return []
    out = [[idx[0], idx[0]]]
    for i in idx[1:]:
        if i - out[-1][1] <= gap + 1:
            out[-1][1] = i
        else:
            out.append([i, i])
    return [tuple(r) for r in out]


def longest(mask_1d):
    rs = runs(mask_1d)
    return max(rs, key=lambda r: r[1] - r[0]) if rs else (0, -1)


# ------------------------------------------------------------ calibration

def detect_axes(gray, x0, x1, y0, y1):
    """y-axis column and x-axis row: longest dark-grey runs in the window."""
    sub = gray[y0:y1, x0:x1]
    dark = (sub > 10) & (sub < 300)          # dark grey, not the black frame
    best_c = max(range(sub.shape[1]), key=lambda j: np.subtract(*longest(dark[:, j])[::-1]))
    best_r = max(range(sub.shape[0]), key=lambda i: np.subtract(*longest(dark[i, :])[::-1]))
    rc = longest(dark[:, best_c])
    cr = longest(dark[best_r, :])
    return {"yaxis_col": x0 + best_c, "yaxis_top_row": y0 + rc[0],
            "xaxis_row": y0 + best_r, "xaxis_right_col": x0 + cr[1]}


def tick_centres(neutral_dark_strip, min_rows):
    """Centres of short marks present in >= min_rows of the strip rows."""
    hits = neutral_dark_strip.sum(axis=0) >= min_rows
    return [0.5 * (a + b) for a, b in runs(hits)]


def calibrate(rgb, gray, row, col):
    W = rgb.shape[1]
    x0, x1 = col * W // 3, (col + 1) * W // 3
    y0, y1 = ROW_WINDOWS[row]
    ax = detect_axes(gray, x0, x1, y0, y1)
    sat = rgb.max(-1) - rgb.min(-1)
    neutral_dark = (gray < 400) & (sat < 40)

    # x ticks: inward marks just above the x axis (rows axis-7 .. axis-2)
    r = ax["xaxis_row"]
    strip = neutral_dark[r - 7:r - 1, ax["yaxis_col"] + 30:ax["xaxis_right_col"] + 3]
    xs = [c + ax["yaxis_col"] + 30 for c in tick_centres(strip, 6)]
    # y ticks: inward marks just right of the y axis (cols axis+2 .. axis+7)
    c = ax["yaxis_col"]
    strip = neutral_dark[ax["yaxis_top_row"] - 2:r - 30, c + 2:c + 8].T
    ys = [rr + ax["yaxis_top_row"] - 2 for rr in tick_centres(strip, 6)]

    x_px = [float(ax["yaxis_col"])] + xs            # axis line = first tick
    y_px = [float(ax["xaxis_row"])] + ys
    xv0, xstep = X_TICKS
    yv0, ystep = Y_TICKS[row]
    # Assign values from spacing (ticks are uniform); drop spurious marks
    # that do not sit on the uniform grid.
    sx = np.median(np.diff(sorted(x_px)))
    sy = np.median(np.diff(sorted(y_px)))
    xvals = [xv0 + xstep * round((p - x_px[0]) / sx) for p in x_px]
    yvals = [yv0 + ystep * round((y_px[0] - p) / sy) for p in y_px]
    keep_x = [abs((p - x_px[0]) / sx - round((p - x_px[0]) / sx)) < 0.1 for p in x_px]
    keep_y = [abs((y_px[0] - p) / sy - round((y_px[0] - p) / sy)) < 0.1 for p in y_px]
    x_px = [p for p, k in zip(x_px, keep_x) if k]
    xvals = [v for v, k in zip(xvals, keep_x) if k]
    y_px = [p for p, k in zip(y_px, keep_y) if k]
    yvals = [v for v, k in zip(yvals, keep_y) if k]

    # Axis limits are also the axis-line ends (MATLAB box off): add them as
    # a cross-check, not as extra fit points.
    fx = np.polyfit(x_px, xvals, 1)
    fy = np.polyfit(y_px, yvals, 1)
    res_x = np.array(x_px) - (np.array(xvals) - fx[1]) / fx[0]
    res_y = np.array(y_px) - (np.array(yvals) - fy[1]) / fy[0]
    # Series legend box (top left): rows in the upper 60% of the plot whose
    # longest neutral dark run is >= 120 px and starts near the y axis.
    top, bot = ax["yaxis_top_row"], ax["xaxis_row"]
    lrows = []
    for rr in range(top + 2, top + int(0.6 * (bot - top))):
        a, b = longest(neutral_dark[rr, ax["yaxis_col"] + 3:ax["xaxis_right_col"]])
        if b - a + 1 >= 120 and a < 120:
            lrows.append((rr, ax["yaxis_col"] + 3 + a, ax["yaxis_col"] + 3 + b))
    assert lrows, "legend box not found"
    legend_box = [min(r[1] for r in lrows), min(r[0] for r in lrows),
                  max(r[2] for r in lrows), max(r[0] for r in lrows)]
    cal = {
        "panel": f"{row} col {col}",
        "axes": ax,
        "legend_box": legend_box,
        "x_ticks_px": [round(p, 1) for p in x_px], "x_ticks_kPa": xvals,
        "y_ticks_px": [round(p, 1) for p in y_px], "y_ticks_value": yvals,
        "x_fit_kPa_per_px": fx[0], "x_fit_offset": fx[1],
        "y_fit_unit_per_px": fy[0], "y_fit_offset": fy[1],
        "x_max_residual_px": float(np.abs(res_x).max()),
        "y_max_residual_px": float(np.abs(res_y).max()),
        "x_axis_end_value": float(np.polyval(fx, ax["xaxis_right_col"])),
        "y_axis_top_value": float(np.polyval(fy, ax["yaxis_top_row"])),
    }
    assert len(x_px) >= 4 and len(y_px) >= 4, cal
    assert cal["x_max_residual_px"] < 1.5 and cal["y_max_residual_px"] < 1.5, cal
    assert abs(cal["x_axis_end_value"] - 40.0) < 0.3, cal
    assert abs(cal["y_axis_top_value"] - Y_TOP[row]) < 0.03 * Y_TOP[row], cal
    return cal


# --------------------------------------------------------------- tracing

def mint_masks(hsv, x0, x1, y0, y1):
    sub = hsv[y0:y1, x0:x1]
    h, s, v = sub[..., 0], sub[..., 1], sub[..., 2]
    mint = (h >= HUE_LO) & (h <= HUE_HI) & (s >= SAT_MIN) & (v > 0.5)
    pale = mint & (s < SAT_LINE)
    return mint, pale


def trace_bend(hsv, cal):
    ax = cal["axes"]
    x0, x1 = ax["yaxis_col"] + 1, ax["xaxis_right_col"]
    y0, y1 = ax["yaxis_top_row"], ax["xaxis_row"] - 1
    mint, pale = mint_masks(hsv, x0, x1, y0, y1)
    # Trace from the high-pressure end leftwards with continuity: the curve
    # end is unambiguous (the band is clear of the model line there), and
    # at low pressure the band runs under other members' bands, where the
    # nearest mint run may be the model line. A column is accepted only if
    # its band run lies within MAX_STEP_PX of the last accepted one; after
    # MAX_GAP_KPA of rejected columns the trace stops (curve lost under
    # other series) rather than jumping to another line.
    MAX_STEP_PX, MAX_GAP_COLS = 6, int(round(1.0 / cal["x_fit_kPa_per_px"]))
    lx0, ly0, lx1, ly1 = cal["legend_box"]
    cols, prev, miss = [], None, 0
    for j in range(mint.shape[1] - 1, -1, -1):
        cand = []
        for a, b in runs(mint[:, j], gap=1):
            if lx0 - 3 <= x0 + j <= lx1 + 3 and y0 + b <= ly1 + 3:
                continue                # legend sample line, not data
            npale = int(pale[a:b + 1, j].sum())
            if npale >= 3:
                cand.append((npale, 0.5 * (a + b), a, b))
        if prev is None:
            if not cand:
                continue
            npale, c, a, b = max(cand)
        else:
            near = [t for t in cand if abs(t[1] - prev) <= MAX_STEP_PX]
            if not near:
                miss += 1
                if miss > MAX_GAP_COLS:
                    break
                continue
            npale, c, a, b = min(near, key=lambda t: abs(t[1] - prev))
        prev, miss = c, 0
        cols.append((x0 + j, y0 + c, y0 + a, y0 + b))
    cols = np.array(cols[::-1], float)
    p = np.polyval([cal["x_fit_kPa_per_px"], cal["x_fit_offset"]], cols[:, 0])
    val = np.polyval([cal["y_fit_unit_per_px"], cal["y_fit_offset"]], cols[:, 1])
    hi = np.polyval([cal["y_fit_unit_per_px"], cal["y_fit_offset"]], cols[:, 2])
    lo = np.polyval([cal["y_fit_unit_per_px"], cal["y_fit_offset"]], cols[:, 3])
    return p, val, lo, hi, cols


def summarise_bend(p, val, lo, hi, step=2.0, half=0.5):
    rows = []
    p_end = p.max()
    t = 0.0
    # A grid row is kept only if traced columns cover both halves of its
    # window (the trace stops where the curve is lost, which would leave a
    # one-sided window whose readings belong to a higher pressure); the
    # pressure written is the mean of the columns actually used.
    while t <= p_end - half:
        sel = np.abs(p - t) <= half
        if (sel.sum() >= 3 and p[sel].min() <= t - 0.5 * half
                and p[sel].max() >= t + 0.5 * half):
            rows.append((float(np.mean(p[sel])), np.median(val[sel]),
                         np.median(lo[sel]), np.median(hi[sel]), int(sel.sum())))
        t += step
    sel = p >= p_end - half
    rows.append((float(np.mean(p[sel])), np.median(val[sel]),
                 np.median(lo[sel]), np.median(hi[sel]), int(sel.sum())))
    return rows


def trace_force(hsv, cal):
    ax = cal["axes"]
    x0, x1 = ax["yaxis_col"] + 1, ax["xaxis_right_col"]
    y0, y1 = ax["yaxis_top_row"], ax["xaxis_row"] + 2
    mint, pale = mint_masks(hsv, x0, x1, y0, y1)
    # tallest mint run per column
    tall = np.zeros(mint.shape[1])
    span = [None] * mint.shape[1]
    # gap=BRIDGE_PX: an error-bar stem crossed by another series' line
    # (lines are 5-7 px thick) stays one run instead of being cut short.
    BRIDGE_PX = 10
    for j in range(mint.shape[1]):
        rs = runs(mint[:, j], gap=BRIDGE_PX)
        if rs:
            a, b = max(rs, key=lambda r: r[1] - r[0])
            tall[j] = b - a + 1
            span[j] = (a, b)
    base = np.median(tall[tall > 0])           # dashed-line thickness
    # Error bars: their caps make a short, wide block and the stem a tall
    # column. A bar = a group of adjacent columns whose run exceeds the line
    # thickness by >= 4 px; centre column = the tallest in the group.
    # Columns inside the legend box are excluded (its glyph is a bar too).
    lx0, ly0, lx1, ly1 = cal["legend_box"]
    for j in range(mint.shape[1]):
        if lx0 - 3 <= x0 + j <= lx1 + 3:
            if span[j] is not None and y0 + span[j][1] <= ly1 + 3:
                tall[j] = 0
    bars = []
    # gap=10: a bar whose stem is broken by the line merges back into one
    for a, b in runs(tall >= base + 4, gap=10):
        # Columns within 1 px of the group's tallest run: on short bars the
        # bridged caps tie with the stem, so take the middle of all tied
        # columns (caps are symmetric about the stem), not the first.
        tied = a + np.flatnonzero(tall[a:b + 1] >= tall[a:b + 1].max() - 1)
        j = int(round(0.5 * (tied.min() + tied.max())))
        if span[j] is None or tall[j] < tall[a:b + 1].max() - 1:
            j = int(tied[np.argmin(np.abs(tied - j))])
        bars.append((j, span[j]))
    # Cross-check each bar centre against the dashed mean line, which passes
    # through the plotted means: take thin mint runs (line-thick, not bar)
    # lying within the bar's vertical span in the columns 12-30 px either
    # side, fit a straight line, and evaluate it at the bar column. This is
    # a diagnostic only (the dashed line is short and crossed by other
    # series, so it is noisier than the bar); the value kept is the bar
    # midpoint, and the difference is written to the CSV as line_check_px.

    def line_at(j, ra, rb):
        xs, ys = [], []
        for dj in list(range(-30, -11)) + list(range(12, 31)):
            k = j + dj
            if not 0 <= k < mint.shape[1]:
                continue
            for a, b in runs(mint[:, k], gap=1):
                if b - a + 1 <= base + 3 and ra - 5 <= 0.5 * (a + b) <= rb + 5:
                    xs.append(k)
                    ys.append(0.5 * (a + b))
        left = [x < j for x in xs]
        if sum(left) < 3 or len(xs) - sum(left) < 3:
            return None
        c = np.polyfit(xs, ys, 1)
        return float(np.polyval(c, j))

    pts = []
    for j, (ra, rb) in bars:
        xc = x0 + j
        mid = 0.5 * (ra + rb)
        ln = line_at(j, ra, rb)
        method = "bar_midpoint"
        half_px = 0.5 * (rb - ra)
        yc = y0 + mid
        p = np.polyval([cal["x_fit_kPa_per_px"], cal["x_fit_offset"]], xc)
        f = np.polyval([cal["y_fit_unit_per_px"], cal["y_fit_offset"]], yc)
        sd = abs(cal["y_fit_unit_per_px"]) * half_px
        pts.append((float(p), float(f), float(sd), xc, yc, y0 + ra, y0 + rb,
                    method, None if ln is None else float(ln - mid)))
    return pts, float(base)


# ------------------------------------------------------------------ main

def write_csv(path, header_lines, cols, rows, fmt):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        for ln in header_lines:
            f.write("# " + ln + "\n")
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join(fm.format(v) for fm, v in zip(fmt, r)) + "\n")


def main():
    got = sha256(PDF)
    assert got == PDF_SHA256, f"PDF hash changed: {got}"
    doc = pymupdf.open(PDF)
    imgs = doc[FIG_PAGE_INDEX].get_images(full=True)
    assert len(imgs) == 1, imgs
    raw = doc.extract_image(imgs[0][0])
    fig_png = os.path.join(HERE, "kubler2023_fig7.png")
    with open(fig_png, "wb") as f:
        f.write(raw["image"])
    im = Image.open(fig_png).convert("RGB")
    rgb_i = np.asarray(im).astype(int)
    gray = rgb_i.sum(-1)
    hsv = mcolors.rgb_to_hsv(rgb_i / 255.0)
    print(f"Fig 7 image {im.size[0]} x {im.size[1]} px, xref {imgs[0][0]}")

    calib = {"pdf": os.path.basename(PDF), "pdf_sha256": got,
             "page_index": FIG_PAGE_INDEX,
             "image_size_px": list(im.size), "panels": {}}
    summary = {}
    for col, typ in enumerate(TYPES):
        for row in ("bend", "force"):
            cal = calibrate(rgb_i, gray, row, col)
            calib["panels"][f"{row}_{typ}"] = cal
            print(f"[{row} {typ}] x ticks {cal['x_ticks_px']} "
                  f"y ticks {cal['y_ticks_px']} max resid "
                  f"{cal['x_max_residual_px']:.2f}/{cal['y_max_residual_px']:.2f} px")
            ax = cal["axes"]
            crop = (ax["yaxis_col"] - 40, ax["yaxis_top_row"] - 20,
                    ax["xaxis_right_col"] + 20, ax["xaxis_row"] + 20)
            ov = im.crop(crop).copy()
            dr = ImageDraw.Draw(ov)
            ox, oy = crop[0], crop[1]

            if row == "bend":
                p, val, lo, hi, cols = trace_bend(hsv, cal)
                rows = summarise_bend(p, val, lo, hi)
                for xc, yc, ya, yb in cols:
                    dr.point((xc - ox, yc - oy), fill=(255, 0, 0))
                for t, v, l, h, n in rows:
                    xp = (t - cal["x_fit_offset"]) / cal["x_fit_kPa_per_px"]
                    yp = (v - cal["y_fit_offset"]) / cal["y_fit_unit_per_px"]
                    dr.ellipse((xp - ox - 5, yp - oy - 5, xp - ox + 5, yp - oy + 5),
                               outline=(255, 0, 0), width=2)
                out = os.path.join(HERE, f"kubler2023_fig7_bend_{typ}.csv")
                write_csv(out, [
                    f"Digitised from Kubler et al. 2023 (arXiv:2305.00967v3) Fig 7, "
                    f"row 1 (quasi-static bending), {TYPE_LABEL[typ]}",
                    "Series: experimental mean (dashed line; shaded band = +/-1 SD "
                    "over five runs), horizontal configuration, vine body at 1.75 kPa",
                    "bend_deg_per_cm = bending angle per nominal length as published "
                    "(deg/cm); band_lo/band_hi = shaded band edges",
                    "Sampled every 2 kPa (median of pixel columns within +/-0.5 kPa, "
                    "kept only if columns cover both halves of the window; "
                    "pressure_kPa = mean of the columns used); last row = end of "
                    "the plotted curve (last 0.5 kPa of columns)",
                    "Published measurements digitised by digitize_kubler2023_fig7.py; "
                    "not our measurements",
                ], ["pressure_kPa", "bend_deg_per_cm", "band_lo_deg_per_cm",
                    "band_hi_deg_per_cm", "n_columns", "series"],
                    [(t, v, l, h, n, "data_mean") for t, v, l, h, n in rows],
                    ["{:.3f}", "{:.4f}", "{:.4f}", "{:.4f}", "{:d}", "{}"])
                summary[f"bend_{typ}"] = rows
                print(f"   bend: {len(cols)} columns traced, {len(rows)} rows "
                      f"-> {os.path.basename(out)}; end {rows[-1][0]:.2f} kPa "
                      f"{rows[-1][1]:.3f} deg/cm")
            else:
                pts, base = trace_force(hsv, cal)
                for p, f, sd, xc, yc, ya, yb, *_ in pts:
                    dr.line((xc - ox - 9, yc - oy, xc - ox + 9, yc - oy), fill=(255, 0, 0), width=2)
                    dr.line((xc - ox, ya - oy, xc - ox, yb - oy), fill=(255, 0, 0), width=1)
                out = os.path.join(HERE, f"kubler2023_fig7_force_{typ}.csv")
                write_csv(out, [
                    f"Digitised from Kubler et al. 2023 (arXiv:2305.00967v3) Fig 7, "
                    f"row 3 (force output), {TYPE_LABEL[typ]}",
                    "Series: lateral force (perpendicular to the growing axis) with "
                    "the vine tip fixed to a 6-axis force-torque sensor, vine body "
                    "at 1.75 kPa; mean of five runs",
                    "force_sd_N = half-height of the published error bar "
                    "(the paper's standard deviation), not reading error",
                    "force_N = centre of the error bar (stems crossed by other "
                    "series' lines are bridged over gaps up to 10 px)",
                    "line_check_N = dashed mean line (fitted 12-30 px either side) "
                    "minus force_N at the bar; diagnostic only, blank if the line "
                    "is not visible on both sides",
                    "Published measurements digitised by digitize_kubler2023_fig7.py; "
                    "not our measurements",
                ], ["pressure_kPa", "force_N", "force_sd_N", "line_check_N", "series"],
                    [(p, f, sd,
                      "" if d is None else f"{d * cal['y_fit_unit_per_px']:.4f}",
                      "data_mean") for p, f, sd, _x, _y, _a, _b, m, d in pts],
                    ["{:.3f}", "{:.4f}", "{:.4f}", "{}", "{}"])
                summary[f"force_{typ}"] = pts
                print(f"   force: line thickness {base:.0f} px, {len(pts)} error "
                      f"bars -> {os.path.basename(out)}; "
                      + ", ".join(f"{p:.1f}:{f:.2f}" for p, f, *_ in pts))
                for p, f, sd, _x, _y, _a, _b, m, d in pts:
                    if m != "bar_midpoint" or (d is not None and abs(d) > 2):
                        print(f"     {p:.1f} kPa: {m}, line-minus-midpoint "
                              f"{'n/a' if d is None else f'{d:+.1f}'} px")
            ov.save(os.path.join(HERE, f"verify_{row}_{typ}.png"))

    with open(os.path.join(HERE, "kubler2023_fig7_calibration.json"), "w",
              encoding="utf-8") as f:
        json.dump(calib, f, indent=1, default=float)

    bad = [k for k, v in summary.items() if k.startswith("force") and len(v) != 10]
    if bad:
        print(f"ASSERTION: expected 10 force points (paper Sec 4.4) in {bad}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
