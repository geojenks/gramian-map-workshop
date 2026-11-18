"""digitize_cherubini2015_figs.py -- digitise Figs 7 and 9 of Cherubini,
Moretti, Vertechy, Fontana 2015, "Experimental characterization of
thermally-activated artificial muscles based on coiled nylon fishing lines",
AIP Advances 5:067158 (doi:10.1063/1.4923315).

What it reads: the embedded raster images of Fig 7 (PDF page index 8) and
Fig 9 (PDF page index 9) of the PDF in source/, no rendering or resampling,
plus cherubini2015_quoted_values.csv (Table I, used only to say which
dataset each Fig 7 marker belongs to, and the legend temperatures of Fig 9).

What it writes (next to this script):
  cherubini2015_fig7.png, cherubini2015_fig9.png   the figure images, as embedded
  cherubini2015_fig7_isometric.csv                 every Fig 7 marker
  cherubini2015_fig9_curves_unloading.csv          Fig 9 right, the four curves
  cherubini2015_fig9_curves_average.csv            Fig 9 left, the four curves
  cherubini2015_fig9_triangles.csv                 Fig 9 isometric triangles (both panels)
  cherubini2015_figs_calibration.json              tick pixels, fits, residuals
  verify_fig7.png, verify_fig9_unloading.png, verify_fig9_average.png

Method
------
Calibration: per panel, the four frame lines are found as the darkest
neutral rows and columns, and the inward tick marks as short dark neutral
marks just inside the left and bottom frame lines. The frame lines are the
first and last ticks (MATLAB box). Tick values come from the frame values
and the uniform tick spacing, read off the figure (Fig 7: 20-140 degC every
20, 10-50 MPa every 10; Fig 9: 0-0.5 every 0.1, 0-80 MPa every 20). A
straight line is fitted per axis; the largest residual is asserted below
1.5 px, and the opposite frame line must land on the axis limit.

Fig 7: the red asterisks (rising temperature) and blue circles (descending
temperature) are found by template matching. The templates are the legend's
own asterisk and circle. A pixel is "red" if R - max(G, B) > 60 and "blue"
if B - max(R, G) > 60; the mask is dilated by 1 px and correlated with the
template; peaks scoring at least 0.45 (fraction of template pixels covered)
and at least 17 px apart are kept; plateau peaks within 4 px are merged.
The marker centre is the score-weighted centre of the 3 x 3 neighbourhood.
Markers in the legend box are dropped. Each marker is assigned to the
dataset (A, B, C) whose Table I line is nearest; the script asserts every
marker lies within 1.5 MPa of its own line and at least 2.5 MPa from the
others, and that each dataset has as many circles as asterisks.

Fig 9: the four curves are classed by hue (legend samples: blue 240 deg,
purple 269, maroon 331, red 0; window +/- 12 deg, saturation >= 0.35).
The filled triangles are found by a morphological opening (7 x 7) of the
coloured pixels in columns within 15 px of the three isometric strains
(Table I), which removes the 4-5 px wide lines; the triangle mask is
dilated by 3 px and excluded from the curve trace. Curves are traced row
by row (stress as the independent variable, since every curve rises
monotonically): the strain is the mean column of the curve's pixels in
that row. Rows with no pixel (gaps of dashed and dotted lines) are left
out; the builder interpolates across them and reports the gap.

Run:
    python -I digitize_cherubini2015_figs.py
"""

from __future__ import annotations

import csv
import hashlib
import json
import os
import sys

import numpy as np
import pymupdf
from matplotlib import colors as mcolors
from PIL import Image, ImageDraw
from scipy import ndimage as nd
from scipy.signal import correlate

HERE = os.path.dirname(os.path.abspath(__file__))
PDF = os.path.join(HERE, "source", "cherubini2015_aipadv067158.pdf")
PDF_SHA256 = "3e41788601c5fecb0a434c23ef849d54cdbeb53316c8a70c51a38a30325e0b90"
QUOTED = os.path.join(HERE, "cherubini2015_quoted_values.csv")

FIG7 = {"page": 8, "size": (951, 735)}
FIG9 = {"page": 9, "size": (1500, 584)}

# Axis tick values read off the figures: (value at left/bottom frame, step,
# value at right/top frame).
FIG7_X = (20.0, 20.0, 140.0)
FIG7_Y = (10.0, 10.0, 50.0)
FIG9_X = (0.0, 0.1, 0.5)
FIG9_Y = (0.0, 20.0, 80.0)

HUES = {"blue": 240.0, "purple": 269.0, "maroon": 331.0, "red": 0.0}
HUE_TOL = 12.0
SAT_MIN = 0.35


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


def quoted():
    out = {}
    with open(QUOTED, encoding="utf-8") as f:
        rows = [ln for ln in f if not ln.startswith("#")]
    for r in csv.DictReader(rows):
        out[r["key"]] = float(r["value"])
    return out


def extract(doc, page, size, name):
    imgs = doc[page].get_images(full=True)
    hit = [i for i in imgs if (i[2], i[3]) == size]
    assert len(hit) == 1, (page, imgs)
    raw = doc.extract_image(hit[0][0])
    path = os.path.join(HERE, name)
    with open(path, "wb") as f:
        f.write(raw["image"])
    im = Image.open(path).convert("RGB")
    assert im.size == size, im.size
    return im, hit[0][0]


# ------------------------------------------------------------ calibration

def line_centre(profile, lo, hi):
    """Darkness-weighted centre of the strongest line in profile[lo:hi]."""
    seg = profile[lo:hi]
    k = int(np.argmax(seg))
    a, b = max(0, k - 2), min(len(seg), k + 3)
    w = seg[a:b]
    return lo + float(np.sum(np.arange(a, b) * w) / np.sum(w))


def calibrate(rgb, win, xspec, yspec, label):
    """win = (row0, row1, col0, col1) search window holding one panel."""
    r0, r1, c0, c1 = win
    g = rgb.sum(-1)
    sat = rgb.max(-1) - rgb.min(-1)
    dark = np.where(sat < 60, 765 - g, 0).astype(float)
    sub = dark[r0:r1, c0:c1]
    colp = sub.sum(0)
    rowp = sub.sum(1)
    W, H = c1 - c0, r1 - r0
    left = c0 + line_centre(colp, 0, W // 4)
    right = c0 + line_centre(colp, 3 * W // 4, W)
    top = r0 + line_centre(rowp, 0, H // 4)
    bottom = r0 + line_centre(rowp, 3 * H // 4, H)

    nd_mask = (g < 600) & (sat < 60)
    # x ticks: inward marks in the rows 2..6 px above the bottom frame line
    br = int(round(bottom))
    strip = nd_mask[br - 6:br - 1, int(left) + 4:int(right) - 3]
    hits = strip.sum(0) >= 4
    xs = []
    for a, b in runs(hits):
        if b - a <= 3:
            cols = np.arange(a, b + 1)
            w = dark[br - 6:br - 1, int(left) + 4 + a:int(left) + 5 + b].sum(0)
            xs.append(int(left) + 4 + float(np.sum(cols * w) / np.sum(w)))
    # y ticks: inward marks in the columns 2..6 px right of the left frame
    lc = int(round(left))
    strip = nd_mask[int(top) + 4:int(bottom) - 3, lc + 2:lc + 7].T
    hits = strip.sum(0) >= 4
    ys = []
    for a, b in runs(hits):
        if b - a <= 3:
            rows = np.arange(a, b + 1)
            w = dark[int(top) + 4 + a:int(top) + 5 + b, lc + 2:lc + 7].sum(1)
            ys.append(int(top) + 4 + float(np.sum(rows * w) / np.sum(w)))

    xv0, xstep, xv1 = xspec
    yv0, ystep, yv1 = yspec
    nx = round((xv1 - xv0) / xstep)
    ny = round((yv1 - yv0) / ystep)
    sx = (right - left) / nx
    sy = (bottom - top) / ny
    x_px, xvals, y_px, yvals = [left], [xv0], [bottom], [yv0]
    for p in xs:
        k = (p - left) / sx
        if abs(k - round(k)) < 0.1 and 0 < round(k) < nx:
            x_px.append(p)
            xvals.append(xv0 + xstep * round(k))
    for p in ys:
        k = (bottom - p) / sy
        if abs(k - round(k)) < 0.1 and 0 < round(k) < ny:
            y_px.append(p)
            yvals.append(yv0 + ystep * round(k))
    x_px.append(right)
    xvals.append(xv1)
    y_px.append(top)
    yvals.append(yv1)
    fx = np.polyfit(x_px, xvals, 1)
    fy = np.polyfit(y_px, yvals, 1)
    res_x = np.array(x_px) - (np.array(xvals) - fx[1]) / fx[0]
    res_y = np.array(y_px) - (np.array(yvals) - fy[1]) / fy[0]
    cal = {
        "panel": label,
        "frame_px": {"left_col": left, "right_col": right,
                     "top_row": top, "bottom_row": bottom},
        "x_ticks_px": [round(p, 2) for p in x_px], "x_ticks_value": xvals,
        "y_ticks_px": [round(p, 2) for p in y_px], "y_ticks_value": yvals,
        "x_fit_unit_per_px": fx[0], "x_fit_offset": fx[1],
        "y_fit_unit_per_px": fy[0], "y_fit_offset": fy[1],
        "x_max_residual_px": float(np.abs(res_x).max()),
        "y_max_residual_px": float(np.abs(res_y).max()),
    }
    # every interior tick must have been found
    assert len(x_px) == nx + 1 and len(y_px) == ny + 1, cal
    assert cal["x_max_residual_px"] < 1.5 and cal["y_max_residual_px"] < 1.5, cal
    return cal


def to_data(cal, col, row):
    return (cal["x_fit_unit_per_px"] * col + cal["x_fit_offset"],
            cal["y_fit_unit_per_px"] * row + cal["y_fit_offset"])


def to_px(cal, x, y):
    return ((x - cal["x_fit_offset"]) / cal["x_fit_unit_per_px"],
            (y - cal["y_fit_offset"]) / cal["y_fit_unit_per_px"])


def legend_box(rgb, cal):
    """Legend box: the long dark neutral rows inside the plot (top and bottom
    edges of the box) and the extent of those runs."""
    g = rgb.sum(-1)
    sat = rgb.max(-1) - rgb.min(-1)
    m = (g < 600) & (sat < 60)
    f = cal["frame_px"]
    l, r = int(f["left_col"]) + 3, int(f["right_col"]) - 3
    t, b = int(f["top_row"]) + 3, int(f["bottom_row"]) - 3
    rows = []
    for rr in range(t, t + (b - t) // 2):
        rs = [x for x in runs(m[rr, l:r]) if x[1] - x[0] >= 120]
        if rs:
            rows.append((rr, l + rs[0][0], l + rs[0][1]))
    assert len(rows) >= 2, "legend box not found"
    return [min(x[1] for x in rows), min(x[0] for x in rows),
            max(x[2] for x in rows), max(x[0] for x in rows)]


# ------------------------------------------------------------------ Fig 7

def match_markers(mask, tmpl, origin, thr, keep):
    """Marker centres (row, col, score). The centre is the centre of the
    template's bounding box: the offset between the score peak and that
    centre is measured by matching the legend marker itself."""
    s = correlate(nd.binary_dilation(mask).astype(float), tmpl,
                  mode="same") / tmpl.sum()
    h, w = tmpl.shape
    cr, cc = origin[0] + (h - 1) / 2.0, origin[1] + (w - 1) / 2.0
    r0, c0 = int(cr) - 4, int(cc) - 4
    loc = s[r0:r0 + 10, c0:c0 + 10]
    pr, pc = np.unravel_index(int(np.argmax(loc)), loc.shape)
    off = (r0 + pr - cr, c0 + pc - cc)
    assert abs(off[0]) <= 1.0 and abs(off[1]) <= 1.0, off
    mx = nd.maximum_filter(s, size=17)
    pk = [tuple(p) for p in np.argwhere((s == mx) & (s >= thr)) if keep(*p)]
    # merge plateau peaks within 4 px
    groups = []
    for p in sorted(pk):
        for gp in groups:
            if abs(gp[0][0] - p[0]) <= 4 and abs(gp[0][1] - p[1]) <= 4:
                gp.append(p)
                break
        else:
            groups.append([p])
    out = []
    for gp in groups:
        r, c = np.mean(gp, axis=0)
        r, c = int(round(r)), int(round(c))
        w = s[r - 1:r + 2, c - 1:c + 2]
        rr, cc = np.mgrid[r - 1:r + 2, c - 1:c + 2]
        out.append((float((rr * w).sum() / w.sum()) - off[0],
                    float((cc * w).sum() / w.sum()) - off[1], float(s[r, c])))
    return out, off


def legend_template(mask, box, rows):
    """The largest component of mask inside the legend box, within rows."""
    l, t, r, b = box
    sub = mask[rows[0]:rows[1], l:l + 120]
    lab, n = nd.label(nd.binary_closing(sub, np.ones((3, 3))) | sub,
                      structure=np.ones((3, 3)))
    sizes = nd.sum(sub, lab, range(1, n + 1))
    k = 1 + int(np.argmax(sizes))
    sl = nd.find_objects((lab == k).astype(int))[0]
    return sub[sl].astype(float), (rows[0] + sl[0].start, l + sl[1].start)


def digitise_fig7(im, q, calib):
    rgb = np.asarray(im).astype(int)
    H, W = rgb.shape[:2]
    cal = calibrate(rgb, (0, H - 60, 60, W - 10), FIG7_X, FIG7_Y, "Fig 7")
    box = legend_box(rgb, cal)
    cal["legend_box_px"] = box
    R, G, B = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    red = (R - np.maximum(G, B)) > 60
    blue = (B - np.maximum(R, G)) > 60
    # legend rows: asterisk entry in the top third of the box, circle in the
    # middle third
    l, t, r, b = box
    third = (b - t) // 3
    t_ast, o_ast = legend_template(red, box, (t + 2, t + third))
    t_cir, o_cir = legend_template(blue, box, (t + third, t + 2 * third))
    f = cal["frame_px"]

    def keep(row, col):
        inside = (f["top_row"] < row < f["bottom_row"]
                  and f["left_col"] < col < f["right_col"])
        in_legend = (l - 5 <= col <= r + 5) and (t - 5 <= row <= b + 5)
        return inside and not in_legend

    ast, off_a = match_markers(red, t_ast, o_ast, 0.45, keep)
    cir, off_c = match_markers(blue, t_cir, o_cir, 0.45, keep)
    found = {"rising": ast, "descending": cir}
    lines = {k: (q[f"tableI_alpha_{k}"], q[f"tableI_beta_{k}"],
                 q[f"isometric_deformation_{k}"]) for k in "ABC"}
    rows = []
    for series, pts in found.items():
        for row, col, score in pts:
            T, s = to_data(cal, col, row)
            d = {k: s - (a * T + b_) for k, (a, b_, _) in lines.items()}
            ds = sorted(d, key=lambda k: abs(d[k]))
            own, other = ds[0], ds[1]
            assert abs(d[own]) < 1.5 and abs(d[other]) > 2.5, (series, T, s, d)
            rows.append((own, lines[own][2], series, T, s, d[own], col, row,
                         score))
    rows.sort(key=lambda x: (x[0], x[2] != "rising", x[3]))
    counts = {}
    for x in rows:
        counts[(x[0], x[2])] = counts.get((x[0], x[2]), 0) + 1
    for k in "ABC":
        assert counts.get((k, "rising"), 0) == counts.get((k, "descending"), 0), counts
    calib["panels"]["fig7"] = cal
    calib["fig7_templates"] = {
        "asterisk_px": list(t_ast.shape), "asterisk_origin_px": list(o_ast),
        "asterisk_pixels": int(t_ast.sum()),
        "circle_px": list(t_cir.shape), "circle_origin_px": list(o_cir),
        "circle_pixels": int(t_cir.sum()),
        "peak_minus_bbox_centre_px": {"asterisk": list(off_a),
                                      "circle": list(off_c)}}

    out = os.path.join(HERE, "cherubini2015_fig7_isometric.csv")
    write_csv(out, [
        "Digitised from Cherubini et al. 2015, AIP Advances 5:067158, Fig 7 "
        "(printed page 067158-8): nominal stress against chamber temperature, "
        "isometric tests on the Reference Coil, second thermal cycle",
        "dataset A/B/C = held at 10/15/20 mm elongation (14.3/21.4/28.6 % of "
        "L0 = 70 mm); series rising = red asterisks, descending = blue circles",
        "T_degC and stress_MPa are the marker centres; off_tableI_MPa = "
        "stress minus the paper's Table I line at T (diagnostic only)",
        "score = fraction of the legend marker's pixels matched (template "
        "matching); x_px, y_px = marker centre in cherubini2015_fig7.png",
        "Published measurements digitised by digitize_cherubini2015_figs.py; "
        "not our measurements",
    ], ["dataset", "deformation_pct", "series", "T_degC", "stress_MPa",
        "off_tableI_MPa", "x_px", "y_px", "score"], rows,
        ["{}", "{}", "{}", "{:.3f}", "{:.4f}", "{:+.4f}", "{:.2f}", "{:.2f}",
         "{:.3f}"])

    ov = im.copy()
    dr = ImageDraw.Draw(ov)
    for ds, _p, series, T, s, _o, col, row, _sc in rows:
        if series == "rising":
            dr.line((col - 6, row, col + 6, row), fill=(0, 200, 0), width=2)
            dr.line((col, row - 6, col, row + 6), fill=(0, 200, 0), width=2)
        else:
            dr.ellipse((col - 3, row - 3, col + 3, row + 3), fill=(255, 140, 0))
    for k, (a, b_, _) in lines.items():
        x0, y0 = to_px(cal, 30.0, a * 30.0 + b_)
        x1, y1 = to_px(cal, 125.0, a * 125.0 + b_)
        dr.line((x0, y0, x1, y1), fill=(0, 200, 255), width=1)
    dr.rectangle(box, outline=(255, 0, 255))
    ov.save(os.path.join(HERE, "verify_fig7.png"))
    print(f"[Fig 7] ticks x {cal['x_ticks_px']} y {cal['y_ticks_px']} max resid "
          f"{cal['x_max_residual_px']:.2f}/{cal['y_max_residual_px']:.2f} px; "
          f"markers {dict((f'{a}-{b}', n) for (a, b), n in sorted(counts.items()))}")
    return rows


# ------------------------------------------------------------------ Fig 9

def hue_masks(rgb):
    hsv = mcolors.rgb_to_hsv(rgb / 255.0)
    h = hsv[..., 0] * 360.0
    ok = hsv[..., 1] >= SAT_MIN
    out = {}
    for name, h0 in HUES.items():
        dh = np.abs((h - h0 + 180.0) % 360.0 - 180.0)
        out[name] = ok & (dh <= HUE_TOL)
    return out


def digitise_fig9_panel(rgb, masks, win, label, strains, temps, calib):
    cal = calibrate(rgb, win, FIG9_X, FIG9_Y, label)
    box = legend_box(rgb, cal)
    cal["legend_box_px"] = box
    f = cal["frame_px"]
    l, t, r, b = box
    # legend order: one line sample per entry, top to bottom
    order = []
    for name, m in masks.items():
        rr = np.flatnonzero(m[t:b, l:l + 60].sum(1) > 20)
        order.append((t + float(rr.mean()) if rr.size else 1e9, name))
    order = [n for _, n in sorted(order)]
    assert order == list(HUES), order
    colour_T = dict(zip(order, temps))

    anyc = np.zeros(rgb.shape[:2], bool)
    for m in masks.values():
        anyc |= m
    plot = np.zeros_like(anyc)
    plot[int(f["top_row"]) + 2:int(f["bottom_row"]) - 1,
         int(f["left_col"]) + 2:int(f["right_col"]) - 1] = True
    plot[t - 3:b + 4, l - 3:r + 4] = False
    near = np.zeros_like(anyc)
    for e in strains:
        c, _ = to_px(cal, e, 0.0)
        near[:, int(c) - 15:int(c) + 16] = True
    tri = nd.binary_opening(anyc & plot & near, structure=np.ones((7, 7)))
    lab, n = nd.label(tri)
    tris = []
    for k, sl in enumerate(nd.find_objects(lab), start=1):
        rr, cc = sl
        row = 0.5 * (rr.start + rr.stop - 1)
        col = 0.5 * (cc.start + cc.stop - 1)
        votes = {name: int((m[sl] & (lab[sl] == k)).sum()) for name, m in masks.items()}
        name = max(votes, key=votes.get)
        e, s = to_data(cal, col, row)
        tris.append((label, colour_T[name], name, e, s, col, row,
                     rr.stop - rr.start, cc.stop - cc.start))
    tri_excl = nd.binary_dilation(tri, iterations=3)

    curves = []
    for name, m in masks.items():
        mm = m & plot & ~tri_excl
        for row in range(int(f["top_row"]) + 2, int(f["bottom_row"]) - 1):
            cols = np.flatnonzero(mm[row])
            if cols.size == 0:
                continue
            # one curve crossing per row: keep the run nearest the median
            rs = runs(mm[row], gap=2)
            med = float(np.median(cols))
            a, bb = min(rs, key=lambda x: min(abs(x[0] - med), abs(x[1] - med)))
            col = 0.5 * (a + bb)
            e, s = to_data(cal, col, row)
            curves.append((colour_T[name], name, s, e, row, col, bb - a + 1,
                           len(rs)))
    calib["panels"][label] = cal
    print(f"[{label}] ticks x {cal['x_ticks_px']} y {cal['y_ticks_px']} max resid "
          f"{cal['x_max_residual_px']:.2f}/{cal['y_max_residual_px']:.2f} px; "
          f"{len(tris)} triangles; legend order {order}")
    return cal, curves, tris, tri


def digitise_fig9(im, q, calib):
    rgb = np.asarray(im).astype(int)
    H, W = rgb.shape[:2]
    masks = hue_masks(rgb)
    strains = [q[f"isometric_deformation_{k}"] / 100.0 for k in "ABC"]
    temps = [q[f"isothermal_temperature_{i}"] for i in (1, 2, 3, 4)]
    results = {}
    for key, label, win in (("average", "Fig 9 left (loading/unloading average)",
                             (30, H - 60, 50, W // 2 + 10)),
                            ("unloading", "Fig 9 right (unloading)",
                             (30, H - 60, W // 2 + 60, W))):
        cal, curves, tris, trimask = digitise_fig9_panel(
            rgb, masks, win, label, strains, temps, calib)
        results[key] = (cal, curves, tris)
        curves.sort(key=lambda x: (x[0], -x[4]))
        out = os.path.join(HERE, f"cherubini2015_fig9_curves_{key}.csv")
        write_csv(out, [
            f"Digitised from Cherubini et al. 2015, AIP Advances 5:067158, "
            f"{label} (printed page 067158-9): nominal stress against "
            f"deformation dL/L0 of the Reference Coil, isothermal tests",
            "Traced row by row: one row per image pixel row that carries the "
            "curve's colour; strain = centre of the curve's pixel run in that "
            "row; rows without a pixel (dash gaps, triangles, legend) are absent",
            "colour = line colour in the figure (legend: 25 degC blue solid, "
            "60 purple dashed, 88 maroon dash-dot, 122 red dotted)",
            "run_px = width of the run used; n_runs = runs of that colour in "
            "the row (above 1 means another piece of the same colour shared "
            "the row)",
            "Published measurements digitised by digitize_cherubini2015_figs.py; "
            "not our measurements",
        ], ["T_degC", "colour", "stress_MPa", "strain", "y_px", "x_px",
            "run_px", "n_runs"], curves,
            ["{:g}", "{}", "{:.4f}", "{:.5f}", "{:d}", "{:.1f}", "{:d}", "{:d}"])
        ov = im.copy()
        dr = ImageDraw.Draw(ov)
        for _T, _n, _s, _e, row, col, _w, _k in curves:
            dr.point((col, row), fill=(0, 220, 0))
        for *_x, col, row, h, w in tris:
            dr.rectangle((col - w / 2, row - h / 2, col + w / 2, row + h / 2),
                         outline=(0, 200, 0))
        dr.rectangle(cal["legend_box_px"], outline=(255, 0, 255))
        f = cal["frame_px"]
        crop = (int(f["left_col"]) - 10, int(f["top_row"]) - 10,
                int(f["right_col"]) + 10, int(f["bottom_row"]) + 10)
        ov.crop(crop).save(os.path.join(HERE, f"verify_fig9_{key}.png"))
    tris = sorted(results["average"][2] + results["unloading"][2],
                  key=lambda x: (x[0], x[3], x[1]))
    write_csv(os.path.join(HERE, "cherubini2015_fig9_triangles.csv"), [
        "Digitised from Cherubini et al. 2015, AIP Advances 5:067158, Fig 9 "
        "(printed page 067158-9): filled triangles = isometric stress "
        "interpolated by the authors at 25-60-88-122 degC",
        "Triangle centre = centre of its bounding box (after a 7 x 7 opening); "
        "used only as a check, no record value comes from this file",
        "Published measurements digitised by digitize_cherubini2015_figs.py; "
        "not our measurements",
    ], ["panel", "T_degC", "colour", "strain", "stress_MPa", "x_px", "y_px",
        "h_px", "w_px"], tris,
        ["{}", "{:g}", "{}", "{:.4f}", "{:.3f}", "{:.1f}", "{:.1f}", "{:d}",
         "{:d}"])
    return results


# ------------------------------------------------------------------ main

def write_csv(path, header_lines, cols, rows, fmt):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        for ln in header_lines:
            f.write("# " + ln + "\n")
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join(fm.format(v) for fm, v in zip(fmt, r)) + "\n")


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    got = sha256(PDF)
    assert got == PDF_SHA256, f"PDF hash changed: {got}"
    q = quoted()
    doc = pymupdf.open(PDF)
    im7, x7 = extract(doc, FIG7["page"], FIG7["size"], "cherubini2015_fig7.png")
    im9, x9 = extract(doc, FIG9["page"], FIG9["size"], "cherubini2015_fig9.png")
    print(f"Fig 7 image {im7.size}, xref {x7}; Fig 9 image {im9.size}, xref {x9}")
    calib = {"pdf": os.path.basename(PDF), "pdf_sha256": got,
             "fig7": {"page_index": FIG7["page"], "xref": x7,
                      "image_size_px": list(im7.size)},
             "fig9": {"page_index": FIG9["page"], "xref": x9,
                      "image_size_px": list(im9.size)},
             "panels": {}}
    digitise_fig7(im7, q, calib)
    digitise_fig9(im9, q, calib)
    with open(os.path.join(HERE, "cherubini2015_figs_calibration.json"), "w",
              encoding="utf-8") as f:
        json.dump(calib, f, indent=1, default=float)
    return 0


if __name__ == "__main__":
    sys.exit(main())
