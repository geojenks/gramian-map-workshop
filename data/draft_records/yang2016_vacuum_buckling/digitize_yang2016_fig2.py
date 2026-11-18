"""digitize_yang2016_fig2.py -- digitise Fig 2A-D of Yang et al. 2016,
"Buckling Pneumatic Linear Actuators Inspired by Muscle", Adv. Mater.
Technol. 1, 1600055 (doi:10.1002/admt.201600055).

What it reads: the single embedded raster of Fig 2 (2100 x 1155 px) on PDF
page index 19 of the Harvard DASH accepted manuscript in source/. No
rendering or resampling.

What it writes (next to this script):
  yang2016_fig2.png                       the figure image, as embedded
  yang2016_fig2A_free_ecoflex.csv         no-load contraction vs vacuum
  yang2016_fig2B_free_elastosil.csv
  yang2016_fig2C_load_elastosil.csv       load vs contraction at dP = 90 kPa
  yang2016_fig2D_load_ecoflex.csv
  yang2016_fig2_calibration.json          tick pixels, fits, residuals,
                                          axis cross-checks
  verify_fig2<panel>.png                  picked points over each panel

Panel pairing is by the label printed inside each panel, not by position:
A Ecoflex, B Elastosil, C Elastosil, D Ecoflex.

Method
------
Calibration. Each panel is a closed black frame with outward ticks on all
four sides: long major ticks (about 27 px) and short minor ticks (about
14 px). The frame lines are found as the longest dark runs in the panel
window. Ticks are found in a strip just outside each frame side; ticks of
16 px or more are majors. Major-tick values are the printed labels, read
off the figure and listed in PANELS below; the count of majors found is
asserted to equal the count of labels. A straight line is fitted per axis
(major ticks only) and the maximum residual is asserted below 1.5 px.
Minor ticks are checked to fall half-way between majors.

Each panel has two scales per direction: strain s (bottom) and length
change in mm (top); dP or loading stress in kPa (left) and force in N
(right). Displacement is read from the top mm axis and force from the
right N axis, so no length or area is typed in. As a cross-check, mm per
unit strain must match the 46.5 mm device length, and N per kPa must match
the 34 mm x 28 mm cross-section (yang2016_text_values.csv); both within 3%.

Markers. Each data point is a filled black square (about 14 px) with a
horizontal error bar (SD of strain, n = 7 samples). Squares are isolated
by a binary opening with a 9 x 9 structuring element, which removes the
frame, bars, caps, tick labels and text strokes. Remaining blobs must be
square (bounding-box sides within 3 px of the median marker size, fill
above 85%); this rejects the solid arrowhead next to dP_crit. The count is
asserted per panel (11 in each, counted on the figure). The marker centre
is the centroid of the opened blob.

Error bars. Along the marker's centre rows, the dark bar is followed
outwards from the marker edge to its last dark column; the cap is the run
of end columns whose dark vertical extent is 8 px or more, and its centre
gives the bar end. The SD is half the distance between the two cap
centres. If no bar shows outside the marker, the SD is smaller than half
a marker and is left blank, with that bound in sd_upper_bound.

Run:
    python -I digitize_yang2016_fig2.py
"""

from __future__ import annotations

import csv
import hashlib
import json
import os
import sys

import numpy as np
import pymupdf
from PIL import Image, ImageDraw
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
PDF = os.path.join(HERE, "source", "yang2016_admt201600055_dash.pdf")
PDF_SHA256 = "9bbcdacf2472bcec6dddf40638c679a9447c08dbfc82df7e79f9080ad8cc9f0d"
FIG_PAGE_INDEX = 19          # 0-based page carrying Fig 2
FIG_SIZE = (2100, 1155)
TEXT_VALUES = os.path.join(HERE, "yang2016_text_values.csv")

DARK = 300                   # sum of RGB below this = ink
MAJOR_MIN_PX = 16            # majors ~27 px, minors ~14 px
STRIP_PX = 34                # how far outside the frame to look for ticks
OPEN_PX = 9                  # structuring element for marker isolation
CAP_RUN_PX = 24              # vertical ink run at least this tall = error-bar cap
MAX_RESID_PX = 1.5
CROSS_CHECK_TOL = 0.03

_STRAIN = [-0.5, -0.4, -0.3, -0.2, -0.1, 0.0]
PANELS = {
    # window = (x0, y0, x1, y1) in the embedded image; only localises search
    "A": dict(window=(60, 0, 640, 560), material="ecoflex", kind="free",
              bottom=_STRAIN, top=[-20, -15, -10, -5, 0],
              left=[0, 1, 2, 3], right=[0, 1, 2, 3], n_points=11,
              left_name="dP_kPa", right_name="F_N"),
    "B": dict(window=(60, 560, 640, 1155), material="elastosil", kind="free",
              bottom=_STRAIN, top=[-20, -15, -10, -5, 0],
              left=[0, 10, 20, 30], right=[0, 10, 20, 30], n_points=11,
              left_name="dP_kPa", right_name="F_N"),
    "C": dict(window=(700, 0, 1330, 560), material="elastosil", kind="load",
              bottom=_STRAIN, top=[-25, -20, -15, -10, -5, 0],
              left=[0, 2, 4, 6, 8, 10], right=[0, 2, 4, 6, 8, 10],
              n_points=11, left_name="sigma_kPa", right_name="T_N"),
    "D": dict(window=(700, 560, 1330, 1155), material="ecoflex", kind="load",
              bottom=_STRAIN, top=[-25, -20, -15, -10, -5, 0],
              left=[0, 0.2, 0.4, 0.6, 0.8, 1.0],
              right=[0, 0.2, 0.4, 0.6, 0.8, 1.0], n_points=11,
              left_name="sigma_kPa", right_name="T_N"),
}
MATERIAL_LABEL = {"ecoflex": "Ecoflex 00-30 (E = 43 kPa)",
                  "elastosil": "Elastosil M4601 (E = 520 kPa)"}


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


def text_values():
    with open(TEXT_VALUES, encoding="utf-8") as f:
        rows = list(csv.DictReader(ln for ln in f if not ln.startswith("#")))
    return {r["quantity"]: float(r["value"]) for r in rows}


# ------------------------------------------------------------ calibration

def find_frame(dark, win):
    x0, y0, x1, y1 = win
    sub = dark[y0:y1, x0:x1]
    rowsum, colsum = sub.sum(1), sub.sum(0)
    # frame sides: the two groups of rows / columns with the most ink
    def two_lines(s):
        thr = 0.6 * s.max()
        groups = runs(s >= thr, gap=1)
        assert len(groups) >= 2, groups
        g = sorted(groups, key=lambda r: -s[r[0]:r[1] + 1].max())[:2]
        g = sorted(g)
        return g[0], g[1]
    (ta, tb), (ba, bb) = two_lines(rowsum)
    (la, lb), (ra, rb) = two_lines(colsum)
    return {"top": (y0 + ta, y0 + tb), "bottom": (y0 + ba, y0 + bb),
            "left": (x0 + la, x0 + lb), "right": (x0 + ra, x0 + rb)}


def ticks_on_side(dark, fr, side):
    """Tick centres (px along the side) and lengths outside the frame."""
    # include the frame lines themselves: an end tick can sit on a corner
    # (e.g. strain -0.5 is the left frame edge, extended outward)
    l0, l1 = fr["left"][0], fr["right"][1] + 1
    t0, t1 = fr["top"][0], fr["bottom"][1] + 1
    if side in ("bottom", "top"):
        if side == "bottom":
            strip = dark[fr["bottom"][1] + 1:fr["bottom"][1] + 1 + STRIP_PX, l0:l1]
        else:
            strip = dark[fr["top"][0] - STRIP_PX:fr["top"][0], l0:l1][::-1]
        off = l0
    else:
        if side == "left":
            strip = dark[t0:t1, fr["left"][0] - STRIP_PX:fr["left"][0]][:, ::-1].T
        else:
            strip = dark[t0:t1, fr["right"][1] + 1:fr["right"][1] + 1 + STRIP_PX].T
        off = t0
    # length of the ink run that starts at the frame, per position
    length = np.zeros(strip.shape[1], int)
    for j in range(strip.shape[1]):
        n = 0
        while n < strip.shape[0] and strip[n, j]:
            n += 1
        length[j] = n
    out = []
    for a, b in runs(length >= 8):
        out.append((off + 0.5 * (a + b), int(length[a:b + 1].max())))
    return out


def fit_axis(ticks, labels, side, decreasing_px):
    majors = [p for p, n in ticks if n >= MAJOR_MIN_PX]
    minors = [p for p, n in ticks if n < MAJOR_MIN_PX]
    assert len(majors) == len(labels), (side, majors, labels)
    px = sorted(majors, reverse=decreasing_px)       # increasing value order
    vals = list(labels)
    fit = np.polyfit(px, vals, 1)
    resid = np.array(px) - (np.array(vals) - fit[1]) / fit[0]
    step = vals[1] - vals[0]
    minor_frac = [float(((np.polyval(fit, p) - vals[0]) / step) % 1.0)
                  for p in minors]
    return {"major_px": [round(p, 1) for p in px], "major_values": vals,
            "minor_px": [round(p, 1) for p in minors],
            "minor_fraction_of_step": [round(f, 3) for f in minor_frac],
            "slope_per_px": float(fit[0]), "offset": float(fit[1]),
            "max_residual_px": float(np.abs(resid).max())}


def calibrate(dark, key):
    spec = PANELS[key]
    fr = find_frame(dark, spec["window"])
    cal = {"panel": key, "frame": {k: [int(a), int(b)] for k, (a, b) in fr.items()},
           "axes": {}}
    for side, decreasing in (("bottom", False), ("top", False),
                             ("left", True), ("right", True)):
        t = ticks_on_side(dark, fr, side)
        ax = fit_axis(t, spec[side], side, decreasing)
        assert ax["max_residual_px"] < MAX_RESID_PX, (key, side, ax)
        assert all(abs(f - 0.5) < 0.05 for f in ax["minor_fraction_of_step"]), \
            (key, side, ax)
        cal["axes"][side] = ax
    return cal


def val(ax, px):
    return float(np.polyval([ax["slope_per_px"], ax["offset"]], px))


def px_of(ax, v):
    return (v - ax["offset"]) / ax["slope_per_px"]


# ---------------------------------------------------------------- markers

def find_markers(dark, cal, n_expected):
    fr = cal["frame"]
    x0, x1 = fr["left"][1] + 3, fr["right"][0] - 3
    y0, y1 = fr["top"][1] + 3, fr["bottom"][0] - 3
    sub = dark[y0:y1, x0:x1]
    op = ndimage.binary_opening(sub, structure=np.ones((OPEN_PX, OPEN_PX)))
    lab, n = ndimage.label(op)
    blobs = []
    for i, sl in enumerate(ndimage.find_objects(lab), start=1):
        h = sl[0].stop - sl[0].start
        w = sl[1].stop - sl[1].start
        area = int((lab[sl] == i).sum())
        blobs.append(dict(h=h, w=w, fill=area / (h * w), sl=sl, i=i))
    size = np.median([b["h"] for b in blobs])
    keep, rejected = [], []
    for b in blobs:
        sl = b["sl"]
        reason = None
        if abs(b["h"] - size) > 3 or b["fill"] <= 0.85:
            reason = "shape"
        else:
            # Raw vertical ink run through the blob's centre row, per column.
            # Marker columns run about the marker height; error-bar cap
            # columns run >= CAP_RUN_PX. Anything in between is not a marker
            # (the dPcrit arrowhead has irregular runs of 15-20 px).
            # measured on the full image: a cap near the frame can reach
            # past the interior crop used for the opening
            rc = y0 + sl[0].start + b["h"] // 2
            cols = list(range(sl[1].start, sl[1].stop))
            run_h = []
            for c in cols:
                col = dark[:, x0 + c]
                rr = [bb - aa + 1 for aa, bb in runs(col) if aa <= rc <= bb]
                run_h.append(rr[0] if rr else 0)
            run_h = np.array(run_h)
            tall = run_h > size + 2
            if np.any(tall & (run_h < CAP_RUN_PX)):
                reason = "irregular"
            else:
                # trim cap columns at the blob ends only
                i0, i1 = 0, len(cols) - 1
                while i0 <= i1 and tall[i0]:
                    i0 += 1
                while i1 >= i0 and tall[i1]:
                    i1 -= 1
                wt = i1 - i0 + 1
                cut_l, cut_r = i0 > 0, i1 < len(cols) - 1
                # A cap may overlap the marker's edge columns, so a trimmed
                # side can be up to 4 px narrower than the marker.
                lo = size - 2 - 4 * (cut_l + cut_r)
                if not (lo <= wt <= size + 2):
                    reason = "width"
                else:
                    b["c0"], b["c1"] = cols[i0], cols[i1]
                    # centre: both edges clean or both trimmed -> midpoint;
                    # one side trimmed -> clean edge plus half a marker.
                    if cut_l and not cut_r:
                        b["cx"] = cols[i1] - (size - 1) / 2.0
                    elif cut_r and not cut_l:
                        b["cx"] = cols[i0] + (size - 1) / 2.0
                    else:
                        b["cx"] = (cols[i0] + cols[i1]) / 2.0
                    b["trim"] = ("L" if cut_l else "") + ("R" if cut_r else "")
                    # error-bar caps that touch or sit inside the marker
                    b["caps"] = [x0 + (cols[a] + cols[e]) / 2.0
                                 for a, e in runs(run_h >= CAP_RUN_PX)]
        b["reason"] = reason
        (keep if reason is None else rejected).append(b)
    pts = []
    for b in keep:
        sl = b["sl"]
        c0, c1 = b["c0"], b["c1"]
        m = lab[sl[0], c0:c1 + 1] == b["i"]
        rows = np.nonzero(m.any(1))[0]
        cy = sl[0].start + (rows[0] + rows[-1]) / 2.0
        cx = b["cx"]
        pts.append(dict(xc=x0 + cx, yc=y0 + cy, h=b["h"], w=c1 - c0 + 1,
                        trim=b["trim"], caps=b["caps"],
                        left=x0 + c0, right=x0 + c1,
                        top=y0 + sl[0].start, bottom=y0 + sl[0].stop - 1))
    rejected = [dict(h=b["h"], w=b["w"], fill=b["fill"], reason=b["reason"],
                     x=x0 + b["sl"][1].start, y=y0 + b["sl"][0].start)
                for b in rejected]
    assert len(pts) == n_expected, (cal["panel"], len(pts), rejected)
    return pts, float(size), rejected


def error_bar(dark, p, cal):
    """Cap centres left and right of a marker along its centre rows."""
    fr = cal["frame"]
    # Caps touching or inside the marker were found as tall ink columns
    # within the marker blob: one each side of the centre gives the bar.
    capl = [c for c in p["caps"] if c < p["xc"] - 0.5]
    capr = [c for c in p["caps"] if c > p["xc"] + 0.5]
    if len(capl) == 1 and len(capr) == 1:
        return [capl[0], capr[0]]
    r = int(round(p["yc"]))
    band = dark[r - 1:r + 2, :].any(0)               # bar is ~3 px thick

    def walk(start, step, stop):
        j, last = start, None
        while (j - stop) * step < 0 and band[j]:
            last = j
            j += step
        return last

    ends = []
    for start, step, stop in ((p["left"] - 1, -1, fr["left"][1] + 1),
                              (p["right"] + 1, +1, fr["right"][0] - 1)):
        last = walk(start, step, stop)
        if last is None or abs(last - start) < 2:
            ends.append(None)
            continue
        # cap: end columns whose vertical ink around r spans >= 8 px
        cap = []
        j = last
        while (j - start) * step >= 0:
            col = dark[r - 14:r + 15, j]
            if sum(b - a + 1 for a, b in runs(col) if a <= 14 <= b) >= 8:
                cap.append(j)
                j -= step
            else:
                break
        ends.append(float(np.mean(cap)) if cap else None)
    return ends


def bar_reach_px(dark, p, cal):
    """Longest run of bar ink (centre rows) leaving the marker sideways, px."""
    fr = cal["frame"]
    r = int(round(p["yc"]))
    band = dark[r - 1:r + 2, :].any(0)
    reach = 0
    # stop at the frame's inner edge so the frame line is never counted
    for start, step, stop in ((p["left"] - 1, -1, fr["left"][1]),
                              (p["right"] + 1, +1, fr["right"][0])):
        j, n = start, 0
        while (j - stop) * step < 0 and band[j]:
            n += 1
            j += step
        reach = max(reach, n)
    return reach


# ------------------------------------------------------------------ main

def write_csv(path, header_lines, cols, rows):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        for ln in header_lines:
            f.write("# " + ln + "\n")
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join("" if v is None else
                             (f"{v:.5g}" if isinstance(v, float) else str(v))
                             for v in r) + "\n")


def main():
    got = sha256(PDF)
    assert got == PDF_SHA256, f"PDF hash changed: {got}"
    tv = text_values()
    doc = pymupdf.open(PDF)
    imgs = [x for x in doc[FIG_PAGE_INDEX].get_images(full=True)
            if (x[2], x[3]) == FIG_SIZE]
    assert len(imgs) == 1, doc[FIG_PAGE_INDEX].get_images(full=True)
    raw = doc.extract_image(imgs[0][0])
    fig_png = os.path.join(HERE, "yang2016_fig2.png")
    with open(fig_png, "wb") as f:
        f.write(raw["image"])
    im = Image.open(fig_png).convert("RGB")
    rgb = np.asarray(im).astype(int)
    dark = rgb.sum(-1) < DARK
    print(f"Fig 2 image {im.size[0]} x {im.size[1]} px, xref {imgs[0][0]}")

    calib = {"pdf": os.path.basename(PDF), "pdf_sha256": got,
             "page_index": FIG_PAGE_INDEX, "image_size_px": list(im.size),
             "panels": {}}
    L0_mm = tv["device_length"]
    area_mm2 = tv["device_width"] * tv["device_depth"]
    for key, spec in PANELS.items():
        cal = calibrate(dark, key)
        ax = cal["axes"]
        # Cross-checks between the paired scales.
        mm_per_strain = ax["top"]["slope_per_px"] / ax["bottom"]["slope_per_px"]
        N_per_kPa = ax["right"]["slope_per_px"] / ax["left"]["slope_per_px"]
        # zero of top axis vs zero of bottom axis (px)
        zero_shift_x = px_of(ax["top"], 0.0) - px_of(ax["bottom"], 0.0)
        zero_shift_y = px_of(ax["right"], 0.0) - px_of(ax["left"], 0.0)
        cal["cross_check"] = {
            "mm_per_unit_strain": mm_per_strain,
            "text_length_mm": L0_mm,
            "N_per_kPa": N_per_kPa,
            "text_area_mm2_times_1e-3": area_mm2 * 1e-3,
            "top_zero_minus_bottom_zero_px": zero_shift_x,
            "right_zero_minus_left_zero_px": zero_shift_y}
        assert abs(mm_per_strain / L0_mm - 1) < CROSS_CHECK_TOL, cal["cross_check"]
        assert abs(N_per_kPa / (area_mm2 * 1e-3) - 1) < CROSS_CHECK_TOL, cal["cross_check"]
        assert abs(zero_shift_x) < 2 and abs(zero_shift_y) < 2, cal["cross_check"]

        pts, size, rejected = find_markers(dark, cal, spec["n_points"])
        cal["marker_size_px"] = size
        cal["rejected_blobs"] = rejected
        print(f"[{key} {spec['material']}] frame {cal['frame']}; max resid "
              + ", ".join(f"{s} {a['max_residual_px']:.2f}" for s, a in ax.items())
              + f" px; mm/strain {mm_per_strain:.2f}, N/kPa {N_per_kPa:.4f}; "
              f"{len(pts)} markers ({size:.0f} px), rejected {rejected}")

        rows = []
        crop = (cal["frame"]["left"][0] - 70, cal["frame"]["top"][0] - 70,
                cal["frame"]["right"][1] + 70, cal["frame"]["bottom"][1] + 70)
        ov = im.crop(crop).copy()
        dr = ImageDraw.Draw(ov)
        ox, oy = crop[0], crop[1]
        half_marker_strain = 0.5 * size * abs(ax["bottom"]["slope_per_px"])
        for p in sorted(pts, key=lambda q: -q["yc"]):        # increasing y value
            lft, rgt = error_bar(dark, p, cal)
            s = val(ax["bottom"], p["xc"])
            dmm = val(ax["top"], p["xc"])
            yk = val(ax["left"], p["yc"])
            yn = val(ax["right"], p["yc"])
            if lft is not None and rgt is not None:
                sd_s = 0.5 * abs(rgt - lft) * abs(ax["bottom"]["slope_per_px"])
                sd_mm = 0.5 * abs(rgt - lft) * abs(ax["top"]["slope_per_px"])
                bar_mid_off_px = 0.5 * (lft + rgt) - p["xc"]
                ub = None
                sd_status = "read"
            else:
                sd_s = sd_mm = bar_mid_off_px = None
                # Bar ink reaching past the marker on its centre rows means a
                # bar exists that we could not read (cap missing or hidden);
                # only a bar wholly inside the marker gets the half-marker bound.
                if bar_reach_px(dark, p, cal) >= 3:
                    ub = None
                    sd_status = "unreadable"
                else:
                    ub = half_marker_strain
                    sd_status = "hidden_in_marker"
            rows.append((yk, yn, s, dmm, sd_s, sd_mm, ub, sd_status,
                         bar_mid_off_px, p["xc"], p["yc"]))
            # overlay: cross at centre, bar ends as ticks
            cx, cy = p["xc"] - ox, p["yc"] - oy
            dr.line((cx - 12, cy, cx + 12, cy), fill=(255, 0, 0), width=2)
            dr.line((cx, cy - 12, cx, cy + 12), fill=(255, 0, 0), width=2)
            for e in (lft, rgt):
                if e is not None:
                    dr.line((e - ox, cy - 9, e - ox, cy + 9), fill=(0, 160, 255), width=2)
        ov.save(os.path.join(HERE, f"verify_fig2{key}.png"))

        if spec["kind"] == "free":
            name = f"yang2016_fig2{key}_free_{spec['material']}.csv"
            hdr = [f"Digitised from Yang et al. 2016 (Adv. Mater. Technol., "
                   f"doi:10.1002/admt.201600055; Harvard DASH accepted "
                   f"manuscript) Fig 2{key}: VAMP in "
                   f"{MATERIAL_LABEL[spec['material']]}, no load",
                   "dP_kPa = applied differential pressure P_ext - P_int "
                   "(vacuum depth), left axis; F_N = right axis, which is dP "
                   "times the cross-section (a rescaled pressure, NOT an "
                   "output force)",
                   "strain = actuation strain s(dP,0), bottom axis; "
                   "dL_mm = L(dP,0) - L(0,0), top axis (read directly)",
                   "strain_sd / dL_sd_mm = half-width of the published "
                   "horizontal error bar (SD over n = 7 samples), not reading "
                   "error; blank unless sd_status is read",
                   "sd_status: read = both caps found; hidden_in_marker = no bar ink outside the marker, sd_upper_bound_strain = half a marker; unreadable = bar ink leaves the marker but a cap is missing or hidden (one-sided whisker, or behind the frame): SD and bound left blank",
                   "bar_mid_minus_marker_px: diagnostic, centre of the bar "
                   "caps minus the marker centroid",
                   "Published measurements digitised by "
                   "digitize_yang2016_fig2.py; not our measurements"]
        else:
            name = f"yang2016_fig2{key}_load_{spec['material']}.csv"
            hdr = [f"Digitised from Yang et al. 2016 (Adv. Mater. Technol., "
                   f"doi:10.1002/admt.201600055; Harvard DASH accepted "
                   f"manuscript) Fig 2{key}: VAMP in "
                   f"{MATERIAL_LABEL[spec['material']]}, loaded by a hung "
                   f"weight at dP = 90 kPa (Fig 2 caption)",
                   "sigma_kPa = loading stress T/A (left axis); T_N = total "
                   "loading force (right axis, read directly)",
                   "strain = actuation strain s(dP,sigma) = (L(dP,sigma) - "
                   "L(0,sigma)) / L(0,0) (paper Eq 3), bottom axis: relative "
                   "to the length under the SAME load with no vacuum; "
                   "dL_mm = L(dP,sigma) - L(0,sigma), top axis (Fig 2 caption)",
                   "strain_sd / dL_sd_mm = half-width of the published "
                   "horizontal error bar (SD over n = 7 samples); blank "
                   "unless sd_status is read",
                   "sd_status: read = both caps found; hidden_in_marker = no bar ink outside the marker, sd_upper_bound_strain = half a marker; unreadable = bar ink leaves the marker but a cap is missing or hidden (one-sided whisker, or behind the frame): SD and bound left blank",
                   "bar_mid_minus_marker_px: diagnostic",
                   "Published measurements digitised by "
                   "digitize_yang2016_fig2.py; not our measurements"]
        cols = [spec["left_name"], spec["right_name"], "strain", "dL_mm",
                "strain_sd", "dL_sd_mm", "sd_upper_bound_strain", "sd_status",
                "bar_mid_minus_marker_px", "marker_x_px", "marker_y_px"]
        write_csv(os.path.join(HERE, name), hdr, cols, rows)
        cal["csv"] = name
        calib["panels"][key] = cal
        for r in rows:
            print(f"   {spec['left_name']} {r[0]:7.3f}  {spec['right_name']} "
                  f"{r[1]:7.3f}  s {r[2]:+.4f}  dL {r[3]:+7.2f} mm  sd "
                  f"{'-' if r[4] is None else f'{r[4]:.4f}'} ({r[7]})  mid-off "
                  f"{'-' if r[8] is None else f'{r[8]:+.1f}'} px")

    with open(os.path.join(HERE, "yang2016_fig2_calibration.json"), "w",
              encoding="utf-8") as f:
        json.dump(calib, f, indent=1, default=float)
    return 0


if __name__ == "__main__":
    sys.exit(main())
