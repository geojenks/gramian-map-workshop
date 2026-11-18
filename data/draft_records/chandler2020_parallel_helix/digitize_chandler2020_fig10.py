"""Digitise the combined-input start positions (black dots) from Chandler et al. 2020 Fig 10C and 10D (alpha = 0),
and the relative chamber inputs at the start of the cycle from Fig 10A.

Paper caption: "Black dots indicate starting positions for successive maximum volume tests". Fourteen tests, peak
volume V = 0.5 ... 7.0 ml in 0.5 ml steps, quasi-static (2 s settle). At cycle position 0 each chamber gets
(relative input from Fig 10A) x V.

Calibration (all measured from the image here, nothing typed except axis label values printed on the figure):
  Fig 10A: plot frame = x 0..1 (cycle position), y 0..1 (relative input); labels 0 and 1 sit on the frame edges.
  Fig 10C: frame = Y +50 (left) .. -50 (right), X -50 (bottom) .. +50 (top), labels at the frame edges.
  Fig 10D: frame = Y +50 .. -50 horizontally; Z from the five horizontal grid lines labelled -20, 0, 20, 40, 60.
Dots: dark pixels inside each frame, opened, then distance-transform peaks (min separation MIN_SEP_PX).
Y and Z of each dot come from Fig 10D, where the dots separate in height. X comes from Fig 10C: the outer dots
there overlap and are partly covered by the traces, so X is interpolated along the chain of C dot centres at the
Y read in D (a C dot must lie within MATCH_TOL_MM in Y). Dots below MERGED_Z_MM are in the clump at the origin.
Resolved dots are ranked by Z, highest first, and given V = 7.0, 6.5, ... This assumes the start position rises
monotonically with peak volume, which the trace colours seen by eye under the dots in Fig 10C support. The
printed ring colours are not a usable check: they mix several traces and did not confirm 6.5 or 7.0 ml.
Writes chandler2020_fig10_start_inputs.csv, chandler2020_fig10CD_start_dots.csv and verify_fig10CD_dots.png.
"""
import csv
import hashlib
import os

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage as ndi

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "source", "chandler2020_fig10.jpeg")
DARK = 90
MIN_SEP_PX = 14
MATCH_TOL_MM = 1.5
MERGED_Z_MM = 3.0     # D dots below this rise sit in the merged clump at the origin
DOT_ERR_MM = 1.0      # dot centre reading error: dots ~2.7 mm across and partly covered by traces
V_MAX_ML = 7.0
V_STEP_ML = 0.5
Z_GRID_LABELS = [60.0, 40.0, 20.0, 0.0, -20.0]  # top to bottom, as printed on Fig 10D
WINDOWS = {"A": (500, 150, 3000, 1100), "C": (300, 4150, 1500, 5350), "D": (1800, 4150, 3000, 5350)}


def sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def frame(img, win, frac):
    x0, y0, x1, y1 = win
    d = img[y0:y1, x0:x1].max(axis=2) < DARK
    rows = np.where(d.sum(1) > frac * d.shape[1])[0]
    cols = np.where(d.sum(0) > frac * d.shape[0])[0]
    top = rows[rows < rows.mean()].mean() + y0
    bot = rows[rows > rows.mean()].mean() + y0
    left = cols[cols < cols.mean()].mean() + x0
    right = cols[cols > cols.mean()].mean() + x0
    return top, bot, left, right


def grid_rows(img, win, left, right):
    x0, y0, x1, y1 = win
    sub = img[y0:y1, int(left) + 6:int(right) - 6]
    g = (np.ptp(sub, axis=2) < 14) & (sub.max(2) > 150) & (sub.max(2) < 235)
    r = np.where(g.mean(1) > 0.25)[0] + y0
    groups = np.split(r, np.where(np.diff(r) > 3)[0] + 1)
    return [float(gr.mean()) for gr in groups if len(gr)]


def start_inputs(img):
    top, bot, left, right = frame(img, WINDOWS["A"], 0.5)
    out = []
    for ch, chan in ((1, 0), (2, 1), (3, 2)):
        col = img[int(top) + 4:int(bot) - 4, int(left) + 6]
        others = [c for c in range(3) if c != chan]
        m = (col[:, chan] > 150) & (col[:, others[0]] < 120) & (col[:, others[1]] < 120)
        rows = np.where(m)[0] + int(top) + 4
        v = (bot - rows.mean()) / (bot - top)
        out.append({"chamber": ch, "relative_input_at_cycle_start": round(float(v), 4), "n_px": int(m.sum())})
    return out, (top, bot, left, right)


def dots(img, win, top, bot, left, right):
    x0, y0 = int(left) + 10, int(top) + 10
    sub = img[y0:int(bot) - 10, x0:int(right) - 10]
    d = ndi.binary_opening(sub.max(2) < DARK, iterations=1)
    dt = ndi.distance_transform_edt(d)
    peaks = (dt == ndi.maximum_filter(dt, size=2 * MIN_SEP_PX + 1)) & (dt >= 4)
    lab, n = ndi.label(ndi.binary_dilation(peaks, iterations=3))
    pts = ndi.center_of_mass(peaks, lab, range(1, n + 1))
    res = []
    for (py, px) in pts:
        r = dt[int(round(py)), int(round(px))]
        res.append((px + x0, py + y0, float(r)))
    return res


def ring_colour(img, x, y, r):
    yy, xx = np.mgrid[-40:41, -40:41]
    rr = np.hypot(xx, yy)
    m = (rr > r + 4) & (rr < r + 14)
    pix = img[int(y) + yy[m], int(x) + xx[m]]
    keep = (np.ptp(pix, axis=1) > 60)
    return [int(c) for c in np.median(pix[keep], axis=0)] if keep.any() else None


def main():
    img = np.asarray(Image.open(SRC).convert("RGB")).astype(int)
    ins, fa = start_inputs(img)
    print("Fig 10A frame (top, bottom, left, right) px:", [round(v, 1) for v in fa])
    for r in ins:
        print("  start input", r)

    cal = {}
    for p in ("C", "D"):
        cal[p] = frame(img, WINDOWS[p], 0.65)
    tC, bC, lC, rC = cal["C"]
    tD, bD, lD, rD = cal["D"]
    yC = lambda px: 50.0 - 100.0 * (px - lC) / (rC - lC)
    xC = lambda py: 50.0 - 100.0 * (py - tC) / (bC - tC)
    yD = lambda px: 50.0 - 100.0 * (px - lD) / (rD - lD)
    gz = grid_rows(img, WINDOWS["D"], lD, rD)
    gz = [g for g in gz if tD + 20 < g < bD - 20]
    assert len(gz) == len(Z_GRID_LABELS), gz
    zfit = np.polyfit(gz, Z_GRID_LABELS, 1)
    zres = np.polyval(zfit, gz) - np.array(Z_GRID_LABELS)
    zD = lambda py: float(np.polyval(zfit, py))
    mm_per_px_C = 100.0 / (rC - lC)
    print("C frame px", [round(v, 1) for v in cal["C"]], "mm/px", round(mm_per_px_C, 4),
          "aspect check (vertical/horizontal span)", round((bC - tC) / (rC - lC), 4))
    print("D frame px", [round(v, 1) for v in cal["D"]], "Z grid rows", [round(g, 1) for g in gz],
          "Z fit mm/px", round(-zfit[0], 4), "max grid residual mm", round(float(abs(zres).max()), 3))

    dC = dots(img, WINDOWS["C"], *cal["C"])
    dD = dots(img, WINDOWS["D"], *cal["D"])
    print(f"dots found: C {len(dC)}, D {len(dD)}")
    # In C the outer dots overlap and are partly covered by the traces drawn over them, so they cannot be told
    # apart one by one. They all lie on one smooth chain, so X is read from C as a function of Y (linear
    # interpolation through the C dot centres sorted by Y), at the Y measured in D, where the dots separate in Z.
    cy_sorted = sorted((yC(cx), xC(cy)) for (cx, cy, cr) in dC)
    cYs = np.array([t[0] for t in cy_sorted]); cXs = np.array([t[1] for t in cy_sorted])
    rows = []
    for (dx, dy, dr) in dD:
        Y, Z = yD(dx), zD(dy)
        r = {"u_D_px": round(dx, 1), "v_D_px": round(dy, 1), "Y_mm": round(Y, 2), "Z_mm": round(Z, 2),
             "ring_rgb_D": ring_colour(img, dx, dy, dr)}
        near = cXs[np.abs(cYs - Y) <= MATCH_TOL_MM]
        if Z < MERGED_Z_MM:
            r["status"] = "merged_near_origin_not_read"
        elif Y < cYs.min() - MATCH_TOL_MM or Y > cYs.max() + MATCH_TOL_MM or not len(near):
            r["status"] = "no_C_dot_within_tolerance"
        else:
            r["X_mm"] = round(float(np.interp(Y, cYs, cXs)), 2)
            h = 0.5
            slope = abs(float(np.interp(Y + h, cYs, cXs) - np.interp(Y - h, cYs, cXs))) / (2 * h)
            # X error: dot reading error, spread of nearby C dots, and the Y error carried along the chain slope
            r["X_err_mm"] = round(max(DOT_ERR_MM, float(np.ptp(near)) / 2, slope * DOT_ERR_MM), 2)
            r["status"] = "ok"
        rows.append(r)
    matched = sorted([r for r in rows if r["status"] == "ok"], key=lambda r: -r["Z_mm"])
    for k, r in enumerate(matched):
        r["V_peak_ml"] = round(V_MAX_ML - V_STEP_ML * k, 2)
        for ins_r in ins:
            r[f"V_C{ins_r['chamber']}_ml"] = round(ins_r["relative_input_at_cycle_start"] * r["V_peak_ml"], 4)
        r["Y_err_mm"] = r["Z_err_mm"] = DOT_ERR_MM
    cdots = [{"u_C_px": round(cx, 1), "v_C_px": round(cy, 1), "Y_C_mm": round(yC(cx), 2), "X_C_mm": round(xC(cy), 2),
              "ring_rgb_C": ring_colour(img, cx, cy, cr)} for (cx, cy, cr) in dC]

    hdr = ["# Digitised from Chandler et al. 2020, Front. Robot. AI 7:119, Fig 10 (alpha = 0). Published data, not ours.",
           f"# Source image source/chandler2020_fig10.jpeg sha256 {sha256(SRC)}",
           "# Paper frame of Fig 10 (X up and Y left in the x-y view, Z = rise of the tip), see PROVENANCE.md.",
           f"# Reading error: dot centre about +-{DOT_ERR_MM} mm (dots partly covered by traces; {mm_per_px_C:.3f} mm/px);"
           " merged dots near the origin not read."]
    p1 = os.path.join(HERE, "chandler2020_fig10_start_inputs.csv")
    with open(p1, "w", newline="", encoding="utf-8") as f:
        f.write("\n".join(hdr[:2]) + "\n# Relative input of each chamber at cycle position 0 (left frame edge of Fig 10A).\n")
        w = csv.DictWriter(f, fieldnames=["chamber", "relative_input_at_cycle_start", "n_px"])
        w.writeheader()
        w.writerows(ins)
    cols = ["V_peak_ml", "V_C1_ml", "V_C2_ml", "V_C3_ml", "X_mm", "Y_mm", "Z_mm", "X_err_mm", "Y_err_mm",
            "Z_err_mm", "status", "u_D_px", "v_D_px", "ring_rgb_D"]
    p2 = os.path.join(HERE, "chandler2020_fig10CD_start_dots.csv")
    with open(p2, "w", newline="", encoding="utf-8") as f:
        f.write("\n".join(hdr) + "\n# Y and Z from Fig 10D; X from the Fig 10C dot chain at that Y (see script docstring).\n")
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in matched + [r for r in rows if r["status"] != "ok"]:
            w.writerow({k: (" ".join(map(str, r[k])) if isinstance(r.get(k), list) else r.get(k, "")) for k in cols})
    p3 = os.path.join(HERE, "chandler2020_fig10C_dot_centres.csv")
    with open(p3, "w", newline="", encoding="utf-8") as f:
        f.write("\n".join(hdr[:2]) + "\n# Dark-blob centres found in Fig 10C (x-y view); outer ones overlap.\n")
        w = csv.DictWriter(f, fieldnames=list(cdots[0]))
        w.writeheader()
        for r in cdots:
            w.writerow({k: (" ".join(map(str, v)) if isinstance(v, list) else v) for k, v in r.items()})

    pil = Image.open(SRC).convert("RGB")
    dr = ImageDraw.Draw(pil)
    for r in cdots:
        u, v = r["u_C_px"], r["v_C_px"]
        dr.ellipse([u - 16, v - 16, u + 16, v + 16], outline=(0, 200, 255), width=4)
    for r in rows:
        u, v = r["u_D_px"], r["v_D_px"]
        col = (255, 0, 255) if r["status"] == "ok" else (0, 200, 255)
        dr.ellipse([u - 22, v - 22, u + 22, v + 22], outline=col, width=5)
        if "V_peak_ml" in r:
            dr.text((u + 26, v - 10), f"{r['V_peak_ml']}", fill=col)
            uc = lC + (50.0 - r["Y_mm"]) / 100.0 * (rC - lC)
            vc = tC + (50.0 - r["X_mm"]) / 100.0 * (bC - tC)
            dr.ellipse([uc - 6, vc - 6, uc + 6, vc + 6], fill=(255, 0, 255))
    pil.crop((300, 4150, 3000, 5350)).save(os.path.join(HERE, "verify_fig10CD_dots.png"))
    print(f"{'V':>4} {'X':>7} {'Y':>7} {'Z':>6} {'Xerr':>5}  ringD")
    for r in matched:
        print(f"{r['V_peak_ml']:4.1f} {r['X_mm']:7.2f} {r['Y_mm']:7.2f} {r['Z_mm']:6.2f} {r['X_err_mm']:5.2f}  {r['ring_rgb_D']}")
    for r in rows:
        if r["status"] != "ok":
            print("not used:", r["status"], r["Y_mm"], r["Z_mm"])
    print("C dot centres (Y, X):", [(r["Y_C_mm"], r["X_C_mm"]) for r in cdots])
    print("wrote", p1, p2, p3)


if __name__ == "__main__":
    main()
