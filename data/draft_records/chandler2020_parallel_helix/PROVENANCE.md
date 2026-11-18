# Chandler et al. 2020 parallel helix actuator: draft records

DRAFT. Not in `data/records`, not in `MANIFEST.json`.

## Source
Chandler, Chauhan, Garbin, Obstein, Valdastri, "Parallel Helix Actuators for Soft Robotic Applications",
Front. Robot. AI 7:119 (2020), doi:10.3389/frobt.2020.00119. Open access, CC BY 4.0.
PDF from the publisher's open-access link, fetched 2026-11-12: `source/chandler2020_frobt.pdf`
(sha256 045e6289b41a9ef40f81f79e13caa7d2f8329a702c5c50efa4f1f4919b50d705). Figure images in `source/`
(Fig 7: sha256 5a9cab2f...c2f4; full hashes are written into each CSV header).

## Regenerate
Interpreter: Gramain_extraction `.venv` python, run with `-I` from this folder.
1. `digitize_chandler2020_fig7.py` -> `chandler2020_fig7C_a0_tip_C{1,2,3}.csv`, `chandler2020_fig7C_a0_calibration.csv`, `verify_fig7C_a0_*.png`
2. `build_chandler2020_draft_records.py` -> `chandler2020_pha_a0_C{1,2,3}.json` (validates against both validators; refuses to overwrite)
3. `digitize_chandler2020_fig10.py` -> `chandler2020_fig10_start_inputs.csv`, `chandler2020_fig10CD_start_dots.csv`, `chandler2020_fig10C_dot_centres.csv`, `verify_fig10CD_dots.png`
4. `sum_of_parts_fig10.py` -> `chandler2020_fig10_sum_of_parts.csv`

`chandler2020_tables_transcribed.csv` holds Tables 2 and 3 typed from the paper's text layer. It is used only for checks; no record value comes from it.

## Fig 7C calibration (alpha = 0)
- The 3D view is treated as an orthographic projection. Six affine coefficients are fitted to 13 points where box edges meet grid lines and tick marks. Fit residual: 0.82 px horizontally, 0.41 px vertically.
- The fitted box corners land at +-40 mm (39.9, -40.3, -40.1, 39.8), which confirms the axis limits.
- The x-y polar inset is calibrated from its rings: the 25 mm ring is 142.4 px in radius (0.176 mm/px), and the ring centres agree to within 0.8 px.
  - Scale check: in all three chambers the inset trace reaches about one stroke half-width beyond the furthest marker. A wrong ring scale would be off by a factor of 1.8.
- 3D reconstruction, per marker:
  - its horizontal image position fixes a line in the x-y plane;
  - that line is crossed with the chamber's inset trace;
  - its vertical image position then gives z.

## Volumes are inferred
Fig 7C has no volume axis, so volumes were assigned as follows.
- Markers are counted back from the last one, taken as 8.0 ml, in 0.2 ml steps. That step is the marker spacing of Fig 7D; we assume it holds in Fig 7C too.
- Where the image gap is more than 1.6 times the local median, round(ratio) - 1 hidden markers are inserted.
- Counting stops at a gap ratio above 2.5, or within 50 px of the origin, where the markers merge into one blob.

Check on the counting: we compare the bend angle from a constant-curvature fit to each point with the paper's Eq 11 Gaussian fit at the counted volume. The angle is calculated with the length-free relation rho/h = (1 - cos t)/(t - sin t).
- C1: the median offset is -13 deg (interquartile -14 to -12), but it is not uniform.
  - From 5 ml up, the volume found by inverting Eq 11 is within 0.3 ml of the counted volume.
  - Below 5 ml the gap grows to 1.3 ml: at the counted 4.2 ml, Eq 11 gives 2.9 ml, an angle offset of about -33 deg.
  - So we take the volume uncertainty as about +-0.3 ml from 5 ml up and up to about 1.3 ml below 5 ml.
  - The `V_est_ml` column in the tip CSVs uses the Table 3 rows as printed. For C2 and C3 it is therefore not meaningful; see the next point.
- C2 and C3 agree only when the Table 3 Gaussian rows for C2 and C3 are swapped:
  - C2: +26 deg as printed, -0 deg (-4 to +4) swapped;
  - C3: -35 deg as printed, -10 deg (-13 to -6) swapped.
  - With the rows as printed, Eq 11 at 8 ml gives C2 199 deg and C3 218 deg, against Table 2's 222 and 204 deg.
  - So the Table 3 Gaussian rows for C2 and C3 appear swapped in the paper. We note this and do not correct it.
- C3 counting near the origin is the weakest. Its lowest point, at 3.2 ml, depends on one inserted hidden marker.

## Reading error
- x and y: the half-width of the inset trace where it is crossed.
  - C1 and C3: 1.6 to 4.3 mm.
  - C2: up to 13 mm below 5 ml and at 7.0 to 7.4 ml. There its inset trace runs along the dashed guide line and the strands cannot be told apart.
- z: carried through from x and y, 0.6 to 1.6 mm (C2 up to 4.8 mm).
- The reprojection residual is zero by construction, so it is not used as an error.
- Records: `free.uncertainty` = worst 3D reading error / free stroke. This gives C1 0.07, C2 0.22 and C3 0.08. Poorly read points are kept and declared through this field rather than filtered out, as the record schema asks.

## Frame
Every record uses one common frame:
- X = -z of the paper (base to tip; Fig 7 plots the rise of the tip back toward the base);
- Y = paper y;
- Z = paper x (inset 0 deg, up).

This is right-handed. No chamber was rotated onto +Y, so each chamber bends in its own direction in Y-Z.
- Our measured bending azimuths sit 4 to 7 deg below Table 3's in all three chambers. This is unexplained; it could be an inset rotation or a definition difference.
- The workshop checker describes its canonical frame as having "+Y main sideways output". These records do not follow that convention.

## Input
The input is the air volume displaced by the syringe (compressible), not the chamber volume. It is stored in m^3 with `unit: null`, because the schema's SI unit list has no volume unit. It is not converted to pressure.

Fig 7 is the up-sweep of a 1.6 ml/s ramp to 8 ml.

## Blocked force
`blocked: null`. The paper has no blocked-force or tip-force measurement. Fig 11 shows only the workspace with tip masses of 2.5 to 7.5 g hung on, which is not a blocked force.

## Other folders
`data/records_draft/` is an unrelated, older folder holding human-hand drafts. This folder is new and separate from it.

## Data availability
The paper's statement says the raw data "will be made available by the authors, without undue reservation". Asking the authors for it would replace the digitising and the volume counting.

## Fig 10 sum-of-parts
- The start dots of the combined-input tests in Fig 10C and 10D were read.
  - The 4.5 to 7.0 ml tests are resolved. The rest merge at the origin.
  - Y and Z come from 10D. X comes from the 10C dot chain at that Y, because the outer dots in 10C overlap and are partly covered by the traces. Reading error is about 1 mm, and 1.6 mm in X at 4.5 ml.
- Relative inputs at the cycle start, read from Fig 10A: C1 0.934, C2 0.503, C3 0.068 of the peak volume.
- Result: the combined positions were read, but the leftover cannot be obtained. The matching single-chamber moves cannot be read.
  - At the cycle start, C2 gets at most 3.5 ml and C3 at most 0.5 ml.
  - Fig 7C resolves C2 only from 4.0 ml and C3 only from 3.2 ml; below that the markers merge at the origin.
  - The script also gives the combined move minus the resolved C1 move (11 to 16 mm). This is not the leftover, because it still contains the C2 and C3 moves.
  - It also gives a bound that assumes each chamber's move grows monotonically with volume. The bound includes zero, so it says nothing.
  - A clean test would need a point where every input lies in the resolved range. For example, with one chamber at zero and the other two at 0.75 of peak, the range is resolved for peaks of 5.5 ml and up. But Fig 10 marks only the cycle start, and the traces carry no step markers, so such points cannot be read. We did not estimate them.
- Further caveats:
  - Fig 7 is a 1.6 ml/s ramp; Fig 10 is quasi-static, with a 2 s settle.
  - We assume Fig 7 and Fig 10 share one frame (same axis labels and orientation).
