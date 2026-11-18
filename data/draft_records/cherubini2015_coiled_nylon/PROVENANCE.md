# Cherubini et al. 2015, coiled nylon actuator: provenance of the DRAFT record

DRAFT. This record is not in `data/records/` or `MANIFEST.json` and has not
been reviewed.

## Source

- Cherubini, Moretti, Vertechy, Fontana, "Experimental characterization of
  thermally-activated artificial muscles based on coiled nylon fishing
  lines". *AIP Advances* 5, 067158 (2015), doi:10.1063/1.4923315.
- What was digitised: the publisher PDF,
  `source/cherubini2015_aipadv067158.pdf`. Its SHA-256 is asserted by the
  digitiser.
- Licence: Creative Commons Attribution 3.0 Unported (stated on printed
  page 067158-1). Redistribution with attribution is allowed.
- Page numbers below are printed pages (067158-N). In the PDF, page index N
  holds printed page N, because index 0 is the publisher's cover page.

## What was digitised

The paper tests one specimen, the "Reference Coil". It is a homochiral coil
twisted from 0.5 mm nylon 6 monofilament under 0.5 kg, then annealed. Its
unloaded length at ambient is 70 mm and its mass 75.6 mg (p. 4-5). It is
heated by hot air in a chamber. Every quoted value is in
`cherubini2015_quoted_values.csv`, with its page.

Two figures were read:

- **Fig 7 (p. 8).** Isometric tests. The coil is held at a fixed stretch
  (datasets A, B and C: 10, 15 and 20 mm, i.e. 14.3, 21.4 and 28.6 % of
  70 mm) and heated from 24 to 120 degC. Nominal stress is plotted against
  temperature: red asterisks for rising and blue circles for falling
  temperature, second thermal cycle. We read every marker: 17 per dataset
  and direction.
- **Fig 9 (p. 9).** Isothermal tensile tests at 25, 60, 88 and 122 degC.
  Nominal stress is plotted against strain. Left panel: loading/unloading
  average. Right panel: unloading. Both panels also carry triangles, which
  are the isometric stresses interpolated to those four temperatures. We
  traced all eight curves and all 24 triangles.

Figs 5-6 were not digitised. Fig 5 shows the cyclic tensile tests at the
four temperatures, and Fig 6 their loading/unloading averages. Fig 9
re-plots the same isothermal curves (average and unloading) with the
isometric points on top, so we read the curves from Fig 9 alone. None of
these figures shows a stroke on heating at constant load.

Nominal stress is the force over the cross-section of the untwisted
precursor fibre (p. 4).

## Method

`digitize_cherubini2015_figs.py` reads the two embedded rasters by size,
without resampling: Fig 7 is 951 x 735 px and Fig 9 is 1500 x 584 px. Its
docstring gives the details.

- **Axis calibration.** Per panel, it finds the frame lines and the inward
  ticks and fits a straight line to each axis. It asserts that every
  interior tick was found and that the largest residual is under 1.5 px.
  Largest residuals: Fig 7, 0.18 px (x) and 0.25 px (y); Fig 9 left, 0.16
  and 0.22 px; Fig 9 right, 0.12 and 0.14 px. These are in
  `cherubini2015_figs_calibration.json`.
- **Fig 7 markers.** It matches templates cut from the legend's own
  asterisk and circle. Each marker is assigned to the nearest Table I line
  (Table I, p. 8). It asserts that each marker is within 1.5 MPa of its own
  line and more than 2.5 MPa from the others.
- **Fig 9 curves.** Curves are separated by colour, and the legend order
  fixes which colour is which temperature. Each is traced row by row. The
  triangles are found first and masked out.

Check the tracing by eye against the `verify_fig7.png`,
`verify_fig9_unloading.png` and `verify_fig9_average.png` overlays.

## Reading error

`cherubini2015_figs_reading_error.json` is written by
`build_cherubini2015_record.py`. Per axis, it combines the calibration
residual with a 1 px pick-off error:

- about 0.15 degC on temperature;
- about 0.07 MPa on stress, which is about 0.013 N of force;
- about 0.2 mm on free displacement (0.18 to 0.22 mm).

A second spread applies to free displacement. The strain at the reference
stress is taken from a line fitted to nearby traced rows. Taking the
nearest traced rows instead moves it by up to 0.18 mm (the
`method_spread_m` entries). The traced curves step in a staircase, and the
line fit smooths that out. So the free-displacement error is about
0.2 mm from reading plus up to about 0.2 mm from method.

These are lower bounds. The bench temperature uncertainty is +/-5 degC
(p. 4), which is much larger. The reference stress (below) comes from a
fit with a standard error of 0.21 MPa, about 0.04 N; that shifts every
blocked force by the same amount.

## Gaps and known limits

- **No free stroke at zero load.** Below about 2 % strain the coils touch
  and the coil is much stiffer; the paper calls this the preload knee
  (p. 7), and it grows with temperature. Our inference is that an unloaded
  coil would hardly shorten on heating. The record therefore describes the
  coil at a pre-loaded operating point.
- **No isotonic test in this paper.** It cites its Ref. 11 for isotonic
  tests. Free displacement is read across the separate isothermal curves
  at one stress.
- **Gap in the 60 degC trace.** The 60 degC unloading curve has a gap of
  0.72 MPa (19.84 to 20.55 MPa) right at the reference stress, where a
  Fig 9 triangle sits on the curve and was masked out. The strain there is
  bridged by the line fit over 14 rows; the nearest rows either side give
  a value 0.14 mm different.
- **Fig 7 data start at about 36 degC**, not at the 24 degC ambient. The
  reference stress at 25 degC is an extrapolation of about 11 degC.
- **The isometric and isothermal tests do not quite agree.** At 28.6 %
  strain and 25 degC the Fig 9 triangles sit at about 22.3-22.6 MPa, but
  the Table I line gives 20.4 MPa there. The 25 degC unloading curve
  reaches 20.3 MPa at strain 0.2605, not 0.286. The paper says the two
  agree "much better" with the unloading curves than the average, not that
  they match.
- **Area.** The paper gives no area directly. We use pi d^2/4 for
  d = 0.5 mm, the definition of nominal stress. Check: 0.5 kg over that
  area gives 24.97 MPa; the paper says 25 MPa.

## Conversion to records

`build_cherubini2015_record.py` builds one record,
`cherubini2015_RC_datasetC.json`. It uses dataset C, the coil held at
20 mm stretch.

- **Why dataset C.** Its stress at ambient is the highest of the three
  (about 20 MPa, against about 10 and 14 MPa for A and B on the Table I
  lines). The higher the stress, the further the hot isothermal curves
  are from their preload knee where they cross it. This is our choice; A
  and B could be built the same way.
- **Reference state.** The coil held at 28.6 % stretch at 25 degC, under
  sigma0 = 20.34 MPa. sigma0 is our straight-line fit of the 17 rising
  points of dataset C, evaluated at 25 degC. Our fit has slope 0.1910
  MPa/degC and intercept 15.56 MPa; Table I gives 0.187 and 15.73. 25 degC
  is the paper's ambient isothermal test temperature.
- **Input.** Chamber air temperature rise above 25 degC, in degC. As in
  `tools/curves_to_record.py`, the unit goes in the channel name and the
  `unit` field is null. We store the rise, not the absolute temperature.
- **Blocked force.** For each rising point of dataset C,
  force_axial_N = -(sigma(T) - sigma0) x A. This is the rise in tension
  above the preload. It is negative because a contractor pulls toward the
  base. The falling-temperature circles are used only as a check: their
  fit slope is 0.1785 MPa/degC against 0.1910 rising.
- **Free displacement.** On each unloading curve (Fig 9 right) we find the
  strain at sigma0, from a straight-line fit of the traced rows within
  1.5 MPa of sigma0. Displacement along X is (strain at T - strain at
  25 degC) x 70 mm, at rises of 35, 63 and 97 degC. It is negative:
  contraction toward the base.
- **Alternatives, for comparison only** (in the reading-error JSON and the
  record's notes). Read from the average curves, the free displacements
  are -3.14, -5.62 and -9.51 mm, against -3.38, -6.75 and -11.93 mm from
  the unloading curves. Referenced to the held strain 0.286 instead of the
  25 degC curve, they are -5.17, -8.54 and -13.72 mm.
- **length_m** is the unloaded length at ambient, 70 mm. The coil works at
  1.286 times that.

The frame has X running from base to tip. The tip is the moving end of the
coil.

Regenerate with:

    python -I digitize_cherubini2015_figs.py
    python -I build_cherubini2015_record.py   # refuses to overwrite; delete the outputs first
