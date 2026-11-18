# Kubler et al. 2023, vine-robot actuators: provenance of the DRAFT records

DRAFT. These three records are not in `data/records/` or `MANIFEST.json`
and have not been reviewed.

## Source

- Kubler, du Pasquier, Low, Djambazi, Aymon, Forster, Agharese, Siegwart,
  Okamura, "A Comparison of Pneumatic Actuators for Soft Growing Vine
  Robots". Published in *Soft Robotics* 11(5), 857-868, 2024,
  doi:10.1089/soro.2023.0169. This comes from Crossref: we searched by
  title, and the DOI's record confirms the title and authors.
- What was digitised: the arXiv preprint, arXiv:2305.00967v3 (15 Sep 2023),
  `source/kubler2023_arxiv2305.00967.pdf`, 26.6 MB. Its SHA-256 is asserted
  by the digitiser. The journal version was not checked, so its Fig 7 may
  differ.
- Licence: arXiv's non-exclusive distribution licence
  (arxiv.org/licenses/nonexclusive-distrib/1.0). It does not grant a right
  to redistribute. Keep the PDF and `kubler2023_fig7.png` local; do not
  commit them to a public repository.

## What was digitised

We used Fig 7, the 40 mm member of each actuator type:

- pouch motor W40 x L40;
- cPAM W40 x L40;
- fPAM W40. The fPAM varies by width only.

Two rows were read:

- **Top row.** Quasi-static bending angle per nominal length (deg/cm)
  against actuator pressure (kPa). We took the experimental mean, which is
  the dashed line; the shaded band is plus or minus 1 SD over five runs.
  The model, a solid line in the same colour, was ignored. The test was in
  the horizontal configuration.
- **Bottom row.** Lateral force (N) against pressure. The front of the vine
  robot was fixed to an ATI Nano-17 six-axis force-torque sensor. We read
  the mean of five runs at ten pressures, with the error bar giving the SD.

The test set-up, from the paper:

- vine body diameter 80 mm;
- body length 420 mm;
- actuated length 360 mm (Section 3, Table 1);
- body pressure 1.75 kPa;
- actuator pressure ramped from 0 to 40 kPa at 0.65 kPa/s.

## Method

`digitize_kubler2023_fig7.py` reads the single embedded raster of Fig 7
(page index 9, 3558 x 2390 px) without resampling. Its docstring describes
the method.

Axis calibration:

- Per panel, the axis lines and inward tick marks are detected and a
  straight line is fitted to each axis.
- Tick pixels, fits and residuals are in `kubler2023_fig7_calibration.json`.
- The maximum residual is asserted to be under 1.5 px. Every panel passed:
  the largest x residual was 0.5 px and the largest y residual 1.06 px.

To check the tracing by eye, compare the `verify_<row>_<type>.png` overlays
(red marks) with the figure.

## Reading error

`kubler2023_fig7_reading_error.json` is written by
`build_kubler2023_records.py` from the calibration. It combines the
calibration residual with a 1 px pick-off error:

- about 0.004 deg/cm on bend (about 0.3 to 0.4 mm of tip Y at the last
  level);
- about 0.01 N on force;
- about 0.045 kPa on pressure.

That is a lower bound.

- **Force readings.** Where lines overlap, the bar centre and the dashed
  mean line either side disagree by up to 0.06 N, on the cPAM (CSV column
  `line_check_N`).
- **Force-point pressures.** These are read from the centre of the error-bar
  caps. Choosing that centre, rather than the first full-height column,
  moved them by up to 7 px, or 0.3 kPa. Take about 0.1 kPa as their reading
  error.

The paper's own run-to-run SD is larger than all of these.

## Gaps and known limits

- **Where the bend data start.** The first bend row is at 8 kPa for the
  pouch, 6 kPa for the cPAM and 2 kPa for the fPAM. Below that, the curve
  runs under other series. The trace stops there rather than jumping to
  another line.
- **Partial windows.** A 2 kPa window is kept only if traced columns cover
  both halves of it, and its pressure is the mean of the columns actually
  used.
- **Force SD.** It includes the thickness of the cap line, about 0.02 N.

## Conversion to records

`build_kubler2023_records.py` converts bend angle to tip displacement. It
assumes:

- one circular arc of constant curvature over the 360 mm actuated length;
- an inextensible centreline;
- the remaining 60 mm of body is ignored.

That 60 mm sits at the tip end (Fig 3, side view). So the record's port is
the end of the actuated length. The vine's physical tip lies 60 mm further
along the final tangent and moves more than the port does. The force
sensor held that physical tip.

The frame has X running from base to tip, with bending toward +Y.
`source.reference` is the arXiv identifier, because that is what we
digitised. The journal DOI is in the citation.

Force goes in as `blocked.force_transverse_N`. The paper reports no axial
component, so `force_axial_N` is null. With these records, the workshop
loader therefore reports "no force data", as it does for the Mosadegh
records.

Regenerate with:

    python -I digitize_kubler2023_fig7.py
    python -I build_kubler2023_records.py   # refuses to overwrite; delete the outputs first
