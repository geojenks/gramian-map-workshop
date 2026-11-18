# Yang et al. 2016, vacuum buckling actuators: provenance of the DRAFT records

DRAFT. These two records are not in `data/records/` or `MANIFEST.json`
and have not been reviewed.

## Source

- Yang, Verma, So, Mosadegh, Keplinger, Lee, Khashai, Lossner, Suo,
  Whitesides, "Buckling Pneumatic Linear Actuators Inspired by Muscle".
  *Advanced Materials Technologies* 1(3), 1600055, 2016,
  doi:10.1002/admt.201600055. Authors, volume and issue are as printed on
  the DASH cover sheet of the copy we used.
- What was digitised: the authors' accepted manuscript in Harvard DASH,
  citable link http://nrs.harvard.edu/urn-3:HUL.InstRepos:29951794. We
  fetched it through the US DOE OSTI public-access record
  (osti.gov/pages/biblio/1387845, PDF link
  osti.gov/pages/servlets/purl/1387845). That file is the DASH PDF with its
  cover sheet. Saved as `source/yang2016_admt201600055_dash.pdf`, 23 pages,
  5,957,023 bytes; the digitiser asserts its SHA-256.
- The Suo group's own copy (suo.seas.harvard.edu) returned HTTP 403. We did
  not try to get round it.
- The published version was not checked, so its Fig 2 may differ in detail.
  The Supporting Information is not in this copy.
- Licence: DASH makes it available "under the terms and conditions
  applicable to Open Access Policy Articles" (cover sheet). We did not
  read those full terms, so we do not know whether they allow
  redistribution. Until that is checked, keep the PDF and
  `yang2016_fig2.png` local and out of any public repository. The verify
  PNGs are crops of the figure (one panel each) and fall under the same
  caution.

## What was digitised

Fig 2 (PDF page 20, printed page 19), one embedded raster of
2100 x 1155 px. Four panels, 11 points each, means of n = 7 samples with
horizontal SD bars:

| Panel | Material | Vertical axis | Horizontal axis | CSV |
|---|---|---|---|---|
| A | Ecoflex 00-30, E = 43 kPa | dP (kPa); right axis F (N) | strain s(dP,0); top axis L(dP,0) - L(0,0) (mm) | `yang2016_fig2A_free_ecoflex.csv` |
| B | Elastosil M4601, E = 520 kPa | as A | as A | `yang2016_fig2B_free_elastosil.csv` |
| C | Elastosil M4601 | loading stress sigma (kPa); right axis T (N) | strain s(90 kPa, sigma); top axis L(dP,sigma) - L(0,sigma) (mm) | `yang2016_fig2C_load_elastosil.csv` |
| D | Ecoflex 00-30 | as C | as C | `yang2016_fig2D_load_ecoflex.csv` |

Note that the loaded panels swap the material order (C is Elastosil, D is
Ecoflex). Panels E and F (dynamic results) were not used.

- dP = P_ext - P_int (paper Eq 1): the depth of vacuum, positive.
- The right axis of A and B, F, is dP times the cross-section. It is a
  rescaled pressure, not a measured output force.
- Under load the paper's strain is s = (L(dP,sigma) - L(0,sigma)) / L(0,0)
  (Eq 3). It is measured from the length under the same load with no
  vacuum, not from the unloaded rest length.
- Values quoted in the text that the scripts use are in
  `yang2016_text_values.csv`, each with its page and sentence: the device
  size 34 x 28 x 46.5 mm, dP = 90 kPa for the loaded tests, the two moduli,
  and n = 7.

## Method

`digitize_yang2016_fig2.py`:

1. Asserts the PDF's SHA-256 and pulls the Fig 2 raster with pymupdf
   (`yang2016_fig2.png`).
2. Finds each panel's frame from the most-inked rows and columns. Finds the
   tick marks on all four sides by the length of ink running outward from
   the frame; major ticks (about 27 px) and minor ticks (about 14 px) are
   separated at 16 px. Fits value against pixel by least squares for each
   axis, using the printed major-tick labels. It asserts the number of
   majors equals the number of labels, that minors fall half-way between
   majors, and that the worst residual is below 1.5 px.
3. Cross-checks the axes against each other and the text: top axis mm per
   unit strain against the 46.5 mm length, right axis N per kPa against the
   34 x 28 mm cross-section, both within 3%; zeros of paired axes within
   2 px. All four panels pass (46.46 to 46.50 mm per unit strain; 0.951 to
   0.953 N per kPa against 0.952).
4. Finds the square markers by a 9 x 9 binary opening and labelling. A blob
   is kept if it is about marker height and nearly solid, and if every
   column's vertical ink run through its centre is either about marker
   height or at least 24 px (an error-bar cap). This rejects the dP_crit
   arrowhead in A and B, whose runs are irregular (15 to 20 px). Cap
   columns at the ends of a blob are trimmed before the marker centre is
   taken; where only one side was trimmed, the centre is the clean edge
   plus half a marker. Asserts 11 markers per panel.
5. Reads each SD bar: caps touching or inside the marker come from the tall
   columns found in step 4; otherwise it walks the bar outward to the cap.
   The SD is half the distance between cap centres. Column `sd_status`
   says how each SD was found: `read` (both caps found);
   `hidden_in_marker` (no bar ink leaves the marker, so the SD is left
   blank and half a marker is given as its upper bound); `unreadable` (bar
   ink leaves the marker but a cap is missing, so SD and bound are both
   blank). Two points are `unreadable`, both in D: sigma about 0.21 kPa
   (bar runs into the left frame) and sigma about 0.41 kPa (one-sided
   whisker with a single visible cap).
6. Writes the four CSVs (with `#` header lines), the calibration JSON
   `yang2016_fig2_calibration.json`, and overlays `verify_fig2A.png` to
   `verify_fig2D.png`: a red cross at each marker centre and blue ticks at
   the caps found. We inspected all four overlays by eye; every marker and
   every visible bar is picked correctly.

## Reading error

`yang2016_fig2_reading_error.json`, written by the builder: for each axis,
sqrt(worst calibration residual^2 + 1 px^2) x units per px. Worst residuals
are 0.37 px or less. In round terms:

- strain: about 0.0015 on all panels;
- displacement (top axis): about 0.065 to 0.069 mm;
- dP: about 0.013 kPa (A) and 0.13 kPa (B);
- load T: about 0.004 N (D) and 0.036 N (C).

The paper's sample-to-sample SD (column `strain_sd`, 0.006 to 0.047 in
strain where read) is larger than this.

## Gaps and known limits

- The free curves end at about 3.4 kPa (Ecoflex) and 34.5 kPa (Elastosil),
  well short of the 90 kPa used for the loaded tests.
- The digitised free curves flatten into a plateau above dP_crit (A reads
  the same strain at three successive levels). They never reverse, and are
  kept as read, not smoothed.
- No blocked force is quoted in the text for these devices. The 20 g and
  500 g in the Fig 1 caption are lifted weights in a demonstration. The
  65 kPa figure is for a different urethane device (Fig S2), and "about
  100 kPa at 0% strain" is a projection for E about 5 MPa.
- Elastosil: the heaviest load in C (about 9.8 N) still leaves an actuation
  strain of about -0.06, well outside SD plus reading error, so the
  actuator was not stalled. The largest load it can lift at 90 kPa is
  above 9.8 N; how that relates to the force at rest length is unknown.
  The record's `blocked` is null.
- Ecoflex: the heaviest load in D (0.981 N) leaves an actuation strain of
  -0.0007, within SD plus reading error (0.017) of zero. So 0.98 N is the
  largest load the vacuum can still lift at 90 kPa. It is not the schema's
  blocked force (tip held at rest length), which is likely lower, because
  the load had already stretched the actuator before the vacuum was
  applied. The record's `blocked` is null; the value is in the notes and
  in `yang2016_fig2_reading_error.json` only.
- The loaded curves (C, D) as a whole have no place in schema v0.2: it has
  free motion against input and blocked force against input, but no
  force-length curve at a fixed input. They are kept in the CSVs only.
- In D the third point (sigma about 0.21 kPa) touches the left frame. Its
  centre may be off by 1 to 2 px (about 0.002 in strain) and its SD is
  unreadable. It is not used by the records.
- No stiffness, mass, volume or work data.

## Conversion to records

`build_yang2016_records.py` writes `yang2016_vamp_ecoflex.json` (from A
and D) and `yang2016_vamp_elastosil.json` (from B and C).

- Input: u = -dP in Pa, as the canon record
  `data/records/abboodi2026_ossa5_vacuum_contract.json` (negative gauge
  pressure, channel range up to 0). Levels run in increasing vacuum.
- Frame X base to tip. We assume one end is fixed (the base) and the other
  free (the tip). The paper's Fig 1C, D show the actuator lifting a hung
  weight; we did not check the test rig, which is described in the SI
  that this copy lacks. Free tip displacement is
  [L(dP,0) - L(0,0), 0, 0], read off the top axis. `polarity_axial` = -1.
- The first level is the digitised rest point as read (dP and strain within
  reading error of zero), not forced to zero.
- `free_direction` is the unit vector of the first level that moves more
  than 2% of the stroke: [-1, 0, 0] for both. `free_stroke_m` is the
  largest displacement magnitude.
- `length_m` is 46.5 mm from the text-values CSV.
- Blocked: null in both records, and `scalars.blocked_force_N` is null.
  The schema's blocked block is force with the tip held at rest length
  (x about 0 at the port). Fig 2C, D are hung-weight tests, and the
  paper's strain under load is measured from L(0,sigma), the length
  already stretched by the weight (Eq 3). So "actuation strain close to 0"
  (printed page 7) means the vacuum no longer lifts the weight, not that
  the tip is at rest length. The builder still checks the heaviest load
  against SD plus reading error and writes the outcome under
  `heaviest_load_test` in `yang2016_fig2_reading_error.json`, with
  `used_as_blocked_force` false. Ecoflex: within tolerance (0.981 N).
  Elastosil: not within tolerance (9.804 N, strain -0.059 against 0.0076).
  A row whose SD is unreadable would be tested on reading error alone.
- The channel range runs from the deepest free-test vacuum to 0, about
  -3.4 kPa (Ecoflex) and -34.5 kPa (Elastosil), because no 90 kPa level
  is in either record.
- `uncertainty.default_fraction` 0.1, `data_completeness` "curves_only",
  `provenance` lists the three CSVs used with their SHA-256 hashes.
- The builder asserts one raw CSV row round-trips into each record,
  validates before writing, and refuses to overwrite.

## Regenerate

From this folder, with the workshop's Python (pymupdf, numpy, scipy,
Pillow):

    set PYTHONIOENCODING=utf-8
    python -I digitize_yang2016_fig2.py
    python -I build_yang2016_records.py --dry-run
    python -I build_yang2016_records.py

The digitiser overwrites its own CSVs, calibration JSON and PNGs. The
builder refuses to overwrite: delete the two record JSONs and
`yang2016_fig2_reading_error.json` first. Then, from the repo root:

    python tools/check_record.py data/draft_records/yang2016_vacuum_buckling/yang2016_vamp_ecoflex.json
    python tools/check_record.py data/draft_records/yang2016_vacuum_buckling/yang2016_vamp_elastosil.json
