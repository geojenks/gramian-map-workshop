# Tandem twist pneu-net: draft records

DRAFT. Not in `data/records`, not in `MANIFEST.json`. Not banked.

## Source
Our own simulation (SOFA v24.06, the gramian_map pipeline, `sweep_modes.py --protocol custom`),
run 12 and 13 November 2026. A two-chamber pneu-net whose distal chamber is rotated about the
finger's axis relative to the proximal one (60 and 90 degrees).

Copied unchanged on 2026-11-19 from the research repository
(`geojenks/Gramain_extraction`, `gramian_map/experiments/pneunet_tandem/draft_records/`).
At copy time the files were not yet committed there; that repository's HEAD was `ce1cbddc7a8b8d384a3f5eadb562113fa9a34194`.
They were built there by `gramian_map/build_tandem_draft_records.py`.

| file | sha256 |
|---|---|
| `pneunet_tandem_twist_60deg.json` | `d30d1793791484444e8fa3015c43766035d3ff680b669aa3d766a1270bccc8be` |
| `pneunet_tandem_twist_90deg.json` | `4d65858f9fba427c81a4ed46e01bbfe1533e6ec804cf6c8a7fda57edd633f2a7` |

## What is different about these records
- Two input channels (proximal and distal chamber pressure). `free.u` holds one pair per level:
  each chamber alone, then the distal chamber rising with the proximal one held at 60 Pa.
- `free.tip_rotvec` holds the tip's rotation vector at each level. No record in `data/records`
  stores tip rotation.
- `scalars.free_stroke_m` and `scalars.free_direction` are empty, so tools that need them
  leave these records out.

Simulation only: comparative, not predictive.

## Tip positions over both inputs (added 2026-11-21)
`tip_surface_points.csv`: the 90 degree finger's tip position on a 7 by 7 grid of the two
chamber pressures (0 to 120 in steps of 20, the record's own input units). 49 rows: 27 marked
`measured` (simulated; 19 of them are the rows of `pneunet_tandem_twist_90deg.json`, 8 are
further simulated points along the top of each input's range) and 22 marked `filled in`
(computed from the others, not simulated). Columns: `p1`, `p2`, `kind`, `source`,
`tip_x_mm`, `tip_y_mm`, `tip_z_mm`. Tip position only, no tip rotation. Its two sideways axes
point the opposite way to the record's (`y` and `z` both change sign); the long axis is the same.

Copied unchanged on 2026-11-21 from the research repository (`geojenks/Gramain_extraction`,
`gramian_map/experiments/pneunet_tandem/knuckle_analysis_20nov/tip_surface_points.csv`,
commit `89231da032bbaa69256823c0cf8afbc3bb3d7538`). Written there by
`gramian_map/experiment_log/scripts/2026-11-20_tandem_knuckle_analysis/knuckle_cycle_analysis_20nov.py`.

| file | sha256 |
|---|---|
| `tip_surface_points.csv` | `915748037f084c8ca8cc35a120c6e63e48a1e64aabea04b16b3e43f8a672a9f3` |

Read by `tools/export_map_paths.py` through `data/map_extra_check_points.json`, which checks the
sign change against the 19 shared rows on every run. Simulation only: comparative, not predictive.
