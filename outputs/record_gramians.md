# The records' Gramians: numbers drawn

Made by `tools/figure_gramians.py` from `data/records/`. Figure: `outputs/record_gramians.png` (and `.svg`).
Simulated magnitudes are comparative, not predictive. Units of the tip block follow each record's input unit.

## Simulated records (stored Gramian)

| record | input span | levels | stored mode 2/mode 1 | stored sqrt(trace), whole spine | tip size | tip 2/1 | tip out of XY | mode 1 out of XY | tip block vs stored, rel. diff | flags |
|---|---|---|---|---|---|---|---|---|---|---|
| wall_0.0005m | 2 to 212 Pa | 106 | 1.5e-1 | 8.2e-3 m/Pa^1/2 | 5.5e-3 m/Pa^1/2 | 2.4e-1 | 0.00 | 0.00 | 1.4e-6 | - |
| wall_0.0010m | 2 to 398 Pa | 199 | 3.6e-1 | 1.2e-2 m/Pa^1/2 | 7.8e-3 m/Pa^1/2 | 3.9e-1 | 0.00 | 0.00 | 7.5e-7 | - |
| wall_0.0015m | 2 to 500 Pa | 250 | 1.4e-1 | 5.4e-3 m/Pa^1/2 | 3.5e-3 m/Pa^1/2 | 2.0e-1 | 0.00 | 0.00 | 8.2e-8 | - |
| wall_0.0020m | 2 to 500 Pa | 250 | 3.9e-2 | 2.9e-3 m/Pa^1/2 | 1.9e-3 m/Pa^1/2 | 5.4e-2 | 0.00 | 0.00 | 2.4e-9 | - |
| cable_finger | 0.375 to 3 mm pull | 8 | 2.5e-1 | 3.9e-1 m^1/2 | 2.4e-1 m^1/2 | 2.2e-1 | 0.20 | 0.05 | 2.6e-10 | - |
| bellows1 | 0 to 400 Pa | 9 | 1.4e-4 | 9.7e-4 m/Pa^1/2 | 5.3e-4 m/Pa^1/2 | 3.1e-5 | 0.00 | 0.00 | 6.1e-13 | - |
| bellows_seg3 | 50 to 400 Pa | 8 | 1.1e-3 | 2.8e-4 m/Pa^1/2 | 1.6e-4 m/Pa^1/2 | 8.6e-5 | 0.00 | 0.00 | 2.8e-14 | - |
| bellows_seg5 | 50 to 400 Pa | 8 | 3.4e-4 | 4.7e-4 m/Pa^1/2 | 2.6e-4 m/Pa^1/2 | 5.8e-5 | 0.00 | 0.00 | 3.7e-14 | - |
| bellows_seg7 | 50 to 400 Pa | 8 | 5.9e-4 | 7.6e-4 m/Pa^1/2 | 4.1e-4 m/Pa^1/2 | 5.3e-4 | 0.00 | 0.00 | 9.9e-14 | - |
| bellows_seg9 | 50 to 400 Pa | 8 | 1.9e-4 | 1.2e-3 m/Pa^1/2 | 6.5e-4 m/Pa^1/2 | 3.8e-5 | 0.00 | 0.00 | 2.4e-13 | - |
| bellows_tendon1 | 0 to 400 Pa | 17 | 2.7e-1 | 1.2e-3 m/Pa^1/2 | 7.5e-4 m/Pa^1/2 | 2.5e-1 | 0.00 | 0.00 | 2.0e-7 | - |
| mckibben_v2_20deg | 5 to 40 Pa | 8 | 4.7e-3 | 7.9e-4 m/Pa^1/2 | 4.3e-4 m/Pa^1/2 | 1.4e-3 | 0.00 | 0.00 | 9.3e-8 | - |
| mckibben_v2_30deg | 5 to 40 Pa | 8 | 1.2e-2 | 1.4e-3 m/Pa^1/2 | 7.2e-4 m/Pa^1/2 | 1.1e-2 | 0.01 | 0.00 | 8.3e-7 | - |
| mckibben_v2_40deg | 5 to 40 Pa | 8 | 4.5e-3 | 1.6e-3 m/Pa^1/2 | 8.5e-4 m/Pa^1/2 | 2.4e-3 | 0.00 | 0.01 | 6.6e-7 | - |
| mckibben_v2_55deg | 6.67 to 40 Pa | 6 | 3.5e-2 | 2.2e-4 m/Pa^1/2 | 3.5e-5 m/Pa^1/2 | 1.0e-1 | 0.14 | 0.57 | 1.7e-15 | free_expansion: floor_noise, blocked_force: floor_noise |
| mckibben_v2_60deg | 6.67 to 40 Pa | 6 | 1.9e-1 | 7.0e-4 m/Pa^1/2 | 3.1e-4 m/Pa^1/2 | 3.0e-3 | 0.01 | 0.23 | 2.8e-16 | - |
| mckibben_v2_70deg | 6.67 to 40 Pa | 6 | 8.1e-3 | 1.6e-3 m/Pa^1/2 | 8.4e-4 m/Pa^1/2 | 1.4e-4 | 0.00 | 0.00 | 1.0e-15 | - |

## Published records (tip block only, computed here from the free tip curve)

| record | source | input span | levels | tip size | tip 2/1 | tip out of XY |
|---|---|---|---|---|---|---|
| abboodi2026_ossa5_axial_extend | doi:10.1038/s41598-026-40604-5 | 11.36 to 129.63 kPa | 45 | 3.8e-5 m/Pa^1/2 | 0 | 0.00 |
| abboodi2026_ossa5_vacuum_contract | doi:10.1038/s41598-026-40604-5 | -94.25 to -16.42 kPa | 39 | 2.2e-5 m/Pa^1/2 | 0 | 0.00 |
| mosadegh2014_fPN | doi:10.1002/adfm.201303288 | 15 to 72 kPa | 5 | 4.8e-4 m/Pa^1/2 | 2.4e-1 | 0.00 |
| mosadegh2014_sPN | doi:10.1002/adfm.201303288 | 0 to 60 kPa | 5 | 1.3e-4 m/Pa^1/2 | 8.3e-3 | 0.00 |

## Not drawn

| record | reason |
|---|---|
| abboodi2026_ossa5_bending | no stored Gramian and no free tip curve (holds: blocked curve) |
| example_rigid_pneumatic_cylinder | no stored Gramian and no free tip curve (holds: headline numbers only) |
| ossa_axial_extend | no stored Gramian and no free tip curve (holds: headline numbers only) |
| ossa_differential_bend | no stored Gramian and no free tip curve (holds: headline numbers only) |
| ossa_vacuum_contract | no stored Gramian and no free tip curve (holds: headline numbers only) |
| shadow_hand_finger | no stored Gramian and no free tip curve (holds: headline numbers only) |
