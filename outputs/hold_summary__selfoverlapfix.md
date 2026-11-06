# Self-hold checks, one row per hand

Written by `tools/tabulate_holds.py` from `data/leaders/hold_*__self__selfoverlapfix.json` and `outputs/follow_*__hold_*__self__selfoverlapfix__fixed.json`. Leaders made 6 Nov with the hand's overlap with itself in the curl score (fit self mm is what is left of it at the generator's pose); checked at commit 1894096 (follow.py as of 702a7a3). Set beside outputs/hold_summary__recheck_93d5b39.md, the first leaders at the same check.

| hand | fingers | curl | fit miss mm | fit face deg | fit palm mm | fit other mm | fit self mm | ball g | verdict | reason | finger states | reach mm | side deg | pen mm | torque | margin | s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| allegro | 4 | 0.55,0.50,0.55,0.55 | 7.54 | 65.17 | 0.00 | 0.00 | 0.00 | 15.69 | green | - | green | 0.05 | 64.10 | 0.90 | yes | 189,189 | 241.90 |
| george_clip | 5 | 0.55,0.55,0.55,0.55,0.20 | 2.12 | 75.02 | 0.00 | 0.81 | 0.00 | 32.69 | green | - | green | 0.50 | 74.88 | 2.40 | yes | 396,396 | 314.40 |
| george_kp | 5 | 0.65,0.65,0.65,0.60,0.30 | 9.93 | 89.26 | 0.00 | 0.00 | 0.00 | 31.12 | green | - | green | 1.71 | 74.12 | 1.36 | yes | 832,832 | 10.80 |
| george_kp_wide | 5 | 0.50,0.45,0.45,0.40,0.20 | 7.77 | 54.00 | 0.00 | 0.00 | 0.03 | 25.94 | green | - | green | 0.32 | 48.92 | 0.60 | yes | 452,452 | 244.20 |
| george_measured | 5 | 0.60,0.60,0.55,0.50,0.35 | 1.80 | 51.61 | 0.00 | 0.00 | 1.24 | 32.76 | green | - | green | 1.40 | 49.41 | 1.06 | yes | 464,464 | 9.30 |
| george_measured_thumb | 5 | 0.60,0.60,0.60,0.55,0.20 | 5.62 | 72.55 | 0.00 | 0.00 | 0.24 | 25.12 | amber | - | amber,green | 2.97 | 62.88 | 3.47 | yes | 1.02e+03,1.02e+03 | 4.10 |
| human_vc | 5 | 0.65,0.60,0.60,0.70,0.45 | 1.40 | 55.36 | 0.00 | 0.00 | 0.00 | 37.31 | green | - | green | 1.33 | 62.42 | 1.34 | yes | 328,328 | 5.30 |
| human_vc_arch | 5 | 0.55,0.50,0.50,0.50,0.45 | 1.11 | 63.02 | 0.00 | 0.00 | 3.41 | 36.12 | green | - | green | 0.36 | 61.90 | 1.26 | yes | 432,432 | 404.80 |
| human_vc_arch_prop | 5 | 0.50,0.45,0.45,0.45,0.40 | 5.83 | 60.57 | 0.00 | 0.00 | 0.18 | 28.42 | green | - | green | 0.52 | 58.47 | 0.60 | yes | 820,820 | 9.70 |
| human_vc_prop | 5 | 0.60,0.60,0.55,0.50,0.45 | 4.10 | 50.84 | 0.00 | 0.00 | 0.00 | 38.57 | green | - | green | 0.71 | 53.90 | 0.58 | yes | 294,294 | 8.70 |
| l20 | 5 | 0.55,0.50,0.40,0.35,0.40 | 1.80 | 70.24 | 0.00 | 0.09 | 0.00 | 23.24 | green | - | green | 0.63 | 70.67 | 1.37 | yes | 328,328 | 257.00 |
| l20_public | 5 | 0.50,0.35,0.30,0.25,0.30 | 1.14 | 59.61 | 0.00 | 0.00 | 0.54 | 26.01 | green | - | green | 0.33 | 59.82 | 0.00 | yes | 240,240 | 11.90 |
| leap | 4 | 0.45,0.40,0.45,0.50 | 13.13 | 84.51 | 0.00 | 0.00 | 0.00 | 25.34 | green | - | green | 0.30 | 43.95 | 0.65 | yes | 304,304 | 12.00 |
| shadow | 5 | 0.35,0.35,0.40,0.40,0.65 | 1.53 | 74.61 | 0.00 | 0.00 | 0.00 | 20.37 | red | torque | red | 1.30 | 57.09 | 4.67 | yes | 0,0 | 19.50 |
| wall_x3_ring | 3 | 0.40,0.15,0.15 | 0.00 | 58.26 | 0.00 | 0.00 | 0.00 | 29.26 | amber | - | amber | 1.24 | 59.77 | 0.00 | yes | 1.48,1.48 | 88.40 |
