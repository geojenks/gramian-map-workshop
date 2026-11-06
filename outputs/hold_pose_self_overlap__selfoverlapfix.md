# Self-overlap of the hand at the stored self-hold poses (leaders hold_<hand>__self__selfoverlapfix)

Written by `tools/hold_pose_self_overlap.py` at 1894096df8a513e3def4c25e5cfa7e217cfbd3e3. Depths in mm. A pair is two of the hand's own bodies whose collision geoms penetrate in MuJoCo (the ball left out). MuJoCo drops contacts between a body and its parent, between pairs the model excludes, and for geoms with no collision bits, so the counts are a lower bound. These are facts about the poses of stored results, not new verdicts.

## Before and after

Before: the same hand's row in `outputs/hold_pose_self_overlap.json`. After: this table. Depths in mm; the follow pose's are the deepest over its frames. Net of rest: each pair's depth less the same pair's depth at the rest pose, the deepest pair taken (the measure the hold generator's curl score now carries).

| hand | verdict before | verdict after | generator pose, finger against finger: before | after | generator pose, any pair net of rest: before | after | follow pose, finger against finger: before | after | follow pose, any pair net of rest: before | after |
|---|---|---|---|---|---|---|---|---|---|---|
| allegro | green | green | 15.7 | 0.0 | 15.7 | 0.0 | 17.5 | 0.0 | 17.5 | 0.9 |
| george_clip | green | green | 1.4 | 0.0 | 1.4 | 0.0 | 1.4 | 9.9 | 1.4 | 9.9 |
| george_kp | green | green | 1.4 | 0.0 | 1.4 | 0.0 | 0.0 | 1.4 | 0.0 | 1.4 |
| george_kp_wide | green | green | 4.3 | 0.0 | 4.3 | 0.0 | 3.7 | 1.5 | 3.7 | 1.5 |
| george_measured | green | green | 1.2 | 1.2 | 1.2 | 1.2 | 0.0 | 0.0 | 0.0 | 0.0 |
| george_measured_thumb | green | amber | 1.2 | 0.2 | 1.2 | 0.2 | 0.4 | 0.0 | 0.4 | 0.0 |
| human_vc | green | green | 0.0 | 0.0 | 0.0 | 0.0 | 4.2 | 4.2 | 4.2 | 4.2 |
| human_vc_arch | green | green | 4.4 | 3.4 | 4.4 | 3.4 | 8.0 | 4.4 | 8.0 | 4.4 |
| human_vc_arch_prop | green | green | 6.7 | 0.2 | 6.7 | 0.2 | 8.5 | 2.3 | 8.5 | 2.3 |
| human_vc_prop | green | green | 0.0 | 0.0 | 0.0 | 0.0 | 2.2 | 2.2 | 2.2 | 2.2 |
| l20 | - | green | - | 0.0 | - | 0.0 | - | 0.0 | - | 0.0 |
| l20_public | - | green | - | 0.0 | - | 0.5 | - | 0.0 | - | 0.6 |
| leap | green | green | 6.0 | 0.0 | 6.0 | 0.0 | 10.6 | 0.7 | 10.6 | 0.7 |
| shadow | red | red | 15.8 | 0.0 | 15.8 | 0.0 | 11.1 | 0.0 | 11.1 | 0.0 |
| wall_x3_ring | green | amber | 12.9 | 0.0 | 12.9 | 0.0 | 12.9 | 0.0 | 12.9 | 0.0 |

## Pairs by pose

| hand | stored verdict | colliding geoms | model's contact excludes | pairs at rest (deepest) | pairs at generator pose (deepest; not at rest) | pairs at follow pose, first frame (deepest; not at rest) | of those, finger against another finger (deepest) |
|---|---|---|---|---|---|---|---|
| allegro | green | 21 | 5 | 0 (0.0) | 0 (0.0; 0 new, 0.0) | 1 (0.9; 1 new, 0.9) | 0 (0.0) |
| george_clip | green | 23 | 0 | 5 (19.2) | 5 (19.2; 0 new, 0.0) | 6 (19.2; 1 new, 9.9) | 1 (9.9) |
| george_kp | green | 16 | 0 | 3 (9.9) | 3 (9.9; 0 new, 0.0) | 6 (9.9; 3 new, 1.4) | 3 (1.4) |
| george_kp_wide | green | 22 | 0 | 4 (19.2) | 5 (19.2; 1 new, 0.0) | 10 (19.2; 6 new, 1.5) | 6 (1.5) |
| george_measured | green | 16 | 0 | 4 (9.9) | 8 (9.9; 4 new, 1.2) | 4 (9.9; 0 new, 0.0) | 0 (0.0) |
| george_measured_thumb | amber | 16 | 0 | 3 (9.9) | 5 (9.9; 2 new, 0.2) | 3 (9.9; 0 new, 0.0) | 0 (0.0) |
| human_vc | green | 16 | 0 | 5 (9.0) | 5 (9.0; 0 new, 0.0) | 7 (9.0; 2 new, 4.2) | 2 (4.2) |
| human_vc_arch | green | 16 | 0 | 5 (9.0) | 19 (9.0; 14 new, 3.4) | 20 (9.0; 15 new, 4.4) | 15 (4.4) |
| human_vc_arch_prop | green | 16 | 0 | 5 (9.0) | 7 (9.0; 2 new, 0.2) | 11 (9.0; 6 new, 2.3) | 6 (2.3) |
| human_vc_prop | green | 16 | 0 | 5 (9.0) | 5 (9.0; 0 new, 0.0) | 11 (9.0; 6 new, 2.2) | 6 (2.2) |
| l20 | green | 27 | 10 | 0 (0.0) | 0 (0.0; 0 new, 0.0) | 0 (0.0; 0 new, 0.0) | 0 (0.0) |
| l20_public | green | 22 | 0 | 12 (28.6) | 10 (28.6; 0 new, 0.0) | 10 (28.6; 0 new, 0.0) | 0 (0.0) |
| leap | green | 71 | 18 | 0 (0.0) | 0 (0.0; 0 new, 0.0) | 1 (0.7; 1 new, 0.7) | 1 (0.7) |
| shadow | red | 38 | 2 | 0 (0.0) | 0 (0.0; 0 new, 0.0) | 0 (0.0; 0 new, 0.0) | 0 (0.0) |
| wall_x3_ring | amber | 10 | 0 | 3 (6.4) | 2 (2.9; 0 new, 0.0) | 2 (3.9; 0 new, 0.0) | 0 (0.0) |

## Is the follow pose the one the verdict was given at?

Each finger's depth into the ball, recomputed at the pose set here with the check's own measure, against the stored value; and how far any finger link sits from where its own solve had it once the whole hand is posed together.

| hand | frames | largest difference from the stored depths | largest link shift in the whole-hand pose |
|---|---|---|---|
| allegro | 2 | 0 | 0 |
| george_clip | 2 | 0 | 0 |
| george_kp | 2 | 0 | 0 |
| george_kp_wide | 2 | 0 | 0 |
| george_measured | 2 | 0 | 0 |
| george_measured_thumb | 2 | 0 | 0 |
| human_vc | 2 | 0 | 0 |
| human_vc_arch | 2 | 0 | 0 |
| human_vc_arch_prop | 2 | 0 | 0 |
| human_vc_prop | 2 | 0 | 0 |
| l20 | 2 | 0 | 0 |
| l20_public | 2 | 0 | 0 |
| leap | 2 | 0 | 0 |
| shadow | 2 | 0 | 0 |
| wall_x3_ring | 2 | 0 | 0 |

## Pairs deeper than 0.5 mm, by name

**allegro** (green)

- rest: none
- generator pose: none
- follow pose, frame 0: palm / th_medial 0.9
- follow pose, frame 1: palm / th_medial 0.9

**george_clip** (green)

- rest: palm / ring_prox 19.2 (rest too); palm / index_prox 18.9 (rest too); palm / middle_prox 18.7 (rest too); palm / thumb_prox 18.1 (rest too); palm / little_prox 14.9 (rest too)
- generator pose: palm / ring_prox 19.2 (rest too); palm / index_prox 18.9 (rest too); palm / middle_prox 18.7 (rest too); palm / thumb_prox 18.1 (rest too); palm / little_prox 14.9 (rest too)
- follow pose, frame 0: palm / ring_prox 19.2 (rest too); palm / index_prox 18.9 (rest too); palm / middle_prox 18.7 (rest too); palm / thumb_prox 18.1 (rest too); palm / little_prox 14.9 (rest too); little_distal / thumb_distal 9.9 (contact points 31.2 apart, radii sum 13.0)
- follow pose, frame 1: palm / ring_prox 19.2 (rest too); palm / index_prox 18.9 (rest too); palm / middle_prox 18.7 (rest too); palm / thumb_prox 18.1 (rest too); palm / little_prox 14.9 (rest too); little_distal / thumb_distal 9.9 (contact points 31.2 apart, radii sum 13.0)

**george_kp** (green)

- rest: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too)
- generator pose: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too)
- follow pose, frame 0: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too); ring_distal / little_distal 1.4
- follow pose, frame 1: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too); ring_distal / little_distal 1.4

**george_kp_wide** (green)

- rest: palm / ring_prox 19.2 (rest too); palm / index_prox 18.9 (rest too); palm / middle_prox 18.7 (rest too); palm / little_prox 14.9 (rest too)
- generator pose: palm / ring_prox 19.2 (rest too); palm / index_prox 18.9 (rest too); palm / middle_prox 18.7 (rest too); palm / little_prox 14.9 (rest too)
- follow pose, frame 0: palm / ring_prox 19.2 (rest too); palm / index_prox 18.9 (rest too); palm / middle_prox 18.7 (rest too); palm / little_prox 14.9 (rest too); ring_prox / little_mid 1.5; ring_mid / little_mid 1.4; ring_mid / little_distal 0.9; ring_prox / little_distal 0.8; middle_distal / ring_distal 0.8 (contact points 13.4 apart, radii sum 17.9)
- follow pose, frame 1: palm / ring_prox 19.2 (rest too); palm / index_prox 18.9 (rest too); palm / middle_prox 18.7 (rest too); palm / little_prox 14.9 (rest too); ring_prox / little_mid 1.5; ring_mid / little_mid 1.4; ring_mid / little_distal 0.9; ring_prox / little_distal 0.8; middle_distal / ring_distal 0.8 (contact points 13.4 apart, radii sum 17.9)

**george_measured** (green)

- rest: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too); palm / thumb_prox 1.1 (rest too)
- generator pose: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too); middle_distal / ring_mid 1.2; palm / thumb_prox 1.1 (rest too); middle_mid / ring_mid 0.9; middle_distal / ring_distal 0.8
- follow pose, frame 0: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too); palm / thumb_prox 1.1 (rest too)
- follow pose, frame 1: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too); palm / thumb_prox 1.1 (rest too)

**george_measured_thumb** (amber)

- rest: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too)
- generator pose: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too)
- follow pose, frame 0: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too)
- follow pose, frame 1: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too)

**human_vc** (green)

- rest: palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / ring_prox 9.0 (rest too); palm / little_prox 9.0 (rest too); palm / thumb_prox 1.0 (rest too)
- generator pose: palm / little_prox 9.0 (rest too); palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / ring_prox 9.0 (rest too); palm / thumb_prox 1.0 (rest too)
- follow pose, frame 0: palm / ring_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / index_prox 9.0 (rest too); palm / little_prox 9.0 (rest too); index_distal / thumb_distal 4.2 (contact points 19.8 apart, radii sum 14.0); index_distal / thumb_mid 2.0; palm / thumb_prox 1.0 (rest too)
- follow pose, frame 1: palm / ring_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / index_prox 9.0 (rest too); palm / little_prox 9.0 (rest too); index_distal / thumb_distal 4.2 (contact points 19.8 apart, radii sum 14.0); index_distal / thumb_mid 2.0; palm / thumb_prox 1.0 (rest too)

**human_vc_arch** (green)

- rest: palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / ring_prox 5.0 (rest too); palm / thumb_prox 1.0 (rest too); palm / little_prox 1.0 (rest too)
- generator pose: palm / middle_prox 9.0 (rest too); palm / index_prox 9.0 (rest too); palm / ring_prox 5.0 (rest too); middle_mid / ring_mid 3.4; ring_mid / little_mid 3.4; ring_distal / little_distal 3.2; middle_distal / ring_distal 3.2; middle_distal / ring_mid 2.6; ring_mid / little_distal 2.4; ring_distal / little_mid 2.4; middle_mid / ring_distal 2.3; ring_prox / little_prox 1.1; middle_prox / ring_prox 1.1; palm / thumb_prox 1.0 (rest too); palm / little_prox 1.0 (rest too)
- follow pose, frame 0: palm / middle_prox 9.0 (rest too); palm / index_prox 9.0 (rest too); palm / ring_prox 5.0 (rest too); ring_mid / little_mid 4.4; middle_mid / ring_mid 3.6; ring_distal / little_mid 3.5; ring_mid / little_distal 3.4; ring_distal / little_distal 3.3 (contact points 8.7 apart, radii sum 14.0); middle_distal / ring_distal 2.7 (contact points 9.2 apart, radii sum 14.0); middle_distal / ring_mid 2.6; ring_prox / little_prox 2.4; middle_mid / ring_distal 2.4; middle_prox / ring_prox 1.7; ring_prox / little_mid 1.6; ring_mid / little_prox 1.5; middle_mid / ring_prox 1.1; palm / thumb_prox 1.0 (rest too); palm / little_prox 1.0 (rest too); middle_prox / ring_mid 0.7
- follow pose, frame 1: palm / middle_prox 9.0 (rest too); palm / index_prox 9.0 (rest too); palm / ring_prox 5.0 (rest too); ring_mid / little_mid 4.4; middle_mid / ring_mid 3.6; ring_distal / little_mid 3.5; ring_mid / little_distal 3.4; ring_distal / little_distal 3.3 (contact points 8.7 apart, radii sum 14.0); middle_distal / ring_distal 2.7 (contact points 9.2 apart, radii sum 14.0); middle_distal / ring_mid 2.6; ring_prox / little_prox 2.4; middle_mid / ring_distal 2.4; middle_prox / ring_prox 1.7; ring_prox / little_mid 1.6; ring_mid / little_prox 1.5; middle_mid / ring_prox 1.1; palm / thumb_prox 1.0 (rest too); palm / little_prox 1.0 (rest too); middle_prox / ring_mid 0.7

**human_vc_arch_prop** (green)

- rest: palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / ring_prox 5.0 (rest too); palm / thumb_prox 1.0 (rest too); palm / little_prox 1.0 (rest too)
- generator pose: palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / ring_prox 5.0 (rest too); palm / little_prox 1.0 (rest too); palm / thumb_prox 1.0 (rest too)
- follow pose, frame 0: palm / middle_prox 9.0 (rest too); palm / index_prox 9.0 (rest too); palm / ring_prox 5.0 (rest too); middle_distal / ring_distal 2.3 (contact points 9.9 apart, radii sum 14.0); middle_mid / ring_mid 1.7; palm / thumb_prox 1.0 (rest too); palm / little_prox 1.0 (rest too); middle_mid / ring_distal 0.7
- follow pose, frame 1: palm / middle_prox 9.0 (rest too); palm / index_prox 9.0 (rest too); palm / ring_prox 5.0 (rest too); middle_distal / ring_distal 2.3 (contact points 9.9 apart, radii sum 14.0); middle_mid / ring_mid 1.7; palm / thumb_prox 1.0 (rest too); palm / little_prox 1.0 (rest too); middle_mid / ring_distal 0.7

**human_vc_prop** (green)

- rest: palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / ring_prox 9.0 (rest too); palm / little_prox 9.0 (rest too); palm / thumb_prox 1.0 (rest too)
- generator pose: palm / middle_prox 9.0 (rest too); palm / little_prox 9.0 (rest too); palm / index_prox 9.0 (rest too); palm / ring_prox 9.0 (rest too); palm / thumb_prox 1.0 (rest too)
- follow pose, frame 0: palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / ring_prox 9.0 (rest too); palm / little_prox 9.0 (rest too); ring_prox / little_mid 2.2; ring_mid / little_mid 1.4; index_mid / middle_prox 1.2; palm / thumb_prox 1.0 (rest too)
- follow pose, frame 1: palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / ring_prox 9.0 (rest too); palm / little_prox 9.0 (rest too); ring_prox / little_mid 2.2; ring_mid / little_mid 1.4; index_mid / middle_prox 1.2; palm / thumb_prox 1.0 (rest too)

**l20** (green)

- rest: none
- generator pose: none
- follow pose, frame 0: none
- follow pose, frame 1: none

**l20_public** (green)

- rest: world / thumb_metacarpals_base1 28.6 (rest too); world / thumb_metacarpals_base2 15.7 (rest too); world / pinky_metacarpals 10.3 (rest too); world / index_metacarpals 9.6 (rest too); world / ring_metacarpals 9.2 (rest too); world / middle_metacarpals 8.0 (rest too); world / pinky_proximal 7.8 (rest too); world / index_proximal 6.8 (rest too); world / ring_proximal 6.4 (rest too); world / thumb_metacarpals 5.8 (rest too); world / middle_proximal 5.0 (rest too); thumb_metacarpals_base1 / thumb_metacarpals 3.3 (rest too)
- generator pose: world / thumb_metacarpals_base1 28.6 (rest too); world / thumb_metacarpals_base2 15.1 (rest too); world / pinky_metacarpals 10.5 (rest too); world / index_metacarpals 9.5 (rest too); world / ring_metacarpals 9.4 (rest too); world / middle_metacarpals 8.2 (rest too); world / pinky_proximal 8.2 (rest too); world / ring_proximal 6.9 (rest too); world / index_proximal 6.5 (rest too); world / middle_proximal 5.5 (rest too)
- follow pose, frame 0: world / thumb_metacarpals_base1 28.6 (rest too); world / thumb_metacarpals_base2 15.2 (rest too); world / pinky_metacarpals 10.5 (rest too); world / index_metacarpals 9.5 (rest too); world / ring_metacarpals 9.4 (rest too); world / middle_metacarpals 8.2 (rest too); world / pinky_proximal 8.2 (rest too); world / ring_proximal 6.9 (rest too); world / index_proximal 6.5 (rest too); world / middle_proximal 5.5 (rest too)
- follow pose, frame 1: world / thumb_metacarpals_base1 28.6 (rest too); world / thumb_metacarpals_base2 15.2 (rest too); world / pinky_metacarpals 10.5 (rest too); world / index_metacarpals 9.5 (rest too); world / ring_metacarpals 9.4 (rest too); world / middle_metacarpals 8.2 (rest too); world / pinky_proximal 8.2 (rest too); world / ring_proximal 6.9 (rest too); world / index_proximal 6.5 (rest too); world / middle_proximal 5.5 (rest too)

**leap** (green)

- rest: none
- generator pose: none
- follow pose, frame 0: mf_ds / rf_ds 0.7 (contact points 25.5 apart, radii sum 25.0)
- follow pose, frame 1: mf_ds / rf_ds 0.7 (contact points 25.5 apart, radii sum 25.0)

**shadow** (red)

- rest: none
- generator pose: none
- follow pose, frame 0: none
- follow pose, frame 1: none

**wall_x3_ring** (amber)

- rest: palm / f0_link0 6.4 (rest too); palm / f1_link0 6.4 (rest too); palm / f2_link0 6.4 (rest too)
- generator pose: palm / f1_link0 2.9 (rest too); palm / f2_link0 2.9 (rest too)
- follow pose, frame 0: palm / f1_link0 3.9 (rest too); palm / f2_link0 3.9 (rest too)
- follow pose, frame 1: palm / f1_link0 3.9 (rest too); palm / f2_link0 3.9 (rest too)

