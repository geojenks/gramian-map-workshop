# Self-overlap of the hand at the stored self-hold poses

Written by `tools/hold_pose_self_overlap.py` at 51793c7dbf1d854a54497f294bd610ea45d1243e. Depths in mm. A pair is two of the hand's own bodies whose collision geoms penetrate in MuJoCo (the ball left out). MuJoCo drops contacts between a body and its parent, between pairs the model excludes, and for geoms with no collision bits, so the counts are a lower bound. These are facts about the poses of stored results, not new verdicts.

## Pairs by pose

| hand | stored verdict | colliding geoms | model's contact excludes | pairs at rest (deepest) | pairs at generator pose (deepest; not at rest) | pairs at follow pose, first frame (deepest; not at rest) | of those, finger against another finger (deepest) |
|---|---|---|---|---|---|---|---|
| allegro | green | 21 | 5 | 0 (0.0) | 1 (15.7; 1 new, 15.7) | 1 (17.5; 1 new, 17.5) | 1 (17.5) |
| george_clip | green | 23 | 0 | 5 (19.2) | 9 (19.2; 4 new, 1.4) | 8 (19.2; 3 new, 1.4) | 3 (1.4) |
| george_kp | green | 16 | 0 | 3 (9.9) | 6 (9.9; 3 new, 1.4) | 3 (9.9; 0 new, 0.0) | 0 (0.0) |
| george_kp_wide | green | 22 | 0 | 4 (19.2) | 9 (19.2; 5 new, 4.3) | 13 (19.2; 9 new, 3.7) | 9 (3.7) |
| george_measured | green | 16 | 0 | 4 (9.9) | 8 (9.9; 4 new, 1.2) | 4 (9.9; 0 new, 0.0) | 0 (0.0) |
| george_measured_thumb | green | 16 | 0 | 3 (9.9) | 7 (9.9; 4 new, 1.2) | 4 (9.9; 1 new, 0.4) | 1 (0.4) |
| human_vc | green | 16 | 0 | 5 (9.0) | 5 (9.0; 0 new, 0.0) | 7 (9.0; 2 new, 4.2) | 2 (4.2) |
| human_vc_arch | green | 16 | 0 | 5 (9.0) | 19 (9.0; 14 new, 4.4) | 17 (9.0; 12 new, 8.0) | 12 (8.0) |
| human_vc_arch_prop | green | 16 | 0 | 5 (9.0) | 13 (9.0; 8 new, 6.7) | 12 (9.0; 7 new, 8.5) | 7 (8.5) |
| human_vc_prop | green | 16 | 0 | 5 (9.0) | 5 (9.0; 0 new, 0.0) | 11 (9.0; 6 new, 2.2) | 6 (2.2) |
| leap | green | 71 | 18 | 0 (0.0) | 1 (6.0; 1 new, 6.0) | 1 (10.6; 1 new, 10.6) | 1 (10.6) |
| shadow | red | 38 | 2 | 0 (0.0) | 6 (15.8; 6 new, 15.8) | 7 (11.1; 7 new, 11.1) | 6 (11.1) |
| wall_x3_ring | green | 10 | 0 | 3 (6.4) | 12 (12.9; 12 new, 12.9) | 15 (12.9; 12 new, 12.9) | 12 (12.9) |

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
| leap | 2 | 0 | 0 |
| shadow | 2 | 0 | 0 |
| wall_x3_ring | 2 | 0 | 0 |

## Pairs deeper than 0.5 mm, by name

**allegro** (green)

- rest: none
- generator pose: ff_tip / th_tip 15.7
- follow pose, frame 0: ff_tip / th_tip 17.5 (contact points 5.1 apart, radii sum 24.0)
- follow pose, frame 1: ff_tip / th_tip 17.5 (contact points 5.1 apart, radii sum 24.0)

**george_clip** (green)

- rest: palm / ring_prox 19.2 (rest too); palm / index_prox 18.9 (rest too); palm / middle_prox 18.7 (rest too); palm / thumb_prox 18.1 (rest too); palm / little_prox 14.9 (rest too)
- generator pose: palm / ring_prox 19.2 (rest too); palm / index_prox 18.9 (rest too); palm / middle_prox 18.7 (rest too); palm / thumb_prox 18.1 (rest too); palm / little_prox 14.9 (rest too); middle_distal / ring_mid 1.4; middle_distal / ring_distal 1.4; middle_mid / ring_mid 0.8
- follow pose, frame 0: palm / ring_prox 19.2 (rest too); palm / index_prox 18.9 (rest too); palm / middle_prox 18.7 (rest too); palm / thumb_prox 18.1 (rest too); palm / little_prox 14.9 (rest too); middle_distal / ring_distal 1.4 (contact points 13.5 apart, radii sum 17.9)
- follow pose, frame 1: palm / ring_prox 19.2 (rest too); palm / index_prox 18.9 (rest too); palm / middle_prox 18.7 (rest too); palm / thumb_prox 18.1 (rest too); palm / little_prox 14.9 (rest too); middle_distal / ring_distal 1.4 (contact points 13.5 apart, radii sum 17.9)

**george_kp** (green)

- rest: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too)
- generator pose: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too); middle_mid / ring_mid 1.4; middle_prox / ring_mid 1.1; middle_prox / ring_prox 0.9
- follow pose, frame 0: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too)
- follow pose, frame 1: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too)

**george_kp_wide** (green)

- rest: palm / ring_prox 19.2 (rest too); palm / index_prox 18.9 (rest too); palm / middle_prox 18.7 (rest too); palm / little_prox 14.9 (rest too)
- generator pose: palm / ring_prox 19.2 (rest too); palm / index_prox 18.9 (rest too); palm / middle_prox 18.7 (rest too); palm / little_prox 14.9 (rest too); middle_distal / ring_distal 4.3; middle_mid / ring_mid 1.7; middle_distal / ring_mid 1.3; middle_mid / ring_distal 1.2
- follow pose, frame 0: palm / ring_prox 19.2 (rest too); palm / index_prox 18.9 (rest too); palm / middle_prox 18.7 (rest too); palm / little_prox 14.9 (rest too); middle_distal / ring_distal 3.7 (contact points 10.9 apart, radii sum 17.9); middle_mid / ring_mid 2.6; middle_mid / ring_distal 2.1; ring_prox / little_mid 1.8; ring_mid / little_mid 1.7; middle_distal / ring_mid 1.5; ring_mid / little_distal 1.2; ring_prox / little_distal 1.2
- follow pose, frame 1: palm / ring_prox 19.2 (rest too); palm / index_prox 18.9 (rest too); palm / middle_prox 18.7 (rest too); palm / little_prox 14.9 (rest too); middle_distal / ring_distal 3.7 (contact points 10.9 apart, radii sum 17.9); middle_mid / ring_mid 2.6; middle_mid / ring_distal 2.1; ring_prox / little_mid 1.8; ring_mid / little_mid 1.7; middle_distal / ring_mid 1.5; ring_mid / little_distal 1.2; ring_prox / little_distal 1.2

**george_measured** (green)

- rest: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too); palm / thumb_prox 1.1 (rest too)
- generator pose: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too); middle_distal / ring_mid 1.2; palm / thumb_prox 1.1 (rest too); middle_mid / ring_mid 0.9; middle_distal / ring_distal 0.8
- follow pose, frame 0: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too); palm / thumb_prox 1.1 (rest too)
- follow pose, frame 1: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too); palm / thumb_prox 1.1 (rest too)

**george_measured_thumb** (green)

- rest: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too)
- generator pose: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too); middle_distal / ring_mid 1.2; middle_mid / ring_mid 0.9; middle_distal / ring_distal 0.8
- follow pose, frame 0: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too)
- follow pose, frame 1: palm / index_prox 9.9 (rest too); palm / ring_prox 6.2 (rest too); palm / middle_prox 2.2 (rest too)

**human_vc** (green)

- rest: palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / ring_prox 9.0 (rest too); palm / little_prox 9.0 (rest too); palm / thumb_prox 1.0 (rest too)
- generator pose: palm / little_prox 9.0 (rest too); palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / ring_prox 9.0 (rest too); palm / thumb_prox 1.0 (rest too)
- follow pose, frame 0: palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / ring_prox 9.0 (rest too); palm / little_prox 9.0 (rest too); index_distal / thumb_distal 4.2 (contact points 19.8 apart, radii sum 14.0); index_distal / thumb_mid 2.0; palm / thumb_prox 1.0 (rest too)
- follow pose, frame 1: palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / ring_prox 9.0 (rest too); palm / little_prox 9.0 (rest too); index_distal / thumb_distal 4.2 (contact points 19.8 apart, radii sum 14.0); index_distal / thumb_mid 2.0; palm / thumb_prox 1.0 (rest too)

**human_vc_arch** (green)

- rest: palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / ring_prox 5.0 (rest too); palm / thumb_prox 1.0 (rest too); palm / little_prox 1.0 (rest too)
- generator pose: palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / ring_prox 5.0 (rest too); middle_mid / ring_mid 4.4; middle_mid / ring_distal 3.4; middle_distal / ring_mid 3.4; ring_distal / little_mid 3.2; middle_prox / ring_prox 2.9; middle_distal / ring_distal 2.7; ring_mid / little_mid 2.6; ring_distal / little_distal 2.2; middle_mid / ring_prox 2.0; middle_prox / ring_mid 1.6; ring_prox / little_prox 1.2; ring_mid / little_distal 1.1; palm / thumb_prox 1.0 (rest too); palm / little_prox 1.0 (rest too); ring_mid / little_prox 0.9
- follow pose, frame 0: palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); index_distal / thumb_mid 8.0; index_distal / thumb_distal 6.9; palm / ring_prox 5.0 (rest too); ring_mid / little_mid 3.9; ring_prox / little_prox 3.1; index_mid / thumb_mid 3.1; ring_mid / little_distal 2.9; ring_distal / little_mid 2.9; ring_distal / little_distal 2.6 (contact points 9.3 apart, radii sum 14.0); index_mid / thumb_distal 2.5 (contact points 17.3 apart, radii sum 15.0); middle_distal / ring_distal 2.4 (contact points 9.4 apart, radii sum 14.0); ring_mid / little_prox 2.1; ring_prox / little_mid 1.9; palm / little_prox 1.0 (rest too); palm / thumb_prox 1.0 (rest too)
- follow pose, frame 1: palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); index_distal / thumb_mid 8.0; index_distal / thumb_distal 6.9; palm / ring_prox 5.0 (rest too); ring_mid / little_mid 3.9; ring_prox / little_prox 3.1; index_mid / thumb_mid 3.1; ring_mid / little_distal 2.9; ring_distal / little_mid 2.9; ring_distal / little_distal 2.6 (contact points 9.3 apart, radii sum 14.0); index_mid / thumb_distal 2.5 (contact points 17.3 apart, radii sum 15.0); middle_distal / ring_distal 2.4 (contact points 9.4 apart, radii sum 14.0); ring_mid / little_prox 2.1; ring_prox / little_mid 1.9; palm / little_prox 1.0 (rest too); palm / thumb_prox 1.0 (rest too)

**human_vc_arch_prop** (green)

- rest: palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / ring_prox 5.0 (rest too); palm / thumb_prox 1.0 (rest too); palm / little_prox 1.0 (rest too)
- generator pose: palm / middle_prox 9.0 (rest too); palm / index_prox 9.0 (rest too); middle_mid / ring_mid 6.7; middle_distal / ring_distal 6.0; middle_mid / ring_distal 5.9; palm / ring_prox 5.0 (rest too); middle_distal / ring_mid 4.7; middle_prox / ring_prox 3.6; middle_prox / ring_mid 2.7; middle_mid / ring_prox 1.1; palm / thumb_prox 1.0 (rest too); palm / little_prox 1.0 (rest too)
- follow pose, frame 0: palm / middle_prox 9.0 (rest too); palm / index_prox 9.0 (rest too); middle_mid / ring_mid 8.5; middle_mid / ring_distal 7.5; middle_distal / ring_distal 6.6 (contact points 6.0 apart, radii sum 14.0); middle_distal / ring_mid 6.6; middle_prox / ring_prox 5.0; palm / ring_prox 5.0 (rest too); middle_prox / ring_mid 4.0; middle_mid / ring_prox 2.5; palm / thumb_prox 1.0 (rest too); palm / little_prox 1.0 (rest too)
- follow pose, frame 1: palm / middle_prox 9.0 (rest too); palm / index_prox 9.0 (rest too); middle_mid / ring_mid 8.5; middle_mid / ring_distal 7.5; middle_distal / ring_distal 6.6 (contact points 6.0 apart, radii sum 14.0); middle_distal / ring_mid 6.6; middle_prox / ring_prox 5.0; palm / ring_prox 5.0 (rest too); middle_prox / ring_mid 4.0; middle_mid / ring_prox 2.5; palm / thumb_prox 1.0 (rest too); palm / little_prox 1.0 (rest too)

**human_vc_prop** (green)

- rest: palm / index_prox 9.0 (rest too); palm / middle_prox 9.0 (rest too); palm / ring_prox 9.0 (rest too); palm / little_prox 9.0 (rest too); palm / thumb_prox 1.0 (rest too)
- generator pose: palm / middle_prox 9.0 (rest too); palm / little_prox 9.0 (rest too); palm / index_prox 9.0 (rest too); palm / ring_prox 9.0 (rest too); palm / thumb_prox 1.0 (rest too)
- follow pose, frame 0: palm / middle_prox 9.0 (rest too); palm / index_prox 9.0 (rest too); palm / ring_prox 9.0 (rest too); palm / little_prox 9.0 (rest too); ring_prox / little_mid 2.2; ring_mid / little_mid 1.4; index_mid / middle_prox 1.2; palm / thumb_prox 1.0 (rest too)
- follow pose, frame 1: palm / middle_prox 9.0 (rest too); palm / index_prox 9.0 (rest too); palm / ring_prox 9.0 (rest too); palm / little_prox 9.0 (rest too); ring_prox / little_mid 2.2; ring_mid / little_mid 1.4; index_mid / middle_prox 1.2; palm / thumb_prox 1.0 (rest too)

**leap** (green)

- rest: none
- generator pose: if_ds / th_ds 6.0
- follow pose, frame 0: if_ds / th_ds 10.6 (contact points 27.3 apart, radii sum 27.9)
- follow pose, frame 1: if_ds / th_ds 10.6 (contact points 27.3 apart, radii sum 27.9)

**shadow** (red)

- rest: none
- generator pose: rh_rfdistal / rh_lfmiddle 15.8; rh_ffdistal / rh_thdistal 6.3; rh_rfdistal / rh_lfproximal 4.7; rh_rfmiddle / rh_lfmiddle 4.3; rh_rfmiddle / rh_lfproximal 3.8; rh_rfdistal / rh_lfdistal 2.8
- follow pose, frame 0: rh_rfdistal / rh_lfmiddle 11.1; rh_rfdistal / rh_lfproximal 9.9; rh_rfmiddle / rh_lfproximal 6.6; rh_rfmiddle / rh_lfmiddle 6.6; rh_ffdistal / rh_thdistal 5.1 (contact points 15.2 apart, radii sum 17.0); rh_rfdistal / rh_lfdistal 1.1 (contact points 15.6 apart, radii sum 14.7)
- follow pose, frame 1: rh_rfdistal / rh_lfmiddle 11.1; rh_rfdistal / rh_lfproximal 9.9; rh_rfmiddle / rh_lfproximal 6.6; rh_rfmiddle / rh_lfmiddle 6.6; rh_ffdistal / rh_thdistal 5.1 (contact points 15.2 apart, radii sum 17.0); rh_rfdistal / rh_lfdistal 1.1 (contact points 15.6 apart, radii sum 14.7)

**wall_x3_ring** (green)

- rest: palm / f0_link0 6.4 (rest too); palm / f1_link0 6.4 (rest too); palm / f2_link0 6.4 (rest too)
- generator pose: f1_distal / f2_distal 12.9; f0_distal / f2_distal 12.9; f0_distal / f1_distal 12.9; f1_link1 / f2_distal 5.9; f1_distal / f2_link1 5.9; f0_distal / f1_link1 5.9; f0_distal / f2_link1 5.9; f0_link1 / f1_distal 5.9; f0_link1 / f2_distal 5.9; f1_link1 / f2_link1 4.0; f0_link1 / f1_link1 4.0; f0_link1 / f2_link1 4.0
- follow pose, frame 0: f1_distal / f2_distal 12.9 (contact points 13.0 apart, radii sum 12.9); f0_distal / f1_distal 12.9 (contact points 13.0 apart, radii sum 12.9); f0_distal / f2_distal 12.9 (contact points 13.0 apart, radii sum 12.9); f1_link1 / f2_distal 3.3; f1_distal / f2_link1 3.3; f0_distal / f1_link1 3.3; f0_distal / f2_link1 3.3; f0_link1 / f2_distal 3.3; f0_link1 / f1_distal 3.3; palm / f2_link0 1.4 (rest too); palm / f0_link0 1.4 (rest too); palm / f1_link0 1.4 (rest too); f1_link1 / f2_link1 0.9; f0_link1 / f2_link1 0.9; f0_link1 / f1_link1 0.9
- follow pose, frame 1: f1_distal / f2_distal 12.9 (contact points 13.0 apart, radii sum 12.9); f0_distal / f1_distal 12.9 (contact points 13.0 apart, radii sum 12.9); f0_distal / f2_distal 12.9 (contact points 13.0 apart, radii sum 12.9); f1_link1 / f2_distal 3.3; f1_distal / f2_link1 3.3; f0_distal / f1_link1 3.3; f0_distal / f2_link1 3.3; f0_link1 / f2_distal 3.3; f0_link1 / f1_distal 3.3; palm / f2_link0 1.4 (rest too); palm / f0_link0 1.4 (rest too); palm / f1_link0 1.4 (rest too); f1_link1 / f2_link1 0.9; f0_link1 / f2_link1 0.9; f0_link1 / f1_link1 0.9

