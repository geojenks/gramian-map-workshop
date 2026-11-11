## Steps

| Log | Step | Command | Exit | Seconds |
|---|---|---|---|---|
| `logs/run0_log.txt` | curves_to_record | `python (analysis) tools/curves_to_record.py data/test_curves/mosadegh2014_fPN.csv --out rt_out/r0/mosadegh_from_curves.json` | 0 | 1 |
| `logs/run0_log.txt` | check_curves_record | `python (analysis) tools/check_record.py rt_out/r0/mosadegh_from_curves.json` | 0 | 0 |
| `logs/run0_log.txt` | table_to_record | `python (analysis) tools/joint_table_to_record.py data/joint_tables/shadow_hand_finger.csv --out rt_out/r0/shadow_finger_from_table.json` | 0 | 0 |
| `logs/run0_log.txt` | check_table_record | `python (analysis) tools/check_record.py rt_out/r0/shadow_finger_from_table.json` | 0 | 1 |
| `logs/run0_log.txt` | build_curves_record_by_id | `python (hands) tools/build_record_hand.py data/hands/rt0_curves_by_id/hand.json` | 1 | 0 |
| `logs/run0_log.txt` | build_curves_record_by_path | `python (hands) tools/build_record_hand.py data/hands/rt0_curves_by_path/hand.json` | 1 | 1 |
| `logs/run0_log.txt` | build_table_record_by_path | `python (hands) tools/build_record_hand.py data/hands/rt0_table_by_path/hand.json` | 1 | 0 |
| `logs/run0_log.txt` | gramians_mujoco_python | `python (hands) tools/figure_gramians.py --records data/records --out-dir rt_out/r0/gramians` | 1 | 1 |
| `logs/run0_log.txt` | gramians_analysis_python | `python (analysis) tools/figure_gramians.py --records data/records --out-dir rt_out/r0/gramians` | 0 | 1 |
| `logs/run_log.txt` | add_mosadegh | `python (hands) tools/add_hand.py data/hands/mosadegh2014_fPN_from_curves_x3_ring/hand.xml rt_out/mosadegh_x3_ring.json` | 0 | 0 |
| `logs/run_log.txt` | add_allegro | `python (hands) tools/add_hand.py external/mujoco_menagerie/wonik_allegro/left_hand.xml rt_out/allegro_left.json` | 0 | 1 |
| `logs/run_log.txt` | list | `python (hands) tools/add_hand.py --list` | 0 | 0 |
| `logs/run_log.txt` | check_leader_sim | `python (hands) tools/check_leader.py data/leaders/w10_cup_physical_x0.1_a0.15_n4__contactfix.json` | 0 | 1 |
| `logs/run_log.txt` | check_leader_human | `python (hands) tools/check_leader.py data/leaders/tennis_new_cup_take3_handframe__nframesfix.json` | 0 | 0 |
| `logs/run_log.txt` | gramians | `python (hands) tools/figure_gramians.py --records data/records --out-dir rt_out/gramians` | 1 | 0 |
| `logs/run_log.txt` | hold_mosadegh | `python (hands) tools/make_hold_leader.py mosadegh_x3_ring --out-dir rt_out/leaders --results-dir rt_out/results` | 0 | 14 |
| `logs/run_log.txt` | hold_allegro | `python (hands) tools/make_hold_leader.py allegro_left --out-dir rt_out/leaders --results-dir rt_out/results` | 0 | 43 |
| `logs/run3_log.txt` | build | `python (hands) tools/build_record_hand.py data/hands/mosadegh2014_fPN_from_curves_x3_ring_flipped/hand.json` | 0 | 1 |
| `logs/run3_log.txt` | add | `python (hands) tools/add_hand.py data/hands/mosadegh2014_fPN_from_curves_x3_ring_flipped/hand.xml rt_out/mosadegh_x3_ring_flipped.json` | 0 | 0 |
| `logs/run3_log.txt` | hold | `python (hands) tools/make_hold_leader.py mosadegh_x3_ring_flipped --out-dir rt_out/leaders --results-dir rt_out/results` | 0 | 8 |
| `logs/run3_log.txt` | follow | `python (hands) tools/run_follow.py --hands mosadegh_x3_ring_flipped --leader-id hold_mosadegh_x3_ring_flipped__self__selfoverlapfix --placement-from rt_out/results/hold_placement_mosadegh_x3_ring_flipped__selfoverlapfix.json --tag fixed --margins --out-dir rt_out/results` | 0 | 105 |
| `logs/run3_log.txt` | failures | `python (analysis) tools/report_failures.py rt_out/results/follow_mosadegh_x3_ring_flipped__hold_mosadegh_x3_ring_flipped__self__selfoverlapfix__fixed.json --out-dir rt_out/results` | 0 | 0 |
| `logs/run2_log.txt` | follow_mosadegh_x3_ring | `python (hands) tools/run_follow.py --hands mosadegh_x3_ring --leader-id hold_mosadegh_x3_ring__self__selfoverlapfix --placement-from rt_out/results/hold_placement_mosadegh_x3_ring__selfoverlapfix.json --tag fixed --margins --out-dir rt_out/results` | 0 | 25 |
| `logs/run2_log.txt` | failures_mosadegh_x3_ring | `python (analysis) tools/report_failures.py rt_out/results/follow_mosadegh_x3_ring__hold_mosadegh_x3_ring__self__selfoverlapfix__fixed.json --out-dir rt_out/results` | 0 | 1 |
| `logs/run2_log.txt` | follow_allegro_left | `python (hands) tools/run_follow.py --hands allegro_left --leader-id hold_allegro_left__self__selfoverlapfix --placement-from rt_out/results/hold_placement_allegro_left__selfoverlapfix.json --tag fixed --margins --out-dir rt_out/results` | 0 | 62 |
| `logs/run2_log.txt` | failures_allegro_left | `python (analysis) tools/report_failures.py rt_out/results/follow_allegro_left__hold_allegro_left__self__selfoverlapfix__fixed.json --out-dir rt_out/results` | 0 | 0 |
| `logs/run2_log.txt` | view | `python (analysis) tools/view_follow.py rt_out/results/follow_mosadegh_x3_ring__hold_mosadegh_x3_ring__self__selfoverlapfix__fixed.json rt_out/results/follow_allegro_left__hold_allegro_left__self__selfoverlapfix__fixed.json --out rt_out/results/follow_runthrough.html --table rt_out/results/follow_runthrough.md` | 0 | 0 |

All steps together: 266 s.

## Verdicts

| Result | Verdicts | Scale margin | Disturbance margin | Joints posed past a limit | Follow time (s) | Units caveat in the result |
|---|---|---|---|---|---|---|
| `results/follow_allegro_left__hold_allegro_left__self__selfoverlapfix__fixed.json` | green 40/40 | 217 | 59.1 (-ty) | none | 61.4 | no |
| `results/follow_mosadegh_x3_ring__hold_mosadegh_x3_ring__self__selfoverlapfix__fixed.json` | red 40/40 (no_contact) | not computed | not computed | none | 24.1 | no |
| `results/follow_mosadegh_x3_ring_flipped__hold_mosadegh_x3_ring_flipped__self__selfoverlapfix__fixed.json` | green 40/40 | 5.76 | 0.497 (-tx) | f0_hinge0 upper by 0.115 to 0.115 in 40 finger-frames; f1_hinge0 upper by 0.115 to 0.115 in 40 finger-frames; f2_hinge0 upper by 0.115 to 0.115 in 40 finger-frames | 103.8 | no |
