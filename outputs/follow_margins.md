# Both margins per hand

Written by `tools/view_follow.py --table` from the results named in the table.

- Scale margin: how many times larger the leader's load on the object could be and the contacts the hand makes still carry it (below 1 they cannot carry the leader's own load; 0 means the leader's own load already fails).
- Disturbance margin: the smallest extra push or twist on the object, on top of the leader's load and as a multiple of that load's size, that the contacts made could no longer resist; tried along 12 directions (a push and a twist each way along each axis), so the true smallest may be lower.

Each hand's figure is the smallest over the frames checked, and, where some frames are red, also the smallest over the frames that pass. "At least" marks a search that stopped at its cap (1024); a direction such as +fy is the worst push (f) or twist (t) found. Neither margin changes the verdict.

| Hand | Leader | Result | Verdicts | Scale margin | Disturbance margin |
|---|---|---|---|---|---|
| allegro | `hold_allegro__self__selfoverlapfix` | `follow_allegro__hold_allegro__self__selfoverlapfix__margins.json` | green 2/2 | 189, at frame 0 | 52.9 (-ty), at frame 0 |
| george_clip | `hold_george_clip__self__selfoverlapfix` | `follow_george_clip__hold_george_clip__self__selfoverlapfix__margins.json` | green 2/2 | 396, at frame 0 | 38.5 (-tx), at frame 0 |
| george_kp | `hold_george_kp__self__selfoverlapfix` | `follow_george_kp__hold_george_kp__self__selfoverlapfix__margins.json` | green 2/2 | 832, at frame 0 | 29.1 (-tx), at frame 0 |
| george_kp_wide | `hold_george_kp_wide__self__selfoverlapfix` | `follow_george_kp_wide__hold_george_kp_wide__self__selfoverlapfix__margins.json` | green 2/2 | 452, at frame 0 | 56.1 (+tz), at frame 0 |
| george_measured | `hold_george_measured__self__selfoverlapfix` | `follow_george_measured__hold_george_measured__self__selfoverlapfix__margins.json` | green 2/2 | 464, at frame 0 | 33.8 (+fy), at frame 0 |
| george_measured_thumb | `hold_george_measured_thumb__self__selfoverlapfix` | `follow_george_measured_thumb__hold_george_measured_thumb__self__selfoverlapfix__margins.json` | amber 2/2 | at least 1024, at frame 0 | 77.5 (+fy), at frame 0 |
| human_vc | `hold_human_vc__self__selfoverlapfix` | `follow_human_vc__hold_human_vc__self__selfoverlapfix__margins.json` | green 2/2 | 328, at frame 0 | 22.2 (-tx), at frame 0 |
| human_vc_arch | `hold_human_vc_arch__self__selfoverlapfix` | `follow_human_vc_arch__hold_human_vc_arch__self__selfoverlapfix__margins.json` | green 2/2 | 432, at frame 0 | 12.4 (-ty), at frame 0 |
| human_vc_arch_prop | `hold_human_vc_arch_prop__self__selfoverlapfix` | `follow_human_vc_arch_prop__hold_human_vc_arch_prop__self__selfoverlapfix__margins.json` | green 2/2 | 820, at frame 0 | 25.2 (-fz), at frame 0 |
| human_vc_prop | `hold_human_vc_prop__self__selfoverlapfix` | `follow_human_vc_prop__hold_human_vc_prop__self__selfoverlapfix__margins.json` | green 2/2 | 294, at frame 0 | 10 (-tx), at frame 0 |
| l20 | `hold_l20__self__selfoverlapfix` | `follow_l20__hold_l20__self__selfoverlapfix__margins.json` | green 2/2 | 328, at frame 0 | 77.1 (+ty), at frame 0 |
| l20_public | `hold_l20_public__self__selfoverlapfix` | `follow_l20_public__hold_l20_public__self__selfoverlapfix__margins.json` | green 2/2 | 240, at frame 0 | 45.9 (+ty), at frame 0 |
| leap | `hold_leap__self__selfoverlapfix` | `follow_leap__hold_leap__self__selfoverlapfix__margins.json` | green 2/2 | 304, at frame 0 | 16.8 (-tx), at frame 0 |
| shadow | `hold_shadow__self__selfoverlapfix` | `follow_shadow__hold_shadow__self__selfoverlapfix__margins.json` | red 2/2 | 0, at frame 0 | 0, at frame 0 |
| wall_x3_ring | `hold_wall_x3_ring__self__selfoverlapfix` | `follow_wall_x3_ring__hold_wall_x3_ring__self__selfoverlapfix__margins.json` | amber 2/2 | 1.48, at frame 0 | 0.0838 (+tx), at frame 0 |
| wall_x3_ring | `w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4__contactfix` | `follow_wall_x3_ring__w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4__contactfix__margins.json` | amber 5/40, red 35/40 | 0, at frame 0; over the frames that pass, 1.04, at frame 22 | 0, at frame 0; over the frames that pass, 0.0259 (-fx), at frame 144 |
