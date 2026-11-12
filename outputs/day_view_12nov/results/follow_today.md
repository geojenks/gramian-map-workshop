# Both margins per hand

Written by `tools/view_follow.py --table` from the results named in the table.

- Scale margin: how many times larger the leader's load on the object could be and the contacts the hand makes still carry it (below 1 they cannot carry the leader's own load; 0 means the leader's own load already fails).
- Disturbance margin: the smallest extra push or twist on the object, on top of the leader's load and as a multiple of that load's size, that the contacts made could no longer resist; tried along 12 directions (a push and a twist each way along each axis), so the true smallest may be lower.

Each hand's figure is the smallest over the frames checked, and, where some frames are red, also the smallest over the frames that pass. "At least" marks a search that stopped at its cap (1024); a direction such as +fy is the worst push (f) or twist (t) found. Neither margin changes the verdict.

| Hand | Leader | Result | Verdicts | Scale margin | Disturbance margin |
|---|---|---|---|---|---|
| allegro_left | `hold_allegro_left__self__selfoverlapfix` | `follow_allegro_left__hold_allegro_left__self__selfoverlapfix__fixed.json` | green 40/40 | 217, at frame 0 | 59.1 (-ty), at frame 0 |
| mosadegh_x3_ring | `hold_mosadegh_x3_ring__self__selfoverlapfix` | `follow_mosadegh_x3_ring__hold_mosadegh_x3_ring__self__selfoverlapfix__fixed.json` | red 40/40 | not computed (no contact or no torque checked) | not computed (no contact or no torque checked) |
| mosadegh_x3_ring_flipped | `hold_mosadegh_x3_ring_flipped__self__selfoverlapfix` | `follow_mosadegh_x3_ring_flipped__hold_mosadegh_x3_ring_flipped__self__selfoverlapfix__fixed.json` | green 40/40 | 5.76, at frame 0 | 0.497 (-tx), at frame 0 |
