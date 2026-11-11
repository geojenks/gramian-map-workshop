# Which fingers and joints fail, and when: Three stand-ins of the record made from the Mosadegh 2014 test curves, on a ring, bend axis outward (run-through example)

Leader `hold_mosadegh_x3_ring_flipped__self__selfoverlapfix`, 40 frames. Read from the stored result; nothing is re-checked. Frames are numbered by their position in the result, from 0: the leader's frame index repeats in it.

## The hand as a whole

| when | verdict | reason | stations with no finger | palm into object (mm) | finger in finger beyond rest (mm) | finger in palm beyond rest (mm) |
|---|---|---|---|---|---|---|
| frames 0 to 39 (40) | green |  |  | 0 | 0 | 3.47e-15 |

## Fingers

Green and off spans are left out. "Stated by" says whether a red is the finger's own (its pad misses the contact) or the whole hand's (the pad is in place and the force test fails).

| finger | when | state | cause | stated by | measured |
|---|---|---|---|---|---|
| (none) | | | | | |

## Joints

Past a limit: rad for a hinge, m for a slide. On a stop: the reaction the check credited (N.m, or N for a slide), which helps the hold and is not a failure.

| joint | finger | when | what | side | by | reaction | decided by |
|---|---|---|---|---|---|---|---|
| f0_hinge0 | f0 | frames 0 to 39 (40) | past limit | upper | 0.115 |  |  |
| f1_hinge0 | f1 | frames 0 to 39 (40) | past limit | upper | 0.115 |  |  |
| f2_hinge0 | f2 | frames 0 to 39 (40) | past limit | upper | 0.115 |  |  |

The torque left unbalanced is stated per finger. Which joint of a finger falls short is not in a result file.
