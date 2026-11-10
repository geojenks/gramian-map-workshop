# Which fingers and joints fail, and when: Three record-fitted stand-ins of wall_0.0010m on a ring (example)

Leader `hold_wall_x3_ring__self__selfoverlapfix`, 2 frames. Read from the stored result; nothing is re-checked. Frames are the leader's frame index.

## The hand as a whole

| when | verdict | reason | stations with no finger | palm into object (mm) | finger in finger beyond rest (mm) | finger in palm beyond rest (mm) |
|---|---|---|---|---|---|---|
| frames 0 to 1 (2) | amber |  |  | 0 |  |  |

## Fingers

Green and off spans are left out. "Stated by" says whether a red is the finger's own (its pad misses the contact) or the whole hand's (the pad is in place and the force test fails).

| finger | when | state | cause | stated by | measured |
|---|---|---|---|---|---|
| f0 | frames 0 to 1 (2) | amber |  |  | off its contact by 0.998 mm; facing 42.7 deg off; into the object 0 mm; unbalanced 0.00128 N.m; unbalanced (slides) 0.123 N |
| f1 | frames 0 to 1 (2) | amber |  |  | off its contact by 1.24 mm; facing 59.8 deg off; into the object 0 mm; unbalanced 0.000691 N.m; unbalanced (slides) 0.0725 N |
| f2 | frames 0 to 1 (2) | amber |  |  | off its contact by 1.24 mm; facing 59.8 deg off; into the object 0 mm; unbalanced 0.000691 N.m; unbalanced (slides) 0.0725 N |

## Joints

Past a limit: rad for a hinge, m for a slide. On a stop: the reaction the check credited (N.m, or N for a slide), which helps the hold and is not a failure.

| joint | finger | when | what | side | by | reaction | decided by |
|---|---|---|---|---|---|---|---|
| f0_hinge2 | f0 | frames 0 to 1 (2) | past limit | upper | 0.00613 |  |  |
| f1_hinge2 | f1 | frames 0 to 1 (2) | past limit | upper | 0.0112 |  |  |
| f2_hinge2 | f2 | frames 0 to 1 (2) | past limit | upper | 0.0112 |  |  |

The torque left unbalanced is stated per finger. Which joint of a finger falls short is not in a result file.
