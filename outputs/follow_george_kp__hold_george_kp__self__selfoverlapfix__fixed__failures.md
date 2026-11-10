# Which fingers and joints fail, and when: George's right hand (measured; thumb from keypoints; VC 1998 muscles; arch assumed)

Leader `hold_george_kp__self__selfoverlapfix`, 2 frames. Read from the stored result; nothing is re-checked. Frames are the leader's frame index.

## The hand as a whole

| when | verdict | reason | stations with no finger | palm into object (mm) | finger in finger beyond rest (mm) | finger in palm beyond rest (mm) |
|---|---|---|---|---|---|---|
| frames 0 to 1 (2) | green |  |  | 0 |  |  |

## Fingers

Green and off spans are left out. "Stated by" says whether a red is the finger's own (its pad misses the contact) or the whole hand's (the pad is in place and the force test fails).

| finger | when | state | cause | stated by | measured |
|---|---|---|---|---|---|
| (none) | | | | | |

## Joints

Past a limit: rad for a hinge, m for a slide. On a stop: the reaction the check credited (N.m, or N for a slide), which helps the hold and is not a failure.

| joint | finger | when | what | side | by | reaction | decided by |
|---|---|---|---|---|---|---|---|
| index_dip | index | frames 0 to 1 (2) | on stop | upper |  | 0.139 | finger |
| middle_dip | middle | frames 0 to 1 (2) | on stop | upper |  | 0.139 | finger |
| ring_dip | ring | frames 0 to 1 (2) | on stop | upper |  | 0.139 | finger |
| little_mcp_add | little | frames 0 to 1 (2) | on stop | upper |  | 0 | finger |
| little_dip | little | frames 0 to 1 (2) | on stop | lower |  | 0.167 | finger |
| thumb_dip | thumb | frames 0 to 1 (2) | on stop | upper |  | 0.138 | finger |

The torque left unbalanced is stated per finger. Which joint of a finger falls short is not in a result file.
