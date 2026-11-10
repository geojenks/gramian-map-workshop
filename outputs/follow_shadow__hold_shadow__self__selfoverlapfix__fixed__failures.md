# Which fingers and joints fail, and when: Shadow Dexterous Hand

Leader `hold_shadow__self__selfoverlapfix`, 2 frames. Read from the stored result; nothing is re-checked. Frames are the leader's frame index.

## The hand as a whole

| when | verdict | reason | stations with no finger | palm into object (mm) | finger in finger beyond rest (mm) | finger in palm beyond rest (mm) |
|---|---|---|---|---|---|---|
| frames 0 to 1 (2) | red | torque |  | 0 |  |  |

## Fingers

Green and off spans are left out. "Stated by" says whether a red is the finger's own (its pad misses the contact) or the whole hand's (the pad is in place and the force test fails).

| finger | when | state | cause | stated by | measured |
|---|---|---|---|---|---|
| index | frames 0 to 1 (2) | red | torque | the whole-hand force test | off its contact by 0.164 mm; facing 28.8 deg off; into the object 0.686 mm; unbalanced 0.00145 N.m |
| middle | frames 0 to 1 (2) | red | torque | the whole-hand force test | off its contact by 0.16 mm; facing 12.9 deg off; into the object 0.455 mm; unbalanced 0.00151 N.m |
| ring | frames 0 to 1 (2) | red | torque | the whole-hand force test | off its contact by 0.161 mm; facing 57.1 deg off; into the object 0.202 mm; unbalanced 0.00096 N.m |
| little | frames 0 to 1 (2) | red | torque | the whole-hand force test | off its contact by 0.437 mm; facing 52 deg off; into the object 0 mm; unbalanced 0.00066 N.m |
| thumb | frames 0 to 1 (2) | red | torque | the whole-hand force test | off its contact by 1.3 mm; facing 41.3 deg off; into the object 4.67 mm |

## Joints

Past a limit: rad for a hinge, m for a slide. On a stop: the reaction the check credited (N.m, or N for a slide), which helps the hold and is not a failure.

| joint | finger | when | what | side | by | reaction | decided by |
|---|---|---|---|---|---|---|---|
| rh_LFJ5 | little | frames 0 to 1 (2) | on stop | upper |  | 0 | redistribution |
| rh_LFJ4 | little | frames 0 to 1 (2) | on stop | lower |  | 0 | redistribution |
| rh_LFJ3 | little | frames 0 to 1 (2) | on stop | lower |  | 0 | redistribution |
| rh_THJ5 | thumb | frames 0 to 1 (2) | on stop | upper |  | 0 | redistribution |
| rh_THJ3 | thumb | frames 0 to 1 (2) | on stop | lower |  | 0 | redistribution |
| rh_THJ1 | thumb | frames 0 to 1 (2) | on stop | lower |  | 0 | redistribution |

The torque left unbalanced is stated per finger. Which joint of a finger falls short is not in a result file.
