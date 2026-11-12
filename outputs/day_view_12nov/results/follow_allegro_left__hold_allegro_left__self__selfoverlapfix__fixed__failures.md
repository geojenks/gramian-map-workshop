# Which fingers and joints fail, and when: Allegro left hand (MuJoCo Menagerie)

Leader `hold_allegro_left__self__selfoverlapfix`, 40 frames. Read from the stored result; nothing is re-checked. Frames are numbered by their position in the result, from 0: the leader's frame index repeats in it.

## The hand as a whole

| when | verdict | reason | stations with no finger | palm into object (mm) | finger in finger beyond rest (mm) | finger in palm beyond rest (mm) |
|---|---|---|---|---|---|---|
| frames 0 to 39 (40) | green |  |  | 0 | 0 | 1.14 |

## Fingers

Green and off spans are left out. "Stated by" says whether a red is the finger's own (its pad misses the contact) or the whole hand's (the pad is in place and the force test fails).

"Own tests missed" is blank where the result does not say.

| finger | when | state | cause | stated by | measured | own tests missed |
|---|---|---|---|---|---|---|
| (none) | | | | | | |

## Joint balance left over

For each amber or red span, what a torque test left unbalanced at each joint: the residual largest in size over the span (signed; N.m on a hinge, N on a slide) and the frame it is at. A test holds the norm of its rows to its bar, not each row. For the finger's own test the bar is on that finger's rows of one joint type. For the whole-hand test it is on all its rows, the object's included, so a joint's residual is a share of it and the bar, one number over rows in N and N.m, is given without a unit; that test has rows only at frames where it reached its torque stage, so none at a frame red on direction or friction. Residuals equal in size share the last column (a coupled pair of joints carries equal and opposite ones); a residual that is rounding noise prints as 0 and names no joint.

| finger | when | test | joint | largest residual | at frame | bar on the norm | largest of its joint type |
|---|---|---|---|---|---|---|---|

## Joints

Past a limit: rad for a hinge, m for a slide. On a stop: the reaction the check credited (N.m, or N for a slide), which helps the hold and is not a failure.

| joint | finger | when | what | side | by | reaction | decided by |
|---|---|---|---|---|---|---|---|
| thj2 | thumb | frames 0 to 39 (40) | on stop | upper |  | 0 N.m | finger |
| thj3 | thumb | frames 0 to 39 (40) | on stop | lower |  | 0 N.m | finger |

The torque left unbalanced is stated per finger, and joint by joint above.
