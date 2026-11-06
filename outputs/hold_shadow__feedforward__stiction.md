# Can each shadow finger feed the hold's contact state forward?

Leader `hold_shadow__self`, stored fixed placement, 2 frame(s), all giving the same numbers; frame 0 is shown. Made by `tools/hold_shadow_feedforward.py` from `gramian_workshop/feedforward.py`; every number is in `outputs/hold_shadow__feedforward.json`.

The check: with the finger at the pose follow.py found and touching at the pad point it chose, can the finger apply the leader's contact force at that instant, with nothing moving? Nothing is settled, the object is not released and its stability is not tested. The leader gives contact forces only, so no contact torque is checked.

## Result

- Verdict, with what the hand commands or leans on (servos within their control range, end stops): 1 of 5 fingers can apply the demanded force (little); index, middle, ring, thumb cannot.
- With the model's joint friction as well: 5 of 5 fingers can apply the demanded force.
- Check against follow.py (its force-range bounds, no friction): 2 of 5 fingers can apply the demanded force (little, thumb); index, middle, ring cannot.

Joint friction is not something the hand commands, so it is not an input to the verdict; it is a property the hand has (ruling of 6 Nov). What the hold needs of it is the minimum friction each joint must supply once the servos and stops have done all they can, against what the model has:

| finger | joint | stiction needed (N.m) | the model's (N.m) | needed / model's |
|---|---|---|---|---|
| index | FFJ2 | 0.0012 | 0.01 | 0.12 |
| index | FFJ1 | 0.0012 | 0.01 | 0.12 |
| middle | MFJ2 | 0.0011 | 0.01 | 0.11 |
| middle | MFJ1 | 0.0011 | 0.01 | 0.11 |
| ring | RFJ2 | 0.0012 | 0.01 | 0.12 |
| ring | RFJ1 | 0.0012 | 0.01 | 0.12 |
| little | none | 0 | | |
| thumb | THJ3 | 0.0034 | 0.01 | 0.34 |

Stiction holds a joint only while nothing else moves it, so this is a necessary condition for the hold, not a demonstration of it; whether the real hand's joints have this much is a question for its data sheet or a measurement, not this model.

Location: the pad point is 0.08 to 1.40 mm from the leader's station (follow.py's reach; green within 2 mm, amber within 10 mm).

## Commands within the control range, no joint friction (the verdict)

| finger | reach (mm) | demanded (N) | torque needed (N.m) | left unbalanced (N.m) | share | follow.py's unbalanced (N.m) | nearest force it can hold: off by (N) | limit (N) | angle (deg) | shear / normal | force reproduced |
|---|---|---|---|---|---|---|---|---|---|---|---|
| index | 1.40 | 0.1000 | 0.00746 | 0.00164 | 22.0 % | 0.00164 | 0.0986 | 0.0050 | 80.4 | 6.34 | no |
| middle | 0.08 | 0.1000 | 0.00754 | 0.00154 | 20.4 % | 0.00154 | 0.0872 | 0.0050 | 60.7 | 1.78 | no |
| ring | 1.31 | 0.1000 | 0.00750 | 0.00166 | 22.1 % | 0.00166 | 0.0990 | 0.0050 | 82.0 | 7.51 | no |
| little | 1.12 | 0.1000 | 0.00637 | 0.00000 | 0.0 % | 0.00000 | 0.0000 | 0.0050 | 0.0 | 0.04 | yes |
| thumb | 0.51 | 0.1000 | 0.00728 | 0.00337 | 46.2 % | 0.00000 | 0.0912 | 0.0050 | 65.8 | 2.33 | no |

Who carries the demanded force's joint torque (least friction, then least input use):

| finger | inputs (N or N.m) | stops | joint friction (share of the model's) | inputs with one side closed by the control range |
|---|---|---|---|---|
| index | FFJ4 +0.0001 (0.0 % of its range); FFJ3 +0.0057 (0.8 % of its range); FFJ0 +0.0032 (0.5 % of its range) | none | none | FFJ4 [+0.000, +0.698] |
| middle | MFJ4 +0.0000 (0.0 % of its range); MFJ3 +0.0059 (0.9 % of its range); MFJ0 +0.0032 (0.5 % of its range) | none | none | none |
| ring | RFJ4 +0.0001 (0.0 % of its range); RFJ3 +0.0057 (0.8 % of its range); RFJ0 +0.0032 (0.5 % of its range) | none | none | RFJ4 [+0.000, +0.698] |
| little | LFJ5 +0.0034 (2.4 % of its range); LFJ4 +0.0014 (0.2 % of its range); LFJ3 +0.0037 (0.4 % of its range); LFJ0 +0.0032 (0.6 % of its range) | LFJ1 -0.0014 N.m | none | LFJ4 [+0.000, +0.698] |
| thumb | THJ5 +0.0016 (0.9 % of its range); THJ4 +0.0060 (1.2 % of its range); THJ2 +0.0013 (0.1 % of its range); THJ1 +0.0011 (0.6 % of its range) | none | none | THJ3 [-0.209, +0.000] |

## Commands within the control range, plus the model's joint friction (necessary condition)

| finger | reach (mm) | demanded (N) | torque needed (N.m) | left unbalanced (N.m) | share | follow.py's unbalanced (N.m) | nearest force it can hold: off by (N) | limit (N) | angle (deg) | shear / normal | force reproduced |
|---|---|---|---|---|---|---|---|---|---|---|---|
| index | 1.40 | 0.1000 | 0.00746 | 0.00000 | 0.0 % | 0.00164 | 0.0000 | 0.0050 | 0.0 | 0.05 | yes |
| middle | 0.08 | 0.1000 | 0.00754 | 0.00000 | 0.0 % | 0.00154 | 0.0000 | 0.0050 | 0.0 | 0.00 | yes |
| ring | 1.31 | 0.1000 | 0.00750 | 0.00000 | 0.0 % | 0.00166 | 0.0000 | 0.0050 | 0.0 | 0.04 | yes |
| little | 1.12 | 0.1000 | 0.00637 | 0.00000 | 0.0 % | 0.00000 | 0.0000 | 0.0050 | 0.0 | 0.04 | yes |
| thumb | 0.51 | 0.1000 | 0.00728 | 0.00000 | 0.0 % | 0.00000 | 0.0000 | 0.0050 | 0.0 | 0.02 | yes |

Who carries the demanded force's joint torque (least friction, then least input use):

| finger | inputs (N or N.m) | stops | joint friction (share of the model's) | inputs with one side closed by the control range |
|---|---|---|---|---|
| index | FFJ4 +0.0000 (0.0 % of its range); FFJ3 +0.0057 (0.8 % of its range); FFJ0 +0.0032 (0.5 % of its range) | FFJ4 +0.0001 N.m | FFJ2 +0.0012 N.m (12 %); FFJ1 -0.0012 N.m (12 %) | FFJ4 [+0.000, +0.698] |
| middle | MFJ4 +0.0000 (0.0 % of its range); MFJ3 +0.0059 (0.9 % of its range); MFJ0 +0.0032 (0.5 % of its range) | none | MFJ2 +0.0011 N.m (11 %); MFJ1 -0.0011 N.m (11 %) | none |
| ring | RFJ4 +0.0001 (0.0 % of its range); RFJ3 +0.0057 (0.8 % of its range); RFJ0 +0.0032 (0.5 % of its range) | none | RFJ2 +0.0012 N.m (12 %); RFJ1 -0.0012 N.m (12 %) | RFJ4 [+0.000, +0.698] |
| little | LFJ5 +0.0034 (2.4 % of its range); LFJ4 +0.0014 (0.2 % of its range); LFJ3 +0.0037 (0.4 % of its range); LFJ0 +0.0032 (0.6 % of its range) | LFJ1 -0.0014 N.m | none | LFJ4 [+0.000, +0.698] |
| thumb | THJ5 +0.0016 (0.9 % of its range); THJ4 +0.0060 (1.2 % of its range); THJ2 +0.0013 (0.1 % of its range); THJ1 +0.0011 (0.6 % of its range) | none | THJ3 +0.0034 N.m (34 %) | THJ3 [-0.209, +0.000] |

## Force range only, no joint friction (follow.py's torque test)

| finger | reach (mm) | demanded (N) | torque needed (N.m) | left unbalanced (N.m) | share | follow.py's unbalanced (N.m) | nearest force it can hold: off by (N) | limit (N) | angle (deg) | shear / normal | force reproduced |
|---|---|---|---|---|---|---|---|---|---|---|---|
| index | 1.40 | 0.1000 | 0.00746 | 0.00164 | 22.0 % | 0.00164 | 0.0930 | 0.0050 | 68.5 | 2.29 | no |
| middle | 0.08 | 0.1000 | 0.00754 | 0.00154 | 20.4 % | 0.00154 | 0.0872 | 0.0050 | 60.7 | 1.78 | no |
| ring | 1.31 | 0.1000 | 0.00750 | 0.00166 | 22.1 % | 0.00166 | 0.0937 | 0.0050 | 69.5 | 2.40 | no |
| little | 1.12 | 0.1000 | 0.00637 | 0.00000 | 0.0 % | 0.00000 | 0.0000 | 0.0050 | 0.0 | 0.04 | yes |
| thumb | 0.51 | 0.1000 | 0.00728 | 0.00000 | 0.0 % | 0.00000 | 0.0000 | 0.0050 | 0.0 | 0.02 | yes |

Who carries the demanded force's joint torque (least friction, then least input use):

| finger | inputs (N or N.m) | stops | joint friction (share of the model's) | inputs with one side closed by the control range |
|---|---|---|---|---|
| index | FFJ4 +0.0001 (0.0 % of its range); FFJ3 +0.0057 (0.6 % of its range); FFJ0 +0.0032 (0.3 % of its range) | none | none | none |
| middle | MFJ4 +0.0000 (0.0 % of its range); MFJ3 +0.0059 (0.6 % of its range); MFJ0 +0.0032 (0.3 % of its range) | none | none | none |
| ring | RFJ4 +0.0001 (0.0 % of its range); RFJ3 +0.0057 (0.6 % of its range); RFJ0 +0.0032 (0.3 % of its range) | none | none | none |
| little | LFJ5 +0.0034 (0.3 % of its range); LFJ4 +0.0014 (0.1 % of its range); LFJ3 +0.0037 (0.4 % of its range); LFJ0 +0.0032 (0.3 % of its range) | LFJ1 -0.0014 N.m | none | none |
| thumb | THJ5 +0.0016 (0.1 % of its range); THJ4 +0.0060 (0.3 % of its range); THJ3 +0.0034 (0.3 % of its range); THJ2 +0.0013 (0.1 % of its range); THJ1 +0.0011 (0.1 % of its range) | none | none | none |

## How to read it

- Torque needed is the joint torque the demanded force asks of the finger (J^T f at the pad point). Left unbalanced is the smallest part of it that the finger's inputs, stops and (where credited) joint friction cannot supply; follow.py passes the finger when this is within 5 % of the torque needed.
- The nearest force it can hold is the force closest to the demanded one that the finger balances exactly at this contact. The force counts as reproduced when it is within 5 % of the demanded force. A small unbalanced torque can still mean a distant force: the two end joints of a finger share one tendon, which gives them equal torque, so without friction or a stop the finger can only hold forces that load those two joints equally.
- An input with one side closed by the control range sits where its position command cannot go further (the joint is posed at the end of the command range), so at this exact pose it can push one way only. The command each input needs is in the JSON (`carried.input_ctrl`, next to `input_ctrl_at_rest`).
- This leader was made from kinematics and asks for pure normal forces. It is a dry run of the method; a leader extracted from a simulation with its shear forces is the real test.
