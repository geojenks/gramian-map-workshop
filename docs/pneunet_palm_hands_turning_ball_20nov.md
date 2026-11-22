# Pneu-net fingers on a palm turning a ball: one stroke and the return (20 Nov 2026)

Simulation only, comparative not predictive. Kinematic, except the MuJoCo section added later the same day. No finite-element run was made for this note.

## The answer

- **One stroke.** Three, four or six copies of one pneu-net record on a palm turn the ball by 88 to 106 degrees in one stroke, slip ignored. That figure needs the tips pressed 12 to 17 mm into the ball's ring. With 5 mm of press it is about 55 degrees, with 2 mm about 35 degrees.
- **Second stroke, one shared input.** There is none. The ball's angle is a function of the input level alone, so bringing the input back turns the ball back by the same amount. At every input level between the start and end of contact the free tip sits 0.1 to 16.7 mm inside the ball, so no return clear of the ball exists.
- **Second stroke, two inputs.** Two sets of three fingers on two separate inputs do give a net turn in this model: 10 to 20 degrees per cycle of the inputs, repeatable, for three of the four records. Each finger still has one path. The gain comes from the second input, not from any finger returning clear. It is marginal: the tips press only 0.2 to 0.7 mm and clearances are 0.1 to 0.6 mm, the same size as the error of the body fit.
- **With contact and friction (MuJoCo, added later).** It did not blow up. One input: the ball turns about 121 degrees and comes back with the fingers. Two inputs: a net turn of 10.5 to 16.7 degrees per cycle survives on a rest with friction, not converged, and fails on a frictionless rest. See "The same hands in MuJoCo" below.
- **As a traditional gripper (MuJoCo, added later).** Three fingers hanging from a plate round the ball, each turned about its own long axis, one input. With the fingers not turned the ball does not turn. Turned 60 degrees, the 2 mm wall fingers turn it 53.1 degrees in a stroke, and it comes back on release. The large turn exists only in a narrow window of finger angle, whose place depends on the assumed gap, tip height and contact softness. See "Fingers hanging from a plate" below.
- **Two things the expected punchline gets wrong.**
  1. A one-input finger *can* break contact. Three of the four records leave the ball at the far end of the stroke and travel up to 27 to 54 mm clear of it. What they cannot do is get back to the start without passing through the contact stretch again.
  2. For the three thinner-walled records the turn is *not* limited by the single path. The tip path alone would allow 213 to 219 degrees. The finger's own body reaches the ball first and cuts this to about 105 degrees. Only the 2 mm wall record uses its whole path (99%).
- **Weight.** No hand carries the 50 g ball's weight over the whole stroke by tip force alone. The best carries it over 93 of 106 degrees.

## The hands

Records used, unchanged: `data/records/wall_0.0005m.json`, `wall_0.0010m.json`, `wall_0.0015m.json`, `wall_0.0020m.json`. Ball and task from `data/tasks/cup_turn.json` (radius 32.9 mm, mass 50 g, friction 1, contact ring 43.63 degrees below the centre, vertical turning axis).

Each hand is N copies of one record (N = 3, 4, 6) spaced evenly round the axis and driven by one shared input. Each finger lies flat on the palm plate. Its bending plane is parallel to the plate, at the height where the tip meets the ball on the ring. Bases sit 86 to 90 mm from the axis.

Contact is not set by a chosen band. The tip is in contact while its free path lies inside the circle the tip centre must ride on (27.6 to 29.7 mm from the axis). The tip is then pushed out onto that circle, in the bending plane, where the record says the finger is soft (4 to 11 N/m against 294 to 474 N/m out of plane). The ball turns by the angle the tip sweeps round the axis.

A mount is kept only if: the tip path stays inside the circle throughout and sweeps one way; the finger body is clear of the ball on the approach; the tip, not the body, is the nearest part to the axis while in contact; neighbouring bodies stay at least two tip radii apart. 4032 mounts were tried per record.

## One stroke, slip ignored

Widest turn found, three fingers:

| Record | Turn | Contact (input, Pa, simulation units) | Tip path used | Deepest press | What limits it |
|---|---|---|---|---|---|
| wall 0.5 mm | 104.7 deg | 6 to 148 | 45.0 of 76.2 mm (59%) | 15.4 mm | finger body reaches the ball (path alone: 214.4 deg) |
| wall 1.0 mm | 106.4 deg | 6 to 222 | 47.1 of 110.0 mm (43%) | 16.6 mm | finger body reaches the ball (path alone: 219.3 deg) |
| wall 1.5 mm | 105.6 deg | 8 to 358 | 48.0 of 76.5 mm (63%) | 16.7 mm | finger body reaches the ball (path alone: 213.0 deg) |
| wall 2.0 mm | 87.7 deg | 8 to 500 | 42.2 of 42.5 mm (99%) | 12.4 mm | the path ends: contact stops at the last input level |

Four fingers give the same turns. Six fingers give 104.7, 103.7, 97.7 and 87.7 degrees; for the 1.0 and 1.5 mm walls the neighbours' bodies are then the limit.

The ring moves 8 to 12% further than the tip travels in contact (45.4 to 53.7 mm against 42.2 to 48.0 mm), because the pushed-out tip rides a larger circle than its free path. So "the object turns as far as the finger moves" holds to within 8% for the 2 mm wall, which uses its whole path, and not for the others, which use 43 to 63% of theirs.

**The turn depends on how hard the tips are allowed to press.** Turn against the largest press allowed (all four records within a few degrees of each other):

| Largest press | 1 mm | 2 mm | 3 mm | 5 mm | 8 mm | 12 mm | 16 mm |
|---|---|---|---|---|---|---|---|
| Turn (deg) | 24 to 27 | 34 to 38 | 42 to 46 | 54 to 58 | 69 to 73 | 86 to 91 | 88 to 105 |

The only cap applied was the press at which the tip force would pass the largest blocked force in the record (16 to 61 mm). It never binds. The headline figures are therefore the geometric maximum, not a figure set by a tolerance, but a designer choosing a gentler press gets a smaller turn.

**Weight.** Press needed for N tips to carry the ball: 3.7 to 21.7 mm depending on record and N. The press is near zero at both ends of the stroke, so the weight is carried only in the middle: for example 66.2 of 87.7 degrees (wall 2.0 mm, three fingers), 93.3 of 105.6 degrees (wall 1.5 mm, four fingers), never (wall 0.5 mm, three or four fingers). The ball needs a rest under it at the ends of each stroke.

## Back and forth, and the return

- The fingertip returns on the path it went out on. This is by construction: a record is one loading sweep, one tip position per input level. Whether the real finger unloads on the same path (hysteresis) is not in the records.
- Between the two ends of contact the free tip is inside the ball at every input level: gap −15.36 to −0.71 mm (wall 0.5), −16.60 to −0.39 mm (wall 1.0), −16.73 to −0.22 mm (wall 1.5), −12.39 to −0.11 mm (wall 2.0). A clear return needs a positive gap at each of those levels. None exists.
- Past the far end of contact the tip is clear of the ball by up to 29.1 mm (wall 0.5), 53.5 mm (wall 1.0), 27.4 mm (wall 1.5). The 2 mm wall has no input left. Before the near end the tip is clear by 0.3 to 3.3 mm.

## Two sets of fingers on two inputs

Six fingers, two sets of three, the second set half a pitch round from the first, each set on its own input. One cycle of the inputs: the first set goes out alone, the second set goes out alone, then both come back together. Slip ignored.

| Record | Each set out | Both back | Net per cycle | Contact (Pa) | Deepest press | Waiting tip clear of ball by | Bodies of the two sets apart (need) |
|---|---|---|---|---|---|---|---|
| wall 0.5 mm | +19.6 deg | −19.6 deg | 19.6 deg | 192 to 208 | 0.7 mm | 0.58 mm | 11.5 mm (10.4) |
| wall 1.5 mm | +10.0 deg | −10.0 deg | 10.0 deg | 468 to 492 | 0.2 mm | 0.33 mm | 14.9 mm (14.7) |
| wall 2.0 mm | +9.7 deg | −9.7 deg | 9.7 deg | 444 to 492 | 0.2 mm | 0.14 mm | 16.2 mm (16.2) |

- The cycle repeats: three cycles give 58.7, 30.1 and 29.1 degrees. The same mount on one shared input gives 0.0.
- Why it works: each set turns the ball forward once on its own, and the shared return turns it back only once. No finger returns clear of the ball.
- No mount passes for wall 1.0 mm, or for two sets of four or six. Every one fails on the bodies of the two sets coming within two tip radii of each other.
- It is a grazing contact. The press is 0.2 to 0.7 mm, far below the 3.7 to 21.7 mm needed to carry the ball, so the ball must sit on a rest throughout and the tips must turn it against that rest's friction. That is not modelled here; the MuJoCo section below does model it. The clearances are no larger than the body-fit error (0.11 to 0.69 mm). Treat this as "not ruled out", not as a working hand.
- `report.txt` from the first script has an earlier, coarser version of this (a single hand-over, 23.3 degrees per set for wall 0.5 mm) and a closing sentence that going again turns the ball back. That sentence covers one set returning alone and is superseded by the table above. The 23.3 degree mount fails the check at every input level.

## The same hands in MuJoCo, with contact and friction

Added later on 20 Nov. This is a different model from the rest of this note, so its numbers need not agree with the kinematic ones. The finger is the fitted hinge-chain stand-in with its joint springs, the ball is a free 50 g body, and MuJoCo solves contact with friction 1. Simulation only, comparative.

**It did not blow up.** Every run reached the end with no MuJoCo warning. The deepest contact overlap was 0.45 to 0.55 mm. No finger touches the ball at zero input, so there is no overlap to start from; the overlap of the free paths with the ball (up to 16.6 mm in the kinematic model) is taken up by the fingers bending and by the ball lifting.

Two rests under the ball were run: a flat rest with no friction, and a seat of three fixed points 10 mm from the axis with friction 1. The seat radius is assumed; the rest's friction torque scales with it. Each seat run was repeated at half and a quarter of the ramp speed (10, 20 and 40 s per ramp of the input).

| Hand | Kinematic, slip ignored | MuJoCo: turn at the end of the stroke | MuJoCo: left over after each return (10, 20, 40 s ramps, seat) | Ball lifted |
|---|---|---|---|---|
| three 2 mm wall fingers, one input | 87.7 deg, 0.0 left | 120.5 to 121.8 deg | 2.4, 1.3, 0.8 deg | 8.3 to 8.6 mm |
| four 1 mm wall fingers, one input | 106.4 deg, 0.0 left | 140.5 to 150.5 deg | 15.5 the other way, 6.6, 4.2 deg | 10.1 to 11.0 mm |
| two sets of three 0.5 mm wall fingers, two inputs | net 19.6 deg per cycle | each set out 10.5 to 13.9 deg | net per cycle 16.7, 12.1, 10.5 deg | none |

What the three questions get as answers:

- **Do the tips slip instead of turning the ball?** The ball does not lag the tips; it turns further than they do. On one outward stroke of the three 2 mm wall fingers the tip centres sweep 87.6 degrees round the axis while touching (the kinematic figure) and the ball turns 120.5 degrees. For the four 1 mm wall fingers it is 106.5 against 140.5 degrees at the slowest ramp. So the ball moves against the tip centres by about 33 degrees in each case. The logs cannot say whether that is the round tip rolling on the ball as the finger curls, or sliding. It reverses on the way back.
- **Does a returning finger drag the ball back?** Yes. Three 2 mm wall fingers on one input turn the ball about 121 degrees and bring it back to within 0.4 degrees on the frictionless rest. On the seat 0.8 to 2.4 degrees is left per cycle, and that shrinks as the ramp is slowed, so it is an effect of ramp speed, not a second stroke.
- **Does the two-input cycle survive contact?** On the seat, yes, in the predicted direction: 10.5 degrees net per cycle at the slowest ramp, against 19.6 kinematic. Each set turns the ball 13.9 degrees going out and the shared return undoes 17.2. The second set turns the ball while the first set is held out, so the net turn is the ball moving against the held fingers. The figure is still falling as the ramp slows (16.7, 12.1, 10.5), so it is not converged. On the frictionless rest it fails: nothing holds the ball between touches, it coasts, tilts up to 21.8 degrees and drifts 8.0 mm, and the angle after three cycles means nothing. The cycle needs the rest's friction.

Not as the kinematic model had it:

- The turn in one stroke is larger: 121 against 87.7 degrees, 141 to 151 against 106.4. The tip centres sweep the kinematic angle; the extra is the ball moving against the tips, as above.
- The tips slide under the ball and lift it 8 to 11 mm. While lifted the ball rides on the tips, not on the rest, which is why the seat and the frictionless rest agree for these two hands. The kinematic section says tip force never carries the weight over the whole stroke; here it does over most of it. The lift is a rigid wedge: these fingers hinge about vertical axes only and cannot give way downwards, where a soft finger could.
- The four 1 mm wall fingers are not settled. With the ball's contact off, the driven tip is still 1.9 mm off its posed path at the slowest ramp (5.7 mm at the fastest): this finger moves too fast over part of its input range for the ramp to be slow. What is left after the return changes size and sign with ramp speed, and at the fastest ramp the ball tilts 29.3 degrees. No conclusion is drawn for this hand beyond "it turns the ball about 140 degrees and brings most of it back".

Changes from `tools/build_record_hand.py`, which is untouched: fingers flat on the palm at the kinematic mounts; joint limits off so contact can bend a finger off its free path; each joint critically damped from its own spring and inertia; links of one finger do not collide with each other; gravity on. Checks made before each contact run: the posed model reproduces the kinematic tip gap at every input level (to 0.000 micrometres against the kinematic stand-in), and no contact at zero input.

Not verified in this model: the finger masses (the records give none, so MuJoCo's default density applies, and the ramp-speed effects depend on it); the stand-in's stiffness against being bent off its path, which is the fitted joint springs and not a measured quantity; the seat radius; the finger body is three rigid capsules, not a soft body, and cannot deflect out of the palm plane, so the lift and the turn figures that go with it are not to be trusted for a soft finger. A finite-element run would be needed for the last.

Command, from the repository root, with a Python that has `mujoco`: `python tools/pneunet_palm_turn_ball_mujoco_20nov.py` (over 10 minutes, needs ffmpeg). Writes `outputs/pneunet_palm_turn_ball_20nov/mujoco/mujoco_report.txt`, `mujoco_results.json`, one model `.xml` per hand and rest, one log `.csv` per run, and the clips below. Every number in this section is from `mujoco_report.txt`, except the per-ramp tip sweeps, which are from `mujoco_slip_check.txt`, written by `python tools/pneunet_palm_turn_ball_mujoco_slip_check_20nov.py` from the logs (no simulation; the tip sweep is a mean over all fingers).

| File (in `outputs/pneunet_palm_turn_ball_20nov/mujoco/`) | Shows |
|---|---|
| `mujoco_wall_0.0020m_x3_seat_rest.mp4`, `.png` | three 2 mm wall fingers turn the ball 121 degrees and turn it back, twice |
| `mujoco_wall_0.0020m_x3_smooth_rest.mp4`, `.png` | the same on the frictionless rest |
| `mujoco_wall_0.0010m_x4_seat_rest.mp4`, `.png` | four 1 mm wall fingers; fastest ramp, not settled, the ball tilts |
| `mujoco_wall_0.0010m_x4_smooth_rest.mp4`, `.png` | the same on the frictionless rest |
| `mujoco_wall_0.0005m_2x3_seat_rest.mp4`, `.png` | two sets of three fingers on two inputs, three cycles, net turn each cycle |
| `mujoco_wall_0.0005m_2x3_smooth_rest.mp4`, `.png` | the same on the frictionless rest: the ball coasts and wanders |

The clips play four times faster than simulated time and show the 10 s ramp.

## Fingers hanging from a plate, as in a traditional soft gripper (MuJoCo, added later)

SIMULATION ONLY. Comparative, not predictive.

The hands above lie flat on a palm and close like an iris. This section rebuilds the hand as the usual three-finger soft gripper: the finger bases sit on one plate above the ball, the fingers hang straight down round it, and the ball sits on the same rest below. Each finger is turned about its own long axis by a set angle, so that its bend is partly round the ball's vertical axis and not only towards it. All three fingers share one input.

The finger is also rebuilt. It has ten links (the number of spine points in the record's rest shape) in place of three, drawn with ribs on the outside and a flat strip on the inside. Every link bends by the same angle and stretches by the same amount. That even bend is assumed: the records hold the tip path and the rest shape only, no bent body shape. The two numbers are solved at every input level, so the posed tip sits on the record's free path to rounding (the three-hinge fit used above is off by 0.11 to 0.69 mm; the repository's tolerance is 5% of the stroke). At full input that is 58 degrees of total bend and 0.70 mm of stretch per 7.3 mm link for the 2 mm wall, 104 degrees and 1.00 mm for the 1.5 mm wall. The springs are set from the record's tip stiffness at zero input along the finger, in the bending plane and out of it. The built model returns those three values; that checks the wiring, not the finger, since the springs were chosen to give them.

**Two corrections since the first version of this section.**

1. The fingers were too heavy. Each link's contact capsule overlaps its neighbours and was given MuJoCo's default density, which made a 2 mm wall finger 37.0 g. It is now 14.96 g (12.43 g for the 1.5 mm wall): the record's material volume at an assumed 1000 kg/m^3. The heavy fingers sagged under their own weight and bent less far than the record.
2. The contact was too soft for links this light: the ball sank up to 3.9 mm into a finger, and several 1.5 mm wall runs were stopped by the script's own limit (overlap over half the finger radius). The time constant of the finger-on-ball contact is now 5 ms in place of MuJoCo's default 20 ms (assumed). The overlap is now at most 0.8 mm for the 2 mm wall. The ball's contacts with its rest points and with the plate were left at the default 20 ms; see the stopped run below.

Every figure and clip from the first version is replaced. The first version's report file was overwritten, so its figures here (37.0 g, and the two that follow) are quoted from the working session and cannot be regenerated. The first version gave 48.6 degrees (2 mm wall) and 50.5 degrees (1.5 mm wall) at a 60 degree finger angle with the tips level with the ball's centre; those are now 53.1 and 76.2. The pattern is the same: no turn at zero angle, a narrow window of large turn near 60 degrees, a miss at 75.

**What happened** (ball turn in degrees in the first stroke, 10 s ramp; no finger touches the ball at zero input):

| Finger turned about its own axis | 2 mm wall, tips below the ball's centre | 2 mm wall, tips level with the centre | 1.5 mm wall, tips below | 1.5 mm wall, tips level |
|---|---|---|---|---|
| 0 degrees | 0.0 | 0.0 | 0.0 (ball lifted) | 0.0 |
| 30 degrees | 1.2 | 1.6 | 6.0 (ball lifted, not settled) | 2.2 |
| 45 degrees | 2.1 | 3.5 | 8.2 (ball lifted) | 5.9 (ball lifted) |
| 50 degrees | 2.6 | 4.9 | 9.2 (ball lifted) | 12.0 (ball lifted) |
| 55 degrees | 3.3 | 7.7 | 10.2 (ball lifted) | 87.8, then the run was stopped |
| 60 degrees | 4.4 | 53.1 | 11.8 (ball lifted, not settled) | 76.2 |
| 65 degrees | 7.1 | 37.2 | 36.8 | 36.1 |
| 70 degrees | 11.9 | 18.0 | 20.7 | 16.4 |
| 75 degrees | fingers miss the ball | fingers miss the ball | fingers miss the ball | fingers miss the ball |

- Turning the fingers about their own axes does turn the ball, but only in a narrow window. Up to 50 degrees the fingers mostly squeeze. With the tips level with the ball's centre the 2 mm wall fingers turn the ball 53.1 degrees at 60 degrees, 37.2 at 65 and 18.0 at 70. At 75 degrees they sweep past without touching.
- The window's edge is sharp and its place is not robust. For the 2 mm wall the turn jumps from 7.7 degrees at a 55 degree finger angle to 53.1 at 60. With the softer default contact the jump sat one step lower (9.1 at 50, 53.3 at 55, 40.2 at 60; kept in `gripper_report_default_contact_softness.txt`). The peak, about 53 degrees, is the same in both. The window also depends on the assumed 2 mm gap and the two tip heights; other gaps were not tried.
- Our reading of the small turns, not checked further: once a finger meets the ball it is blocked, and it can only carry the ball sideways by flexing out of its bending plane, where the record is far stiffer than in the plane (473.8 against 11.44 N/m at the tip for the 2 mm wall).
- The sanity checks hold. With the fingers not turned the ball does not turn. Mirrored angles (-30, -60 degrees) give the same turn with the opposite sign.
- The turn comes back on release, as expected with one set of fingers on one input. After two cycles the 2 mm wall ball ends within 0.1 degrees of its start at every angle except 60 degrees with level tips, where it overshoots by 3.0 degrees per cycle (0.8 at a 40 s ramp). The mirrored run at that angle ends 7.1 degrees off against 5.9, so the overshoot is not exactly mirrored. The 1.5 mm wall at 60 degrees with level tips keeps 12.7 degrees per cycle (6.9 at a 40 s ramp), so that figure depends on speed and is not a settled net turn.
- The large turns are part carry, part slide. In the 2 mm wall, 60 degree, level-tip stroke the ball's surface moves 44.6 mm at the contacts (turning and rising together, not broken down) while 14.0 mm of sliding is logged there (relative motion along the surface at each finger's deepest contact, mean over fingers).
- Slowing the ramp from 10 s to 40 s changes the 2 mm wall first-stroke turn by at most 0.2 degrees at the two angles repeated at 40 s (30 and 60 degrees). The 1.5 mm wall is not settled where it lifts the ball: with low tips it gives 6.0 degrees at 10 s and 9.9 at 40 s (30 degree angle), and 11.8 against 18.2 (60 degree angle); the ball is still turning during the 1 s hold (6.0 to 10.5, and 11.8 to 14.8).
- The 1.5 mm wall fingers curl under the ball and lift it: 20 to 23 mm with low tips from 0 to 55 degrees (the ball touches the plate from 0 to 50 degrees), and 21 to 24 mm with level tips at 45 and 50 degrees. The 2 mm wall fingers lift it only in the 60 degree level-tip run, by 6.1 mm.
- One run did not finish. The 1.5 mm wall, level tips, 55 degrees turned the ball 87.8 degrees in the first stroke and was then stopped at 11.10 s by the script's overlap limit. The limit was meant for the fingers, but the script takes the deepest overlap over every contact, and here it was the ball pressed 3.7 mm down into its rest points (ball dropped 3.95 mm). It is not a result.
- The same happens, short of the limit, in the 1.5 mm wall, level-tip, 60 degree runs: the fingers press the ball 2.5 to 2.6 mm down into the rest points, whose contact is still at MuJoCo's default softness. So the 76.2 degrees there rests on a rest that gives way more than a real one would. The 2 mm wall runs press the ball down by at most 0.4 mm.

**Contact paths.** Each run now writes `contacts_<run>.csv`: every 10 ms, for each finger, the point where it touches the ball (in the world and in the ball's own frame) and the normal force. MuJoCo puts the point midway between the two overlapping surfaces, so it sits half the overlap inside the ball. One file was opened and checked (2 mm wall, level tips, 60 degrees): the points are finite while touching and lie within 0.4 mm of the ball's radius. These are the paths another hand would have to follow. They were not put through the repository's follow check.

**Checks on the finger alone** (no ball):

- Posed at every record level, the tip is on the record's free path to rounding. The record's own out-of-plane drift (0.04 mm for 2 mm, 0.42 mm for 1.5 mm) is not modelled.
- Driven with gravity off, the tip stays within 0.14 mm (2 mm wall) and 0.30 mm (1.5 mm wall) of the free shape at the 10 s ramp, and 0.03 and 0.08 mm at 40 s.
- With gravity on, the hanging finger sags 0.09 mm at zero input and 2.6 mm (2 mm wall) or 4.8 mm (1.5 mm wall) when held bent at full input. That is 6.4% and 6.9% of the free stroke, outside the repository's 5% tolerance for a free path. So in the ball runs the fingers bend a little less far than the record. The mass rests on the assumed density.

**Assumed, not from the records:** the even bend and stretch along the finger; density 1000 kg/m^3; contact time constant 5 ms; 2 mm gap between a hanging finger and the ball; the two tip heights; the angles tried; the rest's contact points 10 mm from the axis; friction 1 (the task file's value).

**Not modelled:** twist of a finger about its long axis (rigid); the finger's cross-section (contact is a round capsule per link, radius from the record's material volume; the ribs drawn are for the eye and may be seen to sink into the ball); change of stiffness with input. The "deepest overlap" in the report file is over every contact, the ball's rest points included. For the 2 mm wall it is at most 0.8 mm. For the 1.5 mm wall with level tips it reaches 2.6 mm in the finished runs, and that is the ball pressed into its rest points, not into a finger. Both are MuJoCo's soft contact, not the finger's skin.

**Not built:** two opposing sets of three fingers, one set turned clockwise and one anticlockwise, with fingers made of several parts stacked along their length. That is the natural next step for turning the ball further than one stroke and holding the turn; it is listed as a follow-up.

Command: `python tools/pneunet_gripper_turn_ball_mujoco_20nov.py` (needs mujoco and ffmpeg; several minutes). Writes `outputs/pneunet_gripper_turn_ball_20nov/`: `gripper_report.txt` (every number above, except those quoted from `gripper_report_default_contact_softness.txt`), `gripper_results.json`, and one `model_*.xml`, `log_*.csv` and `contacts_*.csv` per run.

| File in `outputs/pneunet_gripper_turn_ball_20nov/` | Shows |
|---|---|
| `gripper_wall_0.0020m_free_turned_+30deg.mp4`, `.png` (and `wall_0.0015m`) | the hand with no ball, bending and straightening |
| `gripper_wall_0.0020m_tips_level_turned_+60deg.mp4`, `.png` (and `wall_0.0015m`) | the ball turned 53 degrees (76 for the 1.5 mm wall) and turned back, twice |
| `gripper_wall_0.0020m_tips_level_turned_+30deg.mp4`, `.png` (and `wall_0.0015m`) | the fingers squeeze; the ball turns about 2 degrees |
| `gripper_wall_0.0020m_tips_low_turned_+30deg.mp4`, `_+60deg.mp4`, `.png` (and `wall_0.0015m`) | tips below the ball's centre; the 1.5 mm wall fingers lift the ball |

## What the repository's existing tools say, and why they differ

- `demos/cup_turn.py`, `tools/mount_search_cup_turn.py` and the turning-task scorer pass pneu-nets. They test direction with free mounts and an assumed make-and-break schedule. They do not check that a finger can follow that schedule, and they have no palm.
- `outputs/finger_swap/finger_swap.txt` gives 97.3 degrees per stroke for wall 2.0 mm, from straight-line stroke over ring radius. The figure here, 87.7 degrees, uses the real curved path, a real mount and the tip radius.
- The stored follow check on a three-finger palm hand (`outputs/follow_wall_x3_ring__w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4__contactfix__failures.md`) fails. It was read, not re-run. Part of that failure is three fingers set against a four-station motion.
- No existing tool checks the return path.

## Scripts and commands

Run from the repository root.

- One shared input: `python tools/pneunet_palm_turn_ball_20nov.py` (about 14 s). Writes `outputs/pneunet_palm_turn_ball_20nov/report.txt` and `results.json`.
- Two inputs, and the ring-against-tip-travel percentages: `python tools/pneunet_palm_two_input_cycle_20nov.py` (a few minutes). Writes `two_input_cycle_report.txt` and `two_input_cycle.json` in the same folder.
- Every number in the kinematic sections is from those two report files, except the 97.3 degrees quoted from `outputs/finger_swap/finger_swap.txt`. The two MuJoCo sections name their own scripts and report files.
- Media: `python tools/render_pneunet_palm_turn_ball_20nov.py` (about 3 min, needs ffmpeg). Reads `results.json`.

## Media written

All in `outputs/pneunet_palm_turn_ball_20nov/`, view along the turning axis. All show one shared input; there is no clip of the two-input cycle. The tip is drawn pushed out onto the ball but the body is drawn in its free shape, so the body's end sits inside the ball, away from its tip.

| File | Shows |
|---|---|
| `turning_back_and_forth_wall_0.0020m_x3.mp4`, `.png` | three 2 mm wall fingers turn the ball 88 degrees and turn it back, twice |
| `failed_return_wall_0.0020m_x3.mp4`, `.png` | the same hand coming back for a second stroke; gap of tip to ball going out and coming back |
| `turning_back_and_forth_wall_0.0010m_x4.mp4`, `.png` | four 1 mm wall fingers turn the ball 106 degrees and turn it back |
| `failed_return_wall_0.0010m_x4.mp4`, `.png` | the same hand lets go at the far end, travels clear, then undoes the turn on the way back |
| `turn_against_push_out.png` | turn in one stroke against the largest press allowed, four records |

## Not verified

- Finger body shape. The body is a fitted hinge-chain stand-in (fit error on the free tip path 0.11 to 0.69 mm) in its free shape. The records hold tip paths only, so the body shape under input and under contact load is unchecked. The "body reaches the ball" limit rests on it.
- Stiffness. The in-plane tip stiffness is the record's value at zero input, taken as constant and the same in every in-plane direction, over presses of 12 to 17 mm. The records' blocked forces were measured only at low input.
- Slip and sticking are not modelled in the kinematic sections. The MuJoCo section tests it.
- Hysteresis, as above.
- The follow check was not run on these hands. A MuJoCo run of them was added later; see its own section.
- The hanging gripper: the even bend along the finger, the finger's twist about its own axis (rigid in the model) and the place of the window of large turn are unchecked. A finite-element run of the hand would test them. The contact paths it logs were not put through the follow check.
- Mounts were searched on a 64-level grid with the fingers flat on the plate. Tilted or out-of-plane mounts were not tried.

## How close the hanging gripper comes to a banked turning record (added later)

Short answer: the 2 mm wall gripper matches the forward half of one stroke of the banked wave, and none of the schedule that makes turning continue. One softer run (1.5 mm wall, low tips, 60 degrees) keeps 3.6 to 3.8 degrees per cycle by sliding, a tenth of one stroke of the wave.

The script `tools/gripper_contact_paths_against_turning_leaders_20nov.py` takes the same measures from the gripper's contact logs and from three banked turning records in `data/leaders/`: the authored make-and-break wave on a 33.5 mm ball with four stations (`w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4__contactfix.json`), the same with five stations, and the human tennis-ball demonstration (`tennis_new_cup_take3_derived__nframesfix.json`). A stroke is one contact from touch-down to lift-off. Everything is an angle or a fraction, so ball size and speed do not enter. Forces are not compared: the gripper log holds the normal force only, so who carries the ball's weight was not measured.

| Measure | Authored wave, four stations | Authored wave, five stations | Gripper, 2 mm wall, level tips, 60 degrees, 40 s ramp | Gripper, 2 mm wall, low tips, 60 degrees | Gripper, 1.5 mm wall, low tips, 60 degrees, 40 s ramp |
|---|---|---|---|---|---|
| Largest turn of the ball within one stroke (median) | 39.2 degrees | 16.2 degrees | 56.4 degrees | 4.4 degrees | 21.1 degrees |
| Turn from touch-down to lift-off (median) | 39.2 degrees | 16.2 degrees | -0.8 degrees | 0.0 degrees | 3.6 degrees |
| Turn over the whole record | 386.1 degrees | 387.2 degrees | -1.6 degrees (two cycles) | 0.0 degrees (two cycles) | 7.2 degrees (two cycles) |
| Fewest contacts touching at once | 2 | 2 | 0 | 0 | 0 |
| Fewest other contacts touching when one lifts off | 2 | 2 | 0 | 0 | 0 |
| Share of the record in which nothing touches | 0 % | 0 % | 15.9 % | 35.2 % | 19.8 % |
| Contact's sweep round the axis, in the world (median) | 41.0 degrees | 16.9 degrees | 40.3 degrees | 4.5 degrees | 7.7 degrees |
| Contact's movement on the ball per degree the ball turns | 0.06 | 0.28 | 0.33 | 0.01 | 0.82 |
| Contact's angle from the ball's equator | 51.7 degrees below | 51.8 degrees below | 5.6 degrees above | 6.0 degrees below | 7.0 degrees below |

The share of the record in which nothing touches includes, for the gripper, the time before the fingers first reach the ball.

What this says:

- Where it is close. In its best case the 2 mm wall gripper pushes the ball through 56 degrees in one touching spell, more than one stroke of the four-station wave (39 degrees). Its contact sweeps 40 degrees round the axis, as the wave's station does (41 degrees).
- Where it is not. In the wave a station lifts off while at least two others hold the ball, and returns while not touching. The gripper's three fingers touch together and leave together, so nothing holds the ball while they return, and they return while still touching, which takes the ball back. Its stroke from touch-down to lift-off nets about zero; the wave's nets its full 39 degrees.
- The contacts sit in the wrong place to hold the ball. The wave's stations are 52 degrees below the equator, under the ball (its turning axis points up, and its contact forces sum to the ball's weight, upward). The gripper's are within 7 degrees of the equator, and the ball rests on its three rest points throughout. The ball tilts by at most 0.26 degrees in the 2 mm wall runs used, so the gripper's angles are not affected by tilt.
- How far the contact moves on the ball does not separate the gripper from the banked records. The four-station wave's contact is carried with the ball (0.06); the five-station wave gives 0.28 and the best gripper case 0.33. Sliding and rolling are not told apart.
- One gripper run does keep some turn. The 1.5 mm wall with low tips at 60 degrees ends each cycle 3.6 degrees on (40 s ramp; 3.8 at 10 s), 7.2 after two cycles. The contact moves on the ball by 0.82 of the ball's turn, and all three fingers still leave together: it is a ratchet by sliding, not by lifting off. Per cycle it is about a tenth of one stroke of the four-station wave.
- The best case is narrow, as before: with low tips, or at 45 degrees with level tips, the 2 mm wall's largest turn within a stroke is 4.4 and 3.5 degrees.

Notes on the numbers:

- The 10 s ramp version of the best case shows 12 touching spells where 6 are expected: each finger touches the ball again for about 0.75 s just after it leaves. That halves the script's median, so its summary line reads 29.4 degrees; the six main spells each reach 59.2 degrees (per-stroke list in the `.json`). The 40 s ramp version shows 6 spells and is the one in the table.
- The human record is in the output file but is not used here. Its ball orientation is noisy (rotation off its turning axis is 53 % of the turn, against 9 % for the wave), its median touching spell lasts 0.33 s, which suggests flicker in the touching flags, and its turning axis points down, so the sign of its contact angle is not fixed.
- The 1.5 mm wall, level-tip, 55 and 60 degree runs were left out (soft ball rest, see above).

Outputs: `outputs/gripper_against_turning_leaders_20nov/gripper_against_turning_leaders.txt` and `.json`. Command, from the repository root: `python tools/gripper_contact_paths_against_turning_leaders_20nov.py`. Simulation only: comparative, not predictive.

## One stroke of a recorded learned rotation, replicated in part by three pneu-net fingers (added 21 Nov)

SIMULATION ONLY. Comparative, not predictive. Every number below is from `outputs/pneunet_arrangement_replicates_rl_stroke_20nov/arrangement_report.txt`.

Short answer: three one-input pneu-net fingers, placed by a fit, follow the three logged contact paths of one stroke to about 0.5 mm in geometry. In MuJoCo, with the cylinder's weight carried for them, they turn the cylinder 104 % of the recorded turn with the contacts 1 to 3 mm from the recorded ones. They squeeze at about a fifth of the recorded force, and on their own they do not hold the 24 g cylinder steadily: it sags and keeps slipping.

What was done:

- The recording is the learned policy on the four-finger hand turning a cylinder (`data/rl_logs/hora_seed42_30s*.npz`): radius 32.9 mm, mass 23.9 g, friction 0.72, 30 s, 960 degrees of turn.
- The script finds sections where the same three fingertips touch throughout for at least 0.3 s with the cylinder nearly still in tilt and place. There are six. The one used is log time 3.51 to 4.03 s: middle, ring and thumb touching, cylinder turning 16.4 degrees in 0.52 s.
- Each logged contact path is given one pneu-net finger (the ten-link stand-in of the hanging gripper, 2 mm wall record). The finger bends in the plane square to the cylinder's axis at the height of the logged contact. Its base position, direction, handedness and first and last input level are fitted so the loaded tip path lies on the logged path. The tip load is an assumed squeeze of 0.217 N, twice what friction needs to carry the weight shared by three.
- The fit is good for all six sections and all three wall records tried (worst finger 0.46 to 1.73 mm rms, no finger bodies overlapping). For the section used: 0.44, 0.42 and 0.46 mm rms. Inputs: middle 280 to 352, ring 319 to 298, thumb 268 to 347 of 500 (simulation units). The ring finger relaxes while the other two bend further.
- The arrangement is then run in MuJoCo over 4 s (the recording takes 0.52 s; the soft fingers are not asked to keep that pace). Three cases: cylinder free; cylinder's height fixed but free to slide, tilt and turn; cylinder driven along the logged motion as a check on the fit.

| Measure | Recorded | Arrangement, weight carried | Arrangement, cylinder free |
|---|---|---|---|
| Cylinder turn in the stroke | -16.4 degrees | -17.28 degrees (-0.28 with inputs held constant) | -10.69 degrees, then a further -5.98 after the inputs stop |
| Contact movement in the world: middle, ring, thumb | 11.5, 2.2, 9.5 mm | 11.3, 2.6, 10.3 mm | 6.1, 0.3, 7.3 mm |
| Contact off the recorded contact (median): middle, ring, thumb | | 1.1, 1.9, 2.8 mm | 1.8, 7.3, 9.1 mm |
| The same, measured on the cylinder's surface (median) | | 3.4, 2.0, 3.5 mm | 12.2, 2.5, 10.1 mm |
| Normal force per finger | 1.02, 1.06, 1.08 N | about 0.2 N | 0.14 to 0.29 N |
| Cylinder drop | | none (height fixed) | 6.6 mm (2.6 mm before the stroke starts) |

What this says:

- Contact points and rotation of one stroke can be replicated by an arrangement of single-input pneu-nets: the paths of three fingertips of the jointed hand over this stroke are each close to one finger's free path.
- The force is not replicated. The fingers are soft along the cylinder's surface (14 to 24 N/m at the tip). The recorded 1 N of squeeze would push the tips back 25 to 59 mm, so the arrangement has to work at about 0.2 N.
- At that squeeze the fingers alone do not carry the cylinder steadily. It does not fall, but it sags 6.6 mm and is still turning after the inputs stop.
- With the cylinder driven along the logged motion the contacts stay within 0.5, 1.3 and 2.9 mm (median) of the recorded ones, which checks the fit inside the simulation.
- This is one stroke only. Nothing here returns the fingers for a second stroke.

Assumed, not measured: the squeeze (0.217 N), the 4 s stroke and the rest of the timeline, even bend along the finger, rigid twist, finger friction equal to the recorded 0.72. The cylinder turns 3.2 degrees the other way when first let go, before the stroke; the stroke figures are measured from the stroke's start. So the black line on the cylinder starts at the same place in both halves of a clip only until the cylinder is let go. The figure on screen in a clip's arrangement half keeps counting after the stroke ends: its last frame reads -18.1 degrees with the weight carried (the stroke's -17.28 plus -0.76 in the final 2 s) and -16.7 with the cylinder free (-10.69 plus -5.98).

Media, in `outputs/pneunet_arrangement_replicates_rl_stroke_20nov/`: `recorded_then_arrangement_weight_carried.mp4` (the recorded section five times slower, then the arrangement; dots are the logged contact paths), `recorded_then_arrangement_cylinder_free.mp4`, `recorded_then_arrangement_cylinder_driven_along_logged_motion.mp4`, each with `_recorded.png` and `_arrangement.png` stills. Logs (`log_*.csv`), models (`model_*.xml`) and `arrangement_results.json` are in the same folder.

Command, from the repository root: `python tools/pneunet_arrangement_replicates_rl_stroke_20nov.py --wave-manip <path to Wave_manip>` (needs `mujoco` and ffmpeg; `--no-film` skips the clips).

Not done: the same with the two-pneu-net fingers; a return stroke; a finite-element run.

## The same stroke idea, redone with the pneu-nets standing upright on a palm, shown in three columns (added 21 Nov)

SIMULATION ONLY. Comparative, not predictive. Every number below is from `outputs/pneunet_upright_arrangement_replicates_rl_stroke_21nov/arrangement_report.txt`.

What was wrong with the section above: the fingers lay flat round the cylinder, which does not look like a manipulator, and the clip played the recording and the arrangement one after the other. The section of the log was also not chosen for an even grasp. The numbers in the section above stand, for that flat layout only.

Short answer: three one-input pneu-net fingers standing on a palm below the cylinder, placed by a fit, re-enact one recorded stroke. With the cylinder's weight carried for them they turn it -18.0 degrees against the recorded -16.8, with the contacts a median 2.0 to 3.8 mm from the recorded ones, all on the fingertips. They press at about a tenth of the recorded force. In the run with nothing else holding the cylinder, it falls as soon as it is let go: the simulated squeeze comes out at about a third of the fitted one, under what friction needs.

What was done:

- Section: of the six sections where the same three fingertips touch throughout, three have all contacts on the side wall. The two nearest an equilateral triangle are within 1 degree of each other (largest departure of a gap from 120 degrees: 18.5 and 19.4); the rule takes the larger turn of those. Used: log 2.56 to 3.06 s, index, middle and thumb, gaps 111, 110 and 139 degrees, turn -16.8 degrees in 0.50 s.
- Layout: each finger stands beside the cylinder on a palm plate below it and bends in a plane that touches the cylinder's side, so its tip sweeps along the surface. The squeeze is carried by the finger bending sideways, its stiff direction (400 N/m at the tip, against about 8 N/m along the surface). The fit places each plane, base and lean and sets the first and last input. It includes the bend from the finger's own weight (0.3 to 1.9 mm at the tip).
- Fit: 1.44, 1.66 and 2.14 mm rms for index, middle and thumb (1.5 mm wall record). Inputs: index 227 to 118, middle 216 to 171, thumb 209 to 141 of 500 (simulation units). The palm plate is 79.8 mm below the cylinder's bottom face.
- Squeeze: a tip that is soft along the surface, pressed on a round object, slides off to one side once the squeeze exceeds its stiffness along the surface times the distance to the axis. This limit is our estimate, not a measurement; for these fingers it comes to 0.33 N. An earlier run fitted at the logged 1 N, whose outputs were not kept, turned the cylinder by itself when let go and gave a much smaller stroke. The fit therefore uses half the estimated limit, 0.16 N (the share is assumed). Friction needs 0.109 N per finger to carry the weight shared by three.

| Measure | Recorded | Upright arrangement, weight carried |
|---|---|---|
| Cylinder turn in the stroke | -16.8 degrees | -17.99 degrees (+0.14 with inputs held constant) |
| Contact movement in the world: index, middle, thumb | 14.6, 4.2, 9.7 mm | 8.4, 8.4, 8.9 mm |
| Contact off the recorded contact (median): index, middle, thumb | | 3.8, 2.8, 2.0 mm |
| Normal force: index, middle, thumb | 0.95, 1.25, 1.02 N | fitted 0.16 N each; simulated 0.08, 0.11, 0.13 N |

What this says:

- The turn and the contact places of one stroke transfer to upright one-input fingers. The three fingers move their contacts by nearly equal amounts, where the recorded hand also shifts the cylinder sideways (index 14.6 mm, middle 4.2 mm).
- The force does not transfer. Our estimate is that a stability limit, not only softness, keeps the squeeze well under the recorded one.
- With nothing else holding it, the cylinder drops 30 mm within 0.12 s of being let go. The fitted 0.16 N would be enough for friction to carry the weight (0.109 N needed), but the simulation reaches only about 0.05 N per finger before the stroke. Why is not yet found, so this run does not show that upright fingers cannot hold the cylinder. All three bending planes sit on the fit's bound for outward lean, which is one thing to look at.
- With the cylinder driven along the logged motion the fingers stay in touch for 39 to 76 % of the stroke only; that check is weaker than for the flat layout.

Assumed, not measured: the share of the squeeze limit (0.5), the 4 s stroke and the rest of the timeline, even bend along the finger, rigid twist, finger friction equal to the recorded 0.72, the thresholds that pick the section, the height of the posts above the plate (drawn only).

Media, in `outputs/pneunet_upright_arrangement_replicates_rl_stroke_21nov/`: `recorded_contacts_reenactment_weight_carried.mp4`, three columns playing together at the same fraction of the stroke (the recording, the extracted contact points with their triangle and force arrows, the re-enactment), with `_stroke_start.png`, `_stroke_middle.png` and `_stroke_end.png` stills. There is no clip of the free cylinder because that run stops before the stroke. Logs, models and `arrangement_results.json` are in the same folder.

Command, from the repository root: `python tools/pneunet_upright_arrangement_replicates_rl_stroke_21nov.py --wave-manip <path to Wave_manip>` (needs `mujoco` and ffmpeg; `--no-film` skips the clip; `--azimuth` and `--elevation` move the camera). The third finger stands behind the cylinder from the default camera and is seen through it; a steeper view (`_view_from_above` stills in the same folder) did not show it better.

Not done: finding why the simulated squeeze is a third of the fitted one; the two-pneu-net fingers; a return stroke; a finite-element run.

## 21 Nov: can four two-pneu-net fingers follow the recorded gait, lifting and landing? Reach check

Geometry only, no simulation, comparative. Script:
`tools/two_pneunet_fingers_follow_rl_gait_reach_check_21nov.py`. Output:
`outputs/two_pneunet_fingers_follow_rl_gait_reach_check_21nov/reach_check_report.txt`
(every number below is from that report).

The finger is the measured 90 degree member of the two-pneu-nets-in-series
family: two 36.5 mm pneu-nets end to end, the far one rolled 90 degrees, one
input each. Its tip reach is the table of settled free shapes from the 20 Nov
analysis (27 of 49 input pairs measured, the rest filled in there). One such
finger is placed for each recorded fingertip, clamped end below the cylinder's
bottom face. The window is log 2.56 to 7.06 s: one round of the gait (to
4.71 s) run on until every fingertip has one whole lift, landing and the whole
stay after it. The cylinder turns -161.6 degrees in it.

What the recording asks of a fingertip: whole stays of 0.5 to 1.8 s in which
the contact moves 12 to 35 mm in the world, then 0.3 to 0.5 s off, landing 12
to 34 mm from where it lifted. The finger's tip travels 26.9 mm on the first
input alone and 11.6 mm on the second alone; its furthest reachable points are
30.8 mm apart. The ring's first whole return asks for 3.6 mm more than that.

Result with both inputs free:

| fingertip | median distance from the logged contact (mm) | largest (mm) | tip from the logged contact at lift-off / landing, each whole return (mm) | worst return clearance less the largest gap either side (mm) |
|---|---|---|---|---|
| index | 1.84 | 16.34 | 6.2 / 11.6, 4.1 / 16.3 | 9.6 |
| middle | 0.55 | 7.18 | 1.0 / 3.7, 7.2 / 2.6 | 9.9 |
| ring | 2.69 | 13.26 | 5.5 / 13.3 | 4.6 |
| thumb | 1.47 | 14.01 | 14.0 / 6.0, 8.3 / 6.9 | 10.0 |

Against the thresholds reused from earlier work (median contact error no more
than 3.8 mm, return clearance at least 2.06 mm): 4 of 4 pass both rules.

Where reach falls short: the fingers follow the middle of each stay closely,
but lift off and land 1 to 16 mm from where the recorded fingertip does. The
median hides this, so the pass rule does not test it. In plain terms: each
finger can stroke, lift clear and come back, but its stroke is shorter than
the recorded one at the ends.

Control, the same finger with one input held constant (every measured level
of either input tried, best kept): 1 of 4 passes the first rule and 0 of 4 the
second. Median contact errors are 3.2 to 6.3 mm and the tip is 5 to 26 mm from
the logged contact at lift-off and landing.

How the measures changed, in order, all on 21 Nov (reports of each stage are
kept in the output folder; the thresholds themselves never changed):

- The first run judged a return by the smallest clearance anywhere between
  lift-off and landing. That counts the lift and landing themselves as not
  clear; 1 of 4 passed (`reach_check_report_first_clearance_measure.txt`).
- The clearance was redefined: the tip moves steadily away, travels, then
  moves steadily in; the clearance is the smallest on the travelling part of
  the best such path. This is close to the largest height reached and is not
  the same measure as the 20 Nov report's. 4 of 4 passed, but so did 3 of 4
  of the control, and two fingers hung from above
  (`reach_check_report_before_gap_and_palm_side.txt`).
- Added: the gap to the surface while touching, the second rule (clearance
  less that gap), the palm-side condition, a longer window. 3 of 4 passed the
  second rule; control 0 of 4
  (`reach_check_report_before_search_target_change.txt`).
- Last change: the search had been told to stop improving clearance just
  above the threshold, which is where the failing thumb sat. It now aims at
  the quantity the second rule judges, and the window runs through the stay
  after each landing. Result as in the table. Nothing was adjusted after.

Limits. Every finger but one uses the full 0 to 120 of both inputs, so reach
is at its limit. 65 to 90% of touching samples lie within half a level of a
measured input pair; the rest lean on filled-in table points. Not checked:
finger bodies against the cylinder and each other (the fingers stand 13 to 20
degrees from the cylinder's axis, close alongside it, so this is the likeliest
thing to fail next), tip tilt, forces, speed. No load on the tip is modelled.
A simulation with contact has not been run and there is no clip.

## 21 Nov: four two-pneu-net fingers follow the recorded gait in simulation with contact, and a clip

SIMULATION ONLY, comparative, not predictive. Script:
`tools/two_pneunet_fingers_follow_rl_gait_clip_21nov.py`. Output folder:
`outputs/two_pneunet_fingers_follow_rl_gait_clip_21nov/`; every number below
is from `gait_clip_report.txt` there. The measures, stiffnesses, slow-down and
friction were fixed in the script's header before the first run and nothing
was tuned afterwards (the first run's report is kept as
`gait_clip_report_first_run_no_film.txt`; the two reports are identical line
for line; only a clip caption's wording changed between the runs).

Short answer: not shown. With only the tips touching, the four fingers make
58% of the recorded turn, but two of the finger bodies would have to pass up
to 6 mm through the cylinder to do it, the cylinder tips by up to 52 degrees
from its starting pose, and it falls if its weight is not carried.

What is simulated. The finger is not simulated as a body, because no force
measurement exists for it. Each finger is a tip ball (radius half the finger's
depth) on a spring, pulled towards the measured free tip position for the
inputs it is given. The fingers stand where the reach check put them and take
the inputs the reach check chose; during a lift they follow the return the
reach check judged. The targets do not know where the cylinder is. Only the
tip balls touch. The cylinder is free to slide and turn but cannot fall (a
palm would carry its weight). The gait is run 5 times slower than the
recording. ASSUMED: the tip stiffness (three values from the one-pneu-net
stand-in of the upright arrangement: 7.9, 56.1 and 399.7 N/m, the middle one
the main case), a 5 g tip, friction as logged for the cylinder.

Result, cylinder turn over log 2.56 to 7.06 s (recorded: -161.6 degrees):

| tip stiffness (N/m) | two inputs per finger | one input held (control) | reference: tips sent to the logged contacts, no reach limit |
|---|---|---|---|
| 7.9 | -81.1 deg (50%) | +18.4 deg | -100.3 deg (62%) |
| 56.1 (main, the clips) | -94.3 deg (58%) | +71.4 deg | -146.5 deg (91%) |
| 399.7 | -84.8 deg (52%) | -16.2 deg (10%) | -153.2 deg (95%) |

In plain terms: with both inputs the four fingers keep the cylinder turning
the right way through lifts and landings and get about half to three fifths of
the recorded turn. With one input held they do not: the turn is small or
backwards. The reference shows that the stand-in itself can make nearly the
whole turn at the main and stiff values when reach is no limit, so the
stand-in is not what limits the two-input fingers. What does (short strokes,
gaps to the surface, or where the fingers stand) is not separated: their tips
touch 32 to 53% of the gait against the reference's 53 to 68%.

The comparison with the control is not like for like. The two-input fingers
stand where their bodies cut into the cylinder (next list); the control's
bodies stay clear except the ring (up to 1.4 mm on the logged motion).

What is wrong with it, all in the report:

- The drawn finger bodies cut into the cylinder. On the logged motion the
  index and middle pieces sit up to 3.8 and 6.6 mm inside it (51% and 37% of
  the time) and the thumb's up to 3.4 mm (12%); in the main run index and
  middle up to 6.0 and 6.1 mm (68% and 70%). A real finger
  in these places would collide. Different fingers' pieces stay 13 mm apart.
- The cylinder tips over a long way: its tilt changes by up to 52 degrees from
  its starting pose in the main run (38 in the reference), and its centre
  moves up to 21 mm. The recorded cylinder's axis is 8.2 degrees from vertical
  on average (from the reach check's report).
- Contact is patchy. Touching or not agrees with the log 51 to 75% of the
  gait, and the tips make 9 to 16 whole lifts where the log has 1 or 2.
- Forces are about a tenth of the recorded ones (0.08 to 0.11 N normal against
  1.0 to 1.3 N).
- With the cylinder free it drops at once in all three cases: these tips do
  not carry its weight.
- The inputs jump by up to 45 of 120 between touching samples 33 ms apart, and
  nothing is rate-limited. The finger's speed is not modelled.
- The two-pneu-net finger's own tip stiffness and force have not been
  measured; the stiffnesses are borrowed from a shorter finger.

Clips, three columns each (recording; logged contacts making and breaking; the
simulation), main stiffness, weight carried, all in the output folder:
`recorded_contacts_simulation_two_inputs.mp4`,
`recorded_contacts_simulation_one_input.mp4`,
`recorded_contacts_simulation_logged_path.mp4`, each with `_start`, `_middle`
and `_end` stills. Black dots on the simulated cylinder mark where a tip
touches now; the two-input clip's footer does not say so.

Not made: the longer recorded clip beside one-input pneu-nets with a fourth
added.

## 21 Nov: the fingers refitted so their bodies stay clear of the cylinder, and the simulation run again

GEOMETRY for the refit, SIMULATION for the re-run; both comparative, not
predictive. Refit script:
`tools/two_pneunet_fingers_follow_rl_gait_reach_check_bodies_clear_21nov.py`,
output `outputs/two_pneunet_fingers_follow_rl_gait_reach_check_bodies_clear_21nov/`
(`reach_check_report.txt`). Simulation and clips: the clip tool of the section
above, run with `--bodies-clear`, output
`outputs/two_pneunet_fingers_follow_rl_gait_clip_bodies_clear_21nov/`
(`gait_clip_report.txt`). Every number below is from those two reports. The
first placements' outputs are untouched.

What was wrong. In the first placements the fingers stood so that their drawn
bodies passed through the cylinder (index and middle, up to 6.6 mm on the
logged motion). A real finger there would collide.

What the refit does. Same search, same reach rules and thresholds as the reach
check. Added, and fixed in the script's header before the first run: the drawn
body (straight pieces, clamped end to measured junction to tip, half the
finger's depth each side, more than one depth back from the tip) is kept out
of the logged cylinder. While touching, a cut counts 10 times a tip miss
(ASSUMED weight). A return may not pass through inputs whose body cuts in. The
verdict is the clip tool's body check over the whole logged motion: deepest
cut no more than 0 mm, no margin.

Result of the refit, two inputs per finger:

| finger | deepest body cut on the logged motion (negative = clear) | contact error median, before -> after | clamped end moved |
|---|---|---|---|
| index | -0.0 mm | 1.84 -> 2.44 mm | 15 mm |
| middle | -1.3 mm | 0.55 -> 0.84 mm | 46 mm |
| ring | -1.6 mm | 2.69 -> 2.69 mm | 0 mm |
| thumb | -2.7 mm | 1.47 -> 1.54 mm | 33 mm |

The bodies are clear of the logged cylinder all the time (index only just).
All four fingers still pass both reach rules. Different fingers' pieces stay
12.0 mm apart. The fingers now stand nearly upright, 9 to 18 degrees from the
cylinder's axis, clamped 87 to 93 mm below its centre. Lift-off and landing
still fall well away from the recorded places (up to 17.0 mm, index landing).
Control, one input held, same rule: first reach rule 1 of 4, second 0 of 4.

Result of the simulation on the refitted placements, cylinder turn over log
2.56 to 7.06 s (recorded: -161.6 degrees), weight carried:

| tip stiffness (N/m) | two inputs, first placements | two inputs, refitted | one input held (control), refitted | reference: tips sent to the logged contacts |
|---|---|---|---|---|
| 7.9 | -81.1 deg (50%) | -54.8 deg (34%) | +10.9 deg | -100.3 deg (62%) |
| 56.1 (main, the clips) | -94.3 deg (58%) | -44.8 deg (28%) | +36.6 deg | -146.5 deg (91%) |
| 399.7 | -84.8 deg (52%) | -82.2 deg (51%) | -1.7 deg (1%) | -153.2 deg (95%) |

In plain terms: clearing the bodies did not help the turn. With two inputs the
cylinder still turns the right way and the control's turn is still small or
backwards, but the turn is 28 to 51% of the recorded one, lower than in the
first run at all three stiffnesses (barely at the stiff one). The turn does
not rise steadily with stiffness, so the run is sensitive; no cause is
claimed. At the main stiffness the fingers make -46.3 degrees in the
first round of the gait (recorded -77.0) and nothing more in the second. Why
the second round fails has not been looked into.

Still wrong, main stiffness, two inputs:

- The simulated cylinder tips over, by up to 47.5 degrees, and its centre moves
  up to 15.9 mm. Against the tipped cylinder the index body cuts in by up to
  2.5 mm (34% of the time) and the thumb by 0.4 mm (1%). Before the refit:
  index 6.0 mm (68%), middle 6.1 mm (70%). So the cut is smaller but not gone;
  it comes from the cylinder leaving its recorded pose, which the refit does
  not see.
- Contact is still patchy. Touching or not agrees with the log 93% (index),
  72%, 59% and 66% of the gait (first placements: 75, 64, 51, 59%), but the
  tips make 47, 34, 18 and 22 whole lifts (first placements: 9, 16, 10, 12)
  where the log has 1 or 2.
- Forces are about a tenth of the recorded ones (0.11 to 0.13 N normal against
  1.0 to 1.3 N).
- With the cylinder free it drops at once in all three cases.
- The inputs jump by up to 40 of 120 between samples 33 ms apart; nothing is
  rate-limited and the finger's speed is not modelled.
- As before: only the tip balls touch, the finger is a tip on a spring with a
  borrowed stiffness, and the junction's height is ASSUMED zero.

So: the reach check passes with bodies clear, but this simulation does not
show four two-pneu-net fingers replicating the recorded rotation. This search
found placements that reach; it did not find ones that turn the cylinder
through both rounds.

Clips, same three columns as above, main stiffness, weight carried, in the new
output folder: `recorded_contacts_simulation_two_inputs.mp4`,
`recorded_contacts_simulation_one_input.mp4`,
`recorded_contacts_simulation_logged_path.mp4`, each with `_start`, `_middle`
and `_end` stills.

Not made: the longer recorded clip beside one-input pneu-nets with a fourth
added.

## 21 Nov: the fingers mounted like a hand's fingers and drawn in their real shapes

Question: stand the four two-pneu-net fingers on a palm below the cylinder
like the fingers of a hand (first pneu-net wags sideways, second curls towards
the cylinder), roll each slightly so the wag is biased one way, and draw them
as real pneu-nets. Do the tip faces then follow the recorded contact points?

Result: two of four do. GEOMETRY ONLY, nothing simulated; comparative, not
predictive.

What changed from the earlier reach check:
- The finger is drawn and fitted in its real settled shapes (outer skin of the
  mesh, 25 x 25 input pairs, from the 20 Nov shape model of saved runs):
  `tools/two_pneunet_finger_shapes_export_21nov.py`.
- The tip face (the patch that faces the object when the second pneu-net
  curls) is put on the logged contact point, not the tip's centre with a ball
  around it. This is stricter: a ball can touch on any side, the face cannot.
- Mounting rule, fixed in the header before the first run: axis within 25 deg
  of the cylinder's axis, tip face towards the axis, roll only towards the
  side that helps the recorded stroke, limit 0, 15 or 30 deg; a fourth fit
  with no limits as the comparison. Reach rules unchanged.
  `tools/two_pneunet_fingers_human_like_mounting_reach_check_21nov.py`.

Median distance of the tip face's centre from the logged contact (limit
3.8 mm), by mounting (no roll / roll up to 15 / up to 30 / any mounting):
- index 5.49 / 6.47 / 6.84 / 6.25 mm: fails everywhere.
- middle 1.48 / 0.94 / 1.21 / 1.28 mm: passes everywhere.
- ring 5.08 / 4.88 / 4.78 / 4.21 mm: fails everywhere.
- thumb 2.46 / 2.73 / 2.45 / 2.31 mm: passes everywhere.

- Index and ring use the whole range of both inputs (0 to 120) and still fall
  short; this search found no mounting, hand-like or free, that brings them
  within the limit. It does not bound the finger's reach: the free fit of the
  index finger (6.25 mm) is worse than its no-roll fit (5.49 mm), so the
  search has not converged on contact error. The earlier check, with a ball
  round the tip's centre, passed 4 of 4; the new demand is the tip face, 7.3
  mm off that centre and facing one way (its 30 deg cone is ASSUMED).
- Where the miss sits (clip frames, median by first, middle and last third of
  each touching stretch): index 4.30, 3.29, 10.30 mm; ring 3.67, 3.31, 9.66
  mm; middle 1.23, 0.60, 0.87 mm; thumb 3.57, 2.59, 2.38 mm. Index and ring
  lose the contact towards the end of the stroke.
- Roll: with roll up to 15 deg, 14 to 22% of the wag points at the cylinder on
  the stroke (-2 to -6% with no roll), and fingers passing the return
  clearance rule rise from 1 to 2 of 4. Contact following barely changes.
- In all fits the recorded stroke goes with lowering the first input.
- Chosen for the clip by the rule fixed beforehand: roll up to 15 deg (axes
  9 to 16 deg from the cylinder's axis; tip face normals a median 13 to 32
  deg from pointing straight at the cylinder).
- The earlier progress remark that two fingers "curl away from the cylinder"
  in the previous placements was not checked and should be disregarded.

Clip (`tools/two_pneunet_fingers_human_like_mounting_clip_21nov.py`), three
columns: recording; logged contacts; real pneu-net shapes on the replayed
cylinder. Measured on its frames: contact error median 6.60 (index), 0.87
(middle), 4.98 (ring), 2.65 mm (thumb), largest 18.69, 5.45, 20.07, 13.95 mm;
deepest cut of any skin node into the cylinder 4.28, 2.47, 3.58, 2.69 mm;
fingers never nearer each other than 14.92 mm; lowest skin node 0.43 mm below
the drawn palm plate's top.

Simplified: no forces, no contact, no slip; the cylinder is the recording
replayed, so nothing here shows the fingers turning it; shapes are settled
free shapes, interpolated between input pairs; speed is not modelled.

Files, in `outputs/two_pneunet_fingers_human_like_mounting_21nov/`:
`reach_check_report.txt`, `reach_check_results.json`,
`recorded_contacts_pneunet_fingers_hand_like_roll_up_to_15_deg_16_9_misses_marked.mp4`
(the one to show: 1920 x 1080, each miss drawn on top as a ring at the logged
contact, a dot at the tip face's centre and a line between; same measures),
and the first render without those marks, 1920 x 908:
`recorded_contacts_pneunet_fingers_hand_like_roll_up_to_15_deg.mp4` with
`_start`, `_middle`, `_end` stills, `_report.txt` and `_measures.json`, and
`drawing_check_upright_starts_not_fitted.png`.

Not made: the clip of these fingers turning a free ball over repeated strokes
(agreed on 21 Nov to be made here, MuJoCo only).


## 22 Nov: three clips for the talk (refit with contact anywhere on the tip; cylinder on an axle; one-input pneu-nets beside a longer recording)

Every number below is copied from the named report file. SIMULATION OR GEOMETRY ONLY; comparative, not predictive.

### Contact anywhere on the tip (geometry only)

Script `tools/two_pneunet_fingers_human_like_mounting_contact_anywhere_on_tip_reach_check_21nov.py`; clip tool
`tools/two_pneunet_fingers_contact_anywhere_on_tip_clip_22nov.py`; folder
`outputs/two_pneunet_fingers_human_like_mounting_contact_anywhere_on_tip_21nov/`.

- The four hand-like-mounted fingers were refitted so that any node of the tip may touch the logged contact, not only
  the centre of the tip face. Median contact error: index 4.43 mm (was 6.47), middle 0.79 (0.94), ring 2.37 (4.88),
  thumb 1.07 (2.73). Deepest cut of the skin into the cylinder while touching: 0.22, 0.03, 0.19, 0.02 mm (limit 1).
- By the rules fixed beforehand, 2 of 4 fingers pass everything (middle, thumb). The index still fails the 3.8 mm
  contact rule; the ring passes it but fails a return-clearance rule (-5.4 mm). Contact rule alone: 3 of 4.
- The touching point wanders over the tip by 13.7 to 21.1 mm within a touching stretch. A real tip would roll or slide
  to do this; neither is modelled.
- Changed from the plan, stated in the refit script's header: one fresh start instead of three and half the reach
  check's iterations, to finish overnight. A fit
  with contact allowed anywhere on the whole finger was started as a comparison only and is not used here.
- Clip to show: `recorded_contacts_pneunet_fingers_contact_anywhere_on_the_tip_nearest_tip_node.mp4` (15.47 s,
  1920 x 1080) with `_start`, `_middle`, `_end` stills, `_report.txt`, `_measures.json`. On its frames the nearest tip
  node sits a median 4.51 (index), 0.82 (middle), 2.42 (ring), 1.06 mm (thumb) from the logged contact; frames over
  3.8 mm, drawn red: 165 of 308, 0 of 307, 142 of 308, 14 of 329. Index and ring still lose the contact in the last
  third of each stretch (median 7.40 and 5.21 mm). Over the whole clip the ring's skin cuts up to 2.31 mm into the
  replayed cylinder; the lowest skin node is 0.78 mm below the drawn palm plate's top.
- An earlier render the same night, `recorded_contacts_pneunet_fingers_contact_anywhere_on_the_tip.mp4`, drew the
  node the refit chose at the log sample nearest in time. That lags as the touching point wanders and overstates the
  miss (ring median 3.93 mm). It is kept, not shown. The second render applies the refit's own rule (nearest allowed
  node) to each drawn frame; both measures are in its report.
- "Touching" here is a fitting rule, not a checked contact: the refit report gives the index's nearest tip node a
  median 1.44 mm off the cylinder's surface, up to 8.65 mm.
- This clip replaces the 21 Nov `..._16_9_misses_marked.mp4` as the one to show.
- Simplified: no forces, no contact, no slip; the cylinder is the recording replayed, so this does not show the
  fingers turning it.

### Four two-pneu-net fingers turn a cylinder held on an axle (MuJoCo, contact and friction)

Script `tools/two_pneunet_fingers_turn_cylinder_on_axle_21nov.py`; clip tool
`tools/two_pneunet_fingers_turn_cylinder_on_axle_clip_22nov.py`; folder
`outputs/two_pneunet_fingers_turn_cylinder_on_axle_21nov_constraint_carry/` (`axle_report.txt`, `axle_results.json`).

- The cylinder's centre and axis direction follow the recording; only its turn about its own axis is free, and the
  axle has no friction. Fingers are the hand-like mounting with the tip face's centre aimed at the contact (not the
  refit). Finger stand-in: a tip ball on a spring (ASSUMED stiffness).
- Turn over the recorded window (recorded -161.6 deg): two inputs -63.6 deg (39 %) at the main stiffness, -49.0
  (30 %) soft, -51.9 (32 %) stiff. Reference with the tips driven along the logged paths: -147.0, -149.6, -146.1 deg.
- Fixed beforehand: beat the 21 Nov contact simulation's 28 %. Met by its letter (39 %). The plain reading is "about a
  third of the recorded turn at every stiffness, with or without the axle".
- Of the -63.6 deg, -46.8 came with a tip pressing and -16.8 with none (coasting on the frictionless axle; for 44 %
  of the gait no tip presses above 0.02 N). Tips touch for 5, 2, 26, 29 % of the time (index, middle, ring, thumb)
  against 76 to 82 % logged. Normal forces 0.025 to 0.12 N against about 1.1 N logged. Drawn skins cut up to 4.3 mm
  into the cylinder (only the tip ball collides).
- An earlier run that carried the cylinder by overwriting its position each step is superseded (folder
  `outputs/two_pneunet_fingers_turn_cylinder_on_axle_21nov/`); the run above carries it with a constraint and holds
  the path to 0.092 mm and 0.145 deg.
- Clip: `clip_two_inputs_main.mp4` (16.03 s, 1920 x 1080, 3 times slower) with `_start`, `_middle`, `_end` stills and
  `clip_two_inputs_main_report.txt`.
- Not done: the same run with the refitted (contact anywhere on the tip) inputs. By the rule in the script header the
  refit does not pass all its own rules, so it would be a comparison only.
- Not shown: a free object turned over repeated strokes. With the cylinder free it tips and falls (21 Nov runs).

### One-input pneu-nets beside a longer stretch of the recording

Script `tools/one_input_pneunets_cannot_reset_beside_recording_21nov.py`; folder
`outputs/one_input_pneunets_cannot_reset_beside_recording_21nov/`; clip `one_input_pneunets_beside_recording.mp4`
(19.0 s) with stills and `one_input_pneunets_beside_recording_report.txt`. Four one-input pneu-nets, palm fixed.

- The measure does not support "the one-input pneu-net cannot lift off and reset": in 1 of 7 complete returns the tip
  stays within 2.06 mm of the cylinder for more than half the return.
- Tips are a median 2.2 to 5.9 mm from the logged contacts; links cut up to 6.95 mm into the cylinder. Cause: the
  logged cylinder wanders 31.3 mm and tilts 12.3 deg while the palm is fixed.
- Whether to show it is open. A palm that follows the cylinder, or a shorter window, would be a new variant.


## 22 Nov, later: tips aimed a fixed depth inside the cylinder's side (on the axle, then with the cylinder free and lighter)

Every number below is copied from the named report file. SIMULATION ONLY (MuJoCo, contact and friction); comparative,
not predictive. Asked for after the axle clip above showed the tips breaking contact many times within a touching
stretch.

### What was changed

While the recording has a finger touching, its tip ball is aimed so that its surface is 3.8 mm inside the cylinder's
side. The depth is ASSUMED: it is the 3.8 mm contact limit already used in the reach checks, fixed in the script
header before the first run and not tuned afterwards. With the main spring stiffness (56.1 N/m, ASSUMED) a tip held
that far back pushes with 0.21 N. The finger keeps its own position along and around the cylinder; its own radial
reach is REPLACED by the assumption while touching (rest position moved by a median 3.6 to 4.7 mm, at most 18.3 mm for
the index and 16.8 mm for the ring). So this run says what the gait does if the fingers reach the surface; it no
longer tests whether they reach it.

### Cylinder on the axle

Script `tools/two_pneunet_fingers_turn_cylinder_on_axle_fixed_press_in_22nov.py`; clip tool
`tools/two_pneunet_fingers_fixed_press_in_clip_22nov.py`; folder
`outputs/two_pneunet_fingers_turn_cylinder_on_axle_fixed_press_in_22nov/` (`axle_report.txt`, `axle_results.json`).

- Turn over the recorded window (recorded -161.6 deg): -96.1 deg (59 %) at the main stiffness, -96.8 (60 %) soft,
  -88.8 (55 %) stiff. The earlier axle run without the press-in gave -63.6 deg (39 %).
- No coasting: 0 % of the gait has no tip pressing; -95.9 deg came with a tip pressing.
- Tips touch for 75, 77, 70, 82 % of the gait (index, middle, ring, thumb) against 76, 77, 76, 82 % logged.
- "One contact on, one contact off" in every logged touching stretch holds for the middle and thumb only. Touching
  runs per logged stretch: index 1, 1, 2; middle 1, 1, 1; ring 1, 12; thumb 1, 1, 1. The ring's own position is beyond
  the cylinder's flat ends for 52 % of its touching time.
- Normal force 0.207, 0.206, 0.168, 0.207 N against 1.18, 1.04, 1.35, 1.15 N logged.
- What the clip shows (`contact_breaks_fixed_depth_main_fixed_depth_stiff.txt`, from
  `tools/two_pneunet_fingers_fixed_press_in_contact_breaks_22nov.py`): on the drawn frames the index, middle and thumb
  each have 3 touching stretches, one per logged stretch, each switching on once and off once. The ring has 4: in its
  second stretch it touches from log time 5.21 to 5.55 s, flickers once (5.63 to 5.67 s) and is off until 5.88 s. The
  report's "whole lifts" (3, 4, 13, 5 against 2, 2, 1, 2 logged) count every simulation sample; the extra ones for
  the index, middle and thumb are single samples at the ends of stretches and do not reach a drawn frame.
- Noticed after the runs, not chosen beforehand: the stiff value gives the cleanest contact over every sample (index,
  middle and thumb 3 stretches each, ring 6) with forces near the logged ones, and turns less (55 %). On drawn frames
  it looks the same as the main value (3, 3, 4, 3 stretches). The clip is of the main value, as fixed in the header.
- At the stiff value every finger but the ring has one run per stretch (ring 1, 5) and the forces are near the logged
  ones (1.0 to 1.3 N). At the soft value the contact chatters (index 1, 13, 28).
- Comparison reading, the finger's own rest position moved 3.8 mm towards the axis instead: -106.6 deg (66 %) at the
  main stiffness, but more broken contact (index touching 46 % of the gait; ring 4 and 16 runs).
- The drawn skin is the free shape, not pressed in; it cuts up to 4.3 mm into the cylinder. Only the tip ball
  collides.
- The clip's foot line promises coloured dots where a tip presses. None is visible in the three stills: the dot sits
  on the cylinder's surface behind the drawn skin. The line of text naming the pressing tips is what shows it.
- Clip: `clip_fixed_depth_main.mp4` (16.0 s, 1920 x 1080, 3 times slower) with `_start`, `_middle`, `_end` stills and
  `clip_fixed_depth_main_report.txt`.

### Cylinder free, mass swept downward

Script `tools/two_pneunet_fingers_turn_free_cylinder_lighter_22nov.py`; folder
`outputs/two_pneunet_fingers_turn_free_cylinder_lighter_22nov/` (`free_cylinder_report.txt`).

- Same tips and depth, main stiffness, aimed at the RECORDED cylinder (they do not react to the simulated one). No
  axle, no palm, no floor. Mass list fixed beforehand: 1, 1/2, 1/4, 1/8, 1/16, 1/32 of the recorded 23.9 g, inertia
  scaled with it. HELD: not dropped 30 mm and centre within 30 mm of the recorded centre. TURNS: the recording's sign
  and more than 28 % of the recorded turn.
- Without gravity, recorded mass: held and turned, -72.2 deg (45 %).
- 23.94 g: not held (dropped 30 mm at log time 5.55 s), -41.1 deg (25 %). 11.97 g: not held (dropped 30 mm at log
  time 7.02 of 7.06 s), -61.1 deg (38 %).
- 5.99 g: held and turned, -63.6 deg (39 %). 2.99 g: -67.8 (42 %). 1.50 g: -69.5 (43 %). 0.75 g: -70.2 (43 %).
- Heaviest cylinder in the list that was held and turned: 5.99 g, a quarter of the recorded mass.
- "Held" is loose. At 5.99 g the centre is up to 26.1 mm from the recorded centre (25.4 mm at the end, having sunk up
  to 17.0 mm) and the axis tilts up to 29.3 deg (3.1 deg at the end). Even with no gravity the axis tilts up to
  32.4 deg.
- Contact is more broken than on the axle: at 5.99 g the touching runs per logged stretch are index 1, 5, 1; middle
  1, 6, 1; ring 1, 1; thumb 1, 1, 5.
- The press-in is now also the grip: 0.21 N per tip, and with friction 0.72 up to 0.15 N along the surface, against a
  weight of 0.235 N at the recorded mass.
- The 39 % at 5.99 g (-63.65 deg) matches the earlier axle run without press-in (-63.57 deg) by chance; they are
  different runs.
- On the clip's drawn frames at 5.99 g (`contact_breaks_mass_5p99_g.txt`): touching stretches index 4, middle 5,
  ring 2, thumb 5, against 3, 3, 2, 3 logged; the extra ones are flickers shorter than 0.1 s of log time, plus one
  thumb gap from 6.64 to 6.85 s.
- The end still reads -63.0 deg, not -63.6: it is the last frame of the one second after the gait.
- Clip: `clip_mass_5p99_g.mp4` (16.0 s, 1920 x 1080) with stills and `clip_mass_5p99_g_report.txt`. The cylinder
  visibly sinks onto the drawn finger bodies, which do not collide; a plate is drawn under the fingers but is not in
  the simulation.
- Not done: finer mass steps between 5.99 and 11.97 g; other stiffnesses with the cylinder free; a palm.
