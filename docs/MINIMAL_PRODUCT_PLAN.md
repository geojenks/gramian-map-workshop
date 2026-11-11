# Plan to a minimal product

Written 27 Oct 2026, after the workshop talk was agreed. It sits beside
`docs/WORKSHOP_PLAN.md` and does not replace it: that plan tracks the
research steps; this one lists what must work, end to end, for someone to
bring an actuator or a hand and get a verdict on a task. Moving to dynamics
is being planned in another session and is left out here.

`docs/RUN_THROUGH.md` is one run of the whole flow from a fresh clone
(11 Nov), with where it stopped.

## What the minimal product is

One flow that a visitor can follow with their own data:

1. **Actuator in.** An actuator enters the map from one of three routes,
   and every route ends in the same actuator record (schema in the M-11
   project, `gramian_map/docs/RECORD_SCHEMA.md`). The record is the only
   way in.
2. **Hand.** Actuators are placed into a hand, or a ready-made hand is
   loaded from a model file.
3. **Task.** A task is taken from an example: a simulation's contacts, or a
   recorded human demonstration.
4. **Verdict.** The frame-by-frame force check says, for each frame,
   whether the hand can follow the task by this method.

The verdict is narrow by design. A fail means "this hand cannot do this
task by this method", never "this hand cannot do this task". A task can be
done by one method (a gait, a regrasp, a wrist roll), and a given hand may
need a different one.

## Where each stage stands

| Stage | Working now | Part way | Not started |
|---|---|---|---|
| Actuator in: mesh | SOFA pipeline in the M-11 project; pneunet, bellows, McKibben, cable-finger records | | Run live at the workshop (not planned: hours of compute) |
| Actuator in: test data | Test-curve file format (`docs/TEST_CURVE_FORMAT.md`) and converter to a record (`tools/curves_to_record.py`, both ways); records digitised from published curves (`mosadegh2014_*`, `abboodi2026_*`); `forms/record_form.html` and `tools/check_record.py` | Angle-only curves are kept as `free.bend_angle_rad` and make no tip path, so the stand-in fitter refuses them (ruled, issue 23) | |
| Actuator in: analytical | `shadow_hand_finger` (forward kinematics from published dimensions); a worked example starting from a per-joint table (axis, position, range, torque limit, stiffness; ruled on issue 16): `docs/JOINT_TABLE_FORMAT.md`, `tools/joint_table_to_record.py` (2820556) | The record it writes crashes the hand build (issue 33, run-through of 11 Nov) | |
| Actuator onto the map | `make_atlas.py` in the M-11 project rebuilds the map from the records | | Adding one uploaded record without a full rebuild |
| Hand from a model file | Allegro, Shadow, LEAP (MuJoCo Menagerie), Linker L20 (vendor URDF), human-finger hands; a visitor's own URDF or MJCF through `tools/add_hand.py` (`docs/BRING_YOUR_HAND.md`, 7f5859f, issue 30) | | |
| Hand from actuator records | Hand file and build tool (`tools/build_record_hand.py`): fitted stand-in fingers on a palm, fit check, round trip; `wall_x3_ring` (three `wall_0.0010m` stand-ins on a 30 mm ring) | Verdicts provisional: the drive is the record's unitless level (ruling 2). A record made from test curves needs a contact radius and a bend direction that the docs do not give (issue 32), and a record-built hand added with `tools/add_hand.py` loses the provisional caveat (issue 35) | |
| Task from a simulation | `tools/record_leader.py`, `tools/export_tasks.py`; the task-file format (`docs/TASK_FILE_FORMAT.md`) with `tools/check_leader.py`; corrected `__contactfix` copies of the w10 cup leaders; a static hold leader per hand from its own kinematics (`tools/make_hold_leader.py`, `data/leaders/hold_<hand>__self.json`), checked with margins (`outputs/hold_summary.md` from `tools/tabulate_holds.py`) | The Shadow's hold is red (torque): its coupled J1/J2 pair cannot carry the pad load as posed, and the verdict moves with the pose (an earlier pose of the same search was amber). Given a passive stiffness on every joint it turns green from 0.02 N.m/rad (`tools/hold_stiffness_bracket.py`, table `outputs/hold_shadow__kq_bracket.md`), the margin rising in step with the stiffness within the check's 2 mm deflection allowance, so the spring carries the hold; which stiffness the real tendon has is not known. With the hand's joint end stops credited as reactions (issue 26; `follow(stop_reactions=)`, on by default; `outputs/hold_shadow__kq_bracket__stops.md`) the hold then reads red at kq 0 (index red, middle red, ring red, little green, thumb green), amber at 0.001 to 0.01 and green from 0.02 N.m/rad, against red to 0.005, amber at 0.01 and green from 0.02 without the stops; the scale margins are 0, 1.16, 1.38, 2.1, 3.39, 6.15, 14.8, 29.5, 58.8 against 0, 0.114, 0.229, 0.572, 1.1, 1.92, 4.47, 8.8, 17.5 (3.1 to 10 times, the stops carrying what the spring otherwise had to). The one reaction credited is the little finger's J1 on its upper stop, 0.0044 N.m at kq 0 falling to 0.0032 at 0.01, and none from 0.02 N.m/rad, where the spring carries it; FFJ4, RFJ4, LFJ4, THJ3 rest on their stops but their own drives balance them. A spring put in the model instead (copies of the Shadow with a spring on the fingers' J1, or on J1 and J2, `tools/make_shadow_spring_hand.py`, regenerated per machine and not committed; checked through follow.py unchanged by `tools/hold_shadow_model_spring.py`, table `outputs/hold_shadow__model_spring.md`) does not rescue the hold: the check reads a model spring as a torque k (q - springref) at its own pose, never as a reaction (the pose is identical across every copy, and a copy whose springref sits at each posed angle reproduces the no-spring copy to 1e-7 N.m at k 0.02). A J1-only spring offsets the J1-J2 mismatch, so each finger's unbalanced torque falls to a minimum at a different stiffness (little between 0.0005 and 0.001 N.m/rad, middle between 0.001 and 0.002, index and ring between 0.0017 and 0.002) and grows with the spring beyond it; no single stiffness turns the hand green, and the frames are amber from 0.001 up. Its scale margin equals the uniform-stiffness bracket's at ten times the stiffness (to within 1 %), so it reads spring torque against the mismatch, not the hand's capacity. The "J2 flexes first, J1 at the stop" reading is issue 26's end-stop credit above, which the check gives only where the IK leaves J1 on its stop: the little finger, and only up to kq 0.01 (the index and ring are posed with J4 on its stop, the middle with no joint on one). Each finger as an actuator in its own right (`gramian_workshop/finger_actuator.py`, `tools/hold_shadow_finger_actuators.py`, `outputs/hold_shadow__finger_actuators.md`): a MuJoCo forward model of one finger against the fixed sphere (other fingers made non-colliding) with the J1-J2 tendon servo, the end stops and the joint dry friction as the menagerie models them; the input that presses hardest is bisected for the 0.1 N share and the others are moved for the station and the force direction, and a finger holds when its settled force is within RESID_TOL of the leader's and its contact within REACH_AMBER_M. The Shadow still does not pass. With friction as modelled the middle and little fingers hold (amber, 6.77 and 2.83 mm off station; MFJ1 and LFJ1 against their upper stops at -0.0017 and -0.0015 N.m) and index, ring and thumb reach the magnitude but not the direction (11.5, 11.6 and 7.6 deg off, force departures 0.020, 0.020 and 0.013 N against 0.005); with friction zeroed no finger holds (index, middle and ring 33 to 36 deg off with J1 at the upper stop carrying -0.0003 N.m, little 5.2 deg, thumb 3.2 deg at 0.0056 N). Assembling the fingers' best inputs gives a wrench residual of 0.30 (0.27 without friction) against 0.019, and the released sphere drifts 5.7 to 7.5 mm in 1 s (not held; with the hand's own collisions on, the pose's seven self-colliding finger pairs, deepest 11.1 mm, fling it). The finger-level model therefore says what is missing is force direction at the station, not torque: the pad presses along J3's arc inside the friction cone and the search found no input in range that turns it further. J1 reaches its upper stop only with friction zeroed; with friction as modelled the failing fingers sit on no stop and the friction rows carry the mismatch (up to 0.0008 N.m), while the two holding fingers are the ones on their J1 stop, so the stop credit of issue 26 cannot be what separates holding from failing fingers (numbers posted there). The self-hold poses overlap the hand with itself, greens included (`tools/hold_pose_self_overlap.py`, table `outputs/hold_pose_self_overlap.md`; the 13 stored results are read, no check is re-run): the hold generator and the check measure overlap against the ball only (the generator's other-link overlap and palm penetration, the check's `overlap_mm`, `other_overlap_mm` and `palm_overlap_mm`), each finger is fitted and then solved on its own, and nothing tests one finger against another. At the follow pose 11 of the 13 hands have a finger inside another finger; deepest per hand: allegro 17.5 mm (index tip and thumb tip, their contact points 5.1 mm apart on links whose radii sum to 24.0 mm), wall_x3_ring 12.9 mm (all three distal links through one another), Shadow 11.1 mm, leap 10.6 mm, and 0.4 to 8.5 mm on the seven capsule human hands that have any (george_kp and george_measured have none). The generator's own pose already has most of it (allegro 15.7 mm, wall_x3_ring 12.9, Shadow 15.8, leap 6.0). The palm-to-proximal pairs on the capsule human hands and wall_x3_ring (up to 19.2 mm) are there at the rest pose too, so they are how those models are built, not the hold. The pose set for the table reproduces every stored depth into the ball exactly, so these are the poses the verdicts were given at; MuJoCo leaves out parent-child and excluded pairs, so the counts are a lower bound. A green self-hold therefore says the torque test and the hand-to-ball tests pass at that pose, not that the hand can reach it; testing the hand against itself, in the check and in the generator's score, is issue 27 (ruling of 6 Nov: the overlap is reported only and does not change the verdict). The check now reports it: each frame of a result carries `self_overlap`, the hand's own bodies that penetrate at that frame's whole-hand pose (every finger at its solved pose, a finger not touching at rest), each pair with its depth and the same pair's depth at the rest pose, and the deepest depth beyond rest for a finger against another finger and for a finger against the palm. No verdict, reason or finger state reads it, and no allowance applies (`tests/test_follow_self_overlap.py`: a run with the measure replaced by a made-up overlap gives the same result, and the pairs are those of `outputs/hold_pose_self_overlap.json` at the same poses). It is a lower bound, as the table is. Results stored before 6 Nov do not have the key, and their verdicts stand. The hold generator's score is the other part of issue 27, done the same day: leaders regenerated with the hand against itself in the score (go-ahead of 6 Nov on issue 27; `hold_<hand>__self__selfoverlapfix`, each superseding `hold_<hand>__self`, which stays; `tools/make_hold_leader.py --no-self-term` still reproduces the first leaders): the score adds the deepest penetration between two of the hand's own bodies at the whole-hand pose, net of the rest pose (so the palm-through-proximal pairs the capsule hands and the L20 are built with do not steer the search; ruling of 10 Nov: net of rest stands), weighted like the palm term, and the search also starts from the pose chosen without the term. Tables `outputs/hold_summary__selfoverlapfix.md` and `outputs/hold_pose_self_overlap__selfoverlapfix.md` (before and after, hand by hand); the first leaders checked again at the same commit for a like-for-like verdict, `outputs/hold_summary__recheck_93d5b39.md` (the new leaders and tables record commit 1894096, the pre-rebase form of 85139db with follow.py as of 702a7a3; the stored `outputs/hold_summary.md` is the check of 3 Nov, before the end-stop credit; at this check george_clip's and george_kp's margins are 752 and 1024 against 458 and 704 stored, and the Shadow's little finger is green instead of red). At the generator's own pose the finger-against-finger overlap is zero on 9 of the 13 hands (allegro 15.7 mm to 0, Shadow 15.8 to 0, wall_x3_ring 12.9 to 0, leap 6.0 to 0; human_vc and human_vc_prop had none before either) and 0.2 to 3.4 mm is left on george_measured, george_measured_thumb, human_vc_arch and human_vc_arch_prop (the search keeps it where clearing it costs more contact than the weight buys). At the follow pose, where the check re-solves each finger onto the ball on its own, it is gone on allegro (0.9 mm of thumb against palm remains), Shadow, wall_x3_ring and george_measured_thumb, still zero on george_measured, down on leap (10.6 to 0.7), human_vc_arch (8.0 to 4.4), human_vc_arch_prop (8.5 to 2.3) and george_kp_wide (3.7 to 1.5), unchanged on human_vc (4.2) and human_vc_prop (2.2), and new on george_clip (1.4 to 9.9 mm, little tip through thumb tip) and george_kp (0 to 1.4): 8 of 15 hands still have a finger inside another at the pose the verdict is given at, because the check's finger solve has no such term either (follow.py, issue 27). Verdicts, like for like: 11 hands unchanged (10 green, the Shadow red on torque, now on all five fingers where the first leader at this check had the little finger and thumb green); george_measured_thumb green to amber (little finger 3.0 mm off its station, facing 35 deg) and wall_x3_ring green to amber (each finger's second hinge posed 0.006 to 0.011 rad past its upper limit, 0.0007 to 0.0013 N.m unbalanced); l20 and l20_public (Linker L20, fetched 6 Nov; the menagerie model and the public one) green at their first check. The leap's pose with the term is the finding: no pose on the grid both clears the fingers and holds the ball (pads 8 to 13 mm off it, facing up to 85 deg), yet the check reads green because its own solve bends each finger back onto the ball; wall_x3_ring's pose with the term has the three pads facing 40 to 58 deg from the ball against 2 deg before. Feed-forward check (6 Nov, `gramian_workshop/feedforward.py`, `tools/hold_shadow_feedforward.py`, table `outputs/hold_shadow__feedforward.md`): the question is narrowed to whether each Shadow finger, at the pose and pad point follow.py found, can apply the leader's contact force at that instant; nothing is settled or released and the object's stability is not tested. With follow.py's own bounds the statics reproduce follow.py's unbalanced torque (index 0.00164, middle 0.00154, ring 0.00166 N.m, little and thumb 0) and 2 of 5 fingers can apply the force. Limiting each input to what a command inside its control range can hold at that pose leaves 1 of 5: the thumb's THJ3 is posed at the end of its command range and can push one way only, which follow.py's force-range bounds do not see. Crediting the model's joint dry friction (0.01 N.m) makes it 5 of 5, with the index, middle and ring end joints leaning on 11 to 12 % of it and THJ3 on 34 %. Without friction the nearest force a failing finger can hold is 0.087 to 0.099 N away from the 0.1 N demanded, although the torque shortfall is only about a fifth: the J1-J2 tendon gives both joints the same torque, so the finger holds only forces that load them equally. This does not contradict the finger-by-finger forward model above, where 2 of 5 held with friction as modelled: there each finger was let settle against the sphere and moved off the station (the index by 6.35 mm), here the pose is frozen and only that instant is asked about. That earlier result is evidence that, in the model, joint friction does not keep the finger where the static balance puts it, so the friction reading is the optimistic one. Ruling of 6 Nov: joint friction is not something the hand commands, so it is not an input to the verdict (1 of 5 with servos and stops), but a property the hand has, so the check states what the hold needs of it: the minimum stiction per joint once servos and stops have done all they can, 0.0011 to 0.0012 N.m on each of the index, middle and ring J1 and J2, 0.0034 N.m on THJ3, none on the little finger, against the model's 0.01 N.m (`outputs/hold_shadow__feedforward__stiction.md`, re-rendered from the same JSON). If the real joints have that much friction the hold's forces are feasible to feed forward; the model's figure is the menagerie's, not a measurement. This leader is kinematic and asks for pure normal forces, so it is a dry run of the method for leaders extracted from simulation. Since 10 Nov (issue 29) each finger of a result also carries the joint balance its torque test left, row by row (`joint_residuals`: joint, kind, residual in N.m or N, and the bar the test held the rows' norm to; the redistribution's rows too when it ran), which of its own tests it missed (`missed`) and the friction-cone angles; reported only, no verdict reads them; `failures.py` restates them since 11 Nov (own tests missed, and the joint carrying the largest residual of each torque test, with the bar given as the bar on the rows' norm) | |
| Task from a human demo | `tools/import_demo_leader.py` (tennis ball, take 3); corrected `__nframesfix` copies | Object mass is assumed, not measured | |
| Verdict | `follow.py` for model-file hands; springs read from the model; fingers with fewer drives than joints posed by equilibrium; each joint type checked in its own unit (per-finger test); overlap scored against the task's stated allowance (default 3 mm); stored placements reused; both margins stored with the result and shown (`tools/run_follow.py --margins`, 84fed67, issue 8) | A finger posed past a joint limit is recorded but does not change the verdict (issue 34); redistribution and margins still put all joints under one tolerance; fresh placement searches take 45 to 65 min a hand (screening on a few frames not yet built) | |

### Stored results that predate the current check

Nothing in `outputs/` written before 28 Oct can be quoted as a verdict:

- Results written before 6922372 (26 Oct) predate the check that a finger
  must not sink into the object. 35 of the 49 stored follow results flip
  under the current code (issue 21, 28 Oct). They stay as they are; their
  verdicts are superseded by a fresh placement search.
- Results from the stored `w10_cup_physical_*` leaders aimed, in up to half
  the frames, at a station flagged touching with no contact point: the
  station centre, 8 to 9 mm off the ball. The `__contactfix` copies replace
  those leaders (issue 22, 28 Oct). One stored result is from a corrected
  copy:
  `outputs/follow_wall_x3_ring__w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4__contactfix.json`
  (35 frames red, 5 amber; provisional on ruling 2). `outputs/follow__superseded.md`
  lists the 39 that still need a fresh placement search.
- The two `follow_wall_x3_ring__*__provisional` files were written before
  equilibrium posing (89a6b86). Their reds are an artefact of a pose choice
  from geometry alone, not verdicts on the finger (issue 21 ruling 3).

## Rulings needed before the work

1. **How a record-built finger enters the force check (issue 5).** This
   decides what a soft hand is in this tool. It is the one join between
   "actuator in" and "verdict"; until it exists, an uploaded actuator stops
   at the record.
2. **Units.** The simulated records run at low pressures (the pneunet
   `wall_0.0010m` spans 0 to 398 Pa) and are comparative only. The
   published-data records are in physical units (`mosadegh2014_fPN` spans
   0 to 72 662 Pa). On one map they will mislead. Options: keep two maps,
   or scale the simulated records to physical units by a stated rule.
   10 Nov: the low simulated pressures are consistent with the simulated
   load going as the stated value divided by the time step (100 times at
   0.01 s; geojenks/Gramain_extraction issue 1, open). Not settled.
3. **The hand file.** A plain file that says which record sits at which
   mounting point on a palm. A drag-and-drop page can come later and write
   the same file.

## Order of work

1. **One example per actuator route, shown entering the map.**
   - Mesh: `wall_0.0010m`, pre-run, shown from mesh to record to map.
   - Test data: `mosadegh2014_fPN`, from digitised curves to record.
   - Analytical: a finger from a written Jacobian, through the record form.
     Done from a joint table instead (2820556):
     `data/joint_tables/shadow_hand_finger.csv` to a record.
   - Copy the published-data records into `data/records/` with
     `tools/import_records.py`; the workshop copy predates them.
2. **State the test-data format** (done 28 Oct, da65543). Most visitors will bring test curves for
   one actuator: input against displacement, input against blocked force,
   with units. Write the format down with one filled example.
3. **The hand file and the join into the force check** (rulings 1 and 3;
   done 28 Oct, 0b628b8 and a479ad7 to 9b74767, provisional on ruling 2).
   Then one hand built from records, run on the cup tasks: run on the stored
   cup leaders and on one corrected copy (35 frames red, 5 amber;
   provisional).
4. **Loading a visitor's model file** (done 7f5859f, `tools/add_hand.py`;
   worked example: the left Allegro from MuJoCo Menagerie). One hand loaded
   from a URDF that we did not prepare ourselves.
5. **The task-file spec** (done 28 Oct, faa5b37 to cfbc15b). Publish the leader file format so others can
   bring tasks from their own simulations.
6. **Store and show both margins** with every verdict (issue 8; done
   84fed67).

## The task library

Show the whole tree; fill a few branches completely. Each branch needs a
recorded demonstration, and they cost effort. The empty branches are the
invitation to other groups. Elliott and Connolly's patterns give a citable
way to lay out the top of the tree.

| Task | Method | State |
|---|---|---|
| Hold a ball | Fingertip cup, static | Filled: `cup_hold`; and `hold_<hand>__self`, one self-made hold per hand (the known answer for the check) |
| Turn about the vertical axis | Make/break wave in a fingertip cup | Filled: `cup_turn` |
| Turn a ball in the hand | Human demonstration | Filled: tennis ball, take 3 |
| Turn about each horizontal axis | Gait, regrasp | Empty |
| Turn a screwdriver half a turn | Contacts held, wrist roll | Concept only |

## Abstractions to test, to find the limits

- Fewer fingers: a five-finger demonstration given to four or three
  fingers, by searching ways to reassign the contacts.
- Two hands sharing one task.
- Looser tolerance on the object's path.

Where we expect the method to break, stated in advance:

- soft fingers whose contact point moves as they deform;
- anything dynamic (the check is quasi-static);
- tasks whose contact plan nobody has written yet;
- mixing simulated and physical units (ruling 2).
