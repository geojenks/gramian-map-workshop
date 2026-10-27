# Simulation benchmarks for manipulation: how they score a hand or a policy

Notes compiled 27 Oct 2026.

## How these notes were made

- Each paper was read through a web-fetch tool that returns a model-written summary of the page. So "full text read" means the whole HTML text was passed through that summary step, not read line by line by us. Short quotes came back through the same step, so treat them as near-verbatim.
- Older arXiv papers were read via ar5iv (HTML renderings of arXiv papers). Newer ones via arxiv.org/html.
- Source marks:
  - [full text read]: the whole HTML text was processed.
  - [full text, first ~100k chars]: the page was longer than the tool's limit. The tail (often appendices) was not read. Anything that lives only in an appendix is marked [UNVERIFIED].
  - [abstract]: only the arXiv abstract page.
  - [docs page]: project documentation, not the paper.
- Venues are given only where a page stated them. Otherwise [UNVERIFIED].
- Our own tool, for reference: a per-frame check of whether a given hand can supply the wrench a recorded demonstration needs. Verdict per frame, per hand, per task.

---

## Dexterous-hand benchmarks

### DexMV (Qin et al.)
Source: arXiv 2108.05877, via https://ar5iv.labs.arxiv.org/html/2108.05877 [full text read]. Venue [UNVERIFIED].

What the paper says:
- Tasks: three families, seven variants. Relocate (five YCB objects), Pour (particles from a mug), Place Inside (banana into a mug).
- Hand: Adroit hand in MuJoCo, 30 actuated DoF (24 hand and wrist plus a 6-DoF root).
- Demonstrations: human videos. 3D hand (MANO) and object poses are estimated, then retargeted to the robot by matching fingertip-to-palm and fingertip-to-phalanx vectors. Robot actions come from inverse dynamics on a minimum-jerk fit.
- Scoring: Relocate succeeds if the object ends within 0.1 (units not stated) of the target after 200 steps. The object "does not need a specific orientation". Pour is scored by percentage of particles poured. Place Inside by percentage of object volume inside the mug.
- Physics: inverse dynamics gives torques, but no step checks demonstration forces against the robot's limits.
- Morphology: one hand only.

### DexArt (Bao et al.)
Source: arXiv 2305.05706, via https://ar5iv.labs.arxiv.org/html/2305.05706 [full text read]. arXiv comments say CVPR 2023.

What the paper says:
- Tasks: four. Faucet, Bucket, Laptop, Toilet. 82 articulated objects in all (50 seen, 32 unseen).
- Hand: Allegro (16 DoF) on an xArm6, in SAPIEN.
- Scoring: binary success per task. Faucet: the hand must "firmly grasp the handle and then rotate it by around 90 degrees". Others use a lift height or lid-angle threshold whose values are not given in the text.
- Focus: generalisation to unseen object instances.
- Physics: contact is a boolean collision check. No force checks.
- Morphology: Allegro only. Grippers are argued against in words, not tested.

### Bi-DexHands (Chen et al. 2022)
Source: arXiv 2206.08686, via https://ar5iv.labs.arxiv.org/html/2206.08686 [full text, first ~100k chars]. Abstract at https://arxiv.org/abs/2206.08686 [abstract]. Venue: the arXiv page lists none [UNVERIFIED].

What the paper says:
- Tasks: about 20 bimanual tasks. The abstract says "tens of bimanual manipulation tasks" with thousands of target objects. Examples: Hand Over, several Catch tasks, Lift Pot, Door Open/Close, Bottle Cap, Swing Cup, Open Scissors, Re-Orientation, Pen Cap, Switch, Stack Block, Pour Water. Full list [UNVERIFIED] past the truncation.
- Hand: two Shadow Hands, 24 DoF each, 20 actuated.
- Scoring: per-task dense reward (hand-to-object distance, object-to-goal pose error). Results reported as average reward. Offline and multi-task results normalised between random and expert scores. No success threshold found in the part read.
- Dynamic tasks: yes. Catch Underarm, Catch Over2Underarm, Two Catch Underarm, Catch Abreast involve objects in flight. Hand Over throws the object.
- Grouping: tasks are mapped to infant and child motor milestones (about 5 to 48 months, plus one adult-level task). Grouped Easy, Medium, Hard.
- Physics: joint ranges, stiffness and damping given. Fingertip force/torque readings are observations. No actuator-limit check on success.
- Morphology: Shadow Hand only. Compares RL algorithms (PPO, SAC, MAPPO, offline RL, meta-RL).

### TCDM: "Learning dexterous manipulation from exemplar object trajectories and pre-grasps" (Dasari, Gupta, Kumar)
Source: arXiv 2209.11221, via https://ar5iv.labs.arxiv.org/html/2209.11221 [full text read]. arXiv page says an abridged version was at ICRA 2023.

What the paper says:
- Tasks: 50 (TCDM-50). A harder subset of 30 (TCDM-30) drops ten simple lifting tasks. Tasks come "without any per-task engineering and/or supervision".
- Task source: each task is an exemplar object trajectory (a time series of object poses). 40 from GRAB motion capture, 3 from Adroit expert or tele-op policies, 7 from human animators. Objects from ContactDB and YCB.
- Scoring against the demonstrated motion: yes.
  - Reward is an exponential of object position and orientation error, plus a lift bonus.
  - Episode ends if the object strays more than 0.25 m from the target.
  - Success = fraction of timesteps where object centre-of-mass error is below 1 cm.
  - Also reported: time-averaged position error (m) and orientation error (rad).
  - Humans judged roll-outs scoring 60 to 80% as successful.
- Hands: Shadow Hand (30 DoF incl. arm), D'Hand, D'Manus. Hands were assigned to tasks arbitrarily. No hand-against-hand comparison.
- Physics: joint-position actions through a PD torque controller, gains "tuned to be realistic". No force or torque limit checks. The authors note remaining imperfections such as large forces.
- Dynamic content: includes cyclic tasks (hammering, bottle shaking) as well as goal-reaching ones.
- Grouping: by trajectory source and pre-grasp source. Not by application.

### DexMachina (Mandi et al. 2025)
Source: arXiv 2505.24853, via https://arxiv.org/html/2505.24853 [full text read]. Venue: none stated [UNVERIFIED].

What the paper says:
- Tasks: seven demonstration clips on five articulated objects, taken from ARCTIC. Short and long clips.
- Hands: six. Main results on Inspire, Allegro, XHand, Schunk. Ability and DexRobot Hand added in the hand-design analysis.
- Scoring against the demonstrated motion: yes. ADD-AUC: per-part object vertex distance (ADD) to the demonstrated object pose, integrated over a range of thresholds (area under the curve), so no single tolerance. Threshold range not found in the text read.
- Morphology comparison: yes, explicitly a purpose. Findings: larger, fully actuated hands score higher and learn faster. Allegro does well thanks to long fingers. Actuated DoF matter more than size. Less-actuated hands depart further from the human strategy.
- Physics: Genesis simulator. Hand mass, inertia and collision are estimated. No torque or force-limit analysis.

### ManipTrans and DexManipNet (Li et al. 2025)
Source: arXiv 2503.21860, via https://arxiv.org/html/2503.21860v1 [full text, first ~100k chars]. CVPR 2025 per https://openaccess.thecvf.com/content/CVPR2025/html/Li_ManipTrans_Efficient_Dexterous_Bimanual_Manipulation_Transfer_via_Residual_Learning_CVPR_2025_paper.html [metadata only].

What the paper says:
- Task source: human hand-object data from OakInk-V2 and FAVOR. GRAB and ARCTIC used qualitatively.
- Scoring against the demonstrated motion: yes, with fixed tolerances. A trajectory succeeds if object rotation error < 30°, translation error < 3 cm, mean joint error < 8 cm and mean fingertip error < 6 cm (errors averaged over frames). For bimanual tasks, either hand failing fails the whole trajectory.
- Hands: Inspire (12 DoF) main. Shadow, articulated MANO and Allegro tested, but only qualitatively. Allegro needs a relaxed fingertip tolerance (8 cm).
- Physics: actions clipped to joint limits. Simulated contact force is an observation, a reward term and a termination condition. No actuator torque limits stated.
- Output: DexManipNet, 3.3K episodes, 61 tasks, 1.34M frames, about 600 bimanual.

### CHORD: "Learning Dexterous Manipulation Using Contact Wrench Guidance From Human Demonstration" (Zhu et al. 2026)
Source: arXiv 2607.00033, abstract at https://arxiv.org/abs/2607.00033 [abstract]; body via https://arxiv.org/html/2607.00033v2 [full text, first ~100k chars]. Venue [UNVERIFIED].

What the paper says:
- Closest in spirit to our tool.
- From each demonstration frame, contacts (positions, normals in the object frame) are turned into a polyhedral friction-cone wrench set per object part. Its support function is sampled along fixed unit directions in wrench space.
- The robot's wrench support is compared with the human's per frame, with a relative tolerance, through an exponential kernel. Extra penalties for unintended and missed contacts. This is a reward for RL, not a pass/fail gate.
- How human contacts are obtained is not stated beyond tactile gloves or reconstruction.
- Benchmark: 4,739 bimanual tasks from ARCTIC, TACO, HOT3D, OakInk2, DexYCB, GRAB, H2O and in-house videos. Evaluated on 1,831.
- Scoring: a rollout fails if object position error exceeds 15 cm or rotation error exceeds 40°. "we consider a task success if the completion ratio is larger than 0.7".
- Hands: two Sharpa hands (real); G1 humanoid with three-finger Dex3 hands. Only cross-embodiment test is human five-finger to Dex3. No robot-hand-against-robot-hand comparison.
- Physics: wrist force/torque controller; feasibility left to the simulator. No reported actuator limit check.
- Includes non-force-closure phases (pushing, levering, sliding), stirring, pouring, handover.

### Bench2Dex (2026)
Source: arXiv 2609.15726, via https://arxiv.org/html/2609.15726v1 [full text, first ~100k chars]. Authors not shown in the part read [UNVERIFIED].

What the paper says:
- Tasks: 26 bimanual tasks (tool use, articulated objects, multi-stage). About 1.3K tele-operated demonstrations.
- Hands: 12 arm-hand pairs (e.g. Shadow, Allegro, LEAP, Ability, Schunk, Orca, Sharpa, Revo2, Wuji).
- Scoring: "stable success" — the end condition must hold for 0.5 s. Also stage completion rate and completion time.
- Morphology comparison: yes, per task-hand pair, with a shared tele-op retargeting and shared simulated tactile signal. Authors warn tasks and hands are not fully crossed, so hand effects are not isolated.
- Physics: no contact-force check. Safety flags are drops, high speeds and task violations.
- Task catalogue is in an appendix not read [UNVERIFIED].

### POMDAR: "A Benchmark of Dexterity for Anthropomorphic Robotic Hands" (Liconti et al. 2026)
Source: arXiv 2604.09294, via https://arxiv.org/html/2604.09294v1 [full text read].

What the paper says:
- Aim: formalise "dexterity as task performance across a structured set of manipulation and grasping motions".
- Tasks: 18. Twelve manipulation (scaffolded rods and rails, continuous rotation), six grasp-and-relocate (cylinder, sphere, disc).
- Derived from human taxonomies: Elliott and Connolly manipulation patterns, Ma and Dollar's extension, Feix GRASP taxonomy.
- Scoring: 0.8 × correctness + 0.2 × speed. Speed is human baseline time over system time (six-person study).
- Real and MuJoCo versions. Tele-operated.
- Hands: four ORCA variants (2, 3, 5 fingers without abduction, full 16 DoF). Results mix hand, interface and operator.
- Physics: no force or torque measures. Authors state it omits forceful or inertial behaviour (pushing, tossing, flipping).

### MyoSuite (Caggiano et al.)
Source: arXiv 2205.13600, via https://ar5iv.labs.arxiv.org/html/2205.13600 [full text read]. Venue [UNVERIFIED].

What the paper says:
- 204 tasks in nine families. Hand tasks include key turn, pen twirl, Baoding balls, pose, reach, hold.
- Model: musculoskeletal. MyoHand has 23 joints and 39 muscle-tendon units with Hill-type muscle dynamics.
- Physics: muscle force limits are built in. Variants add fatigue, sarcopenia (50% loss of maximal force), tendon transfer and exoskeleton assistance.
- Scoring: success rates. Key turn uses rotation thresholds (126° or 160°). Other tolerances not stated in the text read.
- Morphology: changes within one model (impairments, tendon transfer), not different hand designs.

---

## General manipulation suites

### ManiSkill (1)
Source: arXiv 2107.14483, via https://ar5iv.labs.arxiv.org/html/2107.14483 [full text, first ~100k chars].

What the paper says:
- Tasks: four. OpenCabinetDoor, OpenCabinetDrawer, PushChair, MoveBucket. 162 PartNet-Mobility objects.
- Robot: mobile base with one or two Panda arms (parallel grippers).
- Scoring: binary, with thresholds. Doors and drawers open to 90% of joint limit and static. Chair within 15 cm of target and upright. Bucket upright on the platform with balls inside. Mean success rate on held-out objects.
- Physics: a solvability check confirms object weight is within the gripper's lifting force. Not used in scoring.
- Grouping: by motion type (revolute, prismatic, planar, free).
- Morphology: none.

### ManiSkill2
Source: arXiv 2302.04659, via https://ar5iv.labs.arxiv.org/html/2302.04659 [full text, first ~100k chars].

What the paper says:
- 20 task families: soft-body (Fill, Hang, Excavate, Pour, Pinch, Write), peg-in-hole assembly, pick-and-place, mobile and articulated.
- Robot: Panda by default; single- and dual-arm mobile robots. No dexterous hands found.
- Scoring: explicit success conditions with tolerances. E.g. PickCube within 2.5 cm and robot static; AssemblingKits within 3 mm height, 4°, 2 cm; Write IoU > 0.8.
- Physics: velocity and spill limits in conditions. No force thresholds.
- Grouping: by task type (rigid or soft, stationary or mobile).

### ManiSkill3
Source: arXiv 2410.00425, via https://arxiv.org/html/2410.00425 [full text, first ~100k chars]. Dexterous tasks from https://maniskill.readthedocs.io/en/latest/tasks/dextrous/index.html [docs page].

What the sources say:
- Paper lists 12 task categories, including dexterous manipulation, humanoid and bimanual, soft body, vision-tactile, drawing and cleaning. 20+ robots.
- Docs list three dexterous tasks, each in five difficulty levels, all with dense rewards:
  - RotateValve (D'Claw): valve turned beyond π/2, π or 2π, by level.
  - RotateSingleObjectInHand (Allegro with tactile sensors): success condition marked "WIP" on the page.
  - TriFingerRotateCube (TriFinger Pro): cube within 0.02 m and 0.1 rad of goal.
- Each docs task card states success and fail conditions.
- No morphology comparison. No force checks found.

### RLBench (James et al.)
Source: arXiv 1909.12271, via https://ar5iv.labs.arxiv.org/html/1909.12271 [full text read]. Venue [UNVERIFIED].

What the paper says:
- 100 hand-designed tasks. Variations (objects, colours) with text descriptions.
- Robot: Panda arm with gripper, in V-REP/PyRep. Arm can be swapped in one line, but no comparison is reported.
- Scoring: sparse +1 on completion, decided by registered success conditions (e.g. object detected by a proximity sensor).
- Demonstrations: unlimited, via motion planning through waypoints.
- Physics: torques observable; no force-based success.

### Meta-World (Yu et al.)
Source: arXiv 1910.10897, via https://ar5iv.labs.arxiv.org/html/1910.10897 [full text, first ~100k chars]. Paper says it updates a CoRL 2019 manuscript.

What the paper says:
- 50 tabletop tasks, Sawyer arm with gripper, MuJoCo.
- Scoring: success when object-to-goal distance is below a threshold, e.g. 5 cm. Per-task thresholds in an appendix [UNVERIFIED].
- Evaluation modes: ML1, MT1, MT10, MT50, ML10, ML45.
- Single shared robot by design. No force checks.

### robosuite (Zhu et al.)
Source: arXiv 2009.12293, via https://ar5iv.labs.arxiv.org/html/2009.12293 [full text read]. The text read describes v1.5.

What the paper says:
- Nine benchmark tasks: Lift, Stack, Pick-and-Place, Nut Assembly, Door, Wipe, Two Arm Lift, Peg-in-Hole, Handover.
- Ten robots (arms, a humanoid GR1, Spot, TIAGo). Nine grippers including the Inspire dexterous hand.
- Scoring: shaped or sparse reward; task-completion metadata. Benchmark returns normalised to 500 per episode.
- Controllers output joint torques. Torque limits differ per robot (IIWA stronger, Jaco weaker) but are not used as a check.
- Benchmarks Panda and Sawyer on the same tasks. Not a morphology study.

### BEHAVIOR-1K (Li et al.)
Source: arXiv 2403.09227, via https://arxiv.org/html/2403.09227 [full text, first ~100k chars].

What the paper says:
- 1,000 household activities. 909 chosen by a survey of 1,461 people, plus 91 from BEHAVIOR-100. 50 scenes, 9,000+ objects.
- Grouped by what people want robots to do (e.g. 200+ cleaning and 200+ cooking activities).
- Scoring: goals written in a predicate logic (BDDL), checked against simulated object states. Success rate plus efficiency measures. A partial success score is in an appendix not read [UNVERIFIED].
- Physics: OmniGibson on PhysX 5 with cloth, fluids, deformables, temperature, wetness. But baselines use a grasp primitive that welds the object once all fingers touch. Fully physical grasping drops performance sharply.
- Robot: a mobile manipulator (Tiago in the real tests). No morphology comparison.

### LIBERO (Liu et al.)
Source: arXiv 2306.03310, via https://arxiv.org/html/2306.03310 [full text, first ~100k chars]. Venue [UNVERIFIED].

What the paper says:
- 130 language-conditioned tasks, on robosuite. Suites: LIBERO-Spatial, -Object, -Goal (10 each), LIBERO-100 (split into LIBERO-90 and LIBERO-Long).
- Scoring: success when all goal predicates hold. Success rate over 20 roll-outs per task. Transfer metrics built on top.
- 50 tele-operated demos per task.
- Grouped by the kind of knowledge transferred, not by application.

### CALVIN (Mees et al.)
Source: arXiv 2112.03227, via https://ar5iv.labs.arxiv.org/html/2112.03227 [full text read]. IEEE manuscript; DOI not checked [UNVERIFIED].

What the paper says:
- 34 tasks in four tabletop environments (A to D). Panda with parallel gripper, PyBullet.
- Scoring: a task oracle checks the required state change between first and last frames. Long-horizon test: 1,000 chains of five instructions.
- No force checks. No morphology comparison.

---

## Grasp-synthesis benchmarks

### DexGraspNet (Wang et al.)
Source: arXiv 2210.02697, via https://ar5iv.labs.arxiv.org/html/2210.02697 [full text read]. Venue [UNVERIFIED].

What the paper says:
- 1.32M Shadow Hand grasps on 5,355 objects.
- Synthesis: differentiable force-closure energy, plus penetration and joint-limit terms.
- Physics check in Isaac Gym: fingers pushed along contact normals; grasp passes if it holds under all six gravity directions for 100 steps with penetration under 0.1 cm. Benchmark use is looser (one direction, under 5 mm).
- Also reports the Q1 grasp-quality metric.
- MANO and Allegro shown only qualitatively.
- Grasps are static holds. Mostly power grasps.

### GenDexGrasp and MultiDex (Li et al.)
Source: arXiv 2210.00722, via https://ar5iv.labs.arxiv.org/html/2210.00722 [full text read]. ICRA 2023 per project page https://sites.google.com/view/gendexgrasp/ [metadata only].

What the paper says:
- "MultiDex contains 436,000 diverse grasping poses for 5 hands and 58 household objects".
- Hands: EZGripper, Barrett, Robotiq-3F, Allegro, Shadow.
- Physics check in Isaac Gym: 0.5 m/s² applied to the object along six axes for 1 s; fail if it moves more than 2 cm.
- Per-hand success reported (e.g. EZGripper about 39 to 43%, Barrett about 70%, Shadow about 77%). This is a grasp generator's success per hand, not framed as a hand-design comparison.

---

## Hand–object demonstration datasets (task sources)

None of these record contact forces. Contact, where given, is geometric.

### ARCTIC (Fan et al.)
Source: arXiv 2204.13662, via https://ar5iv.labs.arxiv.org/html/2204.13662 [full text, first ~100k chars]. Venue [UNVERIFIED].
- 339 bimanual sequences, 10 subjects, 11 articulated objects, about 2.1M images. MANO hands from motion capture; object 6D pose plus articulation angle.
- Contact is binary and geometric. No force.
- Vision benchmarks: motion reconstruction, interaction-field estimation.
- Used as a task source by DexMachina and CHORD (and qualitatively by ManipTrans).

### DexYCB (Chao et al.)
Source: arXiv 2104.04631, via https://ar5iv.labs.arxiv.org/html/2104.04631 [full text, first ~100k chars]. Venue [UNVERIFIED].
- 1,000 sequences, 10 subjects, 20 YCB objects, 582K frames. Grasp from table, hold, sometimes hand over.
- Benchmarks: detection, 6D pose, hand pose, and a human-to-robot handover grasp task (parallel gripper). No force data.
- Used as a task source by CHORD.

### HO-3D (Hampali et al.)
Source: arXiv 1907.01481, via https://ar5iv.labs.arxiv.org/html/1907.01481 [full text read]. Venue [UNVERIFIED].
- 77,558 frames, 68 sequences, 10 people, 10 YCB objects. Markerless annotation by joint optimisation over a sequence.
- Benchmark: hand pose from one RGB image. No force data.

### OakInk (Yang, Lu et al.)
Source: arXiv 2203.15709, via https://ar5iv.labs.arxiv.org/html/2203.15709 [full text read]. Full author list not shown [UNVERIFIED].
- 1,800 objects; 100 real objects recorded (230K frames); about 50K hand-object interactions.
- Intents: use, hold, lift-up, hand-out, receive.
- Physical plausibility measured (penetration, intersection volume, simulated displacement), not forces.
- Authors say transfer to robot hands is left for future work.
- OakInk2 is used by ManipTrans and CHORD.

---

## Our inference

- Nothing found is an "ImageNet for hands" in the sense wanted. Benchmarks either fix one hand and compare learning methods, or compare a few hands on a few tasks.
- Most suites score kinematic outcomes (pose within a tolerance, a predicate becoming true). Forces appear, if at all, as rewards, observations or termination signals.
- Dynamic content (by our reading, since papers rarely use the label):
  - Clearly dynamic: Bi-DexHands catching and throwing; TCDM hammering and shaking; MyoSuite pen twirl and Baoding balls.
  - Mostly quasi-static: DexArt, DexMV, Meta-World, RLBench, CALVIN, LIBERO, grasp benchmarks.
  - POMDAR says itself it omits inertial behaviour.
- Grouping by application exists in BEHAVIOR-1K (by human preference), ManiSkill3 (by category) and Bi-DexHands (by developmental motor skill). POMDAR groups by human manipulation taxonomy. Few group by end use.
- The demonstration datasets give object and hand motion and geometric contact, which is what a per-frame wrench check needs as input, but no measured forces to validate against.
- Our per-frame, per-hand, per-task verdict looks unclaimed. CHORD computes per-frame wrench-space support from demonstrations, but uses it as a training reward, not as a hand-capability verdict. TCDM's per-timestep 1 cm fraction is the nearest per-frame tolerance score.
- A per-frame wrench check could run on CHORD's 4,739 tasks or TCDM's 50 tasks, since both define tasks by demonstrated object trajectories.

---

## Short lists

Score against a demonstrated motion within a tolerance:
- TCDM: fraction of timesteps with object error < 1 cm.
- ManipTrans: rotation < 30°, translation < 3 cm, joint < 8 cm, fingertip < 6 cm.
- CHORD: fail beyond 15 cm or 40°; success if completion ratio > 0.7.
- DexMachina: ADD-AUC over a threshold range (no single tolerance).
- (DexMV uses human demos for training but scores against a goal, not the demo.)

Compare hand designs on the same tasks:
- DexMachina (six hands, explicit design findings).
- Bench2Dex (12 hands, but not fully crossed with tasks).
- POMDAR (four ORCA variants).
- GenDexGrasp (per-hand grasp success, five hands, not framed as a design study).
- Qualitative only: ManipTrans, DexGraspNet.

Check forces or actuator limits:
- MyoSuite: muscle force limits built in, with fatigue and weakness variants.
- DexGraspNet: holds under six gravity directions, penetration limit.
- GenDexGrasp: holds under 0.5 m/s² on six axes, 2 cm limit.
- ManipTrans: contact force as reward term and termination.
- CHORD: per-frame friction-cone wrench-space match, as a reward not a gate.
- ManiSkill (1): object weight vs gripper lifting force, as a solvability check only.
- None found checks per frame whether a hand's actuators can supply the wrench a demonstration needs.
