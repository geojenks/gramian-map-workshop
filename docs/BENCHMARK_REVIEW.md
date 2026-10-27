# Task sets for characterising hands: what exists, and what a workshop set would need

Written 27 October 2026. The question: is there a standard task set for
hands, "like ImageNet", that a hand could pass in full? Is any of it
physics-based, scored against a demonstrated motion, or dynamic? Are there
subsets by application, such as surgery, a general helper, or crafts? And
what would a workshop task set need?

## The result in brief

- **No such set exists.** We found no fixed, community-wide task set that
  hands are scored against and that covers in-hand or dynamic manipulation.
  The nearest shared convention is cube reorientation: count the goal
  orientations reached in a row, each within 0.4 rad. It compares
  controllers on one hand, and its authors warn against comparing hands
  with it (Handa et al. 2022).
- **The pieces exist separately.**
  - Physical hand tests (NIST, AHAP, POMDAR, Box and Blocks) compare hands,
    but score completion, time or final pose. They are almost all slow or
    static.
  - Simulation retargeting work (DexMachina, ManipTrans, TCDM) scores a
    hand against a demonstrated object motion within a tolerance. But the
    score is the outcome of a learned policy, so it mixes the hand with the
    controller. None checks actuator limits.
  - CHORD (Zhu et al. 2026) compares the wrenches a human's and a robot's
    contacts can apply at each frame of a demonstration. It uses this as a
    training reward, not as a verdict on a hand.
- **Subsets by application exist only loosely.** Surgery (FLS, JIGSAWS),
  daily living (Cybathlon, SHAP, AHAP, clinical tests) and manufacturing
  (NIST task boards, competition tracks) each have their own sets. They do
  not share a format. We found nothing for crafts.
- **The gap.** We found no set that gives a verdict per frame, per hand and
  per task on whether a hand can supply the wrench a demonstrated motion
  needs. That is what `follow.py` does. This is a negative finding from a
  limited search.
- **What our check would and would not add.** `follow.py` checks the hand
  statically against a load that includes the object's inertia. A task set
  built on it would test dynamic tasks for the load only. Finger inertia,
  actuator limits linked in time and contact changes are still missing
  (`LITERATURE_REVIEW.md`, section 8, "What a static check cannot see").
  So such a set should not be called a dynamic benchmark until those are in
  (issue 15 already asks for this in the talk).
  *27 October: finger inertia is now an option in `follow.py`, off by
  default (commit 027dd1a). Time-linked limits and contact changes are
  still missing.*

## How the sources were read

Three subagents searched in parallel on 27 October: physical task sets,
simulation benchmarks, and in-hand and dynamic work. Their raw notes are
kept in `docs/benchmark_notes/`. This document merges them. Where two
notes gave different read marks for one source, we keep the higher one.
Where they disagreed on a fact, we say so.

Marks, as in `LITERATURE_REVIEW.md`:

- **[full text read]**: the paper's body text was read, in full or on the
  pages named.
- **[abstract]**: only the abstract or a publisher summary.
- **[metadata only]**: a bibliographic record or a search snippet.
- **[UNVERIFIED]**: we could not reach any source for the claim.

Two cautions:

- Most web pages were read through a fetch tool that returns a summary of
  the page, not the page itself. So "full text read" means the whole text
  went through that summary. We do not quote from those reads.
- We re-checked these ourselves in this session: CHORD and DexMachina (body
  text, through the same tool), the contact-anchored retargeting paper
  (abstract), the NIST SP 1227 draft (that it exists), and Lin and Sun 2015
  (Crossref record).

Every section headed "Our inference" is our reading. Everything else
reports what a source says.

## Physical task sets

| Set | Tasks | How a hand is scored | Against | Dynamic | Hands compared | Mark |
|---|---|---|---|---|---|---|
| NIST SP 1227 draft (2018) | 10 test methods: finger and grasp strength, slip, cycle time, touch, grasp efficiency, force calibration and tracking, in-hand manipulation, pose estimation | in-hand: root-mean-square pose error of the object along a commanded sine path | a commanded path, not a demonstrated one | partly: sine paths at 0.12–0.20 Hz, cycle time | one hand in the worked example | [full text read, pp. 3–5, 55–61] |
| Falco et al. 2020, RA-L | grasp strength, cycle time, finger strength, repeatability | physical measurement | the test rig | no | not checked | [abstract] |
| ASTM F3756-26 | a split-force rig for strength, slip, efficiency, cycle-time tests | force measurement | the rig | no | none published | [abstract, scope only] |
| YCB set and protocols (Calli et al. 2015) | 73 objects; a template for protocols; examples such as pouring and table setting | outcome, with partial points advised | end state | no | each group publishes its own | [full text read, pp. 6–11] |
| Box and Blocks for robots (Morgan et al. 2020) | three protocols built on YCB items | block counts and times against human norms | human norms | no; throwing is banned | baselines on a Barrett WAM and a Franka Panda, against humans | [full text read, pp. 1–3] |
| AHAP (Llop-Harillo et al. 2019) | 26 tasks: 8 grasp types on 3 objects each, plus two others | 1, 0.5 or 0 per grasp and per hold, rated from video, as a percentage of a human's score | a posture shown by an operator | no; 3 s holds | yes: three ARMAR-6 hand versions, two KIT prosthetic versions | [full text read, pp. 3–6] |
| Cruciani et al. 2020, in-hand benchmark | goal hand poses and contact regions on YCB objects, at three levels | final position, orientation and contact-region error; grasp held 5 s | a goal state, not the path | no | public results store; authors say comparisons suit similar hardware | [full text read] |
| Elangovan et al. 2022 | 24 tasks in five groups (pick and place, reorientation, fine work, tools, puzzles) | success and time scores against the fastest of ten people | human baseline | no | one gripper tested | [full text read] |
| Elliott and Connolly benchmark (Coulson et al. 2021) | 13 in-hand patterns from a human taxonomy | qualitative and quantitative | human baseline | no | one hand (10 of 13 patterns) | [abstract] |
| FFP index (Sureshbabu et al. 2019) | form, features and a task series | combined index | — | no | yes | [full text read, p. 1] |
| RGMC 2016–2020 (Sun et al. 2021) | over 40 tasks; service, manufacturing, logistics tracks | points per task, within time limits | task outcome | no | team rankings, each with a whole robot | [full text read] |
| RGMC 2024 in-hand track | 10 waypoints in a 5 cm cube, 20 s each | summed position error | set target points | no; slow | competition results | [full text read, through a summary] |
| Cybathlon arm prosthesis race | 6 stations | points then time, 8 minutes | referees against rules | timed only | public race results | [full text read, selected pages] |
| Clinical tests: Box and Blocks, Jebsen–Taylor, SHAP, ARAT | 1, 7, 20 and 19 items | count or time against norms; ARAT rated 0–3 per item | human norms | speed only | rarely on robot hands | Box and Blocks and Jebsen–Taylor [abstract]; SHAP and ARAT [metadata only] |
| NIST task boards | Kimble et al. 2020, RA-L: three boards. Kimble et al. 2022, Frontiers: four boards | points for completion, time against people | outcome | no | not checked | 2020 [metadata only]; 2022 [abstract] |
| Surgery: FLS peg transfer; JIGSAWS | peg transfer; suturing, knot tying, needle passing | FLS: 300 less time less penalties. JIGSAWS: expert rating, 6–30 | outcome; expert | speed only | JIGSAWS rates surgeons, not hands | [metadata only] |
| Grasp taxonomies: Cutkosky 1989; Feix et al. 2016 | a hierarchy; 33 static grasp types | none: vocabularies that other sets draw on | — | no, by definition | — | Cutkosky [metadata only]; Feix [full text read, pp. 1–2] |

Notes on the table:

- The two NIST task-board papers are different papers. One note cited
  each. We list both.
- The NIST draft calls its in-hand method a kinematic measure of how well
  a hand controls an object's pose.

### Our inference (physical)

- The nearest physical match to "scored against a motion within a
  tolerance" is the NIST in-hand method. But the motion is commanded, not
  demonstrated, and only one hand has results.
- No physical set has throwing, catching, spinning or other tasks where
  the object's momentum matters.
- Most sets score a whole robot, with arm, vision and planning mixed in.
  AHAP, the NIST methods and the ASTM rig try to isolate the hand.

## Simulation benchmarks

| Set | Tasks | How it scores | Against a demonstration | Hands | Forces or limits | Mark |
|---|---|---|---|---|---|---|
| TCDM (Dasari et al.) | 50 object trajectories (40 from GRAB motion capture) | fraction of timesteps with object error under 1 cm | yes | three, assigned to tasks; no comparison | none; PD torque control | [full text read] |
| ManipTrans (Li et al. 2025) | human hand–object data; builds a 3.3K-episode set of 61 tasks | a trajectory passes if mean errors are under 30° and 3 cm (object), 8 cm (joints), 6 cm (fingertips) | yes | Inspire scored; Shadow, MANO and Allegro shown only qualitatively | contact force as reward and termination; no torque limits stated | [full text, first part] |
| DexMachina (Mandi et al. 2025) | 7 ARCTIC clips on 5 articulated objects | area under the success curve of object pose error over a range of thresholds | yes | six hands, compared as a design study | none; virtual object forces help in training and then decay to zero | [full text read]; re-checked |
| CHORD (Zhu et al. 2026) | 4,739 bimanual tasks from eight datasets plus videos; evaluated on 1,831 | fail beyond 15 cm or 40°; success if over 70% of the task is completed | yes | two Sharpa hands; Dex3 on a G1 humanoid | per-frame wrench-set match, as a reward; feasibility left to the simulator | [full text read]; re-checked |
| Bench2Dex (2026) | 26 bimanual tasks, about 1.3K tele-operated demos | condition held 0.5 s; stage completion; time | no | 12 arm–hand pairs; authors warn tasks and hands are not fully crossed | none | [full text, first part] |
| POMDAR (Liconti et al. 2026) | 18 tasks from human taxonomies | 0.8 × correctness + 0.2 × speed against a human | no; demos give only a time | four ORCA variants | none; authors say it omits forceful and inertial tasks | [full text read] |
| Bi-DexHands (Chen et al. 2022) | about 20 bimanual tasks, including catching and throwing | dense reward, normalised | no | Shadow only; compares learning methods | none on success | [full text, first part] |
| MyoSuite (Caggiano et al.) | 204 tasks; hand tasks include key turn, pen twirl, Baoding balls | success rates on thresholds | no | one muscle model, with impairment variants | muscle force limits built in | [full text read] |
| DexMV, DexArt | 7 and 4 task families | goal distance; binary success | no (DexMV trains from demos) | Adroit; Allegro | none | [full text read] |
| DexGraspNet; GenDexGrasp | 1.32M grasps; 436K grasps on 5 hands | grasp holds under disturbance in six directions | no | GenDexGrasp reports per-hand success, not as a design study | disturbance holds | [full text read] |
| ManiSkill 1–3, RLBench, Meta-World, robosuite, LIBERO, CALVIN, BEHAVIOR-1K | 4 to 1,000 tasks | success predicates or distance thresholds | no | mostly arms with grippers; few dexterous tasks | ManiSkill 1 checks lifting force only to confirm a task is solvable | [full text read] or [full text, first part] |

Demonstration datasets used as task sources (ARCTIC, DexYCB, HO-3D,
OakInk) give hand and object motion and geometric contact. None records
contact forces [full text read, or first part].

### Our inference (simulation)

- Benchmarks either fix one hand and compare learning methods, or compare
  a few hands on a few tasks.
- Forces appear, if at all, as rewards, observations or reasons to stop a
  run. None found checks, per frame, whether a hand's actuators can supply
  the wrench a demonstration needs.
- TCDM's per-timestep 1 cm fraction is the nearest per-frame tolerance
  score. CHORD is the nearest per-frame wrench comparison.

## In-hand and dynamic tasks

| Work | Task | How it scores | Compares | Mark |
|---|---|---|---|---|
| OpenAI et al. 2018 | cube reorientation, Shadow hand | goals reached in a row; ends on drop, 80 s timeout or 50 goals | training methods on one hand | [full text read, pp. 3–4, 10–11] |
| OpenAI et al. 2019, Rubik's cube | one-handed cube solving | success rate; drop or timeout fails | training methods | [abstract]; rates [metadata only] |
| Handa et al. 2022, DeXtreme | the same cube task, Allegro hand | goals in a row within 0.4 rad; they say OpenAI used the same tolerance | training methods; they call comparison with OpenAI confounded by the hand | [full text read] |
| Chen et al. 2021; Chen et al. 2023 | reorientation of many shapes; hand facing down | success within a tolerance, drop rate, time | controllers | [abstract]; the 2021 threshold [UNVERIFIED] |
| Qi et al. 2022 | continuous rotation, Allegro | radians rotated, time to fall, a torque term | controllers | [full text read, pp. 6–7] |
| Wang et al. 2024 | pen spinning | rotation; success if at least 180° on hardware | training choices | [full text read] |
| MyoChallenge 2022 | die reorientation, Baoding balls | Baoding: fraction of steps balls overlap targets | competition entries | [abstract]; metric [metadata only] |
| TossingBot; DexCatch; Kim et al. 2014 | throwing (a gripper); throw and catch with hands; catching in flight | landing or catch success | controllers | TossingBot [abstract]; others [metadata only] |
| KaRMA (Peticco and Agrawal 2026) | no tasks: rolling reach of a sphere in a pinch | three kinematic scores | 16 hands | [abstract] |
| Contact-anchored retargeting (Yang et al. 2026) | physics-refined human demos, with recovered contacts and forces, replayed on robot hands | success rates | four hands, not named in the abstract | [abstract]; re-checked |
| Lin and Sun 2015, IJRR 34:1195–1210 | grasp quality as coverage of a task wrench space recorded from human demonstrations | per grasp, to choose a grasp | Barrett and Shadow, in simulation | record [metadata only, Crossref]; content [metadata only, a repository snippet] |

### Our inference (dynamic)

- Each dynamic task is scored by its own paper's outcome rule: 180°,
  overlap fraction, a catch. There is no shared protocol.
- None scores whether the hand could supply the contact wrench over time.

## Three questions across all the sets

**Which score against a demonstrated motion within a tolerance?** All in
simulation:

- TCDM: per timestep, object error under 1 cm.
- ManipTrans: mean errors per trajectory under fixed limits.
- CHORD: fail limits, plus a completion threshold.
- DexMachina: an area over a range of thresholds, so no single tolerance.

Physically, AHAP scores against a shown posture by eye, and NIST against a
commanded path.

**Which compare hand designs on the same tasks?**

- DexMachina: six hands, with stated design findings. Actuated degrees of
  freedom mattered more than size, and less-actuated hands departed further
  from the human's strategy.
- Bench2Dex: 12 hands, not fully crossed with tasks.
- POMDAR: four variants of one hand.
- AHAP: five versions of two hands.
- KaRMA: 16 hands, with no tasks.
- GenDexGrasp: per-hand success, five hands, not framed as design.

**Which check forces or actuator limits?**

- MyoSuite: muscle force limits, with weakness and fatigue variants.
- DexGraspNet and GenDexGrasp: grasps must hold under set disturbances.
- ManipTrans: contact force as reward and as a reason to stop.
- CHORD: a per-frame wrench-set match, as a reward.
- ManiSkill 1: lifting force, only to show a task can be solved.
- Lin and Sun 2015: a demonstrated task wrench space, per grasp.
- The NIST and ASTM methods measure strength and slip directly, but not
  during a task.

## Grouping by application

- **Surgery:** FLS peg transfer and JIGSAWS. Both score people, by time or
  expert rating.
- **Daily living, the "general helper":** Cybathlon, SHAP, AHAP, Box and
  Blocks, Jebsen–Taylor, ARAT, and the RGMC service track. BEHAVIOR-1K
  chose 1,000 household activities by a survey of what people want robots
  to do [full text, first part].
- **Manufacturing:** the NIST task boards and the RGMC manufacturing track.
- **Crafts:** nothing found, in any of the three searches.

Other groupings, not by application: Bi-DexHands by stages of child motor
development, POMDAR and the Elliott and Connolly benchmark by human
manipulation taxonomies, ManiSkill by task type.

## Our inference: the gap

We order the closest precedents as the dynamic-task search did:

1. **DexMachina.** Six hands against demonstrated object motion. The
   nearest match. But simulation only, no wrench check, and the score
   depends on a learned policy.
2. **ManipTrans.** Fixed tolerances against the demonstration per
   trajectory. Hand comparison is secondary.
3. **CHORD.** Per-frame wrench sets from demonstrated contacts. The nearest
   in spirit to `follow.py`, but a training reward with no actuator limits,
   and no robot hand compared with another.
4. **Contact-anchored retargeting.** Recovers contact forces from human
   demos and replays them on four hands, scored by success.
5. **Lin and Sun 2015.** A task wrench space from demonstrations, but for
   static grasps.
6. **POMDAR, Elliott and Connolly, Elangovan, AHAP.** Fixed task lists that
   compare hands, scored by completion and time.
7. **The cube-reorientation convention.** Shared, but it compares
   controllers.

What none does: give a verdict per frame, per hand and per task on whether
the hand's actuators, friction and reach can supply the wrench a
demonstrated motion needs, independent of any learned controller. This is
also where the earlier review placed `follow.py` (section 8 of
`LITERATURE_REVIEW.md`), from a different search.

## Our inference: what a workshop task set would need

**Task format.**

- A task is a demonstrated object motion with its contacts: object pose
  over time, contact points and normals per frame, and the object's mass
  and inertia.
- Sources exist. TCDM's 50 trajectories and CHORD's task list are both
  defined this way. ARCTIC, GRAB, DexYCB and OakInk2 supply the motions and
  geometric contacts.
- None of these datasets measures contact forces. The wrench a task needs
  must be derived from the motion, as `tools/import_demo_leader.py` does.
  So there is no measured ground truth for it.

**Scoring.**

- Per frame: pass or fail, with the reason (reach, friction or torque), and
  the margins `follow.py` already gives.
- Per task: the fraction of frames passed, in the style of TCDM. Whether a
  task passes needs a rule we have not chosen, for example every frame, or
  a fraction with no gap longer than a set time.
- Per hand: tasks passed, overall and per application group.
- Because the check does not depend on a controller, two hands differ only
  by their own geometry, actuators and friction. This is the main thing it
  would add over DexMachina and ManipTrans.

**What it would not yet test.** The hand is checked statically against a
dynamic load. Tasks where finger inertia, actuator speed or contact changes
decide success are not tested yet. These include spinning, catching and
fast regrasps. The set should say so, and not claim to be a dynamic
benchmark until those are in. *27 October: finger inertia is now an
option in `follow.py`, off by default (commit 027dd1a); actuator speed
and contact changes are still not tested.*

**Grouping by application.**

- Tag each task with one or more groups (surgery, helper, manufacturing,
  crafts), rather than build separate sets. Report per group.
- Surgery and crafts have no hand–object motion data with contacts that we
  found. They would need new recordings, or tasks adapted from FLS-style
  drills.
- Existing clinical and competition sets give the task ideas for the
  helper and manufacturing groups. Their scoring would not carry over.

## Two in-hand taxonomies: Elliott and Connolly, and Bullock, Ma and Dollar

Added 27 October 2026. We were asked whether either taxonomy could serve
as the gait layer or the task layer of a workshop set, and whether anyone
scores hands against either. The terms, as they were put to us:

- A **task** is defined only by what the object does, with a tolerance on
  the object's motion.
- A **gait** is a named way of doing a task, with its own contact plan:
  cup gait, pinch roll, finger gaiting, regrasp, pushing against a
  surface, or a learned policy.
- A hand passes a task if any one gait passes our per-frame check. The
  number of gaits that pass measures redundancy.

### In brief

- **Neither taxonomy is a task layer.** Both classify what the hand does,
  not what the object must do. Bullock et al. chose this on purpose, and
  suggest pairing their tree with an object-centric classification.
- **Both fit the gait layer, at different grain.** Elliott and Connolly
  name individual digit movement patterns. Bullock's tree gives coarse
  codes for the hand's contact and motion, and the paper defines finger
  gaiting and regrasping.
- **Three groups score hands against them, each on few hands.** Coulson et
  al. (one hand, against Elliott and Connolly), Zhou et al. (one soft hand,
  50 tasks said to draw on Bullock and on Feix), and POMDAR (four versions
  of one hand, drawing on both). None spans as many hand versions as AHAP
  does, and none checks forces.
- **The main later pointer** is a 2026 review from the NSF HAND
  Engineering Research Center (Shilati et al.). It groups these as
  "taxonomy-inspired" benchmarks and argues for tasks that do not assume a
  human hand.

### Elliott and Connolly 1984

Elliott JM, Connolly KJ. "A classification of manipulative hand
movements". *Developmental Medicine & Child Neurology* 26(3):283–296,
1984. DOI 10.1111/j.1469-8749.1984.tb04445.x. **[abstract]**, from the
Crossref record; the body was not reached.

What the abstract says:

- It separates palmar grips, which hold an object still in the hand, from
  digital patterns, which move it. This is separate from power and
  precision.
- Manipulative movements fall in three classes: sequential patterns, and
  synergies split into simple and reciprocal ones. Each class holds
  several named patterns, which differ mainly in how many digits take part
  and how the thumb is used.
- It covers digit movements that manipulate an object within the hand. It
  excludes moving the whole hand with the wrist or arm while the object is
  held still.

What others report of it (not checked in the paper itself):

- Bullock and Dollar 2011, and Bullock et al. 2013, describe the classes
  the same way. Simple synergies move all digits as one unit, as in
  pinching or squeezing. In the second class the fingers move together and
  the thumb independently, for example rolling an object between thumb and
  index finger. Sequential patterns move digits independently in a
  repeatable order. Both papers call this middle class "complex" in one
  sentence and "reciprocal" in another. Both note a limit the authors
  themselves state: flexion and extension of single digits are not
  included. **[full text read]** of both Bullock papers.
- Shilati et al. 2026 and POMDAR give 13 patterns. POMDAR's text names
  some of them, among them dynamic tripod (writing with a pen), full roll
  and linear step. **[full text read]** of POMDAR's html and of the
  Shilati review's section on these benchmarks; the full list sits in a
  POMDAR figure we did not read.

### Bullock, Ma and Dollar 2013

Bullock IM, Ma RR, Dollar AM. "A Hand-Centric Classification of Human and
Robot Dexterous Manipulation". *IEEE Transactions on Haptics*
6(2):129–144, 2013. DOI 10.1109/TOH.2012.53. **[full text read]**:
introduction, related work, the taxonomy itself, its applications and the
section on regrasping and gaiting. The earlier conference version is
Bullock IM, Dollar AM, "Classifying human manipulation behavior", IEEE
ICORR 2011, DOI 10.1109/icorr.2011.5975408, **[full text read]** for its
related work.

What it classifies, and how:

- One hand movement at one instant, by five yes-or-no questions: is the
  hand in contact; is the contact prehensile; is the hand moving relative
  to the body; is there motion within the hand; is there motion at the
  contact, meaning the object slides, rolls or pivots relative to the
  contact frame. This gives 15 classes, numbered in its Table 2. That
  confirms POMDAR's account of 15 classes, which our earlier notes had
  marked as POMDAR's alone.
- It deliberately leaves out hand shape, so that it applies to any hand.
- The two classes for prehensile motion within the hand (with and without
  motion at the contact) are subdivided by the main axis of motion: three
  rotations and three translations in a frame fixed to the back of the
  hand. One of the twelve cells has no human example in its figure.
- Longer tasks are written as a sequence of classes over time, or as a sum
  of what different parts of the hand do at once.
- It sets out the two views with a bottle-cap example. The object-centric
  view says what the cap must do; the hand-centric view describes the many
  ways a hand can do it. It argues that object-centric classes do not map
  directly onto hand behaviour.
- It defines regrasping (letting go, for instance onto a table, and taking
  hold again) and finger gaiting (exchanging contacts while the hand keeps
  hold).

How the paper applies it:

- To one robot hand, a simple three-fingered design, as a worked example:
  which axes of motion are easy, which need two fingers to hold while the
  third moves, and which the kinematics rule out. This is a qualitative
  discussion, not a score.
- To three daily-living tasks filmed with a head-mounted camera, coded
  frame by frame. It proposes choosing a representative benchmark task set
  from such statistics, paired with an object classification. It does not
  build that set.

### Who scores hands against them

| Work | Draws on | Tasks | Hands scored | Forces | Read |
|---|---|---|---|---|---|
| Coulson, Li, Majidi and Pollard 2021, "The Elliott and Connolly Benchmark" (IEEE-RAS Humanoids), DOI 10.1109/humanoids47582.2021.9555798 | Elliott and Connolly | 13 patterns; Shilati et al. and POMDAR say mostly with YCB objects | one (CMU Foam Hand III, 10 of 13 patterns), with a human baseline | no | [abstract]; Shilati et al. add that rotation and translation are normalised by finger length |
| Zhou et al. 2020, "50 Benchmarks for Anthropomorphic Hand Function-based Dexterity Classification and Kinematics-based Hand Design" (IEEE/RSJ IROS), DOI 10.1109/IROS45743.2020.9340982 | Feix's grasps; Bullock's categories, according to Shilati et al. | 50; POMDAR says the in-hand ones move along Cartesian axes | one (a soft hand with 22 degrees of freedom) | no | [abstract] |
| POMDAR (Liconti et al. 2026), arXiv 2604.09294 | Elliott and Connolly's patterns, Ma and Dollar's pivoting, Feix's grasps; Bullock for the overall split | 12 manipulation and 6 grasping tasks | four versions of the ORCA hand | no | [full text read] |

Shilati et al. 2026, "Benchmarking Dexterity of Multifingered Robot Hands:
A Review and Perspective", arXiv 2609.05585 (NSF HAND Engineering Research
Center), **[full text read]** for its section on taxonomy-inspired
benchmarks. It places all three works above in that group. It notes that
such benchmarks inherit assumptions about a human hand, and sets them
against task-centric ones (clinical tests, task boards, simulation
suites). It proposes three levels of benchmark: component, hand and
system.

### Our inference

- **Task layer.** Neither taxonomy defines a task by what the object does
  with a tolerance. Bullock's axes of motion come closest, but they are
  measured in the hand's frame, not the world's or the object's, and they
  give a direction, not a target. Our task layer would still come from
  demonstrated object motions, as proposed above.
- **Gait layer.** Both fit, and they can be stacked. Bullock's codes say
  what kind of gait it is: prehensile or not, motion at the contact or
  not, hand still or moving. Of the gaits we were given, finger gaiting
  and regrasp are defined in Bullock; pushing against a surface falls in
  its non-prehensile classes; pinch roll is a prehensile gait with motion
  at the contact. Elliott and Connolly would name the finer pattern within
  a class, for human-like hands only. Neither covers a learned policy as a
  gait; that would be tagged by its contact behaviour after the fact.
- **POMDAR already moved from gait to task.** It merged several Elliott
  and Connolly patterns into one task scored by object outcome, so as not
  to penalise hands with fewer fingers or unusual layouts. That is the
  move our structure makes explicit: score the task by the object, and
  count the gaits separately.
- **Redundancy.** None of the three scoring works counts how many ways a
  hand can do a task. Each asks whether the hand can do the named pattern.
  Counting the gaits that pass would be new, as far as this search goes.
- **The AHAP analogue.** Coulson, Zhou and POMDAR are the nearest
  analogues to AHAP for these taxonomies. Each scores one hand or one hand
  family, by success and time or by range of motion. None checks forces or
  actuator limits.

## References and claims we could not verify

- Cutkosky 1989: the DOI (10.1109/70.34763, from a lecture slide) and the
  number of grasp types.
- SHAP: the total of 26 tasks seen in a secondary table, and its combined
  function score.
- The FLS source paper (Fried et al. 2004) and the ring count.
- JIGSAWS: that the robot was a da Vinci, from search snippets only.
- The first author of the Amazon Picking Challenge analysis.
- Chen, Xu and Agrawal 2021: the success threshold. The earlier review
  read its appendices for joint torques, not for this.
- GenHand (npj-series journal, 2026): a snippet says it retargets grasps
  under force equilibrium and reports wrench residuals across four
  grippers. Behind a login.
- Borst, Fischer and Hirzinger 2004, on choosing a task wrench space: no
  record reached.
- Lin and Sun 2015: the publisher page refused access and the abstract is
  withheld from open indexes, so its content rests on a repository snippet.
- Venues for DexMachina, Bi-DexHands, MyoSuite, DexGraspNet, Bench2Dex and
  CHORD.
- The task lists of Bi-DexHands and Bench2Dex past the parts read.
- Elliott and Connolly 1984: the 13 pattern names and their content, read
  only through the abstract and through Bullock, POMDAR and Shilati et al.
- Zhou et al. 2020: that its in-hand tasks use Bullock's categories. This
  is Shilati et al.'s account; Zhou's abstract does not say so.
- Bullock, Feix and Dollar 2014 (dexterous workspace of two- and
  three-fingered precision manipulation, DOI 10.1109/haptics.2014.6775431)
  and Roa et al. 2014 (functional evaluation of manipulation in dexterous
  hand design, DOI 10.1109/ICRA.2014.6907863): both cite Bullock et al.
  2013; no abstract reached, so we say nothing of their content.
