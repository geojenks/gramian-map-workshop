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
| Elliott and Connolly benchmark (Coulson et al. 2021) | 13 in-hand patterns from a human taxonomy | qualitative and quantitative | human baseline | no | one hand (10 of 13 patterns) | [metadata only] |
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
benchmark until those are in.

**Grouping by application.**

- Tag each task with one or more groups (surgery, helper, manufacturing,
  crafts), rather than build separate sets. Report per group.
- Surgery and crafts have no hand–object motion data with contacts that we
  found. They would need new recordings, or tasks adapted from FLS-style
  drills.
- Existing clinical and competition sets give the task ideas for the
  helper and manufacturing groups. Their scoring would not carry over.

## References and claims we could not verify

- Cutkosky 1989: the DOI (10.1109/70.34763, from a lecture slide) and the
  number of grasp types.
- SHAP: the total of 26 tasks seen in a secondary table, and its combined
  function score.
- The FLS source paper (Fried et al. 2004) and the ring count.
- JIGSAWS: that the robot was a da Vinci, from search snippets only.
- The first author of the Amazon Picking Challenge analysis.
- Chen, Xu and Agrawal 2021: the success threshold.
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
