# Benchmarks for in-hand and dynamic manipulation by robot hands

Literature notes, 27 Oct 2026. Question: is there a fixed task set (an
"ImageNet for manipulation") that hands are tested against, with subsets by
application, and how are hands scored on it? Context: a tool that checks,
frame by frame, whether a hand can supply the wrench a recorded demonstration
needs.

Source labels:
- [full text read]: I read body text (pages or sections named where partial).
- [abstract]: I read the arXiv/publisher abstract page only.
- [metadata only]: I only saw a search-result snippet or a bibliographic
  record; the source of the snippet is named.
- [UNVERIFIED]: I could not reach the source at all.

For each source: tasks; scoring; scoring type (physics-based / against a
demonstrated motion within a tolerance / binary success); hand designs or
controllers compared.

---

## 1. In-hand reorientation as a de facto benchmark

### OpenAI et al. 2018, "Learning Dexterous In-Hand Manipulation"
arXiv:1808.00177 [full text read, pp. 3-4 and 10-11]
- Task: reorient a block (and an octagonal prism) on the palm of a Shadow
  hand to a random target orientation. A new goal is issued as soon as the
  current one is reached, until the object is dropped.
- Scoring: number of consecutive successful rotations. A trial ends when the
  object is dropped, a goal is not reached in 80 s, or 50 rotations are
  reached. Reported as mean, median and sorted individual trials (10 trials
  per policy on hardware, 100 in simulation). E.g. block, state-based, real
  robot: median 13.
- The angular tolerance is not on the pages I read. DextrEme (below) states
  that OpenAI's evaluation used 0.4 rad.
- Type: binary success per goal within an orientation tolerance, counted
  consecutively. Not physics-based, not against a demonstration.
- Compares: training randomisations (controllers) on one fixed hand.

### OpenAI et al. 2019, "Solving Rubik's Cube with a Robot Hand"
arXiv:1910.07113 [abstract]
- Task: solve a Rubik's cube one-handed (Shadow hand), plus block
  reorientation.
- Scoring: the abstract gives no metric. OpenAI's blog post (via search
  snippet, https://openai.com/index/solving-rubiks-cube) reports 20 % success
  on maximally hard scrambles (26 face rotations) and 60 % on 15-rotation
  scrambles; a drop or timeout counts as failure. [metadata only]
- DextrEme (below) quotes OpenAI ADR policies on block reorientation scored
  by the same consecutive-successes count.
- Type: binary success. Compares controllers (training methods).

### Handa et al. 2022, "DeXtreme"
arXiv:2210.13702 [full text read, html]
- Task: the OpenAI cube reorientation task on an Allegro hand.
- Scoring: consecutive target orientations reached without dropping. Success
  tolerance 0.4 rad at test (0.1 rad in training). Run ends if stuck more
  than 80 s. No cap on the main count, but they also report a 50-capped
  statistic to compare with OpenAI. Optional "hold" variant: stay within
  tolerance for 10 frames before the goal refreshes. Best-average ADR policy:
  27.8 +/- 19.0 average, median 14, best 112.
- The authors warn that comparison with OpenAI is not like-for-like: a
  four-finger Allegro with locked wrist versus a five-finger Shadow hand.
- Type: binary per goal within tolerance, counted. Compares controllers;
  cross-hand comparison noted as a confound, not a goal.

### Chen, Xu, Agrawal 2021, "A System for General In-Hand Object Re-Orientation"
arXiv:2111.03043 [abstract] (CoRL 2021, per arXiv page)
- Task: reorient over 2,000 object shapes, hand facing up and down
  (simulation).
- Scoring: the abstract gives none. I could not verify the success threshold
  for this paper (the PDF was too large to fetch). [UNVERIFIED for metric]
- Compares controllers on a fixed simulated hand.

### Chen et al. 2023, "Visual Dexterity"
arXiv:2211.11744 [abstract] (Science Robotics 2023, per arXiv page)
- Task: reorient novel, complex shapes with a downward-facing hand, depth
  camera input, real hardware.
- Scoring (abstract): median reorientation time about 7 s; on an unseen duck,
  56 % of trials dropped; of the rest, 75 % ended within 0.4 rad of target.
- A search snippet attributed to this paper further reports 72.3 % success at
  0.4 rad and 25.9 % at 0.1 rad in simulation. I did not open the full text.
  [metadata only, from search over arxiv.org/pdf/2211.11744]
- Type: binary success within an orientation tolerance, plus drop rate and
  time. Compares controllers.

### Qi et al. 2022, "In-Hand Object Rotation via Rapid Motor Adaptation"
arXiv:2210.04887 [full text read, pp. 6-7] (CoRL 2022)
- Task: rotate objects continuously about the z axis with fingertips only
  (Allegro). Real-world object sets: heavy objects, irregular objects.
- Scoring: total rotated angle in radians ("Rotations"), time to fall (TTF),
  and a torque term as energy efficiency. 30 s maximum episode; 20 initial
  grasps by 6 objects per set.
- Type: continuous performance measures; the torque term is the only
  physics-flavoured one. No demonstration. Compares controllers on one hand.

### Our inference (reorientation)
- The shared metric across OpenAI 2018/2019 and DextrEme is "consecutive goals
  reached within 0.4 rad, ending on drop or 80 s timeout, often capped at 50".
  Chen and Qi use different metrics (success rate within a tolerance; radians
  rotated and time to fall). So the "benchmark" is a task convention, not a
  fixed protocol.
- All of these compare controllers on a fixed hand. The one explicit cross-hand
  comparison (DextrEme v OpenAI) is flagged by its authors as confounded.
- None scores against a demonstrated motion, and none checks force or wrench
  feasibility.

---

## 2. Pen spinning, baoding balls, finger gaiting, regrasping

### Wang et al. 2024, "Lessons from Learning to Spin 'Pens'"
arXiv:2407.18902 [full text read, html] (CoRL 2024)
- Task: spin pen-like objects (over ten objects) by finger gaiting.
- Scoring: in simulation, cumulative rotation reward and duration (s). On
  hardware, radians rotated and success rate, where success means rotating at
  least 180 degrees. A 2-pi threshold is used only to filter replay
  trajectories.
- Type: binary success on an angle threshold plus continuous rotation.
  Compares controllers / training choices on one hand.

### Nagabandi et al. 2019, "Deep Dynamics Models for Learning Dexterous Manipulation" (PDDM)
arXiv:1909.11652 [abstract]
- Tasks (abstract): in-hand manipulation, finger gaits, handwriting-like fine
  skills, coordinating multiple free-floating objects with a 24-DoF hand. The
  abstract does not name baoding balls; I did not open the full text to
  confirm. Metrics not stated in abstract.

### Caggiano et al. 2023, "MyoChallenge 2022"
PMLR 220:233-250, https://proceedings.mlr.press/v220/caggiano23a.html [abstract]
- Tasks: die reorientation and baoding balls (two spheres rotating about each
  other) on a simulated musculoskeletal hand (MyoSuite). Phase 2 tests
  generalisation over object and task variations.
- Scoring: not stated on the abstract page. A secondary source (bioRxiv
  10.1101/2024.01.24.577123, via search snippet) says baoding performance was
  the fraction of time steps in which balls overlap their targets.
  [metadata only]
- Compares controllers (competition entries) on one fixed hand model.

### Khandate, Haas-Heger, Ciocarlie 2021, finger gaiting with intrinsic sensing
arXiv:2109.12720 [abstract] (ICRA 2022)
- Task: large-angle reorientation by finger gaiting using precision grasps,
  proprioception and touch only.
- Scoring: abstract compares sample complexity and robustness; metric not
  stated. Controllers compared.

### Furukawa, Namiki, Senoo, Ishikawa 2006, dynamic regrasping
ICRA 2006, pp. 181-187 [metadata only, JSPS grant summary
https://www.jsps.go.jp/j-grantsinaid/23_startup_s/ichiran21/e-data/e01_namiki.pdf
via search]
- High-speed three-finger hand with 1 kHz vision; dynamic regrasping (toss
  and re-catch within the hand). No benchmark or scoring found.

### Our inference (spinning, gaiting, regrasping)
- These tasks recur, but each paper defines its own success (180 degrees,
  2 pi, target overlap fraction). There is no shared protocol.
- Dynamic in-hand tasks (baoding, regrasping, spinning) are scored by
  outcome, never by whether the hand could supply the required contact
  wrench over time.

---

## 3. Throwing and catching

### Zeng et al. 2019/2020, "TossingBot"
arXiv:1903.11239 [abstract] (arXiv page names no venue)
- Task: pick arbitrary objects from a bin and throw them into target boxes,
  some out of reach. Arm with a simple gripper, not a multi-finger hand.
- Scoring: throwing accuracy about 85 %; over 500 mean picks per hour.
- Type: binary landing success. Uses physics (a ballistic model) inside the
  controller, with learned residuals; not physics-based scoring.
- Compares controllers.

### Kim, Shukla, Billard 2014, "Catching Objects in Flight"
IEEE T-RO, DOI 10.1109/TRO.2014.2316022 [metadata only, EPFL infoscience
record via search]
- Task: catch irregular objects (ball, bottle, racket) with an arm and a
  four-finger hand; learned from demonstrated throws. iCub in simulation and a
  real arm. Scoring not seen.

### Lan et al., "DexCatch"
arXiv:2310.08809 [metadata only, search snippet] (CoRL, PMLR 270)
- Throw-and-catch with dexterous hands in simulation; 73 % success across 45
  scenarios. Binary success. Controllers compared.

### Chen et al. 2022, "Bi-DexHands"
arXiv:2206.08686 [abstract]
- Simulator with dozens of bimanual dexterous tasks, including catching a
  flying object, bottle opening, lifting a pot, stacking blocks. Tasks are
  pitched at levels of human motor development. Used to benchmark RL
  algorithms; metric not stated in abstract. Controllers compared.

### DexH2R
arXiv:2506.23152 [metadata only, search snippet]
- Described as a benchmark for dynamic dexterous grasping in human-to-robot
  handover. Not opened.

### Our inference (throwing, catching)
- Throwing and catching are scored as binary outcomes. The well-known
  benchmark-like work (TossingBot) uses a gripper, not a hand. No dynamic
  catching benchmark compares hand designs.

---

## 4. Dexterity definitions, taxonomies and hand-evaluation papers

### Ma & Dollar 2011, "On Dexterity and Dexterous Manipulation"
ICAR 2011, Yale PDF https://www.eng.yale.edu/grablab/pubs/Ma_ICAR2011.pdf
[full text read, pp. 1-3 only]
- A position paper. Collects definitions of dexterity (Bicchi 2000, Li 1989,
  Okamura 2000, Klein/Blaho 1987, Sturges 1990). Argues the field lacks a
  formal account of why in-hand dexterity is needed versus a redundant arm
  with a simple gripper. Cites Wright et al.'s idea of mapping manipulators
  to tasks via a dexterity spectrum, and decomposing tasks into primitives
  (finger gaiting, turning a grasped object, redistributing fingertip
  forces). It reviews in-hand manipulation classes. On pages 1-3 it proposes
  no benchmark or scoring.

### Bullock, Ma, Dollar 2013, "A Hand-Centric Classification of Human and Robot Dexterous Manipulation"
IEEE Trans. Haptics, Yale PDF
https://www.eng.yale.edu/grablab/pubs/Bullock_TOH2013_1.pdf [metadata only,
search snippet of that PDF]
- A hand-centric, motion-centric classification, applied to robot hand
  design and to three activities of daily living. POMDAR (below) describes it
  as five binary descriptors giving 15 classes (that description is POMDAR's,
  not read in Bullock's text).

### Bicchi 2000, "Hands for Dextrous Manipulation and Robust Grasping: a Difficult Road Towards Simplicity"
IEEE T-RA 16(6):652-662, IEEE Xplore doc 897777 [metadata only, Pisa
repository snippet]
- Survey of hand design around dexterity, grasp robustness and human
  operability; argues for minimal task-specific designs. Not a benchmark.

### Cutkosky 1989, grasp taxonomy for manufacturing
IEEE T-RA 5(3):269-279, DOI 10.1109/70.34763 [metadata only, from a lecture
slide deck via search]
- Grasp taxonomy from observing machinists. A secondary source says hand
  designers use it as a dexterity benchmark (count of grasps reproduced).
  The specific counts in that snippet are [UNVERIFIED].

### Feix et al. 2016, "The GRASP Taxonomy of Human Grasp Types"
IEEE THMS 46(1):66-77, DOI 10.1109/THMS.2015.2470657 [metadata only, search
snippet of IEEE Xplore and Yale]
- 33 static, stable, one-handed grasp types (17 if shape and size ignored).
  Static only: no in-hand motion.

### Lin & Sun 2015, "Grasp planning to maximize task coverage"
IJRR 34(9):1195-1210, DOI 10.1177/0278364915583880 [metadata only, USF
Digital Commons record via search]
- Task wrench space modelled as a non-parametric distribution of
  disturbances recorded during human demonstrations. Grasp quality is how
  well the grasp covers that distribution. Tested in simulation with Barrett
  and Shadow hands and on a real robot.
- Type: physics-based (wrench space) and demonstration-derived. But per grasp,
  not per frame of a motion, and aimed at choosing grasps, not ranking hands.

### Borst, Fischer, Hirzinger 2004, "Grasp Planning: How to Choose a Suitable Task Wrench Space"
ICRA 2004, pp. 319-325 [UNVERIFIED: I saw a DLR elib record via search but did not
record which record URL; no DOI reached]
- Classic task wrench space versus grasp wrench space fitting. Content not
  read.

### Peticco & Agrawal 2026, "KaRMA"
arXiv:2605.15548 [abstract] (IROS 2026 per arXiv)
- Kinematic metric: how far a sphere can be translated and reoriented within
  a two-finger pinch by feasible rolling, with joint limits, collisions and
  antipodal force feasibility. Three scores (translation, rotation, grasp
  sensitivity). Evaluated on 16 robot hands.
- Type: model-based, hand-comparing, task-free. Not against a demonstration.

### Our inference (definitions and taxonomies)
- Taxonomies (Cutkosky, Feix, Bullock, Elliott & Connolly) are the usual
  source of benchmark task lists. Cutkosky and Feix are static grasp lists.
- The only demonstration-derived wrench work found (Lin & Sun) works per
  grasp, for grasp choice, not per frame for hand comparison.

---

## 5. Benchmarks that compare hand designs on fixed task sets

### Liconti et al. 2026, "A Benchmark of Dexterity for Anthropomorphic Robotic Hands" (POMDAR)
arXiv:2604.09294 [full text read, html]
- Tasks: 18. Twelve manipulation tasks from Elliott & Connolly's 13 patterns
  plus Ma & Dollar finger pivoting, in vertical, horizontal and
  continuous-rotation configurations held by 3D-printed scaffolds; six pure
  grasping tasks grouped from Feix's types.
- Scoring: 0.8 x correctness + 0.2 x speed. Correctness = fraction of notches
  cleared or rotation normalised to 360 degrees; grasping 0 / 0.5 / 1. Speed =
  human baseline time over robot time, unbounded.
- Human demonstrations (motion-capture glove) give only the time baseline.
  Robots are teleoperated. No force or wrench measured; force-heavy dynamics
  listed as a limitation.
- Compares hand designs: four embodiments of the ORCA hand (2 to 16 DoF).
- Grouping is by mechanical configuration, not by application.

### Coulson, Li, Majidi, Pollard 2021, "The Elliott and Connolly Benchmark"
IEEE-RAS Humanoids, pp. 238-245 [metadata only, NSF PAR record
https://par.nsf.gov/biblio/10293197 seen via search; the page itself refused
connection]
- 13 in-hand manipulation patterns from a human taxonomy; qualitative and
  quantitative metrics; tested the CMU Foam Hand III (10 of 13 patterns).
  Compares hands to a human baseline.

### Elangovan, Chang, Gao, Liarokapis 2022, open-source dexterity test
Frontiers in Robotics and AI 9:808154, DOI 10.3389/frobt.2022.808154
[full text read, PMC html]
- 24 tasks in five groups: pick and place, reorientation, fine manipulation
  (threaded pins, washers, nuts), tool tasks on a rotating rig, puzzles.
- Scoring: success score Ss = points / (4 x 24); time score
  St = log(Tmin)/log(Ttotal) against the fastest human; total
  (ws Ss + wt St) x 100, default equal weights.
- Human baseline: ten subjects. Only one robot device tested (a parallel-jaw
  gripper). Designed to compare systems, including hands.

### Llop-Harillo et al. 2019, "The Anthropomorphic Hand Assessment Protocol (AHAP)"
Robotics and Autonomous Systems 121:103259, DOI 10.1016/j.robot.2019.103259
[metadata only, UJI repository via search]
- 26 postures/tasks with 25 YCB objects; Grasping Ability Score. Five hand
  variants compared (ARMAR-6 hand configurations, KIT prosthetic hand).
  Scored by raters from video.

### Sureshbabu, Metta, Parmiggiani 2019, "A Systematic Approach to Evaluating and Benchmarking Robotic Hands - The FFP Index"
Robotics (MDPI) 8(1):7, DOI 10.3390/robotics8010007 [full text read, p. 1
only]
- Scores hands on Form (anthropomorphism), Features (sensing and actuation)
  and Performance (a task series), with price as context. Compares hands.
  Mentions Biagiotti et al. 2004 indices of anthropomorphism and dexterity,
  and SHAP.

### SHAP (Southampton Hand Assessment Procedure)
https://shap.ecs.soton.ac.uk/about.php [metadata only, search snippet]
- 6 abstract objects and 14 activities of daily living, self-timed. Built for
  prostheses. I found no robot-hand study using it directly.

### Kapandji test on robot hands
[metadata only, search snippet referring to DexLink Hand, arXiv:2606.17418]
- Thumb opposition score 0 to 10, reused by hand designers. Static, kinematic.

### Mandi et al. 2025, "DexMachina: Functional Retargeting for Bimanual Dexterous Manipulation"
arXiv:2505.24853 [full text read, html] (ICML 2026 per NVIDIA page)
- Tasks: long-horizon bimanual manipulation of articulated objects, from
  7 ARCTIC human demonstration clips on 5 objects.
- Scoring: ADD-AUC on object pose tracking against the demonstrated object
  trajectory (ADD per articulated part, area under the success curve over a
  sweep of thresholds). 5 seeds, 20 episodes per checkpoint. Simulation
  only (Genesis); demos cleaned of penetrations; contacts approximated by a
  distance threshold. No explicit force or wrench feasibility check; during
  training virtual object forces help, then decay to zero.
- Compares hand designs: Inspire, Allegro, XHand, Schunk; plus Ability and
  DexRobot in the hardware study. Findings: more actuated, larger hands do
  better; DoF mattered more than size; less-actuated hands deviate more from
  the human strategy. Authors caution results are limited to these tasks.

### Li et al. 2025, "ManipTrans"
arXiv:2503.21860 [full text read, html] (CVPR 2025)
- Tasks: transfer of human bimanual hand-object demonstrations to robot
  hands; builds the DexManipNet dataset (3.3K episodes, e.g. pen capping,
  bottle unscrewing).
- Scoring: per-frame errors against the demonstration, averaged per
  trajectory: object rotation < 30 degrees, object translation < 3 cm, joint
  position < 8 cm, fingertip position < 6 cm. All must pass; either hand
  failing fails a bimanual trajectory.
- Hands: Inspire (main), Shadow, MANO, Allegro. Physics simulation.
- Type: tolerance against a demonstrated motion, inside a physics simulator.
  Hand comparison is secondary (generality of the method).

### Zhu et al. 2026, "CHORD" (contact wrench guidance)
arXiv:2607.00033 [abstract]
- Compares human and robot motion by the object wrench (forces and torques
  induced on the object) as an RL guide. Benchmark of 4,739 bimanual tasks
  from motion capture and videos; 82.12 % average success on 1,831. Hands
  used and any hand comparison not stated in the abstract.

### Yang et al. 2026, contact-anchored retargeting
arXiv:2609.24093 [abstract]
- Physics refinement of MANO demos to recover contacts and forces, then
  contact-anchored retargeting and residual RL. Transfers to four distinct
  robot hands (not named in abstract). Success rates only.

### GenHand (Nature npj-series journal, 2026)
https://www.nature.com/articles/s44182-026-00076-1 [UNVERIFIED: login wall]
- A search snippet says it retargets human grasps under force equilibrium
  and reports net wrench residuals across four grippers. Not confirmed.

### Our inference (hand comparison)
- Physical hand benchmarks (POMDAR, Elliott and Connolly, Elangovan, AHAP, FFP) compare hands
  on fixed task lists, scored by completion and time against a human. They do
  not use a demonstrated trajectory as the reference, nor wrench checks, and
  are mostly quasi-static.
- Simulation retargeting work (DexMachina, ManipTrans) scores hands against a
  demonstrated object trajectory within tolerances. But the verdict is an
  outcome of a learned policy, so it mixes hand capability with policy
  quality. Neither reports a per-frame wrench feasibility verdict.

---

## 6. Benchmarking initiatives in robotics generally

### Cruciani et al. 2020, "Benchmarking In-Hand Manipulation"
IEEE RA-L, arXiv:2001.03070 [full text read, ar5iv]
- Tasks: reach goal grasps on YCB objects. Level I: goal hand pose; Level II:
  goal contact regions; Level III: both.
- Scoring: final-state position error, normalised position error, quaternion
  orientation error; contact-region error by Euclidean or geodesic mesh
  distance. Grasp must hold for 5 s; 5 runs per set; time and failures
  recorded.
- Type: tolerance against a goal state (final configuration only, not the
  path). Not physics-based.
- Platform-agnostic, but demos compare planners/controllers on fixed hardware.
  Authors say comparisons are most meaningful between similar hardware and
  suggest weighing hardware complexity against accuracy.

### Calli et al. 2015, YCB Object and Model Set
IEEE RAM 22(3):36-52, arXiv:1502.03143 [abstract]
- Object set and example task protocols for evaluating planning, learning,
  mechanical design and control; aimed also at prosthetics and
  rehabilitation. Protocols not read.

### IEEE RAM special issue on replicable and measurable robotics research (Sept 2015)
- Bonsignorio & del Pobil, "Toward Replicable and Measurable Robotics
  Research", RAM 22(3):32-35 [metadata only, UC3M portal]. Content not read.
- Falco, Van Wyk, Liu, Carpin 2015, "Grasping the Performance", RAM, NIST page
  https://www.nist.gov/node/1088606 [abstract]. Test methods for hand
  hardware and control, demonstrated on lab hands; intended to match hand
  capability to end-user needs. The NIST page names this as part of a RAM
  special issue on replicable robotics research.

### NIST hand test methods
NIST SP 1227 draft, "Performance Metrics and Test Methods for Robotic Hands"
(2018), https://nvlpubs.nist.gov/nistpubs/SpecialPublications/NIST.SP.1227-draft.pdf
[metadata only, search snippet]
- Hand traits plus task-level performance measures; a later RA-L paper
  reportedly covers grasp strength, cycle time, finger strength, finger
  repeatability. Compares hand hardware. Quasi-static.

### Kimble et al. 2020, NIST assembly task boards
IEEE RA-L 5(2), DOI 10.1109/LRA.2020.2965869 [metadata only, NIST pages via
search]
- Three task boards: pegs, gears, connectors, nuts, belts, cables. Points for
  level of completion; time secondary. Application domain: assembly.

### Sun, Falco, Roa, Calli 2021, RGMC review
arXiv:2108.01483 [full text read, ar5iv] (RA-L)
- IROS Robotic Grasping and Manipulation Competition, 2016-2020. Tracks
  grouped by application: service (silverware, stirring, pouring, ice cubes,
  tearing paper towel, opening bottles), manufacturing (NIST task board),
  logistics (listed only). Scoring: best result per task (2016-17), total
  over five trials (2019-20), time limits; per-task point rules.
- In-hand manipulation tasks were largely dropped because teams could not
  solve them. Hand designs not compared as such. The 2024 edition had an
  in-hand manipulation track [metadata only, summary PDF
https://cse.usf.edu/~yusun/rgmc/RGMC2024-Summary.pdf via search].

### TC on Performance Evaluation and Benchmarking (IEEE RAS)
https://www.ieee-ras.org/performance-evaluation [metadata only, search
snippet]
- Approved 2009; activity reports in RAM 2010 and 2014; summer school on
  reproducible research and benchmarking. Not hand-specific.

### RoboCup@Work and RoCKIn@Work
https://atwork.robocup.org [metadata only, search snippet]
- Industrial mobile manipulation in a test arena; named tests on grasping,
  transport, precision placement, visual servoing. A RoCKIn@Work chapter
  (OAPEN library) covers scored task benchmarks. Not hand-specific.

### COMPARE ecosystem
arXiv:2509.22801 and arXiv:2504.06819 [metadata only, search snippets]
- Community effort for modular, benchmarkable open-source manipulation
  pipelines (perception, grasp planning, motion planning). Software, not
  hands.

### Our inference (general initiatives)
- The general initiatives standardise objects (YCB), artefacts (task boards)
  and protocols, scored by completion, time and final-state error. None is
  physics-based or demonstration-referenced, and none targets dynamic
  manipulation.

---

## 7. Grouping by application domain

- RGMC: service / manufacturing / logistics tracks (above, full text read).
- NIST task boards: assembly (above, metadata only).
- RoboCup@Work: industrial (above, metadata only).
- SHAP and AHAP: activities of daily living (above, metadata only).
- Bullock et al. 2013 apply their classification to three daily-living
  activities (metadata only).
- Surgery: JIGSAWS (Gao et al. 2014), da Vinci kinematics and video from
  eight surgeons doing suturing, knot tying, needle passing, rated by a
  modified OSATS global rating score (6 to 30). [metadata only, JHU PDF
  https://cirl.lcsr.jhu.edu/wp-content/uploads/2015/11/JIGSAWS.pdf via search]
  It scores surgeons, not hands, but it is a recorded-demonstration dataset
  by application.
- Crafts: nothing found.

### Our inference (application grouping)
- Application grouping exists mainly in competitions (RGMC tracks) and
  clinical tests (daily living). No dexterous-hand benchmark found offers
  application subsets of a single fixed task set, in the way the
  "ImageNet for manipulation" idea needs.

---

## Overall: our inference

- No fixed, community-wide task set for in-hand or dynamic hand manipulation
  was found. The nearest de facto convention is cube reorientation scored by
  consecutive goals within 0.4 rad, which compares controllers on one hand.
- Hand-comparing benchmarks exist (POMDAR, Elliott and Connolly, Elangovan, AHAP, FFP, NIST,
  KaRMA), but they are scored by completion, time, or kinematic reach. They
  are mostly quasi-static and not referenced to a demonstrated motion.
- Demonstration-referenced scoring exists in simulation retargeting
  (DexMachina, ManipTrans), with tolerances on object pose. It does compare
  hands, but through a learned policy, and without a per-frame wrench check.
- Wrench reasoning from demonstrations exists for static grasps (Lin & Sun
  task wrench space) and as an RL guide (CHORD), not as a per-frame verdict
  on hand designs.
- So a per-frame, per-hand, per-task wrench feasibility verdict against a
  demonstrated dynamic motion appears to be a gap. This is a negative finding
  from a limited search and should be stated with that caveat.

## Closest precedents to "hand designs scored per task against a demonstrated dynamic motion"

1. DexMachina (arXiv:2505.24853) - six hands, ARCTIC demonstrations, ADD-AUC
   against the demonstrated object trajectory. Nearest match. Simulation
   only; no wrench check; policy-mediated.
2. ManipTrans (arXiv:2503.21860) - four hands, per-frame errors against the
   demonstration with fixed tolerances (30 degrees, 3 cm, 8 cm, 6 cm),
   in physics simulation. Hand comparison is secondary.
3. CHORD (arXiv:2607.00033) - compares human and robot motion by object
   wrench; closest in spirit to a wrench check, but a training signal; hand
   comparison not confirmed.
4. Contact-anchored retargeting (arXiv:2609.24093) - physics-refined demos
   with recovered contact forces, replayed on four hands; success rates per
   hand; hands not named in abstract.
5. Lin & Sun 2015 (DOI 10.1177/0278364915583880) - task wrench space from
   demonstrated disturbances; grasp quality against it; static grasps;
   Barrett and Shadow in simulation.
6. POMDAR, Elliott & Connolly, Elangovan, AHAP - compare hands on fixed task
   lists, but scored by completion and time, not against a demonstration.
7. OpenAI 2018 / DextrEme / Qi et al. - the shared reorientation metric, but
   comparing controllers on fixed hardware.
