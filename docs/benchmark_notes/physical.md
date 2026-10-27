# Physical task sets for characterising robot hands

Literature notes, 27 Oct 2026. Scope: physical (not simulated) task sets and how each scores a hand.

How sources are marked:
- [full text read]: I read the paper or document itself (sometimes only some pages, as stated).
- [abstract]: I read only the abstract or a publisher summary page.
- [metadata only]: I saw only the citation or a search snippet.
- [UNVERIFIED]: a claim I could not check against any source I reached.

Each source records five things: the tasks and how many; how a hand is scored; whether scoring is physics-based, against a demonstrated motion, or rated by humans; whether any tasks are dynamic; and whether results across hands are published.

Everything above the heading "Our inference" reports what the sources say. Interpretation is kept to that section.

---

## Grasp taxonomies

### Cutkosky 1989
- Citation: M. R. Cutkosky, "On grasp choice, grasp models, and the design of hands for manufacturing tasks", IEEE Trans. Robotics and Automation 5(3):269–279, 1989. DOI 10.1109/70.34763 [UNVERIFIED: the DOI comes from a University of Washington lecture slide, courses.cs.washington.edu/courses/cse478/20wi/site/resources/lec23_grasping.pdf. The IEEE Xplore page came back empty.]
- Status: [metadata only].
- Tasks: none. It is a hierarchy of grasp types, built by watching machinists at work. The number of grasp types is [UNVERIFIED]; I did not reach the paper.
- Scoring: none. It is a vocabulary.
- A Yale ICRA 2011 paper calls it the taxonomy most used in robotics (eng.yale.edu/grablab/pubs/Zheng_ICRA2011.pdf) [full text not read; claim from search summary].
- Dynamic tasks: no. Cross-hand results: no.

### Feix et al. 2016, the GRASP taxonomy
- Citation: T. Feix, J. Romero, H.-B. Schmiedmayer, A. M. Dollar, D. Kragic, "The GRASP Taxonomy of Human Grasp Types", IEEE Trans. Human-Machine Systems 46(1), 2016. DOI 10.1109/THMS.2015.2470657.
- Status: [full text read, pages 1–2].
- Tasks: 33 grasp types, which merge to 17 if grasps that differ only in object shape are combined.
- Scope: static, stable grasps with one hand only. In-hand motion and two-hand grasps are left out.
- Scoring: none. It is a vocabulary that other protocols (such as AHAP below) draw from.
- Dynamic tasks: no, excluded by definition. Cross-hand results: no.

---

## Object sets and protocol frameworks

### YCB object and model set (Calli et al. 2015 onward)
- Citation: B. Calli et al., "Benchmarking in Manipulation Research: The YCB Object and Model Set and Benchmarking Protocols", IEEE Robotics and Automation Magazine 22(3), 2015. DOI 10.1109/MRA.2015.2448951; arXiv 1502.03143.
- Status: [full text read, arXiv pages 6–11].
- Objects: 73 IDs in five groups. Food is 1–19, kitchen 20–33, tools 34–53, shapes 54–68, task items 69–73. The task items include a Box and Blocks set, a 9-hole peg test, Lego, a toy aeroplane and a timer.
- Protocol template, set by the authors:
  - task description;
  - setup;
  - robot, hardware or subject;
  - procedure;
  - execution constraints.
- Each protocol is paired with a benchmark that states the scoring and what to submit.
- The authors advise partial points rather than pure success or fail.
- Example protocols in the paper:
  - pouring from a pitcher into a mug;
  - gripper assessment;
  - table setting;
  - block pick and place;
  - peg-insertion learning.
- Box and Blocks is also used as a baseline.
- Example result: the HERB robot poured successfully in 8 of 10 pitcher–mug configurations. The pouring score is cut for liquid left behind or spilt.
- Gripper assessment used a scoring table, applied to the Yale Model T and T42 hands.
- Scoring type: task outcome (physical end state, counted or timed). Not against a demonstration. Not human-rated in the subjective sense.
- Dynamic tasks: none described. Pouring involves moving liquid, but the score is on outcome only.
- Cross-hand results: protocols and results are hosted on arXiv and a portal. Each group publishes its own; there is no central ranking that I found.

### Box and Blocks Test for robots (Morgan et al. 2020)
- Citation: A. S. Morgan et al., "Benchmarking Cluttered Robot Pick-and-Place Manipulation With the Box and Blocks Test", IEEE RA-L 5(2):454–461. DOI 10.1109/LRA.2019.2961053.
- Status: [full text read, pages 1–3].
- Tasks: three protocols of rising difficulty, built on YCB items 68–70.
  - Modified: move 16 blocks, all facing the same way, over a barrier in minimum time. Mainly tests the hand and its grasp.
  - Targeted: as Modified, but each block must land on a marked spot on a template. Tests accuracy and control.
  - Standard: move as many as possible of 100 randomly heaped blocks. One version is timed over one minute, to compare with clinical norms; the other is untimed.
- Rules:
  - Blocks may not be thrown over the barrier.
  - If a container moves more than 2.54 cm or turns more than 10°, the trial scores zero.
  - Any hardware or sensing is allowed, but the YCB objects may not be altered.
- Scoring: counts and times, compared with human norms (one-minute block counts by age group; Modified and Targeted times from small groups of healthy adults).
- Scoring type: physical outcome and time. Human norms give the reference, not a demonstrated motion.
- Dynamic tasks: no. Throwing is banned.
- Cross-hand results: baselines on a Barrett WAM and a Franka Emika Panda, compared with humans. The paper notes robots are far slower than people.

### Anthropomorphic Hand Assessment Protocol, AHAP (Llop-Harillo et al. 2019)
- Citation: I. Llop-Harillo, A. Pérez-González, J. Starke, T. Asfour, "The Anthropomorphic Hand Assessment Protocol (AHAP)", Robotics and Autonomous Systems 121:103259, 2019. DOI 10.1016/j.robot.2019.103259. Data: DOI 10.5281/zenodo.5704892.
- Status: [full text read, pages 3–6].
- Tasks: 26.
  - 8 grasp types, each tried on 3 YCB objects (24 tasks).
  - Pressing a timer button with the index finger.
  - A platform task with a plate.
  - 25 objects in all.
- Procedure:
  - An operator shows the posture, and the subject practises it first.
  - A human operates the hand.
  - The paper says the subject holds each grasp for three seconds.
- Scoring, per repetition, three repetitions per task:
  - Grasp, held 3 s palm up: 1 if the intended grasp type, 0.5 if a different type, 0 if it fails.
  - Then the hand is turned 180° and held 3 s: 1 if the object does not move, 0.5 if it moves, 0 if dropped.
- The Grasping Ability Score is the total as a percentage of what a human hand scores.
- Scoring type: human-rated from video, against a demonstrated posture. The posture match is judged by eye; the hold is a physical outcome. The dataset has five raters.
- Dynamic tasks: no. All holds are static.
- Cross-hand results: yes, a few. The ARMAR-6 hand in three fingertip-pad versions, and the KIT prosthetic hand in two versions. A full test takes 80–100 minutes.

### In-hand manipulation benchmark (Cruciani et al. 2020)
- Citation: S. Cruciani et al., "Benchmarking In-Hand Manipulation", IEEE RA-L 2020. DOI 10.1109/LRA.2020.2964160; arXiv 2001.03070. Project page: robot-learning.cs.utah.edu/project/benchmarking_in_hand_manipulation.
- Status: [abstract].
- Tasks: YCB object meshes with set start and goal poses, and set fingertip positions.
- Scoring: error between reached and goal pose, rolled into one score.
- Scoring type: physical, against a goal pose (not a full demonstrated path).
- Dynamic tasks: not stated in the abstract.
- Cross-hand results: a public results store, with entries for an Allegro hand and an ABB YuMi.

---

## Clinical hand tests used on robot or prosthetic hands

### Southampton Hand Assessment Procedure, SHAP
- Citation: C. M. Light, P. H. Chappell, P. J. Kyberd, Arch. Phys. Med. Rehabil. 83(6):776–783, 2002. Record: eprints.soton.ac.uk/256475 (returned 403). The SHAP website did not resolve.
- Status: [metadata only].
- Tasks, per a search snippet: 6 abstract objects and 14 daily-living activities. The figure of 26 tasks seen in a secondary table is [UNVERIFIED].
- Scoring: the subject times each task. A combined "Index of Function" score is [UNVERIFIED].
- Scoring type: timed task completion against human norms.
- Built for prosthetic hands. Dynamic tasks: speed only. Cross-hand results: not checked.

### Jebsen–Taylor Hand Function Test
- Citation: R. H. Jebsen et al., Arch. Phys. Med. Rehabil. 50:311–319, 1969. PMID 5788487.
- Status: [abstract]. Secondary summaries: sralab.org/rehabilitation-measures/jebsen-taylor-hand-function-test and strokengine.ca/en/assessments/jebsen-hand-function-test-jhft.
- Tasks: 7 timed subtests of everyday hand use.
- Scoring: time only. The quality of movement is not scored. Norms come from 300 people.
- Scoring type: timed outcome.
- Used on a soft prosthetic hand: arXiv 2503.01585 [abstract, via search].
- Dynamic tasks: speed only. Cross-hand results: not found.

### Box and Blocks Test (clinical original)
- Citation: V. Mathiowetz et al., Am. J. Occup. Ther. 39(6):386, 1985. DOI 10.5014/ajot.39.6.386; pubmed.ncbi.nlm.nih.gov/3160243.
- Status: [abstract]; norms also read in Morgan et al. above.
- Task: one. Move as many single blocks as possible from one box to another over a barrier in one minute.
- Norms from 628 adults.
- Scoring type: count in a fixed time.
- Dynamic tasks: speed only. Robot version: see Morgan et al. above.

### Action Research Arm Test, ARAT
- Citation: R. C. Lyle, Int. J. Rehabil. Res. 4(4), 1981. DOI 10.1097/00004356-198112000-00001.
- Status: [metadata only]. Content from secondary pages: strokengine.ca/en/assessments/action-research-arm-test-arat and the SCIRE project.
- Tasks: 19 items. Grasp 6, grip 4, pinch 6, gross movement 3.
- Scoring: each item 0–3 by a clinician, with time limits marking the levels. Maximum 57.
- Scoring type: human rating.
- Dynamic tasks: no. Robot or prosthesis use: not checked.

---

## Standards bodies

### NIST dexterity test methods, draft SP 1227
- Citation: NIST SP 1227 (draft). DOI 10.6028/NIST.SP.1227-draft.
- Status: [full text read, pages 3–5 and 55–61].
- Tasks: 10 test methods.
  - finger strength;
  - grasp strength;
  - slip resistance;
  - cycle time;
  - touch sensitivity;
  - grasp efficiency;
  - force calibration;
  - finger force tracking;
  - in-hand manipulation;
  - object pose estimation.
- In-hand manipulation method:
  - The hand is told to move a held object along a sine path, amplitude times sin(2πft).
  - Motion capture tracks the object.
  - The score is the root-mean-square pose error.
  - The document calls it "a kinematic measure of how well a robotic hand can control the pose".
- Worked example: one three-fingered hand with 7 actuated joints, moving a sphere, cuboid and cylinder. Frequencies of 0.12–0.20 Hz; amplitudes of 7.5 mm and 7.2°.
- Scoring type: physical measurement. The in-hand method scores against a commanded path within a measured error.
- Dynamic tasks: partly. Cycle time, force tracking and the sine paths involve timing, but the sine paths are slow.
- Cross-hand results: only one hand shown in the worked example.

### Falco et al. 2020
- Citation: J. Falco et al., RA-L 2020. DOI 10.1109/LRA.2020.2964164.
- Status: [abstract].
- Covers grasp strength, cycle time, finger strength and repeatability.
- Scoring type: physical. Cross-hand results: not checked.

### NIST project page and ASTM work items
- Source: nist.gov/programs-projects/grasping-manipulation-and-contact-safety-performance-robotic-systems.
- Status: [full text read].
- ASTM committee F45.05 work items:
  - WK83863, grasp strength;
  - WK96472, slip resistance;
  - WK87214 and WK87213, task boards.
- No comparison across hands on the page.

### ASTM F3756-26
- Source: store.astm.org/f3756-26.html.
- Status: [abstract; scope only].
- Defines a split-force measuring rig. It supports the grasp strength, slip resistance, grasp efficiency and cycle time tests.
- Scoring type: physical force measurement. Cross-hand results: none published with the standard.

### NIST task boards (Kimble et al. 2022)
- Citation: K. Kimble et al., Frontiers in Robotics and AI 2022. DOI 10.3389/frobt.2022.999348.
- Status: [abstract].
- Tasks: four main boards with screws, gears, connectors, wires and belts.
- Scoring: a points scheme, with completion time compared against people.
- Scoring type: task outcome and time.
- Dynamic tasks: no. Cross-hand results: not checked.

---

## Competitions

### Robotic Grasping and Manipulation Competition, RGMC, 2016–2020
- Citation: Y. Sun, J. Falco, M. A. Roa, B. Calli, "Research Challenges and Progress in Robotic Grasping and Manipulation Competitions", RA-L. arXiv 2108.01483.
- Status: [full text read, through a page summary].
- Editions: 2016, 2017, 2019 and 2020. Tracks for service, manufacturing and logistics.
- Tasks: more than 40 across the editions, 23 of them service tasks. The 2016 edition had 18 tasks and 20 objects (Springer CCIS 816; task-pool record at pure.skku.edu) [metadata only].
- Scoring:
  - Points for each task completed.
  - 2016–17 counted the best trial; 2019–20 summed five trials.
  - Time limits of 90 and 60 minutes.
- In-hand tasks were mostly not solved and were later dropped.
- Scoring type: task outcome.
- Dynamic tasks: none.
- Cross-hand results: yes, team rankings, though each team uses its own whole robot.

### RGMC 2024 at ICRA
- Source: cse.usf.edu/~yusun/rgmc/2024.html [full text read]. In-hand track rules: arXiv 2502.07472 [full text read, through a page summary].
- Tracks: cloth, in-hand, picking from clutter, hand-over, and a manufacturing task board.
- In-hand track:
  - Move a held object through 10 waypoints inside a 5 cm cube, with 20 s per waypoint.
  - The score is the summed position error.
  - Done once with a known cylinder and once with an unseen object; the better of two runs counts.
- Scoring type: physical, against set target points.
- Dynamic tasks: hand-over involves a moving partner. The in-hand track is slow.
- Cross-hand results: yes, competition results.

### Amazon Picking Challenge 2015
- Citation: "Analysis and Observations from the First Amazon Picking Challenge", IEEE T-ASE (first author Correll [UNVERIFIED]). DOI 10.1109/TASE.2016.2600527; arXiv 1601.05484.
- Status: [full text read, through a page summary].
- Task: pick 12 items from a shelf in 20 minutes.
- Scoring:
  - 10, 15 or 20 points per item, by bin difficulty, with a bonus of 1–3 for hard items.
  - Penalties: −3 for a drop, −5 for damage, −12 for a wrong item.
  - Maximum 190.
- Scoring type: task outcome.
- Dynamic tasks: no.
- Cross-team results: yes. RBO scored 148, MIT 88 and Grizzly 35, of 26 teams.

### Cybathlon arm prosthesis race
- Source: Cybathlon 2020 "Races and Rules", version 2019-03-22 (tugraz.at/fileadmin/user_upload/Institute/INE/BCI_Series_2019/20190322_CYBATHLON_Races_and_Rules.pdf).
- Status: [full text read, pages 2–4, 7–9, 14, 25–30 and 49]. Background: R. Riener editorial 2016, PMC4886429 [abstract].
- Tasks: 6 stations.
  - breakfast;
  - laundry;
  - clean sweep;
  - home improvement;
  - haptic box;
  - wire loop.
- Scoring:
  - Points per task: 101, 102, 104, 108, 115 and 130, totalling 660.
  - 8 minutes in all.
  - Ranked by points, then by time.
- Blue parts may be touched only by the prosthesis.
- Scoring type: task outcome under a time limit, judged by referees against the rules.
- Dynamic tasks: the race is timed. No throwing or catching.
- Cross-hand results: yes, public race results. For example, a SoftHand Pro pilot came 5th of 12 (J. NeuroEng. Rehabil. 2017), and a PLoS ONE 2023 paper compares results [both via search summaries].

---

## Surgical task sets

### Fundamentals of Laparoscopic Surgery, peg transfer
- Status: [metadata only, secondary sources]. The original citation (Fried et al. 2004) is [UNVERIFIED].
- Task: move small rings between pegs, passing each from one instrument to the other. The ring count (six) is [UNVERIFIED].
- Scoring: 300 minus time in seconds minus a penalty for drops.
- Scoring type: time and errors.
- Robot use:
  - Hwang et al., arXiv 2012.12844 [abstract]: da Vinci Research Kit, 3,384 trials, compared with a surgical resident and 9 volunteers on speed, accuracy, consistency and collisions.
  - arXiv 2607.07972: a humanoid robot's times are above 300 s [search snippet only].
- Dynamic tasks: speed only.

### JIGSAWS
- Citation: Y. Gao et al., "JHU-ISI Gesture and Skill Assessment Working Set", M2CAI workshop 2014. The dataset page returned 403.
- Status: [metadata only, search snippets].
- Tasks: 3 (suturing, knot-tying and needle-passing), done by 8 surgeons, 5 times each, on a da Vinci surgical robot [UNVERIFIED: robot named from search snippets only].
- Data: robot kinematics and video, with gestures marked by hand.
- Scoring: a modified global rating scale (OSATS), 6–30, given by an expert.
- Scoring type: human rating. The motions are recorded, so it holds demonstrations, but the score is not a match against them.
- Dynamic tasks: no. Cross-hand results: no; it rates people, not hands.

---

## Our inference

### Closest to "a hand scored per task against a demonstrated motion"
1. NIST in-hand manipulation method. The object path is commanded rather than shown by a person, but it is scored by measured error against that path. This is the nearest physical match.
2. RGMC 2024 in-hand track. Scored by error against set target points, not a full path.
3. Cruciani et al. 2020. Scored against goal poses (end states), not the path between.
4. AHAP. The posture is shown by a person and the hand is scored on matching it. But the match is judged by eye and the hand is driven by a person.
5. JIGSAWS. Holds recorded expert motions but scores by human rating, not by matching them.

None of these sets scores pass or fail within a stated tolerance of a human demonstration.

### Which cover dynamic tasks
- None includes throwing, catching, spinning or other fast tasks where the object's momentum matters.
- Partial cases:
  - NIST: cycle time, force tracking, and slow sine paths up to 0.2 Hz.
  - RGMC 2024: hand-over to a moving partner.
  - Speed tests: Cybathlon (timed race), Box and Blocks, Jebsen–Taylor, SHAP and FLS peg transfer. These reward speed but score only the outcome.
- Morgan et al. ban throwing outright.

### Other points
- Results across hands are published for APC, RGMC, Cybathlon, AHAP (a few hands) and Cruciani. NIST and ASTM methods have no published comparison of hands that I found.
- Subsets by application already exist in part:
  - surgery: FLS and JIGSAWS;
  - daily living: Cybathlon, SHAP, AHAP and the clinical tests;
  - manufacturing: the NIST task boards.
- I found no set for crafts.
- Most sets score whole robots, with arm, vision and planning mixed in. Only AHAP, the NIST methods and ASTM F3756 try to isolate the hand.
