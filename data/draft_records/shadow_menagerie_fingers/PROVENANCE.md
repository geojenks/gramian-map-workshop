# Two Shadow fingers as rigid samples for the map (drafts, 21 Nov 2026)

Two jointed fingers, each as a joint table and draft records:

- the first finger of the **Shadow Hand E3M5**, with its abduction joint;
- one finger of the **Shadow DEX-EE** hand (the model gives its three fingers
  the same joint axes, ranges and actuator force ranges).

Everything here is a **draft**: not in `data/records/MANIFEST.json`, and
waiting on the rulings listed at the end.

## Where the numbers come from

Every number is read by script from two model files of MuJoCo Menagerie at
commit `b846dd12bc459d776cccb3dee0b1d02acbf7a9c7` (the commit
`tools/fetch_hands.py` pins). Both models are Apache-2.0.

| Finger | Model file | sha256 |
|---|---|---|
| Shadow Hand E3M5 first finger | `external/mujoco_menagerie/shadow_hand/right_hand.xml` | `9e6dd847cd1d7f2c86bc89e92e3bb0e8203940046c985c37d1e348b89d46b742` |
| Shadow DEX-EE finger F0 | `source/shadow_dexee.xml` (copied from `shadow_dexee/` at the same commit, with its `LICENSE`, `README.md` and `CHANGELOG.md`) | `7d7ad518b01451f58d3e5f3e59e58719741efead2d7730c9cf800f7d766f267a` |

No datasheet is used. What each table column holds:

- **Axes, joint positions:** each joint's axis and anchor at the model's zero
  pose, turned into the finger's own frame (origin at the first joint, X
  along the line through the joints, Y the side the tip moves to when the
  first bending joint turns positive).
- **Ranges:** the model's joint ranges.
- **Torque limits:** the force range of the model's actuator on that joint.
  These are the simulation model's limits, not a datasheet's. The DEX-EE
  force ranges are not symmetric; the table keeps both ends (`torque_Nm`
  towards the upper range end, `reverse_torque_Nm` the other way).
- **Stiffness:** the joint spring stiffness in the model, which is 0 for
  every joint (the models give the joints no spring). So a record's
  stiffness of 0 means "no passive spring", nothing more. The stiffness a
  position controller adds is not in any record here.
- **Tip point:** for the DEX-EE, the model's own site `F0/fingertip_site`.
  The Shadow Hand model has no fingertip site, so the tip is the far end of
  the last link's mesh on the line through the joints, 32.02 mm past the
  last joint. The stored `shadow_hand_finger` record uses a nominal 26 mm.

The written tables are read back and their tip position is compared with
MuJoCo's at 200 random poses: `tables_against_mujoco.txt` (both pass; the
largest difference is 3e-8 m).

## To make them again

    python data/draft_records/shadow_menagerie_fingers/build_shadow_finger_drafts.py tables
    python data/draft_records/shadow_menagerie_fingers/build_shadow_finger_drafts.py records
    python tools/joint_table_to_record.py data/joint_tables/shadow_e3m5_first_finger.csv --out <new name>.json
    python tools/joint_table_to_record.py data/joint_tables/shadow_dexee_finger.csv --out <new name>.json

`tables` needs `mujoco` (run here with MuJoCo 3.6.0); the rest need numpy
only. Every step refuses to overwrite, so give new names or work in a copy.

## What the models say about abduction

- The stored table `data/joint_tables/shadow_hand_finger.csv` and record
  `shadow_hand_finger` have three bending joints and **no abduction joint**.
- The Shadow Hand E3M5 model **does** have one: `rh_FFJ4`, range -0.349066
  to 0.349066 rad (20 degrees each way), at the same point as the first
  bending joint, with its own actuator `rh_A_FFJ4`. The finger has three
  independent inputs: `rh_A_FFJ4` (abduction), `rh_A_FFJ3` (base bending)
  and `rh_A_FFJ0`, one tendon that drives the last two joints together.
- The DEX-EE finger has four joints, each with its own actuator. Its first
  joint `J0` swings the whole finger sideways, -0.8727 to 0.8727 rad (50
  degrees each way); the other three bend it.

## More than one independent input: how it is handled

The record schema allows several input channels. The workshop's readers do
not: a two-channel record made in memory from the Shadow table passes the
validator, but `gramian_workshop/lookup.py` then reports "free tip curve has
50 input levels but 25 tip positions" and the map drops its tip path. So no
schema change is needed, and none was made.

Instead, as with the Chandler 2020 chambers and the OSSA modes, there is
**one record per independent input**: that input goes from 0 to the upper
end of its range and every other joint is held at 0. The rules are those of
`tools/joint_table_to_record.py`, whose functions are used unchanged.

There is also one record per finger made by that tool itself, with every
joint driven together. It is one arbitrary line through a finger with three
or four inputs, and is here only because it is what the tool makes.

## The records

| Record | Free stroke (mm) | Blocked force (N) | Limited by |
|---|---|---|---|
| `shadow_e3m5_first_finger_rh_A_FFJ4_alone` (abduction) | 35.43 | 9.802 | `rh_FFJ4` |
| `shadow_e3m5_first_finger_rh_A_FFJ3_alone` (base bending) | 144.27 | 9.802 | `rh_FFJ3` |
| `shadow_e3m5_first_finger_rh_A_FFJ0_alone` (last two joints) | 92.48 | 9.802 | `rh_FFJ3` |
| `shadow_e3m5_first_finger_all_joints_together` | 139.4 | 9.871 | `rh_FFJ3` |
| `shadow_dexee_finger_F0_J0_actuator_alone` (side swing) | 131.02 | 3.419 | `F0_J0` |
| `shadow_dexee_finger_F0_J1_actuator_alone` | 95.79 | 7.451 | `F0_J3` |
| `shadow_dexee_finger_F0_J2_actuator_alone` | 96.75 | 7.429 | `F0_J3` |
| `shadow_dexee_finger_F0_J3_actuator_alone` | 54.70 | 7.410 | `F0_J3` |
| `shadow_dexee_finger_all_joints_together` | 161.7 | 7.481 | `F0_J0` |

Numbers are from the script's printed output and `tools/check_record.py`;
the limiting joint is in the `metadata.notes` of the per-input records, and
in the converter's printed output for the two made by the tool. All nine pass
`tools/check_record.py` at tier 1.

How to read the blocked force: the tip is pinned at rest and pushes along
the direction that input first moves it. Every joint of the finger must
hold that push within the torque limit of its own actuator. At the straight
rest pose all bending inputs push the same way, so they share nearly the
same number, set by whichever joint is weakest for its lever (the base
joint on the Shadow Hand; the last joint, 0.3 N.m, on the DEX-EE). It is a
rest-pose figure from model limits: comparative, not predictive.

## What is assumed, and what is not known

- **The Shadow tendon.** The model fixes only the sum of the last two
  joints' angles. The record splits it equally along the way. The end point
  (both at their upper limit) is not an assumption. For the blocked force
  the two joints are each read on their own, as the stored table does.
- **Upper range end only.** Each input runs from 0 to the upper end of its
  range. Abduction and the DEX-EE side swing go as far the other way, and
  the DEX-EE's first bending joint goes further the other way (-1.3963 rad)
  than this way (0.7854 rad). None of that is in the records.
- **Actuation label.** The Shadow Hand records say `cable`, as the stored
  record does. The DEX-EE records say `electric`: the model gives each joint
  a position actuator and does not say how the real hand transmits it.
- **The DEX-EE tip site** sits 6.25 mm off the line through the joints, on
  the side away from the one the tip moves to when the bending joints turn
  positive. This is the model's own site; we have not checked it against
  the hardware.

## On the map

`outputs/map_tip_paths/map_tip_paths_with_drafts_shadow_fingers_21nov.png`
(and `.json`), made with

    python tools/figure_map_tip_paths.py --drafts --tag shadow_fingers_21nov

Three things to know when reading it:

- A hinge swept through 90 degrees lands near a sideways share of 0.5, not
  1: over a quarter circle the tip travels as far back along the finger as
  across it. The stored `shadow_hand_finger` sits at 0.99 only because it
  has no tip path and is placed from its first motion direction.
- The sideways axis does not tell abduction from bending: both are
  "across the finger". The record's `free_direction` does (Z against Y).
- The picture's legend calls every draft "from published figures". These
  are from model files. The wording comes from `demos/backwards_lookup.py`
  and `data/draft_records/FLAGS.json`, which this work did not touch.

## `tools/fetch_hands.py` and the DEX-EE model

At first the tool was left alone and the one model file was copied into
`source/` (27,631 bytes, no meshes); the records here are built from that
copy. Later on 21 Nov 2026 it was ruled that the DEX-EE joins the roster of
off-the-shelf hands, so three shared files were edited:

- `tools/fetch_hands.py`: `shadow_dexee` added to the list. The function
  used to return early when the checkout existed; it now adds any listed
  folder that is missing, and stops if the checkout is at another commit.
- `gramian_workshop/hands.py`: hand `dexee` (Shadow DEX-EE), three fingers,
  four joints and four actuators each.
- `docs/INSTALL_PLAN.md`: one row in the table of hand sources.

Every roster hand needs its own stored hold, so
`python tools/make_hold_leader.py dexee --run` was run. It wrote
`data/leaders/hold_dexee__self__selfoverlapfix.json`,
`outputs/hold_placement_dexee__selfoverlapfix.json` and
`outputs/follow_dexee__hold_dexee__self__selfoverlapfix__fixed.json`.
The hand's check of its own hold is amber, not green: two fingers pass, the
third misses on the direction its pad faces. Reported as it is.

## Ruled on 21 Nov 2026

- One record per independent input. (A finger whose joints share one tendon
  is one input; two inputs can later be drawn as a surface, as done before.)
- The tip point is the tip of the mesh (Shadow: 32.02 mm past the last
  joint), which is what the tables use.
- The DEX-EE joins the roster of off-the-shelf hands (above).

## Still open

1. Which of the nine go on the talk's map; in particular whether the two
   every-joint-together records are dropped.
2. Whether to draw the two-input surface for abduction with base bending.
