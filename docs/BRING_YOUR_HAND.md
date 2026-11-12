# Bring your own hand

You have a model of a hand as a URDF file or a MuJoCo XML file. This page
takes it to a verdict: a hold the hand can do by construction, and the
follow check on that hold, with a report of which fingers and joints fail.
Once the hand is known by name, every other task file can be run on it the
same way.

You need the hands extra (`pip install -e ".[hands]"`, see the README) and
two files:

- the model file, as you have it. It is read where it is and never changed;
- a small description file that you write. It says what the model cannot
  say for itself: a name for the hand, and which body carries each
  fingertip.

Everything else is worked out from the model, by the same rules as for the
hands we prepared. A finger's joints are the hinge and slide joints between
its fingertip body and the base, less any joint that every finger shares
(a wrist). The palm is every body with collision geometry that belongs to
no finger. The fingertip's contact pad comes from the collision geometry of
the fingertip body.

The worked example on this page is the left Allegro hand from MuJoCo
Menagerie. `tools/fetch_hands.py` has already put it on disk, and it is not
one of the hands we prepared. Run every command from the top folder of the
repository.

## Step 1: look at the model

Give the tool the model file alone.

```bash
python tools/add_hand.py external/mujoco_menagerie/wonik_allegro/left_hand.xml
```

It prints the bodies of the model, each indented under its parent, with the
joints each one carries. It marks the last body of each chain, which is
usually a fingertip, and any body with no collision geometry. Below the
list it prints a description to start from, with the last body of every
chain filled in as a fingertip.

## Step 2: write the description file

Save the description as a JSON file anywhere you like. For the example,
save this as `allegro_left.json` in the top folder of the repository:

```json
{
 "schema": "gramian-workshop/hand-description@1",
 "key": "allegro_left",
 "name": "Allegro left hand (MuJoCo Menagerie)",
 "fingers": {
  "thumb": "th_tip",
  "index": "ff_tip",
  "middle": "mf_tip",
  "ring": "rf_tip"
 },
 "datasheet_torque_Nm": [0.70, "Allegro V4 max joint torque (reseller page)"]
}
```

What each line means:

| Line | Needed | Meaning |
| --- | --- | --- |
| `schema` | yes | Always the text shown. |
| `key` | yes | The name the tools will know the hand by. Lower-case letters, digits and underscores, starting with a letter. It must not be the name of a hand we prepared. |
| `name` | no | A title in words, used in results. |
| `fingers` | yes | One entry per finger: a label of your choice, then the name of the body that carries the fingertip. Two fingers or more. |
| `datasheet_torque_Nm` | no | The largest joint torque in N.m and where it was published. It is used only when the model itself gives no torque limits. Leave it out if no figure is published. |
| `datasheet_no_load_speed` | no | The joint motor's no-load speed, as published: the number, its unit (`rad/s`, `deg/s`, `rev/min` or `s/60deg`, seconds to turn 60 degrees) and where it was published, for example `[81, "rev/min", "maker's data sheet"]`. Used only by the torque-at-speed check (`run_follow.py --continuity --torque-speed`), which lowers each joint's torque limit as it moves faster, to nothing at this speed. Leave it out if no figure is published; the check then leaves the hand's limits as they are and says so. |
| `caveat` | no | A sentence that should travel with the hand's results. |

The label `thumb` is the one label with a meaning: the check lets the
object touch a thumb on its last link only. Any other label is yours.

The torque figure in the example is the one this repository already cites
for the right Allegro hand (`gramian_workshop/hands.py`).

## Step 3: check the pair and add the hand

```bash
python tools/add_hand.py external/mujoco_menagerie/wonik_allegro/left_hand.xml allegro_left.json
```

The tool builds the hand exactly as the later steps will. If something is
missing or wrong it prints `Not added.` and a numbered list of what needs
putting right, in words, and adds nothing. Put it right and run the command
again. To check without adding, put `--check-only` at the end.

When the pair is sound it prints what it worked out, for you to read
against what you know of the hand:

- the joints every finger shares, which it reads as a wrist and holds at
  rest;
- the palm bodies;
- for each finger: its joints, which of them are coupled to another joint,
  how many inputs it has, where its torque limits come from, and the size
  of its contact pad;
- notes, for example that the model gives no torque limits, so strength
  will not be tested.

It then prints `Added.` and the two commands of steps 4 and 5 with your
hand's name filled in.

The hand is recorded in `external/added_hands/<key>/` on this machine only;
that folder is not part of the repository. The model is still read from
where it is, so if you move it, add the hand again.

## Step 4: make a hold the hand can do

```bash
python tools/make_hold_leader.py allegro_left
```

This closes the fingers on a ball until every fingertip pad touches it, and
writes that as a task file: `data/leaders/hold_allegro_left__self__selfoverlapfix.json`,
with the hand's placement in `outputs/hold_placement_allegro_left__selfoverlapfix.json`.
The hand can do this hold by construction, so the answer to the check is
known before the check runs. It can take a few minutes. It will not write
over files that are already there.

## Step 5: run the check

```bash
python tools/run_follow.py --hands allegro_left --leader-id hold_allegro_left__self__selfoverlapfix --placement-from outputs/hold_placement_allegro_left__selfoverlapfix.json --tag fixed
```

It prints the verdict: how many frames are green, amber and red, and why
any are red. Below that it prints the failure report: which fingers are
amber or red and over which frames, and which joints are posed past a
limit. The full result is written to
`outputs/follow_allegro_left__hold_allegro_left__self__selfoverlapfix__fixed.json`.
A hold made in step 4 should come out green in every frame. If it does not,
the failure report says which finger to look at first.

The first line printed is a note that the placement file "has no placement
key". That is expected here: the placement came from step 4, not from an
earlier run of the check. When nothing fails, the failure report is the one
line `no finger is amber or red, no joint is past a limit`.

To draw the failure report as a page (it writes a `.md` and a `.html` file
next to the result, and prints their names):

```bash
python tools/report_failures.py outputs/follow_allegro_left__hold_allegro_left__self__selfoverlapfix__fixed.json
```

## After that

The hand is now found by name, so any task file in `data/leaders/` can be
run on it with `tools/run_follow.py --hands <key> --leader-id <task>`. The
task needs as many contact points as the hand has fingers.

```bash
python tools/add_hand.py --list
python tools/add_hand.py --remove allegro_left
```

The first lists every hand added in this copy of the repository. The second takes one out
again; task files and results already written are left where they are.
While a hand is added, tools that run every hand when none is named will
run it too.

## If your file is a URDF

Give the `.urdf` file as the model. The steps are the same. The tool first
builds a MuJoCo model from it and keeps that beside the hand's record:

- joint ranges and efforts are taken as the URDF gives them;
- each `<mimic>` tag becomes a joint coupling;
- collision meshes are read from where they are. A `package://` path is
  looked for beside the URDF and in the folders above it;
- a xacro file must be expanded to a URDF first.

Two things differ from what the URDF shows:

- A link held to its parent by a fixed joint is merged into the parent, so
  it is not a body of the model. If you name such a link as a fingertip,
  the tool tells you which body to name instead.
- The first link of the URDF is merged into the fixed world. The palm test
  does not read the world, so a palm that is the first link is not tested
  against the object. The tool says so in a note.

## What the model must have

The tool says which of these is missing, and for which finger.

- Two fingers or more, each ending in a different body.
- Collision geometry on every fingertip body. In MuJoCo XML a geom with
  `contype` and `conaffinity` both 0 is for display only.
- Hinge or slide joints only between each fingertip and the base. Remove a
  free joint at the base of the hand.
- A range of motion on every finger joint. The hold is posed from the
  ranges.
- No joint shared by some fingers but not all of them.
- For a test of strength: torque or force limits in the model (actuator
  force ranges, or joint efforts in a URDF), or a published figure in the
  description. Without either, the verdict covers reach and contact only.

## What we ran

Both kinds of file were taken through these steps on models that are not in
our table of hands.

The left Allegro hand (MuJoCo XML), exactly as written above:

```text
allegro_left: curl [0.6, 0.6, 0.5, 0.6], sphere R = 30.00 mm at [6.14, -9.23, 94.96] mm, contact miss per finger [1.618, 4.167, 1.519, 2.493] mm, facing [58.4, 84.0, 14.2, 80.3] deg, palm penetration 0.00 mm, other-link overlap 0.00 mm, overlap with itself 0.00 mm, object mass 15.13 g
allegro_left on hold_allegro_left__self__selfoverlapfix: {'green': 40}; red because {}; assignment {'thumb': 0, 'index': 1, 'middle': 2, 'ring': 3}; placement cost 0.00 mm; respaced by [0, 0, 0, 0] deg; 16.3 s
no finger is amber or red, no joint is past a limit
```

The Linker L20 right hand from the maker's URDF
(`external/linkerhand_urdf/L20/right/linkerhand_l20_right.urdf`), added
under the name `l20_from_urdf`:

```text
l20_from_urdf: curl [0.5, 0.35, 0.3, 0.25, 0.3], sphere R = 30.00 mm at [53.08, -3.04, 205.33] mm, contact miss per finger [0.207, 0.649, 1.135, 0.757, 0.161] mm, facing [44.2, 59.6, 20.9, 18.1, 59.0] deg, palm penetration 0.00 mm, other-link overlap 0.00 mm, overlap with itself 0.54 mm (world / ring_proximal), object mass 26.01 g
l20_from_urdf on hold_l20_from_urdf__self__selfoverlapfix: {'green': 40}; red because {}; assignment {'thumb': 0, 'index': 1, 'middle': 2, 'ring': 3, 'pinky': 4}; placement cost 0.00 mm; respaced by [0, 0, 0, 0, 0] deg; 66.1 s
no finger is amber or red, no joint is past a limit
```

We prepared this same hand ourselves earlier as `l20_public`. The model the
tool builds from the URDF has the same joints, ranges, effort limits,
couplings and meshes as that one, and the hold it gives is the same ball in
the same place.
