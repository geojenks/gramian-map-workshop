# A task file from your own MuJoCo simulation

A task file (a "leader" in this repository) says, frame by frame, where the
object is, which contact stations touch it, where, and the force each puts
on it. `docs/TASK_FILE_FORMAT.md` is the format. This page shows how to
write one from the contacts of your own MuJoCo run.

## What the sample does

`examples/mujoco_contacts_to_leader.py` builds a small scene from an XML
string inside the script: a free ball resting under gravity on a ring of
fixed pads below its centre (4 pads by default). It runs until the ball is
at rest, then records 40 frames over 1 s and writes
`outputs/my_mujoco_task.json`. Each pad is one station for the whole run.
It needs `mujoco` and `numpy` only (`pip install -e ".[hands]"`) and takes
under a second.

```bash
python examples/mujoco_contacts_to_leader.py
python examples/mujoco_contacts_to_leader.py --pads 3 --out outputs/my_mujoco_task_3pads.json
```

`--pads` is 3 or more. The file's `id` is its file name without `.json`.
The script stops if the output file already exists; give another `--out`.
After writing, the script prints what to look at: the net force on the
ball over its weight (it should be 1, pointing up), each pad's share of it
(by symmetry, each should carry the same), and how far MuJoCo's raw
contact points lay off the sphere.

The reusable part is one function,
`station_contacts(model, data, object_body_id, station_geom_ids)`. It
returns one frame's `contact`, `contact_point`, `force_on_object_N` and
`contact_normal`, one entry per station.

## Adapting it to your own simulation

1. **Load your model** in place of `build_scene`. The object must be a free
   body with one sphere geom: the format takes a sphere only, and the
   script stops if the object has none.
2. **Choose your stations.** For each one, give the geom id of that
   fingertip, pad or link, or a list of geom ids if several geoms make up
   one station. Keep the same order in every frame: station `i` is the
   same contact site throughout.
3. **Call `station_contacts` once per frame you keep**, after
   `mujoco.mj_forward(model, data)`. After `mj_step`, the contacts and body
   positions still belong to the start of that step; `mj_forward` brings
   them in line with `data.time`.
4. **Write each frame** with `t`, `obj_pos` (`data.xpos` of the object)
   and `obj_quat` (`data.xquat`, already w, x, y, z), plus the four fields
   the function returns. `station_pos` (where each station itself is, here
   the pad centres from `data.geom_xpos`) is optional: the follow check
   reads it only where a touching station has no `contact_point`.
5. **Fill the top of the file** from your model: `object` (radius from the
   sphere geom, mass from `model.body_mass`), `source` (your script, the
   mujoco version, every setting you changed), `stations` (a name each)
   and `caveat`.

## The traps

- **The force comes in the contact's own frame.** `mj_contactForce` gives
  the normal force first, then two tangential ones. The rows of
  `contact.frame` are those three axes in world coordinates; multiply by
  its transpose to get the world force.
- **Its sign depends on which geom is first in the pair.** The force and
  the frame's normal point from `geom1` to `geom2`: it is the force on
  `geom2`. The file wants the force on the object and the normal into the
  object, so both flip when the object is `geom1`. MuJoCo picks the order;
  never assume it.
- **One station can have several contacts.** A mesh fingertip or a box pad
  often touches at more than one point. Sum the forces. The function puts
  the contact point at their force-weighted mean, moved onto the sphere.
- **The forces must carry the weight.** The follow check adds no gravity.
  MuJoCo's contact forces on a free object already include its weight and,
  in a moving run, its acceleration: write them as they are. If the
  object hangs on a joint, a weld, a tendon or a mocap body, that carries
  load the contacts do not show.
- **Contacts that are not stations are lost.** If the object also touches
  a table, a palm or another link, that load is not in the file and the
  forces will not balance. The script counts such contacts and warns.
- **The contact point must lie on the sphere's surface.** MuJoCo's
  `contact.pos` sits midway through the overlap, not on the object. The
  function moves it onto the sphere along the line from the centre. The
  checker reports points more than 1 mm off the surface.

The conventions are those of `docs/TASK_FILE_FORMAT.md`: SI units, one
right-handed frame, forces on the object by the station, normals from the
contact towards the centre, quaternions w, x, y, z. Use z for the axis the
stations go round: the follow check's `respace` option turns stations
about it.

## Checking the file

```bash
python tools/check_leader.py outputs/my_mujoco_task.json
```

It should report no ERROR lines and a net contact force of 1.000 x the
object's weight. The checker's load figure is the size of the net force, so
it reads 1 even if every force has the wrong sign. The sample's own printout
shows the vertical part with its sign, which should be +1. The checker also
warns if a force pulls on the object, or if a stated normal is more than 5
degrees from the direction to the centre.

## Can a hand do it?

We have not run this section end to end on the sample's file. One try, in
Python, with the stored three-finger hand `wall_x3_ring`, the 3-pad file
and `n_eval=5`, was stopped unfinished after ten minutes on a laptop busy
with other work. So treat the commands below as the route, not as a
checked result, and expect the follow check to take a while.

This follows step 8 of "Bringing your own task" in
`docs/TASK_FILE_FORMAT.md`. You need the hand models first
(`python tools/fetch_hands.py`). `tools/run_follow.py --leader-id` only
looks in `data/leaders/`: it opens `data/leaders/<id>.json`, and the id
also goes into the result's file name. So copy the file into
`data/leaders/` first (`cp` on Linux or macOS, `copy` in the Windows
command prompt), keeping its name:

```bash
cp outputs/my_mujoco_task.json data/leaders/
python tools/run_follow.py --hands allegro leap --leader-id my_mujoco_task
```

Name the hands with `--hands`; without it the tool runs every hand it
knows. Give the file at least as many stations as the hand has fingers:
the follow check gives each finger its own station, and with fewer
stations than fingers it stops with an error. Allegro and LEAP have 4 fingers, Shadow 5,
DEX-EE 3. So `--pads 5` for `--hands shadow`, and `--pads 3` (or more) for
`--hands dexee`. A station with no finger still counts in the load the
hand must carry. Each run writes `outputs/follow_<hand>__<id>.json`;
`python tools/report_failures.py outputs/follow_<hand>__<id>.json` draws it
as a page.

Only the command-line tool ties you to `data/leaders/`. In Python,
`gramian_workshop.follow.follow(hand, leader)` takes the parsed file, so a
file can be read from anywhere:

```python
import json
from gramian_workshop.follow import follow
with open("outputs/my_mujoco_task.json") as f:
    result = follow("allegro", json.load(f))
```

## Contacts from another simulator, or from an experiment

The file needs the same per-frame fields whatever made them: `t`,
`obj_pos`, `contact`, `contact_point` and `force_on_object_N`, with
`obj_quat` and `contact_normal` if you have them. Write them by hand for a
few frames, or with your own script from another simulator's contact
report, keeping the traps above in mind. If you have no forces, for example
from a tracked demonstration, derive them from the object's motion as
`tools/import_demo_leader.py` does: the net force is m(a − g) and the
torque Iα, split over the touching stations inside each one's friction
cone. That tool reads our own recordings, so use it as a guide to the
method rather than run it; step 4 of "Bringing your own task" says what
to record about the mass and inertia you assumed. These numbers, like the
sample's, are simulation or estimate: comparative, not predictive.
