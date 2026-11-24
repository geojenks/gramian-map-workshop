# A target for the map page

The map page can draw a target of your own beside the actuators' paths. A
target is a small JSON file that gives values on the map's own axes. You
choose the file under "Task" on the page. It stays in your browser and is
sent nowhere.

The page only draws the target. It does not test any actuator against it:
you compare the target with the paths by eye. The test against a task, with
a cone and a verdict for each actuator, is a different file, written by
`python tools/export_map_paths.py --task my_task.json --tag mine`.

## What the page draws

A target names some of the map's axes and gives a value, a range or a list
of values on each. What you see depends on which three axes the map is
showing.

- An axis the target does not name is free, so the target runs the whole
  length of that axis.
- A target with one value on all three axes shown is a point. With one axis
  free it is a line, with two free it is a surface, and with all of the
  axes it names off the view it is not drawn. The caption under the list
  says which, for each target.
- A range on an axis, written as a low and a high number, widens the target
  along that axis. Ranges on three axes make a box.
- A path, written as a list of values on each axis, is drawn as a line when
  all three axes shown are named. With one of them free it is a surface
  swept along the free axis. With only one named it is the range the path
  covers on that axis.

When the axes shown change, the target is drawn again from the same file.

## The file

```json
{
  "kind": "target for the map page",
  "id": "my_target",
  "name": "Words for the list on the page",
  "note": "How the numbers were made",
  "placement": {"mount": "palm"},
  "show": ["axial_m", "side_y_m", "side_z_m"],
  "targets": [
    {"name": "A point", "at": {"axial_m": 0.0, "side_y_m": 0.015, "side_z_m": 0.0}},
    {"name": "A range", "at": {"side_y_m": [0.010, 0.020]}},
    {"name": "A path", "along": {"axial_m": [0.0, 0.0, 0.001],
                                 "side_y_m": [0.0, 0.008, 0.016],
                                 "side_z_m": [0.0, 0.001, 0.003]}}
  ]
}
```

| Field | Needed | Meaning |
|---|---|---|
| `kind` | yes | exactly `"target for the map page"` |
| `id` | yes | a short name of your own; a second file with the same id replaces the first |
| `targets` | yes | one or more targets, each holding either `at` or `along` |
| `at` | | for each axis named, one number, or a low and a high number |
| `along` | | for each axis named, a list of two or more numbers; the lists of one target are all the same length, one entry per point of the path |
| `name` | no | the words shown in the list, for the file and for each target |
| `note` | no | shown under "How this target was made" |
| `show` | no | three different axes the map switches to when the target is chosen |
| `placement` | no | `{"mount": "palm"}` or `{"mount": "aimed"}` where the values hold for one placement of the finger only; the target is then drawn on that placement and listed, but not drawn, on the other |

Other fields are kept and ignored, so a file can carry its own record of
where it came from.

## The axes and their units

Values are in the map's data units: metres, radians and fractions, not the
millimetres, degrees and per cent the page prints on its axes.

| Axis | Unit | What it is |
|---|---|---|
| `axial_m` | m | tip travel along the actuator's body |
| `side_y_m`, `side_z_m` | m | tip travel sideways, in the body's two crosswise directions |
| `bending_m` | m | the size of the sideways travel |
| `twist_rad` | rad | turn of the tip about the body's length |
| `bend_rad`, `bend_y_rad`, `bend_z_rad` | rad | bend angle (an estimate), and its two parts |
| `stretch_share` | fraction | stretch (an estimate) as a share of the body's length |
| `input_share` | fraction, 0 to 1 | share of the input range |
| `axial_share`, `bending_share`, `side_y_share`, `side_z_share` | fraction | the travels above as a share of the body's length |
| `input_scaled` | | the input, scaled as the page's "How each is measured" says |

The page lists the same axes, with how each is measured, under "How each is
measured". A file that names an axis the map does not have is refused, and
the page says which name it did not know.

Every path on the map starts at rest, at zero on the travel axes. A target
on those axes is travel from rest too.

## From a simulation's task file

If you have a task file in the form of `docs/TASK_FILE_FORMAT.md`, for
example one written from a MuJoCo simulation as `docs/TASK_FROM_MUJOCO.md`
describes, this writes a target from it:

```
python tools/leader_to_map_target.py my_task.json --tag mine
```

It writes `outputs/map_targets/map_target_mine.json` and never overwrites a
file. The target holds one line for each contact station: the path its
contact point takes, on the three tip-travel axes.

How a contact path becomes a line:

- A station makes and breaks contact. Each unbroken stretch of contact is
  one path. The longest stretch of each station is used, and the first
  where two are as long. `--all-stretches` writes every stretch of two
  frames or more. A station that is never in contact for two frames running
  is left out, and the file says so under `left_out`.
- The path is the contact point's position in each frame of the stretch,
  less its position in the stretch's first frame. It is taken in the task
  file's own frame, so the object's motion is not taken off.
- The path is turned into the frame of a finger standing on a palm beside
  the object, which is the placement the map page opens with. The task
  file's +z is up. The radial direction is the level part of the line from
  the object's centre to the contact point, where the stretch starts, and
  the frame stays fixed for the stretch. A contact that starts on the
  vertical line through the object's centre has no radial direction and is
  left out.

The file is marked for the palm placement, so the page does not draw it on
the other placement. A task file where the contacts do not move, such as
the three static pads of `docs/TASK_FROM_MUJOCO.md`, gives lines of no
length.

`examples/map_target_cup_turn_contact_paths.json` is the target this tool
writes from `data/leaders/w12c_cup_analytic_lock.json`. Choose it on the map
page to see what a target looks like.
