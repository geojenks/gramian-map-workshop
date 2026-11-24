# An actuator from a mesh

This page turns a finite-element mesh of a soft actuator into a record the
map can read. The scripts are in `mesh_route/`, with one worked example: a
pneu-net finger driven by its chamber pressure. The simulation runs of that
finger are stored, so you can rebuild its record, see it on the map and put
it on the cup task without a simulator. Runs of your own mesh need SOFA.

Where the files came from, and every line we changed in them, is in
`mesh_route/PROVENANCE.md` and `mesh_route/SOURCE_MANIFEST.json`.

Two of the scripts, the SOFA scene `parametricGripper.py` and its
controller, started from a SoftRobots tutorial and are under the GNU Lesser
General Public License, not this repository's Apache-2.0:
`mesh_route/THIRD_PARTY_NOTICE.md`.

## The steps

Run every step from `mesh_route/` unless it says otherwise.

| Step | Command | Needs | Writes |
|---|---|---|---|
| Make the mesh | `python make_pneunet.py --out <cad folder>/mesh.msh ...` | gmsh | `mesh.msh` |
| Split it into body, backing layer and chamber | `python preprocess.py <cad folder>/mesh.msh <cad folder>/actuator.npz` | numpy, meshio | `actuator.npz` |
| Drive it and push its tip | `python run_shared_body_drive.py ...` | SOFA 24.06 with SoftRobots, under Python 3.10 | `drive_states.csv`, `run_info.json`, `states.npz` per run |
| Build the record | `python sample_to_draft_record.py ...` | numpy, matplotlib | `<id>.json` and a folder of working files |
| Read it, map it, try the task | `tools/check_record.py`, `tools/figure_map_tip_paths.py`, `demos/cup_hold.py` | nothing more | a printout, a picture, the demo's tables |

The `mesh` extra installs meshio and matplotlib:

```bash
pip install -e ".[mesh]"
```

It does not install gmsh or SOFA. The worked example needs neither: its
mesh and its runs are stored.

## Run it without SOFA

From `mesh_route/`, rebuild the split mesh, then the record. Put the record
in a folder of your own, outside `data/records/`:

```bash
cd mesh_route
python preprocess.py cads/actuation_methods/pumped_volume_pneunet/mesh.msh cads/actuation_methods/pumped_volume_pneunet/actuator.npz
python sample_to_draft_record.py --sample experiments/actuation_methods/same_body_pneunet/chamber_pressure_set --cad cads/actuation_methods/pumped_volume_pneunet --id sample_pneunet_pressure_set --name "Pneunet, chamber pressure set (sample)" --physics pneumatic --input_name "chamber pressure (scene units)" --out_root ../my_mesh_drafts
```

Run `preprocess.py` first. The record takes the finger's length and
material volume from `actuator.npz` when it is there
(`record_from_jacobian.py`, lines 433 to 441). Without it the script
still runs and writes a record, but a different one: in
our try, `length_m` came out 0.0669 in place of 0.073 (it is then taken
from the simulated spine) and `material_volume_m3` was empty.

When we ran these, `preprocess.py` printed 8663 points and 26713 tets
(20737 silicone, 5976 backing layer), a chamber of 7340 triangles on 3672
nodes, and an outer skin of 9778 triangles on 4891 nodes. Its
`actuator.npz` was byte for byte the file in the project this route was
developed in. `sample_to_draft_record.py` ended:

```text
Wrote ...sample_pneunet_pressure_set.json (tier 2); 4 free levels; frame auto:178.87deg; blocked yes; stiffness yes
```

Its record was byte for byte the stored one. To check yours (this compares
the text, so it ignores line endings):

```bash
python -c "import sys; a,b=sys.argv[1:]; print(open(a).read()==open(b).read())" ../my_mesh_drafts/sample_pneunet_pressure_set.json expected/sample_pneunet_pressure_set.json
```

It should print `True`. Then, from the top folder of the repository:

```bash
cd ..
python tools/check_record.py my_mesh_drafts/sample_pneunet_pressure_set.json
python tools/figure_map_tip_paths.py --record my_mesh_drafts/sample_pneunet_pressure_set.json --tag my_mesh
python demos/cup_hold.py --record my_mesh_drafts/sample_pneunet_pressure_set.json
```

The first printed, among its lines:

```text
   warn   schema_version 0.2 != reference 0.1
     push force      0.1628 N   (comparative unless calibrated)
     stroke          8.03 mm
     push vs motion  79 deg apart (a bender often pushes along X but moves along Y)
```

The second writes `outputs/map_tip_paths/map_tip_paths_my_mesh.png`. In our
run (`--tag mesh_sample`) the finger sat with the other benders on the
right of the map, marked "(yours)": 98% of its tip motion sideways
(`sideways_share` 0.9808 in the picture's JSON) and a free stroke of
8.027 mm.

The third prints the demo's tables with this finger as the last row. With
the fingers aimed at the ball it reads `d s s s ✓ ✓ ✓ ✓`: one finger is
the wrong direction, two to four are not strong enough, five or more hold.
On the palm mountings it reads `d` for every number of fingers. The demo
writes over the tracked `outputs/cup_hold.*` files; put them back with
`git checkout -- outputs/cup_hold.html outputs/cup_hold.json outputs/cup_hold.txt`.

## Run the simulation yourself

We ran these commands in the project this route was developed in, not from
this repository. Each stored `run_info.json` holds the command line it was
run with, and the commands below are those, with a new output folder: the
runner refuses to write into a folder that already has a
`drive_states.csv`.

You need SOFA 24.06 with its SoftRobots plugin, and the Python 3.10 that
SOFA's Python bindings are built for. If `import Sofa` fails under that
Python, set `SOFA_ROOT` to the SOFA install folder. The scene reads the mesh
named by `CAD_NAME`; its default names a mesh that is not in this
repository, so always set it. In the Windows command prompt, from
`mesh_route/`:

```bat
set CAD_NAME=actuation_methods/pumped_volume_pneunet
set SOFA_PY=<the Python 3.10 that has SOFA>
"%SOFA_PY%" -u run_shared_body_drive.py --drive pressure --levels=5,10,20,30 --no_push --out_dir experiments/actuation_methods/my_runs/chamber_pressure_set/free_levels > free_levels.log 2>&1
"%SOFA_PY%" -u run_shared_body_drive.py --drive pressure --levels=20 --out_dir experiments/actuation_methods/my_runs/chamber_pressure_set/tip_pushes_run_to_completion > tip_pushes.log 2>&1
"%SOFA_PY%" -u run_shared_body_drive.py --drive pressure --levels=20 --hold_line y --out_dir experiments/actuation_methods/my_runs/chamber_pressure_set/tip_held_on_line > tip_held.log 2>&1
"%SOFA_PY%" -u run_shared_body_drive.py --drive none --levels=0 --push_axes yz --out_dir experiments/actuation_methods/my_runs/bare_body/tip_pushes > bare_body.log 2>&1
```

The first sets four chamber pressures with the tip free. The second holds
one pressure and pushes the tip both ways along y and z, for stiffness. The
third holds the tip on a line across its end, free to turn, for the force
it makes. The fourth pushes the tip of the body with nothing driven.

SOFA often does not exit when it has finished. A run is done when its log
says `[complete]` and its folder has `states.npz`; then stop the process.
Do not judge a run by its exit code. The whole route is hours of
compute (`docs/PARTICIPANT_SCRIPT.md`), so start the runs in the
background and watch their logs.

Then build the record as above, with
`--sample experiments/actuation_methods/my_runs/chamber_pressure_set` and
an id of your own.

## Your own mesh

- **Groups.** Name the volume `silicone` (needed) and the stiff backing
  layer `strain_limiter` (if there is one), both 3D groups, and the chamber
  wall `inner_cavity`, a 2D group. More than one chamber is
  `inner_cavity_1`, `inner_cavity_2`, and so on. `preprocess.py` warns if
  it finds no chamber.
- **Format.** Gmsh 2.2 ASCII: SOFA's mesh loader reads that version only.
  In Gmsh, set the options `Mesh.MshFileVersion` to 2.2 and `Mesh.Binary`
  to 0 before saving, as `make_pneunet.py` does.
- **Units and placement.** Metres. The body lies along x, with its base at
  low x, where the scene clamps it. The finger here runs from x = 0 to
  x = 0.073 (from its `actuator.npz`).
- **Where the scene clamps and where the tip is.** `parametricGripper.py`
  sets two boxes for this 73 mm finger, each `[xmin, ymin, zmin, xmax,
  ymax, zmax]` in metres: the clamp `[-0.002, -0.009, -0.013, 0.002,
  0.013, 0.013]` and the tip `[0.068, -0.006, -0.013, 0.074, 0.013,
  0.013]` (lines 76 and 77). If a box misses your mesh entirely, the scene
  makes one from the mesh's bounding box. If a box catches only part of
  your mesh, nothing changes it, so a longer body would have its "tip"
  read at x = 0.068. Then put a `roi.json` next to `actuator.npz`:

  ```json
  {"inlet_clamp_box": [xmin, ymin, zmin, xmax, ymax, zmax],
   "tip_roi_box": [xmin, ymin, zmin, xmax, ymax, zmax]}
  ```

  It wins over both the fixed boxes and the made ones.
- **One drive per record.** A record has one input. For a second input,
  make a second sample, with the first held fixed.
- **Material.** Neo-Hookean, set in `parametricGripper.py` lines 81 to 84:
  Young's modulus 7.0e5 Pa for the body and 5.0e6 Pa for the backing
  layer, Poisson's ratios 0.45 and 0.4. A `materials.json` next to
  `actuator.npz` with `young_modulus_body` and `young_modulus_limiter`
  replaces the two moduli for that mesh only.

Your mesh goes in its own folder under `mesh_route/cads/`, and `CAD_NAME`
is that folder's path below `cads/`.

## Limits

- **Scene units, not kPa.** The chamber pressure is in the scene's own
  units, and the record says so in its input name and notes. SoftRobots'
  pressure constraint acts as its value divided by the time step (0.01 s
  here, `dt_s` in `run_info.json`), and we have not ruled on the physical
  unit. Forces are in newtons as the scene computes them, uncalibrated.
- **Comparative, not predictive.** Like every record here, the numbers
  compare actuators with each other. They do not predict a bench test.
- **Gravity is off** in the scene (`parametricGripper.py` line 78), so the
  finger does not sag under its own weight.
- **The record is a draft.** Keep it out of `data/records/`. The script
  refuses to write into a `records/` folder inside `mesh_route/`, but it
  does not know about `data/records/`, so it will not stop you there. The
  demos take a record from anywhere with `--record`.
- **The schema-version warning.** The record is written by
  `mesh_route/actuator_record.py`, schema 0.2. The map's own copy of that
  file is 0.1 and warns about any other version. Version 0.2 adds the
  meaning of records with several inputs and a block for published
  sources; nothing in 0.1 changed meaning (`actuator_record.py`, lines 23
  and 24). The warning does not stop anything.
- **The comments in `preprocess.py` and `parametricGripper.py` disagree on
  which way the chamber's surface normals point.** We have not changed
  either. The stored runs were made with the code as it is, and the record
  rebuilds from them.
