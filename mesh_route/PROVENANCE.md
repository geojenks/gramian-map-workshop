# Where the files in this folder come from

Two files here, `parametricGripper.py` and `parametricGripperController.py`,
started from a SOFA SoftRobots tutorial and are under the GNU Lesser General
Public License, not the repository's Apache-2.0. `THIRD_PARTY_NOTICE.md` says
what comes from the tutorial.

Everything here except this note, `SOURCE_MANIFEST.json`,
`THIRD_PARTY_NOTICE.md` and `LICENSE.SoftRobots-LGPL-3.0.txt` was copied from
the private project this route was developed in, at commit
`664e4d2cf397c2416905205caeeea32c093916d5`. The folder layout is the same as
there, so the scripts find each other and their data without changes.

`SOURCE_MANIFEST.json` lists, for each copied file, its path in that project,
the sha256 of the original and of the copy, and every line we changed. We
changed lines in five scripts only: comments and messages that named folders
or documents of that project, and a fixed SOFA install path in `run_sweep.py`,
which now reads `SOFA_ROOT` and stops with a message if it is not set. No
change touches a number or a calculation.

## The body

`cads/actuation_methods/pumped_volume_pneunet/mesh.msh` is a pneu-net finger
made by `make_pneunet.py` (needs gmsh). The command recorded for it in that
project was:

    python make_pneunet.py --out cads/actuation_methods/pumped_volume_pneunet/mesh.msh --wall_thickness 0.001 --quiet

We did not rerun it for this copy. `actuator.npz` is not shipped:
`preprocess.py` rebuilds it from `mesh.msh` with numpy and meshio, and in our
check the rebuilt file was byte for byte the one in that project.

## The runs

The folders under `experiments/actuation_methods/same_body_pneunet/` are SOFA
runs of `run_shared_body_drive.py` on that body, made in that project. Each
`run_info.json` holds the command line it was run with (`argv`) and the time it
started. The held run (`tip_held_on_line`) ships without its `states.npz`:
`sample_to_draft_record.py` reads only its `drive_states.csv` and
`run_info.json`.

## The reference record

`expected/sample_pneunet_pressure_set.json` is the draft record that project
built from these runs with `sample_to_draft_record.py`. It is a draft, not one
of the map's records, and it is not in `data/records/`.
