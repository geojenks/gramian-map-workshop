# Add your actuator to the map

One page, five steps, in the order of the workshop talk. You need Python 3.9
or newer and git. Nothing here needs a simulator.

## 0. Install (once)

```bash
git clone https://github.com/geojenks/gramian-map-workshop
cd gramian-map-workshop
python -m venv .venv
source .venv/bin/activate        # Windows PowerShell: .venv\Scripts\Activate.ps1
pip install -e ".[figures]"
```

The clone downloads about 80 MB. On Windows, clone into a short folder such
as `C:\work`. If PowerShell refuses `Activate.ps1`, skip that line and write
`.venv\Scripts\python.exe` for `python` below.

## 1. Nudge the input, record the tip

This is the one step you do on your own bench, or in your own simulation.
Fix the base of the actuator. Step its input (pressure, tendon pull,
voltage) through a few levels and at each level write down:

- **free**: how far the tip has moved from rest, along the actuator (X) and
  sideways (Y);
- **blocked**: with the tip held still, the force it makes, as X and Y parts.

Three levels are enough to start. One input per actuator: for a second
input, make a second file with the first held fixed.

## 2. Write the numbers in a file

Copy the filled example and put your numbers in it:

```bash
cp data/test_curves/mosadegh2014_fPN.csv my_curves.csv
```

Give it your own `id` (the map refuses an id it already has). Every column
name carries its unit, such as `input_kPa`, `tip_dy_mm`, `force_y_N`. The
example gives its force as a size only (`force_N`); put `force_x_N` and
`force_y_N` columns in its place so that your actuator can push. The
whole format is in `docs/TEST_CURVE_FORMAT.md`. If you only have datasheet
values, open `forms/record_form.html` in a browser instead and go to step 4.

If your actuator is a formula and not a test, put the formula in
`examples/analytical_model_to_curves.py` and let it write this file:

```bash
python examples/analytical_model_to_curves.py --out my_curves.csv
```

As shipped it holds a textbook bending finger with made-up numbers, and its
record can push.

## 3. Turn the file into the description

```bash
python tools/curves_to_record.py my_curves.csv --out my_actuator.json
python tools/check_record.py my_actuator.json
```

The second line prints what the tool reads from your record: push direction,
push force, motion, stroke. A force given only as a size, with no direction,
leaves the actuator able to move but not to push.

## 4. See it on the map

```bash
python tools/figure_map_tip_paths.py --record my_actuator.json --tag mine
```

This writes `outputs/map_tip_paths/map_tip_paths_mine.png`: every actuator
of the roster, with yours in a square and marked "(yours)".

## 5. Put the task on it

```bash
python demos/cup_hold.py --record my_actuator.json
python demos/cup_turn.py --record my_actuator.json
```

Your actuator is the last row of each table. Each letter says whether that
many copies of it hold, or turn, the ball in the cup, and if not, why.

## What this does not do yet

- It does not take a mesh or a CAD file. Going from a mesh to the numbers of
  step 1 needs a finite-element simulation. If you have a mesh, follow
  `docs/ACTUATOR_FROM_A_MESH.md` instead: its scripts are in `mesh_route/`,
  and its simulation step needs SOFA.
- The map here is a picture. Records are comparative, not predictive.
- To build a hand from your actuator, carry on with
  `docs/PARTICIPANT_SCRIPT.md`.
