# What a participant does on the day

One list of commands, in order, from nothing on the machine to a verdict
on a hand built from an actuator, and then on a hand brought as a model
file. Follow it from the top. Run every command from the top folder of the
repository, with the virtual environment active.

Every `python` line on this page was run in order, as written, in a fresh
clone on 12 Nov 2026 by `tools/run_participant_script.py`, with one
environment (the hands extra: numpy and MuJoCo, nothing else). What each
printed is in `outputs/participant_script_12nov/log.txt`, and every line
under "You should see" is copied from that log. The install commands of
step 1 download from the internet and were not run for this page; the
repository's install check runs them on clean machines.

Some things on the way do not work as a newcomer would expect. They are
marked where they come up and listed at the end.

## 1. Get the tool (not timed: most of it is downloading)

You need Python 3.9 or newer and git. On Windows, in PowerShell:

```powershell
git clone https://github.com/geojenks/gramian-map-workshop
cd gramian-map-workshop
python -m venv .venv
.venv\Scripts\activate
pip install -e ".[hands]"
python tools/fetch_hands.py
```

On Linux or a Mac the fourth line is `source .venv/bin/activate`. The
last line fetches the hand models into `external/`.

## 2. Check it runs (under a minute)

```bash
python demos/cup_hold.py
python tools/compare_outputs.py outputs/cup_hold.json
```

The first prints a table: one row per actuator record, one column per
number of fingers, and a letter for why each arrangement fails
(the README's "reason ladder" says what the letters mean). It also writes
`outputs/cup_hold.html`: open it in a browser and hover over any cell.

**Does not work today.** The README says the second command should end
`reproduces HEAD`. It does not:

```text
outputs/cup_hold.json: 190 differences from HEAD
```

The demo itself runs; it is the stored result that is out of date
(issue 37). Carry on.

## 3. Your actuator, as test curves (about 14 s)

The example is a published pneu-net bending actuator, as the two curves a
bench test gives (`data/test_curves/mosadegh2014_fPN.csv`;
`docs/TEST_CURVE_FORMAT.md` is the format). Make a record from it, and read
back what the tool takes from the record:

```bash
python tools/curves_to_record.py data/test_curves/mosadegh2014_fPN.csv --out my_actuator.json
python tools/check_record.py my_actuator.json
```

You should see, among the lines of the second:

```text
     push direction  -   from no force data: this actuator cannot push in the tool
     stroke          94.9 mm
```

That is: this file gives how the actuator moves, and no force. Now put it
in the first demo beside the stored records:

```bash
python demos/cup_hold.py --record my_actuator.json
```

Its row is the last one of each table, and it reads "no contact" for every
number of fingers, because the record has no force data:

```text
mosadegh2014_fPN_from_curves      –  –  –  –  –  –  –  – [source:user record (not in the manifest) no_force_data]
```

Two other ways in, if you would rather:

- **Datasheet numbers.** Open `forms/record_form.html` in a browser, type
  the numbers in whatever units you have, and download the record. Then
  use it in place of `my_actuator.json` above.
- **A table of joints** (a rigid finger). This makes a record, with force:

```bash
python tools/joint_table_to_record.py data/joint_tables/shadow_hand_finger.csv --out my_finger.json
python tools/check_record.py my_finger.json
```

**Does not work today.** A record made from a joint table stops here: the
hand build of step 4 crashes on it (issue 33).

## 4. A hand from your actuator (seconds)

A hand file says how many fingers, which record each one is, and where
each stands on the palm. `examples/test_curve_hand/hand.json` is one we
wrote for this page: three fingers of `my_actuator.json` on a ring of
radius 30 mm.

```bash
python tools/build_record_hand.py examples/test_curve_hand/hand.json
```

You should see one line per finger that ends like this, and a last line
naming the model it wrote:

```text
f0: my_actuator.json, fit check PASS, radius 8.00 mm (hand file)
wrote examples\test_curve_hand\hand.xml
```

**Needs choices the docs do not give (issue 32).** Three things in that
hand file had to be found by trial:

- the record is named by its file path. Naming it by its id fails;
- `contact_radius_m` (8 mm here) is our choice. The test-curve format has
  no place for the actuator's size, and without a radius the build stops;
- `bend_axis` points outward from the ring. This record bends towards -Y,
  and with the axis pointing inward the fingers curl away from the ball.

Now tell the tools about the hand. The second file names it and says which
body is each fingertip:

```bash
python tools/add_hand.py examples/test_curve_hand/hand.xml examples/test_curve_hand/description.json
```

It prints what it worked out for each finger, then `Added.` and the next
two commands.

## 5. A task (seconds)

The task for a new hand is a hold it can do by construction: the fingers
close on a ball until every fingertip touches. The answer to the check is
then known before the check runs.

```bash
python tools/make_hold_leader.py test_curve_hand
```

You should see where the ball sits and how far each finger misses it:

```text
test_curve_hand: curl [0.0, 0.0, 0.0], sphere R = 30.00 mm at [0.0, 0.0, 117.48] mm, contact miss per finger [0.0, 0.0, 0.0] mm, facing [6.0, 6.0, 6.0] deg, palm penetration 0.00 mm, other-link overlap 0.00 mm, overlap with itself 0.00 mm, object mass 18.77 g
```

Tasks from elsewhere are files in `data/leaders/`
(`docs/TASK_FILE_FORMAT.md`). To read what the check would take from one:

```bash
python tools/check_leader.py data/leaders/w10_cup_physical_x0.1_a0.15_n4__contactfix.json
```

Running such a task on a new hand needs a search for where to place the
hand, which is a long run. That is not part of this page.

## 6. The verdict (about 106 s)

```bash
python tools/run_follow.py --hands test_curve_hand --leader-id hold_test_curve_hand__self__selfoverlapfix --placement-from outputs/hold_placement_test_curve_hand__selfoverlapfix.json --tag fixed --margins
```

The first line printed is a note that the placement file "has no placement
key". That is expected. Then the verdict:

```text
test_curve_hand on hold_test_curve_hand__self__selfoverlapfix: {'green': 40}; red because {}; assignment {'f0': 0, 'f1': 1, 'f2': 2}; placement cost 0.00 mm; respaced by [0, 0, 0] deg; 105.6 s
```

Green in every frame means the hand can hold the ball at every instant of
the task. Read it with these three cautions:

- it is the hand's own hold, so green is the expected answer, not a
  finding;
- it is not a strength result. The record has no force data, and the
  motor in the built model works in the record's units, not newtons
  (issue 35 is that the result does not say so);
- the same run reports the base joint of each finger posed past its
  limit, and still reads green (issue 34).

The failure report says which fingers and joints to look at. The first
command below writes it as a page next to the result and prints the page's
name. The second writes one page for the whole run, and both margins as a
table:

```bash
python tools/report_failures.py outputs/follow_test_curve_hand__hold_test_curve_hand__self__selfoverlapfix__fixed.json
python tools/view_follow.py outputs/follow_test_curve_hand__hold_test_curve_hand__self__selfoverlapfix__fixed.json --out outputs/my_follow.html --table outputs/my_follow.md
```

Open `outputs/my_follow.html` and the `__failures.html` page in a browser.
The joints past a limit are in the failure report:

```text
  f0_hinge0: past its upper limit by 0.115 rad, frames 0 to 39 (40)
```

## 7. A hand you bring as a model file (about 103 s)

If you have a hand as a MuJoCo XML or URDF file, skip the actuator.
`docs/BRING_YOUR_HAND.md` is the full page; these are its steps on the
left Allegro hand that step 1 fetched. First look at the model:

```bash
python tools/add_hand.py external/mujoco_menagerie/wonik_allegro/left_hand.xml
```

It prints the bodies of the model and a description file to start from.
`examples/my_allegro_left.json` is that file filled in. Add the hand, make
its hold, run the check and write the failure report:

```bash
python tools/add_hand.py external/mujoco_menagerie/wonik_allegro/left_hand.xml examples/my_allegro_left.json
python tools/make_hold_leader.py my_allegro_left
python tools/run_follow.py --hands my_allegro_left --leader-id hold_my_allegro_left__self__selfoverlapfix --placement-from outputs/hold_placement_my_allegro_left__selfoverlapfix.json --tag fixed --margins
python tools/report_failures.py outputs/follow_my_allegro_left__hold_my_allegro_left__self__selfoverlapfix__fixed.json
```

You should see:

```text
my_allegro_left on hold_my_allegro_left__self__selfoverlapfix: {'green': 40}; red because {}; assignment {'thumb': 0, 'index': 1, 'middle': 2, 'ring': 3}; placement cost 0.00 mm; respaced by [0, 0, 0, 0] deg; 55.3 s
  no finger is amber or red, no joint is past a limit
```

The hand is called `my_allegro_left` here, not `allegro_left` as on the
full page, because the repository already holds the files that page's
steps would write, and the hold tool will not write over them.

## 8. Tidy up

```bash
python tools/add_hand.py --list
python tools/add_hand.py --remove test_curve_hand
python tools/add_hand.py --remove my_allegro_left
```

The first lists the hands you added. The others take them out again; the
task files and results stay where they are.

## What does not work today

| Where | What happens | Issue |
|---|---|---|
| Step 2 | The check of the first demo against the stored result lists differences | 37 |
| Step 3 | A record made from a joint table cannot be built into a hand | 33 |
| Step 4 | The hand file needs a file path, a contact radius and a bend direction that the docs do not give | 32 |
| Step 6 | A finger posed past a joint limit still reads green | 34 |
| Step 6 | The result for a hand built from a record does not carry the caution about its units | 35 |
| Not on this page | Drawing the records' Gramians (`tools/figure_gramians.py`) needs matplotlib, which nothing here installs | 36 |

The mesh route (an actuator as a CAD model, through simulation, to a
record) is hours of compute in another project and is not part of the
day. Its records are already in `data/records/`.
