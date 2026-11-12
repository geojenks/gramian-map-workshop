# Plan for the landing page

Written 12 Nov 2026, and reworked the same day after the two-panel mock-up.
It sits beside `docs/MINIMAL_PRODUCT_PLAN.md`: that plan lists what must
work for someone to bring an actuator or a hand and get a verdict; this one
is the page they do it from.

The mock-up is `tools/make_landing_mockup.py`. Its pictures come from
`tools/render_landing_pictures.py` (`outputs/landing_pictures_12nov/`), and
its views of the target beside the attempt from
`tools/render_landing_tasks.py` (`outputs/landing_renders_12nov/`). The
pages are in `outputs/landing_mockup_12nov_c/`; the pages before the views
are kept in `outputs/landing_mockup_12nov_b/`, and the first, one-column
mock-up in `outputs/landing_mockup_12nov/`.

Nothing here is approved work yet. Each work package below starts when we
say so.

## What the page is

Two panels and an answer.

- **Left: hands.** Scroll through the hands we have, each as a picture
  drawn from its model. The last card is "add a hand", with two ways in:
  load a model file (URDF or MuJoCo XML), or make one in the hand builder
  below.
- **Right: tasks.** Scroll through the tasks. Each card has a plain name
  and, for now, a drawing made from the task file; a recorded render of a
  hand doing the task is to come. Variants of one task share a card. The
  last card is "add a task", with extraction from a human demonstration
  marked beta.
- **The answer.** Once a hand and a task are chosen: can this hand do this
  task by this method, frame by frame, and how much headroom is left. With
  a render of the pair where one is stored.
- **Make your manipulator.** The hand builder, with the actuator map.
- **Along the bottom: how this works under the hood.** The steps that go
  into the verdict, in order. Each is to get its worked example.

The page should lead people to the hand builder and the actuator map. That
is the part no one else offers.

## The hand builder ("make your manipulator")

Put characterised actuators onto a hand arrangement: start from an empty
palm, or take a hand we have and swap its fingers.

What we must say on the page, plainly:

- Each finger is modelled on its own, from its record. Coupling between
  fingers through the palm or a shared supply is not modelled.
- A soft finger enters as a stand-in fitted to its record (hinges, springs
  and one drive). It is a first approximation, as good as the record.
- Someone uploading their own hand has to give us one record per finger,
  or a model file.

Where it stands in the tool:

| Piece | Today |
|---|---|
| A hand from records on an empty palm | Works from a hand file by command (`tools/build_record_hand.py`); needs choices the docs do not give (issue 32) |
| A record from test curves | Works; has no force data, so it cannot push |
| A record from a table of joints | Makes a record; the hand build crashes on it (issue 33) |
| Swapping one finger of a model-file hand for a record | Not there |
| Strength verdicts for a record-built hand | Provisional: the motor works in the record's drive level, not newtons, and waits on the units ruling (issues 31, 35) |
| A form in the browser that writes the hand file | Not there (`forms/record_form.html` does this for a record) |

## The actuator map

The choice of each finger's actuator should, in the end, be made on the
actuator map (the atlas of the actuator-map project): one point per
characterised actuator, placed by how alike their capabilities are, with
named capability axes. A visitor says what the finger has to do and the map
shows which actuators can do it.

People must also be able to put their own actuator on the map, including a
new kind that fits none of the families there. That is how the map grows,
and how a visitor finds out what their actuator is like.

Where it stands:

| Piece | Today |
|---|---|
| The atlas | A page in the actuator-map project (`gramian_map/presentation/actuator_atlas.html`). The mock-up shows a picture of it, not the live page |
| Records the tool can build fingers from | The 27 records of `data/records/` here, which are the same, file for file, as the 27 records of the actuator-map project (checked 12 Nov, line endings aside) |
| Choosing a finger's actuator from the map | Not there. Today the hand file names a record by hand |
| A visitor's actuator as a record here | Works by command, from test curves, a table of joints or the form |
| A visitor's actuator on the atlas | Not there. The atlas is built from the actuator-map project's own records |

Two things need a ruling before this is work:

- A record made here from a visitor's test curves has less in it than a
  record the actuator-map project makes from its own simulations. Placing
  it on the atlas means either a rule for what a lesser record may show, or
  a change to that project's record format. That format changes only with
  explicit sign-off.
- Where a visitor's record lives: the records here are a copy of the
  actuator-map project's, so a record added here is not on the atlas
  until something carries it across.

## What the page can and cannot do by itself

The page is static. It shows stored results and renders, and it writes
files. It does not run the check.

- **A stored pair** shows its verdict and render at once.
- **A new hand or task** gives the visitor a file and the commands to run
  on their own machine. The check of a hand's own hold took about 100 s in
  the participant's script; a task that needs a search for where to place
  the hand took 3260 s for the one stored example.
- **Rendering their own.** MuJoCo is what the tool uses, so that route is
  `tools/render3d.py` on their result. SOFA is how the mesh records are
  made, in the actuator-map project, and is hours of compute. For any other
  solver (COMSOL and the like) the way in is the record or the test-curve
  file, not a backend we drive: we take what their solver gives, in our
  format.

Later (ruled 12 Nov): the page is to run from a local server on the
visitor's machine, which runs the check, feeds the static page, and, if the
visitor chooses, sends their hand to the repository as a merge request. Not
a work package yet.

## Samples the page opens with

Today the page has one stored result for a task that is not a hand's own
hold, and it is red and provisional. The page needs a small set of pairs
with real answers, some green, some not:

- static grips, with force control, on the model-file hands;
- the cup wave and the tennis-ball demonstration on at least two hands;
- one record-built hand on each, once the units ruling is made.

This is compute, and it is the longest job in the plan.

## What we claim

The strong line is: classical grasp analysis looks at rigid hands holding
still; this asks the same question of soft, characterised fingers at every
instant of a moving task.

Two cautions before that goes on the page:

- Issue 15 is our own rule: no result is called dynamic until continuity
  and at least one of speed, inertia, torque at speed, impacts or the
  replay land (issues 12, 13, 14). Today each frame is checked on its own.
  Until then the honest wording is "a moving task, checked frame by frame".
  The mock-up's test fails if the page says otherwise.
- "First of its kind" needs a pass through `docs/LITERATURE_REVIEW.md`
  before it is written anywhere.

And the trap we name ourselves: the answer is only as good as the finger
model. We say so on the page, with the fit check of each finger shown.

## Work packages (draft)

Each is small enough for one agent, can be judged by eye or by a named
check when it lands, and owns files no other package owns. Only the view of
the target beside the attempt is started (see its entry). For all of them: `gramian_workshop/follow.py` is not to be edited (changes
it needs go to its owner as an issue); stored outputs are never written
over; nothing is called dynamic; no hand-typed numbers.

**The page itself.** One owner, who folds in what the others make.
- Done when: the page shows whatever pictures, renders and worked examples
  are stored, and says so plainly where one is missing.
- Owns: `tools/make_landing_mockup.py`, `tools/render_landing_pictures.py`,
  `tests/test_landing_mockup.py`, this plan.
- Blocked by: nothing. Machine: laptop.
- Check: `python -m unittest tests.test_landing_mockup`, and the page at
  desktop and phone width.

**Target beside attempt, for each task card and each stored pair.**
- What it shows: the task is checked at a set of still moments. For each,
  the target on one side (the video still where the task has one, else the
  task's contacts drawn on its object) and on the other the chosen hand in
  the pose the check solved, with how far each finger is from its contact,
  the force at each contact and the resultant on the object. All of this is
  in the task file and the stored result already; `tools/render3d.py`
  draws the hand, the contacts and the video still today, not yet the
  forces.
- Done when: a new folder holds that view per task and per stored pair,
  with a list file the page tool can read, as the pictures folder does.
- Owns: a new tool `tools/render_landing_tasks.py`, its test, a new
  `outputs/landing_renders_<date>/`.
- Must not touch: `tools/render3d.py` (call it), the page tool.
- Blocked by: nothing. Machine: laptop (needs MuJoCo).
- Check: its test; the renders by eye.
- Built 12 Nov, for stored pairs: `tools/render_landing_tasks.py` writes one
  page per stored result of a hand on a task, a still of it and
  `renders.json`; the page tool shows the still and links the page.
  `tests/test_render_landing_tasks.py` works every number shown out again
  from the result and task files. One pair is stored today, so there is one
  view. The supplied forces carry the result's own caution (drive level,
  not newtons, until the units ruling). The sums are of the contact forces
  drawn; the object's weight is not in them.
- Still to do: a view per task card with no hand chosen, and the video
  still as the target where the task has one.

**The sample pairs.** Compute only, no code.
- Done when: the stored results cover the list under "Samples the page
  opens with" for the model-file hands, run with `--margins`, and
  `tools/list_superseded_follow.py` has been re-run.
- Owns: new `outputs/follow_*` files, `outputs/follow__superseded.md`.
- Must not touch: any tool or test.
- Blocked by: nothing for model-file hands; the units ruling for
  record-built hands. Machine: the CPU box, one job at a time.
- Check: the full test suite; the page's counts change and nothing else.

**Worked examples for "how this works".**
- Done when: each step on the page has one worked example read from a
  stored result (one frame, one finger, the numbers that decided it).
- Owns: a new tool `tools/make_how_it_works.py`, its test, a new
  `outputs/how_it_works_<date>/`.
- Must not touch: `docs/MARGINS_WORKED_CHECK.md`, the walkthrough tool.
- Blocked by: nothing. Machine: laptop.
- Check: its test, which compares every number shown with the result file.

**Make the record-built hand work as documented** (issues 32 and 33).
- Done when: someone following `docs/BRING_YOUR_HAND.md` builds a hand
  from records without a choice the docs do not give, and a record from a
  table of joints builds without a crash.
- Owns: `tools/build_record_hand.py`, `tools/joint_table_to_record.py`,
  `docs/BRING_YOUR_HAND.md`, their tests.
- Blocked by: nothing. Machine: laptop.
- Check: the full test suite; `tools/run_participant_script.py`.

**The hand builder as a form.**
- Done when: a page in the browser lays out a palm, names a record per
  finger, and writes a hand file and the commands to run.
- Owns: a new `forms/hand_form.html`, its test.
- Must not touch: `forms/record_form.html`.
- Blocked by: the package above. Machine: laptop.
- Check: its test; a hand file it writes builds and passes its own hold.

**Swap one finger of a model-file hand for a record.** New work in the tool.
- Done when: a design note is agreed, then a command takes a hand of the
  hands table, a finger and a record, and writes a new hand that passes
  its own hold.
- Owns: a new module and tool, their tests. The hands table itself changes
  only by agreement.
- Blocked by: the design note. Machine: laptop.
- Check: the full test suite.

**Choose an actuator from the records here.** The first step toward the
map, inside this repository.
- Done when: a static page lists the records of `data/records/` with their
  stored capabilities, takes what a finger has to do (reach, force, number
  of drives) and shortlists the records that meet it.
- Owns: a new tool `tools/make_actuator_chooser.py`, its test, a new
  output folder.
- Must not touch: the records, `tools/check_record.py`.
- Blocked by: nothing; force columns stay provisional until the units
  ruling. Machine: laptop.
- Check: its test, which recomputes the shortlist from the records.

**A visitor's actuator on the atlas.** In the actuator-map project, not
here. Not approved; needs the two rulings under "The actuator map".
- Done when: a record made here (from test curves or a table of joints)
  can be given to the atlas builder and appears as a point, marked as
  visitor-supplied and with what it lacks stated.
- Owns: a new import step in the actuator-map project and its report
  there. No record of that project is edited.
- Blocked by: sign-off on how a lesser record is shown. Machine: laptop.
- Check: that project's `population_report.py`, and the atlas by eye.

**The claim.**
- Done when: a short note says, from `docs/LITERATURE_REVIEW.md` and a
  fresh search, what the nearest published work does and what wording we
  can defend.
- Owns: a new `docs/LANDING_CLAIM_CHECK.md`.
- Blocked by: nothing. Machine: any.
- Check: every statement about a paper carries its reference.

**A task from a single video, beta.** Last, and the least certain.
- Done when: a design note says what a single camera can and cannot give
  (contact points, timing; not forces), before any code.
- Owns: the note. Machine: any.

Order: the sample pairs and the renders make the page worth opening; the
record-built hand and the form make the builder real; the chooser and the
atlas package make the actuator map part of it. The first four packages
share no files and can run side by side.

## Open rulings

- Force units for record-built hands (issues 31, 35). Blocks headroom
  numbers for the hand builder.
- How a visitor's lesser record is shown on the atlas, and where such a
  record lives.
- Ruled 12 Nov: the page says "a moving task, checked frame by frame" for
  now. Issues 12, 13 and 14, which would lift this, are open and unstarted.
- Whether the hands carrying a person's name go on the page.
