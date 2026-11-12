# Plan for the landing page

Written 12 Nov 2026. It sits beside `docs/MINIMAL_PRODUCT_PLAN.md`: that
plan lists what must work for someone to bring an actuator or a hand and
get a verdict; this one is the page they do it from. The first mock-up is
`tools/make_landing_mockup.py` (pictures in `outputs/landing_mockup_12nov/`).

Nothing here is approved work yet. Each line under "Order of work" starts
when we say so.

## What the page is

Two panels and an answer.

- **Left: hands.** Scroll through the hands we have, each as a rendered
  picture. The last card is "add a hand", with two ways in: load a model
  file (URDF or MuJoCo XML), or make one in the hand builder below.
- **Right: tasks.** Scroll through recorded renders of a simulated hand or
  a human hand doing each task. The last card is "add a task", with
  extraction from a human demonstration marked beta.
- **The answer.** Once a hand and a task are chosen: can this hand do this
  task by this method, frame by frame, and how much headroom is left. With
  a render of the pair where one is stored.
- **Along the bottom: how this works under the hood.** The steps that go
  into the verdict, in order, each with its worked example.

The page should lead people to the hand builder. That is the part no one
else offers.

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
  made, in the M-11 project, and is hours of compute. For any other solver
  (COMSOL and the like) the way in is the record or the test-curve file,
  not a backend we drive: we take what their solver gives, in our format.

A server that runs the check for visitors is a separate decision and is not
planned here.

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
- "First of its kind" needs a pass through `docs/LITERATURE_REVIEW.md`
  before it is written anywhere.

And the trap we name ourselves: the answer is only as good as the finger
model. We say so on the page, with the fit check of each finger shown.

## Order of work

Each is a piece that can be judged by eye when it lands.

- Rework the mock-up to the two panels: cards with pictures, the two "add"
  cards, the answer, the bottom strip. Pictures of the hands from their
  models; plain names for the tasks.
- A render for each task card, and for each stored pair, from
  `tools/render3d.py`.
- The "how this works" pages, built from the worked checks we have
  (`docs/MARGINS_WORKED_CHECK.md`, the walkthrough).
- The sample pairs (compute; on the box).
- The hand builder as a form that writes a hand file and shows the
  commands. Needs issues 32 and 33 closed first.
- Swapping a finger on a model-file hand. New work in the tool.
- Task from a video, beta. Today a demonstration comes in from
  three-camera keypoints (`tools/import_demo_leader.py`); a single video is
  not there.

## Open rulings

- Force units for record-built hands (issues 31, 35). Blocks headroom
  numbers for the hand builder.
- Whether the page may say "dynamic" before issue 15's bar is met.
- Whether the hands carrying a person's name go on the page.
- Whether we want a server that runs checks for visitors, or files and
  commands only.
