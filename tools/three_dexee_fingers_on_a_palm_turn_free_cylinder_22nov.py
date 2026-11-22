"""Three Shadow DEX-EE fingers on a palm of our layout, turning a free cylinder.

Simulation only: comparative, not predictive. Not a product hand: the layout
is ours.

Why: two things changed between the three-finger run that failed
(tools/dexee_turns_free_cylinder_lookahead_22nov.py: the stock base, whose
three mounts are fixed relative to each other) and the four-finger run that
passed (tools/four_dexee_fingers_on_a_palm_turn_free_cylinder_22nov.py: four
fingers, each mount fitted freely with a clearance rule). This run changes
only one of them: three fingers, mounts fitted freely by the four-finger
tool's layout rule, unchanged. The three fingers follow the recording's
first, ring and thumb, as in the failed three-finger run.

Everything is the four-finger tool's (read its header): layout rule,
fingertip targets, controller, settings, pass test. That tool was given a
finger count for this; its four-finger result was repeated afterwards and the
repeat is named in the report of this run's folder if it differs.

Written before the first run. In the recording, of these three fingers all
three touch 37 % of the time and only two 63 %. The stock-base run still kept
the cylinder 9 to 10 s; it failed on actuators at their force limit (38 to
62 % of the time) with other hand parts pressing on the cylinder (30 to 58 %).
Our guess: with free mounts nothing but fingertips presses and the force-limit
line passes, but holding on two contacts leaves less margin than four
fingers had, so one seed fails centre or tilt: two of three seeds pass.
What each outcome would say:
- all three pass: the stock base's layout, not the finger count, is what
  failed before;
- fails on force limit or other parts pressing: the layout was not the whole
  cause;
- fails on centre, tilt or dropping, with no pressing and no force limit:
  three fingers are too few for this gait with this controller.

Use: python tools/three_dexee_fingers_on_a_palm_turn_free_cylinder_22nov.py --tag <new tag> [--seeds 0,1,2] [--seconds 10]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import four_dexee_fingers_on_a_palm_turn_free_cylinder_22nov as F  # noqa: E402

if __name__ == "__main__":
    F.set_fingers([0, 2, 3], "three", "three_dexee_fingers_on_a_palm_turn_free_cylinder_22nov.py",
                  "three_dexee_fingers_on_a_palm_turn_free_cylinder_22nov_", "three_dexee_fingers_on_a_palm_22nov")
    sys.exit(F.main())
