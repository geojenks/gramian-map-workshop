"""Diagnostic: the Allegro free-cylinder turn with its torque cap lifted tenfold.

Simulation only: comparative, not predictive. Not a result about the hand: the
real cap is 0.5 N.m. This asks one question about the failed Allegro run of
tools/hands_turn_free_cylinder_lookahead_21nov.py: is the cap in this
simulator what stops it?

Everything is that tool's (controller, settings, pass test, seeds); the only
change is the cap, 0.5 -> 5 N.m. Written before the run: if all three seeds
now pass centre and tilt, the cap was what stopped it; if they still fail
centre or tilt, the controller is. The tool's own "at its limit" line now
counts against 5 N.m, so this file also prints the share of time any actuator
is above the real 0.5 N.m.

Use: python tools/allegro_turn_torque_cap_lifted_22nov.py --tag <new tag> --hands allegro --seeds 0,1,2 --no-film
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shadow_follows_hora_gait_mujoco_21nov as G  # noqa: E402
import hands_turn_free_cylinder_lookahead_21nov as K  # noqa: E402

REAL_CAP, LIFTED_CAP = G.HORA_TAU_MAX, 10 * G.HORA_TAU_MAX

if __name__ == "__main__":
    tag = sys.argv[sys.argv.index("--tag") + 1]
    G.HORA_TAU_MAX = LIFTED_CAP
    print(f"Allegro torque cap lifted from {REAL_CAP} to {LIFTED_CAP} N.m (diagnostic only)", flush=True)
    rc = K.main()
    out = os.path.join(G.ROOT, "outputs", "hands_turn_free_cylinder_lookahead_21nov_" + tag)
    Z = np.load(os.path.join(out, "runs.npz"))
    note = {"made_by": "tools/allegro_turn_torque_cap_lifted_22nov.py", "torque_cap_N_m": LIFTED_CAP,
            "real_cap_N_m": REAL_CAP, "runs": {}}
    for key in sorted(k[:-4] for k in Z.files if k.endswith("|act")):
        tq = np.abs(Z[key + "|act"]) * LIFTED_CAP
        over = float((tq > REAL_CAP).any(1).mean())
        note["runs"][key] = {"share_of_time_any_actuator_above_real_cap": over, "largest_torque_N_m": float(tq.max())}
        print(f"{key}: some actuator above {REAL_CAP} N.m {100 * over:.1f} % of the time; largest torque {tq.max():.2f} N.m")
    with open(os.path.join(out, "torque_cap_lifted_note.json"), "x", encoding="utf-8") as fo:
        json.dump(note, fo, indent=1)
    sys.exit(rc)
