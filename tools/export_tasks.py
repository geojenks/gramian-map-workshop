"""Write the workshop's task files from Wave_manip's banked W20 logs.

    python tools/export_tasks.py --wave-manip ../wave_manip

Reads only (never writes into Wave_manip). Every constant in a task file
is read from a banked log by key path, and the task file records the file,
its sha256, the commit and the key, so no number is typed by hand.

Tasks written:
  cup_hold  hold a ball in a stationary fingertip ring (W20 step 4, the
            stationary cup, banked 6 Oct): demand = the static weight.
  cup_turn  the same cup, turned about the vertical by a make/break wave
            whose duty is the banked ring's (--duty in the logged command).
"""
import argparse
import json
import os
import shlex
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _provenance import read_key, source_entry  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "data", "tasks")

CUP_RUNS = "w20_logs/w20s4_cup_runs.json"
CUP_READOUT = "w20_logs/w20s4_cup_readout.json"
S1_GATES = "w20_logs/w20s1_gates.json"
REGISTRATION = ("experiment_log/2026-10-04_hand_capability_readout_design.md "
                "section 15")

KEYS = {
    "mass_kg": (CUP_RUNS, "gates/G0/ball_run4/obj_mass_kg"),
    "radius_m": (CUP_RUNS, "gates/G0/ball_run4/radius_m"),
    "contact_altitude_deg": (
        CUP_RUNS, "cup/ball/pad_rows_contact_point_altitude_deg/median"),
    "mu": (CUP_READOUT, "contacts/ball/mu"),
    "gravity_m_s2": (S1_GATES, "G0/per_run/ring/gravity"),
    "L_m": (CUP_READOUT, "L_m"),
}


def flag_value(command, flag):
    toks = shlex.split(command, posix=False)
    return float(toks[toks.index(flag) + 1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wave-manip", required=True)
    a = ap.parse_args()
    wm = os.path.abspath(a.wave_manip)
    os.makedirs(OUT, exist_ok=True)

    vals, prov = {}, {}
    for name, (rel, key) in KEYS.items():
        vals[name] = read_key(os.path.join(wm, rel), key)
        prov[name] = source_entry(wm, rel, key)
    g = vals.pop("gravity_m_s2")
    vals["gravity_m_s2"] = abs(float(g[2]))

    cmd_key = "runs/3/command"
    cmd = read_key(os.path.join(wm, CUP_RUNS), cmd_key)
    duty = flag_value(cmd, "--duty")
    prov_duty = {**source_entry(wm, CUP_RUNS, cmd_key), "flag": "--duty"}

    obj = {"shape": "sphere", "radius_m": vals["radius_m"],
           "mass_kg": vals["mass_kg"]}
    common = {
        "object": obj,
        "mu": vals["mu"],
        "gravity_m_s2": vals["gravity_m_s2"],
        "torque_scale_L_m": vals["L_m"],
        "contact_ring": {"altitude_deg": vals["contact_altitude_deg"],
                         "note": "fingertips in one stationary ring below "
                                 "the centre (George, 6 Oct)"},
        "source": {"registration": REGISTRATION, "keys": prov},
        "caveat": "SIMULATION ONLY -- comparative, not predictive",
    }
    tasks = {
        "cup_hold": {"id": "cup_hold", "name": "Hold a ball in a cup",
                     "demand": {"kind": "static_weight"}, **common},
        "cup_turn": {"id": "cup_turn",
                     "name": "Turn a ball in a cup (make/break wave)",
                     "demand": {"kind": "static_weight",
                                "twist": {"axis": [0.0, 0.0, 1.0]}},
                     "wave": {"duty": duty, "source": prov_duty,
                              "duty_sensitivity": [0.5]},
                     **common},
    }
    for tid, t in tasks.items():
        p = os.path.join(OUT, f"{tid}.json")
        with open(p, "w", encoding="utf-8") as f:
            json.dump(t, f, indent=1)
        print(f"wrote {p}")
    print("values:", {**vals, "duty": duty})


if __name__ == "__main__":
    main()
