# The Shadow DEX-EE turning a free cylinder with sampled look-ahead control (22 Nov 2026)

Simulation only: comparative, not predictive. The controller knows the full state and plans with the same model. Printed by `tools/dexee_turns_free_cylinder_lookahead_22nov.py`; what is assumed and what was expected before the run are in its header, and the pass test is in the header of `tools/hands_turn_free_cylinder_lookahead_21nov.py`.

- Score settings: turn 0.2 rad, centre 0.01 m, tilt 0.2 rad, change weight 0.2, speed 0.1 m/s, wobble 2.0 rad/s
- Look-ahead 0.5 s, new choice every 0.02 s, 100 sampled changes, offsets spread 0.03 rad, 4 knots; hold 0.5 s before release; 10.0 s run; logged turn in that time -353 deg
- Recording: of thumb, first and ring, all three touch 37 % of the time, two 63 %, fewer 0.1 %; of all four fingers, three or more touch 100 %
- reference read from outputs\dexee_turns_free_cylinder_lookahead_22nov_smoke_1s_c\references.pkl
- dexee, no feedback: FAIL; held until 0.73 s; turned +72 deg (recording -353); centre moved at most 147 mm, axis tilted at most 73 deg; most time any actuator spent at its limit 0.0 %; other hand parts pressing 3 % of the time; controller's mean change to the commands 0.000 rad; took 0 s to compute
- failed: centre stays within 30 mm; axis stays within 30 deg; turn is 0.5 to 1.5 times the logged turn
- hardest-worked actuators: F2/J0_actuator largest 27 %, mean 13 % of range; F0/J0_actuator largest 18 %, mean 5 % of range; F1/J0_actuator largest 17 %, mean 12 % of range; F0/J2_actuator largest 14 %, mean 8 % of range
- fingertips touching, share of time: first 77 %, ring 70 %, thumb 80 %; cylinder's largest speed in the first 0.1 s 0.067 m/s; hardest-pressed actuator at release 17 % of range
- dexee, feedback, seed 0: FAIL; held until 9.12 s; turned -283 deg (recording -353); centre moved at most 32 mm, axis tilted at most 18 deg; most time any actuator spent at its limit 62.0 %; other hand parts pressing 55 % of the time; controller's mean change to the commands 0.106 rad; took 358 s to compute
- failed: centre stays within 30 mm; no actuator at its limit over 5 % of the time
- hardest-worked actuators: F0/J0_actuator largest 100 %, mean 57 % of range; F1/J0_actuator largest 100 %, mean 70 % of range; F1/J1_actuator largest 100 %, mean 51 % of range; F0/J3_actuator largest 94 %, mean 28 % of range
- fingertips touching, share of time: first 58 %, ring 100 %, thumb 36 %; cylinder's largest speed in the first 0.1 s 0.064 m/s; hardest-pressed actuator at release 17 % of range
- dexee, feedback, seed 1: FAIL; held until 9.00 s; turned -263 deg (recording -353); centre moved at most 33 mm, axis tilted at most 28 deg; most time any actuator spent at its limit 60.4 %; other hand parts pressing 58 % of the time; controller's mean change to the commands 0.133 rad; took 320 s to compute
- failed: centre stays within 30 mm; no actuator at its limit over 5 % of the time
- hardest-worked actuators: F0/J0_actuator largest 100 %, mean 51 % of range; F0/J3_actuator largest 100 %, mean 41 % of range; F1/J1_actuator largest 100 %, mean 63 % of range; F1/J0_actuator largest 100 %, mean 69 % of range
- fingertips touching, share of time: first 57 %, ring 98 %, thumb 15 %; cylinder's largest speed in the first 0.1 s 0.059 m/s; hardest-pressed actuator at release 17 % of range
- dexee, feedback, seed 2: FAIL; held until 10.00 s; turned -332 deg (recording -353); centre moved at most 19 mm, axis tilted at most 17 deg; most time any actuator spent at its limit 37.6 %; other hand parts pressing 30 % of the time; controller's mean change to the commands 0.075 rad; took 344 s to compute
- failed: no actuator at its limit over 5 % of the time
- hardest-worked actuators: F0/J0_actuator largest 100 %, mean 36 % of range; F1/J0_actuator largest 100 %, mean 50 % of range; F1/J1_actuator largest 100 %, mean 30 % of range; F0/J3_actuator largest 83 %, mean 27 % of range
- fingertips touching, share of time: first 82 %, ring 95 %, thumb 58 %; cylinder's largest speed in the first 0.1 s 0.101 m/s; hardest-pressed actuator at release 17 % of range
