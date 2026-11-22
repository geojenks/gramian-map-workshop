# Hands turning a free cylinder with sampled look-ahead control (21 Nov 2026)

Simulation only: comparative, not predictive. The controller knows the full state and plans with the same model. Printed by `tools/hands_turn_free_cylinder_lookahead_21nov.py`; the pass test and what is assumed are in its header.

- Score settings: turn 0.2 rad, centre 0.01 m, tilt 0.2 rad, change weight 0.2, speed 0.1 m/s, wobble 2.0 rad/s
- Look-ahead 0.5 s, new choice every 0.02 s, 100 sampled changes, offsets spread 0.03 rad, 4 knots; hold 0.5 s before release; 10.0 s run; logged turn in that time -353 deg
- Allegro base fit to the logged link positions: mean miss 0.00 mm, largest 0.00 mm (40 moments, 17 links)
- Allegro ball fingertip against the logged cylinder, whenever the log has that finger touching: gap median -0.15 mm, from -0.44 to +2.12 mm (3715 samples; negative is overlap)
- Allegro commands rebuilt from logged torque; logged torque at its 5.0 N.m cap in 0.0 % of joint samples
- allegro, no feedback: FAIL; held until 0.22 s; turned -97 deg (recording -353); centre moved at most 141 mm, axis tilted at most 137 deg; most time any actuator spent at its limit 0.0 %; other hand parts pressing 0 % of the time; controller's mean change to the commands 0.000 rad; took 0 s to compute
- failed: centre stays within 30 mm; axis stays within 30 deg; turn is 0.5 to 1.5 times the logged turn
- hardest-worked actuators: rfa0 largest 5 %, mean 2 % of range; mfa0 largest 4 %, mean 1 % of range; ffa0 largest 4 %, mean 2 % of range; rfa3 largest 4 %, mean 1 % of range
- allegro, feedback, seed 0: FAIL; held until 1.70 s; turned -350 deg (recording -353); centre moved at most 49 mm, axis tilted at most 37 deg; most time any actuator spent at its limit 0.0 %; other hand parts pressing 0 % of the time; controller's mean change to the commands 0.102 rad; took 104 s to compute
- failed: centre stays within 30 mm; axis stays within 30 deg
- hardest-worked actuators: mfa1 largest 25 %, mean 4 % of range; tha2 largest 21 %, mean 3 % of range; tha3 largest 20 %, mean 2 % of range; mfa2 largest 17 %, mean 3 % of range
- allegro, feedback, seed 1: FAIL; held until 4.54 s; turned -350 deg (recording -353); centre moved at most 38 mm, axis tilted at most 67 deg; most time any actuator spent at its limit 0.0 %; other hand parts pressing 0 % of the time; controller's mean change to the commands 0.105 rad; took 108 s to compute
- failed: centre stays within 30 mm; axis stays within 30 deg
- hardest-worked actuators: mfa3 largest 35 %, mean 3 % of range; tha1 largest 30 %, mean 6 % of range; mfa0 largest 26 %, mean 4 % of range; mfa1 largest 26 %, mean 9 % of range
- allegro, feedback, seed 2: FAIL; held until 2.34 s; turned -356 deg (recording -353); centre moved at most 33 mm, axis tilted at most 29 deg; most time any actuator spent at its limit 0.0 %; other hand parts pressing 0 % of the time; controller's mean change to the commands 0.104 rad; took 107 s to compute
- failed: centre stays within 30 mm
- hardest-worked actuators: rfa3 largest 20 %, mean 2 % of range; mfa0 largest 18 %, mean 2 % of range; ffa0 largest 17 %, mean 3 % of range; ffa3 largest 17 %, mean 1 % of range
