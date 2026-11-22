# Hands turning a free cylinder with sampled look-ahead control (21 Nov 2026)

Simulation only: comparative, not predictive. The controller knows the full state and plans with the same model. Printed by `tools/hands_turn_free_cylinder_lookahead_21nov.py`; the pass test and what is assumed are in its header.

- Score settings: turn 0.2 rad, centre 0.01 m, tilt 0.2 rad, change weight 0.2, speed 0.1 m/s, wobble 2.0 rad/s
- Look-ahead 0.5 s, new choice every 0.02 s, 100 sampled changes, offsets spread 0.03 rad, 4 knots; hold 0.5 s before release; 10.0 s run; logged turn in that time -353 deg
- Allegro base fit to the logged link positions: mean miss 0.00 mm, largest 0.00 mm (40 moments, 17 links)
- Allegro ball fingertip against the logged cylinder, whenever the log has that finger touching: gap median -0.15 mm, from -0.44 to +2.12 mm (3715 samples; negative is overlap)
- Allegro commands rebuilt from logged torque; logged torque at its 0.5 N.m cap in 0.0 % of joint samples
- allegro, no feedback: FAIL; held until 0.22 s; turned -97 deg (recording -353); centre moved at most 141 mm, axis tilted at most 137 deg; most time any actuator spent at its limit 0.0 %; other hand parts pressing 0 % of the time; controller's mean change to the commands 0.000 rad; took 0 s to compute
- failed: centre stays within 30 mm; axis stays within 30 deg; turn is 0.5 to 1.5 times the logged turn
- hardest-worked actuators: rfa0 largest 50 %, mean 23 % of range; mfa0 largest 43 %, mean 10 % of range; ffa0 largest 42 %, mean 19 % of range; rfa3 largest 39 %, mean 9 % of range
- allegro, feedback, seed 0: FAIL; held until 1.99 s; turned -342 deg (recording -353); centre moved at most 31 mm, axis tilted at most 30 deg; most time any actuator spent at its limit 5.8 %; other hand parts pressing 0 % of the time; controller's mean change to the commands 0.122 rad; took 101 s to compute
- failed: centre stays within 30 mm; axis stays within 30 deg; no actuator at its limit over 5 % of the time
- hardest-worked actuators: ffa0 largest 100 %, mean 37 % of range; mfa3 largest 100 %, mean 16 % of range; mfa1 largest 100 %, mean 33 % of range; mfa0 largest 100 %, mean 27 % of range
