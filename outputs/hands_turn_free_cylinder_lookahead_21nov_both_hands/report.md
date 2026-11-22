# Hands turning a free cylinder with sampled look-ahead control (21 Nov 2026)

Simulation only: comparative, not predictive. The controller knows the full state and plans with the same model. Printed by `tools/hands_turn_free_cylinder_lookahead_21nov.py`; the pass test and what is assumed are in its header.

- Score settings: turn 0.2 rad, centre 0.01 m, tilt 0.2 rad, change weight 0.2, speed 0.1 m/s, wobble 2.0 rad/s
- Look-ahead 0.5 s, new choice every 0.02 s, 100 sampled changes, offsets spread 0.03 rad, 4 knots; hold 0.5 s before release; 10.0 s run; logged turn in that time -353 deg
- Allegro base fit to the logged link positions: mean miss 0.00 mm, largest 0.00 mm (40 moments, 17 links)
- Allegro ball fingertip against the logged cylinder, whenever the log has that finger touching: gap median -0.15 mm, from -0.44 to +2.12 mm (3715 samples; negative is overlap)
- Allegro commands rebuilt from logged torque; logged torque at its 0.5 N.m cap in 0.0 % of joint samples
- Shadow base fit: cost 1.966e-01 at the first guess, 4.042e-03 after the search (m^2, 30 moments)
- reach, Shadow first finger: mean miss 3.4 mm, largest 14.6 mm; at a joint limit 68 % of the time
- reach, Shadow middle finger: mean miss 1.4 mm, largest 9.0 mm; at a joint limit 43 % of the time
- reach, Shadow ring finger: mean miss 3.6 mm, largest 16.8 mm; at a joint limit 67 % of the time
- reach, Shadow thumb finger: mean miss 1.7 mm, largest 11.2 mm; at a joint limit 8 % of the time
- other hand parts cut into the logged cylinder by at most 7.4 mm (mean 1.6 mm)
- allegro, no feedback: FAIL; held until 0.22 s; turned -97 deg (recording -353); centre moved at most 141 mm, axis tilted at most 137 deg; most time any actuator spent at its limit 0.0 %; other hand parts pressing 0 % of the time; controller's mean change to the commands 0.000 rad; took 0 s to compute
- failed: centre stays within 30 mm; axis stays within 30 deg; turn is 0.5 to 1.5 times the logged turn
- hardest-worked actuators: rfa0 largest 50 %, mean 23 % of range; mfa0 largest 43 %, mean 10 % of range; ffa0 largest 42 %, mean 19 % of range; rfa3 largest 39 %, mean 9 % of range
- allegro, feedback, seed 0: FAIL; held until 1.99 s; turned -342 deg (recording -353); centre moved at most 31 mm, axis tilted at most 30 deg; most time any actuator spent at its limit 5.8 %; other hand parts pressing 0 % of the time; controller's mean change to the commands 0.122 rad; took 157 s to compute
- failed: centre stays within 30 mm; axis stays within 30 deg; no actuator at its limit over 5 % of the time
- hardest-worked actuators: ffa0 largest 100 %, mean 37 % of range; mfa3 largest 100 %, mean 16 % of range; mfa1 largest 100 %, mean 33 % of range; mfa0 largest 100 %, mean 27 % of range
- allegro, feedback, seed 1: FAIL; held until 3.87 s; turned -350 deg (recording -353); centre moved at most 33 mm, axis tilted at most 33 deg; most time any actuator spent at its limit 5.1 %; other hand parts pressing 0 % of the time; controller's mean change to the commands 0.110 rad; took 130 s to compute
- failed: centre stays within 30 mm; axis stays within 30 deg; no actuator at its limit over 5 % of the time
- hardest-worked actuators: ffa0 largest 100 %, mean 24 % of range; mfa1 largest 100 %, mean 41 % of range; tha1 largest 100 %, mean 33 % of range; tha2 largest 100 %, mean 25 % of range
- allegro, feedback, seed 2: FAIL; held until 3.14 s; turned -344 deg (recording -353); centre moved at most 48 mm, axis tilted at most 40 deg; most time any actuator spent at its limit 11.6 %; other hand parts pressing 0 % of the time; controller's mean change to the commands 0.138 rad; took 143 s to compute
- failed: centre stays within 30 mm; axis stays within 30 deg; no actuator at its limit over 5 % of the time
- hardest-worked actuators: ffa0 largest 100 %, mean 26 % of range; ffa1 largest 100 %, mean 30 % of range; ffa3 largest 100 %, mean 13 % of range; mfa0 largest 100 %, mean 20 % of range
- shadow_tied, no feedback: FAIL; held until 1.46 s; turned -89 deg (recording -353); centre moved at most 24 mm, axis tilted at most 74 deg; most time any actuator spent at its limit 0.0 %; other hand parts pressing 28 % of the time; controller's mean change to the commands 0.000 rad; took 2 s to compute
- failed: axis stays within 30 deg; turn is 0.5 to 1.5 times the logged turn
- hardest-worked actuators: rh_A_RFJ4 largest 43 %, mean 9 % of range; rh_A_THJ2 largest 28 %, mean 6 % of range; rh_A_RFJ3 largest 27 %, mean 7 % of range; rh_A_MFJ3 largest 23 %, mean 4 % of range
- shadow_tied, feedback, seed 0: FAIL; held until 9.13 s; turned -287 deg (recording -353); centre moved at most 21 mm, axis tilted at most 31 deg; most time any actuator spent at its limit 0.0 %; other hand parts pressing 32 % of the time; controller's mean change to the commands 0.076 rad; took 306 s to compute
- failed: axis stays within 30 deg
- hardest-worked actuators: rh_A_RFJ4 largest 63 %, mean 12 % of range; rh_A_FFJ3 largest 55 %, mean 17 % of range; rh_A_FFJ0 largest 41 %, mean 8 % of range; rh_A_THJ2 largest 39 %, mean 8 % of range
- shadow_tied, feedback, seed 1: PASS; held until 10.00 s; turned -331 deg (recording -353); centre moved at most 16 mm, axis tilted at most 17 deg; most time any actuator spent at its limit 0.0 %; other hand parts pressing 49 % of the time; controller's mean change to the commands 0.062 rad; took 275 s to compute
- hardest-worked actuators: rh_A_RFJ4 largest 60 %, mean 13 % of range; rh_A_FFJ3 largest 38 %, mean 15 % of range; rh_A_THJ2 largest 38 %, mean 7 % of range; rh_A_RFJ3 largest 35 %, mean 11 % of range
- shadow_tied, feedback, seed 2: FAIL; held until 6.78 s; turned -341 deg (recording -353); centre moved at most 20 mm, axis tilted at most 43 deg; most time any actuator spent at its limit 0.0 %; other hand parts pressing 31 % of the time; controller's mean change to the commands 0.073 rad; took 285 s to compute
- failed: axis stays within 30 deg
- hardest-worked actuators: rh_A_RFJ4 largest 59 %, mean 10 % of range; rh_A_FFJ0 largest 39 %, mean 4 % of range; rh_A_THJ2 largest 39 %, mean 7 % of range; rh_A_FFJ3 largest 32 %, mean 9 % of range
