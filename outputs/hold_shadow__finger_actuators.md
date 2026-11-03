# shadow: self-hold with each finger as an actuator

Leader `hold_shadow__self`, fixed placement, pose from `outputs/follow_shadow__hold_shadow__self__fixed.json`, variants as_modelled, no_joint_friction (joint dry friction as modelled / zeroed). Written by `tools/hold_shadow_finger_actuators.py` from `outputs/hold_shadow__finger_actuators__<variant>.json`. A finger holds its share when its settled force on the object is within RESID_TOL (0.05) of the leader's and its contact within REACH_AMBER_M (10 mm) of the station (green: within 2 mm). The hand row sums the assembled fingers' forces to a wrench about the object's centre (torques over the radius) against the leader's, and releases the sphere under the leader's net station force reversed as its weight; held means the centre drifts at most DEFLECT_MAX_M (2 mm) in 1 s.

**The Shadow does not pass the finger-as-actuator check.** Frames failing: as_modelled frame 0: fingers not holding ['index', 'ring', 'thumb'], wrench ok False, release held False; as_modelled frame 1: fingers not holding ['index', 'ring', 'thumb'], wrench ok False, release held False; no_joint_friction frame 0: fingers not holding ['index', 'middle', 'ring', 'little', 'thumb'], wrench ok False, release held False; no_joint_friction frame 1: fingers not holding ['index', 'middle', 'ring', 'little', 'thumb'], wrench ok False, release held False. Reasons by finger: index (direction), little (direction), middle (direction), ring (direction), thumb (direction).

## Per finger

| variant | frame | finger | holds | why | demanded N | achieved N | force departure N | angle deg | point departure mm | start point departure mm | joints at stop (N.m) | friction max N.m | press input (note) | J0 tendon input rad | saturated | evals | s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| as_modelled | 0 | index | no | direction | 0.1000 | 0.0995 | 0.0200 | 11.5 | 6.35 | 1.04 | - | 0.0008 | rh_A_FFJ3 + (ok) | 2.211 of 3.142 | - | 346 | 66.8 |
| as_modelled | 0 | middle | yes (amber) | - | 0.1000 | 0.0996 | 0.0028 | 1.6 | 6.77 | 0.67 | rh_MFJ1 hi -0.0017 | 0.0002 | rh_A_MFJ3 + (ok) | 2.169 of 3.142 | - | 124 | 22.0 |
| as_modelled | 0 | ring | no | direction | 0.1000 | 0.0995 | 0.0202 | 11.6 | 5.99 | 1.28 | - | 0.0008 | rh_A_RFJ3 + (ok) | 2.197 of 3.142 | - | 320 | 72.3 |
| as_modelled | 0 | little | yes (amber) | - | 0.1000 | 0.1001 | 0.0044 | 2.5 | 2.83 | 1.21 | rh_LFJ4 lo +0.0014; rh_LFJ1 hi -0.0015 | 0.0003 | rh_A_LFJ3 + (ok) | 2.202 of 3.142 | - | 530 | 14.3 |
| as_modelled | 0 | thumb | no | direction | 0.1000 | 0.1001 | 0.0132 | 7.6 | 3.91 | 3.84 | - | 0.0002 | rh_A_THJ4 + (ok) | - | - | 609 | 19.5 |
| as_modelled | 1 | index | no | direction | 0.1000 | 0.0995 | 0.0200 | 11.5 | 6.35 | 1.04 | - | 0.0008 | rh_A_FFJ3 + (ok) | 2.211 of 3.142 | - | 346 | 71.7 |
| as_modelled | 1 | middle | yes (amber) | - | 0.1000 | 0.0996 | 0.0028 | 1.6 | 6.77 | 0.67 | rh_MFJ1 hi -0.0017 | 0.0002 | rh_A_MFJ3 + (ok) | 2.169 of 3.142 | - | 124 | 25.1 |
| as_modelled | 1 | ring | no | direction | 0.1000 | 0.0995 | 0.0202 | 11.6 | 5.99 | 1.28 | - | 0.0008 | rh_A_RFJ3 + (ok) | 2.197 of 3.142 | - | 320 | 71.0 |
| as_modelled | 1 | little | yes (amber) | - | 0.1000 | 0.1001 | 0.0044 | 2.5 | 2.83 | 1.21 | rh_LFJ4 lo +0.0014; rh_LFJ1 hi -0.0015 | 0.0003 | rh_A_LFJ3 + (ok) | 2.202 of 3.142 | - | 530 | 14.0 |
| as_modelled | 1 | thumb | no | direction | 0.1000 | 0.1001 | 0.0132 | 7.6 | 3.91 | 3.84 | - | 0.0002 | rh_A_THJ4 + (ok) | - | - | 609 | 17.9 |
| no_joint_friction | 0 | index | no | direction | 0.1000 | 0.0995 | 0.0600 | 35.0 | 3.41 | 1.02 | rh_FFJ1 hi -0.0003 | 0.0000 | rh_A_FFJ3 + (ok) | 1.799 of 3.142 | - | 524 | 78.8 |
| no_joint_friction | 0 | middle | no | direction | 0.1000 | 0.0995 | 0.0569 | 33.2 | 3.00 | 0.68 | rh_MFJ1 hi -0.0003 | 0.0000 | rh_A_MFJ3 + (ok) | 1.713 of 3.142 | - | 452 | 68.9 |
| no_joint_friction | 0 | ring | no | direction | 0.1000 | 0.0995 | 0.0608 | 35.5 | 3.23 | 1.26 | rh_RFJ1 hi -0.0003 | 0.0000 | rh_A_RFJ3 + (ok) | 1.801 of 3.142 | - | 417 | 68.1 |
| no_joint_friction | 0 | little | no | direction | 0.1000 | 0.0997 | 0.0091 | 5.2 | 3.85 | 1.25 | rh_LFJ4 lo +0.0013; rh_LFJ1 hi -0.0015 | 0.0000 | rh_A_LFJ3 + (ok) | 2.237 of 3.142 | - | 662 | 7.8 |
| no_joint_friction | 0 | thumb | no | direction | 0.1000 | 0.0999 | 0.0056 | 3.2 | 3.79 | 3.86 | - | 0.0000 | rh_A_THJ4 + (ok) | - | - | 308 | 5.4 |
| no_joint_friction | 1 | index | no | direction | 0.1000 | 0.0995 | 0.0600 | 35.0 | 3.41 | 1.02 | rh_FFJ1 hi -0.0003 | 0.0000 | rh_A_FFJ3 + (ok) | 1.799 of 3.142 | - | 524 | 86.5 |
| no_joint_friction | 1 | middle | no | direction | 0.1000 | 0.0995 | 0.0569 | 33.2 | 3.00 | 0.68 | rh_MFJ1 hi -0.0003 | 0.0000 | rh_A_MFJ3 + (ok) | 1.713 of 3.142 | - | 452 | 80.8 |
| no_joint_friction | 1 | ring | no | direction | 0.1000 | 0.0995 | 0.0608 | 35.5 | 3.23 | 1.26 | rh_RFJ1 hi -0.0003 | 0.0000 | rh_A_RFJ3 + (ok) | 1.801 of 3.142 | - | 417 | 63.2 |
| no_joint_friction | 1 | little | no | direction | 0.1000 | 0.0997 | 0.0091 | 5.2 | 3.85 | 1.25 | rh_LFJ4 lo +0.0013; rh_LFJ1 hi -0.0015 | 0.0000 | rh_A_LFJ3 + (ok) | 2.237 of 3.142 | - | 662 | 7.5 |
| no_joint_friction | 1 | thumb | no | direction | 0.1000 | 0.0999 | 0.0056 | 3.2 | 3.79 | 3.86 | - | 0.0000 | rh_A_THJ4 + (ok) | - | - | 308 | 5.0 |

## Hand

| variant | frame | fingers holding | assembled wrench residual (scaled) | isolated sum residual | tolerance | wrench ok | release drift mm, all collisions (held) | release drift mm, no self-collision (held) | self-colliding pairs at pose (deepest mm) |
|---|---|---|---|---|---|---|---|---|---|
| as_modelled | 0 | 2 of 5 (middle, little) | 0.2955 | 0.0874 | 0.0194 | no | 1076.32 (not held) | 5.73 (not held) | 7 (11.10) |
| as_modelled | 1 | 2 of 5 (middle, little) | 0.2955 | 0.0874 | 0.0194 | no | 1076.32 (not held) | 5.73 (not held) | 7 (11.10) |
| no_joint_friction | 0 | 0 of 5 | 0.2716 | 0.2114 | 0.0194 | no | 7.54 (not held) | 6.58 (not held) | 7 (11.10) |
| no_joint_friction | 1 | 0 of 5 | 0.2716 | 0.2114 | 0.0194 | no | 7.54 (not held) | 6.58 (not held) | 7 (11.10) |

## Self-colliding finger pairs at the follow pose (frame 0)

| body 1 | body 2 | depth mm |
|---|---|---|
| rh_palm | rh_lfknuckle | 0.09 |
| rh_ffdistal | rh_thdistal | 5.09 |
| rh_rfmiddle | rh_lfproximal | 6.58 |
| rh_rfmiddle | rh_lfmiddle | 6.58 |
| rh_rfdistal | rh_lfproximal | 9.94 |
| rh_rfdistal | rh_lfmiddle | 11.10 |
| rh_rfdistal | rh_lfdistal | 1.10 |
