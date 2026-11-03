# shadow: self-hold against passive joint stiffness

Leader `hold_shadow__self`, fixed placement, 2 frames, scale margins. Written by `tools/hold_stiffness_bracket.py` from `outputs/hold_shadow__kq_bracket__stops.json`. passive joint stiffness bracket (follow(joint_stiffness=)); kq 0 is the pessimistic bound (the model's own springs only), larger kq stands in for a spring the real hand carries or could be fitted with; a joint resting on a range limit is credited with the moment the stop can supply (issue 26).

| kq N.m/rad | verdict | reason | finger states (frame 1) | worst unbalanced N.m | scale margin | s | stop reactions credited (frame 1) |
|---|---|---|---|---|---|---|---|
| 0 | red | torque | index red, middle red, ring red, little green, thumb green | 0.00166 | 0,0 | 3.4 | little: rh_LFJ1 hi 0.0044 |
| 0.001 | amber | - | index amber, middle amber, ring amber, little green, thumb green | 0.00158 | 1.16,1.16 | 16.4 | little: rh_LFJ1 hi 0.0043 |
| 0.002 | amber | - | index amber, middle amber, ring amber, little green, thumb green | 0.00150 | 1.38,1.38 | 16.6 | little: rh_LFJ1 hi 0.0042 |
| 0.005 | amber | - | index amber, middle amber, ring amber, little green, thumb green | 0.00126 | 2.1,2.1 | 16.9 | little: rh_LFJ1 hi 0.0039 |
| 0.01 | amber | - | index amber, middle amber, ring amber, little green, thumb green | 0.00087 | 3.39,3.39 | 15.2 | little: rh_LFJ1 hi 0.0032 |
| 0.02 | green | - | index green, middle green, ring green, little green, thumb green | 0.00009 | 6.15,6.15 | 14.4 | - |
| 0.05 | green | - | index green, middle green, ring green, little green, thumb green | 0.00000 | 14.8,14.8 | 9.8 | - |
| 0.1 | green | - | index green, middle green, ring green, little green, thumb green | 0.00000 | 29.5,29.5 | 7.5 | - |
| 0.2 | green | - | index green, middle green, ring green, little green, thumb green | 0.00000 | 58.8,58.8 | 7.4 | - |
