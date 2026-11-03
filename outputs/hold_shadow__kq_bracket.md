# shadow: self-hold against passive joint stiffness

Leader `hold_shadow__self`, fixed placement, 2 frames, scale margins. Written by `tools/hold_stiffness_bracket.py` from `outputs/hold_shadow__kq_bracket.json`. passive joint stiffness bracket; kq 0 is the pessimistic bound (the model has no J1/J2 spring), larger kq stands in for a spring the real hand could be fitted with.

| kq N.m/rad | verdict | reason | finger states (frame 1) | worst unbalanced N.m | scale margin | s |
|---|---|---|---|---|---|---|
| 0 | red | torque | index red, middle red, ring red, little red, thumb green | 0.00166 | 0,0 | 3.8 |
| 0.001 | red | torque | index red, middle red, ring red, little red, thumb green | 0.00158 | 0.114,0.114 | 17.4 |
| 0.002 | red | torque | index red, middle red, ring red, little red, thumb green | 0.00150 | 0.229,0.229 | 17.6 |
| 0.005 | red | torque | index red, middle red, ring red, little red, thumb green | 0.00126 | 0.572,0.572 | 17.2 |
| 0.01 | amber | - | index amber, middle amber, ring amber, little green, thumb green | 0.00087 | 1.1,1.1 | 18.1 |
| 0.02 | green | - | index green, middle green, ring green, little green, thumb green | 0.00009 | 1.92,1.92 | 18.0 |
| 0.05 | green | - | index green, middle green, ring green, little green, thumb green | 0.00000 | 4.47,4.47 | 17.2 |
| 0.1 | green | - | index green, middle green, ring green, little green, thumb green | 0.00000 | 8.8,8.8 | 188.8 |
| 0.2 | green | - | index green, middle green, ring green, little green, thumb green | 0.00000 | 17.5,17.5 | 264.4 |
