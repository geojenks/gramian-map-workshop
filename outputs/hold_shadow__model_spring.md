# shadow: self-hold on model copies with a joint spring

Leader `hold_shadow__self`, fixed placement, 2 frames, scale margins, joint_stiffness 0. Written by `tools/hold_shadow_model_spring.py` from `outputs/hold_shadow__model_spring.json`. model copies with a joint spring (tools/make_shadow_spring_hand.py), checked through follow.py unchanged, which reads the spring as a torque load k (q - springref) at the geometric pose. Fingers in order index, middle, ring, little, thumb.

| variant | k N.m/rad | joints | springref rad | verdict | reason | finger states (frame 1) | unbalanced N.m per finger (frame 1) | scale margin | s |
|---|---|---|---|---|---|---|---|---|---|
| j1_k0 | 0 | J1 | 0 | red | torque | index red, middle red, ring red, little red, thumb green | 0.00164, 0.00154, 0.00166, 0.00100, 0.00000 | 0,0 | 11.3 |
| j1_k0.0005 | 0.0005 | J1 | 0 | red | torque | index red, middle red, ring red, little red, thumb green | 0.00133, 0.00112, 0.00137, 0.00044, 0.00000 | 0.571,0.571 | 16.3 |
| j1_k0.001 | 0.001 | J1 | 0 | amber | - | index amber, middle amber, ring amber, little green, thumb green | 0.00102, 0.00070, 0.00108, 0.00011, 0.00000 | 1.1,1.1 | 12.7 |
| j1_k0.0017 | 0.0017 | J1 | 0 | amber | - | index amber, middle green, ring amber, little amber, thumb green | 0.00059, 0.00010, 0.00067, 0.00089, 0.00000 | 1.67,1.67 | 13.7 |
| j1_k0.002 | 0.002 | J1 | 0 | amber | - | index green, middle green, ring amber, little amber, thumb green | 0.00040, 0.00015, 0.00050, 0.00122, 0.00000 | 1.92,1.92 | 5.3 |
| j1_k0.005 | 0.005 | J1 | 0 | amber | - | index amber, middle amber, ring amber, little amber, thumb green | 0.00146, 0.00269, 0.00124, 0.00455, 0.00000 | 4.46,4.46 | 4.0 |
| j1_k0.02 | 0.02 | J1 | 0 | amber | - | index amber, middle amber, ring amber, little amber, thumb green | 0.01076, 0.01538, 0.00994, 0.02122, 0.00000 | 17.4,17.4 | 54.3 |
| j1j2_k0.02 | 0.02 | J1J2 | 0 | red | torque | index red, middle red, ring red, little red, thumb green | 0.00364, 0.00563, 0.00484, 0.01313, 0.00000 | 0,0 | 66.6 |
| j1_k0.02_ref0.94248 | 0.02 | J1 | 0.94248 | red | torque | index red, middle red, ring red, little red, thumb green | 0.00257, 0.00205, 0.00339, 0.00789, 0.00000 | 0,0 | 3.5 |
| j1_k0.02_refposed | 0.02 | J1 | posed (per joint) | red | torque | index red, middle red, ring red, little red, thumb green | 0.00164, 0.00154, 0.00166, 0.00100, 0.00000 | 0,0 | 4.3 |
| j1j2_k0.02_refposed | 0.02 | J1J2 | posed (per joint) | red | torque | index red, middle red, ring red, little red, thumb green | 0.00164, 0.00154, 0.00166, 0.00100, 0.00000 | 0,0 | 4.2 |
