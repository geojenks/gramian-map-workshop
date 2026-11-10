Made by tools/margins_worked_check.py on 2026-11-10, follow.py at fbe006b27b7d2182bb6254314f227484e2bead13.

| Case | Expected scale | Ours, tight | Ours, default | Default, wrench rows only | Expected disturbance | Ours, tight | Ours, default | Holds (tight, default) |
|---|---|---|---|---|---|---|---|---|
| pinch | 2.5 | 2.5 | 2.882 | 2.632 | 0 | 0 | 0.04996 | True, True |
| pinch, load between pyramid edges | 2.31 | 2.31 | 2.662 | 2.431 | 0 | 0 | 0.04974 | True, True |
| pinch at its limit | 1 | 1 | 1.36 | 1.053 | 0 | 0 | 0.04975 | True, True |
| pinch at twice its limit | 0.5 | 0.5 | 0.7776 | 0.5263 | 0 | 0 | 0 | False, False |
| pinch, frictionless | 0 | 0 | 0 | 0 | 0 | 0 | 0 | False, False |
| pinch, twist about its own axis | 0 | 0 | 0 | 0 | 0 | 0 | 0 | False, False |
| four contacts | 5 | 5 | 5.572 | 5.263 | 2.5 | 2.495 | 2.847 | True, True |

| Units multiplied by | Case | Load (N) | Finger limit (N) | Expected scale | Ours, tight | Ours, default | Expected to hold | Holds (tight, default) |
|---|---|---|---|---|---|---|---|---|
| 1 | pinch | 4 | 10 | 2.5 | 2.5 | 2.882 | True | True, True |
| 1 | pinch at twice its limit | 20 | 10 | 0.5 | 0.5 | 0.7776 | False | False, False |
| 10 | pinch | 40 | 100 | 2.5 | 2.5 | 6.048 | True | True, True |
| 10 | pinch at twice its limit | 200 | 100 | 0.5 | 0.5 | 4.041 | False | False, True |
| 100 | pinch | 400 | 1000 | 2.5 | 2.501 | 37.75 | True | True, True |
| 100 | pinch at twice its limit | 2000 | 1000 | 0.5 | 0.5003 | 35.75 | False | False, True |
