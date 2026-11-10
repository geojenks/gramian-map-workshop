# Our margins on grasps whose answer is known

Issue 10. We run our two margins in `gramian_workshop/follow.py` on small
grasps whose answer follows from the friction pyramid by hand:

- the scale margin, `_scale_margin`: how many times larger the leader's
  load could get and still be held;
- the disturbance margin, `_disturbance_margin`: the smallest extra push or
  twist, as a share of the leader's load, that breaks the grip.

**Result.** With a tight tolerance (the wrench missed by at most a
millionth of its norm), both margins match the hand answers in every case:
the scale margin to the bisection step, the disturbance margin to the 1 %
bisection step of its search. That includes the boundary case (scale
margin one), the zero-disturbance case and three cases that must fail.

At the tolerance `follow()` uses (`RESID_TOL`, 5 % of the wrench norm), the
margins read higher than the hand answers. The gap grows with the size of
the forces in newtons. Multiply the load and the finger strength by ten,
and no hand answer changes, but ours does. At ten times, a pinch asked to
hold twice what friction allows passes the hold test. This is reported on
issue 10 for the owner of `follow.py`; we changed nothing in it.

No published worked example can be reproduced from what is in the repo.
The papers that `docs/LITERATURE_REVIEW.md` cites for these measures are
not stored here: Li and Sastry's memo, the handbook chapter of
Prattichizzo and Trinkle, Ferrari and Canny 1992, and Roa and Suárez 2015.
So the checks below are closed-form cases only.

Script: `tools/margins_worked_check.py`. Output:
`outputs/margins_worked_check.json` and `outputs/margins_worked_check.md`.
Tests: `tests/test_margins_worked_grasp.py`.

## The model hand

Each contact is a finger with one joint that pushes along the contact
normal. In `follow.py` terms its columns are Tr = nᵀ and one input column
of F newtons, so its normal force N is between 0 and F. Tangential force
loads no joint. Friction is the model's pyramid: 8 edges, inscribed in the
cone of coefficient μ. For a normal along x the pyramid's edges point along
y, z and their diagonals; for a normal along y, along x, z and their
diagonals. Along an edge the pyramid reaches the full μN, as the cone does.
Midway between two edges it reaches only μN cos(π/8).

The object's centre is at the origin. Contacts sit a distance a from it.
Torque is scaled by L = a, as the code divides torque by L. The load W is
the wrench the contacts must produce. Parameters: a = 0.05 m, μ = 0.5,
F = 10 N, load w = 4 N along z.

Both margins are computed twice: with `resid_tol` = 1e-6 (tight), and with
the default.

## The pinch

Two contacts across x: c1 at (a, 0, 0) pushing along −x, c2 at (−a, 0, 0)
pushing along +x. Write c1's force as (−N1, y1, z1) and c2's as
(N2, y2, z2). The wrench they produce, torque divided by a:

- force: (N2 − N1, y1 + y2, z1 + z2);
- torque: (0, z2 − z1, y1 − y2).

**Scale margin, load on an edge.** For a load w along z: N1 = N2, y1 = y2
= 0, and z1 = z2 = w/2 (the torque about y must vanish). Each z is at most
μN, and N at most F. So the largest load is 2μF, and the scale margin is

  s = 2μF / w.

**Load between two edges.** For a load at π/8 from z, in the y–z plane,
each contact carries half of it, along the direction midway between two
pyramid edges. There the pyramid reaches μN cos(π/8), so

  s = 2μF cos(π/8) / w.

The true cone would give 2μF / w. The gap is the model's inscribed
pyramid (the handbook's choice), not an error in the margin.

**Boundary.** With w = 2μF, s = 1: the leader's load is exactly the most
the pinch can hold.

**Must fail.**

- Twice the limit, w = 4μF: s = 0.5, and the hold test fails on torque.
- No friction, μ = 0: no contact can push along z, so the hold test fails
  on friction and both margins are 0.
- A twist about x as the load: the torque about x is 0 for any forces, so
  the hold test fails on direction and both margins are 0.

**Zero disturbance.** The torque about x is always 0. So a twist about x,
however small, cannot be resisted: the disturbance margin is 0, along
±tx, whatever the load. Two point contacts cannot twist an object about
the line through them.

At the default tolerance the wrench may be missed by `RESID_TOL` of its
norm. A twist the contacts cannot make at all then holds up to exactly that
share, so the margin is `RESID_TOL`, not 0. Our default-tolerance column
agrees.

## Four contacts

Add a second pinch across y: c3 at (0, a, 0) pushing along −y, c4 at
(0, −a, 0) pushing along +y. Write c3's force as (x3, −N3, z3) and c4's as
(x4, N4, z4). The wrench, torque divided by a:

- force: (N2 − N1 + x3 + x4, y1 + y2 − N3 + N4, z1 + z2 + z3 + z4);
- torque: (z3 − z4, z2 − z1, y1 − y2 − x3 + x4).

**Scale margin.** All four contacts can carry z, each up to μF, with no
torque. So the largest load along z is 4μF, and

  s = 4μF / w.

**Disturbance margin.** The disturbance is e·w along one of the 12 signed
axes, added to the load. The margin is the smallest e at which the grip
breaks.

Twist about x: the torque is z3 − z4, and each z is at most μF in size, so
it is at most 2μF. The load can still be carried by c1 and c2 (z1 = z2 =
w/2, needing w ≤ 2μF). So this axis breaks at

  e* = 2μF / w.

Twist about y gives the same, by symmetry, so the axis the search names
among these four is not part of the answer.

Every other axis holds at e*: the table below gives contact forces that
make the load plus e*·w along that axis, with every normal force at F and
every tangential force on a pyramid edge or inside it. The tests check
each one by plain arithmetic. So no axis breaks below e*, and the
disturbance margin is e*. For the push along z the construction needs
(w + e*w)/4 ≤ μF, which holds when w ≤ 2μF.

| Axis | Tangential forces (all normals at F) |
|---|---|
| ±x push | x3 = x4 = ±μF; z1 = z2 = w/2 |
| ±y push | y1 = y2 = ±μF; z3 = z4 = w/2 |
| ±z push | z1 = z2 = z3 = z4 = (w ± e*w)/4 |
| ±x twist | z3 = ±μF, z4 = ∓μF; z1 = z2 = w/2 |
| ±y twist | z2 = ±μF, z1 = ∓μF; z3 = z4 = w/2 |
| ±z twist | y1 = ±μF, y2 = ∓μF; z3 = z4 = w/2 |

## Our numbers against the hand answers

Tolerances: the scale margin bisects to `cap.BISECT_TOL` (1e-4) below 16
and returns the lower end; the tests allow twice that. The disturbance
margin bisects each axis to `MARGIN_REL` (1 %) of the margin and returns
the lower end, so ours may sit up to 1 % below the hand answer; the tests
allow exactly that. The tight tolerance shares the unit effect described
below, at a millionth per newton of load: at 400 N the scale margin may
read high by 400 millionths of itself, and the units test allows that.

"Default, wrench rows only" is the scale margin we would expect at the
default tolerance if only the wrench could be missed by 5 % and the inputs
kept within their limits: s / (1 − `RESID_TOL`).

From `outputs/margins_worked_check.md`:

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

## The disagreement at the default tolerance

With the tight tolerance every case agrees. With the default, the scale
margin reads above even the "wrench rows only" reading, the four-contact
disturbance margin reads above e*, and both grow with the units.

What goes wrong, in plain terms. The hold test is one least-squares
problem whose rows have three different units: the object wrench
(newtons, torque divided by L), each finger's joint balance (divided by
L), and each input's bound (input use plus slack equals 1, no unit). One
tolerance, 5 % of the wrench norm in newtons, is allowed over all of them
together. So an input may run past its limit by up to 5 % of the load's
size in newtons, in units of its own limit: 20 % for a 4 N load, ten times
its limit for a 200 N load. A finger can then push harder than it can.

The smallest case: the pinch above with F = 100 N per finger and a load
of 200 N along z. Friction allows at most 2μF = 100 N, so it must fail. At
the tight tolerance `_redistribute` says it fails on torque; at the
default it says it holds. The same pinch at F = 10 N and 20 N fails at
both. The units table above shows the scale margin and the verdict at one,
ten and a hundred times.

We have not measured how far this moves the stored follow results: it
depends on each frame's leader wrench in newtons, which no stored output
records. The same tolerance appears in `_apply_limit`, whose docstring
notes the slack is set as for a 1 N wrench.

The tests mark the two unit checks as expected failures. When the owner
of `follow.py` resolves it, they will show as unexpected successes, and
should then become ordinary tests.
