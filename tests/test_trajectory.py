"""follow(trajectory=True): the linear programme over a run of checked
frames, on small made-up stacks (python -m unittest). Needs scipy; without
it the tests are skipped."""
import math
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import follow as fo  # noqa: E402
from gramian_workshop import trajectory as tj  # noqa: E402

try:
    import scipy  # noqa: F401
except ImportError:
    scipy = None

LIM = np.array([[-2.0, 3.0]])     # one two-sided motor
DT = 0.1                          # with LAG, e = exp(-1)
LAG = 0.1


def _cols(lim, nj, passive=()):
    """C for a finger whose actuator a drives joint a alone, one column per
    input sense (fo._input_senses order), then passive columns (t on joint
    j, as +t and -t)."""
    cols = []
    for a, v in fo._input_senses(lim):
        e = np.zeros(nj)
        e[a] = v
        cols.append(e)
    for j, t in passive:
        e = np.zeros(nj)
        e[j] = t
        cols += [e, -e]
    return np.array(cols).T


def _rec(k, gens, ws, lim=LIM, finger=0, passive=()):
    """A frame with one contact on finger, one generator per (wrench row 0
    and 1 entries, joint torque) in gens, L = 1."""
    nj = len(lim)
    Gs = np.zeros((6, len(gens)))
    Fc = np.zeros((3, len(gens)))
    for e, (g0, g1, tau) in enumerate(gens):
        Gs[0, e], Gs[1, e] = g0, g1
        Fc[:nj, e] = tau
    Tr = np.eye(nj, 3)
    Js = [(Tr, _cols(lim, nj, passive))]
    w = np.zeros(6)
    w[:len(ws)] = ws
    return tj.stack_record(k, k * DT, w, Gs, np.zeros(len(gens), int), Fc, Js, 1.0, None,
                           [finger], [lim], fo.RESID_TOL)


def _demand(k, u, **kw):
    """A frame whose wrench needs net torque u from the motor."""
    return _rec(k, [(1.0, 0.0, [u])], [1.0], **kw)


def _either(k, eps):
    """A frame that holds with net torque -1 exactly, or +1 with residual
    eps in wrench row 1: the least-residual solution is the -1 one."""
    return _rec(k, [(1.0, 0.0, [1.0]), (1.0, eps, [-1.0])], [1.0, eps])


@unittest.skipIf(scipy is None, "scipy not installed")
class Frame(unittest.TestCase):
    def test_never_looser_than_the_l2_test(self):
        rng = np.random.default_rng(3)
        n_held = 0
        for _ in range(40):
            gens = [(rng.normal(), rng.normal(), [rng.normal()]) for _ in range(3)]
            r = _rec(0, gens, rng.normal(size=2))
            if tj._solve([r], [1.0])[0]:
                n_held += 1
                rhs = np.concatenate([r["ws"], r["jr"], np.ones(r["nx"])])
                tol = max(r["tol_frac"] * r["wn"], fo.cap.FEAS_REL)
                self.assertTrue(fo._feasible(r["Mx"], rhs, tol)[0])
        self.assertGreater(n_held, 0)

    def test_l1_can_be_stricter(self):
        # rows 1 and 2 cannot be made: residual (d, d), L2 within tol, L1 not
        d = 0.6 * fo.RESID_TOL
        r = _rec(0, [(1.0, 0.0, [0.0])], [1.0, d, d])
        rhs = np.concatenate([r["ws"], r["jr"], np.ones(r["nx"])])
        tol = max(r["tol_frac"] * r["wn"], fo.cap.FEAS_REL)
        self.assertTrue(fo._feasible(r["Mx"], rhs, tol)[0])
        self.assertFalse(tj._solve([r], [1.0])[0])

    def test_scale_above_one_grows_the_contact_torque(self):
        # s > 1: the wrench stays, the contact torque grows, so the motor
        # gives s u; past its range (and the residual budget) it fails
        r = _demand(0, 1.0)
        ok, xs = tj._solve([r], [2.0])
        self.assertTrue(ok)
        self.assertAlmostEqual(fo._net_torque(LIM, xs[0][0])[0], 2.0, places=6)
        self.assertFalse(tj._solve([r], [3.5])[0])
        # past the motor's top hi the cheapest residual is the wrench's:
        # cone weight 1 - eps gives torque s (1 - eps) = hi, L1 eps = 1 -
        # hi / s, within the budget up to s = hi / (1 - tol_frac)
        m = tj.check([r])["frames"][0]["unlinked_margin"]
        self.assertAlmostEqual(m["margin"], LIM[0, 1] / (1 - fo.RESID_TOL), delta=1e-3)


@unittest.skipIf(scipy is None, "scipy not installed")
class Links(unittest.TestCase):
    def test_lag_row_binds_on_net_torque(self):
        e = math.exp(-DT / LAG)
        recs = [_demand(0, 0.0), _demand(1, 1.0)]
        ok, xs = tj._solve(recs, [1.0, 1.0], lag=LAG)
        self.assertTrue(ok)
        reach = (1 - e) * LIM[0, 1]
        self.assertTrue(tj._solve([_demand(0, 0.0), _demand(1, 0.9 * reach)], [1.0, 1.0],
                                  lag=LAG)[0])
        self.assertFalse(tj._solve([_demand(0, 0.0), _demand(1, 1.2 * reach)], [1.0, 1.0],
                                   lag=LAG)[0])
        # and with a rate limit of the same reach
        rate = reach / (np.max(np.abs(LIM)) * DT)
        self.assertFalse(tj._solve([_demand(0, 0.0), _demand(1, 1.2 * reach)], [1.0, 1.0],
                                   rate=rate)[0])
        self.assertTrue(tj._solve([_demand(0, 0.0), _demand(1, 1.2 * reach)], [1.0, 1.0])[0])

    def test_links_use_input_senses_not_column_positions(self):
        # motor 0 pulls only (one column), motor 1 is two-sided (two), then
        # passive stiffness columns on both joints: these are not inputs, so
        # the lag does not limit them
        lim = np.array([[0.0, 1.5], [-1.0, 1.0]])
        t = 0.5
        e = math.exp(-DT / LAG)
        passive = ((0, t), (1, t))

        def frame(k, tau):
            return _rec(k, [(1.0, 0.0, tau)], [1.0], lim=lim, passive=passive)

        reach = (1 - e) * lim[0, 1]
        # joint 0 needs more than the motor can reach, less than it plus
        # the passive column; joint 1 needs the push sense of motor 1
        need = [reach + 0.5 * t, 0.5 * (1 - e) * lim[1, 0]]
        recs = [frame(0, [0.0, 0.0]), frame(1, need)]
        ok, xs = tj._solve(recs, [1.0, 1.0], lag=LAG)
        self.assertTrue(ok)
        u0, u1 = fo._net_torque(lim, xs[0][0]), fo._net_torque(lim, xs[1][0])
        tol = 1e-7
        self.assertTrue(np.all(u1 - e * u0 <= (1 - e) * lim[:, 1] + tol))
        self.assertTrue(np.all(u1 - e * u0 >= (1 - e) * lim[:, 0] - tol))
        # past the motor's reach plus the passive column: fails
        recs = [frame(0, [0.0, 0.0]), frame(1, [reach + 2 * t, 0.0])]
        self.assertTrue(tj._solve(recs[1:], [1.0])[0])
        self.assertFalse(tj._solve(recs, [1.0, 1.0], lag=LAG)[0])

    def test_a_finger_new_to_the_frame_starts_from_zero(self):
        e = math.exp(-DT / LAG)
        reach = (1 - e) * LIM[0, 1]
        # finger 1 was not in frame 0: its torque starts from 0, not free
        recs = [_demand(0, LIM[0, 1], finger=0), _demand(1, 1.2 * reach, finger=1)]
        self.assertTrue(tj._solve(recs[1:], [1.0])[0])
        self.assertFalse(tj._solve(recs, [1.0, 1.0], lag=LAG)[0])
        recs = [_demand(0, LIM[0, 1], finger=0), _demand(1, 0.9 * reach, finger=1)]
        self.assertTrue(tj._solve(recs, [1.0, 1.0], lag=LAG)[0])
        # the same finger at its top in frame 0 reaches it
        recs = [_demand(0, LIM[0, 1], finger=1), _demand(1, 1.2 * reach, finger=1)]
        self.assertTrue(tj._solve(recs, [1.0, 1.0], lag=LAG)[0])


@unittest.skipIf(scipy is None, "scipy not installed")
class Runs(unittest.TestCase):
    def test_chain_fails_where_the_trajectory_holds(self):
        # frame 1 may end at -1 or +1; the chain carries its least-residual
        # solution (-1), from which frame 2's demand is out of reach, but
        # the whole run can take +1 and hold
        e = math.exp(-DT / LAG)
        top_from = lambda u: e * u + (1 - e) * LIM[0, 1]  # noqa: E731
        need = 0.5 * (top_from(-1.0) + top_from(1.0))
        recs = [_demand(0, 0.0), _either(1, 0.2 * fo.RESID_TOL), _demand(2, need)]
        out = tj.check(recs, lag=LAG)
        self.assertTrue(all(f["unlinked_holds"] for f in out["frames"]))
        self.assertEqual(len(out["runs"]), 1)
        self.assertTrue(out["runs"][0]["holds"])
        self.assertTrue(out["frames"][1]["chain_lp_holds"])
        self.assertFalse(out["frames"][2]["chain_lp_holds"])
        # the run's margin is no more than any pair's, nor any frame's alone
        rm = out["runs"][0]["margin"]["margin"]
        self.assertGreaterEqual(rm, 1.0)
        for p in out["pairs"]:
            self.assertLessEqual(rm, p["margin"]["margin"] + fo.cap.BISECT_TOL)
        for f in out["frames"]:
            self.assertLessEqual(rm, f["unlinked_margin"]["margin"] + fo.cap.BISECT_TOL)

    def test_per_frame_margin_does_not_assume_zero_holds(self):
        # scaling the middle frame alone to 0 needs its torque to drop to 0
        # and back under a slow lag, which fails although s = 1 holds; the
        # per-frame search must still find the margin above it
        recs = [_demand(k, 1.5) for k in range(3)]
        out = tj.check(recs, lag=1.0, per_frame=True)
        self.assertTrue(out["runs"][0]["holds"])
        self.assertFalse(tj._solve(recs, [1.0, 0.0, 1.0], lag=1.0)[0])
        for f in out["frames"]:
            self.assertGreaterEqual(f["trajectory_margin"]["margin"], 1.0)

    def test_first_break_is_found(self):
        e = math.exp(-DT / LAG)
        jump = 1.2 * (1 - e) * LIM[0, 1]
        n, brk = 8, 5
        recs = [_demand(k, 0.0 if k < brk else jump) for k in range(n)]
        out = tj.check(recs, lag=LAG)
        self.assertEqual(len(out["runs"]), 1)
        run = out["runs"][0]
        self.assertFalse(run["holds"])
        self.assertEqual(run["first_break_k"], brk)
        self.assertAlmostEqual(run["first_break_t"], brk * DT)
        self.assertTrue(tj._solve(recs[:brk], [1.0] * brk, lag=LAG)[0])

    def test_no_links_no_chain(self):
        recs = [_demand(0, 0.0), _demand(1, 1.0)]
        out = tj.check(recs)
        self.assertTrue(out["runs"][0]["holds"])
        self.assertNotIn("chain_lp_holds", out["frames"][0])


def _motion(k, u, u_motion):
    """A frame whose wrench needs net torque u at rest, and whose finger's
    own motion at the recorded speed needs u_motion more of it (the
    motion torque is scaled by speed^2 in trajectory.top_speed)."""
    r = _demand(k, u)
    r["taus"] = [np.array([u_motion])]
    r["jr"] = fo._joint_rhs(r["Js"], r["taus"], 1.0)
    r["jr_motion"] = r["jr"].copy()
    return r


def _tight(recs):
    """The same frames with no residual budget beyond FEAS_REL, so the
    speeds below are the analytic ones (with RESID_TOL the budget lets a
    step shrink by up to its share of the demand, and the speed found is
    that much higher)."""
    for r in recs:
        r["tol_frac"] = 0.0
    return recs


@unittest.skipIf(scipy is None, "scipy not installed")
class TopSpeed(unittest.TestCase):
    def test_rate_limit_sets_the_speed(self):
        # a step of 1.5 in DT against rate 1 (3 per second at most) needs
        # dt = 0.5 s between the frames: a fifth of the recorded speed
        recs = _tight([_demand(0, 0.0), _demand(1, 1.5)])
        out = tj.top_speed(recs, rate=1.0)
        v = 1.0 * LIM[0, 1] * DT / 1.5
        self.assertAlmostEqual(out["speed"], v, delta=fo.cap.BISECT_TOL)
        self.assertFalse(out["capped"])
        self.assertTrue(out["runs"][0]["contiguous"])
        self.assertTrue(tj._solve(recs, [1.0, 1.0], rate=1.0, speed=v - 1e-4)[0])
        self.assertFalse(tj._solve(recs, [1.0, 1.0], rate=1.0, speed=v + 1e-3)[0])
        # with the budget the step can shrink, so the speed found is higher
        soft = tj.top_speed([_demand(0, 0.0), _demand(1, 1.5)], rate=1.0)["speed"]
        self.assertGreater(soft, v)
        self.assertLess(soft, v / (1 - 2 * fo.RESID_TOL))
        self.assertEqual(out["unholdable"], [])

    def test_lag_sets_the_speed(self):
        # from 0 the lag reaches (1 - e) hi in one step: e = 1 - u / hi
        u = 2.4
        recs = _tight([_demand(0, 0.0), _demand(1, u)])
        out = tj.top_speed(recs, lag=LAG)
        e = 1.0 - u / LIM[0, 1]
        v = -DT / (LAG * math.log(e))
        self.assertAlmostEqual(out["speed"], v, delta=fo.cap.BISECT_TOL)

    def test_speed_one_is_the_run_check(self):
        for step in (0.2, 2.0):
            recs = [_demand(0, 0.0), _demand(1, step)]
            held = tj.check(recs, rate=1.0)["runs"][0]["holds"]
            self.assertEqual(tj.top_speed(recs, rate=1.0)["speed"] >= 1.0, held)

    def test_speed_zero_drops_the_rate_and_frees_the_lag(self):
        recs = [_demand(0, 0.0), _demand(1, 2.9)]
        self.assertFalse(tj._solve(recs, [1.0, 1.0], rate=1.0, lag=LAG)[0])
        self.assertTrue(tj._solve(recs, [1.0, 1.0], rate=1.0, lag=LAG, speed=0.0)[0])
        # a demand outside the range stays out of reach at speed 0
        recs = [_demand(0, 0.0), _demand(1, 3.5)]
        self.assertFalse(tj._solve(recs, [1.0, 1.0], lag=LAG, speed=0.0)[0])

    def test_motion_torque_scales_with_speed_squared(self):
        # rest needs 2, the motion 2 more: over the limit 3 as recorded,
        # within it below speed sqrt(0.5)
        recs = _tight([_motion(0, 2.0, 2.0)])
        out = tj.top_speed(recs)
        self.assertAlmostEqual(out["speed"], math.sqrt(0.5), delta=fo.cap.BISECT_TOL)
        self.assertTrue(tj._solve(recs, [1.0], speed=0.5)[0])
        self.assertFalse(tj._solve(recs, [1.0])[0])
        self.assertEqual(out["unholdable"], [])

    def test_motion_can_be_the_rescue_and_the_breaks_are_named(self):
        # frame 0 holds as recorded; a friction failure and a frame that
        # needs 4 at rest (over the limit at every speed) both break the
        # motion, so frames 0 and 3 are runs of their own
        bad = _demand(1, 0.0)
        bad.update(ok=False, reason="friction")
        recs = [_demand(0, 1.0), bad, _demand(2, 4.0), _demand(3, 1.0)]
        out = tj.top_speed(recs, rate=1.0)
        self.assertEqual([(r["k_first"], r["k_last"]) for r in out["runs"]], [(0, 0), (3, 3)])
        self.assertTrue(all(r["speed"] >= 1.0 for r in out["runs"]))
        self.assertGreaterEqual(out["speed"], 1.0)
        self.assertFalse(out["holds_everywhere"])
        self.assertEqual([(u["k"], u["reason"]) for u in out["unholdable"]],
                         [(1, "friction"), (2, "torque")])
        # with its own motion torque counted, slowing rescues a frame that
        # needs 0.5 at rest and 4 more for its motion, so it stays in its
        # run and the motion holds everywhere
        recs = [_motion(0, 1.0, 1.0), _motion(1, 0.5, 4.0)]
        out = tj.top_speed(recs, rate=1.0)
        self.assertEqual(len(out["runs"]), 1)
        self.assertTrue(out["holds_everywhere"])
        self.assertGreater(out["speed"], 0.0)
