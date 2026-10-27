"""follow(rate_limit=..., lag_s=...): the reachable box, its column bounds,
the bounded solves, and the default path unchanged (python -m unittest).
The hand tests need the hand models (see gramian_workshop/hands.py);
without them they are skipped."""
import json
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import follow as fo  # noqa: E402

LEADER = os.path.join(ROOT, "data", "leaders", "w10_cup_physical_x0.3_a0.15_sl0.3t_n4.json")
STORED = os.path.join(ROOT, "outputs", "follow_allegro__w10_cup_physical_x0.3_a0.15_sl0.3t_n4.json")


def _hand(key):
    try:
        return fo.Hand(key)
    except Exception:          # models not fetched
        return None


def _net_range(lim, ncols, l, h):
    """Smallest and largest net torque per actuator over the column box."""
    lo, hi = np.zeros(len(lim)), np.zeros(len(lim))
    for j, (a, v) in enumerate(fo._input_senses(lim)):
        lo[a] += min(v * l[j], v * h[j])
        hi[a] += max(v * l[j], v * h[j])
    return lo, hi


class Box(unittest.TestCase):
    lim = np.array([[-2.0, 3.0], [0.0, 1.5], [-1.0, 1.0]])

    def test_column_box_gives_the_net_range(self):
        # all three sign cases of a two-sided motor, and a one-sided input
        for u_lo, u_hi in (([0.5, 0.2, -0.3], [2.0, 1.0, 0.4]),
                           ([-1.5, 0.0, -0.9], [-0.5, 0.7, -0.2]),
                           ([-0.4, 0.3, 0.1], [1.2, 0.3, 0.6])):
            u_lo, u_hi = np.array(u_lo), np.array(u_hi)
            l, h = fo._input_box(self.lim, 5 + 4, u_lo, u_hi)
            lo, hi = _net_range(self.lim, 5, l, h)
            np.testing.assert_allclose(lo, u_lo, atol=1e-12)
            np.testing.assert_allclose(hi, u_hi, atol=1e-12)
            # the passive stiffness columns after the inputs keep [0, 1]
            np.testing.assert_array_equal(l[5:], 0.0)
            np.testing.assert_array_equal(h[5:], 1.0)

    def test_reach_limits(self):
        u0 = np.array([1.0, 0.5, -0.5])
        lo, hi = self.lim[:, 0], self.lim[:, 1]
        # a slow lag or a slow rate pins the box at the previous torque
        for kw in ({"lag": 1e6}, {"rate": 1e-9}):
            a, b = fo._reach(self.lim, u0, 0.05, **kw)
            np.testing.assert_allclose(a, u0, atol=1e-6)
            np.testing.assert_allclose(b, u0, atol=1e-6)
        # a fast one opens it to the whole range
        for kw in ({"lag": 1e-6}, {"rate": 1e6}):
            a, b = fo._reach(self.lim, u0, 0.05, **kw)
            np.testing.assert_allclose(a, lo, atol=1e-9)
            np.testing.assert_allclose(b, hi, atol=1e-9)
        # a rate: within rate * the larger limit * dt of u0
        a, b = fo._reach(self.lim, u0, 0.1, rate=2.0)
        np.testing.assert_allclose(a, np.maximum(lo, u0 - 0.2 * np.array([3.0, 1.5, 1.0])))
        np.testing.assert_allclose(b, np.minimum(hi, u0 + 0.2 * np.array([3.0, 1.5, 1.0])))
        # no previous torque counts as 0; one past the range is clipped first
        a, b = fo._reach(self.lim, None, 0.1, lag=0.1)
        e = np.exp(-1.0)
        np.testing.assert_allclose(a, (1 - e) * lo)
        np.testing.assert_allclose(b, (1 - e) * hi)
        a, _ = fo._reach(self.lim, np.array([9.0, 0.5, -0.5]), 0.1, rate=1e-9)
        self.assertAlmostEqual(a[0], 3.0)

    def test_net_torque(self):
        x = np.array([0.5, 0.25, 1.0, 0.0, 0.5, 0.9, 0.9])
        np.testing.assert_allclose(fo._net_torque(self.lim, x), [1.0, 1.5, -0.5])


class Solves(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(1)
        self.Tr, self.C = rng.normal(size=(3, 3)), np.hstack([np.eye(3), -np.eye(3)])
        self.fv = rng.normal(size=3) * 0.3

    def test_unit_box_is_the_old_test(self):
        cols, n = (self.Tr, self.C), self.C.shape[1]
        a = fo._columns_hold(cols, self.fv)
        b = fo._hold_solve(cols, self.fv, None, (np.zeros(n), np.ones(n)))
        self.assertEqual(a[0], b[0])
        self.assertAlmostEqual(a[1], b[1], places=12)

    def test_tight_box_fails_a_hold(self):
        cols, n = (self.Tr, self.C), self.C.shape[1]
        self.assertTrue(fo._columns_hold(cols, self.fv)[0])
        # the input rows are least-squares rows, as in the [0, 1] test, so x
        # may pass its bound by about the residual; the test still fails
        ok = fo._hold_solve(cols, self.fv, None, (np.zeros(n), np.zeros(n)))[0]
        self.assertFalse(ok)

    def test_held_x_stays_in_its_box(self):
        cols, n = (self.Tr, self.C), self.C.shape[1]
        l, h = np.full(n, 0.1), np.full(n, 0.6)
        ok, _, x = fo._hold_solve(cols, self.fv, None, (l, h))
        self.assertTrue(np.all(x >= l - 1e-12))

    def test_stack_rhs_with_unit_boxes_is_the_old_one(self):
        Js = [(np.zeros((2, 3)), np.ones((2, 4))), None, (np.zeros((3, 3)), np.ones((3, 2)))]
        taus = [None, None, np.array([0.1, -0.2, 0.3])]
        jr, span, low = fo._stack_rhs(Js, taus, [None, None, (np.zeros(2), np.ones(2))], 0.05)
        np.testing.assert_allclose(jr, fo._joint_rhs(Js, taus, 0.05))
        np.testing.assert_array_equal(span, np.ones(6))
        np.testing.assert_array_equal(low, np.zeros(6))


class Follow(unittest.TestCase):
    def setUp(self):
        if _hand("allegro") is None or not os.path.exists(STORED):
            self.skipTest("hand models or stored result not found")
        with open(LEADER) as fh:
            self.L = json.load(fh)
        with open(STORED) as fh:
            self.P = json.load(fh)

    def test_off_by_default_and_on_adds_a_readout(self):
        kw = dict(n_eval=4, placement=self.P, margin=True)
        a = fo.follow("allegro", self.L, **kw)
        b = fo.follow("allegro", self.L, **kw, rate_limit=None, lag_s=None)
        self.assertEqual(a["frames"], b["frames"])
        self.assertFalse(any("actuator_link" in x for fr in a["frames"] for x in fr["fingers"]))
        c = fo.follow("allegro", self.L, **kw, lag_s=0.05)
        self.assertIsNone(c["frames"][0]["actuator_dt_s"])
        self.assertGreater(c["frames"][1]["actuator_dt_s"], 0.0)
        later = [x["actuator_link"] for fr in c["frames"][1:] for x in fr["fingers"]
                 if "actuator_link" in x and "u_lo" in x["actuator_link"]]
        self.assertTrue(later)
        for r in later:
            self.assertTrue(np.all(np.array(r["u_lo"]) <= np.array(r["u_hi"]) + 1e-12))

    def test_a_very_fast_lag_changes_nothing(self):
        kw = dict(n_eval=4, placement=self.P, margin=True)
        a = fo.follow("allegro", self.L, **kw)
        c = fo.follow("allegro", self.L, **kw, lag_s=1e-9)
        for fa, fc in zip(a["frames"], c["frames"]):
            self.assertEqual(fa["verdict"], fc["verdict"])
            self.assertEqual(fa["scale_margin"], fc["scale_margin"])


if __name__ == "__main__":
    unittest.main()
