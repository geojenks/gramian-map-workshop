"""The per-finger torque test for a finger with both hinges and slides
checks each joint type in its own unit (ruled on issue 21): hinge torque
against the hinges' allowance, slide force against the slides', never
summed (python -m unittest)."""
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import follow as fo  # noqa: E402


class ByKind(unittest.TestCase):
    """A made-up finger: a slide and two hinges, one input that pushes the
    slide and turns the hinges in fixed ratios."""

    def setUp(self):
        self.C = np.array([[4.0], [0.04], [0.02]])     # N, N.m, N.m at full input
        self.Tr = np.zeros((3, 3))
        self.kinds = ["slide", "hinge", "hinge"]

    def test_mixed_norm_hid_a_hinge_failure(self):
        # the load the input cannot make at the hinges: half of its slide
        # force, but the wrong ratio between the hinges
        tau = np.array([2.0, 0.02, 0.02])
        ok_old, un_old, _ = fo._hold_solve((self.Tr, self.C), np.zeros(3), tau)
        self.assertTrue(ok_old)                  # the slide's 2 N swamped the norm
        ok, un, x = fo._hold_solve((self.Tr, self.C), np.zeros(3), tau, None, self.kinds)
        self.assertFalse(ok)
        self.assertGreater(un["hinge"], fo.UNACT_TOL * np.linalg.norm(tau[1:]))
        self.assertEqual(set(un), {"hinge", "slide"})

    def test_a_load_the_input_makes_passes(self):
        tau = 0.6 * self.C[:, 0]
        ok, un, x = fo._hold_solve((self.Tr, self.C), np.zeros(3), tau, None, self.kinds)
        self.assertTrue(ok)
        self.assertAlmostEqual(float(x[0]), 0.6, places=6)
        self.assertLess(max(un.values()), 1e-6)

    def test_one_kind_is_the_old_test(self):
        rng = np.random.default_rng(1)
        Tr, C, fv = rng.normal(size=(3, 3)), rng.normal(size=(3, 4)), rng.normal(size=3)
        a = fo._hold_solve((Tr, C), fv)
        b = fo._hold_solve((Tr, C), fv, None, None, ["hinge"] * 3)
        self.assertEqual(a[:2], b[:2])
        np.testing.assert_array_equal(a[2], b[2])


if __name__ == "__main__":
    unittest.main()
