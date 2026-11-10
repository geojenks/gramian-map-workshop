"""Our scale and disturbance margins against grasps whose answer follows
from the friction pyramid by hand (issue 10; derivations in
docs/MARGINS_WORKED_CHECK.md). Each contact is a one-joint finger pushing
along the contact normal with at most F newtons; friction is the 8-edge
pyramid of coefficient mu, whose edges lie along the coordinate axes for
these contacts. Needs numpy only (python -m unittest).

Two tests are expected failures: at follow()'s default tolerance the
verdict and the scale margin change when the load and the finger strength
are both multiplied by ten, though every hand answer is unchanged. They
record the disagreement reported on issue 10 for the owner of follow.py;
when it is resolved they will show as unexpected successes."""
import math
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

from gramian_workshop import follow as fo  # noqa: E402
from gramian_workshop.vendor import capability_core as cap  # noqa: E402
import margins_worked_check as mw  # noqa: E402

TIGHT = mw.TIGHT
A, L, MU, F, W = mw.A, mw.L, mw.MU, mw.F, mw.W_LOAD
C0 = np.zeros(3)
# the scale margin bisects to cap.BISECT_TOL and returns the lower end; the
# tight tolerance lets the wrench be missed by 1e-6 of its norm
SCALE_TOL = 2 * cap.BISECT_TOL


def _run(grasp, Wv, mu=MU, f_max=F, resid_tol=TIGHT):
    pts, nrms = grasp
    Js = mw.fingers(nrms, f_max)
    ok = fo._redistribute(pts, nrms, Js, C0, Wv, mu, L, resid_tol=resid_tol)
    sm = fo._scale_margin(pts, nrms, Js, C0, Wv, mu, L, resid_tol=resid_tol)
    return ok, sm


def _disturb(grasp, Wv, mu=MU, f_max=F, resid_tol=TIGHT):
    pts, nrms = grasp
    return fo._disturbance_margin(pts, nrms, mw.fingers(nrms, f_max), C0, Wv, mu, L,
                                  resid_tol=resid_tol)


def _in_pyramid(f, n, mu, f_max):
    """Plain check: normal force in [0, f_max], tangential force inside the
    regular octagon of circumradius mu * N with corners along the two
    tangent axes and their diagonals."""
    n = np.asarray(n, float)
    N = float(f @ n)
    if N < -1e-12 or N > f_max + 1e-12:
        return False
    t = f - N * n
    # tangent axes as the model builds them: for a normal along x, y and z;
    # for a normal along y, x and z
    t1 = np.array([0.0, 1.0, 0.0]) if abs(n[0]) > 0.9 else np.array([1.0, 0.0, 0.0])
    t2 = np.cross(n, t1)
    u, v = float(t @ t1), float(t @ t2)
    for k in range(8):
        ang = math.pi / 8 + k * math.pi / 4
        if u * math.cos(ang) + v * math.sin(ang) > mu * N * math.cos(math.pi / 8) + 1e-12:
            return False
    return True


class Pinch(unittest.TestCase):
    """Two contacts facing each other across x, load w along z."""

    def test_scale_margin_on_an_edge(self):
        ok, sm = _run(mw.PINCH, mw.wrench(fz=W))
        self.assertTrue(ok[0])
        self.assertAlmostEqual(sm["margin"], 2 * MU * F / W, delta=SCALE_TOL)

    def test_scale_margin_between_edges(self):
        th = math.pi / 8
        _, sm = _run(mw.PINCH, W * np.array([0, math.sin(th), math.cos(th), 0, 0, 0]))
        self.assertAlmostEqual(sm["margin"], 2 * MU * F * math.cos(math.pi / 8) / W,
                               delta=SCALE_TOL)

    def test_boundary_margin_one(self):
        ok, sm = _run(mw.PINCH, mw.wrench(fz=2 * MU * F))
        self.assertTrue(ok[0])
        self.assertAlmostEqual(sm["margin"], 1.0, delta=SCALE_TOL)

    def test_just_past_boundary_fails(self):
        ok, sm = _run(mw.PINCH, mw.wrench(fz=2 * MU * F * 1.001))
        self.assertEqual(ok, (False, "torque"))
        self.assertLess(sm["margin"], 1.0)

    def test_twice_the_limit_fails_at_half(self):
        Wv = mw.wrench(fz=4 * MU * F)
        ok, sm = _run(mw.PINCH, Wv)
        self.assertEqual(ok, (False, "torque"))
        self.assertAlmostEqual(sm["margin"], 0.5, delta=SCALE_TOL)
        self.assertEqual(_disturb(mw.PINCH, Wv)["margin"], 0.0)

    def test_frictionless_fails(self):
        ok, sm = _run(mw.PINCH, mw.wrench(fz=W), mu=0.0)
        self.assertEqual(ok, (False, "friction"))
        self.assertEqual(sm["margin"], 0.0)

    def test_twist_about_own_axis_fails(self):
        ok, sm = _run(mw.PINCH, mw.wrench(tx=W * L))
        self.assertEqual(ok, (False, "direction"))
        self.assertEqual(sm["margin"], 0.0)

    def test_zero_disturbance(self):
        # it holds the load, but nothing resists a twist about x
        dm = _disturb(mw.PINCH, mw.wrench(fz=W))
        self.assertEqual(dm["margin"], 0.0)
        self.assertIn(dm["direction"], ("+tx", "-tx"))

    def test_zero_disturbance_default_tolerance_is_the_tolerance(self):
        # a twist no contact can make may be missed by RESID_TOL of |ws|
        dm = _disturb(mw.PINCH, mw.wrench(fz=W), resid_tol=None)
        self.assertLessEqual(dm["margin"], fo.RESID_TOL + 1e-9)
        self.assertGreaterEqual(dm["margin"], fo.RESID_TOL * (1 - fo.MARGIN_REL) - 1e-9)


class FourContacts(unittest.TestCase):
    """Two pinches at right angles, across x and across y, load w along z."""

    @classmethod
    def setUpClass(cls):
        cls.Wv = mw.wrench(fz=W)
        cls.ok, cls.sm = _run(mw.FOUR, cls.Wv)
        cls.dm = _disturb(mw.FOUR, cls.Wv)

    def test_scale_margin(self):
        self.assertTrue(self.ok[0])
        self.assertAlmostEqual(self.sm["margin"], 4 * MU * F / W, delta=SCALE_TOL)

    def test_disturbance_margin(self):
        e = 2 * MU * F / W
        # the search bisects each axis to MARGIN_REL of the margin and returns
        # the lower end
        self.assertLessEqual(self.dm["margin"], e * (1 + 1e-6))
        self.assertGreaterEqual(self.dm["margin"], e * (1 - fo.MARGIN_REL) - 1e-9)
        # tied in four directions (twist about x or y, either sense); which
        # one the search names is not part of the answer
        self.assertIn(self.dm["direction"], ("+tx", "-tx", "+ty", "-ty"))

    def test_every_axis_holds_at_the_margin(self):
        """Hand-written contact forces producing w + e* d along each of the
        12 axes (the doc's table), checked by arithmetic: each force in its
        pyramid and within F, and the sum equal to the target. So no axis
        breaks below e* = 2 mu F / w."""
        e = 2 * MU * F / W
        m = MU * F
        pts, nrms = mw.FOUR
        for i, sg in fo.DISTURB_DIRS:
            # forces, contact by contact: (x, y, z) components
            f = np.zeros((4, 3))
            f[0, 0], f[1, 0], f[2, 1], f[3, 1] = -F, F, -F, F   # all normals at F
            tgt = np.zeros(6)
            tgt[i] = sg * e * W
            tgt[2] += W
            if i == 0:                    # push along x: c3, c4 slide along x
                f[2, 0] = f[3, 0] = sg * m
                f[0, 2] = f[1, 2] = W / 2
            elif i == 1:                  # push along y: c1, c2 slide along y
                f[0, 1] = f[1, 1] = sg * m
                f[2, 2] = f[3, 2] = W / 2
            elif i == 2:                  # push along z: shared by all four
                f[:, 2] = (W + sg * e * W) / 4
            elif i == 3:                  # twist about x: c3 and c4 opposed in z
                f[2, 2], f[3, 2] = sg * m, -sg * m
                f[0, 2] = f[1, 2] = W / 2
            elif i == 4:                  # twist about y: c1 and c2 opposed in z
                f[0, 2], f[1, 2] = -sg * m, sg * m
                f[2, 2] = f[3, 2] = W / 2
            else:                         # twist about z: c1 and c2 opposed in y
                f[0, 1], f[1, 1] = sg * m, -sg * m
                f[2, 2] = f[3, 2] = W / 2
            for fi, n in zip(f, nrms):
                self.assertTrue(_in_pyramid(fi, n, MU, F), (i, sg, fi))
            got = np.zeros(6)
            for fi, p in zip(f, pts):
                got[:3] += fi
                got[3:] += np.cross(p, fi) / L
            np.testing.assert_allclose(got, tgt, atol=1e-9, err_msg=str((i, sg)))


class ForceUnits(unittest.TestCase):
    """Multiplying the load and every finger's strength by the same k
    changes no hand answer."""

    def test_tight_tolerance_scale_free(self):
        # the same unit effect as at the default tolerance, a millionth per
        # newton: an input may overrun by TIGHT * |ws| of its limit, so the
        # margin may read high by that share
        s = 2 * MU * F / W
        for k in (10.0, 100.0):
            ok, sm = _run(mw.PINCH, mw.wrench(fz=W * k), f_max=F * k)
            self.assertTrue(ok[0])
            self.assertAlmostEqual(sm["margin"], s, delta=SCALE_TOL + s * TIGHT * W * k)

    @unittest.expectedFailure
    def test_default_tolerance_verdict_scale_free(self):
        # twice the limit, in ten-times-larger units: should fail
        ok, _ = _run(mw.PINCH, mw.wrench(fz=4 * MU * F * 10), f_max=F * 10,
                     resid_tol=None)
        self.assertFalse(ok[0])

    @unittest.expectedFailure
    def test_default_tolerance_scale_margin_scale_free(self):
        _, s1 = _run(mw.PINCH, mw.wrench(fz=W), resid_tol=None)
        _, s10 = _run(mw.PINCH, mw.wrench(fz=W * 10), f_max=F * 10, resid_tol=None)
        self.assertAlmostEqual(s10["margin"], s1["margin"], delta=0.01 * s1["margin"])


if __name__ == "__main__":
    unittest.main()
