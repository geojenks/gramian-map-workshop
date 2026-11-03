"""follow.py credits a joint resting on a range limit with the moment the
stop can supply (issue 26, 3 Nov): which joints count as on a stop, the
column's sign and size, the old test reproduced with the credit off, a
demand into the stop holdable (however large: the reaction is unbounded)
and one out of it not, the stacked system dropping the stop's bound row,
a drive parallel to the stop absorbed by it, and the Shadow's shared
J1/J2 motor with J1 on its stop (python -m unittest). Needs mujoco; the Shadow case also needs its model
(GMW_MENAGERIE), else it is skipped."""
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import follow as fo  # noqa: E402

try:
    import mujoco  # noqa: F401
except ImportError:
    mujoco = None


def _pose(hand, ik, qfree):
    mj = fo._mj()
    ik.d.qpos[:] = hand.apply_coupling(fo._q_full(hand.model, ik, np.asarray(qfree, float)))
    mj.mj_kinematics(hand.model, ik.d)
    mj.mj_comPos(hand.model, ik.d)


def _tip_jac(hand, finger, ik):
    mj = fo._mj()
    m = hand.model
    jacp = np.zeros((3, m.nv))
    p = finger.tip_pos(ik.d)
    mj.mj_jac(m, ik.d, jacp, None, p, finger.tip_body)
    return jacp[:, finger.dof], p


class Stack(unittest.TestCase):
    """The hold solve's rows and the stacked system with a stop's unbounded
    column: the column keeps its place in x, its bound row and slack go,
    and with every bound finite both are exactly as before (no mujoco)."""
    def setUp(self):
        rng = np.random.default_rng(3)
        self.Gs = rng.normal(size=(6, 4))
        self.cidx = np.array([0, 0, 2, 2])          # Js entry 1 has no torque check
        self.Fc = rng.normal(size=(3, 4))
        self.Js = [(rng.normal(size=(2, 3)), rng.normal(size=(2, 3))), None,
                   (rng.normal(size=(3, 3)), rng.normal(size=(3, 2)))]
        self.L = 0.1

    def test_stop_box_unbounds_the_stops_only(self):
        l, h = fo._stop_box(5, 2)
        np.testing.assert_array_equal(l, 0.0)
        np.testing.assert_array_equal(h[:-2], 1.0)
        self.assertTrue(np.all(np.isinf(h[-2:])))
        # from an actuator-link box: the inputs' bounds are kept, the box untouched
        base = (np.full(4, 0.2), np.full(4, 0.7))
        l, h = fo._stop_box(4, 1, base)
        np.testing.assert_array_equal(l, 0.2)
        np.testing.assert_array_equal(h[:-1], 0.7)
        self.assertTrue(np.isinf(h[-1]))
        np.testing.assert_array_equal(base[1], 0.7)

    def test_bound_rows(self):
        C = np.arange(6.0).reshape(2, 3)
        l, h = np.zeros(3), np.ones(3)
        A, b, ab = fo._bound_rows(C, l, h)
        np.testing.assert_array_equal(
            A, np.block([[C, np.zeros((2, 3))], [np.eye(3), np.eye(3)]]))
        np.testing.assert_array_equal(b, np.ones(3))
        A, b, ab = fo._bound_rows(C, l, np.array([1.0, np.inf, 0.5]))
        self.assertEqual(A.shape, (4, 5))
        np.testing.assert_array_equal(A[:2, :3], C)
        np.testing.assert_array_equal(A[2:, :3], np.eye(3)[[0, 2]])
        np.testing.assert_array_equal(A[2:, 3:], np.eye(2))
        np.testing.assert_array_equal(b, [1.0, 0.5])

    def test_a_column_parallel_to_a_stop_is_absorbed(self):
        # a drive on the stop's joint alone, in the stop's sense, is absorbed
        # (zeroed in the solve); the opposite sense, a coupled drive and the
        # stop itself are not; nothing is absorbed without a stop
        c = np.array([0.0, 1.0, 0.0])
        C = np.column_stack([3.0 * c, -2.0 * c, c + [0.5, 0.0, 0.0], c])
        h = np.array([1.0, 1.0, 1.0, np.inf])
        np.testing.assert_array_equal(fo._absorbed(C, h), [True, False, False, False])
        np.testing.assert_array_equal(fo._absorbed(C, np.ones(4)), False)
        l = np.array([0.2, 0.0, 0.0, 0.0])
        A, b, ab = fo._bound_rows(C, l, h)
        np.testing.assert_array_equal(ab, [True, False, False, False])
        np.testing.assert_array_equal(A[:3, 0], 0.0)          # zeroed in the solve
        np.testing.assert_array_equal(A[:3, 1:4], C[:, 1:])
        self.assertEqual(A.shape, (6, 7))                    # three bound rows kept
        np.testing.assert_array_equal(b, [0.8, 1.0, 1.0])
        # the solve keeps the drive at its lower bound and the stop takes the
        # rest; the solution is then re-split drive first: demand 1.0 at the
        # joint, the drive makes it alone (x 1/3), reaction 0; demand 5.0,
        # the drive at its bound makes 3.0 and the stop the overflow 2.0; a
        # demand the forced push overshoots by more than the opposite drive
        # can take back (at most 2.0) is not holdable
        self.assertEqual(fo._absorb_pairs(C, h), [(0, 3, 3.0)])
        self.assertEqual(fo._absorb_pairs(C, h, opposed=True), [(1, 3, 2.0)])
        self.assertEqual(fo._absorb_pairs(C, np.ones(4)), [])
        np.testing.assert_allclose(fo._drive_first(C, l, h, [0.2, 0.0, 0.0, 0.4]), [1 / 3, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(fo._drive_first(C, l, h, [0.2, 0.0, 0.0, 4.4]), [1.0, 0.0, 0.0, 2.0])
        # the opposed drive (-2c) pulling against the stop is backed off first:
        # 0.5 of it took 1.0 of reaction, the rest goes to the drive
        np.testing.assert_allclose(fo._drive_first(C, l, h, [0.2, 0.5, 0.0, 1.4]), [1 / 3, 0.0, 0.0, 0.0])
        for x in ([0.2, 0.0, 0.0, 0.4], [0.2, 0.0, 0.0, 4.4], [0.2, 0.5, 0.0, 1.4]):
            np.testing.assert_allclose(C @ fo._drive_first(C, l, h, x), C @ np.array(x))
        np.testing.assert_array_equal(fo._drive_first(C, l, np.ones(4), [0.2, 0.0, 0.0, 0.4]), [0.2, 0.0, 0.0, 0.4])
        ok, resid, x = fo._hold_solve((np.eye(3), C), np.array([0.0, 1.0, 0.0]), box=(l, h))
        self.assertTrue(ok)
        np.testing.assert_allclose(x, [1 / 3, 0.0, 0.0, 0.0], atol=1e-9)
        ok, resid, x = fo._hold_solve((np.eye(3), C), np.array([0.0, 5.0, 0.0]), box=(l, h))
        self.assertTrue(ok)
        np.testing.assert_allclose(x, [1.0, 0.0, 0.0, 2.0], atol=1e-9)
        ok, _, _ = fo._hold_solve((np.eye(3), C), np.array([0.0, -2.0, 0.0]), box=(l, h))
        self.assertFalse(ok)
        # the stacked system zeroes the same column in its joint rows
        Js = [(np.eye(3), C), None, self.Js[2]]
        Mx, nt, nx, abm = fo._torque_stack(self.Gs, self.cidx, self.Fc, Js, self.L,
                                           [(l, h), None, None])
        np.testing.assert_array_equal(abm, [True, False, False, False, False, False])
        np.testing.assert_array_equal(Mx[6:6 + nt, 4], 0.0)
        np.testing.assert_array_equal(Mx[6:6 + nt, 5:8], np.vstack([-C[:, 1:], np.zeros((3, 3))]) / self.L)
        # and the stacked input use is re-split finger by finger
        x = np.array([0.2, 0.0, 0.0, 0.4, 0.3, 0.1])
        np.testing.assert_allclose(fo._drive_first_stack(Js, [(l, h), None, None], x),
                                   [1 / 3, 0.0, 0.0, 0.0, 0.3, 0.1])
        np.testing.assert_array_equal(fo._drive_first_stack(Js, None, x), x)

    def test_stack_drops_the_unbounded_columns_row(self):
        Mx0, nt, nx, ab0 = fo._torque_stack(self.Gs, self.cidx, self.Fc, self.Js, self.L)
        self.assertEqual((nt, nx), (5, 5))
        self.assertEqual(Mx0.shape, (6 + 5 + 5, 4 + 5 + 5))
        unit = [(np.zeros(3), np.ones(3)), None, (np.zeros(2), np.ones(2))]
        Mx1, nt1, nx1, ab1 = fo._torque_stack(self.Gs, self.cidx, self.Fc, self.Js, self.L, unit)
        np.testing.assert_array_equal(Mx1, Mx0)         # finite boxes: as before
        self.assertEqual((nt1, nx1), (nt, nx))
        jr, span, low = fo._stack_rhs(self.Js, None, unit, self.L)
        np.testing.assert_array_equal(jr, np.zeros(5))
        np.testing.assert_array_equal(span, np.ones(5))
        np.testing.assert_array_equal(low, np.zeros(5))
        # the last column of the third finger a stop's: no bound row, no slack
        stop = [(np.zeros(3), np.ones(3)), None, (np.zeros(2), np.array([1.0, np.inf]))]
        Mx2, nt2, nx2, ab2 = fo._torque_stack(self.Gs, self.cidx, self.Fc, self.Js, self.L, stop)
        self.assertEqual((nt2, nx2), (nt, nx))
        self.assertEqual(Mx2.shape, (6 + 5 + 4, 4 + 5 + 4))
        np.testing.assert_array_equal(Mx2[:11, :9], Mx0[:11, :9])
        np.testing.assert_array_equal(Mx2[:11, 9:], 0.0)
        np.testing.assert_array_equal(Mx2[11:, :4], 0.0)
        np.testing.assert_array_equal(Mx2[11:, 4:9], np.eye(5)[:4])
        np.testing.assert_array_equal(Mx2[11:, 9:], np.eye(4))
        taus = [np.array([0.1, 0.2]), None, np.array([0.0, 0.3, 0.0])]
        jr, span, low = fo._stack_rhs(self.Js, taus, stop, self.L)
        np.testing.assert_allclose(jr, -np.concatenate([taus[0], taus[2]]) / self.L)
        np.testing.assert_array_equal(span, np.ones(4))
        np.testing.assert_array_equal(low, np.zeros(5))


@unittest.skipIf(mujoco is None, "mujoco not installed")
class Wall(unittest.TestCase):
    """wall_x3_ring finger f0: slide [0, 0.024 m], hinge0 [0, 1.004],
    hinge1 [0, 1.305], hinge2 frozen (range 2.5e-9 wide); one input, a
    pull only (lim [0, 2.31]); no coupling, so E is the identity."""
    @classmethod
    def setUpClass(cls):
        cls.hand = fo.Hand("wall_x3_ring")
        cls.f = cls.hand.fingers[0]
        cls.ik = fo.FingerIK(cls.hand, cls.f)
        cls.M, cls.lim = fo.torque_limits(cls.hand, cls.f)
        cls.mid = [0.012, 0.5, 0.6, 0.0]

    def _cols(self, q, stops):
        _pose(self.hand, self.ik, q)
        J, p = _tip_jac(self.hand, self.f, self.ik)
        return fo._finger_columns(self.hand, self.f, self.ik, self.M, self.lim, J, p, 0.0, stops)

    def test_which_joints_are_on_a_stop(self):
        _pose(self.hand, self.ik, self.mid)
        # mid-range: only the frozen hinge2, on both its limits
        self.assertEqual(fo._stops(self.hand, self.f, self.ik), [(3, "lo", 1.0), (3, "hi", -1.0)])
        self.assertEqual(fo._beyond_limits(self.hand, self.f, self.ik), [])
        _pose(self.hand, self.ik, [0.012, 0.0, 1.30544694, 0.0])
        st = fo._stops(self.hand, self.f, self.ik)
        self.assertIn((1, "lo", 1.0), st)          # hinge0 at its lower limit
        self.assertIn((2, "hi", -1.0), st)         # hinge1 at its upper
        self.assertNotIn((0, "lo", 1.0), st)       # the slide is mid-range
        _pose(self.hand, self.ik, [0.0236400948 - 0.5 * fo.STOP_TOL, 0.5, 0.6, 0.0])
        self.assertIn((0, "hi", -1.0), fo._stops(self.hand, self.f, self.ik))

    def test_a_joint_posed_past_a_limit_is_not_on_a_stop(self):
        # the equilibrium posing can leave the frozen hinge2 past its upper
        # limit: no stop is credited, the record says how far past
        _pose(self.hand, self.ik, [0.012, 0.5, 0.6, 0.15])
        self.assertEqual(fo._stops(self.hand, self.f, self.ik), [])
        bl = fo._beyond_limits(self.hand, self.f, self.ik)
        self.assertEqual([(b["joint"], b["side"]) for b in bl], [("f0_hinge2", "hi")])
        self.assertAlmostEqual(bl[0]["by"], 0.15, places=9)

    def test_columns(self):
        q = [0.012, 0.0, 0.6, 0.0]
        _, C0 = self._cols(q, ())
        self.assertEqual(C0.shape[1], 1)           # the one input sense
        stops = [(1, "lo", 1.0), (3, "hi", -1.0)]
        _, C = self._cols(q, stops)
        self.assertEqual(C.shape[1], 3)
        np.testing.assert_array_equal(C[:, 0], C0[:, 0])
        E = fo._free_map(self.hand, self.f, self.ik)
        sc = fo._stop_scale(E, self.M, self.lim)
        # the column size: the one input column's norm (scaled like an input)
        self.assertAlmostEqual(sc, float(self.lim[0][1]) * float(np.linalg.norm(self.M[0])), places=9)
        np.testing.assert_allclose(C[:, 1], np.array([0, 1, 0, 0]) * sc)
        np.testing.assert_allclose(C[:, 2], np.array([0, 0, 0, -1]) * sc)
        # off reproduces the old columns exactly
        _, C_off = self._cols(q, [])
        np.testing.assert_array_equal(C_off, C0)

    def test_holdable_into_the_stop_only(self):
        q = [0.012, 0.0, 0.6, 0.0]
        stops = [(1, "lo", 1.0)]
        Tr, C0 = self._cols(q, ())
        _, C = self._cols(q, stops)
        I4 = np.eye(4)                              # demand the joint torque directly
        into = np.array([0.0, 0.01, 0.0, 0.0])      # +0.01 N.m at hinge0, into its lower stop
        box = fo._stop_box(C.shape[1], len(stops))
        ok0, _, _ = fo._hold_solve((I4, C0), into)
        ok1, _, x = fo._hold_solve((I4, C), into, box=box)
        self.assertFalse(ok0)                       # the pull alone cannot (it moves every joint)
        self.assertTrue(ok1)
        use = fo._stop_use(self.hand, self.f, self.ik, self.M, self.lim, 0.0,
                           [{"joint": "f0_hinge0", "side": "lo"}], x, "finger")
        self.assertEqual(len(use), 1)
        self.assertAlmostEqual(use[0]["reaction"], 0.01, places=6)
        self.assertEqual(use[0]["decided_by"], "finger")
        self.assertEqual(set(use[0]), {"joint", "side", "reaction", "decided_by"})
        ok2, _, _ = fo._hold_solve((I4, C), -into, box=box)  # out of the stop: nothing can
        self.assertFalse(ok2)
        # a demand far above the finger's own strength (a thousand times its
        # strongest input column) is held by the stop alone, its reaction
        # read back exactly: the reaction has no upper bound
        sc = fo._stop_scale(fo._free_map(self.hand, self.f, self.ik), self.M, self.lim)
        big = np.array([0.0, 1000.0 * sc, 0.0, 0.0])
        ok3, _, x3 = fo._hold_solve((I4, C), big, box=box)
        self.assertTrue(ok3)
        use = fo._stop_use(self.hand, self.f, self.ik, self.M, self.lim, 0.0,
                           [{"joint": "f0_hinge0", "side": "lo"}], x3, "finger")
        self.assertAlmostEqual(use[0]["reaction"] / big[1], 1.0, places=9)
        self.assertAlmostEqual(float(x3[0]), 0.0, places=9)   # the pull is not used

    def test_stiffness_columns_come_before_the_stops(self):
        q = [0.012, 0.0, 0.6, 0.0]
        _pose(self.hand, self.ik, q)
        J, p = _tip_jac(self.hand, self.f, self.ik)
        stops = [(1, "lo", 1.0)]
        _, C = fo._finger_columns(self.hand, self.f, self.ik, self.M, self.lim, J, p, 0.01, stops)
        n_in = len(fo._input_senses(self.lim))
        self.assertEqual(C.shape[1], n_in + 2 * len(self.ik.free) + 1)
        x = np.zeros(C.shape[1])
        x[-1] = 0.5
        use = fo._stop_use(self.hand, self.f, self.ik, self.M, self.lim, 0.01,
                           [{"joint": "f0_hinge0", "side": "lo"}], x, "redistribution")
        E = fo._free_map(self.hand, self.f, self.ik)
        self.assertAlmostEqual(use[0]["reaction"], 0.5 * fo._stop_scale(E, self.M, self.lim))


@unittest.skipIf(mujoco is None, "mujoco not installed")
class ShadowLittleFinger(unittest.TestCase):
    """LFJ1 and LFJ2 share one motor (M row [.., 1, 1], lim +-1 N.m) with no
    spring: a pad load wanting different moments at the two is unholdable
    unless J1 rests on its stop, which then takes the difference."""
    @classmethod
    def setUpClass(cls):
        try:
            cls.hand = fo.Hand("shadow")
        except Exception as e:                      # model not fetched here
            raise unittest.SkipTest(f"shadow model unavailable: {e}")
        cls.f = cls.hand.fingers[[g.label for g in cls.hand.fingers].index("little")]
        cls.ik = fo.FingerIK(cls.hand, cls.f)
        cls.M, cls.lim = fo.torque_limits(cls.hand, cls.f)

    def test_j1_on_its_stop_takes_the_difference(self):
        hi1 = float(self.hand.model.jnt_range[self.f.joints[4]][1])
        _pose(self.hand, self.ik, [0.3, 0.0, 0.5, 0.8, hi1])
        stops = fo._stops(self.hand, self.f, self.ik)
        self.assertEqual(stops, [(4, "hi", -1.0)])
        J, p = _tip_jac(self.hand, self.f, self.ik)
        _, C0 = fo._finger_columns(self.hand, self.f, self.ik, self.M, self.lim, J, p, 0.0)
        _, C = fo._finger_columns(self.hand, self.f, self.ik, self.M, self.lim, J, p, 0.0, stops)
        box = fo._stop_box(C.shape[1], len(stops))
        I5 = np.eye(5)
        for t2, t1, old, new in [(0.3, 0.2, False, True),   # J1 wants less: the stop takes 0.1
                                 (0.3, 0.4, False, False),  # J1 wants more: nothing can
                                 (0.3, 0.3, True, True)]:   # equal: the motor alone
            t = np.array([0.0, 0.0, 0.0, t2, t1])
            with self.subTest(t2=t2, t1=t1):
                self.assertEqual(fo._hold_solve((I5, C0), t)[0], old)
                ok, _, x = fo._hold_solve((I5, C), t, box=box)
                self.assertEqual(ok, new)
                if new and t1 < t2:
                    use = fo._stop_use(self.hand, self.f, self.ik, self.M, self.lim, 0.0,
                                       [{"joint": "rh_LFJ1", "side": "hi"}], x, "finger")
                    self.assertAlmostEqual(use[0]["reaction"], t2 - t1, places=6)


if __name__ == "__main__":
    unittest.main()
