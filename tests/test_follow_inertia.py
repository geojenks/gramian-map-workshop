"""follow(inertia=True): the fit, the torque's sign, and the default path
unchanged (python -m unittest). The hand tests need the hand models (see
gramian_workshop/hands.py); without them they are skipped."""
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


class Fit(unittest.TestCase):
    def test_local_derivs_of_a_quadratic(self):
        dt = np.linspace(-0.1, 0.1, 7)
        a, b, c = np.array([0.3, -1.0]), np.array([2.0, 0.5]), np.array([-4.0, 7.0])
        Q = a + np.outer(dt, b) + np.outer(dt ** 2, c)
        qd, qdd = fo._local_derivs(dt, Q)
        np.testing.assert_allclose(qd, b, atol=1e-10)
        np.testing.assert_allclose(qdd, 2 * c, atol=1e-8)


class DefaultPath(unittest.TestCase):
    def test_no_torque_is_the_old_test(self):
        rng = np.random.default_rng(0)
        Tr, C, fv = rng.normal(size=(3, 3)), rng.normal(size=(3, 6)), rng.normal(size=3)
        self.assertEqual(fo._columns_hold((Tr, C), fv), fo._columns_hold((Tr, C), fv, None))

    def test_joint_rhs_signs_and_zeros(self):
        Js = [(np.zeros((2, 3)), None), None, (np.zeros((3, 3)), None)]
        r = fo._joint_rhs(Js, [None, None, np.array([0.1, -0.2, 0.3])], 0.05)
        np.testing.assert_allclose(r, [0, 0, -2.0, 4.0, -6.0])


class Dynamics(unittest.TestCase):
    def setUp(self):
        self.hand = _hand("allegro")
        if self.hand is None:
            self.skipTest("hand models not found")

    def test_sign_and_weight(self):
        mj = fo._mj()
        hand = self.hand
        f = hand.fingers[0]
        ik = fo.FingerIK(hand, f)
        m = hand.model
        q = hand.apply_coupling(fo._q_full(m, ik, ik.rng.mean(1)))
        dof = f.dof[1]
        qdd = np.zeros(m.nv)
        qdd[dof] = 2.0
        motion, weight = fo._joint_dynamics(ik, q, np.zeros(m.nv), qdd)
        # an accelerating joint needs torque in its own sense: M_jj * qdd
        Mf = np.zeros((m.nv, m.nv))
        d = mj.MjData(m)
        d.qpos[:] = q
        mj.mj_forward(m, d)
        if hasattr(d, "qM"):
            mj.mj_fullM(m, Mf, d.qM)
        else:  # newer MuJoCo: mj_fullM(m, d, dst)
            mj.mj_fullM(m, d, Mf)
        np.testing.assert_allclose(motion, Mf @ qdd, rtol=1e-9, atol=1e-12)
        self.assertGreater(motion[dof], 0.0)
        # at rest the weight term is MuJoCo's bias force
        np.testing.assert_allclose(weight, d.qfrc_bias, rtol=1e-9, atol=1e-12)


class Follow(unittest.TestCase):
    def setUp(self):
        if _hand("allegro") is None or not os.path.exists(STORED):
            self.skipTest("hand models or stored result not found")
        with open(LEADER) as fh:
            self.L = json.load(fh)
        with open(STORED) as fh:
            self.P = json.load(fh)

    def test_off_by_default_and_on_adds_a_readout(self):
        a = fo.follow("allegro", self.L, n_eval=3, placement=self.P)
        b = fo.follow("allegro", self.L, n_eval=3, placement=self.P, inertia=False)
        self.assertEqual(a["frames"], b["frames"])
        c = fo.follow("allegro", self.L, n_eval=3, placement=self.P, inertia=True)
        items = [x for fr in c["frames"] for x in fr["fingers"] if "inertia" in x]
        self.assertTrue(items)
        self.assertTrue(all("weight_share" in x["inertia"] for x in items))
        self.assertFalse(any("inertia" in x for fr in a["frames"] for x in fr["fingers"]))


if __name__ == "__main__":
    unittest.main()
