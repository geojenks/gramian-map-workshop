"""follow.py reads each joint's spring from the hand model (issue 21): the
values read, the torque's sign, the balance with the drive on the free
path, the apply limit's spring load, and hands without springs unchanged
(python -m unittest). Needs mujoco; without it the tests are skipped."""
import json
import os
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import follow as fo  # noqa: E402

HAND_XML = os.path.join(ROOT, "data", "hands", "wall_0.0010m_x3_ring", "hand.xml")
LEADER = os.path.join(ROOT, "data", "leaders", "w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4.json")
STORED = os.path.join(ROOT, "outputs", "follow_wall_x3_ring__w10_cup_physical_x0.3_a0.0915_sl0.3t"
                      "_R33.5_n4__provisional.json")

try:
    import mujoco  # noqa: F401
except ImportError:
    mujoco = None


def _pose(hand, finger, ik, qfree):
    """Put ik.d at the finger's free joints qfree (the rest at the base pose)."""
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


@unittest.skipIf(mujoco is None, "mujoco not installed")
class ReadFromModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hand = fo.Hand("wall_x3_ring")

    def test_values_are_the_model_files(self):
        joints = {j.get("name"): j for j in ET.parse(HAND_XML).iter("joint")}
        m = self.hand.model
        for f in self.hand.fingers:
            k, ref = fo.model_springs(self.hand, f)
            names = [m.joint(j).name for j in f.joints]
            np.testing.assert_allclose(k, [float(joints[n].get("stiffness")) for n in names])
            np.testing.assert_allclose(ref, [float(joints[n].get("springref")) for n in names])

    def test_no_springs_in_a_hand_without_them(self):
        hand = fo.Hand("human_vc")         # local model: no fetch needed
        self.assertTrue(all(fo.model_springs(hand, f) is None for f in hand.fingers))

    def test_torque_sign(self):
        # the spring pulls back toward its rest position, so holding a pose
        # past it needs torque in the pose's own sense: k (q - ref)
        hand, f = self.hand, self.hand.fingers[0]
        ik = fo.FingerIK(hand, f)
        q = ik.rng[:, 0] + 0.5 * (ik.rng[:, 1] - ik.rng[:, 0])
        _pose(hand, f, ik, q)
        J, p = _tip_jac(hand, f, ik)
        M, lim = fo.torque_limits(hand, f)
        rec, tau = fo._spring_item(hand, f, ik, M, lim, J, p, fo.model_springs(hand, f))
        k, ref = fo.model_springs(hand, f)
        np.testing.assert_allclose(tau, k * (q - ref), rtol=1e-12, atol=1e-15)
        self.assertEqual(len(rec["share"]), len(q))

    def test_drive_holds_the_springs_on_the_free_path(self):
        # the build tool writes coef_j = K_j r_j, so at q = r a the drive
        # at a balances the springs exactly: with no contact force the hold
        # test passes, using a / (top of the drive range) of the input
        hand, f = self.hand, self.hand.fingers[1]
        ik = fo.FingerIK(hand, f)
        k, ref = fo.model_springs(hand, f)
        M, lim = fo.torque_limits(hand, f)
        coef = M[0]                                    # tendon moment = K r
        r = coef / k
        for share in (0.25, 0.5, 0.9):
            a = share * lim[0, 1]
            q = np.clip(r * a, ik.rng[:, 0], ik.rng[:, 1])
            _pose(hand, f, ik, q)
            J, p = _tip_jac(hand, f, ik)
            cols = fo._finger_columns(hand, f, ik, M, lim, J, p, 0.0)
            tau = fo._spring_item(hand, f, ik, M, lim, J, p, (k, ref))[1]
            ok, res, x = fo._hold_solve(cols, np.zeros(3), tau)
            self.assertTrue(ok)
            self.assertLess(res, 1e-6 * np.linalg.norm(tau))
            self.assertAlmostEqual(float(x[0]), share, places=6)

    def test_off_the_free_path_the_springs_are_unbalanced(self):
        # bend the middle hinge alone: the one drive cannot hold that pose
        hand, f = self.hand, self.hand.fingers[2]
        ik = fo.FingerIK(hand, f)
        q = np.zeros(len(ik.free))
        q[2] = ik.rng[2, 1]
        _pose(hand, f, ik, q)
        J, p = _tip_jac(hand, f, ik)
        M, lim = fo.torque_limits(hand, f)
        cols = fo._finger_columns(hand, f, ik, M, lim, J, p, 0.0)
        tau = fo._spring_item(hand, f, ik, M, lim, J, p, fo.model_springs(hand, f))[1]
        ok, res, _ = fo._hold_solve(cols, np.zeros(3), tau)
        self.assertFalse(ok)
        self.assertGreater(res, fo.UNACT_TOL * np.linalg.norm(tau))


@unittest.skipIf(mujoco is None, "mujoco not installed")
class StiffnessColumns(unittest.TestCase):
    """joint_stiffness columns (_finger_columns): a hinge's is kq D / arm,
    a slide's is kq D with no lever arm (issue 21)."""

    def test_slide_has_no_lever_arm(self):
        hand = fo.Hand("wall_x3_ring")
        f = hand.fingers[0]
        ik = fo.FingerIK(hand, f)
        m = hand.model
        slide = int(fo._mj().mjtJoint.mjJNT_SLIDE)
        kinds = [int(m.jnt_type[j]) == slide for j in ik.free]
        self.assertTrue(any(kinds) and not all(kinds))
        _pose(hand, f, ik, np.zeros(len(ik.free)))
        M, lim = fo.torque_limits(hand, f)
        J, p = _tip_jac(hand, f, ik)
        kq, D = 0.7, fo.DEFLECT_MAX_M
        n = len(ik.free)
        # the same contact pushed further out along the finger: the slide's
        # column must not change, the hinges' must shrink with the arm
        cols = []
        for extra in (0.0, 0.05):
            pc = p + extra * (p - ik.d.xanchor[ik.free[0]])
            C = fo._finger_columns(hand, f, ik, M, lim, J, pc, kq)[1][:, -2 * n:]
            cols.append(C)
            for c_, j in enumerate(ik.free):
                col = C[:, 2 * c_]
                want = kq * D if kinds[c_] else \
                    kq * D / max(float(np.linalg.norm(pc - ik.d.xanchor[j])), 1e-3)
                self.assertAlmostEqual(float(col[c_]), want, places=12)
                self.assertEqual(float(np.abs(np.delete(col, c_)).max()), 0.0)
                np.testing.assert_array_equal(C[:, 2 * c_ + 1], -col)
        for c_ in range(n):
            a, b = cols[0][c_, 2 * c_], cols[1][c_, 2 * c_]
            if kinds[c_]:
                self.assertEqual(a, b)
            else:
                self.assertGreater(a, b)

    def test_hinge_only_hand_unchanged(self):
        # a hinge-only hand: the columns follow the rule before the fix
        hand = fo.Hand("human_vc")
        f = hand.fingers[0]
        ik = fo.FingerIK(hand, f)
        _pose(hand, f, ik, ik.rng.mean(1))
        M, lim = fo.torque_limits(hand, f)
        J, p = _tip_jac(hand, f, ik)
        n = len(ik.free)
        C = fo._finger_columns(hand, f, ik, M, lim, J, p, 0.4)[1][:, -2 * n:]
        for c_, j in enumerate(ik.free):
            arm = float(np.linalg.norm(p - ik.d.xanchor[j]))
            self.assertEqual(float(C[c_, 2 * c_]), 0.4 * fo.DEFLECT_MAX_M / max(arm, 1e-3))


class ApplyLimitLoad(unittest.TestCase):
    """The apply limit on a made-up stack: two opposed contacts on a ball,
    each finger one joint row and two input columns."""

    def setUp(self):
        self.c = np.zeros(3)
        self.pts = [np.array([0.03, 0.0, 0.0]), np.array([-0.03, 0.0, 0.0])]
        self.nrms = [np.array([-1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])]
        Tr = np.array([[0.0, 0.0, 0.05]])            # pushing along z loads the joint
        C = np.array([[0.1, -0.1]])                    # 0.1 N.m either way
        self.Js = [(Tr, C), (Tr, C)]

    def run_(self, taus):
        return fo._apply_limit(self.pts, self.nrms, self.Js, self.c, 1.0, 0.03, taus=taus)

    def test_zero_load_is_the_old_reading(self):
        a, b = self.run_(None), self.run_([np.zeros(1), np.zeros(1)])
        for key in ("push_N", "twist_Nm"):
            for d in a[key]:
                self.assertAlmostEqual(a[key][d], b[key][d], delta=fo.MARGIN_REL * max(a[key][d], 1e-9) + 1e-6)

    def test_a_spring_load_shifts_the_push(self):
        # holding the springs takes input one way and gives it the other:
        # each joint can make +-0.1 N.m, the springs need +0.05 N.m, so
        # the inputs have 0.05 left toward +z and 0.15 toward -z
        free = self.run_(None)["push_N"]
        part = self.run_([np.array([0.05]), np.array([0.05])])["push_N"]
        self.assertGreater(free["+z"], 0.0)
        self.assertAlmostEqual(free["+z"], free["-z"], delta=fo.MARGIN_REL * free["+z"])
        self.assertAlmostEqual(part["+z"] / free["+z"], 0.5, delta=2 * fo.MARGIN_REL)
        self.assertAlmostEqual(part["-z"] / free["-z"], 1.5, delta=2 * fo.MARGIN_REL)
        # more than the inputs can hold: nothing can be pushed at all
        none = self.run_([np.array([0.5]), np.array([0.5])])
        self.assertTrue(all(v == 0.0 for v in none["push_N"].values()))


@unittest.skipIf(mujoco is None, "mujoco not installed")
class Follow(unittest.TestCase):
    def setUp(self):
        if not os.path.exists(STORED):
            self.skipTest("stored result not found")
        with open(LEADER) as fh:
            self.L = json.load(fh)
        with open(STORED) as fh:
            self.P = json.load(fh)

    def test_record_hand_carries_springs_and_caveat(self):
        r = fo.follow("wall_x3_ring", self.L, n_eval=4, placement=self.P)
        self.assertIn("joint_springs", r["settings"])
        self.assertIn("PROVISIONAL", r["caveat"])
        used = [x for fr in r["frames"] for x in fr["fingers"] if x.get("torque_checked")]
        self.assertTrue(used)
        self.assertTrue(all("spring" in x for x in used))


@unittest.skipIf(mujoco is None, "mujoco not installed")
class FollowWithoutSprings(unittest.TestCase):
    def test_hand_without_springs_has_no_new_keys(self):
        stored = os.path.join(ROOT, "outputs", "follow_george_kp__tennis_new_cup_take3_handframe.json")
        leader = os.path.join(ROOT, "data", "leaders", "tennis_new_cup_take3_handframe.json")
        with open(stored) as fh:
            P = json.load(fh)
        with open(leader) as fh:
            L = json.load(fh)
        r = fo.follow("george_kp", L, n_eval=3, placement=P)
        self.assertNotIn("joint_springs", r["settings"])
        self.assertNotIn("caveat", r)
        self.assertFalse(any("spring" in x for fr in r["frames"] for x in fr["fingers"]))


if __name__ == "__main__":
    unittest.main()
