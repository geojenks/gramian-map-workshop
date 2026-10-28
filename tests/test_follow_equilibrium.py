"""Fingers with fewer inputs than sprung joints are posed by equilibrium
(follow.py, "Fingers with fewer inputs than joints"; issue 21): which
fingers qualify, the joint balance's derivative, the pose returned holding
exactly for the drive and force found, and rigid hands untouched
(python -m unittest). Needs mujoco; without it the tests are skipped."""
import json
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import follow as fo  # noqa: E402

LEADER = os.path.join(ROOT, "data", "leaders", "w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4.json")
STORED = os.path.join(ROOT, "outputs", "follow_wall_x3_ring__w10_cup_physical_x0.3_a0.0915_sl0.3t"
                      "_R33.5_n4__provisional.json")

try:
    import mujoco  # noqa: F401
except ImportError:
    mujoco = None


class Cone(unittest.TestCase):
    def test_projection(self):
        n = np.array([0.0, 0.0, 1.0])
        inside = np.array([0.3, 0.1, 1.0])
        np.testing.assert_array_equal(fo._cone_project(inside, n, 0.5), inside)
        np.testing.assert_array_equal(fo._cone_project(np.array([0.0, 0.1, -1.0]), n, 0.5),
                                      np.zeros(3))
        f = fo._cone_project(np.array([2.0, 0.0, 1.0]), n, 0.5)
        self.assertAlmostEqual(np.hypot(f[0], f[1]), 0.5 * f[2], places=12)
        # the nearest point of the cone: the step is normal to its surface
        self.assertAlmostEqual(float((np.array([2.0, 0.0, 1.0]) - f) @ f), 0.0, places=12)


@unittest.skipIf(mujoco is None, "mujoco not installed")
class Qualify(unittest.TestCase):
    def test_record_fingers_qualify_rigid_do_not(self):
        hand = fo.Hand("wall_x3_ring")
        for f in hand.fingers:
            eq = fo.FingerIK(hand, f).eq
            self.assertIsNotNone(eq)
            self.assertEqual(eq["M"].shape, (1, 4))
            self.assertEqual(list(eq["hinge"]), [False, True, True, True])
        for key in ("human_vc", "george_kp"):
            hand = fo.Hand(key)
            self.assertTrue(all(fo.FingerIK(hand, f).eq is None for f in hand.fingers), key)

    def test_menagerie_hands_do_not_qualify(self):
        # Shadow's fingers have fewer inputs than joints but no springs
        for key in ("shadow", "allegro"):
            try:
                hand = fo.Hand(key)
            except Exception:
                self.skipTest("hand models not fetched")
            self.assertTrue(all(fo.FingerIK(hand, f).eq is None for f in hand.fingers), key)


@unittest.skipIf(mujoco is None, "mujoco not installed")
class Balance(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hand = fo.Hand("wall_x3_ring")
        cls.f = cls.hand.fingers[1]
        cls.ik = fo.FingerIK(cls.hand, cls.f)

    def test_derivative_matches_finite_differences(self):
        ik, body = self.ik, self.f.tip_body
        q = np.array([0.004, 0.5, 0.3, -0.2])
        u = np.array([1.1])
        f = np.array([0.2, -0.3, 0.1])
        p_l = np.array([0.01, -0.006, 0.0])
        G, H, _ = ik._balance(q, u, f, body, p_l)
        h = 1e-7
        Hfd = np.zeros_like(H)
        for i in range(len(q)):
            dq = np.zeros(len(q))
            dq[i] = h
            Hfd[:, i] = (ik._balance(q + dq, u, f, body, p_l)[0]
                         - ik._balance(q - dq, u, f, body, p_l)[0]) / (2 * h)
        np.testing.assert_allclose(H, Hfd, rtol=1e-5, atol=1e-6)
        # and the balance is the energy's gradient
        g = np.zeros(len(q))
        for i in range(len(q)):
            dq = np.zeros(len(q))
            dq[i] = h
            g[i] = (ik._energy(q + dq, u, f, body, p_l) - ik._energy(q - dq, u, f, body, p_l)) / (2 * h)
        np.testing.assert_allclose(G, g, rtol=1e-5, atol=1e-8)

    def test_no_contact_force_gives_the_free_path(self):
        # with f = 0 the equilibrium is the build tool's q = r a
        ik, eq = self.ik, self.ik.eq
        r = eq["M"][0] / eq["k"]
        for a in (0.3, 1.2, 2.0):
            q, bal, _, _ = ik._equilibrium(np.zeros(4), np.array([a]), np.zeros(3),
                                           self.f.tip_body, np.zeros(3))
            np.testing.assert_allclose(q, r * a, atol=1e-9)
            self.assertLess(bal, 1e-9)


@unittest.skipIf(mujoco is None, "mujoco not installed")
class Pose(unittest.TestCase):
    def setUp(self):
        if not os.path.exists(STORED):
            self.skipTest("stored result not found")
        with open(LEADER) as fh:
            self.L = json.load(fh)
        with open(STORED) as fh:
            self.P = json.load(fh)

    def test_pose_is_an_equilibrium_of_the_drive_and_force_found(self):
        hand, leader = fo.Hand("wall_x3_ring"), fo.Leader(self.L)
        Rw = np.array(self.P["placement"]["rotation_matrix"])
        t = np.array(self.P["placement"]["translation_m"])
        k = 0
        c = leader.center(k, Rw, t)
        checked = 0
        for a, f in enumerate(hand.fingers):
            i = self.P["assignment"][f.label]
            if i not in leader.made(k):
                continue
            ik = fo.FingerIK(hand, f)
            p, _, _ = leader.station(k, i, Rw, t)
            fv = leader.force(k, i, Rw)
            (b, _, _), r = ik.best_tangent(p, c, leader.R_obj, force=fv)
            q, info = r[0], r[8]
            # a link standing clear of the surface does not touch: the gap
            # counts in the reach
            self.assertGreaterEqual(r[1], max(0.0, -r[6]))
            u, force = np.array(info["drive"]), np.array(info["force_N"])
            lim = ik.eq["lim"]
            self.assertTrue(np.all(u >= lim[:, 0]) and np.all(u <= lim[:, 1]))
            # the force pushes into the object, within the friction cone (the
            # link's outward normal at the touch points into the object)
            n_in = np.asarray(r[4])
            self.assertGreaterEqual(float(force @ n_in),
                                    np.linalg.norm(force) / np.sqrt(2) - 1e-9)
            # balance at the pose returned, at the contact point it reports
            ik._set_pose(q)
            p_l = ik.d.xmat[b].reshape(3, 3).T @ (np.asarray(r[3]) - ik.d.xpos[b])
            G = ik._balance(q, u, force, b, p_l)[0]
            scale = np.linalg.norm(ik.eq["k"] * (q - ik.eq["ref"])) + np.linalg.norm(ik.eq["M"].T @ u)
            self.assertLess(float(np.linalg.norm(G)), 1e-6 * scale)
            checked += 1
        self.assertGreater(checked, 0)

    def test_result_records_the_equilibrium(self):
        r = fo.follow("wall_x3_ring", self.L, n_eval=2, placement=self.P)
        self.assertIn("pose", r["settings"])
        self.assertEqual(r["settings"]["pose_fingers"], ["f0", "f1", "f2"])
        on = [x for fr in r["frames"] for x in fr["fingers"] if x["state"] != "off"]
        self.assertTrue(on and all("equilibrium" in x for x in on))


@unittest.skipIf(mujoco is None, "mujoco not installed")
class RigidUnchanged(unittest.TestCase):
    def test_no_new_keys(self):
        stored = os.path.join(ROOT, "outputs", "follow_george_kp__tennis_new_cup_take3_handframe.json")
        leader = os.path.join(ROOT, "data", "leaders", "tennis_new_cup_take3_handframe.json")
        with open(stored) as fh:
            P = json.load(fh)
        with open(leader) as fh:
            L = json.load(fh)
        r = fo.follow("george_kp", L, n_eval=3, placement=P)
        self.assertNotIn("pose", r["settings"])
        self.assertFalse(any("equilibrium" in x for fr in r["frames"] for x in fr["fingers"]))


if __name__ == "__main__":
    unittest.main()
