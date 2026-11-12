"""follow(inertia=True): the fit, the torque's sign, the hand's own weight
(issue 13: where the leader's gravity is read from, how it is checked and
turned into the hand's frame, and that it is in the torque tested) and the
default path unchanged (python -m unittest). The hand tests need the hand
models (see gramian_workshop/hands.py); without them they are skipped."""
import copy
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
HOLD = os.path.join(ROOT, "data", "leaders", "hold_allegro__self.json")
HOLD_STRING = ("9.81 m/s^2 along [-0.4747, 0.3635, -0.8016] (the direction of the stations' "
               "net force, which the object's weight balances)")


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

    def _posed(self):
        hand = self.hand
        ik = fo.FingerIK(hand, hand.fingers[0])
        q = hand.apply_coupling(fo._q_full(hand.model, ik, ik.rng.mean(1)))
        return hand, ik, q

    def test_weight_under_a_tilted_gravity_is_the_sum_over_the_links(self):
        # weight = -sum over bodies of m_b J_com,b^T g, with g the vector given,
        # not the model's own; the model's gravity is put back afterwards
        mj = fo._mj()
        hand, ik, q = self._posed()
        m = hand.model
        g_hand = 9.81 * np.array([0.3, -0.5, 0.8]) / np.linalg.norm([0.3, -0.5, 0.8])
        g_model = m.opt.gravity.copy()
        z = np.zeros(m.nv)
        motion, weight = fo._joint_dynamics(ik, q, z, z, g_hand)
        np.testing.assert_allclose(m.opt.gravity, g_model)
        d = mj.MjData(m)
        d.qpos[:] = q
        mj.mj_forward(m, d)
        tot = np.zeros(m.nv)
        for b in range(1, m.nbody):
            jacp = np.zeros((3, m.nv))
            mj.mj_jacBodyCom(m, d, jacp, None, b)
            tot += float(m.body_mass[b]) * (jacp.T @ g_hand)
        np.testing.assert_allclose(weight, -tot, rtol=1e-9, atol=1e-12)
        np.testing.assert_allclose(motion, 0.0, atol=1e-12)
        self.assertGreater(np.abs(weight[hand.fingers[0].dof]).max(), 0.0)

    def test_reversing_gravity_flips_the_weight(self):
        hand, ik, q = self._posed()
        z = np.zeros(hand.model.nv)
        g = np.array([0.0, 0.0, -9.81])
        w1 = fo._joint_dynamics(ik, q, z, z, g)[1]
        w2 = fo._joint_dynamics(ik, q, z, z, -g)[1]
        np.testing.assert_allclose(w1, -w2, rtol=1e-9, atol=1e-12)
        # None takes the model's own gravity, as before
        w0 = fo._joint_dynamics(ik, q, z, z)[1]
        np.testing.assert_allclose(w0, fo._joint_dynamics(ik, q, z, z, hand.model.opt.gravity)[1])

    def test_the_models_gravity_is_restored_after_an_error(self):
        hand, ik, q = self._posed()
        g_model = hand.model.opt.gravity.copy()
        with self.assertRaises(ValueError):
            fo._joint_dynamics(ik, q[:-1], np.zeros(2), np.zeros(2), np.array([1.0, 0.0, 0.0]))
        np.testing.assert_allclose(hand.model.opt.gravity, g_model)


class LeaderGravity(unittest.TestCase):
    def test_a_list_is_the_direction_gravity_acts_in(self):
        g = fo._leader_gravity({"frame": {"gravity": [0, 0, -2]}})
        self.assertEqual(g["dir"], [0.0, 0.0, -1.0])
        self.assertEqual(g["g"], fo.G_STANDARD)

    def test_the_hold_string_points_against_the_net_force(self):
        g = fo._leader_gravity({"frame": {"gravity": HOLD_STRING}})
        v = np.array([-0.4747, 0.3635, -0.8016])
        np.testing.assert_allclose(g["dir"], -v / np.linalg.norm(v), atol=1e-12)
        self.assertEqual(g["g"], 9.81)
        g = fo._leader_gravity({"frame": {"gravity": "12.5 m/s2 along [0, 0, 1]"}})
        self.assertEqual((g["g"], g["dir"]), (12.5, [0.0, 0.0, -1.0]))

    def test_absent_assumes_minus_z_and_says_so(self):
        for L in ({}, {"frame": {}}, {"frame": {"units": "m"}}):
            g = fo._leader_gravity(L)
            self.assertEqual(g["dir"], [0.0, 0.0, -1.0])
            self.assertIn("assumed", g["source"])

    def test_no_single_direction_gives_none(self):
        for e in ("varies per frame (the derived forces carry it)",
                  "none: the stations' forces sum to zero", "", [0, 0, 0], [1, 2]):
            self.assertIsNone(fo._leader_gravity({"frame": {"gravity": e}})["dir"], e)

    def test_the_check_reads_the_leaders_own_forces(self):
        L = {"object": {"mass_kg": 2.0},
             "frames": [{"force_on_object_N": [[0, 0, 19.62], [5, 5, 5]], "contact": [True, False]},
                        {"force_on_object_N": [[0, 0, 9.81], [0, 0, 9.81]], "contact": [True, True]}]}
        c = fo._gravity_check(L, fo._leader_gravity(L))
        self.assertAlmostEqual(c["cos_with_up"], 1.0)
        self.assertAlmostEqual(c["size_over_mg"], 1.0)
        np.testing.assert_allclose(c["mean_net_force_N"], [0, 0, 19.62])
        c = fo._gravity_check(L, fo._leader_gravity({"frame": {"gravity": [0, 0, 1]}}))
        self.assertAlmostEqual(c["cos_with_up"], -1.0)
        self.assertIsNone(fo._gravity_check(L, {"dir": None, "g": 9.81}))
        c = fo._gravity_check({"frames": L["frames"]}, fo._leader_gravity(L))
        self.assertIsNone(c["size_over_mg"])


class WeightPlan(unittest.TestCase):
    def setUp(self):
        self.hand = _hand("allegro")
        if self.hand is None or not os.path.exists(HOLD):
            self.skipTest("hand models or hold leader not found")
        with open(HOLD) as fh:
            self.L = json.load(fh)

    def test_the_direction_is_turned_into_the_hands_frame(self):
        c, s = np.cos(np.radians(90)), np.sin(np.radians(90))
        Rw = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        rec, g = fo._weight_plan(self.hand, self.L, Rw)
        self.assertTrue(rec["applied"])
        self.assertIsNone(rec["why_not"])
        d = np.asarray(fo._leader_gravity(self.L)["dir"])
        np.testing.assert_allclose(g, 9.81 * Rw @ d, atol=1e-12)
        np.testing.assert_allclose(rec["direction_hand"], Rw @ d, atol=1e-12)
        np.testing.assert_allclose(rec["direction_leader"], d)
        self.assertGreaterEqual(rec["check"]["cos_with_up"], fo.GRAVITY_CHECK_COS)
        self.assertAlmostEqual(rec["check"]["size_over_mg"], 1.0, places=2)

    def test_no_direction_leaves_the_weight_out(self):
        L = copy.deepcopy(self.L)
        L["frame"]["gravity"] = "varies per frame (the derived forces carry it)"
        rec, g = fo._weight_plan(self.hand, L, np.eye(3))
        self.assertIsNone(g)
        self.assertFalse(rec["applied"])
        self.assertTrue(rec["why_not"])
        self.assertIsNone(rec["check"])

    def test_a_disagreeing_check_leaves_the_weight_out(self):
        # gravity given along the net force itself: the forces would then push
        # the object the way it falls, so the direction is refused
        L = copy.deepcopy(self.L)
        L["frame"]["gravity"] = [-0.4747, 0.3635, -0.8016]
        rec, g = fo._weight_plan(self.hand, L, np.eye(3))
        self.assertIsNone(g)
        self.assertIn("cos", rec["why_not"])
        self.assertLess(rec["check"]["cos_with_up"], 0.0)

    def test_a_record_built_hand_leaves_the_weight_out(self):
        hand = _hand("wall_x3_ring")
        p = os.path.join(ROOT, "data", "leaders", "hold_wall_x3_ring__self.json")
        if hand is None or hand.record is None or not os.path.exists(p):
            self.skipTest("record-built hand or its hold leader not found")
        with open(p) as fh:
            L = json.load(fh)
        rec, g = fo._weight_plan(hand, L, np.eye(3))
        self.assertIsNone(g)
        self.assertIn("records", rec["why_not"])
        self.assertIn("unsourced", rec["masses"])


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

    def _items(self, r):
        return [x["inertia"] for fr in r["frames"] for x in fr["fingers"] if "inertia" in x]

    def test_the_weight_is_in_the_torque_added(self):
        c = fo.follow("allegro", self.L, n_eval=3, placement=self.P, inertia=True)
        w = c["settings"]["inertia_weight"]
        self.assertTrue(w["applied"])
        self.assertIn("assumed", w["source"])        # this leader carries no gravity entry
        Rw = np.array(c["placement"]["rotation_matrix"])
        np.testing.assert_allclose(w["direction_hand"], Rw @ [0, 0, -1], atol=1e-12)
        self.assertGreaterEqual(w["check"]["cos_with_up"], fo.GRAVITY_CHECK_COS)
        items = self._items(c)
        self.assertTrue(items)
        for it in items:
            # total = weight + motion where fitted, the weight alone where not
            motion = np.asarray(it["motion_Nm"]) if it["fitted"] else 0.0
            np.testing.assert_allclose(it["total_Nm"], np.asarray(it["weight_Nm"]) + motion,
                                       atol=1e-12)
            self.assertTrue(np.abs(it["weight_Nm"]).max() > 0.0)

    def test_the_weight_is_counted_when_the_window_is_too_short(self):
        # the same frames ten times further apart in time: the fit window
        # (+/- FIT_HALF_S) then holds one frame, so the motion cannot be
        # fitted, but the weight is still counted (the poses are unchanged)
        L = copy.deepcopy(self.L)
        for fr in L["frames"]:
            fr["t"] = 10.0 * float(fr["t"])
        c = fo.follow("allegro", L, n_eval=3, placement=self.P, inertia=True)
        items = self._items(c)
        self.assertTrue(items)
        a = fo.follow("allegro", self.L, n_eval=3, placement=self.P, inertia=True)
        for it, ia in zip(items, self._items(a)):
            self.assertFalse(it["fitted"])
            self.assertEqual(it["window_n"], 1)
            self.assertNotIn("motion_Nm", it)
            np.testing.assert_allclose(it["total_Nm"], it["weight_Nm"])
            np.testing.assert_allclose(it["weight_Nm"], ia["weight_Nm"], rtol=1e-9, atol=1e-12)
            self.assertTrue(ia["fitted"])

    def test_an_unreadable_gravity_leaves_the_weight_out_but_keeps_the_motion(self):
        L = copy.deepcopy(self.L)
        L["frame"] = {"gravity": "varies per frame (the derived forces carry it)"}
        c = fo.follow("allegro", L, n_eval=3, placement=self.P, inertia=True)
        w = c["settings"]["inertia_weight"]
        self.assertFalse(w["applied"])
        self.assertTrue(w["why_not"])
        items = self._items(c)
        self.assertTrue(items)
        self.assertFalse(any("weight_Nm" in it for it in items))
        self.assertTrue(any(it["fitted"] for it in items))

    def test_a_huge_gravity_fails_the_torque_test(self):
        # the weight is on the right-hand side of each finger's torque test:
        # a hundred times gravity, in the direction the check accepts, turns
        # fingers that passed red for torque
        L = copy.deepcopy(self.L)
        L["frame"] = {"gravity": "981.0 m/s^2 along [0, 0, 1] (test)"}
        a = fo.follow("allegro", self.L, n_eval=3, placement=self.P, inertia=True)
        b = fo.follow("allegro", L, n_eval=3, placement=self.P, inertia=True)
        self.assertTrue(b["settings"]["inertia_weight"]["applied"])
        self.assertEqual(b["settings"]["inertia_weight"]["g_m_s2"], 981.0)
        missed = lambda r: sum("torque" in (x.get("missed") or []) for fr in r["frames"]
                               for x in fr["fingers"])
        self.assertGreater(missed(b), missed(a))
        wa = [np.asarray(it["weight_Nm"]) for it in self._items(a)]
        wb = [np.asarray(it["weight_Nm"]) for it in self._items(b)]
        for x, y in zip(wa, wb):
            np.testing.assert_allclose(y, 100.0 * x, rtol=1e-9, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
