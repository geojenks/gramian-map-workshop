"""finger_actuator.py: a Shadow finger as an actuator in its own right. The
scene holds the follow pose with the inputs read from it, the inputs found
for a finger are its own servos (the J0 tendon among them), the torque
closure identity closes at a settled reading, and the verdict rules on
synthetic readings (python -m unittest). Needs mujoco and the menagerie
Shadow (GMW_MENAGERIE); without them the model tests are skipped."""
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import finger_actuator as fa  # noqa: E402
from gramian_workshop import follow as fo  # noqa: E402

try:
    import mujoco  # noqa: F401
    from gramian_workshop.hands import Hand
    HAND = Hand("shadow")
except Exception:  # mujoco missing, or the menagerie not found
    mujoco = None
    HAND = None


class Verdicts(unittest.TestCase):
    """classify() on hand-made readings: no model needed."""

    def _reading(self, f, p, contact=True, at_stop=(None,), saturated=(False,)):
        return {"force_on_object_N": np.asarray(f, float), "point": np.asarray(p, float),
                "tip": np.asarray(p, float), "contact": contact,
                "at_stop": list(at_stop), "saturated": list(saturated)}

    def test_holds_within_tolerances(self):
        f_star, p_star = np.array([0.1, 0.0, 0.0]), np.zeros(3)
        v = fa.classify(self._reading([0.102, 0.001, 0.0], [0.001, 0.0, 0.0]), p_star, f_star, 0.005, 0.01)
        self.assertTrue(v["holds"])
        self.assertTrue(v["green"])
        self.assertIsNone(v["why"])

    def test_reasons(self):
        f_star, p_star = np.array([0.1, 0.0, 0.0]), np.zeros(3)
        cases = [
            (self._reading([0.0, 0.0, 0.0], [0.02, 0, 0], contact=False), "no_contact"),
            (self._reading([0.1, 0.0, 0.0], [0.02, 0, 0]), "reach"),
            (self._reading([0.07, 0.07, 0.0], [0, 0, 0]), "direction"),
            (self._reading([0.05, 0.0, 0.0], [0, 0, 0], saturated=(True,)), "strength_saturated"),
            (self._reading([0.05, 0.0, 0.0], [0, 0, 0], at_stop=("hi",)), "strength_stop"),
            (self._reading([0.05, 0.0, 0.0], [0, 0, 0]), "strength"),
            (self._reading([0.2, 0.0, 0.0], [0, 0, 0]), "force_excess"),
        ]
        for rd, why in cases:
            v = fa.classify(rd, p_star, f_star, 0.005, 0.01)
            self.assertFalse(v["holds"], why)
            self.assertEqual(v["why"], why)

    def test_residual_scales(self):
        f_star, p_star = np.array([0.1, 0.0, 0.0]), np.zeros(3)
        r = fa.residual(self._reading([0.105, 0.0, 0.0], [0.0, 0.01, 0.0]), p_star, f_star, 0.005, 0.01)
        np.testing.assert_allclose(r, [1, 0, 0, 0, 1, 0])

    def test_gravity_parsing(self):
        g, d = fa.gravity_of({"gravity": "9.81 m/s^2 along [-0.7861, 0.1159, -0.6072]"})
        self.assertAlmostEqual(g, 9.81)
        self.assertAlmostEqual(float(np.linalg.norm(d)), 1.0, places=3)


@unittest.skipIf(HAND is None, "mujoco or the menagerie Shadow not available")
class ShadowUnit(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hand = HAND
        # a sphere well clear of the hand: the pose test needs no contact
        cls.m = fa.build_scene(cls.hand, [0.0, 0.0, 1.0], 0.03, friction_loss=False)
        cls.mj = fa._mj()
        cls.sph = cls.mj.mj_name2id(cls.m, cls.mj.mjtObj.mjOBJ_GEOM, fa.SPHERE_GEOM)

    def test_scene_has_the_hand_and_the_sphere(self):
        m = self.m
        self.assertEqual(m.nq, self.hand.model.nq)
        self.assertEqual(m.geom_type[self.sph], self.mj.mjtGeom.mjGEOM_SPHERE)
        np.testing.assert_allclose(m.opt.gravity, 0.0)

    def test_inputs_are_the_fingers_own(self):
        m = self.m
        for f in self.hand.fingers:
            ins = fa.finger_inputs(m, f)
            names = [m.actuator(a).name for a in ins]
            self.assertTrue(all(f.label[:2].upper() in n or n.startswith("rh_A_" + {"index": "FF", "middle": "MF", "ring": "RF", "little": "LF", "thumb": "TH"}[f.label]) for n in names), names)
            if f.label != "thumb":
                self.assertTrue(any(n.endswith("J0") for n in names), names)
                self.assertEqual(len(ins), len(f.joints) - 1)   # J1 and J2 share the tendon input
            else:
                self.assertEqual(len(ins), len(f.joints))

    def test_pose_is_held_by_its_own_inputs(self):
        m = self.m
        rng = np.random.default_rng(0)
        q = m.qpos0.copy()
        # one finger moved at a time, so no two fingers meet
        f = next(g for g in self.hand.fingers if g.label == "index")
        lo, hi = np.asarray(f.ranges).T
        q[f.qadr] = lo + (0.2 + 0.6 * rng.random(len(f.qadr))) * (hi - lo)
        ctrl = fa.ctrl_for_pose(m, q)
        d = self.mj.MjData(m)
        steps, settled = fa.settle(m, d, q, ctrl)
        self.assertTrue(settled)
        # a position servo at a free pose (no stop, no contact, no weight) holds the angle it was read from
        for f in self.hand.fingers:
            tend = [j for j in f.joints if m.joint(j).name.endswith(("J1", "J2"))]
            own = [j for j in f.joints if j not in tend]
            np.testing.assert_allclose(d.qpos[[m.jnt_qposadr[j] for j in own]], q[[m.jnt_qposadr[j] for j in own]], atol=1e-3)
            if tend:   # the tendon servo holds the sum of J1 and J2
                self.assertAlmostEqual(float(sum(d.qpos[m.jnt_qposadr[j]] for j in tend)),
                                       float(sum(q[m.jnt_qposadr[j]] for j in tend)), delta=1e-3)

    def test_closure_identity_at_a_contact(self):
        m = fa.build_scene(self.hand, [0.0, 0.0, 1.0], 0.03, friction_loss=False)
        f = next(g for g in self.hand.fingers if g.label == "index")
        # put the sphere on the index tip
        d = self.mj.MjData(m)
        self.mj.mj_forward(m, d)
        tip = f.tip_pos(d)
        nrm = np.array([0.0, 0.0, 1.0])
        m.geom_pos[self.sph] = tip + nrm * (0.03 - 0.001)
        m.body_pos[m.geom_bodyid[self.sph]] = 0.0
        unit = fa.FingerUnit(self.hand, m, f, self.sph)
        q = m.qpos0.copy()
        ctrl = fa.ctrl_for_pose(m, q)
        rd = unit.run(q, ctrl, ctrl[unit.inputs])
        self.assertTrue(rd["settled"])
        if rd["contact"]:
            cl = rd["closure"]
            self.assertLess(float(np.abs(cl["unclosed"]).max()), 1e-5)
            self.assertGreater(float(np.abs(cl["object"]).max()), 0.0)


if __name__ == "__main__":
    unittest.main()
