"""feedforward.py: can a finger apply a demanded contact force at one
instant? The bounded least squares against brute force, the balance and the
nearest holdable force on a two-joint finger whose answers can be worked by
hand, and the Shadow hold as a smoke test: with follow.py's bounds the
unbalanced torque is follow.py's own (python -m unittest). The Shadow test
needs mujoco and the menagerie Shadow (GMW_MENAGERIE) and is skipped
without them."""
import itertools
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

from gramian_workshop import feedforward as ff  # noqa: E402

try:
    import mujoco  # noqa: F401
    from gramian_workshop.hands import Hand
    Hand("shadow")
    HAVE_SHADOW = os.path.exists(os.path.join(ROOT, "outputs", "hold_placement_shadow.json"))
except Exception:  # mujoco missing, or the menagerie not found
    HAVE_SHADOW = False


def brute(A, b, lo, hi):
    """Best feasible point over every choice of which variables sit on which bound."""
    n = A.shape[1]
    best = np.inf
    for pick in itertools.product((0, 1, 2), repeat=n):      # lo, hi, free
        x = np.zeros(n)
        free = np.array([p == 2 for p in pick])
        if any((p == 0 and not np.isfinite(lo[i])) or (p == 1 and not np.isfinite(hi[i]))
               for i, p in enumerate(pick)):
            continue
        for i, p in enumerate(pick):
            if p == 0:
                x[i] = lo[i]
            elif p == 1:
                x[i] = hi[i]
        if free.any():
            x[free] = np.linalg.lstsq(A[:, free], b - A[:, ~free] @ x[~free], rcond=None)[0]
        if np.all(x >= lo - 1e-12) and np.all(x <= hi + 1e-12):
            best = min(best, float(np.linalg.norm(A @ x - b)))
    return best


class BoundedLeastSquares(unittest.TestCase):
    def test_matches_brute_force(self):
        rng = np.random.default_rng(0)
        for trial in range(200):
            n, mrows = int(rng.integers(1, 5)), int(rng.integers(1, 6))
            A = rng.normal(size=(mrows, n))
            if trial % 2:                                    # badly scaled rows and columns
                A = A * 10.0 ** rng.integers(-4, 5, size=(mrows, 1)) * 10.0 ** rng.integers(-3, 4, size=(1, n))
            b = A @ rng.normal(size=n) + rng.normal(size=mrows) * np.abs(A).max()
            lo = -np.abs(rng.normal(size=n))
            hi = np.abs(rng.normal(size=n))
            lo[rng.random(n) < 0.2] = -np.inf
            hi[rng.random(n) < 0.2] = np.inf
            x = ff.bvls(A, b, lo, hi)
            self.assertTrue(np.all(x >= lo) and np.all(x <= hi))
            got, want = float(np.linalg.norm(A @ x - b)), brute(A, b, lo, hi)
            self.assertLessEqual(got, want * (1 + 1e-7) + 1e-9 * np.linalg.norm(b))

    def test_bounds_out_of_order(self):
        with self.assertRaises(ValueError):
            ff.bvls(np.eye(2), np.ones(2), np.array([1.0, 0.0]), np.array([0.0, 1.0]))


def two_joint(a_lo=-1.0, a_hi=1.0, friction=0.0, stops=()):
    """A planar finger: joint 1 feels fx, joint 2 feels fy, and one input (a
    tendon) puts the same torque on both."""
    return {"J": np.array([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]]), "inputs": [0], "B": np.array([[1.0, 1.0]]),
            "a_lo": np.array([a_lo]), "a_hi": np.array([a_hi]), "a_src": [["force range", "force range"]],
            "stops": list(stops), "friction": np.full(2, friction), "passive": np.zeros(2)}


class TwoJointFinger(unittest.TestCase):
    """One tendon over two joints holds only forces that load both equally."""

    def test_force_it_can_hold(self):
        T, f = two_joint(), np.array([0.1, 0.1, 0.0])
        b, nr = ff.balance(T, f, False), ff.nearest_force(T, f, False)
        self.assertAlmostEqual(b["unbalanced_Nm"], 0.0, places=9)
        self.assertAlmostEqual(b["input_force"][0], 0.1, places=7)
        self.assertAlmostEqual(nr["departure_N"], 0.0, places=7)

    def test_force_across_it(self):
        T, f = two_joint(), np.array([0.1, 0.0, 0.0])
        b, nr = ff.balance(T, f, False), ff.nearest_force(T, f, False)
        self.assertAlmostEqual(b["unbalanced_Nm"], 0.1 / np.sqrt(2), places=9)
        np.testing.assert_allclose(nr["force_N"], [0.05, 0.05, 0.0], atol=1e-7)
        self.assertAlmostEqual(nr["departure_N"], 0.1 / np.sqrt(2), places=7)
        self.assertLess(nr["balance_left_Nm"], 1e-7)

    def test_friction_widens_what_it_holds(self):
        T, f = two_joint(friction=0.01), np.array([0.1, 0.0, 0.0])
        b, nr = ff.balance(T, f, True), ff.nearest_force(T, f, True)
        self.assertAlmostEqual(b["unbalanced_Nm"], 0.08 / np.sqrt(2), places=9)
        np.testing.assert_allclose(nr["force_N"], [0.06, 0.04, 0.0], atol=1e-6)
        np.testing.assert_allclose(b["friction_share"], [1.0, 1.0], atol=1e-6)
        # not credited: the same finger as without friction
        self.assertAlmostEqual(ff.balance(T, f, False)["unbalanced_Nm"], 0.1 / np.sqrt(2), places=9)

    def test_friction_used_no_more_than_needed(self):
        T, f = two_joint(friction=0.01), np.array([0.104, 0.1, 0.0])
        b = ff.balance(T, f, True)
        self.assertAlmostEqual(b["unbalanced_Nm"], 0.0, places=8)
        np.testing.assert_allclose(b["friction_share"], [0.2, 0.2], atol=1e-3)   # 0.002 each, opposite
        self.assertAlmostEqual(ff.nearest_force(T, f, True)["departure_N"], 0.0, places=7)

    def test_stop_takes_one_sense_only(self):
        T = two_joint(stops=[(1, "lo", 1.0)])                # joint 2 on its lower stop
        self.assertAlmostEqual(ff.balance(T, [0.0, 0.1, 0.0], False)["unbalanced_Nm"], 0.0, places=9)
        self.assertAlmostEqual(ff.balance(T, [0.0, 0.1, 0.0], False)["stop_Nm"][1], 0.1, places=7)
        self.assertAlmostEqual(ff.balance(T, [0.1, 0.0, 0.0], False)["unbalanced_Nm"], 0.1 / np.sqrt(2), places=9)

    def test_input_range_limits_it(self):
        T, f = two_joint(a_lo=0.0, a_hi=0.03), np.array([0.1, 0.1, 0.0])
        b, nr = ff.balance(T, f, False), ff.nearest_force(T, f, False)
        self.assertAlmostEqual(b["unbalanced_Nm"], 0.07 * np.sqrt(2), places=9)
        np.testing.assert_allclose(nr["force_N"], [0.03, 0.03, 0.0], atol=1e-7)

    def test_describe_force(self):
        dsc = ff.describe_force([0.1, 0.1, 0.0], [0.1, 0.0, 0.0], [1.0, 0.0, 0.0])
        self.assertAlmostEqual(dsc["angle_to_demanded_deg"], 45.0, places=9)
        self.assertAlmostEqual(dsc["shear_over_normal"], 1.0, places=9)


@unittest.skipUnless(HAVE_SHADOW, "needs mujoco, the menagerie Shadow and the stored hold placement")
class ShadowHold(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import hold_shadow_feedforward as tool
        cls.tool = tool
        cls.out = tool.run("shadow", frames=1)

    def test_follow_bounds_reproduce_follow(self):
        for x in self.out["frames"][0]["fingers"]:
            v = x["variants"]["follow_bounds"]
            self.assertAlmostEqual(v["unbalanced_Nm"], x["follow_unbalanced_Nm"], places=6, msg=x["finger"])

    def test_more_means_never_worse(self):
        for x in self.out["frames"][0]["fingers"]:
            fb, nf, am = (x["variants"][k] for k in ("follow_bounds", "no_joint_friction", "as_modelled"))
            self.assertLessEqual(fb["unbalanced_Nm"], nf["unbalanced_Nm"] + 1e-9, msg=x["finger"])
            self.assertLessEqual(am["unbalanced_Nm"], nf["unbalanced_Nm"] + 1e-9, msg=x["finger"])
            self.assertLessEqual(am["nearest"]["departure_N"], nf["nearest"]["departure_N"] + 1e-7, msg=x["finger"])

    def test_answers_agree_with_each_other(self):
        for x in self.out["frames"][0]["fingers"]:
            for name, v in x["variants"].items():
                msg = f"{x['finger']} {name}"
                self.assertLess(v["nearest"]["balance_left_Nm"], 1e-6 * v["needed_norm_Nm"], msg=msg)
                if v["unbalanced_Nm"] < 1e-9:                 # it balances the force: nothing to depart by
                    self.assertLess(v["nearest"]["departure_N"], 1e-6, msg=msg)
                    self.assertEqual(v["force"], "yes", msg=msg)
                else:
                    self.assertGreater(v["nearest"]["departure_N"], 1e-6, msg=msg)
                c = v["carried"]
                self.assertTrue(np.all(np.asarray(c["input_force"]) >= np.asarray(v["input_force_lo"]) - 1e-9), msg=msg)
                self.assertTrue(np.all(np.asarray(c["input_force"]) <= np.asarray(v["input_force_hi"]) + 1e-9), msg=msg)
                self.assertTrue(np.all(np.abs(c["friction_Nm"]) <= np.asarray(v["frictionloss_Nm"]) + 1e-9), msg=msg)

    def test_report_builds_from_stored_form(self):
        import json
        text = self.tool.table(json.loads(json.dumps(self.out, default=self.tool._enc)))
        self.assertIn("| thumb |", text)


if __name__ == "__main__":
    unittest.main()
