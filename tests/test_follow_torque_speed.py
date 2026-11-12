"""follow(torque_speed=True): the no-load speed figures, the straight-line
derating, its intersection with the actuator link's box, and the whole
check on a hand (python -m unittest). The hand tests need the hand models
(see gramian_workshop/hands.py) and a stored result; without them they are
skipped."""
import json
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import follow as fo  # noqa: E402
from gramian_workshop import hands  # noqa: E402

LEADER = os.path.join(ROOT, "data", "leaders", "w10_cup_physical_x0.3_a0.15_sl0.3t_n4.json")
STORED = os.path.join(ROOT, "outputs", "follow_allegro__w10_cup_physical_x0.3_a0.15_sl0.3t_n4.json")


def _hand(key):
    try:
        return fo.Hand(key)
    except Exception:          # models not fetched
        return None


def _touching(result):
    return [(fr, x) for fr in result["frames"] for x in fr["fingers"] if "continuity" in x]


def _tested(x):
    """Whether the finger's torque test ran (it leaves the unbalanced torque)."""
    return "unbalanced_Nm" in x or "unbalanced_N" in x


class NoLoadSpeed(unittest.TestCase):
    def test_published_figures_convert_to_rad_s(self):
        w, rec = hands.no_load_rad_s(hands.HANDS["leap"])
        self.assertAlmostEqual(w, 81.0 * 2.0 * np.pi / 60.0)
        self.assertEqual((rec["value"], rec["unit"]), (81.0, "rev/min"))
        self.assertAlmostEqual(rec["rad_s"], w)
        w, rec = hands.no_load_rad_s(hands.HANDS["allegro"])
        self.assertAlmostEqual(w, (np.pi / 3.0) / 0.11)
        self.assertEqual(rec["unit"], "s/60deg")
        self.assertTrue(rec["source"])

    def test_hands_without_a_figure_give_none(self):
        for key in ("shadow", "l20", "l20_public"):
            if key in hands.HANDS:
                self.assertIsNone(hands.no_load_rad_s(hands.HANDS[key]))

    def test_bad_figures_are_refused(self):
        with self.assertRaises(ValueError):
            hands.no_load_rad_s({"datasheet_no_load_speed": (1.0, "furlongs/fortnight", "x")})
        with self.assertRaises(ValueError):
            hands.no_load_rad_s({"datasheet_no_load_speed": (0.0, "rad/s", "x")})

    def test_every_unit_is_a_speed(self):
        for unit, f in hands.SPEED_UNITS.items():
            self.assertGreater(float(f(1.0)), 0.0, unit)
        self.assertAlmostEqual(hands.SPEED_UNITS["deg/s"](180.0), np.pi)
        self.assertAlmostEqual(hands.SPEED_UNITS["s/60deg"](1.0), np.pi / 3.0)


def _speed(rates, kinds=None):
    kinds = kinds or ["hinge"] * len(rates)
    return [{"joint": f"j{i}", "kind": k, "rate": r} for i, (k, r) in enumerate(zip(kinds, rates))]


class StraightLine(unittest.TestCase):
    lim = np.array([[-0.7, 0.7], [-0.5, 0.5], [-0.3, 0.3]])

    def test_at_rest_nothing_changes(self):
        ts = fo._speed_limits(self.lim, _speed([0.0, 0.0, 0.0]), 2.0)
        self.assertFalse(ts["derated"])
        np.testing.assert_allclose(ts["lo"], self.lim[:, 0])
        np.testing.assert_allclose(ts["hi"], self.lim[:, 1])
        for r in ts["joints"]:
            self.assertIsNone(r["speed_share"])
            self.assertEqual(r["limit_rest"], r["limit_at_speed"])

    def test_half_the_no_load_speed_halves_the_driving_side_only(self):
        ts = fo._speed_limits(self.lim, _speed([1.0, -1.0, 0.0]), 2.0)
        self.assertTrue(ts["derated"])
        np.testing.assert_allclose(ts["hi"], [0.35, 0.5, 0.3])      # j0 drives +: hi halves
        np.testing.assert_allclose(ts["lo"], [-0.7, -0.25, -0.3])   # j1 drives -: lo halves
        self.assertAlmostEqual(ts["joints"][0]["speed_share"], 0.5)
        self.assertEqual(ts["joints"][0]["limit_rest"], [-0.7, 0.7])
        self.assertEqual(ts["joints"][0]["limit_at_speed"], [-0.7, 0.35])

    def test_at_and_beyond_no_load_the_driving_side_is_zero_never_negative(self):
        ts = fo._speed_limits(self.lim, _speed([2.0, -5.0, 0.1]), 2.0)
        self.assertEqual(ts["hi"][0], 0.0)
        self.assertEqual(ts["lo"][1], 0.0)
        self.assertTrue(np.all(ts["lo"] <= 0.0) and np.all(ts["hi"] >= 0.0))
        self.assertAlmostEqual(ts["hi"][2], 0.3 * 0.95)

    def test_the_derated_side_is_the_motor_torque_in_the_sense_of_motion(self):
        # one hinge, lim [-1, 1], turning positive at the no-load speed: its
        # motor can push nothing positive but can still brake with the full
        # negative torque. _hold_solve's t is the motor torque the contact
        # force needs (E^T J^T f, f the finger's force on the object), so
        # with Tr = I a positive t is a motor push in the sense of motion
        lim = np.array([[-1.0, 1.0]])
        ts = fo._speed_limits(lim, _speed([2.0]), 2.0)
        lo, hi, _ = fo._speed_box(ts)
        ncols = len(fo._input_senses(lim))
        box = fo._input_box(lim, ncols, lo, hi)
        cols = (np.eye(1), np.array([[1.0, -1.0]]))     # push column, pull column
        ok_drive, _, _ = fo._hold_solve(cols, np.array([0.5]), None, box)
        ok_brake, _, _ = fo._hold_solve(cols, np.array([-0.5]), None, box)
        self.assertFalse(ok_drive)      # the driving side is spent at this speed
        self.assertTrue(ok_brake)       # the braking side is untouched
        box_rest = fo._input_box(lim, ncols, lim[:, 0], lim[:, 1])
        self.assertTrue(fo._hold_solve(cols, np.array([0.5]), None, box_rest)[0])

    def test_a_slide_keeps_its_limits(self):
        ts = fo._speed_limits(self.lim, _speed([1.0, 1.0, 1.0], ["hinge", "slide", "hinge"]), 2.0)
        np.testing.assert_allclose(ts["hi"], [0.35, 0.5, 0.15])
        self.assertIsNone(ts["joints"][1]["no_load_rad_s"])
        self.assertIsNone(ts["joints"][1]["speed_share"])

    def test_no_link_box_is_the_limits_at_speed(self):
        ts = fo._speed_limits(self.lim, _speed([1.0, -1.0, 0.0]), 2.0)
        lo, hi, over = fo._speed_box(ts)
        np.testing.assert_allclose(lo, ts["lo"])
        np.testing.assert_allclose(hi, ts["hi"])
        self.assertEqual(over, [])

    def test_link_box_is_intersected_and_speed_wins_a_conflict(self):
        ts = fo._speed_limits(self.lim, _speed([1.0, -1.0, 0.0]), 2.0)
        # j0: link wants [0.5, 0.7] but speed allows at most 0.35 -> pinned at 0.35
        # j1: link [-0.4, -0.3] against speed lo -0.25 -> pinned at -0.25
        # j2: link [-0.1, 0.1] inside the limits -> the link's box
        u_lo = np.array([0.5, -0.4, -0.1])
        u_hi = np.array([0.7, -0.3, 0.1])
        lo, hi, over = fo._speed_box(ts, u_lo, u_hi)
        self.assertEqual(over, [0, 1])
        np.testing.assert_allclose(lo, [0.35, -0.25, -0.1])
        np.testing.assert_allclose(hi, [0.35, -0.25, 0.1])
        self.assertTrue(np.all(lo <= hi + 1e-12))

    def test_one_per_joint(self):
        class F:
            joints = [3, 5, 8]

        class IK:
            free = [3, 5, 8]
        self.assertTrue(fo._one_per_joint(np.eye(3), F, IK))
        self.assertFalse(fo._one_per_joint(np.ones((3, 3)), F, IK))
        self.assertFalse(fo._one_per_joint(np.eye(3)[:2], F, IK))
        IK.free = [8, 3, 5]
        self.assertTrue(fo._one_per_joint(np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]]), F, IK))


class Follow(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if _hand("allegro") is None or not os.path.exists(STORED):
            raise unittest.SkipTest("hand models or stored result not found")
        with open(LEADER) as fh:
            cls.L = json.load(fh)
        with open(STORED) as fh:
            cls.P = json.load(fh)
        cls.kw = dict(n_eval=5, placement=cls.P)
        cls.c = fo.follow("allegro", cls.L, **cls.kw, continuity=True)
        cls.t = fo.follow("allegro", cls.L, **cls.kw, continuity=True, torque_speed=True)

    def test_off_by_default(self):
        self.assertNotIn("torque_speed", self.c["settings"])
        self.assertFalse(any("torque_speed" in x for fr in self.c["frames"] for x in fr["fingers"]))
        self.assertFalse(any("torque_speed" in fr for fr in self.c["frames"]))
        self.assertFalse(any("torque at speed" in x.get("missed", []) for fr in self.c["frames"]
                             for x in fr["fingers"]))

    def test_on_records_each_fitted_fingers_limits(self):
        st = self.t["settings"]["torque_speed"]
        self.assertTrue(st["applied"])
        self.assertAlmostEqual(st["no_load"]["rad_s"], (np.pi / 3.0) / 0.11)
        self.assertNotIn("why_not", st)
        rows = _touching(self.t)
        self.assertTrue(rows)
        seen = 0
        for fr, x in rows:
            # the limits are recorded where the torque test ran on a fitted finger
            if x["continuity"]["speed"] is None or not _tested(x):
                self.assertNotIn("torque_speed", x)
                continue
            self.assertIn("torque_speed", x)
            ts = x["torque_speed"]
            self.assertEqual(len(ts["joints"]), len(x["q"]))
            for r, s in zip(ts["joints"], x["continuity"]["speed"]):
                self.assertEqual(r["joint"], s["joint"])
                self.assertEqual(r["rate_rad_s"], s["rate"])
                lo0, hi0 = r["limit_rest"]
                lo1, hi1 = r["limit_at_speed"]
                self.assertLessEqual(lo0, lo1)
                self.assertLessEqual(lo1, 0.0)
                self.assertLessEqual(0.0, hi1)
                self.assertLessEqual(hi1, hi0)
                if r["rate_rad_s"] > 0:
                    self.assertEqual(lo0, lo1)
                elif r["rate_rad_s"] < 0:
                    self.assertEqual(hi0, hi1)
            self.assertEqual(len(ts["u_lo"]), len(ts["u_hi"]))
            self.assertTrue(all(a <= b + 1e-12 for a, b in zip(ts["u_lo"], ts["u_hi"])))
            self.assertEqual(ts["link_overridden"], [])        # no actuator link in this run
            if ts["derated"]:
                self.assertIn("fails_only_at_speed", ts)
            seen += 1
        self.assertGreater(seen, 0)
        for fr in self.t["frames"]:
            self.assertIn("torque_speed", fr)
            self.assertIsInstance(fr["torque_speed"]["derated_fingers"], list)

    def test_a_huge_no_load_speed_is_continuity_alone(self):
        spec = hands.HANDS["allegro"]
        keep = spec["datasheet_no_load_speed"]
        spec["datasheet_no_load_speed"] = (1e9, "rad/s", "test")
        try:
            e = fo.follow("allegro", self.L, **self.kw, continuity=True, torque_speed=True)
        finally:
            spec["datasheet_no_load_speed"] = keep
        for fc, fe in zip(self.c["frames"], e["frames"]):
            self.assertEqual(fc["verdict"], fe["verdict"])
            self.assertEqual(fc["reason"], fe["reason"])
            for xc, xe in zip(fc["fingers"], fe["fingers"]):
                self.assertEqual(xc["state"], xe["state"])
                self.assertEqual(xc.get("missed"), xe.get("missed"))
                if "torque_speed" in xe:
                    self.assertNotIn("torque at speed", xe["missed"])

    def test_a_tiny_no_load_speed_fails_for_torque_at_speed(self):
        spec = hands.HANDS["allegro"]
        keep = spec["datasheet_no_load_speed"]
        spec["datasheet_no_load_speed"] = (1e-9, "rad/s", "test")
        try:
            d = fo.follow("allegro", self.L, **self.kw, continuity=True, torque_speed=True)
        finally:
            spec["datasheet_no_load_speed"] = keep
        fitted = [x for fr, x in _touching(d) if "torque_speed" in x]
        self.assertTrue(fitted)
        for x in fitted:
            for r in x["torque_speed"]["joints"]:
                if r["speed_share"] is None or r["speed_share"] < 1.0:
                    continue       # a joint fitted at (numerically) rest keeps its limits
                if r["rate_rad_s"] > 0:
                    self.assertEqual(r["limit_at_speed"][1], 0.0)
                else:
                    self.assertEqual(r["limit_at_speed"][0], 0.0)
        at_speed = [x for x in fitted if "torque at speed" in x["missed"]]
        self.assertTrue(at_speed)
        for x in at_speed:
            self.assertIs(x["torque_speed"]["fails_only_at_speed"], True)
            self.assertNotIn("torque", x["missed"])
        # a finger that fails at the derated limits but would also fail at
        # rest says "torque", as before
        for x in fitted:
            if "torque" in x["missed"]:
                self.assertIs(x["torque_speed"]["fails_only_at_speed"], False)
        frames_with = [fr for fr in d["frames"] if fr["torque_speed"]["derated_fingers"]]
        self.assertTrue(frames_with)
        self.assertTrue(any(fr["torque_speed"]["fails_only_at_speed"] is not None
                            for fr in frames_with))

    def test_a_hand_without_a_figure_is_unchanged_and_says_why(self):
        # the Allegro with its figure taken away stands for a hand that has
        # none published (shadow, the L20s, the human hands)
        spec = hands.HANDS["allegro"]
        keep = spec.pop("datasheet_no_load_speed")
        try:
            t = fo.follow("allegro", self.L, **self.kw, continuity=True, torque_speed=True)
        finally:
            spec["datasheet_no_load_speed"] = keep
        st = t["settings"]["torque_speed"]
        self.assertFalse(st["applied"])
        self.assertIsNone(st["no_load"])
        self.assertIn("no no-load speed", st["why_not"])
        self.assertEqual(self.c["frames"], t["frames"])
        self.assertIsNone(hands.no_load_rad_s(hands.HANDS["shadow"]))

    def test_the_record_is_json(self):
        s = json.dumps(self.t, default=float)
        self.assertIn('"torque_speed"', s)

    def test_with_the_actuator_link_the_bounds_still_nest(self):
        # the derated limits intersect the link's reachable range; the
        # record says where the speed bound won
        t = fo.follow("allegro", self.L, **self.kw, continuity=True, torque_speed=True,
                      rate_limit=1e-3)
        rows = [x["torque_speed"] for fr, x in _touching(t) if "torque_speed" in x]
        self.assertTrue(rows)
        for ts in rows:
            self.assertTrue(all(a <= b + 1e-12 for a, b in zip(ts["u_lo"], ts["u_hi"])))
            for name in ts["link_overridden"]:
                self.assertIn(name, [r["joint"] for r in ts["joints"]])

    def test_refusals(self):
        with self.assertRaises(ValueError):
            fo.follow("allegro", self.L, **self.kw, torque_speed=True)
        with self.assertRaises(ValueError):
            fo.follow("allegro", self.L, **self.kw, continuity=True, torque_speed=True,
                      top_speed=True)

    def test_a_hand_with_actuator_moment_limits_is_refused(self):
        # the shadow's limits are on actuator moments, not one per joint; a
        # speed figure given to it cannot be applied joint by joint
        if _hand("shadow") is None:
            self.skipTest("shadow model not found")
        spec = hands.HANDS["shadow"]
        self.assertNotIn("datasheet_no_load_speed", spec)
        spec["datasheet_no_load_speed"] = (1.0, "rad/s", "test")
        try:
            with self.assertRaises(ValueError) as cm:
                fo.follow("shadow", self.L, n_eval=2, continuity=True, torque_speed=True)
        finally:
            del spec["datasheet_no_load_speed"]
        self.assertIn("actuator moments", str(cm.exception))


if __name__ == "__main__":
    unittest.main()
