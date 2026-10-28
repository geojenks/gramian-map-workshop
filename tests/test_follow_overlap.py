"""follow.py scores overlap between hand and object against the task's own
allowance (ruling of 28 Oct, issue 22): the allowance is read from the
leader or overridden, the push-out leaves overlap within it alone, the
verdict never passes a finger beyond it, and the result records it
(python -m unittest). The reader and grading tests need numpy only; the
rest need mujoco and are skipped without it."""
import json
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import follow as fo  # noqa: E402

LEADER = os.path.join(ROOT, "data", "leaders", "w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n5.json")
STORED = os.path.join(ROOT, "outputs", "follow_human_vc__w10_cup_physical_x0.3_a0.0915_sl0.3t"
                      "_R33.5_n5__respaced__side.json")

try:
    import mujoco  # noqa: F401
except ImportError:
    mujoco = None


def _load(p):
    with open(p, encoding="utf-8") as fh:
        return json.load(fh)


class Reader(unittest.TestCase):
    def test_default_when_absent(self):
        a = fo.overlap_allowance({})
        self.assertEqual((a["contact_m"], a["other_m"], a["other_allowed"]),
                         (fo.OVERLAP_ALLOWED_M, fo.OVERLAP_ALLOWED_M, True))
        self.assertEqual(a["source"], {"contact": "default", "other": "default"})
        self.assertEqual(fo.OVERLAP_ALLOWED_M, 0.002)

    def test_from_the_leader(self):
        a = fo.overlap_allowance({"allowed_overlap_m": 0.001,
                                  "other_contact": {"allowed": True, "max_overlap_m": 0.0}})
        self.assertEqual((a["contact_m"], a["other_m"]), (0.001, 0.0))
        self.assertEqual(a["source"], {"contact": "leader file", "other": "leader file"})

    def test_other_contact_not_allowed_keeps_clear(self):
        a = fo.overlap_allowance({"other_contact": {"allowed": False}})
        self.assertFalse(a["other_allowed"])
        self.assertEqual(a["other_m"], -fo.TOUCH_TOL_M)

    def test_overrides_win(self):
        L = {"allowed_overlap_m": 0.001, "other_contact": {"allowed": False}}
        a = fo.overlap_allowance(L, allowed_overlap_m=0.004, other_contact=0.003)
        self.assertEqual((a["contact_m"], a["other_m"], a["other_allowed"]), (0.004, 0.003, True))
        self.assertEqual(a["source"], {"contact": "override", "other": "override"})
        self.assertFalse(fo.overlap_allowance({}, other_contact=False)["other_allowed"])

    def test_bad_values_stop(self):
        with self.assertRaises(ValueError):
            fo.overlap_allowance({"allowed_overlap_m": -0.001})
        with self.assertRaises(ValueError):
            fo.overlap_allowance({"other_contact": {"allowed": False, "max_overlap_m": 0.001}})


class Grading(unittest.TestCase):
    def test_rank(self):
        a = fo.overlap_allowance({"allowed_overlap_m": 0.002,
                                  "other_contact": {"allowed": True, "max_overlap_m": 0.001}})
        self.assertEqual(fo._overlap_rank(0.002, 0.001, a), 0)       # at the allowance: fine
        self.assertEqual(fo._overlap_rank(0.0021, 0.0, a), 1)
        self.assertEqual(fo._overlap_rank(0.0, 0.0031, a), 2)        # other: 1 + 2 mm
        self.assertEqual(fo._overlap_rank(0.0041, 0.0, a), 2)        # contact: 2 + 2 mm

    def test_no_cost_within_the_allowance(self):
        a = fo.overlap_allowance({})
        self.assertEqual(fo._over(0.0019, 0.002, a), 0.0)
        self.assertAlmostEqual(fo._over(0.003, 0.0025, a), 0.001)

    def test_old_single_allowance_is_the_whole_finger_at_once(self):
        # before 28 Oct: green to PEN_GREEN_M and amber to PEN_AMBER_M on
        # the deepest point of the whole finger
        a = fo.overlap_allowance({}, allowed_overlap_m=fo.PEN_GREEN_M)
        for pc, po in [(0.001, 0.0029), (0.0031, 0.0), (0.0, 0.0059), (0.0061, 0.0), (0.004, 0.007)]:
            pen = max(pc, po)
            old = 0 if pen <= fo.PEN_GREEN_M else 1 if pen <= fo.PEN_AMBER_M else 2
            self.assertEqual(fo._overlap_rank(pc, po, a), old, (pc, po))


@unittest.skipIf(mujoco is None, "mujoco not installed")
class PushOut(unittest.TestCase):
    def test_overlap_within_the_allowance_is_left_alone(self):
        # a pose solved with no push-out, whose links sink in by d: with an
        # allowance no step of the solve exceeds, the second try's push-out
        # must not move it at all (an allowance of just d is not enough:
        # the solve can pass deeper on its way, and the push-out then acts
        # there). solve_tangent keeps the better of its two tries.
        hand = fo.Hand("human_vc")
        L = fo.Leader(_load(LEADER))
        P = _load(STORED)
        Rw = np.array(P["placement"]["rotation_matrix"])
        t = np.array(P["placement"]["translation_m"])
        L.phi = np.radians(P["respace_deg"])
        found, shallower = 0, []
        for a_, f in enumerate(hand.fingers):
            ik = fo.FingerIK(hand, f)
            i = P["assignment"][f.label]
            for k in range(0, len(L.frames), 37):
                if i not in L.made(k):
                    continue
                p, _, c = L.station(k, i, Rw, t)
                for b in ik.proxies():
                    free = ik.solve_tangent(b, p, c, L.R_obj, ball_pen=False)
                    d = max(free[6], free[7])
                    if d <= 0.0005:
                        continue
                    found += 1
                    ik.allow = fo.overlap_allowance({}, allowed_overlap_m=0.05)
                    held = ik._solve_tangent(b, p, c, L.R_obj, None, True, fo.PUSH_USE)
                    np.testing.assert_array_equal(held[0], free[0])
                    for a in (0.0, 0.002, 0.05):
                        ik.allow = fo.overlap_allowance({}, allowed_overlap_m=a)
                        both = [ik._solve_tangent(b, p, c, L.R_obj, None, True, x)
                                for x in (fo.PUSH_CLEAR, fo.PUSH_USE)]
                        kept = ik.solve_tangent(b, p, c, L.R_obj)
                        # a green first try is kept; else the better of the two
                        want = (ik.rank(both[0]) if ik.rank(both[0])[0] == 0
                                else min(ik.rank(x) for x in both))
                        self.assertEqual(ik.rank(kept), want)
                    # with no allowance the push-out acts (it is one row in
                    # a least-squares solve against reach, so it need not
                    # end shallower in every case)
                    ik.allow = fo.overlap_allowance({}, allowed_overlap_m=0.0)
                    pushed = ik._solve_tangent(b, p, c, L.R_obj, None, True, fo.PUSH_CLEAR)
                    self.assertFalse(np.array_equal(pushed[0], free[0]))
                    shallower.append(max(pushed[6], pushed[7]) < d)
        if not found:
            self.skipTest("no pose here sinks in without the push-out")
        self.assertGreater(np.mean(shallower), 0.5)


@unittest.skipIf(mujoco is None, "mujoco not installed")
class Verdict(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        L, P = _load(LEADER), _load(STORED)
        cls.r = fo.follow("human_vc", L, n_eval=3, placement=P)
        cls.wide = fo.follow("human_vc", L, n_eval=3, placement=P, allowed_overlap_m=0.05)
        cls.stated = fo.follow("human_vc", {**L, "allowed_overlap_m": 0.001,
                                            "other_contact": {"allowed": False}},
                               n_eval=3, placement=P)

    def test_recorded(self):
        s = self.r["settings"]
        self.assertEqual(s["allowed_overlap_m"], fo.OVERLAP_ALLOWED_M)
        self.assertEqual(s["other_contact"], {"allowed": True, "max_overlap_m": fo.OVERLAP_ALLOWED_M})
        self.assertEqual(s["overlap_source"], {"contact": "default", "other": "default"})
        s = self.stated["settings"]
        self.assertEqual(s["allowed_overlap_m"], 0.001)
        self.assertEqual(s["other_contact"], {"allowed": False, "max_overlap_m": None,
                                              "clearance_m": fo.TOUCH_TOL_M})
        self.assertEqual(s["overlap_source"], {"contact": "leader file", "other": "leader file"})

    def test_every_posed_finger_reports_its_overlap(self):
        for r in (self.r, self.wide, self.stated):
            a = fo.overlap_allowance({}, r["settings"]["allowed_overlap_m"],
                                     r["settings"]["other_contact"])
            for fr in r["frames"]:
                self.assertIn("palm_overlap_mm", fr)
                for x in fr["fingers"]:
                    if x["state"] == "off":
                        continue
                    pc, po = (-np.inf if x[key] is None else x[key] / 1e3
                              for key in ("overlap_mm", "other_overlap_mm"))
                    within = fo._overlap_rank(pc, po, a) == 0
                    self.assertEqual(x["overlap_within_allowance"], within)
                    if x["state"] == "green":
                        self.assertTrue(within)

    def test_a_wide_allowance_never_fails_on_overlap(self):
        for fr in self.wide["frames"]:
            self.assertNotEqual(fr["reason"], "palm penetrates")
            for x in fr["fingers"]:
                self.assertNotEqual(x.get("reason"), "penetrates")


if __name__ == "__main__":
    unittest.main()
