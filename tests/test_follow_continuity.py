"""follow(continuity=True, speed_limit=...) (issue 12): the carried solve,
the readout, the speed test, and the default path unchanged (python -m
unittest). The hand tests need the hand models (see
gramian_workshop/hands.py) and a stored result; without them they are
skipped."""
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


def _touching(result):
    """(frame, finger record) for every finger record with a continuity readout."""
    return [(fr, x) for fr in result["frames"] for x in fr["fingers"] if "continuity" in x]


class CarriedSolve(unittest.TestCase):
    def setUp(self):
        self.hand = _hand("allegro")
        if self.hand is None or not os.path.exists(STORED):
            self.skipTest("hand models or stored result not found")
        with open(LEADER) as fh:
            self.L = json.load(fh)
        with open(STORED) as fh:
            self.P = json.load(fh)

    def _first_station(self):
        leader = fo.Leader(self.L)
        Rw = np.array(self.P["placement"]["rotation_matrix"])
        t = np.array(self.P["placement"]["translation_m"])
        k = next(k for k in range(len(leader.frames)) if leader.made(k))
        i = leader.made(k)[0]
        p, _, _ = leader.station(k, i, Rw, t)
        return leader, k, i, p, leader.center(k, Rw, t), Rw

    def test_solving_from_its_own_solution_returns_it(self):
        leader, k, i, p, c, Rw = self._first_station()
        ik = fo.FingerIK(self.hand, self.hand.fingers[0])
        (b, _, _), r = ik.best_tangent(p, c, leader.R_obj, force=leader.force(k, i, Rw))
        r2 = ik.solve_tangent(b, p, c, leader.R_obj, q0=r[0], force=leader.force(k, i, Rw))
        np.testing.assert_allclose(r2[0], r[0], atol=1e-6)

    def test_no_state_is_the_fixed_starts(self):
        leader, k, i, p, c, Rw = self._first_station()
        ik = fo.FingerIK(self.hand, self.hand.fingers[0])
        f = leader.force(k, i, Rw)
        bb, r = ik.best_tangent(p, c, leader.R_obj, force=f)
        bb2, r2, note = fo._carried_solve(ik, None, p, c, leader.R_obj, True, f)
        self.assertEqual(bb, bb2)
        np.testing.assert_allclose(r2[0], r[0])
        self.assertEqual(note, {"start": "fresh", "link_held": False})

    def test_a_carried_green_pose_is_kept(self):
        leader, k, i, p, c, Rw = self._first_station()
        ik = fo.FingerIK(self.hand, self.hand.fingers[0])
        f = leader.force(k, i, Rw)
        (b, _, _), r = ik.best_tangent(p, c, leader.R_obj, force=f)
        st = {"k": 0, "t": 0.0, "q": r[0], "body": b, "off": False}
        bb, r2, note = fo._carried_solve(ik, st, p, c, leader.R_obj, True, f)
        self.assertEqual(bb[0], b)
        np.testing.assert_allclose(r2[0], r[0], atol=1e-6)
        if ik.rank(r)[0] == 0:
            self.assertEqual(note["start"], "carried")
        # whichever start won, the pose returned is no worse by class than
        # the fixed starts' best
        self.assertLessEqual(ik.rank(r2)[0], ik.rank(r)[0])


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
        cls.a = fo.follow("allegro", cls.L, **cls.kw)
        cls.c = fo.follow("allegro", cls.L, **cls.kw, continuity=True)

    def test_off_by_default(self):
        b = fo.follow("allegro", self.L, **self.kw, continuity=False, speed_limit=None)
        self.assertEqual(self.a["frames"], b["frames"])
        self.assertNotIn("continuity", self.a["settings"])
        self.assertFalse(any("continuity" in x for fr in self.a["frames"] for x in fr["fingers"]))
        self.assertFalse(any("speed" in x.get("missed", []) for fr in self.a["frames"]
                             for x in fr["fingers"]))

    def test_on_adds_the_readout_to_every_touching_finger(self):
        self.assertIsNone(self.c["settings"]["continuity"]["speed_limit_rad_s"])
        rows = _touching(self.c)
        self.assertTrue(rows)
        for fr, x in rows:
            co = x["continuity"]
            self.assertIn(co["start"], ("carried", "fresh"))
            self.assertEqual(len(co["speed"] or x["q"]), len(x["q"]))
            self.assertIsNone(co["within_limit"])
            self.assertIsNone(co["limit_rad_s"])
        # every touching finger has one, and no off finger does
        for fr in self.c["frames"]:
            for x in fr["fingers"]:
                self.assertEqual("continuity" in x, x["state"] != "off")

    def test_first_touch_is_fresh_and_later_frames_link_back(self):
        seen = {}
        for fr in self.c["frames"]:
            for x in fr["fingers"]:
                if "continuity" not in x:
                    continue
                co = x["continuity"]
                if x["finger"] not in seen:
                    self.assertEqual(co["start"], "fresh")
                    self.assertIsNone(co["from_frame"])
                    self.assertNotIn("jump", co)
                else:
                    k0, q0, t0, off = seen[x["finger"]]
                    self.assertEqual(co["from_frame"], k0)
                    self.assertAlmostEqual(co["dt_s"], float(self.L["frames"][fr["k"]]["t"]) - t0)
                    self.assertEqual(co["spans_off"], off)
                    # the jump is this pose less the previous one, at its rate
                    for j, (qn, qp) in zip(co["jump"], zip(x["q"], q0)):
                        self.assertAlmostEqual(j["dq"], qn - qp)
                        self.assertAlmostEqual(j["rate"], (qn - qp) / co["dt_s"])
                seen[x["finger"]] = (fr["k"], x["q"], float(self.L["frames"][fr["k"]]["t"]), False)
            for x in fr["fingers"]:
                if x["state"] == "off" and x["finger"] in seen:
                    k0, q0, t0, _ = seen[x["finger"]]
                    seen[x["finger"]] = (k0, q0, t0, True)

    def test_speed_is_the_window_fit(self):
        leader = fo.Leader(self.L)
        hand = fo.Hand("allegro")
        Rw = np.array(self.P["placement"]["rotation_matrix"])
        t = np.array(self.P["placement"]["translation_m"])
        checked = 0
        for fr, x in _touching(self.c):
            co = x["continuity"]
            if co["speed"] is None:
                self.assertLess(co["speed_points"], fo.FIT_MIN_POINTS)
                continue
            f = hand.fingers[[g.label for g in hand.fingers].index(x["finger"])]
            ik = fo.FingerIK(hand, f)
            body = f.tip_body if x["phalanx"] == "distal" else int(hand.model.body_parentid[f.tip_body])
            dt, Q = fo._window_q(ik, leader, fr["k"], x["station"], Rw, t, body, np.array(x["q"]))
            self.assertEqual(len(dt), co["speed_points"])
            qd = fo._local_derivs(dt, Q)[0][ik.qadr]
            np.testing.assert_allclose([s["rate"] for s in co["speed"]], qd, rtol=1e-9, atol=1e-12)
            self.assertAlmostEqual(co["max_hinge_rad_s"], float(np.abs(qd).max()))
            checked += 1
        self.assertGreater(checked, 0)

    def test_a_tiny_limit_fails_for_speed(self):
        d = fo.follow("allegro", self.L, **self.kw, continuity=True, speed_limit=1e-12)
        self.assertEqual(d["settings"]["continuity"]["speed_limit_rad_s"], 1e-12)
        fitted = [x for fr, x in _touching(d) if x["continuity"]["speed"] is not None]
        self.assertTrue(fitted)
        for x in fitted:
            self.assertIs(x["continuity"]["within_limit"], False)
            self.assertEqual(x["state"], "red")
            self.assertIn("speed", x["missed"])
            self.assertNotIn("used_in_force_check", x)
            self.assertNotIn("unbalanced_Nm", x)        # no force test ran
        self.assertTrue(any(x["reason"] == "speed" for x in fitted))
        # a finger with no fit (too few frames in its window) is not tested
        for fr, x in _touching(d):
            if x["continuity"]["speed"] is None:
                self.assertIsNone(x["continuity"]["within_limit"])
                self.assertNotIn("speed", x["missed"])

    def test_a_huge_limit_is_continuity_alone(self):
        e = fo.follow("allegro", self.L, **self.kw, continuity=True, speed_limit=1e9)
        for fc, fe in zip(self.c["frames"], e["frames"]):
            self.assertEqual(fc["verdict"], fe["verdict"])
            for xc, xe in zip(fc["fingers"], fe["fingers"]):
                self.assertEqual(xc["state"], xe["state"])
                self.assertEqual(xc.get("q"), xe.get("q"))
                if "continuity" in xc:
                    self.assertIs(xe["continuity"]["within_limit"],
                                  None if xc["continuity"]["speed"] is None else True)

    def test_wrist_search_is_refused(self):
        if _hand("shadow") is None:
            self.skipTest("shadow model not found")
        with self.assertRaises(ValueError):
            fo.follow("shadow", self.L, n_eval=2, placement=self.P, wrist=True, continuity=True)


if __name__ == "__main__":
    unittest.main()
