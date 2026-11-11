"""The failing-fingers-and-joints reporter (gramian_workshop/failures.py) on
a made-up result: it needs neither mujoco nor a hand model, and restates
only what the result says."""
import glob
import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import failures  # noqa: E402


def finger(name, state, **kw):
    return dict({"finger": name, "station": 0, "state": state}, **kw)


RESULT = {
    "hand": "made_up", "hand_name": "Made-up hand", "leader": "made_up_leader",
    "resid_tol": 0.05,
    "frames": [
        {"k": 0, "verdict": "green", "reason": None, "unserved": [], "fingers": [
            finger("a", "green", reach_mm=0.5, unbalanced_Nm=0.0),
            finger("b", "off")]},
        {"k": 5, "verdict": "amber", "reason": None, "unserved": [], "fingers": [
            finger("a", "amber", reach_mm=4.0, unbalanced_Nm=0.002,
                   beyond_limit=[{"joint": "a_j2", "side": "hi", "by": 0.01}]),
            finger("b", "green", stops=[{"joint": "b_j1", "side": "lo", "reaction": 0.004,
                                         "decided_by": "finger"}])]},
        {"k": 10, "verdict": "amber", "reason": None, "unserved": [], "fingers": [
            finger("a", "amber", reach_mm=6.0, unbalanced_Nm=0.003,
                   beyond_limit=[{"joint": "a_j2", "side": "hi", "by": 0.02}]),
            finger("b", "green")]},
        {"k": 15, "verdict": "red", "reason": "torque", "unserved": [1], "fingers": [
            finger("a", "red", reach_mm=20.0, reason="reach"),
            finger("b", "red", reach_mm=1.0, unbalanced_Nm=0.01)]},
    ],
}


def rows(*jr):
    return [{"joint": j, "kind": k, "residual": r, "tolerance": t} for j, k, r, t in jr]


# the same shape with the keys follow.py writes since issue 29
NEW = {
    "hand": "made_up", "leader": "made_up_leader",
    "frames": [
        {"k": 0, "verdict": "amber", "reason": None, "fingers": [
            finger("a", "amber", unbalanced_Nm=0.003, missed=["reach", "torque"],
                   beyond_limit=[{"joint": "a_j2", "side": "hi", "by": 0.01, "kind": "hinge"}],
                   joint_residuals={"finger": rows(("a_j1", "hinge", 0.001, 0.0005),
                                                   ("a_j2", "hinge", -0.003, 0.0005),
                                                   ("a_s", "slide", 0.2, 0.01))}),
            finger("b", "green", missed=[],
                   stops=[{"joint": "b_s", "side": "lo", "kind": "slide", "reaction": 0.5,
                           "decided_by": "finger"}],
                   joint_residuals={"finger": rows(("b_j1", "hinge", 0.0, 0.0005))})]},
        {"k": 1, "verdict": "amber", "reason": None, "fingers": [
            finger("a", "amber", unbalanced_Nm=0.002, missed=["torque"],
                   joint_residuals={"finger": rows(("a_j1", "hinge", -0.002, 0.0004),
                                                   ("a_j2", "hinge", 0.001, 0.0004),
                                                   ("a_s", "slide", 0.1, 0.01))}),
            finger("b", "green", missed=[])]},
        {"k": 2, "verdict": "red", "reason": "torque", "fingers": [
            finger("a", "red", unbalanced_Nm=0.0, missed=[],
                   joint_residuals={"redistribution": rows(("a_j1", "hinge", 0.004, 0.002),
                                                           ("a_j2", "hinge", -0.006, 0.002))}),
            finger("b", "red", reason="reach", missed=["reach"])]},
    ],
}


class JointResiduals(unittest.TestCase):
    def setUp(self):
        self.s = failures.summarise(NEW)
        self.amber, self.red = self.s["fingers"]["a"]

    def test_missed_is_the_union_in_test_order(self):
        self.assertEqual(self.amber["missed"], ["reach", "torque"])
        self.assertEqual(self.red["missed"], [])
        self.assertEqual(self.s["fingers"]["b"][-1]["missed"], ["reach"])

    def test_largest_residual_by_size_signed_and_by_joint_type(self):
        r = {x["joint"]: x for x in self.amber["residuals"]["finger"]}
        self.assertEqual((r["a_j2"]["residual"], r["a_j2"]["at"]), (-0.003, 0))
        self.assertEqual((r["a_j1"]["residual"], r["a_j1"]["at"]), (-0.002, 1))
        self.assertEqual(r["a_j2"]["tolerance"], (0.0004, 0.0005))
        self.assertEqual([x["joint"] for x in r.values() if x["largest_of_kind"]],
                         ["a_j2", "a_s"])
        self.assertNotIn("redistribution", self.amber["residuals"])

    def test_whole_hand_rows_without_own_rows(self):
        self.assertEqual(list(self.red["residuals"]), ["redistribution"])
        b = failures.brief(self.s)
        self.assertIn("in the whole-hand test, largest residual at a_j2, -0.006 N.m "
                      "(the bar on that test's whole residual: 0.002)", b)
        self.assertIn("amber frames 0 to 1 (2): missed reach, torque; largest residual at "
                      "a_j2, -0.003 N.m (the bar on the norm of its rows: 0.0004 to "
                      "0.0005 N.m); largest residual at a_s, 0.2 N", b)

    def test_equal_residuals_share_the_name_and_noise_names_none(self):
        r = {"frames": [{"k": 0, "verdict": "amber", "fingers": [
            finger("a", "amber", missed=["torque"], joint_residuals={"finger": rows(
                ("j1", "hinge", 0.001, 0.0004), ("j2", "hinge", -0.001, 0.0004),
                ("j3", "hinge", 2e-16, 0.0004))}),
            finger("b", "amber", missed=["reach"], joint_residuals={"finger": rows(
                ("k1", "hinge", 3e-16, 0.0004), ("k2", "hinge", -1e-16, 0.0004))})]}]}
        s = failures.summarise(r)
        b = failures.brief(s)
        self.assertIn("largest residual at j1 and j2, equal in size, 0.001 N.m", b)
        self.assertNotIn("k1", b)
        t = failures.to_text(s)
        self.assertIn("| j3 | 0 | 0 | 0.0004 N.m |  |", t)
        self.assertIn("| k1 | 0 | 0 | 0.0004 N.m |  |", t)

    def test_units_come_from_the_joint_kind(self):
        self.assertIn("a_j2: past its upper limit by 0.01 rad, frame 0", failures.brief(self.s))
        t = failures.to_text(self.s)
        self.assertIn("| b_s | b | frame 0 | on stop | lower |  | 0.5 N | finger |", t)
        self.assertIn("| a | frames 0 to 1 (2) | the finger's own torque test | a_j2 | "
                      "-0.003 N.m | 0 | 0.0004 to 0.0005 N.m | yes |", t)
        self.assertIn("| a | frame 2 | the whole-hand torque test | a_j1 | 0.004 N.m | 2 | "
                      "0.002 |  |", t)
        self.assertNotIn("is not in this result file", t)
        self.assertNotIn("over its bar", t)
        page = failures.to_html(self.s)
        self.assertIn("missed: reach, torque", page)
        self.assertNotIn("is not in this result file", page)


class Failures(unittest.TestCase):
    def setUp(self):
        self.s = failures.summarise(RESULT)

    def test_finger_spans(self):
        a = self.s["fingers"]["a"]
        self.assertEqual([(x["k0"], x["k1"], x["state"], x["cause"]) for x in a],
                         [(0, 0, "green", "green"), (5, 10, "amber", "amber"),
                          (15, 15, "red", "reach")])
        self.assertEqual(a[1]["reach_mm"], (4.0, 6.0))
        self.assertEqual(a[2]["stated_by"], failures.OWN)

    def test_red_without_own_reason_takes_the_frames(self):
        b = self.s["fingers"]["b"][-1]
        self.assertEqual((b["cause"], b["stated_by"]), ("torque", failures.WHOLE))

    def test_joints(self):
        j = self.s["joints"]
        self.assertEqual(list(j), ["a_j2", "b_j1"])
        self.assertEqual((j["a_j2"][0]["k0"], j["a_j2"][0]["k1"], j["a_j2"][0]["by"]),
                         (5, 10, (0.01, 0.02)))
        self.assertEqual((j["b_j1"][0]["kind"], j["b_j1"][0]["n"]), ("on stop", 1))

    def test_frames(self):
        self.assertEqual([(f["verdict"], f["n"]) for f in self.s["frames"]],
                         [("green", 1), ("amber", 2), ("red", 1)])
        self.assertEqual(self.s["frames"][-1]["unserved"], [1])

    def test_text_and_page(self):
        b = failures.brief(self.s)
        self.assertIn("a: red (reach) frame 15; amber in 2 of 4 frames", b)
        self.assertIn("a_j2: past its upper limit by 0.01 to 0.02, frames 5 to 10 (2)", b)
        self.assertIn("| a_j2 | a | frames 5 to 10 (2) | past limit | upper |",
                      failures.to_text(self.s))
        page = failures.to_html(self.s)
        self.assertTrue(page.lstrip().lower().startswith("<!doctype html"))
        self.assertIn("a_j2, frame 5: past its upper limit by 0.01", page)

    def test_empty_and_old_results(self):
        s = failures.summarise({"hand": "h", "frames": []})
        self.assertIn("no finger is amber or red", failures.brief(s))
        failures.to_text(s), failures.to_html(s)

    def test_repeated_frame_index_is_named_by_position(self):
        r = dict(RESULT, frames=[dict(f, k=0) for f in RESULT["frames"]])
        s = failures.summarise(r)
        self.assertEqual(s["ks"], [0, 1, 2, 3])
        self.assertIn("a_j2: past its upper limit by 0.01 to 0.02, frames 1 to 2 (2)",
                      failures.brief(s))
        self.assertIn("numbered by their position", failures.to_text(s))
        self.assertFalse(self.s["by_position"])

    def test_old_result_says_the_joint_is_not_named(self):
        self.assertFalse(self.s["has_joint_residuals"])
        self.assertIsNone(self.s["fingers"]["a"][1]["missed"])
        t = failures.to_text(self.s)
        self.assertIn("falls short is not in this result file", t)
        self.assertNotIn("Joint balance left over", t)
        self.assertIn("past its upper limit by 0.01 to 0.02, frames", failures.brief(self.s))

    def test_never_overwrites(self):
        with tempfile.TemporaryDirectory() as d:
            _s, paths = failures.write(RESULT, "follow_made_up", d)
            self.assertTrue(all(os.path.exists(p) for p in paths))
            with self.assertRaises(FileExistsError):
                failures.write(RESULT, "follow_made_up", d)
            failures.write(RESULT, "follow_made_up", d, tag="again")

    def test_stored_results_reconcile(self):
        """Every stored result reads without error, and the spans add back
        up to the finger states in the file."""
        for p in sorted(glob.glob(os.path.join(ROOT, "outputs", "follow_*.json"))):
            with open(p, encoding="utf-8") as f:
                r = json.load(f)
            s = failures.summarise(r)
            want = {}
            for fr in r["frames"]:
                for x in fr["fingers"]:
                    want[x["state"]] = want.get(x["state"], 0) + 1
            self.assertEqual(s["finger_states"], want, p)
            failures.brief(s), failures.to_text(s), failures.to_html(s)


if __name__ == "__main__":
    unittest.main()
