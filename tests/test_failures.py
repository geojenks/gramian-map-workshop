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
