"""follow.py stores what a placement depends on (its placement key), so a
later run can reuse the placement instead of searching again
(tools/run_follow.py --reuse-placement). The key must match for the same
hand, leader and search, and differ when any of them changes; re-scoring
a stored placement must give the stored verdicts (python -m unittest).
Needs mujoco; skipped without it."""
import importlib.util
import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import follow as fo  # noqa: E402

LEADER = os.path.join(ROOT, "data", "leaders", "w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4.json")
# a tiny search, so the test runs in seconds
TINY = {"n_frames": 2, "n_samples": 3, "iters": 1}

try:
    import mujoco  # noqa: F401
except ImportError:
    mujoco = None


def _load(p):
    with open(p, encoding="utf-8") as fh:
        return json.load(fh)


def _run_follow():
    spec = importlib.util.spec_from_file_location("run_follow",
                                                  os.path.join(ROOT, "tools", "run_follow.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class SearchCode(unittest.TestCase):
    def test_check_code_is_not_part_of_it(self):
        src = fo._search_code()
        self.assertIn("def placement_search", src)
        self.assertIn("def _solve_tangent", src)
        # the per-frame check can change without moving the placement
        for name in ("def _redistribute", "def _hold_solve", "def _follow", "def _scale_margin"):
            self.assertNotIn(name, src)
        # no machine-specific paths
        self.assertNotIn(ROOT, src)


@unittest.skipIf(mujoco is None, "mujoco not installed")
class Reuse(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.L = _load(LEADER)
        cls.r = fo.follow("leap", cls.L, n_eval=3, place=TINY)

    def test_key_stored_and_reproduced(self):
        k = self.r["placement_key"]
        self.assertEqual(k["hand"], "leap")
        self.assertEqual(k["leader"], self.L["id"])
        self.assertIsNotNone(self.r["placement_search_s"])
        self.assertTrue(fo.same_placement_key(k, fo.placement_key("leap", self.L, place=TINY)))
        # through JSON, as a stored result holds it
        self.assertTrue(fo.same_placement_key(json.loads(json.dumps(k)),
                                              fo.placement_key("leap", self.L, place=TINY)))

    def test_key_changes_with_what_the_search_reads(self):
        k = self.r["placement_key"]
        for other in (fo.placement_key("allegro", self.L, place=TINY),
                      fo.placement_key("leap", {**self.L, "mu": 0.5}, place=TINY),
                      fo.placement_key("leap", self.L, seed=1, place=TINY),
                      fo.placement_key("leap", self.L, respace=True, place=TINY),
                      fo.placement_key("leap", self.L, place={**TINY, "n_samples": 4}),
                      fo.placement_key("leap", self.L, place=TINY,
                                       allow=fo.overlap_allowance(self.L, 0.001))):
            self.assertFalse(fo.same_placement_key(k, other))

    def test_rescoring_gives_the_stored_verdicts(self):
        again = fo.follow("leap", self.L, n_eval=3, placement=json.loads(json.dumps(self.r)))
        self.assertEqual([f["verdict"] for f in again["frames"]],
                         [f["verdict"] for f in self.r["frames"]])
        self.assertEqual(again["placement_key"], self.r["placement_key"])
        self.assertTrue(again["settings"]["reused_placement"])

    def test_run_follow_finds_the_matching_file(self):
        rf = _run_follow()
        key = fo.placement_key("leap", self.L, place=TINY)
        with tempfile.TemporaryDirectory() as d:
            stale = {**self.r, "placement_key": {**self.r["placement_key"], "seed": 7}}
            for name, res in (("follow_leap__a.json", stale), ("follow_leap__b.json", self.r)):
                with open(os.path.join(d, name), "w", encoding="utf-8") as fh:
                    json.dump(res, fh, default=float)
            p, got = rf.stored_placement(d, "leap", key)
            self.assertEqual(os.path.basename(p), "follow_leap__b.json")
            self.assertEqual(got["placement"], json.loads(json.dumps(self.r["placement"])))
            self.assertEqual(rf.stored_placement(d, "allegro", key), (None, None))


if __name__ == "__main__":
    unittest.main()
