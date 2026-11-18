"""The backwards lookup (gramian_workshop/lookup.py): shares lie in [0, 1],
the three whole-sweep shares add to one, every record is ranked or left out
with a reason, and two records reproduce the prototype's numbers.

tests/lookup_windowed.json is a byte-for-byte copy of the prototype's output
(aimed mount, yaw 90, N = 4, task cup_turn). numpy only; writes nothing."""
import json
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.finger import load_manifest  # noqa: E402
from gramian_workshop.lookup import WINDOWS, rank  # noqa: E402
from gramian_workshop.task import load_task  # noqa: E402

PROTOTYPE = os.path.join(ROOT, "tests", "lookup_windowed.json")
CHECKED = ("wall_0.0020m", "mckibben_v2_40deg")
TOL = 1e-9
SHARES = WINDOWS + ("normal_whole", "other_tangent_whole")


class TestLookup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        with open(PROTOTYPE, encoding="utf-8") as f:
            cls.proto = json.load(f)
        cls.rows, cls.left_out = rank(load_task("cup_turn"), cls.proto["n"],
                                      "aimed", 90.0)
        cls.by_id = {r["id"]: r for r in cls.rows}

    def test_shares_in_unit_interval(self):
        for r in self.rows:
            for k in SHARES:
                v = r[k]
                if math.isnan(v):
                    continue
                self.assertGreaterEqual(v, -TOL, (r["id"], k))
                self.assertLessEqual(v, 1.0 + TOL, (r["id"], k))

    def test_whole_sweep_shares_add_to_one(self):
        for r in self.rows:
            total = r["share_whole"] + r["normal_whole"] + r["other_tangent_whole"]
            self.assertAlmostEqual(total, 1.0, delta=TOL, msg=r["id"])

    def test_every_record_ranked_or_left_out(self):
        ranked = [r["id"] for r in self.rows]
        left = [lo["id"] for lo in self.left_out]
        self.assertFalse(set(ranked) & set(left))
        self.assertEqual(set(ranked) | set(left),
                         set(load_manifest()["records"]))
        for lo in self.left_out:
            self.assertTrue(lo["reason"].strip(), lo["id"])

    def test_ranked_by_first_third(self):
        s = [r["share_first_third"] for r in self.rows]
        self.assertEqual(s, sorted(s, reverse=True))

    def test_matches_prototype(self):
        proto = {r["id"]: r for r in self.proto["rows"]}
        for rid in CHECKED:
            got, want = self.by_id[rid], proto[rid]
            self.assertEqual(got["levels"], want["levels"], rid)
            self.assertEqual(got["flags"], want["flags"], rid)
            for k in SHARES:
                self.assertAlmostEqual(got[k], want[k], delta=TOL,
                                       msg=f"{rid} {k}")
            for g, w in zip(got["best_window_u"], want["best_window_u"]):
                self.assertAlmostEqual(g, w, delta=TOL, msg=f"{rid} best_window_u")


if __name__ == "__main__":
    unittest.main()
