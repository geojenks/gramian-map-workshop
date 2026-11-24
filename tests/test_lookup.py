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

try:
    import scipy  # noqa: F401
except ImportError:
    scipy = None
NEEDS_SCIPY = 'the map builder needs scipy: pip install -e ".[map]"'

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

    def test_single_input_rows_hold_nothing(self):
        # every manifest record ranked here has one input, so one row each
        ids = [r["id"] for r in self.rows]
        self.assertEqual(len(ids), len(set(ids)))
        for r in self.rows:
            self.assertEqual(r["held"], [], r["id"])


class TestLines(unittest.TestCase):
    """Records with two inputs: one row per measured line."""

    @staticmethod
    def _rec(u, y):
        return {"input": {"channels": [{"name": "a", "unit": "Pa"},
                                       {"name": "b", "unit": "Pa"}]},
                "free": {"u": u, "tip_displacement_m": y}}

    def test_share_does_not_depend_on_where_the_line_starts(self):
        import numpy as np
        from gramian_workshop.lookup import _row, free_lines

        class F:
            id = name = "made_up"

            def flags(self):
                return []
        steps = np.array([[0.0, 0, 0], [1, 0.2, 0], [2.5, 0.1, 0.3],
                          [3, 1, 0.5], [3.2, 2, 0.4]])
        a = [0.0, 10, 20, 30, 40]
        # the same line of `a` twice: from rest, and with b held at 50 and
        # the tip already moved by a fixed offset
        off = np.array([0.4, -0.7, 0.2])
        u = [[v, 0.0] for v in a] + [[v, 50.0] for v in a]
        y = steps.tolist() + (steps + off).tolist()
        found, why = free_lines(self._rec(u, y))
        self.assertIsNone(why)
        on_a = [f for f in found if f[0]["swept_input"] == "a"]
        self.assertEqual([f[0]["held"][0]["at"] for f in on_a], [0.0, 50.0])
        # b has only two levels at each setting of a: no line
        self.assertEqual(len(found), 2)
        e, nn = np.array([0.6, 0.8, 0.0]), np.array([0.0, 0.0, 1.0])
        t = np.cross(nn, e)
        r0, r1 = (_row(F(), *f, e, nn, t) for f in on_a)
        for k in SHARES:
            self.assertAlmostEqual(r0[k], r1[k], delta=TOL, msg=k)

    def test_no_line_gives_a_reason(self):
        from gramian_workshop.lookup import free_lines
        u = [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]]
        found, why = free_lines(self._rec(u, [[0.0, 0, 0]] * 3))
        self.assertIsNone(found)
        self.assertIn("no line", why)

    @unittest.skipIf(scipy is None, NEEDS_SCIPY)
    def test_twist_drafts_give_the_lines_the_map_export_draws(self):
        import numpy as np
        sys.path.insert(0, os.path.join(ROOT, "tools"))
        sys.path.insert(0, os.path.join(ROOT, "demos"))
        from backwards_lookup import draft_records
        from export_map_paths import split_legs
        from gramian_workshop.finger import load_record
        from gramian_workshop.lookup import free_lines
        seen = 0
        for path in draft_records():
            rec = load_record(path)
            U = np.asarray(rec["free"]["u"], float)
            if U.ndim != 2 or U.shape[1] < 2:
                continue
            seen += 1
            found, why = free_lines(rec)
            self.assertIsNone(why, rec["id"])
            legs = split_legs(U)
            self.assertEqual(len(found), len(legs), rec["id"])
            Y = np.asarray(rec["free"]["tip_displacement_m"], float)
            for (about, u, y), (c, held, rows) in zip(found, legs):
                self.assertEqual([h["at"] for h in about["held"]],
                                 list(held.values()), rec["id"])
                np.testing.assert_array_equal(u, U[rows, c])
                np.testing.assert_array_equal(y, Y[rows])
        self.assertGreaterEqual(seen, 2)   # the two two-pneu-net fingers

    def test_ranking_with_drafts_leaves_no_two_input_record_out(self):
        sys.path.insert(0, os.path.join(ROOT, "demos"))
        from backwards_lookup import draft_records
        rows, left_out = rank(load_task("cup_turn"), 4, "palm", 90.0,
                              draft_records())
        self.assertFalse([lo for lo in left_out if "input levels but" in lo["reason"]])
        held = [r for r in rows if any(h["at"] != 0 for h in r["held"])]
        self.assertTrue(held)              # a line that starts away from rest


@unittest.skipIf(scipy is None, NEEDS_SCIPY)
class TestWorkedOutLines(unittest.TestCase):
    """Lines worked out across a two-input record's surface
    (demos/backwards_lookup.py --worked-out-lines)."""

    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, os.path.join(ROOT, "tools"))
        sys.path.insert(0, os.path.join(ROOT, "demos"))
        import backwards_lookup as bl
        from gramian_workshop.finger import load_record
        cls.drafts = bl.draft_records()
        cls.recs = [load_record(p) for p in cls.drafts]
        cls.known = (set(load_manifest()["records"])
                     | {r["id"] for r in cls.recs})
        cls.more = staticmethod(bl.worked_out_lines(cls.known))
        cls.two = [r for r in cls.recs if cls.more(r)]

    def test_both_twist_fingers_get_lines(self):
        self.assertEqual(sorted(r["id"] for r in self.two),
                         ["pneunet_tandem_twist_60deg",
                          "pneunet_tandem_twist_90deg"])

    def test_surface_edges_are_the_logged_single_input_lines(self):
        # positions, not shares: a share cannot see a flipped axis
        import numpy as np
        import export_map_paths as ex
        from gramian_workshop.lookup import free_lines
        decl = ex.load_two_input_rules(ex.TWO_INPUT_RULES, self.known)
        for rec in self.two:
            s = ex.surface_of(rec, decl[rec["id"]])
            g = np.stack([np.asarray(s["values"][k], float) for k in ex.XYZ], -1)
            found, _ = free_lines(rec)
            alone = {ab["swept_input"]: (u, y) for ab, u, y in found
                     if ab["held"][0]["at"] == 0.0}
            for c, edge in ((0, g[:, 0]), (1, g[0, :])):
                u, y = alone[s["inputs"][c]["name"]]
                np.testing.assert_array_equal(u, s["inputs"][c]["values"])
                np.testing.assert_array_equal(edge, y)

    def test_lines_are_the_map_export_surface_and_stroke_line(self):
        import numpy as np
        path = os.path.join(ROOT, "site", "data",
                            "map_paths_with_drafts_palm_mount.json")
        with open(path, encoding="utf-8") as f:
            page = {a["id"]: a for a in json.load(f)["actuators"]}
        strokes = 0
        for rec in self.two:
            a = page[rec["id"]]
            s = a["surface"]
            # not the comparison-only lines, which span fewer levels
            lines = [ln for ln in self.more(rec)
                     if "check" not in ln[0]["worked_out"]]
            names = [i["name"] for i in s["inputs"]]
            for about, u, y in lines:
                c = names.index(about["swept_input"])
                k = s["inputs"][1 - c]["values"].index(about["held"][0]["at"])
                self.assertNotEqual(about["held"][0]["at"], 0.0)
                want = [[(s["values"][key][m][k] if c == 0 else
                          s["values"][key][k][m]) for key in
                         ("axial_m", "side_y_m", "side_z_m")]
                        for m in range(len(u))]
                np.testing.assert_allclose(y, want, rtol=0, atol=1e-15)
            # the page strokes along the first part of one of them: its
            # points start where that line starts and lie on it
            for t in a["tasks"]:
                sl = (t.get("return_clear") or {}).get("stroke_line")
                if not sl or sl.get("what") != "surface line":
                    continue
                hit = [y for about, u, y in lines
                       if about["swept_input"] == sl["swept_input"]
                       and about["held"][0]["at"] == sl["held_at"]]
                self.assertEqual(len(hit), 1, rec["id"])
                line, pts = np.asarray(hit[0]), np.asarray(sl["points_m"])
                np.testing.assert_allclose(pts[0], line[0], rtol=0, atol=1e-15)
                a0, d = line[:-1], np.diff(line, axis=0)
                for p in pts:
                    s01 = np.clip(((p - a0) * d).sum(1) / (d * d).sum(1), 0, 1)
                    off = np.linalg.norm(a0 + s01[:, None] * d - p, axis=1)
                    self.assertLess(off.min(), 1e-12, rec["id"])
                strokes += 1
        self.assertEqual(strokes, 2)

    def test_rows_are_marked_and_logged_rows_are_not(self):
        plain, _ = rank(load_task("cup_turn"), 4, "palm", 90.0, self.drafts)
        self.assertFalse([r for r in plain if "worked_out" in r])
        rows, _ = rank(load_task("cup_turn"), 4, "palm", 90.0, self.drafts,
                       self.more)
        logged = [r for r in rows if "worked_out" not in r]
        self.assertEqual(sorted(map(str, logged)), sorted(map(str, plain)))
        pairs = [r for r in rows if r.get("worked_out", {}).get("check")]
        self.assertTrue(pairs)
        self.assertEqual(sum(r["worked_out"]["check"] == "simulated"
                             for r in pairs) * 2, len(pairs))
        worked = [r for r in rows if "worked_out" in r and r not in pairs]
        self.assertTrue(worked)
        by = {(r["id"], r["swept_input"], r["held"][0]["at"]): r
              for r in logged if r["held"]}
        seen = 0
        for r in worked:
            w = r["worked_out"]
            self.assertEqual(w["rule"], "series")
            self.assertTrue(w["basis"].strip())
            self.assertEqual(w["simulated_points_on_this_line"] == 0,
                             w["worst_position_gap_share_at_them"] is None)
            key = (r["id"], r["swept_input"], r["held"][0]["at"])
            self.assertEqual(w["also_logged"], key in by)
            if key in by:          # worked out against simulated, as shares
                seen += 1
                for k in SHARES:
                    self.assertAlmostEqual(r[k], by[key][k], delta=0.01,
                                           msg=f"{key} {k}")
        self.assertEqual(seen, 2)


if __name__ == "__main__":
    unittest.main()
