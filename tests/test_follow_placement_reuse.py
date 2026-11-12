"""follow.py stores what a placement depends on (its placement key), so a
later run can reuse the placement instead of searching again
(tools/run_follow.py --reuse-placement). The key must match for the same
hand, leader and search, and differ when any of them changes; re-scoring
a stored placement must give the stored verdicts (python -m unittest).
Keys are compared on the hand-entry fields the search reads, so a field
added to the entry for the per-frame check leaves the stored continuity
placements (outputs/continuity/) reusable (issue 40). Needs mujoco for the
parts that build a hand; skipped without it."""
import contextlib
import glob
import importlib.util
import io
import json
import os
import re
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


# a stored placement key, as a result holds it (a copy of the Allegro
# continuity key's shape; the hashes are not real)
STORED = {"hand": "allegro",
          "hand_spec": {"name": "Allegro (Wonik)", "base": "menagerie",
                        "file": "wonik_allegro/right_hand.xml",
                        "fingers": {"index": "ff_tip", "middle": "mf_tip",
                                    "ring": "rf_tip", "thumb": "th_tip"},
                        "datasheet_torque_Nm": [0.7, "Allegro V4 max joint torque (reseller page)"]},
          "hand_model_sha256": "m" * 64, "leader": "w10", "leader_sha256": "l" * 64,
          "seed": 0, "respace": False,
          "place": {"n_frames": 10, "n_samples": 80, "iters": 4, "pad_step": 6},
          "ik_iters": 60, "allowed_overlap_m": 0.003, "other_overlap_m": 0.003,
          "search_code_sha256": "s" * 64, "mujoco": "3.17.0", "numpy": "2.5.4"}


def _with_spec(**fields):
    return {**STORED, "hand_spec": {**STORED["hand_spec"], **fields}}


class SpecFields(unittest.TestCase):
    """PLACEMENT_SPEC_FIELDS is exactly the set of hand-entry fields the
    search code reads (spec["x"] or spec.get("x") in _search_code)."""

    def test_fields_are_the_ones_the_search_reads(self):
        src = fo._search_code()
        read = set(re.findall(r"""spec(?:\[|\.get\()\s*["'](\w+)["']""", src))
        self.assertEqual(read, set(fo.PLACEMENT_SPEC_FIELDS))
        self.assertEqual(len(set(fo.PLACEMENT_SPEC_FIELDS)), len(fo.PLACEMENT_SPEC_FIELDS))


class Compare(unittest.TestCase):
    """same_placement_key and placement_key_differences on stored keys
    (no mujoco needed)."""

    def test_a_field_the_search_does_not_read_is_ignored(self):
        # the no-load speed, added for torque at speed (0429d0f); the name;
        # an added hand's marker; a caveat
        for k in (_with_spec(datasheet_no_load_speed=[0.11, "s/60deg", "reseller page"]),
                  _with_spec(name="Allegro"), _with_spec(added=True), _with_spec(caveat="x")):
            self.assertTrue(fo.same_placement_key(STORED, k))
            self.assertEqual(fo.placement_key_differences(STORED, k), [])
        # and without the name at all
        k = {**STORED, "hand_spec": {f: STORED["hand_spec"][f] for f in fo.PLACEMENT_SPEC_FIELDS}}
        self.assertTrue(fo.same_placement_key(STORED, k))

    def test_a_field_the_search_reads_is_not(self):
        for k in (_with_spec(fingers={**STORED["hand_spec"]["fingers"], "thumb": "th_pad"}),
                  _with_spec(datasheet_torque_Nm=[0.9, "x"]), _with_spec(file="other.xml"),
                  _with_spec(base="local")):
            self.assertFalse(fo.same_placement_key(STORED, k))
        spec = dict(STORED["hand_spec"])
        del spec["datasheet_torque_Nm"]
        self.assertFalse(fo.same_placement_key(STORED, {**STORED, "hand_spec": spec}))

    def test_differences_name_the_fields(self):
        k = {**_with_spec(fingers={**STORED["hand_spec"]["fingers"], "thumb": "th_pad"}),
             "seed": 1, "place": {**STORED["place"], "n_samples": 90},
             "hand_model_sha256": "n" * 64}
        d = fo.placement_key_differences(STORED, k)
        self.assertEqual([f for f, _, _ in d],
                         ["hand_model_sha256", "hand_spec.fingers", "place.n_samples", "seed"])
        self.assertEqual(d[2][1:], (80, 90))
        self.assertEqual(d[3][1:], (0, 1))
        # a field one side lacks
        k = {f: v for f, v in STORED.items() if f != "mujoco"}
        self.assertEqual(fo.placement_key_differences(STORED, k), [("mujoco", "3.17.0", None)])
        self.assertEqual(fo.placement_key_differences(k, STORED), [("mujoco", None, "3.17.0")])
        # no key at all: every field of the other
        self.assertEqual(len(fo.placement_key_differences(None, STORED)), len(STORED))
        self.assertFalse(fo.same_placement_key(None, STORED))
        # through JSON, tuples become lists: no difference
        self.assertEqual(fo.placement_key_differences(
            STORED, _with_spec(datasheet_torque_Nm=(0.7, "Allegro V4 max joint torque (reseller page)"))), [])


class NearMisses(unittest.TestCase):
    """run_follow.stored_placement_mismatches lists the stored results for
    the same hand and leader whose key differs, with the fields that
    differ (no mujoco needed)."""

    def test_lists_same_hand_and_leader_only(self):
        rf = _run_follow()
        base = {"hand": "allegro", "leader": "w10", "placement": {"p": 1},
                "placement_search_s": 1404.3}
        stale = {**base, "placement_key": {**STORED, "seed": 7}}
        speed = {**base, "placement_key": STORED}     # matches once the speed is ignored
        other = {**base, "leader": "big", "placement_key": {**STORED, "leader": "big"}}
        nokey = {**base}
        now = _with_spec(datasheet_no_load_speed=[0.11, "s/60deg", "reseller page"])
        with tempfile.TemporaryDirectory() as d:
            for name, res in (("follow_allegro__a.json", stale), ("follow_allegro__b.json", speed),
                              ("follow_allegro__c.json", other), ("follow_allegro__d.json", nokey),
                              ("follow_leap__e.json", {**stale, "hand": "leap"})):
                with open(os.path.join(d, name), "w", encoding="utf-8") as fh:
                    json.dump(res, fh)
            p, got = rf.stored_placement(d, "allegro", now)
            self.assertEqual(os.path.basename(p), "follow_allegro__b.json")
            misses = rf.stored_placement_mismatches(d, "allegro", "w10", now)
            self.assertEqual([os.path.basename(p) for p, _, _ in misses],
                             ["follow_allegro__a.json", "follow_allegro__d.json"])
            self.assertEqual(misses[0][1], [("seed", 7, 0)])
            self.assertEqual(misses[0][2], 1404.3)
            self.assertEqual(misses[1][1], [])
            # a key the stored one lacks a field of
            misses = rf.stored_placement_mismatches(d, "allegro", "w10", {**now, "seed": 7})
            self.assertEqual([os.path.basename(p) for p, _, _ in misses],
                             ["follow_allegro__b.json", "follow_allegro__d.json"])
            self.assertEqual(misses[0][1], [("seed", 0, 7)])
            # and the report names the field and the missing key
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                rf.report_mismatches("allegro", misses)
            text = out.getvalue()
            self.assertIn("differs in seed", text)
            self.assertIn("seed: stored 0; now 7", text)
            self.assertIn("placement key is missing", text)
            self.assertIn("its search took 1404 s", text)


@unittest.skipIf(mujoco is None, "mujoco not installed")
class StoredContinuity(unittest.TestCase):
    """Every stored continuity result (outputs/continuity/) still carries
    the key today's search would: nothing but the machine-bound fields
    (the compiled-model hash, which holds the model's file paths, and the
    mujoco and numpy versions) may differ anywhere; here, where the hand
    files are the ones the runs used, nothing may."""
    FILES = sorted(glob.glob(os.path.join(ROOT, "outputs", "continuity", "follow_*.json")))
    MACHINE = {"hand_model_sha256", "mujoco", "numpy"}

    def _today(self, r):
        st = r["settings"]
        with open(os.path.join(ROOT, "data", "leaders", r["leader"] + ".json"),
                  encoding="utf-8") as fh:
            L = json.load(fh)
        # run_follow passes False (--no-other-contact) or the overlap in m
        oc = st.get("other_contact") or {}
        other = (None if st["overlap_source"]["other"] != "override" else
                 oc.get("max_overlap_m") if oc.get("allowed") else False)
        allowed = None if st["overlap_source"]["contact"] != "override" else st["allowed_overlap_m"]
        try:
            return fo.placement_key(r["hand"], L, respace=st["respace"], place=st["place"],
                                    allow=fo.overlap_allowance(L, allowed, other))
        except FileNotFoundError as e:
            self.skipTest(f"hand model not found ({e})")

    def test_stored_files_exist(self):
        self.assertGreaterEqual(len(self.FILES), 1)

    def test_only_machine_fields_may_differ(self):
        for p in self.FILES:
            r = _load(p)
            d = fo.placement_key_differences(r["placement_key"], self._today(r))
            self.assertTrue({f for f, _, _ in d} <= self.MACHINE,
                            f"{os.path.basename(p)}: key differs in {d}")

    def test_match_where_the_hand_files_are_the_runs(self):
        rf = _run_follow()
        for p in self.FILES:
            r = _load(p)
            key = self._today(r)
            d = fo.placement_key_differences(r["placement_key"], key)
            machine = [(f, a, b) for f, a, b in d if f in self.MACHINE]
            if machine:
                self.skipTest(f"{os.path.basename(p)}: this machine's {machine} differ from the "
                              "run's, so the stored key cannot match here")
            self.assertTrue(fo.same_placement_key(r["placement_key"], key), d)
            got, _ = rf.stored_placement(os.path.dirname(p), r["hand"], key)
            self.assertIsNotNone(got)
            self.assertEqual(rf.stored_placement_mismatches(os.path.dirname(p), r["hand"],
                                                            r["leader"], key), [])


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
                      {**k, "hand_spec": {**k["hand_spec"], "fingers": {}}},
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
