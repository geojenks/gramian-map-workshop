"""follow.py reports the hand against itself and the verdict takes no
account of it (ruling of 6 Nov, issue 27): the pair list on a model whose
answer is known by construction (two capsules a set distance apart; a
child capsule buried in its parent, which MuJoCo does not report), the
depth beyond rest, which pair counts as finger against finger and which as
finger against palm, and, on stored self-hold results whose hand models
are in the repository, that the frame record's pairs are the ones
tools/hold_pose_self_overlap.py listed at the same poses and that a run
whose measure is replaced by a made-up deep overlap gives the same
verdicts (python -m unittest). Needs mujoco; without it everything is
skipped."""
import json
import os
import sys
import types
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import follow as fo  # noqa: E402

try:
    import mujoco
except ImportError:
    mujoco = None

# bodies: a (1), a_child (2, inside a at every pose), b (3)
XML = """
<mujoco>
  <worldbody>
    <body name="a" pos="0 0 0">
      <joint name="ja" type="hinge" axis="0 0 1"/>
      <geom name="ga" type="capsule" fromto="0 0 0 0.05 0 0" size="0.008"/>
      <body name="a_child" pos="0.01 0 0">
        <joint name="jc" type="hinge" axis="0 0 1"/>
        <geom name="gc" type="capsule" fromto="0 0 0 0.03 0 0" size="0.008"/>
      </body>
    </body>
    <body name="b" pos="0 0.010 0">
      <joint name="jb" type="slide" axis="0 1 0"/>
      <geom name="gb" type="capsule" fromto="0 0 0 0.05 0 0" size="0.008"/>
    </body>
  </worldbody>
</mujoco>
"""
TOUCHING = [0.0, 0.0, 0.0]          # a and b: axes 10 mm apart, radii 8 mm each
APART = [0.0, 0.0, 0.010]           # axes 20 mm apart
SHALLOWER = [0.0, 0.0, 0.002]       # axes 12 mm apart
TWO_FINGERS = {1: 0, 2: 0, 3: 1}    # a and its child one finger, b another
ONE_FINGER = {1: 0, 2: 0}           # b belongs to no finger: the palm


def _load(p):
    with open(p, encoding="utf-8") as fh:
        return json.load(fh)


@unittest.skipIf(mujoco is None, "mujoco not installed")
class KnownPairs(unittest.TestCase):
    def setUp(self):
        self.m = mujoco.MjModel.from_xml_string(XML)
        self.hand = types.SimpleNamespace(model=self.m, apply_coupling=lambda q: q)

    def _so(self, q, rest_q, own):
        rest = fo.self_pairs(self.m, np.array(rest_q))
        return fo.self_overlap(self.hand, np.array(q), rest, own)

    def test_depth_of_two_parallel_capsules(self):
        ps = fo.self_pairs(self.m, np.array(TOUCHING))
        self.assertAlmostEqual(ps[(1, 3)] * 1e3, 6.0, places=3)

    def test_clear_when_moved_apart(self):
        self.assertNotIn((1, 3), fo.self_pairs(self.m, np.array(APART)))

    def test_parent_and_child_are_not_reported(self):
        # the child capsule lies inside its parent's at every pose, and
        # MuJoCo leaves the pair out: the measure is a lower bound
        self.assertNotIn((1, 2), fo.self_pairs(self.m, np.array(TOUCHING)))

    def test_the_data_passed_in_is_the_one_used(self):
        d = mujoco.MjData(self.m)
        fo.self_pairs(self.m, np.array(SHALLOWER), d)
        self.assertEqual(list(d.qpos), SHALLOWER)

    def test_finger_against_finger(self):
        so = self._so(TOUCHING, APART, TWO_FINGERS)
        self.assertAlmostEqual(so["finger_finger_mm"], 6.0, places=3)
        self.assertEqual(so["finger_finger_pair"], ["a", "b"])
        self.assertEqual((so["finger_palm_mm"], so["finger_palm_pair"]), (0.0, None))
        self.assertEqual([(p["body1"], p["body2"], p["kind"]) for p in so["pairs"]],
                         [("a", "b", "finger-finger"), ("a_child", "b", "finger-finger")])
        self.assertEqual(so["pairs"][0]["rest_mm"], 0.0)

    def test_finger_against_palm(self):
        so = self._so(TOUCHING, APART, ONE_FINGER)
        self.assertAlmostEqual(so["finger_palm_mm"], 6.0, places=3)
        self.assertEqual(so["finger_palm_pair"], ["a", "b"])
        self.assertEqual((so["finger_finger_mm"], so["finger_finger_pair"]), (0.0, None))
        self.assertEqual({p["kind"] for p in so["pairs"]}, {"finger-palm"})

    def test_same_finger_and_palm_pairs_are_listed_but_not_headline(self):
        for own, kind in (({1: 0, 2: 0, 3: 0}, "same finger"), ({}, "palm-palm")):
            so = self._so(TOUCHING, APART, own)
            self.assertEqual({p["kind"] for p in so["pairs"]}, {kind})
            self.assertEqual((so["finger_finger_mm"], so["finger_palm_mm"]), (0.0, 0.0))

    def test_a_pair_as_deep_as_at_rest_counts_nothing(self):
        so = self._so(TOUCHING, TOUCHING, TWO_FINGERS)
        self.assertEqual((so["finger_finger_mm"], so["finger_finger_pair"]), (0.0, None))
        # it is still listed, with both depths
        self.assertAlmostEqual(so["pairs"][0]["depth_mm"], 6.0, places=3)
        self.assertAlmostEqual(so["pairs"][0]["rest_mm"], 6.0, places=3)

    def test_only_the_depth_beyond_rest_counts(self):
        so = self._so(TOUCHING, SHALLOWER, TWO_FINGERS)      # 6 mm against 4 mm at rest
        self.assertAlmostEqual(so["finger_finger_mm"], 2.0, places=3)
        self.assertEqual(so["finger_finger_pair"], ["a", "b"])

    def test_nothing_when_clear(self):
        so = self._so(APART, APART, TWO_FINGERS)
        self.assertEqual(so, {"finger_finger_mm": 0.0, "finger_finger_pair": None,
                              "finger_palm_mm": 0.0, "finger_palm_pair": None, "pairs": []})

    def test_rest_defaults_to_the_models_own(self):
        # qpos0 is the touching pose, so nothing is beyond rest there
        so = fo.self_overlap(self.hand, np.array(TOUCHING), own=TWO_FINGERS)
        self.assertEqual(so["finger_finger_mm"], 0.0)

    def test_body_owners(self):
        fingers = [types.SimpleNamespace(joints=[0], tip_body=2),      # ja: a and its child
                   types.SimpleNamespace(joints=[], tip_body=3)]       # no joint of its own
        hand = types.SimpleNamespace(model=self.m, fingers=fingers)
        self.assertEqual(fo.body_owners(hand), {1: 0, 2: 0, 3: 1})


@unittest.skipIf(mujoco is None, "mujoco not installed")
class StoredHolds(unittest.TestCase):
    """Stored self-hold results of hands whose models are in the repository,
    re-scored with their stored placements."""
    HANDS = ("wall_x3_ring", "human_vc")
    _kept = {}

    @staticmethod
    def _run(key, **kw):
        """(the stored result, the same run again with the stored
        placement and kw)."""
        stored = _load(os.path.join(ROOT, "outputs", f"follow_{key}__hold_{key}__self__fixed.json"))
        leader = _load(os.path.join(ROOT, "data", "leaders", f"hold_{key}__self.json"))
        fixed = {k: stored[k] for k in ("placement", "assignment", "respace_deg", "placement_cost_mm")}
        return stored, fo.follow(key, leader, placement=fixed, margin=True, **kw)

    @classmethod
    def _as_stored(cls, key):
        """The stored run reproduced: the stored runs are from before the
        end stops were credited, so stop_reactions=False, which takes a
        minute on these hands; kept for the class."""
        if key not in cls._kept:
            cls._kept[key] = cls._run(key, stop_reactions=False)
        return cls._kept[key]

    def test_pairs_are_the_tools(self):
        table = {r["hand"]: r for r in _load(os.path.join(ROOT, "outputs", "hold_pose_self_overlap.json"))["hands"]}
        for key in self.HANDS:
            _, r = self._as_stored(key)
            self.assertEqual(r["self_overlap"]["in_verdict"], False)
            self.assertEqual(r["self_overlap"]["colliding_geoms"], table[key]["model"]["colliding_geoms"])
            rest = {(p["body1"], p["body2"]): p["depth_mm"] for p in table[key]["rest"]}
            got = {(p["body1"], p["body2"]): p["depth_mm"] for p in r["self_overlap"]["rest_pairs"]}
            self.assertEqual(set(got), set(rest), key)
            for fr, tf in zip(r["frames"], table[key]["follow"]):
                self.assertEqual(fr["k"], tf["k"])
                want = {(p["body1"], p["body2"]): p for p in tf["pairs"]}
                got = {(p["body1"], p["body2"]): p for p in fr["self_overlap"]["pairs"]}
                self.assertEqual(set(got), set(want), key)
                for pair, p in got.items():
                    self.assertAlmostEqual(p["depth_mm"], want[pair]["depth_mm"], places=6, msg=key)
                    self.assertAlmostEqual(p["rest_mm"], rest.get(pair, 0.0), places=6, msg=key)
                # the tool's finger-against-finger pairs are all clear at rest
                ff = [p["depth_mm"] for p in tf["pairs"] if p["kind"] == "finger-finger"]
                self.assertTrue(ff, key)
                self.assertAlmostEqual(fr["self_overlap"]["finger_finger_mm"], max(ff), places=6, msg=key)

    def test_the_stored_verdict_stands(self):
        # a finger is inside another finger at this pose by more than a
        # centimetre, and the stored verdict, reasons and finger states stand
        stored, r = self._as_stored(self.HANDS[0])
        self.assertGreater(r["frames"][0]["self_overlap"]["finger_finger_mm"], 10.0)
        self.assertEqual([(f["k"], f["verdict"], f["reason"]) for f in r["frames"]],
                         [(f["k"], f["verdict"], f["reason"]) for f in stored["frames"]])
        self.assertEqual([[x.get("state") for x in f["fingers"]] for f in r["frames"]],
                         [[x.get("state") for x in f["fingers"]] for f in stored["frames"]])

    def test_the_verdict_takes_no_account_of_it(self):
        # with the measure replaced by a made-up overlap of a metre, every
        # other key of the result is the same (a hand that runs in seconds)
        key = "george_clip"
        _, r = self._run(key)
        self.assertTrue(r["frames"][0]["self_overlap"]["pairs"])     # the real measure saw something
        real = fo.self_overlap
        fake = {"finger_finger_mm": 1e3, "finger_finger_pair": ["x", "y"],
                "finger_palm_mm": 1e3, "finger_palm_pair": ["x", "z"], "pairs": []}
        fo.self_overlap = lambda *a, **k: dict(fake)
        try:
            _, r2 = self._run(key)
        finally:
            fo.self_overlap = real
        self.assertEqual(r2["frames"][0]["self_overlap"], fake)
        for x in (r, r2):
            x.pop("elapsed_s", None)
            for f in x["frames"]:
                f.pop("self_overlap")
        self.assertEqual(json.dumps(r, sort_keys=True), json.dumps(r2, sort_keys=True))


if __name__ == "__main__":
    unittest.main()
