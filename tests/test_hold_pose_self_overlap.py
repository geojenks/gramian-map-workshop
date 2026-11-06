"""tools/hold_pose_self_overlap.py: the pair list on a model whose answer is
known by construction (two capsules a set distance apart; a child capsule
buried in its parent, which MuJoCo does not report), and, where a stored
hold result and its hand model are present, that the pose set here is the
one the stored depths were read at (python -m unittest). Needs mujoco;
without it everything is skipped."""
import glob
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

try:
    import mujoco
    import hold_pose_self_overlap as hp
except Exception:  # mujoco missing
    mujoco = None

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


@unittest.skipIf(mujoco is None, "mujoco not installed")
class KnownPairs(unittest.TestCase):
    def setUp(self):
        self.m = mujoco.MjModel.from_xml_string(XML)

    def _pairs(self, q):
        return {(p["body1"], p["body2"]): p["depth_mm"] for p in hp.pairs_at(mujoco, self.m, -1, np.array(q), {})}

    def test_depth_of_two_parallel_capsules(self):
        # axes 10 mm apart, radii 8 mm each: 6 mm deep
        ps = self._pairs([0.0, 0.0, 0.0])
        self.assertAlmostEqual(ps[("a", "b")], 6.0, places=3)

    def test_clear_when_moved_apart(self):
        ps = self._pairs([0.0, 0.0, 0.010])            # axes 20 mm apart
        self.assertNotIn(("a", "b"), ps)

    def test_parent_and_child_are_not_reported(self):
        # the child capsule lies inside its parent's at every pose, and
        # MuJoCo leaves the pair out: the list is a lower bound
        self.assertNotIn(("a", "a_child"), self._pairs([0.0, 0.0, 0.0]))


@unittest.skipIf(mujoco is None, "mujoco not installed")
class StoredPose(unittest.TestCase):
    def test_pose_is_the_one_the_stored_depths_were_read_at(self):
        done = 0
        for p in sorted(glob.glob(os.path.join(ROOT, "outputs", "follow_*__hold_*__self__fixed.json"))):
            key = os.path.basename(p)[len("follow_"):].split("__hold_")[0]
            try:
                rec = hp.one(mujoco, key, p)
            except Exception:  # the hand's model is not on this machine
                continue
            for f in rec["follow"]:
                d = f["pose_check"]["max_abs_difference_mm"]
                if d is not None:
                    self.assertLess(d, 1e-6, key)
                self.assertLess(f["pose_check"]["whole_hand_shift_mm"], 1e-9, key)
            done += 1
            if done == 2:
                break
        if not done:
            self.skipTest("no stored hold result with its hand model here")


if __name__ == "__main__":
    unittest.main()
