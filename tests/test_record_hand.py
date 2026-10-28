"""The record-built hand (tools/build_record_hand.py): rebuilding it from its
hand file gives the stored model, and the model reproduces the fit. Needs
mujoco; without it the tests are skipped."""
import importlib.util
import json
import os
import re
import shutil
import sys
import tempfile
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
HAND = os.path.join(ROOT, "data", "hands", "wall_0.0010m_x3_ring")
# rebuilt model against the stored one: numbers are written to 9 significant
# figures, in metres, radians and the drive level
REBUILD_RTOL = 1e-6
REBUILD_ATOL = 1e-12

try:
    import mujoco  # noqa: F401
except ImportError:
    mujoco = None


def _tool():
    spec = importlib.util.spec_from_file_location(
        "build_record_hand", os.path.join(ROOT, "tools", "build_record_hand.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@unittest.skipIf(mujoco is None, "mujoco not installed")
class RecordHand(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        shutil.copy(os.path.join(HAND, "hand.json"), cls.tmp)
        cls.rep = _tool().build(os.path.join(cls.tmp, "hand.json"))

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp)

    def test_rebuild_matches_stored_model(self):
        with open(os.path.join(HAND, "hand.xml"), encoding="utf-8") as fh:
            stored = fh.read()
        with open(os.path.join(self.tmp, "hand.xml"), encoding="utf-8") as fh:
            rebuilt = fh.read()
        # the generated comment names the hand file's path, which differs here
        drop = lambda s: [l for l in s.splitlines() if not l.startswith("<!--")]
        a, b = drop(stored), drop(rebuilt)
        self.assertEqual(len(a), len(b))
        # the text must match and the numbers agree to a relative and an
        # absolute tolerance (ruled on issue 21): the near-zero last-hinge
        # limit is round-off in a fitted ratio of about 1e-9, and differs in
        # its last written digits between machines
        num = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?")
        for la, lb in zip(a, b):
            self.assertEqual(num.sub("#", la), num.sub("#", lb))
            xa = [float(x) for x in num.findall(la)]
            xb = [float(x) for x in num.findall(lb)]
            np.testing.assert_allclose(xa, xb, rtol=REBUILD_RTOL, atol=REBUILD_ATOL,
                                       err_msg=la)

    def test_round_trip(self):
        self.assertEqual(self.rep["status"], "built")
        for lab, rt in self.rep["round_trip"].items():
            # the model at q = r a(u) puts the tip where the fit does
            self.assertAlmostEqual(rt["tip_err_max_m"], rt["fit_free_err_max_m"],
                                   delta=1e-9, msg=lab)
            # drive and springs balance there
            self.assertLess(rt["drive_spring_balance_max"], 1e-6, msg=lab)

    def test_stored_report_matches(self):
        with open(os.path.join(HAND, "fit_report.json"), encoding="utf-8") as fh:
            stored = json.load(fh)
        self.assertEqual(stored["round_trip"].keys(), self.rep["round_trip"].keys())
        for lab in stored["round_trip"]:
            self.assertAlmostEqual(stored["round_trip"][lab]["tip_err_max_m"],
                                   self.rep["round_trip"][lab]["tip_err_max_m"],
                                   delta=1e-12)

    def test_fingertip_is_the_chain_end(self):
        # ruled on issue 16: a record-built finger's tip is the record's own
        # tip point, the end of the chain; model-file hands keep their rule
        from gramian_workshop.hands import Hand
        hand = Hand("wall_x3_ring")
        self.assertIsNotNone(hand.record)
        m = hand.model
        for f in hand.fingers:
            self.assertEqual(f.tip_rule, "record chain end")
            # the far end of the distal capsule's axis, as the tool writes it
            g = [g for g in range(m.ngeom) if m.geom_bodyid[g] == f.tip_body][0]
            Rm = np.zeros(9)
            mujoco.mju_quat2Mat(Rm, m.geom_quat[g])
            ends = [m.geom_pos[g] + s * Rm.reshape(3, 3)[:, 2] * m.geom_size[g][1]
                    for s in (-1, 1)]
            end = max(ends, key=np.linalg.norm)            # the one away from the joint
            np.testing.assert_allclose(f.tip_local, end, atol=1e-9)
            # and the round trip's offset of the loader's tip in this build
            self.assertLess(self.rep["round_trip"][f.label]["loader_tip_offset_m"], 1e-12)
        other = Hand("human_vc")
        self.assertIsNone(other.record)
        self.assertTrue(all(f.tip_rule == "farthest collision point" for f in other.fingers))

    def test_loads_in_force_check(self):
        from gramian_workshop.hands import hand_card
        card = hand_card("wall_x3_ring", n=200)
        self.assertEqual(card["n_actuators"], 3)
        self.assertEqual(card["shared_joints"], [])
        for f in card["fingers"]:
            self.assertEqual(f["torque_limits"], "actuator forcerange")


if __name__ == "__main__":
    unittest.main()
