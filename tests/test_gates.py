"""Gates V0-V5 as tests (python -m unittest). V0 and V5's export check need
a Wave_manip checkout: set WAVE_MANIP=/path/to/wave_manip, else skipped."""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import gates  # noqa: E402
from gramian_workshop.arrange import Arrangement  # noqa: E402
from gramian_workshop.finger import Finger, all_fingers, load_record  # noqa: E402
from gramian_workshop.readout import soft_hold  # noqa: E402
from gramian_workshop.task import load_task  # noqa: E402
from gramian_workshop.turn import schedule  # noqa: E402

WM = os.environ.get("WAVE_MANIP")


class Gates(unittest.TestCase):
    @unittest.skipUnless(WM, "set WAVE_MANIP to compare against the source")
    def test_v0_vendored_identical(self):
        self.assertTrue(gates.v0(os.path.abspath(WM))["pass"])

    def test_v1_solver_vs_brute_force(self):
        self.assertTrue(gates.v1()["pass"])

    def test_v2_frames(self):
        self.assertTrue(gates.v2(load_task("cup_hold"))["pass"])

    def test_v3_v4_closure_and_symmetry(self):
        v3, v4 = gates.v3_v4(load_task("cup_hold"))
        self.assertTrue(v3["pass"])
        self.assertTrue(v4["pass"])

    def test_v5_provenance(self):
        wm = os.path.abspath(WM) if WM else None
        self.assertTrue(gates.v5(wm)["pass"])


class Smoke(unittest.TestCase):
    def test_template_record_loads_and_reads(self):
        f = Finger(load_record(os.path.join(ROOT, "examples",
                                            "template_actuator.json")))
        self.assertEqual(f.b_rule, "polarity")
        task = load_task("cup_hold")
        a = Arrangement(f, task, 3, "aimed", 0.0)
        c = a.cup
        r = soft_hold(c.points, c.normals, c.com, c.weight_demand(), c.mu,
                      c.L, a.B, a.F, a.D)
        self.assertEqual(r["reason"], "met")

    def test_schedule_counts(self):
        made = schedule(4, 0.82)
        self.assertEqual(sorted(set(made.sum(1))), [3, 4])

    def test_every_record_loads(self):
        self.assertEqual(len(all_fingers()), 22)


if __name__ == "__main__":
    unittest.main()
