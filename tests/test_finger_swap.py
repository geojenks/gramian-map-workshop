"""The finger swap demo (demos/finger_swap.py): at its default setting the
before hand (McKibben) is not a turner and the after hand (the backwards
lookup's pick) is, by the turn check's own verdict, and both verdicts agree
with the stored outputs/cup_turn.json.

Runs the check only; draws and writes nothing. numpy only. The exception is
TestFingerSwapSlide, which runs the demo with --slide: it writes
outputs/finger_swap/finger_swap_slide.{png,mp4} (needs matplotlib and
ffmpeg) and checks the other four outputs are left byte for byte."""
import contextlib
import importlib.util
import io
import json
import os
import sys
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.task import load_task  # noqa: E402

spec = importlib.util.spec_from_file_location(
    "finger_swap", os.path.join(ROOT, "demos", "finger_swap.py"))
fs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fs)

STORED = os.path.join(ROOT, "outputs", "cup_turn.json")
TASK, N, MOUNT, YAW = "cup_turn", 4, "aimed", 90.0
TOL = 1e-9


class TestFingerSwap(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.task = load_task(TASK)
        row, _, _ = fs.pick_after(cls.task, N, MOUNT, YAW)
        cls.after_id = row["id"]
        cls.before = fs.hand_result(fs.DEFAULT_BEFORE, cls.task, N, MOUNT, YAW)
        cls.after = fs.hand_result(cls.after_id, cls.task, N, MOUNT, YAW)
        with open(STORED, encoding="utf-8") as f:
            cls.stored = json.load(f)

    def test_defaults(self):
        self.assertEqual(fs.DEFAULT_BEFORE, "mckibben_v2_40deg")
        self.assertEqual(self.after_id, "wall_0.0020m")

    def test_before_is_not_a_turner(self):
        self.assertFalse(self.before["turner"])

    def test_after_is_a_turner(self):
        self.assertTrue(self.after["turner"])

    def test_agrees_with_stored_cup_turn(self):
        turners = set(self.stored["prediction_detail"]["7"]["turners"])
        duty = self.task["wave"]["duty"]
        for h in (self.before, self.after):
            self.assertTrue(h["stored_verdict_possible"], h["id"])
            self.assertEqual(h["turner"], h["id"] in turners, h["id"])
            row = self.stored["summary"][
                f"{h['id']}|{MOUNT}_yaw{int(YAW)}|{duty}|{N}"]
            self.assertEqual(h["soft_held_all"], row["soft_held_all"], h["id"])
            self.assertAlmostEqual(h["drive_azimuth"], row["drive_azimuth"],
                                   delta=TOL, msg=h["id"])
            self.assertAlmostEqual(h["drive_offaxis"], row["drive_offaxis"],
                                   delta=TOL, msg=h["id"])
            s = fs.stored_check(h, TASK, self.task)
            self.assertTrue(s["compared"], h["id"])
            self.assertTrue(s["held_agrees"], h["id"])
            self.assertTrue(s["turner_agrees"], h["id"])


class TestFingerSwapSlide(unittest.TestCase):

    def test_slide_writes_new_files_only(self):
        originals = [os.path.join(fs.OUT_DIR, f"finger_swap.{ext}")
                     for ext in ("png", "mp4", "txt", "json")]
        if not os.path.exists(originals[0]):
            self.skipTest("outputs/finger_swap/finger_swap.png not found")
        before = {}
        for p in originals:
            if os.path.exists(p):
                with open(p, "rb") as f:
                    before[p] = f.read()
        start = time.time()
        with contextlib.redirect_stdout(io.StringIO()):
            rc = fs.main(["--slide"])
        self.assertEqual(rc, 0)
        for name in ("finger_swap_slide.png", "finger_swap_slide.mp4"):
            p = os.path.join(fs.OUT_DIR, name)
            self.assertTrue(os.path.exists(p), name)
            self.assertGreater(os.path.getsize(p), 0, name)
            # written by this run, not left over from an earlier one
            self.assertGreaterEqual(os.path.getmtime(p), start - 2, name)
        for p, data in before.items():
            with open(p, "rb") as f:
                self.assertEqual(f.read(), data, os.path.basename(p))


if __name__ == "__main__":
    unittest.main()
