"""The record Gramians figure: the script runs on the stored records, and
every record is either drawn or listed with the reason it is left out."""
import glob
import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))

try:
    import matplotlib  # noqa: F401
    HAVE_MPL = True
except ImportError:
    HAVE_MPL = False


@unittest.skipUnless(HAVE_MPL, "needs matplotlib (the analysis venv has it)")
class TestFigureGramians(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        import matplotlib
        matplotlib.use("Agg")
        import figure_gramians
        cls.tmp = tempfile.TemporaryDirectory()
        figure_gramians.main(["--out-dir", cls.tmp.name])
        cls.base = os.path.join(cls.tmp.name, figure_gramians.STEM)
        with open(cls.base + ".json", encoding="utf-8") as f:
            cls.out = json.load(f)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_outputs_written(self):
        for ext in (".png", ".svg", ".json", ".md"):
            path = self.base + ext
            self.assertTrue(os.path.isfile(path), path)
            self.assertGreater(os.path.getsize(path), 0, path)

    def test_every_record_drawn_or_named(self):
        on_disk = {os.path.splitext(os.path.basename(p))[0]
                   for p in glob.glob(os.path.join(ROOT, "data", "records", "*.json"))}
        on_disk.discard("MANIFEST")
        drawn = [d["id"] for d in self.out["drawn"]]
        left = [lo["id"] for lo in self.out["left_out"]]
        self.assertEqual(len(drawn), len(set(drawn)))
        self.assertEqual(len(left), len(set(left)))
        self.assertFalse(set(drawn) & set(left))
        self.assertEqual(set(drawn) | set(left), on_disk)

    def test_left_out_records_have_reasons(self):
        for lo in self.out["left_out"]:
            self.assertTrue(lo["reason"].strip(), lo["id"])

    def test_tip_block_matches_stored_gramian(self):
        checked = [d for d in self.out["drawn"] if "tip_block_rel_diff" in d]
        self.assertTrue(checked)
        for d in checked:
            self.assertLess(d["tip_block_rel_diff"], 1e-4, d["id"])


if __name__ == "__main__":
    unittest.main()
