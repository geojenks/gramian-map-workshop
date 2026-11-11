"""The walkthrough page: it builds from the stored run, shows what the logs
and tables.md hold, escapes it, and never writes over a file."""
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import make_walkthrough as mw  # noqa: E402

RUN = os.path.join(ROOT, "outputs", "run_through_11nov")


class TestMakeWalkthrough(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.page = mw.build(RUN)

    def test_every_verdict_of_the_table_is_shown(self):
        rows = mw.parse_verdicts(os.path.join(RUN, "tables.md"))
        self.assertEqual(len(rows), 3)
        for v in rows:
            for k in ("Verdicts", "Scale margin", "Disturbance margin"):
                self.assertIn(mw.E(v[k]), self.page, (k, v[k]))

    def test_logged_steps_are_shown_as_logged(self):
        log = mw.parse_log(os.path.join(RUN, "logs", "run0_log.txt"))
        s = log["build_table_record_by_path"]
        self.assertEqual(s["exit"], 1)
        self.assertIn("Singular matrix", s["out"])
        self.assertIn(mw.E(s["cmd"]), self.page)
        self.assertIn("exit 1 &middot; %d s" % s["secs"], self.page)

    def test_placeholders_are_escaped(self):
        self.assertNotIn("<hands env>", self.page)
        self.assertNotIn("<clone>", self.page)
        self.assertIn("&lt;hands env&gt;", self.page)

    def test_no_machine_paths(self):
        self.assertNotIn("Users", self.page)

    def test_out_is_never_written_over(self):
        with tempfile.TemporaryDirectory() as d:
            out = os.path.join(d, "page.html")
            cmd = [sys.executable, os.path.join(ROOT, "tools", "make_walkthrough.py"),
                   "--out", out]
            self.assertEqual(subprocess.run(cmd, capture_output=True).returncode, 0)
            first = open(out, "rb").read()
            self.assertNotEqual(subprocess.run(cmd, capture_output=True).returncode, 0)
            self.assertEqual(open(out, "rb").read(), first)


if __name__ == "__main__":
    unittest.main()
