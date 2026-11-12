"""The participant's script (docs/PARTICIPANT_SCRIPT.md) against the stored
log of the run that tested it: the page's commands are the ones that were
run, in order, and every line it says you should see is in the log."""
import io
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import run_participant_script as m  # noqa: E402

LOG = os.path.join(ROOT, "outputs", "participant_script_12nov", "log.txt")


def read(path):
    with io.open(path, encoding="utf-8") as f:
        return f.read()


class Page(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.page = read(m.PAGE)
        cls.steps = m.read_log(LOG)

    def test_the_page_commands_are_the_ones_run(self):
        run, left = m.commands(self.page)
        self.assertEqual(run, [s["cmd"] for s in self.steps])
        self.assertTrue(any(l.startswith("git clone") for l in left))
        self.assertTrue(all(l in read(LOG) for l in left))

    def test_you_should_see_lines_are_in_the_log(self):
        printed = set(l.strip() for s in self.steps for l in s["out"])
        shown = [l.strip() for lang, lines in m.blocks(self.page) if lang == "text"
                 for l in lines if l.strip()]
        self.assertTrue(shown)
        for line in shown:
            self.assertIn(line, printed)

    def test_only_the_step_the_page_marks_fails(self):
        self.assertEqual([s["name"] for s in self.steps if s["exit"] != 0],
                         ["compare_outputs"])
        self.assertIn("Does not work today.** The README says the second command",
                      self.page)

    def test_no_machine_paths_and_nothing_left_to_fill(self):
        self.assertNotIn("Users", read(LOG))
        self.assertNotIn("Users", self.page)
        self.assertNotIn("@", self.page)


class Runner(unittest.TestCase):
    PAGE = ("```powershell\npip install x\n```\n"
            "```bash\npython -c \"print('here')\"\n```\n"
            "```text\nhere\n```\n")

    def test_commands_and_names(self):
        run, left = m.commands(self.PAGE)
        self.assertEqual(run, ["python -c \"print('here')\""])
        self.assertEqual(left, ["pip install x"])
        seen = {}
        self.assertEqual([m.name_of("python tools/a.py x", seen) for _ in range(2)],
                         ["a", "a_2"])

    def test_runs_logs_and_never_overwrites(self):
        with tempfile.TemporaryDirectory() as d:
            page, log = os.path.join(d, "p.md"), os.path.join(d, "log.txt")
            with io.open(page, "w", encoding="utf-8") as f:
                f.write(self.PAGE)
            out = io.StringIO()
            old, sys.stdout = sys.stdout, out
            try:
                m.main(["--clone", d, "--log", log, "--page", page])
                with self.assertRaises(FileExistsError):
                    m.main(["--clone", d, "--log", log, "--page", page])
            finally:
                sys.stdout = old
            steps = m.read_log(log)
            self.assertEqual(len(steps), 1)
            self.assertEqual((steps[0]["exit"], steps[0]["out"]), (0, ["here"]))
            self.assertIn("  pip install x", read(log))


if __name__ == "__main__":
    unittest.main()
