"""The day-view page (tools/make_day_view.py): it builds from the stored
logs and pictures, shows no machine path, and never writes over a file."""
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import make_day_view as m  # noqa: E402


class DayView(unittest.TestCase):
    def setUp(self):
        self.page = m.build()

    def test_every_screen_is_on_the_page(self):
        n = sum(1 for s in m.SCREENS if s[0] != "head")
        self.assertEqual(self.page.count('<article class="screen'), n)
        for s in m.SCREENS:
            if s[0] == "page":
                self.assertIn('src="shots/%s.png"' % s[1], self.page)
                self.assertTrue(os.path.isfile(os.path.join(m.DAY, "shots", s[1] + ".png")))

    def test_terminal_text_is_the_logs(self):
        ctx = m.load(m.DAY, m.RUN)
        s = ctx["logs"]["verdicts"]["follow_mosadegh_x3_ring"]
        self.assertEqual(s["exit"], 0)
        row = [l for l in s["out"].split("\n") if "red because" in l][0]
        self.assertIn(m.E(row), self.page)

    def test_no_machine_paths_and_a_fragment(self):
        self.assertNotIn("Users", self.page)
        self.assertNotIn("<body", self.page)
        self.assertTrue(self.page.startswith("<title>"))

    def test_inline_holds_the_pictures(self):
        page = m.build(inline=True)
        self.assertNotIn('src="shots/', page)
        self.assertIn("data:image/png;base64,", page)

    def test_never_overwrites(self):
        with tempfile.TemporaryDirectory() as d:
            out = os.path.join(d, "p.html")
            m.main(["--out", out])
            with self.assertRaises(FileExistsError):
                m.main(["--out", out])


if __name__ == "__main__":
    unittest.main()
