"""The mock-up of the landing page (tools/make_landing_mockup.py): the page
builds from the repository's own tables and stored results, what it shows
for a pair is what the stored result file says, and it never writes over a
page that is there."""
import collections
import io
import json
import os
import re
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import make_landing_mockup as m  # noqa: E402


class Page(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.d = m.data(ROOT)
        cls.page = m.build(ROOT)

    def test_every_hand_has_its_own_hold_and_tasks_are_current(self):
        keys = [h["key"] for h in self.d["hands"]]
        self.assertTrue(keys)
        self.assertEqual(sorted(self.d["holds"]), sorted(keys))
        ids = [t["id"] for t in self.d["tasks"]]
        self.assertEqual(len(ids), len(set(ids)))
        for i in ids:
            self.assertTrue(os.path.isfile(os.path.join(ROOT, "data", "leaders", i + ".json")))

    def test_a_result_shown_is_what_its_file_says(self):
        self.assertTrue(self.d["results"])
        holds = {h: v["id"] for h, v in self.d["holds"].items()}
        for pair, shown in self.d["results"].items():
            hand, task = pair.split("|")
            for r in shown:
                with io.open(os.path.join(ROOT, r["file"]), encoding="utf-8") as f:
                    stored = json.load(f)
                self.assertEqual(stored["hand"], hand)
                self.assertEqual(stored["leader"], holds[hand] if task == m.OWN else task)
                self.assertEqual(collections.Counter(r["frames"]),
                                 collections.Counter(f["verdict"] for f in stored["frames"]))
                self.assertEqual(stored.get("caveat"), r["caveat"])

    def test_the_page_holds_the_data_and_one_card_each(self):
        got = json.loads(re.search(r'<script type="application/json" id="data">(.*?)</script>',
                                   self.page, re.S).group(1).replace("<\\/", "</"))
        self.assertEqual(got, json.loads(json.dumps(self.d)))
        self.assertEqual(self.page.count('data-kind="hand"'), len(self.d["hands"]))
        self.assertEqual(self.page.count('data-kind="task"'), len(self.d["tasks"]) + 1)

    def test_no_names_no_machine_paths_no_outside_scripts(self):
        self.assertNotIn("Users", self.page)
        self.assertNotIn("george", self.page.lower())
        self.assertNotIn("<script src", self.page)
        self.assertNotIn("<html", self.page)
        self.assertIn("<title>", self.page[:2000])

    def test_opens_on_a_pair_and_never_overwrites(self):
        hand = self.d["hands"][0]["key"]
        self.assertIn('data-hand="%s" data-task="%s"' % (hand, m.OWN),
                      m.build(ROOT, hand, m.OWN))
        self.assertIn('data-hand="" data-task=""', self.page)
        with self.assertRaises(SystemExit):
            m.build(ROOT, "no_such_hand", m.OWN)
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, "page.html")
            old, sys.stdout = sys.stdout, io.StringIO()
            try:
                m.main(["--out", out])
                with self.assertRaises(FileExistsError):
                    m.main(["--out", out])
            finally:
                sys.stdout = old
            with io.open(out, encoding="utf-8") as f:
                self.assertEqual(f.read(), self.page)


if __name__ == "__main__":
    unittest.main()
