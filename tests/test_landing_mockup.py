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
        self.assertEqual(self.page.count('data-kind="task"'), len(self.d["cards"]) + 1)
        on_cards = [v["id"] for c in self.d["cards"] for v in c["variants"]]
        self.assertEqual(sorted(on_cards), sorted(t["id"] for t in self.d["tasks"]))
        self.assertEqual(self.page.count('class="chip"'),
                         sum(len(c["variants"]) for c in self.d["cards"] if len(c["variants"]) > 1))

    def test_headroom_is_the_smallest_stored_margin(self):
        seen = 0
        for shown in self.d["results"].values():
            for r in shown:
                with io.open(os.path.join(ROOT, r["file"]), encoding="utf-8") as f:
                    ms = [fr.get("scale_margin") for fr in json.load(f)["frames"]]
                if any(x is None for x in ms):
                    self.assertIsNone(r["headroom"])
                else:
                    seen += 1
                    # the page's sentence holds only if every load up to the margin is held
                    self.assertTrue(all(x["contiguous"] for x in ms), r["file"])
                    self.assertEqual(r["headroom"]["least"], min(x["margin"] for x in ms))
        self.assertTrue(seen)

    def test_builds_without_pictures_and_shows_those_given(self):
        with tempfile.TemporaryDirectory() as tmp:
            bare = m.build(ROOT, pictures_folder=tmp, renders_folder=tmp)
        self.assertIn("no picture yet", bare)
        self.assertNotIn("data:image", bare)
        self.assertIn('<script type="application/json" id="renders">{}</script>', bare)
        folder = os.path.join(ROOT, m.PICTURES)
        with io.open(os.path.join(folder, "pictures.json"), encoding="utf-8") as f:
            listed = json.load(f)
        self.assertEqual(self.page.count("data:image/png;base64,"),
                         len(listed["hands"]) + (1 if listed.get("atlas") else 0) + len(self.stills()))
        with io.open(os.path.join(ROOT, "data", "records", "MANIFEST.json"), encoding="utf-8") as f:
            self.assertEqual(self.d["records"], len(json.load(f)["records"]))
        self.assertIn("There are %d records" % self.d["records"], self.page)

    def stills(self):
        with io.open(os.path.join(ROOT, m.RENDERS, "renders.json"), encoding="utf-8") as f:
            return [v for v in json.load(f)["views"] if v.get("still")]

    def test_a_render_shown_is_of_a_result_shown(self):
        got = json.loads(re.search(r'<script type="application/json" id="renders">(.*?)</script>',
                                   self.page, re.S).group(1).replace("<\\/", "</"))
        files = [r["file"] for shown in self.d["results"].values() for r in shown]
        self.assertTrue(got)
        for result, v in got.items():
            self.assertIn(result, files)
            name = v["view"].rsplit("/", 1)[1]
            self.assertTrue(os.path.isfile(os.path.join(ROOT, m.RENDERS, name)))
            self.assertEqual(v["view"], "../%s/%s" % (os.path.basename(m.RENDERS), name))
            with io.open(os.path.join(ROOT, result), encoding="utf-8") as f:
                self.assertEqual(v["moments"], len(json.load(f)["frames"]))

    def test_no_names_no_machine_paths_no_outside_scripts(self):
        text = re.sub(r'data:image/png;base64,[A-Za-z0-9+/=]+', "", self.page)
        self.assertNotIn("Users", text)
        self.assertNotIn("george", text.lower())
        for word in ("dynamic", "first of its kind"):
            self.assertNotIn(word, text.lower())
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
