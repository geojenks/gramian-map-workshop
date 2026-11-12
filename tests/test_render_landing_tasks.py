"""The views of the target beside the attempt (tools/render_landing_tasks.py):
every number a stored view shows is what the result file and the task file
say, worked out again here without the tool. Where MuJoCo is installed, the
tool is also run again and must give the stored view's data. The same holds
for the views of a task on its own, with no hand chosen."""
import io
import json
import math
import os
import re
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import make_landing_mockup as m  # noqa: E402

FOLDER = os.path.join(ROOT, m.RENDERS)
ALONE = os.path.join(ROOT, m.TASK_VIEWS)


def read(path):
    with io.open(path, encoding="utf-8") as f:
        return json.load(f)


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def placed(Rw, deg, F):
    """A task force as the check sees it: turned about the vertical by the
    contact's respacing, then by the stored placement."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    v = [c * F[0] - s * F[1], s * F[0] + c * F[1], F[2]]
    return [sum(Rw[r][j] * v[j] for j in range(3)) for r in range(3)]


class Views(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.listed = read(os.path.join(FOLDER, "renders.json"))["views"]
        cls.views = []
        for v in cls.listed:
            with io.open(os.path.join(FOLDER, v["view"]), encoding="utf-8") as f:
                text = f.read()
            d = json.loads(re.search(r'<script type="application/json" id="data">(.*?)</script>',
                                     text, re.S).group(1).replace("<\\/", "</"))
            cls.views.append((v, text, d))

    def test_every_view_is_of_a_task_result_on_the_page(self):
        # a result stored after the folder was written has no view yet; the page says so
        shown = [r["file"] for pair, rs in m.data(ROOT)["results"].items()
                 if not pair.endswith("|" + m.OWN) for r in rs]
        self.assertTrue(self.listed)
        for v in self.listed:
            self.assertIn(v["result"], shown)
        for v in self.listed:
            if v.get("still"):
                self.assertTrue(os.path.isfile(os.path.join(FOLDER, v["still"])))

    def test_what_a_view_shows_is_what_the_files_say(self):
        seen = 0
        for v, _, d in self.views:
            src = read(os.path.join(ROOT, v["result"]))
            L = read(os.path.join(ROOT, "data", "leaders", src["leader"] + ".json"))
            Rw = src["placement"]["rotation_matrix"]
            deg = src.get("respace_deg") or [0.0] * L["n_stations"]
            self.assertEqual((d["hand"], d["task"], d["caveat"]), (src["hand"], src["leader"], src.get("caveat")))
            self.assertEqual(len(d["frames"]), len(src["frames"]))
            self.assertEqual(v["moments"], len(src["frames"]))
            # the check stores a supplied force only for a finger with fewer inputs
            # than sprung joints; a view of any other hand shows none
            stored = sum(1 for fr in src["frames"] for x in fr["fingers"]
                         if (x.get("equilibrium") or {}).get("force_N") is not None)
            forces = 0
            for shown, fr in zip(d["frames"], src["frames"]):
                lf = L["frames"][fr["k"]]
                self.assertEqual((shown["k"], shown["verdict"], shown["reason"]),
                                 (fr["k"], fr["verdict"], fr["reason"]))
                self.assertEqual(shown["unserved"], fr.get("unserved") or [])
                self.assertAlmostEqual(shown["t"], lf["t"], places=3)
                asked = {}
                for c in shown["contacts"]:
                    asked[c["i"]] = placed(Rw, deg[c["i"]], lf["force_on_object_N"][c["i"]])
                    self.assertEqual(c["touch"], bool(lf["contact"][c["i"]]))
                    for a, b in zip(c["force"], asked[c["i"]]):
                        self.assertAlmostEqual(a, b, places=4)
                    self.assertAlmostEqual(c["force_N"], norm(asked[c["i"]]), places=3)
                touching = [i for i in asked if lf["contact"][i]]
                self.assertEqual(shown["task_sum"]["of"], len(touching))
                total = [sum(asked[i][a] for i in touching) for a in range(3)]
                self.assertAlmostEqual(shown["task_sum"]["force_N"], norm(total), places=3)
                given = []
                for row, x in zip(shown["fingers"], fr["fingers"]):
                    self.assertEqual((row["finger"], row["state"], row["station"]),
                                     (x["finger"], x["state"], x.get("station")))
                    if "reach_mm" in x:
                        self.assertAlmostEqual(row["off_mm"], x["reach_mm"], delta=0.051)
                    F = (x.get("equilibrium") or {}).get("force_N")
                    if F is None:
                        self.assertIsNone(row["force"])
                        continue
                    forces += 1
                    given.append(F)
                    self.assertAlmostEqual(row["force_N"], norm(F), places=3)
                    self.assertAlmostEqual(row["apart_N"], x["equilibrium"]["force_departure_N"], places=3)
                    # the two arrows drawn for a finger differ by what the result stores
                    apart = norm([F[a] - asked[x["station"]][a] for a in range(3)])
                    self.assertAlmostEqual(apart, x["equilibrium"]["force_departure_N"], places=6)
                if given:
                    self.assertEqual(shown["hand_sum"]["of"], len(given))
                    self.assertAlmostEqual(shown["hand_sum"]["force_N"],
                                           norm([sum(F[a] for F in given) for a in range(3)]), places=3)
                else:
                    self.assertIsNone(shown["hand_sum"])
            self.assertEqual(forces, stored, v["result"])
            seen += forces
        self.assertTrue(seen)

    def test_no_names_no_machine_paths_no_outside_scripts(self):
        for _, text, _ in self.views:
            own = text.split("<script>")[0] + text.split("<script>")[-1]   # the libraries are not our words
            self.assertNotIn("Users", own)
            self.assertNotIn("george", own.lower())
            for word in ("dynamic", "first of its kind"):
                self.assertNotIn(word, own.lower())
            self.assertNotIn("<script src", text)
            self.assertNotIn("<html", text[:3000])
            self.assertIn("<title>", text[:2000])

    def test_the_tool_gives_the_stored_view_again(self):
        try:
            import mujoco  # noqa: F401
        except ImportError:
            self.skipTest("MuJoCo is not installed")
        import render_landing_tasks as t
        pairs = dict((rel, (hand, task)) for rel, hand, task in t.stored_pairs(ROOT))
        for v, _, d in self.views:
            again = t.view_data(ROOT, v["result"], *pairs[v["result"]])
            self.assertEqual(json.loads(json.dumps(again)), d)


class TaskViews(unittest.TestCase):
    """A task on its own: the moments are the ones the check looks at, and
    the forces are the task file's, as written there."""

    @classmethod
    def setUpClass(cls):
        cls.listed = read(os.path.join(ALONE, "tasks.json"))["cards"]
        cls.views = []
        for c in cls.listed:
            with io.open(os.path.join(ALONE, c["view"]), encoding="utf-8") as f:
                text = f.read()
            d = json.loads(re.search(r'<script type="application/json" id="data">(.*?)</script>',
                                     text, re.S).group(1).replace("<\\/", "</"))
            cls.views.append((c, text, d))

    def test_one_view_for_each_task_card_and_a_still_for_each_task(self):
        cards = m.data(ROOT)["cards"]
        self.assertEqual([c["card"] for c in self.listed], [c["key"] for c in cards])
        for (c, _, d), card in zip(self.views, cards):
            self.assertEqual((d["card"], d["name"]), (card["key"], card["name"]))
            self.assertEqual([(v["task"], v["label"]) for v in d["variants"]],
                             [(v["id"], v["label"]) for v in card["variants"]])
            self.assertEqual([v["task"] for v in c["variants"]], [v["id"] for v in card["variants"]])
            for row, v in zip(c["variants"], d["variants"]):
                self.assertEqual(row["moments"], len(v["frames"]))
                most = max(f["task_sum"]["of"] for f in v["frames"])
                self.assertEqual(v["frames"][row["shown"] - 1]["task_sum"]["of"], most)
                if row.get("still"):
                    self.assertTrue(os.path.isfile(os.path.join(ALONE, row["still"])))

    def test_what_a_task_view_shows_is_what_the_task_file_says(self):
        for _, _, d in self.views:
            for v in d["variants"]:
                L = read(os.path.join(ROOT, "data", "leaders", v["task"] + ".json"))
                n = len(L["frames"])
                # 40 frames spread evenly from the first to the last, as the check takes them
                ks = [int(j * (n - 1) / 39.0 + 1e-9) for j in range(40)]
                self.assertEqual([f["k"] for f in v["frames"]], ks)
                self.assertEqual((v["caveat"], v["all_frames"], v["points"]),
                                 (L.get("caveat"), n, L["n_stations"]))
                self.assertEqual(v["radius"], L["object"]["radius_m"])
                self.assertEqual(v["mass_kg"], L["object"].get("mass_kg"))
                c0, largest = L["frames"][0]["obj_pos"], 0.0
                for shown in v["frames"]:
                    lf = L["frames"][shown["k"]]
                    self.assertAlmostEqual(shown["t"], lf["t"], places=3)
                    for a in range(3):
                        self.assertAlmostEqual(shown["ball"][a], lf["obj_pos"][a] - c0[a], places=4)
                    total = [0.0, 0.0, 0.0]
                    for c in shown["contacts"]:
                        F = lf["force_on_object_N"][c["i"]]
                        self.assertEqual(c["touch"], bool(lf["contact"][c["i"]]))
                        for a in range(3):
                            self.assertAlmostEqual(c["force"][a], F[a], places=4)
                        self.assertAlmostEqual(c["force_N"], norm(F), places=3)
                        if c["touch"]:
                            total = [total[a] + F[a] for a in range(3)]
                            largest = max(largest, c["force_N"])
                    touching = [i for i in range(L["n_stations"]) if lf["contact"][i]]
                    self.assertEqual([c["i"] for c in shown["contacts"] if c["touch"]], touching)
                    self.assertEqual(shown["task_sum"]["of"], len(touching))
                    self.assertAlmostEqual(shown["task_sum"]["force_N"], norm(total), places=3)
                self.assertAlmostEqual(v["largest_contact_N"], largest, places=4)

    def test_a_task_view_and_a_pair_view_agree_on_the_task(self):
        # placing the hand turns the forces, it does not change their size
        alone = dict((v["task"], v) for _, _, d in self.views for v in d["variants"])
        seen = 0
        for row in read(os.path.join(FOLDER, "renders.json"))["views"]:
            with io.open(os.path.join(FOLDER, row["view"]), encoding="utf-8") as f:
                pair = json.loads(re.search(r'<script type="application/json" id="data">(.*?)</script>',
                                            f.read(), re.S).group(1).replace("<\\/", "</"))
            by_k = dict((f["k"], f) for f in alone[pair["task"]]["frames"])
            # the moments of the task view are the ones the stored check looked at
            self.assertEqual([f["k"] for f in pair["frames"]],
                             [f["k"] for f in alone[pair["task"]]["frames"]])
            for f in pair["frames"]:
                if f["k"] in by_k:
                    seen += 1
                    self.assertEqual(f["task_sum"]["of"], by_k[f["k"]]["task_sum"]["of"])
                    self.assertAlmostEqual(f["task_sum"]["force_N"], by_k[f["k"]]["task_sum"]["force_N"],
                                           places=3)
        self.assertTrue(seen)

    def test_no_names_no_machine_paths_no_outside_scripts(self):
        for _, text, _ in self.views:
            own = text.split("<script>")[0] + text.split("<script>")[-1]   # the libraries are not our words
            self.assertNotIn("Users", own)
            self.assertNotIn("george", own.lower())
            for word in ("dynamic", "first of its kind"):
                self.assertNotIn(word, own.lower())
            self.assertNotIn("<script src", text)
            self.assertNotIn("<html", text[:3000])
            self.assertIn("<title>", text[:2000])

    def test_the_tool_gives_the_stored_task_view_again(self):
        try:
            import mujoco  # noqa: F401
        except ImportError:
            self.skipTest("MuJoCo is not installed")
        import render_landing_tasks as t
        for (_, _, d), card in zip(self.views, m.data(ROOT)["cards"]):
            self.assertEqual(json.loads(json.dumps(t.card_data(ROOT, card))), d)


if __name__ == "__main__":
    unittest.main()
