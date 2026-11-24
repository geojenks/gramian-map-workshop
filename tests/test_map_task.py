"""A task of the reader's own for the map page: tools/export_map_paths.py
--task. The file it writes must hold the task's cone and every actuator's
test for both placements of the finger, and a task file the map cannot read
must be refused with a reason."""
import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import export_map_paths as X  # noqa: E402


def stored_task():
    with open(os.path.join(ROOT, "data", "tasks", "cup_turn.json"),
              encoding="utf-8") as fo:
        return json.load(fo)


class OwnTask(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dir = tempfile.TemporaryDirectory()
        task = stored_task()
        task["id"] = "test_small_ball"
        task["name"] = "A smaller ball"
        task["object"]["radius_m"] = task["object"]["radius_m"] / 2.0
        cls.task = task
        cls.path = cls.write(task, "small.json")
        cls.doc = X.build_task(cls.path)

    @classmethod
    def tearDownClass(cls):
        cls.dir.cleanup()

    @classmethod
    def write(cls, task, name):
        path = os.path.join(cls.dir.name, name)
        with open(path, "w", encoding="utf-8") as fo:
            json.dump(task, fo)
        return path

    def test_kind_and_task(self):
        self.assertEqual(self.doc["kind"], X.TASK_KIND)
        self.assertEqual(self.doc["task"]["id"], "test_small_ball")
        self.assertEqual(self.doc["task"]["sha256"], X.sha256(self.path))

    def test_both_placements(self):
        self.assertEqual(set(self.doc["mounts"]), set(X.OWN_MOUNTS))
        for mount, m in self.doc["mounts"].items():
            self.assertEqual(m["cone"]["task"], "test_small_ball", mount)
            self.assertTrue(m["tasks"], mount)
            for entry in m["tasks"].values():
                self.assertEqual(entry["task"], "test_small_ball", mount)

    def test_stored_cones_are_the_pages(self):
        # the page compares these with its own cones before adding the task
        for mount, cone_file in X.OWN_MOUNTS.items():
            want = [{k: c.get(k) for k in X.OWN_CONE_KEYS}
                    for c in X.load_task_cones(cone_file)]
            self.assertEqual(self.doc["mounts"][mount]["stored_cones"], want)

    def test_smaller_ball_shortens_the_stroke(self):
        # the stroke is an angle carried over, so its travel follows the ball
        for mount, cone_file in X.OWN_MOUNTS.items():
            ours = X.build(True, task_cones=cone_file)["task_cones"][0]
            mine = self.doc["mounts"][mount]["cone"]
            self.assertAlmostEqual(mine["stroke"]["travel_m"],
                                   ours["stroke"]["travel_m"] / 2.0,
                                   delta=1e-9 + 1e-6 * ours["stroke"]["travel_m"])

    def test_page_knows_the_kind(self):
        with open(os.path.join(ROOT, "site", "map.html"),
                  encoding="utf-8") as fo:
            self.assertIn('TASK_KIND = "%s"' % X.TASK_KIND, fo.read())

    def test_json_safe(self):
        json.dumps(self.doc, allow_nan=False)

    def refused(self, change, words):
        task = json.loads(json.dumps(self.task))
        change(task)
        with self.assertRaises(ValueError) as e:
            X.load_task_file(self.write(task, "bad.json"))
        self.assertIn(words, str(e.exception))

    def test_stored_id_refused(self):
        self.refused(lambda t: t.update(id="cup_turn"), "an id of its own")

    def test_bad_numbers_refused(self):
        self.refused(lambda t: t["object"].update(radius_m=0), "above 0")
        self.refused(lambda t: t.update(mu="0.5"), "must be a number")
        self.refused(lambda t: t["contact_ring"].update(altitude_deg=90),
                     "between -90 and 90")
        self.refused(lambda t: t["wave"].update(duty=1.5), "at most 1")

    def test_command_line(self):
        out = os.path.join(self.dir.name, "out")
        os.makedirs(out)
        args = ["--task", self.path, "--tag", "t", "--out", out]
        self.assertEqual(X.main(args), 0)
        with open(os.path.join(out, "map_task_t.json"), encoding="utf-8") as fo:
            self.assertEqual(json.load(fo)["kind"], X.TASK_KIND)
        self.assertEqual(X.main(args), 1)                  # will not overwrite
        self.assertEqual(X.main(["--task", self.path, "--site"]), 1)


if __name__ == "__main__":
    unittest.main()
