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


class OwnTarget(unittest.TestCase):
    """tools/leader_to_map_target.py: a simulation's task file as a target
    the map page draws. The stored example must be what the tool writes
    today, on axes the page has, and in the frame of the palm placement."""
    EXAMPLE = os.path.join(ROOT, "examples",
                           "map_target_cup_turn_contact_paths.json")
    LEADER = os.path.join(ROOT, "data", "leaders",
                          "w12c_cup_analytic_lock.json")

    @classmethod
    def setUpClass(cls):
        import leader_to_map_target as T
        cls.T = T
        with open(cls.EXAMPLE, encoding="utf-8") as fo:
            cls.example = json.load(fo)
        cls.dir = tempfile.TemporaryDirectory()

    @classmethod
    def tearDownClass(cls):
        cls.dir.cleanup()

    def leader(self, points, name="made.json", centre=(0.0, 0.0, 0.0)):
        """A one-station task file whose contact point visits `points`;
        None is a frame out of contact."""
        frames = [{"t": 0.1 * i, "obj_pos": list(centre),
                   "contact": [p is not None],
                   "contact_point": [None if p is None else list(p)]}
                  for i, p in enumerate(points)]
        path = os.path.join(self.dir.name, name)
        with open(path, "w", encoding="utf-8") as fo:
            json.dump({"id": "made", "frames": frames}, fo)
        return path

    def test_page_knows_the_kind(self):
        with open(os.path.join(ROOT, "site", "map.html"),
                  encoding="utf-8") as fo:
            self.assertIn('TARGET_KIND = "%s"' % self.T.KIND, fo.read())

    def test_example_is_what_the_tool_writes(self):
        self.assertEqual(self.T.convert(self.LEADER), self.example)
        json.dumps(self.example, allow_nan=False)

    def test_example_axes_are_the_pages(self):
        with open(os.path.join(ROOT, "site", "data", "map_paths.json"),
                  encoding="utf-8") as fo:
            have = {q["key"] for q in json.load(fo)["quantities"]}
        self.assertLessEqual(set(self.example["show"]), have)
        for t in self.example["targets"]:
            self.assertLessEqual(set(t["along"]), have)
            self.assertEqual(len({len(v) for v in t["along"].values()}), 1)

    def test_example_runs_along_the_palm_cones_direction(self):
        # the stored palm cone asks for travel along its drawn direction;
        # a turning task's contact paths must run mainly the same way
        cone = X.build(True, task_cones=X.OWN_MOUNTS["palm"])["task_cones"][0]
        self.assertEqual(cone["mount"], "palm")
        self.assertEqual(list(cone["axes"]), list(self.T.AXES))
        d = cone["drawn_direction"]
        size = sum(v * v for v in d) ** 0.5
        self.assertEqual(self.example["placement"]["yaw_deg"],
                         float(cone["yaw_deg"]))
        self.assertEqual(len(self.example["targets"]), 8)
        for t in self.example["targets"]:
            end = [t["along"][k][-1] for k in self.T.AXES]
            length = sum(v * v for v in end) ** 0.5
            self.assertGreater(length, 0.005, t["name"])
            along = sum(a * b for a, b in zip(end, d)) / size / length
            self.assertGreater(along, 0.9, t["name"])

    def test_frame_of_a_finger_on_the_palm(self):
        # contact at +x of the centre: up is axial, the way round (+y here)
        # is sideways y, and toward the centre is sideways z
        for step, axis in (((0, 0, 1), "axial_m"), ((0, 1, 0), "side_y_m"),
                           ((-1, 0, 0), "side_z_m")):
            pts = [(0.03 + 0.001 * i * step[0], 0.001 * i * step[1],
                    0.001 * i * step[2]) for i in range(3)]
            along = self.T.convert(self.leader(pts))["targets"][0]["along"]
            for k in self.T.AXES:
                self.assertAlmostEqual(along[k][-1],
                                       0.002 if k == axis else 0.0,
                                       places=6, msg=(axis, k))

    def test_still_contact_gives_no_travel(self):
        doc = self.T.convert(self.leader([(0.03, 0.0, 0.01)] * 4))
        for v in doc["targets"][0]["along"].values():
            self.assertEqual(v, [0.0] * 4)

    def test_longest_stretch_or_all(self):
        a, b = (0.03, 0.0, 0.0), (0.03, 0.001, 0.0)
        path = self.leader([a, b, None, a, b, b, None, a])
        one = self.T.convert(path)["targets"]
        self.assertEqual([t["frames"] for t in one], [[3, 5]])
        every = self.T.convert(path, all_stretches=True)["targets"]
        self.assertEqual([t["frames"] for t in every], [[0, 1], [3, 5]])

    def test_refused_or_left_out(self):
        with self.assertRaises(ValueError) as e:
            self.T.convert(os.path.join(ROOT, "data", "tasks",
                                        "cup_turn.json"))
        self.assertIn("not a task file with frames", str(e.exception))
        with self.assertRaises(ValueError) as e:        # never two frames
            self.T.convert(self.leader([(0.03, 0, 0), None, (0.03, 0, 0)]))
        self.assertIn("never in contact for two frames", str(e.exception))
        with self.assertRaises(ValueError) as e:        # no radial direction
            self.T.convert(self.leader([(0, 0, 0.03), (0, 0.001, 0.03)]))
        self.assertIn("no radial direction", str(e.exception))

    def test_command_line(self):
        out = os.path.join(self.dir.name, "out")
        args = [self.LEADER, "--tag", "t", "--out", out]
        self.assertEqual(self.T.main(args), 0)
        with open(os.path.join(out, "map_target_t.json"),
                  encoding="utf-8") as fo:
            self.assertEqual(json.load(fo), self.example)
        self.assertEqual(self.T.main(args), 1)             # will not overwrite


if __name__ == "__main__":
    unittest.main()
