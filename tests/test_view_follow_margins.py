"""The page and table of both margins (tools/view_follow.py) and the
comparison of a re-run with its stored result (tools/compare_follow.py),
on made-up results (python -m unittest). No mujoco needed."""
import copy
import importlib.util
import os
import subprocess
import sys
import tempfile
import json
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _tool(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT, "tools", name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


vf = _tool("view_follow")
cf = _tool("compare_follow")


def _finger(name, state):
    return {"finger": name, "station": 0, "state": state}


def _result(margins=True):
    """Three frames: green, red, amber, with both margins on each."""
    fr = []
    for k, (verd, sm, dm) in enumerate([("green", 3.0, 0.5), ("red", 0.0, 0.0), ("amber", 1.2, 0.25)]):
        f = {"k": k, "verdict": verd, "reason": "torque" if verd == "red" else None,
             "fingers": [_finger("f0", verd)]}
        if margins:
            f["scale_margin"] = {"margin": sm, "capped": False, "contiguous": True}
            f["disturbance_margin"] = {"margin": dm, "leader_wrench": 1.0,
                                       "direction": "+fx" if dm else None, "capped": False}
        fr.append(f)
    return {"hand": "h", "hand_name": "Hand", "leader": "hold_h", "assignment": {"f0": 0},
            "placement_cost_mm": 0.0, "placement": {"t": [0, 0, 0]}, "respace_deg": [0],
            "settings": {"scale_margin": margins, "disturbance_margin": margins, "x": 1},
            "frames": fr}


class Formatting(unittest.TestCase):
    def test_fmt_margin(self):
        self.assertEqual(vf.fmt_margin(None), "not computed")
        self.assertEqual(vf.fmt_margin({"margin": 1024.0, "capped": True}), "at least 1024")
        self.assertEqual(vf.fmt_margin({"margin": 1.4775390625, "capped": False}), "1.48")
        self.assertEqual(vf.fmt_margin({"margin": 77.46, "direction": "+fy", "capped": False}),
                         "77.5 (+fy)")
        self.assertIn("not one run", vf.fmt_margin({"margin": 2.0, "contiguous": False}))

    def test_smallest(self):
        r = _result()
        self.assertEqual(vf.smallest(r["frames"], "scale_margin")[1], 1)
        self.assertIsNone(vf.smallest(_result(False)["frames"], "scale_margin"))
        fr = copy.deepcopy(r["frames"])
        for f in fr:
            f["scale_margin"] = None
        self.assertEqual(vf.smallest(fr, "scale_margin"), (None, None))

    def test_hand_text_gives_the_passing_frames_too(self):
        t = vf.hand_margin_text(_result()["frames"], "scale_margin")
        self.assertEqual(t, "0, at frame 1; over the frames that pass, 1.2, at frame 2")
        self.assertEqual(vf.hand_margin_text(_result(False)["frames"], "scale_margin"), "not stored")


class PageAndTable(unittest.TestCase):
    def test_page_and_table(self):
        follow_html = os.path.join(ROOT, "outputs", "follow.html")
        before = os.path.getmtime(follow_html) if os.path.exists(follow_html) else None
        with tempfile.TemporaryDirectory() as d:
            paths = []
            for i, r in enumerate((_result(), _result(False))):
                p = os.path.join(d, f"r{i}.json")
                with open(p, "w", encoding="utf-8") as f:
                    json.dump(r, f)
                paths.append(p)
            page, table = os.path.join(d, "p.html"), os.path.join(d, "t.md")
            subprocess.run([sys.executable, os.path.join(ROOT, "tools", "view_follow.py"), *paths,
                            "--out", page, "--table", table], check=True, capture_output=True)
            with open(page, encoding="utf-8") as f:
                h = f.read()
            with open(table, encoding="utf-8") as f:
                t = f.read()
        for key in vf.KEYS:
            self.assertIn(vf.esc(vf.MEANING[key]), h)
            self.assertIn(vf.MEANING[key], t)
        self.assertIn("Margins frame by frame", h)
        self.assertIn("disturbance margin 0.5 (+fx)", h)  # beside the verdict, on hover
        rows = [ln for ln in t.splitlines() if ln.startswith("| h |")]
        self.assertEqual(len(rows), 2)
        self.assertIn("0.25 (+fx), at frame 2", rows[0])
        self.assertIn("not stored", rows[1])
        after = os.path.getmtime(follow_html) if os.path.exists(follow_html) else None
        self.assertEqual(before, after)  # --out leaves the default page alone


class Compare(unittest.TestCase):
    def test_same(self):
        r = _result()
        self.assertEqual(cf.compare(r, copy.deepcopy(r))[:2], ([], []))

    def test_margins_asked_for_are_a_note(self):
        old = _result(False)
        diff, sets, notes = cf.compare(_result(), old)
        self.assertEqual((diff, sets), ([], []))
        self.assertTrue(any("scale_margin" in n for n in notes))

    def test_finger_state_differs(self):
        new = _result()
        new["frames"][1]["fingers"][0]["state"] = "amber"
        diff, _, _ = cf.compare(new, _result())
        self.assertEqual(diff, ["frame 1, f0 state: red -> amber"])

    def test_scale_margin_differs(self):
        new = _result()
        new["frames"][0]["scale_margin"]["margin"] = 3.5
        self.assertEqual(len(cf.compare(new, _result())[0]), 1)

    def test_setting_listed_apart(self):
        new = _result()
        new["settings"]["x"] = 2
        diff, sets, _ = cf.compare(new, _result())
        self.assertEqual(diff, [])
        self.assertEqual(sets, ["setting x: 1 -> 2"])


if __name__ == "__main__":
    unittest.main()
