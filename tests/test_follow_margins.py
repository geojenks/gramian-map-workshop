"""tools/run_follow.py --margins (and --scale-margin, --disturbance-margin)
turns on both margins in follow(), and a re-run of a stored hold with its
stored placement keeps its verdicts and finger states (python -m unittest).
The end-to-end check needs mujoco and is skipped without it."""
import contextlib
import importlib.util
import io
import json
import os
import sys
import tempfile
import unittest
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

try:
    import mujoco  # noqa: F401
except ImportError:
    mujoco = None

HAND = "george_measured_thumb"
LID = f"hold_{HAND}__self__selfoverlapfix"
STORED = os.path.join(ROOT, "outputs", f"follow_{HAND}__{LID}__fixed.json")


def _tool(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT, "tools", name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _argv(out_dir, *flags):
    return ["run_follow.py", "--hands", HAND, "--leader-id", LID, "--frames", "2",
            "--placement-from", STORED, "--tag", "t", "--out-dir", out_dir, *flags]


class Flags(unittest.TestCase):
    """The flags reach follow(); follow itself is replaced, so no mujoco."""

    def _kw(self, *flags):
        rf = _tool("run_follow")
        with open(STORED, encoding="utf-8") as f:
            stored = json.load(f)
        seen = {}

        def fake(*a, **kw):
            seen.update(kw)
            return json.loads(json.dumps(stored))
        with tempfile.TemporaryDirectory() as d, mock.patch.object(rf, "follow", fake), \
                mock.patch.object(rf, "placement_key", lambda *a, **kw: None), \
                mock.patch.object(sys, "argv", _argv(d, *flags)), \
                contextlib.redirect_stdout(io.StringIO()) as out:
            rf.main()
        return seen, out.getvalue()

    def test_defaults_off(self):
        kw, _ = self._kw()
        self.assertEqual((kw["margin"], kw["disturbance"]), (False, False))

    def test_margins_turns_on_both(self):
        kw, out = self._kw("--margins")
        self.assertEqual((kw["margin"], kw["disturbance"]), (True, True))
        self.assertIn("scale margin: smallest", out)

    def test_each_on_its_own(self):
        self.assertEqual(self._kw("--scale-margin")[0]["disturbance"], False)
        self.assertEqual(self._kw("--disturbance-margin")[0]["margin"], False)

    def test_margin_lines(self):
        rf = _tool("run_follow")
        fr = [{"k": 0, "scale_margin": {"margin": 1024.0, "capped": True},
               "disturbance_margin": {"margin": 2.0, "direction": "-tz", "capped": False}},
              {"k": 3, "scale_margin": {"margin": 5.0, "capped": False},
               "disturbance_margin": None}]
        self.assertEqual(rf.margin_lines({"frames": fr}),
                         ["scale margin: smallest 5 (frame 3)",
                          "disturbance margin: smallest 2 (frame 0, worst direction -tz)"])
        self.assertEqual(rf.margin_lines({"frames": [{"k": 0}]}), [])


@unittest.skipIf(mujoco is None, "needs mujoco")
class EndToEnd(unittest.TestCase):
    def test_rerun_keeps_the_stored_verdicts(self):
        rf, cf = _tool("run_follow"), _tool("compare_follow")
        with tempfile.TemporaryDirectory() as d:
            with mock.patch.object(sys, "argv", _argv(d, "--margins")), \
                    contextlib.redirect_stdout(io.StringIO()):
                rf.main()
            with open(os.path.join(d, f"follow_{HAND}__{LID}__t.json"), encoding="utf-8") as f:
                new = json.load(f)
        with open(STORED, encoding="utf-8") as f:
            old = json.load(f)
        diff, _, _ = cf.compare(new, old)
        self.assertEqual(diff, [])
        for fr in new["frames"]:
            self.assertIn("margin", fr["scale_margin"])
            self.assertIn("margin", fr["disturbance_margin"])


if __name__ == "__main__":
    unittest.main()
