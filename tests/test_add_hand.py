"""A hand from a model file we did not prepare: the description file, the
folder of added hands that gramian_workshop/hands.py reads, and
tools/add_hand.py end to end on small models written here."""
import contextlib
import importlib.util
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop import hands  # noqa: E402

try:
    import mujoco
except ImportError:
    mujoco = None

TOOL = os.path.join(ROOT, "tools", "add_hand.py")
_spec = importlib.util.spec_from_file_location("add_hand", TOOL)
add_hand = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(add_hand)

SCHEMA = add_hand.DESCRIPTION_SCHEMA


def finger_mjcf(name, y):
    return f"""
      <body name="{name}_prox" pos="0.03 {y} 0">
        <joint name="{name}_j0" axis="0 1 0" range="-0.5 1.5" actuatorfrcrange="-1 1"/>
        <geom type="capsule" fromto="0 0 0 0.04 0 0" size="0.008"/>
        <body name="{name}_tip" pos="0.04 0 0">
          <joint name="{name}_j1" axis="0 1 0" range="0 1.5" actuatorfrcrange="-1 1"/>
          <geom type="capsule" fromto="0 0 0 0.03 0 0" size="0.007"/>
        </body>
      </body>"""


MJCF = f"""<mujoco>
  <worldbody>
    <body name="palm">
      <geom type="box" size="0.03 0.04 0.01"/>
      {finger_mjcf("a", -0.03)}{finger_mjcf("b", 0.0)}{finger_mjcf("c", 0.03)}
    </body>
  </worldbody>
</mujoco>
"""


def finger_urdf(name, y, mimic):
    box = ('<collision><origin xyz="0.02 0 0"/><geometry><box size="0.04 0.012 0.012"/></geometry>'
           '</collision><inertial><mass value="0.02"/>'
           '<inertia ixx="1e-6" iyy="1e-6" izz="1e-6" ixy="0" ixz="0" iyz="0"/></inertial>')
    return f"""
  <link name="{name}_prox">{box}</link>
  <link name="{name}_dist">{box}</link>
  <link name="{name}_pad">{box}</link>
  <joint name="{name}_j0" type="revolute"><parent link="base"/><child link="{name}_prox"/>
    <origin xyz="0.03 {y} 0"/><axis xyz="0 1 0"/><limit lower="-0.5" upper="1.5" effort="1" velocity="1"/></joint>
  <joint name="{name}_j1" type="revolute"><parent link="{name}_prox"/><child link="{name}_dist"/>
    <origin xyz="0.04 0 0"/><axis xyz="0 1 0"/><limit lower="0" upper="1.5" effort="1" velocity="1"/>
    {f'<mimic joint="{name}_j0" multiplier="0.8" offset="0"/>' if mimic else ''}</joint>
  <joint name="{name}_fix" type="fixed"><parent link="{name}_dist"/><child link="{name}_pad"/>
    <origin xyz="0.04 0 0"/></joint>"""


URDF = f"""<?xml version="1.0"?>
<robot name="tiny">
  <link name="base"><collision><geometry><box size="0.06 0.08 0.02"/></geometry></collision></link>
  {finger_urdf("a", -0.03, True)}{finger_urdf("b", 0.0, False)}{finger_urdf("c", 0.03, False)}
</robot>
"""


def desc(**over):
    d = {"schema": SCHEMA, "key": "tiny_hand", "name": "Tiny hand",
         "fingers": {"thumb": "a_tip", "index": "b_tip", "middle": "c_tip"}}
    d.update(over)
    return d


class Folder(unittest.TestCase):
    """A temporary folder, and the table of hands put back afterwards."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = self._tmp.name
        self.reg = os.path.join(self.tmp, "added")
        self._before = dict(hands.HANDS)

    def tearDown(self):
        hands.HANDS.clear()
        hands.HANDS.update(self._before)
        self._tmp.cleanup()

    def write(self, name, text):
        p = os.path.join(self.tmp, name)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w", encoding="utf-8") as f:
            f.write(text if isinstance(text, str) else json.dumps(text))
        return p

    def entry(self, folder, **over):
        e = {"schema": hands.ADDED_SCHEMA, "key": folder, "name": "A hand", "model": "hand.xml",
             "fingers": {"thumb": "a_tip", "index": "b_tip"}}
        e.update(over)
        return self.write(os.path.join("added", folder, hands.ADDED_ENTRY), e)

    def run_tool(self, *args):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = add_hand.main(list(args) + ["--registry", self.reg])
        return code, out.getvalue()


class TestDescription(unittest.TestCase):
    def test_a_sound_description_has_no_problems(self):
        self.assertEqual(add_hand.check_description(desc()), [])
        self.assertEqual(add_hand.check_description(
            desc(datasheet_torque_Nm=[0.7, "a data sheet"], caveat="early model")), [])
        self.assertEqual(add_hand.check_description(
            desc(datasheet_no_load_speed=[81, "rev/min", "a data sheet"])), [])

    def test_each_fault_is_said_in_words(self):
        for d, word in [
            (desc(schema="x"), '"schema"'),
            (desc(key="Bad Key"), '"key"'),
            (desc(key=next(iter(hands.BUILT_IN))), "already has"),
            (desc(fingers={}), '"fingers" is missing'),
            (desc(fingers={"thumb": "a_tip"}), "Only one finger"),
            (desc(fingers={"thumb": "a_tip", "index": "a_tip"}), "more than one finger"),
            (desc(fingers={"thumb": 3, "index": "b_tip"}), "body name in quotes"),
            (desc(datasheet_torque_Nm=0.7), '"datasheet_torque_Nm"'),
            (desc(datasheet_torque_Nm=[-1, "x"]), '"datasheet_torque_Nm"'),
            (desc(datasheet_no_load_speed=[81, "rev/min"]), '"datasheet_no_load_speed"'),
            (desc(datasheet_no_load_speed=[81, "furlongs", "x"]), '"datasheet_no_load_speed"'),
            (desc(datasheet_no_load_speed=[0, "rev/min", "x"]), '"datasheet_no_load_speed"'),
            (desc(colour="red"), '"colour" is not something'),
            ([1, 2], "one JSON object"),
        ]:
            bad = add_hand.check_description(d)
            self.assertTrue(any(word in b for b in bad), (word, bad))

    def test_every_fault_is_reported_at_once(self):
        bad = add_hand.check_description({"key": "Bad Key", "fingers": {"t": "x"}, "colour": 1})
        self.assertEqual(len(bad), 4)


class TestAddedFolder(Folder):
    def test_an_entry_is_found_by_name_and_dropped_again(self):
        self.entry("some_hand", datasheet_torque_Nm=[0.5, "a data sheet"],
                   datasheet_no_load_speed=[60, "rev/min", "a data sheet"])
        self.assertEqual(hands.load_added(self.reg, warn=False), ["some_hand"])
        spec = hands.HANDS["some_hand"]
        self.assertEqual(spec["fingers"], {"thumb": "a_tip", "index": "b_tip"})
        self.assertEqual(spec["datasheet_torque_Nm"], (0.5, "a data sheet"))
        self.assertEqual(spec["datasheet_no_load_speed"], (60.0, "rev/min", "a data sheet"))
        self.assertAlmostEqual(hands.no_load_rad_s(spec)[0], 2.0 * 3.141592653589793)
        self.assertTrue(os.path.samefile(os.path.dirname(hands.model_path("some_hand")),
                                         os.path.join(self.reg, "some_hand")))
        os.remove(os.path.join(self.reg, "some_hand", hands.ADDED_ENTRY))
        self.assertEqual(hands.load_added(self.reg, warn=False), [])
        self.assertNotIn("some_hand", hands.HANDS)

    def test_a_bad_entry_is_skipped_and_named_not_raised(self):
        self.entry("good_hand")
        self.write(os.path.join("added", "broken", hands.ADDED_ENTRY), "{not json")
        self.entry("clash", key=next(iter(hands.BUILT_IN)))
        self.entry("one_finger", fingers={"thumb": "a_tip"})
        self.assertEqual(hands.load_added(self.reg, warn=False), ["good_hand"])
        self.assertEqual(len(hands.ADDED_PROBLEMS), 3)
        for k in hands.BUILT_IN:
            self.assertNotIn("added", hands.HANDS[k])

    def test_a_missing_folder_adds_nothing(self):
        self.assertEqual(hands.load_added(os.path.join(self.tmp, "nowhere"), warn=False), [])
        self.assertEqual(set(hands.HANDS), set(self._before) - {
            k for k, v in self._before.items() if v.get("added")})


@unittest.skipIf(mujoco is None, "mujoco is not installed in this Python")
class TestTool(Folder):
    def test_mjcf_is_checked_added_listed_and_removed(self):
        m = self.write("tiny.xml", MJCF)
        d = self.write("tiny.json", desc())
        code, out = self.run_tool(m, d)
        self.assertEqual(code, 0, out)
        self.assertIn('Added. The tools now know this hand as "tiny_hand"', out)
        self.assertIn("palm (bodies of no finger that have collision geometry): palm", out)
        hands.load_added(self.reg, warn=False)
        h = hands.Hand("tiny_hand")
        self.assertEqual([f.label for f in h.fingers], ["thumb", "index", "middle"])
        self.assertEqual([len(f.joints) for f in h.fingers], [2, 2, 2])
        code, out = self.run_tool(m, d)                # a second time: refused in words
        self.assertEqual(code, 2)
        self.assertIn("already added", out)
        self.assertEqual(self.run_tool(m, d, "--replace")[0], 0)
        self.assertIn("tiny_hand: Tiny hand", self.run_tool("--list")[1])
        self.assertEqual(self.run_tool("--remove", "tiny_hand")[0], 0)
        self.assertIn("No hands have been added", self.run_tool("--list")[1])
        self.assertTrue(os.path.isfile(m))             # the visitor's model is left alone

    def test_check_only_adds_nothing(self):
        code, out = self.run_tool(self.write("tiny.xml", MJCF), self.write("tiny.json", desc()),
                                  "--check-only")
        self.assertEqual(code, 0, out)
        self.assertFalse(os.path.exists(os.path.join(self.reg, "tiny_hand")))

    def test_model_alone_lists_bodies_and_a_starter(self):
        code, out = self.run_tool(self.write("tiny.xml", MJCF))
        self.assertEqual(code, 0)
        self.assertIn("a_tip", out)
        start = json.loads(out[out.index("{"):])
        self.assertEqual(sorted(start["fingers"].values()), ["a_tip", "b_tip", "c_tip"])

    def test_urdf_keeps_its_mimic_coupling_and_names_fused_links(self):
        u = self.write("tiny.urdf", URDF)
        d = self.write("tiny.json", desc(fingers={"thumb": "a_dist", "index": "b_dist",
                                                  "middle": "c_dist"}))
        code, out = self.run_tool(u, d)
        self.assertEqual(code, 0, out)
        self.assertIn("1 couplings from <mimic> tags", out)
        hands.load_added(self.reg, warn=False)
        h = hands.Hand("tiny_hand")
        self.assertEqual([h.model.joint(j).name for j in h.coupled], ["a_j1"])
        self.assertEqual([h.inputs(f)[0] for f in h.fingers], [1, 2, 2])
        d2 = self.write("fused.json", desc(key="fused", fingers={
            "thumb": "a_pad", "index": "b_dist", "middle": "c_dist"}))
        code, out = self.run_tool(u, d2)
        self.assertEqual(code, 2)
        self.assertIn('Name "a_dist" instead', out)

    def test_faults_in_the_pair_are_said_in_words(self):
        m = self.write("tiny.xml", MJCF)
        nolimit = self.write("nolimit.xml", MJCF.replace('range="0 1.5" ', "", 1))
        display = self.write("display.xml", MJCF.replace(
            'fromto="0 0 0 0.03 0 0" size="0.007"',
            'fromto="0 0 0 0.03 0 0" size="0.007" contype="0" conaffinity="0"', 1))
        for model, fingers, word in [
            (m, {"thumb": "a_tipp", "index": "b_tip"}, 'no body named "a_tipp". Close names: a_tip'),
            (m, {"thumb": "a_tip", "index": "a_prox"}, "no joint of its own"),
            (m, {"thumb": "a_tip", "index": "palm", "middle": "c_tip"}, "no joint of its own"),
            (nolimit, {"thumb": "a_tip", "index": "b_tip"}, "has no range of motion"),
            (display, {"thumb": "a_tip", "index": "b_tip"}, "has no collision geometry"),
        ]:
            code, out = self.run_tool(model, self.write("d.json", desc(fingers=fingers)))
            self.assertEqual(code, 2, out)
            self.assertIn(word, out)
            self.assertNotIn("Traceback", out)
            self.assertFalse(os.path.exists(os.path.join(self.reg, "tiny_hand")))
        code, out = self.run_tool(self.write("notes.txt", "hello"), self.write("d.json", desc()))
        self.assertEqual(code, 2)
        self.assertIn("not well-formed XML", out)

    def test_a_broken_description_gives_a_message_not_a_traceback(self):
        m = self.write("tiny.xml", MJCF)
        for text in ['{"schema": "%s", "key": "tiny_hand", "fingers": {"thumb": "a_tip"' % SCHEMA,
                     json.dumps({"key": "Bad Key", "fingers": "a_tip"}), ""]:
            r = subprocess.run([sys.executable, TOOL, m, self.write("broken.json", text),
                                "--registry", self.reg], capture_output=True, text=True)
            self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
            self.assertIn("Not added.", r.stdout)
            self.assertNotIn("Traceback", r.stdout + r.stderr)

    def test_adding_a_hand_leaves_the_placement_search_code_as_it_was(self):
        from gramian_workshop import follow
        before = follow._search_code()
        self.assertEqual(self.run_tool(self.write("tiny.xml", MJCF),
                                       self.write("tiny.json", desc()))[0], 0)
        self.assertEqual(follow._search_code(), before)


if __name__ == "__main__":
    unittest.main()
