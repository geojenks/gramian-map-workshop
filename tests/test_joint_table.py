"""The joint-table format (docs/JOINT_TABLE_FORMAT.md) and its converter,
tools/joint_table_to_record.py. The worked example in data/joint_tables/
is the Shadow finger; its record must agree with the stored
data/records/shadow_hand_finger.json where the doc says it does. numpy
only."""
import contextlib
import importlib.util
import io
import json
import math
import os
import re
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.vendor import actuator_record as ar  # noqa: E402

STORED_RECORD = os.path.join(ROOT, "data", "records", "shadow_hand_finger.json")
EXAMPLE = os.path.join(ROOT, "data", "joint_tables", "shadow_hand_finger.csv")
OUT = os.path.join(ROOT, "outputs", "joint_table",
                   "shadow_hand_finger_from_joint_table.json")
MENAGERIE_XML = os.path.join(ROOT, "external", "mujoco_menagerie",
                             "shadow_hand", "right_hand.xml")
TOL = 1e-6


def _load(name):
    spec = importlib.util.spec_from_file_location(
        name, os.path.join(ROOT, "tools", name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


jt = _load("joint_table_to_record")
check_record = _load("check_record")

HEAD = "id,t\nname,t\nsource,literature\ndetail,a test table\n\n"
COLS = ("joint,axis_x,axis_y,axis_z,pos_x_mm,pos_y_mm,pos_z_mm,"
        "min_deg,max_deg,torque_Nm,stiffness_Nm_per_rad\n")
ROWS = ("A,0,0,1,0,0,0,0,90,1,0.5\n"
        "B,0,0,1,40,0,0,0,90,1,0.5\n"
        "tip,,,,70,0,0,,,,\n")


class Example(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rec, cls.notes = jt.build_record(EXAMPLE)
        cls.old = ar.load(STORED_RECORD)
        cls.details, cls.joints, cls.tip, _ = jt.read_joint_table(EXAMPLE)

    def test_converts_and_validates(self):
        errors, _ = ar.validate(self.rec)
        self.assertEqual(errors, [])
        self.assertEqual(ar.tier(self.rec), 1)

    def test_check_record_passes_on_the_stored_output(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(check_record.check(OUT))

    def test_stored_output_is_a_fresh_conversion(self):
        with open(OUT, encoding="utf-8") as f:
            stored = json.load(f)
        sha = re.compile(r"sha256 [0-9a-f]{64}")
        for r in (stored, self.rec):
            r["metadata"]["notes"] = sha.sub("sha256 -",
                                             r["metadata"]["notes"])
        self.assertEqual(stored, json.loads(json.dumps(self.rec)))

    def test_agrees_with_the_stored_record(self):
        rows = {f: ok for f, _, _, ok in jt.compare(self.rec, self.old, TOL)}
        for field in ("metadata.length_m", "metadata.actuation_physics",
                      "scalars.free_direction", "scalars.polarity_axial",
                      "scalars.effective_dof"):
            self.assertTrue(rows[field], field)

    def test_full_flex_is_the_stored_stroke(self):
        last = self.rec["free"]["tip_displacement_m"][-1]
        self.assertAlmostEqual(float(np.linalg.norm(last)),
                               self.old["scalars"]["free_stroke_m"],
                               delta=TOL)

    def test_stroke_is_the_largest_displacement(self):
        d = np.linalg.norm(self.rec["free"]["tip_displacement_m"], axis=1)
        self.assertEqual(self.rec["scalars"]["free_stroke_m"], float(d.max()))
        self.assertGreater(d.max(), d[-1])

    def test_table_numbers_are_the_stored_records(self):
        L = [0.045, 0.025, 0.026]
        xs = [j["pos"][0] for j in self.joints] + [self.tip[0]]
        self.assertTrue(np.allclose(np.diff(xs), L, atol=1e-12))
        self.assertAlmostEqual(self.tip[0] - self.joints[0]["pos"][0],
                               self.old["metadata"]["length_m"], places=12)
        text = self.old["source"]["detail"] + self.old["metadata"]["notes"]
        self.assertIn("45/25/26 mm", text)
        for name, deg in zip(("MCP", "PIP", "DIP"), (80, 100, 70)):
            self.assertIn(f"{name} {deg} deg", text)
        for j, deg in zip(self.joints, (80, 100, 70)):
            self.assertAlmostEqual(j["hi"], math.radians(deg), places=12)
            self.assertEqual(j["lo"], 0.0)

    @unittest.skipUnless(os.path.exists(MENAGERIE_XML),
                         "external/mujoco_menagerie not fetched")
    def test_torque_is_the_menagerie_models(self):
        root = ET.parse(MENAGERIE_XML).getroot()
        parent = {c: p for p in root.iter() for c in p}

        def forcerange(cls):
            node = next(d for d in root.iter("default")
                        if d.get("class") == cls)
            while node is not None:
                pos = node.find("position")
                if pos is not None and pos.get("forcerange"):
                    return [float(x) for x in pos.get("forcerange").split()]
                node = parent.get(node)
            return None

        acts = {a.get("name"): a for a in root.iter("position")
                if a.get("name")}
        lims = [max(abs(x) for x in forcerange(acts[n].get("class")))
                for n in ("rh_A_FFJ3", "rh_A_FFJ0")]
        self.assertEqual([j["torque"] for j in self.joints],
                         [lims[0], lims[1], lims[1]])
        for j in root.iter("joint"):
            self.assertIsNone(j.get("stiffness"))
        self.assertEqual([j["stiffness"] for j in self.joints], [0.0] * 3)

    def test_push_is_limited_by_the_base_joint(self):
        self.assertAlmostEqual(self.rec["scalars"]["blocked_force_N"],
                               1.0 / 0.096, places=9)
        self.assertTrue(any("J3" in n and "limited" in n
                            for n in self.notes))
        f = self.rec["blocked"]["force_N"][-1]
        e = jt.first_motion(self.joints, self.tip)
        self.assertTrue(np.allclose(e, [0.0, 1.0, 0.0], atol=1e-12))
        self.assertTrue(np.allclose(np.array(f) / np.linalg.norm(f), e,
                                    atol=1e-12))

    def test_stiffness_is_zero_without_springs(self):
        self.assertEqual(set(self.rec["stiffness"]["k_N_per_m"]), {0.0})


class Kinematics(unittest.TestCase):
    def test_jacobian_matches_a_finite_difference(self):
        _, joints, tip, _ = jt.read_joint_table(EXAMPLE)
        rng = np.random.default_rng(0)
        for _ in range(5):
            q = np.array([rng.uniform(j["lo"], j["hi"]) for j in joints])
            J = jt.jacobian(joints, tip, q)
            h = 1e-7
            for i in range(len(q)):
                dq = np.zeros_like(q)
                dq[i] = h
                fd = (jt.forward(joints, tip, q + dq)[0]
                      - jt.forward(joints, tip, q - dq)[0]) / (2 * h)
                self.assertTrue(np.allclose(J[:, i], fd, atol=1e-7))

    def test_out_of_plane_axis(self):
        joints = [{"name": "A", "axis": np.array([0.0, 0.0, 1.0]),
                   "pos": np.zeros(3), "lo": 0.0, "hi": 1.0},
                  {"name": "B", "axis": np.array([0.0, -1.0, 0.0]),
                   "pos": np.array([0.03, 0.0, 0.0]), "lo": 0.0, "hi": 1.0}]
        tip = np.array([0.05, 0.0, 0.0])
        q = np.array([0.4, 0.7])
        J = jt.jacobian(joints, tip, q)
        h = 1e-7
        for i in range(2):
            dq = np.zeros(2)
            dq[i] = h
            fd = (jt.forward(joints, tip, q + dq)[0]
                  - jt.forward(joints, tip, q - dq)[0]) / (2 * h)
            self.assertTrue(np.allclose(J[:, i], fd, atol=1e-7))


class Refusals(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.dir)

    def _file(self, text):
        p = os.path.join(self.dir, "t.csv")
        with open(p, "w", encoding="utf-8", newline="") as f:
            f.write(text)
        return p

    def _refused(self, text, *words):
        with self.assertRaises(jt.JointTableError) as cm:
            jt.build_record(self._file(text))
        for w in words:
            self.assertIn(w, str(cm.exception))

    def test_a_good_table_converts(self):
        rec, _ = jt.build_record(self._file(HEAD + COLS + ROWS))
        self.assertEqual(ar.validate(rec)[0], [])
        self.assertGreater(min(rec["stiffness"]["k_N_per_m"]), 0.0)

    def test_missing_column(self):
        self._refused(HEAD + COLS.replace(",torque_Nm", "")
                      + ROWS.replace(",1,0.5", ",0.5"), "torque")

    def test_length_without_unit(self):
        self._refused(HEAD + COLS.replace("pos_x_mm", "pos_x") + ROWS,
                      "pos_x")

    def test_unknown_unit(self):
        self._refused(HEAD + COLS.replace("torque_Nm", "torque_kgcm") + ROWS,
                      "kgcm")

    def test_empty_torque_cell(self):
        self._refused(HEAD + COLS + ROWS.replace("90,1,0.5", "90,,0.5", 1),
                      "unknown")

    def test_unknown_only_for_torque_and_stiffness(self):
        self._refused(HEAD + COLS
                      + ROWS.replace("0,90,1", "0,unknown,1", 1), "max")

    def test_unknown_torque_drops_the_push(self):
        rec, notes = jt.build_record(self._file(
            HEAD + COLS + ROWS.replace("90,1,0.5", "90,unknown,0.5", 1)))
        self.assertIsNone(rec["blocked"])
        self.assertTrue(any("blocked" in n for n in notes))

    def test_missing_tip(self):
        self._refused(HEAD + COLS + ROWS.replace("tip,,,,70,0,0,,,,\n", ""),
                      "tip")

    def test_range_must_hold_rest(self):
        self._refused(HEAD + COLS + ROWS.replace("0,0,0,0,90", "0,0,0,10,90",
                                                 1), "0")

    def test_missing_required_detail(self):
        self._refused(HEAD.replace("detail,a test table\n", "") + COLS
                      + ROWS, "detail")

    def test_refuses_to_overwrite(self):
        out = os.path.join(self.dir, "out.json")
        with open(out, "w") as f:
            f.write("{}")
        with self.assertRaises(SystemExit) as cm:
            jt.main([EXAMPLE, "--out", out])
        self.assertIn("refusing to overwrite", str(cm.exception))
        with open(out) as f:
            self.assertEqual(f.read(), "{}")


if __name__ == "__main__":
    unittest.main()
