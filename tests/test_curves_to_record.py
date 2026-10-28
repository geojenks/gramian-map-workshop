"""The test-curve format (docs/TEST_CURVE_FORMAT.md) and its converter,
tools/curves_to_record.py. The round trip: the filled example in
data/test_curves/ was written from the published-data record
mosadegh2014_fPN; converting it back must reproduce that record's curves.
numpy only."""
import contextlib
import importlib.util
import io
import json
import os
import shutil
import sys
import tempfile
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gramian_workshop.record_fit import fit_record  # noqa: E402
from gramian_workshop.vendor import actuator_record as ar  # noqa: E402

RECORD = os.path.join(ROOT, "data", "records", "mosadegh2014_fPN.json")
EXAMPLE = os.path.join(ROOT, "data", "test_curves", "mosadegh2014_fPN.csv")
STORED = os.path.join(ROOT, "outputs", "test_curves",
                      "mosadegh2014_fPN_from_curves.json")


def _load(name):
    spec = importlib.util.spec_from_file_location(
        name, os.path.join(ROOT, "tools", name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ctr = _load("curves_to_record")
check_record = _load("check_record")

HEAD = ("id,t\nname,t\nsource,experiment\ndetail,test rig\n"
        "motion,bends\n")


class RoundTrip(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old = ar.load(RECORD)
        cls.new, cls.notes = ctr.build_record(EXAMPLE)

    def test_example_is_the_record_written_out(self):
        with open(EXAMPLE, encoding="utf-8", newline="") as f:
            self.assertEqual(f.read(), ctr.record_to_curves(self.old))

    def test_curves_reproduce_the_record(self):
        cmp = ctr.compare(self.new, self.old)
        for name, v in cmp["curves"].items():
            self.assertIn(v["status"], ("identical", "both absent"), name)
        self.assertEqual(cmp["curves"]["free.tip_displacement_m"]["status"],
                         "identical")
        self.assertEqual(cmp["curves"]["blocked.force_transverse_N"]
                         ["status"], "identical")
        for k in ("polarity_axial", "blocked_force_N"):
            self.assertEqual(self.new["scalars"][k], self.old["scalars"][k])

    def test_headline_numbers_follow_the_workshop_rules(self):
        """Stroke is the largest tip displacement; the motion direction is
        the first level beyond 2% of it (docs/RECORDS.md). The stored
        record used other rules, so these two differ from it."""
        d = np.asarray(self.old["free"]["tip_displacement_m"])
        m = np.linalg.norm(d, axis=1)
        self.assertEqual(self.new["scalars"]["free_stroke_m"], m.max())
        k = int(np.argmax(m > 0.02 * m.max()))
        np.testing.assert_allclose(self.new["scalars"]["free_direction"],
                                   d[k] / m[k], rtol=0, atol=1e-15)

    def test_record_passes_check_record(self):
        errors, _ = ar.validate(self.new)
        self.assertEqual(errors, [])
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(check_record.check(STORED))

    def test_stored_output_reproduces(self):
        self.assertEqual(ar.load(STORED), self.new)


class Inputs(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def write(self, text, name="c.csv"):
        p = os.path.join(self.tmp, name)
        with open(p, "w", encoding="utf-8", newline="") as f:
            f.write(text)
        return p

    def test_angle_only_is_accepted(self):
        p = self.write(HEAD + "curve,input_bar,angle_deg\n"
                       "free,0,0\nfree,0.5,-10\nfree,1,-30\n")
        rec, _ = ctr.build_record(p)
        self.assertEqual(ar.validate(rec)[0], [])
        self.assertEqual(ar.tier(rec), 1)
        self.assertNotIn("tip_displacement_m", rec["free"])
        np.testing.assert_allclose(rec["free"]["bend_angle_rad"],
                                   np.radians([0, -10, -30]))
        self.assertEqual(rec["free"]["u"], [[0.0], [5e4], [1e5]])
        s = rec["scalars"]
        self.assertIsNone(s["free_stroke_m"])
        self.assertEqual(s["free_direction"], [0.0, -1.0, 0.0])
        self.assertIsNone(s["blocked_force_N"])
        out = os.path.join(self.tmp, "r.json")
        ar.save(rec, out)
        fit = fit_record(out)
        self.assertEqual(fit["status"], "refused")
        self.assertIn("no free path", fit["reason"])

    def test_backbone_gives_tip_path_and_length(self):
        p = self.write(HEAD + "force_direction,axial\n"
                       "curve,input_kPa,point,x_cm,y_cm,force_gf\n"
                       "backbone,0,0,0,0,\nbackbone,0,1,3,0,\n"
                       "backbone,0,2,6,0,\n"
                       "backbone,20,0,0,0,\nbackbone,20,1,2.9,0.5,\n"
                       "backbone,20,2,5.5,1.5,\n"
                       "blocked,20,,,,30\nblocked,10,,,,12\n")
        rec, _ = ctr.build_record(p)
        self.assertEqual(ar.validate(rec)[0], [])
        np.testing.assert_allclose(rec["free"]["tip_displacement_m"],
                                   [[0, 0, 0], [-0.005, 0.015, 0]],
                                   atol=1e-15)
        self.assertAlmostEqual(rec["metadata"]["length_m"], 0.06, places=15)
        self.assertEqual(rec["blocked"]["u"], [[1e4], [2e4]])
        np.testing.assert_allclose(rec["blocked"]["force_axial_N"],
                                   [12 * 9.80665e-3, 30 * 9.80665e-3])
        self.assertIsNone(rec["blocked"]["force_transverse_N"])

    def test_force_components_give_the_vector(self):
        p = self.write(HEAD + "curve,input_psi,force_x_N,force_y_N\n"
                       "blocked,1,0.1,0.5\nblocked,2,0.2,1.0\n")
        rec, _ = ctr.build_record(p)
        self.assertEqual(rec["blocked"]["force_N"],
                         [[0.1, 0.5, 0.0], [0.2, 1.0, 0.0]])
        self.assertEqual(rec["blocked"]["force_axial_N"], [0.1, 0.2])
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(check_record.check(
                self._saved(rec)))

    def _saved(self, rec):
        out = os.path.join(self.tmp, "rec.json")
        ar.save(rec, out)
        return out

    def test_malformed_files_are_refused(self):
        bad = {
            "unknown unit": "curve,input_kPa,angle_grad\nfree,1,2\n",
            "no input column": "curve,angle_deg\nfree,2\n",
            "repeated input": "curve,input_kPa,angle_deg\n"
                              "free,1,2\nfree,1,3\n",
            "size without direction": "curve,input_kPa,force_N\n"
                                      "blocked,1,2\n",
            "tip path twice": "curve,input_kPa,tip_dx_mm,tip_dy_mm,point,"
                              "x_mm,y_mm\nfree,1,0,1,,,\n"
                              "backbone,0,0,,,0,0\nbackbone,0,1,,,1,0\n",
            "half a tip path": "curve,input_kPa,tip_dx_mm,tip_dy_mm\n"
                               "free,1,0,1\nfree,2,0,\n",
            "unknown curve": "curve,input_kPa,angle_deg\nsideways,1,2\n",
        }
        for why, table in bad.items():
            with self.subTest(why):
                with self.assertRaises(ctr.CurveFileError):
                    ctr.build_record(self.write(HEAD + table))
        with self.assertRaises(ctr.CurveFileError):
            ctr.build_record(self.write("id,t\ncurve,input_kPa,angle_deg\n"
                                        "free,1,2\n"))

    def test_existing_output_is_never_overwritten(self):
        out = self.write("keep me", "r.json")
        with self.assertRaises(SystemExit), \
                contextlib.redirect_stdout(io.StringIO()):
            ctr.main([EXAMPLE, "--out", out])
        with open(out) as f:
            self.assertEqual(f.read(), "keep me")


if __name__ == "__main__":
    unittest.main()
