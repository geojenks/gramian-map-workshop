"""tools/check_leader.py: every stored leader passes, and the checker catches
the faults the format rules out (python -m unittest). numpy only."""
import copy
import glob
import importlib.util
import io
import os
import unittest
from contextlib import redirect_stdout

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LEADERS = sorted(glob.glob(os.path.join(ROOT, "data", "leaders", "*.json")))


def _tool():
    spec = importlib.util.spec_from_file_location(
        "check_leader", os.path.join(ROOT, "tools", "check_leader.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


cl = _tool()


class StoredLeaders(unittest.TestCase):
    def test_there_are_leaders(self):
        self.assertTrue(LEADERS)

    def test_every_leader_passes(self):
        for p in LEADERS:
            with self.subTest(leader=os.path.basename(p)):
                rep, summary = cl.check(cl.load(p))
                self.assertEqual(rep.errors, {}, cl.Report.lines(rep.errors, []))
                self.assertIsNotNone(summary)

    def test_command_line_exit_code(self):
        with redirect_stdout(io.StringIO()):
            self.assertEqual(cl.main(LEADERS), 0)
            self.assertEqual(cl.main(["--table"] + LEADERS), 0)


def _minimal():
    """A two-station, two-frame leader that passes cleanly."""
    def frame(t, z):
        c = [0.0, 0.0, z]
        return {"t": t, "obj_pos": c, "obj_quat": [1.0, 0.0, 0.0, 0.0],
                "contact": [True, True],
                "contact_point": [[0.03, 0.0, z], [-0.03, 0.0, z]],
                "station_pos": [[0.04, 0.0, z], [-0.04, 0.0, z]],
                "contact_normal": [[-1.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                "force_on_object_N": [[-1.0, 0.0, 0.1], [1.0, 0.0, 0.1]]}
    return {"schema": cl.SCHEMA, "id": "minimal", "n_stations": 2,
            "object": {"shape": "sphere", "radius_m": 0.03, "mass_kg": 0.02},
            "frames": [frame(0.0, 0.1), frame(0.01, 0.1)]}


class Faults(unittest.TestCase):
    def errors(self, L):
        rep, _ = cl.check(L)
        return " | ".join(rep.errors)

    def warnings(self, L):
        rep, _ = cl.check(L)
        return " | ".join(rep.warnings)

    def test_minimal_is_clean(self):
        rep, s = cl.check(_minimal())
        self.assertEqual(rep.errors, {})
        self.assertEqual(rep.warnings, {})
        self.assertEqual(s["touching_min"], 2)

    def test_missing_mass(self):
        L = _minimal()
        del L["object"]["mass_kg"]
        self.assertIn("mass_kg", self.errors(L))

    def test_not_a_sphere(self):
        L = _minimal()
        L["object"]["shape"] = "box"
        self.assertIn("spheres only", self.errors(L))

    def test_wrong_schema(self):
        L = _minimal()
        L["schema"] = "something/else"
        self.assertIn("schema", self.errors(L))

    def test_lengths_not_in_metres(self):
        L = _minimal()
        L["frame"] = {"units": "mm"}
        self.assertIn("metres", self.errors(L))

    def test_time_must_increase(self):
        L = _minimal()
        L["frames"][1]["t"] = 0.0
        self.assertIn("does not increase", self.errors(L))

    def test_list_length_must_match_stations(self):
        L = _minimal()
        L["frames"][0]["contact"] = [True]
        self.assertIn("one entry per station", self.errors(L))

    def test_touching_station_needs_a_point(self):
        L = _minimal()
        L["frames"][0]["contact_point"][0] = None
        L["frames"][0]["station_pos"][0] = None
        self.assertIn("neither contact_point nor station_pos", self.errors(L))

    def test_station_pos_fallback_is_a_warning(self):
        L = _minimal()
        L["frames"][0]["contact_point"][0] = None
        self.assertEqual(self.errors(L), "")
        self.assertIn("uses its station_pos instead", self.warnings(L))

    def test_force_where_not_touching_is_a_warning(self):
        L = _minimal()
        L["frames"][0]["contact"][1] = False
        self.assertEqual(self.errors(L), "")
        self.assertIn("ignores that force", self.warnings(L))

    def test_pulling_force_is_a_warning(self):
        L = _minimal()
        L["frames"][0]["force_on_object_N"][0] = [1.0, 0.0, 0.0]
        self.assertIn("pulls on the object", self.warnings(L))

    def test_outward_normal_is_a_warning(self):
        L = _minimal()
        L["frames"][0]["contact_normal"][0] = [1.0, 0.0, 0.0]
        self.assertIn("does not point from the contact", self.warnings(L))

    def test_point_off_the_surface_is_a_warning(self):
        L = _minimal()
        L["frames"][0]["contact_point"][0] = [0.04, 0.0, 0.1]
        self.assertIn("off the sphere's surface", self.warnings(L))

    def test_a_stored_leader_with_its_mass_removed_fails(self):
        L = copy.deepcopy(cl.load(LEADERS[0]))
        L["object"]["mass_kg"] = None
        self.assertIn("mass_kg", self.errors(L))


if __name__ == "__main__":
    unittest.main()
