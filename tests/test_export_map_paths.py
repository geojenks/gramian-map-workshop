"""tools/export_map_paths.py: the records as tip paths for the 3D map page.
All checks run on the real records (data/records and data/draft_records);
only the hand-placed checks use temporary fixture files. Nothing here writes
to outputs/ or to site/. numpy only."""
import contextlib
import copy
import csv
import importlib.util
import io
import json
import os
import shutil
import sys
import tempfile
import unittest

import numpy as np

try:
    import scipy  # noqa: F401
except ImportError:
    raise unittest.SkipTest('the map builder needs scipy: '
                            'pip install -e ".[map]"')

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "demos"))
import gramian_workshop  # noqa: E402
from gramian_workshop.finger import Finger, load_manifest, load_record  # noqa: E402
from gramian_workshop.lookup import free_curve  # noqa: E402
from backwards_lookup import draft_records  # noqa: E402

TANDEM = ("pneunet_tandem_twist_60deg", "pneunet_tandem_twist_90deg")


def _load(name):
    spec = importlib.util.spec_from_file_location(
        name, os.path.join(ROOT, "tools", name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


emp = _load("export_map_paths")
fig = _load("figure_map_tip_paths")
DOC = None


def setUpModule():
    global DOC
    here = os.path.normcase(os.path.abspath(gramian_workshop.__file__))
    assert here.startswith(os.path.normcase(ROOT) + os.sep), (
        f"gramian_workshop imported from {here}, not from {ROOT}")
    DOC = emp.build(drafts=True)


def _all_records():
    """{id: (record, quality, draft)} for every record and draft record."""
    out = {}
    for rid, e in load_manifest()["records"].items():
        out[rid] = (load_record(rid), e.get("quality"), False)
    for path, q in draft_records().items():
        rec = load_record(path)
        out[rec["id"]] = (rec, q, True)
    return out


def _caveats():
    with open(emp.CAVEATS, encoding="utf-8") as fo:
        return json.load(fo)["records"]


def _by_id():
    return {a["id"]: a for a in DOC["actuators"]}


class BriefFacts(unittest.TestCase):
    """The facts the export is built on, checked on the records."""

    def test_six_of_27_records_have_no_free_curve(self):
        m = load_manifest()["records"]
        self.assertEqual(len(m), 27)
        bad = [r for r in m if free_curve(load_record(r))[2] is not None]
        self.assertEqual(len(bad), 6, bad)

    def test_simulated_records_start_at_exactly_zero(self):
        for rid, (rec, _, _) in _all_records().items():
            if (rec.get("source") or {}).get("kind") != "simulation":
                continue
            first = rec["free"]["tip_displacement_m"][0]
            self.assertEqual([float(v) for v in first], [0.0, 0.0, 0.0], rid)

    def test_published_record_does_not_start_at_zero(self):
        first = load_record("mosadegh2014_fPN")["free"]["tip_displacement_m"][0]
        np.testing.assert_allclose(1e3 * np.asarray(first), [-32.5, -50.9, 0.0],
                                   atol=0.1)

    def test_tandem_inputs_are_pairs(self):
        recs = _all_records()
        for rid in TANDEM:
            rec = recs[rid][0]
            self.assertEqual(len(rec["input"]["channels"]), 2)
            for row in rec["free"]["u"]:
                self.assertEqual(len(row), 2, rid)
            self.assertIsNotNone(free_curve(rec)[2], rid)

    # "x is the base-to-tip axis" is stated by the schema and by
    # gramian_workshop/finger.py; the records hold nothing to check it with.


class StoredRows(unittest.TestCase):
    """Test 1: every exported level is a stored tip displacement row."""

    def test_levels_equal_stored_rows(self):
        recs = _all_records()
        for a in DOC["actuators"]:
            rec = recs[a["id"]][0]
            disp = rec["free"]["tip_displacement_m"]
            u = rec["free"]["u"]
            # single-input records store free.u as one-value lists
            multi = len(rec["input"]["channels"]) > 1
            for leg in a["legs"]:
                v = leg["values"]
                got = list(zip(v["axial_m"], v["side_y_m"], v["side_z_m"]))
                if not multi:
                    self.assertEqual(len(a["legs"]), 1, a["id"])
                    self.assertEqual(got, [tuple(r) for r in disp], a["id"])
                    self.assertEqual(list(got[-1]), list(disp[-1]), a["id"])
                    self.assertEqual(leg["input"]["values"],
                                     np.ravel(np.asarray(u, float)).tolist())
                    continue
                c = leg["channel"]
                for k, xyz in enumerate(got):
                    uc = leg["input"]["values"][k]
                    cands = [tuple(disp[i]) for i in range(len(u)) if u[i][c] == uc]
                    self.assertIn(xyz, cands, (a["id"], leg["label"], k))

    def test_single_input_label_is_the_input_name(self):
        recs = _all_records()
        for a in DOC["actuators"]:
            chans = recs[a["id"]][0]["input"]["channels"]
            if len(chans) > 1:
                continue
            self.assertEqual([g["label"] for g in a["legs"]], [chans[0]["name"]],
                             a["id"])
            self.assertNotIn("alone", a["legs"][0]["label"], a["id"])

    def test_bending_is_length_of_sideways_part(self):
        for a in DOC["actuators"]:
            for leg in a["legs"]:
                v = leg["values"]
                np.testing.assert_array_equal(
                    v["bending_m"], np.hypot(v["side_y_m"], v["side_z_m"]))


class FlatFigure(unittest.TestCase):
    """Test 2: the flat figure's tip path (relative to the first level) is
    recovered from the export."""

    def test_paths_agree(self):
        recs = _all_records()
        fig_paths = {}
        for rid, (rec, q, _) in recs.items():
            row, _ = fig.place(Finger(rec, q), rec)
            if row is not None and row["path"] is not None:
                fig_paths[rid] = np.asarray(row["path"])
        exp = {a["id"]: a for a in DOC["actuators"] if not a["hand_placed"]}
        # Expected differences in membership:
        #  - mckibben_v2_70deg: the figure draws it; the export leaves it out
        #    ("draw": false in data/map_caveats.json, settle audit).
        #  - the two tandem twist drafts: the export splits them into legs;
        #    the figure has no path for them, because free_curve rejects a
        #    two-channel free.u (and the figure also needs
        #    scalars.free_stroke_m, which they leave empty; the export does
        #    not need it).
        #  - the joints of the rigid hand fingers drafted from public hand
        #    models: the figure draws them; the export leaves them out
        #    ("draw": false in data/map_caveats.json, because they have no
        #    picture and no reference entry yet).
        cav = _caveats()
        rigid_joints = sorted(
            rid for rid in recs if rid.startswith("shadow_")
            and (cav.get(rid) or {}).get("draw") is False)
        self.assertEqual(len(rigid_joints), 9)
        self.assertEqual(sorted(set(fig_paths) - set(exp)),
                         sorted(["mckibben_v2_70deg"] + rigid_joints))
        self.assertEqual(sorted(set(exp) - set(fig_paths)), sorted(TANDEM))
        for rid in sorted(set(fig_paths) & set(exp)):
            legs = exp[rid]["legs"]
            self.assertEqual(len(legs), 1, rid)
            v = legs[0]["values"]
            x, y, z = (np.asarray(v[k]) for k in ("axial_m", "side_y_m", "side_z_m"))
            mine = np.column_stack([np.hypot(y - y[0], z - z[0]), x - x[0]])
            np.testing.assert_array_equal(mine, fig_paths[rid], err_msg=rid)


class LeftOut(unittest.TestCase):
    """Test 3: what is left out, and why."""

    def test_left_out_list(self):
        cav = _caveats()
        expected = {}
        for rid, (rec, _, _) in _all_records().items():
            if (cav.get(rid) or {}).get("draw") is False:
                expected[rid] = cav[rid]["reason"]
                continue
            why = free_curve(rec)[2]
            if why is not None and rid not in TANDEM:
                expected[rid] = why
        got = {lo["id"]: lo["reason"] for lo in DOC["left_out"]}
        self.assertEqual(got, expected)
        self.assertEqual(len(DOC["left_out"]), len(got))
        self.assertIn("mckibben_v2_70deg", got)
        self.assertEqual(set(_by_id()) & set(got), set())

    def test_caveat_carried(self):
        a = _by_id()["mckibben_v2_20deg"]
        self.assertEqual(a["caveat"], _caveats()["mckibben_v2_20deg"]["caveat"])
        self.assertIsNone(_by_id()["bellows1"]["caveat"])


class DraftSource(unittest.TestCase):
    """A draft's source flag agrees with the record's own source block."""

    def test_source_flag_from_record(self):
        seen = set()
        for a in DOC["actuators"]:
            src = [f for f in a["flags"] if f.startswith("source:")]
            if not a["draft"]:
                self.assertEqual(src, [], a["id"])
                continue
            want = ("draft_from_published_figures"
                    if a["source_kind"] == "literature"
                    else "draft_from_" + a["source_kind"])
            self.assertEqual(src, ["source:" + want], a["id"])
            seen.add(a["source_kind"])
        self.assertEqual(seen, {"literature", "simulation"})


class Twist(unittest.TestCase):
    """Test 4: twist only where the record holds tip rotation; the tandem
    drafts split into exactly three legs."""

    def test_twist_null_without_rotvec(self):
        recs = _all_records()
        for a in DOC["actuators"]:
            held = "tip_rotvec" in (recs[a["id"]][0].get("free") or {})
            for leg in a["legs"]:
                if held:
                    self.assertIsNotNone(leg["values"]["twist_rad"], a["id"])
                else:
                    self.assertIsNone(leg["values"]["twist_rad"], a["id"])

    def test_tandem_legs(self):
        recs = _all_records()
        exp = _by_id()
        sizes = {"pneunet_tandem_twist_60deg": [13, 7, 7],
                 "pneunet_tandem_twist_90deg": [7, 7, 7]}
        for rid in TANDEM:
            rec = recs[rid][0]
            U = np.asarray(rec["free"]["u"], float)
            Y = rec["free"]["tip_displacement_m"]
            R = rec["free"]["tip_rotvec"]
            # (channel, which other channel, its held value), written out
            want = [(0, 1, 0.0), (1, 0, 0.0), (1, 0, 60.0)]
            legs = exp[rid]["legs"]
            self.assertEqual(len(legs), 3, rid)
            self.assertEqual([len(g["input"]["values"]) for g in legs], sizes[rid])
            for leg, (c, o, held) in zip(legs, want):
                rows = sorted(np.flatnonzero(U[:, o] == held), key=lambda i: U[i, c])
                self.assertEqual(leg["channel"], c)
                self.assertEqual(leg["input"]["values"], [U[i, c] for i in rows])
                self.assertEqual(leg["values"]["twist_rad"], [R[i][0] for i in rows])
                self.assertEqual(leg["values"]["axial_m"], [Y[i][0] for i in rows])
                self.assertEqual(leg["values"]["side_y_m"], [Y[i][1] for i in rows])
                self.assertEqual(leg["values"]["side_z_m"], [Y[i][2] for i in rows])
                self.assertFalse(leg["lead_in_from_origin"])
            self.assertEqual(legs[2]["input"]["values"][0], 0.0)
            v2 = legs[2]["values"]
            self.assertTrue(any(v2[k][0] != 0.0
                                for k in ("axial_m", "side_y_m", "side_z_m")))
            self.assertIn("held at 60 Pa", legs[2]["label"])
            self.assertIn("proximal", legs[2]["label"])
            self.assertTrue(legs[2]["label"].startswith("distal"))


class InputShare(unittest.TestCase):
    """Test 5: input share starts at 0 only where the first input is 0,
    ends at 1 and is monotonic, on every leg whose other inputs are at zero.
    On a leg with another input held above zero, the share starts at that
    input's share and does not fall."""

    def test_input_share(self):
        held_legs = []
        for a in DOC["actuators"]:
            for leg in a["legs"]:
                s = np.asarray(leg["values"]["input_share"])
                u0 = leg["input"]["values"][0]
                self.assertEqual(s[-1], 1.0, a["id"])
                self.assertTrue(np.all((s >= 0.0) & (s <= 1.0)),
                                (a["id"], leg["label"]))
                if "held at" in leg["label"]:
                    held_legs.append(a["id"])
                    self.assertTrue(np.all(np.diff(s) >= 0),
                                    (a["id"], leg["label"]))
                    continue
                self.assertEqual(s[0] == 0.0, u0 == 0.0, (a["id"], leg["label"]))
                self.assertTrue(np.all(np.diff(s) > 0), (a["id"], leg["label"]))
        self.assertEqual(sorted(held_legs), sorted(TANDEM))

    def test_held_leg_share(self):
        # proximal chamber held at 60 Pa, largest 120 Pa anywhere in the
        # record: the share is 0.5 until the distal input passes 60 Pa
        recs = _all_records()
        for rid in TANDEM:
            U = np.abs(np.asarray(recs[rid][0]["free"]["u"], float))
            top0, top1 = U[:, 0].max(), U[:, 1].max()
            self.assertEqual((top0, top1), (120.0, 120.0), rid)
            leg = _by_id()[rid]["legs"][2]
            s = leg["values"]["input_share"]
            self.assertEqual(s[0], 0.5, rid)
            want = [max(60.0 / top0, abs(v) / top1)
                    for v in leg["input"]["values"]]
            self.assertEqual(s, want, rid)
            self.assertEqual(s[:4], [0.5] * 4, rid)

    def test_shares_without_held_input_unchanged(self):
        # every leg of every single-input actuator, and the two alone legs of
        # each tandem draft: |input| / largest |input| in the leg, as before
        recs = _all_records()
        for a in DOC["actuators"]:
            multi = len(recs[a["id"]][0]["input"]["channels"]) > 1
            for leg in a["legs"]:
                if "held at" in leg["label"]:
                    continue
                if not multi:
                    self.assertEqual(len(a["legs"]), 1, a["id"])
                u = np.abs(np.asarray(leg["input"]["values"], float))
                self.assertEqual(leg["values"]["input_share"],
                                 [float(v) / u.max() for v in u],
                                 (a["id"], leg["label"]))

    def test_vacuum_records_share_is_size_over_largest_size(self):
        # vacuum inputs are negative: the share is |u| / max |u|, so these
        # end at 1 and never rise above it
        for rid in ("abboodi2026_ossa5_vacuum_contract", "yang2016_vamp_ecoflex",
                    "yang2016_vamp_elastosil"):
            leg = _by_id()[rid]["legs"][0]
            u = np.asarray(leg["input"]["values"])
            self.assertTrue(np.all(u < 0), rid)
            np.testing.assert_array_equal(leg["values"]["input_share"],
                                          np.abs(u) / np.abs(u).max(), rid)

    def test_rows_stored_the_other_way_are_reordered(self):
        # a real record with its free rows reversed in memory (the file is
        # not touched) must give the same leg as the record as stored
        rid = "mosadegh2014_fPN"
        rec = load_record(rid)
        flipped = copy.deepcopy(rec)
        for k in ("u", "tip_displacement_m"):
            flipped["free"][k] = flipped["free"][k][::-1]
        want, why = emp.legs_of(rec)
        self.assertIsNone(why)
        got, why = emp.legs_of(flipped)
        self.assertIsNone(why)
        self.assertEqual(got, want)
        self.assertTrue(got[0]["lead_in_from_origin"])

    def test_lead_in_only_off_origin(self):
        # Single-input records only: there are no other channels, so the
        # lead-in depends on the first point alone. The two-input tandem
        # drafts are checked in Twist.test_tandem_legs (the leg with the
        # proximal chamber held at 60 Pa starts off the origin but has no
        # lead-in, because the other channel is not at zero).
        recs = _all_records()
        for a in DOC["actuators"]:
            if len(recs[a["id"]][0]["input"]["channels"]) > 1:
                continue
            for leg in a["legs"]:
                v = leg["values"]
                first = (v["axial_m"][0], v["side_y_m"][0], v["side_z_m"][0])
                self.assertEqual(leg["lead_in_from_origin"],
                                 any(x != 0.0 for x in first), a["id"])
        self.assertTrue(_by_id()["mosadegh2014_fPN"]["legs"][0]["lead_in_from_origin"])
        self.assertFalse(_by_id()["bellows1"]["legs"][0]["lead_in_from_origin"])


class HandPlaced(unittest.TestCase):
    """Test 6: hand-placed fixtures round trip; one without a basis, or
    naming a record file, is refused. The real file stays empty."""

    ENTRY = {
        "id": "fixture_hand_placed", "name": "Fixture, hand placed",
        "physics": "pneumatic", "source_kind": "literature", "draft": False,
        "has_force": False, "flags": [], "caveat": None,
        "basis": "test fixture only",
        "legs": [{"label": "chamber pressure", "channel": 0,
                  "lead_in_from_origin": False,
                  "input": {"name": "chamber pressure", "unit": "Pa",
                            "values": [0.0, 1.0, 2.0]},
                  "values": {"axial_m": [0.0, 0.001, 0.002],
                             "bending_m": [0.0, 0.0, 0.0],
                             "twist_rad": None,
                             "side_y_m": [0.0, 0.0, 0.0],
                             "side_z_m": [0.0, 0.0, 0.0],
                             "input_share": [0.0, 0.5, 1.0]}}],
        "surface": None}

    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def _fixture(self, entry):
        p = os.path.join(self.tmp, "hand.json")
        with open(p, "w", encoding="utf-8") as fo:
            json.dump({"about": "fixture", "actuators": [entry]}, fo)
        return p

    def test_real_file_is_empty(self):
        with open(emp.HAND_PLACED, encoding="utf-8") as fo:
            self.assertEqual(json.load(fo)["actuators"], [])
        self.assertFalse(any(a["hand_placed"] for a in DOC["actuators"]))

    def test_round_trip(self):
        # the entry comes back as written, plus the keys that need a body
        # length, all null: nothing is computed for a hand-placed entry
        doc = emp.build(drafts=True, hand_placed=self._fixture(self.ENTRY))
        got = json.loads(emp.render(doc))["actuators"][-1]
        leg = self.ENTRY["legs"][0]
        want_leg = {**leg, "values": {**leg["values"],
                                      **dict.fromkeys(emp.LENGTH_KEYS)},
                    "length_sensitivity": None}
        self.assertEqual(got, {**self.ENTRY, "hand_placed": True,
                               "legs": [want_leg],
                               "length": {"value_m": None, "source": None,
                                          "detail": None},
                               "bend_check": None,
                               "tasks": None, "force": None,
                               "force_withheld": None,
                               # no name, reference or picture either: a
                               # hand-placed entry keeps the plain marker
                               **dict.fromkeys(emp.SHOWN_KEYS)})
        self.assertNotIn("record_file", got)
        self.assertEqual(doc["actuators"][:-1], DOC["actuators"])

    def test_supplied_length_values_kept(self):
        # values the file supplies for the new keys are carried as they are
        leg = copy.deepcopy(self.ENTRY["legs"][0])
        leg["values"]["axial_share"] = [0.0, 0.01, 0.02]
        doc = emp.build(drafts=True, hand_placed=self._fixture(
            {**self.ENTRY, "legs": [leg]}))
        got = doc["actuators"][-1]["legs"][0]["values"]
        self.assertEqual(got["axial_share"], [0.0, 0.01, 0.02])
        self.assertIsNone(got["bend_rad"])
        bad = copy.deepcopy(leg)
        bad["values"]["bend_rad"] = [0.0]          # wrong length
        with self.assertRaises(ValueError):
            emp.build(drafts=True, hand_placed=self._fixture(
                {**self.ENTRY, "legs": [bad]}))
        bad = copy.deepcopy(leg)
        bad["values"]["made_up"] = [0.0, 0.0, 0.0]  # unknown key
        with self.assertRaises(ValueError):
            emp.build(drafts=True, hand_placed=self._fixture(
                {**self.ENTRY, "legs": [bad]}))

    def test_length_keys_must_be_null(self):
        # a hand-placed entry has no body length, so it may not give one,
        # nor a bend check or a length sensitivity
        ok = {**self.ENTRY, "length": {"value_m": None, "source": None,
                                       "detail": None}, "bend_check": None}
        emp.build(drafts=True, hand_placed=self._fixture(ok))
        bad_leg = {**self.ENTRY["legs"][0],
                   "length_sensitivity": {"bend_rad": [0.0, 0.0, 0.0],
                                          "stretch_share": [0.0, 0.0, 0.0]}}
        bad = {"length": {**self.ENTRY, "length": {"value_m": 0.05,
                                                    "source": "record",
                                                    "detail": None}},
               "bend_check": {**self.ENTRY, "bend_check": []},
               "length_sensitivity": {**self.ENTRY, "legs": [bad_leg]}}
        for what, entry in bad.items():
            with self.subTest(what=what):
                with self.assertRaises(ValueError):
                    emp.build(drafts=True, hand_placed=self._fixture(entry))

    def test_tasks_and_force_must_be_null(self):
        # both are computed from a record, so a hand-placed entry has none
        emp.build(drafts=True, hand_placed=self._fixture(
            {**self.ENTRY, "tasks": None, "force": None}))
        for k, v in (("tasks", []), ("force", {"axial_N": [1.0]})):
            with self.subTest(key=k):
                with self.assertRaises(ValueError):
                    emp.build(drafts=True, hand_placed=self._fixture(
                        {**self.ENTRY, k: v}))

    def test_missing_basis_refused(self):
        bad = {k: v for k, v in self.ENTRY.items() if k != "basis"}
        with self.assertRaises(ValueError):
            emp.build(drafts=True, hand_placed=self._fixture(bad))

    def test_record_file_refused(self):
        bad = {**self.ENTRY, "record_file": "data/records/bellows1.json"}
        with self.assertRaises(ValueError):
            emp.build(drafts=True, hand_placed=self._fixture(bad))

    def test_surface_refused(self):
        # a surface is predicted from a record's rows; hand-placed entries
        # carry "surface": null, and one without the key is refused too
        bad = {**self.ENTRY, "surface": {"basis": "made up"}}
        with self.assertRaises(ValueError):
            emp.build(drafts=True, hand_placed=self._fixture(bad))
        missing = {k: v for k, v in self.ENTRY.items() if k != "surface"}
        with self.assertRaises(ValueError):
            emp.build(drafts=True, hand_placed=self._fixture(missing))


GRID_KEYS = ("axial_m", "side_y_m", "side_z_m", "bending_m", "twist_rad",
             "input_share", "bend_rad", "stretch_share", "bend_y_rad",
             "bend_z_rad", "axial_share", "bending_share")


def _row(U, p0, p1):
    """The one free row with inputs (p0, p1)."""
    hits = [i for i in range(len(U)) if U[i][0] == p0 and U[i][1] == p1]
    assert len(hits) == 1, (p0, p1, hits)
    return hits[0]


# declarations used in memory only, beside the real sidecar file
SUM = {"rule": "sum", "why": "test only"}
SERIES = {"rule": "series", "base_to_tip": [0, 1], "why": "test only"}
XYZ = ("axial_m", "side_y_m", "side_z_m")
SURFACE_ORDER = ["rule", "basis", "inputs", "values", "tests", "tests_note",
                 "worst_position_gap_share", "worst_twist_gap_share",
                 "worst_sum_position_gap_share", "worst_sum_twist_gap_share",
                 "worst_gap_share_by_key", "extra_checks"]
TEST_ORDER = ["inputs", "interpolated", "measured", "predicted",
              "position_gap_m", "position_gap_share", "twist_gap_rad",
              "predicted_sum", "sum_position_gap_m", "sum_position_gap_share",
              "sum_twist_gap_rad"]


def _rotm(rv):
    """Rotation matrix of a rotation vector (Rodrigues' formula)."""
    rv = np.asarray(rv, float)
    th = np.linalg.norm(rv)
    if th == 0.0:
        return np.eye(3)
    k = rv / th
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * K @ K


def _rotvec_of(M):
    """Rotation vector of a rotation matrix (angles below pi)."""
    th = np.arccos(np.clip((np.trace(M) - 1.0) / 2.0, -1.0, 1.0))
    if th == 0.0:
        return np.zeros(3)
    return th / (2.0 * np.sin(th)) * np.array(
        [M[2, 1] - M[1, 2], M[0, 2] - M[2, 0], M[1, 0] - M[0, 1]])


def _series(dA, rA, dB, rB):
    """The far segment (dB, rB) carried on the near segment's tip (dA, rA):
    displacement dA + R_A dB, rotation R_A R_B, recomputed here with numpy."""
    RA = _rotm(rA)
    return (np.asarray(dA, float) + RA @ np.asarray(dB, float),
            _rotvec_of(RA @ _rotm(rB)))


class Surface(unittest.TestCase):
    """The computed surface for two-input actuators and its test points,
    each checked against the record rows: the declared series rule, and the
    plain sum kept beside it for comparison."""

    def test_surface_only_for_tandem(self):
        got = sorted(a["id"] for a in DOC["actuators"] if a["surface"] is not None)
        self.assertEqual(got, sorted(TANDEM))
        for a in DOC["actuators"]:
            self.assertIn("surface", a)
            self.assertEqual(list(a), emp.ACTUATOR_KEYS, a["id"])
        for rid in TANDEM:
            s = _by_id()[rid]["surface"]
            self.assertEqual(list(s), SURFACE_ORDER)
            self.assertEqual(s["rule"], "series")
            for t in s["tests"]:
                self.assertEqual(list(t), TEST_ORDER)

    def test_real_sidecar(self):
        # both tandem drafts declared series, proximal (channel 0) first
        with open(emp.TWO_INPUT_RULES, "rb") as fo:
            raw = fo.read()
        self.assertNotIn(b"\r", raw)
        recs = json.loads(raw.decode("utf-8"))["records"]
        self.assertEqual(sorted(recs), sorted(TANDEM))
        for rid in TANDEM:
            self.assertEqual(recs[rid]["rule"], "series")
            self.assertEqual(recs[rid]["base_to_tip"], [0, 1])
            self.assertIn("proximal", _all_records()[rid][0]["input"]
                          ["channels"][0]["name"])
            self.assertTrue(recs[rid]["why"].strip())

    def test_basis_names_near_and_far(self):
        for rid in TANDEM:
            s = _by_id()[rid]["surface"]
            self.assertEqual(s["basis"], emp.SERIES_BASIS.format(
                near="proximal chamber pressure (p1)",
                far="distal chamber pressure (p2)"))
        rec = _all_records()[TANDEM[0]][0]
        self.assertEqual(emp.surface_of(rec, SUM)["basis"], emp.SUM_BASIS)

    def test_grid_shapes_and_inputs(self):
        recs = _all_records()
        sizes = {"pneunet_tandem_twist_60deg": (13, 7),
                 "pneunet_tandem_twist_90deg": (7, 7)}
        for rid in TANDEM:
            rec = recs[rid][0]
            s = _by_id()[rid]["surface"]
            n0, n1 = sizes[rid]
            self.assertEqual([len(x["values"]) for x in s["inputs"]], [n0, n1])
            for c in (0, 1):
                self.assertEqual(s["inputs"][c]["name"],
                                 rec["input"]["channels"][c]["name"])
                self.assertEqual(s["inputs"][c]["unit"],
                                 rec["input"]["channels"][c]["unit"])
                vals = s["inputs"][c]["values"]
                self.assertEqual(vals[0], 0.0)
                self.assertTrue(np.all(np.diff(np.abs(vals)) > 0), rid)
            self.assertEqual(list(s["values"]), list(GRID_KEYS))
            for k in GRID_KEYS:
                self.assertEqual(np.asarray(s["values"][k]).shape, (n0, n1),
                                 (rid, k))

    def test_edges_equal_alone_legs(self):
        for rid in TANDEM:
            a = _by_id()[rid]
            s = a["surface"]
            leg0, leg1 = a["legs"][0], a["legs"][1]
            self.assertIn("alone", leg0["label"])
            self.assertIn("alone", leg1["label"])
            self.assertEqual(s["inputs"][0]["values"], leg0["input"]["values"])
            self.assertEqual(s["inputs"][1]["values"], leg1["input"]["values"])
            for k in GRID_KEYS:
                g = s["values"][k]
                self.assertEqual([r[0] for r in g], leg0["values"][k], (rid, k))
                self.assertEqual(g[0], leg1["values"][k], (rid, k))

    def test_series_grid_from_record_rows(self):
        recs = _all_records()
        for rid in TANDEM:
            free = recs[rid][0]["free"]
            U, Y, R = free["u"], free["tip_displacement_m"], free["tip_rotvec"]
            s = _by_id()[rid]["surface"]
            v = s["values"]
            for i, p0 in enumerate(s["inputs"][0]["values"]):
                for j, p1 in enumerate(s["inputs"][1]["values"]):
                    a, b = _row(U, p0, 0.0), _row(U, 0.0, p1)
                    xyz, rv = _series(Y[a], R[a], Y[b], R[b])
                    got = [v[k][i][j] for k in XYZ]
                    np.testing.assert_allclose(got, xyz, rtol=0, atol=1e-12)
                    self.assertAlmostEqual(v["bending_m"][i][j],
                                           float(np.hypot(got[1], got[2])),
                                           delta=1e-15)
                    self.assertAlmostEqual(v["twist_rad"][i][j], rv[0],
                                           delta=1e-9)
                    self.assertEqual(v["input_share"][i][j],
                                     max(abs(p0), abs(p1)) / 120.0)

    def test_grid_is_sum_of_record_rows(self):
        # the plain sum, from a "sum" declaration made in memory
        recs = _all_records()
        for rid in TANDEM:
            free = recs[rid][0]["free"]
            U, Y, R = free["u"], free["tip_displacement_m"], free["tip_rotvec"]
            s = emp.surface_of(recs[rid][0], SUM)
            v = s["values"]
            for i, p0 in enumerate(s["inputs"][0]["values"]):
                for j, p1 in enumerate(s["inputs"][1]["values"]):
                    a, b = _row(U, p0, 0.0), _row(U, 0.0, p1)
                    xyz = [Y[a][k] + Y[b][k] for k in range(3)]
                    self.assertEqual(v["axial_m"][i][j], xyz[0])
                    self.assertEqual(v["side_y_m"][i][j], xyz[1])
                    self.assertEqual(v["side_z_m"][i][j], xyz[2])
                    self.assertAlmostEqual(v["bending_m"][i][j],
                                           (xyz[1] ** 2 + xyz[2] ** 2) ** 0.5,
                                           delta=1e-15)
                    self.assertEqual(v["twist_rad"][i][j], R[a][0] + R[b][0])
                    self.assertEqual(v["input_share"][i][j],
                                     max(abs(p0), abs(p1)) / 120.0)

    def test_six_test_points(self):
        recs = _all_records()
        for rid in TANDEM:
            U = recs[rid][0]["free"]["u"]
            both = [list(map(float, r)) for r in U if r[0] != 0.0 and r[1] != 0.0]
            tests = _by_id()[rid]["surface"]["tests"]
            self.assertEqual(len(tests), 6, rid)
            self.assertEqual([t["inputs"] for t in tests], both)
            self.assertEqual(both, [[60.0, p] for p in (20.0, 40.0, 60.0,
                                                        80.0, 100.0, 120.0)])
            for t in tests:
                self.assertIs(t["interpolated"], False)

    def test_series_test_points_from_record_rows(self):
        # predicted is the distal row carried on the proximal row's tip
        recs = _all_records()
        for rid in TANDEM:
            free = recs[rid][0]["free"]
            U, Y, R = free["u"], free["tip_displacement_m"], free["tip_rotvec"]
            s = _by_id()[rid]["surface"]
            shares, twist_gaps, twists = [], [], []
            for t in s["tests"]:
                p0, p1 = t["inputs"]
                r, a, b = _row(U, p0, p1), _row(U, p0, 0.0), _row(U, 0.0, p1)
                xyz, rv = _series(Y[a], R[a], Y[b], R[b])
                p = t["predicted"]
                np.testing.assert_allclose([p[k] for k in XYZ], xyz,
                                           rtol=0, atol=1e-12)
                self.assertAlmostEqual(p["twist_rad"], rv[0], delta=1e-9)
                self.assertAlmostEqual(p["bending_m"],
                                       float(np.hypot(p["side_y_m"],
                                                      p["side_z_m"])),
                                       delta=1e-15)
                self.assertEqual(p["input_share"],
                                 max(abs(p0), abs(p1)) / 120.0)
                gap = float(np.linalg.norm(np.asarray(Y[r]) - xyz))
                length = float(np.linalg.norm(Y[r]))
                tg = abs(R[r][0] - rv[0])
                self.assertAlmostEqual(t["position_gap_m"], gap, delta=1e-12)
                self.assertAlmostEqual(t["position_gap_share"], gap / length,
                                       delta=1e-10)
                self.assertAlmostEqual(t["twist_gap_rad"], tg, delta=1e-9)
                shares.append(t["position_gap_share"])
                twist_gaps.append(t["twist_gap_rad"])
                twists.append(abs(R[r][0]))
            # worst_twist_gap_share: the largest twist gap over the tests,
            # divided by the largest |measured twist| over the tests
            self.assertEqual(s["worst_position_gap_share"], max(shares))
            self.assertEqual(s["worst_twist_gap_share"],
                             max(twist_gaps) / max(twists))

    def test_series_numbers_at_highest_inputs(self):
        # the test point at (60, 120) Pa under the series rule: each
        # draft's position gap and predicted twist checked on its own, so
        # one failing does not hide the others
        want_gap = {"pneunet_tandem_twist_60deg": 0.0175e-3,
                    "pneunet_tandem_twist_90deg": 0.0348e-3}
        want_twist = {"pneunet_tandem_twist_60deg": 0.0384,
                      "pneunet_tandem_twist_90deg": 0.0765}
        for rid in TANDEM:
            t = _by_id()[rid]["surface"]["tests"][-1]
            self.assertEqual(t["inputs"], [60.0, 120.0])
            with self.subTest(rid=rid, check="position gap"):
                self.assertAlmostEqual(t["position_gap_m"], want_gap[rid],
                                       delta=0.0005e-3)
            with self.subTest(rid=rid, check="predicted twist"):
                self.assertAlmostEqual(t["predicted"]["twist_rad"],
                                       want_twist[rid], delta=0.0005)

    def test_test_points_from_record_rows(self):
        # measured is the row itself; predicted_sum is the sum of the two
        # single-input rows, recomputed here from the record
        recs = _all_records()
        for rid in TANDEM:
            free = recs[rid][0]["free"]
            U, Y, R = free["u"], free["tip_displacement_m"], free["tip_rotvec"]
            for t in _by_id()[rid]["surface"]["tests"]:
                p0, p1 = t["inputs"]
                r, a, b = _row(U, p0, p1), _row(U, p0, 0.0), _row(U, 0.0, p1)
                m, p = t["measured"], t["predicted_sum"]
                self.assertEqual(list(m), list(GRID_KEYS))
                self.assertEqual(list(p), list(GRID_KEYS))
                self.assertEqual([m["axial_m"], m["side_y_m"], m["side_z_m"]],
                                 Y[r])
                self.assertEqual(m["twist_rad"], R[r][0])
                want = [Y[a][k] + Y[b][k] for k in range(3)]
                self.assertEqual([p["axial_m"], p["side_y_m"], p["side_z_m"]],
                                 want)
                self.assertEqual(p["twist_rad"], R[a][0] + R[b][0])
                for q, xyz in ((m, Y[r]), (p, want)):
                    self.assertAlmostEqual(q["bending_m"],
                                           (xyz[1] ** 2 + xyz[2] ** 2) ** 0.5,
                                           delta=1e-15)
                share = max(abs(p0), abs(p1)) / 120.0
                self.assertEqual(m["input_share"], share)
                self.assertEqual(p["input_share"], share)

    def test_gaps_recomputed(self):
        recs = _all_records()
        for rid in TANDEM:
            free = recs[rid][0]["free"]
            U, Y, R = free["u"], free["tip_displacement_m"], free["tip_rotvec"]
            s = _by_id()[rid]["surface"]
            shares, twist_gaps, twists = [], [], []
            for t in s["tests"]:
                p0, p1 = t["inputs"]
                r, a, b = _row(U, p0, p1), _row(U, p0, 0.0), _row(U, 0.0, p1)
                d = [Y[r][k] - (Y[a][k] + Y[b][k]) for k in range(3)]
                gap = sum(x * x for x in d) ** 0.5
                length = sum(x * x for x in Y[r]) ** 0.5
                tg = abs(R[r][0] - (R[a][0] + R[b][0]))
                self.assertAlmostEqual(t["sum_position_gap_m"], gap, delta=1e-15)
                self.assertAlmostEqual(t["sum_position_gap_share"], gap / length,
                                       delta=1e-12)
                self.assertAlmostEqual(t["sum_twist_gap_rad"], tg, delta=1e-15)
                shares.append(gap / length)
                twist_gaps.append(tg)
                twists.append(abs(R[r][0]))
            self.assertAlmostEqual(s["worst_sum_position_gap_share"],
                                   max(shares), delta=1e-12)
            self.assertAlmostEqual(s["worst_sum_twist_gap_share"],
                                   max(twist_gaps) / max(twists), delta=1e-12)

    def test_numbers_at_highest_inputs(self):
        # the plain sum at the test point at (60, 120) Pa, in mm and
        # degrees, as computed independently from the record rows before
        # the export was written
        want = {"pneunet_tandem_twist_60deg": (1.42, 0.07, 2.50, -1.24),
                "pneunet_tandem_twist_90deg": (0.62, 0.04, 4.41, 0.07)}
        for rid, (gap, share, tm, tp) in want.items():
            t = _by_id()[rid]["surface"]["tests"][-1]
            self.assertEqual(t["inputs"], [60.0, 120.0])
            self.assertAlmostEqual(1e3 * t["sum_position_gap_m"], gap, delta=0.01)
            self.assertAlmostEqual(t["sum_position_gap_share"], share, delta=0.01)
            self.assertAlmostEqual(np.degrees(t["measured"]["twist_rad"]), tm,
                                   delta=0.01)
            self.assertAlmostEqual(np.degrees(t["predicted_sum"]["twist_rad"]),
                                   tp, delta=0.01)
        t = _by_id()["pneunet_tandem_twist_60deg"]["surface"]["tests"][-1]
        np.testing.assert_allclose(
            [1e3 * t["measured"][k] for k in ("axial_m", "side_y_m", "side_z_m")],
            [1.48, 17.48, 9.24], atol=0.01)
        np.testing.assert_allclose(
            [1e3 * t["predicted_sum"][k] for k in ("axial_m", "side_y_m", "side_z_m")],
            [2.83, 17.04, 9.28], atol=0.01)

    def test_sum_declaration_reproduces_old_surface(self):
        # a "sum" declaration gives the plain-sum surface: its predictions,
        # gaps and worst values are the series surface's sum comparison,
        # and it carries no comparison of its own
        recs = _all_records()
        for rid in TANDEM:
            new = _by_id()[rid]["surface"]
            old = emp.surface_of(recs[rid][0], SUM)
            self.assertEqual(list(old), SURFACE_ORDER)
            self.assertEqual(old["rule"], "sum")
            self.assertEqual(old["inputs"], new["inputs"])
            self.assertEqual(old["tests_note"], new["tests_note"])
            self.assertEqual(len(old["tests"]), len(new["tests"]))
            for o, n in zip(old["tests"], new["tests"]):
                self.assertEqual(list(o), TEST_ORDER)
                self.assertEqual(o["inputs"], n["inputs"])
                self.assertEqual(o["measured"], n["measured"])
                self.assertEqual(o["predicted"], n["predicted_sum"])
                self.assertEqual(o["position_gap_m"], n["sum_position_gap_m"])
                self.assertEqual(o["position_gap_share"],
                                 n["sum_position_gap_share"])
                self.assertEqual(o["twist_gap_rad"], n["sum_twist_gap_rad"])
                for k in ("predicted_sum", "sum_position_gap_m",
                          "sum_position_gap_share", "sum_twist_gap_rad"):
                    self.assertIsNone(o[k])
            self.assertEqual(old["worst_position_gap_share"],
                             new["worst_sum_position_gap_share"])
            self.assertEqual(old["worst_twist_gap_share"],
                             new["worst_sum_twist_gap_share"])
            self.assertIsNone(old["worst_sum_position_gap_share"])
            self.assertIsNone(old["worst_sum_twist_gap_share"])

    def _without_rows(self, rid, drop):
        """A copy of the draft record, in memory only, with the free rows
        whose inputs are in `drop` removed."""
        rec = copy.deepcopy(_all_records()[rid][0])
        free = rec["free"]
        keep = [i for i, r in enumerate(free["u"]) if list(r) not in drop]
        for k in ("u", "tip_displacement_m", "tip_rotvec"):
            free[k] = [free[k][i] for i in keep]
        return rec

    def test_interpolated_when_input_is_not_a_level(self):
        # 60 Pa removed from the proximal chamber's single-input leg: the
        # six test points at 60 Pa use the midpoint of the 50 and 70 Pa rows
        rid = "pneunet_tandem_twist_60deg"
        rec = self._without_rows(rid, [[60.0, 0.0]])
        s = emp.surface_of(rec, SERIES)
        free = rec["free"]
        U, Y, R = free["u"], free["tip_displacement_m"], free["tip_rotvec"]
        lo, hi = _row(U, 50.0, 0.0), _row(U, 70.0, 0.0)
        self.assertEqual(len(s["tests"]), 6)
        self.assertNotIn(60.0, s["inputs"][0]["values"])
        for t in s["tests"]:
            self.assertIs(t["interpolated"], True)
            b = _row(U, 0.0, t["inputs"][1])
            want = [(Y[lo][k] + Y[hi][k]) / 2 + Y[b][k] for k in range(3)]
            got = [t["predicted_sum"][k] for k in ("axial_m", "side_y_m", "side_z_m")]
            np.testing.assert_allclose(got, want, rtol=0, atol=1e-15)
            self.assertAlmostEqual(t["predicted_sum"]["twist_rad"],
                                   (R[lo][0] + R[hi][0]) / 2 + R[b][0],
                                   delta=1e-15)
            # series: the proximal displacement and rotation vector are each
            # taken halfway between the 50 and 70 Pa rows, then composed
            dA = [(Y[lo][k] + Y[hi][k]) / 2 for k in range(3)]
            rA = [(R[lo][k] + R[hi][k]) / 2 for k in range(3)]
            xyz, rv = _series(dA, rA, Y[b], R[b])
            np.testing.assert_allclose([t["predicted"][k] for k in XYZ], xyz,
                                       rtol=0, atol=1e-12)
            self.assertAlmostEqual(t["predicted"]["twist_rad"], rv[0],
                                   delta=1e-9)

    def test_no_rotation_gives_null_twist(self):
        # a copy with free.tip_rotvec removed, in memory only, declared sum:
        # twist is null everywhere, never zero, and the result still renders
        rec = copy.deepcopy(_all_records()["pneunet_tandem_twist_60deg"][0])
        del rec["free"]["tip_rotvec"]
        s = emp.surface_of(rec, SUM)
        self.assertIsNone(s["values"]["twist_rad"])
        self.assertEqual(len(s["tests"]), 6)
        for t in s["tests"]:
            self.assertIsNone(t["measured"]["twist_rad"])
            self.assertIsNone(t["predicted"]["twist_rad"])
            self.assertIsNone(t["twist_gap_rad"])
        self.assertIsNone(s["worst_twist_gap_share"])
        self.assertIsNotNone(s["worst_position_gap_share"])
        emp.render({"surface": s})

    def test_series_without_rotation_refused(self):
        # the series rule needs the near segment's tip rotation
        rec = copy.deepcopy(_all_records()["pneunet_tandem_twist_60deg"][0])
        del rec["free"]["tip_rotvec"]
        with self.assertRaisesRegex(ValueError, "series.*tip rotation"):
            emp.surface_of(rec, SERIES)

    def test_no_combined_rows_gives_no_tests(self):
        rid = "pneunet_tandem_twist_90deg"
        both = [[60.0, p] for p in (20.0, 40.0, 60.0, 80.0, 100.0, 120.0)]
        s = emp.surface_of(self._without_rows(rid, both), SERIES)
        self.assertEqual(s["tests"], [])
        self.assertIsNone(s["tests_note"])
        self.assertIsNone(s["worst_position_gap_share"])
        self.assertIsNone(s["worst_twist_gap_share"])
        self.assertIsNone(s["worst_sum_position_gap_share"])
        self.assertIsNone(s["worst_sum_twist_gap_share"])
        self.assertEqual(s["worst_gap_share_by_key"],
                         dict.fromkeys(emp.ESTIMATE_KEYS))
        self.assertEqual(np.asarray(s["values"]["axial_m"]).shape, (7, 7))
        emp.render({"surface": s})

    def test_input_outside_leg_is_refused(self):
        # 70 Pa and above removed from the proximal leg: 60 Pa still lies
        # inside it; 50 Pa and above removed: 60 Pa lies outside, refused
        rid = "pneunet_tandem_twist_60deg"
        above = [[p, 0.0] for p in (70.0, 80.0, 90.0, 100.0, 110.0, 120.0)]
        self.assertIsNotNone(emp.surface_of(self._without_rows(rid, above),
                                            SERIES))
        with self.assertRaises(ValueError):
            emp.surface_of(self._without_rows(rid, above + [[60.0, 0.0],
                                                            [50.0, 0.0]]),
                           SERIES)

    def test_tests_note(self):
        for rid in TANDEM:
            self.assertEqual(_by_id()[rid]["surface"]["tests_note"],
                             "All six have the proximal chamber pressure (p1) "
                             "held at 60 Pa.")
        rid = "pneunet_tandem_twist_60deg"
        # one test point left
        drop = [[60.0, p] for p in (20.0, 40.0, 60.0, 80.0, 100.0)]
        s = emp.surface_of(self._without_rows(rid, drop), SERIES)
        self.assertEqual(s["tests_note"],
                         "The one test point has the proximal chamber "
                         "pressure (p1) at 60 Pa and the distal chamber "
                         "pressure (p2) at 120 Pa.")
        # one test row's proximal input relabelled 50 Pa, in memory only:
        # no input is common to all six
        rec = copy.deepcopy(_all_records()[rid][0])
        rec["free"]["u"][_row(rec["free"]["u"], 60.0, 20.0)] = [50.0, 20.0]
        s = emp.surface_of(rec, SERIES)
        self.assertEqual(s["tests_note"],
                         "The six test points are spread over both inputs.")


class TwoInputRules(unittest.TestCase):
    """data/map_two_input_rules.json: a missing declaration gives no
    surface; malformed declarations are refused. Fixtures are temporary
    files; the real file is never written."""

    GOOD = {"rule": "series", "base_to_tip": [0, 1], "why": "fixture"}

    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def _fixture(self, records):
        p = os.path.join(self.tmp, "rules.json")
        with open(p, "w", encoding="utf-8") as fo:
            json.dump({"about": "fixture", "records": records}, fo)
        return p

    def test_no_declaration_gives_null(self):
        rec = _all_records()[TANDEM[0]][0]
        self.assertIsNone(emp.surface_of(rec, None))
        doc = emp.build(drafts=True, rules=self._fixture({}))
        self.assertTrue(all(a["surface"] is None for a in doc["actuators"]))
        # everything else is unchanged, bar the second check, which then
        # has only the legs to read tip positions from
        def less(a):
            tasks = a["tasks"] and [{k: v for k, v in t.items()
                                     if k != "return_clear"} for t in a["tasks"]]
            return {**a, "surface": None, "tasks": tasks}
        self.assertEqual([less(a) for a in doc["actuators"]],
                         [less(a) for a in DOC["actuators"]])
        by = {a["id"]: a for a in doc["actuators"]}
        for rid in TANDEM:
            self.assertEqual(
                _task_entry(by[rid])["return_clear"]["positions_read_from"],
                "its legs")

    def test_build_without_drafts(self):
        # the declarations name drafts; a build without drafts still checks
        # them and has no surface
        doc = emp.build(drafts=False)
        self.assertFalse(any(a["id"] in TANDEM for a in doc["actuators"]))
        self.assertTrue(all(a["surface"] is None for a in doc["actuators"]))
        with self.assertRaises(ValueError):
            emp.build(drafts=False, rules=self._fixture(
                {"no_such_record": self.GOOD}))

    def test_declared_rules_match_the_real_file(self):
        doc = emp.build(drafts=True, rules=self._fixture(
            {rid: self.GOOD for rid in TANDEM}))
        self.assertEqual(doc["actuators"], DOC["actuators"])

    def test_bad_entries_refused(self):
        rid = TANDEM[0]
        g = self.GOOD
        bad = {
            "unknown id": {"no_such_record": g},
            "unknown rule": {rid: {**g, "rule": "product"}},
            "no rule": {rid: {k: v for k, v in g.items() if k != "rule"}},
            "same index twice": {rid: {**g, "base_to_tip": [0, 0]}},
            "index out of range": {rid: {**g, "base_to_tip": [0, 2]}},
            "one index": {rid: {**g, "base_to_tip": [1]}},
            "three indices": {rid: {**g, "base_to_tip": [0, 1, 0]}},
            "indices as true/false": {rid: {**g, "base_to_tip": [False, True]}},
            "indices as text": {rid: {**g, "base_to_tip": ["0", "1"]}},
            "series with no order": {rid: {"rule": "series", "why": "x"}},
            "sum with an order": {rid: {"rule": "sum", "base_to_tip": [0, 1],
                                        "why": "x"}},
            "no why": {rid: {"rule": "series", "base_to_tip": [0, 1]}},
            "empty why": {rid: {**g, "why": "  "}},
            "extra key": {rid: {**g, "fits_better": True}},
            "not an object": {rid: "series"},
        }
        for what, records in bad.items():
            with self.subTest(what=what):
                with self.assertRaises(ValueError):
                    emp.build(drafts=True, rules=self._fixture(records))
        # the reverse order is accepted
        emp.load_two_input_rules(
            self._fixture({rid: {**g, "base_to_tip": [1, 0]}}), {rid})

    def test_declaration_on_one_input_record_refused(self):
        one = next(a["id"] for a in DOC["actuators"] if len(a["legs"]) == 1)
        with self.assertRaisesRegex(ValueError, "two input channels"):
            emp.build(drafts=True, rules=self._fixture({one: self.GOOD}))
        with self.assertRaisesRegex(ValueError, "two input channels"):
            emp.build(drafts=True, rules=self._fixture(
                {one: {"rule": "sum", "why": "fixture"}}))


NO_LENGTH_IDS = ["abboodi2026_ossa5_axial_extend",
                 "abboodi2026_ossa5_vacuum_contract",
                 "chandler2020_pha_a0_C1", "chandler2020_pha_a0_C2",
                 "chandler2020_pha_a0_C3", "mosadegh2014_fPN",
                 "mosadegh2014_sPN"]
# The lengths data/map_lengths.json gives, in metres. The two Mosadegh records
# are left without one on purpose (the file's note says why).
ESTIMATED_LENGTHS = {"abboodi2026_ossa5_axial_extend": 0.12,
                     "abboodi2026_ossa5_vacuum_contract": 0.12,
                     "chandler2020_pha_a0_C1": 0.0499,
                     "chandler2020_pha_a0_C2": 0.05,
                     "chandler2020_pha_a0_C3": 0.0483}
STILL_NO_LENGTH_IDS = ["mosadegh2014_fPN", "mosadegh2014_sPN"]


def _real_lengths():
    with open(emp.LENGTHS, encoding="utf-8") as fo:
        return json.load(fo)["records"]


def _arc_tip(L, t, stretch=0.0, toward=(1.0, 0.0)):
    """Tip displacement from rest (L, 0, 0) of an even arc of length
    L (1 + stretch) bent by t radians, bending toward the unit direction
    `toward` in the y-z plane."""
    r = L * (1.0 + stretch) / t
    side = r * (1.0 - np.cos(t))
    return [r * np.sin(t) - L, side * toward[0], side * toward[1]]


class BodyShape(unittest.TestCase):
    """The quantities that need the body length, on made-up tips whose
    answer is known: an even arc, a stretched arc, a pure extender."""

    L = 0.073

    def test_even_arc_comes_back(self):
        for deg in (30.0, 90.0, 170.0, 200.0):
            t = np.radians(deg)
            for toward in ((1.0, 0.0), (0.0, 1.0), (0.6, -0.8)):
                with self.subTest(deg=deg, toward=toward):
                    s = emp.body_shape(_arc_tip(self.L, t, 0.0, toward), self.L)
                    self.assertAlmostEqual(s["bend_rad"], t, delta=1e-12)
                    self.assertAlmostEqual(s["stretch_share"], 0.0, delta=1e-12)
                    self.assertAlmostEqual(s["bend_y_rad"], t * toward[0],
                                           delta=1e-12)
                    self.assertAlmostEqual(s["bend_z_rad"], t * toward[1],
                                           delta=1e-12)

    def test_300_degrees_has_no_estimate(self):
        # the tip of a 300 degree arc is about 0.19 of the length from the
        # base, under CHORD_FLOOR: no estimate, but the shares are given
        tip = _arc_tip(self.L, np.radians(300.0))
        c = float(np.hypot(self.L + tip[0], tip[1]))
        self.assertLess(c, emp.CHORD_FLOOR * self.L)
        s = emp.body_shape(tip, self.L)
        for k in emp.ESTIMATE_KEYS:
            self.assertIsNone(s[k], k)
        self.assertEqual(s["axial_share"], tip[0] / self.L)
        self.assertEqual(s["bending_share"], float(np.hypot(tip[1], tip[2])) / self.L)

    def test_stretched_arc(self):
        for deg in (30.0, 90.0, 170.0, 200.0):
            t = np.radians(deg)
            with self.subTest(deg=deg):
                s = emp.body_shape(_arc_tip(self.L, t, 0.1), self.L)
                self.assertAlmostEqual(s["bend_rad"], t, delta=1e-12)
                self.assertAlmostEqual(s["stretch_share"], 0.1, delta=1e-12)

    def test_pure_extender(self):
        for dx in (0.01, -0.01):
            s = emp.body_shape([dx, 0.0, 0.0], self.L)
            self.assertEqual(s["bend_rad"], 0.0)
            self.assertEqual(s["bend_y_rad"], 0.0)
            self.assertEqual(s["bend_z_rad"], 0.0)
            self.assertAlmostEqual(s["stretch_share"], dx / self.L, delta=1e-15)
            self.assertEqual(s["axial_share"], dx / self.L)
            self.assertEqual(s["bending_share"], 0.0)
            self.assertFalse(any(np.isnan(v) for v in s.values()))

    def test_tip_behind_the_base(self):
        # the tip straight behind the base: no bend direction, no estimate
        s = emp.body_shape([-1.5 * self.L, 0.0, 0.0], self.L)
        for k in emp.ESTIMATE_KEYS:
            self.assertIsNone(s[k], k)
        self.assertEqual(s["axial_share"], -1.5)
        self.assertEqual(s["bending_share"], 0.0)

    def test_zero_displacement(self):
        self.assertEqual(emp.body_shape([0.0, 0.0, 0.0], self.L),
                         dict.fromkeys(emp.LENGTH_KEYS, 0.0))

    def test_no_length(self):
        self.assertEqual(emp.body_shape([0.01, 0.02, 0.0], None),
                         dict.fromkeys(emp.LENGTH_KEYS))
        leg = _by_id()["mosadegh2014_fPN"]["legs"][0]
        for k in emp.LENGTH_KEYS:
            self.assertIsNone(leg["values"][k], k)
        self.assertIsNone(leg["length_sensitivity"])
        # the same record given a length gets every value
        legs, why = emp.legs_of(load_record("mosadegh2014_fPN"), 0.05)
        self.assertIsNone(why)
        self.assertTrue(all(legs[0]["values"][k] is not None
                            for k in emp.LENGTH_KEYS))

    def test_tip_near_the_base_null_at_that_level_only(self):
        # one level of a real record, in memory only, moved so the tip is
        # 0.1 of the length from the base
        rid = "wall_0.0010m"
        rec = copy.deepcopy(load_record(rid))
        L = rec["metadata"]["length_m"]
        k = 3
        rec["free"]["tip_displacement_m"][k] = [-L, 0.1 * L, 0.0]
        leg = emp.legs_of(rec)[0][0]
        want = emp.legs_of(load_record(rid))[0][0]
        for key in emp.ESTIMATE_KEYS:
            v = leg["values"][key]
            self.assertIsNone(v[k], key)
            self.assertEqual(v[:k] + v[k + 1:],
                             want["values"][key][:k] + want["values"][key][k + 1:])
        for key in emp.SENSITIVITY_KEYS:
            v = leg["length_sensitivity"][key]
            self.assertIsNone(v[k], key)
            self.assertTrue(all(x is not None for i, x in enumerate(v) if i != k))
        self.assertEqual(leg["values"]["axial_share"][k], -1.0)
        self.assertAlmostEqual(leg["values"]["bending_share"][k], 0.1, delta=1e-15)

    def test_sensitivity_is_half_the_difference(self):
        leg = _by_id()["wall_0.0010m"]["legs"][0]
        L = 0.073
        v = leg["values"]
        for i, xyz in enumerate(zip(v["axial_m"], v["side_y_m"], v["side_z_m"])):
            lo = emp.body_shape(xyz, L * 0.95)
            hi = emp.body_shape(xyz, L * 1.05)
            for k in emp.SENSITIVITY_KEYS:
                self.assertEqual(leg["length_sensitivity"][k][i],
                                 abs(hi[k] - lo[k]) / 2.0)
                self.assertGreaterEqual(leg["length_sensitivity"][k][i], 0.0)


class Lengths(unittest.TestCase):
    """data/map_lengths.json: estimated lengths for records that hold none.
    Fixtures are temporary files; the real file is never written."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def _fixture(self, records):
        p = os.path.join(self.tmp, "lengths.json")
        with open(p, "w", encoding="utf-8") as fo:
            json.dump({"note": "fixture", "records": records}, fo)
        return p

    def test_real_file(self):
        with open(emp.LENGTHS, "rb") as fo:
            raw = fo.read()
        self.assertNotIn(b"\r", raw)
        doc = json.loads(raw.decode("utf-8"))
        self.assertEqual({k: e["length_m"] for k, e in doc["records"].items()},
                         ESTIMATED_LENGTHS)
        for rid, e in doc["records"].items():
            self.assertEqual(sorted(e), ["length_m", "source"], rid)
            self.assertTrue(e["source"].strip(), rid)
        self.assertEqual(sorted(list(ESTIMATED_LENGTHS) + STILL_NO_LENGTH_IDS),
                         NO_LENGTH_IDS)
        self.assertTrue(doc["note"].strip())

    def test_length_source_per_actuator(self):
        recs = _all_records()
        est = _real_lengths()
        not_own, none = [], []
        for a in DOC["actuators"]:
            own = (recs[a["id"]][0].get("metadata") or {}).get("length_m")
            if own:
                self.assertNotIn(a["id"], est)
                self.assertEqual(a["length"], {"value_m": own,
                                               "source": "record",
                                               "detail": None}, a["id"])
                continue
            not_own.append(a["id"])
            if a["id"] in est:
                e = est[a["id"]]
                self.assertEqual(a["length"], {"value_m": e["length_m"],
                                               "source": "estimate",
                                               "detail": e["source"]}, a["id"])
            else:
                self.assertEqual(a["length"], {"value_m": None, "source": None,
                                               "detail": None}, a["id"])
                for leg in a["legs"]:
                    for k in emp.LENGTH_KEYS:
                        self.assertIsNone(leg["values"][k], (a["id"], k))
                    self.assertIsNone(leg["length_sensitivity"], a["id"])
                none.append(a["id"])
        self.assertEqual(sorted(not_own), NO_LENGTH_IDS)
        self.assertEqual(sorted(none), STILL_NO_LENGTH_IDS)

    def test_bad_entries_refused(self):
        rid = "mosadegh2014_fPN"
        g = {"length_m": 0.05, "source": "fixture"}
        bad = {
            "unknown id": {"no_such_record": g},
            "record holds its own length": {"bellows1": g},
            "draft holds its own length": {"pneunet_tandem_twist_60deg": g},
            "zero": {rid: {**g, "length_m": 0.0}},
            "negative": {rid: {**g, "length_m": -0.05}},
            "text": {rid: {**g, "length_m": "0.05"}},
            "true": {rid: {**g, "length_m": True}},
            "not a number": {rid: {**g, "length_m": float("nan")}},
            "infinite": {rid: {**g, "length_m": float("inf")}},
            "no length": {rid: {"source": "fixture"}},
            "no source": {rid: {"length_m": 0.05}},
            "empty source": {rid: {**g, "source": "  "}},
            "not an object": {rid: 0.05},
        }
        for what, records in bad.items():
            with self.subTest(what=what):
                with self.assertRaises(ValueError):
                    emp.build(drafts=True, lengths=self._fixture(records))
        # a draft id is checked even when drafts are left out
        with self.assertRaises(ValueError):
            emp.build(drafts=False, lengths=self._fixture(
                {"pneunet_tandem_twist_60deg": g}))

    def test_estimate_reaches_the_document(self):
        rid = "mosadegh2014_fPN"
        self.assertIn(rid, STILL_NO_LENGTH_IDS)
        doc = emp.build(drafts=True, lengths=self._fixture(
            {**_real_lengths(),
             rid: {"length_m": 0.05, "source": "fixture: read off a figure"}}))
        a = {x["id"]: x for x in doc["actuators"]}[rid]
        self.assertEqual(a["length"], {"value_m": 0.05, "source": "estimate",
                                       "detail": "fixture: read off a figure"})
        v = a["legs"][0]["values"]
        for i, xyz in enumerate(zip(v["axial_m"], v["side_y_m"], v["side_z_m"])):
            s = emp.body_shape(xyz, 0.05)
            for k in emp.LENGTH_KEYS:
                self.assertEqual(v[k][i], s[k], (k, i))
        self.assertIsNotNone(a["legs"][0]["length_sensitivity"])
        # every other actuator is unchanged
        others = [x for x in doc["actuators"] if x["id"] != rid]
        self.assertEqual(others, [x for x in DOC["actuators"] if x["id"] != rid])


class LengthPins(unittest.TestCase):
    """The new quantities on the real records, at the values computed when
    they were added. Angles in radians; degrees in the comments."""

    def _top(self, rid):
        leg = _by_id()[rid]["legs"][0]
        return (leg["values"]["bend_rad"][-1], leg["values"]["stretch_share"][-1],
                leg["length_sensitivity"]["bend_rad"][-1],
                leg["length_sensitivity"]["stretch_share"][-1])

    def test_estimated_lengths(self):
        # Top of the free path, with the lengths from data/map_lengths.json.
        # Extender +0.113 and contractor -0.049 of a 120 mm body, no bend;
        # the three chambers bend 208.3, 218.7 and 201.7 deg (the paper's own
        # largest fitted bends are 227, 222 and 204 deg).
        want = {"abboodi2026_ossa5_axial_extend": (0.0, 0.113, 0.0005),
                "abboodi2026_ossa5_vacuum_contract": (0.0, -0.049, 0.0005),
                "chandler2020_pha_a0_C1": (208.3, -0.092, 0.0005),
                "chandler2020_pha_a0_C2": (218.7, 0.005, 0.0005),
                "chandler2020_pha_a0_C3": (201.7, 0.116, 0.0005)}
        for rid, (deg, stretch, tol) in want.items():
            v = _by_id()[rid]["legs"][0]["values"]
            self.assertAlmostEqual(np.degrees(v["bend_rad"][-1]), deg,
                                   delta=0.05, msg=rid)
            self.assertAlmostEqual(v["stretch_share"][-1], stretch,
                                   delta=tol, msg=rid)

    def test_wall_0010(self):
        # bend 164.29 deg, stretch +0.0923; stretch at the length 5% shorter
        # +0.1864 and 5% longer +0.0155
        bend, stretch, sb, ss = self._top("wall_0.0010m")
        self.assertAlmostEqual(bend, 2.8674638388888187, delta=1e-12)
        self.assertAlmostEqual(np.degrees(bend), 164.29, delta=0.005)
        self.assertAlmostEqual(stretch, 0.09234622454477459, delta=1e-12)
        self.assertAlmostEqual(sb, 0.13107523950212063, delta=1e-12)
        self.assertAlmostEqual(ss, 0.0854160819437052, delta=1e-12)
        leg = _by_id()["wall_0.0010m"]["legs"][0]
        v = leg["values"]
        top = (v["axial_m"][-1], v["side_y_m"][-1], v["side_z_m"][-1])
        self.assertAlmostEqual(emp.body_shape(top, 0.073 * 0.95)["stretch_share"],
                               0.18637584201033963, delta=1e-12)
        self.assertAlmostEqual(emp.body_shape(top, 0.073 * 1.05)["stretch_share"],
                               0.015543678122929228, delta=1e-12)

    def test_bellows1(self):
        # stretch +0.162, bend 0.66 deg
        bend, stretch, sb, ss = self._top("bellows1")
        self.assertAlmostEqual(stretch, 0.16161719966273735, delta=1e-12)
        self.assertAlmostEqual(bend, 0.01150357386188945, delta=1e-12)
        self.assertAlmostEqual(sb, 0.0004960724506205853, delta=1e-12)
        self.assertAlmostEqual(ss, 0.00810222273553407, delta=1e-12)

    def test_kubler_drafts(self):
        # these drafts' tips were built from an even arc over the actuated
        # length (build_kubler2023_records.py), so the stretch comes back
        # zero to rounding at every level: that is by construction, not a
        # finding
        want = {"kubler2023_cPAM_W40xL40": (1.0095822151576161, 0.0,
                                            0.05057381030908481,
                                            0.008706759404609687),
                "kubler2023_fPAM_W40": (0.4538344747375816, 0.0,
                                        0.02274566059105959,
                                        0.0017352726648035),
                "kubler2023_pouch_W40xL40": (0.5394742904744392, 0.0,
                                             0.027036396295169907,
                                             0.0024554704372700686)}
        for rid, (bend, stretch, sb, ss) in want.items():
            with self.subTest(rid=rid):
                got = self._top(rid)
                self.assertAlmostEqual(got[0], bend, delta=1e-12)
                self.assertAlmostEqual(got[1], stretch, delta=1e-15)
                self.assertAlmostEqual(got[2], sb, delta=1e-12)
                self.assertAlmostEqual(got[3], ss, delta=1e-12)
                v = _by_id()[rid]["legs"][0]["values"]["stretch_share"]
                self.assertLess(max(abs(x) for x in v), 1e-15)

    def test_tandem_bend_check(self):
        # (measured tilt, estimated bend) per leg at its top level, in
        # degrees: 60deg draft 29.60 / 40.24, 35.47 / 16.21, 43.61 / 29.73;
        # 90deg draft 30.71 / 41.49, 36.74 / 17.17, 39.46 / 25.82. The
        # estimate does not match, as expected for a body that bends in two
        # parts; this pins the numbers, it does not validate anything.
        want = {"pneunet_tandem_twist_60deg": [
                    (0.5166859899775555, 0.7023702871739019),
                    (0.6191011480036955, 0.28297009828400294),
                    (0.7610520952898279, 0.5189515737484536)],
                "pneunet_tandem_twist_90deg": [
                    (0.5359245227163716, 0.7240989055994297),
                    (0.6412120185747003, 0.299667166729286),
                    (0.6886792290260307, 0.45064419072270495)]}
        recs = _all_records()
        for rid in TANDEM:
            a = _by_id()[rid]
            R = recs[rid][0]["free"]["tip_rotvec"]
            U = recs[rid][0]["free"]["u"]
            self.assertEqual(len(a["bend_check"]), 3)
            tops = ([120.0, 0.0], [0.0, 120.0], [60.0, 120.0])
            for b, leg, (tilt, bend), top in zip(a["bend_check"], a["legs"],
                                                 want[rid], tops):
                self.assertEqual(list(b), ["leg", "measured_tilt_rad",
                                           "estimated_bend_rad"])
                self.assertEqual(b["leg"], leg["label"])
                self.assertEqual(b["estimated_bend_rad"],
                                 leg["values"]["bend_rad"][-1])
                self.assertAlmostEqual(b["measured_tilt_rad"], tilt, delta=1e-12)
                self.assertAlmostEqual(b["estimated_bend_rad"], bend, delta=1e-12)
                # recomputed here: the turned x axis's angle from x
                M = _rotm(R[_row(U, *top)])
                self.assertAlmostEqual(b["measured_tilt_rad"],
                                       float(np.arccos(np.clip(M[0, 0], -1, 1))),
                                       delta=1e-9)
        for a in DOC["actuators"]:
            if a["id"] not in TANDEM:
                self.assertIsNone(a["bend_check"], a["id"])

    def test_tandem_worst_gap_share_by_key(self):
        want = {"pneunet_tandem_twist_60deg": {
                    "bend_rad": 0.0005640011478160988,
                    "stretch_share": 0.000859802559203919,
                    "bend_y_rad": 0.0008985987347522021,
                    "bend_z_rad": 0.0006330611128240905},
                "pneunet_tandem_twist_90deg": {
                    "bend_rad": 0.0016869142195679123,
                    "stretch_share": 0.0006809493854197799,
                    "bend_y_rad": 0.0026480139331747005,
                    "bend_z_rad": 0.0005214696486826655}}
        for rid in TANDEM:
            s = _by_id()[rid]["surface"]
            got = s["worst_gap_share_by_key"]
            self.assertEqual(list(got), emp.ESTIMATE_KEYS)
            for k in emp.ESTIMATE_KEYS:
                self.assertAlmostEqual(got[k], want[rid][k], delta=1e-12, msg=(rid, k))
                gaps = [abs(t["measured"][k] - t["predicted"][k]) for t in s["tests"]]
                top = max(abs(t["measured"][k]) for t in s["tests"])
                self.assertEqual(got[k], max(gaps) / top)

    def test_test_points_carry_the_shape(self):
        recs = _all_records()
        for rid in TANDEM:
            Y = recs[rid][0]["free"]["tip_displacement_m"]
            U = recs[rid][0]["free"]["u"]
            for t in _by_id()[rid]["surface"]["tests"]:
                want = emp.body_shape(Y[_row(U, *t["inputs"])], 0.073)
                for k in emp.LENGTH_KEYS:
                    self.assertEqual(t["measured"][k], want[k])
                    self.assertIsNotNone(t["predicted"][k])
                    self.assertIsNotNone(t["predicted_sum"][k])
                p = t["predicted"]
                self.assertEqual(p["bend_rad"], emp.body_shape(
                    [p[k] for k in XYZ], 0.073)["bend_rad"])

    def test_surface_without_length(self):
        rec = copy.deepcopy(_all_records()[TANDEM[0]][0])
        del rec["metadata"]["length_m"]
        s = emp.surface_of(rec, SERIES)
        for k in emp.LENGTH_KEYS:
            self.assertIsNone(s["values"][k], k)
            for t in s["tests"]:
                self.assertIsNone(t["measured"][k])
                self.assertIsNone(t["predicted"][k])
        self.assertEqual(s["worst_gap_share_by_key"],
                         dict.fromkeys(emp.ESTIMATE_KEYS))
        self.assertIsNotNone(s["worst_position_gap_share"])
        emp.render({"surface": s})

    def test_test_point_near_the_base(self):
        # one test point's measured tip, in memory only, moved to 0.1 of the
        # length from the base: its estimates are null, and the worst gaps
        # by key come from the other points alone
        for rid in TANDEM:
            with self.subTest(rid=rid):
                rec = copy.deepcopy(_all_records()[rid][0])
                L = rec["metadata"]["length_m"]
                orig = emp.surface_of(copy.deepcopy(rec), SERIES)
                t0 = orig["tests"][0]
                U = rec["free"]["u"]
                rec["free"]["tip_displacement_m"][_row(U, *t0["inputs"])] = \
                    [-L, 0.1 * L, 0.0]
                s = emp.surface_of(rec, SERIES)
                moved, rest = s["tests"][0], s["tests"][1:]
                self.assertEqual(moved["inputs"], t0["inputs"])
                self.assertEqual(rest, orig["tests"][1:])
                self.assertGreaterEqual(len(rest), 1)
                for k in emp.ESTIMATE_KEYS:
                    self.assertIsNone(moved["measured"][k], k)
                    self.assertIsNotNone(moved["predicted"][k], k)
                    gaps = [abs(t["measured"][k] - t["predicted"][k])
                            for t in rest]
                    top = max(abs(t["measured"][k]) for t in rest)
                    self.assertEqual(s["worst_gap_share_by_key"][k],
                                     max(gaps) / top, k)
                self.assertEqual(moved["measured"]["axial_share"], -1.0)
                self.assertEqual(s["values"], orig["values"])
                emp.render({"surface": s})


class DocumentShape(unittest.TestCase):
    """Every actuator and leg carries the new keys; the quantities list
    says which group each belongs to and which are estimates."""

    def test_new_keys_everywhere(self):
        for a in DOC["actuators"]:
            self.assertIn("length", a)
            self.assertIn("bend_check", a)
            self.assertEqual(list(a["length"]), ["value_m", "source", "detail"])
            for leg in a["legs"]:
                self.assertEqual(list(leg), emp.LEG_KEYS, a["id"])
                self.assertEqual(list(leg["values"]), emp.VALUE_KEYS, a["id"])
                n = len(leg["input"]["values"])
                if a["length"]["value_m"] is None:
                    self.assertIsNone(leg["length_sensitivity"])
                    continue
                self.assertEqual(list(leg["length_sensitivity"]),
                                 ["bend_rad", "stretch_share"])
                for k in emp.LENGTH_KEYS:
                    self.assertEqual(len(leg["values"][k]), n, (a["id"], k))

    def test_length_tolerance(self):
        self.assertEqual(emp.LENGTH_TOLERANCE, 0.05)
        self.assertEqual(DOC["length_tolerance"], emp.LENGTH_TOLERANCE)
        keys = list(DOC)
        self.assertEqual(keys.index("length_tolerance"),
                         keys.index("quantities") + 1)

    def test_quantities(self):
        q = DOC["quantities"]
        self.assertEqual(len(q), 12)
        self.assertEqual([x["key"] for x in q[:6]],
                         ["axial_m", "bending_m", "twist_rad", "side_y_m",
                          "side_z_m", "input_share"])
        groups = {"axial_m": "Tip travel", "bending_m": "Tip travel",
                  "twist_rad": "Tip travel", "side_y_m": "Tip travel",
                  "side_z_m": "Tip travel", "input_share": "Input",
                  "bend_rad": "Body shape, estimated from the tip position",
                  "stretch_share": "Body shape, estimated from the tip position",
                  "bend_y_rad": "Body shape, estimated from the tip position",
                  "bend_z_rad": "Body shape, estimated from the tip position",
                  "axial_share": "Tip travel as a share of length",
                  "bending_share": "Tip travel as a share of length"}
        self.assertEqual({x["key"]: x["group"] for x in q}, groups)
        for x in q:
            self.assertEqual(list(x), ["key", "label", "unit", "group",
                                       "estimate", "definition"])
            self.assertIs(x["estimate"], x["key"] in emp.ESTIMATE_KEYS)
            if x["estimate"]:
                d = x["definition"]
                for phrase in ("one even arc", "only extends",
                               "only part of its length", "360 degrees",
                               "30% of the body length"):
                    self.assertIn(phrase, d, (x["key"], phrase))
        labels = {x["key"]: (x["label"], x["unit"]) for x in q[6:]}
        self.assertEqual(labels, {
            "bend_rad": ("Bend angle (estimate)", "rad"),
            "stretch_share": ("Stretch (estimate)", ""),
            "bend_y_rad": ("Bend toward y (estimate)", "rad"),
            "bend_z_rad": ("Bend toward z (estimate)", "rad"),
            "axial_share": ("Axial tip travel ÷ length", ""),
            "bending_share": ("Sideways tip travel ÷ length", "")})
        stretch = next(x for x in q if x["key"] == "stretch_share")
        self.assertIn("depends strongly on the body length", stretch["definition"])


def _stored_separation():
    with open(os.path.join(ROOT, "outputs", "backwards_lookup.json"),
              encoding="utf-8") as fo:
        sep = json.load(fo)["separation"]
    return list(sep["compared"]), set(sep["stored_turners"])


def _task_entry(a, task="cup_turn"):
    return next(t for t in a["tasks"] if t["task"] == task)


class TaskCone(unittest.TestCase):
    """The cup turn drawn as a cone: declared in a sidecar, each record
    tested against its own needed direction at its measured levels."""

    def _cones(self, **change):
        with open(emp.TASK_CONES, encoding="utf-8") as fo:
            doc = json.load(fo)
        doc["cones"][0].update(change)
        p = os.path.join(self.tmp, "cones.json")
        with open(p, "w", encoding="utf-8") as fo:
            json.dump(doc, fo)
        return p

    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def test_document(self):
        keys = list(DOC)
        self.assertEqual(keys.index("task_cones"),
                         keys.index("length_tolerance") + 1)
        self.assertEqual(keys.index("force_note"), keys.index("task_cones") + 1)
        (c,) = DOC["task_cones"]
        with open(emp.TASK_CONES, encoding="utf-8") as fo:
            declared = json.load(fo)["cones"][0]
        for k, v in declared.items():
            if k == "stroke":
                # the document adds the travel worked out from the task file
                # (checked in Stroke); the rest is the sidecar's own
                self.assertIn("travel_m", c[k])
                self.assertEqual({x: y for x, y in c[k].items()
                                  if x != "travel_m"}, v, k)
                continue
            if k == "return_clear":
                # the document adds how many stations it is read at
                # (checked in ReturnClear)
                self.assertEqual({x: y for x, y in c[k].items()
                                  if x != "stations"}, v, k)
                continue
            self.assertEqual(c[k], v, k)
        with open(os.path.join(ROOT, "data", "tasks", "cup_turn.json"),
                  encoding="utf-8") as fo:
            self.assertEqual(c["name"], json.load(fo)["name"])
        self.assertEqual(c["axes"], ["axial_m", "side_y_m", "side_z_m"])
        self.assertAlmostEqual(float(np.linalg.norm(c["drawn_direction"])), 1.0)
        # the placement is the one the stored lookup was run with
        self.assertIsNone(c["least_travel_m"])
        self.assertIn("display", c["half_angle_basis"])

    def test_every_record_actuator_has_an_entry(self):
        for a in DOC["actuators"]:
            (t,) = a["tasks"]
            self.assertEqual(t["task"], "cup_turn")
            if "reason" in t:
                self.assertEqual(list(t), ["task", "reason"])
                continue
            self.assertEqual(len(t["legs"]), len(a["legs"]), a["id"])
            for leg, g in zip(a["legs"], t["legs"]):
                n = len(leg["input"]["values"])
                self.assertEqual(len(g["angle_deg"]), n, a["id"])
                self.assertEqual(len(g["inside"]), n, a["id"])

    def test_angles_follow_the_record(self):
        # recomputed here from the leg's tip positions and the lookup's own
        # needed direction for the record
        from gramian_workshop.lookup import directions
        from gramian_workshop.task import load_task
        task = load_task("cup_turn")
        recs = _all_records()
        half = DOC["task_cones"][0]["half_angle_deg"]
        for a in DOC["actuators"]:
            t = _task_entry(a)
            if "reason" in t:
                continue
            rec, quality, _ = recs[a["id"]]
            e, nn, _ = directions(Finger(rec, quality), task, 4, "aimed", 90.0)
            np.testing.assert_allclose(t["needed_direction"], e, atol=1e-12)
            np.testing.assert_allclose(t["contact_normal"], nn, atol=1e-12)
            for leg, g in zip(a["legs"], t["legs"]):
                v = leg["values"]
                P = np.array([v["axial_m"], v["side_y_m"], v["side_z_m"]]).T
                for p, ang, ins in zip(P, g["angle_deg"], g["inside"]):
                    r = float(np.linalg.norm(p))
                    if r == 0.0:
                        self.assertIsNone(ang, a["id"])
                        self.assertIsNone(ins, a["id"])
                        continue
                    want = float(np.degrees(np.arccos(np.clip(
                        p @ e / (r * np.linalg.norm(e)), -1.0, 1.0))))
                    self.assertAlmostEqual(ang, want, places=9, msg=a["id"])
                    self.assertIs(ins, want <= half, a["id"])
            angs = [x for g in t["legs"] for x in g["angle_deg"] if x is not None]
            self.assertEqual(t["least_angle_deg"], min(angs))
            self.assertIs(t["enters"],
                          any(i is True for g in t["legs"] for i in g["inside"]))

    def test_clean_records_inside_are_the_stored_turners(self):
        # among the records the stored lookup compared, the ones that enter
        # the cone are exactly its stored turners; and this holds for every
        # half-angle between the two bounds worked out here, so it does not
        # hang on the 45 degrees chosen for the display
        compared, turners = _stored_separation()
        by = _by_id()
        on_map = [r for r in compared if r in by]
        self.assertTrue(turners <= set(on_map))
        entering = {r for r in on_map if _task_entry(by[r])["enters"]}
        self.assertEqual(entering, turners)
        low = max(_task_entry(by[r])["least_angle_deg"] for r in turners)
        high = min(_task_entry(by[r])["least_angle_deg"]
                   for r in on_map if r not in turners)
        half = DOC["task_cones"][0]["half_angle_deg"]
        self.assertLess(low, half)
        self.assertLess(half, high)
        self.assertGreater(high - low, 40.0)
        for h in (low + 0.01, high - 0.01):
            doc = emp.build(drafts=True,
                            task_cones=self._cones(half_angle_deg=h))
            got = {a["id"] for a in doc["actuators"]
                   if a["id"] in on_map and _task_entry(a)["enters"]}
            self.assertEqual(got, turners, h)
        for h, changed in ((low - 0.01, True), (high + 0.01, True)):
            doc = emp.build(drafts=True,
                            task_cones=self._cones(half_angle_deg=h))
            got = {a["id"] for a in doc["actuators"]
                   if a["id"] in on_map and _task_entry(a)["enters"]}
            self.assertIs(got != turners, changed, h)

    def test_thin_wall_finger_leaves_at_high_input(self):
        # wall_0.0010m curls over: inside from its first step, outside again
        # before its top level
        a = _by_id()["wall_0.0010m"]
        (g,) = _task_entry(a)["legs"]
        u = a["legs"][0]["input"]["values"]
        (run,) = g["inside_input"]
        self.assertEqual(run[0], u[1])
        self.assertLess(run[1], u[-1])
        self.assertIs(g["inside"][-1], False)
        self.assertIsNone(g["inside"][0])

    def test_inside_input_is_runs_of_measured_levels(self):
        self.assertEqual(emp._runs([False, True, True, False, True],
                                   [0.0, 1.0, 2.0, 3.0, 4.0]),
                         [[1.0, 2.0], [4.0, 4.0]])
        self.assertEqual(emp._runs([], []), [])

    def test_least_travel_is_a_floor(self):
        least = 0.01
        doc = emp.build(drafts=True,
                        task_cones=self._cones(least_travel_m=least))
        self.assertEqual(doc["task_cones"][0]["least_travel_m"], least)
        cut = 0
        for a, b in zip(DOC["actuators"], doc["actuators"]):
            ta, tb = _task_entry(a), _task_entry(b)
            if "reason" in ta:
                continue
            e = np.array(ta["needed_direction"])
            for leg, ga, gb in zip(a["legs"], ta["legs"], tb["legs"]):
                v = leg["values"]
                P = np.array([v["axial_m"], v["side_y_m"], v["side_z_m"]]).T
                for p, ia, ib in zip(P, ga["inside"], gb["inside"]):
                    if ia is None:
                        self.assertIsNone(ib)
                        continue
                    self.assertIs(ib, bool(ia and float(p @ e) >= least))
                    cut += ia and not ib
        self.assertGreater(cut, 0)

    def test_bad_cones_refused(self):
        bad = {"half angle 0": {"half_angle_deg": 0},
               "half angle 90": {"half_angle_deg": 90},
               "half angle text": {"half_angle_deg": "45"},
               "no basis": {"half_angle_basis": " "},
               "no reason for no floor": {"least_travel_note": ""},
               "negative floor": {"least_travel_m": -0.01},
               "n": {"n": 4.5}, "mount": {"mount": None},
               "yaw": {"yaw_deg": None}, "task": {"task": ""}}
        for what, change in bad.items():
            with self.subTest(what=what):
                with self.assertRaises(ValueError):
                    emp.build(drafts=True, task_cones=self._cones(**change))
        with open(emp.TASK_CONES, encoding="utf-8") as fo:
            doc = json.load(fo)
        doc["cones"] *= 2
        p = os.path.join(self.tmp, "twice.json")
        with open(p, "w", encoding="utf-8") as fo:
            json.dump(doc, fo)
        with self.assertRaises(ValueError):
            emp.build(drafts=True, task_cones=p)

    def test_angle(self):
        self.assertAlmostEqual(emp.angle_deg([1, 0, 0], [0, 2, 0]), 90.0)
        self.assertAlmostEqual(emp.angle_deg([1, 0, 0], [-3, 0, 0]), 180.0)
        self.assertIsNone(emp.angle_deg([0, 0, 0], [0, 1, 0]))


class HeldTipForce(unittest.TestCase):
    """Each actuator's "force" is its record's blocked block as it stands:
    same levels, same numbers, null for a part the record does not hold."""

    PAIRS = (("axial_N", "force_axial_N"),
             ("sideways_size_N", "force_transverse_N"),
             ("vector_N", "force_N"), ("moment_Nm", "moment_Nm"),
             ("twist_tip_Nm", "twist_tip_Nm"))

    def test_as_stored(self):
        recs = _all_records()
        seen = 0
        for a in DOC["actuators"]:
            b = recs[a["id"]][0].get("blocked") or {}
            f = a["force"]
            if a["id"] in DOC["force_withheld"]:
                # left off by data/map_force_withheld.json (class ForceWithheld)
                self.assertIsNone(f, a["id"])
                continue
            if not b.get("u"):
                self.assertIsNone(f, a["id"])
                continue
            seen += 1
            self.assertEqual(f["method"], b.get("method"), a["id"])
            self.assertEqual(f["moment_method"], b.get("moment_method"))
            n = len(b["u"])
            self.assertEqual(len(f["input"]["values"]), n)
            for row, want in zip(f["input"]["values"], b["u"]):
                self.assertEqual(row, [float(x) for x in
                                       (want if isinstance(want, list) else [want])])
            for ours, theirs in self.PAIRS:
                with self.subTest(id=a["id"], key=ours):
                    if b.get(theirs) is None:
                        self.assertIsNone(f[ours])
                    else:
                        self.assertEqual(f[ours], np.asarray(
                            b[theirs], float).tolist())
            self.assertEqual(len(f["size_N"]), n)
            self.assertIs(f["direction_known"], f["vector_N"] is not None)
        self.assertGreater(seen, 15)

    def test_size_says_what_it_is(self):
        for a in DOC["actuators"]:
            f = a["force"]
            if f is None:
                continue
            ax, sd, vec = f["axial_N"], f["sideways_size_N"], f["vector_N"]
            with self.subTest(id=a["id"]):
                if vec is not None:
                    self.assertEqual(f["size_is"], "whole force")
                    np.testing.assert_allclose(
                        f["size_N"], np.linalg.norm(vec, axis=1), rtol=1e-12)
                elif ax is not None and sd is not None:
                    self.assertEqual(f["size_is"], "whole force")
                    np.testing.assert_allclose(f["size_N"], np.hypot(ax, sd),
                                               rtol=1e-12)
                elif ax is not None:
                    self.assertEqual(f["size_is"], "axial part only")
                    self.assertEqual(f["size_N"], [abs(x) for x in ax])
                    self.assertIsNone(f["off_axis_deg"])
                else:
                    self.assertEqual(f["size_is"], "sideways part only")
                    self.assertEqual(f["size_N"], [abs(x) for x in sd])
                    self.assertIsNone(f["off_axis_deg"])
        by = _by_id()
        self.assertEqual(by["mosadegh2014_fPN"]["force"]["size_is"],
                         "sideways part only")
        self.assertEqual(by["cherubini2015_RC_datasetC"]["force"]["size_is"],
                         "axial part only")
        self.assertIsNone(by["yang2016_vamp_ecoflex"]["force"])

    def test_bending_finger_pushes_along_its_length(self):
        # held at rest, the thin-wall finger's force lies within a few
        # degrees of its long axis. (Its angle to the cup turn's needed
        # direction is a right angle by how the finger is placed, so that
        # is not tested as a finding.)
        f = _by_id()["wall_0.0010m"]["force"]
        rec = load_record("wall_0.0010m")
        self.assertEqual(f["vector_N"][-1], rec["blocked"]["force_N"][-1])
        self.assertLess(f["off_axis_deg"][-1], 5.0)

    def test_angle_to_task_needs_the_whole_vector(self):
        for a in DOC["actuators"]:
            f = a["force"]
            if f is None:
                continue
            if f["vector_N"] is None:
                self.assertEqual(f["to_task"], [], a["id"])
            else:
                (t,) = f["to_task"]
                self.assertEqual(len(t["to_needed_direction_deg"]),
                                 len(f["vector_N"]))

    def test_note(self):
        for phrase in ("not measured, not zero", "comparative, not predictive",
                       "not along the tip path", "inlet centroid"):
            self.assertIn(phrase, DOC["force_note"])

    def test_bad_shapes_become_null(self):
        self.assertIsNone(emp._column({"k": [1.0, 2.0]}, "k", 3))
        self.assertIsNone(emp._column({"k": [1.0, None]}, "k", 2))
        self.assertIsNone(emp._column({"k": [[1, 2], [3, 4]]}, "k", 2, 3))
        self.assertEqual(emp._column({"k": [1, 2]}, "k", 2), [1.0, 2.0])
        self.assertIsNone(emp.force_of({"blocked": {"u": [[1.0]]}}, []))
        self.assertIsNone(emp.force_of({}, []))


def _push(a, task="cup_turn"):
    return _task_entry(a, task).get("push")


def _cup_task():
    with open(os.path.join(ROOT, "data", "tasks", "cup_turn.json"),
              encoding="utf-8") as fo:
        return json.load(fo)


class HoldPush(unittest.TestCase):
    """The push each finger must make to hold the ball (the repo's hold
    solve, unit force per finger, 1 over the smallest margin over the duty
    schedule's contact sets), and what each record can push along its own
    push direction with the tip held."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def _push_of(self, rid, scale=1.0):
        # the export's own push_of on a copy of the record (in memory only)
        # with every stored held-tip force multiplied by `scale`
        rec, quality, _ = _all_records()[rid]
        rec = copy.deepcopy(rec)
        b = rec["blocked"]
        for k in ("force_N", "force_axial_N", "force_transverse_N"):
            if b.get(k):
                b[k] = (np.asarray(b[k], float) * scale).tolist()
        a = _by_id()[rid]
        (c,) = DOC["task_cones"]
        cone = {k: c[k] for k in ("task", "n", "mount", "yaw_deg",
                                  "push_threshold_ratio")}
        from gramian_workshop.task import load_task
        return emp.push_of(rec, quality, a["legs"], emp.force_of(rec, []),
                           _task_entry(a), cone, load_task("cup_turn"),
                           c["push"]["needed_N"])

    def test_needed_reproduces_and_matches_the_formula(self):
        p = DOC["task_cones"][0]["push"]
        self.assertEqual(round(p["needed_N"], 4), 0.3555)
        self.assertEqual(p["duty"], _cup_task()["wave"]["duty"])
        self.assertEqual(p["worst_contact_count"], 3)
        # independently, from the task file alone: weight / (2 sin |altitude|)
        t = _cup_task()
        w = t["object"]["mass_kg"] * t["gravity_m_s2"]
        alt = np.radians(t["contact_ring"]["altitude_deg"])
        want = w / (2.0 * abs(np.sin(alt)))
        self.assertLess(abs(p["needed_N"] - want) / want, 1e-3)

    def test_needed_is_the_same_for_every_record(self):
        task = DOC["task_cones"][0]["push"]["needed_N"]
        got = {}
        for a in DOC["actuators"]:
            if a["tasks"] is None or _push(a) is None:
                continue
            p = _push(a)
            if p["rule"] == "force_N":
                self.assertIsNotNone(p["needed_N"], a["id"])
            if p["needed_N"] is not None:
                got[a["id"]] = p["needed_N"]
        self.assertGreater(len(got), 10)
        for rid, v in got.items():
            self.assertLess(abs(v - task), 1e-6, rid)

    def test_wall_ratio_from_the_record(self):
        rec, _, _ = _all_records()["wall_0.0010m"]
        f = np.asarray(rec["blocked"]["force_N"], float)
        b = f[-1] / np.linalg.norm(f[-1])
        p = _push(_by_id()["wall_0.0010m"])
        need = DOC["task_cones"][0]["push"]["needed_N"]
        np.testing.assert_allclose(p["available_N"], f @ b, rtol=1e-12)
        np.testing.assert_allclose(p["ratio"], (f @ b) / need, rtol=1e-12)
        self.assertEqual(round(p["ratio"][-1], 3), 0.682)
        self.assertEqual(p["meets_input"], [])
        self.assertEqual(p["rule"], "force_N")
        self.assertIs(p["direction_known"], True)

    def test_needed_does_not_depend_on_the_record_force(self):
        need = DOC["task_cones"][0]["push"]["needed_N"]
        # the cable finger's held-tip force is left off the map
        # (data/map_force_withheld.json), so its push is worked out here from
        # the record as stored, in memory
        cable = self._push_of("cable_finger")
        self.assertGreater(max(cable["available_N"]), 1000.0)
        self.assertEqual(cable["needed_N"], need)
        # ten times the force: the same needed push, ten times the ratio
        p1, p10 = self._push_of("wall_0.0010m"), self._push_of("wall_0.0010m", 10.0)
        self.assertEqual(p1["needed_N"], p10["needed_N"])
        np.testing.assert_allclose(p10["ratio"], np.asarray(p1["ratio"]) * 10,
                                   rtol=1e-9)

    def test_size_only_records(self):
        need = DOC["task_cones"][0]["push"]["needed_N"]
        seen = 0
        for a in DOC["actuators"]:
            if a["tasks"] is None or _push(a) is None:
                continue
            p = _push(a)
            if p["direction_known"]:
                self.assertIsNone(p["size_only_ratio"], a["id"])
                continue
            seen += 1
            self.assertIsNone(p["available_N"], a["id"])
            self.assertIsNone(p["ratio"], a["id"])
            self.assertIsNone(p["both_input"], a["id"])
            self.assertTrue(p["both_reason"], a["id"])
            np.testing.assert_allclose(
                p["size_only_ratio"], np.asarray(a["force"]["size_N"]) / need,
                rtol=1e-12)
        self.assertGreaterEqual(seen, 5)

    def test_no_push_without_a_force(self):
        for a in DOC["actuators"]:
            if a["tasks"] is None:
                continue
            if a["force"] is None:
                self.assertIsNone(_push(a), a["id"])
            else:
                self.assertIsNotNone(_push(a), a["id"])

    def _check_both(self, p, a):
        if not p["both_input"]:
            return 0
        u = [row[0] for row in p["input"]["values"]]
        runs = [r for g in _task_entry(a)["legs"] for r in g["inside_input"]]
        n = 0
        for lo, hi in p["both_input"]:
            for k, x in enumerate(u):
                if lo <= x <= hi:
                    n += 1
                    self.assertGreaterEqual(p["ratio"][k], 1.0)
                    self.assertTrue(any(m <= x <= M
                                        for m, M in p["meets_input"]))
                    self.assertTrue(any(m <= x <= M for m, M in runs))
        return n

    def test_both_lies_in_meets_and_in_the_cone(self):
        for a in DOC["actuators"]:
            if a["tasks"] is None or _push(a) is None:
                continue
            p = _push(a)
            if p["both_input"] is not None:
                self.assertIsNone(p["both_reason"], a["id"])
                self._check_both(p, a)
        # not empty on a record made to push ten times harder: every
        # held-tip level of the thin wall then meets the need, and only
        # those inside the cone run are in both
        a = _by_id()["wall_0.0010m"]
        p = self._push_of("wall_0.0010m", 10.0)
        self.assertTrue(p["meets_input"])
        self.assertGreater(self._check_both(p, a), 0)
        (run,) = _task_entry(a)["legs"][0]["inside_input"]
        want = [row[0] for row, r in zip(p["input"]["values"], p["ratio"])
                if r >= 1.0 and run[0] <= row[0] <= run[1]]
        self.assertEqual(p["both_input"], [[want[0], want[-1]]])

    def test_multi_input_both_is_null_with_reason(self):
        for rid in TANDEM:
            p = _push(_by_id()[rid])
            self.assertIsNone(p["both_input"], rid)
            self.assertEqual(p["both_reason"], emp.MULTI_INPUT_BOTH)
            self.assertIn("input pairs", p["both_reason"])
            # runs over pairs give each end as the whole stored row
            for lo, hi in p["meets_input"]:
                self.assertEqual(len(lo), 2)
                self.assertEqual(len(hi), 2)

    def test_by_duty_and_by_n(self):
        p = DOC["task_cones"][0]["push"]
        self.assertEqual([d["duty"] for d in p["by_duty"]],
                         _cup_task()["wave"]["duty_sensitivity"])
        half = next(d for d in p["by_duty"] if d["duty"] == 0.5)
        self.assertIsNone(half["needed_N"])
        self.assertEqual(half["reason"], "direction")
        self.assertEqual(half["worst_contact_count"], 2)
        self.assertEqual([b["n"] for b in p["by_n"]], [3, 4, 5, 6])
        at4 = next(b for b in p["by_n"] if b["n"] == 4)
        self.assertEqual(at4["needed_N"], p["needed_N"])
        for b in p["by_n"]:
            self.assertIs(b["needed_N"] is None, b["reason"] is not None)

    def test_passers_range(self):
        p = DOC["task_cones"][0]["push"]
        compared, _ = _stored_separation()
        by = _by_id()
        ok, fail = [], []
        for r in compared:
            if r not in by or _push(by[r]) is None:
                continue
            q = _push(by[r])
            (ok if max(q["available_N"]) >= p["needed_N"] else fail).append(
                max(q["available_N"]))
        lo, hi = p["passers_hold_range_N"]
        self.assertEqual((lo, hi), (max(fail), min(ok)))
        self.assertLess(lo, p["needed_N"])
        self.assertLess(p["needed_N"], hi)

    def test_basis_from_the_sidecar(self):
        (c,) = DOC["task_cones"]
        with open(emp.TASK_CONES, encoding="utf-8") as fo:
            declared = json.load(fo)["cones"][0]
        self.assertEqual(c["push"]["basis"], declared["push_basis"])
        self.assertEqual(c["push"]["threshold_ratio"],
                         declared["push_threshold_ratio"])
        for phrase in ("hold solve", "mass", "gravity", "friction",
                       "altitude", "duty", "only its size is tested",
                       "held at rest", "comparative, not predictive",
                       "fully settled"):
            self.assertIn(phrase, c["push"]["basis"], phrase)
        # words only: no digit in the basis
        self.assertFalse(any(ch.isdigit() for ch in c["push"]["basis"]))

    def _sidecar(self, drop=(), **change):
        with open(emp.TASK_CONES, encoding="utf-8") as fo:
            doc = json.load(fo)
        for k in drop:
            doc["cones"][0].pop(k)
        doc["cones"][0].update(change)
        p = os.path.join(self.tmp, "cones.json")
        with open(p, "w", encoding="utf-8") as fo:
            json.dump(doc, fo)
        return p

    def test_no_push_declared_gives_no_push(self):
        doc = emp.build(drafts=True, task_cones=self._sidecar(
            drop=("push_basis", "push_threshold_ratio")))
        self.assertIsNone(doc["task_cones"][0]["push"])
        for a in doc["actuators"]:
            for t in a["tasks"] or []:
                self.assertNotIn("push", t, a["id"])

    def test_hand_placed_have_no_push(self):
        for a in DOC["actuators"]:
            if a["hand_placed"]:
                self.assertIsNone(a["tasks"])
                self.assertIsNone(a["force"])

    def test_bad_push_declarations_refused(self):
        for what, (drop, change) in {
                "basis alone": (("push_threshold_ratio",), {}),
                "ratio alone": (("push_basis",), {}),
                "blank basis": ((), {"push_basis": " "}),
                "zero ratio": ((), {"push_threshold_ratio": 0}),
                "text ratio": ((), {"push_threshold_ratio": "1"})}.items():
            with self.subTest(what=what):
                with self.assertRaises(ValueError):
                    emp.build(drafts=True,
                              task_cones=self._sidecar(drop, **change))


def _wave_manip():
    """A Wave_manip checkout: WAVE_MANIP if set, else the one beside this
    repository's projects folder. None where neither is a git repo."""
    for p in (os.environ.get("WAVE_MANIP"),
              os.path.join(os.path.dirname(ROOT), "ARIA", "Wave_manip")):
        if p and os.path.exists(os.path.join(p, ".git")):
            return p
    return None


class Stroke(unittest.TestCase):
    """The cone's far end: one stroke of the wave. The travel comes from the
    sidecar's stroke angle and the task file's ball radius and contact-ring
    altitude; each actuator's stroke entry is recomputed here from its leg
    values and its own needed direction."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def _sidecar(self, stroke=None, drop=False):
        with open(emp.TASK_CONES, encoding="utf-8") as fo:
            doc = json.load(fo)
        c = doc["cones"][0]
        if drop:
            del c["stroke"]
            # the second check needs a stroke to come back from
            c.pop("return_clear", None)
        elif stroke is not None:
            c["stroke"] = stroke
        p = os.path.join(self.tmp, f"cones_{len(os.listdir(self.tmp))}.json")
        with open(p, "w", encoding="utf-8") as fo:
            json.dump(doc, fo)
        return p

    @staticmethod
    def _declared():
        with open(emp.TASK_CONES, encoding="utf-8") as fo:
            return json.load(fo)["cones"][0]["stroke"]

    def test_travel_is_from_the_task_file(self):
        t = _cup_task()
        s = DOC["task_cones"][0]["stroke"]
        want = (np.radians(self._declared()["angle_deg"])
                * t["object"]["radius_m"]
                * np.cos(np.radians(t["contact_ring"]["altitude_deg"])))
        self.assertAlmostEqual(s["travel_m"], float(want), places=15)
        # the inverse of the repo's own turn-per-stroke rule
        R, alt = t["object"]["radius_m"], np.radians(t["contact_ring"]["altitude_deg"])
        self.assertAlmostEqual(float(np.degrees(s["travel_m"] / (R * np.cos(alt)))),
                               self._declared()["angle_deg"], places=12)

    def test_sidecar_stroke_is_the_document_stroke_less_travel(self):
        s = DOC["task_cones"][0]["stroke"]
        self.assertEqual({k: v for k, v in s.items() if k != "travel_m"},
                         self._declared())
        self.assertEqual(sorted(s), ["angle_deg", "basis", "source", "travel_m"])
        c = DOC["task_cones"][0]
        self.assertIsNone(c["least_travel_m"])
        self.assertNotIn("ruling", c["least_travel_note"])
        self.assertIn("stroke", c["least_travel_note"])

    def test_actuator_strokes_recomputed(self):
        travel = DOC["task_cones"][0]["stroke"]["travel_m"]
        half = DOC["task_cones"][0]["half_angle_deg"]
        seen = {"first level": 0, "interpolated": 0, "never": 0,
                "second leg reaches": 0}
        for a in DOC["actuators"]:
            if a["tasks"] is None:
                continue
            t = _task_entry(a)
            if "reason" in t:
                self.assertNotIn("stroke", t, a["id"])
                continue
            s = t["stroke"]
            self.assertEqual(sorted(s), ["furthest_along_m", "inside_to_end",
                                         "legs", "reaches_end"], a["id"])
            self.assertEqual(len(s["legs"]), len(a["legs"]), a["id"])
            e = np.array(t["needed_direction"])
            furthest = []
            for i, (leg, g) in enumerate(zip(a["legs"], s["legs"])):
                v, u = leg["values"], leg["input"]["values"]
                P = np.array([v["axial_m"], v["side_y_m"], v["side_z_m"]]).T
                along = [float(p @ e) for p in P]
                np.testing.assert_allclose(g["along_m"], along, rtol=0,
                                           atol=1e-15, err_msg=a["id"])
                furthest += along
                angs = []
                for p in P:
                    r = float(np.linalg.norm(p))
                    angs.append(None if r == 0.0 else float(np.degrees(np.arccos(
                        np.clip(p @ e / (r * np.linalg.norm(e)), -1.0, 1.0)))))
                hit = [k for k, x in enumerate(along) if x >= travel]
                self.assertIs(g["reaches_end"], bool(hit), a["id"])
                if not hit:
                    seen["never"] += 1
                    self.assertIsNone(g["end_input"], a["id"])
                    self.assertIsNone(g["end_angle_deg"], a["id"])
                    self.assertIs(g["inside_to_end"], False, a["id"])
                    self.assertEqual(g["end_input_is"],
                                     "interpolated between logged levels")
                    continue
                k = hit[0]
                if i > 0:
                    seen["second leg reaches"] += 1
                if k == 0:
                    seen["first level"] += 1
                    self.assertEqual(g["end_input"], u[0], a["id"])
                    self.assertEqual(g["end_input_is"], "already at or beyond "
                                     "the end at the first logged level")
                else:
                    seen["interpolated"] += 1
                    want = u[k - 1] + (travel - along[k - 1]) / \
                        (along[k] - along[k - 1]) * (u[k] - u[k - 1])
                    self.assertTrue(np.isclose(g["end_input"], want, rtol=1e-12,
                                               atol=0.0), a["id"])
                    self.assertTrue(min(u[k - 1], u[k]) <= g["end_input"]
                                    <= max(u[k - 1], u[k]), a["id"])
                    self.assertEqual(g["end_input_is"],
                                     "interpolated between logged levels")
                if angs[k] is None:
                    self.assertIsNone(g["end_angle_deg"], a["id"])
                else:
                    self.assertAlmostEqual(g["end_angle_deg"], angs[k],
                                           places=9, msg=a["id"])
                self.assertIs(g["inside_to_end"],
                              all(x is None or x <= half for x in angs[:k + 1]),
                              a["id"])
            self.assertEqual(s["furthest_along_m"], max(furthest), a["id"])
            self.assertIs(s["reaches_end"],
                          any(g["reaches_end"] for g in s["legs"]), a["id"])
            self.assertIs(s["inside_to_end"],
                          any(g["inside_to_end"] for g in s["legs"]), a["id"])
        # every branch is exercised by the real records
        for what, n in seen.items():
            self.assertGreater(n, 0, what)

    # atan2(across, along) against the stored angle, in degrees. The stored
    # angle comes from arccos, which loses digits near 0 and 180 degrees; the
    # worst gap on the export of 21 Nov 2026 was 1.6e-11 degrees.
    ACROSS_ANGLE_TOL_DEG = 1e-9

    def test_across_gives_the_stored_angle(self):
        # the picture drawn from along and across cannot disagree with the
        # stored angle, at any level of any leg of any actuator
        n, worst = 0, 0.0
        for a in DOC["actuators"]:
            if a["tasks"] is None:
                continue
            t = _task_entry(a)
            if "reason" in t:
                continue
            for i, (tl, g) in enumerate(zip(t["legs"], t["stroke"]["legs"])):
                for k, ang in enumerate(tl["angle_deg"]):
                    if ang is None:
                        continue
                    got = float(np.degrees(np.arctan2(g["across_m"][k],
                                                      g["along_m"][k])))
                    worst = max(worst, abs(got - ang))
                    self.assertLessEqual(abs(got - ang), self.ACROSS_ANGLE_TOL_DEG,
                                         f"{a['id']} leg {i} level {k}")
                    n += 1
        self.assertGreater(n, 0)

    def test_across_is_a_size_from_the_same_vector(self):
        seen_null_angle = 0
        for a in DOC["actuators"]:
            if a["tasks"] is None:
                continue
            t = _task_entry(a)
            if "reason" in t:
                continue
            e = np.array(t["needed_direction"])
            for leg, tl, g in zip(a["legs"], t["legs"], t["stroke"]["legs"]):
                self.assertEqual(len(g["across_m"]), len(g["along_m"]), a["id"])
                v = leg["values"]
                for k, (s, c) in enumerate(zip(g["along_m"], g["across_m"])):
                    p = [v["axial_m"][k], v["side_y_m"][k], v["side_z_m"][k]]
                    if s is None or None in p:
                        self.assertIsNone(c, a["id"])
                        continue
                    self.assertIsNotNone(c, a["id"])
                    self.assertGreaterEqual(c, 0.0, a["id"])
                    p = np.array(p, float)
                    # Pythagoras on the same vector: along and across make up
                    # the whole tip travel
                    self.assertAlmostEqual(s * s + c * c, float(p @ p),
                                           delta=1e-15, msg=a["id"])
                    if tl["angle_deg"][k] is None:
                        seen_null_angle += 1
                        self.assertEqual(c, 0.0, a["id"])
        # the rest level, where there is no angle, is in the real export
        self.assertGreater(seen_null_angle, 0)

    def test_across_on_a_hand_built_leg(self):
        # needed direction along x: a tip at (3, 4, 0) is 3 along and 4
        # across; one at (-1, 0, -2) is 1 behind and 2 across, never -2
        e = np.array([1.0, 0.0, 0.0])
        leg = {"values": {"axial_m": [0.0, 3.0, -1.0],
                          "side_y_m": [0.0, 4.0, 0.0],
                          "side_z_m": [0.0, 0.0, -2.0]},
               "input": {"values": [0.0, 1.0, 2.0]}}
        g = emp.stroke_of([leg], [[None, 53.13, 116.57]], e, 45.0, 10.0)["legs"][0]
        self.assertEqual(g["along_m"], [0.0, 3.0, -1.0])
        self.assertEqual(g["across_m"], [0.0, 4.0, 2.0])

    def test_stroke_changes_nothing_else(self):
        doc = emp.build(drafts=True, task_cones=self._sidecar(drop=True))
        self.assertIsNone(doc["task_cones"][0]["stroke"])
        for a, b in zip(DOC["actuators"], doc["actuators"]):
            self.assertEqual(a["id"], b["id"])
            if a["tasks"] is None:
                self.assertIsNone(b["tasks"])
                continue
            ta, tb = _task_entry(a), _task_entry(b)
            self.assertNotIn("stroke", tb, a["id"])
            self.assertNotIn("return_clear", tb, a["id"])
            self.assertEqual({k: v for k, v in ta.items()
                              if k not in ("stroke", "return_clear")},
                             tb, a["id"])

    def test_bad_strokes_refused(self):
        good = self._declared()
        src = good["source"]
        cases = {
            "not an object": [40],
            "zero angle": {**good, "angle_deg": 0},
            "negative angle": {**good, "angle_deg": -5},
            "above the script's limit": {**good, "angle_deg": 50.001},
            "boolean angle": {**good, "angle_deg": True},
            "text angle": {**good, "angle_deg": "40"},
            "no angle": {k: v for k, v in good.items() if k != "angle_deg"},
            "no source": {k: v for k, v in good.items() if k != "source"},
            "source not an object": {**good, "source": "w20s4"},
            "source without commit": {**good, "source": {
                k: v for k, v in src.items() if k != "commit"}},
            "source without flag": {**good, "source": {
                k: v for k, v in src.items() if k != "flag"}},
            "blank sha256": {**good, "source": {**src, "sha256": " "}},
            "dirty not true or false": {**good, "source": {**src, "dirty": 0}},
            "no basis": {k: v for k, v in good.items() if k != "basis"},
            "blank basis": {**good, "basis": "  "}}
        for what, stroke in cases.items():
            with self.subTest(what=what):
                with self.assertRaises(ValueError):
                    emp.load_task_cones(self._sidecar(stroke))
        # the script's own limit is allowed
        (c,) = emp.load_task_cones(self._sidecar({**good, "angle_deg": 50}))
        self.assertEqual(c["stroke"]["angle_deg"], 50.0)

    def test_source_pin(self):
        # the stroke is the one in the pinned Wave_manip run command
        import hashlib
        import shlex
        import subprocess
        wm = _wave_manip()
        if wm is None:
            self.skipTest("no Wave_manip checkout (set WAVE_MANIP)")
        src = self._declared()["source"]
        got = subprocess.run(["git", "-C", wm, "show",
                              f"{src['commit']}:{src['path']}"],
                             capture_output=True)
        if got.returncode != 0:
            self.skipTest(f"commit {src['commit']} is not in {wm}")
        self.assertFalse(src["dirty"])
        self.assertEqual(hashlib.sha256(got.stdout).hexdigest(), src["sha256"])
        node = json.loads(got.stdout.decode("utf-8"))
        for part in src["key"].split("/"):
            node = node[part] if isinstance(node, dict) else node[int(part)]
        # the same tokenising as tools/export_tasks.py's flag_value
        toks = shlex.split(node, posix=False)
        self.assertEqual(float(toks[toks.index(src["flag"]) + 1]),
                         self._declared()["angle_deg"])
        # and the same file the task file takes its duty from
        wave = _cup_task()["wave"]["source"]
        self.assertEqual([wave[k] for k in ("commit", "path", "sha256", "key")],
                         [src[k] for k in ("commit", "path", "sha256", "key")])


def _gramain():
    """A Gramain_extraction checkout: GRAMAIN_EXTRACTION if set, else the
    one beside this repository's projects folder. None where neither is a
    git repo."""
    for p in (os.environ.get("GRAMAIN_EXTRACTION"),
              os.path.join(os.path.dirname(ROOT), "ARIA", "Gramain_extraction")):
        if p and os.path.exists(os.path.join(p, ".git")):
            return p
    return None


def _git_show(repo, commit, path):
    import subprocess
    got = subprocess.run(["git", "-C", repo, "show", f"{commit}:{path}"],
                         capture_output=True)
    return got.stdout if got.returncode == 0 else None


def _value_at_line(raw, symbol, line):
    """The number assigned to `symbol` on 1-based `line` of raw file bytes,
    or None where that line does not assign it."""
    import re
    lines = raw.decode("utf-8").splitlines()
    if line > len(lines):
        return None
    m = re.match(rf"\s*{re.escape(symbol)}\s*=\s*([0-9.eE+-]+)", lines[line - 1])
    return float(m.group(1)) if m else None


def _in_scene(rec):
    """A simulation run in the scene the pressure correction is for, with an
    input in Pa (the provenance rule the sidecar's applies_to states)."""
    return ((rec.get("source") or {}).get("kind") == "simulation"
            and (rec.get("provenance") or {}).get("scene") == emp.SCENE_NAME
            and any(c.get("unit") == "Pa"
                    for c in (rec.get("input") or {}).get("channels") or []))


def _without(doc, drop):
    """A deep copy of the document's scaled actuators with the keys in
    `drop` taken out of each leg's values, each leg, and each force."""
    out = copy.deepcopy(doc["scaled"]["actuators"])
    for e in out.values():
        for leg in e["legs"]:
            for k in drop:
                leg["values"].pop(k, None)
                leg.pop(k, None)
                leg["reasons"].pop(k, None)
        if e["force"] is not None:
            for k in drop:
                e["force"].pop(k, None)
    return out


class Scaled(unittest.TestCase):
    """The document's "scaled" block: body moduli (the record's own, or
    data/map_stiffness.json), the pressure correction
    (data/map_pressure_scale.json), and travel, input and held-tip force
    with size and material taken out. Fixtures are temporary files; the real
    sidecars are never written."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.S = DOC["scaled"]
        self.acts = {a["id"]: a for a in DOC["actuators"]}

    def tearDown(self):
        shutil.rmtree(self.tmp)

    @staticmethod
    def _real(path):
        with open(path, "rb") as fo:
            raw = fo.read()
        return raw, json.loads(raw.decode("utf-8"))

    def _fixture(self, name, doc):
        p = os.path.join(self.tmp, name)
        with open(p, "w", encoding="utf-8") as fo:
            json.dump(doc, fo)
        return p

    # ---- the sidecars as written ----

    def test_stiffness_file(self):
        raw, doc = self._real(emp.STIFFNESS)
        self.assertNotIn(b"\r", raw)
        self.assertTrue(doc["note"].strip())
        recs = _all_records()
        # declared for exactly the scene's simulation runs that hold no
        # modulus of their own
        want = sorted(rid for rid, (r, _, _) in recs.items()
                      if (r.get("source") or {}).get("kind") == "simulation"
                      and (r.get("provenance") or {}).get("scene")
                      == emp.SCENE_NAME
                      and emp.record_modulus(r) is None)
        self.assertEqual(sorted(doc["records"]), want)
        for rid, e in doc["records"].items():
            self.assertEqual(sorted(e), ["modulus_Pa", "source", "why"], rid)
            self.assertEqual(sorted(e["source"]),
                             sorted(list(emp.PIN_KEYS) + ["cad_dir"]), rid)
            self.assertTrue(e["why"].strip(), rid)
            # the record names the same CAD folder
            self.assertEqual(e["source"]["cad_dir"],
                             "gramian_map/" + recs[rid][0]["provenance"]["cad_dir"])
            self.assertEqual(recs[rid][0]["metadata"].get("materials") or {}, {})

    def test_pressure_scale_file(self):
        raw, doc = self._real(emp.PRESSURE_SCALE)
        self.assertNotIn(b"\r", raw)
        self.assertTrue(doc["note"].strip())
        self.assertEqual(list(doc["factors"]), ["scene_pressure"])
        e = doc["factors"]["scene_pressure"]
        self.assertIs(e["ruled"], False)
        # applies_to is the provenance rule, over every record and draft
        # record, drawn or not
        recs = _all_records()
        self.assertEqual(e["applies_to"],
                         sorted(rid for rid, (r, _, _) in recs.items()
                                if _in_scene(r)))
        # and the document carries it with its factor
        self.assertEqual(self.S["pressure_scale"]["scene_pressure"],
                         {**e, "factor": 1.0 / e["dt_s"]})

    def test_pins_against_the_repository(self):
        import hashlib
        import subprocess
        ge = _gramain()
        if ge is None:
            self.skipTest("no Gramain_extraction checkout "
                          "(set GRAMAIN_EXTRACTION)")
        _, stiff = self._real(emp.STIFFNESS)
        _, scale = self._real(emp.PRESSURE_SCALE)
        pins = [(rid, e["source"], e["modulus_Pa"])
                for rid, e in stiff["records"].items()]
        for e in scale["factors"].values():
            pins += [("dt", p, e["dt_s"]) for p in e["dt_sources"]]
        for who, src, value in pins:
            with self.subTest(who=who, commit=src["commit"][:7]):
                self.assertEqual(src["repo"], "Gramain_extraction")
                raw = _git_show(ge, src["commit"], src["path"])
                if raw is None:
                    self.skipTest(f"commit {src['commit']} is not in {ge}")
                self.assertEqual(hashlib.sha256(raw).hexdigest(), src["sha256"])
                self.assertEqual(_value_at_line(raw, src["symbol"], src["line"]),
                                 value)
        # no materials.json overrides the scene's modulus in any declared
        # record's CAD folder, on disk or anywhere in the history
        for rid, e in stiff["records"].items():
            with self.subTest(rid=rid, check="materials.json"):
                path = e["source"]["cad_dir"] + "/materials.json"
                self.assertFalse(os.path.exists(os.path.join(ge, path)), rid)
                log = subprocess.run(
                    ["git", "-C", ge, "log", "--all", "--format=%H", "--",
                     path], capture_output=True, text=True)
                self.assertEqual(log.returncode, 0)
                self.assertEqual(log.stdout.strip(), "", rid)

    def test_pressure_evidence(self):
        # the evidence file as pinned, and its two time steps follow the
        # impulse reading 0.01 / dt to within 2%
        import hashlib
        ge = _gramain()
        if ge is None:
            self.skipTest("no Gramain_extraction checkout "
                          "(set GRAMAIN_EXTRACTION)")
        _, scale = self._real(emp.PRESSURE_SCALE)
        ev = scale["factors"]["scene_pressure"]["evidence"]
        path = os.path.join(ge, ev["path"])
        if not os.path.exists(path):
            self.skipTest(f"{ev['path']} is not in {ge} (it is untracked)")
        with open(path, "rb") as fo:
            raw = fo.read()
        self.assertEqual(hashlib.sha256(raw).hexdigest(), ev["sha256"])
        with open(os.path.join(ge, ev["script"]), "rb") as fo:
            self.assertEqual(hashlib.sha256(fo.read()).hexdigest(),
                             ev["script_sha256"])
        runs = json.loads(raw.decode("utf-8"))["runs"]
        dts = [r["dt"] for r in runs]
        self.assertIn(scale["factors"]["scene_pressure"]["dt_s"], dts)
        self.assertGreaterEqual(len(set(dts)), 2)
        for r in runs:
            want = 0.01 / r["dt"]
            self.assertLess(abs(r["ratio_pressure_over_gravity"] - want) / want,
                            0.02, r["dt"])

    # ---- the loaders refuse bad entries ----

    def test_bad_stiffness_refused(self):
        _, good_doc = self._real(emp.STIFFNESS)
        rid = "bellows1"
        g = good_doc["records"][rid]
        src = g["source"]
        bad = {
            "unknown id": {"no_such_record": g},
            "record holds its own": {"mckibben_v2_20deg": g},
            "left-out record holds its own": {"mckibben_v2_70deg": g},
            "zero": {rid: {**g, "modulus_Pa": 0.0}},
            "negative": {rid: {**g, "modulus_Pa": -7e5}},
            "text": {rid: {**g, "modulus_Pa": "7e5"}},
            "true": {rid: {**g, "modulus_Pa": True}},
            "not a number": {rid: {**g, "modulus_Pa": float("nan")}},
            "no modulus": {rid: {k: v for k, v in g.items()
                                 if k != "modulus_Pa"}},
            "no source": {rid: {k: v for k, v in g.items() if k != "source"}},
            "source is text": {rid: {**g, "source": "the scene"}},
            "no why": {rid: {k: v for k, v in g.items() if k != "why"}},
            "blank why": {rid: {**g, "why": " "}},
            "not an object": {rid: 7e5},
            **{f"source without {k}": {rid: {**g, "source": {
                kk: vv for kk, vv in src.items() if kk != k}}}
               for k in emp.PIN_KEYS},
            "short sha256": {rid: {**g, "source": {**src, "sha256": "abc"}}},
            "line zero": {rid: {**g, "source": {**src, "line": 0}}},
            "line as text": {rid: {**g, "source": {**src, "line": "80"}}},
            "blank commit": {rid: {**g, "source": {**src, "commit": " "}}},
        }
        for what, records in bad.items():
            with self.subTest(what=what):
                with self.assertRaises(ValueError):
                    emp.build(drafts=True, stiffness=self._fixture(
                        "stiffness.json", {"note": "fixture",
                                           "records": records}))
        # a draft id is checked even when drafts are left out
        with self.assertRaises(ValueError):
            emp.build(drafts=False, stiffness=self._fixture(
                "stiffness.json", {"records": {"no_such_record": g}}))

    def test_bad_pressure_scale_refused(self):
        _, good_doc = self._real(emp.PRESSURE_SCALE)
        g = good_doc["factors"]["scene_pressure"]

        def one(**change):
            return {"factors": {"scene_pressure": {**g, **change}}}

        def drop(k):
            return {"factors": {"scene_pressure": {
                kk: v for kk, v in g.items() if kk != k}}}

        pin = g["dt_sources"][0]
        bad = {
            "not an object": {"factors": {"scene_pressure": 100}},
            "zero dt": one(dt_s=0.0),
            "text dt": one(dt_s="0.01"),
            "no dt": drop("dt_s"),
            "no dt sources": drop("dt_sources"),
            "empty dt sources": one(dt_sources=[]),
            "dt source without line": one(dt_sources=[
                {k: v for k, v in pin.items() if k != "line"}]),
            "no evidence": drop("evidence"),
            "evidence without sha256": one(evidence={
                k: v for k, v in g["evidence"].items() if k != "sha256"}),
            "no why": drop("why"),
            "no chosen_by": drop("chosen_by"),
            "ruled as text": one(ruled="no"),
            "no ruled": drop("ruled"),
            "empty applies_to": one(applies_to=[]),
            "unknown id": one(applies_to=g["applies_to"] + ["no_such_record"]),
            "literature record": one(applies_to=g["applies_to"]
                                     + ["mosadegh2014_fPN"]),
            "cable record": one(applies_to=g["applies_to"] + ["cable_finger"]),
            "in two entries": {"factors": {"scene_pressure": g,
                                           "again": {**g,
                                                     "applies_to": ["bellows1"]}}},
        }
        for what, doc in bad.items():
            with self.subTest(what=what):
                with self.assertRaises(ValueError):
                    emp.build(drafts=True, pressure_scale=self._fixture(
                        "scale.json", doc))

    # ---- the values ----

    def test_every_drawn_actuator_has_an_entry(self):
        self.assertEqual(list(self.S["actuators"]), list(self.acts))
        for rid, e in self.S["actuators"].items():
            self.assertEqual([x["label"] for x in e["legs"]],
                             [x["label"] for x in self.acts[rid]["legs"]], rid)
        self.assertEqual([q["key"] for q in self.S["quantities"]],
                         ["input_scaled", "side_y_share", "side_z_share"])
        for q in self.S["quantities"] + self.S["force_quantities"]:
            self.assertTrue(q["definition"].strip(), q["key"])
            self.assertIn(q["key"], emp.__doc__)
        # travel over length along x and sideways is not repeated
        main = {q["key"] for q in DOC["quantities"]}
        self.assertFalse(main & {q["key"] for q in self.S["quantities"]})
        self.assertTrue({"axial_share", "bending_share"} <= main)
        self.assertEqual(list(DOC)[-1], "scaled")

    def test_stiffness_per_actuator(self):
        recs = _all_records()
        _, stiff = self._real(emp.STIFFNESS)
        for rid, e in self.S["actuators"].items():
            st, rec = e["stiffness"], recs[rid][0]
            mat = (rec.get("metadata") or {}).get("materials") or {}
            with self.subTest(rid=rid):
                if "young_modulus_body" in mat:
                    self.assertEqual(st["source"], "record")
                    self.assertEqual(st["modulus_Pa"], mat["young_modulus_body"])
                    self.assertEqual(st["limiter_modulus_Pa"],
                                     mat["young_modulus_limiter"])
                    self.assertEqual(st["limiter_over_body"],
                                     mat["young_modulus_limiter"]
                                     / mat["young_modulus_body"])
                    self.assertIsNone(st["pin"])
                elif rid in stiff["records"]:
                    d = stiff["records"][rid]
                    self.assertEqual(st["source"], "declared")
                    self.assertEqual(st["modulus_Pa"], d["modulus_Pa"])
                    self.assertEqual(st["pin"], d["source"])
                    self.assertEqual(st["detail"], d["why"])
                else:
                    self.assertIsNone(st["modulus_Pa"])
                    self.assertEqual(st["reason"], emp.NO_MODULUS)
                    # only published records go without, and they lose
                    # their scaled pressure and scaled force. The one
                    # exception is the finger simulated from a mesh on the
                    # project page: its record names no scene and no
                    # materials, so nothing can be declared for it in
                    # data/map_stiffness.json (test_stiffness_file), and
                    # its input has no unit to scale.
                    if rid == "sample_pneunet_pressure_set":
                        self.assertEqual(self.acts[rid]["source_kind"],
                                         "simulation")
                        self.assertIsNone(rec.get("provenance"))
                        self.assertEqual(
                            [c.get("unit") for c in rec["input"]["channels"]],
                            [None])
                    else:
                        self.assertEqual(self.acts[rid]["source_kind"],
                                         "literature")
                    if e["force"] is not None:
                        for k in ("size_scaled", "axial_scaled",
                                  "sideways_scaled"):
                            self.assertIsNone(e["force"][k])
                            self.assertTrue(e["force"]["reasons"][k].strip())
                    for leg in e["legs"]:
                        self.assertIsNone(leg["values"]["input_scaled"])
                        self.assertTrue(leg["reasons"]["input_scaled"].strip())
                        self.assertNotIn("waiting", leg["reasons"]["input_scaled"])

    def test_scaled_values_recomputed(self):
        recs = _all_records()
        _, scale = self._real(emp.PRESSURE_SCALE)
        corrected = set(scale["factors"]["scene_pressure"]["applies_to"])
        factor = 1.0 / scale["factors"]["scene_pressure"]["dt_s"]
        for rid, e in self.S["actuators"].items():
            a = self.acts[rid]
            E = e["stiffness"]["modulus_Pa"]
            L = a["length"]["value_m"]
            for leg, s in zip(a["legs"], e["legs"]):
                with self.subTest(rid=rid, leg=leg["label"]):
                    u = leg["input"]
                    want = None
                    if u["unit"] == "Pa" and E is not None:
                        f = factor if rid in corrected else 1.0
                        want = [v * f / E for v in u["values"]]
                        form = emp.FORM_PRESSURE
                    elif a["physics"] == "cable" and u["unit"] == "m" and L:
                        want = [v / L for v in u["values"]]
                        form = emp.FORM_PULL
                    self.assertEqual(s["values"]["input_scaled"], want)
                    if want is None:
                        self.assertIsNone(s["input_scaled_form"])
                        self.assertTrue(s["reasons"]["input_scaled"].strip())
                    else:
                        self.assertEqual(s["input_scaled_form"], form)
                        self.assertNotIn("input_scaled", s["reasons"])
                        d = s["input_scaled_definition"]
                        self.assertIn(u["name"], d)
                        self.assertEqual(rid in corrected and form
                                         == emp.FORM_PRESSURE,
                                         "not yet ruled on" in d)
                    for k, src in (("side_y_share", "side_y_m"),
                                   ("side_z_share", "side_z_m")):
                        self.assertEqual(
                            s["values"][k],
                            None if L is None
                            else [v / L for v in leg["values"][src]])
            f = a["force"]
            if f is None:
                self.assertIsNone(e["force"])
                continue
            sf = e["force"]
            with self.subTest(rid=rid, part="force"):
                if E is None or L is None:
                    self.assertIsNone(sf["size_scaled"])
                    continue
                d = E * L * L
                self.assertEqual(sf["size_scaled"], [v / d for v in f["size_N"]])
                self.assertEqual(sf["axial_scaled"],
                                 None if f["axial_N"] is None
                                 else [v / d for v in f["axial_N"]])
                if f["vector_N"] is not None:
                    side = [float(np.hypot(v[1], v[2])) for v in f["vector_N"]]
                    self.assertEqual(sf["sideways_scaled"], [v / d for v in side])
                rows = f["input"]["values"]
                for c, unit in enumerate(f["input"]["units"]):
                    col = [r[c] for r in sf["input_scaled"]]
                    if unit == "Pa":
                        fc = factor if rid in corrected else 1.0
                        self.assertEqual(col, [r[c] * fc / E for r in rows])
                    elif a["physics"] == "cable" and unit == "m":
                        self.assertEqual(col, [r[c] / L for r in rows])
            # the record's own blocked inputs are the ones scaled
            self.assertEqual(len(sf["input_scaled"]),
                             len(recs[rid][0]["blocked"]["u"]))

    def test_cable_pull_over_length(self):
        # the cable finger's input is a tendon pull in metres: over L, with
        # no stiffness and no pressure correction
        e = self.S["actuators"]["cable_finger"]
        a = self.acts["cable_finger"]
        (leg,) = a["legs"]
        self.assertEqual(a["physics"], "cable")
        self.assertEqual(leg["input"]["unit"], "m")
        self.assertEqual(e["legs"][0]["input_scaled_form"], emp.FORM_PULL)
        L = a["length"]["value_m"]
        self.assertEqual(e["legs"][0]["values"]["input_scaled"],
                         [v / L for v in leg["input"]["values"]])
        d = e["legs"][0]["input_scaled_definition"]
        self.assertIn("No stiffness and no correction", d)
        self.assertNotIn("cable_finger",
                         self.S["pressure_scale"]["scene_pressure"]["applies_to"])

    def test_held_leg_says_only_its_own_input_is_scaled(self):
        for rid in TANDEM:
            for leg, s in zip(self.acts[rid]["legs"],
                              self.S["actuators"][rid]["legs"]):
                held = ", with" in leg["label"] and " alone," not in leg["label"]
                with self.subTest(rid=rid, leg=leg["label"]):
                    self.assertEqual("only this leg's own input is scaled"
                                     in s["input_scaled_definition"], held)
                    if held:
                        self.assertEqual(s["values"]["input_scaled"][0] == 0.0,
                                         leg["input"]["values"][0] == 0.0)

    def test_removing_the_correction_changes_only_the_scaled_input(self):
        doc = emp.build(drafts=True, pressure_scale=self._fixture(
            "scale.json", {"note": "fixture", "factors": {}}))
        # everything outside "scaled" is the same
        self.assertEqual({k: v for k, v in doc.items() if k != "scaled"},
                         {k: v for k, v in DOC.items() if k != "scaled"})
        self.assertEqual(doc["scaled"]["pressure_scale"], {})
        drop = ("input_scaled", "input_scaled_definition")
        self.assertEqual(_without(doc, drop), _without(DOC, drop))
        _, scale = self._real(emp.PRESSURE_SCALE)
        e = scale["factors"]["scene_pressure"]
        factor = 1.0 / e["dt_s"]
        for rid, s in doc["scaled"]["actuators"].items():
            old = DOC["scaled"]["actuators"][rid]
            for leg, was in zip(s["legs"], old["legs"]):
                now, then = (leg["values"]["input_scaled"],
                             was["values"]["input_scaled"])
                with self.subTest(rid=rid, leg=leg["label"]):
                    if rid not in e["applies_to"]:
                        self.assertEqual(now, then)
                        self.assertEqual(leg["input_scaled_definition"],
                                         was["input_scaled_definition"])
                        continue
                    # the corrected value is the uncorrected one times 1 / dt
                    for v, w in zip(now, then):
                        self.assertAlmostEqual(v * factor, w,
                                               delta=1e-12 * max(1.0, abs(w)))
                    self.assertIn("with no correction",
                                  leg["input_scaled_definition"])

    def test_without_the_sidecar_moduli_only_record_moduli_remain(self):
        doc = emp.build(drafts=True, stiffness=self._fixture(
            "stiffness.json", {"note": "fixture", "records": {}}))
        self.assertEqual({k: v for k, v in doc.items() if k != "scaled"},
                         {k: v for k, v in DOC.items() if k != "scaled"})
        for rid, e in doc["scaled"]["actuators"].items():
            src = e["stiffness"]["source"]
            self.assertIn(src, ("record", None), rid)
            if src is None and self.acts[rid]["physics"] != "cable":
                for leg in e["legs"]:
                    self.assertIsNone(leg["values"]["input_scaled"], rid)


class ForceWithheld(unittest.TestCase):
    """data/map_force_withheld.json: a listed record's held-tip force is left
    off the map, with its reason, and the record is not changed. Fixtures
    are temporary files; the real sidecar is never written."""

    RID = "cable_finger"

    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def _fixture(self, records):
        p = os.path.join(self.tmp, "withheld.json")
        with open(p, "w", encoding="utf-8") as fo:
            json.dump({"note": "fixture", "records": records}, fo)
        return p

    @staticmethod
    def _sidecar():
        with open(emp.FORCE_WITHHELD, "rb") as fo:
            raw = fo.read()
        return raw, json.loads(raw.decode("utf-8"))

    def test_sidecar_file(self):
        raw, doc = self._sidecar()
        self.assertNotIn(b"\r", raw)
        self.assertTrue(doc["note"].strip())
        self.assertEqual(list(doc["records"]), [self.RID])
        e = doc["records"][self.RID]
        a = _by_id()[self.RID]
        self.assertEqual(e["evidence"]["path"], a["record_file"])
        self.assertEqual(e["evidence"]["sha256"], a["record_sha256"])
        self.assertIn("probe_tendon1.csv", e["evidence"]["probe"])
        self.assertEqual(DOC["force_withheld"], doc["records"])

    def test_no_force_values_anywhere(self):
        a = _by_id()[self.RID]
        self.assertIsNone(a["force"])
        self.assertIs(a["has_force"], False)
        self.assertIn("no_force_data", a["flags"])
        self.assertIsNone(DOC["scaled"]["actuators"][self.RID]["force"])
        for t in a["tasks"]:
            self.assertIsNone(t.get("push"), t["task"])
        # none of the record's stored force numbers is anywhere in the
        # document's text
        rec = load_record(self.RID)
        b = rec["blocked"]
        nums = set(np.ravel(np.asarray(b["force_N"], float)).tolist())
        for k in ("force_axial_N", "force_transverse_N"):
            nums |= set(float(v) for v in b[k])
        nums.add(float(rec["scalars"]["blocked_force_N"]))
        text = emp.render(DOC)
        for v in sorted(x for x in nums if x != 0.0):
            self.assertNotIn(json.dumps(v), text, v)

    def test_reason_carried(self):
        _, doc = self._sidecar()
        reason = doc["records"][self.RID]["reason"]
        self.assertTrue(reason.startswith("Held-tip force left off"))
        for a in DOC["actuators"]:
            self.assertEqual(a["force_withheld"],
                             reason if a["id"] == self.RID else None, a["id"])
        self.assertEqual(DOC["force_withheld"][self.RID]["reason"], reason)

    def test_push_is_the_no_force_state(self):
        # the same as a record that never held a force: a task entry with
        # "push": null, so neither a pass nor a fail
        mine = _task_entry(_by_id()[self.RID])
        self.assertIn("push", mine)
        self.assertIsNone(mine["push"])
        never = [a for a in DOC["actuators"]
                 if not a["hand_placed"] and a["force"] is None
                 and a["force_withheld"] is None
                 and "reason" not in _task_entry(a)]
        self.assertTrue(never)
        for a in never:
            t = _task_entry(a)
            self.assertIsNone(t["push"], a["id"])
            self.assertEqual(list(t), list(mine), a["id"])

    def test_only_the_listed_record_changes(self):
        doc = emp.build(drafts=True, force_withheld=self._fixture({}))
        self.assertEqual(emp.build(drafts=True, force_withheld=None), doc)
        self.assertEqual(doc["force_withheld"], {})
        old = {a["id"]: a for a in doc["actuators"]}
        self.assertEqual(set(old), set(_by_id()))
        for rid, a in _by_id().items():
            if rid == self.RID:
                continue
            self.assertEqual(a, old[rid], rid)
            self.assertEqual(DOC["scaled"]["actuators"][rid],
                             doc["scaled"]["actuators"][rid], rid)
        for k in DOC:
            if k not in ("actuators", "scaled", "force_withheld"):
                self.assertEqual(DOC[k], doc[k], k)
        # without the entry the record's force is back as stored, and its
        # placement on the task is the same either way; only the push differs
        was, now = old[self.RID], _by_id()[self.RID]
        self.assertIsNotNone(was["force"])
        self.assertIs(was["has_force"], True)
        self.assertIsNotNone(_task_entry(was)["push"])
        drop = lambda t: {k: v for k, v in t.items() if k != "push"}  # noqa: E731
        self.assertEqual([drop(t) for t in now["tasks"]],
                         [drop(t) for t in was["tasks"]])
        self.assertEqual(now["legs"], was["legs"])

    def test_bad_entries_refused(self):
        _, doc = self._sidecar()
        g = doc["records"][self.RID]
        ev = g["evidence"]
        bad = {
            "unknown id": {"no_such_record": g},
            "not an object": {self.RID: "left off"},
            "no reason": {self.RID: {"evidence": ev}},
            "blank reason": {self.RID: {**g, "reason": " "}},
            "no evidence": {self.RID: {"reason": g["reason"]}},
            "no sha256": {self.RID: {**g, "evidence": {
                k: v for k, v in ev.items() if k != "sha256"}}},
            "another file": {self.RID: {**g, "evidence": {
                **ev, "path": "data/records/bellows1.json"}}},
            "sha256 not the file's": {self.RID: {**g, "evidence": {
                **ev, "sha256": "0" * 64}}},
            "no probe": {self.RID: {**g, "evidence": {
                k: v for k, v in ev.items() if k != "probe"}}},
        }
        for what, records in bad.items():
            with self.subTest(what=what):
                with self.assertRaises(ValueError):
                    emp.build(drafts=True,
                              force_withheld=self._fixture(records))
        # an unknown id is refused when drafts are left out too
        with self.assertRaises(ValueError):
            emp.build(drafts=False, force_withheld=self._fixture(
                {"no_such_record": g}))


class _SidecarFixture(unittest.TestCase):
    """A temporary folder for fixture files; the real files are never
    written."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def _write(self, name, doc):
        p = os.path.join(self.tmp, name)
        with open(p, "w", encoding="utf-8") as fo:
            json.dump(doc, fo)
        return p


class Names(_SidecarFixture):
    """data/map_names.json: the name each actuator is shown by and the
    reference its card links to. No network: the links are checked for
    their form only."""

    RID = "mosadegh2014_fPN"
    OWN = "bellows1"
    KINDS = {"mosadegh2014_fPN": "literature", "bellows1": "simulation"}

    @staticmethod
    def _sidecar():
        with open(emp.NAMES, "rb") as fo:
            raw = fo.read()
        return raw, json.loads(raw.decode("utf-8"))

    def _good(self):
        return {"note": "fixture",
                "references": {"r": {
                    "citation": "A. Author (2014). A title. A journal 1, 2.",
                    "authors_short": "Author", "year": 2014,
                    "doi": "10.1/x", "url": "https://doi.org/10.1/x",
                    "open_copy": None, "checked": "2026-11-20",
                    "checked_how": "fixture"}},
                "records": {self.RID: {
                    "name": "A name", "detail": "A sentence.",
                    "name_from": "authors", "name_source": "The title.",
                    "reference": "r"}}}

    def _raises(self, doc, text):
        with self.assertRaises(ValueError) as cm:
            emp.load_names(self._write("names.json", doc), self.KINDS)
        self.assertIn(text, str(cm.exception))

    def test_sidecar_file(self):
        raw, doc = self._sidecar()
        self.assertNotIn(b"\r", raw)
        self.assertTrue(doc["note"].startswith(
            "Read by tools/export_map_paths.py."))
        self.assertIn("data/records", doc["note"])
        # every drawn actuator that is not hand-placed has a name
        drawn = {a["id"] for a in DOC["actuators"] if not a["hand_placed"]}
        self.assertEqual(set(doc["records"]), drawn)
        used = {e["reference"] for e in doc["records"].values()} - {None}
        self.assertEqual(used, set(doc["references"]))

    def test_carried_to_each_actuator(self):
        _, doc = self._sidecar()
        for a in DOC["actuators"]:
            e = doc["records"].get(a["id"])
            if e is None:
                for k in emp.NO_NAME:
                    self.assertIsNone(a[k], (a["id"], k))
                continue
            self.assertEqual(a["display_name"], e["name"])
            self.assertEqual(a["name_detail"], e["detail"])
            self.assertEqual(a["name_from"], e["name_from"])
            self.assertEqual(a["name_source"], e["name_source"])
            if e["reference"] is None:
                self.assertIsNone(a["reference"], a["id"])
            else:
                r = doc["references"][e["reference"]]
                self.assertEqual(
                    a["reference"],
                    {k: r[k] for k in ("citation", "authors_short", "year",
                                       "doi", "url", "open_copy")})

    def test_record_name_is_kept(self):
        # the display name is carried beside the record's own name
        for a in DOC["actuators"]:
            if a["hand_placed"]:
                continue
            rec = _all_records()[a["id"]][0]
            self.assertEqual(a["name"], rec.get("name") or a["id"])

    def test_literature_has_a_working_form_of_link(self):
        for a in DOC["actuators"]:
            if a["source_kind"] != "literature" or a["hand_placed"]:
                continue
            r = a["reference"]
            self.assertIsNotNone(r, a["id"])
            self.assertEqual(r["url"], "https://doi.org/" + r["doi"])
            self.assertTrue(r["doi"].startswith("10."), a["id"])
            self.assertIn(str(r["year"]), r["citation"])
            if r["open_copy"] is not None:
                self.assertTrue(r["open_copy"]["url"].startswith("https://"))

    def test_our_own_records_say_so(self):
        for a in DOC["actuators"]:
            if a["hand_placed"] or a["source_kind"] == "literature":
                continue
            self.assertIsNone(a["reference"], a["id"])
            self.assertIn(a["name_from"], ("field", "description"), a["id"])
            self.assertTrue(a["name_source"].startswith(
                "Our own simulated design, not published."), a["id"])

    def test_names_are_short_and_differ(self):
        names = [a["display_name"] for a in DOC["actuators"]
                 if a["display_name"] is not None]
        self.assertEqual(len(names), len(set(names)))
        for n in names:
            self.assertLessEqual(len(n), emp.NAME_MAX, n)

    def test_fixture_loads(self):
        got = emp.load_names(self._write("names.json", self._good()),
                             self.KINDS)
        self.assertEqual(got[self.RID]["display_name"], "A name")
        self.assertEqual(got[self.RID]["reference"]["url"],
                         "https://doi.org/10.1/x")

    def test_no_file_means_no_names(self):
        self.assertEqual(emp.load_names(None, self.KINDS), {})
        doc = emp.build(drafts=True, names=None)
        for a in doc["actuators"]:
            for k in emp.NO_NAME:
                self.assertIsNone(a[k], (a["id"], k))

    def test_unknown_id_refused(self):
        d = self._good()
        d["records"]["no_such_record"] = d["records"].pop(self.RID)
        self._raises(d, "no record or draft record has that id")

    def test_empty_name_refused(self):
        for key in ("name", "detail", "name_source"):
            d = self._good()
            d["records"][self.RID][key] = " "
            self._raises(d, f"has no '{key}'")

    def test_long_name_refused(self):
        d = self._good()
        d["records"][self.RID]["name"] = "x" * (emp.NAME_MAX + 1)
        self._raises(d, "longer than")

    def test_same_name_twice_refused(self):
        d = self._good()
        d["records"][self.OWN] = dict(d["records"][self.RID],
                                      name_from="description", reference=None)
        self._raises(d, "has the same name")

    def test_name_from_checked(self):
        d = self._good()
        d["records"][self.RID]["name_from"] = "guess"
        self._raises(d, "name_from must be one of")

    def test_missing_reference_refused(self):
        d = self._good()
        d["records"][self.RID]["reference"] = "nope"
        self._raises(d, "holds no reference")

    def test_literature_needs_a_reference(self):
        d = self._good()
        d["records"][self.RID]["reference"] = None
        self._raises(d, "it needs a reference")
        # our own record may have none
        d = self._good()
        d["records"] = {self.OWN: dict(d["records"][self.RID],
                                       reference=None)}
        got = emp.load_names(self._write("own.json", d), self.KINDS)
        self.assertIsNone(got[self.OWN]["reference"])

    def test_link_must_be_the_doi(self):
        d = self._good()
        d["references"]["r"]["url"] = "https://example.org/paper"
        self._raises(d, "followed by the DOI")

    def test_reference_fields_checked(self):
        for key in ("citation", "authors_short", "doi", "checked",
                    "checked_how"):
            d = self._good()
            del d["references"]["r"][key]
            self._raises(d, f"has no '{key}'")
        d = self._good()
        d["references"]["r"]["year"] = "2014"
        self._raises(d, "whole number")

    def test_open_copy_checked(self):
        d = self._good()
        d["references"]["r"]["open_copy"] = {"url": "http://x.org/a",
                                             "what": "a copy"}
        self._raises(d, "https link")
        d = self._good()
        d["references"]["r"]["open_copy"] = {"url": "https://x.org/a"}
        self._raises(d, "has no 'what'")


class Thumbnails(_SidecarFixture):
    """data/map_thumbnails.json: the small picture of each actuator. Nothing
    is drawn here: the pictures are checked as files, and the mesh pins
    against the mesh repository where there is a checkout."""

    RID = "bellows1"
    KNOWN = {"bellows1", "mosadegh2014_fPN"}
    PNG = b"\x89PNG\r\n\x1a\n"

    @staticmethod
    def _sidecar():
        with open(emp.THUMBNAILS, "rb") as fo:
            raw = fo.read()
        return raw, json.loads(raw.decode("utf-8"))

    def _site(self):
        site = os.path.join(self.tmp, "site")
        os.makedirs(os.path.join(site, "thumbs"), exist_ok=True)
        with open(os.path.join(site, "thumbs", "a.png"), "wb") as fo:
            fo.write(self.PNG)
        return site

    def _mesh(self):
        return {"kind": "mesh", "image": "thumbs/a.png", "sources": [{
            "draws": "body", "repo": "Gramain_extraction",
            "commit": "abc", "path": "cads/x/actuator.npz",
            "sha256": "0" * 64}]}

    def _load(self, entry, rid=None):
        p = self._write("thumbs.json", {"note": "fixture",
                                        "records": {rid or self.RID: entry}})
        return emp.load_thumbnails(p, self.KNOWN, self._site())

    def _raises(self, entry, text, rid=None):
        with self.assertRaises(ValueError) as cm:
            self._load(entry, rid)
        self.assertIn(text, str(cm.exception))

    def test_sidecar_file(self):
        raw, doc = self._sidecar()
        self.assertNotIn(b"\r", raw)
        self.assertTrue(doc["note"].startswith(
            "Read by tools/render_map_thumbnails.py"))
        self.assertIn("data/records", doc["note"])
        drawn = {a["id"] for a in DOC["actuators"] if not a["hand_placed"]}
        self.assertEqual(set(doc["records"]), drawn)

    def test_carried_to_each_actuator(self):
        _, doc = self._sidecar()
        self.assertEqual(DOC["thumbnail_note"], emp.THUMBNAIL_NOTE)
        for a in DOC["actuators"]:
            e = doc["records"].get(a["id"])
            if e is None:
                self.assertIsNone(a["thumbnail"], a["id"])
                continue
            t = a["thumbnail"]
            self.assertEqual(sorted(t), ["basis", "image", "kind"])
            self.assertEqual(t["image"], e["image"])
            self.assertEqual(t["kind"], e["kind"])
            self.assertEqual(t["basis"], emp.MESH_BASIS
                             if e["kind"] == "mesh" else e["basis"])

    def test_real_geometry_only_for_simulated_records(self):
        # a literature record has no mesh, so its picture is a schematic and
        # says so; a simulated record's picture is drawn from its mesh
        for a in DOC["actuators"]:
            if a["hand_placed"]:
                continue
            want = ("schematic" if a["source_kind"] == "literature"
                    else "mesh")
            self.assertEqual(a["thumbnail"]["kind"], want, a["id"])

    def test_pictures_are_png_files_on_the_site(self):
        _, doc = self._sidecar()
        for rid, e in doc["records"].items():
            p = os.path.join(emp.SITE_DIR, *e["image"].split("/"))
            with open(p, "rb") as fo:
                self.assertEqual(fo.read(8), self.PNG, rid)
            name = os.path.basename(e["image"])
            if e["kind"] == "mesh":
                # one picture per mesh, named by the record
                self.assertEqual(name, rid + ".png")
            else:
                self.assertEqual(name, f"schematic_{e['schematic']}.png")
        # no picture is left on the site that the file does not list
        listed = {os.path.basename(e["image"])
                  for e in doc["records"].values()}
        self.assertEqual(
            set(os.listdir(os.path.join(emp.SITE_DIR, "thumbs"))), listed)

    def test_schematic_drawings_exist_in_the_render_script(self):
        _, doc = self._sidecar()
        with open(os.path.join(ROOT, "tools", "render_map_thumbnails.py"),
                  encoding="utf-8") as fo:
            src = fo.read()
        for rid, e in doc["records"].items():
            if e["kind"] == "schematic":
                self.assertIn(f'"{e["schematic"]}"', src, rid)

    def test_mesh_pins(self):
        repo = _gramain()
        if repo is None:
            self.skipTest("no Gramain_extraction checkout")
        import hashlib
        _, doc = self._sidecar()
        seen = 0
        for rid, e in doc["records"].items():
            for src in e.get("sources") or []:
                self.assertEqual(src["repo"], "Gramain_extraction")
                raw = _git_show(repo, src["commit"], src["path"])
                self.assertIsNotNone(raw, (rid, src["path"]))
                self.assertEqual(hashlib.sha256(raw).hexdigest(),
                                 src["sha256"], (rid, src["path"]))
                seen += 1
        self.assertGreater(seen, 0)

    def test_fixture_loads(self):
        self.assertEqual(self._load(self._mesh()), {self.RID: {
            "image": "thumbs/a.png", "kind": "mesh",
            "basis": emp.MESH_BASIS}})
        got = self._load({"kind": "schematic", "schematic": "pneunet",
                          "image": "thumbs/a.png", "basis": "As described."},
                         "mosadegh2014_fPN")
        self.assertEqual(got["mosadegh2014_fPN"]["basis"], "As described.")

    def test_no_file_means_plain_markers(self):
        self.assertEqual(emp.load_thumbnails(None, self.KNOWN), {})
        doc = emp.build(drafts=True, thumbnails=None)
        for a in doc["actuators"]:
            self.assertIsNone(a["thumbnail"], a["id"])

    def test_unknown_id_refused(self):
        self._raises(self._mesh(), "no record or draft record has that id",
                     "no_such_record")

    def test_kind_checked(self):
        e = self._mesh()
        e["kind"] = "photo"
        self._raises(e, "kind must be one of")

    def test_image_path_checked(self):
        for bad in ("thumbs/a.jpg", "../a.png", "/thumbs/a.png",
                    "thumbs\\a.png", "C:/a.png"):
            e = self._mesh()
            e["image"] = bad
            self._raises(e, "must be a .png path under site/")

    def test_missing_image_refused(self):
        e = self._mesh()
        e["image"] = "thumbs/not_drawn.png"
        self._raises(e, "is not there")

    def test_mesh_sources_checked(self):
        e = self._mesh()
        e["sources"] = []
        self._raises(e, "must list its sources")
        for key in ("repo", "commit", "path", "sha256"):
            e = self._mesh()
            del e["sources"][0][key]
            self._raises(e, "the source is missing")
        e = self._mesh()
        e["sources"][0]["sha256"] = "xyz"
        self._raises(e, "sha256")
        e = self._mesh()
        e["sources"][0]["draws"] = "paint"
        self._raises(e, "'draws' must be one of")
        e = self._mesh()
        e["sources"][0]["draws"] = "braid"
        self._raises(e, "exactly one source must draw the body")

    def test_schematic_needs_its_basis(self):
        for key in ("schematic", "basis"):
            e = {"kind": "schematic", "schematic": "pneunet",
                 "image": "thumbs/a.png", "basis": "As described."}
            del e[key]
            self._raises(e, f"has no '{key}'")


class SiteFile(unittest.TestCase):
    """Test 7: a fresh --site export equals site/data/map_paths.json on disk
    (compared as parsed JSON, so a line-ending change on checkout does not
    matter)."""

    def test_site_file_is_current(self):
        with open(emp.SITE_FILE, encoding="utf-8") as fo:
            on_disk = json.load(fo)
        self.assertTrue(on_disk["drafts_included"])
        self.assertEqual(json.loads(emp.render(DOC)), on_disk)


class OwnRecords(unittest.TestCase):
    """--record: a record file from outside the repository is worked out for
    the map page, and the stored entries are the same with or without it."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        src = next(a for a in DOC["actuators"]
                   if not a.get("hand_placed") and a.get("tasks"))
        with open(os.path.join(emp.ROOT, src["record_file"]),
                  encoding="utf-8") as fo:
            rec = json.load(fo)
        cls.stored_id = rec["id"]
        cls.stored = os.path.join(cls.tmp, "stored_id.json")
        with open(cls.stored, "w", encoding="utf-8") as fo:
            json.dump(rec, fo)
        rec["id"] = "own_record_for_the_test"
        cls.own = os.path.join(cls.tmp, "own.json")
        with open(cls.own, "w", encoding="utf-8") as fo:
            json.dump(rec, fo)
        cls.with_own = emp.build(drafts=True, own_records=[cls.own])

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp)

    def test_stored_entries_unchanged(self):
        doc = dict(self.with_own)
        del doc["own_records"]
        self.assertEqual(emp.render(doc), emp.render(DOC))
        self.assertNotIn("own_records", DOC)

    def test_own_entry(self):
        own = self.with_own["own_records"]
        self.assertEqual(own["left_out"], [])
        (a,) = own["actuators"]
        self.assertEqual(a["id"], "own_record_for_the_test")
        self.assertTrue(a["yours"])
        self.assertFalse(a["draft"])
        self.assertEqual(a["record_file"], "own.json")
        self.assertTrue(a["legs"])
        self.assertIn("own_record_for_the_test", own["scaled"])
        cones = {c["task"]: c for c in self.with_own["task_cones"]}
        for t in a["tasks"]:
            if "needed_direction" in t:
                self.assertAlmostEqual(
                    t["off_drawn_direction_deg"],
                    emp.angle_deg(t["needed_direction"],
                                  cones[t["task"]]["drawn_direction"]))

    def test_id_already_on_the_map_is_refused(self):
        with self.assertRaises(ValueError) as cm:
            emp.build(drafts=True, own_records=[self.stored])
        self.assertIn(f"'{self.stored_id}' is already on the map",
                      str(cm.exception))
        with self.assertRaises(ValueError):
            emp.build(drafts=True, own_records=[self.own, self.own])

    def test_command(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(emp.main(["--record", self.own, "--site"]), 1)
            self.assertEqual(emp.main(
                ["--record", os.path.join(self.tmp, "none.json"),
                 "--out", self.tmp]), 1)
            self.assertEqual(emp.main(
                ["--record", self.stored, "--out", self.tmp]), 1)
            args = ["--record", self.own, "--out", self.tmp, "--tag", "t"]
            self.assertEqual(emp.main(args), 0)
            self.assertEqual(emp.main(args), 1)
        said = out.getvalue()
        self.assertIn("REFUSING: --record is not used with --site", said)
        self.assertIn("no such file", said)
        self.assertIn("is already on the map", said)
        self.assertIn("REFUSING to overwrite:", said)
        self.assertEqual(os.listdir(self.tmp).count("map_own_records_t.json"),
                         1)
        with open(os.path.join(self.tmp, "map_own_records_t.json"),
                  encoding="utf-8") as fo:
            doc = json.load(fo)
        self.assertEqual(doc["kind"], emp.OWN_KIND)
        self.assertEqual(sorted(doc["mounts"]), ["aimed", "palm"])
        for mount, m in doc["mounts"].items():
            self.assertEqual([c["mount"] for c in m["task_cones"]], [mount])
            self.assertEqual([a["id"] for a in m["actuators"]],
                             ["own_record_for_the_test"])

    def test_page_reads_the_same_kind_and_placements(self):
        with open(os.path.join(emp.ROOT, "site", "map.html"),
                  encoding="utf-8") as fo:
            page = fo.read()
        self.assertIn('OWN_KIND = "%s"' % emp.OWN_KIND, page)
        for mount in emp.OWN_MOUNTS:
            self.assertIn("%s: { file:" % mount, page)


class Repeatable(unittest.TestCase):
    """Test 8: two runs give byte-identical output; an existing output is
    never overwritten."""

    def test_two_builds_same_text(self):
        self.assertEqual(emp.render(emp.build(drafts=True)).encode("utf-8"),
                         emp.render(DOC).encode("utf-8"))

    def test_two_runs_same_bytes_and_no_overwrite(self):
        tmp = tempfile.mkdtemp()
        try:
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                self.assertEqual(emp.main(["--drafts", "--out", tmp, "--tag", "a"]), 0)
                self.assertEqual(emp.main(["--drafts", "--out", tmp, "--tag", "b"]), 0)
                self.assertEqual(emp.main(["--drafts", "--out", tmp, "--tag", "a"]), 1)
            self.assertIn("REFUSING to overwrite:", out.getvalue())
            with open(os.path.join(tmp, "map_paths_with_drafts_a.json"), "rb") as f:
                a = f.read()
            with open(os.path.join(tmp, "map_paths_with_drafts_b.json"), "rb") as f:
                b = f.read()
            self.assertEqual(a, b)
            self.assertNotIn(b"\r\n", a)
        finally:
            shutil.rmtree(tmp)


class ExtraCheckPoints(unittest.TestCase):
    """data/map_extra_check_points.json: further simulated tip positions
    with both inputs on, used only to check a computed surface."""
    RID = "pneunet_tandem_twist_90deg"

    def _entry(self):
        return emp.load_extra_checks(emp.EXTRA_CHECKS, {self.RID})[self.RID]

    def _sidecar(self, change):
        with open(emp.EXTRA_CHECKS, encoding="utf-8") as fo:
            doc = json.load(fo)
        change(doc["records"][self.RID])
        fd, path = tempfile.mkstemp(suffix=".json")
        with os.fdopen(fd, "w", encoding="utf-8") as fo:
            json.dump(doc, fo)
        self.addCleanup(os.remove, path)
        return path

    def test_real_file_is_pinned_and_unchanged(self):
        with open(emp.EXTRA_CHECKS, encoding="utf-8") as fo:
            e = json.load(fo)["records"][self.RID]
        self.assertEqual(sorted(e), sorted(emp.EXTRA_KEYS))
        self.assertEqual(emp.sha256(os.path.join(emp.ROOT, *e["file"].split("/"))),
                         e["sha256"])

    def test_only_simulated_rows_are_read(self):
        with open(emp.EXTRA_CHECKS, encoding="utf-8") as fo:
            e = json.load(fo)["records"][self.RID]
        with open(os.path.join(emp.ROOT, *e["file"].split("/")),
                  encoding="utf-8", newline="") as fo:
            table = list(csv.DictReader(fo))
        used = [t for t in table if t["kind"] == "measured"]
        got = self._entry()
        self.assertEqual(len(got["rows"]), len(used))
        self.assertEqual(sum(got["left_out"].values()), len(table) - len(used))
        self.assertTrue(got["left_out"])        # the file does hold other rows
        self.assertEqual([0.0, 0.0, 0.0],
                         [r for r in got["rows"] if r[:2] == (0.0, 0.0)][0][2])

    def test_points_in_the_export(self):
        s = _by_id()[self.RID]["surface"]
        x = s["extra_checks"]
        rec = _all_records()[self.RID][0]
        held = {tuple(u) for u in rec["free"]["u"]}
        rows = {(p0, p1): xyz for p0, p1, xyz in self._entry()["rows"]}
        new = sorted(p for p in rows if p not in held and p[0] and p[1])
        self.assertEqual(sorted(tuple(t["inputs"]) for t in x["points"]), new)
        self.assertTrue(new)
        self.assertEqual(x["shared_rows_matched"], len(set(rows) & held))
        self.assertEqual(x["note"], emp.extra_checks_note(
            len(new), x["file"], x["left_out"]))
        for t in x["points"]:
            self.assertEqual([t["measured"][k] for k in XYZ],
                             rows[tuple(t["inputs"])])
            self.assertIsNone(t["measured"]["twist_rad"])
            self.assertIsNotNone(t["predicted"]["twist_rad"])
            m = np.array([t["measured"][k] for k in XYZ])
            q = np.array([t["predicted"][k] for k in XYZ])
            self.assertAlmostEqual(t["position_gap_m"],
                                   float(np.linalg.norm(m - q)), places=15)
        self.assertEqual(x["worst_position_gap_m"],
                         max(t["position_gap_m"] for t in x["points"]))
        self.assertEqual(x["worst_position_gap_share"],
                         max(t["position_gap_share"] for t in x["points"]))

    def test_the_rest_of_the_surface_is_untouched(self):
        rec = _all_records()[self.RID][0]
        with_file = emp.surface_of(rec, SERIES, extra=self._entry())
        without = emp.surface_of(rec, SERIES)
        self.assertIsNone(without["extra_checks"])
        for k in SURFACE_ORDER:
            if k != "extra_checks":
                self.assertEqual(with_file[k], without[k], k)
        # a build without the sidecar: no actuator carries extra checks
        doc = emp.build(drafts=True, extra_checks=None)
        for a in doc["actuators"]:
            if a["surface"]:
                self.assertIsNone(a["surface"]["extra_checks"])
        self.assertIsNone(_by_id()["pneunet_tandem_twist_60deg"]
                          ["surface"]["extra_checks"])

    def test_wrong_axes_stop_the_export(self):
        rec = _all_records()[self.RID][0]
        for signs in ([1, 1, 1], [1, -1, 1], [1, 1, -1], [-1, -1, -1]):
            path = self._sidecar(lambda e: e.update(to_record_axes=signs))
            entry = emp.load_extra_checks(path, {self.RID})[self.RID]
            with self.assertRaisesRegex(ValueError, "axes"):
                emp.surface_of(rec, SERIES, extra=entry)

    def test_too_few_shared_rows_stop_the_export(self):
        entry = self._entry()
        rec = _all_records()[self.RID][0]
        held = {tuple(u) for u in rec["free"]["u"]}
        # drop every row the record also holds, bar the rest row
        keep = [r for r in entry["rows"]
                if r[:2] == (0.0, 0.0) or r[:2] not in held]
        self.assertLess(len(keep), len(entry["rows"]))
        with self.assertRaisesRegex(ValueError, "needed to check its axes"):
            emp.surface_of(rec, SERIES, extra=dict(entry, rows=keep))

    def test_bad_entries(self):
        bad = [lambda e: e.update(sha256="0" * 64),
               lambda e: e.update(to_record_axes=[1, -1, 0]),
               lambda e: e.update(why=" "),
               lambda e: e.pop("use"),
               lambda e: e.update(extra_key=1),
               lambda e: e.update(inputs=["p1", "nope"]),
               lambda e: e.update(use={"column": "kind", "value": "filled in"})]
        for change in bad:
            with self.assertRaises(ValueError):
                emp.load_extra_checks(self._sidecar(change), {self.RID})
        with self.assertRaisesRegex(ValueError, "no record"):
            emp.load_extra_checks(emp.EXTRA_CHECKS, set())


class ReturnClear(unittest.TestCase):
    """The second check: after the stroke the finger must break contact and
    come back clear of the ball. Each result is recomputed here from the
    document's own legs and surface."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def _sidecar(self, change):
        with open(emp.TASK_CONES, encoding="utf-8") as fo:
            doc = json.load(fo)
        change(doc["cones"][0])
        p = os.path.join(self.tmp, f"cones_{len(os.listdir(self.tmp))}.json")
        with open(p, "w", encoding="utf-8") as fo:
            json.dump(doc, fo)
        return p

    @staticmethod
    def _at(al, no, s):
        out = []
        for a0, a1, n0, n1 in zip(al, al[1:], no, no[1:]):
            if a0 != a1 and min(a0, a1) <= s <= max(a0, a1):
                out.append(n0 + (s - a0) / (a1 - a0) * (n1 - n0))
        return out

    def test_cone_carries_the_declaration(self):
        with open(emp.TASK_CONES, encoding="utf-8") as fo:
            declared = json.load(fo)["cones"][0]["return_clear"]
        c = DOC["task_cones"][0]["return_clear"]
        self.assertEqual({k: v for k, v in c.items() if k != "stations"},
                         declared)
        self.assertEqual(c["stations"], emp.RETURN_STATIONS)
        self.assertEqual(sorted(declared), sorted(emp.RETURN_KEYS))
        self.assertIn("display", declared["drawn_depth_basis"])

    def test_results_recomputed(self):
        cone = DOC["task_cones"][0]
        travel, depth = cone["stroke"]["travel_m"], cone["return_clear"]["drawn_depth_m"]
        seen = {"no stroke path": 0, "one input": 0, "two inputs": 0}
        for a in DOC["actuators"]:
            if a["tasks"] is None:
                continue
            t = _task_entry(a)
            if "reason" in t:
                self.assertNotIn("return_clear", t, a["id"])
                continue
            r = t["return_clear"]
            passing = [i for i, g in enumerate(t["stroke"]["legs"])
                       if g["inside_to_end"]]
            if not passing:
                self.assertEqual(r, {"stroke_leg": None, "passes": None,
                                     "reason": emp.NO_STROKE_PATH}, a["id"])
                seen["no stroke path"] += 1
                continue
            self.assertEqual(r["stroke_leg"], passing[0], a["id"])
            e, n = np.array(t["needed_direction"]), np.array(t["contact_normal"])
            lines = []
            for leg in a["legs"]:
                v = leg["values"]
                P = np.array([[0.0, 0.0, 0.0]] + list(zip(
                    v["axial_m"], v["side_y_m"], v["side_z_m"])), float)
                lines.append((list(P @ e), list(P @ n)))
            if a["surface"] is not None:
                v = a["surface"]["values"]
                G = np.stack([np.array(v[k], float) for k in
                              ("axial_m", "side_y_m", "side_z_m")], -1)
                for row in list(G) + list(G.transpose(1, 0, 2)):
                    lines.append((list(row @ e), list(row @ n)))
            seen["two inputs" if a["surface"] is not None else "one input"] += 1
            self.assertEqual(len(r["stations_along_m"]), emp.RETURN_STATIONS)
            np.testing.assert_allclose(
                r["stations_along_m"],
                [travel * (j + 1) / emp.RETURN_STATIONS
                 for j in range(emp.RETURN_STATIONS)], rtol=0, atol=1e-12)
            al, no = lines[passing[0]]
            end = next(i for i, x in enumerate(al) if x >= travel)
            for s, out, inn in zip(r["stations_along_m"], r["outward_reach_m"],
                                   r["inward_reach_m"]):
                ref = self._at(al[:end + 1], no[:end + 1], s)[0]
                there = [x for la, ln in lines for x in self._at(la, ln, s)]
                self.assertAlmostEqual(out, max(ref - min(there), 0.0),
                                       places=12, msg=a["id"])
                self.assertAlmostEqual(inn, max(max(there) - ref, 0.0),
                                       places=12, msg=a["id"])
            self.assertEqual(r["least_outward_reach_m"], min(r["outward_reach_m"]))
            self.assertEqual(r["furthest_inward_reach_m"], max(r["inward_reach_m"]))
            self.assertIs(r["passes"], r["least_outward_reach_m"] >= depth)
        for k, v in seen.items():
            self.assertGreater(v, 0, k)

    def test_verdicts_on_the_records_held(self):
        by = {a["id"]: a for a in DOC["actuators"]}
        for a in DOC["actuators"]:
            if a["tasks"] is None or "reason" in _task_entry(a):
                continue
            r = _task_entry(a)["return_clear"]
            if r["stroke_leg"] is None:
                continue
            # nothing held passes, and no verdict hangs on the drawn depth:
            # every least outward reach is under a hundredth of a millimetre
            self.assertIs(r["passes"], False, a["id"])
            self.assertLess(r["least_outward_reach_m"], 1e-5, a["id"])
            if a["surface"] is None and len(a["legs"]) == 1:
                self.assertEqual(max(r["outward_reach_m"]), 0.0, a["id"])
        for rid in TANDEM:
            r = _task_entry(by[rid])["return_clear"]
            self.assertEqual(r["stroke_leg"], 0)
            self.assertIn("surface", r["positions_read_from"])
            # the second input takes the tip further into the ball, not out
            self.assertGreater(r["furthest_inward_reach_m"], 1e-3, rid)

    def test_verdicts_do_not_hang_on_the_drawn_depth(self):
        doc = emp.build(drafts=True, task_cones=self._sidecar(
            lambda c: c["return_clear"].update(drawn_depth_m=1.1e-5)))
        self.assertEqual(doc["task_cones"][0]["return_clear"]["drawn_depth_m"],
                         1.1e-5)
        for a, b in zip(DOC["actuators"], doc["actuators"]):
            self.assertEqual(a["tasks"], b["tasks"], a["id"])

    def test_dropping_it_changes_nothing_else(self):
        doc = emp.build(drafts=True, task_cones=self._sidecar(
            lambda c: c.pop("return_clear")))
        self.assertIsNone(doc["task_cones"][0]["return_clear"])
        for a, b in zip(DOC["actuators"], doc["actuators"]):
            if a["tasks"] is None:
                self.assertIsNone(b["tasks"])
                continue
            ta, tb = _task_entry(a), _task_entry(b)
            self.assertNotIn("return_clear", tb, a["id"])
            self.assertEqual({k: v for k, v in ta.items()
                              if k != "return_clear"}, tb, a["id"])
            self.assertEqual({k: v for k, v in a.items() if k != "tasks"},
                             {k: v for k, v in b.items() if k != "tasks"})

    def test_made_up_finger_that_can_lift(self):
        def leg(y):
            return {"values": {"axial_m": [0.0, 10.0], "side_y_m": [y, y],
                               "side_z_m": [0.0, 0.0]}}
        t = {"needed_direction": [1.0, 0.0, 0.0],
             "contact_normal": [0.0, 1.0, 0.0],
             "stroke": {"legs": [{"inside_to_end": True},
                                 {"inside_to_end": False}]}}
        # the second leg sits 3 further out of the object all along
        r = emp.return_clear_of([leg(0.0), leg(-3.0)], None, t,
                                {"drawn_depth_m": 2.0}, 8.0)
        self.assertEqual(r["stroke_leg"], 0)
        self.assertEqual(r["stations_along_m"][-1], 8.0)
        np.testing.assert_allclose(r["outward_reach_m"], 3.0)
        np.testing.assert_allclose(r["inward_reach_m"], 0.0)
        self.assertIs(r["passes"], True)
        self.assertIs(emp.return_clear_of([leg(0.0), leg(-3.0)], None, t,
                                          {"drawn_depth_m": 4.0}, 8.0)["passes"],
                      False)
        # further in is not further out
        r = emp.return_clear_of([leg(0.0), leg(3.0)], None, t,
                                {"drawn_depth_m": 2.0}, 8.0)
        np.testing.assert_allclose(r["outward_reach_m"], 0.0)
        np.testing.assert_allclose(r["inward_reach_m"], 3.0)
        self.assertIs(r["passes"], False)
        # no leg passes the first check: no stroke path
        t["stroke"]["legs"][0]["inside_to_end"] = False
        self.assertEqual(emp.return_clear_of([leg(0.0), leg(-3.0)], None, t,
                                             {"drawn_depth_m": 2.0}, 8.0),
                         {"stroke_leg": None, "passes": None,
                          "reason": emp.NO_STROKE_PATH})

    def test_bad_declarations_refused(self):
        bad = [lambda c: c.update(return_clear=[0.002]),
               lambda c: c["return_clear"].update(drawn_depth_m=0),
               lambda c: c["return_clear"].update(drawn_depth_m=-0.002),
               lambda c: c["return_clear"].update(drawn_depth_m="0.002"),
               lambda c: c["return_clear"].update(drawn_depth_m=True),
               lambda c: c["return_clear"].update(basis=" "),
               lambda c: c["return_clear"].update(drawn_depth_basis=""),
               lambda c: c["return_clear"].update(extra_key=1),
               lambda c: c["return_clear"].pop("basis"),
               lambda c: c.pop("stroke")]
        for i, change in enumerate(bad):
            with self.subTest(case=i):
                with self.assertRaises(ValueError):
                    emp.load_task_cones(self._sidecar(change))


class ReturnClearFromOwnStart(unittest.TestCase):
    """The second check where the cone declares "stroke_from": "own_start"
    (data/map_task_cones_palm_mount.json: the finger standing on the palm
    beside the ball). Each result is recomputed here from the document's
    own legs and surface. The page's own document does not use it."""

    PALM = os.path.join(emp.ROOT, "data", "map_task_cones_palm_mount.json")

    @classmethod
    def setUpClass(cls):
        cls.doc = emp.build(drafts=True, task_cones=cls.PALM)

    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def _sidecar(self, change):
        with open(self.PALM, encoding="utf-8") as fo:
            doc = json.load(fo)
        change(doc["cones"][0])
        p = os.path.join(self.tmp, f"cones_{len(os.listdir(self.tmp))}.json")
        with open(p, "w", encoding="utf-8") as fo:
            json.dump(doc, fo)
        return p

    @staticmethod
    def _lines(a):
        """Every line of tip positions the document holds for one
        actuator: (what, held input, held level, points)."""
        out = []
        for i, leg in enumerate(a["legs"]):
            v = leg["values"]
            P = [p for p in zip(v["axial_m"], v["side_y_m"], v["side_z_m"])
                 if None not in p]
            out.append(("leg", i, None, np.array([(0.0, 0.0, 0.0)] + P)))
        if a["surface"] is not None:
            v, ins = a["surface"]["values"], a["surface"]["inputs"]
            G = np.stack([np.array(v[k], float) for k in
                          ("axial_m", "side_y_m", "side_z_m")], -1)
            for i, level in enumerate(ins[0]["values"]):
                out.append(("surface line", ins[0]["name"], level, G[i]))
            for j, level in enumerate(ins[1]["values"]):
                out.append(("surface line", ins[1]["name"], level, G[:, j]))
        return out

    @staticmethod
    def _normal_at(P, e, n, s):
        """Travel along n wherever the line through P is at travel s
        along e."""
        al, no = P @ e, P @ n
        return [n0 + (s - a0) / (a1 - a0) * (n1 - n0)
                for a0, a1, n0, n1 in zip(al, al[1:], no, no[1:])
                if a0 != a1 and min(a0, a1) <= s <= max(a0, a1)]

    def _entries(self):
        for a in self.doc["actuators"]:
            if a["tasks"] is None:
                continue
            t = _task_entry(a)
            if "reason" not in t:
                yield a, t, t["return_clear"]

    def test_declaration(self):
        with open(self.PALM, encoding="utf-8") as fo:
            declared = json.load(fo)["cones"][0]
        with open(emp.TASK_CONES, encoding="utf-8") as fo:
            page = json.load(fo)["cones"][0]
        c = self.doc["task_cones"][0]
        self.assertEqual(c["mount"], "palm")
        self.assertEqual(c["return_clear"]["stroke_from"],
                         emp.STROKE_FROM_OWN_START)
        self.assertEqual({k: v for k, v in c["return_clear"].items()
                          if k != "stations"}, declared["return_clear"])
        self.assertIsNone(c["push"])
        # the same task, finger count, yaw, angle, stroke and depth as the
        # page's own declaration
        for k in ("task", "n", "yaw_deg", "half_angle_deg"):
            self.assertEqual(declared[k], page[k], k)
        self.assertEqual(declared["stroke"]["angle_deg"],
                         page["stroke"]["angle_deg"])
        self.assertEqual(declared["stroke"]["source"], page["stroke"]["source"])
        self.assertEqual(declared["return_clear"]["drawn_depth_m"],
                         page["return_clear"]["drawn_depth_m"])

    def test_the_pages_own_document_does_not_use_it(self):
        self.assertNotIn("stroke_from", DOC["task_cones"][0]["return_clear"])
        self.assertEqual(DOC["task_cones"][0]["mount"], "aimed")
        for a in DOC["actuators"]:
            for t in a["tasks"] or []:
                self.assertNotIn("stroke_from", t.get("return_clear") or {},
                                 a["id"])

    def test_results_recomputed(self):
        cone = self.doc["task_cones"][0]
        travel, half = cone["stroke"]["travel_m"], cone["half_angle_deg"]
        depth = cone["return_clear"]["drawn_depth_m"]
        seen = {"no stroke line": 0, "one input": 0, "two inputs": 0}
        for a, t, r in self._entries():
            e, n = np.array(t["needed_direction"]), np.array(t["contact_normal"])
            lines = self._lines(a)
            # every line that can be a stroke path, judged from its own start
            strokes = []
            for what, held, level, P in lines:
                Q = P - P[0]
                k = next((i for i, x in enumerate(Q @ e) if x >= travel), None)
                if not k:
                    continue
                worst = max(np.degrees(np.arccos(np.clip(
                    (q @ e) / np.linalg.norm(q), -1.0, 1.0)))
                    for q in Q[1:k + 1] if np.linalg.norm(q) > 0)
                if worst <= half:
                    strokes.append((what, held, level, P[:k + 1], worst))
            if not strokes:
                self.assertEqual(r, {"stroke_from": "own_start",
                                     "stroke_line": None, "passes": None,
                                     "reason": emp.NO_STROKE_LINE}, a["id"])
                seen["no stroke line"] += 1
                continue
            seen["two inputs" if a["surface"] is not None else "one input"] += 1
            best = -np.inf
            for what, held, level, P, worst in strokes:
                start = float(P[0] @ e)
                st = [start + travel * (j + 1) / emp.RETURN_STATIONS
                      for j in range(emp.RETURN_STATIONS)]
                st[-1] = min(st[-1], float(P[-1] @ e))
                ref = [self._normal_at(P, e, n, s)[0] for s in st]
                for _, _, _, R in lines:
                    back = [self._normal_at(R, e, n, s) for s in [start] + st]
                    if not all(back):
                        continue
                    best = max(best, min(x - max(b) for x, b in
                                         zip(ref, back[1:])))
            self.assertAlmostEqual(r["least_outward_reach_m"], max(best, 0.0),
                                   places=12, msg=a["id"])
            self.assertEqual(r["least_outward_reach_m"],
                             min(r["outward_reach_m"]), a["id"])
            self.assertIs(r["passes"], bool(max(best, 0.0) >= depth), a["id"])
            # the two lines reported, and the gap between their own points
            S = np.array(r["stroke_line"]["points_m"])
            B = np.array(r["return_line"]["points_m"])
            self.assertEqual(len(S), emp.RETURN_STATIONS + 1)
            self.assertEqual(len(B), emp.RETURN_STATIONS + 1)
            np.testing.assert_allclose(S[0], r["stroke_start_m"], atol=1e-15)
            np.testing.assert_allclose(S[1:] @ e, r["stations_along_m"],
                                       atol=1e-12)
            np.testing.assert_allclose(B[1:] @ e, r["stations_along_m"],
                                       atol=1e-12)
            np.testing.assert_allclose(
                np.maximum((S[1:] - B[1:]) @ n, 0.0), r["outward_reach_m"],
                atol=1e-12)
            self.assertAlmostEqual((S[-1] - S[0]) @ e, travel, places=12)
            self.assertLessEqual(r["stroke_worst_angle_deg"], half)
        self.assertGreater(seen["no stroke line"], 0)
        self.assertGreater(seen["one input"], 0)
        self.assertGreater(seen["two inputs"], 0)

    def test_verdicts_on_the_records_held(self):
        """One input cannot come back clear. The two fingers of two
        pneu-nets can: they stroke with the far chamber at its top level
        and come back along the path with it off."""
        passers = []
        for a, t, r in self._entries():
            if r["passes"] is None:
                continue
            if r["passes"]:
                passers.append(a["id"])
            if a["surface"] is None:
                self.assertEqual(r["least_outward_reach_m"], 0.0, a["id"])
                self.assertIs(r["passes"], False, a["id"])
                continue
            ins = a["surface"]["inputs"]
            s, b = r["stroke_line"], r["return_line"]
            self.assertEqual(s["what"], "surface line", a["id"])
            self.assertEqual(s["held_input"], ins[1]["name"], a["id"])
            self.assertEqual(s["held_at"], max(ins[1]["values"]), a["id"])
            self.assertEqual(s["swept_input"], ins[0]["name"], a["id"])
            self.assertEqual(b["what"], "leg", a["id"])
            self.assertEqual(a["legs"][b["leg"]]["input"]["name"],
                             ins[0]["name"], a["id"])
            self.assertGreater(r["stroke_start_into_object_m"], 0.0, a["id"])
        self.assertEqual(sorted(passers), ["pneunet_tandem_twist_60deg",
                                           "pneunet_tandem_twist_90deg"])

    def test_first_check_from_rest_still_holds_for_every_stroke_line(self):
        """On the records held, every actuator given a stroke line also
        passes the first check as the page defines it (from rest), so the
        new reading adds no passer of the first check."""
        for a, t, r in self._entries():
            self.assertEqual(r["passes"] is not None,
                             t["stroke"]["inside_to_end"], a["id"])

    def test_verdicts_do_not_hang_on_the_drawn_depth(self):
        """Every pass or fail is the same for any depth from a hundredth
        of a millimetre up to 7 mm, as the declaration says."""
        for a, t, r in self._entries():
            if r["passes"] is None:
                continue
            least = r["least_outward_reach_m"]
            self.assertTrue(least < 1e-5 or least > 7e-3, (a["id"], least))

    def test_gap_is_far_larger_than_the_surface_error(self):
        """The stroke line lies on a surface worked out from the two
        single-input paths. Where the document holds simulated points with
        both inputs on, they differ from that surface, along the contact
        normal, by under a hundredth of the passing gap."""
        checked = 0
        for a, t, r in self._entries():
            if not r["passes"]:
                continue
            n = np.array(t["contact_normal"])
            s = a["surface"]
            pts = list(s["tests"]) + list((s.get("extra_checks") or {})
                                          .get("points") or [])
            self.assertTrue(pts, a["id"])
            for x in pts:
                d = np.array([x["measured"][k] - x["predicted"][k] for k in
                              ("axial_m", "side_y_m", "side_z_m")])
                self.assertLess(abs(d @ n), r["least_outward_reach_m"] / 100,
                                (a["id"], x["inputs"]))
                checked += 1
        self.assertGreater(checked, 0)

    def test_loader_rejects_a_bad_stroke_from(self):
        def no_basis(c):
            del c["return_clear"]["stroke_from_basis"]

        def unknown(c):
            c["return_clear"]["stroke_from"] = "anywhere"

        def empty_basis(c):
            c["return_clear"]["stroke_from_basis"] = " "

        def stray(c):
            c["return_clear"]["other"] = 1

        for change in (no_basis, unknown, empty_basis, stray):
            with self.assertRaises(ValueError, msg=change.__name__):
                emp.load_task_cones(self._sidecar(change))

    def test_cli_refuses_another_cone_file_for_the_site(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(emp.main(["--site", "--task-cones", self.PALM]), 1)


if __name__ == "__main__":
    unittest.main()
