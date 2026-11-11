"""follow.py says which joint falls short and which test a finger missed,
and the verdict takes no account of it (issue 29, 10 Nov): the finger's own
torque test row by row on made-up columns, single joint type and mixed
(the rows' norm is the unbalanced torque the test reports, the tolerance
its bar); the redistribution's rows cut per finger; the whole-hand torque
stack's rows on a made-up stack, recomputed from the solution's cone
weights and the input use it reports (with and without a stop column and
each finger's own torque), and within the bar when the stack passes; the
joint kind; and, on
stored self-hold results whose hand models are in the repository, that
every touching finger carries the keys, that the rows agree with the
stored unbalanced torque, that "missed" agrees with the finger's state and
reason, and that the stored verdicts, states and reasons are reproduced
(python -m unittest). Needs mujoco; without it the model tests are
skipped."""
import json
import os
import sys
import types
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from gramian_workshop import follow as fo  # noqa: E402
from gramian_workshop.vendor import capability_core as cap  # noqa: E402

try:
    import mujoco
except ImportError:
    mujoco = None

XML = """
<mujoco>
  <worldbody>
    <body name="a">
      <joint name="h0" type="hinge" axis="0 0 1"/>
      <geom type="capsule" fromto="0 0 0 0.03 0 0" size="0.005"/>
      <body name="b" pos="0.03 0 0">
        <joint name="s1" type="slide" axis="1 0 0"/>
        <geom type="capsule" fromto="0 0 0 0.02 0 0" size="0.005"/>
        <body name="c" pos="0.02 0 0">
          <joint name="h2" type="hinge" axis="0 1 0"/>
          <geom type="capsule" fromto="0 0 0 0.02 0 0" size="0.005"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def _load(p):
    with open(p, encoding="utf-8") as fh:
        return json.load(fh)


class Rows(unittest.TestCase):
    """The finger's own rows on made-up columns (no mujoco: the hand is a
    stand-in whose joint names and kinds are given)."""
    def setUp(self):
        rng = np.random.default_rng(7)
        self.Tr = rng.normal(size=(3, 3))
        self.C = rng.normal(size=(3, 2))          # fewer inputs than rows: a residual is left
        self.fv = rng.normal(size=3)
        names = ["h0", "s1", "h2"]
        kinds = ["hinge", "slide", "hinge"]
        model = types.SimpleNamespace(joint=lambda j: types.SimpleNamespace(name=names[j]))
        self.hand = types.SimpleNamespace(model=model)
        self.ik = types.SimpleNamespace(free=[0, 1, 2])
        self.kinds = kinds
        fo._joint_kind, self._real_kind = (lambda m, j: kinds[j]), fo._joint_kind

    def tearDown(self):
        fo._joint_kind = self._real_kind

    def test_single_type_rows_norm_is_the_unbalanced_torque(self):
        cols = (self.Tr, self.C)
        ok, un, x = fo._hold_solve(cols, self.fv)
        rows = fo._own_residuals(self.hand, self.ik, cols, self.fv, None, x, None)
        self.assertEqual([r["joint"] for r in rows], ["h0", "s1", "h2"])
        self.assertEqual([r["kind"] for r in rows], self.kinds)
        self.assertAlmostEqual(np.linalg.norm([r["residual"] for r in rows]), un, places=12)
        bar = fo.UNACT_TOL * np.linalg.norm(self.Tr @ self.fv) + 1e-9
        self.assertEqual({r["tolerance"] for r in rows}, {bar})
        self.assertEqual(ok, un <= bar)
        self.assertGreater(un, 0.0)                 # the case is not trivially balanced

    def test_tau_is_part_of_the_target(self):
        cols = (self.Tr, self.C)
        tau = np.array([0.1, -0.2, 0.3])
        _, un, x = fo._hold_solve(cols, self.fv, tau)
        rows = fo._own_residuals(self.hand, self.ik, cols, self.fv, tau, x, None)
        self.assertAlmostEqual(np.linalg.norm([r["residual"] for r in rows]), un, places=12)
        t = self.Tr @ self.fv + tau
        self.assertAlmostEqual(rows[0]["tolerance"], fo.UNACT_TOL * np.linalg.norm(t) + 1e-9)
        # the row really is C x - t
        np.testing.assert_allclose([r["residual"] for r in rows], self.C @ x - t)

    def test_mixed_types_each_in_its_own_unit(self):
        cols = (self.Tr, self.C)
        ok, un, x = fo._hold_solve(cols, self.fv, kinds=self.kinds)
        self.assertIsInstance(un, dict)
        rows = fo._own_residuals(self.hand, self.ik, cols, self.fv, None, x, self.kinds)
        t = self.Tr @ self.fv
        for kd in ("hinge", "slide"):
            sel = [r for r in rows if r["kind"] == kd]
            self.assertAlmostEqual(np.linalg.norm([r["residual"] for r in sel]), un[kd], places=12)
            bar = fo.UNACT_TOL * np.linalg.norm(t[np.asarray(self.kinds) == kd]) + 1e-9
            self.assertEqual({r["tolerance"] for r in sel}, {bar})
        self.assertEqual(ok, all(un[k] <= [r for r in rows if r["kind"] == k][0]["tolerance"]
                                 for k in un))

    def test_split_rows_cuts_by_each_fingers_rows(self):
        Js = [(np.zeros((2, 3)), np.zeros((2, 4))), None, (np.zeros((3, 3)), np.zeros((3, 1)))]
        r = np.arange(5.0)
        out = fo._split_rows(Js, [0, 1, 2], r)
        self.assertEqual(set(out), {0, 2})
        np.testing.assert_array_equal(out[0], [0.0, 1.0])
        np.testing.assert_array_equal(out[2], [2.0, 3.0, 4.0])


@unittest.skipIf(mujoco is None, "mujoco not installed")
class Kinds(unittest.TestCase):
    def test_joint_kind_and_rows(self):
        m = mujoco.MjModel.from_xml_string(XML)
        self.assertEqual([fo._joint_kind(m, j) for j in range(3)], ["hinge", "slide", "hinge"])
        hand = types.SimpleNamespace(model=m)
        ik = types.SimpleNamespace(free=[0, 2])         # the slide coupled away: no row
        self.assertEqual(fo._joint_rows(hand, ik),
                         [{"joint": "h0", "kind": "hinge"}, {"joint": "h2", "kind": "hinge"}])


@unittest.skipIf(mujoco is None, "mujoco not installed")
class StoredHolds(unittest.TestCase):
    """Stored self-hold results (the leaders with the self-overlap fix,
    made with the end stops credited) of hands whose models are in the
    repository, re-scored with their stored placements."""
    HANDS = ("george_clip", "wall_x3_ring")
    _kept = {}

    @classmethod
    def _run(cls, key):
        if key not in cls._kept:
            stored = _load(os.path.join(ROOT, "outputs",
                                        f"follow_{key}__hold_{key}__self__selfoverlapfix__fixed.json"))
            leader = _load(os.path.join(ROOT, "data", "leaders", f"hold_{key}__self__selfoverlapfix.json"))
            fixed = {k: stored[k] for k in ("placement", "assignment", "respace_deg", "placement_cost_mm")}
            on = bool(stored["settings"]["stop_reactions"]["on"])
            cls._kept[key] = (stored, fo.follow(key, leader, placement=fixed, margin=True, stop_reactions=on))
        return cls._kept[key]

    def _touching(self, r):
        for fr in r["frames"]:
            for x in fr["fingers"]:
                if x.get("state") not in (None, "off"):
                    yield fr, x

    def test_every_touching_finger_carries_the_keys(self):
        for key in self.HANDS:
            _, r = self._run(key)
            n = 0
            for fr, x in self._touching(r):
                n += 1
                self.assertIn("missed", x, key)
                if x.get("state") == "red" and not x.get("used_in_force_check"):
                    continue                        # carries nothing: no force tests
                self.assertIn("cone_angle_deg", x, key)
                self.assertIn("cone_half_angle_deg", x, key)
                if x.get("torque_checked"):
                    rows = x["joint_residuals"]["finger"]
                    self.assertTrue(rows, key)
                    for row in rows:
                        self.assertEqual(set(row), {"joint", "kind", "residual", "tolerance"})
                        self.assertIn(row["kind"], ("hinge", "slide"))
            self.assertGreater(n, 0, key)

    def test_rows_agree_with_the_stored_unbalanced(self):
        for key in self.HANDS:
            _, r = self._run(key)
            for fr, x in self._touching(r):
                if not x.get("torque_checked"):
                    continue
                rows = x["joint_residuals"]["finger"]
                by = {}
                for row in rows:
                    by.setdefault(row["kind"], []).append(row["residual"])
                if "unbalanced_N" in x:             # both joint types, each in its own unit
                    self.assertAlmostEqual(np.linalg.norm(by["hinge"]), x["unbalanced_Nm"], places=9, msg=key)
                    self.assertAlmostEqual(np.linalg.norm(by["slide"]), x["unbalanced_N"], places=9, msg=key)
                else:
                    self.assertAlmostEqual(np.linalg.norm([row["residual"] for row in rows]),
                                           x["unbalanced_Nm"], places=9, msg=key)
                # the row's test is the one "missed" reports
                over = any(np.linalg.norm(v) > [row for row in rows if row["kind"] == k][0]["tolerance"]
                           for k, v in by.items())
                self.assertEqual("torque" in x["missed"], over, (key, x["finger"]))

    def test_missed_agrees_with_state_and_reason(self):
        first = {"reach": "reach", "nail side": "facing", "penetrates": "overlap"}
        for key in self.HANDS:
            _, r = self._run(key)
            for fr, x in self._touching(r):
                if x["state"] == "green":
                    self.assertEqual(x["missed"], [], (key, x["finger"]))
                elif x["state"] == "red":
                    self.assertIn(first[x["reason"]], x["missed"], (key, x["finger"]))
                else:
                    self.assertTrue(x["missed"], (key, x["finger"]))
                self.assertEqual(x["missed"], [t for t in ("reach", "facing", "overlap", "friction cone", "torque")
                                               if t in x["missed"]])         # test order

    def test_redistribution_rows_when_the_stack_ran(self):
        # the ring hand's hold is amber: the whole-hand force test ran
        _, r = self._run("wall_x3_ring")
        seen = 0
        for fr, x in self._touching(r):
            red = x.get("joint_residuals", {}).get("redistribution")
            if red is None:
                continue
            seen += 1
            self.assertEqual([(d["joint"], d["kind"]) for d in red],
                             [(d["joint"], d["kind"]) for d in x["joint_residuals"]["finger"]])
            self.assertEqual(len({d["tolerance"] for d in red}), 1)
            self.assertGreater(red[0]["tolerance"], 0.0)
        self.assertGreater(seen, 0)
        # the stack held the norm of its whole residual, these rows among
        # them, to that bar: where it passed (the frame's verdict is not a
        # torque red), the norm of every finger's rows is within it
        passed = 0
        for fr in r["frames"]:
            rows = [d for x in fr["fingers"]
                    for d in x.get("joint_residuals", {}).get("redistribution", [])]
            if not rows or fr["verdict"] == "red":
                continue
            passed += 1
            self.assertLessEqual(np.linalg.norm([d["residual"] for d in rows]),
                                 rows[0]["tolerance"] + 1e-12)
        self.assertGreater(passed, 0)

    def test_beyond_limit_and_stops_carry_the_kind(self):
        for key in self.HANDS:
            _, r = self._run(key)
            for fr, x in self._touching(r):
                for e in x.get("beyond_limit", []) + x.get("stops", []):
                    self.assertIn(e["kind"], ("hinge", "slide"), key)

    def test_the_stored_verdict_stands(self):
        for key in self.HANDS:
            stored, r = self._run(key)
            self.assertEqual([(f["k"], f["verdict"], f["reason"]) for f in r["frames"]],
                             [(f["k"], f["verdict"], f["reason"]) for f in stored["frames"]], key)
            self.assertEqual([[(x.get("state"), x.get("reason")) for x in f["fingers"]] for f in r["frames"]],
                             [[(x.get("state"), x.get("reason")) for x in f["fingers"]] for f in stored["frames"]], key)

    def test_nothing_else_changed(self):
        # every key the stored result has reads the same; the keys the run
        # adds are this issue's (and the per-frame self overlap of issue 27,
        # which these results were stored before)
        allowed = {"joint_residuals", "missed", "cone_angle_deg", "cone_half_angle_deg",
                   "kind", "self_overlap"}
        for key in self.HANDS:
            stored, r = self._run(key)
            a, b = json.loads(json.dumps(stored)), json.loads(json.dumps(r))
            added = set()

            def trim(x, y):
                for kk in list(y):
                    if kk not in x:
                        added.add(kk)
                        y.pop(kk)
            for x in (a, b):
                x.pop("elapsed_s", None)
            trim(a, b)
            for fa, fb in zip(a["frames"], b["frames"]):
                trim(fa, fb)
                for ga, gb in zip(fa["fingers"], fb["fingers"]):
                    trim(ga, gb)
                    for ea, eb in zip(ga.get("beyond_limit", []) + ga.get("stops", []),
                                      gb.get("beyond_limit", []) + gb.get("stops", [])):
                        trim(ea, eb)
            self.assertLessEqual(added, allowed, key)
            self.assertEqual(json.dumps(a, sort_keys=True), json.dumps(b, sort_keys=True), key)


class Redistribution(unittest.TestCase):
    """The whole-hand torque stack's joint rows on a made-up stack (no
    mujoco): three contacts, a finger with a hinge-and-slide pair of rows,
    one with three rows, one with no torque check. The rows _redistribute
    reports are recomputed here from the cone weights of the same solve and
    the input use it reports: Tr_a F_a lambda + tau_a - C_a x_a, in the
    rows' own units, with and without a stop column and each finger's own
    torque. Its bar is RESID_TOL of the scaled wrench (or FEAS_REL), back in
    those units, and when the stack passes the rows' norm is within it."""
    def setUp(self):
        rng = np.random.default_rng(29)
        self.pts = [np.array(p) for p in ([0.03, 0.0, 0.0], [-0.02, 0.02, 0.01], [0.0, -0.03, -0.01])]
        self.nrms = [-p / np.linalg.norm(p) for p in self.pts]      # inward
        self.c = np.zeros(3)
        self.mu, self.L = 0.5, 0.1
        self.Js = [(rng.normal(size=(2, 3)), rng.normal(size=(2, 3))), None,
                   (rng.normal(size=(3, 3)), rng.normal(size=(3, 2)))]
        self.rng = rng

    def _rows(self, W, taus, bounds):
        """_redistribute's verdict and reported rows, and the cone weights
        of the solve it ran (the same deterministic solve, re-run here)."""
        out = {}
        ok, why = fo._redistribute(self.pts, self.nrms, self.Js, self.c, W, self.mu, self.L,
                                   taus=taus, bounds=bounds, out=out)
        self.assertIn("joint_residual", out, why)
        ws = cap.scale_wrench(W, self.L)
        Gs, cidx, Fc = cap.generators(self.pts, self.nrms, self.c, self.mu, self.L)
        Mx, nt, nx, ab = fo._torque_stack(Gs, cidx, Fc, self.Js, self.L, bounds)
        if bounds is None:
            jr = np.zeros(nt) if taus is None else fo._joint_rhs(self.Js, taus, self.L)
            span = np.ones(nx)
        else:
            jr, span, _ = fo._stack_rhs(self.Js, taus, bounds, self.L)
        tol = max(fo.RESID_TOL * float(np.linalg.norm(ws)), cap.FEAS_REL)
        ok2, _, xs = fo._feasible_x(Mx, np.concatenate([ws, jr, span]), tol, fo._stops_in(bounds))
        self.assertEqual(ok, ok2)
        self.assertAlmostEqual(out["joint_tol"], tol * self.L)
        return ok, out, xs[:Gs.shape[1]], cidx, Fc

    def _check(self, W, taus, bounds):
        ok, out, lam, cidx, Fc = self._rows(W, taus, bounds)
        on = list(range(len(self.Js)))
        rows = fo._split_rows(self.Js, on, np.asarray(out["joint_residual"]))
        xs = fo._split_stack(self.Js, on, np.asarray(out["x"]))
        self.assertEqual(set(rows), {0, 2})
        for a in rows:
            Tr, C = self.Js[a]
            f = Fc[:, cidx == a] @ lam[cidx == a]          # the contact's force
            want = Tr @ f - C @ xs[a]
            if taus is not None and taus[a] is not None:
                want = want + np.asarray(taus[a])
            self.assertEqual(len(rows[a]), Tr.shape[0])
            np.testing.assert_allclose(rows[a], want, atol=1e-9)
        if ok:
            self.assertLessEqual(np.linalg.norm(out["joint_residual"]), out["joint_tol"] + 1e-12)
        return ok

    def _feasible_wrench(self, taus, bounds):
        """A wrench the stack can carry exactly: cone weights picked, and each
        finger's first column fixed so its box holds the balance."""
        Gs, cidx, Fc = cap.generators(self.pts, self.nrms, self.c, self.mu, self.L)
        lam = self.rng.uniform(0.2, 1.0, Gs.shape[1])
        W = Gs @ lam
        W[3:] *= self.L
        for a in (0, 2):
            Tr, C = self.Js[a]
            t = Tr @ (Fc[:, cidx == a] @ lam[cidx == a])
            if taus is not None and taus[a] is not None:
                t = t + np.asarray(taus[a])
            l, h = (np.zeros(C.shape[1]), np.ones(C.shape[1])) if bounds is None or bounds[a] is None else bounds[a]
            x0 = np.where(np.isfinite(h), l + 0.5 * (np.asarray(h) - l), l + 0.5)
            C[:, 0] += (t - C @ x0) / x0[0]
        return W

    def test_rows_recomputed_from_the_solution(self):
        # an arbitrary wrench (the stack most likely fails): the rows are
        # the joint balance at the reported input use, in N.m / N
        W = self.rng.normal(size=6) * [1.0, 1.0, 1.0, 0.01, 0.01, 0.01]
        self._check(W, None, None)

    def test_rows_with_a_stop_column_and_own_torque(self):
        # finger 2's third column is a stop (unbounded; _stop_box), finger 0
        # has a torque its own motion needs: the rows carry both
        bounds = [(np.zeros(3), np.ones(3)), None, fo._stop_box(2, 1)]
        taus = [np.array([0.002, -0.001]), None, None]
        W = self.rng.normal(size=6) * [1.0, 1.0, 1.0, 0.01, 0.01, 0.01]
        self._check(W, taus, bounds)

    def test_rows_within_the_bar_when_the_stack_passes(self):
        self.assertTrue(self._check(self._feasible_wrench(None, None), None, None))
        bounds = [(np.zeros(3), np.ones(3)), None, fo._stop_box(2, 1)]
        taus = [np.array([0.002, -0.001]), None, None]
        self.assertTrue(self._check(self._feasible_wrench(taus, bounds), taus, bounds))


if __name__ == "__main__":
    unittest.main()
