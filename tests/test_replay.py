"""replay.py (issue 14): the pieces that can be checked without a hand model
(rotation helpers, the pad stiffnesses on a leader worked by hand, the
verdicts, the joint path, the station servo's parking, the runner's
guards), the physics of the harness on small models (free fall, contact
force sign, the pad stiffness reached), and, with the menagerie Allegro and
the stored continuity result, the model and feedforward invariants: the
path hits the stored poses at the checked times, the weight fed forward is
the check's own, the contact load is J^T f, lifted fingers stand clear, the
hand alone tracks its path, and the check's path readout is as stored
(python -m unittest). The Allegro tests need mujoco, GMW_MENAGERIE and the
stored result, and are skipped without them."""
import contextlib
import io
import json
import os
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

from gramian_workshop import replay as rp  # noqa: E402
from gramian_workshop import follow as fo  # noqa: E402

try:
    import mujoco as mj
    HAVE_MUJOCO = True
except Exception:
    HAVE_MUJOCO = False

RESULT = os.path.join(ROOT, "outputs", "continuity",
                      "follow_allegro__w10_cup_physical_x0.3_a0.0915_sl0.3t_R33.5_n4"
                      "__continuity__inertia__n100.json")
try:
    from gramian_workshop.hands import Hand
    Hand("allegro")
    HAVE_ALLEGRO = HAVE_MUJOCO and os.path.exists(RESULT)
except Exception:  # mujoco missing, or the menagerie not found
    HAVE_ALLEGRO = False


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def synthetic_leader(R=0.03, mass=0.05):
    """Two stations, three frames at t = 0, 1, 2; the object moving +x at
    1 m/s. Station 0 always on, sunk 1 mm with 0.5 N; station 1 on at frames
    0 and 2 sunk 2 mm with 2 N, and unrecorded (None) at frame 1. Sinks are
    into pad radius + R, as leader_pad_stiffness reads them."""
    pad = rp.STATION_PAD_RADIUS_M
    frames = []
    for k in range(3):
        c = np.array([1.0 * k, 0.0, 0.1])
        s0 = c + np.array([0.0, 0.0, -(pad + R - 0.001)])
        s1 = c + np.array([0.0, pad + R - 0.002, 0.0])
        on1 = k != 1
        frames.append({"t": float(k), "obj_pos": c.tolist(), "obj_quat": [1.0, 0.0, 0.0, 0.0],
                       "station_pos": [s0.tolist(), s1.tolist() if on1 else None],
                       "contact": [True, on1],
                       "force_on_object_N": [[0.0, 0.0, 0.5], [0.0, -2.0, 0.0] if on1 else [0.0, 0.0, 0.0]],
                       "contact_point": [None, None], "contact_normal": [None, None]})
    return {"id": "synthetic", "n_stations": 2, "object": {"radius_m": R, "mass_kg": mass},
            "frames": frames}


class Rotations(unittest.TestCase):
    def test_quat_to_mat(self):
        np.testing.assert_allclose(rp.quat_to_mat([1, 0, 0, 0]), np.eye(3))
        a = np.pi / 2
        q = [np.cos(a / 2), 0, 0, np.sin(a / 2)]      # 90 deg about z
        np.testing.assert_allclose(rp.quat_to_mat(q), rot_z(a), atol=1e-12)

    @unittest.skipUnless(HAVE_MUJOCO, "needs mujoco")
    def test_rotvec_round_trip(self):
        a = 0.7
        np.testing.assert_allclose(rp.rotvec(rot_z(a)), [0, 0, a], atol=1e-9)
        self.assertAlmostEqual(rp.angle_between(rot_z(0.2), rot_z(0.9)), 0.7, places=9)
        self.assertAlmostEqual(rp.angle_between(rot_z(0.2), rot_z(0.2)), 0.0, places=9)

    @unittest.skipUnless(HAVE_MUJOCO, "needs mujoco")
    def test_turning_axis_and_signed_turns(self):
        Rs = [rot_z(0.1 * k) for k in range(6)]
        axis = rp.turning_axis(Rs)
        np.testing.assert_allclose(axis, [0, 0, 1], atol=1e-9)
        turns = rp.signed_turns(Rs, axis)
        np.testing.assert_allclose(turns, [0.1 * k for k in range(6)], atol=1e-9)
        # turned the other way: negative about the same axis
        np.testing.assert_allclose(rp.signed_turns(Rs[::-1], axis)[-1], -0.5, atol=1e-9)
        self.assertIsNone(rp.turning_axis([np.eye(3), np.eye(3)]))

    @unittest.skipUnless(HAVE_MUJOCO, "needs mujoco")
    def test_leader_rotations_are_placed(self):
        L = synthetic_leader()
        Rw = rot_z(0.3)
        Rs = rp.leader_rotations(L, Rw)
        self.assertEqual(len(Rs), 3)
        np.testing.assert_allclose(Rs[0], Rw)
        L["frames"][1]["obj_quat"] = None
        self.assertIsNone(rp.leader_rotations(L, Rw))


class PadStiffness(unittest.TestCase):
    def test_leader_pad_stiffness_worked_by_hand(self):
        L = synthetic_leader()
        K, rec = rp.leader_pad_stiffness(L)
        # ratios: station 0 at 0.5 N / 1 mm = 500 N/m (three frames), station 1
        # at 2 N / 2 mm = 1000 N/m (two frames): median over five is 500
        self.assertAlmostEqual(K, 500.0, places=6)
        self.assertEqual(rec["contacts"], 5)
        self.assertAlmostEqual(rec["sink_median_m"], 0.001, places=9)
        self.assertAlmostEqual(rec["force_median_N"], 0.5, places=9)
        self.assertEqual(rec["pad_radius_m"], rp.STATION_PAD_RADIUS_M)

    def test_leader_pad_stiffness_needs_a_sink(self):
        L = synthetic_leader()
        for f in L["frames"]:        # push every station out to the pad surface
            c = np.array(f["obj_pos"])
            f["station_pos"] = [(c + (np.array(s) - c) / np.linalg.norm(np.array(s) - c)
                                 * (rp.STATION_PAD_RADIUS_M + 0.03)).tolist() if s else None
                                for s in f["station_pos"]]
        with self.assertRaises(ValueError):
            rp.leader_pad_stiffness(L)

    def test_allowance_stiffness(self):
        L = synthetic_leader()
        K, med = rp.pad_stiffness(L, 0.003)
        # forces in contact: 0.5, 0.5, 0.5, 2, 2: median 0.5 N over 3 mm
        self.assertAlmostEqual(med, 0.5)
        self.assertAlmostEqual(K, 0.5 / 0.003)
        with self.assertRaises(ValueError):
            rp.pad_stiffness(L, 0.0)

    @unittest.skipUnless(HAVE_MUJOCO, "needs mujoco")
    def test_pad_contact_reaches_the_asked_stiffness(self):
        """A sphere of the object's mass on a plane with the pad setting
        sinks by weight / K (within 2 %); MuJoCo's default is far stiffer."""
        R, mass, g = 0.03, 0.05, 9.81
        xml = (f'<mujoco><worldbody><body name="object" pos="0 0 1"><freejoint/>'
               f'<geom name="object_geom" type="sphere" size="{R}" mass="{mass}"/></body>'
               f'</worldbody></mujoco>')
        model = mj.MjModel.from_xml_string(xml)
        info = {"object_geom": mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "object_geom")}
        for K in (300.0, 1400.0):
            rec = rp.pad_contact(model, info, K, mass, g, R)
            self.assertAlmostEqual(rec["reached_N_m"] / K, 1.0, delta=0.02)
            self.assertEqual(rec["asked_N_m"], K)
            self.assertLess(model.geom_solref[info["object_geom"], 0], 0)   # direct setting
            np.testing.assert_allclose(model.geom_solimp[info["object_geom"]], rec["solimp"])
        # MuJoCo's default contact is not rigid: its stiffness scales with the
        # object's mass (the "default" pad setting records what it reached)
        d1 = rp._measure_stiffness(None, None, None, mass, g, R)
        d2 = rp._measure_stiffness(None, None, None, 2 * mass, g, R)
        self.assertGreater(d1, 0.0)
        self.assertAlmostEqual(d2 / d1, 2.0, delta=0.2)


class FakeLeader:
    def __init__(self, R=0.03):
        self.R_obj = R


def rows_from(errs, turn_sim=90.0, turn_leader=100.0, contacts=2):
    out = []
    for i, e in enumerate(errs):
        out.append({"k": i, "t": 0.1 * i, "centre_sim_m": [e, 0.0, 0.0], "centre_leader_m": [0.0, 0.0, 0.0],
                    "centre_err_m": float(e), "orient_err_deg": 1.0,
                    "turn_sim_deg": turn_sim * (i + 1) / len(errs),
                    "turn_leader_deg": turn_leader * (i + 1) / len(errs),
                    "contacts": ["index"] if contacts else [], "n_contacts": contacts,
                    "net_force_sim_N": [0, 0, 0], "net_force_leader_N": [0, 0, 0],
                    "force_sim_N": {"index": [0.0, 0.0, 0.3]}, "force_leader_N": {"index": [0.0, 0.0, 0.6]},
                    "leader_on": [1]})
    return out


class Verdicts(unittest.TestCase):
    def test_follows(self):
        s = rp.summary(rows_from([0.001, 0.002, 0.001]), FakeLeader(), 0.003)
        self.assertEqual(s["verdict"], "follows")
        self.assertIsNone(s["first_over_tol_t"])
        self.assertAlmostEqual(s["turn_share"], 0.9)
        self.assertEqual(s["frames_with_no_contact"], 0)
        self.assertAlmostEqual(s["forces_vs_leader"]["index"]["ratio_median"], 0.5)
        self.assertEqual(s["forces_vs_leader"]["index"]["touching_share"], 1.0)

    def test_lost_and_dropped(self):
        s = rp.summary(rows_from([0.001, 0.004, 0.002]), FakeLeader(), 0.003)
        self.assertEqual(s["verdict"], "lost")
        self.assertEqual(s["first_over_tol_k"], 1)
        self.assertIsNone(s["dropped_t"])
        s = rp.summary(rows_from([0.001, 0.004, 0.05]), FakeLeader(), 0.003)
        self.assertEqual(s["verdict"], "dropped")
        self.assertAlmostEqual(s["dropped_t"], 0.2)

    def test_held_not_turned(self):
        s = rp.summary(rows_from([0.001] * 3, turn_sim=10.0, turn_leader=100.0), FakeLeader(), 0.003)
        self.assertEqual(s["verdict"], "held, not turned")
        # a leader that barely turns is judged on the centre alone
        s = rp.summary(rows_from([0.001] * 3, turn_sim=0.0, turn_leader=1.0), FakeLeader(), 0.003)
        self.assertEqual(s["verdict"], "follows")

    def test_offset_about_mean(self):
        """A steady offset counts against the tolerance but not as error
        about the mean."""
        s = rp.summary(rows_from([0.004] * 4), FakeLeader(), 0.003)
        self.assertEqual(s["verdict"], "lost")
        self.assertAlmostEqual(s["offset_mean_size_m"], 0.004)
        self.assertAlmostEqual(s["centre_err_about_mean_max_m"], 0.0)

    def test_spans(self):
        self.assertTrue(rp._spans([14], 10, 14))
        self.assertFalse(rp._spans([14], 14, 20))      # the jump frame itself is the start
        self.assertFalse(rp._spans([14], 15, 20))
        self.assertFalse(rp._spans([], 0, 100))


class StubHand:
    coupled = {}

    def apply_coupling(self, q):
        return q


class JointPath(unittest.TestCase):
    def test_linear_between_held_outside(self):
        p = rp.Path(StubHand(), [1.0, 2.0, 4.0], [[0.0, 10.0], [1.0, 10.0], [3.0, 20.0]])
        np.testing.assert_allclose(p.q(0.0), [0.0, 10.0])
        np.testing.assert_allclose(p.q(1.5), [0.5, 10.0])
        np.testing.assert_allclose(p.q(3.0), [2.0, 15.0])
        np.testing.assert_allclose(p.q(9.0), [3.0, 20.0])
        np.testing.assert_allclose(p.qdot(1.5), [1.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(p.qdot(3.0), [1.0, 5.0], atol=1e-6)
        np.testing.assert_allclose(p.qdot(0.5), [0.0, 0.0], atol=1e-9)

    def test_station_lift_and_on_distance(self):
        L = fo.Leader(synthetic_leader())
        self.assertAlmostEqual(rp.station_on_distance(L, 0), rp.STATION_PAD_RADIUS_M - 0.001)
        self.assertAlmostEqual(rp.station_on_distance(L, 1), rp.STATION_PAD_RADIUS_M - 0.002)
        self.assertEqual(rp.station_lift(L, 0, 0, 0.0), 0.0)         # on: no lift
        self.assertEqual(rp.station_lift(L, 1, 1, 0.0), 0.0)         # unrecorded: no lift
        Lj = synthetic_leader()
        c = np.array(Lj["frames"][1]["obj_pos"])
        Lj["frames"][1]["station_pos"][1] = (c + [0.0, 0.05, 0.0]).tolist()
        Lj["frames"][1]["contact"][1] = False
        L2 = fo.Leader(Lj)
        self.assertAlmostEqual(rp.station_lift(L2, 1, 1, 0.006), 0.05 - 0.03 - 0.006)


class PathJumps(unittest.TestCase):
    def test_reads_the_stored_readout(self):
        class H:
            spec = {"datasheet_no_load_speed": (2.0, "rad/s", "made up")}
        result = {"frames": [
            {"k": 0, "fingers": [{"finger": "index", "continuity": {"jump": []}}]},
            {"k": 3, "fingers": [{"finger": "index", "continuity": {
                "from_frame": 0, "dt_s": 0.1, "link_changed": False,
                "jump": [{"joint": "a", "kind": "hinge", "dq": 0.1, "rate": 1.0},
                         {"joint": "b", "kind": "hinge", "dq": -0.15, "rate": -1.5}]}}]},
            {"k": 6, "fingers": [{"finger": "index", "continuity": {
                "from_frame": 3, "dt_s": 0.1, "link_changed": True,
                "jump": [{"joint": "a", "kind": "hinge", "dq": 0.3, "rate": 3.0}]}}]},
            {"k": 9, "fingers": []}]}
        j = rp.path_jumps(result, H())
        self.assertEqual(j["n_checked_with_jump_readout"], 2)
        self.assertEqual(j["fastest_rate_rad_s"], 3.0)
        self.assertEqual(j["first_link_change_k"], 6)
        self.assertEqual(j["first_above_no_load_k"], 6)
        self.assertEqual([f["joint"] for f in j["link_changes"]], ["a"])
        self.assertEqual(j["no_load_speed"]["rad_s"], 2.0)
        # the fastest joint at k=3 is b, by size
        k3 = [f for f in j["above_no_load"]]
        self.assertEqual(k3[0]["k"], 6)

    def test_no_published_speed(self):
        class H:
            spec = {}
        j = rp.path_jumps({"frames": []}, H())
        self.assertIsNone(j["no_load_speed"])
        self.assertIsNone(j["fastest_rate_rad_s"])
        self.assertEqual(j["above_no_load"], [])


class Drivers(unittest.TestCase):
    def test_feedforward_settings(self):
        self.assertEqual(set(rp.FEEDFORWARD), {"all", "standing"})
        self.assertTrue(all(rp.FEEDFORWARD["all"].values()))
        st = rp.FEEDFORWARD["standing"]
        self.assertTrue(st["contact"] and st["weight"])
        self.assertFalse(st["motion"] or st["damping"])

    def test_servo_hold_uses_the_standing_parts(self):
        """hold() applies no motion or damping torque however the driver was
        set; step() applies what it was set with."""
        calls = []

        class FF:
            def tau(self, t, **parts):
                calls.append(parts)
                return np.zeros(2)

        class Data:
            qpos = np.zeros(2)
            qvel = np.zeros(2)
            ctrl = np.zeros(2)
            qfrc_applied = np.zeros(2)
        path = rp.Path(StubHand(), [0.0, 1.0], [[0.0, 1.0], [2.0, 3.0]])
        info = {"hand_qpos": np.arange(2), "hand_dof": np.arange(2)}
        d = rp.ServoDriver(path, info, FF(), [1, 0])
        data = Data()
        d.step(data, 0.5)
        np.testing.assert_allclose(data.ctrl, [2.0, 1.0])      # ctrl[a] = q[ctrl_qpos[a]]
        self.assertEqual(calls[-1], rp.FEEDFORWARD["all"])
        d.hold(data, 0.5)
        self.assertEqual(calls[-1], rp.FEEDFORWARD["standing"])
        d2 = rp.ServoDriver(path, info, FF(), [1, 0], **rp.FEEDFORWARD["standing"])
        d2.step(data, 0.5)
        self.assertEqual(calls[-1], rp.FEEDFORWARD["standing"])
        f = rp.Frozen(d, 0.25)
        f.start(data, 0.9)
        np.testing.assert_allclose(data.qpos, [0.5, 1.5])      # the pose at t0, not at t
        np.testing.assert_allclose(data.qvel, 0.0)
        self.assertEqual(calls[-1], rp.FEEDFORWARD["standing"])

    def test_kinematic_driver(self):
        class Data:
            qpos = np.zeros(2)
            qvel = np.zeros(2)
        path = rp.Path(StubHand(), [0.0, 1.0], [[0.0, 1.0], [2.0, 3.0]])
        info = {"hand_qpos": np.arange(2), "hand_dof": np.arange(2)}
        d = rp.KinematicDriver(path, info)
        data = Data()
        d.step(data, 0.5)
        np.testing.assert_allclose(data.qpos, [1.0, 2.0])
        np.testing.assert_allclose(data.qvel, [2.0, 2.0], atol=1e-6)
        d.hold(data, 0.5)
        np.testing.assert_allclose(data.qvel, 0.0)

    def test_station_driver_parks_unrecorded_stations(self):
        L = fo.Leader(synthetic_leader())
        d = rp.StationDriver(L, np.eye(3), L.c0, [0, 3], [0, 3], [0, 3])   # identity placement
        out, bad = d.pos(0.0)
        self.assertEqual(bad.tolist(), [False, False])
        np.testing.assert_allclose(out[0], L.frames[0]["station_pos"][0])
        out, bad = d.pos(0.5)
        self.assertEqual(bad.tolist(), [False, True])         # station 1 unrecorded at frame 1
        np.testing.assert_allclose(out[0], np.array(L.frames[0]["station_pos"][0]) + [0.5, 0, 0])
        np.testing.assert_allclose(out[1], [0, 0, rp.StationDriver.PARK_M])
        out, bad = d.pos(2.0)
        self.assertEqual(bad.tolist(), [False, False])
        out, bad = d.pos(5.0)                                  # held after the last frame
        np.testing.assert_allclose(out[1], L.frames[2]["station_pos"][1])

    @unittest.skipUnless(HAVE_MUJOCO, "needs mujoco")
    def test_stations_model(self):
        L = fo.Leader(synthetic_leader())
        model, info = rp.stations_model(L, np.eye(3), L.c0, 1.0, [0, 0, -9.81])
        self.assertEqual(mj.mjtIntegrator(int(model.opt.integrator)).name,
                         "mjINT_" + rp.STATION_INTEGRATOR.upper())
        self.assertEqual(model.nu, 3 * L.n)
        self.assertEqual(len(info["station_bodies"]), L.n)
        self.assertTrue(all(model.actuator_forcelimited))
        d = rp.StationDriver(L, np.eye(3), L.c0, info["station_qpos"], info["station_dof"],
                             info["station_ctrl"])
        data = mj.MjData(model)
        d.start(data, 1.0)
        q1 = info["station_qpos"][1]
        np.testing.assert_allclose(data.qpos[q1:q1 + 3], [0, 0, rp.StationDriver.PARK_M])
        d.step(data, 2.0)          # reappears: put there directly, at rest
        np.testing.assert_allclose(data.qpos[q1:q1 + 3], L.frames[2]["station_pos"][1])
        np.testing.assert_allclose(data.qvel[info["station_dof"][1]:info["station_dof"][1] + 3], 0.0)


@unittest.skipUnless(HAVE_MUJOCO, "needs mujoco")
class SmallModels(unittest.TestCase):
    def test_contact_force_on_the_object(self):
        """A sphere named as the object resting on a plane: the net contact
        force on it is its weight, upward."""
        m, g = 0.05, 9.81
        xml = (f'<mujoco><option gravity="0 0 -{g}" timestep="0.002"/><worldbody>'
               f'<geom type="plane" size="1 1 .1"/><body pos="0 0 0.03"><freejoint/>'
               f'<geom name="object_geom" type="sphere" size="0.03" mass="{m}"/></body>'
               f'</worldbody></mujoco>')
        model = mj.MjModel.from_xml_string(xml)
        data = mj.MjData(model)
        for _ in range(1500):
            mj.mj_step(model, data)
        info = {"object_geom": mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "object_geom")}
        F, who, n, per = rp.net_contact_force(model, data, info)
        self.assertEqual(n, 1)
        np.testing.assert_allclose(F, [0, 0, m * g], atol=1e-3)
        self.assertEqual(who, {"other"})
        np.testing.assert_allclose(per["other"], F)

    def test_seat_object_round_trip(self):
        xml = ('<mujoco><worldbody><body name="object"><freejoint/>'
               '<geom name="object_geom" type="sphere" size="0.03" mass="0.05"/></body>'
               '</worldbody></mujoco>')
        model = mj.MjModel.from_xml_string(xml)
        data = mj.MjData(model)
        info = {"object_qpos": 0, "object_dof": 0}
        R = rot_z(0.4)
        rp.seat_object(data, info, [0.1, 0.2, 0.3], R, [1, 0, 0], [0, 0, 2])
        np.testing.assert_allclose(rp.quat_to_mat(data.qpos[3:7]), R, atol=1e-9)
        np.testing.assert_allclose(data.qvel, [1, 0, 0, 0, 0, 2])

    def test_leader_state_velocities(self):
        L = fo.Leader(synthetic_leader())
        Rs = rp.leader_rotations(L.L, np.eye(3))
        c, R, v, w = rp.leader_state(L, 1, np.eye(3), L.c0, Rs)
        np.testing.assert_allclose(c, L.frames[1]["obj_pos"])
        np.testing.assert_allclose(v, [1.0, 0.0, 0.0])
        np.testing.assert_allclose(w, 0.0)


@unittest.skipUnless(HAVE_ALLEGRO, "needs mujoco, the menagerie Allegro and the stored result")
class Allegro(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(RESULT, encoding="utf-8") as f:
            cls.result = json.load(f)
        with open(os.path.join(ROOT, "data", "leaders", cls.result["leader"] + ".json"),
                  encoding="utf-8") as f:
            cls.lj = json.load(f)
        cls.hand = Hand(cls.result["hand"])
        cls.leader = fo.Leader(cls.lj)
        cls.Rw = np.array(cls.result["placement"]["rotation_matrix"], float)
        cls.t = np.array(cls.result["placement"]["translation_m"], float)
        cls.g, _ = rp.leader_gravity_hand(cls.lj, cls.Rw)

    def fresh_model(self):
        model, info = rp.build_model(self.hand, self.leader, self.Rw, self.t, 1.0, self.g)
        Rs = rp.leader_rotations(self.lj, self.Rw)
        info["turn_axis"] = rp.turning_axis(Rs)
        return model, info, Rs

    def test_build_model(self):
        model, info, _ = self.fresh_model()
        hm = self.hand.model
        self.assertEqual(model.nq, hm.nq + 7)
        self.assertEqual(model.nv, hm.nv + 6)
        self.assertEqual(model.nu, hm.nu)
        np.testing.assert_allclose(model.opt.gravity, self.g)
        og = info["object_geom"]
        self.assertAlmostEqual(float(model.geom_size[og, 0]), self.leader.R_obj)
        self.assertAlmostEqual(float(model.body_mass[info["object_body"]]), self.leader.mass)
        np.testing.assert_allclose(model.body_pos[info["object_body"]],
                                   self.leader.center(0, self.Rw, self.t))
        self.assertGreater(len(info["hand_collision_geoms"]), 0)
        self.assertNotIn(info["object_body"], info["hand_bodies"])
        self.assertIsNotNone(info["ghost_mocap"])
        self.assertEqual(rp.contact_options(model)["timestep_s"], float(hm.opt.timestep))

    def test_free_fall(self):
        """The object alone, well away from the hand, falls half g T squared
        along the leader's gravity (to the integrator's first-order error)."""
        model, info, _ = self.fresh_model()
        rp.servo_drive(model, info)
        data = mj.MjData(model)
        oq, od = info["object_qpos"], info["object_dof"]
        start = np.array([0.0, 0.0, 2.0])
        data.qpos[oq:oq + 3] = start
        n = 250
        for _ in range(n):
            mj.mj_step(model, data)
        T = n * model.opt.timestep
        drop = data.qpos[oq:oq + 3] - start
        gn = np.linalg.norm(self.g)
        expect = 0.5 * gn * T * T * (1 + 1.0 / n)        # semi-implicit Euler's exact sum
        np.testing.assert_allclose(drop @ (self.g / gn), expect, rtol=1e-6)
        self.assertEqual(data.ncon, 0)

    def test_servo_drive_record(self):
        model, info, _ = self.fresh_model()
        rec = rp.servo_drive(model, info)
        hm = self.hand.model
        self.assertEqual(rec["actuators"], hm.nu)
        self.assertEqual(sorted(rec["ctrl_qpos"]), list(range(hm.nq)))
        self.assertEqual(rec["unserved_joints"], [])
        self.assertTrue(all(k > 0 for k in rec["kp_Nm_per_rad"]))
        # only hand-object pairs collide afterwards
        og = info["object_geom"]
        self.assertEqual((int(model.geom_contype[og]), int(model.geom_conaffinity[og])), (0, 1))
        for g in info["hand_collision_geoms"]:
            self.assertEqual((int(model.geom_contype[g]), int(model.geom_conaffinity[g])), (1, 0))

    def test_kinematic_drive_record(self):
        model, info, _ = self.fresh_model()
        rec = rp.kinematic_drive(model, info)
        self.assertEqual(rec["drive"], "kinematic")
        np.testing.assert_allclose(model.body_gravcomp[info["hand_bodies"]], 1.0)
        self.assertTrue((model.dof_armature[info["hand_dof"]] >= rp.ARMATURE).all())
        if model.nu:
            np.testing.assert_allclose(model.actuator_gainprm, 0.0)

    def test_stored_path_hits_the_stored_poses(self):
        model, info, _ = self.fresh_model()
        for lift in ("rest", "leader"):
            path, notes = rp.stored_path(self.hand, self.result, self.leader, lift=lift,
                                         model=model, info=info, Rw=self.Rw, t=self.t)
            self.assertEqual(notes["lift"], lift)
            self.assertEqual(notes["checked_frames"], len(self.result["frames"]))
            for fr in self.result["frames"]:
                q = path.q(float(self.leader.frames[fr["k"]]["t"]))
                for f, x in zip(self.hand.fingers, fr["fingers"]):
                    if "q" in x:
                        np.testing.assert_allclose(q[f.qadr], x["q"], atol=1e-12)
                    elif lift == "rest":
                        np.testing.assert_allclose(q[f.qadr], self.hand.base_q[f.qadr])
        with self.assertRaises(ValueError):
            rp.stored_path(self.hand, self.result, self.leader, lift="hover")

    def test_lifted_fingers_stand_clear(self):
        """With the leader lift, every finger the check left off is clear of
        the object at its checked frames, and no further than it was asked
        to lift (plus the clearance tolerance)."""
        model, info, _ = self.fresh_model()
        path, notes = rp.stored_path(self.hand, self.result, self.leader, lift="leader",
                                     model=model, info=info, Rw=self.Rw, t=self.t)
        n_lifts = 0
        for lab, rec in notes["lifts_m"].items():
            if isinstance(rec, str):
                continue
            for k, want, got in rec["frames"]:
                n_lifts += 1
                self.assertGreater(got, 0.0, f"{lab} at k={k} touches the object when off")
        self.assertGreater(n_lifts, 0)
        geo = rp.geometry_check(model, info, self.hand, path, self.result, self.leader,
                                self.Rw, self.t)
        self.assertGreater(geo["lifted_clearance_min_m"], 0.0)
        self.assertLess(geo["ball_center_vs_leader_max_m"], 1e-9)
        allow = fo.overlap_allowance(self.lj)["contact_m"]
        # the check's own allowance, plus what the pad point's projection can add
        self.assertLessEqual(geo["touching_sink_max_m"], allow + geo["pad_world_vs_model_nearest_max_m"])

    def test_feedforward_weight_is_the_checks(self):
        """Feedforward.weight at a checked pose equals the stored weight_Nm
        on each touching finger's joints (the check's own mj_rne under the
        leader's gravity)."""
        model, info, _ = self.fresh_model()
        path, _ = rp.stored_path(self.hand, self.result, self.leader, lift="rest")
        ff = rp.Feedforward(model, info, self.hand, self.result, self.leader, path, self.Rw)
        n = 0
        for fr in self.result["frames"]:
            q = path.q(float(self.leader.frames[fr["k"]]["t"]))
            w = ff.weight(q)
            for f, x in zip(self.hand.fingers, fr["fingers"]):
                stored = (x.get("inertia") or {}).get("weight_Nm")
                if stored is None:
                    continue
                np.testing.assert_allclose(w[f.dof], stored, atol=1e-9)
                n += 1
        self.assertGreater(n, 50)

    def test_feedforward_contact_is_jt_f(self):
        """The contact torque fed forward at a checked frame is the stored
        force at the stored pad point mapped to the joints: the same as
        MuJoCo's mj_applyFT of that force at that point."""
        model, info, _ = self.fresh_model()
        path, _ = rp.stored_path(self.hand, self.result, self.leader, lift="rest")
        ff = rp.Feedforward(model, info, self.hand, self.result, self.leader, path, self.Rw)
        data = mj.MjData(model)
        self.assertGreater(ff.notes["contact_items"], 0)
        checked = 0
        for j, fr in enumerate(self.result["frames"][:20]):
            q = path.q(float(self.leader.frames[fr["k"]]["t"]))
            data.qpos[info["hand_qpos"]] = q
            mj.mj_forward(model, data)
            expect = np.zeros(model.nv)
            for f, x in zip(self.hand.fingers, fr["fingers"]):
                if "q" not in x:
                    continue
                b = (int(f.tip_body) if x.get("phalanx", "distal") == "distal"
                     else int(model.body_parentid[f.tip_body]))
                force = self.leader.force(fr["k"], int(x["station"]), self.Rw)
                mj.mj_applyFT(model, data, force, np.zeros(3), np.asarray(x["pad_world"], float),
                              b, expect)
                checked += 1
            np.testing.assert_allclose(ff.contact_Nm[j], expect[info["hand_dof"]], atol=1e-9)
            np.testing.assert_allclose(ff.tau(float(self.leader.frames[fr["k"]]["t"]), motion=False,
                                              weight=False, damping=False), ff.contact_Nm[j])
        self.assertGreater(checked, 0)

    def test_damping_feedforward(self):
        model, info, _ = self.fresh_model()
        path, _ = rp.stored_path(self.hand, self.result, self.leader, lift="rest")
        ff = rp.Feedforward(model, info, self.hand, self.result, self.leader, path, self.Rw)
        tm = 0.5 * (path.T[3] + path.T[4])
        np.testing.assert_allclose(ff.damping(tm), model.dof_damping[info["hand_dof"]] * path.qdot(tm))
        np.testing.assert_allclose(ff.tau(tm, contact=False, motion=False, weight=False), ff.damping(tm))
        np.testing.assert_allclose(ff.damping(path.T[0] - 1.0), 0.0)   # held outside: no speed

    def test_hand_alone_tracks_its_path(self):
        """The servo with the check's feedforward follows the stored path
        (leader lift, the headline) with the object's collisions off, over
        the first ten leader steps: every joint within 2 degrees. (The rest
        lift sends a finger that goes off to the rest pose in one leader
        step, which the servo cannot follow; track records that.)"""
        model, info, Rs = self.fresh_model()
        rec = rp.servo_drive(model, info)
        path, _ = rp.stored_path(self.hand, self.result, self.leader, lift="leader", model=model,
                                 info=info, Rw=self.Rw, t=self.t)
        ff = rp.Feedforward(model, info, self.hand, self.result, self.leader, path, self.Rw)
        drv = rp.ServoDriver(path, info, ff, rec["ctrl_qpos"], contact=False)
        ks = [fr["k"] for fr in self.result["frames"]]
        tr = rp.track_check(model, info, self.hand, drv, path, self.leader, ks[0], ks[0] + 10)
        self.assertEqual(tr["frames"], 11)
        self.assertLess(tr["joint_err_max_rad"], np.radians(2.0))
        self.assertLess(tr["tip_err_max_m"], 0.003)
        og = info["object_geom"]      # the object's collisions are restored after
        self.assertEqual((int(model.geom_contype[og]), int(model.geom_conaffinity[og])), (0, 1))

    def test_frozen_hand_holds_the_object(self):
        """The control: the hand frozen at the first checked pose with the
        object seated at rest keeps it in hand over five leader steps, with
        contact throughout."""
        model, info, Rs = self.fresh_model()
        rec = rp.servo_drive(model, info)
        K, _ = rp.leader_pad_stiffness(self.lj)
        rp.pad_contact(model, info, K, self.leader.mass, float(np.linalg.norm(self.g)), self.leader.R_obj)
        path, _ = rp.stored_path(self.hand, self.result, self.leader, lift="leader", model=model,
                                 info=info, Rw=self.Rw, t=self.t)
        ff = rp.Feedforward(model, info, self.hand, self.result, self.leader, path, self.Rw)
        drv = rp.ServoDriver(path, info, ff, rec["ctrl_qpos"])
        ks = [fr["k"] for fr in self.result["frames"]]
        T0 = float(self.leader.frames[ks[0]]["t"])
        rows = rp.simulate(model, info, rp.Frozen(drv, T0), self.leader, self.Rw, self.t, Rs,
                           ks[0], ks[0] + 5, rp.finger_owner(self.hand),
                           follows={lab: int(i) for lab, i in self.result["assignment"].items()},
                           at_rest=True)
        self.assertEqual(len(rows), 6)
        s = rp.summary(rows, self.leader, 0.003)
        self.assertLess(s["centre_err_max_m"], self.leader.R_obj)
        self.assertEqual(s["frames_with_no_contact"], 0)
        self.assertNotEqual(s["verdict"], "dropped")
        self.assertTrue(set(rows[-1]["contacts"]) <= {f.label for f in self.hand.fingers})

    def test_path_jumps_of_the_stored_result(self):
        """The stored continuity readout, as the replay reports it: the
        index finger's IK branch flips at k=14 and that step is faster than
        the published no-load speed."""
        j = rp.path_jumps(self.result, self.hand)
        self.assertEqual(j["first_link_change_k"], 14)
        self.assertEqual(j["link_changes"][0]["finger"], "index")
        self.assertGreater(j["fastest_rate_rad_s"], j["no_load_speed"]["rad_s"])
        self.assertEqual(j["first_above_no_load_k"], 14)

    def test_replay_guards(self):
        with self.assertRaises(ValueError):
            rp.replay(self.result, dict(self.lj, id="another"), hand=self.hand)
        kw = dict(windows=False, frozen=False, track=False, control=False, hand=self.hand)
        with self.assertRaises(ValueError):
            rp.replay(self.result, self.lj, pads="default", pad_k=100.0, **kw)
        with self.assertRaises(ValueError):
            rp.replay(self.result, self.lj, pads="given", **kw)
        with self.assertRaises(ValueError):
            rp.replay(self.result, self.lj, pads="foam", **kw)
        with self.assertRaises(ValueError):
            rp.replay(self.result, self.lj, feedforward="some", **kw)
        with self.assertRaises(ValueError):
            rp.replay(self.result, self.lj, drive="magic", **kw)

    def test_leader_pads_are_the_default_and_recorded(self):
        """A short replay (free run only) records the pad setting it used,
        the feedforward parts and the path readout."""
        short = dict(self.result, frames=self.result["frames"][:3])
        rec = rp.replay(short, self.lj, windows=False, frozen=False, track=False, control=False,
                        hand=self.hand)
        pad = rec["settings"]["object"]["pad_contact"]
        self.assertEqual(pad["pads"], "leader")
        K, src = rp.leader_pad_stiffness(self.lj)
        self.assertEqual(pad["asked_N_m"], K)
        self.assertEqual(pad["derivation"]["contacts"], src["contacts"])
        self.assertEqual(rec["settings"]["drive"]["feedforward"]["setting"], "all")
        self.assertEqual(rec["free"]["summary"]["frames"], short["frames"][-1]["k"] - short["frames"][0]["k"] + 1)
        self.assertEqual(rec["path_jumps"]["first_link_change_k"], None)     # k=14 is past frame 3
        self.assertIn(rec["free"]["summary"]["verdict"], ("follows", "lost", "dropped", "held, not turned"))
        rec2 = rp.replay(short, self.lj, pads="default", windows=False, frozen=False, track=False,
                         control=False, hand=self.hand)
        pad2 = rec2["settings"]["object"]["pad_contact"]
        self.assertEqual(pad2["pads"], "default")
        self.assertNotIn("asked_N_m", pad2)
        self.assertGreater(pad2["reached_N_m"], pad["reached_N_m"])


class Runner(unittest.TestCase):
    def test_fresh_never_overwrites(self):
        import run_replay
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "a.json")
            self.assertEqual(run_replay.fresh(p), p)
            open(p, "w").close()
            p2 = run_replay.fresh(p)
            self.assertNotEqual(p2, p)
            self.assertTrue(p2.endswith("__2.json"))
            open(p2, "w").close()
            self.assertTrue(run_replay.fresh(p).endswith("__3.json"))

    def test_argument_guards(self):
        import run_replay
        for argv in (["x", "r.json", "--pad-k", "5"],
                     ["x", "r.json", "--pads", "given"],
                     ["x", "r.json", "--pads", "default", "--pad-k", "5"],
                     ["x", "r.json", "--pads", "foam"]):
            with mock.patch.object(sys, "argv", argv), self.assertRaises(SystemExit),                     contextlib.redirect_stderr(io.StringIO()):
                run_replay.main()


if __name__ == "__main__":
    unittest.main()
