"""Rigid hands from MuJoCo models: fingers, inputs, limits, fingertip reach.

Needs `mujoco` (pip install -e .[hands]) and the hand models: run
tools/fetch_hands.py once, or set GMW_MENAGERIE / GMW_WAVE_MANIP to local
checkouts.

One rule for every hand, stated rather than tuned per hand:
  fingertip point  the vertex of the distal link's collision geometry
                   farthest from that link's own joint (for primitive geoms,
                   the farthest point of their bounding box)
  inputs           actuators whose moment touches the finger's joints (the
                   model's own actuators); a model with no actuators (the
                   L20 here) counts its uncoupled joints as inputs
  torque limits    from the model only: actuator forcerange through the
                   actuator moment, or joint actuatorfrcrange; none if the
                   model gives none (the tool then says "strength not
                   checked", never an invented number)
"""
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DEFAULT_MENAGERIE = os.path.join(ROOT, "external", "mujoco_menagerie")
DEFAULT_WAVE_MANIP = os.path.join(ROOT, "external", "wave_manip_models")

HANDS = {
    "allegro": {
        "name": "Allegro (Wonik)", "base": "menagerie",
        "file": "wonik_allegro/right_hand.xml",
        "fingers": {"index": "ff_tip", "middle": "mf_tip", "ring": "rf_tip",
                    "thumb": "th_tip"}},
    "shadow": {
        "name": "Shadow Dexterous Hand", "base": "menagerie",
        "file": "shadow_hand/right_hand.xml",
        "fingers": {"index": "rh_ffdistal", "middle": "rh_mfdistal",
                    "ring": "rh_rfdistal", "little": "rh_lfdistal",
                    "thumb": "rh_thdistal"}},
    "leap": {
        "name": "LEAP Hand", "base": "menagerie",
        "file": "leap_hand/right_hand.xml",
        "fingers": {"index": "if_ds", "middle": "mf_ds", "ring": "rf_ds",
                    "thumb": "th_ds"}},
    "l20": {
        "name": "Linker L20", "base": "wave_manip",
        "file": "linker_l20_right/l20_right.xml",
        "fingers": {"thumb": "thumb_distal", "index": "index_distal",
                    "middle": "middle_distal", "ring": "ring_distal",
                    "little": "pinky_distal"}},
}


def model_path(key):
    spec = HANDS[key]
    if spec["base"] == "menagerie":
        base = os.environ.get("GMW_MENAGERIE", DEFAULT_MENAGERIE)
    else:
        base = os.environ.get("GMW_WAVE_MANIP", DEFAULT_WAVE_MANIP)
    return os.path.join(base, spec["file"])


def _mj():
    try:
        import mujoco
    except ImportError as e:                                   # pragma: no cover
        raise ImportError("rigid hands need mujoco: pip install -e .[hands]") from e
    return mujoco


class Finger:
    """One finger of a rigid hand."""

    def __init__(self, hand, label, tip_body):
        mj = _mj()
        m = hand.model
        self.hand, self.label = hand, label
        self.tip_body = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, tip_body)
        if self.tip_body < 0:
            raise ValueError(f"{hand.key}: no body {tip_body}")
        joints, b = [], self.tip_body
        while b > 0:
            for j in range(m.body_jntadr[b], m.body_jntadr[b] + m.body_jntnum[b]):
                if m.jnt_type[j] in (mj.mjtJoint.mjJNT_HINGE, mj.mjtJoint.mjJNT_SLIDE):
                    joints.append(j)
            b = int(m.body_parentid[b])
        self.joints = sorted(joints)
        self.dof = np.array([m.jnt_dofadr[j] for j in self.joints])
        self.qadr = np.array([m.jnt_qposadr[j] for j in self.joints])
        self.ranges = np.array([m.jnt_range[j] for j in self.joints])
        self.tip_local = self._tip_point()

    def _tip_point(self):
        """Farthest collision point of the distal link from its own joint,
        in the distal body's frame."""
        mj = _mj()
        m = self.hand.model
        b = self.tip_body
        jnt = m.body_jntadr[b]
        anchor = np.array(m.jnt_pos[jnt]) if m.body_jntnum[b] else np.zeros(3)
        best, bestd = None, -1.0
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != b:
                continue
            if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0:
                continue                                   # visual only
            R = np.array(m.geom_quat[g])
            Rm = np.zeros(9)
            mj.mju_quat2Mat(Rm, R)
            Rm = Rm.reshape(3, 3)
            if m.geom_type[g] == mj.mjtGeom.mjGEOM_MESH:
                mid = m.geom_dataid[g]
                v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]
                                + m.mesh_vertnum[mid]]
                pts = v @ Rm.T + m.geom_pos[g]
            else:
                s = m.geom_size[g]
                ext = np.array([s[0], s[0] if m.geom_type[g] != mj.mjtGeom.mjGEOM_BOX
                                else s[1], s[0] if m.geom_type[g] == mj.mjtGeom.mjGEOM_SPHERE
                                else (s[2] if m.geom_type[g] == mj.mjtGeom.mjGEOM_BOX
                                      else s[1] + s[0])])
                corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1)
                                    for z in (-1, 1)]) * ext
                pts = corners @ Rm.T + m.geom_pos[g]
            d = np.linalg.norm(pts - anchor, axis=1)
            i = int(np.argmax(d))
            if d[i] > bestd:
                best, bestd = pts[i], float(d[i])
        if best is None:
            raise ValueError(f"{self.hand.key}/{self.label}: no collision geometry on the tip link")
        return best

    def tip_pos(self, data):
        R = data.xmat[self.tip_body].reshape(3, 3)
        return data.xpos[self.tip_body] + R @ self.tip_local

    def tip_jacobian(self, data):
        mj = _mj()
        m = self.hand.model
        jacp = np.zeros((3, m.nv))
        mj.mj_jac(m, data, jacp, None, self.tip_pos(data), self.tip_body)
        return jacp


class Hand:
    def __init__(self, key, path=None):
        mj = _mj()
        self.key = key
        self.spec = HANDS[key]
        self.path = path or model_path(key)
        if not os.path.exists(self.path):
            raise FileNotFoundError(
                f"{self.path} not found: run tools/fetch_hands.py, or set "
                "GMW_MENAGERIE / GMW_WAVE_MANIP")
        self.model = mj.MjModel.from_xml_path(self.path)
        self.data = mj.MjData(self.model)
        mj.mj_forward(self.model, self.data)
        self.fingers = [Finger(self, lab, body)
                        for lab, body in self.spec["fingers"].items()]
        # joints every finger passes through (a wrist) belong to the hand,
        # not to any one finger
        shared = set.intersection(*[set(f.joints) for f in self.fingers])
        self.shared_joints = sorted(shared)
        for f in self.fingers:
            keep = [i for i, j in enumerate(f.joints) if j not in shared]
            f.joints = [f.joints[i] for i in keep]
            f.dof, f.qadr, f.ranges = f.dof[keep], f.qadr[keep], f.ranges[keep]
        self._coupling()

    def _coupling(self):
        """Joint couplings: equality constraints (joint-joint polynomials)."""
        mj = _mj()
        m = self.model
        self.coupled = {}                              # dependent -> (master, polycoef)
        for e in range(m.neq):
            if m.eq_type[e] == mj.mjtEq.mjEQ_JOINT and m.eq_active0[e]:
                self.coupled[int(m.eq_obj1id[e])] = (int(m.eq_obj2id[e]),
                                                     np.array(m.eq_data[e][:5]))

    def apply_coupling(self, q):
        m = self.model
        for dep, (master, c) in self.coupled.items():
            x = q[m.jnt_qposadr[master]]
            q[m.jnt_qposadr[dep]] = c[0] + c[1] * x + c[2] * x ** 2 + c[3] * x ** 3 + c[4] * x ** 4
        return q

    def inputs(self, finger):
        """(number of independent inputs, torque limits per joint or None,
        source)."""
        mj = _mj()
        m, d = self.model, self.data
        if m.nu:
            mom = np.zeros((m.nu, m.nv))
            mj.mju_sparse2dense(mom, d.actuator_moment, d.moment_rownnz,
                                d.moment_rowadr, d.moment_colind)
            acts = [a for a in range(m.nu) if np.any(mom[a, finger.dof] != 0)]
            lim = None
            if acts and all(m.actuator_forcelimited[a] for a in acts):
                lim = "actuator forcerange"
            return len(acts), lim, "model actuators"
        free = [j for j in finger.joints if j not in self.coupled]
        lim = "joint actuatorfrcrange" if all(m.jnt_actfrclimited[j] for j in free) else None
        return len(free), lim, "uncoupled joints (model has no actuators)"

    def workspace(self, finger, n=3000, seed=0):
        """Fingertip points (hand frame) over uniform joint samples within
        their ranges, couplings applied."""
        mj = _mj()
        rng = np.random.default_rng(seed)
        m, d = self.model, mj.MjData(self.model)
        pts = np.zeros((n, 3))
        lo, hi = finger.ranges[:, 0], finger.ranges[:, 1]
        for i in range(n):
            q = np.array(m.qpos0)
            q[finger.qadr] = rng.uniform(lo, hi)
            d.qpos[:] = self.apply_coupling(q)
            mj.mj_kinematics(m, d)
            pts[i] = finger.tip_pos(d)
        return pts


def hand_card(key, n=3000):
    """Summary per finger: joints, inputs, limits, reach extent (mm)."""
    h = Hand(key)
    out = {"key": key, "name": h.spec["name"], "model": os.path.relpath(h.path, ROOT),
           "n_joints": int(h.model.njnt), "n_actuators": int(h.model.nu),
           "shared_joints": [h.model.joint(j).name for j in h.shared_joints],
           "couplings": len(h.coupled), "fingers": []}
    for f in h.fingers:
        n_in, lim, src = h.inputs(f)
        pts = h.workspace(f, n)
        out["fingers"].append({
            "finger": f.label, "joints": len(f.joints), "inputs": n_in,
            "inputs_source": src, "torque_limits": lim,
            "reach_mm": float(1e3 * np.linalg.norm(pts - pts.mean(0), axis=1).max()),
            "workspace_extent_mm": (1e3 * (pts.max(0) - pts.min(0))).round(1).tolist(),
        })
    return out
