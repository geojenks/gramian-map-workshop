"""A soft finger from a gramian_map record (W20 step 6, registered model).

Registration: Wave_manip experiment_log/2026-10-21_w20s6_soft_finger_registration.md
section 3. A record is read, never edited. Every vector here is in the
record's canonical mounting frame (X = base -> tip, dominant small-signal
transverse output along +Y) until a mount rotates it into the world.

What the finger takes from its record:
  b    the PUSH direction: the direction of the blocked tip force at the
       record's highest blocked input level (rule order in `push_direction`)
  F    the blocked force magnitude at that level -- COMPARATIVE ONLY, never
       a strength claim (records are uncalibrated; registration section 1)
  d    the MOTION direction: scalars.free_direction (small-signal, unit)
  S    the stroke: scalars.free_stroke_m

One input per finger: the finger pushes only along b (f = s b, s >= 0) and
moves its tip only along d. Passive stiffness is not modelled.
"""
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(os.path.dirname(HERE), "data")


def _unit(v):
    v = np.asarray(v, float)
    n = float(np.linalg.norm(v))
    return None if n == 0.0 else v / n


def load_manifest():
    with open(os.path.join(DATA, "records", "MANIFEST.json")) as f:
        return json.load(f)


def load_record(rid_or_path):
    p = rid_or_path
    if not os.path.exists(p):
        p = os.path.join(DATA, "records",
                         load_manifest()["records"][rid_or_path]["file"])
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def push_direction(rec):
    """(b unit or None, F or None, rule name). Rules, first that applies:
    force_N        blocked.force_N, last level (the full C2 vector)
    axial_transverse  [force_axial_N, force_transverse_N, 0], last level;
                   the transverse magnitude is taken along canonical +Y
    polarity       Tier 0: [polarity_axial, 0, 0] with scalars.blocked_force_N
    none           no force data: the finger cannot push"""
    b = rec.get("blocked") or {}
    if b.get("force_N"):
        v = np.asarray(b["force_N"][-1], float)
        return _unit(v), float(np.linalg.norm(v)), "force_N"
    if b.get("force_axial_N"):
        tr = b.get("force_transverse_N") or [0.0] * len(b["force_axial_N"])
        v = np.array([b["force_axial_N"][-1], abs(tr[-1]), 0.0])
        return _unit(v), float(np.linalg.norm(v)), "axial_transverse"
    s = rec.get("scalars") or {}
    if s.get("blocked_force_N") and s.get("polarity_axial"):
        return (np.array([float(np.sign(s["polarity_axial"])), 0.0, 0.0]),
                float(abs(s["blocked_force_N"])), "polarity")
    return None, None, "none"


STIFF_AXES = (("k_axial_N_per_m", 0), ("k_N_per_m", 1),
              ("k_out_of_plane_N_per_m", 2))


def passive_stiffness(rec):
    """(k (3,) canonical [x, y, z] N/m, notes). Registration addendum
    (section 13): per axis, the MINIMUM over the record's probed input
    levels (a passive force available at every probed level); a negative
    minimum (unstable or probe artefact) counts as 0, a missing axis as 0.
    `k_N_per_m` is the transverse axis, which the records state as +Y."""
    st = rec.get("stiffness") or {}
    k = np.zeros(3)
    notes = []
    if not st:
        return k, ["no_stiffness_data"]
    if "+Y" not in str(st.get("direction", "+Y")):
        notes.append(f"transverse direction {st.get('direction')!r} read as +Y")
    for key, ax in STIFF_AXES:
        v = st.get(key)
        if not v:
            notes.append(f"k_{'xyz'[ax]} missing -> 0")
            continue
        m = float(min(v))
        if m < 0:
            notes.append(f"k_{'xyz'[ax]} min {m:.4g} < 0 -> 0")
            m = 0.0
        k[ax] = m
    return k, notes


class Finger:
    def __init__(self, rec, quality=None):
        s = rec.get("scalars") or {}
        self.id = rec["id"]
        self.name = rec.get("name", self.id)
        self.physics = (rec.get("metadata") or {}).get("actuation_physics")
        self.b, self.F, self.b_rule = push_direction(rec)
        fd = s.get("free_direction")
        self.d = None if fd is None else _unit(fd)
        self.S = s.get("free_stroke_m")
        self.k, self.k_notes = passive_stiffness(rec)
        self.quality = dict(quality or {})

    @classmethod
    def from_id(cls, rid):
        m = load_manifest()["records"][rid]
        return cls(load_record(rid), m.get("quality"))

    def flags(self):
        out = [f"{k}:{v}" for k, v in sorted(self.quality.items())]
        if self.b is None:
            out.append("no_force_data")
        if self.d is None:
            out.append("no_motion_direction")
        return out


def all_fingers(extra=()):
    """Every manifest record, then any extra record files (a participant's
    own actuator, written to the gramian_map record schema)."""
    m = load_manifest()["records"]
    out = [Finger(load_record(rid), e.get("quality")) for rid, e in m.items()]
    for p in extra:
        f = Finger(load_record(p))
        f.quality = {"source": "user record (not in the manifest)"}
        out.append(f)
    return out


# ----------------------------------------------------------------------
# mounts (registration section 4)
# ----------------------------------------------------------------------
def _frame(a, b):
    """Orthonormal right-handed frame [a, b', a x b'] with b' = b minus its
    a part (a, b need not be orthogonal)."""
    a = _unit(a)
    b = np.asarray(b, float) - float(np.dot(b, a)) * a
    b = _unit(b)
    return np.column_stack([a, b, np.cross(a, b)])


def canonical_pair(fing):
    """The finger's (push axis, second axis) in its own frame: b and the
    part of canonical +Y orthogonal to it (canonical +Z if b is along Y)."""
    b = fing.b if fing.b is not None else np.array([1.0, 0.0, 0.0])
    y = np.array([0.0, 1.0, 0.0])
    if abs(float(np.dot(b, y))) > 1.0 - 1e-9:
        y = np.array([0.0, 0.0, 1.0])
    return b, y


def mount_rotation(fing, style, n, e_r, e_phi, yaw_rad=0.0,
                   up=(0.0, 0.0, 1.0)):
    """World-from-canonical rotation for one finger at one contact.

    aimed  the push direction b points along the contact normal n (into the
           object); the finger's second axis points up the surface at
           yaw 0 and along the azimuth e_phi at yaw 90 deg.
    palm   the finger stands on a palm: canonical X is world up; canonical
           +Y points radially inward at yaw 0 and along e_phi at yaw 90 deg.
    """
    up = np.asarray(up, float)
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    if style == "aimed":
        b, y = canonical_pair(fing)
        m = up - float(np.dot(up, n)) * n
        m = m / np.linalg.norm(m)
        W = _frame(n, c * m + s * np.asarray(e_phi, float))
        C = _frame(b, y)
    elif style == "palm":
        W = _frame(up, c * (-np.asarray(e_r, float))
                   + s * np.asarray(e_phi, float))
        C = np.eye(3)
    else:
        raise ValueError(f"unknown mount style {style}")
    return W @ C.T
