"""Copies of one record's finger arranged on the cup ring (registration
sections 4 and 5): world push and motion directions per contact, and the
per-finger drive metrics for the turn."""
import math

import numpy as np

from .finger import mount_rotation
from .task import Cup


class Arrangement:
    def __init__(self, finger, task, n, style="aimed", yaw_deg=0.0,
                 phase0_rad=0.0):
        self.finger = finger
        self.cup = Cup(task, n, phase0_rad)
        self.style = style
        self.yaw_deg = float(yaw_deg)
        c = self.cup
        self.R = [mount_rotation(finger, style, c.normals[i], c.e_r[i],
                                 c.e_phi[i], math.radians(yaw_deg))
                  for i in range(c.n)]
        self.B = [None if finger.b is None else R @ finger.b for R in self.R]
        self.D = [None if finger.d is None else R @ finger.d for R in self.R]
        self.F = [finger.F] * c.n

    def passive(self, delta_max_m):
        """Per contact, the world passive force at full deflection: one
        column R e_j k_j delta_max per canonical axis with k_j > 0
        (addendum section 13.1)."""
        k = self.finger.k
        axes = [j for j in range(3) if k[j] > 0] if delta_max_m > 0 else []
        return [np.column_stack([R[:, j] * k[j] * delta_max_m for j in axes])
                if axes else np.zeros((3, 0)) for R in self.R]

    def drive(self, i):
        """(azimuthal share, off-axis fraction) of finger i's tangent-plane
        motion, assuming it complies freely along the contact normal."""
        d = self.D[i]
        if d is None:
            return 0.0, 1.0
        n, e_phi = self.cup.normals[i], self.cup.e_phi[i]
        pt = d - float(np.dot(d, n)) * n
        npt = float(np.linalg.norm(pt))
        if npt == 0.0:
            return 0.0, 1.0
        along = float(np.dot(pt, e_phi))
        return along, float(np.linalg.norm(pt - along * e_phi)) / npt

    def turn_per_stroke_deg(self, i=0):
        if self.finger.S is None:
            return None
        along, _ = self.drive(i)
        return math.degrees(self.finger.S * abs(along)
                            / (self.cup.R * math.cos(self.cup.alt)))

    def subset(self, mask, delta_max_m=None):
        idx = np.where(mask)[0]
        c = self.cup
        out = (c.points[idx], c.normals[idx], [self.B[i] for i in idx],
               [self.F[i] for i in idx], [self.D[i] for i in idx])
        if delta_max_m is None:
            return out
        Q = self.passive(delta_max_m)
        return out + ([Q[i] for i in idx],)
