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

    def subset(self, mask):
        idx = np.where(mask)[0]
        c = self.cup
        return (c.points[idx], c.normals[idx], [self.B[i] for i in idx],
                [self.F[i] for i in idx], [self.D[i] for i in idx])
