"""Task files (data/tasks/*.json, written by tools/export_tasks.py) and the
cup's contact geometry."""
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TASKS = os.path.join(os.path.dirname(HERE), "data", "tasks")


def load_task(tid):
    with open(os.path.join(TASKS, f"{tid}.json")) as f:
        return json.load(f)


class Cup:
    """N contacts equally spaced in azimuth on one ring of a sphere.

    com is the origin. altitude < 0 is below the centre (George's cup)."""

    def __init__(self, task, n, phase0_rad=0.0):
        self.task = task
        self.R = float(task["object"]["radius_m"])
        self.alt = np.radians(float(task["contact_ring"]["altitude_deg"]))
        self.mu = float(task["mu"])
        self.L = float(task["torque_scale_L_m"])
        self.n = int(n)
        self.com = np.zeros(3)
        phi = phase0_rad + 2.0 * np.pi * np.arange(self.n) / max(self.n, 1)
        self.phi = phi
        ca, sa = np.cos(self.alt), np.sin(self.alt)
        self.e_r = np.column_stack([np.cos(phi), np.sin(phi), 0 * phi])
        self.e_phi = np.column_stack([-np.sin(phi), np.cos(phi), 0 * phi])
        z = np.array([0.0, 0.0, 1.0])
        self.points = self.R * (ca * self.e_r + sa * z)
        self.normals = -(ca * self.e_r + sa * z)       # into the object

    def weight_demand(self):
        """The static weight: m g upward in the world, zero moment about the
        centre of mass (W20 step 4 demand (ii))."""
        m = float(self.task["object"]["mass_kg"])
        g = float(self.task["gravity_m_s2"])
        return np.array([0.0, 0.0, m * g, 0.0, 0.0, 0.0])
