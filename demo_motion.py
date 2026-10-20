# demo_motion.py ver 2
"""Repeatable six-axis demo using the active POC kinematics motion limits."""
from __future__ import annotations
import math
import numpy as np


class CannedMotion:
    # name, pose index, seconds; amplitudes come from platform kinematics cfg.
    STAGES = (
        ("HEAVE", 2, 2.0),
        ("ROLL", 3, 2.0),
        ("PITCH", 4, 2.0),
        ("YAW", 5, 2.0),
        ("SURGE", 0, 2.0),
        ("SWAY", 1, 2.0),
    )

    def __init__(self, dof_limits):
        limits = np.asarray(dof_limits, dtype=float)
        if limits.shape != (6,) or not np.all(np.isfinite(limits)) or np.any(limits <= 0):
            raise ValueError("CannedMotion requires six finite positive DOF limits")
        # Translation limits are mm; angular limits are radians, matching POC.
        self.stages = tuple((name, idx, float(limits[idx]), duration)
                            for name, idx, duration in self.STAGES)
        self.period = sum(stage[3] for stage in self.stages)

    def pose(self, elapsed):
        t = elapsed % self.period
        pose = np.zeros(6, dtype=float)
        acc = 0.0
        for name, idx, amplitude, duration in self.stages:
            if t < acc + duration:
                phase = (t - acc) / duration
                # Starts and ends at neutral; reaches both signs within each stage.
                pose[idx] = amplitude * math.sin(2.0 * math.pi * phase)
                return pose, name
            acc += duration
        return pose, "NEUTRAL"
