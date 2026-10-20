# forward_solver.py ver 2
"""Bounded NumPy forward-pose solver used by UDP mode."""
from __future__ import annotations
import math
import numpy as np


def solve_pose_from_muscle_lengths(kin, target_muscle_lengths, seed, bounds, max_iter=35, return_diagnostics=False):
    target = np.asarray(target_muscle_lengths, dtype=float)
    pose = np.clip(np.asarray(seed, dtype=float).copy(), -bounds, bounds)
    steps = np.array([0.10, 0.10, 0.10, 1e-4, 1e-4, 1e-4], dtype=float)
    damping = 1e-3

    def residual(p):
        actuator_lengths = np.asarray(kin.inverse_kinematics(p, return_lengths=True)[1], dtype=float)
        return actuator_lengths - float(kin.FIXED_HARDWARE_LENGTH) - target

    r = residual(pose)
    cost = float(r @ r)
    iterations = 0
    for _ in range(max_iter):
        if math.sqrt(cost / 6.0) <= 0.15:
            break
        iterations += 1
        jac = np.zeros((6, 6), dtype=float)
        for col in range(6):
            plus, minus = pose.copy(), pose.copy()
            plus[col] = min(bounds[col], plus[col] + steps[col])
            minus[col] = max(-bounds[col], minus[col] - steps[col])
            den = plus[col] - minus[col]
            if abs(den) > 1e-12:
                jac[:, col] = (residual(plus) - residual(minus)) / den
        normal = jac.T @ jac
        normal += np.diag(damping * np.maximum(np.diag(normal), 1.0))
        try:
            delta = np.linalg.solve(normal, -(jac.T @ r))
        except np.linalg.LinAlgError:
            damping *= 10.0
            continue
        candidate = np.clip(pose + delta, -bounds, bounds)
        cr = residual(candidate)
        ccost = float(cr @ cr)
        if ccost < cost:
            pose, r, cost = candidate, cr, ccost
            damping = max(1e-9, damping * 0.3)
        else:
            damping = min(1e12, damping * 10.0)
    rms = math.sqrt(cost / 6.0)
    if return_diagnostics:
        return pose, rms, {"iterations": iterations, "max_iterations": int(max_iter)}
    return pose, rms
