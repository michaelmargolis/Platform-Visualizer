# test_solver_diagnostics.py ver 1
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from forward_solver import solve_pose_from_muscle_lengths

class LinearKin:
    FIXED_HARDWARE_LENGTH = 0.0
    def inverse_kinematics(self, pose, return_lengths=False):
        lengths = np.asarray(pose, dtype=float)
        return (None, lengths) if return_lengths else None

kin = LinearKin()
target = np.array([1.0, -2.0, 3.0, 0.01, -0.02, 0.03], dtype=float)
bounds = np.array([10.0, 10.0, 10.0, 1.0, 1.0, 1.0], dtype=float)
pose, rms, diag = solve_pose_from_muscle_lengths(
    kin, target, np.zeros(6), bounds, return_diagnostics=True
)
assert rms < 0.15, rms
assert isinstance(diag["iterations"], int)
assert 0 <= diag["iterations"] <= diag["max_iterations"]
print(f'PASS: solver diagnostics iterations={diag["iterations"]}, rms={rms:.6f} mm')
