# test_solver.py ver 3
from pathlib import Path
import argparse, sys, math
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from poc_platform import PocPlatform
from forward_solver import solve_pose_from_muscle_lengths
ap=argparse.ArgumentParser(); ap.add_argument("--poc-root",required=True); a=ap.parse_args()
p=PocPlatform(a.poc_root)
truth=np.array([18.,-12.,25.,math.radians(3),math.radians(-2),math.radians(2.5)])
_pts, actuator=p.kin.inverse_kinematics(truth,return_lengths=True)
target=np.asarray(actuator)-p.kin.FIXED_HARDWARE_LENGTH
solved,rms=solve_pose_from_muscle_lengths(p.kin,target,np.zeros(6),p.bounds)
assert rms < 0.05, rms
print(f"PASS: {p.selected_platform} forward solver residual {rms:.4f} mm")
