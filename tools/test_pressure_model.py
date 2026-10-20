# test_pressure_model.py ver 1
"""Regression checks for the Unity-compatible pressure model and slew limiter."""
from __future__ import annotations

import tempfile
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from pressure_model import EmpiricalPressureModel, MuscleDynamics

HEADER = "cycle,dir,step,pressure,d0,d1,d2,d3,d4,d5,t0,t1,t2,t3,t4,t5\n"


def row(direction, pressure, contraction):
    values = ",".join([str(contraction)] * 6)
    return f"0,{direction},0,{pressure},{values},0,0,0,0,0,0\n"


def make_curve(path, load, rising_zero, rising_full, falling_zero, falling_full):
    path.write_text(
        f"WEIGHT,{load}\n" + HEADER
        + row(0, 0, rising_zero) + row(0, 6000, rising_full)
        + row(1, 0, falling_zero) + row(1, 6000, falling_full),
        encoding="utf-8",
    )


def main():
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        low, high = td / "low.csv", td / "high.csv"
        make_curve(low, 40, 0, 100, 0, 120)
        make_curve(high, 80, 0, 200, 0, 220)
        model = EmpiricalPressureModel([low, high])
        estimate = model.estimate([3000] * 6, 60)
        assert abs(estimate.contractions_mm[0] - 75.0) < 1e-9

        falling_file = td / "falling.csv"
        make_curve(falling_file, 40, 0, 100, 20, 140)
        model = EmpiricalPressureModel([falling_file])
        model.estimate([4000] * 6, 40)
        falling = model.estimate([3000] * 6, 40)
        assert falling.directions[0] == 1
        assert abs(falling.contractions_mm[0] - 80.0) < 1e-9

    dynamics = MuscleDynamics(True, 100.0, 40.0)
    dynamics.step([50] * 6, 0.0)
    result = dynamics.step([100, 0, 100, 0, 100, 0], 0.05)
    assert result == [55.0, 48.0, 55.0, 48.0, 55.0, 48.0]

    real_model = EmpiricalPressureModel([
        ROOT / "pressure_data" / "PtoD_40.csv",
        ROOT / "pressure_data" / "PtoD_80.csv",
    ])
    real = real_model.estimate([3000] * 6, 40.0)
    assert len(real.contractions_mm) == 6
    assert all(0.0 <= v <= 250.0 for v in real.contractions_mm)

    print("PASS: pressure interpolation/load interpolation matches Unity regression cases")
    print("PASS: rising/falling direction selection matches Unity regression case")
    print("PASS: separate contraction/extension slew rates match Unity regression case")
    print("PASS: bundled PtoD_40/PtoD_80 tables parse and produce six finite contractions")


if __name__ == "__main__":
    main()
