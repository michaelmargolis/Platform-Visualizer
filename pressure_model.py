# pressure_model.py ver 1
"""Unity-compatible empirical pressure-to-contraction model and simple muscle slew limiter."""
from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path


@dataclass
class PressureEstimate:
    contractions_mm: list[float]
    directions: list[int]
    saturated: list[bool]


class _LoadCurves:
    def __init__(self, load: float):
        self.load = float(load)
        # direction -> pressure -> six averaged contractions
        self.directions: list[dict[int, list[float]]] = [{}, {}]


class EmpiricalPressureModel:
    """Port of the current Unity EmpiricalPressureModel behaviour."""

    def __init__(self, csv_files, direction_deadband_mbar: float = 25.0):
        paths = [Path(p) for p in csv_files]
        if not paths:
            raise ValueError("At least one P-to-D CSV is required")
        self.loads = [self._parse(path) for path in paths]
        self.loads.sort(key=lambda c: c.load)
        self.direction_deadband_mbar = float(direction_deadband_mbar)
        self.previous_pressures = [0.0] * 6
        self.directions = [0] * 6
        self.has_previous = False

    def reset(self):
        self.previous_pressures = [0.0] * 6
        self.directions = [0] * 6
        self.has_previous = False

    def estimate(self, pressures_mbar, requested_load: float) -> PressureEstimate:
        if pressures_mbar is None or len(pressures_mbar) != 6:
            raise ValueError("Six pressure values are required")
        pressures = [float(v) for v in pressures_mbar]
        contractions = [0.0] * 6
        saturated = [False] * 6
        for muscle, pressure in enumerate(pressures):
            if self.has_previous:
                delta = pressure - self.previous_pressures[muscle]
                if delta > self.direction_deadband_mbar:
                    self.directions[muscle] = 0  # recorded rising sweep
                elif delta < -self.direction_deadband_mbar:
                    self.directions[muscle] = 1  # recorded falling sweep
            direction = self.directions[muscle]
            saturated[muscle] = (
                pressure <= 0.0
                or pressure >= 6000.0
                or requested_load <= self.loads[0].load
                or requested_load >= self.loads[-1].load
            )
            contractions[muscle] = self._interpolate_load(
                pressure, muscle, direction, float(requested_load)
            )
            self.previous_pressures[muscle] = pressure
        self.has_previous = True
        return PressureEstimate(contractions, list(self.directions), saturated)

    def _interpolate_load(self, pressure: float, muscle: int, direction: int, requested_load: float) -> float:
        if requested_load <= self.loads[0].load:
            return self._interpolate_pressure(self.loads[0], pressure, muscle, direction)
        if requested_load >= self.loads[-1].load:
            return self._interpolate_pressure(self.loads[-1], pressure, muscle, direction)
        upper = 1
        while upper < len(self.loads) and self.loads[upper].load < requested_load:
            upper += 1
        low, high = self.loads[upper - 1], self.loads[upper]
        fraction = (requested_load - low.load) / (high.load - low.load)
        low_value = self._interpolate_pressure(low, pressure, muscle, direction)
        high_value = self._interpolate_pressure(high, pressure, muscle, direction)
        return low_value + (high_value - low_value) * fraction

    @staticmethod
    def _interpolate_pressure(curve: _LoadCurves, pressure: float, muscle: int, direction: int) -> float:
        points = curve.directions[direction]
        keys = sorted(points)
        if not keys:
            raise ValueError("Pressure curve is empty")
        lower = keys[0]
        upper = keys[-1]
        for key in keys:
            if key <= pressure:
                lower = key
            if key >= pressure:
                upper = key
                break
        low = points[lower][muscle]
        high = points[upper][muscle]
        if lower == upper:
            return low
        fraction = (pressure - lower) / (upper - lower)
        return low + (high - low) * fraction

    @staticmethod
    def _parse(path: Path) -> _LoadCurves:
        if not path.is_file():
            raise FileNotFoundError(f"Pressure table not found: {path}")
        load = None
        # direction -> pressure -> [sum[6], count]
        accum: list[dict[int, tuple[list[float], int]]] = [{}, {}]
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            for fields in csv.reader(stream):
                if not fields:
                    continue
                if fields[0].strip().upper() == "WEIGHT":
                    if len(fields) < 2:
                        raise ValueError(f"Missing WEIGHT value in {path}")
                    load = float(fields[1])
                    continue
                try:
                    int(fields[0])
                except (ValueError, TypeError):
                    continue
                if len(fields) < 10:
                    continue
                try:
                    direction = int(fields[1])
                    pressure = int(fields[3])
                    values = [float(fields[4 + i]) for i in range(6)]
                except (ValueError, IndexError):
                    continue
                if direction not in (0, 1):
                    continue
                if pressure not in accum[direction]:
                    accum[direction][pressure] = ([0.0] * 6, 0)
                sums, count = accum[direction][pressure]
                for i, value in enumerate(values):
                    sums[i] += value
                accum[direction][pressure] = (sums, count + 1)
        if load is None or load <= 0:
            raise ValueError(f"P-to-D CSV is missing its load: {path}")
        curve = _LoadCurves(load)
        for direction in (0, 1):
            if not accum[direction]:
                raise ValueError(f"P-to-D CSV is missing direction {direction}: {path}")
            for pressure, (sums, count) in accum[direction].items():
                curve.directions[direction][pressure] = [v / count for v in sums]
        return curve


class MuscleDynamics:
    """Port of Unity MuscleDynamics: slew contraction targets before the pose solve."""

    def __init__(self, enabled=True, contraction_rate_mm_s=100.0, extension_rate_mm_s=100.0):
        self.enabled = bool(enabled)
        self.contraction_rate_mm_s = float(contraction_rate_mm_s)
        self.extension_rate_mm_s = float(extension_rate_mm_s)
        self.current = [0.0] * 6
        self.initialized = False

    def reset(self):
        self.current = [0.0] * 6
        self.initialized = False

    def step(self, targets_mm, delta_time_seconds: float) -> list[float]:
        if targets_mm is None or len(targets_mm) != 6:
            raise ValueError("Six contraction targets are required")
        targets = [float(v) for v in targets_mm]
        dt = float(delta_time_seconds)
        if (not self.initialized) or (not self.enabled) or dt <= 0.0:
            self.current = list(targets)
            self.initialized = True
            return list(self.current)

        contraction_step = max(0.0, self.contraction_rate_mm_s) * dt
        extension_step = max(0.0, self.extension_rate_mm_s) * dt
        for i, target in enumerate(targets):
            difference = target - self.current[i]
            limit = contraction_step if difference >= 0.0 else extension_step
            change = max(-limit, min(limit, difference))
            self.current[i] += change
        return list(self.current)
