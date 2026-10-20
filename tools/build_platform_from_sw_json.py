# build_platform_from_sw_json.py ver 2
"""Compile SOLIDWORKS platform-geometry JSON into visualizer geometry."""
from __future__ import annotations

import argparse
import hashlib
import importlib
import json
from pathlib import Path
import sys

import numpy as np

FORMAT_VERSION = 2
DEFAULT_Z_OFFSET_MM = 84.0
FRAME_PLAN_X_OFFSET_MM = 30.0
DEFAULT_UPRIGHT_RADIUS_MM = 50.0


def convex_hull(points):
    pts = sorted(set((float(x), float(y)) for x, y in points))
    if len(pts) <= 1:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def xyz_rows(items, z_offset):
    return np.asarray(
        [[float(p["x"]), float(p["y"]), float(p["z"]) - z_offset] for p in items],
        dtype=float,
    )


def frame_plan_point(point):
    # Structural-member export is in the weldment sketch plane. This is the
    # established conversion used by platform_geometry_from_sw_export_v3.py.
    return [
        float(point["x"]) - float(point["z"]) + FRAME_PLAN_X_OFFSET_MM,
        float(point["y"]),
    ]


def assign_uprights(uprights_xy, base_xy):
    assignments = [[] for _ in uprights_xy]
    uprights_xy = np.asarray(uprights_xy, dtype=float)
    for ch, point in enumerate(np.asarray(base_xy, dtype=float)):
        index = int(np.argmin(np.linalg.norm(uprights_xy - point, axis=1)))
        assignments[index].append(ch)
    if sorted(len(x) for x in assignments) != [2, 2, 2]:
        raise ValueError(f"Expected each upright to serve two channels; got {assignments}")
    return assignments


def load_poc_geometry(poc_root, config_module):
    root = Path(poc_root).expanduser().resolve()
    sys.path.insert(0, str(root))
    try:
        module = importlib.import_module(config_module)
        cfg = module.PlatformConfig()
    finally:
        if sys.path and sys.path[0] == str(root):
            sys.path.pop(0)
    return (
        cfg,
        np.asarray(cfg.base_coords, dtype=float),
        np.asarray(cfg.platform_coords, dtype=float),
    )


def clean_xyz(arr):
    return [[round(float(v), 6) for v in row] for row in np.asarray(arr)]


def clean_xy(arr):
    return [[round(float(v), 6) for v in row] for row in np.asarray(arr)]


def compile_geometry(source_path, poc_root, config_module, z_offset, upright_radius):
    source_path = Path(source_path).expanduser().resolve()
    data = json.loads(source_path.read_text())
    fixed = xyz_rows(data.get("fixed_attachments", []), z_offset)
    moving = xyz_rows(data.get("moving_attachments", []), z_offset)
    if fixed.shape != (6, 3) or moving.shape != (6, 3):
        raise ValueError("Source JSON must contain six fixed and six moving attachments")

    cfg, cfg_fixed, cfg_moving = load_poc_geometry(poc_root, config_module)
    if cfg_fixed.shape != (6, 3) or cfg_moving.shape != (6, 3):
        raise ValueError(f"{config_module} must contain six explicit XYZ points")
    if not np.allclose(fixed, cfg_fixed, atol=0.01):
        raise ValueError("CAD fixed attachments do not match the POC MDX config")
    if not np.allclose(moving, cfg_moving, atol=0.01):
        raise ValueError("CAD moving attachments do not match the POC MDX config")

    frame_lines = []
    frame_points = []
    for line in data.get("frame_lines", []):
        p1 = frame_plan_point(line["start"])
        p2 = frame_plan_point(line["end"])
        frame_lines.append([p1, p2])
        frame_points.extend((p1, p2))
    if not frame_lines:
        raise ValueError("Source JSON contains no frame lines")

    uprights = data.get("uprights", [])
    if len(uprights) != 3:
        raise ValueError("Source JSON must contain three uprights")
    upright_xy = [[float(p["x"]), float(p["y"])] for p in uprights]
    assignments = assign_uprights(upright_xy, fixed[:, :2])

    fixed_outline = convex_hull(frame_points)
    configured_outline = getattr(cfg, "PLATFORM_UNDERSIDE_HULL_XY", None)
    moving_outline = (
        [[float(x), float(y)] for x, y in configured_outline]
        if configured_outline
        else convex_hull(moving[:, :2])
    )
    digest = hashlib.sha256(source_path.read_bytes()).hexdigest()

    return {
        "format_version": FORMAT_VERSION,
        "platform_id": "MDX Suspended Platform",
        "poc_config_module": config_module,
        "source": {"file": source_path.name, "sha256": digest},
        "moving": {
            "outline_xy_mm": clean_xy(moving_outline),
            "attachments_xyz_mm": clean_xyz(moving),
        },
        "fixed": {
            "outline_xy_mm": clean_xy(fixed_outline),
            "frame_lines_xy_mm": [clean_xy(line) for line in frame_lines],
            "attachments_xyz_mm": clean_xyz(fixed),
            "uprights": [
                {
                    "center_xy_mm": [round(float(p[0]), 6), round(float(p[1]), 6)],
                    "radius_mm": round(float(upright_radius), 6),
                    "channels": list(map(int, channels)),
                }
                for p, channels in zip(upright_xy, assignments)
            ],
        },
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", required=True)
    ap.add_argument("--poc-root", required=True)
    ap.add_argument("--config-module", default="kinematics.cfg_SuspendedPlatformMdx")
    ap.add_argument("--z-offset-mm", type=float, default=DEFAULT_Z_OFFSET_MM)
    ap.add_argument("--upright-radius-mm", type=float, default=DEFAULT_UPRIGHT_RADIUS_MM)
    ap.add_argument("--output", required=True)
    args = ap.parse_args()

    result = compile_geometry(
        args.source, args.poc_root, args.config_module, args.z_offset_mm, args.upright_radius_mm
    )
    out = Path(args.output).expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + "\n")
    print(f"Source JSON: {Path(args.source).resolve()}")
    print(f"Output JSON: {out}")
    print("PASS: 6 fixed points, 6 moving points, 3 uprights and frame lines compiled")


if __name__ == "__main__":
    main()
