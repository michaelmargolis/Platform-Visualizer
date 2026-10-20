# build_platform.py ver 3
"""Compile semantic platform DXF geometry into POC-coordinate visual.json."""
from __future__ import annotations

import argparse
import hashlib
import importlib
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np

from semantic_dxf import load_platform_dxf, resolve_muscle_pairs

FORMAT_VERSION = 1


def similarity_fit(src, dst, reflected=False):
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    work = src.copy()
    if reflected:
        work[:, 0] *= -1.0
    sc, dc = work.mean(axis=0), dst.mean(axis=0)
    x, y = work - sc, dst - dc
    u, _s, vt = np.linalg.svd(x.T @ y)
    r = u @ vt
    if np.linalg.det(r) < 0.0:
        u[:, -1] *= -1.0
        r = u @ vt
    denom = float(np.sum(x * x))
    if denom <= 1e-12:
        raise ValueError("Cannot fit degenerate attachment geometry")
    scale = float(np.sum((x @ r) * y) / denom)
    offset = dc - scale * (sc @ r)
    predicted = scale * (work @ r) + offset
    rms = float(np.sqrt(np.mean(np.sum((predicted - dst) ** 2, axis=1))))
    return {"rms": rms, "scale": scale, "rotation": r, "offset": offset, "reflected": reflected}


def transform_points(points, fit):
    p = np.asarray(points, dtype=float)
    if fit["reflected"]:
        p = p.copy(); p[:, 0] *= -1.0
    return fit["scale"] * (p @ fit["rotation"]) + fit["offset"]


def load_poc_geometry(poc_root, config_module):
    root = Path(poc_root).expanduser().resolve()
    if not root.exists():
        raise FileNotFoundError(root)
    sys.path.insert(0, str(root))
    try:
        module = importlib.import_module(config_module)
    finally:
        if sys.path and sys.path[0] == str(root):
            sys.path.pop(0)
    cfg = module.PlatformConfig()
    base = np.asarray(cfg.base_coords, dtype=float)
    moving = np.asarray(getattr(cfg, "platform_coords", getattr(cfg, "platform_coords_xy", None)), dtype=float)
    if base.shape != (6, 3) or moving.ndim != 2 or moving.shape[0] != 6 or moving.shape[1] not in (2, 3):
        raise ValueError(f"{config_module}: expected six base XYZ and six moving XY/XYZ points")
    return cfg, base, moving


def resolve_channel_mapping(model, muscle_pairs, cfg_base_xy, cfg_moving_xy):
    msrc = np.asarray([c.center for c in model.moving_attachments], dtype=float)
    fsrc = np.asarray([c.center for c in model.fixed_attachments], dtype=float)
    origin_index = int(np.argmin(np.linalg.norm(msrc, axis=1)))
    origin_distance = float(np.linalg.norm(msrc[origin_index]))
    origin_tol = max(25.0, 3.0 * min(c.radius for c in model.moving_attachments))
    if origin_distance > origin_tol:
        raise ValueError(
            f"No MOVING_ATTACHMENTS circle identifies channel 0 near DXF origin; nearest is {origin_distance:.1f} mm away"
        )

    fixed_for_moving = {m: f for m, f in muscle_pairs}
    if len(fixed_for_moving) != 6:
        raise ValueError("Could not resolve exactly one fixed attachment for every moving attachment")

    others = [i for i in range(6) if i != origin_index]
    candidates = []
    for tail in itertools.permutations((1, 2, 3, 4, 5)):
        moving_to_channel = [None] * 6
        moving_to_channel[origin_index] = 0
        for mi, channel in zip(others, tail):
            moving_to_channel[mi] = channel
        ordered_m = np.asarray([msrc[moving_to_channel.index(ch)] for ch in range(6)])
        ordered_f = np.asarray([fsrc[fixed_for_moving[moving_to_channel.index(ch)]] for ch in range(6)])
        for reflected in (False, True):
            mf = similarity_fit(ordered_m, cfg_moving_xy, reflected)
            ff = similarity_fit(ordered_f, cfg_base_xy, reflected)
            candidates.append((mf["rms"] + ff["rms"], moving_to_channel, mf, ff))

    candidates.sort(key=lambda x: x[0])
    best = candidates[0]
    second = candidates[1]
    if second[0] - best[0] < 50.0 and second[0] < best[0] * 2.0:
        raise ValueError(
            f"DXF channel mapping is ambiguous: best score {best[0]:.1f} mm, second {second[0]:.1f} mm"
        )
    return origin_index, best, second


def assign_uprights(uprights_xy, cfg_base_xy):
    uprights_xy = np.asarray(uprights_xy, dtype=float)
    assignments = [[] for _ in range(len(uprights_xy))]
    for ch, p in enumerate(cfg_base_xy):
        ds = np.linalg.norm(uprights_xy - p, axis=1)
        assignments[int(np.argmin(ds))].append(ch)
    if sorted(len(a) for a in assignments) != [2, 2, 2]:
        raise ValueError(f"Expected each upright to serve exactly two fixed attachments; got {assignments}")
    return assignments


def clean_points(arr):
    return [[round(float(x), 6), round(float(y), 6)] for x, y in np.asarray(arr)]


def compile_platform(dxf_path, poc_root, config_module, platform_id=None):
    dxf_path = Path(dxf_path).resolve()
    model = load_platform_dxf(dxf_path)
    muscle_pairs = resolve_muscle_pairs(model)
    cfg, cfg_base, cfg_moving = load_poc_geometry(poc_root, config_module)

    origin_index, best, second = resolve_channel_mapping(model, muscle_pairs, cfg_base[:, :2], cfg_moving[:, :2])
    _score, moving_to_channel, moving_fit, fixed_fit = best
    fixed_for_moving = {m: f for m, f in muscle_pairs}

    moving_outline = transform_points(model.moving_outline.points, moving_fit)
    fixed_outline = transform_points(model.fixed_outline.points, fixed_fit)
    upright_centers = transform_points([u.center for u in model.uprights], fixed_fit)
    upright_radii = [abs(fixed_fit["scale"]) * u.radius for u in model.uprights]
    upright_channels = assign_uprights(upright_centers, cfg_base[:, :2])

    channel_rows = []
    for ch in range(6):
        mi = moving_to_channel.index(ch)
        fi = fixed_for_moving[mi]
        channel_rows.append({"channel": ch, "moving_dxf_index": mi, "fixed_dxf_index": fi})

    digest = hashlib.sha256(dxf_path.read_bytes()).hexdigest()
    source_name = dxf_path.name
    return {
        "format_version": FORMAT_VERSION,
        "platform_id": platform_id or str(getattr(cfg, "PLATFORM_NAME", config_module)),
        "poc_config_module": config_module,
        "source": {"dxf": source_name, "sha256": digest},
        "validation": {
            "channel0_moving_dxf_index": origin_index,
            "mapping_score_mm": round(float(best[0]), 6),
            "next_mapping_score_mm": round(float(second[0]), 6),
            "moving_fit_rms_mm": round(float(moving_fit["rms"]), 6),
            "fixed_fit_rms_mm": round(float(fixed_fit["rms"]), 6),
            "reflected": bool(moving_fit["reflected"]),
            "moving_scale": round(float(moving_fit["scale"]), 9),
            "fixed_scale": round(float(fixed_fit["scale"]), 9),
            "channels": channel_rows,
        },
        "moving": {
            "outline_xy_mm": clean_points(moving_outline),
            "attachments_xy_mm": clean_points(cfg_moving[:, :2]),
        },
        "fixed": {
            "outline_xy_mm": clean_points(fixed_outline),
            "attachments_xyz_mm": [[round(float(v), 6) for v in p] for p in cfg_base],
            "uprights": [
                {
                    "center_xy_mm": [round(float(c[0]), 6), round(float(c[1]), 6)],
                    "radius_mm": round(float(r), 6),
                    "channels": list(map(int, chans)),
                }
                for c, r, chans in zip(upright_centers, upright_radii, upright_channels)
            ],
        },
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dxf", required=True)
    ap.add_argument("--poc-root", required=True)
    ap.add_argument("--config-module", default="kinematics.cfg_SuspendedPlatformMdx")
    ap.add_argument("--platform-id")
    ap.add_argument("--output", required=True)
    args = ap.parse_args()

    result = compile_platform(args.dxf, args.poc_root, args.config_module, args.platform_id)
    source = Path(args.dxf).expanduser().resolve()
    out = Path(args.output).expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + "\n")

    v = result["validation"]
    print(f"Source DXF: {source}")
    print(f"Source SHA256: {result["source"]["sha256"]}")
    print(f"Output JSON: {out}")
    print("PASS: semantic DXF contains 1 moving outline, 6 moving attachments, 1 fixed outline, 6 fixed attachments, 3 uprights, 6 muscles")
    print(f"PASS: channel 0 identified at moving DXF attachment index {v['channel0_moving_dxf_index']}")
    print(f"PASS: channel mapping score {v['mapping_score_mm']:.2f} mm; next candidate {v['next_mapping_score_mm']:.2f} mm")
    print(f"Moving fit RMS: {v['moving_fit_rms_mm']:.2f} mm   Fixed fit RMS: {v['fixed_fit_rms_mm']:.2f} mm")
    print(f"DXF reflection used: {v['reflected']}   (output geometry is now in POC coordinates)")
    print("Channel mapping:")
    for row in v["channels"]:
        print(f"  {row['channel']}: moving DXF {row['moving_dxf_index']} -> fixed DXF {row['fixed_dxf_index']}")


if __name__ == "__main__":
    main()
