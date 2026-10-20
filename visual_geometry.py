# visual_geometry.py ver 3
"""Load validated, precompiled platform visual geometry."""
from __future__ import annotations
from dataclasses import dataclass
import json
from pathlib import Path
import numpy as np


@dataclass(frozen=True)
class UprightVisual:
    center_xy: np.ndarray
    radius: float
    channels: tuple[int, int]


@dataclass(frozen=True)
class VisualGeometry:
    path: Path
    platform_id: str
    source_file: str
    source_sha256: str
    config_module: str
    moving_outline: np.ndarray
    moving_attachments: np.ndarray | None
    fixed_outline: np.ndarray
    fixed_frame_lines: tuple[np.ndarray, ...]
    fixed_attachments: np.ndarray
    uprights: tuple[UprightVisual, ...]


def _as_lines(items):
    lines = []
    for item in items or []:
        arr = np.asarray(item, dtype=float)
        if arr.shape != (2, 2):
            raise ValueError("Each fixed frame line must contain two XY points")
        lines.append(arr)
    return tuple(lines)


def load_visual_geometry(path):
    path = Path(path).expanduser().resolve()
    data = json.loads(path.read_text())
    version = int(data.get("format_version", 1))
    if version not in (1, 2):
        raise ValueError(f"Unsupported visual geometry format in {path}: {version}")

    moving = np.asarray(data["moving"]["outline_xy_mm"], dtype=float)
    fixed = np.asarray(data["fixed"]["outline_xy_mm"], dtype=float)
    attachments = np.asarray(data["fixed"]["attachments_xyz_mm"], dtype=float)
    moving_attachments_data = data.get("moving", {}).get("attachments_xyz_mm")
    moving_attachments = (
        np.asarray(moving_attachments_data, dtype=float)
        if moving_attachments_data is not None else None
    )
    frame_lines = _as_lines(data.get("fixed", {}).get("frame_lines_xy_mm", []))

    if moving.ndim != 2 or moving.shape[1] != 2 or len(moving) < 3:
        raise ValueError("Compiled moving outline is invalid")
    if fixed.ndim != 2 or fixed.shape[1] != 2 or len(fixed) < 3:
        raise ValueError("Compiled fixed outline is invalid")
    if attachments.shape != (6, 3):
        raise ValueError("Compiled fixed attachments must be 6x3")
    if moving_attachments is not None and moving_attachments.shape != (6, 3):
        raise ValueError("Compiled moving attachments must be 6x3")

    uprights = []
    for item in data["fixed"]["uprights"]:
        ch = tuple(int(v) for v in item["channels"])
        if len(ch) != 2:
            raise ValueError("Each compiled upright must reference exactly two channels")
        uprights.append(
            UprightVisual(
                np.asarray(item["center_xy_mm"], dtype=float),
                float(item["radius_mm"]),
                ch,
            )
        )
    if len(uprights) != 3:
        raise ValueError("Compiled visual geometry must contain exactly three uprights")

    source = data.get("source", {})
    source_file = str(source.get("file", source.get("dxf", "?")))
    return VisualGeometry(
        path=path,
        platform_id=str(data.get("platform_id", "platform")),
        source_file=source_file,
        source_sha256=str(source.get("sha256", "")),
        config_module=str(data.get("poc_config_module", "")),
        moving_outline=moving,
        moving_attachments=moving_attachments,
        fixed_outline=fixed,
        fixed_frame_lines=frame_lines,
        fixed_attachments=attachments,
        uprights=tuple(uprights),
    )
