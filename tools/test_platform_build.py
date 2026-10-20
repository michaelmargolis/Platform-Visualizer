# test_platform_build.py ver 3
"""Regression check for the bundled MDX SOLIDWORKS geometry build."""
from __future__ import annotations

import argparse
from pathlib import Path
import tempfile

from build_platform_from_sw_json import compile_geometry

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poc-root", required=True)
    args = ap.parse_args()

    result = compile_geometry(
        ROOT / "platforms" / "mdx_suspended" / "source.json",
        args.poc_root,
        "kinematics.cfg_SuspendedPlatformMdx",
        84.0,
        50.0,
    )
    assert len(result["fixed"]["attachments_xyz_mm"]) == 6
    assert len(result["moving"]["attachments_xyz_mm"]) == 6
    assert len(result["fixed"]["uprights"]) == 3
    assert len(result["fixed"]["frame_lines_xy_mm"]) >= 4
    assert len(result["moving"]["outline_xy_mm"]) >= 20
    print("PASS: SOLIDWORKS JSON compiles to MDX platform visual geometry")


if __name__ == "__main__":
    main()
