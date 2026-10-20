# test_moving_platform_egg.py ver 2
"""Static sanity checks for the bundled moving-platform EGG assets."""
from __future__ import annotations

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def parse_vertices(text):
    vertices = {}
    for m in re.finditer(
        r"<Vertex>\s+(\d+)\s*\{\s*([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s*\}",
        text,
    ):
        vertices[int(m.group(1))] = tuple(float(m.group(i)) for i in range(2, 5))
    return vertices


def check_egg(path, expected_min_z, expected_max_z, min_vertices):
    text = path.read_text(encoding="ascii")
    assert text.count("{") == text.count("}"), f"unbalanced EGG braces: {path.name}"
    assert "<CoordinateSystem> { Z-up }" in text
    assert "<VertexPool> platform" in text
    vertices = parse_vertices(text)
    assert len(vertices) >= min_vertices, f"too few vertices in {path.name}: {len(vertices)}"
    xs = [p[0] for p in vertices.values()]
    ys = [p[1] for p in vertices.values()]
    zs = [p[2] for p in vertices.values()]
    assert min(zs) == expected_min_z and max(zs) == expected_max_z, (
        path.name, min(zs), max(zs)
    )
    assert max(xs) > min(xs) and max(ys) > min(ys)
    print(
        f"EGG sanity OK: {path.name}: {len(vertices)} vertices, "
        f"bbox X={min(xs):.1f}..{max(xs):.1f}, "
        f"Y={min(ys):.1f}..{max(ys):.1f}, Z={min(zs):.1f}..{max(zs):.1f}"
    )


def main() -> int:
    check_egg(ROOT / "assets" / "mdx_moving_platform.egg", -40.0, 0.0, 184)
    check_egg(ROOT / "assets" / "chair_v2_moving_platform.egg", -40.0, 0.0, 20)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
