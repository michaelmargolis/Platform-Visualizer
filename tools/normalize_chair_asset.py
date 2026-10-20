# normalize_chair_asset.py ver 1
"""Remove the baked support from seat_lowpoly OBJ data and normalize chair coordinates.

Source convention for this project:
- src_7e7e7e is the baked grey support block to remove.
- XY origin becomes the centroid of that removed support footprint.
- Z=0 becomes the lowest remaining chair vertex so the procedural support can sit below it.
"""
from __future__ import annotations

import argparse
from pathlib import Path

SUPPORT_MATERIAL = "src_7e7e7e"
MATERIAL_RGB = {
    "padding_red": (1.0, 0.0, 0.0),
    "src_323232": (0.196078, 0.196078, 0.196078),
    "src_696969": (0.411765, 0.411765, 0.411765),
}


def parse_obj(path: Path):
    verts = []
    normals = []
    triangles = []
    material = None
    support_vertex_ids = set()
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw.strip()
        if line.startswith("usemtl "):
            material = line.split(None, 1)[1]
        elif line.startswith("v "):
            _, x, y, z = line.split()[:4]
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("vn "):
            _, x, y, z = line.split()[:4]
            normals.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            refs = []
            for token in line.split()[1:]:
                parts = token.split("/")
                vi = int(parts[0]) - 1
                ni = int(parts[2]) - 1 if len(parts) > 2 and parts[2] else None
                refs.append((vi, ni))
                if material == SUPPORT_MATERIAL:
                    support_vertex_ids.add(vi)
            if len(refs) != 3:
                raise ValueError("Expected triangulated OBJ")
            triangles.append((material, refs))
    return verts, normals, triangles, support_vertex_ids


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("obj", type=Path)
    ap.add_argument("output", type=Path)
    args = ap.parse_args()

    verts, normals, triangles, support_ids = parse_obj(args.obj)
    if not support_ids:
        raise SystemExit(f"No {SUPPORT_MATERIAL} support geometry found")

    support_pts = [verts[i] for i in support_ids]
    sx0 = min(p[0] for p in support_pts); sx1 = max(p[0] for p in support_pts)
    sy0 = min(p[1] for p in support_pts); sy1 = max(p[1] for p in support_pts)
    support_cx = (sx0 + sx1) / 2.0
    support_cy = (sy0 + sy1) / 2.0

    kept = [(mat, refs) for mat, refs in triangles if mat != SUPPORT_MATERIAL]
    kept_ids = {vi for _mat, refs in kept for vi, _ni in refs}
    chair_z0 = min(verts[i][2] for i in kept_ids)

    # Emit unique EGG vertices per triangle. This is intentionally simple and robust.
    out = []
    out.append("<CoordinateSystem> { Z-up }")
    for name, (r, g, b) in MATERIAL_RGB.items():
        out.append(f"<Material> {name} {{ <Scalar> diffr {{ {r:.6f} }} <Scalar> diffg {{ {g:.6f} }} <Scalar> diffb {{ {b:.6f} }} <Scalar> diffa {{ 1 }} }}")
    out.append("<Group> seat_lowpoly_normalized {")
    out.append("  <VertexPool> vpool {")
    vertex_rows = []
    polys = []
    next_id = 0
    for mat, refs in kept:
        ids = []
        for vi, ni in refs:
            x, y, z = verts[vi]
            x -= support_cx
            y -= support_cy
            z -= chair_z0
            if ni is not None:
                nx, ny, nz = normals[ni]
                ntext = f" <Normal> {{ {nx:.7f} {ny:.7f} {nz:.7f} }}"
            else:
                ntext = ""
            vertex_rows.append(f"    <Vertex> {next_id} {{ {x:.6f} {y:.6f} {z:.6f}{ntext} }}")
            ids.append(next_id)
            next_id += 1
        polys.append((mat, ids))
    out.extend(vertex_rows)
    out.append("  }")
    for mat, ids in polys:
        mref = mat if mat in MATERIAL_RGB else "src_323232"
        out.append(f"  <Polygon> {{ <MRef> {{ {mref} }} <VertexRef> {{ {' '.join(map(str, ids))} <Ref> {{ vpool }} }} }}")
    out.append("}")
    args.output.write_text("\n".join(out) + "\n", encoding="utf-8")

    print(f"Removed support material: {SUPPORT_MATERIAL}")
    print(f"Support footprint: {sx1-sx0:.3f} x {sy1-sy0:.3f} mm")
    print(f"Support centroid: ({support_cx:.3f}, {support_cy:.3f}) mm")
    print(f"Remaining chair local Z minimum normalized from {chair_z0:.3f} mm to 0")
    print(f"Triangles retained: {len(kept)}")
    print(f"Output: {args.output.resolve()}")

if __name__ == "__main__":
    main()
