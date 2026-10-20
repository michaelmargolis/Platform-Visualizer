# semantic_dxf.py ver 1
"""Strict lightweight ASCII-DXF reader for POC platform source drawings.

Expected semantic layers:
  MOVING_PLATFORM      one closed polyline
  MOVING_ATTACHMENTS   six circles
  FIXED_BASE           one closed polyline
  FIXED_ATTACHMENTS    six circles
  UPRIGHTS             three circles
  MUSCLES              six LINE/LWPOLYLINE/POLYLINE connections

Everything on other layers is ignored.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math

SEMANTIC_LAYERS = {
    "MOVING_PLATFORM", "MOVING_ATTACHMENTS", "FIXED_BASE",
    "FIXED_ATTACHMENTS", "UPRIGHTS", "MUSCLES",
}


@dataclass(frozen=True)
class Circle:
    center: tuple[float, float]
    radius: float


@dataclass(frozen=True)
class Polyline:
    points: tuple[tuple[float, float], ...]
    closed: bool


@dataclass(frozen=True)
class PlatformDxf:
    moving_outline: Polyline
    fixed_outline: Polyline
    moving_attachments: tuple[Circle, ...]
    fixed_attachments: tuple[Circle, ...]
    uprights: tuple[Circle, ...]
    muscles: tuple[Polyline, ...]


def _pairs(path: Path):
    lines = path.read_text(errors="ignore").splitlines()
    out = []
    for i in range(0, len(lines) - 1, 2):
        try:
            code = int(lines[i].strip())
        except ValueError:
            continue
        out.append((code, lines[i + 1].strip()))
    return out


def _values(data):
    vals = {}
    for code, value in data:
        vals.setdefault(code, []).append(value)
    return vals


def _f(vals, code, index=0, default=0.0):
    try:
        return float(vals[code][index])
    except (KeyError, IndexError, ValueError):
        return float(default)


def _i(vals, code, index=0, default=0):
    try:
        return int(float(vals[code][index]))
    except (KeyError, IndexError, ValueError):
        return int(default)


def _entity_records(path: Path):
    pairs = _pairs(path)
    start = None
    end = None
    for i in range(len(pairs) - 1):
        if pairs[i] == (0, "SECTION") and pairs[i + 1] == (2, "ENTITIES"):
            start = i + 2
            break
    if start is None:
        raise ValueError(f"DXF has no ENTITIES section: {path}")
    for i in range(start, len(pairs)):
        if pairs[i] == (0, "ENDSEC"):
            end = i
            break
    if end is None:
        raise ValueError(f"DXF ENTITIES section is not terminated: {path}")

    i = start
    while i < end:
        if pairs[i][0] != 0:
            i += 1
            continue
        etype = pairs[i][1]

        if etype == "POLYLINE":
            i += 1
            header = []
            while i < end and pairs[i][0] != 0:
                header.append(pairs[i]); i += 1
            hvals = _values(header)
            layer = hvals.get(8, ["0"])[0].upper()
            points = []
            while i < end:
                if pairs[i] == (0, "SEQEND"):
                    i += 1
                    break
                if pairs[i] != (0, "VERTEX"):
                    break
                i += 1
                data = []
                while i < end and pairs[i][0] != 0:
                    data.append(pairs[i]); i += 1
                vals = _values(data)
                points.append((_f(vals, 10), _f(vals, 20)))
            yield etype, layer, hvals, points
            continue

        i += 1
        data = []
        while i < end and pairs[i][0] != 0:
            data.append(pairs[i]); i += 1
        vals = _values(data)
        layer = vals.get(8, ["0"])[0].upper()
        yield etype, layer, vals, None


def load_platform_dxf(path):
    path = Path(path)
    buckets = {name: [] for name in SEMANTIC_LAYERS}

    for etype, layer, vals, old_points in _entity_records(path):
        if layer not in SEMANTIC_LAYERS:
            continue

        if etype == "CIRCLE":
            buckets[layer].append(Circle((_f(vals, 10), _f(vals, 20)), _f(vals, 40)))
        elif etype == "LINE":
            buckets[layer].append(Polyline(((_f(vals, 10), _f(vals, 20)), (_f(vals, 11), _f(vals, 21))), False))
        elif etype == "LWPOLYLINE":
            xs, ys = vals.get(10, []), vals.get(20, [])
            pts = tuple((float(x), float(y)) for x, y in zip(xs, ys))
            buckets[layer].append(Polyline(pts, bool(_i(vals, 70) & 1)))
        elif etype == "POLYLINE":
            buckets[layer].append(Polyline(tuple(old_points or ()), bool(_i(vals, 70) & 1)))
        else:
            raise ValueError(f"Unsupported entity {etype!r} on semantic layer {layer}")

    def exactly(layer, n, kind=None):
        items = buckets[layer]
        if len(items) != n:
            raise ValueError(f"{layer}: expected {n} entities, found {len(items)}")
        if kind is not None and not all(isinstance(x, kind) for x in items):
            raise ValueError(f"{layer}: contains an unexpected entity type")
        return tuple(items)

    moving_outline = exactly("MOVING_PLATFORM", 1, Polyline)[0]
    fixed_outline = exactly("FIXED_BASE", 1, Polyline)[0]
    if not moving_outline.closed or len(moving_outline.points) < 3:
        raise ValueError("MOVING_PLATFORM must be one closed polyline with at least 3 vertices")
    if not fixed_outline.closed or len(fixed_outline.points) < 3:
        raise ValueError("FIXED_BASE must be one closed polyline with at least 3 vertices")

    moving = exactly("MOVING_ATTACHMENTS", 6, Circle)
    fixed = exactly("FIXED_ATTACHMENTS", 6, Circle)
    uprights = exactly("UPRIGHTS", 3, Circle)
    muscles = exactly("MUSCLES", 6, Polyline)
    if any(len(m.points) < 2 for m in muscles):
        raise ValueError("Each MUSCLES entity must contain at least two points")

    return PlatformDxf(moving_outline, fixed_outline, moving, fixed, uprights, muscles)


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def resolve_muscle_pairs(model: PlatformDxf):
    """Return [(moving_index, fixed_index), ...] in DXF MUSCLES entity order."""
    mcenters = [c.center for c in model.moving_attachments]
    fcenters = [c.center for c in model.fixed_attachments]
    radii = [c.radius for c in (*model.moving_attachments, *model.fixed_attachments)]
    tolerance = max(25.0, 3.0 * min(radii))
    pairs = []

    def nearest(point, centers):
        ds = [distance(point, q) for q in centers]
        idx = min(range(len(ds)), key=ds.__getitem__)
        return idx, ds[idx]

    for n, muscle in enumerate(model.muscles):
        a, b = muscle.points[0], muscle.points[-1]
        am, af = nearest(a, mcenters), nearest(a, fcenters)
        bm, bf = nearest(b, mcenters), nearest(b, fcenters)
        option_ab = am[1] + bf[1]
        option_ba = bm[1] + af[1]
        if option_ab <= option_ba:
            mi, md, fi, fd = am[0], am[1], bf[0], bf[1]
        else:
            mi, md, fi, fd = bm[0], bm[1], af[0], af[1]
        if md > tolerance or fd > tolerance:
            raise ValueError(
                f"MUSCLES entity {n} does not terminate at one moving and one fixed attachment "
                f"(nearest distances {md:.1f}, {fd:.1f} mm; tolerance {tolerance:.1f} mm)"
            )
        pairs.append((mi, fi))

    if len({m for m, _ in pairs}) != 6 or len({f for _, f in pairs}) != 6:
        raise ValueError("MUSCLES connections are not one-to-one across all six attachment points")
    return pairs
