"""2D 幾何の小さなヘルパー(外部依存なし)。

点は (x, y) のタプル、ポリゴンは点のリスト。閉じたポリゴンは
始点を末尾に重複させない(暗黙に閉じている)表現で扱う。
"""
from __future__ import annotations

import math
from typing import Iterable, Sequence

Point = tuple[float, float]


def dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def signed_area(poly: Sequence[Point]) -> float:
    """符号付き面積。反時計回り (CCW) で正。"""
    s = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return s / 2.0


def polyline_length(pts: Sequence[Point], closed: bool = False) -> float:
    total = sum(dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
    if closed and len(pts) > 1:
        total += dist(pts[-1], pts[0])
    return total


def bbox(points: Iterable[Point]) -> tuple[float, float, float, float]:
    xs, ys = [], []
    for x, y in points:
        xs.append(x)
        ys.append(y)
    return min(xs), min(ys), max(xs), max(ys)


def point_in_polygon(pt: Point, poly: Sequence[Point]) -> bool:
    """偶奇規則による内外判定(境界上の扱いは不定)。"""
    x, y = pt
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            x_cross = xi + (y - yi) * (xj - xi) / (yj - yi)
            if x < x_cross:
                inside = not inside
        j = i
    return inside


def point_segment_distance(p: Point, a: Point, b: Point) -> float:
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0.0:
        return dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def distance_to_polygon(p: Point, poly: Sequence[Point]) -> float:
    n = len(poly)
    return min(point_segment_distance(p, poly[i], poly[(i + 1) % n]) for i in range(n))


def arc_segments(radius: float, sweep_rad: float, chord_tol: float) -> int:
    """弦の許容誤差(サジッタ)chord_tol 以下で円弧を近似する分割数。"""
    sweep = abs(sweep_rad)
    if radius <= chord_tol:
        return max(1, math.ceil(sweep / (math.pi / 2)))
    # サジッタ s = r (1 - cos(θ/2)) <= tol  →  θ <= 2 acos(1 - tol/r)
    max_step = 2.0 * math.acos(1.0 - chord_tol / radius)
    return max(1, math.ceil(sweep / max_step))


def remove_duplicate_points(pts: list[Point], tol: float = 1e-9) -> list[Point]:
    out: list[Point] = []
    for p in pts:
        if not out or dist(out[-1], p) > tol:
            out.append(p)
    return out
