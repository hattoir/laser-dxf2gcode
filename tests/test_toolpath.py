import math

import pytest

from src.config import LayerSettings, Profile
from src.dxf_reader import Contour
from src.errors import GeometryError
from src.geometry import dist, point_in_polygon, signed_area
from src.toolpath import compute_nesting, offset_polygon, plan_toolpaths


def circle(cx, cy, r, n=720):
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def make_profile(kerf=0.2, **cut):
    cut_settings = dict(power=100, feed=250, passes=1)
    cut_settings.update(cut)
    return Profile(name="t", kerf=kerf, default_layer="cut", layers={
        "cut": LayerSettings(name="cut", **cut_settings),
        "engrave": LayerSettings(name="engrave", power=30, feed=3000, through_cut=False,
                                 kerf_compensation=False, order=0),
    })


def contours(*polys, layer="0"):
    return [Contour(list(p), True, layer, f"poly{i}") for i, p in enumerate(polys)]


# ---------------- 1. 内外判定 ----------------

def test_nesting_concentric_circles():
    polys = [circle(100, 100, r) for r in (40, 10, 30, 20)]  # わざと順不同
    depth, parent = compute_nesting(polys)
    assert depth == [0, 3, 1, 2]
    assert parent == [None, 3, 0, 2]


def test_nesting_side_by_side_parts():
    polys = [rect(0, 0, 50, 50), rect(60, 0, 110, 50), circle(25, 25, 5), circle(85, 25, 5)]
    depth, parent = compute_nesting(polys)
    assert depth == [0, 0, 1, 1]
    assert parent[2] == 0 and parent[3] == 1


def test_nesting_hole_touching_outline_vertex():
    # 穴の頂点が外形の辺上に乗っていても判定できる(境界上の点は多数決から除外)
    outer = rect(0, 0, 40, 40)
    hole = [(0, 20), (10, 10), (20, 20), (10, 30)]
    depth, _ = compute_nesting([outer, hole])
    assert depth == [0, 1]


# ---------------- 2. カーフ補正 ----------------

def _radii(poly, cx, cy):
    return [math.hypot(x - cx, y - cy) for x, y in poly]


def test_kerf_hole_r10_becomes_r10_1():
    plan = plan_toolpaths(contours(rect(50, 50, 150, 150), circle(100, 100, 10)), make_profile(kerf=0.2))
    hole = [p for p in plan.paths if p.role == "hole"][0]
    r = _radii(hole.points[:-1], 100, 100)
    assert abs(max(r) - 10.1) < 0.005
    assert abs(sum(r) / len(r) - 10.1) < 0.005


def test_kerf_outer_shrinks_by_half_kerf():
    plan = plan_toolpaths(contours(circle(100, 100, 10)), make_profile(kerf=0.2))
    outer = plan.paths[0]
    assert outer.role == "outer"
    r = _radii(outer.points[:-1], 100, 100)
    assert abs(sum(r) / len(r) - 9.9) < 0.005


def test_kerf_square_outer_keeps_sharp_corners():
    plan = plan_toolpaths(contours(rect(10, 10, 30, 30)), make_profile(kerf=0.2))
    xs = [p[0] for p in plan.paths[0].points]
    ys = [p[1] for p in plan.paths[0].points]
    assert abs(min(xs) - 10.1) < 1e-3 and abs(max(xs) - 29.9) < 1e-3
    assert abs(min(ys) - 10.1) < 1e-3 and abs(max(ys) - 29.9) < 1e-3


def test_kerf_zero_leaves_geometry():
    plan = plan_toolpaths(contours(circle(100, 100, 10)), make_profile(kerf=0.0))
    r = _radii(plan.paths[0].points[:-1], 100, 100)
    assert max(abs(x - 10) for x in r) < 1e-4


def test_orientation_scrap_on_right():
    plan = plan_toolpaths(contours(rect(0, 0, 50, 50), circle(25, 25, 5)), make_profile())
    for p in plan.paths:
        area = signed_area(p.points[:-1])
        assert (area > 0) if p.role == "outer" else (area < 0)


def test_feature_smaller_than_kerf_is_error():
    with pytest.raises(GeometryError):
        plan_toolpaths(contours(circle(100, 100, 0.05)), make_profile(kerf=0.2))


def test_engrave_layer_not_compensated():
    cs = [Contour(circle(100, 100, 10), True, "engrave", "e")]
    plan = plan_toolpaths(cs, make_profile(kerf=0.2))
    r = _radii(plan.paths[0].points[:-1], 100, 100)
    assert max(abs(x - 10) for x in r) < 1e-9


# ---------------- 3. 切断順序(安全に関わる) ----------------

def _index(plan, pred):
    return [i for i, p in enumerate(plan.paths) if pred(p)]


def test_holes_always_before_their_outline():
    # 部品 3 つ、それぞれ穴 2 つ。開始位置を変えても穴が先
    polys = []
    for k in range(3):
        x = 20 + k * 70
        polys += [rect(x, 20, x + 60, 80), circle(x + 15, 50, 5), circle(x + 45, 50, 5)]
    for start in [(0, 0), (400, 415), (230, 50)]:
        plan = plan_toolpaths(contours(*polys), make_profile(), start=start)
        assert plan.parts == 3 and plan.holes == 6
        for p_idx, p in enumerate(plan.paths):
            if p.role != "outer":
                continue
            hole_idx = _index(plan, lambda q: q.role == "hole" and q.part_id == p.part_id)
            assert len(hole_idx) == 2
            assert all(h < p_idx for h in hole_idx), f"穴が外形より後: start={start}"


def test_deepest_first_with_island_part():
    # 板(0) の窓(1) の中に小部品(2) とその穴(3)
    polys = [rect(0, 0, 100, 100), rect(20, 20, 80, 80), circle(50, 50, 20), circle(50, 50, 5)]
    plan = plan_toolpaths(contours(*polys), make_profile())
    assert [p.depth for p in plan.paths] == [3, 2, 1, 0]


def test_all_holes_of_all_parts_before_any_outline():
    polys = [rect(0, 0, 50, 50), circle(25, 25, 5), rect(100, 0, 150, 50), circle(125, 25, 5)]
    plan = plan_toolpaths(contours(*polys), make_profile())
    roles = [p.role for p in plan.paths]
    assert roles == ["hole", "hole", "outer", "outer"]


def test_engrave_and_open_paths_come_before_through_cuts():
    cs = contours(rect(0, 0, 50, 50), circle(25, 25, 5))
    cs.append(Contour([(5, 5), (45, 5)], False, "0", "slit"))
    cs.append(Contour(circle(10, 40, 3), True, "engrave", "mark"))
    plan = plan_toolpaths(cs, make_profile())
    assert [p.role for p in plan.paths] == ["mark", "open", "hole", "outer"]


def test_greedy_nearest_neighbour():
    polys = [circle(x, 50, 3) for x in (300, 20, 200, 100)]
    plan = plan_toolpaths(contours(*polys), make_profile(), start=(0, 0))
    centers = [round(sum(p[0] for p in c.points[:-1]) / (len(c.points) - 1)) for c in plan.paths]
    assert centers == [20, 100, 200, 300]


def test_closed_path_returns_to_start():
    plan = plan_toolpaths(contours(rect(10, 10, 30, 30)), make_profile())
    p = plan.paths[0]
    assert dist(p.points[0], p.points[-1]) < 1e-9


# ---------------- 5. リードイン ----------------

def test_lead_in_on_scrap_side():
    outer = rect(50, 50, 100, 100)
    plan = plan_toolpaths(contours(outer, circle(75, 75, 5)), make_profile(), lead_in=2.0, start=(0, 0))
    o = [p for p in plan.paths if p.role == "outer"][0]
    h = [p for p in plan.paths if p.role == "hole"][0]
    assert o.lead_in and not h.lead_in
    lp = o.points[0]
    assert not point_in_polygon(lp, outer)
    assert abs(dist(lp, o.points[1]) - 2.0) < 1e-6
    assert dist(o.points[1], o.points[-1]) < 1e-9  # 輪郭は進入点に戻って閉じる


def test_lead_in_skipped_when_no_room():
    # 2 つの部品が 0.5mm しか離れていないと、2mm のリードインは隣の部品に食い込む
    polys = [rect(10, 10, 30, 30), rect(30.5, 10, 50, 30), rect(10, 30.5, 50, 50)]
    plan = plan_toolpaths(contours(*polys), make_profile(kerf=0.0), lead_in=2.0, start=(30.25, 30.25))
    for p in plan.paths:
        if p.lead_in:
            lp = p.points[0]
            assert all(not point_in_polygon(lp, q) for q in polys)
