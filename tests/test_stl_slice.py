"""STL の輪切り: 分割された板が 1 つの外形になり、穴と円が復元されること。"""
import math
import struct

import pytest

from src.cli import main
from src.dxf_reader import read_dxf
from src.errors import Dxf2GcodeError
from src.geometry import bbox, signed_area
from src.stl_slice import as_circle, merge_loops, read_stl, slice_loops, slice_shape, write_dxf


def prism(poly, z0, z1):
    """多角形を押し出した角柱の側面の三角形(輪切りには側面だけあれば足りる)。"""
    tris = []
    n = len(poly)
    for i in range(n):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % n]
        tris.append(((x0, y0, z0), (x1, y1, z0), (x1, y1, z1)))
        tris.append(((x0, y0, z0), (x1, y1, z1), (x0, y0, z1)))
    return tris


def write_binary_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"\0" * 80 + struct.pack("<I", len(tris)))
        for t in tris:
            f.write(struct.pack("<12f", 0, 0, 0, *t[0], *t[1], *t[2]) + b"\0\0")
    return path


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def ngon(cx, cy, r, n=32):
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def test_slice_square_prism(tmp_path):
    p = write_binary_stl(tmp_path / "a.stl", prism(rect(0, 0, 40, 30), 0, 5))
    loops = slice_loops(read_stl(p), 2.5)
    assert len(loops) == 1
    assert bbox(loops[0]) == pytest.approx((0, 0, 40, 30))


def test_slicing_exactly_at_a_vertex_height_still_works(tmp_path):
    p = write_binary_stl(tmp_path / "a.stl", prism(rect(0, 0, 40, 30), 0, 5))
    assert len(slice_loops(read_stl(p), 0.0)) == 1


def test_split_quadrants_merge_into_one_outline_with_holes(tmp_path):
    # 4 分割された板(接しているだけ)+ 穴 2 つ → 外形 1 つ、穴 2 つ
    quads = [rect(0, 0, 50, 50), rect(-50, 0, 0, 50), rect(-50, -50, 0, 0), rect(0, -50, 50, 0)]
    files = [write_binary_stl(tmp_path / f"q{i}.stl", prism(q, 0, 5)) for i, q in enumerate(quads)]
    files.append(write_binary_stl(tmp_path / "h1.stl", prism(ngon(20, 20, 3), 0, 5)))
    files.append(write_binary_stl(tmp_path / "h2.stl", prism(ngon(-20, -20, 5), 0, 5)))
    shape = slice_shape(files, 2.5)
    assert len(shape.outers) == 1 and len(shape.holes) == 2
    assert bbox(shape.outers[0]) == pytest.approx((0, 0, 100, 100))       # 左下が原点にそろう
    diams = sorted(round(h.circle[2], 2) for h in shape.holes)
    assert diams == [6.0, 10.0]                                           # 円の直径が復元される


def test_scale_and_drop_and_add_holes(tmp_path):
    plate = write_binary_stl(tmp_path / "plate.stl", prism(rect(-50, -50, 50, 50), 0, 5))
    h_keep = write_binary_stl(tmp_path / "keep.stl", prism(ngon(30, 30, 3), 0, 5))
    h_drop = write_binary_stl(tmp_path / "drop.stl", prism(ngon(0, 20, 3), 0, 5))
    splice = write_binary_stl(tmp_path / "splice.stl", prism(rect(-10, 10, 10, 30), -5, 0))   # 継ぎ板
    shape = slice_shape([plate, h_keep, h_drop], 2.5, drop_holes_under=[splice],
                        add_holes=[(-30, -30, 6.5)], scale=0.5)
    assert shape.dropped == 1 and shape.added == 1 and len(shape.holes) == 2
    assert bbox(shape.outers[0]) == pytest.approx((0, 0, 50, 50))
    got = sorted((round(h.circle[0], 2), round(h.circle[1], 2), round(h.circle[2], 2)) for h in shape.holes)
    assert got == [(10.0, 10.0, 3.25), (40.0, 40.0, 3.0)]                 # 位置も直径も 1/2


def test_as_circle_rejects_squares_and_ellipses():
    assert as_circle(rect(0, 0, 10, 10)) is None
    assert as_circle([(10 * math.cos(a), 6 * math.sin(a)) for a in
                      [2 * math.pi * k / 32 for k in range(32)]]) is None
    c = as_circle(ngon(5, 7, 2.5, 16))
    assert c == pytest.approx((5, 7, 5.0))


def test_merge_even_odd_nested_loops():
    polys = merge_loops([rect(0, 0, 10, 10), rect(3, 3, 7, 7)])
    areas = sorted(signed_area(p) for p in polys)
    assert areas == pytest.approx([-16, 100])


def test_no_section_is_an_error(tmp_path):
    p = write_binary_stl(tmp_path / "a.stl", prism(rect(0, 0, 40, 30), 0, 5))
    with pytest.raises(Dxf2GcodeError):
        slice_shape([p], 50.0)


def test_dxf_roundtrip_and_cli(tmp_path):
    plate = write_binary_stl(tmp_path / "plate.stl", prism(rect(0, 0, 80, 60), 0, 5))
    hole = write_binary_stl(tmp_path / "hole.stl", prism(ngon(40, 30, 5), 0, 5))
    out = tmp_path / "p.dxf"
    assert main(["slice", str(plate), str(hole), "--z", "2.5", "--scale", "0.5", "-o", str(out)]) == 0
    res = read_dxf(out)
    assert len(res.closed) == 2
    big = max(res.closed, key=lambda c: c.area)
    assert bbox(big.points) == pytest.approx((0, 0, 40, 30))
    # そのまま変換でき、穴が先・外形が後になる
    g = tmp_path / "p.gcode"
    assert main(["convert", str(out), "-p", "mdf_2_5mm", "--align", "lower-left", "-o", str(g)]) == 0
    roles = [l.split("role=")[1].split()[0] for l in g.read_text().splitlines() if "role=" in l]
    assert roles[0] == "hole" and roles[-1] == "outer"
    shape = slice_shape([plate, hole], 2.5)
    write_dxf(tmp_path / "q.dxf", shape, layer="cut")
    assert {c.layer for c in read_dxf(tmp_path / "q.dxf").contours} == {"cut"}


def test_as_circle_with_uneven_vertex_spacing():
    # 片側だけ細かく分割された円(STL でよくある)。頂点の平均では中心がずれる
    angles = [2 * math.pi * k / 64 for k in range(33)] + [math.pi + math.pi * k / 9 for k in range(1, 9)]
    poly = [(189 + 1.7 * math.cos(a), 149 + 1.7 * math.sin(a)) for a in angles]
    assert as_circle(poly) == pytest.approx((189, 149, 3.4), abs=1e-6)
