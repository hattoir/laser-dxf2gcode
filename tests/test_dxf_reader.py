import math

import ezdxf
import pytest

from src.dxf_reader import bulge_arc_points, read_dxf
from src.errors import DxfUnitError
from src.geometry import dist, signed_area


def _save(doc, tmp_path, name="t.dxf"):
    p = tmp_path / name
    doc.saveas(p)
    return p


def _new_doc(units=4):
    doc = ezdxf.new("R2010")
    doc.header["$INSUNITS"] = units
    return doc


def test_lines_are_chained_into_closed_loop(tmp_path):
    doc = _new_doc()
    msp = doc.modelspace()
    # 順番も向きもバラバラの 4 本 + 端点に 0.005mm の誤差
    msp.add_line((10, 0), (10, 10))
    msp.add_line((0, 0), (10.005, 0))
    msp.add_line((0, 10), (0, 0))
    msp.add_line((10, 10), (0, 10))
    res = read_dxf(_save(doc, tmp_path))
    assert len(res.closed) == 1 and not res.open
    assert abs(res.closed[0].area - 100.0) < 0.1


def test_open_path_is_warned_and_kept_separately(tmp_path):
    doc = _new_doc()
    msp = doc.modelspace()
    msp.add_line((0, 0), (10, 0))
    msp.add_line((10, 0), (10, 10))
    msp.add_line((10, 10), (0, 10.5))  # 0.5mm 開いている
    msp.add_line((0, 10), (0, 0))
    res = read_dxf(_save(doc, tmp_path))
    assert not res.closed
    assert len(res.open) == 1
    assert any("閉じていないパス" in w for w in res.warnings)


def test_layer_name_is_kept(tmp_path):
    doc = _new_doc()
    msp = doc.modelspace()
    msp.add_circle((50, 50), 5, dxfattribs={"layer": "engrave"})
    msp.add_circle((80, 50), 5, dxfattribs={"layer": "cut"})
    res = read_dxf(_save(doc, tmp_path))
    assert sorted(c.layer for c in res.contours) == ["cut", "engrave"]


def test_circle_chord_tolerance(tmp_path):
    doc = _new_doc()
    doc.modelspace().add_circle((0, 0), 10)
    res = read_dxf(_save(doc, tmp_path), chord_tol=0.05)
    pts = res.closed[0].points
    # 頂点は円周上、辺の中点は弦の許容誤差以内
    for i, p in enumerate(pts):
        q = pts[(i + 1) % len(pts)]
        assert abs(math.hypot(*p) - 10) < 1e-9
        mid = ((p[0] + q[0]) / 2, (p[1] + q[1]) / 2)
        assert 10 - math.hypot(*mid) <= 0.05 + 1e-9


def test_non_mm_units_stop(tmp_path):
    doc = _new_doc(units=1)  # inch
    doc.modelspace().add_circle((0, 0), 1)
    with pytest.raises(DxfUnitError):
        read_dxf(_save(doc, tmp_path))
    res = read_dxf(_save(doc, tmp_path), assume_mm=True)
    assert any("mm ではありません" in w for w in res.warnings)


def test_unitless_stops(tmp_path):
    doc = _new_doc(units=0)
    doc.modelspace().add_circle((0, 0), 1)
    with pytest.raises(DxfUnitError):
        read_dxf(_save(doc, tmp_path))


def test_mirrored_ocs_circle_is_converted_to_wcs(tmp_path):
    # 押し出し方向 -Z の円は OCS の X が反転している
    doc = _new_doc()
    doc.modelspace().add_circle((30, 5), 2, dxfattribs={"extrusion": (0, 0, -1)})
    res = read_dxf(_save(doc, tmp_path))
    pts = res.closed[0].points
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    assert abs(cx - (-30)) < 1e-6 and abs(cy - 5) < 1e-6


def test_lwpolyline_bulge_and_arc_chain(tmp_path):
    doc = _new_doc()
    msp = doc.modelspace()
    # 角丸なしの長穴: 直線 2 本 + 半円のふくらみ 2 つ
    msp.add_lwpolyline([(0, 0, 0), (20, 0, 1), (20, 10, 0), (0, 10, 1)], format="xyb", close=True)
    # ARC と LINE の連結
    msp.add_arc((50, 0), 5, 0, 180)
    msp.add_line((45, 0), (55, 0))
    res = read_dxf(_save(doc, tmp_path))
    assert len(res.closed) == 2
    slot = max(res.closed, key=lambda c: c.area)
    # 直線近似で面積は「周長 × 弦誤差」程度小さくなる
    assert abs(slot.area - (20 * 10 + math.pi * 25)) < (40 + math.pi * 10) * 0.05
    half = min(res.closed, key=lambda c: c.area)
    assert abs(half.area - math.pi * 25 / 2) < math.pi * 5 * 0.05


def test_bulge_direction():
    # 正のふくらみは反時計回り: (0,0)->(2,0) の半円は下側(y<0)を通る
    pts = bulge_arc_points((0, 0), (2, 0), 1.0, 0.01)
    assert min(p[1] for p in pts) < -0.99
    assert dist(pts[-1], (2, 0)) < 1e-12


def test_spline_and_ellipse(tmp_path):
    doc = _new_doc()
    msp = doc.modelspace()
    msp.add_ellipse((0, 0), major_axis=(10, 0), ratio=0.5)
    fit = [(0, 0), (10, 5), (20, 0), (10, -5), (0, 0)]
    msp.add_spline(fit)
    res = read_dxf(_save(doc, tmp_path))
    ell = [c for c in res.closed if "ELLIPSE" in c.source][0]
    assert abs(ell.area - math.pi * 10 * 5) < 50 * 0.05  # 周長 ≈ 48mm × 弦誤差
    assert any("SPLINE" in c.source for c in res.closed)


def test_duplicate_contours_removed(tmp_path):
    doc = _new_doc()
    msp = doc.modelspace()
    msp.add_circle((0, 0), 5)
    msp.add_circle((0, 0), 5)
    res = read_dxf(_save(doc, tmp_path))
    assert len(res.closed) == 1
    assert any("重複" in w for w in res.warnings)


def test_insert_block_is_exploded(tmp_path):
    doc = _new_doc()
    blk = doc.blocks.new("HOLE")
    blk.add_circle((0, 0), 2)
    msp = doc.modelspace()
    msp.add_blockref("HOLE", (100, 100), dxfattribs={"layer": "cut"})
    res = read_dxf(_save(doc, tmp_path))
    c = res.closed[0]
    assert c.layer == "cut"
    assert abs(sum(p[0] for p in c.points) / len(c.points) - 100) < 1e-6
    assert signed_area(c.points) != 0
