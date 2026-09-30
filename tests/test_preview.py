import math
import xml.etree.ElementTree as ET

from src.config import LayerSettings, Machine, Profile
from src.dxf_reader import Contour
from src.gcode import generate_gcode, verify_gcode
from src.preview import build_report, chain_time, estimate_time, extract_burns, render_svg
from src.toolpath import plan_toolpaths


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def circle(cx, cy, r, n=90):
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def profile(passes=2):
    return Profile("t", 0.2, "cut", {"cut": LayerSettings("cut", power=100, feed=600, passes=passes)})


def job(passes=2):
    m = Machine()
    cs = [Contour(rect(50, 50, 100, 100), True, "0", "o"), Contour(circle(75, 75, 5), True, "0", "h")]
    plan = plan_toolpaths(cs, profile(passes))
    g = generate_gcode(plan, m)
    return m, plan, g, verify_gcode(g.text, m)


def test_straight_line_time():
    m = Machine()
    # 100mm を 600mm/min(10mm/s)、加速度 1000mm/s² → 10s + v/a = 10.01s
    t = chain_time([((0, 0), (100, 0), 600)], m)
    assert abs(t - 10.01) < 1e-6


def test_corners_slow_down():
    m = Machine()
    straight = chain_time([((0, 0), (50, 0), 6000), ((50, 0), (100, 0), 6000)], m)
    corner = chain_time([((0, 0), (50, 0), 6000), ((50, 0), (50, 50), 6000)], m)
    assert corner > straight


def test_y_axis_rate_limit():
    m = Machine()  # Y の最大 8000mm/min
    tx = chain_time([((0, 0), (300, 0), 20000)], m)
    ty = chain_time([((0, 0), (0, 300), 20000)], m)
    assert ty > tx


def test_estimate_includes_passes():
    m, _, _, p1 = job(passes=1)
    _, _, _, p3 = job(passes=3)
    t1, t3 = estimate_time(p1, m), estimate_time(p3, m)
    assert 2.8 < t3.cut_s / t1.cut_s < 3.2


def test_burns_and_report_numbers():
    m, plan, g, parsed = job(passes=2)
    burns = extract_burns(parsed, g.text)
    assert len(burns) == 4 and [b.role for b in burns] == ["hole", "hole", "outer", "outer"]
    rep = build_report(g.text, parsed, m, plan, {"source": "x"}, [])
    assert "部品数(外形)  : 1" in rep.text and "穴の数        : 1" in rep.text
    # 外形 50 角は外へ 0.1(角は R0.1 の円弧)、穴 R5 は内へ 0.1 → R4.9
    expected = 2 * (4 * 50 + 2 * math.pi * 0.1 + 2 * math.pi * 4.9)
    assert abs(rep.cut_length - expected) / expected < 0.01
    assert 49.85 < rep.bbox[0] < 49.95 and 100.05 < rep.bbox[2] < 100.15


def test_detail_window_svg():
    m, plan, g, parsed = job(passes=1)
    svg = render_svg(g.text, parsed, m, plan, "t", window=(40, 40, 110, 110))
    root = ET.fromstring(svg)
    vb = [float(v) for v in root.get("viewBox").split()]
    assert vb[2] < 120  # 使用範囲 + 余白だけを表示


def test_svg_well_formed_and_contains_elements():
    m, plan, g, parsed = job(passes=2)
    svg = render_svg(g.text, parsed, m, plan, "t", ["info"], ["warn"])
    root = ET.fromstring(svg)
    ns = "{http://www.w3.org/2000/svg}"
    texts = [t.text for t in root.iter(ns + "text")]
    assert "1×2" in texts and "2×2" in texts               # 番号とパス回数
    assert any("400 × 415" in (t or "") for t in texts)   # 加工エリアの枠
    dashed = [e for e in root.iter(ns + "g") if e.get("stroke-dasharray")]
    assert dashed and len(list(dashed[0])) >= 2            # 早送りの破線
