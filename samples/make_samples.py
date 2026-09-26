"""通しテスト用のサンプル DXF を ezdxf で作る。

    python samples/make_samples.py

bracket.dxf   … 入れ子構造を一通り含む(機械座標で配置済み)
    - 部品 A: 角丸の板 120×80(LWPOLYLINE のふくらみ)
        Ø4.2 のボルト穴 ×4(CIRCLE)、長穴(LINE + ARC を連結)、楕円穴(ELLIPSE)
        角窓(LINE ×4)の中に 島部品 Ø24(CIRCLE)とその穴 Ø6 → 階層 0/1/2/3
    - 部品 B: SPLINE の外形と、POLYLINE(2D, ふくらみ付き)の穴
    - 刻印: レイヤー "engrave" に円と線
    すべてレイヤー "0"(Fusion 360 の既定)+ "engrave"
bracket_marks.dxf … bracket.dxf に --engrave で重ねる刻印(文字)
centered.dxf  … Fusion 360 のスケッチ原点が部品中心にある想定(負の座標を含む)
    → そのままだとエリア外で停止。--align lower-left で配置する例
"""
import sys
from pathlib import Path

import ezdxf

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from src.calibrate import text_polylines  # noqa: E402  (刻印用の簡易ストロークフォント)


def rounded_rect(msp, x0, y0, w, h, r, layer="0"):
    b = 0.41421356237309503  # tan(90°/4): 90° の円弧
    pts = [
        (x0 + r, y0, 0), (x0 + w - r, y0, b),
        (x0 + w, y0 + r, 0), (x0 + w, y0 + h - r, b),
        (x0 + w - r, y0 + h, 0), (x0 + r, y0 + h, b),
        (x0, y0 + h - r, 0), (x0, y0 + r, b),
    ]
    msp.add_lwpolyline(pts, format="xyb", close=True, dxfattribs={"layer": layer})


def slot(msp, cx, cy, length, width, layer="0"):
    """横長の長穴を LINE 2 本 + ARC 2 本で描く(端点連結のテスト)。"""
    r = width / 2
    x0, x1 = cx - length / 2 + r, cx + length / 2 - r
    msp.add_line((x0, cy - r), (x1, cy - r), dxfattribs={"layer": layer})
    msp.add_arc((x1, cy), r, -90, 90, dxfattribs={"layer": layer})
    msp.add_line((x1, cy + r), (x0, cy + r), dxfattribs={"layer": layer})
    msp.add_arc((x0, cy), r, 90, 270, dxfattribs={"layer": layer})


def make_bracket(path: Path, ox=20.0, oy=20.0):
    doc = ezdxf.new("R2010")
    doc.header["$INSUNITS"] = 4  # mm
    doc.layers.add("engrave", color=5)
    msp = doc.modelspace()

    # 部品 A
    rounded_rect(msp, ox, oy, 120, 80, 6)
    for dx, dy in [(8, 8), (112, 8), (8, 72), (112, 72)]:
        msp.add_circle((ox + dx, oy + dy), 2.1)                      # Ø4.2 ボルト穴
    slot(msp, ox + 30, oy + 62, 30, 6)
    msp.add_ellipse((ox + 30, oy + 30), major_axis=(10, 0), ratio=0.5)
    wx, wy = ox + 60, oy + 15                                        # 角窓 40×40(LINE ×4)
    for a, b in [((0, 0), (40, 0)), ((40, 0), (40, 40)), ((40, 40), (0, 40)), ((0, 40), (0, 0))]:
        msp.add_line((wx + a[0], wy + a[1]), (wx + b[0], wy + b[1]))
    msp.add_circle((wx + 20, wy + 20), 12)                           # 島部品
    msp.add_circle((wx + 20, wy + 20), 3)                            # 島部品の穴

    # 部品 B: SPLINE 外形 + 2D POLYLINE の穴
    bx, by = 170.0, 20.0
    fit = [(bx, by + 30), (bx + 20, by), (bx + 60, by + 5), (bx + 70, by + 40),
           (bx + 40, by + 70), (bx + 10, by + 60), (bx, by + 30)]
    msp.add_spline(fit)
    pl = msp.add_polyline2d([(bx + 25, by + 25), (bx + 45, by + 25), (bx + 45, by + 40), (bx + 25, by + 40)],
                            close=True)
    pl.vertices[1].dxf.bulge = 0.5

    # 刻印(貫通しない)
    msp.add_circle((ox + 100, oy + 62), 5, dxfattribs={"layer": "engrave"})
    msp.add_line((ox + 94, oy + 62), (ox + 106, oy + 62), dxfattribs={"layer": "engrave"})
    msp.add_line((ox + 100, oy + 56), (ox + 100, oy + 68), dxfattribs={"layer": "engrave"})

    doc.saveas(path)


def make_marks(path: Path, ox=20.0, oy=20.0):
    """bracket.dxf に --engrave で重ねる刻印用 DXF(レイヤーは何でもよい)。"""
    doc = ezdxf.new("R2010")
    doc.header["$INSUNITS"] = 4
    msp = doc.modelspace()
    for pl in text_polylines("PART 1", ox + 12, oy + 30, 6.0):
        msp.add_lwpolyline(pl)
    for pl in text_polylines("MDF 5.0", ox + 12, oy + 18, 4.0):
        msp.add_lwpolyline(pl)
    return doc.saveas(path)


def make_centered(path: Path):
    doc = ezdxf.new("R2010")
    doc.header["$INSUNITS"] = 4
    msp = doc.modelspace()
    rounded_rect(msp, -40, -25, 80, 50, 5)
    msp.add_circle((-30, -15), 2.1)
    msp.add_circle((30, -15), 2.1)
    msp.add_circle((-30, 15), 2.1)
    msp.add_circle((30, 15), 2.1)
    msp.add_circle((0, 0), 10)
    doc.saveas(path)


if __name__ == "__main__":
    make_bracket(HERE / "bracket.dxf")
    make_marks(HERE / "bracket_marks.dxf")
    make_centered(HERE / "centered.dxf")
    print("wrote bracket.dxf / bracket_marks.dxf / centered.dxf in", HERE)
