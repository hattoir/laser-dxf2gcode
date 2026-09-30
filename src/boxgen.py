"""組み木(フィンガージョイント)の箱を生成する。上面が開いた箱(ごみ箱・小物入れ)用。

外寸 W(幅)× D(奥行き)× H(高さ)、板厚 T。板は 5 枚:
    前・後  … W × H
    左・右  … D × H
    底      … W × D

箱の角や辺では 2 枚(角では 3 枚)の板が同じ場所を取り合う。そこを
「どの板の指(タブ)が受け持つか」を次の規則で決め、受け持たない板からは
その部分を切り欠く。どの点もちょうど 1 枚の板だけが占めるので、隙間も
重なりもなく組み上がる(tests/test_boxgen.py で 3D に並べて検査している)。

    縦の辺(前/後 と 左/右): 高さ [T, H] を奇数個に分け、偶数番目 = 前後、奇数番目 = 左右
    下の辺(前/後 と 底)   : 幅 [T, W-T] を奇数個に分け、偶数番目 = 底、奇数番目 = 前後
    下の辺(左/右 と 底)   : 奥行き [T, D-T] を奇数個に分け、偶数番目 = 底、奇数番目 = 左右
    下の角(T×T×T の立方体): 前後の板

寸法はすべて「仕上がり寸法」。カーフ補正は通常の変換と同じく後段で行うので、
カーフを正しく測ってあれば設計どおりの寸法で嵌まる。板厚 T は必ずノギスで
測った実際の厚さを使うこと(表示の厚さとずれていると指の深さが合わない)。
"""
from __future__ import annotations

from dataclasses import dataclass

import pyclipper

from .errors import Dxf2GcodeError
from .geometry import Point, signed_area

SCALE = 10000.0


@dataclass
class Panel:
    name: str
    width: float    # ローカル座標 u の長さ
    height: float   # ローカル座標 v の長さ
    poly: list[Point]


def finger_count(length: float, target: float) -> int:
    """長さ length を target 前後の幅に分けるときの分割数(両端を揃えるため奇数)。"""
    n = max(3, int(round(length / target)))
    if n % 2 == 0:
        n = n + 1 if length / target > n else n - 1
    return max(3, n)


def _segments(start: float, end: float, target: float) -> list[tuple[float, float]]:
    n = finger_count(end - start, target)
    step = (end - start) / n
    return [(start + i * step, start + (i + 1) * step) for i in range(n)]


def _rect(u0: float, v0: float, u1: float, v1: float) -> list[Point]:
    return [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]


def _subtract(width: float, height: float, holes: list[list[Point]]) -> list[Point]:
    pc = pyclipper.Pyclipper()
    pc.AddPath(pyclipper.scale_to_clipper(_rect(0, 0, width, height), SCALE), pyclipper.PT_SUBJECT, True)
    if holes:
        pc.AddPaths(pyclipper.scale_to_clipper(holes, SCALE), pyclipper.PT_CLIP, True)
    res = pc.Execute(pyclipper.CT_DIFFERENCE, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO)
    # CleanPolygon(一直線上の余分な頂点を除く)は整数座標用。mm に戻す前にかけること
    # (mm の小数に直接かけると 2.5 が 2 に切り捨てられる)
    res = [pyclipper.CleanPolygon(p) for p in res]
    polys = [[(float(x), float(y)) for x, y in p] for p in pyclipper.scale_from_clipper(res, SCALE)]
    if len(polys) != 1:
        raise Dxf2GcodeError(f"板の形が 1 つの輪郭になりません({len(polys)} 個)。寸法と板厚を見直してください")
    p = [(float(x), float(y)) for x, y in polys[0]]
    return p if signed_area(p) > 0 else p[::-1]


def build_butt_panels(W: float, D: float, H: float, T: float) -> list[Panel]:
    """突き付け(組み木なし)の箱。5 枚ともただの長方形なので、手で切る場合に向く。

    底の上に壁を載せ、左右の板は前後の板の間に挟む。外寸は W × D × H のまま。
        底      W × D
        前・後  W × (H - T)
        左・右  (D - 2T) × (H - T)
    """
    if T <= 0:
        raise Dxf2GcodeError("板厚は正の値にしてください")
    if min(W, D) <= 4 * T or H <= 2 * T:
        raise Dxf2GcodeError("箱が板厚に対して小さすぎます")
    h = H - T
    return [Panel("front", W, h, _rect(0, 0, W, h)), Panel("back", W, h, _rect(0, 0, W, h)),
            Panel("left", D - 2 * T, h, _rect(0, 0, D - 2 * T, h)),
            Panel("right", D - 2 * T, h, _rect(0, 0, D - 2 * T, h)),
            Panel("bottom", W, D, _rect(0, 0, W, D))]


def build_panels(W: float, D: float, H: float, T: float, finger: float) -> list[Panel]:
    """上面が開いた箱の 5 枚の板(各板のローカル座標、左下が原点)。"""
    if T <= 0:
        raise Dxf2GcodeError("板厚は正の値にしてください")
    for name, v in (("幅", W), ("奥行き", D), ("高さ", H)):
        if v < 6 * T:
            raise Dxf2GcodeError(f"{name} {v}mm は板厚 {T}mm に対して小さすぎます(板厚の 6 倍以上)")
    z_seg = _segments(T, H, finger)         # 縦の辺
    x_seg = _segments(T, W - T, finger)     # 前後と底の辺
    y_seg = _segments(T, D - T, finger)     # 左右と底の辺
    for segs, what in ((z_seg, "縦"), (x_seg, "幅方向"), (y_seg, "奥行き方向")):
        if segs[0][1] - segs[0][0] < T:
            raise Dxf2GcodeError(f"{what}の指が板厚より細くなります。--finger を大きくしてください")

    # 前・後(u = x, v = z)
    holes = []
    for i, (z0, z1) in enumerate(z_seg):
        if i % 2 == 1:  # 奇数番目は左右の板の指
            holes += [_rect(0, z0, T, z1), _rect(W - T, z0, W, z1)]
    for i, (x0, x1) in enumerate(x_seg):
        if i % 2 == 0:  # 偶数番目は底の指
            holes.append(_rect(x0, 0, x1, T))
    front = _subtract(W, H, holes)

    # 左・右(u = y, v = z)
    holes = [_rect(0, 0, T, T), _rect(D - T, 0, D, T)]  # 下の角は前後の板
    for i, (z0, z1) in enumerate(z_seg):
        if i % 2 == 0:  # 偶数番目は前後の板の指
            holes += [_rect(0, z0, T, z1), _rect(D - T, z0, D, z1)]
    for i, (y0, y1) in enumerate(y_seg):
        if i % 2 == 0:
            holes.append(_rect(y0, 0, y1, T))
    side = _subtract(D, H, holes)

    # 底(u = x, v = y)
    holes = [_rect(0, 0, T, T), _rect(W - T, 0, W, T), _rect(0, D - T, T, D), _rect(W - T, D - T, W, D)]
    for i, (x0, x1) in enumerate(x_seg):
        if i % 2 == 1:
            holes += [_rect(x0, 0, x1, T), _rect(x0, D - T, x1, D)]
    for i, (y0, y1) in enumerate(y_seg):
        if i % 2 == 1:
            holes += [_rect(0, y0, T, y1), _rect(W - T, y0, W, y1)]
    bottom = _subtract(W, D, holes)

    return [Panel("front", W, H, front), Panel("back", W, H, list(front)),
            Panel("left", D, H, side), Panel("right", D, H, list(side)),
            Panel("bottom", W, D, bottom)]


@dataclass
class Placed:
    panel: Panel
    x: float
    y: float
    sheet: int

    def poly(self) -> list[Point]:
        return [(u + self.x, v + self.y) for u, v in self.panel.poly]


def pack(panels: list[Panel], sheet_w: float, sheet_h: float, gap: float = 4.0,
         margin: float = 5.0) -> list[Placed]:
    """棚詰め(高い板から左→右、はみ出たら次の段、段がはみ出たら次の板)。"""
    order = sorted(panels, key=lambda p: (-p.height, -p.width))
    out: list[Placed] = []
    sheet, x, y, row_h = 0, margin, margin, 0.0
    for p in order:
        if p.width + 2 * margin > sheet_w or p.height + 2 * margin > sheet_h:
            raise Dxf2GcodeError(f"板 '{p.name}'({p.width:g}×{p.height:g}mm)が材料 {sheet_w:g}×{sheet_h:g}mm に入りません")
        if x + p.width > sheet_w - margin:          # 次の段
            x, y, row_h = margin, y + row_h + gap, 0.0
        if y + p.height > sheet_h - margin:         # 次の材料
            sheet, x, y, row_h = sheet + 1, margin, margin, 0.0
        out.append(Placed(p, x, y, sheet))
        x += p.width + gap
        row_h = max(row_h, p.height)
    return out
