"""STL を水平に輪切りにして、2D の外形と穴を DXF にする。

3D プリント用の STL しか無い部品(板状のもの)を、レーザーで切るための入口。
複数の STL を渡すと、同じ高さで輪切りにしたものを 1 つの形に合体させる
(3D プリント用に分割された板を、1 枚の板として扱える)。

    1. 各三角形と平面 z = 一定 の交線(線分)を求める
    2. 線分を端点でつないで閉ループにする
    3. 全ループを偶奇規則で合成する(外形の中のループ = 穴、接している外形どうしは 1 つになる)
    4. 円に見える穴は CIRCLE として書き出す(STL は円を多角形にしているので、直径を復元する)
"""
from __future__ import annotations

import math
import struct
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import ezdxf
import pyclipper

from .errors import Dxf2GcodeError
from .geometry import Point, bbox, signed_area

SCALE = 1000.0      # pyclipper 用(1 µm 単位)
JOIN_DIGITS = 3     # 端点を同じ点とみなす丸め(0.001 mm)

Triangle = tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]


def read_stl(path: str | Path) -> list[Triangle]:
    """バイナリ / ASCII の STL を読む。"""
    data = Path(path).read_bytes()
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if len(data) == 84 + n * 50:
            out = []
            for i in range(n):
                v = struct.unpack("<12f", data[84 + i * 50: 84 + i * 50 + 48])
                out.append((v[3:6], v[6:9], v[9:12]))
            return out
    verts = []
    for line in data.decode("utf-8", "ignore").splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == "vertex":
            verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
    if not verts or len(verts) % 3:
        raise Dxf2GcodeError(f"STL を読めません: {path}")
    return [(verts[i], verts[i + 1], verts[i + 2]) for i in range(0, len(verts), 3)]


def footprint(path: str | Path) -> tuple[float, float, float, float]:
    """STL を真上から見た外接矩形 (x0, y0, x1, y1)。"""
    tris = read_stl(path)
    return bbox((v[0], v[1]) for t in tris for v in t)


def slice_loops(tris: list[Triangle], z: float) -> list[list[Point]]:
    """平面 z で切った断面の閉ループ(向きは不定)。"""
    # 頂点がちょうど平面上にあると交点が定まらないので、わずかにずらす
    while any(abs(v[2] - z) < 1e-7 for t in tris for v in t):
        z += 1e-4
    segs: list[tuple[Point, Point]] = []
    for t in tris:
        pts = []
        for i in range(3):
            a, b = t[i], t[(i + 1) % 3]
            if (a[2] - z) * (b[2] - z) < 0:
                s = (z - a[2]) / (b[2] - a[2])
                pts.append((a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1])))
        if len(pts) == 2 and math.dist(pts[0], pts[1]) > 1e-9:
            segs.append((pts[0], pts[1]))

    def key(p: Point) -> tuple[float, float]:
        return (round(p[0], JOIN_DIGITS), round(p[1], JOIN_DIGITS))

    ends: dict[tuple[float, float], list[tuple[int, int]]] = defaultdict(list)
    for i, (a, b) in enumerate(segs):
        ends[key(a)].append((i, 0))
        ends[key(b)].append((i, 1))
    used = [False] * len(segs)
    loops: list[list[Point]] = []
    for i in range(len(segs)):
        if used[i]:
            continue
        used[i] = True
        loop = [segs[i][0], segs[i][1]]
        while key(loop[-1]) != key(loop[0]):
            nxt = [(j, e) for j, e in ends[key(loop[-1])] if not used[j]]
            if not nxt:
                break
            j, e = nxt[0]
            used[j] = True
            loop.append(segs[j][1 - e])
        if len(loop) > 3 and key(loop[-1]) == key(loop[0]):
            loops.append(loop[:-1])
    return loops


def merge_loops(loops: list[list[Point]]) -> list[list[Point]]:
    """偶奇規則で合成する。結果は外形が反時計回り、穴が時計回り。"""
    if not loops:
        return []
    pc = pyclipper.Pyclipper()
    for lp in loops:
        pc.AddPath(pyclipper.scale_to_clipper(lp, SCALE), pyclipper.PT_SUBJECT, True)
    res = pc.Execute(pyclipper.CT_UNION, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD)
    res = [pyclipper.CleanPolygon(p, 2) for p in res]     # 整数座標のうちに、一直線上の余分な点を除く
    return [[(x / SCALE, y / SCALE) for x, y in p] for p in res if len(p) >= 3]


def as_circle(poly: list[Point], tol: float = 0.02) -> tuple[float, float, float] | None:
    """多角形が円なら (中心 x, 中心 y, 直径)。STL の円は頂点が円周上に乗っている。

    頂点の並びが不均等な円(片側だけ細かく分割されている)でも中心がずれないよう、
    頂点の平均ではなく最小二乗の円当てはめ(x² + y² + Ax + By + C = 0)で中心を求める。
    """
    n = len(poly)
    if n < 8:
        return None
    # 桁落ちを避けるため、外接矩形の中心を原点に移してから解く
    x0, y0, x1, y1 = bbox(poly)
    ox, oy = (x0 + x1) / 2, (y0 + y1) / 2
    sxx = sxy = syy = sx = sy = sxz = syz = sz = 0.0
    for px, py in poly:
        x, y = px - ox, py - oy
        z = x * x + y * y
        sxx += x * x; sxy += x * y; syy += y * y
        sx += x; sy += y; sxz += x * z; syz += y * z; sz += z
    # 正規方程式 [[sxx,sxy,sx],[sxy,syy,sy],[sx,sy,n]] · [A,B,C] = -[sxz,syz,sz]
    m = [[sxx, sxy, sx, -sxz], [sxy, syy, sy, -syz], [sx, sy, float(n), -sz]]
    for i in range(3):
        piv = max(range(i, 3), key=lambda r: abs(m[r][i]))
        if abs(m[piv][i]) < 1e-12:
            return None
        m[i], m[piv] = m[piv], m[i]
        for r in range(3):
            if r != i:
                f = m[r][i] / m[i][i]
                m[r] = [a - f * b for a, b in zip(m[r], m[i])]
    A, B, C = (m[i][3] / m[i][i] for i in range(3))
    cx, cy = -A / 2, -B / 2
    r2 = cx * cx + cy * cy - C
    if r2 <= 0:
        return None
    r = math.sqrt(r2)
    if max(abs(math.hypot(px - ox - cx, py - oy - cy) - r) for px, py in poly) > max(tol, 0.01 * r):
        return None
    return cx + ox, cy + oy, 2 * r


@dataclass
class Hole:
    poly: list[Point]
    circle: tuple[float, float, float] | None   # (cx, cy, 直径)

    @property
    def center(self) -> Point:
        if self.circle:
            return self.circle[0], self.circle[1]
        x0, y0, x1, y1 = bbox(self.poly)
        return (x0 + x1) / 2, (y0 + y1) / 2


@dataclass
class Shape:
    outers: list[list[Point]] = field(default_factory=list)
    holes: list[Hole] = field(default_factory=list)
    dropped: int = 0      # 取り除いた穴の数
    added: int = 0        # 追加した穴の数

    def bounds(self) -> tuple[float, float, float, float]:
        return bbox(p for o in self.outers for p in o)


def slice_shape(stl_paths: list[str | Path], z: float, *, drop_holes_under: list[str | Path] = (),
                add_holes: list[tuple[float, float, float]] = (), scale: float = 1.0) -> Shape:
    """STL(複数可)を z で輪切りにして 1 つの形にする。

    drop_holes_under: この STL の真下/真上(外接矩形の中)に中心がある穴を取り除く
                      (分割された板をつなぐ「継ぎ板」用の穴を消すのに使う)
    add_holes: 追加する円い穴 (x, y, 直径)。座標は元の STL の座標系・縮小前の寸法
    scale: 全体の倍率(穴の直径も同じ倍率で縮む)
    """
    if scale <= 0:
        raise Dxf2GcodeError("scale は正の値にしてください")
    loops: list[list[Point]] = []
    for p in stl_paths:
        loops += slice_loops(read_stl(p), z)
    if not loops:
        raise Dxf2GcodeError(f"高さ z={z:g} の断面に形がありません(STL の高さの範囲を確認してください)")
    polys = merge_loops(loops)
    shape = Shape()
    boxes = [footprint(p) for p in drop_holes_under]
    for poly in polys:
        if signed_area(poly) > 0:
            shape.outers.append(poly)
            continue
        h = Hole(poly[::-1], as_circle(poly))
        cx, cy = h.center
        if any(b[0] - 0.5 <= cx <= b[2] + 0.5 and b[1] - 0.5 <= cy <= b[3] + 0.5 for b in boxes):
            shape.dropped += 1
            continue
        shape.holes.append(h)
    for x, y, d in add_holes:
        n = 48
        poly = [(x + d / 2 * math.cos(2 * math.pi * k / n), y + d / 2 * math.sin(2 * math.pi * k / n))
                for k in range(n)]
        shape.holes.append(Hole(poly, (x, y, d)))
        shape.added += 1

    # 左下を原点にそろえてから縮小する
    x0, y0, _, _ = shape.bounds()

    def tf(p: Point) -> Point:
        return ((p[0] - x0) * scale, (p[1] - y0) * scale)

    shape.outers = [[tf(p) for p in o] for o in shape.outers]
    shape.holes = [Hole([tf(p) for p in h.poly],
                        (*tf((h.circle[0], h.circle[1])), h.circle[2] * scale) if h.circle else None)
                   for h in shape.holes]
    return shape


def write_dxf(path: str | Path, shape: Shape, layer: str = "0") -> None:
    doc = ezdxf.new("R2010")
    doc.header["$INSUNITS"] = 4
    if layer != "0":
        doc.layers.add(layer)
    msp = doc.modelspace()
    for o in shape.outers:
        msp.add_lwpolyline(o, close=True, dxfattribs={"layer": layer})
    for h in shape.holes:
        if h.circle:
            msp.add_circle((h.circle[0], h.circle[1]), h.circle[2] / 2, dxfattribs={"layer": layer})
        else:
            msp.add_lwpolyline(h.poly, close=True, dxfattribs={"layer": layer})
    doc.saveas(path)


def describe(shape: Shape) -> list[str]:
    x0, y0, x1, y1 = shape.bounds()
    lines = [f"外形 {len(shape.outers)} 個 / 大きさ {x1 - x0:.1f} × {y1 - y0:.1f} mm / 穴 {len(shape.holes)} 個"
             + (f"(取り除いた穴 {shape.dropped} 個)" if shape.dropped else "")
             + (f"(追加した穴 {shape.added} 個)" if shape.added else "")]
    groups: dict[str, int] = defaultdict(int)
    for h in shape.holes:
        if h.circle:
            groups[f"円 Ø{h.circle[2]:.2f}"] += 1
        else:
            bx = bbox(h.poly)
            groups[f"角 {bx[2] - bx[0]:.1f}×{bx[3] - bx[1]:.1f}"] += 1
    for tag, n in sorted(groups.items(), key=lambda kv: -kv[1]):
        lines.append(f"  穴 {tag} × {n}")
    return lines
