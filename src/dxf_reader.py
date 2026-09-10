"""DXF 読み込み: エンティティを直線近似し、端点を連結して閉ループにする。

処理の流れ
    1. $INSUNITS を確認(mm 以外 / 不明なら停止)
    2. LINE / LWPOLYLINE / POLYLINE / CIRCLE / ARC / ELLIPSE / SPLINE を
       ポリライン(点列)に変換。曲線は弦の許容誤差 chord_tol で直線近似
       (INSERT はブロックを展開して同様に処理)
    3. 同じレイヤー内で、端点が join_tol 以内で繋がるポリラインを連結
    4. 始点と終点が繋がったものを閉ループ、繋がらないものを開いたパスとする

座標はすべて WCS の XY。CIRCLE/ARC/LWPOLYLINE は OCS を持つので、押し出し
方向が -Z のもの(Fusion 360 で裏返しの面に描いたスケッチで起きる)も
正しく WCS に変換する。ここを怠ると部品が左右反転する。
"""
from __future__ import annotations

import math
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import ezdxf
from ezdxf.math import Vec3

from .errors import DxfUnitError, Dxf2GcodeError
from .geometry import Point, arc_segments, bbox, dist, distance_to_polygon, remove_duplicate_points, signed_area

DEFAULT_CHORD_TOL = 0.05  # mm: 曲線を直線近似するときの弦の許容誤差(サジッタ)
DEFAULT_JOIN_TOL = 0.01   # mm: 端点を「繋がっている」とみなす距離

INSUNITS_NAMES = {
    0: "単位なし", 1: "inch", 2: "feet", 3: "mile", 4: "mm", 5: "cm", 6: "m",
    7: "km", 8: "microinch", 9: "mil", 10: "yard", 13: "micron", 14: "dm",
}
INSUNITS_MM = 4

SUPPORTED = {"LINE", "LWPOLYLINE", "POLYLINE", "CIRCLE", "ARC", "ELLIPSE", "SPLINE", "INSERT"}


@dataclass
class Contour:
    """読み込んだ輪郭 1 本。closed=True のとき points は始点を末尾に重複させない。"""
    points: list[Point]
    closed: bool
    layer: str
    source: str = ""

    @property
    def area(self) -> float:
        return abs(signed_area(self.points)) if self.closed else 0.0


@dataclass
class ReadResult:
    contours: list[Contour]
    warnings: list[str] = field(default_factory=list)
    insunits: int = INSUNITS_MM
    entity_counts: dict[str, int] = field(default_factory=dict)

    @property
    def closed(self) -> list[Contour]:
        return [c for c in self.contours if c.closed]

    @property
    def open(self) -> list[Contour]:
        return [c for c in self.contours if not c.closed]


@dataclass
class _Raw:
    points: list[Point]
    closed: bool
    layer: str
    source: str


# --------------------------------------------------------------------------
# 公開 API
# --------------------------------------------------------------------------

def read_dxf(path: str | Path, chord_tol: float = DEFAULT_CHORD_TOL,
             join_tol: float = DEFAULT_JOIN_TOL, assume_mm: bool = False) -> ReadResult:
    """DXF を読み込んで輪郭のリストを返す。

    assume_mm=True なら $INSUNITS が mm でなくても警告だけで続行する
    (Fusion 360 の書き出し設定で単位が入らない場合の逃げ道。数値は変換しない)。
    """
    if chord_tol <= 0 or join_tol <= 0:
        raise Dxf2GcodeError("chord_tol と join_tol は正の値にしてください")
    try:
        doc = ezdxf.readfile(str(path))
    except (IOError, ezdxf.DXFStructureError) as exc:
        raise Dxf2GcodeError(f"DXF を読み込めません: {path}: {exc}") from exc

    warnings: list[str] = []
    insunits = int(doc.header.get("$INSUNITS", 0))
    check_units(insunits, assume_mm, warnings)

    raws, counts = extract_entities(doc.modelspace(), chord_tol, warnings)
    contours = chain_paths(raws, join_tol, warnings)
    contours = remove_duplicate_contours(contours, max(join_tol, chord_tol), warnings)
    for c in contours:
        if not c.closed:
            warnings.append(
                f"閉じていないパス: レイヤー '{c.layer}' {c.source} "
                f"始点({c.points[0][0]:.3f}, {c.points[0][1]:.3f}) 終点({c.points[-1][0]:.3f}, {c.points[-1][1]:.3f})"
                " → カーフ補正なしで、閉じた輪郭より先に切ります")
    return ReadResult(contours=contours, warnings=warnings, insunits=insunits, entity_counts=dict(counts))


def check_units(insunits: int, assume_mm: bool, warnings: list[str]) -> None:
    if insunits == INSUNITS_MM:
        return
    name = INSUNITS_NAMES.get(insunits, f"コード {insunits}")
    msg = f"DXF の単位 ($INSUNITS) が mm ではありません: {name}"
    if assume_mm:
        warnings.append(msg + "(--assume-mm 指定のため mm とみなして続行。数値は変換しません)")
        return
    raise DxfUnitError(
        msg + "。Fusion 360 側で mm にして書き出し直すか、数値が mm であることを確認した上で "
        "--assume-mm を付けてください")


# --------------------------------------------------------------------------
# エンティティ → 点列
# --------------------------------------------------------------------------

def extract_entities(entities, chord_tol: float, warnings: list[str],
                     layer_override: str | None = None, depth: int = 0) -> tuple[list[_Raw], Counter]:
    raws: list[_Raw] = []
    counts: Counter = Counter()
    skipped: Counter = Counter()
    z_warned = False
    for e in entities:
        t = e.dxftype()
        layer = e.dxf.get("layer", "0")
        if layer_override is not None and layer == "0":
            layer = layer_override  # ブロック内のレイヤー 0 は INSERT のレイヤーを継承
        if t not in SUPPORTED:
            skipped[t] += 1
            continue
        counts[t] += 1
        if t == "INSERT":
            if depth > 16:
                warnings.append("INSERT の入れ子が深すぎるため打ち切りました")
                continue
            sub, sub_counts = extract_entities(e.virtual_entities(), chord_tol, warnings, layer, depth + 1)
            raws.extend(sub)
            counts.update(sub_counts)
            continue
        try:
            pts3, closed = _entity_points(e, chord_tol)
        except Exception as exc:  # 壊れたエンティティは警告して飛ばす
            warnings.append(f"{t}(handle={e.dxf.get('handle', '?')}) を変換できません: {exc}")
            continue
        if not z_warned and any(abs(p.z) > 1e-6 for p in pts3):
            warnings.append("Z 座標が 0 でない図形があります。Z は無視して XY に投影します")
            z_warned = True
        pts = remove_duplicate_points([(p.x, p.y) for p in pts3])
        if closed and len(pts) > 1 and dist(pts[0], pts[-1]) < 1e-9:
            pts.pop()
        if len(pts) < 2:
            continue  # 長さ 0 の線
        raws.append(_Raw(pts, closed and len(pts) >= 3, layer, f"{t}(handle={e.dxf.get('handle', '?')})"))
    for t, n in skipped.items():
        warnings.append(f"未対応のエンティティ {t} を {n} 個無視しました")
    return raws, counts


def _entity_points(e, tol: float) -> tuple[list[Vec3], bool]:
    t = e.dxftype()
    if t == "LINE":
        return [Vec3(e.dxf.start), Vec3(e.dxf.end)], False
    if t == "CIRCLE":
        r = e.dxf.radius
        n = max(8, arc_segments(r, 2 * math.pi, tol))
        c = Vec3(e.dxf.center)
        ocs = e.ocs()
        pts = [ocs.to_wcs(Vec3(c.x + r * math.cos(2 * math.pi * k / n), c.y + r * math.sin(2 * math.pi * k / n), c.z))
               for k in range(n)]
        return pts, True
    if t == "ARC":
        r = e.dxf.radius
        a0 = math.radians(e.dxf.start_angle)
        sweep = math.radians((e.dxf.end_angle - e.dxf.start_angle) % 360.0) or 2 * math.pi
        n = arc_segments(r, sweep, tol)
        c = Vec3(e.dxf.center)
        ocs = e.ocs()
        pts = [ocs.to_wcs(Vec3(c.x + r * math.cos(a0 + sweep * k / n), c.y + r * math.sin(a0 + sweep * k / n), c.z))
               for k in range(n + 1)]
        return pts, False
    if t == "LWPOLYLINE":
        ocs = e.ocs()
        elev = e.dxf.get("elevation", 0.0)
        verts = [(x, y, b) for x, y, b in e.get_points("xyb")]
        pts2 = _bulge_polyline(verts, bool(e.closed), tol)
        return [ocs.to_wcs(Vec3(x, y, elev)) for x, y in pts2], bool(e.closed)
    if t == "POLYLINE":
        if e.is_2d_polyline:
            ocs = e.ocs()
            elev = Vec3(e.dxf.get("elevation", (0, 0, 0))).z
            verts = [(v.dxf.location.x, v.dxf.location.y, v.dxf.get("bulge", 0.0)) for v in e.vertices]
            pts2 = _bulge_polyline(verts, e.is_closed, tol)
            return [ocs.to_wcs(Vec3(x, y, elev)) for x, y in pts2], e.is_closed
        if e.is_3d_polyline:
            return [Vec3(v.dxf.location) for v in e.vertices], e.is_closed
        raise ValueError("ポリフェース/メッシュは未対応")
    if t in ("ELLIPSE", "SPLINE"):
        pts = [Vec3(p) for p in e.flattening(tol)]
        closed = len(pts) > 2 and pts[0].isclose(pts[-1], abs_tol=1e-9)
        if t == "SPLINE" and e.closed:
            closed = True
        if closed and pts[0].isclose(pts[-1], abs_tol=1e-9):
            pts.pop()
        return pts, closed
    raise ValueError(f"未対応: {t}")


def _bulge_polyline(verts: list[tuple[float, float, float]], closed: bool, tol: float) -> list[Point]:
    """(x, y, bulge) の列を、ふくらみ(円弧)を直線近似した点列にする。"""
    if not verts:
        return []
    out: list[Point] = [(verts[0][0], verts[0][1])]
    n = len(verts)
    last = n if closed else n - 1
    for i in range(last):
        x1, y1, b = verts[i]
        x2, y2, _ = verts[(i + 1) % n]
        if abs(b) > 1e-12:
            out.extend(bulge_arc_points((x1, y1), (x2, y2), b, tol))
        else:
            out.append((x2, y2))
    if closed and len(out) > 1 and dist(out[0], out[-1]) < 1e-9:
        out.pop()
    return out


def bulge_arc_points(p1: Point, p2: Point, bulge: float, tol: float) -> list[Point]:
    """p1→p2 のふくらみ円弧を近似した点列(p1 は含まず p2 を含む)。

    bulge = tan(θ/4)。正なら反時計回り。中心は弦の中点から左法線方向に
    c(1-b²)/(4b) の位置にある(c は弦長)。
    """
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    c = math.hypot(dx, dy)
    if c < 1e-12:
        return [p2]
    theta = 4.0 * math.atan(bulge)
    mx, my = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
    h = c * (1 - bulge * bulge) / (4 * bulge)
    cx, cy = mx - dy / c * h, my + dx / c * h
    r = math.hypot(p1[0] - cx, p1[1] - cy)
    a1 = math.atan2(p1[1] - cy, p1[0] - cx)
    n = arc_segments(r, theta, tol)
    pts = [(cx + r * math.cos(a1 + theta * k / n), cy + r * math.sin(a1 + theta * k / n)) for k in range(1, n)]
    pts.append(p2)
    return pts


# --------------------------------------------------------------------------
# 端点の連結
# --------------------------------------------------------------------------

def chain_paths(raws: list[_Raw], tol: float, warnings: list[str]) -> list[Contour]:
    """同じレイヤーの開いたポリラインを端点で繋ぎ、閉ループを作る。"""
    result: list[Contour] = []
    by_layer: dict[str, list[_Raw]] = defaultdict(list)
    for r in raws:
        if r.closed:
            result.append(Contour(list(r.points), True, r.layer, r.source))
        else:
            by_layer[r.layer].append(r)

    for layer, items in by_layer.items():
        result.extend(_chain_layer(items, layer, tol, warnings))

    final: list[Contour] = []
    for c in result:
        if c.closed and abs(signed_area(c.points)) < 1e-6:
            warnings.append(f"面積 0 の閉ループ(往復した線?)を開いたパスとして扱います: {c.source}")
            c = Contour(c.points + [c.points[0]], False, c.layer, c.source)
        final.append(c)
    return final


def _chain_layer(items: list[_Raw], layer: str, tol: float, warnings: list[str]) -> list[Contour]:
    cell = tol

    def key(p: Point) -> tuple[int, int]:
        return (math.floor(p[0] / cell), math.floor(p[1] / cell))

    grid: dict[tuple[int, int], list[tuple[int, int]]] = defaultdict(list)  # cell -> [(item, end 0/1)]
    for i, it in enumerate(items):
        grid[key(it.points[0])].append((i, 0))
        grid[key(it.points[-1])].append((i, 1))

    used = [False] * len(items)

    def find(p: Point) -> list[tuple[float, int, int]]:
        kx, ky = key(p)
        found = []
        for gx in (kx - 1, kx, kx + 1):
            for gy in (ky - 1, ky, ky + 1):
                for i, end in grid.get((gx, gy), ()):
                    if used[i]:
                        continue
                    q = items[i].points[0] if end == 0 else items[i].points[-1]
                    d = dist(p, q)
                    if d <= tol:
                        found.append((d, i, end))
        found.sort()
        return found

    out: list[Contour] = []
    for i, it in enumerate(items):
        if used[i]:
            continue
        used[i] = True
        pts = list(it.points)
        sources = [it.source]
        closed = False
        for direction in ("forward", "backward"):
            while True:
                if len(pts) > 2 and dist(pts[0], pts[-1]) <= tol:
                    closed = True
                    break
                tip = pts[-1] if direction == "forward" else pts[0]
                cands = find(tip)
                if not cands:
                    break
                if len(cands) > 1:
                    warnings.append(
                        f"レイヤー '{layer}' の ({tip[0]:.3f}, {tip[1]:.3f}) で 3 本以上の線が接しています"
                        "(分岐・重複線の可能性)。最も近い線に繋ぎます")
                _, j, end = cands[0]
                used[j] = True
                seg = list(items[j].points)
                sources.append(items[j].source)
                if direction == "forward":
                    if end == 1:
                        seg.reverse()
                    pts.extend(seg[1:])
                else:
                    if end == 0:
                        seg.reverse()
                    pts[:0] = seg[:-1]
            if closed:
                break
        if closed:
            pts.pop()  # 終点(≒始点)を落として暗黙に閉じる
        out.append(Contour(pts, closed, layer, _summarize_sources(sources)))
    return out


def _summarize_sources(sources: list[str]) -> str:
    if len(sources) == 1:
        return sources[0]
    kinds = Counter(s.split("(")[0] for s in sources)
    desc = "+".join(f"{k}x{n}" for k, n in kinds.items())
    handles = ",".join(s.split("handle=")[-1].rstrip(")") for s in sources[:4])
    more = ",..." if len(sources) > 4 else ""
    return f"{desc}(handles={handles}{more})"


def remove_duplicate_contours(contours: list[Contour], tol: float, warnings: list[str]) -> list[Contour]:
    """同じ位置に重なった閉ループを 1 本にする(二度切りは焦げ・発火の原因)。"""
    kept: list[Contour] = []
    for c in contours:
        if not c.closed:
            kept.append(c)
            continue
        bb = bbox(c.points)
        dup = False
        for k in kept:
            if not k.closed or k.layer != c.layer:
                continue
            kb = bbox(k.points)
            if max(abs(a - b) for a, b in zip(bb, kb)) > tol * 2:
                continue
            if abs(k.area - c.area) > max(1e-6, 0.01 * k.area):
                continue
            if all(distance_to_polygon(p, k.points) <= tol * 2 for p in c.points):
                dup = True
                break
        if dup:
            warnings.append(f"重複した輪郭を 1 本にまとめました: レイヤー '{c.layer}' {c.source}")
        else:
            kept.append(c)
    return kept
