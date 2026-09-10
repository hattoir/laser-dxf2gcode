"""ツールパス計画: 入れ子判定 → カーフ補正 → 切断順序 → (任意)リードイン。

このツールの核心部分。考え方は README の「設計の要点」を参照。

    偶数階層(0, 2, ...) = 外形(部品の輪郭)  → 内側へ kerf/2 オフセット
    奇数階層(1, 3, ...) = 穴                  → 外側へ kerf/2 オフセット

切断順序は「深い階層から」。同じ階層の中では直前の終点に最も近い輪郭を
貪欲に選ぶ。これで「ある部品の穴は、必ずその部品の外形より先に切られる」
ことが保証される(穴は外形より 1 つ深い階層なので)。

補正後の閉ループは向きをそろえる: 外形 = 反時計回り、穴 = 時計回り。
こうすると「進行方向の右側が常に捨て側(スクラップ)」になり、
リードインの向きを外形・穴の区別なく決められる。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

import pyclipper

from .config import LayerSettings, Profile
from .dxf_reader import Contour
from .errors import GeometryError
from .geometry import (Point, bbox, dist, distance_to_polygon, point_in_polygon,
                       point_segment_distance, signed_area)

CLIPPER_SCALE = 10000.0  # 0.1 µm 単位の整数にしてから pyclipper に渡す


@dataclass
class CutPath:
    """1 本のレーザー軌跡(1 パス分)。points は走る順で、閉じた輪郭は始点に戻って終わる。"""
    points: list[Point]
    layer: str
    settings: LayerSettings
    role: str                      # "outer" | "hole" | "open" | "mark"(貫通しない層)
    depth: int = -1                # 入れ子の階層。貫通切断の閉ループ以外は -1
    part_id: int | None = None     # 所属する部品(外形)の番号
    source: str = ""
    original: list[Point] | None = None  # 補正前の輪郭(プレビュー用)
    closed: bool = True
    lead_in: bool = False          # True なら points[0]→points[1] がリードイン

    @property
    def start(self) -> Point:
        return self.points[0]

    @property
    def end(self) -> Point:
        return self.points[-1]


@dataclass
class Plan:
    paths: list[CutPath]
    warnings: list[str] = field(default_factory=list)
    parts: int = 0
    holes: int = 0
    open_paths: int = 0
    kerf: float = 0.0


# --------------------------------------------------------------------------
# 入れ子判定
# --------------------------------------------------------------------------

def _representative_inside(inner: Sequence[Point], outer: Sequence[Point]) -> bool:
    """inner が outer の内側にあるか。境界上に乗った頂点は飛ばし、多数決で判定する。"""
    votes_in = votes_out = 0
    step = max(1, len(inner) // 16)
    for p in inner[::step]:
        if distance_to_polygon(p, outer) < 1e-6:
            continue
        if point_in_polygon(p, outer):
            votes_in += 1
        else:
            votes_out += 1
        if votes_in + votes_out >= 5:
            break
    return votes_in > votes_out


def compute_nesting(polys: Sequence[Sequence[Point]]) -> tuple[list[int], list[int | None]]:
    """各閉ループの入れ子階層と親(自分を直接含む最小のループ)を求める。

    depth[i] = i を含む他のループの数。0 = 一番外側。
    """
    n = len(polys)
    areas = [abs(signed_area(p)) for p in polys]
    boxes = [bbox(p) for p in polys]
    depth = [0] * n
    parent: list[int | None] = [None] * n
    for i in range(n):
        bi = boxes[i]
        best = None
        for j in range(n):
            if i == j or areas[j] <= areas[i]:
                continue
            bj = boxes[j]
            if bi[0] < bj[0] - 1e-9 or bi[1] < bj[1] - 1e-9 or bi[2] > bj[2] + 1e-9 or bi[3] > bj[3] + 1e-9:
                continue  # 外接矩形が収まらなければ内側ではない
            if _representative_inside(polys[i], polys[j]):
                depth[i] += 1
                if best is None or areas[j] < areas[best]:
                    best = j
        parent[i] = best
    return depth, parent


# --------------------------------------------------------------------------
# カーフ補正
# --------------------------------------------------------------------------

def orient(poly: list[Point], ccw: bool) -> list[Point]:
    return poly if (signed_area(poly) > 0) == ccw else poly[::-1]


def offset_polygon(poly: Sequence[Point], delta: float, arc_tol: float = 0.01) -> list[list[Point]]:
    """閉ループを delta [mm] だけオフセットする(正 = 外側へ広げる、負 = 内側へ縮める)。

    角は丸め (JT_ROUND)。レーザーのビームは円なので、穴の凸角を外側へ
    オフセットしたときに円弧で回り込むのが幾何学的に正しい軌跡になる。
    結果は 0 個(消えた)・1 個・複数個(くびれで分裂)のいずれか。
    """
    if delta == 0:
        return [list(poly)]
    pco = pyclipper.PyclipperOffset()
    pco.ArcTolerance = max(0.25, arc_tol * CLIPPER_SCALE)
    pco.AddPath(pyclipper.scale_to_clipper(orient(list(poly), True), CLIPPER_SCALE),
                pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
    res = pyclipper.scale_from_clipper(pco.Execute(delta * CLIPPER_SCALE), CLIPPER_SCALE)
    out = [[(float(x), float(y)) for x, y in r] for r in res]
    # 外周だけ残す(穴側の輪郭が内部に島を作っても、そこは捨て材の中)
    return [r for r in out if len(r) >= 3 and signed_area(r) > 0]


# --------------------------------------------------------------------------
# 計画
# --------------------------------------------------------------------------

@dataclass
class _Item:
    poly: list[Point]
    closed: bool
    layer: str
    settings: LayerSettings
    role: str
    depth: int = -1
    part_id: int | None = None
    source: str = ""
    original: list[Point] | None = None


def plan_toolpaths(contours: Sequence[Contour], profile: Profile, kerf: float | None = None,
                   lead_in: float = 0.0, start: Point = (0.0, 0.0), arc_tol: float = 0.01) -> Plan:
    """輪郭のリストから、補正済み・順序付きの CutPath のリストを作る。

    kerf: None ならプロファイルの値。lead_in: 外形の進入線の長さ [mm](0 でなし)。
    start: ヘッドの初期位置(最初の輪郭の選択に使う)。
    """
    kerf = profile.kerf if kerf is None else kerf
    if kerf < 0:
        raise GeometryError("kerf は 0 以上にしてください")
    warnings: list[str] = []
    unmatched: set[str] = set()

    marks: list[_Item] = []
    through_open: list[_Item] = []
    through_closed: list[Contour] = []
    through_settings: list[LayerSettings] = []

    for c in contours:
        settings, matched = profile.settings_for(c.layer)
        if not matched and c.layer not in unmatched:
            unmatched.add(c.layer)
            warnings.append(f"DXF レイヤー '{c.layer}' はプロファイルに無いため '{settings.name}' の設定で加工します")
        if not settings.through_cut:
            marks.append(_Item(list(c.points), c.closed, c.layer, settings, "mark", source=c.source,
                               original=list(c.points)))
        elif not c.closed:
            through_open.append(_Item(list(c.points), False, c.layer, settings, "open", source=c.source,
                                      original=list(c.points)))
        else:
            through_closed.append(c)
            through_settings.append(settings)

    # --- 入れ子判定とカーフ補正(貫通切断の閉ループ) ---
    depths, parents = compute_nesting([c.points for c in through_closed])
    part_of: list[int | None] = [None] * len(through_closed)
    part_ids: dict[int, int] = {}
    for i in sorted(range(len(through_closed)), key=lambda k: depths[k]):
        if depths[i] % 2 == 0:
            part_ids[i] = len(part_ids) + 1
            part_of[i] = part_ids[i]
        else:
            par = parents[i]
            part_of[i] = part_ids.get(par) if par is not None else None

    cut_items: list[_Item] = []
    for i, c in enumerate(through_closed):
        s = through_settings[i]
        is_hole = depths[i] % 2 == 1
        role = "hole" if is_hole else "outer"
        delta = 0.0
        if s.kerf_compensation and kerf > 0:
            delta = kerf / 2 if is_hole else -kerf / 2
        results = offset_polygon(c.points, delta, arc_tol)
        if not results:
            what = "穴" if is_hole else "外形"
            raise GeometryError(
                f"{what}(レイヤー '{c.layer}' {c.source})がカーフ補正 {delta:+.3f}mm で消えてしまいます。"
                "カーフより細い形状は切れません")
        if len(results) > 1:
            warnings.append(f"カーフ補正で輪郭が {len(results)} 本に分かれました(細いくびれ): {c.source}")
        for r in results:
            cut_items.append(_Item(orient(r, ccw=not is_hole), True, c.layer, s, role, depths[i], part_of[i],
                                   c.source, list(c.points)))
    if any(not s.kerf_compensation for s in through_settings):
        warnings.append("kerf_compensation: false の貫通切断レイヤーがあります(寸法は線の中心になります)")

    # --- 順序付け ---
    paths: list[CutPath] = []
    pos = start
    # 1) 貫通しない層(刻印など)。部品が落ちる前に全部済ませる
    for order_key in sorted({(m.settings.order, m.layer) for m in marks}):
        group = [m for m in marks if (m.settings.order, m.layer) == order_key]
        pos = _greedy(group, pos, paths, 0.0, [], warnings)
    # 2) 貫通切断の開いたパス(スリット等)。閉ループより先に切る
    pos = _greedy(through_open, pos, paths, 0.0, [], warnings)
    # 3) 貫通切断の閉ループ: 深い階層から
    all_polys = [(it.poly, it.depth) for it in cut_items]
    for d in sorted({it.depth for it in cut_items}, reverse=True):
        group = [it for it in cut_items if it.depth == d]
        pos = _greedy(group, pos, paths, lead_in, all_polys, warnings)

    return Plan(paths=paths, warnings=warnings,
                parts=sum(1 for d in depths if d % 2 == 0),
                holes=sum(1 for d in depths if d % 2 == 1),
                open_paths=len(through_open) + sum(1 for m in marks if not m.closed),
                kerf=kerf)


def _nearest_on_polygon(p: Point, poly: Sequence[Point]) -> tuple[float, int, Point]:
    """poly 上で p に最も近い点。(距離, 辺の番号 i(poly[i]→poly[i+1]), 点)。"""
    best = (math.inf, 0, poly[0])
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
        q = (a[0] + t * dx, a[1] + t * dy)
        d = dist(p, q)
        if d < best[0]:
            best = (d, i, q)
    return best


def _nearest_vertex(p: Point, poly: Sequence[Point]) -> tuple[float, int]:
    i = min(range(len(poly)), key=lambda k: dist(p, poly[k]))
    return dist(p, poly[i]), i


def _greedy(items: list[_Item], pos: Point, out: list[CutPath], lead_in: float,
            all_polys: list[tuple[list[Point], int]], warnings: list[str]) -> Point:
    """直前の終点に最も近いものから順に並べる(貪欲法)。閉ループは開始点も回転する。"""
    remaining = list(items)
    while remaining:
        best_k, best_d, best_how = 0, math.inf, None
        for k, it in enumerate(remaining):
            if it.closed:
                d, vi = _nearest_vertex(pos, it.poly)
                how = ("vertex", vi)
            else:
                d0, d1 = dist(pos, it.poly[0]), dist(pos, it.poly[-1])
                d, how = (d0, ("fwd", 0)) if d0 <= d1 else (d1, ("rev", 0))
            if d < best_d:
                best_k, best_d, best_how = k, d, how
        it = remaining.pop(best_k)
        if not it.closed:
            pts = it.poly if best_how[0] == "fwd" else it.poly[::-1]
            out.append(CutPath(list(pts), it.layer, it.settings, it.role, it.depth, it.part_id,
                               it.source, it.original, closed=False))
            pos = pts[-1]
            continue

        use_lead = lead_in > 0 and it.role == "outer"
        pts, lead_pt = None, None
        if use_lead:
            pts, lead_pt = _with_lead_in(it, pos, lead_in, all_polys)
            if pts is None:
                warnings.append(f"リードインを置ける場所が見つからないため省略しました: {it.source}")
        if pts is None:
            vi = best_how[1]
            ring = it.poly[vi:] + it.poly[:vi]
            pts = ring + [ring[0]]
        out.append(CutPath(pts, it.layer, it.settings, it.role, it.depth, it.part_id, it.source,
                           it.original, closed=True, lead_in=lead_pt is not None))
        pos = pts[-1]
    return pos


def _with_lead_in(it: _Item, pos: Point, length: float,
                  all_polys: list[tuple[list[Point], int]]) -> tuple[list[Point] | None, Point | None]:
    """外形に進入線を付ける。候補点: 現在位置に最も近い点 → 長い辺の中点(長い順)。

    進入線の始点は「捨て側(進行方向の右)」に length 離した点。その点が
    どの輪郭からも十分離れていて、かつ入れ子の階層がこの外形と同じ
    (= この部品の周りの捨て材の上)であることを確認する。
    """
    poly = it.poly  # 外形は CCW にそろえてある → 右側が外(捨て側)
    n = len(poly)
    _, ei, q = _nearest_on_polygon(pos, poly)
    candidates = [(ei, q)]
    edges = sorted(range(n), key=lambda i: -dist(poly[i], poly[(i + 1) % n]))
    for i in edges[:8]:
        a, b = poly[i], poly[(i + 1) % n]
        candidates.append((i, ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)))
    for i, q in candidates:
        a, b = poly[i], poly[(i + 1) % n]
        L = dist(a, b)
        if L < 1e-9:
            continue
        nx, ny = (b[1] - a[1]) / L, -(b[0] - a[0]) / L  # 右法線
        lp = (q[0] + nx * length, q[1] + ny * length)
        if any(distance_to_polygon(lp, p) < 0.5 * length for p, _ in all_polys):
            continue
        level = sum(1 for p, _ in all_polys if point_in_polygon(lp, p))
        if level != it.depth:
            continue
        # q を頂点として挿入し、q から一周して q に戻る
        ring = [q] + poly[i + 1:] + poly[:i + 1]
        if dist(ring[1], q) < 1e-9:
            ring = ring[1:]
        return [lp] + ring + [ring[0]], lp
    return None, None
