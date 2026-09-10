"""SVG プレビューと加工レポート。

プレビューは Plan ではなく「生成済みの G-code テキスト」を読み直して描く。
こうすると、G-code 生成部に不具合があってもプレビューに現れるので、
目視確認が本当に「流すコード」の確認になる。補正前の輪郭(DXF の形)は
比較用に薄い灰色で重ねる。
"""
from __future__ import annotations

import bisect
import html
import math
import re
from dataclasses import dataclass, field
from typing import Sequence

from .config import Machine
from .gcode import Motion, ParsedGcode
from .geometry import Point, dist, polyline_length
from .toolpath import Plan

# --------------------------------------------------------------------------
# 所要時間の推定(GRBL の先読みプランナの簡易版)
# --------------------------------------------------------------------------


def _axis_limited(ux: float, uy: float, lim_x: float, lim_y: float) -> float:
    """方向 (ux, uy) に進むときの、各軸の上限から決まる合成の上限。"""
    vals = []
    if abs(ux) > 1e-12:
        vals.append(lim_x / abs(ux))
    if abs(uy) > 1e-12:
        vals.append(lim_y / abs(uy))
    return min(vals) if vals else math.inf


def _trapezoid_time(L: float, v0: float, v1: float, vmax: float, a: float) -> float:
    if L <= 0:
        return 0.0
    d_acc = max(0.0, (vmax * vmax - v0 * v0) / (2 * a))
    d_dec = max(0.0, (vmax * vmax - v1 * v1) / (2 * a))
    if d_acc + d_dec <= L:
        return (vmax - v0) / a + (vmax - v1) / a + (L - d_acc - d_dec) / vmax
    vp = math.sqrt(max(v0 * v0, v1 * v1, (2 * a * L + v0 * v0 + v1 * v1) / 2))
    return max(0.0, (vp - v0) / a) + max(0.0, (vp - v1) / a)


def chain_time(segments: Sequence[tuple[Point, Point, float]], machine: Machine) -> float:
    """停止から始まり停止で終わる、連続した直線ブロックの所要時間 [s]。

    segments: (始点, 終点, 指令速度 mm/min)。角ではジャンクション偏差の式
    v² = a·δ·sin(θ/2)/(1−sin(θ/2)) で減速し、前後方向の 2 パスで
    加減速が間に合う速度に制限する(GRBL と同じ考え方)。
    """
    segs = []
    for a_pt, b_pt, feed in segments:
        L = dist(a_pt, b_pt)
        if L < 1e-9:
            continue
        ux, uy = (b_pt[0] - a_pt[0]) / L, (b_pt[1] - a_pt[1]) / L
        vmax = min(feed / 60.0, _axis_limited(ux, uy, machine.max_rate_x / 60, machine.max_rate_y / 60))
        acc = _axis_limited(ux, uy, machine.accel_x, machine.accel_y)
        segs.append((L, ux, uy, vmax, acc))
    n = len(segs)
    if n == 0:
        return 0.0
    # 各ブロック入口の最大速度(角の制限)
    vj = [0.0] * (n + 1)
    for i in range(1, n):
        L0, ux0, uy0, v0, a0 = segs[i - 1]
        L1, ux1, uy1, v1, a1 = segs[i]
        cos_t = -(ux0 * ux1 + uy0 * uy1)
        if cos_t > 0.999999:
            v = 0.0
        elif cos_t < -0.999999:
            v = math.inf
        else:
            sin_h = math.sqrt(0.5 * (1.0 - cos_t))
            v = math.sqrt(min(a0, a1) * machine.junction_deviation * sin_h / (1.0 - sin_h))
        vj[i] = min(v, v0, v1)
    # 後ろ向き → 前向き
    for i in range(n - 1, -1, -1):
        L, _, _, _, acc = segs[i]
        vj[i] = min(vj[i], math.sqrt(vj[i + 1] ** 2 + 2 * acc * L))
    for i in range(n):
        L, _, _, _, acc = segs[i]
        vj[i + 1] = min(vj[i + 1], math.sqrt(vj[i] ** 2 + 2 * acc * L))
    return sum(_trapezoid_time(L, vj[i], vj[i + 1], vmax, acc) for i, (L, _, _, vmax, acc) in enumerate(segs))


def split_chains(parsed: ParsedGcode) -> list[list[Motion]]:
    """G-code を「止まらずに走る区間」に分ける。G0 は 1 ブロックずつ、
    G1 は M4/M5 などの命令を挟まずに連続する範囲を 1 区間とする。"""
    cmd_lines = [no for no, _ in parsed.commands]
    motion_lines = {m.line_no for m in parsed.motions}
    non_motion = sorted(set(cmd_lines) - motion_lines)

    chains: list[list[Motion]] = []
    prev: Motion | None = None
    for m in parsed.motions:
        same = (prev is not None and prev.kind == "G1" and m.kind == "G1"
                and bisect.bisect_left(non_motion, prev.line_no) == bisect.bisect_left(non_motion, m.line_no))
        if same:
            chains[-1].append(m)
        else:
            chains.append([m])
        prev = m
    return chains


@dataclass
class TimeEstimate:
    cut_s: float
    travel_s: float

    @property
    def total_s(self) -> float:
        return self.cut_s + self.travel_s


def estimate_time(parsed: ParsedGcode, machine: Machine) -> TimeEstimate:
    cut = travel = 0.0
    rapid_feed = max(machine.max_rate_x, machine.max_rate_y)
    for ch in split_chains(parsed):
        feed_of = (lambda m: rapid_feed) if ch[0].kind == "G0" else (lambda m: m.feed)
        t = chain_time([(m.start, m.end, feed_of(m)) for m in ch], machine)
        if ch[0].kind == "G0":
            travel += t
        else:
            cut += t
    return TimeEstimate(cut, travel)


def fmt_duration(s: float) -> str:
    s = int(round(s))
    return f"{s // 3600:d}:{s % 3600 // 60:02d}:{s % 60:02d}"


# --------------------------------------------------------------------------
# 焼き区間(M4〜M5)の抽出
# --------------------------------------------------------------------------

_ROLE = re.compile(r";\s*path (\d+)/\d+ layer=(\S*) role=(\w+)")


@dataclass
class Burn:
    points: list[Point]
    role: str
    path_no: int | None
    line_no: int


def extract_burns(parsed: ParsedGcode, text: str) -> list[Burn]:
    roles: dict[int, tuple[int, str]] = {}
    for no, raw in enumerate(text.splitlines(), 1):
        m = _ROLE.search(raw)
        if m:
            roles[no] = (int(m.group(1)), m.group(3))
    role_lines = sorted(roles)

    burns: list[Burn] = []
    prev: Motion | None = None
    for m in parsed.motions:
        if not m.burning:
            prev = m
            continue
        cont = (prev is not None and prev.burning and burns and burns[-1].points[-1] == m.start
                and bisect.bisect_right(role_lines, prev.line_no) == bisect.bisect_right(role_lines, m.line_no))
        if cont:
            burns[-1].points.append(m.end)
        else:
            k = bisect.bisect_right(role_lines, m.line_no) - 1
            path_no, role = roles[role_lines[k]] if k >= 0 else (None, "?")
            burns.append(Burn([m.start, m.end], role, path_no, m.line_no))
        prev = m
    return burns


def burns_bbox(burns: Sequence[Burn]) -> tuple[float, float, float, float] | None:
    pts = [p for b in burns for p in b.points]
    if not pts:
        return None
    return (min(p[0] for p in pts), min(p[1] for p in pts), max(p[0] for p in pts), max(p[1] for p in pts))


# --------------------------------------------------------------------------
# SVG
# --------------------------------------------------------------------------

def _color(i: int, n: int) -> str:
    """切断順のグラデーション: 青(最初)→ 緑 → 赤(最後)。"""
    t = 0.0 if n <= 1 else i / (n - 1)
    return f"hsl({240 - 240 * t:.0f},85%,42%)"


def render_svg(text: str, parsed: ParsedGcode, machine: Machine, plan: Plan | None = None,
               title: str = "", info_lines: Sequence[str] = (), warn_lines: Sequence[str] = (),
               window: tuple[float, float, float, float] | None = None) -> str:
    """SVG を返す。window=(x0, y0, x1, y1) を与えるとその範囲だけを拡大表示する。

    座標は 1 単位 = 1mm。機械の Y は上向き、SVG は下向きなので Y を反転する。
    """
    W, H = machine.x_max, machine.y_max
    wx0, wy0, wx1, wy1 = window if window else (0.0, 0.0, W, H)
    k = max(0.35, max(wx1 - wx0, wy1 - wy0) / max(W, H))  # 文字・余白の倍率
    m = 12.0 * k
    header_h = (8.0 + 5.0 * (len(info_lines) + len(warn_lines))) * k
    vx, vy = wx0 - m, -wy1 - m - header_h
    vw, vh = (wx1 - wx0) + 2 * m, (wy1 - wy0) + 2 * m + header_h

    def X(x: float) -> float:
        return x

    def Y(y: float) -> float:
        return -y

    def pts(ps: Sequence[Point]) -> str:
        return " ".join(f"{x:.3f},{-y:.3f}" for x, y in ps)

    o: list[str] = []
    px_w = 1000
    o.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{vx:.2f} {vy:.2f} {vw:.2f} {vh:.2f}" '
             f'width="{px_w}" height="{px_w * vh / vw:.0f}" font-family="sans-serif">')
    o.append(f'<rect x="{vx:.2f}" y="{vy:.2f}" width="{vw:.2f}" height="{vh:.2f}" fill="#ffffff"/>')

    # 加工エリアとグリッド(拡大表示では 10mm 間隔)
    step = 50 if (wx1 - wx0) > 200 or (wy1 - wy0) > 200 else 10
    o.append(f'<rect x="0" y="{Y(H)}" width="{W}" height="{H}" fill="#fafafa" stroke="#222" stroke-width="{0.6 * k:.2f}"/>')
    g = []
    for gx in range(0, int(W) + 1, step):
        g.append(f'<line x1="{gx}" y1="{Y(0)}" x2="{gx}" y2="{Y(H)}"/>')
        if wx0 - 1e-9 <= gx <= wx1 + 1e-9:
            o.append(f'<text x="{gx}" y="{Y(wy0) + m * 0.4:.2f}" font-size="{3 * k:.2f}" text-anchor="middle" '
                     f'fill="#666">{gx}</text>')
    for gy in range(0, int(H) + 1, step):
        g.append(f'<line x1="0" y1="{Y(gy)}" x2="{W}" y2="{Y(gy)}"/>')
        if wy0 - 1e-9 <= gy <= wy1 + 1e-9:
            o.append(f'<text x="{wx0 - 1.5 * k:.2f}" y="{Y(gy) + k:.2f}" font-size="{3 * k:.2f}" text-anchor="end" '
                     f'fill="#666">{gy}</text>')
    o.append(f'<g stroke="#e4e4e4" stroke-width="{0.2 * k:.2f}">' + "".join(g) + "</g>")
    o.append(f'<text x="{W}" y="{Y(H) - 1.5 * k:.2f}" font-size="{3.2 * k:.2f}" text-anchor="end" fill="#222">'
             f'加工エリア {W:g} × {H:g} mm</text>')
    o.append(f'<circle cx="0" cy="0" r="{1.2 * k:.2f}" fill="#222"/>'
             f'<text x="{2 * k:.2f}" y="{-2 * k:.2f}" font-size="{3 * k:.2f}" fill="#222">原点 (0,0)</text>')

    # 補正前の輪郭(DXF の形)
    if plan is not None:
        o.append(f'<g fill="none" stroke="#9a9a9a" stroke-width="{0.12 * k:.3f}">')
        for p in plan.paths:
            if p.original:
                tag = "polygon" if p.closed else "polyline"
                o.append(f'<{tag} points="{pts(p.original)}"/>')
        o.append("</g>")

    # 早送り(G0)
    o.append(f'<g stroke="#7a7a7a" stroke-width="{0.25 * k:.3f}" stroke-dasharray="{1.6 * k:.2f},{1.2 * k:.2f}" '
             'opacity="0.75" fill="none">')
    for mo in parsed.motions:
        if mo.kind == "G0" and dist(mo.start, mo.end) > 1e-6:
            o.append(f'<line x1="{X(mo.start[0]):.3f}" y1="{Y(mo.start[1]):.3f}" '
                     f'x2="{X(mo.end[0]):.3f}" y2="{Y(mo.end[1]):.3f}"/>')
    o.append("</g>")

    # 焼き区間: 同じ輪郭の繰り返し(パス回数)は 1 本にまとめ、番号に ×N を付ける
    burns = extract_burns(parsed, text)
    groups: dict[tuple, list[Burn]] = {}
    for b in burns:
        key = (b.path_no, round(b.points[0][0], 3), round(b.points[0][1], 3), len(b.points))
        groups.setdefault(key, []).append(b)
    uniq = list(groups.values())
    n = len(uniq)
    o.append('<g fill="none" stroke-linejoin="round" stroke-linecap="round">')
    for i, grp in enumerate(uniq):
        b = grp[0]
        width = (0.3 if b.role == "mark" else 0.45) * k
        dash = f' stroke-dasharray="{0.8 * k:.2f},{0.5 * k:.2f}"' if b.role == "mark" else ""
        o.append(f'<polyline points="{pts(b.points)}" stroke="{_color(i, n)}" stroke-width="{width:.3f}"{dash}/>')
    o.append("</g>")
    # 開始点と番号
    o.append(f'<g font-size="{3.2 * k:.2f}" font-weight="bold" paint-order="stroke" stroke="#fff" '
             f'stroke-width="{0.8 * k:.2f}">')
    for i, grp in enumerate(uniq):
        b = grp[0]
        sx, sy = b.points[0]
        label = f"{i + 1}" + (f"×{len(grp)}" if len(grp) > 1 else "")
        o.append(f'<circle cx="{X(sx):.3f}" cy="{Y(sy):.3f}" r="{0.8 * k:.2f}" fill="{_color(i, n)}" stroke="none"/>')
        o.append(f'<text x="{X(sx) + 1.2 * k:.3f}" y="{Y(sy) - 1.2 * k:.3f}" fill="{_color(i, n)}">{label}</text>')
    o.append("</g>")

    # 材料の使用範囲
    bb = burns_bbox(burns)
    if bb:
        x0, y0, x1, y1 = bb
        o.append(f'<rect x="{X(x0):.3f}" y="{Y(y1):.3f}" width="{x1 - x0:.3f}" height="{y1 - y0:.3f}" '
                 f'fill="none" stroke="#e08000" stroke-width="{0.3 * k:.2f}" '
                 f'stroke-dasharray="{3 * k:.2f},{1.5 * k:.2f}"/>')
        o.append(f'<text x="{X(x0):.3f}" y="{Y(y1) - 1.2 * k:.3f}" font-size="{3 * k:.2f}" fill="#e08000">'
                 f'使用範囲 {x1 - x0:.1f} × {y1 - y0:.1f} mm</text>')

    # 凡例
    o.append(f'<text x="{wx0:.2f}" y="{Y(wy0) + m * 0.85:.2f}" font-size="{2.6 * k:.2f}" fill="#333">'
             '線の色 = 切断順(青→緑→赤)/ 数字 = 順番(×N はパス回数)/ 破線(灰) = 早送り G0(レーザー OFF)'
             ' / 細い灰線 = 補正前の DXF 輪郭 / 橙の破線 = 材料の使用範囲</text>')

    # ヘッダ(最後に描く。拡大表示では加工エリアの塗りの上に重なるため白帯を敷く)
    o.append(f'<rect x="{vx:.2f}" y="{vy:.2f}" width="{vw:.2f}" height="{header_h:.2f}" fill="#ffffff"/>')
    y = vy + 7.0 * k
    o.append(f'<text x="{wx0:.2f}" y="{y:.2f}" font-size="{5 * k:.2f}" font-weight="bold">{html.escape(title)}</text>')
    for line in info_lines:
        y += 5.0 * k
        o.append(f'<text x="{wx0:.2f}" y="{y:.2f}" font-size="{3.6 * k:.2f}" fill="#333">{html.escape(line)}</text>')
    for line in warn_lines:
        y += 5.0 * k
        o.append(f'<text x="{wx0:.2f}" y="{y:.2f}" font-size="{3.6 * k:.2f}" fill="#c00000">{html.escape(line)}</text>')
    o.append("</svg>")
    return "\n".join(o) + "\n"


# --------------------------------------------------------------------------
# 加工レポート
# --------------------------------------------------------------------------

@dataclass
class Report:
    text: str
    cut_length: float
    travel_length: float
    time: TimeEstimate
    bbox: tuple[float, float, float, float] | None
    burns: int
    contours: int
    extra: dict = field(default_factory=dict)


def build_report(text: str, parsed: ParsedGcode, machine: Machine, plan: Plan,
                 meta: dict[str, str], warnings: Sequence[str]) -> Report:
    cut_len = sum(dist(m.start, m.end) for m in parsed.motions if m.burning)
    travel_len = sum(dist(m.start, m.end) for m in parsed.motions if m.kind == "G0")
    t = estimate_time(parsed, machine)
    burns = extract_burns(parsed, text)
    bb = burns_bbox(burns)

    L: list[str] = []
    L.append("=" * 60)
    L.append(" 加工レポート")
    L.append("=" * 60)
    for k, v in meta.items():
        L.append(f"{k:<14}: {v}")
    L.append("-" * 60)
    L.append(f"部品数(外形)  : {plan.parts}")
    L.append(f"穴の数        : {plan.holes}")
    if plan.open_paths:
        L.append(f"開いたパス    : {plan.open_paths}")
    L.append(f"輪郭の数      : {len(plan.paths)}")
    L.append(f"パス総数      : {len(burns)}(輪郭 × パス回数 = 発振 M4〜M5 の回数)")
    L.append(f"切断距離 合計 : {cut_len:,.1f} mm(パス回数込み)")
    L.append(f"移動距離 合計 : {travel_len:,.1f} mm(G0 早送り)")
    L.append(f"推定所要時間  : {fmt_duration(t.total_s)}(切断 {fmt_duration(t.cut_s)} + 移動 {fmt_duration(t.travel_s)})")
    L.append(f"                ※ 送り速度・加速度 {machine.accel_x:g}/{machine.accel_y:g} mm/s²・"
             f"ジャンクション偏差 {machine.junction_deviation:g} mm からの概算")
    if bb:
        L.append(f"材料の使用範囲: X {bb[0]:.2f} 〜 {bb[2]:.2f}, Y {bb[1]:.2f} 〜 {bb[3]:.2f}"
                 f"  ({bb[2] - bb[0]:.2f} × {bb[3] - bb[1]:.2f} mm)")
    L.append("-" * 60)
    L.append("レイヤー別:")
    per_layer: dict[str, list] = {}
    for p in plan.paths:
        e = per_layer.setdefault(p.layer, [p.settings, 0, 0.0])
        e[1] += 1
        e[2] += polyline_length(p.points) * p.settings.passes
    for layer, (s, cnt, length) in per_layer.items():
        pp = s.pass_settings()
        detail = ", ".join(f"{q.power:g}%/{q.feed:g}" for q in pp)
        L.append(f"  {layer:<10} → 設定 '{s.name}': {s.passes} パス [{detail}] (出力%/mm/min)  "
                 f"輪郭 {cnt} 本, 切断 {length:,.1f} mm")
    if warnings:
        L.append("-" * 60)
        L.append("警告:")
        for w in warnings:
            L.append(f"  ! {w}")
    if machine.unverified:
        L.append("-" * 60)
        L.append("未確認の仮定値(machine.yaml の unverified): " + ", ".join(machine.unverified))
    L.append("-" * 60)
    L.append("流す前に:")
    L.append("  1. SVG プレビューを開き、形・位置・切断順(内→外)を目視で確認する")
    L.append("  2. 必ず端材でテストしてから本番材を切る")
    L.append("  3. 換気・火焔警報($154=1)を確認し、加工中はその場を離れない")
    return Report("\n".join(L) + "\n", cut_len, travel_len, t, bb, len(burns), len(plan.paths))
