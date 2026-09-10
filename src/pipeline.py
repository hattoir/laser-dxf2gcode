"""DXF → 計画 → G-code → 検証 → プレビュー/レポート をつなぐ処理。

convert コマンドとキャリブレーション(calibrate)の両方がここを通る。
ファイルの書き出し順は SVG → レポート → G-code(最後)。途中で失敗したら
G-code は書かれない。同名の古い G-code が残っていると誤って流す危険が
あるので、失敗時は .stale に改名して退避する。
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

from .config import Machine, Profile
from .dxf_reader import Contour
from .errors import Dxf2GcodeError
from .gcode import GcodeResult, fmt, generate_gcode, verify_gcode
from .geometry import bbox
from .preview import Report, build_report, fmt_duration, render_svg
from .toolpath import Plan, plan_toolpaths


@dataclass
class Outcome:
    plan: Plan
    gcode: GcodeResult
    report: Report
    gcode_path: Path
    svg_path: Path
    report_path: Path
    frame_path: Path | None = None
    warnings: list[str] = field(default_factory=list)
    detail_path: Path | None = None


def output_paths(out_gcode: Path) -> tuple[Path, Path, Path]:
    base = out_gcode.with_suffix("")
    return out_gcode, base.with_name(base.name + "_preview.svg"), base.with_name(base.name + "_report.txt")


def transform_contours(contours: Sequence[Contour], align: str = "none", margin: float = 5.0,
                       offset: tuple[float, float] = (0.0, 0.0)) -> tuple[list[Contour], tuple[float, float]]:
    """配置: align="lower-left" なら外接矩形の左下を (margin, margin) に移し、さらに offset を足す。"""
    dx, dy = offset
    if align == "lower-left" and contours:
        x0, y0, _, _ = bbox(p for c in contours for p in c.points)
        dx += margin - x0
        dy += margin - y0
    elif align != "none":
        raise Dxf2GcodeError(f"--align は none / lower-left のいずれか: {align}")
    if dx == 0 and dy == 0:
        return list(contours), (0.0, 0.0)
    moved = [Contour([(x + dx, y + dy) for x, y in c.points], c.closed, c.layer, c.source) for c in contours]
    return moved, (dx, dy)


def frame_gcode(report: Report, machine: Machine) -> str | None:
    """材料の使用範囲の外周を、レーザー OFF(G0 S0)でなぞるだけの G-code。
    材料を置いた位置が合っているかをヘッドの動きで確かめるためのもの。"""
    if report.bbox is None:
        return None
    x0, y0, x1, y1 = report.bbox
    lines = ["; frame: laser OFF, rapid moves only (check material placement)",
             "G21 G90 G94", "M5", "S0"]
    for x, y in [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]:
        lines.append(f"G0 X{fmt(x)} Y{fmt(y)} S0")
    if machine.return_home:
        lines.append(f"G0 X{fmt(machine.park_x)} Y{fmt(machine.park_y)} S0")
    lines.append("M5")
    text = "\n".join(lines) + "\n"
    verify_gcode(text, machine)
    return text


def _retire_stale(path: Path) -> Path | None:
    if path.exists():
        stale = path.with_name(path.name + ".stale")
        os.replace(path, stale)
        return stale
    return None


def _write_atomic(path: Path, text: str) -> None:
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    os.replace(tmp, path)


def process(contours: Sequence[Contour], profile: Profile, machine: Machine, out_gcode: Path, *,
            kerf: float | None = None, lead_in: float = 0.0, pass_mode: str = "path",
            meta: dict[str, str] | None = None, read_warnings: Sequence[str] = (),
            title: str = "", write_frame: bool = False) -> Outcome:
    out_gcode = Path(out_gcode)
    out_gcode.parent.mkdir(parents=True, exist_ok=True)
    gcode_path, svg_path, report_path = output_paths(out_gcode)
    frame_path = out_gcode.with_name(out_gcode.stem + "_frame.gcode") if write_frame else None
    meta = dict(meta or {})
    try:
        plan = plan_toolpaths(contours, profile, kerf=kerf, lead_in=lead_in,
                              start=(machine.park_x, machine.park_y),
                              bounds=(0.0, 0.0, machine.x_max, machine.y_max))
        meta.setdefault("profile", profile.name)
        meta["kerf"] = f"{plan.kerf:.3f} mm"
        result = generate_gcode(plan, machine, pass_mode=pass_mode, meta=meta)
        parsed = verify_gcode(result.text, machine)
        warnings = list(read_warnings) + plan.warnings + result.warnings
        report = build_report(result.text, parsed, machine, plan, meta, warnings)
        info = [f"{k}: {v}" for k, v in meta.items()]
        info.append(f"部品 {plan.parts} / 穴 {plan.holes} / パス総数 {report.burns} / 切断 {report.cut_length:,.0f} mm"
                    f" / 推定 {fmt_duration(report.time.total_s)}")
        warn_lines = []
        if machine.unverified:
            warn_lines.append("未確認の仮定値あり: " + ", ".join(machine.unverified))
        if warnings:
            warn_lines.append(f"警告 {len(warnings)} 件(レポート参照)")
        svg = render_svg(result.text, parsed, machine, plan, title or out_gcode.name, info, warn_lines)
        detail = None
        if report.bbox:
            x0, y0, x1, y1 = report.bbox
            pad = 8.0
            detail = render_svg(result.text, parsed, machine, plan, (title or out_gcode.name) + "(拡大)",
                                info, warn_lines, window=(x0 - pad, y0 - pad, x1 + pad, y1 + pad))
        frame = frame_gcode(report, machine) if write_frame else None
    except Dxf2GcodeError:
        for p in (gcode_path, frame_path):
            if p is not None:
                _retire_stale(p)
        raise

    _write_atomic(svg_path, svg)
    detail_path = None
    if detail:
        detail_path = svg_path.with_name(svg_path.name.replace("_preview.svg", "_detail.svg"))
        _write_atomic(detail_path, detail)
    _write_atomic(report_path, report.text)
    if frame_path and frame:
        _write_atomic(frame_path, frame)
    _write_atomic(gcode_path, result.text)  # 最後に書く
    return Outcome(plan, result, report, gcode_path, svg_path, report_path, frame_path, warnings, detail_path)
