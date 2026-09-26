"""コマンドライン。

    python dxf2gcode.py convert part.dxf -p mdf_5mm
    python dxf2gcode.py convert 外形.dxf 穴.dxf -p mdf_5mm     (複数スケッチをまとめる)
    python dxf2gcode.py calibrate coupon -p mdf_5mm
    python dxf2gcode.py calibrate kerf -p mdf_5mm
    python dxf2gcode.py calibrate apply-kerf -p mdf_5mm --strip-width 9.82

終了コード: 0 成功 / 2 加工エリア外 / 3 その他の安全・入力エラー
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .config import Profile, load_machine, load_profile
from .dxf_reader import DEFAULT_CHORD_TOL, DEFAULT_JOIN_TOL, Contour, read_dxf
from .errors import ConfigError, Dxf2GcodeError, WorkAreaError
from .pipeline import Outcome, process, transform_contours

EXIT_AREA = 2
EXIT_ERROR = 3


def _print_outcome(o: Outcome) -> None:
    print(o.report.text)
    print("出力:")
    print(f"  G-code   : {o.gcode_path}")
    print(f"  プレビュー: {o.svg_path}(加工エリア全体での位置)")
    if o.detail_path:
        print(f"  拡大図   : {o.detail_path}(切断順の確認)")
    print(f"  レポート : {o.report_path}")
    if o.frame_path:
        print(f"  枠確認用 : {o.frame_path}(レーザー OFF で使用範囲の外周をなぞる)")
    print()
    print(">>> SVG プレビューを目視確認してから流してください。必ず端材でテストしてから本番材を。<<<")


def _resolve_layer(profile: Profile, name: str) -> str:
    """プロファイルにある層の正式な名前を返す(大文字小文字は区別しない)。"""
    for k in profile.layers:
        if k.lower() == name.lower():
            return k
    raise ConfigError(f"プロファイル '{profile.name}' に層 '{name}' がありません"
                      f"(ある層: {', '.join(profile.layers)})")


def _read_labeled(path: Path, layer: str | None, a: argparse.Namespace) -> tuple[list[Contour], list[str]]:
    """DXF を読み、layer を指定したらすべての輪郭をその層に付け替える。"""
    res = read_dxf(path, chord_tol=a.chord_tol, join_tol=a.join_tol, assume_mm=a.assume_mm)
    if not res.contours:
        raise Dxf2GcodeError(f"{path} に加工できる図形がありません")
    warnings = [f"{path.name}: {w}" for w in res.warnings]
    # DXF のハンドルはファイルごとに振り直されるため、出どころにファイル名を付ける
    return ([Contour(c.points, c.closed, layer or c.layer, f"{path.name}:{c.source}")
             for c in res.contours], warnings)


def cmd_convert(a: argparse.Namespace) -> int:
    machine = load_machine(a.machine)
    profile = load_profile(a.profile)
    src = Path(a.input[0])

    as_layer = _resolve_layer(profile, a.as_layer) if a.as_layer else None
    contours: list[Contour] = []
    warnings: list[str] = []
    for path in a.input:
        cs, w = _read_labeled(Path(path), as_layer, a)
        contours += cs
        warnings += w
    names = [Path(x).name for x in a.input]
    # Fusion 360 はスケッチ 1 つにつき DXF 1 ファイルなので、外形と穴が別ファイルになる。
    # まとめて読み込めば、入れ子判定も切断順序もファイルの区別なく効く。
    meta = {"source": names[0] if len(names) == 1 else f"{len(names)} files: {', '.join(names)}",
            "profile": profile.name}
    if as_layer:
        meta["source layer"] = f"全体を '{as_layer}' として加工"

    if a.engrave:
        eng_layer = _resolve_layer(profile, a.engrave_layer)
        if profile.layers[eng_layer].through_cut:
            warnings.append(f"--engrave で指定した層 '{eng_layer}' は through_cut: true です。"
                            "刻印ではなく貫通切断になります")
        names = []
        for path in a.engrave:
            cs, w = _read_labeled(Path(path), eng_layer, a)
            contours += cs
            warnings += w
            names.append(Path(path).name)
        meta["engrave"] = f"{', '.join(names)} → 層 '{eng_layer}'"

    # 位置合わせは全ファイルまとめて行う(刻印と切断の相対位置を崩さないため)
    contours, (dx, dy) = transform_contours(contours, a.align, a.margin, tuple(a.offset))
    if not a.return_home:
        machine.return_home = False
    out = Path(a.output) if a.output else Path("out") / (src.stem + ".gcode")
    if dx or dy:
        meta["placement"] = f"moved by ({dx:+.3f}, {dy:+.3f}) mm"
    if a.lead_in:
        meta["lead-in"] = f"{a.lead_in:g} mm"
    o = process(contours, profile, machine, out, kerf=a.kerf, lead_in=a.lead_in, pass_mode=a.pass_mode,
                min_gap=a.min_gap,
                meta=meta, read_warnings=warnings, title=f"{src.name}  [{profile.name}]",
                write_frame=a.frame)
    _print_outcome(o)
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="dxf2gcode", description="Fusion 360 DXF → GRBL G-code(Creality Falcon2)")
    ap.add_argument("-m", "--machine", default=None, help="機械設定 YAML(既定: machine.yaml)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("convert", help="DXF を G-code に変換(+ SVG プレビュー + レポート)")
    c.add_argument("input", nargs="+", metavar="DXF",
                   help="入力 DXF(複数指定可。Fusion 360 はスケッチごとに別ファイルになるため)")
    c.add_argument("-p", "--profile", default="mdf_5mm", help="材料プロファイル名 または YAML パス")
    c.add_argument("-o", "--output", help="出力 G-code(既定: out/<入力名>.gcode)")
    c.add_argument("--kerf", type=float, default=None, help="カーフ幅 [mm](プロファイルの値を上書き)")
    c.add_argument("--lead-in", type=float, default=0.0, help="外形のリードイン長さ [mm](既定 0 = なし)")
    c.add_argument("--chord-tol", type=float, default=DEFAULT_CHORD_TOL, help="曲線近似の弦の許容誤差 [mm]")
    c.add_argument("--join-tol", type=float, default=DEFAULT_JOIN_TOL, help="端点を繋ぐ許容誤差 [mm]")
    c.add_argument("--assume-mm", action="store_true", help="$INSUNITS が mm でなくても mm とみなす")
    c.add_argument("--engrave", action="append", metavar="DXF",
                   help="刻印用の DXF(複数指定可)。中身をすべて --engrave-layer の層として、切断より先に加工する")
    c.add_argument("--engrave-layer", default="engrave", help="--engrave で使う層の名前(既定 engrave)")
    c.add_argument("--as-layer", metavar="LAYER",
                   help="入力 DXF のレイヤー名を無視し、全体をこの層として加工する")
    c.add_argument("--align", choices=["none", "lower-left"], default="none",
                   help="配置: lower-left = 図形の左下を (margin, margin) へ移動")
    c.add_argument("--margin", type=float, default=5.0, help="--align lower-left の余白 [mm]")
    c.add_argument("--offset", type=float, nargs=2, default=[0.0, 0.0], metavar=("DX", "DY"),
                   help="全体を平行移動 [mm]")
    c.add_argument("--pass-mode", choices=["path", "cycle"], default="path",
                   help="path = 輪郭ごとに回数分続けて切る(既定)/ cycle = 全体を 1 周ずつ繰り返す")
    c.add_argument("--no-return-home", dest="return_home", action="store_false", help="末尾の原点復帰をしない")
    c.add_argument("--min-gap", type=float, default=0.5,
                   help="切断線どうしがこの距離 [mm] より近ければ警告(既定 0.5、0 で無効)")
    c.add_argument("--frame", action="store_true", help="使用範囲をレーザー OFF でなぞる G-code も出力")
    c.set_defaults(func=cmd_convert)

    from .calibrate import add_calibrate_parser
    add_calibrate_parser(sub)
    return ap


def main(argv: list[str] | None = None) -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except AttributeError:
            pass
    args = build_parser().parse_args(argv)
    generates = args.cmd == "convert" or getattr(args, "cal_cmd", None) in ("coupon", "kerf")
    try:
        return args.func(args)
    except WorkAreaError as exc:
        print(f"\n[中止] {exc}", file=sys.stderr)
        print("G-code は出力していません。", file=sys.stderr)
        return EXIT_AREA
    except Dxf2GcodeError as exc:
        print(f"\n[中止] {exc}", file=sys.stderr)
        if generates:
            print("G-code は出力していません。", file=sys.stderr)
        return EXIT_ERROR
