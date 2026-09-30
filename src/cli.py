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
from .preview import fmt_duration

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
    if a.mark_only:
        # 切らずに線を浅く刻印するだけ。手で切るときの案内(ケガキ線)にする
        as_layer = _resolve_layer(profile, a.mark_layer)
        if profile.layers[as_layer].through_cut:
            raise ConfigError(f"--mark-only の層 '{as_layer}' は through_cut: true です(貫通切断になってしまう)")
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
    if a.mark_only:
        meta["mode"] = f"MARK ONLY(切らずに線だけ刻印。層 '{as_layer}')"
    elif as_layer:
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
                min_gap=a.min_gap, cooldown=a.cooldown,
                meta=meta, read_warnings=warnings, title=f"{src.name}  [{profile.name}]",
                write_frame=a.frame)
    _print_outcome(o)
    return 0


def cmd_box(a: argparse.Namespace) -> int:
    """上面が開いた組み木の箱(ごみ箱・小物入れ)を生成して G-code まで作る。"""
    import ezdxf

    from .boxgen import build_butt_panels, build_panels, pack
    from .calibrate import text_polylines, text_width

    machine = load_machine(a.machine)
    profile = load_profile(a.profile)
    W, D, H = a.size
    T = a.thickness
    finger = a.finger or max(3 * T, 6.0)
    sheet_w, sheet_h = a.sheet if a.sheet else (machine.x_max, machine.y_max)
    if sheet_w > machine.x_max or sheet_h > machine.y_max:
        raise ConfigError(f"材料 {sheet_w:g}×{sheet_h:g}mm が加工エリア {machine.x_max:g}×{machine.y_max:g}mm より大きいです")
    panels = build_butt_panels(W, D, H, T) if a.joint == "butt" else build_panels(W, D, H, T, finger)
    cut_layer = "cut"
    if a.mark_only:
        cut_layer = _resolve_layer(profile, a.mark_layer)
        if profile.layers[cut_layer].through_cut:
            raise ConfigError(f"--mark-only の層 '{cut_layer}' は through_cut: true です")
    placed = pack(panels, sheet_w, sheet_h, gap=a.gap, margin=a.margin)
    sheets = sorted({p.sheet for p in placed})
    out = Path(a.output)
    out.parent.mkdir(parents=True, exist_ok=True)

    joint = "突き付け(長方形のみ)" if a.joint == "butt" else f"組み木(指の幅 目安 {finger:g} mm)"
    mode = " / 【切らずに線だけ刻印 = 手で切る用】" if a.mark_only else ""
    print(f"箱: 外寸 {W:g} × {D:g} × 高さ {H:g} mm / 板厚 {T:g} mm / {joint}{mode}")
    for pn in panels:
        print(f"  {pn.name:<6} {pn.width:g} × {pn.height:g} mm")
    for si in sheets:
        suffix = "" if len(sheets) == 1 else f"_{si + 1}"
        dxf_path = out.with_name(out.stem + suffix + ".dxf")
        gcode_path = out.with_name(out.stem + suffix + ".gcode")
        doc = ezdxf.new("R2010")
        doc.header["$INSUNITS"] = 4
        msp = doc.modelspace()
        names = []
        for pl in (p for p in placed if p.sheet == si):
            msp.add_lwpolyline(pl.poly(), close=True, dxfattribs={"layer": cut_layer})
            names.append(pl.panel.name)
            if a.label and pl.panel.name == "front":
                h = min(12.0, H * 0.25, 0.7 * W * 6.0 / (len(a.label) * 6.0 - 2.0))
                u = pl.x + (W - text_width(a.label, h)) / 2
                v = pl.y + (H + T) / 2 - h / 2
                for stroke in text_polylines(a.label, u, v, h):
                    msp.add_lwpolyline(stroke, dxfattribs={"layer": "engrave"})
        doc.saveas(dxf_path)
        res = read_dxf(dxf_path)
        meta = {"source": dxf_path.name, "profile": profile.name,
                "box": f"{W:g} x {D:g} x H{H:g} mm, T={T:g} mm, sheet {si + 1}/{len(sheets)}: {' '.join(names)}"}
        o = process(res.contours, profile, machine, gcode_path, meta=meta, read_warnings=res.warnings,
                    title=f"box {W:g}x{D:g}x{H:g} T{T:g} ({si + 1}/{len(sheets)}) [{profile.name}]",
                    write_frame=True, pass_mode=a.pass_mode, cooldown=a.cooldown)
        bb = o.report.bbox
        print()
        print(f"[材料 {si + 1}/{len(sheets)}] 板: {', '.join(names)}")
        print(f"  使用範囲 {bb[2] - bb[0]:.0f} × {bb[3] - bb[1]:.0f} mm / パス総数 {o.report.burns}"
              f" / 推定 {fmt_duration(o.report.time.total_s)}")
        print(f"  G-code: {o.gcode_path}")
        print(f"  拡大図: {o.detail_path}")
        print(f"  枠確認: {o.frame_path}")
        for w in o.warnings:
            print(f"  ! {w}")
    print()
    if a.joint == "butt":
        print("組み立て: 底の上に前後の板を立て、その間に左右の板を挟んで木工用ボンドで接着。")
    else:
        print("組み立て: 前後の板で左右を挟み、底をはめてから木工用ボンドで接着。")
    if a.mark_only:
        print("これは刻印だけのデータです。線に沿って手で切ってください(docs/MANUAL_CUT.md)。")
    print("板厚は必ずノギスで測った値を --thickness に入れること(表示とずれると指の深さが合わない)。")
    print(">>> SVG を目視確認してから流してください。必ず端材でテストしてから本番材を。<<<")
    return 0


def cmd_slice(a: argparse.Namespace) -> int:
    """STL を水平に輪切りにして、外形と穴の DXF を作る。"""
    from .stl_slice import describe, slice_shape, write_dxf

    shape = slice_shape(a.input, a.z, drop_holes_under=a.drop_holes_under or [],
                        add_holes=[tuple(h) for h in (a.add_hole or [])], scale=a.scale)
    out = Path(a.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    write_dxf(out, shape, layer=a.layer)
    print(f"STL {len(a.input)} 個を z={a.z:g} で輪切り(倍率 {a.scale:g})")
    for line in describe(shape):
        print(line)
    print(f"DXF: {out}")
    print("次: python dxf2gcode.py convert <この DXF> -p <プロファイル> --align lower-left")
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
    c.add_argument("--cooldown", type=float, default=0.0,
                   help="発振のたびにレーザー OFF で待つ秒数(熱がこもって炎が出るのを防ぐ。既定 0)")
    c.add_argument("--mark-only", action="store_true",
                   help="切らずに、切る線を浅く刻印するだけにする(手で切るときの案内線)")
    c.add_argument("--mark-layer", default="score", help="--mark-only で使う層(既定 score)")
    c.add_argument("--frame", action="store_true", help="使用範囲をレーザー OFF でなぞる G-code も出力")
    c.set_defaults(func=cmd_convert)

    b = sub.add_parser("box", help="上面が開いた組み木の箱(ごみ箱・小物入れ)を生成")
    b.add_argument("--size", type=float, nargs=3, required=True, metavar=("W", "D", "H"),
                   help="外寸 幅 奥行き 高さ [mm]")
    b.add_argument("--thickness", type=float, required=True,
                   help="板厚 [mm]。必ずノギスで測った実際の値を入れる")
    b.add_argument("-p", "--profile", default="mdf_5mm")
    b.add_argument("--finger", type=float, help="指の幅の目安 [mm](既定: 板厚の 3 倍、最小 6)")
    b.add_argument("--sheet", type=float, nargs=2, metavar=("W", "H"),
                   help="材料の大きさ [mm](既定: 加工エリア全体)。入りきらなければ複数枚に分ける")
    b.add_argument("--gap", type=float, default=4.0, help="板と板の間隔 [mm]")
    b.add_argument("--margin", type=float, default=5.0, help="材料の端からの余白 [mm]")
    b.add_argument("--label", default="", help="前の板の中央に刻印する文字(英大文字・数字)")
    b.add_argument("--pass-mode", choices=["path", "cycle"], default="path",
                   help="cycle = 全部の板を 1 周ずつ順に回る(炎が出にくい。箱の板には穴がないので順序の問題もない)")
    b.add_argument("--joint", choices=["finger", "butt"], default="finger",
                   help="finger = 組み木(レーザーで切る用)/ butt = 突き付け(ただの長方形。手で切る用)")
    b.add_argument("--cooldown", type=float, default=0.0, help="発振のたびにレーザー OFF で待つ秒数")
    b.add_argument("--mark-only", action="store_true",
                   help="切らずに、切る線を浅く刻印するだけにする(手で切るときの案内線)")
    b.add_argument("--mark-layer", default="score", help="--mark-only で使う層(既定 score)")
    b.add_argument("-o", "--output", default="out/box.gcode")
    b.set_defaults(func=cmd_box)

    sl = sub.add_parser("slice", help="STL を水平に輪切りにして外形と穴の DXF を作る(板状の部品用)")
    sl.add_argument("input", nargs="+", metavar="STL",
                    help="入力 STL(複数指定すると 1 つの形に合体する。分割された板を 1 枚にできる)")
    sl.add_argument("--z", type=float, required=True, help="輪切りにする高さ [mm](板の厚みの中ほど)")
    sl.add_argument("--scale", type=float, default=1.0, help="全体の倍率(既定 1。0.5 で半分の大きさ)")
    sl.add_argument("--drop-holes-under", nargs="+", metavar="STL",
                    help="この STL の真上/真下にある穴を取り除く(継ぎ板用の穴を消すとき)")
    sl.add_argument("--add-hole", type=float, nargs=3, action="append", metavar=("X", "Y", "D"),
                    help="円い穴を追加(STL の座標・縮小前の直径。複数指定可)")
    sl.add_argument("--layer", default="0", help="書き出す DXF のレイヤー名")
    sl.add_argument("-o", "--output", required=True, help="出力 DXF")
    sl.set_defaults(func=cmd_slice)

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
    generates = args.cmd in ("convert", "box") or getattr(args, "cal_cmd", None) in ("coupon", "kerf", "ladder")
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
