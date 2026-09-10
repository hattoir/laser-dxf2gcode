"""キャリブレーション用の治具データ生成と、測定値の設定ファイルへの書き戻し。

    calibrate coupon     … 20mm 角 ×4 を、それぞれ違う 出力/速度/パス回数 で切る
                           (各ピースに設定値を刻印。対応表もレポートに出す)
    calibrate kerf       … カーフ測定用の細片(50×10mm、カーフ補正なし)を切る
    calibrate apply-kerf … ノギスの測定値からカーフを逆算してプロファイルに書き戻す
    calibrate apply-cut  … 決まった切断条件(出力/速度/パス回数)を書き戻す

治具は一度 DXF として書き出し、それを通常の convert と同じ経路
(DXF 読み込み → 計画 → G-code → 検証 → プレビュー)で処理する。
キャリブレーションで確かめた経路が、そのまま本番で使う経路になる。
"""
from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from datetime import date
from pathlib import Path

import ezdxf
import yaml

from .config import MAX_KERF, LayerSettings, Profile, load_machine, load_profile, resolve_profile_path
from .dxf_reader import read_dxf
from .errors import ConfigError, Dxf2GcodeError
from .pipeline import process

# --------------------------------------------------------------------------
# 刻印用の簡易ストロークフォント(幅 4 × 高さ 6 の格子。Y は上向き)
# --------------------------------------------------------------------------

FONT: dict[str, list[list[tuple[float, float]]]] = {
    "0": [[(0, 0), (4, 0), (4, 6), (0, 6), (0, 0)], [(0, 0), (4, 6)]],
    "1": [[(1, 5), (2, 6), (2, 0)], [(1, 0), (3, 0)]],
    "2": [[(0, 6), (4, 6), (4, 3), (0, 3), (0, 0), (4, 0)]],
    "3": [[(0, 6), (4, 6), (4, 0), (0, 0)], [(0, 3), (4, 3)]],
    "4": [[(0, 6), (0, 3), (4, 3)], [(4, 6), (4, 0)]],
    "5": [[(4, 6), (0, 6), (0, 3), (4, 3), (4, 0), (0, 0)]],
    "6": [[(4, 6), (0, 6), (0, 0), (4, 0), (4, 3), (0, 3)]],
    "7": [[(0, 6), (4, 6), (1, 0)]],
    "8": [[(0, 0), (4, 0), (4, 6), (0, 6), (0, 0)], [(0, 3), (4, 3)]],
    "9": [[(4, 3), (0, 3), (0, 6), (4, 6), (4, 0), (0, 0)]],
    "A": [[(0, 0), (0, 4), (2, 6), (4, 4), (4, 0)], [(0, 3), (4, 3)]],
    "B": [[(0, 0), (0, 6), (3, 6), (4, 5), (4, 4), (3, 3), (0, 3)], [(3, 3), (4, 2), (4, 1), (3, 0), (0, 0)]],
    "C": [[(4, 6), (0, 6), (0, 0), (4, 0)]],
    "D": [[(0, 0), (0, 6), (2, 6), (4, 4), (4, 2), (2, 0), (0, 0)]],
    "E": [[(4, 6), (0, 6), (0, 0), (4, 0)], [(0, 3), (3, 3)]],
    "F": [[(4, 6), (0, 6), (0, 0)], [(0, 3), (3, 3)]],
    "K": [[(0, 0), (0, 6)], [(4, 6), (0, 3), (4, 0)]],
    "N": [[(0, 0), (0, 6), (4, 0), (4, 6)]],
    "P": [[(0, 0), (0, 6), (4, 6), (4, 3), (0, 3)]],
    ".": [[(1.5, 0), (2.5, 0)]],
    " ": [],
}
ADVANCE = 6.0  # 文字送り(格子単位)


def text_width(s: str, height: float) -> float:
    return (len(s) * ADVANCE - 2.0) * height / 6.0 if s else 0.0


def text_polylines(s: str, x: float, y: float, height: float) -> list[list[tuple[float, float]]]:
    k = height / 6.0
    out = []
    for i, ch in enumerate(s.upper()):
        if ch not in FONT:
            raise Dxf2GcodeError(f"刻印フォントに無い文字: {ch!r}")
        for stroke in FONT[ch]:
            out.append([(x + (i * ADVANCE + px) * k, y + py * k) for px, py in stroke])
    return out


# --------------------------------------------------------------------------
# テストピース(クーポン)
# --------------------------------------------------------------------------

@dataclass
class Variant:
    label: str
    power: float
    feed: float
    passes: int


def parse_variants(spec: str) -> list[Variant]:
    """'100:250:4,100:300:4' → [Variant(A,...), Variant(B,...)](出力%:速度:回数)"""
    out = []
    for i, item in enumerate(s for s in spec.split(",") if s.strip()):
        try:
            p, f, n = item.split(":")
            out.append(Variant("ABCDEFGH"[i], float(p), float(f), int(n)))
        except (ValueError, IndexError) as exc:
            raise Dxf2GcodeError(f"--variants の書式は 出力:速度:回数 をカンマ区切り(最大 8 個): {item!r}") from exc
    if not out:
        raise Dxf2GcodeError("--variants が空です")
    return out


def default_variants(base: LayerSettings) -> list[Variant]:
    """プロファイルの cut 設定を基準に、エネルギーが同じか少ない側の 4 条件。
    (いきなり強い条件を試さない。基準で切れなければ --variants で上げていく)"""
    return [
        Variant("A", base.power, base.feed, base.passes),                          # 基準
        Variant("B", base.power, round(base.feed * 1.25), base.passes),            # 速く
        Variant("C", base.power, base.feed, max(1, base.passes - 1)),              # 回数を減らす
        Variant("D", round(base.power * 0.8), base.feed, base.passes),             # 弱く
    ]


def _variant_lines(v: Variant) -> list[str]:
    return [f"P{v.power:g}", f"F{v.feed:g}", f"N{v.passes}"]


def write_coupon_dxf(path: Path, variants: list[Variant], origin: tuple[float, float],
                     size: float = 20.0, gap: float = 6.0, engrave: bool = True) -> None:
    doc = ezdxf.new("R2010")
    doc.header["$INSUNITS"] = 4
    msp = doc.modelspace()
    ox, oy = origin
    for i, v in enumerate(variants):
        x0 = ox + i * (size + gap)
        layer = f"cut_{v.label}"
        doc.layers.add(layer)
        msp.add_lwpolyline([(x0, oy), (x0 + size, oy), (x0 + size, oy + size), (x0, oy + size)],
                           close=True, dxfattribs={"layer": layer})
        if not engrave:
            continue
        # 大きな記号(左)と 設定値 3 行(右)
        for pl in text_polylines(v.label, x0 + 2.0, oy + size / 2 - 3.0, 6.0):
            msp.add_lwpolyline(pl, dxfattribs={"layer": "engrave"})
        lines = _variant_lines(v)
        avail = size - 8.0 - 1.2
        h = min(2.2, 6.0 * avail / max(len(s) * ADVANCE - 2 for s in lines))
        for j, s in enumerate(lines):
            y = oy + size - 3.0 - h - j * (h + 2.2)
            for pl in text_polylines(s, x0 + 8.0, y, h):
                msp.add_lwpolyline(pl, dxfattribs={"layer": "engrave"})
    doc.saveas(path)


def coupon_profile(base: Profile, variants: list[Variant]) -> Profile:
    layers = {f"cut_{v.label}": LayerSettings(f"cut_{v.label}", power=v.power, feed=v.feed, passes=v.passes,
                                              through_cut=True, kerf_compensation=True, order=10,
                                              air_assist=True)
              for v in variants}
    eng = base.layers.get("engrave") or LayerSettings("engrave", 30, 3000, 1, False, False, 0, False)
    layers["engrave"] = eng
    prof = Profile(f"{base.name}-coupon", base.kerf, None, layers)
    prof.validate()
    return prof


def cmd_coupon(a: argparse.Namespace) -> int:
    machine = load_machine(a.machine)
    base = load_profile(a.profile)
    cut = base.layers.get(a.base_layer)
    if cut is None:
        raise ConfigError(f"プロファイル {base.name} に層 '{a.base_layer}' がありません")
    variants = parse_variants(a.variants) if a.variants else default_variants(cut)
    out = Path(a.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    dxf_path = out.with_suffix(".dxf")
    write_coupon_dxf(dxf_path, variants, tuple(a.origin), a.size, engrave=not a.no_engrave)
    res = read_dxf(dxf_path)
    prof = coupon_profile(base, variants)
    table = ["テストピース対応表(左から順に並ぶ。刻印: 記号 / P=出力% F=速度mm/min N=パス回数)",
             f"  カーフ補正: {base.kerf:.3f} mm で補正済み → 正しければ {a.size:g} mm 角に仕上がる"]
    for v in variants:
        table.append(f"  {v.label}: 出力 {v.power:g}%  速度 {v.feed:g} mm/min  {v.passes} パス")
    meta = {"source": dxf_path.name, "profile": f"{base.name} (coupon)", "coupon kerf": f"{base.kerf:.3f}"}
    o = process(res.contours, prof, machine, out, meta=meta, read_warnings=res.warnings,
                title=f"calibration coupon [{base.name}]")
    with open(o.report_path, "a", encoding="utf-8") as f:
        f.write("\n".join(table) + "\n")
    from .cli import _print_outcome
    _print_outcome(o)
    print("\n".join(table))
    print(f"\n  DXF: {dxf_path}")
    print("  測定後: 切り抜けた中で最も弱い(速い)条件を選び `calibrate apply-cut` で書き戻す。")
    print(f"  ピースの寸法 S をノギスで測ったら: calibrate apply-kerf -p {base.name} --square S "
          f"--coupon-kerf {base.kerf:.3f}")
    return 0


# --------------------------------------------------------------------------
# カーフ測定用の細片
# --------------------------------------------------------------------------

def write_kerf_dxf(path: Path, origin: tuple[float, float], length: float, width: float, count: int) -> None:
    doc = ezdxf.new("R2010")
    doc.header["$INSUNITS"] = 4
    doc.layers.add("kerf_test")
    msp = doc.modelspace()
    ox, oy = origin
    for i in range(count):
        y0 = oy + i * (width + 5.0)
        msp.add_lwpolyline([(ox, y0), (ox + length, y0), (ox + length, y0 + width), (ox, y0 + width)],
                           close=True, dxfattribs={"layer": "kerf_test"})
    doc.saveas(path)


def cmd_kerf(a: argparse.Namespace) -> int:
    machine = load_machine(a.machine)
    base = load_profile(a.profile)
    cut = base.layers.get(a.base_layer)
    if cut is None:
        raise ConfigError(f"プロファイル {base.name} に層 '{a.base_layer}' がありません")
    out = Path(a.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    dxf_path = out.with_suffix(".dxf")
    write_kerf_dxf(dxf_path, tuple(a.origin), a.length, a.width, a.count)
    res = read_dxf(dxf_path)
    layer = LayerSettings("kerf_test", power=cut.power, feed=cut.feed, passes=cut.passes,
                          per_pass=list(cut.per_pass), through_cut=True, kerf_compensation=False,
                          air_assist=cut.air_assist)
    prof = Profile(f"{base.name}-kerf", 0.0, None, {"kerf_test": layer})
    meta = {"source": dxf_path.name, "profile": f"{base.name} (kerf test, NO kerf compensation)"}
    o = process(res.contours, prof, machine, out, meta=meta, read_warnings=res.warnings,
                title=f"kerf test {a.length:g}x{a.width:g} [{base.name}]")
    from .cli import _print_outcome
    _print_outcome(o)
    print(f"\n細片 {a.count} 本({a.length:g} × {a.width:g} mm、カーフ補正なし)")
    print("  細片の幅 w を数か所ノギスで測り、平均を取る。カーフ k = 設計幅 - w")
    print(f"  書き戻し: calibrate apply-kerf -p {base.name} --strip-width w --nominal {a.width:g}")
    return 0


# --------------------------------------------------------------------------
# 設定ファイルへの書き戻し(コメントを保ったまま 1 行だけ書き換える)
# --------------------------------------------------------------------------

_KEYLINE = re.compile(r"^(?P<indent>\s*)(?P<key>[^\s#:][^:#]*?)\s*:(?P<rest>.*)$")


def set_yaml_value(text: str, keypath: list[str], value: str, comment: str) -> str:
    """keypath(例: ['mdf_5mm', 'layers', 'cut', 'feed'])の行の値とコメントを書き換える。

    PyYAML で読み書きするとコメント(実測値/仮定値の注記)が消えるため、
    インデントを見ながら該当行だけを置き換える。
    """
    lines = text.splitlines(keepends=True)
    stack: list[tuple[int, str]] = []
    for i, line in enumerate(lines):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        m = _KEYLINE.match(line.rstrip("\r\n"))
        if not m:
            continue
        indent = len(m.group("indent"))
        while stack and stack[-1][0] >= indent:
            stack.pop()
        stack.append((indent, m.group("key")))
        if [k for _, k in stack] == keypath:
            rest = m.group("rest")
            col = line.find("#", len(m.group("indent")) + len(m.group("key")) + 1 + len(rest.split("#")[0]))
            head = f"{m.group('indent')}{m.group('key')}: {value}"
            pad = max(1, (col if "#" in rest else 0) - len(head))
            eol = "\n" if line.endswith("\n") else ""
            lines[i] = f"{head}{' ' * pad}# {comment}{eol}"
            return "".join(lines)
    raise ConfigError(f"設定ファイルに {'.'.join(keypath)} が見つかりません")


def _get(d: dict, keypath: list[str]):
    for k in keypath:
        d = d[k]
    return d


def update_profile_file(path: Path, updates: list[tuple[list[str], object, str]], dry_run: bool = False) -> str:
    text = path.read_text(encoding="utf-8")
    new = text
    for keypath, value, comment in updates:
        new = set_yaml_value(new, keypath, f"{value}", comment)
    data = yaml.safe_load(new)  # 書き換え後も正しく読めて、値が入っていることを確認
    for keypath, value, _ in updates:
        got = _get(data, keypath)
        if float(got) != float(value):
            raise ConfigError(f"書き戻しの検証に失敗: {'.'.join(keypath)} = {got}(期待値 {value})")
    if not dry_run:
        tmp = path.with_name(path.name + ".tmp")
        tmp.write_text(new, encoding="utf-8", newline="")
        tmp.replace(path)
    return new


def kerf_from_measurement(a: argparse.Namespace, current: float) -> tuple[float, str]:
    if a.kerf is not None:
        return a.kerf, "直接指定"
    if a.strip_width is not None:
        nominal = a.nominal if a.nominal is not None else 10.0
        return nominal - a.strip_width, f"細片幅 {a.strip_width:g}mm(設計 {nominal:g}mm)から逆算"
    if a.square is not None:
        nominal = a.nominal if a.nominal is not None else 20.0
        k0 = a.coupon_kerf if a.coupon_kerf is not None else current
        # 補正 k0 で切ったピースの寸法 S = nominal + k0 - k  →  k = nominal + k0 - S
        return nominal + k0 - a.square, f"テストピース {a.square:g}mm(設計 {nominal:g}mm, 補正 {k0:g}mm)から逆算"
    raise Dxf2GcodeError("--strip-width / --square / --kerf のいずれかを指定してください")


def cmd_apply_kerf(a: argparse.Namespace) -> int:
    prof = load_profile(a.profile)
    kerf, how = kerf_from_measurement(a, prof.kerf)
    kerf = round(kerf, 3)
    if not (0.0 < kerf <= MAX_KERF):
        raise ConfigError(f"逆算したカーフ {kerf:.3f}mm は不自然です(0〜{MAX_KERF}mm)。測定をやり直してください")
    path = resolve_profile_path(a.profile)
    comment = f"【実測値】{date.today().isoformat()} {how}"
    update_profile_file(path, [([prof.name, "kerf"], f"{kerf:.3f}", comment)], a.dry_run)
    print(f"kerf: {prof.kerf:.3f} → {kerf:.3f} mm  ({how})")
    if abs(kerf - prof.kerf) > 0.15:
        print("  注意: 以前の値から 0.15mm 以上変わりました。測定値を再確認してください")
    print("  (dry-run: ファイルは変更していません)" if a.dry_run else f"  更新: {path}")
    return 0


def cmd_apply_cut(a: argparse.Namespace) -> int:
    prof = load_profile(a.profile)
    if a.layer not in prof.layers:
        raise ConfigError(f"プロファイル {prof.name} に層 '{a.layer}' がありません")
    stamp = f"【実測値】{date.today().isoformat()}" + (f" {a.note}" if a.note else "")
    updates = []
    for key, val, unit in (("power", a.power, "[%]"), ("feed", a.feed, "[mm/min]"), ("passes", a.passes, "")):
        if val is not None:
            if key == "passes":
                val = int(val)
            updates.append(([prof.name, "layers", a.layer, key], f"{val:g}", f"{stamp} {unit}".rstrip()))
    if not updates:
        raise Dxf2GcodeError("--power / --feed / --passes のいずれかを指定してください")
    path = resolve_profile_path(a.profile)
    update_profile_file(path, updates, a.dry_run)
    load_profile(a.profile) if not a.dry_run else None  # 検証(範囲チェック)
    for kp, v, _ in updates:
        print(f"{'.'.join(kp[2:])} = {v}")
    print("  (dry-run: ファイルは変更していません)" if a.dry_run else f"  更新: {path}")
    return 0


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def add_calibrate_parser(sub) -> None:
    cal = sub.add_parser("calibrate", help="キャリブレーション(テストピース / カーフ測定 / 書き戻し)")
    cs = cal.add_subparsers(dest="cal_cmd", required=True)

    c = cs.add_parser("coupon", help="20mm 角 ×4 を条件を変えて切るテストピース")
    c.add_argument("-p", "--profile", default="mdf_5mm")
    c.add_argument("--base-layer", default="cut", help="基準にする層(既定 cut)")
    c.add_argument("--variants", help="出力%%:速度:回数 をカンマ区切り。例 100:250:4,100:300:4,90:250:4,100:250:3")
    c.add_argument("--origin", type=float, nargs=2, default=[10.0, 10.0], metavar=("X", "Y"))
    c.add_argument("--size", type=float, default=20.0, help="ピースの一辺 [mm]")
    c.add_argument("--no-engrave", action="store_true", help="設定値を刻印しない")
    c.add_argument("-o", "--output", default="out/coupon.gcode")
    c.set_defaults(func=cmd_coupon)

    k = cs.add_parser("kerf", help="カーフ測定用の細片(補正なし)")
    k.add_argument("-p", "--profile", default="mdf_5mm")
    k.add_argument("--base-layer", default="cut")
    k.add_argument("--length", type=float, default=50.0)
    k.add_argument("--width", type=float, default=10.0)
    k.add_argument("--count", type=int, default=1, help="細片の本数(複数本測って平均すると精度が上がる)")
    k.add_argument("--origin", type=float, nargs=2, default=[10.0, 10.0], metavar=("X", "Y"))
    k.add_argument("-o", "--output", default="out/kerf_test.gcode")
    k.set_defaults(func=cmd_kerf)

    ak = cs.add_parser("apply-kerf", help="測定値からカーフを逆算してプロファイルに書き戻す")
    ak.add_argument("-p", "--profile", default="mdf_5mm")
    g = ak.add_mutually_exclusive_group(required=True)
    g.add_argument("--strip-width", type=float, help="kerf コマンドの細片の実測幅 [mm]")
    g.add_argument("--square", type=float, help="coupon のピースの実測寸法 [mm]")
    g.add_argument("--kerf", type=float, help="カーフを直接指定 [mm]")
    ak.add_argument("--nominal", type=float, help="設計寸法(既定: 細片 10 / ピース 20)")
    ak.add_argument("--coupon-kerf", type=float, help="coupon を切ったときのカーフ補正値(既定: 現在のプロファイル値)")
    ak.add_argument("--dry-run", action="store_true")
    ak.set_defaults(func=cmd_apply_kerf)

    ac = cs.add_parser("apply-cut", help="決まった切断条件をプロファイルに書き戻す")
    ac.add_argument("-p", "--profile", default="mdf_5mm")
    ac.add_argument("--layer", default="cut")
    ac.add_argument("--power", type=float)
    ac.add_argument("--feed", type=float)
    ac.add_argument("--passes", type=int)
    ac.add_argument("--note", default="", help="コメントに残すメモ(例: 'coupon B')")
    ac.add_argument("--dry-run", action="store_true")
    ac.set_defaults(func=cmd_apply_cut)
