"""auto-trash-navigator の主板・底板を、1 枚の MDF から切る 1/2 スケール版として生成する。

    python jobs/atn_half_scale/make.py            (既定: 倍率 0.5)
    python jobs/atn_half_scale/make.py 0.7        (倍率を変える)

元データは 3D プリント用に 4 分割された STL(../auto-trash-navigator/ATN_print/_individual_parts)。
読むだけで、そちらのプロジェクトは変更しない。

やっていること
    1. 4 分割の STL を輪切りにして 1 枚の外形に合体(src/stl_slice.py)
    2. 継ぎ板(splice_*)用の穴を取り除く(1 枚板なので不要)
    3. 主板に、柱の位置の穴を 8 個追加する
       (3D プリント版は主板と柱が一体。板 1 枚では柱を作れないので、
        スペーサーとネジで底板とつなげるようにする。位置と直径は底板の柱用の穴と同じ)
    4. 各板について「レーザー切断用」と「線だけ刻印して手で切る用」の G-code を、
       2.5mm 用と 6mm 用のプロファイルそれぞれで出力する
"""
import contextlib
import io
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(ROOT))

from src.cli import main  # noqa: E402
from src.stl_slice import describe, slice_shape, write_dxf  # noqa: E402

PARTS = ROOT.parent / "auto-trash-navigator" / "ATN_print" / "_individual_parts"
QUADS = ("FL", "FR", "RL", "RR")
MAIN_Z = 2.5       # 主板の平らな部分は z = 0〜5
BOTTOM_Z = -44.5   # 底板は z = -46〜-43
PROFILES = ("mdf_2_5mm", "mdf_6mm")


def run(args):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        code = main(args)
    return code, buf.getvalue()


def build(scale: float) -> None:
    if not PARTS.exists():
        sys.exit(f"STL が見つかりません: {PARTS}")
    out_dir = ROOT / "out" / "atn_half_scale"
    out_dir.mkdir(parents=True, exist_ok=True)

    # 底板(柱用の穴の位置を知るため、まず等倍で)
    bottom_files = [PARTS / f"bottom_plate_{q}.stl" for q in QUADS]
    splice_bot = sorted(PARTS.glob("splice_bot_*.stl"))
    full = slice_shape(bottom_files, BOTTOM_Z, drop_holes_under=splice_bot)
    # slice_shape は左下を原点にそろえるので、元の座標(中心原点、400mm 角)に戻す
    x_off = y_off = -200.0
    posts = [(h.circle[0] + x_off, h.circle[1] + y_off, h.circle[2]) for h in full.holes if h.circle]

    bottom = slice_shape(bottom_files, BOTTOM_Z, drop_holes_under=splice_bot, scale=scale)
    main_plate = slice_shape([PARTS / f"main_plate_{q}.stl" for q in QUADS], MAIN_Z,
                             drop_holes_under=sorted(PARTS.glob("splice_main_*.stl")),
                             add_holes=posts, scale=scale)

    shapes = {"main_plate": main_plate, "bottom_plate": bottom}
    for name, shape in shapes.items():
        write_dxf(HERE / f"{name}.dxf", shape)
        print(f"== {name}(倍率 {scale:g})→ jobs/atn_half_scale/{name}.dxf")
        for line in describe(shape):
            print("   " + line)

    print()
    print(f"{'ファイル':<40}{'部品/穴':>8}{'時間':>10}  警告")
    for name in shapes:
        dxf = str(HERE / f"{name}.dxf")
        for prof in PROFILES:
            tag = prof.replace("mdf_", "")
            jobs = {
                "cut": ["convert", dxf, "-p", prof, "--align", "lower-left", "--margin", "5",
                        "--pass-mode", "cycle", "--cooldown", "2", "--frame",
                        "-o", str(out_dir / f"{name}_{tag}_cut.gcode")],
                "mark": ["convert", dxf, "-p", prof, "--align", "lower-left", "--margin", "5",
                         "--mark-only", "--frame", "-o", str(out_dir / f"{name}_{tag}_mark.gcode")],
            }
            for kind, args in jobs.items():
                code, text = run(args)
                fname = f"{name}_{tag}_{kind}.gcode"
                if code != 0:
                    print(f"{fname:<40}  失敗(exit {code})")
                    print(text)
                    continue
                report = (out_dir / f"{name}_{tag}_{kind}_report.txt").read_text(encoding="utf-8")
                get = lambda key: next((l.split(":", 1)[1].strip() for l in report.splitlines() if l.startswith(key)), "?")
                warns = [l.strip()[2:] for l in report.splitlines() if l.strip().startswith("! ") and "プロファイルに無い" not in l]
                t = get("推定所要時間").split("(")[0]
                print(f"{fname:<40}{get('部品数(外形)') + '/' + get('穴の数'):>8}{t:>10}  {len(warns)} 件")
                for w in warns[:3]:
                    print("      ! " + w[:110])


if __name__ == "__main__":
    build(float(sys.argv[1]) if len(sys.argv) > 1 else 0.5)
