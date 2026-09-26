"""CLI の通しテスト(サンプル DXF を自作して変換する)。"""
import ezdxf
import pytest

from src.cli import EXIT_AREA, EXIT_ERROR, main
from src.gcode import strip_comment


def _dxf(path, shapes, units=4):
    doc = ezdxf.new("R2010")
    doc.header["$INSUNITS"] = units
    msp = doc.modelspace()
    for kind, *args in shapes:
        if kind == "rect":
            x0, y0, x1, y1 = args
            msp.add_lwpolyline([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], close=True)
        elif kind == "circle":
            msp.add_circle(args[0], args[1])
    doc.saveas(path)
    return path


def test_convert_writes_gcode_svg_report(tmp_path, capsys):
    src = _dxf(tmp_path / "part.dxf", [("rect", 20, 20, 80, 60), ("circle", (50, 40), 2.1)])
    out = tmp_path / "part.gcode"
    assert main(["convert", str(src), "-o", str(out)]) == 0
    assert out.exists()
    assert (tmp_path / "part_preview.svg").exists()
    assert (tmp_path / "part_report.txt").exists()
    cmds = [c for c in (strip_comment(l) for l in out.read_text().splitlines()) if c]
    assert cmds[0] == "G21 G90 G94" and cmds[-1] == "M5"
    assert "目視確認" in capsys.readouterr().out


def test_convert_out_of_area_exits_without_gcode(tmp_path, capsys):
    src = _dxf(tmp_path / "big.dxf", [("rect", -10, 10, 50, 50)])
    out = tmp_path / "big.gcode"
    assert main(["convert", str(src), "-o", str(out)]) == EXIT_AREA
    assert not out.exists()
    err = capsys.readouterr().err
    assert "G-code は出力しません" in err and "X=-" in err


def test_failed_run_retires_stale_gcode(tmp_path):
    out = tmp_path / "p.gcode"
    ok = _dxf(tmp_path / "ok.dxf", [("rect", 20, 20, 80, 60)])
    assert main(["convert", str(ok), "-o", str(out)]) == 0
    bad = _dxf(tmp_path / "bad.dxf", [("rect", 380, 20, 420, 60)])
    assert main(["convert", str(bad), "-o", str(out)]) == EXIT_AREA
    assert not out.exists()                                  # 古い G-code を誤って流さない
    assert (tmp_path / "p.gcode.stale").exists()


def test_align_lower_left_fixes_negative_origin(tmp_path):
    src = _dxf(tmp_path / "c.dxf", [("rect", -40, -25, 40, 25), ("circle", (0, 0), 5)])
    out = tmp_path / "c.gcode"
    assert main(["convert", str(src), "-o", str(out), "--align", "lower-left", "--margin", "5"]) == 0


def test_inch_dxf_is_rejected(tmp_path):
    src = _dxf(tmp_path / "in.dxf", [("rect", 1, 1, 2, 2)], units=1)
    out = tmp_path / "in.gcode"
    assert main(["convert", str(src), "-o", str(out)]) == EXIT_ERROR
    assert not out.exists()


def test_calibrate_coupon_and_kerf(tmp_path):
    assert main(["calibrate", "coupon", "-o", str(tmp_path / "coupon.gcode")]) == 0
    assert main(["calibrate", "kerf", "--count", "2", "-o", str(tmp_path / "kerf.gcode")]) == 0
    text = (tmp_path / "coupon.gcode").read_text()
    # 刻印(mark)がすべての貫通切断より先
    roles = [l.split("role=")[1].split()[0] for l in text.splitlines() if "role=" in l]
    first_cut = next(i for i, r in enumerate(roles) if r != "mark")
    assert all(r != "mark" for r in roles[first_cut:])


def test_engrave_file_is_engraved_before_cutting(tmp_path):
    cut = _dxf(tmp_path / "cut.dxf", [("rect", 20, 20, 80, 60)])
    marks = _dxf(tmp_path / "marks.dxf", [("circle", (50, 40), 5)])
    out = tmp_path / "j.gcode"
    assert main(["convert", str(cut), "--engrave", str(marks), "-o", str(out)]) == 0
    text = out.read_text()
    roles = [l.split("role=")[1].split()[0] for l in text.splitlines() if "role=" in l]
    layers = [l.split("layer=")[1].split()[0] for l in text.splitlines() if "layer=" in l]
    assert roles[0] == "mark" and layers[0] == "engrave"
    assert set(roles[1:]) == {"outer"}      # 刻印の円は穴として切られない
    assert "hole" not in roles


def test_engrave_keeps_relative_position_after_align(tmp_path):
    from src.gcode import parse_gcode
    from src.preview import extract_burns
    cut = _dxf(tmp_path / "cut.dxf", [("rect", -40, -25, 40, 25)])
    marks = _dxf(tmp_path / "marks.dxf", [("circle", (0, 0), 5)])
    out = tmp_path / "a.gcode"
    assert main(["convert", str(cut), "--engrave", str(marks), "-o", str(out),
                 "--align", "lower-left", "--margin", "5"]) == 0
    text = out.read_text()
    mark = [b for b in extract_burns(parse_gcode(text), text) if b.role == "mark"][0]
    pts = mark.points[:-1]   # 閉じたパスは始点に戻るので、重複する終点を除く
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    assert abs(cx - 45) < 0.05 and abs(cy - 30) < 0.05   # 板の中心に乗ったまま


def test_as_layer_treats_whole_file_as_engrave(tmp_path):
    src = _dxf(tmp_path / "m.dxf", [("rect", 20, 20, 80, 60)])
    out = tmp_path / "m.gcode"
    assert main(["convert", str(src), "--as-layer", "engrave", "-o", str(out)]) == 0
    text = out.read_text()
    assert "role=mark" in text and "role=outer" not in text


def test_unknown_layer_name_is_rejected(tmp_path):
    src = _dxf(tmp_path / "m.dxf", [("rect", 20, 20, 80, 60)])
    out = tmp_path / "m.gcode"
    assert main(["convert", str(src), "--as-layer", "nosuch", "-o", str(out)]) == EXIT_ERROR
    assert not out.exists()
