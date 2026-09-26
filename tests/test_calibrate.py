import argparse
import shutil
from pathlib import Path

import pytest
import yaml

from src.calibrate import (FONT, default_variants, kerf_from_measurement, parse_variants, set_yaml_value,
                           text_polylines, text_width, update_profile_file, write_coupon_dxf, write_kerf_dxf)
from src.config import LayerSettings, load_profile
from src.dxf_reader import read_dxf
from src.errors import ConfigError, Dxf2GcodeError
from src.geometry import bbox

ROOT = Path(__file__).resolve().parent.parent


def test_set_yaml_value_keeps_comments_and_structure():
    text = (ROOT / "profiles" / "mdf_5mm.yaml").read_text(encoding="utf-8")
    new = set_yaml_value(text, ["mdf_5mm", "layers", "cut", "feed"], "300", "【実測値】test")
    data = yaml.safe_load(new)
    assert data["mdf_5mm"]["layers"]["cut"]["feed"] == 300
    assert data["mdf_5mm"]["layers"]["engrave"]["feed"] == 3000   # 同名キーの別の層は触らない
    assert "【実測値】test" in new
    # 変更した 1 行以外は同一
    diff = [(a, b) for a, b in zip(text.splitlines(), new.splitlines()) if a != b]
    assert len(diff) == 1 and len(text.splitlines()) == len(new.splitlines())


def test_set_yaml_value_missing_key():
    with pytest.raises(ConfigError):
        set_yaml_value("a:\n  b: 1\n", ["a", "c"], "2", "x")


def test_update_profile_file_roundtrip(tmp_path):
    p = tmp_path / "mdf_5mm.yaml"
    shutil.copy(ROOT / "profiles" / "mdf_5mm.yaml", p)
    update_profile_file(p, [(["mdf_5mm", "kerf"], "0.180", "【実測値】2026-09-11 test")])
    prof = load_profile(str(p))
    assert prof.kerf == pytest.approx(0.18)
    assert "【実測値】2026-09-11 test" in p.read_text(encoding="utf-8")


def _ns(**kw):
    base = dict(kerf=None, strip_width=None, square=None, nominal=None, coupon_kerf=None)
    base.update(kw)
    return argparse.Namespace(**base)


def test_kerf_from_strip_width():
    k, how = kerf_from_measurement(_ns(strip_width=9.82), current=0.2)
    assert k == pytest.approx(0.18)
    assert "9.82" in how


def test_kerf_from_coupon_square():
    # 補正 0.20 で切って 20.04mm に仕上がった → 実際のカーフは 0.16
    k, _ = kerf_from_measurement(_ns(square=20.04, coupon_kerf=0.20), current=0.3)
    assert k == pytest.approx(0.16)


def test_kerf_requires_measurement():
    with pytest.raises(Dxf2GcodeError):
        kerf_from_measurement(_ns(), current=0.2)


def test_parse_variants():
    v = parse_variants("100:250:4, 90:300:3")
    assert [(x.label, x.power, x.feed, x.passes) for x in v] == [("A", 100, 250, 4), ("B", 90, 300, 3)]
    with pytest.raises(Dxf2GcodeError):
        parse_variants("100:250")


def test_default_variants_never_exceed_base_energy():
    base = LayerSettings("cut", power=100, feed=250, passes=4)
    for v in default_variants(base):
        assert v.power <= base.power and v.feed >= base.feed and v.passes <= base.passes


def test_font_covers_labels():
    for s in ["P100", "F3000", "N4", "ABCD", "K0.5"]:
        assert text_polylines(s, 0, 0, 2.0)
    assert set("0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ .-/") <= set(FONT)
    for ch in FONT:
        assert text_polylines(ch, 0, 0, 3.0) or ch == " "
    assert text_width("P100", 6.0) == pytest.approx(22.0)


def test_coupon_dxf_labels_inside_squares(tmp_path):
    variants = parse_variants("100:250:4,100:3000:12,80:250:4,55.5:1250:3")
    p = tmp_path / "c.dxf"
    write_coupon_dxf(p, variants, (10, 10), size=20)
    res = read_dxf(p)
    squares = sorted((c for c in res.contours if c.layer.startswith("cut_")), key=lambda c: c.points[0][0])
    assert len(squares) == 4
    marks = [c for c in res.contours if c.layer == "engrave"]
    for m in marks:
        x0, y0, x1, y1 = bbox(m.points)
        assert any(sq_x0 + 0.5 < x0 and x1 < sq_x1 - 0.5 and sq_y0 + 0.5 < y0 and y1 < sq_y1 - 0.5
                   for sq_x0, sq_y0, sq_x1, sq_y1 in (bbox(s.points) for s in squares)), "刻印がピースからはみ出している"


def test_kerf_strip_dxf(tmp_path):
    p = tmp_path / "k.dxf"
    write_kerf_dxf(p, (10, 10), 50, 10, 3)
    res = read_dxf(p)
    assert len(res.closed) == 3
    for c in res.closed:
        x0, y0, x1, y1 = bbox(c.points)
        assert (x1 - x0, y1 - y0) == (pytest.approx(50), pytest.approx(10))
