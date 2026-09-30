"""profiles/ にあるすべての材料プロファイルが読めて、検証を通ること。"""
from pathlib import Path

import pytest

from src.config import load_machine, load_profile

ROOT = Path(__file__).resolve().parent.parent
PROFILES = sorted(p.stem for p in (ROOT / "profiles").glob("*.yaml"))


def test_machine_yaml_loads():
    m = load_machine()
    assert m.x_max == 400 and m.y_max == 415


@pytest.mark.parametrize("name", PROFILES)
def test_profile_loads_and_validates(name):
    prof = load_profile(name)
    assert prof.name == name
    assert "cut" in prof.layers and prof.layers["cut"].through_cut
    assert "engrave" in prof.layers and not prof.layers["engrave"].through_cut
    assert 0 < prof.kerf < 1


@pytest.mark.parametrize("name", PROFILES)
def test_every_value_is_labelled_measured_or_assumed(name):
    # 数値の行には必ず【実測値】か【仮定値】のコメントを付ける(README の約束)
    text = (ROOT / "profiles" / f"{name}.yaml").read_text(encoding="utf-8")
    for line in text.splitlines():
        key = line.split("#")[0].strip()
        if any(key.startswith(k + ":") for k in ("kerf", "power", "feed", "passes")):
            assert "【実測値】" in line or "【仮定値】" in line, f"{name}: 注記がない行: {line}"
