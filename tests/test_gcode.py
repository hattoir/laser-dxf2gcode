"""G-code 生成と安全検証のテスト(安全に関わるので重点的に)。"""
import math
import re

import pytest

from src.config import LayerSettings, Machine, Profile
from src.dxf_reader import Contour
from src.errors import GcodeSafetyError, WorkAreaError
from src.gcode import generate_gcode, parse_gcode, power_to_s, strip_comment, verify_gcode
from src.toolpath import plan_toolpaths


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def circle(cx, cy, r, n=90):
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def profile(kerf=0.2, **cut):
    c = dict(power=100, feed=250, passes=2)
    c.update(cut)
    return Profile("t", kerf, "cut", {
        "cut": LayerSettings("cut", **c),
        "engrave": LayerSettings("engrave", power=30, feed=3000, through_cut=False, kerf_compensation=False, order=0),
    })


def machine(**kw):
    return Machine(**kw)


def build(polys, prof=None, mach=None, **kw):
    cs = [Contour(list(p), True, "0", f"p{i}") for i, p in enumerate(polys)]
    plan = plan_toolpaths(cs, prof or profile())
    return plan, generate_gcode(plan, mach or machine(), **kw)


def commands(text):
    return [c for c in (strip_comment(l) for l in text.splitlines()) if c]


# ---------------- 4. エリア検査 ----------------

@pytest.mark.parametrize("poly", [
    rect(-1, 10, 20, 20),          # X が負
    rect(10, -0.5, 20, 20),        # Y が負(0.5mm でも)
    rect(390, 10, 401, 20),        # X が 400 を 1mm 超え
    rect(10, 400, 20, 415.5),      # Y が 415 を超え
    rect(500, 500, 520, 520),      # 完全に外
])
def test_out_of_area_raises_and_no_gcode(poly):
    plan = plan_toolpaths([Contour(poly, True, "0", "bad")], profile(kerf=0.0))
    with pytest.raises(WorkAreaError) as ei:
        generate_gcode(plan, machine())
    assert "G-code は出力しません" in str(ei.value)
    assert "bad" in str(ei.value)  # 該当パスが表示される


def test_exact_boundary_is_allowed():
    _, g = build([rect(0, 0, 400, 415)], prof=profile(kerf=0.0))
    assert "X400.000 Y415.000" in g.text


def test_violation_after_kerf_offset_detected():
    # 外形はカーフ補正で外側(捨て側)へ k/2 広がる。原点に接して描いた部品はパスが X=-0.1 になる → 停止
    plan = plan_toolpaths([Contour(rect(0, 0, 50, 50), True, "0", "o")], profile(kerf=0.2))
    with pytest.raises(WorkAreaError):
        generate_gcode(plan, machine())


def test_small_work_area_machine():
    with pytest.raises(WorkAreaError):
        build([rect(10, 10, 150, 150)], mach=machine(x_max=100, y_max=100))


def test_lead_in_outside_area_detected():
    # 端から 0.5mm の部品に 2mm のリードイン → 進入点が X=-1.5 → 停止(最終防壁)
    cs = [Contour(rect(0.5, 10, 20, 30), True, "0", "near-edge")]
    plan = plan_toolpaths(cs, profile(kerf=0.0), lead_in=2.0, start=(0, 20))
    assert plan.paths[0].lead_in and plan.paths[0].points[0][0] < 0
    with pytest.raises(WorkAreaError):
        generate_gcode(plan, machine())


def test_lead_in_respects_bounds_when_given():
    cs = [Contour(rect(0.5, 10, 20, 30), True, "0", "near-edge")]
    plan = plan_toolpaths(cs, profile(kerf=0.0), lead_in=2.0, start=(0, 20), bounds=(0, 0, 400, 415))
    lp = plan.paths[0].points[0]
    assert plan.paths[0].lead_in and 0 <= lp[0] <= 400 and 0 <= lp[1] <= 415
    generate_gcode(plan, machine())


# ---------------- 5. G-code の規則 ----------------

def test_every_g0_has_s0_and_laser_off():
    _, g = build([rect(50, 50, 100, 100), circle(75, 75, 5), circle(60, 60, 3)])
    laser_on = False
    n_g0 = 0
    for c in commands(g.text):
        if "M4" in c.split():
            laser_on = True
        if "M5" in c.split():
            laser_on = False
        if re.search(r"\bG0\b", c):
            n_g0 += 1
            assert re.search(r"\bS0\b", c), c
            assert not laser_on, f"発振中の G0: {c}"
    assert n_g0 >= 3 * 2 + 2


def test_header_and_footer():
    _, g = build([rect(50, 50, 100, 100)])
    cmds = commands(g.text)
    assert cmds[0] == "G21 G90 G94"
    assert cmds[1] == "M5"
    assert cmds[2] == "S0"
    assert cmds[3].startswith("G0 ") and cmds[3].endswith("S0")
    assert cmds[-1] == "M5"


def test_ends_with_m5_even_without_return_home():
    _, g = build([rect(50, 50, 100, 100)], mach=machine(return_home=False))
    cmds = commands(g.text)
    assert cmds[-1] == "M5"
    assert not any(c.startswith("G0 X0.000 Y0.000") for c in cmds[4:])


def test_each_burn_is_m4_then_g1_then_m5():
    _, g = build([rect(50, 50, 100, 100), circle(75, 75, 5)])
    cmds = commands(g.text)
    for i, c in enumerate(cmds):
        if c.startswith("M4"):
            assert cmds[i - 1].startswith("G0 ") and cmds[i - 1].endswith("S0")
            assert cmds[i + 1].startswith("G1 ") and " F" in cmds[i + 1]
            j = i + 1
            while cmds[j].startswith("G1"):
                j += 1
            assert cmds[j] == "M5"
    assert not any(c.startswith("M3") for c in cmds)


def test_passes_repeat_and_per_pass_override():
    prof = profile(passes=3, per_pass=[{"power": 50, "feed": 400}])
    _, g = build([rect(50, 50, 100, 100)], prof=prof)
    cmds = commands(g.text)
    m4 = [c for c in cmds if c.startswith("M4")]
    assert m4 == ["M4 S500", "M4 S1000", "M4 S1000"]
    feeds = [re.search(r"F(\S+)", c).group(1) for c in cmds if c.startswith("G1") and " F" in c]
    assert feeds == ["400", "250", "250"]
    assert g.burns == 3


def test_pass_mode_path_keeps_holes_complete_before_outline():
    plan, g = build([rect(50, 50, 100, 100), circle(75, 75, 5)], prof=profile(passes=3))
    roles = re.findall(r"role=(\w+)", g.text)
    assert roles == ["hole"] * 3 + ["outer"] * 3


def test_pass_mode_cycle():
    plan, g = build([rect(50, 50, 100, 100), circle(75, 75, 5)], prof=profile(passes=2), pass_mode="cycle")
    roles = re.findall(r"role=(\w+)", g.text)
    assert roles == ["hole", "outer", "hole", "outer"]


def test_power_percent_to_s_and_clamp():
    w = []
    assert power_to_s(100, 1000) == 1000
    assert power_to_s(30, 1000) == 300
    assert power_to_s(50, 255) == 128
    assert power_to_s(150, 1000, w) == 1000
    assert power_to_s(-5, 1000, w) == 0
    assert len(w) == 2


def test_s_value_uses_machine_s_max():
    _, g = build([rect(50, 50, 100, 100)], prof=profile(power=40, passes=1), mach=machine(s_max=255))
    assert "M4 S102" in g.text


def test_comments_are_ascii():
    cs = [Contour(rect(50, 50, 100, 100), True, "切断", "p")]
    plan = plan_toolpaths(cs, profile())
    g = generate_gcode(plan, machine(), meta={"source": "部品.dxf"})
    g.text.encode("ascii")


# ---------------- 生成後検証(verify_gcode) 自体のテスト ----------------

GOOD = "G21 G90 G94\nM5\nS0\nG0 X0 Y0 S0\nG0 X10 Y10 S0\nM4 S500\nG1 X20 Y10 F300\nM5\nG0 X0 Y0 S0\nM5\n"


def test_verify_accepts_good():
    parsed = verify_gcode(GOOD, machine())
    assert sum(1 for m in parsed.motions if m.burning) == 1


@pytest.mark.parametrize("bad, err", [
    (GOOD.replace("G0 X10 Y10 S0", "G0 X10 Y10"), GcodeSafetyError),           # G0 に S0 なし
    (GOOD.replace("M5\nG0 X0 Y0 S0\nM5\n", "G0 X0 Y0 S0\nM5\n"), GcodeSafetyError),  # 発振中の G0
    (GOOD.rstrip("\n").rsplit("\n", 1)[0] + "\n", GcodeSafetyError),          # 末尾 M5 なし
    (GOOD.replace("M4 S500", "M3 S500"), GcodeSafetyError),                    # M3 は禁止
    (GOOD.replace("X20 Y10", "X400.5 Y10"), WorkAreaError),                    # 範囲外
    (GOOD.replace("M4 S500", "M4 S1500"), GcodeSafetyError),                   # S 最大超え
    (GOOD.replace("G21 G90 G94", "G21 G91 G94"), GcodeSafetyError),            # 相対座標
    (GOOD.replace("G1 X20 Y10 F300", "G2 X20 Y10 I5 J0 F300"), GcodeSafetyError),  # 円弧は未対応
    ("M5\n" + GOOD, GcodeSafetyError),                                         # 先頭がヘッダでない
])
def test_verify_rejects_bad(bad, err):
    with pytest.raises(err):
        verify_gcode(bad, machine())


def test_parse_motion_kinds():
    parsed = parse_gcode(GOOD)
    kinds = [(m.kind, m.burning) for m in parsed.motions]
    assert kinds == [("G0", False), ("G0", False), ("G1", True), ("G0", False)]
