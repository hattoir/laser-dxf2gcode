"""組み木の箱: 5 枚を 3D に組んだとき、辺・角のどの点もちょうど 1 枚だけが占めること。"""
import pytest

from src.boxgen import build_panels, finger_count, pack
from src.errors import Dxf2GcodeError
from src.geometry import bbox, point_in_polygon, signed_area

CASES = [(100, 100, 74, 2.5, 7.5), (100, 100, 74, 2.7, 8.1), (120, 80, 60, 6.0, 18.0), (60, 90, 40, 3.0, 9.0)]


def owners(panels, W, D, H, T, x, y, z):
    p = {q.name: q.poly for q in panels}
    n = 0
    n += 0 <= y <= T and point_in_polygon((x, z), p["front"])
    n += D - T <= y <= D and point_in_polygon((x, z), p["back"])
    n += 0 <= x <= T and point_in_polygon((y, z), p["left"])
    n += W - T <= x <= W and point_in_polygon((y, z), p["right"])
    n += 0 <= z <= T and point_in_polygon((x, y), p["bottom"])
    return n


def frange(a, b, step):
    v = a
    while v < b:
        yield v
        v += step


@pytest.mark.parametrize("W,D,H,T,f", CASES)
def test_every_edge_point_is_owned_by_exactly_one_panel(W, D, H, T, f):
    ps = build_panels(W, D, H, T, f)
    cross = (0.27 * T, 0.73 * T)
    far = lambda L: (L - 0.73 * T, L - 0.27 * T)
    # 縦の 4 辺(下の角を含む)
    for xs in (cross, far(W)):
        for ys in (cross, far(D)):
            for z in frange(0.013, H, 0.21):
                for x in xs:
                    for y in ys:
                        assert owners(ps, W, D, H, T, x, y, z) == 1, (x, y, z)
    # 下の 4 辺
    for x in frange(0.013, W, 0.21):
        for y in cross + far(D):
            for z in cross:
                assert owners(ps, W, D, H, T, x, y, z) == 1, (x, y, z)
    for y in frange(0.013, D, 0.21):
        for x in cross + far(W):
            for z in cross:
                assert owners(ps, W, D, H, T, x, y, z) == 1, (x, y, z)


@pytest.mark.parametrize("W,D,H,T,f", CASES)
def test_faces_and_inside(W, D, H, T, f):
    ps = build_panels(W, D, H, T, f)
    assert owners(ps, W, D, H, T, W / 2, T / 2, H / 2) == 1        # 前の面
    assert owners(ps, W, D, H, T, T / 2, D / 2, H / 2) == 1        # 左の面
    assert owners(ps, W, D, H, T, W / 2, D / 2, T / 2) == 1        # 底の面
    assert owners(ps, W, D, H, T, W / 2, D / 2, H / 2) == 0        # 箱の中は空
    assert owners(ps, W, D, H, T, W / 2, D / 2, H + 1) == 0        # 上は開いている


@pytest.mark.parametrize("W,D,H,T,f", CASES)
def test_volume_equals_shell(W, D, H, T, f):
    ps = build_panels(W, D, H, T, f)
    vol = sum(abs(signed_area(p.poly)) * T for p in ps)
    shell = W * D * H - (W - 2 * T) * (D - 2 * T) * (H - T)
    assert vol == pytest.approx(shell, rel=1e-6)


@pytest.mark.parametrize("W,D,H,T,f", CASES)
def test_panels_keep_outer_dimensions(W, D, H, T, f):
    for p in build_panels(W, D, H, T, f):
        x0, y0, x1, y1 = bbox(p.poly)
        assert (x0, y0) == pytest.approx((0, 0)) and (x1, y1) == pytest.approx((p.width, p.height))


def test_finger_count_is_odd():
    for L in (30, 71.5, 95, 200):
        for t in (5, 7.5, 10):
            assert finger_count(L, t) % 2 == 1


def test_too_small_box_is_rejected():
    with pytest.raises(Dxf2GcodeError):
        build_panels(20, 20, 20, 6, 10)


def test_pack_uses_multiple_sheets_when_needed():
    ps = build_panels(100, 100, 74, 2.5, 7.5)
    one = pack(ps, 400, 415)
    assert {p.sheet for p in one} == {0}
    small = pack(ps, 220, 190)
    assert len({p.sheet for p in small}) >= 2
    for pl in small:  # 余白の内側に収まる
        x0, y0, x1, y1 = bbox(pl.poly())
        assert x0 >= 5 - 1e-9 and y0 >= 5 - 1e-9 and x1 <= 215 + 1e-9 and y1 <= 185 + 1e-9
