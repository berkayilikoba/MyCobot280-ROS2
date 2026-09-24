"""twist_to_inner_steering: C++ limo_base kinematics::twistToInnerSteering ile aynı
davranış (beklenen sayılar test_limo_kinematics.cpp'dekilerle birebir aynı).
Simülasyondaki cmd_vel adaptörü bu fonksiyonu kullanır; gerçek robot sürücüsüyle
aynı cmd_vel anlamını üretmesi bu testlerle garanti altına alınır."""

import math

import pytest

from limo_kinematics.kinematics import inner_to_central, twist_to_inner_steering

L, T, M = 0.2, 0.172, 0.48869
EPS = 1e-9


def test_straight_and_stop():
    assert twist_to_inner_steering(0.5, 0.0, L, T, M) == 0.0
    assert twist_to_inner_steering(0.0, 0.0, L, T, M) == 0.0


def test_pure_rotation_locks_with_sign():
    assert twist_to_inner_steering(0.0, 0.7, L, T, M) == pytest.approx(M, abs=EPS)
    assert twist_to_inner_steering(0.0, -0.7, L, T, M) == pytest.approx(-M, abs=EPS)


def test_known_curvature_and_reverse():
    assert twist_to_inner_steering(0.5, 0.5, L, T, M) == pytest.approx(0.21542295681067072, abs=EPS)
    assert twist_to_inner_steering(-0.5, 0.5, L, T, M) == pytest.approx(-0.21542295681067072, abs=EPS)


def test_clamps():
    assert twist_to_inner_steering(0.3, 1.0, L, T, M) == pytest.approx(M, abs=EPS)
    assert twist_to_inner_steering(0.3, -1.0, L, T, M) == pytest.approx(-M, abs=EPS)
    assert twist_to_inner_steering(0.01, 2.0, L, T, M) == pytest.approx(M, abs=EPS)


@pytest.mark.parametrize('v,w', [(float('nan'), 0.1), (0.1, float('nan')), (float('inf'), 0.1), (0.1, -float('inf'))])
def test_non_finite_returns_none(v, w):
    assert twist_to_inner_steering(v, w, L, T, M) is None


def test_grid_is_finite_and_bounded():
    v = -1.0
    while v <= 1.0:
        w = -3.0
        while w <= 3.0:
            r = twist_to_inner_steering(v, w, L, T, M)
            assert r is not None and math.isfinite(r) and abs(r) <= M + EPS, (v, w)
            w += 0.05
        v += 0.05


def test_central_angle_reproduces_commanded_radius():
    # Adaptörün asıl amacı: v/w yaw hızı komutu -> arka aks merkezli yarıçap L/tan(central) ~ |v/w|
    for v, w in [(0.3, 0.3), (0.3, 0.5), (0.15, 0.2), (0.45, 0.2)]:
        c = inner_to_central(twist_to_inner_steering(v, w, L, T, M), L, T)
        assert L / math.tan(abs(c)) == pytest.approx(abs(v / w), rel=1e-6), (v, w)
