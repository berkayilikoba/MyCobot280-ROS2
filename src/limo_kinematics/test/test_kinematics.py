"""limo_kinematics sözleşme testleri.

Beklenen sayılar elle hesaplandı ve üç bağımsız kaynaktan geliyor:
  * limo_base/include/limo_base/limo_driver.h:130-132  (gerçek robot sürücüsü)
      max_inner_angle_ = 0.48869 (28°), track_ = 0.172, wheelbase_ = 0.2
  * limo_description/urdf/limo_ackerman.xacro          (fixture: limo_description_ackermann.urdf)
  * limo_car/gazebo/ackermann_with_sensor.xacro        (fixture: limo_car_sim_ackermann.urdf)

Footprint iç teğet yarıçap beklentileri Nav2'nin çalışan yığında verdiği değerle
doğrulandı: diferansiyel footprint + 0.02 padding için Nav2 logu
"computed inscribed radius (0.120)" yazdı.
"""

import math
import os

import pytest

from limo_kinematics.geometry import AckermannGeometry, load_geometry
from limo_kinematics.kinematics import (
    central_to_inner,
    inner_to_central,
    max_curvature,
    min_turning_radius_rear_axle,
    radius_at_offset,
)
from limo_kinematics.footprint import (
    circumscribed_radius,
    inscribed_radius,
    pad_footprint,
    parse_footprint,
)
from limo_kinematics.report import check_nav2_yaml, recommend
from limo_kinematics.cli import main

HERE = os.path.dirname(os.path.abspath(__file__))
FIX = os.path.join(HERE, 'fixtures')
DESC = os.path.join(FIX, 'limo_description_ackermann.urdf')
SIM = os.path.join(FIX, 'limo_car_sim_ackermann.urdf')

TOL = 1e-3

DRIVER_L = 0.2
DRIVER_T = 0.172
DRIVER_INNER = 0.48869

ACK_FP = '[[-0.08,-0.1],[0.22,-0.1],[0.22,0.1],[-0.08,0.1]]'
DIFF_FP = '[[-0.15,-0.10],[-0.15,0.10],[0.15,0.10],[0.15,-0.10]]'


# ---------------------------------------------------------------- geometry

def test_load_description_urdf():
    g = load_geometry(DESC)
    assert isinstance(g, AckermannGeometry)
    assert g.wheelbase == pytest.approx(0.2, abs=TOL)
    assert g.track == pytest.approx(0.14, abs=TOL)
    assert g.wheel_radius == pytest.approx(0.045, abs=TOL)
    assert g.max_inner_steer == pytest.approx(0.523598767, abs=1e-6)
    assert g.rear_axle_x == pytest.approx(-0.1, abs=TOL)


def test_load_sim_urdf():
    g = load_geometry(SIM)
    assert g.wheelbase == pytest.approx(0.24, abs=TOL)
    assert g.track == pytest.approx(0.168, abs=TOL)
    assert g.wheel_radius == pytest.approx(0.045, abs=TOL)
    assert g.max_inner_steer == pytest.approx(0.523598767, abs=1e-6)
    assert g.rear_axle_x == pytest.approx(-0.12, abs=TOL)


def test_geometry_is_frozen():
    g = load_geometry(DESC)
    with pytest.raises(Exception):
        g.wheelbase = 1.0


def test_load_rejects_urdf_without_steering(tmp_path):
    p = tmp_path / 'no_steer.urdf'
    p.write_text(
        '<robot name="x"><link name="base_link"/><link name="w"/>'
        '<joint name="j" type="continuous"><parent link="base_link"/><child link="w"/>'
        '<origin xyz="0 0.1 0"/><axis xyz="0 1 0"/></joint></robot>')
    with pytest.raises(ValueError):
        load_geometry(str(p))


# ---------------------------------------------------------------- kinematics

def test_min_radius_real_driver():
    # L/tan(28°) + T/2 = 0.37615 + 0.086
    r = min_turning_radius_rear_axle(DRIVER_L, DRIVER_T, DRIVER_INNER)
    assert r == pytest.approx(0.46215, abs=TOL)


def test_min_radius_description_and_sim():
    assert min_turning_radius_rear_axle(0.2, 0.14, 0.523598767) == pytest.approx(0.41641, abs=TOL)
    assert min_turning_radius_rear_axle(0.24, 0.168, 0.523598767) == pytest.approx(0.49969, abs=TOL)


@pytest.mark.parametrize('args', [
    (0.0, 0.172, 0.4),        # dingil mesafesi sıfır
    (-0.2, 0.172, 0.4),       # negatif dingil mesafesi
    (0.2, -0.1, 0.4),         # negatif iz
    (0.2, 0.172, 0.0),        # sıfır direksiyon -> sonsuz yarıçap
    (0.2, 0.172, math.pi / 2),
    (0.2, 0.172, float('nan')),
])
def test_min_radius_rejects_invalid(args):
    with pytest.raises(ValueError):
        min_turning_radius_rear_axle(*args)


def test_radius_at_base_link():
    # base_link arka aksın L/2 önünde: sqrt(R^2 + 0.1^2)
    assert radius_at_offset(0.46215, 0.1) == pytest.approx(0.47284, abs=TOL)
    assert radius_at_offset(0.5, 0.0) == pytest.approx(0.5, abs=1e-12)


def test_max_curvature():
    assert max_curvature(0.5) == pytest.approx(2.0)
    with pytest.raises(ValueError):
        max_curvature(0.0)


def test_steer_conversion_matches_driver():
    # limo_driver.cpp:656-678 ile aynı formüller, işaret korunmalı
    inner = 0.48869
    c = inner_to_central(inner, DRIVER_L, DRIVER_T)
    expected_c = math.atan(DRIVER_L / (DRIVER_L / math.tan(inner) + DRIVER_T / 2))
    assert c == pytest.approx(expected_c, abs=1e-9)
    assert inner_to_central(-inner, DRIVER_L, DRIVER_T) == pytest.approx(-expected_c, abs=1e-9)


@pytest.mark.parametrize('inner', [0.05, 0.2, 0.35, 0.48869, -0.3])
def test_steer_conversion_roundtrip(inner):
    c = inner_to_central(inner, DRIVER_L, DRIVER_T)
    assert central_to_inner(c, DRIVER_L, DRIVER_T) == pytest.approx(inner, abs=1e-9)


def test_steer_conversion_zero():
    assert inner_to_central(0.0, DRIVER_L, DRIVER_T) == 0.0
    assert central_to_inner(0.0, DRIVER_L, DRIVER_T) == 0.0


# ---------------------------------------------------------------- footprint

def test_parse_footprint_string_and_list():
    pts = parse_footprint(ACK_FP)
    assert pts == [(-0.08, -0.1), (0.22, -0.1), (0.22, 0.1), (-0.08, 0.1)]
    assert parse_footprint([[1, 2], [3, 4], [5, 6]]) == [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]
    with pytest.raises(ValueError):
        parse_footprint('[[0,0],[1,1]]')   # 3'ten az köşe


def test_pad_footprint_follows_nav2_rule():
    # nav2_costmap_2d padFootprint: x += sign(x)*p, y += sign(y)*p
    padded = pad_footprint(parse_footprint(ACK_FP), 0.02)
    expected = [(-0.10, -0.12), (0.24, -0.12), (0.24, 0.12), (-0.10, 0.12)]
    # pytest.approx iç içe tuple listesini karşılaştıramıyor; düzleştirerek kıyasla
    assert len(padded) == len(expected)
    assert [c for p in padded for c in p] == pytest.approx([c for p in expected for c in p])


def test_inscribed_matches_nav2_log():
    assert inscribed_radius(parse_footprint(DIFF_FP), 0.02) == pytest.approx(0.120, abs=1e-6)
    assert inscribed_radius(parse_footprint(ACK_FP), 0.02) == pytest.approx(0.100, abs=1e-6)
    assert inscribed_radius(parse_footprint(ACK_FP)) == pytest.approx(0.08, abs=1e-6)


def test_circumscribed():
    assert circumscribed_radius(parse_footprint(ACK_FP)) == pytest.approx(math.hypot(0.22, 0.1), abs=1e-6)
    assert circumscribed_radius(parse_footprint(ACK_FP), 0.02) == pytest.approx(math.hypot(0.24, 0.12), abs=1e-6)


# ---------------------------------------------------------------- report

def test_recommend_rounds_up_to_centimetre():
    rec = recommend(r_min_base=0.47284, footprint=parse_footprint(ACK_FP), padding=0.02)
    assert rec['minimum_turning_radius'] == pytest.approx(0.48)
    assert rec['min_turning_r'] == pytest.approx(0.48)
    assert rec['inflation_radius_min'] == pytest.approx(0.10, abs=1e-6)
    # tam santimetrede yukarı yuvarlama yapılmamalı
    assert recommend(0.47, parse_footprint(ACK_FP), 0.02)['minimum_turning_radius'] == pytest.approx(0.47)


def _write_yaml(tmp_path, smac, mppi, infl_local, infl_global):
    p = tmp_path / 'nav2.yaml'
    p.write_text(f"""
planner_server:
  ros__parameters:
    GridBased:
      minimum_turning_radius: {smac}
controller_server:
  ros__parameters:
    FollowPath:
      AckermannConstraints:
        min_turning_r: {mppi}
local_costmap:
  local_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: {infl_local}
global_costmap:
  global_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: {infl_global}
""")
    return str(p)


def test_check_yaml_flags_too_small_radius(tmp_path):
    rec = recommend(0.47284, parse_footprint(ACK_FP), 0.02)
    y = _write_yaml(tmp_path, 0.4, 0.4, 0.15, 0.15)
    findings = {f.param: f for f in check_nav2_yaml(y, rec)}
    assert set(findings) == {
        'planner_server.ros__parameters.GridBased.minimum_turning_radius',
        'controller_server.ros__parameters.FollowPath.AckermannConstraints.min_turning_r',
        'local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius',
        'global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius',
    }
    smac = findings['planner_server.ros__parameters.GridBased.minimum_turning_radius']
    assert smac.actual == pytest.approx(0.4)
    assert smac.recommended == pytest.approx(0.48)
    assert smac.ok is False
    assert findings['local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius'].ok is True


def test_check_yaml_all_ok(tmp_path):
    rec = recommend(0.47284, parse_footprint(ACK_FP), 0.02)
    y = _write_yaml(tmp_path, 0.5, 0.48, 0.1, 0.2)
    assert all(f.ok for f in check_nav2_yaml(y, rec))


def test_check_yaml_missing_key_is_reported_not_crash(tmp_path):
    rec = recommend(0.47284, parse_footprint(ACK_FP), 0.02)
    p = tmp_path / 'empty.yaml'
    p.write_text('planner_server:\n  ros__parameters: {}\n')
    findings = check_nav2_yaml(str(p), rec)
    assert len(findings) == 4
    assert all(f.actual is None and f.ok is False for f in findings)


# ---------------------------------------------------------------- cli

def test_cli_driver_override_fails_on_current_config(tmp_path, capsys):
    y = _write_yaml(tmp_path, 0.4, 0.4, 0.15, 0.15)
    rc = main([DESC,
               '--driver', str(DRIVER_L), str(DRIVER_T), str(DRIVER_INNER),
               '--footprint', ACK_FP, '--padding', '0.02',
               '--nav2-yaml', y])
    out = capsys.readouterr().out
    assert rc == 1
    assert '0.462' in out      # arka aks yarıçapı
    assert '0.473' in out      # base_link yarıçapı
    assert '0.48' in out       # öneri


def test_cli_urdf_only_passes_when_yaml_ok(tmp_path, capsys):
    y = _write_yaml(tmp_path, 0.43, 0.43, 0.15, 0.15)
    rc = main([DESC, '--footprint', ACK_FP, '--padding', '0.02', '--nav2-yaml', y])
    out = capsys.readouterr().out
    assert rc == 0
    assert '0.416' in out and '0.428' in out


def test_cli_without_yaml_returns_zero(capsys):
    rc = main([SIM])
    out = capsys.readouterr().out
    assert rc == 0
    assert '0.24' in out and '0.168' in out and '0.500' in out


def test_cli_reports_diff_urdf_cleanly(tmp_path, capsys):
    # Direksiyonsuz (diferansiyel) URDF: traceback değil, anlaşılır mesaj + çıkış 2
    p = tmp_path / 'diff.urdf'
    p.write_text(
        '<robot name="x"><link name="base_link"/><link name="w"/>'
        '<joint name="j" type="continuous"><parent link="base_link"/><child link="w"/>'
        '<origin xyz="0 0.1 0"/><axis xyz="0 1 0"/></joint></robot>')
    rc = main([str(p)])
    captured = capsys.readouterr()
    assert rc == 2
    assert 'Traceback' not in captured.err + captured.out
    assert 'steering' in captured.err.lower() or 'direksiyon' in captured.err.lower()


def test_cli_missing_file_is_clean(capsys):
    rc = main(['/nonexistent/robot.urdf'])
    captured = capsys.readouterr()
    assert rc == 2
    assert 'Traceback' not in captured.err + captured.out
