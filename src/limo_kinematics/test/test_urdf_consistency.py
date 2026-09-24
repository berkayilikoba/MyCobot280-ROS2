"""Faz 1 sözleşmesi: üç URDF gerçek robotla aynı geometriyi tanımlamalı.

Hedef değerler ve kaynakları:
  * dingil mesafesi 0.200, iz 0.172, iç direksiyon sınırı 0.48869 (28°)
      -> limo_base/include/limo_base/limo_driver.h:130-132
  * tekerlek yarıçapı 0.045 -> üç URDF'te de zaten böyle, Gazebo wheel_diameter 0.09
  * sensör konumları -> robottaki ROS 1 limo_start.launch (static_transform_publisher)
      laser : 0.105 0 0.08, YAW -0.15  (konumsal argüman sırası x y z yaw pitch roll;
              tf2_echo ile ölçüldü: RPY [0, 0, -0.150])
      camera: 0.105 0 0.1   (çerçeve adı camera_link — kamera sürücülerinin kök çerçevesi)
      imu   : 0 0 0

Bu testler xacro'yu çalıştırır; ROS ortamı ve workspace source edilmiş olmalı.
"""

import math
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest

from limo_kinematics.geometry import load_geometry
from limo_kinematics.kinematics import min_turning_radius_rear_axle, radius_at_offset

SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

ACKERMANN_SOURCES = {
    'description_ackermann': os.path.join(SRC, 'limo_description', 'urdf', 'limo_ackerman.xacro'),
    'sim_ackermann': os.path.join(SRC, 'limo_car', 'gazebo', 'ackermann_with_sensor.xacro'),
}
DIFF_SOURCE = os.path.join(SRC, 'limo_description', 'urdf', 'limo_four_diff.xacro')
ALL_SOURCES = dict(ACKERMANN_SOURCES, description_four_diff=DIFF_SOURCE)

L, T, INNER, WHEEL_R = 0.200, 0.172, 0.48869, 0.045
TOL = 1e-4

SENSORS = {
    'laser_link': ((0.105, 0.0, 0.08), (0.0, 0.0, -0.15)),
    'camera_link': ((0.105, 0.0, 0.1), (0.0, 0.0, 0.0)),
    'imu_link': ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
}

pytestmark = pytest.mark.skipif(shutil.which('xacro') is None,
                                reason='xacro yok — ROS ortamını source edin')


def _expand(src, tmp_path_factory):
    out = tmp_path_factory.mktemp('urdf') / (os.path.basename(src) + '.urdf')
    res = subprocess.run(['xacro', src], capture_output=True, text=True)
    assert res.returncode == 0, f'xacro başarısız: {src}\n{res.stderr}'
    out.write_text(res.stdout)
    return str(out)


@pytest.fixture(scope='module')
def urdfs(tmp_path_factory):
    return {k: _expand(v, tmp_path_factory) for k, v in ALL_SOURCES.items()}


def _root(path):
    return ET.parse(path).getroot()


def _floats(s, n=3):
    vals = [float(v) for v in (s or '').split()]
    return tuple(vals) if vals else (0.0,) * n


def _joint_to(root, child):
    for j in root.findall('joint'):
        if j.find('child').get('link') == child:
            return j
    return None


# ---------------------------------------------------------------- geometri

@pytest.mark.parametrize('name', sorted(ACKERMANN_SOURCES))
def test_ackermann_geometry_matches_driver(urdfs, name):
    g = load_geometry(urdfs[name])
    assert g.wheelbase == pytest.approx(L, abs=TOL), name
    assert g.track == pytest.approx(T, abs=TOL), name
    assert g.max_inner_steer == pytest.approx(INNER, abs=TOL), name
    assert g.wheel_radius == pytest.approx(WHEEL_R, abs=TOL), name
    assert g.rear_axle_x == pytest.approx(-L / 2, abs=TOL), name


def test_all_ackermann_models_agree_on_turning_radius(urdfs):
    radii = []
    for name in ACKERMANN_SOURCES:
        g = load_geometry(urdfs[name])
        r = radius_at_offset(min_turning_radius_rear_axle(g.wheelbase, g.track, g.max_inner_steer),
                             abs(g.rear_axle_x))
        radii.append(r)
    assert radii == pytest.approx([0.47282] * len(radii), abs=1e-3)


def test_four_diff_wheel_positions(urdfs):
    root = _root(urdfs['description_four_diff'])
    wheels = [j for j in root.findall('joint') if j.get('type') == 'continuous']
    assert len(wheels) == 4
    xs = sorted(round(_floats(j.find('origin').get('xyz'))[0], 4) for j in wheels)
    ys = sorted(round(abs(_floats(j.find('origin').get('xyz'))[1]), 4) for j in wheels)
    assert xs == pytest.approx([-L / 2, -L / 2, L / 2, L / 2], abs=TOL)
    assert ys == pytest.approx([T / 2] * 4, abs=TOL)


# ---------------------------------------------------------------- sensörler

@pytest.mark.parametrize('name', sorted(ALL_SOURCES))
@pytest.mark.parametrize('link', sorted(SENSORS))
def test_sensor_pose_matches_field_calibration(urdfs, name, link):
    root = _root(urdfs[name])
    j = _joint_to(root, link)
    assert j is not None, f'{name}: {link} yok'
    assert j.find('parent').get('link') == 'base_link', f'{name}: {link} base_link altında değil'
    o = j.find('origin')
    xyz, rpy = SENSORS[link]
    assert _floats(o.get('xyz')) == pytest.approx(xyz, abs=TOL), f'{name}:{link} xyz'
    assert _floats(o.get('rpy')) == pytest.approx(rpy, abs=TOL), f'{name}:{link} rpy'


@pytest.mark.parametrize('name', sorted(ALL_SOURCES))
def test_old_camera_frame_name_is_gone(urdfs, name):
    names = {l.get('name') for l in _root(urdfs[name]).findall('link')}
    assert 'depth_camera_link' not in names


@pytest.mark.parametrize('name', sorted(ALL_SOURCES))
def test_gazebo_references_point_to_existing_links(urdfs, name):
    root = _root(urdfs[name])
    links = {l.get('name') for l in root.findall('link')}
    refs = {g.get('reference') for g in root.findall('gazebo') if g.get('reference')}
    assert refs <= links, f'{name}: tanımsız link\'e gazebo referansı: {sorted(refs - links)}'


# ---------------------------------------------------------------- Gazebo eklentisi

def test_sim_ackermann_plugin_matches_driver(urdfs):
    xml = open(urdfs['sim_ackermann']).read()
    root = ET.fromstring(xml)
    plugin = next(p for p in root.iter('plugin') if 'ackermann_drive' in (p.get('filename') or ''))
    assert float(plugin.find('max_steer').text) == pytest.approx(INNER, abs=TOL)
    assert float(plugin.find('wheel_separation').text) == pytest.approx(T, abs=TOL)
    assert float(plugin.find('wheel_diameter').text) == pytest.approx(2 * WHEEL_R, abs=TOL)


# ---------------------------------------------------------------- bütünlük

@pytest.mark.skipif(shutil.which('check_urdf') is None, reason='check_urdf yok')
@pytest.mark.parametrize('name', sorted(ALL_SOURCES))
def test_check_urdf_passes(urdfs, name):
    res = subprocess.run(['check_urdf', urdfs[name]], capture_output=True, text=True)
    assert res.returncode == 0, res.stderr + res.stdout
    assert 'root Link: base_footprint' in res.stdout
