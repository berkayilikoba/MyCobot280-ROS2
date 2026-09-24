"""limo_autonomy.mission sözleşmesi — ROS'tan bağımsız görev durum makinesi.

Eylem biçimi: (tur, waypoint) — tur ∈ {'goto', 'home', 'done'}; 'done' için waypoint None.
Sonuçlar: 'succeeded' | 'failed' | 'timeout' | 'canceled'.
"""

import pytest

from limo_autonomy.mission import MissionPlanner, Waypoint, load_mission

A = Waypoint('a', 1.0, 0.0, 0.0)
B = Waypoint('b', 2.0, 1.0, 90.0)
C = Waypoint('c', 0.0, 2.0, 180.0)
HOME = Waypoint('home', 0.0, 0.0, 0.0)


def mk(**kw):
    args = dict(waypoints=[A, B, C], home=HOME, loop=False, max_laps=1,
                max_retries=1, max_consecutive_failures=2)
    args.update(kw)
    return MissionPlanner(**args)


# ------------------------------------------------------------------ temel akış

def test_start_goes_to_first_waypoint():
    assert mk().start() == ('goto', A)


def test_all_succeed_then_home_then_done():
    m = mk()
    assert m.start() == ('goto', A)
    assert m.report('succeeded') == ('goto', B)
    assert m.report('succeeded') == ('goto', C)
    assert m.report('succeeded') == ('home', HOME)
    assert m.report('succeeded') == ('done', None)
    s = m.stats()
    assert s['succeeded'] == 3 and s['failed'] == 0 and s['skipped'] == 0 and s['laps'] == 1
    assert s['end_reason'] == 'completed'


def test_done_is_terminal():
    m = mk(waypoints=[A])
    m.start(); m.report('succeeded'); m.report('succeeded')
    assert m.report('succeeded') == ('done', None)
    assert m.report('failed') == ('done', None)


# ------------------------------------------------------------------ yeniden deneme / atlama

def test_retry_then_success():
    m = mk(max_retries=1)
    m.start()
    assert m.report('failed') == ('goto', A)        # 1 yeniden deneme
    assert m.report('succeeded') == ('goto', B)
    assert m.stats()['retries'] == 1
    assert m.stats()['succeeded'] == 1


def test_retries_exhausted_skips_to_next():
    m = mk(max_retries=1, max_consecutive_failures=5)
    m.start()
    assert m.report('timeout') == ('goto', A)       # deneme 2
    assert m.report('failed') == ('goto', B)        # A atlandı
    s = m.stats()
    assert s['skipped'] == 1 and s['failed'] == 2


def test_zero_retries_skips_immediately():
    m = mk(max_retries=0, max_consecutive_failures=5)
    m.start()
    assert m.report('failed') == ('goto', B)


def test_consecutive_skips_abort_to_home():
    m = mk(max_retries=0, max_consecutive_failures=2)
    m.start()
    assert m.report('failed') == ('goto', B)        # A atlandı (1)
    assert m.report('failed') == ('home', HOME)     # B atlandı (2) -> eve dön
    assert m.report('failed') == ('done', None)     # eve dönüş sonucu ne olursa olsun bitir
    assert m.stats()['end_reason'] == 'too_many_failures'


def test_success_resets_consecutive_failures():
    m = mk(waypoints=[A, B, C, A], max_retries=0, max_consecutive_failures=2)
    m.start()
    assert m.report('failed') == ('goto', B)        # 1
    assert m.report('succeeded') == ('goto', C)     # sıfırlandı
    assert m.report('failed') == ('goto', A)        # 1 (2 değil)


def test_canceled_counts_as_failure():
    m = mk(max_retries=0, max_consecutive_failures=5)
    m.start()
    assert m.report('canceled') == ('goto', B)
    assert m.stats()['failed'] == 1


def test_unknown_outcome_rejected():
    m = mk()
    m.start()
    with pytest.raises(ValueError):
        m.report('maybe')


# ------------------------------------------------------------------ döngü (devriye)

def test_loop_wraps_and_counts_laps():
    m = mk(waypoints=[A, B], loop=True, max_laps=2)
    assert m.start() == ('goto', A)
    assert m.report('succeeded') == ('goto', B)
    assert m.report('succeeded') == ('goto', A)     # tur 2
    assert m.report('succeeded') == ('goto', B)
    assert m.report('succeeded') == ('home', HOME)  # max_laps doldu
    assert m.report('succeeded') == ('done', None)
    assert m.stats()['laps'] == 2


def test_infinite_loop_when_max_laps_zero():
    m = mk(waypoints=[A, B], loop=True, max_laps=0)
    m.start()
    for _ in range(20):
        kind, _ = m.report('succeeded')
        assert kind == 'goto'


# ------------------------------------------------------------------ dış olaylar

def test_request_return_home_preempts():
    m = mk()
    m.start()
    m.request_return_home('battery_low')
    assert m.report('succeeded') == ('home', HOME)
    assert m.report('succeeded') == ('done', None)
    assert m.stats()['end_reason'] == 'battery_low'


def test_no_home_means_done_directly():
    m = mk(home=None, max_retries=0, max_consecutive_failures=1)
    m.start()
    assert m.report('failed') == ('done', None)
    assert m.stats()['end_reason'] == 'too_many_failures'


def test_empty_waypoints_rejected():
    with pytest.raises(ValueError):
        MissionPlanner(waypoints=[], home=HOME)


def test_report_before_start_rejected():
    with pytest.raises(RuntimeError):
        mk().report('succeeded')


# ------------------------------------------------------------------ YAML

def test_load_mission(tmp_path):
    p = tmp_path / 'm.yaml'
    p.write_text("""
home: {x: 0.0, y: 0.0, yaw: 0.0}
loop: true
max_laps: 3
max_retries: 2
max_consecutive_failures: 3
waypoints:
  - {name: kapi, x: 2.5, y: 0.0, yaw: 0.0}
  - {name: kose, x: 3.0, y: -3.0, yaw: -90.0}
""")
    cfg = load_mission(str(p))
    assert cfg['waypoints'] == [Waypoint('kapi', 2.5, 0.0, 0.0), Waypoint('kose', 3.0, -3.0, -90.0)]
    assert cfg['home'] == Waypoint('home', 0.0, 0.0, 0.0)
    assert cfg['loop'] is True and cfg['max_laps'] == 3
    assert cfg['max_retries'] == 2 and cfg['max_consecutive_failures'] == 3
    MissionPlanner(**cfg)  # doğrudan kurulabilmeli


def test_load_mission_defaults_and_errors(tmp_path):
    p = tmp_path / 'm.yaml'
    p.write_text("waypoints:\n  - {name: a, x: 1, y: 2}\n")
    cfg = load_mission(str(p))
    assert cfg['waypoints'] == [Waypoint('a', 1.0, 2.0, 0.0)]
    assert cfg['home'] is None and cfg['loop'] is False
    assert cfg['max_retries'] == 1 and cfg['max_consecutive_failures'] == 2 and cfg['max_laps'] == 1
    bad = tmp_path / 'b.yaml'
    bad.write_text("waypoints:\n  - {name: a, x: 1}\n")          # y eksik
    with pytest.raises(ValueError):
        load_mission(str(bad))
