#!/usr/bin/env python3
"""T1 (0,5 m ileri-geri) ve T2 (+90 / -90 yerinde donus) kalibrasyon testi.

Her adimdan once/sonra lidar taramasi ve odometri kaydedilir (npz), sonra
analiz.py ICP ile gercek hareketi odometriyle karsilastirir.

Guvenlik: hareket yonunde 0,25 m'den yakin engel -> dur; her hareketin
zaman siniri var; cikista her durumda sifir komutu gonderilir.
"""
import math, time, sys
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

V_LIN, V_ANG = 0.10, 0.30          # m/s, rad/s
MESAFE, ACI = 0.50, math.radians(90)
ENGEL = 0.25                       # m
GOVDE = 0.20                       # m: lidarin kendi govdesini/kolu gordugu yaricap (olculdu: arka 0,04-0,19 m)
CIKTI = sys.argv[1] if len(sys.argv) > 1 else 'olcum.npz'

rclpy.init()
n = rclpy.create_node('limo_kalibrasyon')
pub = n.create_publisher(Twist, '/cmd_vel', 10)
son = {'odom': None, 'scan': None}
n.create_subscription(Odometry, '/odom', lambda m: son.__setitem__('odom', m), 10)
n.create_subscription(LaserScan, '/scan', lambda m: son.__setitem__('scan', m), qos_profile_sensor_data)


def spin(s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.01)


def poz():
    m = son['odom']
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return np.array([p.x, p.y, 2 * math.atan2(q.z, q.w)])


def tarama():
    m = son['scan']
    r = np.array(m.ranges, dtype=float)
    a = m.angle_min + np.arange(len(r)) * m.angle_increment
    return r, a, m.range_min, m.range_max


def engel_var(yon):
    """yon: +1 ileri, -1 geri, 0 donus (her yonde yarim govde)."""
    r, a, rmin, rmax = tarama()
    ok = np.isfinite(r) & (r > max(rmin, GOVDE)) & (r < rmax)
    x, y = r * np.cos(a), r * np.sin(a)
    if yon == 0:
        yakin = ok & (np.hypot(x, y) < GOVDE + 0.05)
    else:
        yakin = ok & (np.abs(y) < 0.15) & (yon * x > 0) & (yon * x < GOVDE + ENGEL)
    return bool(yakin.any())


def dur():
    for _ in range(15):
        pub.publish(Twist())
        spin(0.05)


kayit = {}


def anlik(ad):
    spin(1.5)                       # durup sensorlerin oturmasini bekle
    r, a, rmin, rmax = tarama()
    kayit[ad + '_odom'] = poz()
    kayit[ad + '_r'] = r
    kayit[ad + '_a'] = a
    kayit['rmin'], kayit['rmax'] = rmin, rmax
    print(f'[{ad}] odom x={kayit[ad+"_odom"][0]:+.3f} y={kayit[ad+"_odom"][1]:+.3f} '
          f'yaw={math.degrees(kayit[ad+"_odom"][2]):+.1f}')


def ilerle(hedef, yon, sinir):
    bas = poz(); t0 = time.time(); cmd = Twist(); cmd.linear.x = yon * V_LIN
    while True:
        spin(0.05)
        d = math.hypot(*(poz()[:2] - bas[:2]))
        if d >= hedef: sebep = 'hedef'; break
        if time.time() - t0 > sinir: sebep = 'zaman'; break
        if engel_var(yon): sebep = 'ENGEL'; break
        pub.publish(cmd)
    dur(); return sebep, d


def don(hedef, yon, sinir):
    bas = poz()[2]; t0 = time.time(); cmd = Twist(); cmd.angular.z = yon * V_ANG
    while True:
        spin(0.05)
        d = abs(math.atan2(math.sin(poz()[2] - bas), math.cos(poz()[2] - bas)))
        if d >= hedef: sebep = 'hedef'; break
        if time.time() - t0 > sinir: sebep = 'zaman'; break
        if engel_var(0): sebep = 'ENGEL'; break
        pub.publish(cmd)
    dur(); return sebep, math.degrees(d)


try:
    t = time.time()
    while (son['odom'] is None or son['scan'] is None) and time.time() - t < 15:
        spin(0.2)
    if son['odom'] is None or son['scan'] is None:
        print('odom veya scan yok, hareket yok'); sys.exit(1)
    adimlar = sys.argv[2] if len(sys.argv) > 2 else 'ileri,geri,don'
    anlik('s0')
    if 'ileri' in adimlar: print('T1 ileri', ilerle(MESAFE, +1, 9.0)); anlik('s1')
    print('T1 geri ', ilerle(MESAFE, -1, 9.0)); anlik('s2')
    print('T2 +90  ', don(ACI, +1, 10.0)); anlik('s3')
    print('T2 -90  ', don(ACI, -1, 10.0)); anlik('s4')
finally:
    dur()
    if kayit:
        np.savez(CIKTI, **kayit); print('kaydedildi', CIKTI)
