#!/usr/bin/env python3
"""Haritalama icin ileri-geri surus: en fazla MESAFE ileri, sonra baslangica geri.

Guvenlik: lidarla hareket yonunde govdenin 0,25 m onunde engel -> dur;
her hareketin zaman siniri var; cikista her durumda sifir komutu.
Lidar 0,20 m icini (robotun kendi govdesi/kolu) yok sayar.
"""
import math, time, sys
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

MESAFE = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
V = 0.10
GOVDE, ENGEL = 0.20, 0.25

rclpy.init()
n = rclpy.create_node('limo_ileri_geri')
pub = n.create_publisher(Twist, '/cmd_vel', 10)
son = {'odom': None, 'scan': None}
n.create_subscription(Odometry, '/odom', lambda m: son.__setitem__('odom', m), 10)
n.create_subscription(LaserScan, '/scan', lambda m: son.__setitem__('scan', m), qos_profile_sensor_data)


def spin(s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.01)


def xy():
    p = son['odom'].pose.pose.position
    return np.array([p.x, p.y])


def en_yakin(yon):
    m = son['scan']
    r = np.array(m.ranges, dtype=float)
    a = m.angle_min + np.arange(len(r)) * m.angle_increment
    ok = np.isfinite(r) & (r > GOVDE) & (r < m.range_max)
    x, y = r * np.cos(a), r * np.sin(a)
    serit = ok & (np.abs(y) < 0.15) & (yon * x > 0)
    return float((yon * x[serit]).min()) if serit.any() else float('inf')


def dur():
    for _ in range(15):
        pub.publish(Twist()); spin(0.05)


def git(hedef, yon, sinir):
    bas = xy(); t0 = time.time(); cmd = Twist(); cmd.linear.x = yon * V
    while True:
        spin(0.05)
        d = float(np.hypot(*(xy() - bas)))
        yakin = en_yakin(yon)
        if d >= hedef: sebep = 'hedef'; break
        if time.time() - t0 > sinir: sebep = 'zaman'; break
        if yakin < GOVDE + ENGEL: sebep = f'ENGEL {yakin:.2f} m'; break
        pub.publish(cmd)
    dur(); spin(1.0)
    return sebep, round(float(np.hypot(*(xy() - bas))), 3)


try:
    t = time.time()
    while (son['odom'] is None or son['scan'] is None) and time.time() - t < 15:
        spin(0.2)
    if son['odom'] is None or son['scan'] is None:
        print('odom veya scan yok, hareket yok'); sys.exit(1)
    spin(2.0)                              # SLAM ilk taramayi alsin
    print(f'on bosluk {en_yakin(+1):.2f} m, arka bosluk {en_yakin(-1):.2f} m (govdeden itibaren -{GOVDE})')
    s1, d1 = git(MESAFE, +1, MESAFE / V + 4); print('ileri', s1, d1)
    spin(1.0)
    s2, d2 = git(d1, -1, d1 / V + 4); print('geri ', s2, d2)
finally:
    dur()
