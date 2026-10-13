#!/usr/bin/env python3
"""Duvar mesafe testi (PC, Humble): robot düz bir duvara bakarak verilen mesafelerde durur (derinlik doğruluğu / gürültü).

Mesafe = duvarın base_link'e uzaklığı, lidardan (/scan_sabit, laser_duz: base_link'e göre x 0.137 m, yön düzeltilmiş),
öndeki ±3° ışınların medyanı. Komut cmd_vel_smoothed -> collision_monitor (Nav2 açık olmalı).
İsteğe bağlı başta yerinde dönüş (--don derece, + sol) ve sonda geri dönüş (başlangıç pozuna, odom ile).
Lidar arkayı görmez: geri gidiş --geri_max (m, başlangıca göre) ile sınırlı.
Her durakta --bekle s durur; durak zamanları --log CSV'ye (analiz: analiz/duvar_analiz.py).

Kullanım: duvar_testi.py --don -90 --mesafeler 0.35,0.5,0.7,1.0,1.3,1.6,2.0 --geri_max 1.4 --log duraklar.csv
"""

import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

LIDAR_X = 0.137


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Duvar(Node):
    def __init__(self, a):
        super().__init__('duvar_testi')
        self.a = a
        self.pub = self.create_publisher(Twist, a.cmd_topic, 10)
        self.poz = None
        self.on = None
        self.create_subscription(Odometry, '/odom_duz', self.odom_cb, 20)
        self.create_subscription(LaserScan, '/scan_sabit', self.scan_cb, qos_profile_sensor_data)

    def odom_cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.poz = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))

    def scan_cb(self, m):
        r = np.array(m.ranges)
        aci = m.angle_min + np.arange(len(r)) * m.angle_increment
        k = (np.abs(np.arctan2(np.sin(aci), np.cos(aci))) < math.radians(3)) & np.isfinite(r) & (r > 0.05)
        self.on = (float(np.median(r[k])) + LIDAR_X, time.time()) if k.sum() >= 3 else None

    def bekle_veri(self):
        t = time.time()
        while (self.poz is None or self.on is None) and time.time() - t < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.poz is None or self.on is None:
            raise RuntimeError('odom_duz / scan_sabit gelmiyor')

    def gonder(self, v=0.0, w=0.0):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.pub.publish(t)

    def dur(self, s=0.5):
        t = time.time()
        while time.time() - t < s:
            self.gonder()
            rclpy.spin_once(self, timeout_sec=0.05)

    def don(self, hedef_yaw):
        t0 = time.time()
        while time.time() - t0 < 30:
            rclpy.spin_once(self, timeout_sec=0.02)
            e = wrap(hedef_yaw - self.poz[2])
            if abs(e) < math.radians(0.7):
                break
            self.gonder(0, max(0.12, min(self.a.donus, 1.5 * abs(e))) * np.sign(e))
        self.dur()

    def mesafeye_git(self, hedef, x0, y0, yaw0):
        """lidar ön mesafesi hedefe gelene kadar ileri/geri (yön yaw0'da tutulur)."""
        t0 = time.time()
        while time.time() - t0 < 40:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.on is None or time.time() - self.on[1] > 0.5:
                self.gonder()
                continue
            e = self.on[0] - hedef                        # + : duvar uzak, ileri git
            geri = -((self.poz[0] - x0) * math.cos(yaw0) + (self.poz[1] - y0) * math.sin(yaw0))
            if e < 0 and geri > self.a.geri_max:
                self.get_logger().warn(f'geri sınırı ({self.a.geri_max} m): hedef {hedef} m atlandı')
                break
            if abs(e) < 0.01:
                break
            v = np.sign(e) * max(0.03, min(self.a.hiz, 0.8 * abs(e)))
            w = 1.5 * wrap(yaw0 - self.poz[2])
            self.gonder(v, w)
        self.dur(0.8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--don', type=float, default=0.0, help='başta yerinde dönüş (derece, + sol)')
    ap.add_argument('--mesafeler', default='0.35,0.5,0.7,1.0,1.3,1.6,2.0')
    ap.add_argument('--bekle', type=float, default=4.0)
    ap.add_argument('--hiz', type=float, default=0.12)
    ap.add_argument('--donus', type=float, default=0.4)
    ap.add_argument('--geri_max', type=float, default=1.4)
    ap.add_argument('--donme', action='store_true', help='sonda başlangıç pozuna dönme')
    ap.add_argument('--cmd_topic', default='cmd_vel_smoothed')
    ap.add_argument('--log', default='duvar_duraklar.csv')
    a = ap.parse_args()
    rclpy.init()
    n = Duvar(a)
    n.bekle_veri()
    xs, ys, yaws = n.poz
    if a.don:
        n.don(wrap(yaws + math.radians(a.don)))
    x0, y0, yaw0 = n.poz
    satirlar = []
    for d in [float(x) for x in a.mesafeler.split(',')]:
        n.mesafeye_git(d, x0, y0, yaw0)
        t1 = time.time()
        olc = []
        while time.time() - t1 < a.bekle:
            n.gonder()
            rclpy.spin_once(n, timeout_sec=0.05)
            if n.on:
                olc.append(n.on[0])
        t2 = time.time()
        m = float(np.median(olc)) if olc else float('nan')
        satirlar.append((d, t1 + 0.5, t2, m))
        n.get_logger().info(f'durak {d:.2f} m: lidar {m:.3f} m ({t2 - t1:.1f} s)')
    if not a.donme:
        # başlangıca dön: önce dönüş yönüne geri/ileri (odom), sonra ilk yön
        t0 = time.time()
        while time.time() - t0 < 40:
            rclpy.spin_once(n, timeout_sec=0.02)
            ileri = (xs - n.poz[0]) * math.cos(yaw0) + (ys - n.poz[1]) * math.sin(yaw0)
            if abs(ileri) < 0.01:
                break
            n.gonder(np.sign(ileri) * max(0.03, min(a.hiz, 0.8 * abs(ileri))), 1.5 * wrap(yaw0 - n.poz[2]))
        n.dur()
        n.don(yaws)
        n.get_logger().info(f'başlangıca dönüldü: dx {n.poz[0] - xs:+.3f} dy {n.poz[1] - ys:+.3f} '
                            f'dyaw {math.degrees(wrap(n.poz[2] - yaws)):+.1f}°')
    with open(a.log, 'w') as f:
        f.write('hedef_m,t_bas,t_bit,lidar_m\n')
        for s in satirlar:
            f.write('%.2f,%.3f,%.3f,%.4f\n' % s)
    n.dur()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
