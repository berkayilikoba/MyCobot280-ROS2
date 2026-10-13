#!/usr/bin/env python3
"""Kapsama turu (PC, Humble): dikdörtgen kenarları + her köşede yerinde 360° dönüş (VLMaps: tüm duvarlar, çok açı).

Odom (/odom_duz) ile sürülür; komut cmd_vel_smoothed -> collision_monitor (Nav2 açık olmalı).
Robot başlangıç işaretinde, ilk kenara bakarak başlar; köşede önce 360° (--spin_w), sonra sola 90°.
Kullanım: kapsama_turu.py [--kenarlar 2,1,2,1] [--hiz 0.15] [--spin_w 0.5]
"""

import argparse
import math
import time

import rclpy

from duvar_testi import Duvar, wrap


def duz_git(n, uzunluk, hiz):
    x0, y0, yaw0 = n.poz
    t0 = time.time()
    while time.time() - t0 < uzunluk / hiz * 3 + 10:
        rclpy.spin_once(n, timeout_sec=0.02)
        ileri = (n.poz[0] - x0) * math.cos(yaw0) + (n.poz[1] - y0) * math.sin(yaw0)
        kalan = uzunluk - ileri
        if kalan < 0.005:
            break
        n.gonder(max(0.03, min(hiz, 0.8 * kalan)), 1.5 * wrap(yaw0 - n.poz[2]))
    n.dur(0.5)


def spin(n, w):
    top, onceki, t0 = 0.0, n.poz[2], time.time()
    while top < 2 * math.pi - math.radians(1) and time.time() - t0 < 2 * math.pi / w * 2.5 + 5:
        kalan = 2 * math.pi - top
        n.gonder(0, max(0.15, min(w, 2.0 * kalan)))
        rclpy.spin_once(n, timeout_sec=0.02)
        top += abs(wrap(n.poz[2] - onceki))
        onceki = n.poz[2]
    n.dur(0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--kenarlar', default='2,1,2,1')
    ap.add_argument('--hiz', type=float, default=0.15)
    ap.add_argument('--spin_w', type=float, default=0.5)
    ap.add_argument('--donus', type=float, default=0.4)
    ap.add_argument('--cmd_topic', default='cmd_vel_smoothed')
    a = ap.parse_args()
    a.geri_max, a.bekle = 0.0, 0.0
    rclpy.init()
    n = Duvar(a)
    n.bekle_veri()
    xs, ys, yaws = n.poz
    for i, L in enumerate(float(x) for x in a.kenarlar.split(',')):
        hedef_yaw = wrap(yaws + i * math.pi / 2)
        n.don(hedef_yaw)
        duz_git(n, L, a.hiz)
        spin(n, a.spin_w)
        n.get_logger().info(f'kenar {i + 1} ({L} m) + 360° bitti: x {n.poz[0] - xs:+.3f} y {n.poz[1] - ys:+.3f}')
    n.don(yaws)
    n.get_logger().info(f'tur bitti: başlangıca fark dx {n.poz[0] - xs:+.3f} dy {n.poz[1] - ys:+.3f} '
                        f'dyaw {math.degrees(wrap(n.poz[2] - yaws)):+.1f}°')
    n.dur()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
