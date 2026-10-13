#!/usr/bin/env python3
"""Yerinde dönüş testi (PC, Humble): verilen açısal hızlarda 360° dönüşler (renk-derinlik eşlemesi, bulanıklık).

Komut cmd_vel_smoothed -> collision_monitor (Nav2 açık olmalı). Açı /odom_duz'dan birikimli.
Kullanım: donus_testi.py [--hizlar 0.3,-0.8] [--aci 360] [--bekle 2]   (+ sol, - sağ; rad/s)
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Donus(Node):
    def __init__(self, topic):
        super().__init__('donus_testi')
        self.pub = self.create_publisher(Twist, topic, 10)
        self.yaw = None
        self.create_subscription(Odometry, '/odom_duz', self.cb, 20)

    def cb(self, m):
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

    def gonder(self, w):
        t = Twist()
        t.angular.z = float(w)
        self.pub.publish(t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hizlar', default='0.3,-0.8')
    ap.add_argument('--aci', type=float, default=360.0)
    ap.add_argument('--bekle', type=float, default=2.0)
    ap.add_argument('--cmd_topic', default='cmd_vel_smoothed')
    a = ap.parse_args()
    rclpy.init()
    n = Donus(a.cmd_topic)
    while n.yaw is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    for w in [float(x) for x in a.hizlar.split(',')]:
        top, onceki, t0 = 0.0, n.yaw, time.time()
        hedef = math.radians(a.aci)
        while top < hedef and time.time() - t0 < hedef / abs(w) * 2.5 + 5:
            kalan = hedef - top
            n.gonder(math.copysign(max(0.15, min(abs(w), 2.0 * kalan)), w))
            rclpy.spin_once(n, timeout_sec=0.02)
            d = math.atan2(math.sin(n.yaw - onceki), math.cos(n.yaw - onceki))
            top += abs(d)
            onceki = n.yaw
        t1 = time.time()
        while time.time() - t1 < a.bekle:
            n.gonder(0.0)
            rclpy.spin_once(n, timeout_sec=0.05)
        n.get_logger().info(f'{w:+.2f} rad/s: {math.degrees(top):.1f}° döndü, {t1 - t0:.1f} s')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
