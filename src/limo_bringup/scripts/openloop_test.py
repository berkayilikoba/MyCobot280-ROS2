#!/usr/bin/env python3
"""Açık çevrim araç testi: /cmd_vel'e sabit komutlar verir, /odom (Gazebo gerçek
konumu) ile gerçekleşeni ölçer. Nav2 çalışmıyor olmalı (yalnız Gazebo + robot).

Senaryolar (her biri robotun o anki konumundan):
  circle_full   v=0.3, w=1.0  -> tam kilit çember yarıçapı
  circle_r06    v=0.3, w=0.5  -> 0.6 m komutunun gerçekleşen yarıçapı
  step_05       v=0.5, w=0    -> hızın %90'ına varış süresi
  creep_005     v=0.05, w=0   -> düşük hızda hareket (deadband)
  reverse_02    v=-0.2, w=0   -> geri gidiş
Kullanım: openloop_test.py --out sonuc.json
"""

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def fit_circle(pts):
    """Kasa yöntemi: en küçük kareler çember uydurma. (r, cx, cy) döner."""
    n = len(pts)
    sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts)
    mx, my = sx / n, sy / n
    u = [p[0] - mx for p in pts]; v = [p[1] - my for p in pts]
    suu = sum(a * a for a in u); svv = sum(b * b for b in v); suv = sum(a * b for a, b in zip(u, v))
    suuu = sum(a ** 3 for a in u); svvv = sum(b ** 3 for b in v)
    suvv = sum(a * b * b for a, b in zip(u, v)); svuu = sum(b * a * a for a, b in zip(u, v))
    det = suu * svv - suv * suv
    if abs(det) < 1e-12:
        return float('inf'), 0.0, 0.0
    uc = (0.5 * (suuu + suvv) * svv - 0.5 * (svvv + svuu) * suv) / det
    vc = (0.5 * (svvv + svuu) * suu - 0.5 * (suuu + suvv) * suv) / det
    r = math.sqrt(uc * uc + vc * vc + (suu + svv) / n)
    return r, uc + mx, vc + my


class T(Node):
    def __init__(self):
        super().__init__('openloop_test', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = []           # (t, x, y, vx_body)
        self.create_subscription(Odometry, '/odom', self._odom, 50)

    def _odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.odom.append((t, m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.x))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def drive(self, v, w, dur):
        """dur saniye (sim) komut ver; o aralığın odom örneklerini döndür."""
        msg = Twist(); msg.linear.x = float(v); msg.angular.z = float(w)
        t0 = self.now(); k0 = len(self.odom)
        while self.now() - t0 < dur:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.odom[k0:], t0

    def stop(self, dur=2.0):
        self.drive(0.0, 0.0, dur)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True)
    ap.add_argument('--order', default='circle_full,circle_r06,step_05,creep_005,reverse_02')
    a = ap.parse_args()
    rclpy.init(); n = T()
    t0 = time.monotonic()
    while (not n.odom or n.now() == 0.0) and time.monotonic() - t0 < 90:
        rclpy.spin_once(n, timeout_sec=0.2)
    res = {}
    n.stop(2.0)

    def circle(name, v, w, dur=14.0):
        s, _ = n.drive(v, w, dur)
        pts = [(x, y) for (_, x, y, _) in s[len(s) // 4:]]  # ilk çeyrek: direksiyonun oturması
        r, _, _ = fit_circle(pts)
        spd = [abs(q[3]) for q in s[len(s) // 4:]]
        res[name] = {'cmd_v': v, 'cmd_w': w, 'cmd_radius': round(v / w, 3), 'radius_m': round(r, 3),
                     'mean_speed': round(sum(spd) / len(spd), 3) if spd else None}
        n.stop(3.0)

    def straight(name, v, dur=6.0):
        s, tstart = n.drive(v, 0.0, dur)
        d = math.hypot(s[-1][1] - s[0][1], s[-1][2] - s[0][2]) if len(s) > 1 else 0.0
        t90 = next((t - tstart for (t, _, _, vx) in s if abs(vx) >= 0.9 * abs(v)), None)
        res[name] = {'cmd_v': v, 'distance_m': round(d, 3), 'expected_m': round(abs(v) * dur, 3),
                     't_to_90pct_s': round(t90, 2) if t90 is not None else None,
                     'final_speed': round(s[-1][3], 3) if s else None}
        n.stop(2.0)

    SCEN = {
        'circle_full': lambda: circle('circle_full', 0.3, 1.0),
        'circle_r06': lambda: circle('circle_r06', 0.3, 0.5),
        'step_05': lambda: straight('step_05', 0.5),
        'fwd_03': lambda: straight('fwd_03', 0.3),
        'fwd_03_after': lambda: straight('fwd_03_after', 0.3),
        'creep_005': lambda: straight('creep_005', 0.05),
        'reverse_02': lambda: straight('reverse_02', -0.2),
        'half_lock_03': lambda: circle('half_lock_03', 0.3, 0.3),
    }
    for key in a.order.split(','):
        if key.startswith('c:'):          # c:<v>:<w> -> genel çember testi
            _, v, w = key.split(':')
            circle(key, float(v), float(w))
            print(key, json.dumps(res[key]), flush=True)
            continue
        SCEN[key]()
        print(key, json.dumps(res[key]), flush=True)

    json.dump(res, open(a.out, 'w'), indent=2)
    print(json.dumps(res, indent=2))
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
