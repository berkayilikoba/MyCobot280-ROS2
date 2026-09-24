#!/usr/bin/env python3
"""Direksiyon basamak yanıtı: sabit hızda eğrilik komutunu basamakla değiştirir ve
gerçekleşen eğriliği (odom yaw hızı / hız) zamanla ölçer. Nav2 çalışmıyor olmalı.

Adımlar (v sabit): düz -> +k -> 0 -> -k. Her basamak için hedef eğriliğin
%63 ve %90'ına varış süresi raporlanır. Boş dünyada çalıştırın.
Kullanım: steer_step.py --out sonuc.json [--v 0.3] [--k 2.0] [--hold 4]
"""

import argparse
import json

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class S(Node):
    def __init__(self):
        super().__init__('steer_step', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.samples = []   # (t, vx, wz)
        self.create_subscription(Odometry, '/odom', self._odom, 100)

    def _odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.samples.append((t, m.twist.twist.linear.x, m.twist.twist.angular.z))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def drive(self, v, w, dur):
        msg = Twist(); msg.linear.x = float(v); msg.angular.z = float(w)
        t0 = self.now()
        while self.now() - t0 < dur:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
        return t0


def step_metrics(samples, t0, t1, k_from, k_to):
    seg = [(t - t0, (wz / vx) if abs(vx) > 0.05 else None) for (t, vx, wz) in samples if t0 <= t < t1]
    span = k_to - k_from
    out = {'k_from': k_from, 'k_to': k_to}
    for frac in (0.63, 0.9):
        thr = k_from + frac * span
        hit = next((dt for dt, k in seg if k is not None and (k - thr) * (1 if span > 0 else -1) >= 0), None)
        out[f't{int(frac * 100)}_s'] = round(hit, 2) if hit is not None else None
    ks = [k for _, k in seg[len(seg) // 2:] if k is not None]
    out['k_final'] = round(sum(ks) / len(ks), 3) if ks else None
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--v', type=float, default=0.3)
    ap.add_argument('--k', type=float, default=2.0)
    ap.add_argument('--hold', type=float, default=4.0)
    a = ap.parse_args()
    rclpy.init(); n = S()
    while not n.samples or n.now() == 0.0:
        rclpy.spin_once(n, timeout_sec=0.2)
    v, k, h = a.v, a.k, a.hold
    n.drive(0.0, 0.0, 1.0)
    n.drive(v, 0.0, 3.0)
    res = {'v': v, 'k_cmd': k, 'steps': []}
    for k_from, k_to in ((0.0, k), (k, 0.0), (0.0, -k), (-k, k)):
        t0 = n.drive(v, v * k_to, h)
        res['steps'].append(step_metrics(n.samples, t0, t0 + h, k_from, k_to))
        print(json.dumps(res['steps'][-1]), flush=True)
    n.drive(0.0, 0.0, 1.0)
    json.dump(res, open(a.out, 'w'), indent=2)
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
