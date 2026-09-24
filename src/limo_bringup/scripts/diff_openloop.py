#!/usr/bin/env python3
"""Diff sim açık çevrim düzgünlük testi (Nav2 kapalı, boş dünya).

Düz sürüş (v=0.4, w=0, 6 s): yaw hızı std'si ve yanal sapma -> yalpalama ölçüsü.
Dönüş (v=0.3, w=0.6, 5 s) ve yerinde dönüş (w=1.0, 4 s): gerçekleşen/komut yaw hızı oranı.
"""
import json
import math
import statistics as stt
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rclpy.init()
n = rclpy.create_node('diff_openloop', parameter_overrides=[
    rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
pub = n.create_publisher(Twist, '/cmd_vel', 10)
S = []


def od(m):
    q = m.pose.pose.orientation
    S.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y,
              math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z), m.twist.twist.linear.x, m.twist.twist.angular.z))


n.create_subscription(Odometry, '/odom', od, 100)


def now():
    return n.get_clock().now().nanoseconds * 1e-9


while not S or now() == 0:
    rclpy.spin_once(n, timeout_sec=0.2)


def drive(v, w, d):
    m = Twist()
    m.linear.x, m.angular.z = float(v), float(w)
    t0 = now()
    k = len(S)
    while now() - t0 < d:
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.02)
    return S[k:]


res = {}
drive(0, 0, 1)
seg = drive(0.4, 0, 6)[40:]          # ilk ~1.5 s hızlanma
x0, y0, th0 = seg[0][1], seg[0][2], seg[0][3]
lat = [-(x - x0) * math.sin(th0) + (y - y0) * math.cos(th0) for _, x, y, *_ in seg]
res['duz_wz_std'] = round(stt.pstdev([s[5] for s in seg]), 3)
res['duz_vx_ort'] = round(stt.mean([s[4] for s in seg]), 3)
res['duz_yanal_sapma_max_m'] = round(max(abs(v) for v in lat), 3)
drive(0, 0, 1.5)
seg = drive(0.3, 0.6, 5)[50:]
res['donus_wz_oran'] = round(stt.mean([s[5] for s in seg]) / 0.6, 2)
res['donus_wz_std'] = round(stt.pstdev([s[5] for s in seg]), 3)
drive(0, 0, 1.5)
seg = drive(0, 1.0, 4)[50:]
res['yerinde_wz_oran'] = round(stt.mean([s[5] for s in seg]) / 1.0, 2)
drive(0, 0, 1)
print(json.dumps(res), flush=True)
if len(sys.argv) > 1:
    json.dump(res, open(sys.argv[1], 'w'))
