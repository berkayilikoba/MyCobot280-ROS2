#!/usr/bin/env python3
"""Yavaşça geri git (0.08 m/s). Güvenlik: arka sektörde (|açı|>120°) gövde (0.21 m) ötesinde 0.45 m içinde
engel -> dur. Mesafe /odom'dan. Kullanım: geri_git.py [metre=0.3]"""
import math, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

HEDEF = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
GOVDE, ENGEL, V = 0.21, 0.45, 0.08
rclpy.init(); n = rclpy.create_node('geri_git')
pub = n.create_publisher(Twist, '/cmd_vel', 10)
st = {'xy': None, 'scan': None}
n.create_subscription(Odometry, '/odom', lambda m: st.__setitem__(
    'xy', (m.pose.pose.position.x, m.pose.pose.position.y)), 20)
n.create_subscription(LaserScan, '/scan', lambda m: st.__setitem__('scan', m), qos_profile_sensor_data)

def arka_bosluk():
    m = st['scan']; a = m.angle_min; best = float('inf')
    for r in m.ranges:
        if abs(math.degrees(math.atan2(math.sin(a), math.cos(a)))) > 120 and math.isfinite(r) and r > GOVDE:
            best = min(best, r)
        a += m.angle_increment
    return best

t = time.time()
while (st['xy'] is None or st['scan'] is None) and time.time() - t < 8:
    rclpy.spin_once(n, timeout_sec=0.2)
if st['xy'] is None or st['scan'] is None:
    print('odom/scan gelmedi'); sys.exit(2)
print(f'arka bosluk: {arka_bosluk():.2f} m')
if arka_bosluk() < ENGEL:
    print('IPTAL: arkada engel'); sys.exit(3)
x0, y0 = st['xy']; t0 = time.time(); sebep = 'tamam'
cmd = Twist(); cmd.linear.x = -V
try:
    while math.hypot(st['xy'][0] - x0, st['xy'][1] - y0) < HEDEF:
        if time.time() - t0 > HEDEF / V + 5:
            sebep = 'zaman asimi'; break
        if arka_bosluk() < ENGEL:
            sebep = f'arkada engel {arka_bosluk():.2f} m'; break
        pub.publish(cmd); rclpy.spin_once(n, timeout_sec=0.05)
finally:
    for _ in range(15):
        pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
print(f'geri: {math.hypot(st["xy"][0] - x0, st["xy"][1] - y0):.3f} m, sebep: {sebep}')
