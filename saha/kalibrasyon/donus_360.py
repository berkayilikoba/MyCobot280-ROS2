#!/usr/bin/env python3
"""Yerinde 360° dönüş (FOUR_DIFF). Güvenlik: tarama halkasında 0.21-0.35 m arası engel -> dur
(0.21 altı robotun kendi gövdesi/kolu). Açı /odom yönünden (IMU) birikimli izlenir; 35 s zaman aşımı.
Kullanım: donus_360.py [derece=360] [hiz_rad_s=0.3]"""
import math, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

HEDEF = math.radians(float(sys.argv[1]) if len(sys.argv) > 1 else 360.0)
W = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
GOVDE, HALKA, ZAMAN = 0.21, 0.35, 35.0
rclpy.init(); n = rclpy.create_node('donus_360')
pub = n.create_publisher(Twist, '/cmd_vel', 10)
st = {'yaw': None, 'scan': None}
n.create_subscription(Odometry, '/odom', lambda m: st.__setitem__('yaw', math.atan2(
    2 * m.pose.pose.orientation.w * m.pose.pose.orientation.z,
    1 - 2 * m.pose.pose.orientation.z ** 2)), 20)
n.create_subscription(LaserScan, '/scan', lambda m: st.__setitem__('scan', m), qos_profile_sensor_data)

def en_yakin():
    m = st['scan']
    v = [r for r in m.ranges if math.isfinite(r) and r > GOVDE]
    return min(v) if v else float('inf')

def dur():
    for _ in range(15):
        pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)

t = time.time()
while (st['yaw'] is None or st['scan'] is None) and time.time() - t < 8:
    rclpy.spin_once(n, timeout_sec=0.2)
if st['yaw'] is None or st['scan'] is None:
    print('odom/scan gelmedi, dönülmedi'); sys.exit(2)
d0 = en_yakin()
print(f'baslangic: en yakin engel {d0:.2f} m (govde {GOVDE} m alti yok sayildi)')
if d0 < HALKA:
    print(f'IPTAL: {HALKA} m icinde engel var'); sys.exit(3)
toplam, onceki, t0, sebep = 0.0, st['yaw'], time.time(), 'tamam'
cmd = Twist(); cmd.angular.z = W
try:
    while abs(toplam) < HEDEF:
        if time.time() - t0 > ZAMAN:
            sebep = 'zaman asimi'; break
        if en_yakin() < HALKA:
            sebep = f'engel {en_yakin():.2f} m'; break
        pub.publish(cmd); rclpy.spin_once(n, timeout_sec=0.05)
        y = st['yaw']; dy = math.atan2(math.sin(y - onceki), math.cos(y - onceki)); onceki = y
        toplam += dy
finally:
    dur()
print(f'donus: {math.degrees(toplam):.1f} derece, {time.time() - t0:.1f} s, sebep: {sebep}')
n.destroy_node(); rclpy.shutdown()
