#!/usr/bin/env python3
"""Gerçek kayıttaki /cmd_vel'i sime oynatır; sim tepkisini (odom v, IMU w) kaydeder -> gerçekle karşılaştırma.
Kullanım: sim_ikiz.py <cikti.npz> <bag_dizini> <baslangic_s> <sure_s>"""
import subprocess, sys, time
import numpy as np, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
out, bag, bas, sure = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
rclpy.init(); n = rclpy.create_node('sim_ikiz')   # duvar saati: bag de duvar saatinde oynuyor
C, O, I = [], [], []
n.create_subscription(Twist, '/cmd_vel', lambda m: C.append((time.time(), m.linear.x, m.angular.z)), 50)
n.create_subscription(Odometry, '/odom', lambda m: O.append((time.time(), m.twist.twist.linear.x)), 50)
n.create_subscription(Imu, '/imu', lambda m: I.append((time.time(), m.angular_velocity.z)), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 3: rclpy.spin_once(n, timeout_sec=0.05)
p = subprocess.Popen(['ros2', 'bag', 'play', bag, '--topics', '/cmd_vel', '--start-offset', str(bas)],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
t0 = time.time()
while time.time() - t0 < sure: rclpy.spin_once(n, timeout_sec=0.01)
np.savez(out, cmd=np.array(C), odom=np.array(O), imu=np.array(I))
p.terminate()
try:
    p.wait(5)
except subprocess.TimeoutExpired:
    p.kill()
print(f'kaydedildi: cmd {len(C)} odom {len(O)} imu {len(I)}')
