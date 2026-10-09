#!/usr/bin/env python3
"""Durağan robotta sensör istatistikleri (sim ↔ gerçek karşılaştırma). Gerçek bag analiziyle aynı yöntem:
IMU std/ortalama, lidar hız/nokta sayısı/görüş açısı, ışın başına menzil gürültüsü (MAD, menzile göre).
Kullanım: sensor_olc.py <cikti.json> [sure_s=30]"""
import json, math, sys, time
import numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
S = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
rclpy.init(); n = rclpy.create_node('sensor_olc', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
I, L = [], []
n.create_subscription(Imu, '/imu', lambda m: I.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z, m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)), qos_profile_sensor_data)
n.create_subscription(LaserScan, '/scan', lambda m: L.append(m), qos_profile_sensor_data)
t = time.time()
while time.time() - t < 10 and (not I or not L): rclpy.spin_once(n, timeout_sec=0.2)
I.clear(); L.clear(); t0 = n.get_clock().now().nanoseconds * 1e-9
while n.get_clock().now().nanoseconds * 1e-9 - t0 < S: rclpy.spin_once(n, timeout_sec=0.05)
A = np.array(I); r = {}
if len(A):
    r['imu'] = dict(hz=round((len(A) - 1) / (A[-1, 6] - A[0, 6]), 1), gyro_std=[round(v, 5) for v in A[:, :3].std(0)], acc_std=[round(v, 4) for v in A[:, 3:6].std(0)], acc_mean=[round(v, 3) for v in A[:, 3:6].mean(0)])
if L:
    NB = 720; M = []
    for m in L:
        rr = np.array(m.ranges); a = m.angle_min + np.arange(len(rr)) * m.angle_increment
        b = ((np.arctan2(np.sin(a), np.cos(a)) + math.pi) / (2 * math.pi) * NB).astype(int) % NB
        row = np.full(NB, np.nan); ok = np.isfinite(rr) & (rr > m.range_min) & (rr < m.range_max); row[b[ok]] = rr[ok]; M.append(row)
    M = np.array(M); med = np.nanmedian(M, 0); mad = 1.4826 * np.nanmedian(np.abs(M - med), 0); frac = np.isfinite(M).mean(0)
    st = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in L]
    rs = {}
    for lo, hi in [(0.2, 0.5), (0.5, 1), (1, 2), (2, 4), (4, 8)]:
        k = (frac > 0.5) & (med >= lo) & (med < hi)
        if k.sum() > 5: rs[f'{lo}-{hi}m'] = round(1000 * float(np.median(mad[k])), 1)
    m0 = L[-1]
    r['lidar'] = dict(hz=round((len(L) - 1) / (st[-1] - st[0]), 2), nokta=len(m0.ranges), aci=[round(math.degrees(m0.angle_min)), round(math.degrees(m0.angle_max))], menzil=[m0.range_min, m0.range_max], gurultu_mm=rs)
print(json.dumps(r, ensure_ascii=False)); json.dump(r, open(sys.argv[1], 'w'), ensure_ascii=False, indent=1)
