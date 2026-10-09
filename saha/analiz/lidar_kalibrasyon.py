#!/usr/bin/env python3
"""Lidarın base_link'e göre konumu ve yönü (dış kalibrasyon), sürüş kayıtlarından — robota dokunmadan.

  yaw : düz gidişte (vx > 0.1, |wz| < 0.03) ardışık taramalar ICP ile eşlenir; robot base_link x'i boyunca
        ilerlediği için taramanın lidar çerçevesindeki kayma yönü = -yaw.
  x, y: yerinde dönüşlerde (|wz| > 0.05, |vx| < 0.03) dönüş öncesi/sonrası tarama ICP; taban saf dönerse
        lidar ötelemesi t = R_l^T (R(θ) - I) p  ->  p (en küçük kareler, önyükleme ile %90 aralık).
        Not: p dönme merkezine göredir (4 teker kayar; geometrik merkezden birkaç cm farklı olabilir).

Kullanım (Humble): lidar_kalibrasyon.py <bag> [<bag> ...] [--odom /odom_duz]
  bag'lerde /scan ve odom (tercihen /odom_duz) olmalı; içinde yerinde dönüş ve düz gidiş olan kayıtlar
  (ör. saha/dikdortgen_tur.sh kayıtları). 09.10.2026, 3 dikdörtgen tur: yaw -9.66°, (0.137, 0.015) m.
"""

import argparse
import math
import os

import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'icp_fonk.py')).read())   # icp()


def oku(uri, odom_konu):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/scan', odom_konu]))
    O, T, P = [], [], []
    while r.has_next():
        konu, veri, _ = r.read_next()
        if konu == '/scan':
            m = deserialize_message(veri, LaserScan)
            rr = np.array(m.ranges)
            a = m.angle_min + np.arange(len(rr)) * m.angle_increment
            k = np.isfinite(rr) & (rr > 0.25) & (rr < 8)       # 0.25 m altı: gövde/kol yansıması
            T.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            P.append(np.c_[rr[k] * np.cos(a[k]), rr[k] * np.sin(a[k])])
        else:
            m = deserialize_message(veri, Odometry)
            q = m.pose.pose.orientation
            O.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                      math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)),
                      m.twist.twist.linear.x, m.twist.twist.angular.z))
    O = np.array(O)
    O = O[np.argsort(O[:, 0])]
    O[:, 1] = np.unwrap(O[:, 1])
    return O, np.array(T), P


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', nargs='+')
    ap.add_argument('--odom', default='/odom_duz')
    a = ap.parse_args()
    veri = [oku(b, a.odom) for b in a.bag]

    # ---- yaw: düz gidiş ----
    acilar = []
    for O, T, P in veri:
        for i in range(0, len(T) - 4, 2):
            j = i + 4
            m = (O[:, 0] > T[i]) & (O[:, 0] < T[j])
            if m.sum() < 5 or not (np.all(O[m, 2] > 0.1) and np.all(np.abs(O[m, 3]) < 0.03)):
                continue
            r = icp(P[i], P[j], np.mean(O[m, 2]) * (T[j] - T[i]), 0.0, 0.0)
            if r is None or r[4] < 0.7 or math.hypot(r[0], r[1]) < 0.05:
                continue
            acilar.append(math.degrees(math.atan2(r[1], r[0])))
    acilar = np.array(acilar)
    if len(acilar) < 10:
        raise SystemExit(f'yeterli düz gidiş yok ({len(acilar)} çift)')
    yaw = -np.median(acilar)
    print(f'yaw: {yaw:+.2f}° (düz gidiş çifti {len(acilar)}, IQR {-np.percentile(acilar, 75):+.2f}..'
          f'{-np.percentile(acilar, 25):+.2f}°)')

    # ---- konum: yerinde dönüş ----
    yl = math.radians(yaw)
    cl, sl = math.cos(yl), math.sin(yl)
    A, b = [], []
    for O, T, P in veri:
        don = (np.abs(O[:, 3]) > 0.05) & (np.abs(O[:, 2]) < 0.03)
        bas = None
        for k, f in enumerate(np.r_[don, False]):
            if f and bas is None:
                bas = k
            elif not f and bas is not None:
                if O[k - 1, 0] - O[bas, 0] > 1.0:
                    i = np.searchsorted(T, O[bas, 0] - 0.4) - 1
                    j = np.searchsorted(T, O[k - 1, 0] + 0.6)
                    if i >= 0 and j < len(T):
                        dth = np.interp(T[j], O[:, 0], O[:, 1]) - np.interp(T[i], O[:, 0], O[:, 1])
                        best = None
                        for gx in (-0.2, -0.1, 0, 0.1, 0.2):
                            for gy in (-0.2, -0.1, 0, 0.1, 0.2):
                                r = icp(P[i], P[j], gx, gy, dth)
                                if r and (best is None or r[4] > best[4]):
                                    best = r
                        if best and best[4] > 0.45:
                            x, y, th = best[:3]
                            c, s = math.cos(th), math.sin(th)
                            A += [[c - 1, -s], [s, c - 1]]
                            b += [cl * x - sl * y, sl * x + cl * y]
                bas = None
    A, b = np.array(A), np.array(b)
    n = len(b) // 2
    if n < 3:
        raise SystemExit(f'yeterli yerinde dönüş yok ({n})')
    p = np.linalg.lstsq(A, b, rcond=None)[0]
    rng = np.random.default_rng(0)
    ps = []
    for _ in range(500):
        idx = rng.integers(0, n, n)
        ii = np.ravel([[2 * k, 2 * k + 1] for k in idx])
        ps.append(np.linalg.lstsq(A[ii], b[ii], rcond=None)[0])
    ps = np.array(ps)
    print(f'konum: x {p[0]:+.3f} m ({np.percentile(ps[:, 0], 5):+.3f}..{np.percentile(ps[:, 0], 95):+.3f}), '
          f'y {p[1]:+.3f} m ({np.percentile(ps[:, 1], 5):+.3f}..{np.percentile(ps[:, 1], 95):+.3f})  '
          f'[%90 aralık, {n} dönüş, artık RMS {np.sqrt(np.mean((b - A @ p) ** 2)) * 100:.1f} cm — teker kayması dahil]')
    print(f'TF: ros2 run tf2_ros static_transform_publisher --x {p[0]:.3f} --y {p[1]:.3f} --z 0.02 '
          f'--yaw {math.radians(yaw):.4f} --frame-id base_link --child-frame-id laser_frame')


if __name__ == '__main__':
    main()
