#!/usr/bin/env python3
"""Derinlik kamerasının zemine göre yüksekliği ve eğimi (kayıttan, robota dokunmadan).

Her N. derinlik karesinin alt %40'ına RANSAC ile düzlem oturtur; zemin normalinden kameranın
optik merkezinin zeminden yüksekliğini ve ROS base_link -> camera_link roll/pitch'ini verir (yaw ölçülemez).
base_link zemine paralel kabul edilir (2B navigasyon varsayımı), yani sonuç doğrudan TF'e yazılabilir.

Kullanım (Humble):
  zemin_egimi.py <kamera_bag> <sensor_bag> [--her 40] [--taban_z 0.15]
  kamera_bag: /camera/depth/image_raw içeren bag; sensor_bag: /camera/depth/camera_info içeren bag
09.10.2026 sonucu (38 kare): yükseklik 17.8 cm, roll +0.43°, pitch -1.42° (kamera hafif yukarı bakıyor).
"""

import argparse
import math

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image


def oku(uri, konu, ilk=False):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=[konu]))
    while r.has_next():
        yield r.read_next()
        if ilk:
            return


def duzlem(X, rng, deneme=200, esik=0.01):
    """RANSAC + SVD inceltme. Dönüş: (yukarı birim normal, d, iç oran, RMS m)."""
    best = None
    for _ in range(deneme):
        p = X[rng.choice(len(X), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        ic = np.abs(X @ n - n @ p[0]) < esik
        if best is None or ic.sum() > best.sum():
            best = ic
    P = X[best]
    c = P.mean(0)
    n = np.linalg.svd(P - c)[2][2]
    if n[1] > 0:            # optik çerçevede y aşağı: yukarı normal -y yönünde
        n = -n
    d = -n @ c
    return n, d, best.mean(), float(np.sqrt(np.mean((P @ n + d) ** 2)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('kamera_bag')
    ap.add_argument('sensor_bag')
    ap.add_argument('--her', type=int, default=40, help='her N. kare')
    ap.add_argument('--taban_z', type=float, default=0.15, help='base_link yerden yükseklik (URDF)')
    a = ap.parse_args()

    ci = deserialize_message(next(oku(a.sensor_bag, '/camera/depth/camera_info', ilk=True))[1], CameraInfo)
    fx, fy, cx, cy = ci.k[0], ci.k[4], ci.k[2], ci.k[5]
    rng = np.random.default_rng(0)
    sonuc = []
    for i, (_, veri, _) in enumerate(oku(a.kamera_bag, '/camera/depth/image_raw')):
        if i % a.her:
            continue
        m = deserialize_message(veri, Image)
        z = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width).astype(float) / 1000.0
        v, u = np.mgrid[0:m.height, 0:m.width]
        s = (z > 0.4) & (z < 2.5) & (v > m.height * 0.6)
        if s.sum() < 500:
            continue
        X = np.c_[(u[s] - cx) * z[s] / fx, (v[s] - cy) * z[s] / fy, z[s]]
        X = X[rng.choice(len(X), min(5000, len(X)), replace=False)]
        n, d, oran, rms = duzlem(X, rng)
        if oran < 0.5:          # karede zemin baskın değil (duvar/engel)
            continue
        # optik çerçevede ölçüm -> ROS base_link->camera_link: pitch + = burun aşağı, roll + = sağ taraf aşağı
        sonuc.append((abs(d), math.degrees(math.asin(-n[2])), -math.degrees(math.atan2(n[0], -n[1])), oran, rms))
    R = np.array(sonuc)
    q = lambda c: (np.median(R[:, c]), np.percentile(R[:, c], 25), np.percentile(R[:, c], 75))
    h, p, r = q(0), q(1), q(2)
    print(f'kare: {len(R)} (zemin iç oranı medyan {np.median(R[:, 3]):.2f}, düzlem RMS {np.median(R[:, 4]) * 1000:.1f} mm)')
    print(f'optik merkez yüksekliği: {h[0] * 100:.1f} cm (IQR {h[1] * 100:.1f}-{h[2] * 100:.1f})')
    print(f'pitch: {p[0]:+.2f}° (IQR {p[1]:+.2f}..{p[2]:+.2f})   roll: {r[0]:+.2f}° (IQR {r[1]:+.2f}..{r[2]:+.2f})')
    print(f'TF önerisi base_link -> camera_link: z = {h[0] - a.taban_z:.3f} m, '
          f'roll = {math.radians(r[0]):+.4f} rad, pitch = {math.radians(p[0]):+.4f} rad')


if __name__ == '__main__':
    main()
