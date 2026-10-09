#!/usr/bin/env python3
"""bag2vlmaps.py çıktısının doğruluk kontrolü (rapor + şekil).

  1) Derinlik-renk hizası: renk görüntüsündeki kenarların hizalı derinlik süreksizliklerine uzaklığı.
  2) Zemin: dünyaya taşınan zemin noktalarının yüksekliği (tüm tur boyunca ~0 olmalı) -> poz + roll/pitch.
  3) Kamera-lidar: aynı pozlarla birleştirilen derinlik dilimi (yerden 0.1-0.5 m) ile lidar taramalarının
     üst üste oturması; kamera yaw'ı için arama (en iyi uyan yaw ofseti).
  4) Odom kayması: turun ilk ve son durağan lidar taramaları arası ICP (başlangıca dönüş hatası).

Kullanım (Humble): vlmaps_dogrula.py <sahne_dizini> <sensor_bag> [--her 10]
Lidar dış kalibrasyonu (base_link'e göre) 09.10 kayıtlarından ölçüldü: (0.137, 0.015) m, yaw -9.66°
(fabrika TF'i (0, 0, 0.02) ve yaw 0 YANLIŞ). Değiştirmek için --lidar x y yaw_derece.
"""

import argparse
import math
import os

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan

exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'icp_fonk.py')).read())   # icp()


def q2R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pozlar(yol):
    P = np.loadtxt(yol)
    out = []
    for p in P:
        M = np.eye(4)
        M[:3, :3] = q2R(*p[3:7])
        M[:3, 3] = p[:3]
        out.append(M)
    return out


def taramalar(uri):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
    out = []
    while r.has_next():
        m = deserialize_message(r.read_next()[1], LaserScan)
        rr = np.array(m.ranges)
        a = m.angle_min + np.arange(len(rr)) * m.angle_increment
        k = np.isfinite(rr) & (rr > 0.25) & (rr < 6)
        out.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, np.c_[rr[k] * np.cos(a[k]), rr[k] * np.sin(a[k])]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sahne')
    ap.add_argument('sensor_bag')
    ap.add_argument('--her', type=int, default=10)
    ap.add_argument('--lidar', type=float, nargs=3, default=[0.137, 0.015, -9.66])
    a = ap.parse_args()
    S = a.sahne
    K = np.array([[float(v) for v in open(os.path.join(S, 'vlmaps_config.yaml')).read()
                   .split('cam_calib_mat:')[1].strip(' []\n').split(',')]]).reshape(3, 3)
    Pk, Pb = pozlar(os.path.join(S, 'poses_kamera.txt')), pozlar(os.path.join(S, 'poses.txt'))
    zt = np.loadtxt(os.path.join(S, 'zaman.txt'))
    N = len(Pk)
    rapor = []

    # ---- 1) derinlik-renk hizası ----
    hiz = []
    for i in range(0, N, max(1, N // 40)):
        rgb = cv2.imread(os.path.join(S, 'rgb', f'{i:06d}.png'), cv2.IMREAD_GRAYSCALE)
        d = np.load(os.path.join(S, 'depth', f'{i:06d}.npy'))
        # yalnız iki tarafı da geçerli ölçüm olan gerçek süreksizlikler: delik (0) sınırları ve görüş alanı kenarı hariç
        gecerli = cv2.erode((d > 0).astype(np.uint8), np.ones((5, 5))) > 0
        dmax, dmin = cv2.dilate(d, np.ones((3, 3))), cv2.erode(np.where(d > 0, d, 99).astype(np.float32), np.ones((3, 3)))
        dk = (gecerli & (dmax - dmin > 0.15 * dmin) & (dmax - dmin > 0.10)).astype(np.uint8)   # >%15 ve >10 cm sıçrama
        rk = cv2.Canny(cv2.GaussianBlur(rgb, (5, 5), 0), 50, 150)
        if dk.sum() < 200 or rk.sum() == 0:
            continue
        uz = cv2.distanceTransform(1 - (rk > 0).astype(np.uint8), cv2.DIST_L2, 3)
        hiz.append((np.median(uz[dk > 0]), np.mean(uz[dk > 0] <= 3)))
        if i == 0 or len(hiz) == 10:
            kar = cv2.cvtColor(rgb, cv2.COLOR_GRAY2BGR)
            kar[dk > 0] = (0, 0, 255)
            cv2.imwrite(os.path.join(S, f'dogrulama_hiza_{i:06d}.png'), kar)
    hiz = np.array(hiz)
    rapor.append(f'1) derinlik-renk hizası ({len(hiz)} kare): derinlik kenarının en yakın renk kenarına uzaklığı '
                 f'medyan {np.median(hiz[:, 0]):.1f} px, <=3 px oranı {np.median(hiz[:, 1]):.2f}')

    # ---- 2) zemin + derinlik dilimi (dünya) ----
    zem, dilim = [], []
    rng = np.random.default_rng(0)
    for i in range(0, N, a.her):
        d = np.load(os.path.join(S, 'depth', f'{i:06d}.npy'))
        v, u = np.nonzero((d > 0.3) & (d < 3.0))
        if len(u) > 4000:
            s = rng.choice(len(u), 4000, replace=False)
            v, u = v[s], u[s]
        z = d[v, u]
        X = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)])
        W = Pk[i] @ X
        zem.append((i, np.median(W[2][W[2] < 0.05]) if np.any(W[2] < 0.05) else np.nan))   # dünya z=0 zemin
        m = (W[2] > 0.10) & (W[2] < 0.50)
        dilim.append(W[:2, m].T)
    zem = np.array(zem)
    zz = zem[:, 1]
    rapor.append(f'2) zemin yüksekliği (yerden, olması gereken 0): medyan {np.nanmedian(zz) * 100:+.1f} cm, '
                 f'tur boyunca aralık {np.nanpercentile(zz, 5) * 100:+.1f}..{np.nanpercentile(zz, 95) * 100:+.1f} cm (%5-%95)')

    # ---- 3) kamera-lidar ----
    sc = taramalar(a.sensor_bag)
    lx, ly, lyaw = a.lidar[0], a.lidar[1], math.radians(a.lidar[2])
    Tb = np.array([p[:3, 3] for p in Pb])
    yawb = np.array([math.atan2(p[1, 0], p[0, 0]) for p in Pb])
    lid = []
    for t, pts in sc[::2]:
        i = int(np.argmin(np.abs(zt[:, 1] - t)))
        if abs(zt[i, 1] - t) > 0.1:
            continue
        c, s = math.cos(yawb[i] + lyaw), math.sin(yawb[i] + lyaw)
        cb, sb = math.cos(yawb[i]), math.sin(yawb[i])
        ox, oy = Tb[i, 0] + cb * lx - sb * ly, Tb[i, 1] + sb * lx + cb * ly
        lid.append(np.c_[ox + c * pts[:, 0] - s * pts[:, 1], oy + s * pts[:, 0] + c * pts[:, 1]])
    L = np.vstack(lid)
    agac = cKDTree(L)
    D = np.vstack(dilim)

    def uyum(dyaw_deg):
        """kamera yaw'ına dyaw eklenirse derinlik dilimi lidar'a ne kadar oturur (her kare kendi kamera merkezi etrafında)."""
        out = []
        idx = list(range(0, N, a.her))
        for k, i in enumerate(idx):
            pts = dilim[k]
            if len(pts) == 0:
                continue
            cx, cy = Pk[i][0, 3], Pk[i][1, 3]
            c, s = math.cos(math.radians(dyaw_deg)), math.sin(math.radians(dyaw_deg))
            q = np.c_[cx + c * (pts[:, 0] - cx) - s * (pts[:, 1] - cy), cy + s * (pts[:, 0] - cx) + c * (pts[:, 1] - cy)]
            out.append(agac.query(q)[0])
        e = np.concatenate(out)
        return np.median(e), np.mean(e < 0.05)

    u0 = uyum(0.0)
    tarama = [(dy, *uyum(dy)) for dy in np.arange(-6, 6.01, 0.5)]
    en = min(tarama, key=lambda r: r[1])
    rapor.append(f'3) kamera-lidar: derinlik dilimi noktalarının lidar haritasına uzaklığı medyan {u0[0] * 100:.1f} cm, '
                 f'<5 cm oranı {u0[1]:.2f} (kamera yaw 0). En iyi kamera yaw ofseti {en[0]:+.1f}° -> '
                 f'medyan {en[1] * 100:.1f} cm, <5 cm {en[2]:.2f}')

    # ---- 4) başlangıca dönüş (ilk/son lidar taraması) ----
    t0, t1 = zt[0, 1], zt[-1, 1]
    a0 = min(range(len(sc)), key=lambda k: abs(sc[k][0] - t0))
    a1 = min(range(len(sc)), key=lambda k: abs(sc[k][0] - t1))
    dth = yawb[-1] - yawb[0]
    best = None
    for gx in (-0.1, 0, 0.1):
        for gy in (-0.1, 0, 0.1):
            r = icp(sc[a0][1], sc[a1][1], gx, gy, dth)
            if r and (best is None or r[4] > best[4]):
                best = r
    x, y, th, _, fit = best
    # lidar çerçevesindeki değişimi gövdeye çevir (yaw ofseti + konum)
    c, s = math.cos(lyaw), math.sin(lyaw)
    Tl = np.array([[c, -s, lx], [s, c, ly], [0, 0, 1]])
    Ml = np.array([[math.cos(th), -math.sin(th), x], [math.sin(th), math.cos(th), y], [0, 0, 1]])
    Mb = Tl @ Ml @ np.linalg.inv(Tl)
    odom_d = np.linalg.inv(Pb[0]) @ Pb[-1]
    rapor.append(f'4) başlangıca dönüş (ilk-son kare): gerçek (lidar ICP, uyum {fit:.2f}) '
                 f'{math.hypot(Mb[0, 2], Mb[1, 2]) * 100:.1f} cm / {math.degrees(math.atan2(Mb[1, 0], Mb[0, 0])):+.1f}°; '
                 f'odom {math.hypot(odom_d[0, 3], odom_d[1, 3]) * 100:.1f} cm / '
                 f'{math.degrees(math.atan2(odom_d[1, 0], odom_d[0, 0])):+.1f}° -> poz hatası tur sonunda ~'
                 f'{math.hypot(Mb[0, 2] - odom_d[0, 3], Mb[1, 2] - odom_d[1, 3]) * 100:.1f} cm')

    # ---- şekil: üstten görünüş ----
    lo, hi = np.minimum(L.min(0), D.min(0)) - 0.2, np.maximum(L.max(0), D.max(0)) + 0.2
    olc = 100                                                      # px/m
    Wd, Hd = int((hi[0] - lo[0]) * olc), int((hi[1] - lo[1]) * olc)
    img = np.full((Hd, Wd, 3), 255, np.uint8)
    pix = lambda P: (((P[:, 0] - lo[0]) * olc).astype(int), (Hd - 1 - (P[:, 1] - lo[1]) * olc).astype(int))
    for P, renk in ((L, (0, 0, 0)), (D, (0, 140, 255))):
        x_, y_ = pix(P)
        ok = (x_ >= 0) & (x_ < Wd) & (y_ >= 0) & (y_ < Hd)
        img[y_[ok], x_[ok]] = renk
    x_, y_ = pix(Tb[:, :2])
    for k in range(1, len(x_)):
        cv2.line(img, (x_[k - 1], y_[k - 1]), (x_[k], y_[k]), (200, 0, 0), 1)
    cv2.putText(img, 'siyah: lidar  turuncu: derinlik (0.1-0.5 m)  mavi: odom yolu', (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
    cv2.imwrite(os.path.join(S, 'dogrulama_ustten.png'), img)

    metin = '\n'.join(rapor)
    open(os.path.join(S, 'dogrulama.txt'), 'w').write(metin + '\n')
    print(metin)


if __name__ == '__main__':
    main()
