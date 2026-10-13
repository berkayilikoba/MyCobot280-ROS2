#!/usr/bin/env python3
"""duvar_testi.py kaydının analizi: mesafeye göre derinlik sapması, gürültü ve doluluk (D435 ile aynı ölçütler).

Duraklar: /odom_duz'da robotun hareketsiz olduğu, başlangıç yönünden farklı (dönülmüş) yöndeki >=2.5 s aralıklar.
Her durakta:
  lidar_m        base_link -> duvar, lidar tam ön (±3° medyan) + 0.137 (laser_duz x). Lidar σ ~5 mm.
  kamera_m       görüntü merkezi 10x10 piksel derinlik medyanı + kamera x (kamera_kalibrasyon.yaml, CAD 0.084)
  fark           kamera_m - lidar_m (- = kamera daha yakın gösteriyor). Sabit kısım kamera/lidar x ofseti
                 belirsizliğini de içerir; mesafeyle değişen kısım derinlik ölçek hatasıdır.
  duzlem_rms     merkez 120x120 piksele (kamera çerçevesi) uydurulan düzlemden RMS, tek kare, kareler medyanı
  sigma_zaman    merkez piksellerin zamansal std'si (medyan; Dabai 1 mm adımlı: yakında 0 çıkabilir)
  doluluk        merkez 120x120 pikselde geçerli derinlik oranı
(13.10 ilk deneme: lidar ±25° doğru uydurmasıyla duvar açısı mesafeyle 5-12° değişti, güvenilmez; bırakıldı.)
Kullanım (Humble): duvar_analiz.py <kayit_dizini (sensor/ + kamera/)> [--her 3]
"""

import argparse
import math
import os

import numpy as np
import yaml
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_msgs.msg import TFMessage

from bag2vlmaps import T, damga, okuyucu, q2R, rpy2R, zincir

BURADA = os.path.dirname(os.path.abspath(__file__))
LIDAR = (0.137, 0.015, 0.0)          # laser_duz: base_link'e göre (yön zaten düzeltilmiş)
TABAN_Z = 0.15                        # base_link yerden


def duraklar(odom, yaw0, min_s=2.5):
    t, x, y, yaw = odom.T
    v = np.hypot(np.gradient(x, t), np.gradient(y, t))
    w = np.abs(np.gradient(np.unwrap(yaw), t))
    sabit = (v < 0.004) & (w < 0.01)
    out, i = [], 0
    while i < len(t):
        if not sabit[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(t) and sabit[j + 1]:
            j += 1
        dyaw = abs(math.atan2(math.sin(yaw[i] - yaw0), math.cos(yaw[i] - yaw0)))
        if t[j] - t[i] >= min_s and dyaw > math.radians(30):
            out.append((t[i] + 0.3, t[j] - 0.2))
        i = j + 1
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('kayit')
    ap.add_argument('--kalib', default=os.path.join(BURADA, '..', 'config', 'kamera_kalibrasyon.yaml'))
    ap.add_argument('--her', type=int, default=3, help='durak içinde her N. derinlik karesi')
    a = ap.parse_args()
    sb, kb = os.path.join(a.kayit, 'sensor'), os.path.join(a.kayit, 'kamera')

    odom, taramalar, tfs, Kd = [], [], {}, None
    for konu, veri, _ in okuyucu(sb, ['/odom_duz', '/scan_sabit', '/tf_static', '/camera/depth/camera_info']):
        if konu == '/odom_duz':
            m = deserialize_message(veri, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            odom.append((damga(m.header), p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))))
        elif konu == '/scan_sabit':
            m = deserialize_message(veri, LaserScan)
            r = np.array(m.ranges)
            taramalar.append((damga(m.header), m.angle_min + np.arange(len(r)) * m.angle_increment, r))
        elif konu == '/tf_static':
            for tt in deserialize_message(veri, TFMessage).transforms:
                tr, q = tt.transform.translation, tt.transform.rotation
                tfs[(tt.header.frame_id, tt.child_frame_id)] = T(q2R(q.x, q.y, q.z, q.w), [tr.x, tr.y, tr.z])
        elif Kd is None:
            Kd = np.array(deserialize_message(veri, CameraInfo).k).reshape(3, 3)
    odom = np.array(sorted(odom))
    D = duraklar(odom, odom[0, 3])
    print(f'{len(D)} durak bulundu')
    kal = yaml.safe_load(open(a.kalib))['base_link_to_camera_link']
    T_b_do = T(rpy2R(*kal['rotation_rpy_rad']), kal['translation_m']) @ zincir(
        tfs, 'camera_link', 'camera_depth_frame', 'camera_depth_optical_frame')

    # durak başına derinlik kareleri
    kareler = [[] for _ in D]
    sayac = [0] * len(D)
    for _, veri, _ in okuyucu(kb, ['/camera/depth/image_raw']):
        m = deserialize_message(veri, Image)
        t = damga(m.header)
        for i, (t1, t2) in enumerate(D):
            if t1 <= t <= t2:
                sayac[i] += 1
                if sayac[i] % a.her == 0:
                    kareler[i].append(np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(np.float32) / 1000)
    H, W = kareler[0][0].shape
    vv, uu = np.mgrid[0:H, 0:W]
    ray = np.stack([(uu - Kd[0, 2]) / Kd[0, 0], (vv - Kd[1, 2]) / Kd[1, 1], np.ones_like(uu, float)])  # z=1 ışınları
    R_b, t_b = T_b_do[:3, :3], T_b_do[:3, 3]
    ray_b = np.einsum('ij,jhw->ihw', R_b, ray)                       # base_link'te ışın yönleri (z_opt=1 başına)

    sonuclar = []
    print(f'{"lidar_m":>8} {"kamera_m":>8} {"fark_cm":>8} {"fark_%":>7} {"duzlem_rms_mm":>13} {"sigma_zaman_mm":>14} '
          f'{"doluluk":>8} {"kare":>5}')
    for (t1, t2), Zs in zip(D, kareler):
        L = []
        for ts, aci, r in taramalar:
            if t1 <= ts <= t2:
                k = (np.abs(np.arctan2(np.sin(aci), np.cos(aci))) < math.radians(3)) & np.isfinite(r) & (r > 0.05)
                if k.sum() >= 3:
                    L.append(np.median(r[k]))
        if not Zs or not L:
            continue
        d_lidar = float(np.median(L)) + LIDAR[0]                      # base_link -> duvar (tam ön)
        Z = np.stack(Zs)
        merkez = Z[:, H // 2 - 5:H // 2 + 5, W // 2 - 5:W // 2 + 5]
        d_kam = float(np.median(merkez[merkez > 0]) + T_b_do[0, 3])   # optik eksen ~ base_link x
        roi = Z[:, H // 2 - 60:H // 2 + 60, W // 2 - 60:W // 2 + 60]  # merkez 120x120 piksel (duvar)
        doluluk = float((roi > 0).mean())
        Zn = np.where(roi > 0, roi, np.nan)
        sd = np.nanstd(Zn, 0)[(roi > 0).mean(0) > 0.9]
        vv2, uu2 = np.mgrid[H // 2 - 60:H // 2 + 60, W // 2 - 60:W // 2 + 60]
        rms = []
        for Zk in roi:
            g = Zk > 0
            if g.sum() < 500:
                continue
            z = Zk[g]
            Q = np.stack([(uu2[g] - Kd[0, 2]) * z / Kd[0, 0], (vv2[g] - Kd[1, 2]) * z / Kd[1, 1], z], 1)
            c = Q.mean(0)
            _, _, Vt = np.linalg.svd(Q - c, full_matrices=False)
            rms.append(np.sqrt(np.mean(((Q - c) @ Vt[2]) ** 2)))
        s = {'lidar_m': round(d_lidar, 3), 'kamera_m': round(d_kam, 3), 'fark_cm': round((d_kam - d_lidar) * 100, 2),
             'fark_%': round((d_kam - d_lidar) / d_lidar * 100, 2),
             'duzlem_rms_mm': round(float(np.median(rms) * 1000), 2) if rms else None,
             'sigma_zaman_mm': round(float(np.median(sd) * 1000), 2) if len(sd) else None,
             'doluluk': round(doluluk, 3), 'kare': len(Z)}
        sonuclar.append(s)
        print(f'{s["lidar_m"]:8.3f} {s["kamera_m"]:8.3f} {s["fark_cm"]:+8.2f} {s["fark_%"]:+7.2f} '
              f'{s["duzlem_rms_mm"] or 0:13.2f} {s["sigma_zaman_mm"] or 0:14.2f} {s["doluluk"]:8.3f} {s["kare"]:5d}')
    with open(os.path.join(a.kayit, 'duvar_sonuc.yaml'), 'w') as f:
        yaml.safe_dump({'kamera': 'Orbbec Dabai DC1 (varsayılan ayar, 640x400 derinlik)', 'duraklar': sonuclar}, f,
                       allow_unicode=True, sort_keys=False)


if __name__ == '__main__':
    main()
