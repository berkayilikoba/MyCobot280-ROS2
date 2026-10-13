#!/usr/bin/env python3
"""Robot kaydını (kayit_baslat.sh: kamera + sensor bag) VLMaps sahne biçimine dönüştürür.

Çıktı (github.com/vlmaps/vlmaps "custom dataset" biçimi):
  <cikti>/rgb/000000.png         renk, bozulması giderilmiş (pinhole), 640x480
  <cikti>/depth/000000.npy       float32 (480, 640), METRE, renk görüntüsüne hizalı; ölçüm yoksa 0
  <cikti>/poses.txt              gövde pozu (base_footprint, mobile_base): px py pz qx qy qz qw — dünya = odom, z=0 zemin, ROS ekseni
  <cikti>/poses_kamera.txt       renk kamerası optik çerçeve pozu (camera_base: x sağ, y aşağı, z ileri), aynı dünya
  <cikti>/zaman.txt              kare_no renk_damgasi derinlik_damgasi (s)
  <cikti>/vlmaps_config.yaml     VLMaps map_config için pose_info + cam_calib_mat
  <cikti>/bilgi.yaml             kaynak, kalibrasyon, atlanan kareler

İşlem:
  - Her renk karesine en yakın derinlik karesi (|dt| <= --esik_ms); aradaki robot hareketi pozla telafi edilir.
  - Derinlik noktaları: derinlik optik -> dünya (derinlik anındaki poz) -> renk optik (renk anındaki poz) -> renk
    iç parametresiyle izdüşüm, en yakın nokta kazanır (z-tamponu).
  - Gövde pozu: /odom_duz (odom->base_link) doğrusal ara değer; çevresinde --bosluk_s'den uzun odom boşluğu olan
    kareler atlanır.
  - base_link -> camera_link: --kalib (varsayılan saha/config/kamera_kalibrasyon.yaml); camera_link -> optik
    çerçeveler kayıttaki /tf_static'ten (Orbbec sürücüsü).

Kullanım (Humble):
  bag2vlmaps.py <kamera_bag> <sensor_bag> <cikti_dizini> [--her 1] [--esik_ms 30]
"""

import argparse
import math
import os

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

BURADA = os.path.dirname(os.path.abspath(__file__))


# ---------- dönüşüm yardımcıları ----------
def q2R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R2q(R):
    """(qx, qy, qz, qw)"""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def rpy2R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def T(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def okuyucu(uri, konular):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=konular))
    while r.has_next():
        yield r.read_next()


def damga(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


# ---------- sensör bag'i: odom, tf_static, camera_info ----------
def sensor_oku(uri):
    odom, tfs, ci = [], {}, {}
    for konu, veri, _ in okuyucu(uri, ['/odom_duz', '/tf_static', '/camera/color/camera_info',
                                       '/camera/depth/camera_info']):
        if konu == '/odom_duz':
            m = deserialize_message(veri, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            odom.append((damga(m.header), p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))))
        elif konu == '/tf_static':
            for t in deserialize_message(veri, TFMessage).transforms:
                tr, q = t.transform.translation, t.transform.rotation
                tfs[(t.header.frame_id, t.child_frame_id)] = T(q2R(q.x, q.y, q.z, q.w), [tr.x, tr.y, tr.z])
        elif konu.split('/')[2] not in ci:
            m = deserialize_message(veri, CameraInfo)
            ci[konu.split('/')[2]] = m
    O = np.array(odom)
    O = O[np.argsort(O[:, 0])]
    O[:, 3] = np.unwrap(O[:, 3])
    return O, tfs, ci


def zincir(tfs, *cerceveler):
    M = np.eye(4)
    for a, b in zip(cerceveler[:-1], cerceveler[1:]):
        if (a, b) not in tfs:
            raise KeyError(f'/tf_static içinde {a} -> {b} yok (kayıtta Orbbec sürücüsü açık mıydı?)')
        M = M @ tfs[(a, b)]
    return M


def poz_ara(O, t, bosluk_s):
    """odom->base_link (4x4) t anında; çevrede uzun boşluk varsa None."""
    j = np.searchsorted(O[:, 0], t)
    if j <= 0 or j >= len(O) or O[j, 0] - O[j - 1, 0] > bosluk_s:
        return None
    f = (t - O[j - 1, 0]) / (O[j, 0] - O[j - 1, 0])
    x, y, yaw = (O[j - 1, 1:] + f * (O[j, 1:] - O[j - 1, 1:]))
    return T(rpy2R(0, 0, yaw), [x, y, 0.0])


# ---------- görüntüler ----------
def goruntu(m):
    if m.encoding == 'rgb8':
        return cv2.cvtColor(np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3), cv2.COLOR_RGB2BGR)
    if m.encoding == 'bgr8':
        return np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).copy()
    if m.encoding in ('16UC1', 'mono16'):
        return np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
    raise ValueError(f'desteklenmeyen kodlama {m.encoding}')


def derinlik_hizala(dmm, Kd, Kc, T_c_d, boyut, zmin=0.2, zmax=8.0):
    """Derinlik (mm, derinlik optik) -> renk optik çerçevesine izdüşüm, metre, z-tamponu (en yakın kazanır)."""
    H, W = dmm.shape
    v, u = np.nonzero((dmm > zmin * 1000) & (dmm < zmax * 1000))
    z = dmm[v, u].astype(np.float64) / 1000.0
    P = np.stack([(u - Kd[0, 2]) * z / Kd[0, 0], (v - Kd[1, 2]) * z / Kd[1, 1], z, np.ones_like(z)])
    Q = T_c_d @ P
    on = Q[2] > 1e-3
    Q = Q[:, on]
    uc = np.round(Kc[0, 0] * Q[0] / Q[2] + Kc[0, 2]).astype(int)
    vc = np.round(Kc[1, 1] * Q[1] / Q[2] + Kc[1, 2]).astype(int)
    Hc, Wc = boyut
    ic = (uc >= 0) & (uc < Wc) & (vc >= 0) & (vc < Hc)
    uc, vc, zc = uc[ic], vc[ic], Q[2, ic]
    sira = np.argsort(-zc)                      # uzaktan yakına yaz: son yazılan (en yakın) kalır
    out = np.zeros((Hc, Wc), np.float32)
    out[vc[sira], uc[sira]] = zc[sira]
    return out


def kenar_suz(dmm, oran):
    """Uçan piksel: 3x3 komşulukta (geçerli komşular) derinlik farkı > oran*z olan pikseli sıfırla."""
    z = dmm.astype(np.float32)
    gec = z > 0
    zb = np.where(gec, z, np.nan)
    mx = cv2.dilate(np.where(gec, z, 0), np.ones((3, 3), np.uint8))
    mn = -cv2.dilate(np.where(gec, -z, -1e9).astype(np.float32), np.ones((3, 3), np.uint8))
    kotu = gec & ((mx - z > oran * z) | (z - mn > oran * z))
    out = dmm.copy()
    out[kotu] = 0
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('kamera_bag')
    ap.add_argument('sensor_bag')
    ap.add_argument('cikti')
    ap.add_argument('--kalib', default=os.path.join(BURADA, '..', 'config', 'kamera_kalibrasyon.yaml'))
    ap.add_argument('--her', type=int, default=1, help='her N. renk karesi')
    ap.add_argument('--esik_ms', type=float, default=30.0, help='renk-derinlik eşleme eşiği')
    ap.add_argument('--bosluk_s', type=float, default=0.25, help='odom boşluğu bundan uzunsa kareyi atla')
    ap.add_argument('--taban_z', type=float, default=0.15, help='base_link yerden yükseklik (URDF)')
    ap.add_argument('--zmax', type=float, default=8.0,
                    help='bundan uzak derinliği at (m). Dabai σ: 1-2 m 4 mm, 2-4 m 28 mm, 4-8 m 89 mm (13.10)')
    ap.add_argument('--kenar', type=float, default=0.0,
                    help='uçan piksel süzgeci: 3x3 komşulukta derinlik farkı > kenar*z olan pikselleri at (0=kapalı)')
    ap.add_argument('--max_w', type=float, default=0.0,
                    help='gövde açısal hızı bundan büyükse (rad/s) kareyi atla (bulanıklık/eşleme hatası; 0=kapalı)')
    a = ap.parse_args()

    kal = yaml.safe_load(open(a.kalib))['base_link_to_camera_link']
    T_b_cl = T(rpy2R(*kal['rotation_rpy_rad']), kal['translation_m'])
    T_b_cl[2, 3] += a.taban_z           # base_link -> base_footprint: dünya z = 0 zemin (poses.txt z = 0)
    O, tfs, ci = sensor_oku(a.sensor_bag)
    T_cl_co = zincir(tfs, 'camera_link', 'camera_depth_frame', 'camera_color_frame', 'camera_color_optical_frame')
    T_cl_do = zincir(tfs, 'camera_link', 'camera_depth_frame', 'camera_depth_optical_frame')
    T_b_co, T_b_do = T_b_cl @ T_cl_co, T_b_cl @ T_cl_do
    Kc, Dc = np.array(ci['color'].k).reshape(3, 3), np.array(ci['color'].d)
    Kd = np.array(ci['depth'].k).reshape(3, 3)
    Hc, Wc = ci['color'].height, ci['color'].width

    # derinlik karelerini damgalarıyla belleğe al (~0.5 MB/kare); renk akışı sırayla işlenir
    derin, d_cerceve = [], None
    for _, v, _ in okuyucu(a.kamera_bag, ['/camera/depth/image_raw']):
        m = deserialize_message(v, Image)
        d_cerceve = d_cerceve or m.header.frame_id
        derin.append((damga(m.header), goruntu(m)))
    dT = np.array([d[0] for d in derin])
    if d_cerceve == 'camera_color_optical_frame':   # sürücüde depth_registration:=true (donanım hizası)
        T_b_do = T_b_co                                 # derinlik zaten renk çerçevesinde, K = renk K
        Kd = Kc.copy()

    for alt in ('rgb', 'depth'):
        os.makedirs(os.path.join(a.cikti, alt), exist_ok=True)
    f_poz = open(os.path.join(a.cikti, 'poses.txt'), 'w')
    f_kam = open(os.path.join(a.cikti, 'poses_kamera.txt'), 'w')
    f_zam = open(os.path.join(a.cikti, 'zaman.txt'), 'w')
    say = {'renk': 0, 'yazilan': 0, 'derinlik_yok': 0, 'odom_boslugu': 0, 'hizli_donus': 0}
    w_odom = np.gradient(O[:, 3], O[:, 0])
    n = 0
    for _, veri, _ in okuyucu(a.kamera_bag, ['/camera/color/image_raw']):
        say['renk'] += 1
        if (say['renk'] - 1) % a.her:
            continue
        m = deserialize_message(veri, Image)
        tc = damga(m.header)
        j = int(np.argmin(np.abs(dT - tc)))
        td = dT[j]
        if abs(td - tc) * 1000 > a.esik_ms:
            say['derinlik_yok'] += 1
            continue
        Tw_bc, Tw_bd = poz_ara(O, tc, a.bosluk_s), poz_ara(O, td, a.bosluk_s)
        if Tw_bc is None or Tw_bd is None:
            say['odom_boslugu'] += 1
            continue
        if a.max_w > 0 and abs(np.interp(tc, O[:, 0], w_odom)) > a.max_w:
            say['hizli_donus'] += 1
            continue
        Tw_co = Tw_bc @ T_b_co
        T_c_d = np.linalg.inv(Tw_co) @ Tw_bd @ T_b_do        # derinlik optik (td) -> renk optik (tc)
        rgb = cv2.undistort(goruntu(m), Kc, Dc)               # ROS rational_polynomial sırası = OpenCV
        dmm = derin[j][1]
        if a.kenar > 0:
            dmm = kenar_suz(dmm, a.kenar)
        dep = derinlik_hizala(dmm, Kd, Kc, T_c_d, (Hc, Wc), zmax=a.zmax)
        cv2.imwrite(os.path.join(a.cikti, 'rgb', f'{n:06d}.png'), rgb)
        np.save(os.path.join(a.cikti, 'depth', f'{n:06d}.npy'), dep)
        for f, M in ((f_poz, Tw_bc), (f_kam, Tw_co)):
            f.write(' '.join(f'{v:.6f}' for v in (*M[:3, 3], *R2q(M[:3, :3]))) + '\n')
        f_zam.write(f'{n} {tc:.6f} {td:.6f}\n')
        n += 1
    say['yazilan'] = n
    for f in (f_poz, f_kam, f_zam):
        f.close()

    R_b_co = T_b_co[:3, :3]
    cfg = {'pose_info': {
        'pose_type': 'mobile_base', 'rot_type': 'quat',
        'camera_height': round(float(T_b_co[2, 3]), 4),             # zeminden (gövde pozu = base_footprint)
        'base2cam_rot': [round(float(v), 6) for v in R_b_co.ravel()],
        'base_forward_axis': [1, 0, 0], 'base_left_axis': [0, 1, 0], 'base_up_axis': [0, 0, 1]},
        'cam_calib_mat': [round(float(v), 4) for v in Kc.ravel()]}
    with open(os.path.join(a.cikti, 'vlmaps_config.yaml'), 'w') as f:
        f.write('# VLMaps config/map_config/vlmaps.yaml içine (pose_info + cam_calib_mat).\n'
                '# poses.txt = gövde (mobile_base), ROS eksenleri. Kamera gövdeye göre ayrıca '
                f'x={T_b_co[0, 3]:.3f} m ileride; mobile_base modu bu ötelemeyi kullanmıyorsa\n'
                '# poses_kamera.txt (camera_base, x sağ / y aşağı / z ileri) tam kamera pozunu içerir.\n'
                '# VLMaps kodunda denenmedi: eksen ayarlarını ilk sahnede doğrulayın.\n')
        yaml.safe_dump(cfg, f, sort_keys=False, default_flow_style=None)
    bilgi = {'kaynak': {'kamera_bag': os.path.abspath(a.kamera_bag), 'sensor_bag': os.path.abspath(a.sensor_bag)},
             'dunya_cercevesi': 'odom (robot açılışındaki konum), z yukarı, z=0 zemin; poses.txt = base_footprint', 'poz_kaynagi': '/odom_duz',
             'sayilar': say, 'suzgecler': {'zmax_m': a.zmax, 'kenar_oran': a.kenar, 'max_w_rad_s': a.max_w, 'her': a.her}, 'kalibrasyon': kal,
             'T_base_renkoptik': [[round(float(v), 6) for v in r] for r in T_b_co],
             'renk_K': Kc.ravel().round(4).tolist(), 'renk_bozulma_giderildi': Dc.round(5).tolist(),
             'derinlik_K': Kd.ravel().round(4).tolist()}
    with open(os.path.join(a.cikti, 'bilgi.yaml'), 'w') as f:
        yaml.safe_dump(bilgi, f, sort_keys=False, allow_unicode=True, default_flow_style=None)
    print(f'yazılan kare {n} / renk {say["renk"]} (derinlik eşi yok {say["derinlik_yok"]}, '
          f'odom boşluğu {say["odom_boslugu"]}, hızlı dönüş {say["hizli_donus"]}) -> {a.cikti}')


if __name__ == '__main__':
    main()
