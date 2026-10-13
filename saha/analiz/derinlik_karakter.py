#!/usr/bin/env python3
"""Sabit sahne derinlik karakterizasyonu (kamera ayar / kamera karşılaştırması; D435 ile aynı ölçütler).

Her alt dizin bir bag (kamera_ayar_testi.sh çıktısı) ya da tek bag verilir. Robot DURURKEN alınmış olmalı.
Ölçütler (derinlik görüntüsü, ham piksel ızgarası):
  kare_hz        derinlik kare hızı
  doluluk        geçerli (z>0) piksel oranı, kareler ortalaması; ayrıca 0.3-1, 1-2, 2-4 m bölgelerinde
  sigma_mm       piksel başına zamansal std (geçerli olduğu karelerin >=%80'inde); mesafe aralığına göre medyan
  sigma_bagil    sigma / z (%), aralığa göre
  kirpisma       zamanla geçerli<->geçersiz değişen piksel oranı (titreşen delikler)
  ucan_piksel    komşusundan >%10 farklı derinliğe sahip, iki tarafı da tutarsız piksel oranı (kenar "uçan" noktaları)

Kullanım (Humble): derinlik_karakter.py <dizin_veya_bag> [...] [--json cikti.json]
"""

import argparse
import glob
import json
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

ARALIK = [(0.3, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 8.0)]


def derinlik_oku(bag, konu='/camera/depth/image_raw'):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[konu]))
    kareler, t = [], []
    while r.has_next():
        _, veri, _ = r.read_next()
        m = deserialize_message(veri, Image)
        z = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(np.float32) / 1000.0
        kareler.append(z)
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    return np.stack(kareler), np.array(t)


def analiz(bag):
    Z, t = derinlik_oku(bag)
    n, h, w = Z.shape
    gecerli = Z > 0
    sonuc = {'kare': int(n), 'boyut': f'{w}x{h}',
             'kare_hz': round(float((n - 1) / (t[-1] - t[0])), 1) if n > 1 else 0.0,
             'doluluk': round(float(gecerli.mean()), 3)}
    oran = gecerli.mean(0)
    kararli = oran >= 0.8
    Zn = np.where(gecerli, Z, np.nan)
    med = np.nanmedian(Zn, 0)
    sd = np.nanstd(Zn, 0)
    sonuc['kirpisma'] = round(float(((oran > 0.05) & (oran < 0.95)).mean()), 3)
    for a, b in ARALIK:
        bolge = (med >= a) & (med < b)
        k = bolge & kararli
        ad = f'{a:g}-{b:g}m'
        sonuc[f'piksel_{ad}'] = round(float(bolge.mean()), 3)
        if k.sum() < 200:
            continue
        sonuc[f'doluluk_{ad}'] = round(float(gecerli[:, bolge].mean()), 3)
        sonuc[f'sigma_mm_{ad}'] = round(float(np.median(sd[k]) * 1000), 2)
        sonuc[f'sigma_bagil_%_{ad}'] = round(float(np.median(sd[k] / med[k]) * 100), 3)
    # uçan piksel: medyan görüntüde yatay/dikey komşuların ikisinden de >%10 farklı ve komşular birbirine yakın değil
    m = np.nan_to_num(med, nan=0.0)
    ucan = np.zeros_like(m, bool)
    for dy, dx in ((0, 1), (1, 0)):
        a = np.roll(m, (dy, dx), (0, 1))
        b = np.roll(m, (-dy, -dx), (0, 1))
        ok = (m > 0) & (a > 0) & (b > 0)
        fa = np.abs(m - a) > 0.1 * m
        fb = np.abs(m - b) > 0.1 * m
        ara = ((m - a) * (m - b)) < 0  # a ile b arasında: kenar geçişinde ara değer (uçan nokta)
        ucan |= ok & fa & fb & ara
    sonuc['ucan_piksel_%'] = round(float(ucan[1:-1, 1:-1].mean() * 100), 3)
    return sonuc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('yollar', nargs='+')
    ap.add_argument('--json')
    a = ap.parse_args()
    baglar = []
    for y in a.yollar:
        if glob.glob(os.path.join(y, '*.db3')):
            baglar.append(y)
        else:
            baglar += sorted(d for d in glob.glob(os.path.join(y, '*')) if glob.glob(os.path.join(d, '*.db3')))
    tum = {}
    for b in baglar:
        s = analiz(b)
        tum[os.path.basename(b.rstrip('/'))] = s
        print(f'== {os.path.basename(b.rstrip("/"))}')
        for k, v in s.items():
            print(f'  {k:24s} {v}')
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(tum, f, indent=1, ensure_ascii=False)


if __name__ == '__main__':
    main()
