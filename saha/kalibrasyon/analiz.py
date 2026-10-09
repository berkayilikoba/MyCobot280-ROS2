#!/usr/bin/env python3
"""Kalibrasyon npz dosyalarindan: lidar taramalarini ICP ile eslestirip
gercek hareketi bulur, odometri ile karsilastirir.

Kullanim: analiz.py dosya.npz once_adi sonra_adi [...]
"""
import sys, math
import numpy as np
from scipy.spatial import cKDTree

GOVDE, UZAK = 0.22, 8.0


def noktalar(d, ad):
    r, a = d[ad + '_r'], d[ad + '_a']
    ok = np.isfinite(r) & (r > GOVDE) & (r < UZAK)
    return np.c_[r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]


def icp(src, dst, th0, t0, iter=60):
    """src'yi dst'ye oturtan (R,t): dst ~ R@src + t. Hareket = lidar1'in lidar0'daki pozu."""
    agac = cKDTree(dst)
    th, t = th0, np.array(t0, float)
    for _ in range(iter):
        R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        p = src @ R.T + t
        dist, idx = agac.query(p)
        esik = max(0.05, np.percentile(dist, 80))
        m = dist < esik
        a, b = src[m], dst[idx[m]]
        ca, cb = a.mean(0), b.mean(0)
        H = (a - ca).T @ (b - cb)
        U, _, Vt = np.linalg.svd(H)
        Rn = Vt.T @ U.T
        if np.linalg.det(Rn) < 0:
            Vt[1] *= -1; Rn = Vt.T @ U.T
        th = math.atan2(Rn[1, 0], Rn[0, 0]); t = cb - Rn @ ca
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    dist, _ = cKDTree(dst).query(src @ R.T + t)
    return th, t, float(np.median(dist)), float((dist < 0.05).mean())


def fark(d, a, b):
    oa, ob = d[a + '_odom'], d[b + '_odom']
    c, s = math.cos(oa[2]), math.sin(oa[2])
    dx, dy = ob[0] - oa[0], ob[1] - oa[1]
    # odom hareketi, baslangic govde cercevesinde
    ileri, yan = c * dx + s * dy, -s * dx + c * dy
    dyaw = math.atan2(math.sin(ob[2] - oa[2]), math.cos(ob[2] - oa[2]))
    return ileri, yan, dyaw


dosya, cift = sys.argv[1], sys.argv[2:]
d = np.load(dosya)
for a, b in zip(cift[0::2], cift[1::2]):
    ileri, yan, dyaw = fark(d, a, b)
    p0, p1 = noktalar(d, a), noktalar(d, b)
    adaylar = []
    for sth in (+1, -1):
        for st in (+1, -1):
            adaylar.append(icp(p1, p0, sth * dyaw, (st * ileri, 0.0)))
    th, t, med, oran = min(adaylar, key=lambda x: x[2])
    print(f'{a}->{b}: ODOM ileri={ileri:+.3f} m yan={yan:+.3f} m donus={math.degrees(dyaw):+.2f} deg')
    print(f'        LIDAR(ICP) tx={t[0]:+.3f} m ty={t[1]:+.3f} m donus={math.degrees(th):+.2f} deg '
          f'| eslesme medyan hata={med*100:.1f} cm, <5cm oran={oran:.0%}')
    if abs(dyaw) > math.radians(20):
        # yerinde donuste lidar kaymasi -> lidarin donus merkezine uzakligi
        L = np.hypot(*t) / (2 * abs(math.sin(th / 2)))
        print(f'        donus olcegi (odom/lidar) = {abs(dyaw)/abs(th):.3f} | lidar-donus merkezi mesafesi ~ {L*100:.1f} cm')
    else:
        print(f'        oteleme olcegi (odom/lidar) = {abs(ileri)/max(1e-6, np.hypot(*t)):.3f}')
