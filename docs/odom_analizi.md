# Gerçek LIMO odometri analizi (24.09 kayıtları)

Kayıtlar: `saha/oturum_20260924_163343/bag` (teleop haritalama), `saha/nav_20260924_164729/bag` (Nav2).
Referans: ardışık lidar taramaları arası ICP (2 tarama arayla, ~0.33 s), uyum ≥ %70 olan çiftler.
Betikler: `saha/analiz/` (`odom_saglik.py`, `scan_cek.py`, `icp_ardisik.py`).

## 1. Yavaş dönüşler odom yönünden siliniyor (ana bulgu)

Upstream Foxy sürücü, `robottaki upstream Foxy limo_base, src/limo_driver.cpp:502`:

    if(delta_theta_< 0.1 && delta_theta_> -0.1) delta_theta_=0;

IMU örneği başına 0.1° ölü bant → ~100 Hz'de 10°/s (0.175 rad/s) altındaki dönüşler odom yönüne yansımıyor.

| ICP dönüş hızı [rad/s] | odom/ICP (Nav2 kaydı) | IMU yönü/ICP |
|---|---|---|
| 0.05–0.10 | 0.665 | 1.19* |
| 0.10–0.20 | 0.677 | 1.07 |
| 0.20–0.40 | 0.912 | 1.04 |
| 0.40–0.80 | 0.992 | 1.00 |

\* küçük açılarda ICP gürültülü.

İlerlerken biriken yön hatası (ICP'ye göre): **ham odom −92.8° / 17.2 m, IMU yönü −6.7°**.
Teleop kaydında etkisi küçük (büyük dönüşler hızlı): −8.6° vs −6.9° / 8.3 m.
MPPI/RPP yol düzeltmeleri tam bu yavaş bölgede → AMCL sürekli yön düzeltiyor (Nav2 kaydında map→odom yaw
12° kaydı, 31 kez >2° sıçrama) → robot yalpalıyor.

Odom konumu (ileri hız) doğru: ICP oranı 0.98–1.00. Sürücünün twist.wz'si (tekerlekten) dönüşte IMU'nun ~%91–96'sı.

## 2. Odom mesaj kaybı ve gecikme (Wi-Fi)

| | Haritalama (kamera açık) | Nav2 |
|---|---|---|
| Gelen / beklenen (50 Hz) | 21350 / 29924 (%71) | 30996 / 44510 (%70) |
| Damga aralığı > 0.2 s | 340 | 419 |
| Alış − damga gecikmesi medyan / p95 | 0.67 / 1.12 s | 0.03 / 1.23 s |
| IMU gelen / beklenen | %97 | %74 |

Bu düzeltmeyle çözülmez; Nav2'nin robotta (Docker) çalışmasını gerektirir.

## Düzeltme: `saha/robot/odom_duzelt.py` (robotta, /tmp'den)

- Sürücü `pub_odom_tf:=False` ile başlar (launch argümanı; robottaki dosyalar değişmez).
- Düğüm: konum = sürücü vx × IMU yönü; yön = IMU yönelimi; wz = IMU jiroskobu → `/odom_duz` + TF odom→base_link.
- Nav2 (`config/nav2_limo_gercek.yaml`): bt_navigator / controller_server / velocity_smoother `odom_topic: odom_duz`.
- Kayıt üzerinde doğrulama (Nav2 bag, 5× oynatma): ilerlerken yön hatası **−19.8° / 16.4 m** (ham −92.8° / 17.2 m).
  Robotta (Foxy) henüz çalıştırılmadı.
- Başlat/durdur: `saha/nav_baslat.sh`, `harita_baslat.sh`, `nav_dur.sh`, `harita_bitir.sh` güncellendi.
- Kalıcı çözüm: Humble çatalımız (`limo_ws/src/limo_base`) ölü bandı parametre yaptı (`yaw_dead_band_deg`, varsayılan 0).
