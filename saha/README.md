# Gerçek LIMO saha betikleri

Gerçek robotta haritalama, Nav2 ile otonom sürüş, dikdörtgen tur ve robot üzerinde rosbag kaydı.
Komutlar depo kökünden (`cd ~/limo_ws`) verilir.
Robot tarafı **dokunulmadan** kullanılır: robottaki Foxy sürücüleri yalnızca launch argümanlarıyla başlatılır,
kendi dosyalarımız robota `/tmp` üzerinden kopyalanır, kayıtlar robotta `~/limo_kayit/` altına yazılır.

## Mimari

```text
Robot (Jetson Orin Nano, Ubuntu 20.04, ROS 2 Foxy, domain 77)       PC (Ubuntu 22.04, ROS 2 Humble, domain 77)
  limo_base (ttyTHS0, pub_odom_tf:=False)  -> /odom, /imu            scan_sabitle.py: /scan -> /scan_sabit
  robot/odom_duzelt.py (/tmp)              -> /odom_duz + TF          slam_toolbox (haritalama) veya
  ydlidar T-mini                           -> /scan                     map_server + AMCL + Nav2 (MPPI)
  orbbec Dabai DC1 (isteğe bağlı)          -> /camera/...             dikdortgen.py / hedefe_git.py
  ros2 bag record (kayit_baslat.sh)        -> ~/limo_kayit/           RViz
```

**odom_duzelt.py neden var:** robottaki upstream sürücü (`limo_driver.cpp:502`) IMU örneği başına 0.1°'lik ölü bant
uyguluyor; ~10°/s altındaki dönüşler odom yönüne hiç yansımıyor. Nav2 sürüşünde ham odom 23 m'de +73° yön hatası
biriktirdi, `odom_duz` +4.7° (ICP'ye göre). Ayrıntı: `docs/odom_analizi.md`.

## Kurulum (PC, bir kez)

```bash
# Humble + Nav2 + slam_toolbox kurulu olmalı
cd ~/limo_ws && colcon build --symlink-install
ssh-copy-id agilex@192.168.0.200        # parolasız SSH şart (betikler BatchMode kullanır)
```

`~/.bashrc` içinde `ROS_LOCALHOST_ONLY=1` varsa robotu göremezsiniz; betikler kendi içinde `0` yapar,
elle açtığınız terminallerde (teleop, rviz) `ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=77` verin.

Ayarlar `ortam.sh` içinde, ortam değişkeniyle değiştirilebilir:

| Değişken | Varsayılan | Anlamı |
|---|---|---|
| `ROBOT` | `agilex@192.168.0.200` | Robot_Lab ağında sabit IP |
| `LIMO_VERI` | `~/limo_veri` | PC'de oturum çıktıları (harita, bag, log) |
| `LIMO_DOMAIN` | `77` | ROS_DOMAIN_ID |
| `SURE` | 1800 (nav), 1200 (kamera), 600 (kayıt) | robot süreçleri bu kadar saniye sonra kendiliğinden kapanır |

## 1. Haritalama

Robotu **başlangıç işaretine** koyup başlatın: haritanın orijini o nokta, x ekseni robotun baktığı yön olur.

```bash
saha/harita_baslat.sh
# ayrı terminalde sürün:
source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=0 \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.5
# odayı yavaşça dolaşın, başladığınız yere dönün; teleop'u kapatın, sonra:
saha/harita_bitir.sh            # -> ~/limo_veri/oturum_<zaman>/harita.yaml
```

## 2. Nav2 (AMCL)

```bash
NAV_PARAMS=saha/config/nav2_limo_gercek_025.yaml saha/nav_baslat.sh ~/limo_veri/oturum_<zaman>/harita.yaml
saha/poz_ver.sh                 # robot başlangıç işaretindeyse (harita orijini); değilse RViz "2D Pose Estimate"
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz   # (ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=0)
saha/nav_dur.sh                 # durdurma (acil durumda da bu: önce robot sürücüsünü kapatır)
```

- `nav2_limo_gercek_025.yaml`: 0.25 m/s (ilk denemeler), `nav2_limo_gercek.yaml`: 0.35 m/s. Kontrolcü MPPI.
- Hedefleri duvar/eşyadan **en az 0.5 m** uzağa verin; dibine verilen hedefte robot dönecek yer bulamıyor.
- Teleop açıkken Nav2 sürmeyin: teleop doğrudan `/cmd_vel`'e yazar, çarpışma monitörünü atlar.

## 3. Dikdörtgen tur + kayıt (kamera veri seti)

```bash
saha/nav_baslat.sh <harita.yaml>     # çarpışma monitörü + AMCL referansı için
saha/poz_ver.sh
saha/kamera_baslat.sh                # isteğe bağlı: Orbbec renk+derinlik
# robot başlangıç işaretinde, ilk (2 m) kenara bakıyor:
saha/dikdortgen_tur.sh               # 2 m -> sol 90° -> 1 m -> sol 90° -> 2 m -> sol 90° -> 1 m -> sol 90°
saha/kayit_bitir.sh --indir          # (dikdortgen_tur.sh kaydı zaten kapatır; --indir yalnız kopyalamak için)
```

`dikdortgen_tur.sh` ek argümanları `dikdortgen.py`'ye geçirir: `--tur 3`, `--hiz 0.15`, `--kenarlar 2,1,2,1`, `--sag`.
Rota **odom ile** sürülür (`--cerceve odom`); AMCL bu odada yerinde dönüşlerde 5–32 cm sıçradı ve turda ~50 cm kaydı,
odom ise 6 m'lik turu başlangıca 4.9 cm / −3.3° farkla kapattı (başlangıç–bitiş taraması ICP, uyum 0.97).

Kayıt (robotta `~/limo_kayit/<ad>_<zaman>/`), iki ayrı bag — sonradan zaman damgasıyla birleştirin (aynı robot saati):

| bag | konular | ölçülen hız (09.10) |
|---|---|---|
| `sensor` | `/odom` `/odom_duz` `/imu` `/scan` `/tf` `/tf_static` `/cmd_vel` `/cmd_vel_smoothed` `/amcl_pose` `/limo_status` `/camera/*/camera_info` `/camera/depth_to_color` | odom 49, IMU 96, lidar 6 Hz |
| `kamera` | `/camera/color/image_raw` (640×480 rgb8) `/camera/depth/image_raw` (640×400 16UC1) | ~20 Hz (kamera 30 Hz yayınlıyor, kayıt ~1/3 düşürüyor) |

Kamerali kayıt ~27 MB/s (~1.6 GB/dk); robotta ~17 GB boş. `kayit_baslat.sh` 5 GB altında başlamaz.

## Bilinen sorunlar / yapılacaklar

- Kamera görüntüleri kayıtta ~20 Hz: sıkıştırılmış konu ya da DDS soket tamponu (sudo, robot sorumlusu onayı) gerekli.
- Derinlik renk görüntüsüne hizalı değil (640×400 vs 640×480); `camera_info` + `depth_to_color` kayıtta.
- Kameranın `base_link`'e göre konumu (dış kalibrasyon) TF'te yok — mezura ile ölçülüp eklenmeli; kare pozu için şart.
- Dönüşlerde 4 teker kaydığı için odom yer değiştirmesinde ~1–8 cm/dönüş hata var (yön hatası ≤1°).
- AMCL bu odada güvenilir değil (tarama–harita örtüşmesi ~0.5); kare pozları için kayıttan sonra lidar eşleştirmesi önerilir.

## Analiz betikleri (`analiz/`)

`odom_saglik.py` (odom/IMU hız, gecikme, boşluk), `scan_cek.py` + `icp_ardisik.py` (ardışık tarama ICP ile odom
yön/mesafe doğrulaması). Kullanım örnekleri `docs/odom_analizi.md` içinde.
