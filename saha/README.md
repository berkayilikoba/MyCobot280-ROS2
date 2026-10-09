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
Kaydı PC'ye almak: `kayit_bitir.sh --indir` (→ `~/limo_veri/robot_kayit/`). Bag'ler depoya girmez; ortak sürücüye
koyun, PC'ye aldıktan sonra robottan silin.

## 4. Kamera dış kalibrasyonu

`config/kamera_kalibrasyon.yaml` — `base_link -> camera_link` = (0.084, 0, 0.028) m, roll +0.43°, pitch −1.42°.

- x, y: resmi AgileX URDF. z, roll, pitch: derinlik görüntüsündeki zemin düzleminden ölçüldü
  (`analiz/zemin_egimi.py`, 38 kare, düzlem RMS 3 mm): kamera yerden 17.8 cm, ~1.4° yukarı bakıyor. Yaw ölçülmedi.
- `nav_baslat.sh` / `harita_baslat.sh` bu dönüşümü statik TF olarak yayınlar; robottaki kayıt `/tf_static`'e alır.
- **09.10 kayıtlarında bu halka yok** — işlerken yaml'daki değerle statik ekleyin (komut yaml'da).
- Kare pozu: `T_map_optik(t) = T_map_base(t) · T_base_camlink · T_camlink_optik` (son halka sürücünün `/tf_static`'inde).

## Bilinen sorunlar / yapılacaklar

- Kamera görüntüleri kayıtta ~20 Hz: sıkıştırılmış konu ya da DDS soket tamponu (sudo, robot sorumlusu onayı) gerekli.
- Kamera açıkken `/odom_duz`'da kısa boşluklar var (09.10: 84 s'de 15 kez >0.2 s, en uzun 0.33 s; kamerasız turda 0).
  Konumu bozmaz (entegrasyon damga aralığıyla), ama yüksek hızda poz ara değerlemesi gerekir.
- Derinlik renk görüntüsüne hizalı değil (640×400 vs 640×480); `camera_info` + `depth_to_color` kayıtta.
- Kamera yaw'ı ve derinlik sensörünün yanal ofseti ölçülmedi (derinlik dilimi ↔ lidar eşleştirmesiyle ölçülebilir).
- **Lidar TF'i yanlış:** robottaki fabrika `open_ydlidar_launch.py` `base_link -> laser_frame` = (0, 0, 0.02), yaw 0
  yayınlıyor; kayıtlardan ölçülen (yerinde dönüş + düz gidiş ICP) (0.137, 0.015) m, **yaw −9.66°**. AMCL'nin dönüşte
  sıçraması ve kayması büyük olasılıkla bundan. Düzeltme (yapılacak): `scan_sabitle.py` çıktısını kendi çerçevesine
  (`laser_duz`) yayınlayıp doğru statik TF vermek, haritayı yeniden çıkarmak.
- Dönüşlerde 4 teker kaydığı için odom yer değiştirmesinde ~1–8 cm/dönüş hata var (yön hatası ≤1°).
- AMCL bu odada güvenilir değil (tarama–harita örtüşmesi ~0.5); kare pozları için kayıttan sonra lidar eşleştirmesi önerilir.
- 09.10'daki ilk kamerali deneme (`~/gunes_kayit/dikdortgen_kamera_*`, tek kayıt süreci) bozuk: lidar 0, odom 35 Hz.
  Doğru kayıt: `~/gunes_kayit/kamera_tur_20261009_122158/{sensor,kamera}`.

## Analiz betikleri (`analiz/`)

| Betik | Ne yapar |
|---|---|
| `odom_saglik.py <bag> <çıktı_öneki> [/odom_duz]` | odom/IMU hızı, gecikme, boşluklar, AMCL `map->odom` sıçramaları |
| `scan_cek.py` + `icp_ardisik.py` | ardışık lidar taramalarını ICP ile eşleyip odom yönü/mesafesini doğrular (dönüş hızına göre) |
| `icp_karsilastir.py` | durağan anlar arası ICP ile odom karşılaştırması |
| `zemin_egimi.py <kamera_bag> <sensor_bag>` | kameranın zeminden yüksekliği ve roll/pitch'i (dış kalibrasyon) |
| `bag2vlmaps.py <kamera_bag> <sensor_bag> <çıktı>` | kaydı VLMaps sahnesine çevirir: bozulması giderilmiş RGB, renge hizalı metre derinlik (.npy), kare pozları (gövde + kamera), VLMaps ayarı |
| `vlmaps_dogrula.py <sahne> <sensor_bag>` | sahne doğruluğu: derinlik–renk hizası, zemin yüksekliği, kamera–lidar örtüşmesi, başlangıca dönüş hatası |

Kullanım örnekleri ve 24.09 / 08.10 sonuçları: `docs/odom_analizi.md`.
