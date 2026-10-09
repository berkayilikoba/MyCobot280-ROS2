# LIMO ROS 2 Humble çalışma alanı

AgileX LIMO (+ myCobot 280) için ROS 2 Humble yığını: gerçek robotta haritalama, Nav2 ile otonom sürüş ve
kamera/odom veri seti kaydı; gerçek ölçümlerle beslenmiş Gazebo 11 simülasyonu.

## Nereden başlamalı?

| Ne yapmak istiyorsunuz? | Bakın |
|---|---|
| Gerçek robotta harita çıkarmak, Nav2 ile sürmek, dikdörtgen tur + rosbag kaydı | [`saha/README.md`](saha/README.md) |
| Kayıtlardaki kamera karelerine poz eşlemek (dış kalibrasyon, çerçeve zinciri) | [`saha/config/kamera_kalibrasyon.yaml`](saha/config/kamera_kalibrasyon.yaml) |
| Simülasyonda Nav2 / kontrolcü denemek | [Simülasyon](#simülasyon) |
| Robotun odom/sensör davranışını anlamak | [`docs/odom_analizi.md`](docs/odom_analizi.md), [`docs/sensor_modeli.md`](docs/sensor_modeli.md) |

## Robot

| | |
|---|---|
| Bilgisayar | Jetson Orin Nano, Ubuntu 20.04, L4T R35.4.1 |
| Sürücüler | ROS 2 **Foxy**, robotta `~/limo_ros2_ws` (fabrika; dokunmayın) |
| Sürüş modu | FOUR_DIFF (4 teker diferansiyel); dönüşte tekerler kayar |
| Sensörler | YDLIDAR T-mini (6 Hz, arka ~96° kör), taban IMU (100 Hz), Orbbec Dabai DC1 renk+derinlik (30 Hz) |
| Ağ | Wi-Fi `Robot_Lab`, sabit IP **192.168.0.200**, kullanıcı `agilex`, `ROS_DOMAIN_ID=77` |

PC tarafı Humble, robot tarafı Foxy; standart konular/TF üzerinden konuşurlar (aynı domain, `ROS_LOCALHOST_ONLY=0`).
Robot başka ekiplerce de kullanılıyor: oradaki Noetic çalışma alanlarına, `~/.bashrc`'ye ve verilere dokunmayın.

## Kurulum (PC: Ubuntu 22.04, ROS 2 Humble)

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-robot-localization ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-teleop-twist-keyboard \
  python3-opencv python3-scipy
git clone https://github.com/HSGunes/limo_ws.git ~/limo_ws && cd ~/limo_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install && source install/setup.bash
ssh-copy-id agilex@192.168.0.200          # saha betikleri parolasız SSH ister
```

## Dizinler

| Yol | İçerik |
|---|---|
| `saha/` | Gerçek robot betikleri (harita, Nav2, tur, kayıt, kamera), robot tarafı düğüm (`robot/odom_duzelt.py`), gerçek robot Nav2/SLAM ayarları (`config/`), haritalar, analiz betikleri |
| `src/limo_bringup` | Sim ve Nav2 launch dosyaları, Nav2 parametreleri (sim), davranış ağaçları, sim ölçüm betikleri (`scripts/`) |
| `src/limo_description`, `src/limo_car`, `src/mycobot_description` | URDF/xacro, Gazebo modelleri ve dünyalar |
| `src/limo_kinematics` | Kinematik hesapları, sim taban tepki modeli (`taban_modeli`), testler |
| `src/limo_autonomy` | Görev planlayıcı + `patrol` düğümü + testler |
| `src/limo_base`, `src/limo_msgs` | Humble sürücü çatalı (yaw ölü bandı parametreli; robotta henüz kullanılmıyor), mesajlar |
| `docker/` | Jetson için Humble imajı (henüz robotta derlenmedi; şu an gerek yok) |
| `docs/` | Bulgular ve ölçümler |

## Simülasyon

```bash
ros2 launch limo_bringup limo_sim_nav2.launch.py mode:=diff localization:=amcl        # AMCL + hazır harita
ros2 launch limo_bringup limo_sim_nav2.launch.py mode:=ackermann localization:=slam   # SLAM
ros2 launch limo_autonomy patrol.launch.py use_sim_time:=true set_initial_pose:=true  # ayrı terminal: devriye
```

Sim, gerçek robottan ölçülen değerlerle beslenir ([`docs/sensor_modeli.md`](docs/sensor_modeli.md)): lidar 6 Hz / 481
nokta / 7 mm gürültü / arka kör sektör, IMU gürültüsü, taban tepki modeli (ileri harekette 0.3 s gecikme + yavaş
hızlanma; diff modunda `taban_modeli` düğümü). myCobot eklemleri simde kilitli (`kol_kilit:=true`, varsayılan).

Ölçüm betikleri (`src/limo_bringup/scripts/`): `bench_many.sh` (headless, 6 hedefli Nav2 turu × N, özet),
`patrol_run.sh`, `video_run.sh`, `sim_ikiz.py` (gerçek kayıttaki komutları sime oynatıp tepkiyi karşılaştırır),
`sensor_olc.py`. Paralel koşularda her koşu ayrı `ROS_DOMAIN_ID` (≤101) kullanır.

## Ekip çalışma kuralları

**Robot:**
- Robotu aynı anda tek kişi/tek yığın sürer. Başlamadan `saha/nav_dur.sh` ile kimsenin sürücüsünün açık olmadığına
  bakın (betikler robotta sürücü açıksa zaten başlamaz).
- Robot hareket ederken başında biri olsun; acil durdurma `saha/nav_dur.sh` (önce taban sürücüsünü kapatır).
- Robot süreçleri `SURE` saniye sonra kendiliğinden kapanır (Nav2 30 dk, kamera 20 dk, kayıt 10 dk).
- Kayıtlar robotta `~/limo_kayit/` altına yazılır; işiniz bitince PC'ye alıp robottan silin (diskte ~17 GB boş).
  `sudo` gerektiren her şey (paket kurulumu, ağ, sistem ayarı) robot sorumlusunun onayıyla.

**Git:**
- `main`'e doğrudan değil, dal + pull request ile katkı verin (`git switch -c ad/konu`).
- Rosbag ve büyük veri depoya girmez (`.gitignore`: `*.db3`); paylaşmak için ortak sürücü kullanın, yolunu PR'da yazın.
- Gerçek robotta denenmiş değişiklikte ölçümü (ör. köşe hataları, konu hızları) PR açıklamasına ekleyin.

## Durum (09.10.2026)

- Gerçek robotta: haritalama, Nav2 (MPPI + AMCL, 3/3 hedef), dikdörtgen tur (odom ile; 6 m'de başlangıca 4.9 cm /
  −3.3° dönüş, lidar ICP ile ölçüldü), robot üzerinde kayıpsız odom/IMU/lidar kaydı, kamerali kayıt çalışıyor.
- Açık sorunlar: kamera kaydı ~20 Hz (yayın 30 Hz), AMCL yeni odada güvenilir değil, kamera yaw kalibrasyonu
  ölçülmedi. Ayrıntı: [`saha/README.md#bilinen-sorunlar--yapılacaklar`](saha/README.md#bilinen-sorunlar--yapılacaklar).
- Sim bulguları ve kontrolcü karşılaştırmaları: [`docs/gece_raporu_2026-09-23.md`](docs/gece_raporu_2026-09-23.md)
  (raporda geçen `bench/...` ölçüm klasörleri depoda değil).
