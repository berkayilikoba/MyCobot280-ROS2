# LIMO ROS 2 Humble çalışma alanı

AgileX LIMO için ROS 2 Humble + Nav2 yığını, Gazebo 11 simülasyonu, otonom devriye katmanı ve Jetson (L4T r35) Docker ortamı.

## Paketler (`src/`)

| Paket | İçerik |
|---|---|
| `limo_base` | Gerçek robot sürücüsü (seri port), kinematik yardımcıları + gtest |
| `limo_msgs` | LIMO mesajları |
| `limo_bringup` | Gerçek robot ve sim launch dosyaları, Nav2 parametreleri (`param/nav2_{diff,ackermann}.yaml`), davranış ağaçları, harita, ölçüm betikleri (`scripts/`) |
| `limo_description`, `limo_car`, `mycobot_description` | URDF/xacro, Gazebo modelleri ve dünyalar |
| `limo_kinematics` | Ackermann/diff geometri ve kinematik hesapları, sim için `ackermann_cmd_adapter` + testler |
| `limo_autonomy` | Görev planlayıcı + `patrol` düğümü (waypoint devriyesi, yeniden deneme, eve dönüş) + testler |

## Kurulum (Ubuntu 22.04, ROS 2 Humble)

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-robot-localization ros-humble-gazebo-ros-pkgs ros-humble-xacro python3-opencv
cd limo_ws && rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install && source install/setup.bash
```

## Simülasyon

```bash
# diff modu, AMCL + hazır harita, Nav2 (RViz ile)
ros2 launch limo_bringup limo_sim_nav2.launch.py mode:=diff localization:=amcl
# Ackermann modu, SLAM ile
ros2 launch limo_bringup limo_sim_nav2.launch.py mode:=ackermann localization:=slam
# ayrı terminalde otonom devriye
ros2 launch limo_autonomy patrol.launch.py use_sim_time:=true set_initial_pose:=true
```

Ackermann için ileri akışlı görev: `mission_file:=$(ros2 pkg prefix --share limo_autonomy)/config/sim_patrol_ackermann.yaml`.
SLAM modunda devriyeye `localizer:=slam_toolbox set_initial_pose:=false` verin.

## Ölçüm ve kayıt betikleri (`src/limo_bringup/scripts/`)

- `bench_many.sh <çıktı> <diff|ackermann> <N> <domain>`: yalıtılmış headless sim'de 6 hedefli Nav2 turunu N kez koşar
  (`nav_bench.py`), `bench_summary.py` ile özetler.
- `patrol_run.sh`: uçtan uca devriye testi. `video_run.sh`: üstten kameralı video + rosbag kaydı.
- `steer_step.py`, `diff_openloop.py`, `openloop_test.py`: Nav2'siz açık çevrim araç testleri.
- Paralel koşularda her koşu kendi `ROS_DOMAIN_ID` (≤101) ve Gazebo portunu kullanır; makine başına ~3 sim.

## Gerçek robot / Jetson

`docker/README.md`: Jetson (Ubuntu 20.04, L4T r35) için Humble imajı, compose profilleri (`limo_base`, `nav`, `slam`,
`patrol`), udev kuralı. İmaj henüz Jetson'da derlenmedi.

## Durum

Ayrıntılı bulgular, ölçümler ve açık sorunlar: [`docs/gece_raporu_2026-09-23.md`](docs/gece_raporu_2026-09-23.md).
Hepsi simülasyonda doğrulandı; gerçek robotta henüz denenmedi. Raporda geçen `bench/...` ölçüm klasörleri depoda değil.
