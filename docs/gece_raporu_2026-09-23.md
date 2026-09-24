# LIMO: Gece Çalışması Raporu (23 Eylül 2026, 02:10–08:00)

## Özet

| | Gece başı | Sabah |
|---|---|---|
| Diff, nav_bench (6 hedef × 3 tur) | 18/18 | **18/18** (açıklık arttı) |
| Ackermann, nav_bench | **2/12** | **16/18** (%89) |
| Diff devriye, AMCL (2 tur + eve dönüş) | yok | **8/8 + ev** (2/2 koşu, biri son yapılandırmayla) |
| Diff devriye, SLAM | yok | **8/8 + ev** (2/2 koşu) |
| Ackermann devriye, AMCL, döngü görevi | yok | **8/8 + ev** (2/2 koşu) |
| Ackermann devriye, SLAM | yok | **0/2**: SLAM kayması (açık sorun 2) |
| Jetson Docker (Humble, L4T r35) | yok | dosyalar hazır, **derlenmedi** |

Hepsi simülasyon (Gazebo 11, headless; AMCL + bilinen harita ya da SLAM). Gerçek robotta henüz hiçbir şey denenmedi.

## Bulunan kök nedenler (sırayla, hepsi ölçümle)

1. **Sim direksiyonunun anlamı** (gece öncesi bulundu): gazebo_ros_ackermann_drive `angular.z`'yi direksiyon açısı olarak
   alıyor, Nav2 ve gerçek sürücü ise yaw hızı gönderiyor. Çözüm: `limo_kinematics/ackermann_cmd_adapter`.
2. **Sim direksiyon eklemi çok yavaştı**: `effort=5, friction=2.0` değerleriyle direksiyon 4 s'de hedef eğriliğin yalnızca
   %44'üne varıyordu. Nav2 her dönüşü aşıp sütun ve duvarlara giriyordu (MPPI ve RPP'de aynı belirti). Düzeltme
   `effort=20, damping=0.5, friction=0.2` oldu: tam kilide t90 = 0.29 s. Ölçüm aracı: `scripts/steer_step.py`.
3. **Şişirme yarıçapı ayak izinin çevrel yarıçapından küçüktü** (0.15 < ~0.22, iki profilde de): Humble'da Smac
   ve MPPI ayak izini yalnızca "belki çarpışır" maliyet eşiğinin üstünde denetliyor. Bu eşik o yarıçaptan hesaplandığı
   için 0.15–0.22 bandında hiç denetim yoktu. Planlar duvar uçlarını sıyırıyordu. Humble bunun için uyarı vermiyor.
   Değer 0.35 yapıldı (cost_scaling 4.0).
4. **Sim tekerlek çarpışma silindirleri yarım tekerlek dışarı kaymıştı**: simdeki gövde 0.262 m genişlikteydi (gerçeği
   0.22), tekerlekler ayak izinin dışında kalıyordu ve etkin iz genişliği 0.217 m'ydi (eklenti 0.172 kabul ediyor).
   Düzeltmeden sonra tam kilit yarıçapı 0.502'den 0.465'e indi (teori 0.473).
5. **Diff yalpalaması simdeki tekerlek fiziğinden** (23 Eylül öğleden sonra): Nav2 kapalıyken açık çevrim düz sürüşte
   bile yaw hızı ±0.3–0.5 rad/s savruluyordu. Sürtünme yönü (`fdir1`) tekerlekle birlikte dönüyordu. Bu düzeltildikten
   sonra da Gazebo'nun 4 tekerlekli kaymalı dönüşü koşudan koşuya tutarsız kaldı (aynı komuta dönüş oranı 0.0–0.6).
   Diff modeli artık `gazebo_ros_planar_move` kullanıyor: gövdeye komut edilen v/w'yi uygular, gerçek LIMO kartının
   v/w arayüzüyle aynı. Kapalı çevrim ölçüm: yaw hızı RMS 0.64 → 0.31, salınım 3.7 → 1.8 /m, tur 110 → 87 s,
   min açıklık 0.03 → 0.14 m, 18/18. MPPI tarafında `wz_std 0.6` (9/12) ve TwirlingCritic (10/12) denendi, kötüleştirdi.

## Nav2'de değişen parametreler (`limo_bringup/param`)

- İki profil: `inflation_radius 0.15 → 0.35`, `cost_scaling_factor 3.0 → 4.0` (global ve yerel).
- Ackermann Smac Hybrid: `reverse_penalty 2.0 → 5.0`, `analytic_expansion_max_length 3.0 → 1.5`.
  Gerekçe: gerçek robotta lidar `angle_min/max = ±100°` (ydlidar_tmini.yaml), yani arkada 160° kör alan var.
  Geri manevra en aza indirildi.
- Ackermann BT'leri (`navigate_to_pose_ackermann.xml`, `navigate_through_poses_ackermann.xml`): **yerel costmap artık
  temizlenmiyor**. Arkadaki engeller yalnızca costmap belleğinde duruyor; temizlenince araç geri geri duvara gidiyordu.

Denenip **reddedilenler** (ölçüm `bench/x4–x10`): RPP (geri takipte salınıyor), DUBIN (ileri-yalnız; duvar ucunda
donuyor), yalnızca yol geçersizleşince yeniden planlayan BT (fark yok), CostCritic ağırlığı 10 (daha kötü),
`PathAngleCritic.forward_preference: false` (15/18), PreferForward 2 + vx_min −0.3 (13/18).

## Otonomi katmanı (`limo_autonomy`)

- `patrol` düğümü: görev YAML'ı, döngü/tur sayısı, yeniden deneme, ardışık hata sonrası eve dönüş, düşük pilde eve dönüş.
  18 birim test geçiyor.
- Bu gece düzeltilenler:
  - SLAM modunda sonsuz bekleme (Humble slam_toolbox lifecycle düğümü değil, `get_state` servisi yok).
  - **Düğüm kesilince aktif Nav2 hedefi iptal edilmiyordu** (gerçek robotta araç son hedefe gitmeye devam ederdi).
    Kök neden ölçümle bulundu: rclpy'nin kendi SIGINT işleyicisi bağlamı hemen kapatıyor, sonrasında `rclpy.ok()`
    False dönüyor ve iptal isteği gönderilemiyor. Çözüm `SignalHandlerOptions.NO` ve yarıda kalan her çıkışta iptal.
    Uçtan uca doğrulandı: `bt_navigator: Goal canceled`, `controller_server: Goal was canceled. Stopping the robot.`
- Ackermann için ayrı görev: `config/sim_patrol_ackermann.yaml`. Waypoint yönelimleri gidiş yönüne teğet, yani
  araç yerinde dönmek zorunda kalmıyor. Varsayılan görevdeki keyfi yönelimler Ackermann'a dar alanda üç noktalı
  manevra yaptırıyor ve bu güvenilir değil (bkz. açık sorunlar).

## Jetson Docker (`limo_ws/docker/`, agy yazdı, ben denetleyip düzelttim)

- `Dockerfile.jetson`: `dustynv/ros:humble-ros-base-l4t-r35.4.1` tabanı (etiketler Docker Hub'da doğrulandı). Bu
  imajlarda Nav2 **yok** (jetson-containers `version.py` ve `ros2_build.sh`); Nav2, slam_toolbox ve robot_localization
  kaynaktan derleniyor. Yanlarında YDLidar-SDK ve ydlidar_ros2_driver (humble dalı) da derleniyor.
- `docker-compose.yml` profilleri: `limo_base`, `nav`, `slam`, `patrol`.
- Diğer dosyalar: udev kuralı, README.
- Düzelttiğim hatalar: `slam_params_file=` yazılmıştı, ros2 launch sözdizimi `:=` olmalı. SLAM için Nav2 servisi
  (`use_localization:=false`) eksikti. README'de ROS 1 / `ROS_DOMAIN_ID` hakkında yanlış bir not vardı.
- **Derlenmedi**: bu makine amd64, arm64 emülasyonu yok, disk %95 dolu. Yalnızca `docker build --check` ve
  `docker compose config` ile statik denetim yapıldı. Risk: `nav2_mppi_controller`'ın xtensor bağımlılığı Ubuntu
  20.04'te sorun çıkarabilir.

## Açık sorunlar / robotta doğrulanacaklar

1. **Ackermann dar alan manevrası**: duvar dibinde yön çevirme (bench'teki `baslangica_donus`, 0.55 m önünde duvar
   varken 180° dönüş) hâlâ %50 civarında başarısız. Varsayılan devriye görevinde Ackermann takıldı. Humble MPPI geri
   segmentlerde donuyor, RPP salınıyor. Öneriler: görevleri ileri akışlı tasarlamak, arka görüş (lidar açısını
   robotta ölçmek, T-mini Pro 360° döner, gövde ne kadarını kapatıyor?) ya da mümkünse diff modunu kullanmak.
2. **Ackermann + SLAM**: slam_toolbox'un map→odom dönüşümü, aracın bölme boyunca doğuya ilk 1 m'lik hareketinde
   y'de +0.485 m sıçrıyor (ölçüm: `bench/final/ack_loop_slam4_tf.json`). Araç kendini hedefte sanıyor, kalan 50° yön
   farkını Ackermann ile yerinde düzeltemiyor ve donuyor. Diff + SLAM'de görülmedi. Olası nedenler (doğrulanmadı):
   özelliksiz sim arenasında tarama eşleme dejenerasyonu ve Ackermann'ın hızlı eğrisel hareketi. Gerçek ortamda
   ayrıca denenmeli.
3. Sim lidarı ±120°, gerçek yapılandırma ±100° (sim biraz iyimser).
4. Gerçek robotta doğrulanmamış olanlar: limo_base sürücüsü (seri port yok), lazer z yüksekliği, taban portu
   `ttyTHS0`/`ttyTHS1`, lidar udev ID'leri, yaw ölü bandının varsayılanının 0 olması (davranış değişikliği).
5. Upstream `limo_ackerman*.xacro` ve `limo_steering_hinge.xacro` dosyalarında aynı tekerlek kayması duruyor
   (bizim sim yolu kullanmıyor).
6. `src/limo_bringup/launch/` altında yanlışlıkla oluşmuş `build/`, `install/`, `log/` dizinleri var (silmedim).

## Önerim

- **Devriye ve otonom görevler için şimdilik diff modu** kullanılmalı: bench 18/18, AMCL ve SLAM devriyeleri 2'şer
  koşuda hatasız.
- Ackermann AMCL + bilinen haritayla, **ileri akışlı görevlerle** (`sim_patrol_ackermann.yaml` gibi) güvenilir
  çalışıyor. Dar alanda yön çevirme ve SLAM'le haritalama sırasında navigasyon henüz güvenilir değil.
- Robotta ilk sırada: lidarın gerçek görüş açısı (arka kör alan), taban seri portu ve sürücü, lazer montaj yüksekliği.

## Değişen dosyalar (gece)

- `limo_car/gazebo/ackermann.xacro`: direksiyon eklemi dinamiği, tekerlek çarpışma konumu.
- `limo_description/urdf/limo_xacro.xacro`: diff tekerlek çarpışma konumu.
- `limo_description/urdf/limo_four_diff.xacro`: diff sürüş eklentisi planar_move + tekerlek joint_states, tekerlek sürtünme yönü.
- `limo_bringup/param/nav2_{ackermann,diff}.yaml`, `behavior_trees/navigate_*_ackermann.xml`.
- `limo_bringup/launch/limo_sim_nav2.launch.py`: `nav_to_pose_bt` argümanı eklendi.
- `limo_bringup/scripts/`: `nav_bench.py` artık plan geometrisini ve komutları kaydediyor. Yeni betikler:
  `steer_step.py`, `steer_step_run.sh`, `patrol_run.sh`. `bench_run.sh`'a `BENCH_BT` eklendi.
- `limo_autonomy`: `patrol_node.py`, `launch/patrol.launch.py` (localizer, başlangıç pozu, zaman aşımı argümanları),
  `config/sim_patrol_ackermann.yaml`.
- `limo_ws/docker/*`.
- Yedekler ve tüm ölçümler: `/home/gunes/limo_robot/bench/` (`*.bak-0530`, `x4`–`x10`, `patrol`, `final`).

## Tekrar üretmek için

```bash
# 6 hedefli ölçüm turu (3 tekrar)
limo_ws/src/limo_bringup/scripts/bench_many.sh /tmp/b ackermann 3 10
# uçtan uca devriye
MISSION=$(ros2 pkg prefix --share limo_autonomy)/config/sim_patrol_ackermann.yaml \
  limo_ws/src/limo_bringup/scripts/patrol_run.sh /tmp/p ackermann 20 amcl
```
