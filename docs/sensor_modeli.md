# LIMO sensör modeli: gerçek ölçüm → simülasyon

Kaynak: 24.09.2026 saha kayıtları, robotun **durduğu** aralıklar (odom hızı ≈ 0)
- `saha/oturum_20260924_163343/bag` (haritalama; Wi-Fi kaybı yok) — lidar için esas
- `saha/nav_20260924_164729/bag` (Nav2) — IMU için ek

Yöntem: IMU için durağan aralıklarda ortalama/std; lidar için ışınlar 0.5° açı kutularına toplanıp
kutu başına **medyan mutlak sapma** (×1.4826), çünkü ortamda hareket eden insanlar std'yi bozuyordu.
Sim karşılaştırması: `limo_ws/src/limo_bringup/scripts/sensor_olc.py` (aynı yöntem, bench dünyası, 30 s durağan).

## IMU (taban kartı, `/imu`, çerçeve `imu_link`)

| | Gerçek | Sim (önce) | Sim (sonra) |
|---|---|---|---|
| Yayın hızı | ~92 Hz (robotta) | 100 Hz | 90 Hz ayar, 83 Hz ölçülen |
| Jiroskop σ x / y / z [rad/s] | 0.00088 / 0.0016 / 0.00087 | diff 0.0002; Ackermann gürültüsüz* | 0.00087 / 0.0016 / 0.00088 |
| Jiroskop çözünürlüğü | **0.1°/s** (0.001745 rad/s adım) | — | modellenmedi (Gazebo nicemleme yok) |
| Jiroskop sapması | ~8×10⁻⁵ rad/s | — | — (ihmal) |
| İvmeölçer σ [m/s²] | 0.025 her eksende | diff 0.017 | 0.025 |
| İvmeölçer z ortalaması | 10.04 (g=9.81 → %2.4 ölçek) | 9.80 | 9.80 (Gazebo işaretli sapma veremiyor) |
| Duruşta yön kayması | ~0 (kart ölü bant uyguluyor) | 0 | 0 |
| Konu | `/imu` | diff `/imu/data`; Ackermann varsayılan | iki modda `/imu` |

\* Ackermann IMU bloğu ROS 1 etiketleri (`topicName`, `gaussianNoise`) kullanıyordu; ROS 2 eklentisi yok sayıyordu.

**Ackermann gövde titreşimi:** gürültü ayarından bağımsız olarak sim Ackermann aracı dururken titriyordu
(ivme σ z 0.25 m/s², jiroskop σ y 0.017 rad/s — gerçeğin ~10 katı). Kaynak varsayılan (sert) ODE tekerlek teması;
direksiyon/hız PID'i değil (varyant testi `kalibrasyon/ack_titresim`). Tekerleklere `kp 1e6, kd 100, minDepth 0.001`
verilince gerçekle aynı oldu. Direksiyon etkisi: tam kilit yarıçapı 0.465 → 0.475 m (teori 0.473), %63 yanıt 0.21 → 0.38 s.

## Lidar (YDLIDAR T-mini, `/scan`, çerçeve `laser_frame`)

| | Gerçek | Sim (önce) | Sim (sonra) |
|---|---|---|---|
| Tarama hızı | ~6 Hz | Ackermann 8, diff 10 Hz | 6 Hz |
| Nokta sayısı | 650–662 / 360° (taramadan taramaya değişir) | Ackermann 720 / 240°, diff 120 / 360° | 481 / 264° (aynı açısal çözünürlük, ~0.55°) |
| Etkin görüş açısı | **−123° … +141°** (arka ~96° kör) | Ackermann ±120°, diff 360° | −123° … +141° |
| Kendi gövdesini görme | sol arka +141°…+179°, 0.04–0.14 m | yok | kör sektör olarak modellendi |
| Hiç dönmeyen bölge | sağ arka −167°…−123° | yok | kör sektör olarak modellendi |
| Menzil gürültüsü σ | 4.4 mm (≤2 m), 6 mm (2–4 m), 12 mm (4–8 m) | 10 mm | 7 mm sabit (ölçülen 7–8 mm) |
| Yüzeyde kayıp ölçüm | %5–10 | yok | **modellenmedi** (Gazebo ray sensörü desteklemiyor) |
| Menzil sınırı | sürücü 0.02–16 m (T-mini belgesi 12 m) | 0.2–8 m / 0.3–15 m | 0.10–12 m |

Not: gerçek sürücü tarama başına nokta sayısını sabit tutmuyor (654/656/658); slam_toolbox bunu reddettiği için
gerçek robotta `kalibrasyon/scan_sabitle.py` kullanılıyor. Sim sabit 481 nokta verdiği için simde gerekmiyor.

## Taban modeli (diff / FOUR_DIFF): komut → hareket

Kaynak: `saha/oturum_20260924_163343/bag`, teleop komutları (`/cmd_vel`) ve ölçülen tepki (`/odom` v, `/imu` w).
Model: saf gecikme → birinci derece gecikme (τ) → ivme sınırı → kazanç; komut kesilince (watchdog) hedef 0.
Parametreler tüm oturum üzerinde ızgara aramasıyla, en küçük hataya göre bulundu (`kalibrasyon/taban_modeli.json`).

| | v (ileri) | w (dönüş) |
|---|---|---|
| Saf gecikme | 0.30 s | 0.05 s |
| τ | 0.45 s | 0.20 s |
| İvme sınırı | 0.15 m/s² (arama alt sınırı) | 6 rad/s² |
| Kazanç | 0.95 | 1.05 |
| Watchdog (en iyi uyum) | ~1.0 s | ~0.5 s |
| Açıklanan varyans (tüm oturum) | 0.86 | 0.71 |

- Tekerlekler dönerken gözle görülür biçimde kayıyor (4 teker diferansiyelin doğası), ama taban kartı gövde dönüşünü
  kapalı çevrimle sağlıyor: yerinde dönüşte gerçek/komut 1.06–1.12, ilerlerken 0.97; düz gidişte odom/komut 0.98.
- Watchdog donanımda ayrıca test edilmedi; "komut kesilince dur" varsayımı "son komut sürer"den belirgin daha iyi uyuyor.
- Simde uygulama: `limo_kinematics/taban_modeli.py` + `taban_modeli` düğümü (diff modunda `gazebo_limo_mycobot.launch.py`
  başlatır; planar_move `cmd_vel_taban`'ı dinler).

**Doğrulama (sim ikizi):** kayıttaki 97 s'lik teleop komutları (483–580 s) sime aynen oynatıldı
(`scripts/sim_ikiz.py`), sim tepkisi gerçekle karşılaştırıldı:

| | Eski sim (komut anında) | Yeni sim (taban modeli) |
|---|---|---|
| v R² / RMSE | 0.41 / 0.061 m/s | **0.85 / 0.031 m/s** |
| w R² / RMSE | 0.71 / 0.177 rad/s | **0.95 / 0.074 rad/s** |

Kalan fark: gerçek taban ileri harekete ~0.6 s gecikmeyle ama hızlı bir sıçramayla başlıyor; model daha yumuşak yükseliyor.

## Modellenmeyenler / açık noktalar

- Jiroskop nicemlemesi (0.1°/s) ve IMU yön ölü bandı — Gazebo'da karşılığı yok; gerekirse sim tarafında küçük bir süzgeç düğümü.
- Lidar ışın kaybı %5–10 ve taramadan taramaya değişen nokta sayısı.
- Ackermann modunun taban tepkisi ölçülmedi (kayıtlar FOUR_DIFF modunda).
- Odometri: gerçek kartta konum = taban hızı × IMU yönü; ölçek hatası henüz kesin ölçülmedi (0.98–1.09, mezura testi gerekli).
  Sim odometrisi gerçek konumdan (Gazebo) geliyor, hatasız.
- Wi-Fi gecikmesi (odom 0.03–1.1 s, anlık 5.8 s) simde yok — bu Nav2'nin robotta çalıştırılmasıyla çözülecek, sim ile değil.

## Değişen dosyalar

- `limo_ws/src/limo_car/gazebo/sensor.xacro` — Ackermann lidar + IMU
- `limo_ws/src/limo_car/gazebo/ackermann.xacro` — tekerlek teması
- `limo_ws/src/limo_description/urdf/limo_four_diff.xacro` — diff lidar + IMU
- Yedekler: bu klasörde `*.bak`
