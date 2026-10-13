# Derinlik kamerası testleri (Orbbec Dabai DC1 → D435 karşılaştırması)

Aynı oda, aynı robot, aynı betikler: D435 takıldığında aşağıdaki testler aynı komutlarla tekrarlanır ve
sonuç tabloları yan yana konur. Tüm kayıtlar robotun diskine (`kayit_baslat.sh`), sonra PC → Drive.

## Ön koşul

```bash
SURE=5400 saha/nav_baslat.sh ~/limo_veri/oturum_20261009_162308/harita.yaml   # sürücüler + Nav2 (çarpışma monitörü)
saha/poz_ver.sh                                                                # robot başlangıç işaretinde, +x'e bakıyor
SURE=5000 saha/kamera_baslat.sh
```

Başlangıç işaretinde lidar: ön 3.85 m (eşyalı), sol 1.86 m, sağ 0.72 m (odanın uzun düz duvarı).

## Testler

| # | Test | Komut | Analiz | Ölçüt |
|---|---|---|---|---|
| 1 | Ayar karşılaştırması (sabit sahne, 8 s/ayar) | `saha/kamera_ayar_testi.sh` | `analiz/derinlik_karakter.py <dizin>` | kare hızı, doluluk, zamansal σ (mesafe aralığına göre), kırpışma, uçan piksel |
| 2 | Duvar mesafe testi (sağ duvar, 0.35–2.0 m, 4 s/durak) | `kayit_baslat.sh duvar; duvar_testi.py --don -90 --mesafeler 0.35,0.5,0.7,1.0,1.3,1.6,2.0 --geri_max 1.4; kayit_bitir.sh` | `analiz/duvar_analiz.py <kayit>` | lidara göre sapma (cm, %), düzlem RMS, zamansal σ, duvar doluluğu |
| 3 | Dikdörtgen tur, hız taraması (0.15 / 0.30 m/s) | `dikdortgen_tur.sh --hiz 0.30 --donus 0.6` | `bag2vlmaps.py` + `vlmaps_dogrula.py` | derinlik-renk hizası, zemin düzlüğü, kamera-lidar örtüşmesi, başlangıca dönüş |
| 4 | VLMaps süzgeçleri (aynı tur) | `bag2vlmaps.py ... --zmax 3.5 --kenar 0.04 --max_w 0.8` | `vlmaps_dogrula.py` | kamera-lidar örtüşmesi (<5 cm oranı) |

D435 için değişecekler: `kamera_baslat.sh` (realsense2_camera), konu adları (`/camera/color/image_raw`,
`/camera/depth/image_rect_raw` → `kayit_baslat.sh`), `config/kamera_kalibrasyon.yaml` (base_link → kamera; zemin
yöntemiyle yeniden ölçülmeli: `analiz/zemin_egimi.py`).

## Sonuçlar — Orbbec Dabai DC1 (13.10.2026)

### 1. Ayar karşılaştırması (sabit sahne, robot başlangıç işaretinde, 8 s/ayar) — `ayar_20261013_152500`

| ayar | boyut | doluluk | σ 0.3–1 m | σ 1–2 m | σ 2–4 m | σ 4–8 m | kırpışma | uçan piksel |
|---|---|---|---|---|---|---|---|---|
| varsayılan | 640×400 | 0.796 | 2.51 mm (0.39%) | 3.77 mm (0.30%) | 28.4 mm (0.94%) | 89.5 mm (1.74%) | 0.166 | 0.016% |
| `depth_registration:=true` (HW hiza) | 640×480 | 0.657 | 2.77 | 3.85 | 28.8 | 88.8 | 0.147 | **0.006%** |
| `enable_soft_filter:=false` | 640×400 | 0.803 | 2.54 | 3.83 | 28.1 | 88.5 | 0.174 | 0.050% |
| `enable_ldp:=false` | 640×400 | 0.796 | 2.48 | 3.77 | 27.9 | 88.3 | 0.164 | 0.017% |

- Ayarlar gürültüyü değiştirmiyor; σ mesafeyle ~z² büyüyor (yapılandırılmış ışık): 2 m'nin üstü VLMaps için gürültülü.
- HW hizalama uçan pikselleri ~3× azaltıyor; doluluk düşüşü renk görüntüsünde derinlik görüş alanı dışındaki şeritten.
- Yumuşak süzgeç kapalıyken uçan piksel 3× artıyor → açık kalmalı (varsayılan).
- Kare hızı ölçümü kısa kayıtta güvenilmez (kayıt açılışı); turlarda derinlik 29.9–30.0, renk 26–30 Hz.

### 2. Duvar mesafe testi (odanın uzun düz duvarı, robot dik) — `duvar_20261013_153501`

| lidar (m) | kamera merkez (m) | fark | düzlem RMS (merkez 120×120) | zamansal σ | doluluk |
|---|---|---|---|---|---|
| 0.375 | 0.330 | −4.5 cm | 0.4 mm | <0.5 mm | 1.00 |
| 0.499 | 0.453 | −4.6 cm | 0.6 mm | <0.5 mm | 1.00 |
| 0.698 | 0.638 | −6.0 cm | 1.3 mm | <0.5 mm | 1.00 |
| 0.996 | 0.937 | −5.9 cm | 9.4 mm | <0.5 mm | 1.00 |
| 1.301 | 1.256 | −4.6 cm | 11.5 mm | 0.5 mm | 1.00 |
| 1.599 | 1.498 | −10.1 cm | 11.1 mm | 1.5 mm | 1.00 |
| 1.999 | 1.888 | −11.1 cm | 12.0 mm | 2.6 mm | 1.00 |

- Mesafe = base_link'ten; lidar tam ön ±3° (+0.137 m), kamera görüntü merkezi (+0.084 m, CAD).
- **0.4–1.3 m'de sabit ~−5 cm fark**: büyük olasılıkla kamera/lidar x ofseti (kamera x CAD değeri, ölçülmedi). Mesafeyle
  değişen kısım (1.6–2.0 m'de ek ~−5 cm, ~%3) derinlik ölçek hatası olabilir — **doğrulanmadı**: kamera x'i ölçülmeli
  (ör. bilinen mesafede düz hedef + metre).
- Zamansal σ yakında 0 (Dabai mm adımlı, sabit sahne); 2 m'de 2.6 mm (ayar testindeki 1–2 m σ ile uyumlu).
- Düzlem RMS 1 m ve ötesinde ~10 mm'ye sıçrıyor: merkez bölgesi genişledikçe duvardaki nesneler girmiş olabilir
  (görüntüden **doğrulanmadı**).
- İlk analiz denemesi (lidar ±25° duvar doğrusu) duvar açısını mesafeyle 5–12° değişken buldu → bırakıldı.

### 3. Hız taraması (dikdörtgen tur)

| kayıt | hız | köşe hatası | derinlik / renk Hz |
|---|---|---|---|
| `dikdortgen_20261013_144019` | 0.15 m/s | ≤2.1 cm, ≤0.6° | 30.0 / 27.3 |
| `dikdortgen_20261013_153935` | 0.30 m/s, dönüş 0.6 rad/s | ≤0.6 cm, ≤0.8° | 29.2 / 26.4 |

`vlmaps_dogrula.py` (bag2vlmaps sahnesi):

| ölçüt | 0.15 m/s (her 2. kare) | 0.30 m/s (her 3. kare) |
|---|---|---|
| derinlik-renk hizası | medyan 1.9 px, ≤3 px %76 | medyan 1.9 px, ≤3 px %77 |
| zemin yüksekliği | −0.1 cm (−0.6..+1.1) | 0.0 cm (−0.5..+1.1) |
| kamera-lidar örtüşmesi | medyan 0.5 cm, <5 cm %88 (en iyi yaw −0.5°) | medyan 0.5 cm, <5 cm %88 (en iyi yaw −1.0°) |
| başlangıca dönüş: lidar ICP / odom | 0.8 cm, −1.2° / 0.7 cm, −0.6° | **5.8 cm**, −0.9° / 0.3 cm, −0.5° |

- Kamera tarafı 0.30 m/s'de bozulmuyor (hiza, zemin, örtüşme aynı).
- **0.30 m/s'de odom 6 cm kayıyor ve bunu fark etmiyor** (odom 0.3 cm diyor, lidar 5.8 cm): tekerlek kayması.
  VLMaps için ya 0.15 m/s ya da lidar/ICP düzeltmeli poz.
- Kamera yaw ofseti iki turda −0.5° / −1.0°: kalibrasyon yaw'ı ~−0.7° düzeltilebilir (ölçüt üzerindeki etkisi küçük).

### VLMaps süzgeçleri (0.30 m/s turu, aynı kareler)

| | A: süzgeçsiz | B: `--zmax 3.5 --kenar 0.04 --max_w 0.8` |
|---|---|---|
| yazılan kare | 534 | 532 (2 hızlı dönüş atıldı) |
| zemin | 0.0 cm (−0.5..+1.1) | +0.1 cm (−0.5..+1.2) |
| kamera-lidar <5 cm | %88 (medyan 0.5 cm) | %88 (medyan 0.5 cm) |
| hiza | 1.9 px | ölçülemedi: `--kenar 0.04` gerçek derinlik kenarlarını da siliyor |

- Süzgeçler bu ölçütle **ölçülebilir iyileşme getirmedi**. Örtüşmeyen %12 uzak/uçan piksel değil; büyük olasılıkla
  lidar düzleminin görmediği 0.1–0.5 m yükseklikteki nesneler (masa ayağı vb.) — doğrulanmadı.
- `--kenar` daha küçük oranla (ör. 0.02) ya da yalnız tek taraflı sıçramada denenmeli; `--zmax` VLMaps ızgarasında
  (gürültü 2–4 m'de 28 mm) yine mantıklı, etkisi gerçek VLMaps haritasında ölçülmeli.

### 4. Yerinde dönüş — `donus_20261013_160413`

360° sola 0.3 rad/s (20.2 s), 360° sağa 0.8 rad/s (8.8 s); derinlik 29.8, renk 28.5 Hz. Henüz analiz edilmedi
(kayıt robottan Drive'a aktarılıyordu). Analiz: `bag2vlmaps.py` + `vlmaps_dogrula.py` hiza ölçütü dönüş hızına göre.

### 5. Kapsama turu — `kapsama_20261013_160700` (dikdörtgen 2×1 m + her köşede 360° @0.5 rad/s)

`kapsama_turu.py`: 136 s, 4.9 GB; derinlik 29.1, renk 25.8 Hz; başlangıca odom farkı 0.5 / 6.6 cm, −0.2°.
Tüm duvarlar 4 noktadan 360° görüldü (VLMaps için en iyi kapsama). Henüz sahneye dönüştürülmedi.

## Bilinen sorunlar / sonraki adımlar

- **Robot içi DDS:** sensör kaydedicisi paylaşımlı bellekte /odom /imu'yu almıyordu → `config/fastdds_udp.xml` (kayit_baslat.sh).
- **Kamera ilk açılışta yayın yapmayabiliyor** ("color frame is not decoded"): kamera_dur.sh + kamera_baslat.sh.
- **Wi-Fi darboğazı:** robot→PC ~6 MB/s, kamera kaydı 41 MB/s → 1 dk kayıt ≈ 7 dk aktarım. Robot diski (98 GB, ~10 GB boş)
  günde ~2 kamera turuna yetiyor. Drive (rclone ortak istemci kimliği) sık 403 rateLimitExceeded veriyor: kendi
  Google Cloud client ID'si ile çözülür.
- **Yapılmadı:** HW hizalamalı tur (`depth_registration:=true`; bag2vlmaps artık destekliyor), kamera x ofsetinin
  ölçümü (duvar testindeki −5 cm), dönüş/kapsama kayıtlarının analizi, gerçek VLMaps (LSeg) haritası.