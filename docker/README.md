# LIMO Jetson ROS 2 Humble Docker Ortamı

Bu dizin, Jetson (Ubuntu 20.04, JetPack 5 / L4T r35) üzerinde LIMO robotu için ROS 2 Humble konteyner ortamını kurmak ve çalıştırmak için gerekli yapılandırmaları içerir.

## 1. L4T Sürümünü Belirleme ve Taban İmaj Seçimi

Robot üzerindeki JetPack/L4T sürümünü öğrenmek için şu komutu çalıştırın:

```bash
cat /etc/nv_tegra_release
```

Çıktıdaki sürüme uygun taban imaj etiketini seçin:
- **R35.4.1** için: `dustynv/ros:humble-ros-base-l4t-r35.4.1` (varsayılan)
- **R35.3.1** için: `dustynv/ros:humble-ros-base-l4t-r35.3.1`
- **R35.2.1** için: `dustynv/ros:humble-ros-base-l4t-r35.2.1`

> **Not:** Bu taban imajlarda ROS 2 kaynaktan `/opt/ros/humble/install` altına derlenmiştir. Ubuntu 20.04 tabanlı olduğundan `apt install ros-humble-*` paketleri çalışmaz.

## 2. udev Kuralı Kurulumu

LiDAR'ın `/dev/ydlidar` olarak tanınması için udev kuralını sisteme kopyalayın:

```bash
sudo cp docker/99-limo.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

*Not:* Cihazın vendor ve product ID değerlerini doğrulamak için `udevadm info -a -n /dev/ttyUSB0` çıktısını kontrol edin.

## 3. .dockerignore Dosyasını Kopyalama

Derleme bağlamının (build context) gereksiz yere büyümesini önlemek için `.dockerignore` dosyasını çalışma alanı kök dizinine kopyalayın:

```bash
# limo_ws kök dizinindeyken:
cp docker/dockerignore.txt .dockerignore
```

## 4. Docker İmajını Derleme

İmajı `limo_ws` kök dizininden derleyin:

```bash
docker build -f docker/Dockerfile.jetson \
  -t limo:humble-jetson \
  --build-arg BASE_IMAGE=dustynv/ros:humble-ros-base-l4t-r35.4.1 \
  .
```

### Önemli: Kaynak Derleme Süresi ve Swap Bellek Önerisi

- Nav2, SLAM ve diğer ROS 2 paketlerinin Jetson üzerinde kaynaktan derlenmesi **uzun sürer (saatler alabilir)**.
- Derleme sırasında yetersiz bellekten (OOM) kaynaklı kilitlenmeleri önlemek için Jetson üzerinde **en az 4-8 GB swap** alanı oluşturulması şiddetle önerilir:

```bash
# 8 GB swap oluşturma örneği:
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

## 5. Docker Compose ile Başlatma

Konteyner servislerini çalıştırmak için `docker-compose.yml` yapılandırmasını kullanabilirsiniz:

```bash
# Sadece robot tabanı ve sensörleri başlatma (limo_base)
docker compose up limo_base

# Nav2 navigasyon yığını ile birlikte başlatma (Ackermann modu)
docker compose --profile nav up

# Farklı sürüş modu veya harita belirtmek için ortam değişkenleri kullanılabilir:
LIMO_MODE=diff LIMO_MAP=yeni_harita.yaml docker compose --profile nav up

# slam_toolbox ile haritalama (SLAM) modunda başlatma
docker compose --profile slam up

# Otonom devriye (patrol) modunda başlatma
docker compose --profile patrol up
```

## 6. ROS 1 ile Birlikte Çalışma ve ros1_bridge Notu

- ROS 1 (Noetic) master tabanlıdır, DDS kullanmaz; `ROS_DOMAIN_ID` ROS 1 ile çakışmayı ne yaratır ne önler.
  Gerçek çakışma donanımdadır: ROS 1 `limo_base` ve lidar sürücüsü çalışırken `/dev/ttyTHS0` ve `/dev/ydlidar`
  meşguldür. Konteyneri başlatmadan önce ROS 1 tarafındaki sürücüleri durdurun.
- `ROS_DOMAIN_ID`, aynı ağdaki başka ROS 2 makineleri (ör. geliştirme bilgisayarı) ile ayrışmak içindir.
- **Bu imajda `ros1_bridge` BULUNMAMAKTADIR.**

## 7. RealSense Kamera Sürücüsü Notu

- Intel RealSense (D435) kamera sürücüsü (`realsense2_camera`) **bu Docker imajına HENÜZ eklenmemiştir** ve ayrı bir kurulum adımı gerektirir.

## 8. SLAM ile Navigasyon

`--profile slam` hem `slam` (slam_toolbox) hem `nav2_slam` (Nav2, `use_localization:=false`) servislerini açar.
Devriyeyi SLAM ile kullanacaksanız `patrol` komutuna `localizer:=slam_toolbox` ekleyin (Humble slam_toolbox
lifecycle düğümü değildir; devriye düğümü bu durumda yalnız bt_navigator'ı bekler).

## 9. Doğrulanmamış Noktalar / Bilinen Riskler

Bu dosyalar Jetson'da henüz DERLENMEDİ (geliştirme makinesi amd64, arm64 emülasyonu yok). Yalnız
`docker build --check` ve `docker compose config` ile statik olarak denetlendi.
- `nav2_mppi_controller` Humble'da xtensor'a bağlıdır; Ubuntu 20.04 deposundaki libxtensor-dev sürümü yetersiz
  kalabilir. Derleme burada düşerse xtensor/xtl'yi kaynaktan kurmak gerekir (doğrulanacak).
- `rosdep install`, taban imajda kaynaktan kurulu ROS paketlerini `AMENT_PREFIX_PATH` üzerinden atlar; atlanmayan
  bir anahtar `ros-humble-*` apt paketi ararsa ilgili anahtarı `--skip-keys` listesine ekleyin.
- LIMO taban seri portu `ttyTHS0` (robotta görülen); upstream `ttyTHS1` yazıyordu — robotta doğrulayın.
