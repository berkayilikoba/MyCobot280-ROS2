# MyCobot280-ROS2

Elephant Robotics **myCobot 280** robot kolunu **ROS 2** ile kontrol etmek için hazırlanmış çalışma alanı (workspace).

> ⚠️ Bu README'deki `<...>` ile gösterilen yerleri (paket adı, launch dosyası vb.) kendi repo içeriğine göre doldur.

---

## 1. Gereksinimler

| Bileşen | Önerilen sürüm |
|---|---|
| İşletim sistemi | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Python | 3.10+ |
| Robot | myCobot 280 (M5 / Pi / Jetson Nano) |
| Bağlantı | USB-C kablo (M5 sürümü) veya seri port (Pi sürümü) |

> Farklı bir Ubuntu / ROS 2 dağıtımı kullanıyorsan aşağıdaki `humble` ifadelerini kendi dağıtımınla değiştir.

---

## 2. ROS 2 Kurulumu

ROS 2 henüz kurulu değilse resmi rehberi izle:
<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

Kurulumu doğrula:

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

Her terminalde otomatik yüklenmesi için:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Sistem Bağımlılıkları

```bash
sudo apt update
sudo apt install -y \
  git \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2
```

MoveIt 2 kullanıyorsan (isteğe bağlı):

```bash
sudo apt install -y ros-humble-moveit
```

`rosdep` ilk kez kullanılacaksa:

```bash
sudo rosdep init   # daha önce çalıştırıldıysa hata verebilir, sorun değil
rosdep update
```

---

## 4. Python Bağımlılıkları

Robotla haberleşme için Elephant Robotics'in resmi kütüphanesi gerekir:

```bash
pip3 install pymycobot --upgrade
```

---

## 5. Repoyu Klonlama ve Derleme

```bash
# Workspace oluştur
mkdir -p ~/mycobot_ws/src
cd ~/mycobot_ws/src

# Repoyu klonla
git clone https://github.com/berkayilikoba/MyCobot280-ROS2.git

# ROS bağımlılıklarını otomatik kur
cd ~/mycobot_ws
rosdep install --from-paths src --ignore-src -r -y

# Derle
colcon build --symlink-install

# Workspace'i yükle
source install/setup.bash
```

Workspace'in her terminalde otomatik yüklenmesi için:

```bash
echo "source ~/mycobot_ws/install/setup.bash" >> ~/.bashrc
```

---

## 6. Robot Bağlantısı ve Seri Port Ayarı

1. Robotu USB ile bilgisayara bağla ve aç.
2. Portu bul:

   ```bash
   ls /dev/ttyUSB* /dev/ttyACM*
   ```

   Genellikle `/dev/ttyUSB0` veya `/dev/ttyACM0` olur.

3. Seri porta erişim izni ver (bir kez yapılır, ardından **oturumu kapatıp aç** ya da bilgisayarı yeniden başlat):

   ```bash
   sudo usermod -aG dialout $USER
   ```

4. Anlık geçici izin gerekirse:

   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```

### Bağlantı parametreleri

| Model | Port | Baudrate |
|---|---|---|
| myCobot 280 M5 | `/dev/ttyUSB0` (veya `ttyACM0`) | `115200` |
| myCobot 280 Pi | `/dev/ttyAMA0` | `1000000` |

### Robotun firmware'ini kontrol et

M5 sürümünde robotun **Transponder (Atom) / Basic firmware** modunda olması gerekir. Gerekirse Elephant Robotics'in **myStudio** aracıyla firmware'i yükle:
<https://github.com/elephantrobotics/myStudio>

Bağlantıyı hızlıca test etmek için:

```bash
python3 -c "
from pymycobot.mycobot import MyCobot
mc = MyCobot('/dev/ttyUSB0', 115200)
print(mc.get_angles())
"
```

Açı değerleri yazdırılıyorsa bağlantı çalışıyor demektir.

---

## 7. Çalıştırma

> Aşağıdaki paket ve launch dosyası adlarını repondaki gerçek isimlerle değiştir.

**Robot modelini RViz'de görüntüle:**

```bash
ros2 launch <paket_adi> <display.launch.py>
```

**Gerçek robotu bağla ve kontrol et:**

```bash
ros2 launch <paket_adi> <robot.launch.py> port:=/dev/ttyUSB0 baud:=115200
```

**Çalışan node ve topic'leri kontrol et:**

```bash
ros2 node list
ros2 topic list
ros2 topic echo /joint_states
```

---

## 8. Sık Karşılaşılan Sorunlar

| Sorun | Çözüm |
|---|---|
| `Permission denied: '/dev/ttyUSB0'` | `dialout` grubuna eklendiğinden emin ol, oturumu yeniden aç |
| Port görünmüyor | Farklı bir USB kablo / port dene; `dmesg \| tail` ile cihazın algılandığını kontrol et |
| `ModuleNotFoundError: pymycobot` | `pip3 install pymycobot --upgrade` komutunu çalıştır |
| `Package '<...>' not found` | `source ~/mycobot_ws/install/setup.bash` komutunu çalıştır |
| Robot yanıt vermiyor | Baudrate'i ve firmware modunu kontrol et, robotu yeniden başlat |
| Derleme hatası | `rm -rf build install log` ile temizleyip `colcon build --symlink-install` komutunu tekrar çalıştır |

---

## 9. Proje Yapısı

```
MyCobot280-ROS2/
├── <paket_1>/      # Örn. robot açıklaması (URDF / meshes)
├── <paket_2>/      # Örn. kontrol node'ları
├── .gitignore
└── README.md
```

---

## Lisans

Lisans bilgisini buraya ekle (ör. MIT, Apache-2.0).
