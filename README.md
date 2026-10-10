

```markdown
# MyCobot280-ROS2

Elephant Robotics **myCobot 280** robot kolunu **ROS 2 Humble** ile simülasyon (Gazebo) ve gerçek donanım üzerinde kontrol etmek için hazırlanmış çalışma alanı.

---

## 1. Gereksinimler

| Bileşen | Sürüm / Açıklama |
|---|---|
| İşletim sistemi | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Python | 3.10+ |
| Robot | myCobot 280 (M5 / Pi) |

---

## 2. Kurulum ve Derleme

1. Çalışma alanınızın `src` dizinine gidin ve reponuzu klonlayın:
   ```bash
   cd ~/colcon_ws/src
   git clone [https://github.com/berkayilikoba/MyCobot280-ROS2.git](https://github.com/berkayilikoba/MyCobot280-ROS2.git)

```

2. Bağımlılıkları yükleyin ve çalışma alanını derleyin:
```bash
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

```



---

## 3. Çalıştırma Komutları

### A. Gazebo Simülasyonu ve Pick-and-Place

1. **Simülasyonu Başlatın:**
```bash
ros2 launch mycobot_gazebo gazebo_pick.launch.py

```


2. **RViz Görselleştirmesini Başlatın:**
```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix mycobot_gazebo)/share/mycobot_gazebo/rviz/pick_place.rviz --ros-args -p use_sim_time:=true

```


3. **Pick and Place Betiğini Çalıştırın:**
```bash
ros2 run mycobot_gazebo cm

```



### B. Gerçek Robot Testi (`real_robot`)

1. Robotu USB ile bağlayıp seri port iznini verin:
```bash
sudo chmod 666 /dev/ttyUSB0

```


2. Test düğümünü çalıştırın:
```bash
ros2 run real_robot real_test

```



---

## 4. Proje Yapısı

```text
MyCobot280-ROS2/
├── mycobot_gazebo/    # Gazebo simülasyonu, launch, rviz ve Python betikleri
└── real_robot/        # Gerçek donanım kontrol ve test paketleri

```

```

```
