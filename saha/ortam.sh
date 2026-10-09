# saha/ betiklerinin ortak ayarları — her betik `source "$(dirname "$0")/ortam.sh"` ile yükler.
# Değiştirmek için ortam değişkeni ver, ör.:  ROBOT=agilex@192.168.0.201 LIMO_VERI=/data ./nav_baslat.sh ...
SAHA=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)   # bu klasör
WS=$(cd "$SAHA/.." && pwd)                                             # limo_ws (colcon çalışma alanı)
CFG=$SAHA/config
ROBOT=${ROBOT:-agilex@192.168.0.200}          # Robot_Lab ağında sabit IP
VERI=${LIMO_VERI:-$HOME/limo_veri}            # oturum çıktıları (harita, bag, log) — depoya girmez
DOMAIN=${LIMO_DOMAIN:-77}                     # robot Foxy sürücüleriyle aynı ROS_DOMAIN_ID
mkdir -p "$VERI"

ros_humble() {   # PC tarafı ROS ortamı (~/.bashrc ROS_LOCALHOST_ONLY=1 koyuyorsa robotu görmez: 0 şart)
  set +u; source /opt/ros/humble/setup.bash; [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"; set -u
  export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=0; unset FASTRTPS_DEFAULT_PROFILES_FILE
}

# Robotta Foxy ortamıyla komut çalıştır: robot_cmd '<komutlar>'
robot_cmd() {
  ssh -o BatchMode=yes -o ConnectTimeout=5 "$ROBOT" "source /opt/ros/foxy/setup.bash; source ~/limo_ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=0; $1"
}

# Arka planda (&) başlatılan launch'lar SIGINT'i yok sayıyor (SigIgn bit 2): önce INT, kapanmazsa TERM,
# en son launch'ın sahipsiz bıraktığı düğümler için desenle TERM.
kapat() {  # kapat <pkill deseni> [düğüm deseni]
  pkill -INT -f "$1"; for _ in 1 2 3 4 5 6; do pgrep -f "$1" >/dev/null || break; sleep 0.5; done
  pkill -TERM -f "$1"; sleep 2
  [ -n "${2:-}" ] && pkill -TERM -f "$2"
  return 0
}

# base_link -> camera_link (Orbbec sürücüsünün kök çerçevesi; sürücü camera_link -> depth/color/optical zincirini
# kendisi yayınlar, bu halka eksikti). x, y: resmi AgileX URDF; z, roll, pitch: zemin düzleminden ölçüldü
# (analiz/zemin_egimi.py, 09.10). Değerler ve kaynakları: config/kamera_kalibrasyon.yaml.
# nav_baslat.sh / harita_baslat.sh çağırır; robottaki kayıt /tf_static'e alır.
kamera_tf_yayinla() {  # kamera_tf_yayinla <log dosyası>
  ros2 run tf2_ros static_transform_publisher --x 0.084 --y 0 --z 0.028 --roll 0.0075 --pitch -0.0248 --yaw 0 \
    --frame-id base_link --child-frame-id camera_link > "$1" 2>&1 &
}

# base_link -> laser_duz: lidarın ölçülen dış kalibrasyonu (analiz/lidar_kalibrasyon.py, 09.10 üç tur:
# yaw -9.66° [IQR -10.1..-9.1], x 0.137 [0.124..0.149], y 0.015 [0.003..0.034] m; dönme merkezine göre).
# Fabrika laser_frame TF'i (0, 0, 0.02, yaw 0) yanlış; scan_sabitle.py taramayı laser_duz çerçevesinde yayınlar.
lidar_tf_yayinla() {  # lidar_tf_yayinla <log dosyası>
  ros2 run tf2_ros static_transform_publisher --x 0.137 --y 0.015 --z 0.02 --roll 0 --pitch 0 --yaw -0.1686 \
    --frame-id base_link --child-frame-id laser_duz > "$1" 2>&1 &
}
