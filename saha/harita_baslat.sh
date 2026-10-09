#!/usr/bin/env bash
# Gerçek LIMO haritalama oturumunu başlatır:
#   robotta (Foxy, domain 77): limo_base (ttyTHS0) + ydlidar T-mini   [en fazla 30 dk, sonra kendiliğinden kapanır]
#   bu PC'de (Humble): slam_toolbox (config/slam_limo_v2.yaml, canlı) + rosbag kaydı
# Sürüş (ayrı terminal; ~/.bashrc ROS_LOCALHOST_ONLY=1 koyduğu için 0 şart):
#   source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=0 && \
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.5 Bitirmek için: harita_bitir.sh
set -u
source "$(dirname "$(readlink -f "$0")")/ortam.sh"
TS=$(date +%Y%m%d_%H%M%S); OTURUM=$VERI/oturum_$TS; mkdir -p "$OTURUM"
echo "$OTURUM" > "$VERI/aktif_oturum"
scp -q -o BatchMode=yes "$CFG/tmini_ros2.yaml" "$ROBOT:/tmp/tmini_ros2.yaml" && scp -q -o BatchMode=yes "$SAHA/robot/odom_duzelt.py" "$ROBOT:/tmp/odom_duzelt.py" || { echo "robota erisilemedi"; exit 1; }
# Denetim ayrı ssh çağrısında: aynı komut satırı başlatma metnini de içerirse pgrep kendi kabuğunu eşleştirir.
ssh -o BatchMode=yes "$ROBOT" 'pgrep -f "limo_base[.]launch|open_ydlidar_launc[h]|odom_duzel[t].py" >/dev/null' \
  && { echo "UYARI: robotta surucu zaten calisiyor, iptal"; exit 3; }
ssh -o BatchMode=yes "$ROBOT" 'source /opt/ros/foxy/setup.bash; source ~/limo_ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=0
  (timeout -s INT 1800 ros2 launch limo_base limo_base.launch.py port_name:=ttyTHS0 pub_odom_tf:=False > /tmp/harita_base.log 2>&1 &)
  (timeout -s INT 1800 python3 /tmp/odom_duzelt.py > /tmp/harita_odom_duzelt.log 2>&1 &)
  (timeout -s INT 1800 ros2 launch limo_base open_ydlidar_launch.py params_file:=/tmp/tmini_ros2.yaml > /tmp/harita_lidar.log 2>&1 &)
  echo "robot suruculeri basladi"' || exit 3
sleep 9
set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=0; unset FASTRTPS_DEFAULT_PROFILES_FILE
kamera_tf_yayinla "$OTURUM/kamera_tf.log"
# T-mini tarama başına 654/656/658 nokta veriyor; Karto sabit sayı istiyor -> sabit 656'lık ızgaraya örnekle
python3 "$SAHA/scan_sabitle.py" 656 > "$OTURUM/scan_sabitle.log" 2>&1 &
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$CFG/slam_limo_v3.yaml \
  use_sim_time:=false > "$OTURUM/slam.log" 2>&1 &
ros2 bag record -o "$OTURUM/bag" /scan /odom /odom_duz /imu /tf /tf_static /cmd_vel /map /limo_status > "$OTURUM/bag.log" 2>&1 &
sleep 6
echo "== kontrol"
timeout 6 ros2 topic hz /scan 2>&1 | grep -m1 "average rate" || echo "UYARI: /scan gelmiyor"
timeout 6 ros2 topic hz /odom 2>&1 | grep -m1 "average rate" || echo "UYARI: /odom gelmiyor"
grep -m1 -iE "Registering sensor|error" "$OTURUM/slam.log" || echo "slam: henuz tarama islenmedi"
echo "Oturum: $OTURUM"
