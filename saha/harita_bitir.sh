#!/usr/bin/env bash
# Haritayı kaydeder (pgm/yaml + slam_toolbox poz grafiği), kayıtları ve robot sürücülerini kapatır.
set -u
source "$(dirname "$(readlink -f "$0")")/ortam.sh"
OTURUM=$(cat "$VERI/aktif_oturum")
set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=0; unset FASTRTPS_DEFAULT_PROFILES_FILE
# robotu durdur (güvenlik): birkaç sıfır komutu
timeout 5 ros2 topic pub -r 10 -t 10 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
timeout 30 ros2 run nav2_map_server map_saver_cli -f "$OTURUM/harita" --ros-args -p map_subscribe_transient_local:=true 2>&1 | grep -iE "saved|error" | head -2
timeout 15 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$OTURUM/harita_posegraph'}" >/dev/null 2>&1 \
  && echo "poz grafigi kaydedildi"
# Süreçleri desenle kapat ([x] hilesi: pkill kendi kabuk satırını eşleştirmesin)
pkill -INT -f "ros2 bag recor[d]"; pkill -INT -f "scan_sabitl[e].py"
kapat "online_async_launc[h]" "async_slam_toolbo[x]_node"
sleep 4
ssh -o BatchMode=yes "$ROBOT" 'pkill -INT -f "limo_base[.]launch"; pkill -INT -f "open_ydlidar_launc[h]"; pkill -INT -f "odom_duzel[t].py"'
sleep 4   # denetim ayrı ssh çağrısında: aynı satırda olunca grep pkill komut satırını eşleştiriyordu
ssh -o BatchMode=yes "$ROBOT" 'pgrep -fa "limo_bas[e]|ydlidar_ros[2]|odom_duzel[t]" || echo "robotta surucu kalmadi"'
ros2 bag info "$OTURUM/bag" 2>/dev/null | grep -E "Duration|Bag size"
ls "$OTURUM"
