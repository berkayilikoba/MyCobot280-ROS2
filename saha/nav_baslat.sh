#!/usr/bin/env bash
# Gerçek LIMO'da Nav2: robotta Foxy sürücüler (SURE saniye, varsayılan 30 dk); bu PC'de scan süzgeci + Nav2 (AMCL + harita).
# Kullanım: [SURE=saniye] nav_baslat.sh <harita.yaml>     Durdurmak: nav_dur.sh
# SURE: robot sürücüleri bu süre sonra kendiliğinden kapanır (güvenlik; varsayılan 1800 s = 30 dk)
set -u
SURE=${SURE:-1800}
source "$(dirname "$(readlink -f "$0")")/ortam.sh"
HARITA=${1:?harita.yaml}; TS=$(date +%Y%m%d_%H%M%S); OTURUM=$VERI/nav_$TS; mkdir -p "$OTURUM"; echo "$OTURUM" > "$VERI/aktif_nav"
scp -q -o BatchMode=yes "$CFG/tmini_ros2.yaml" "$ROBOT:/tmp/tmini_ros2.yaml" && scp -q -o BatchMode=yes "$SAHA/robot/odom_duzelt.py" "$ROBOT:/tmp/odom_duzelt.py" || { echo "robota erisilemedi"; exit 1; }
ssh -o BatchMode=yes "$ROBOT" 'pgrep -f "limo_base[.]launch|open_ydlidar_launc[h]|odom_duzel[t].py" >/dev/null' && { echo "UYARI: robotta surucu zaten calisiyor"; exit 3; }
ssh -o BatchMode=yes "$ROBOT" 'source /opt/ros/foxy/setup.bash; source ~/limo_ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=0
  (timeout -s INT '"$SURE"' ros2 launch limo_base limo_base.launch.py port_name:=ttyTHS0 pub_odom_tf:=False > /tmp/nav_base.log 2>&1 &)
  (timeout -s INT '"$SURE"' python3 /tmp/odom_duzelt.py > /tmp/nav_odom_duzelt.log 2>&1 &)
  (timeout -s INT '"$SURE"' ros2 launch limo_base open_ydlidar_launch.py params_file:=/tmp/tmini_ros2.yaml > /tmp/nav_lidar.log 2>&1 &)
  echo "robot suruculeri basladi"' || exit 3
sleep 9
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=0; unset FASTRTPS_DEFAULT_PROFILES_FILE
kamera_tf_yayinla "$OTURUM/kamera_tf.log"
lidar_tf_yayinla "$OTURUM/lidar_tf.log"
python3 "$SAHA/scan_sabitle.py" 656 0.22 > "$OTURUM/scan_sabitle.log" 2>&1 &
ros2 launch limo_bringup limo_nav2.launch.py mode:=diff use_sim_time:=false use_localization:=true \
  params_file:=${NAV_PARAMS:-$CFG/nav2_limo_gercek.yaml} map:=$HARITA > "$OTURUM/nav2.log" 2>&1 &
ros2 bag record -o "$OTURUM/bag" /scan /scan_sabit /odom /odom_duz /imu /tf /tf_static /cmd_vel /cmd_vel_nav /plan \
  /amcl_pose /local_costmap/published_footprint > "$OTURUM/bag.log" 2>&1 &
sleep 12
timeout 6 ros2 topic hz /odom_duz 2>&1 | grep -m1 average || echo "UYARI: /odom_duz yok (robotta /tmp/nav_odom_duzelt.log)"
grep -m3 -iE "error|fatal|failed to" "$OTURUM/nav2.log" || echo "nav2: hata yok"
echo "Oturum: $OTURUM"
