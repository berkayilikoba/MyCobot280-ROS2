#!/usr/bin/env bash
# dikdortgen.py'yi gerçek robottan önce simde dener: headless Gazebo (diff) + AMCL + Nav2, sonra tur.
# Sim dünyasında başlangıcın 0.6 m solunda duvar var: varsayılan sağa dönüş (--sag). Ek argümanlar dikdortgen.py'ye.
# Kullanım: sim_dikdortgen.sh [--cerceve odom] [--tur 2] ...      Çıktı: köşe hataları ~/limo_veri/sim_dikdortgen_<zaman>.csv
set -u
source "$(dirname "$(readlink -f "$0")")/ortam.sh"
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID=${SIM_DOMAIN:-42} ROS_LOCALHOST_ONLY=1 GAZEBO_MASTER_URI=http://127.0.0.1:$((11345 + ${SIM_DOMAIN:-42}))
TS=$(date +%Y%m%d_%H%M%S); LOG=$VERI/sim_dikdortgen_$TS.log; CSV=$VERI/sim_dikdortgen_$TS.csv
setsid ros2 launch limo_bringup limo_sim_nav2.launch.py mode:=diff localization:=amcl headless:=true use_rviz:=false \
  nav2_params_file:="${SIM_PARAMS:-$WS/src/limo_bringup/param/nav2_diff.yaml}" > "$LOG" 2>&1 &
L=$!; PG=$(ps -o pgid= -p $L | tr -d ' ')
trap 'kill -INT -- -$PG 2>/dev/null; sleep 5; kill -KILL -- -$PG 2>/dev/null' EXIT
for _ in $(seq 120); do grep -q "lifecycle_manager_localization.*Managed nodes are active" "$LOG" && break; sleep 0.5; done
ros2 topic pub -t 3 -w 0 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}, covariance: [0.01,0,0,0,0,0, 0,0.01,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.007]}}" >/dev/null
for _ in $(seq 120); do grep -q "lifecycle_manager_navigation.*Managed nodes are active" "$LOG" && break; sleep 0.5; done
sleep 3
timeout 300 python3 "$SAHA/dikdortgen.py" --sim --sag --log "$CSV" "$@" 2>&1 | grep -vE "bayat"
echo "köşe hataları: $CSV   (launch günlüğü: $LOG)"
