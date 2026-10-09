#!/usr/bin/env bash
# AMCL'ye başlangıç pozu verir (RViz "2D Pose Estimate" yerine).  Kullanım: poz_ver.sh [x=0] [y=0] [yaw_derece=0]
# Haritalamayı robot başlangıç işaretindeyken başlattıysanız harita orijini = işaret: poz_ver.sh (argümansız).
set -u
source "$(dirname "$(readlink -f "$0")")/ortam.sh"
X=${1:-0}; Y=${2:-0}; YAW=${3:-0}
read QZ QW < <(python3 -c "import math;a=math.radians($YAW);print(math.sin(a/2),math.cos(a/2))")
ros_humble
ros2 topic pub -t 3 -w 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: $X, y: $Y}, orientation: {z: $QZ, w: $QW}}, covariance: [0.0025,0,0,0,0,0, 0,0.0025,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.003]}}" >/dev/null
echo "baslangic pozu verildi: x=$X y=$Y yaw=${YAW}°"
