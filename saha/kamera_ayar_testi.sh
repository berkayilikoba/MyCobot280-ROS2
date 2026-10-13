#!/bin/bash
# Orbbec Dabai ayar karşılaştırması: robot DURURKEN aynı sahneden her ayarla kısa kamera kaydı (robotta ~/limo_kayit/ayar_<zaman>/<ad>).
# Kullanım: kamera_ayar_testi.sh [saniye]   Sonra: analiz/derinlik_karakter.py <dizin>
source "$(dirname "$0")/ortam.sh"
SN=${1:-8}; Z=$(date +%Y%m%d_%H%M%S); D=/home/agilex/limo_kayit/ayar_$Z
scp -q -o BatchMode=yes "$(dirname "$0")/config/kamera_qos.yaml" "$ROBOT:/tmp/" || exit 1
declare -A AYAR=(
  [varsayilan]=""
  [hw_hiza]="depth_registration:=true"
  [filtre_yok]="enable_soft_filter:=false"
  [ldp_yok]="enable_ldp:=false"
)
for ad in varsayilan hw_hiza filtre_yok ldp_yok; do
  echo "== $ad ${AYAR[$ad]}"
  robot_cmd "(timeout -s INT 60 ros2 launch orbbec_camera dabai.launch.py enable_point_cloud:=false enable_ir:=false ${AYAR[$ad]} > /tmp/kamera_$ad.log 2>&1 &); sleep 14; mkdir -p $D
    timeout -s INT $SN ros2 bag record -o $D/$ad --qos-profile-overrides-path /tmp/kamera_qos.yaml /camera/color/image_raw /camera/depth/image_raw /camera/color/camera_info /camera/depth/camera_info /tf_static > /dev/null 2>&1
    pkill -INT -f orbbec_camera_node; pkill -INT -f 'dabai.launch'; sleep 4; pkill -9 -f orbbec_camera_node; grep -m1 -iE 'not decoded' /tmp/kamera_$ad.log; ros2 bag info $D/$ad 2>/dev/null | grep -E 'Topic: /camera/(color|depth)/image_raw' | sed 's/Serialization.*//'"
done
echo "$D"
