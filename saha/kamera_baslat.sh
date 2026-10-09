#!/usr/bin/env bash
# Robottaki Orbbec derinlik kamerasını (Dabai DC1, USB 2bc5:0657) açar: renk 640x480 + derinlik 640x400, 30 Hz.
# Nokta bulutu ve IR kapalı (yük/disk). SURE saniye sonra kendiliğinden kapanır.  Kullanım: [SURE=1200] kamera_baslat.sh
set -u
source "$(dirname "$(readlink -f "$0")")/ortam.sh"
SURE=${SURE:-1200}
ssh -o BatchMode=yes "$ROBOT" 'pgrep -f "orbbec_camer[a]" >/dev/null' && { echo "kamera zaten acik"; exit 0; }
robot_cmd "(timeout -s INT $SURE ros2 launch orbbec_camera dabai.launch.py enable_point_cloud:=false enable_ir:=false > /tmp/kamera.log 2>&1 &); sleep 12; grep -m2 -iE 'error|fail' /tmp/kamera.log; ros2 topic list --no-daemon 2>/dev/null | grep -E 'camera/(color|depth)/image_raw'" \
  | sed 's/^/robot: /'
echo "Not: 'Failed to set 2035' hatası zararsız (kayıt etkilenmiyor). Kapatmak: kamera_dur.sh"
