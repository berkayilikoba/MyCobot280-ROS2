#!/usr/bin/env bash
# Robotun KENDİ diskine kayıt başlatır (Wi-Fi kaybı/gecikmesi olmaz; 24.09'da PC'ye kayıtta odom %30 kayıp, 1 s gecikme vardı).
# İki ayrı kayıt süreci: tek süreçte ham görüntüler odom/IMU/lidar'ı da boğuyordu (09.10: lidar 0 mesaj, odom 35 Hz).
#   <dizin>/sensor : odom, odom_duz, imu, scan, tf, tf_static, cmd_vel, amcl_pose, limo_status, kamera camera_info + extrinsic
#   <dizin>/kamera : renk + derinlik ham görüntü (yalnız kamera açıksa; ~27 MB/s, ~1.6 GB/dk)
# Kullanım: [SURE=600] kayit_baslat.sh [ad=kayit] [--kamerasiz]      Bitirmek: kayit_bitir.sh
set -u
source "$(dirname "$(readlink -f "$0")")/ortam.sh"
AD=${1:-kayit}; KAMERA=1; [ "${2:-}" = "--kamerasiz" ] && KAMERA=0
SURE=${SURE:-600}
DIZIN="~/limo_kayit/${AD}_$(date +%Y%m%d_%H%M%S)"
ssh -o BatchMode=yes "$ROBOT" 'pgrep -f "ros2 bag recor[d]" >/dev/null' && { echo "UYARI: robotta kayit zaten acik"; exit 3; }
BOS=$(ssh -o BatchMode=yes "$ROBOT" "df -BG --output=avail / | tail -1 | tr -dc 0-9")
echo "robot diskinde bos: ${BOS} GB"; [ "$BOS" -lt 5 ] && { echo "HATA: 5 GB'tan az yer var"; exit 4; }
SENSOR="/odom /odom_duz /imu /scan /tf /tf_static /cmd_vel /cmd_vel_smoothed /amcl_pose /limo_status /camera/color/camera_info /camera/depth/camera_info /camera/depth_to_color"
robot_cmd "mkdir -p $DIZIN
  (timeout -s INT $SURE ros2 bag record -o $DIZIN/sensor $SENSOR > $DIZIN/sensor.log 2>&1 &)
  if [ $KAMERA = 1 ]; then (timeout -s INT $SURE ros2 bag record -o $DIZIN/kamera /camera/color/image_raw /camera/depth/image_raw > $DIZIN/kamera.log 2>&1 &); fi
  sleep 5; grep -h -m1 'All requested topics' $DIZIN/*.log | cut -c1-90"
echo "$DIZIN" > "$VERI/aktif_kayit"
echo "kayit basladi: robot:$DIZIN (en fazla $SURE s)"
