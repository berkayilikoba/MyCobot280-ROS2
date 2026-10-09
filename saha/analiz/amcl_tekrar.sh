#!/usr/bin/env bash
# AMCL'yi bir kayıt üzerinde farklı ayarlarla tekrar çalıştırıp ölçer (robot gerekmez).
# Her /amcl_pose'ta son taramanın haritaya örtüşmesi (duvara <=10 cm) ve konum belirsizliği -> <çıktı>/olcum.jsonl
#
# Kullanım:
#   amcl_tekrar.sh <çıktı_dizini> <nav2_params.yaml> <domain> <bag> <harita.yaml> "<x y yaw_rad>" <ilk_poz_sn> <bitis_sn> \
#                  [hiz=2] [tarama_konusu=/scan_sabit]
#   ilk_poz_sn: kayıt başından itibaren başlangıç pozunun verileceği an; bitis_sn: oynatmanın kesileceği an.
#   Robot üzerinde alınan kayıtlarda /scan_sabit yok: tarama_konusu=/scan verin (AMCL'ye /scan_sabit olarak aktarılır).
# Ayar karşılaştırması için birden çok koşuyu farklı domain'lerle paralel başlatın (ör. 61, 62).
set -u
OUT=$1; P=$2; D=$3; BAG=$4; HAR=$5; read X Y TH <<< "$6"; T0=$7; T1=$8; RATE=${9:-2}; KONU=${10:-/scan_sabit}
A=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
rm -rf "$OUT"; mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_DOMAIN_ID=$D ROS_LOCALHOST_ONLY=1; unset FASTRTPS_DEFAULT_PROFILES_FILE
PIDS=()
trap 'kill -INT ${PIDS[@]} 2>/dev/null; sleep 2; kill -KILL ${PIDS[@]} 2>/dev/null' EXIT
ros2 run nav2_map_server map_server --ros-args --params-file "$P" -p yaml_filename:="$HAR" -p use_sim_time:=true > "$OUT/map.log" 2>&1 & PIDS+=($!)
ros2 run nav2_amcl amcl --ros-args --params-file "$P" -p use_sim_time:=true > "$OUT/amcl.log" 2>&1 & PIDS+=($!)
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lm -p use_sim_time:=true -p autostart:=true \
  -p "node_names:=['map_server','amcl']" > "$OUT/lm.log" 2>&1 & PIDS+=($!)
python3 "$A/tf_suz.py" > "$OUT/tf_suz.log" 2>&1 & PIDS+=($!)          # kayıttaki map->odom'u at (AMCL kendisi yayınlasın)
python3 "$A/amcl_olc.py" "$HAR" "$OUT/olcum.jsonl" > "$OUT/olc.log" 2>&1 & PIDS+=($!)
sleep 8
REMAP=(--remap /tf:=/tf_bag); [ "$KONU" != /scan_sabit ] && REMAP+=("$KONU:=/scan_sabit")
ros2 bag play "$BAG" --clock --rate "$RATE" --topics "$KONU" /tf /tf_static "${REMAP[@]}" > "$OUT/play.log" 2>&1 & PIDS+=($!)
sleep "$(python3 -c "print($T0/$RATE)")"
QZ=$(python3 -c "import math;print(math.sin($TH/2))"); QW=$(python3 -c "import math;print(math.cos($TH/2))")
ros2 topic pub -t 3 -w 0 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: $X, y: $Y}, orientation: {z: $QZ, w: $QW}}, covariance: [0.01,0,0,0,0,0, 0,0.01,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0076]}}" > "$OUT/init.log" 2>&1
sleep "$(python3 -c "print(($T1-$T0)/$RATE)")"
python3 - "$OUT/olcum.jsonl" <<'PY'
import json, sys, numpy as np
R = [json.loads(l) for l in open(sys.argv[1])]
if not R: sys.exit('ölçüm yok (AMCL başlamadı mı? amcl.log)')
ov = np.array([r['ov'] for r in R]); sd = np.array([r['std'] for r in R])
print(f'{len(R)} ölçüm: tarama-harita örtüşmesi ort {ov.mean():.3f} (en kötü %10: {np.percentile(ov, 10):.3f}), '
      f'konum std medyan {np.median(sd) * 100:.1f} cm; son poz ({R[-1]["x"]:.2f}, {R[-1]["y"]:.2f}, {R[-1]["yaw"]:.1f}°)')
PY
