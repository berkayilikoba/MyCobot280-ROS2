#!/usr/bin/env bash
# Devriyenin video + rosbag kaydı: kameralı dünya (bench/video/video_world.model) + Nav2 + patrol.
# Kullanım: [MISSION=gorev.yaml] video_run.sh <cikti_dizini> <mode> <domain> <amcl|slam> <baslik> [wall_s=900]
set -u
OUT=${1:?}; MODE=${2:?}; DOM=${3:?}; LOC=${4:?}; TITLE=${5:?}; WALL=${6:-900}
WS=${LIMO_WS:-$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)}   # scripts/ -> limo_ws
WORLD=$WS/src/limo_car/worlds/video_world.model
mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID=$DOM ROS_LOCALHOST_ONLY=1 RCUTILS_COLORIZED_OUTPUT=0
export GAZEBO_MASTER_URI=http://127.0.0.1:$((11345 + DOM))

setsid ros2 launch limo_bringup limo_sim_nav2.launch.py mode:=$MODE localization:=$LOC headless:=true \
  use_rviz:=false world:=$WORLD > "$OUT/launch.log" 2>&1 &
LPID=$!; LPG=$(ps -o pgid= -p $LPID | tr -d ' ')
REC_PIDS=()
cleanup() {
  for p in "${REC_PIDS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done
  sleep 6
  kill -INT -- -"$LPG" 2>/dev/null
  for _ in $(seq 1 20); do kill -0 -- -"$LPG" 2>/dev/null || break; sleep 0.5; done
  kill -KILL -- -"$LPG" 2>/dev/null
}
trap cleanup EXIT INT TERM

timeout 120 bash -c "until grep -q 'lifecycle_manager_localization.*Managed nodes are active\|lifecycle_manager_navigation.*Managed nodes are active' '$OUT/launch.log'; do sleep 1; done" \
  || { echo BRINGUP_FAILED > "$OUT/status"; exit 4; }

setsid ros2 bag record -o "$OUT/rosbag" /odom /scan /cmd_vel /cmd_vel_nav /plan /tf /tf_static /amcl_pose \
  /map /rosout /local_costmap/published_footprint /robot_description > "$OUT/bag.log" 2>&1 &
REC_PIDS+=($!)
setsid python3 "$WS/src/limo_bringup/scripts/video_recorder.py" --out "$OUT/raw.mp4" --title "$TITLE" \
  > "$OUT/recorder.log" 2>&1 &
REC_PIDS+=($!)
sleep 3

[ -n "${MISSION:-}" ] && PL0=(mission_file:=$MISSION) || PL0=()
if [ "$LOC" = amcl ]; then PL=(localizer:=amcl set_initial_pose:=true); else PL=(localizer:=slam_toolbox set_initial_pose:=false); fi
timeout --signal=INT --kill-after=10 "$WALL" ros2 launch limo_autonomy patrol.launch.py use_sim_time:=true \
  "${PL0[@]}" "${PL[@]}" > "$OUT/patrol.log" 2>&1
echo "PATROL_RC=$?" > "$OUT/status"
sleep 3   # son kareler
grep -E "outcome=|mission finished" "$OUT/patrol.log"
