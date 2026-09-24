#!/usr/bin/env bash
# Uçtan uca devriye koşusu: headless Gazebo + Nav2 (+AMCL ya da SLAM) + limo_autonomy patrol.
# Görev dosyası: MISSION=/yol/gorev.yaml ortam değişkeni (yoksa paket varsayılanı).
# Kullanım: patrol_run.sh <cikti_dizini> <mode> <domain 1-100> <amcl|slam> [nav2_params.yaml] [wall_s=1500]
set -u
OUT=${1:?}; MODE=${2:?}; DOM=${3:?}; LOC=${4:?}; PARAMS=${5:-}; WALL=${6:-1500}
WS=${LIMO_WS:-$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)}   # scripts/ -> limo_ws
mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID=$DOM ROS_LOCALHOST_ONLY=1 RCUTILS_COLORIZED_OUTPUT=0
export GAZEBO_MASTER_URI=http://127.0.0.1:$((11345 + DOM))

ARGS=(mode:=$MODE localization:=$LOC headless:=true use_rviz:=false)
[ -n "$PARAMS" ] && ARGS+=(nav2_params_file:=$PARAMS)
setsid ros2 launch limo_bringup limo_sim_nav2.launch.py "${ARGS[@]}" > "$OUT/launch.log" 2>&1 &
LPID=$!; PGID=$(ps -o pgid= -p $LPID | tr -d ' ')
cleanup() {
  kill -INT -- -"$PGID" 2>/dev/null
  for _ in $(seq 1 20); do kill -0 -- -"$PGID" 2>/dev/null || break; sleep 0.5; done
  kill -KILL -- -"$PGID" 2>/dev/null
}
trap cleanup EXIT INT TERM

[ -n "${MISSION:-}" ] && PL0=(mission_file:=$MISSION) || PL0=()
if [ "$LOC" = amcl ]; then PL=(localizer:=amcl set_initial_pose:=true); else PL=(localizer:=slam_toolbox set_initial_pose:=false); fi
timeout --signal=INT --kill-after=10 "$WALL" ros2 launch limo_autonomy patrol.launch.py use_sim_time:=true "${PL0[@]}" "${PL[@]}" \
  > "$OUT/patrol.log" 2>&1
RC=$?
echo "PATROL_RC=$RC" > "$OUT/status"
grep -E "outcome=|Patrol mission finished" "$OUT/patrol.log"
exit $RC
