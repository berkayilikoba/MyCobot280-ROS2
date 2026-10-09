#!/usr/bin/env bash
# Boş dünyada Ackermann direksiyon basamak testi (yalıtılmış, süreç grubu temizlenir).
# Kullanım: steer_step_run.sh <cikti_dizini> <domain> [urdf_model] [steer_step.py argümanları...]
set -u
OUT=${1:?}; DOM=${2:?}; URDF=${3:-}; shift 3 2>/dev/null || shift $#
WS=${LIMO_WS:-$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)}   # scripts/ -> limo_ws
mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID=$DOM ROS_LOCALHOST_ONLY=1 RCUTILS_COLORIZED_OUTPUT=0
export GAZEBO_MASTER_URI=http://127.0.0.1:$((11345 + DOM))
ARGS=(mode:=${MODE:-ackermann} headless:=true use_rviz:=false gui:=false world:=${WORLD:-/opt/ros/humble/share/gazebo_ros/worlds/empty.world})
[ -n "$URDF" ] && ARGS+=(urdf_model:=$URDF)
setsid ros2 launch limo_description gazebo_limo_mycobot.launch.py "${ARGS[@]}" > "$OUT/launch.log" 2>&1 &
LPID=$!; PGID=$(ps -o pgid= -p $LPID | tr -d ' ')
trap 'kill -INT -- -$PGID 2>/dev/null; sleep 5; kill -KILL -- -$PGID 2>/dev/null' EXIT
timeout 60 bash -c 'until ros2 topic list 2>/dev/null | grep -q "^/odom$"; do sleep 1; done'
sleep 5
if [ -n "${TEST:-}" ]; then   # ör. TEST=diff_openloop.py
  timeout 120 python3 "$WS/src/limo_bringup/scripts/$TEST" "$OUT/result.json" "$@" 2>&1 | tee "$OUT/test.log"
else
  timeout 120 python3 "$WS/src/limo_bringup/scripts/steer_step.py" --out "$OUT/result.json" "$@" 2>&1 | tee "$OUT/test.log"
fi
