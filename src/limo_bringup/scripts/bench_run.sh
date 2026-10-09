#!/usr/bin/env bash
# Tek bir yalıtılmış Nav2 ölçüm koşusu: headless Gazebo + AMCL (bilinen harita) + Nav2
# + nav_bench.py. Paralel koşular birbirini görmesin diye her koşu kendi
# ROS_DOMAIN_ID'si, Gazebo master portu ve süreç grubunda çalışır.
#
# Kullanım:
#   bench_run.sh <cikti_dizini> <mode: ackermann|diff> <domain_id 1-100> [nav2_params.yaml] [timeout_s]
# Çıkış kodu: nav_bench.py'nin kodu (0 = tur tamamlandı; hedeflerin başarısı JSON'da).
set -u

OUT=${1:?cikti dizini}; MODE=${2:?mode}; DOM=${3:?domain id}
PARAMS=${4:-}; TIMEOUT=${5:-120}
WS=${LIMO_WS:-$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)}   # scripts/ -> limo_ws
SCRIPTS=$WS/src/limo_bringup/scripts
mkdir -p "$OUT"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID=$DOM
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:$((11345 + DOM))
export RCUTILS_COLORIZED_OUTPUT=0

ARGS=(mode:=$MODE localization:=amcl headless:=true use_rviz:=false)
[ -n "$PARAMS" ] && ARGS+=(nav2_params_file:=$PARAMS)
[ -n "${BENCH_BT:-}" ] && ARGS+=(nav_to_pose_bt:=$BENCH_BT)
[ -n "${BENCH_URDF:-}" ] && ARGS+=(urdf_model:=$BENCH_URDF)

setsid ros2 launch limo_bringup limo_sim_nav2.launch.py "${ARGS[@]}" > "$OUT/launch.log" 2>&1 &
LPID=$!
PGID=$(ps -o pgid= -p $LPID | tr -d ' ')

BAGPID=
cleanup() {
  [ -n "$BAGPID" ] && { kill -INT $BAGPID 2>/dev/null; sleep 2; }
  kill -INT -- -"$PGID" 2>/dev/null
  for _ in $(seq 1 20); do kill -0 -- -"$PGID" 2>/dev/null || break; sleep 0.5; done
  kill -KILL -- -"$PGID" 2>/dev/null
}
trap cleanup EXIT INT TERM

# Yalnızca LOKALİZASYONU bekle. Navigasyon yığını burada beklenemez: global costmap
# map->base_link dönüşümünü bekliyor, o da AMCL başlangıç pozunu alınca oluşuyor —
# pozu nav_bench.py veriyor ve Nav2'nin aktifleşmesini kendisi bekliyor.
# (İlk sürüm ikisini birden bekliyordu: kilitlenme, 150 s sonra SIGINT.)
ok=0
for _ in $(seq 1 300); do
  if grep -q "lifecycle_manager_localization.*Managed nodes are active" "$OUT/launch.log"; then
    ok=1; break
  fi
  if grep -q "Failed to bring up all requested nodes" "$OUT/launch.log"; then break; fi
  kill -0 $LPID 2>/dev/null || break
  sleep 0.5
done
if [ $ok -ne 1 ]; then
  echo "BRINGUP_FAILED" > "$OUT/status"
  echo "BRINGUP_FAILED (bkz. $OUT/launch.log)" >&2
  exit 4
fi

if [ -n "${BENCH_BAG:-}" ]; then   # teşhis: seçili konuları kaydet (ör. "/scan /tf /tf_static /odom /cmd_vel")
  ros2 bag record -o "$OUT/bag" $BENCH_BAG > "$OUT/bag.log" 2>&1 &
  BAGPID=$!
fi

timeout --signal=INT --kill-after=10 "${BENCH_WALL_LIMIT:-1800}" python3 "$SCRIPTS/nav_bench.py" --mode "$MODE" --out "$OUT/result.json" \
  --timeout "$TIMEOUT" --label "$(basename "$OUT")" > "$OUT/bench.log" 2>&1
RC=$?
echo "BENCH_RC=$RC" > "$OUT/status"
grep '^OZET' "$OUT/bench.log" || tail -5 "$OUT/bench.log"
exit $RC
