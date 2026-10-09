# saha/ betiklerinin ortak ayarları — her betik `source "$(dirname "$0")/ortam.sh"` ile yükler.
# Değiştirmek için ortam değişkeni ver, ör.:  ROBOT=agilex@192.168.0.201 LIMO_VERI=/data ./nav_baslat.sh ...
SAHA=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)   # bu klasör
WS=$(cd "$SAHA/.." && pwd)                                             # limo_ws (colcon çalışma alanı)
CFG=$SAHA/config
ROBOT=${ROBOT:-agilex@192.168.0.200}          # Robot_Lab ağında sabit IP
VERI=${LIMO_VERI:-$HOME/limo_veri}            # oturum çıktıları (harita, bag, log) — depoya girmez
DOMAIN=${LIMO_DOMAIN:-77}                     # robot Foxy sürücüleriyle aynı ROS_DOMAIN_ID
mkdir -p "$VERI"

ros_humble() {   # PC tarafı ROS ortamı (~/.bashrc ROS_LOCALHOST_ONLY=1 koyuyorsa robotu görmez: 0 şart)
  set +u; source /opt/ros/humble/setup.bash; [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"; set -u
  export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=0; unset FASTRTPS_DEFAULT_PROFILES_FILE
}

# Robotta Foxy ortamıyla komut çalıştır: robot_cmd '<komutlar>'
robot_cmd() {
  ssh -o BatchMode=yes -o ConnectTimeout=5 "$ROBOT" "source /opt/ros/foxy/setup.bash; source ~/limo_ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=0; $1"
}

# Arka planda (&) başlatılan launch'lar SIGINT'i yok sayıyor (SigIgn bit 2): önce INT, kapanmazsa TERM,
# en son launch'ın sahipsiz bıraktığı düğümler için desenle TERM.
kapat() {  # kapat <pkill deseni> [düğüm deseni]
  pkill -INT -f "$1"; for _ in 1 2 3 4 5 6; do pgrep -f "$1" >/dev/null || break; sleep 0.5; done
  pkill -TERM -f "$1"; sleep 2
  [ -n "${2:-}" ] && pkill -TERM -f "$2"
  return 0
}
