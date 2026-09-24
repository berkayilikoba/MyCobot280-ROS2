#!/usr/bin/env bash
set -euo pipefail

# ROS_DOMAIN_ID varsayılan olarak 0 ayarlanır
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Çalışma alanı dizinleri
UNDERLAY="${UNDERLAY:-/opt/nav2_ws}"
LIMO_WS="${LIMO_WS:-/opt/limo_ws}"

# ROS 2 ortamlarını sırayla kaynakla (setup betikleri için tanımlanmamış değişken kontrolü geçici kapatılır)
set +u
if [ -f "/opt/ros/humble/install/setup.bash" ]; then
    source "/opt/ros/humble/install/setup.bash"
fi

if [ -f "${UNDERLAY}/install/setup.bash" ]; then
    source "${UNDERLAY}/install/setup.bash"
fi

if [ -f "${LIMO_WS}/install/setup.bash" ]; then
    source "${LIMO_WS}/install/setup.bash"
fi
set -u

exec "$@"
