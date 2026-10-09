#!/usr/bin/env bash
# Acil/normal durdurma: sıfır hız, Nav2 + kayıtlar kapanır, robot sürücüleri kapanır.
source "$(dirname "$(readlink -f "$0")")/ortam.sh"
source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=0; unset FASTRTPS_DEFAULT_PROFILES_FILE
pkill -INT -f "hedefe_gi[t].py"
ssh -o BatchMode=yes "$ROBOT" 'pkill -INT -f "limo_base[.]launch"' &   # en hızlı durdurma: taban sürücüsünü kapat
timeout 3 ros2 topic pub -r 20 -t 20 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
pkill -INT -f "ros2 bag recor[d]"; pkill -INT -f "scan_sabitl[e].py"; pkill -INT -f "child-frame-id camera_lin[k]"
kapat "limo_nav2[.]launch" "/opt/ros/humble/lib/nav2_[a-z_]*/|/opt/ros/humble/lib/[a-z_]*/[a-z_]*_server |nav2_amcl/amc[l]|collision_monito[r]"
wait
ssh -o BatchMode=yes "$ROBOT" 'pkill -INT -f "open_ydlidar_launc[h]"; pkill -INT -f "odom_duzel[t].py"'
sleep 3   # denetim ayrı ssh çağrısında: aynı satırda olunca grep pkill komut satırını eşleştiriyordu
ssh -o BatchMode=yes "$ROBOT" 'pgrep -fa "limo_bas[e]|ydlidar_ros[2]|odom_duzel[t]" || echo "robotta surucu kalmadi"'
