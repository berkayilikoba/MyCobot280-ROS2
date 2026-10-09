#!/usr/bin/env python3
"""/scan -> /scan_sabit: sabit açı ızgarasına yeniden örnekler (N nokta, -pi..pi).
Neden: slam_toolbox (Karto) her taramada aynı nokta sayısını ister; YDLIDAR T-mini 654/656/658 nokta ve
değişken açı artışı veriyor -> sayısı ilk taramadan farklı olanlar atılıyor, harita donuyordu (24.09 ölçümü).
Her hücreye o hücreye düşen en yakın geçerli ölçüm yazılır; boş hücre = inf."""
import math
import sys

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

N = int(sys.argv[1]) if len(sys.argv) > 1 else 656
# Bu mesafenin altındaki ölçümler atılır: lidar robotun kendi gövdesini/kolunu 0.04-0.19 m'de görüyor
# (24.09 ölçümü). Harita için 0 bırakılabilir (SLAM min_laser_range ile zaten süzüyor); Nav2 için 0.22.
MIN_R = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0


def main():
    rclpy.init()
    n = rclpy.create_node('scan_sabitle')
    pub = n.create_publisher(LaserScan, '/scan_sabit', qos_profile_sensor_data)
    inc = 2 * math.pi / N

    def cb(m):
        out = LaserScan()
        out.header = m.header
        out.angle_min, out.angle_increment = -math.pi, inc
        out.angle_max = -math.pi + inc * (N - 1)
        out.range_min, out.range_max = max(m.range_min, MIN_R), m.range_max
        out.scan_time = m.scan_time
        out.time_increment = m.scan_time / N if m.scan_time > 0 else 0.0
        r = [math.inf] * N
        a = m.angle_min
        for d in m.ranges:
            if math.isfinite(d) and max(m.range_min, MIN_R) <= d <= m.range_max:
                k = int(round((math.atan2(math.sin(a), math.cos(a)) + math.pi) / inc)) % N
                if d < r[k]:
                    r[k] = d
            a += m.angle_increment
        out.ranges = r
        pub.publish(out)

    n.create_subscription(LaserScan, '/scan', cb, qos_profile_sensor_data)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
