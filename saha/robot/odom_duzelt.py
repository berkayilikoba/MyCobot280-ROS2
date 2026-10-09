#!/usr/bin/env python3
"""Robotta (Foxy) çalışır: limo_base'in ham odom'unu IMU yönüyle yeniden hesaplar.

Neden: upstream Foxy limo_driver.cpp:502 her IMU örneğinde |Δyaw| < 0.1° ise Δyaw = 0 yapıyor
(~100 Hz'de 10°/s altındaki dönüşler siliniyor). 24.09 Nav2 kaydında ICP'ye göre ilerlerken 17 m'de
odom yönü −93° birikmiş hata, IMU yönü −7° (bkz. kalibrasyon/odom_analizi.md). Konum hızı (vx) ise doğru (ICP oranı 0.98–1.00).

Girdi:  odom (limo_base, pub_odom_tf:=False ile başlatılır: TF'i bu düğüm yayınlar), imu
Çıktı:  odom_duz (aynı çerçeveler; Nav2 odom_topic) + TF odom->base_link
  konum: vx (sürücü) × IMU yaw ile entegre; yaw: IMU yönelimi (sarmalanmamış, başlangıca göre)
  twist: vx sürücüden, wz IMU jiroskopundan (sürücünün tekerlek wz'si dönüşte ~%9 düşük)
IMU 0.5 s gelmezse ham odom'un yaw artışına geri döner.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def stamp_s(st):
    return st.sec + st.nanosec * 1e-9


class OdomDuzelt(Node):

    def __init__(self):
        super().__init__('odom_duzelt')
        self.imu_yaw = None      # sarmalanmamış IMU yaw
        self.imu_t = None
        self.imu_wz = 0.0
        self.yaw0 = None         # ilk IMU yaw (odom yönü 0'dan başlasın)
        self.ham_yaw = None
        self.son_t = None
        self.x = self.y = self.yaw = 0.0
        self.pub = self.create_publisher(Odometry, 'odom_duz', 50)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Imu, 'imu', self._imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self._odom, qos_profile_sensor_data)
        self.n_yedek = 0
        self.create_timer(10.0, self._rapor)

    def _imu(self, m):
        y = yaw_of(m.orientation)
        self.imu_yaw = y if self.imu_yaw is None else self.imu_yaw + wrap(y - self.imu_yaw)
        self.imu_t = stamp_s(m.header.stamp)
        self.imu_wz = m.angular_velocity.z

    def _odom(self, m):
        t = stamp_s(m.header.stamp)
        ham = yaw_of(m.pose.pose.orientation)
        if self.son_t is None:
            self.son_t, self.ham_yaw = t, ham
            return
        dt = t - self.son_t
        if dt <= 0.0:
            return
        dt = min(dt, 0.5)        # sürücü takılırsa sıçrama yapma
        imu_taze = self.imu_t is not None and abs(t - self.imu_t) < 0.5
        if imu_taze:
            if self.yaw0 is None:
                self.yaw0 = self.imu_yaw - self.yaw
            yeni = self.imu_yaw - self.yaw0
        else:
            yeni = self.yaw + wrap(ham - self.ham_yaw)
            self.yaw0 = None     # IMU dönünce yeniden hizala
            self.n_yedek += 1
        vx = m.twist.twist.linear.x
        ort = 0.5 * (self.yaw + yeni)
        self.x += math.cos(ort) * vx * dt
        self.y += math.sin(ort) * vx * dt
        self.yaw = yeni
        self.son_t, self.ham_yaw = t, ham

        o = Odometry()
        o.header = m.header
        o.child_frame_id = m.child_frame_id
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        o.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        o.pose.covariance = m.pose.covariance
        o.twist.twist.linear.x = vx
        o.twist.twist.angular.z = self.imu_wz if imu_taze else m.twist.twist.angular.z
        o.twist.covariance = m.twist.covariance
        self.pub.publish(o)

        tr = TransformStamped()
        tr.header = m.header
        tr.child_frame_id = m.child_frame_id
        tr.transform.translation.x, tr.transform.translation.y = self.x, self.y
        tr.transform.rotation = o.pose.pose.orientation
        self.tf.sendTransform(tr)

    def _rapor(self):
        self.get_logger().info('x=%.2f y=%.2f yaw=%.1f deg, IMU yedek adim=%d'
                               % (self.x, self.y, math.degrees(self.yaw), self.n_yedek))


def main():
    rclpy.init()
    n = OdomDuzelt()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
