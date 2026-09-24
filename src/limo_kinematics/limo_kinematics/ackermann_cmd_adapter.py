"""Simülasyon için cmd_vel anlam adaptörü (yalnızca Gazebo Ackermann modeli).

Sorun (ölçüldü, Faz 3): gazebo_ros_ackermann_drive eklentisi Twist.angular.z'yi
DİREKSİYON AÇISI [rad] olarak uygular. Boş dünyada açık çevrim ölçümü: w=0.2 ile
v=0.15/0.30/0.45 -> yarıçap 1.004/1.002/0.997 m (v'den bağımsız, L/tan(0.2)).
Gerçek LIMO sürücüsü ve Nav2 ise angular.z'yi YAW HIZI [rad/s] kabul eder.
Aynı /cmd_vel simülasyonda başka bir eğriliğe dönüşüyordu; MPPI'nin takip
başarısızlıklarının kök nedeni buydu.

Bu düğüm /cmd_vel'i (yaw hızı anlamında) alır, gerçek sürücünün yaptığı dönüşümü
aynen uygular (limo_base twistToInnerSteering -> iç açı, kırpma) ve eklentinin
beklediği merkez direksiyon açısını üretir. Sonuç: /cmd_vel simülasyonda ve
robotta aynı anlama gelir.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from limo_kinematics.kinematics import inner_to_central, twist_to_inner_steering


class AckermannCmdAdapter(Node):

    def __init__(self):
        super().__init__('ackermann_cmd_adapter')
        # Varsayılanlar gerçek sürücü sabitleri (limo_driver.h:130-132)
        self.L = self.declare_parameter('wheelbase', 0.2).value
        self.T = self.declare_parameter('track', 0.172).value
        self.max_inner = self.declare_parameter('max_inner_steer', 0.48869).value
        in_topic = self.declare_parameter('input_topic', 'cmd_vel').value
        out_topic = self.declare_parameter('output_topic', 'cmd_vel_steer').value
        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Twist, in_topic, self._cb, 10)
        self.get_logger().info(
            f'{in_topic} (yaw hızı) -> {out_topic} (merkez direksiyon açısı), '
            f'L={self.L} T={self.T} max_inner={self.max_inner}')

    def _cb(self, msg):
        out = Twist()
        inner = twist_to_inner_steering(msg.linear.x, msg.angular.z, self.L, self.T, self.max_inner)
        if inner is None:
            self.get_logger().warn('sonlu olmayan cmd_vel -> dur', throttle_duration_sec=1.0)
            self.pub.publish(out)
            return
        out.linear.x = msg.linear.x
        out.angular.z = inner_to_central(inner, self.L, self.T)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannCmdAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
