"""Simülasyon düğümü: /cmd_vel -> TabanModeli -> cmd_vel_taban (diff planar_move bunu dinler).

Yalnızca simde kullanılır; gerçek robotta taban kartı bu tepkiyi zaten kendisi verir.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from limo_kinematics.taban_modeli import KanalParam, TabanModeli, V_VARSAYILAN, W_VARSAYILAN


class TabanModeliDugumu(Node):

    def __init__(self):
        super().__init__('taban_modeli')

        def kanal(ad, v):
            return KanalParam(*(self.declare_parameter(f'{ad}_{k}', getattr(v, k)).value
                                for k in ('gecikme', 'tau', 'ivme', 'kazanc', 'watchdog')))
        self.model = TabanModeli(kanal('v', V_VARSAYILAN), kanal('w', W_VARSAYILAN))
        self.pub = self.create_publisher(Twist, self.declare_parameter('output_topic', 'cmd_vel_taban').value, 10)
        self.create_subscription(Twist, self.declare_parameter('input_topic', 'cmd_vel').value, self._komut, 10)
        self.create_timer(1.0 / self.declare_parameter('hz', 100.0).value, self._adim)
        self.get_logger().info(f'taban modeli: v={self.model.v.p} w={self.model.w.p}')

    def _simdi(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _komut(self, m):
        self.model.komut(self._simdi(), m.linear.x, m.angular.z)

    def _adim(self):
        v, w = self.model.adim(self._simdi())
        out = Twist()
        out.linear.x, out.angular.z = v, w
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    n = TabanModeliDugumu()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
