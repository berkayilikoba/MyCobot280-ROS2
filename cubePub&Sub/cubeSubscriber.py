#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class CubeSubscriberNode(Node):
    def __init__(self):
        super().__init__('cube_subscriber_node')
        
        # Konum topic'ine abone olma
        self.subscription = self.create_subscription(
            PointStamped,
            '/cube_target_position',
            self.target_callback,
            10
        )
        self.get_logger().info("Cube Subscriber Node başlatıldı, küp koordinatları dinleniyor...")

    def target_callback(self, msg: PointStamped):
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        frame = msg.header.frame_id
        self.get_logger().info(
            f"[HEDEF KÜP ALINDI] Frame: '{frame}' | X: {x:.4f} m, Y: {y:.4f} m, Z: {z:.4f} m"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CubeSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()