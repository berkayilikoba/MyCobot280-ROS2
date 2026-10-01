#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

class CubePublisherNode(Node):
    def __init__(self):
        super().__init__('cube_publisher_node')

        # Küp pozisyonunu yayınlayan publisher
        self.target_pub = self.create_publisher(PointStamped, '/cube_target_position', 10)
        
        # RViz üzerinde görselleştirme için Marker publisher
        self.marker_pub = self.create_publisher(Marker, '/cube_marker', 10)

        # 1 Hz frekansında periyodik yayın
        self.timer = self.create_timer(1.0, self.publish_cube)

        # myCobot 280 çalışma alanı ve gripper için güvenli koordinatlar (metre cinsinden)
        self.cube_x = 0.18   # Robotun 18 cm önü
        self.cube_y = 0.00   # Merkez hizası
        self.cube_z = 0.03   # Güvenli kavrama yüksekliği
        self.cube_size = 0.025  # 2.5 cm küp boyutu

        self.get_logger().info("Cube Publisher Node başlatıldı.")
        self.get_logger().info(f"Hedef Küp Konumu: X={self.cube_x:.3f}, Y={self.cube_y:.3f}, Z={self.cube_z:.3f}")

    def publish_cube(self):
        now = self.get_clock().now().to_msg()

        # 1. PointStamped Mesajı (Subscriber node için)
        point_msg = PointStamped()
        point_msg.header.stamp = now
        point_msg.header.frame_id = 'joint1'  # URDF'e göre 'joint1' veya 'base_link'
        point_msg.point.x = self.cube_x
        point_msg.point.y = self.cube_y
        point_msg.point.z = self.cube_z
        self.target_pub.publish(point_msg)

        # 2. RViz Marker Mesajı (3D görselleştirme için)
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = 'joint1'
        marker.ns = 'manipulation_targets'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Pozisyon
        marker.pose.position.x = self.cube_x
        marker.pose.position.y = self.cube_y
        marker.pose.position.z = self.cube_z
        marker.pose.orientation.w = 1.0

        # Boyutlar
        marker.scale.x = self.cube_size
        marker.scale.y = self.cube_size
        marker.scale.z = self.cube_size

        # Renk (Turuncu, saydamlık yok)
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = CubePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()