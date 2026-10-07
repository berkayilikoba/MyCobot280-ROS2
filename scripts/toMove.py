#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import time

class MoveToCubeSlider(Node):
    def __init__(self):
        super().__init__('move_to_cube_slider_node')
        
        # Slider kontrolcüsünün dinlediği joint_states topic'ine yayın yapıyoruz
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        
        self.joints = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6',
            'gripper_controller',
        ]

    def send_angles(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joints
        msg.position = positions
        self.joint_publisher.publish(msg)

    def publish_cube_marker(self):
        marker = Marker()
        marker.header.frame_id = "joint1"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pick_place_cube"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Küpün konumu (Çalışma alanındaki hedef koordinat)
        marker.pose.position.x = 0.15
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime.sec = 0
        self.marker_publisher.publish(marker)

    def execute(self):
        time.sleep(1.0)
        
        # Küpü RViz'de görünür kılmak için birkaç kez marker yayınla
        for _ in range(5):
            self.publish_cube_marker()
            time.sleep(0.1)

        self.get_logger().info("Küpün üzerine doğru yaklaşılıyor...")
        
        # MyCobot 280'in küpe uzanmasını sağlayan hedef eklem açıları (radyan)
        target_angles = [0.0, 1.1, -1.3, 0.5, 0.0, -1.5, 0.0]
        
        for _ in range(10):
            self.send_angles(target_angles)
            self.publish_cube_marker()
            time.sleep(0.2)
            
        self.get_logger().info("Hedef konuma ulaşıldı ve duruldu.")

def main(args=None):
    rclpy.init(args=args)
    node = MoveToCubeSlider()
    node.execute()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()