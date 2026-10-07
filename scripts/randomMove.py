#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import random
import time

class RandomMoveAndCube(Node):
    def __init__(self):
        super().__init__('random_move_and_cube')
        
        # Publisherlar
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
        # RViz çalışma alanında görünecek küp marker'ı
        marker = Marker()
        marker.header.frame_id = "joint1"  # Robotun taban referans çerçevesi
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pick_place_cube"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Küpün konumu (Robotun ön çalışma alanı)
        marker.pose.position.x = 0.15
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.05
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Küp boyutları (metre cinsinden, örn: 4cm küp)
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        
        # Renk (Kırmızı ve tamamen opak)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime.sec = 0  # Kalıcı olması için
        self.marker_publisher.publish(marker)

    def run_sequence(self):
        time.sleep(1.0)
        
        # Küpü çalışma alanına spawn et
        self.get_logger().info("Küp çalışma alanına spawn ediliyor...")
        for _ in range(5):  # RViz bazen ilk mesajı kaçırabilir, garanti olması için birkaç kez yollayalım
            self.publish_cube_marker()
            time.sleep(0.2)

        # Rastgele hareket döngüsü
        for _ in range(3):
            arm_angles = [random.uniform(-0.4, 0.4) for _ in range(6)]
            gripper_angle = [random.uniform(-0.5, 0.5)]
            safe_random = arm_angles + gripper_angle
            
            self.get_logger().info(f"Açılar gönderiliyor: {safe_random}")
            self.send_angles(safe_random)
            self.publish_cube_marker() # Küpün görünmeye devam etmesi için
            time.sleep(2.0)
        
        # Home konumu
        self.get_logger().info("Home konumuna dönülüyor...")
        self.send_angles([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.publish_cube_marker()

def main(args=None):
    rclpy.init(args=args)
    node = RandomMoveAndCube()
    node.run_sequence()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()