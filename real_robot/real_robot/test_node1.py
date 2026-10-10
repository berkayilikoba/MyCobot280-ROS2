"""
Bu Python betiği, ROS 2 ve pymycobot kütüphanesini kullanarak gerçek bir 
MyCobot 280 robot koluyla iletişim kurmak ve temel hareket testleri gerçekleştirmek 
için tasarlanmıştır.

Yaptığı işlemler sırasıyla:
1. Belirtilen seri port (/dev/ttyUSB0) üzerinden MyCobot robotuyla seri haberleşme başlatır.
2. Robotu başlangıç (home) pozisyonuna ([0, 0, 0, 0, 0, 0]) güvenli bir hızda gönderir.
3. Donanımın ve bağlantının düzgün çalıştığını doğrulamak için bir test açısına hareket ettirir.
4. Testler tamamlandığında ROS 2 düğümünü (node) düzgün bir şekilde kapatır.
"""


#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from pymycobot.mycobot import MyCobot

class RealRobotTestNode(Node):
    def __init__(self):
        super().__init__('real_robot_test_node')
        self.get_logger().info("Gerçek MyCobot 280 bağlantısı başlatılıyor...")
        
        # Port adını kendi sistemine göre düzenle (örn: /dev/ttyUSB0 veya /dev/ttyACM0)
        port = "/dev/ttyUSB0"
        baud = 115200
        
        try:
            self.mc = MyCobot(port, baud)
            time.sleep(0.5)
            self.get_logger().info("MyCobot başarıyla bağlandı!")
        except Exception as e:
            self.get_logger().error(f"Bağlantı hatası: {e}")
            self.mc = None

    def send_test_pose(self):
        if not self.mc:
            return
        
        # Robotu başlangıç (home) pozisyonuna getir [0, 0, 0, 0, 0, 0]
        self.get_logger().info("Robot Home pozisyonuna gönderiliyor...")
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 30) # 30 hız değeri
        time.sleep(3.0)

        # Hafif bir test hareketi
        self.get_logger().info("Test açısına gidiliyor...")
        self.mc.send_angles([0, 30, -30, 0, 0, 0], 30)
        time.sleep(3.0)
        
        self.get_logger().info("Test hareketi tamamlandı.")

def main(args=None):
    rclpy.init(args=args)
    node = RealRobotTestNode()
    node.send_test_pose()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()