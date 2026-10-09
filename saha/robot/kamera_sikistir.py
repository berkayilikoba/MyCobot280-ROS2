#!/usr/bin/env python3
"""Robotta çalışır (Foxy). Robotta compressed_image_transport kurulu olmadığı için kamerayı Wi-Fi'ye
sıkıştırılmış verir: renkli -> JPEG (sensor_msgs/CompressedImage, .../image_raw/compressed),
derinlik -> kayıpsız 16-bit PNG (sensor_msgs/CompressedImage, format '16UC1; png', /camera/depth/image_png).
Ham görüntüye yalnızca robot içinde abone olunur; ağdan ham görüntü geçmez.
Kullanım: kamera_sikistir.py [hz=10] [jpeg_kalite=80]"""
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

HZ = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
KALITE = int(sys.argv[2]) if len(sys.argv) > 2 else 80


def main():
    rclpy.init()
    n = rclpy.create_node('kamera_sikistir')
    pub_c = n.create_publisher(CompressedImage, '/camera/color/image_raw/compressed', qos_profile_sensor_data)
    pub_d = n.create_publisher(CompressedImage, '/camera/depth/image_png', qos_profile_sensor_data)
    son = {'c': 0.0, 'd': 0.0}

    def renk(m):
        if time.time() - son['c'] < 1.0 / HZ:
            return
        son['c'] = time.time()
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        if m.encoding == 'rgb8':
            a = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode('.jpg', a, [cv2.IMWRITE_JPEG_QUALITY, KALITE])
        if ok:
            o = CompressedImage(); o.header = m.header; o.format = 'bgr8; jpeg compressed bgr8'
            o.data = buf.tobytes(); pub_c.publish(o)

    def derinlik(m):
        if time.time() - son['d'] < 1.0 / HZ:
            return
        son['d'] = time.time()
        a = np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
        ok, buf = cv2.imencode('.png', a, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        if ok:
            o = CompressedImage(); o.header = m.header; o.format = '16UC1; png'
            o.data = buf.tobytes(); pub_d.publish(o)

    n.create_subscription(Image, '/camera/color/image_raw', renk, qos_profile_sensor_data)
    n.create_subscription(Image, '/camera/depth/image_raw', derinlik, qos_profile_sensor_data)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
