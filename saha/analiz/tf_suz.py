#!/usr/bin/env python3
"""Bag oynatimi icin: /tf_bag'deki donusumlerden 'map' ebeveynlileri at, gerisini /tf'e aktar."""
import rclpy
from tf2_msgs.msg import TFMessage
rclpy.init(); n = rclpy.create_node('tf_suz')
pub = n.create_publisher(TFMessage, '/tf', 100)
def cb(m):
    m.transforms = [t for t in m.transforms if t.header.frame_id != 'map']
    if m.transforms: pub.publish(m)
n.create_subscription(TFMessage, '/tf_bag', cb, 100)
rclpy.spin(n)
