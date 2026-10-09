#!/usr/bin/env python3
"""Tekrar oynatmada AMCL kalitesi: her /amcl_pose'ta son /scan_sabit'in harita örtüşmesi (duvara <=10 cm),
konum belirsizliği; satır satır JSONL yazar. Kullanım: olc.py <harita.yaml> <cikti.jsonl>"""
import json, math, sys
import cv2, numpy as np, yaml, rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
y = yaml.safe_load(open(sys.argv[1])); im = cv2.imread(sys.argv[1].replace('.yaml', '.pgm'), -1)
res = y['resolution']; ox, oy = y['origin'][:2]; H, W = im.shape
dist = cv2.distanceTransform(1 - (im < 50).astype(np.uint8), cv2.DIST_L2, 5) * res
out = open(sys.argv[2], 'w')
rclpy.init(); n = rclpy.create_node('olc', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
st = {}
n.create_subscription(LaserScan, '/scan_sabit', lambda m: st.__setitem__('s', m), qos_profile_sensor_data)
def amcl(m):
    if 's' not in st: return
    p = m.pose.pose; th = 2 * math.atan2(p.orientation.z, p.orientation.w); s = st['s']
    a = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment; r = np.array(s.ranges); k = np.isfinite(r) & (r < 6)
    X = p.position.x + r[k] * np.cos(th + a[k]); Y = p.position.y + r[k] * np.sin(th + a[k])
    ci = ((X - ox) / res).astype(int); ri = H - 1 - ((Y - oy) / res).astype(int); v = (ci >= 0) & (ci < W) & (ri >= 0) & (ri < H)
    ov = float((dist[ri[v], ci[v]] <= 0.10).mean()) if v.any() else 0.0
    out.write(json.dumps({'t': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 'x': p.position.x, 'y': p.position.y,
                          'yaw': math.degrees(th), 'std': math.sqrt(abs(m.pose.covariance[0])), 'ov': ov}) + '\n'); out.flush()
n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl, QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
try: rclpy.spin(n)
except BaseException: pass
