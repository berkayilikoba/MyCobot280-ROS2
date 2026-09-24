#!/usr/bin/env python3
"""Üstten bakan sim kamerasından açıklamalı video kaydı (limo_car/worlds/video_world.model ile).

Kamera görüntüsünün üstüne çizilenler: robot ayak izi (/odom, Gazebo gerçek konumu), geçilen iz,
Nav2 global planı (/plan) ve hedef. Sağ panel: mod, sim saati, hız, devriye olay günlüğü (/rosout).
Kamera (0,0,10) m'de aşağı bakıyor, hfov 0.8455 rad, 720x720 -> zeminde 80 px/m;
görüntü sağı = dünya +x, görüntü yukarısı = dünya +y.

Kullanım: video_recorder.py --out video.mp4 --title "..." (SIGINT ile durdurulur, dosya kapanır)
"""

import argparse
import collections
import math
import re

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

IMG = 720
PANEL = 560
PX_PER_M = (IMG / 2) / math.tan(0.8455 / 2) / 10.0
FOOTPRINT = [(-0.161, -0.11), (-0.161, 0.11), (0.161, 0.11), (0.161, -0.11)]


def w2p(x, y):
    return int(round(IMG / 2 + x * PX_PER_M)), int(round(IMG / 2 - y * PX_PER_M))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self, out, title, subtitle):
        super().__init__('video_recorder', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.bridge = CvBridge()
        self.writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), 15, (IMG + PANEL, IMG))
        self.title, self.subtitle = title, subtitle
        self.pose = None
        self.twist = (0.0, 0.0)
        self.cmd = (0.0, 0.0)
        self.trail = collections.deque(maxlen=4000)
        self.plan = []
        self.events = collections.deque(maxlen=12)
        self.frames = 0
        self.sim_t = 0.0
        self.create_subscription(Image, '/overhead/cam/image_raw', self._img, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.create_subscription(Path, '/plan', self._plan, 5)
        self.create_subscription(Twist, '/cmd_vel', self._cmd, 20)
        self.create_subscription(Log, '/rosout', self._log, 50)

    def _odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.twist = (m.twist.twist.linear.x, m.twist.twist.angular.z)
        if not self.trail or math.hypot(self.trail[-1][0] - self.pose[0], self.trail[-1][1] - self.pose[1]) > 0.02:
            self.trail.append(self.pose[:2])

    def _plan(self, m):
        self.plan = [(q.pose.position.x, q.pose.position.y, yaw_of(q.pose.orientation)) for q in m.poses]

    def _cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def _log(self, m):
        if m.name != 'patrol':
            return
        msg = m.msg
        w = re.match(r"Waypoint '(\w+)': outcome=(\w+), elapsed=([\d.]+)s", msg)
        if w:
            res = {'succeeded': 'BASARILI', 'failed': 'BASARISIZ', 'timeout': 'ZAMAN ASIMI',
                   'canceled': 'IPTAL'}.get(w.group(2), w.group(2))
            self.events.append((self.sim_t, f'{w.group(1)}: {res} ({float(w.group(3)):.1f} s)'))
        elif msg.startswith('Navigating to goal: '):
            xy = msg[len('Navigating to goal: '):].rstrip('.').split()
            self.events.append((self.sim_t, f'yeni hedef -> ({xy[0]}, {xy[1]})'))
        elif msg.startswith('Patrol mission finished'):
            st = re.search(r"'succeeded': (\d+), 'failed': (\d+).*'laps': (\d+)", msg)
            txt = (f'GOREV BITTI: {st.group(1)} basarili, {st.group(2)} hata, {st.group(3)} tur'
                   if st else 'GOREV BITTI')
            self.events.append((self.sim_t, txt))

    def _img(self, m):
        self.sim_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        img = self.bridge.imgmsg_to_cv2(m, 'bgr8').copy()
        if img.shape[0] != IMG or img.shape[1] != IMG:
            img = cv2.resize(img, (IMG, IMG))
        # plan
        if len(self.plan) > 1:
            pts = np.array([w2p(x, y) for x, y, _ in self.plan], np.int32)
            cv2.polylines(img, [pts], False, (0, 200, 0), 2, cv2.LINE_AA)
            gx, gy, gyaw = self.plan[-1]
            c = w2p(gx, gy)
            cv2.circle(img, c, 9, (0, 160, 0), 2, cv2.LINE_AA)
            tip = w2p(gx + 0.45 * math.cos(gyaw), gy + 0.45 * math.sin(gyaw))
            cv2.arrowedLine(img, c, tip, (0, 160, 0), 2, cv2.LINE_AA, tipLength=0.35)
        # iz
        if len(self.trail) > 1:
            pts = np.array([w2p(x, y) for x, y in self.trail], np.int32)
            cv2.polylines(img, [pts], False, (255, 120, 0), 2, cv2.LINE_AA)
        # robot ayak izi + yön
        if self.pose:
            x, y, th = self.pose
            c, s = math.cos(th), math.sin(th)
            poly = np.array([w2p(x + c * px - s * py, y + s * px + c * py) for px, py in FOOTPRINT], np.int32)
            cv2.polylines(img, [poly], True, (0, 0, 230), 2, cv2.LINE_AA)
            cv2.arrowedLine(img, w2p(x, y), w2p(x + 0.35 * c, y + 0.35 * s), (0, 0, 230), 2, cv2.LINE_AA,
                            tipLength=0.4)
        frame = np.full((IMG, IMG + PANEL, 3), 245, np.uint8)
        frame[:, :IMG] = img
        self._panel(frame, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        self.writer.write(frame)
        self.frames += 1

    def _panel(self, f, t):
        x0 = IMG + 18

        def put(txt, y, scale=0.55, color=(30, 30, 30), bold=1):
            cv2.putText(f, txt, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, bold, cv2.LINE_AA)
        put(self.title, 38, 0.72, (20, 20, 20), 2)
        put(self.subtitle, 66, 0.5, (90, 90, 90))
        put(f'sim zamani: {t:7.1f} s', 110)
        put(f'hiz (olculen): v={self.twist[0]:+.2f} m/s  w={self.twist[1]:+.2f} rad/s', 138)
        put(f'komut /cmd_vel: v={self.cmd[0]:+.2f} m/s  w={self.cmd[1]:+.2f}', 166)
        if self.pose:
            put(f'konum: x={self.pose[0]:+.2f} y={self.pose[1]:+.2f} yaw={math.degrees(self.pose[2]):+.0f} deg',
                194)
        put('Gosterim:', 236, 0.55, (20, 20, 20), 2)
        cv2.line(f, (x0, 258), (x0 + 30, 258), (0, 0, 230), 3)
        put('    robot ayak izi (0.32 x 0.22 m) ve yonu', 263)
        cv2.line(f, (x0, 284), (x0 + 30, 284), (255, 120, 0), 3)
        put('    gecilen iz', 289)
        cv2.line(f, (x0, 310), (x0 + 30, 310), (0, 200, 0), 3)
        put('    Nav2 global plani + hedef pozu', 315)
        put('Devriye olaylari:', 360, 0.55, (20, 20, 20), 2)
        y = 388
        for te, msg in list(self.events)[-12:]:
            col = (0, 130, 0) if ('BASARILI' in msg or 'BITTI' in msg) else (
                (0, 0, 200) if ('BASARISIZ' in msg or 'ASIMI' in msg) else (60, 60, 60))
            put(f'{te:6.1f} s  {msg}', y, 0.5, col, 2 if 'BITTI' in msg else 1)
            y += 25

    def close(self):
        self.writer.release()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--title', default='LIMO Nav2 otonom devriye')
    ap.add_argument('--subtitle', default='Gazebo 11 simulasyonu, ROS 2 Humble')
    a = ap.parse_args()
    rclpy.init()
    n = Recorder(a.out, a.title, a.subtitle)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.close()
        print(f'{n.frames} kare yazildi -> {a.out}', flush=True)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
