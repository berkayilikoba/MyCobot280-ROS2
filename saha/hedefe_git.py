#!/usr/bin/env python3
"""AMCL başlangıç pozu verir ve Nav2'ye tek hedef gönderir. Zaman aşımı/kesintide hedefi iptal eder.
Kullanım: hedefe_git.py <x0 y0 yaw0> <x y yaw> [zaman_asimi_s=120]"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.signals import SignalHandlerOptions


def poz(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    x0, y0, t0, x, y, t = map(float, sys.argv[1:7])
    zaman = float(sys.argv[7]) if len(sys.argv) > 7 else 120.0
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # kesintide bağlam açık kalsın, iptal gidebilsin
    nav = BasicNavigator()
    bitti = False
    try:
        ilk = poz(nav, x0, y0, t0)
        ilk.header.stamp.sec = ilk.header.stamp.nanosec = 0  # 0 = en güncel TF; Wi-Fi gecikmesinde "gelecekten" reddini önler
        nav.setInitialPose(ilk)
        nav.waitUntilNav2Active(localizer='amcl')
        print(f'Nav2 hazir; hedef ({x:.2f}, {y:.2f}, {math.degrees(t):.0f} deg)', flush=True)
        nav.goToPose(poz(nav, x, y, t))
        bas = time.time()
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb and int(time.time() - bas) % 3 == 0:
                print(f'  kalan {fb.distance_remaining:.2f} m, {time.time() - bas:.0f} s', flush=True)
                time.sleep(1.0)
            if time.time() - bas > zaman:
                print('ZAMAN ASIMI -> iptal', flush=True)
                nav.cancelTask()
                break
        r = nav.getResult()
        print('SONUC:', {TaskResult.SUCCEEDED: 'BASARILI', TaskResult.CANCELED: 'IPTAL',
                         TaskResult.FAILED: 'BASARISIZ'}.get(r, str(r)), f'({time.time() - bas:.0f} s)', flush=True)
        bitti = True
    except KeyboardInterrupt:
        print('kesildi', flush=True)
    finally:
        if not bitti:
            try:
                nav.cancelTask()
                print('hedef iptal edildi', flush=True)
            except BaseException as e:
                print('iptal basarisiz:', e, flush=True)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
