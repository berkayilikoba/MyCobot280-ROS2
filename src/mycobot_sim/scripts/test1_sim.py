#!/usr/bin/env python3
"""MyCobot 280 RViz testi: rastgele konuma git -> bekle -> base konuma dön -> bekle -> tekrar."""
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

MOVE_RANDOM, HOLD_RANDOM, MOVE_HOME, HOLD_HOME = range(4)


class RandomMover(Node):
    def __init__(self):
        super().__init__('test1_sim')

        self.declare_parameter('joint_names', [
            'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
            'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
        ])
        self.declare_parameter('topic', '/joint_states')
        self.declare_parameter('max_angle', 1.0)    # rad, her eklem için +/- sınır
        self.declare_parameter('speed', 0.25)       # rad/s (yavaş)
        self.declare_parameter('hold_time', 2.0)    # s, hedefte bekleme süresi
        self.declare_parameter('rate', 30.0)        # Hz

        self.names = list(self.get_parameter('joint_names').value)
        self.max_angle = float(self.get_parameter('max_angle').value)
        self.speed = float(self.get_parameter('speed').value)
        self.hold_time = float(self.get_parameter('hold_time').value)
        rate = float(self.get_parameter('rate').value)
        topic = self.get_parameter('topic').value

        self.dt = 1.0 / rate
        self.home = [0.0] * len(self.names)
        self.current = list(self.home)
        self.hold_left = 0.0

        self.state = MOVE_RANDOM
        self.target = self.new_random_target()

        self.pub = self.create_publisher(JointState, topic, 10)
        self.timer = self.create_timer(self.dt, self.step)
        self.get_logger().info(f'{topic} üzerine {len(self.names)} eklem yayınlanıyor.')

    def new_random_target(self):
        t = [random.uniform(-self.max_angle, self.max_angle) for _ in self.names]
        self.get_logger().info('Rastgele hedefe gidiliyor: ' + ', '.join(f'{x:+.2f}' for x in t))
        return t

    def move_towards_target(self):
        """Hedefe sabit hızla ilerler, vardıysa True döner."""
        max_step = self.speed * self.dt
        reached = True
        for i, (cur, tgt) in enumerate(zip(self.current, self.target)):
            diff = tgt - cur
            if abs(diff) > 1e-3:
                reached = False
                self.current[i] = cur + max(-max_step, min(max_step, diff))
        return reached

    def step(self):
        if self.state == MOVE_RANDOM:
            if self.move_towards_target():
                self.state, self.hold_left = HOLD_RANDOM, self.hold_time
                self.get_logger().info('Hedefe varıldı, bekleniyor.')

        elif self.state == HOLD_RANDOM:
            self.hold_left -= self.dt
            if self.hold_left <= 0:
                self.target = list(self.home)
                self.state = MOVE_HOME
                self.get_logger().info('Base konuma dönülüyor.')

        elif self.state == MOVE_HOME:
            if self.move_towards_target():
                self.state, self.hold_left = HOLD_HOME, self.hold_time
                self.get_logger().info('Base konumda, bekleniyor.')

        elif self.state == HOLD_HOME:
            self.hold_left -= self.dt
            if self.hold_left <= 0:
                self.target = self.new_random_target()
                self.state = MOVE_RANDOM

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = list(self.current)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RandomMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
