#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""키보드 입력을 sensor_msgs/Joy로 변환하여 /joy에 발행하는 시험용 노드."""

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy


class KeyboardJoyNode(Node):

    def __init__(self):
        super().__init__('keyboard_joy_node')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('speed_axis', 0.10)
        self.declare_parameter('steer_axis', 0.50)

        joy_topic = str(self.get_parameter('joy_topic').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self._speed_axis = float(self.get_parameter('speed_axis').value)
        self._steer_axis = float(self.get_parameter('steer_axis').value)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self._pub = self.create_publisher(Joy, joy_topic, qos)

        self._linear = 0.0
        self._steer = 0.0
        self._terminal_settings = None

        if not sys.stdin.isatty():
            raise RuntimeError(
                '키보드 입력을 받을 수 있는 터미널에서 실행해야 합니다.'
            )

        self._fd = sys.stdin.fileno()
        self._terminal_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

        self._timer = self.create_timer(
            1.0 / publish_rate,
            self._timer_callback
        )

        self.get_logger().info(
            '키보드 Joy 준비 완료 | '
            'W:전진 S:후진 A:좌회전 D:우회전 '
            'C:조향중립 Space/X:정지 '
            '+/-:속도조절 Q:정지 후 종료'
        )

        self._print_state()

    def _print_state(self):
        self.get_logger().info(
            f'현재 입력: linear={self._linear:+.2f}, '
            f'steer={self._steer:+.2f}, '
            f'speed_axis={self._speed_axis:.2f}'
        )

    def _publish(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()

        # 기존 bldc_drive_node 축 매핑
        # axes[1] = 전진/후진
        # axes[3] = 조향
        msg.axes = [
            0.0,
            float(self._linear),
            0.0,
            float(self._steer),
        ]

        msg.buttons = []
        self._pub.publish(msg)

    def _handle_key(self, key: str):

        if key in ('w', 'W'):
            self._linear = self._speed_axis

        elif key in ('s', 'S'):
            self._linear = -self._speed_axis

        elif key in ('a', 'A'):
            # 현재 bldc_drive_node 실제 계산 기준
            self._steer = self._steer_axis

        elif key in ('d', 'D'):
            self._steer = -self._steer_axis

        elif key in ('c', 'C'):
            self._steer = 0.0

        elif key in (' ', 'x', 'X'):
            self._linear = 0.0
            self._steer = 0.0

        elif key in ('+', '='):
            self._speed_axis = min(
                1.0,
                self._speed_axis + 0.05
            )

            if self._linear != 0.0:
                self._linear = (
                    self._speed_axis
                    if self._linear > 0.0
                    else -self._speed_axis
                )

        elif key in ('-', '_'):
            self._speed_axis = max(
                0.05,
                self._speed_axis - 0.05
            )

            if self._linear != 0.0:
                self._linear = (
                    self._speed_axis
                    if self._linear > 0.0
                    else -self._speed_axis
                )

        elif key in ('q', 'Q'):
            self._linear = 0.0
            self._steer = 0.0

            for _ in range(3):
                self._publish()
                time.sleep(0.05)

            rclpy.shutdown()
            return

        else:
            return

        self._print_state()

    def _timer_callback(self):

        while select.select(
            [sys.stdin],
            [],
            [],
            0.0
        )[0]:

            key = sys.stdin.read(1)
            self._handle_key(key)

            if not rclpy.ok():
                return

        self._publish()

    def destroy_node(self):

        if self._terminal_settings is not None:
            termios.tcsetattr(
                self._fd,
                termios.TCSADRAIN,
                self._terminal_settings
            )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)
    node = KeyboardJoyNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            node._linear = 0.0
            node._steer = 0.0

            for _ in range(3):
                node._publish()
                time.sleep(0.05)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
