#!/usr/bin/env python3
"""
Test Pan-Tilt Camera.

Sweeps the camera through pan and tilt positions.

Usage:
    ros2 run tidybot_bringup test_pan_tilt_real.py
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class TestPanTilt(Node):
    def __init__(self):
        super().__init__('test_pan_tilt_real')

        self.state_received = False
        self.current_pan = 0.0
        self.current_tilt = 0.0

        # Publisher
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10
        )

        # Subscriber
        self.state_sub = self.create_subscription(
            Float64MultiArray, '/camera/pan_tilt_state', self.state_callback, 10
        )

        self.get_logger().info('Waiting for pan-tilt state...')

    def state_callback(self, msg):
        if not self.state_received:
            self.state_received = True
            self.get_logger().info('Connected!')

        if len(msg.data) >= 2:
            self.current_pan = msg.data[0]
            self.current_tilt = msg.data[1]

    def send_pan_tilt(self, pan, tilt, duration=1.0):
        """Send pan-tilt command."""
        msg = Float64MultiArray()
        msg.data = [pan, tilt]

        for _ in range(int(duration * 20)):
            self.pan_tilt_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_test(self):
        # Wait for connection (with shorter timeout since pan-tilt might not publish state initially)
        timeout = 3.0
        start = time.time()
        while not self.state_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.state_received:
            self.get_logger().warn('No pan-tilt state received (may still work)...')
            self.get_logger().info('Continuing with test anyway...')

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('PAN-TILT TEST')
        self.get_logger().info('=' * 50)

        # Test positions: (pan, tilt, description)
        positions = [
            (0.0, 0.0, 'Center'),
            (0.5, 0.0, 'Pan left (~30 deg)'),
            (-0.5, 0.0, 'Pan right (~30 deg)'),
            (0.0, 0.0, 'Center'),
            (0.0, 0.3, 'Tilt up (~17 deg)'),
            (0.0, -0.3, 'Tilt down (~17 deg)'),
            (0.0, 0.0, 'Center'),
        ]

        for pan, tilt, description in positions:
            self.get_logger().info(f'{description}: pan={pan:.2f}, tilt={tilt:.2f}')
            self.send_pan_tilt(pan, tilt, duration=1.0)

        self.get_logger().info('')
        self.get_logger().info('Pan-tilt test complete!')

        return True


def main():
    rclpy.init()
    node = TestPanTilt()
    try:
        node.run_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
