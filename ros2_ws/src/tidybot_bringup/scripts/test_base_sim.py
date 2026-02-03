#!/usr/bin/env python3
"""
TidyBot2 Base Position Control Demo

Sends position targets to the base - the robot drives there automatically.
Matches the real TidyBot2's set_target_position() interface.

Topics used:
- /base/target_pose (Pose2D) - target position [x, y, theta]
- /base/goal_reached (Bool) - notification when goal is reached

Usage:
    # Terminal 1: Start simulation
    ros2 launch tidybot_bringup sim.launch.py

    # Terminal 2: Run this demo
    ros2 run tidybot_bringup test_base_sim.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import time


class TestBase(Node):

    def __init__(self):
        super().__init__('test_base')

        # Publisher for target pose
        self.target_pub = self.create_publisher(Pose2D, '/base/target_pose', 10)

        # Subscriber for goal reached
        self.goal_reached = False
        self.goal_sub = self.create_subscription(
            Bool, '/base/goal_reached', self.goal_callback, 10
        )

        # Control loop at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.command_sent = False

        self.get_logger().info('=' * 50)
        self.get_logger().info('TidyBot2 Forward Motion Test')
        self.get_logger().info('=' * 50)

    def goal_callback(self, msg: Bool):
        if msg.data:
            self.goal_reached = True
            self.get_logger().info('Goal reached!')
            self.get_logger().info('=' * 50)

    def send_target(self, x, y, theta=0.0):
        msg = Pose2D()
        msg.x = x
        msg.y = y
        msg.theta = theta
        self.target_pub.publish(msg)

    def control_loop(self):
        # Send command once: move 0.5m forward
        if not self.command_sent:
            self.get_logger().info('Moving forward 1.0m...')
            self.send_target(1.0, 0.0)
            self.command_sent = True


def main(args=None):
    rclpy.init(args=args)
    node = TestBase()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
