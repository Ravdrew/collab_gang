#!/usr/bin/env python3
"""
TidyBot2 Arm Control Demo

Demonstrates how to control the bimanual WX250s arms using ROS2.
The arms move from home position to a forward reaching position.

Topics used:
- /right_arm/cmd (ArmCommand) - right arm joint position commands
- /left_arm/cmd (ArmCommand) - left arm joint position commands

ArmCommand message:
- joint_positions: [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
- duration: interpolation time (0 = immediate)

Usage:
    # Terminal 1: Start simulation
    ros2 launch tidybot_bringup sim.launch.py

    # Terminal 2: Run this demo
    ros2 run tidybot_bringup test_arms_sim.py
"""

import rclpy
from rclpy.node import Node
import time

from tidybot_msgs.msg import ArmCommand
from std_msgs.msg import Float64MultiArray


# Joint positions: [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
HOME_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
FORWARD_POSITION = [0.0, 0.4, 0.5, 0.0, -0.3, 0.0]  # Reaching forward

MOVE_DURATION = 2.0  # seconds


class TestArms(Node):
    """Arm control demo - move from home to forward position."""

    def __init__(self):
        super().__init__('test_arms')

        # Publishers for arm commands
        self.right_arm_pub = self.create_publisher(ArmCommand, '/right_arm/cmd', 10)
        self.left_arm_pub = self.create_publisher(ArmCommand, '/left_arm/cmd', 10)
        self.right_gripper_pub = self.create_publisher(Float64MultiArray, '/right_gripper/cmd', 10)
        self.left_gripper_pub = self.create_publisher(Float64MultiArray, '/left_gripper/cmd', 10)

        # State machine
        self.state = 'OPEN_GRIPPERS'
        self.state_start_time = None
        self.logged_state = None  # Track which state we've logged

        # Wait for connections to establish
        self.get_logger().info('=' * 50)
        self.get_logger().info('TidyBot2 Arm Control Demo')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Waiting for connections...')
        time.sleep(0.5)
        self.get_logger().info('Sequence: Open grippers -> Move arms -> Close grippers')
        self.get_logger().info('')

        # Control loop at 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)

    def send_arm_command(self, pub, positions, duration=0.0):
        """Send arm command with given joint positions."""
        cmd = ArmCommand()
        cmd.joint_positions = positions
        cmd.duration = duration
        pub.publish(cmd)

    def send_gripper_command(self, pub, position):
        """Send gripper command with normalized position (0.0 = open, 1.0 = closed)."""
        msg = Float64MultiArray()
        msg.data = [float(position)]
        pub.publish(msg)

    def control_loop(self):
        now = time.time()

        # Initialize state timing
        if self.state_start_time is None:
            self.state_start_time = now

        elapsed = now - self.state_start_time

        # Log state change once
        if self.state != self.logged_state:
            if self.state == 'OPEN_GRIPPERS':
                self.get_logger().info('Opening grippers...')
            elif self.state == 'MOVE_ARMS':
                self.get_logger().info('Moving arms forward...')
                self.send_arm_command(self.right_arm_pub, FORWARD_POSITION, duration=MOVE_DURATION)
                # self.send_arm_command(self.left_arm_pub, FORWARD_POSITION, duration=MOVE_DURATION)
            elif self.state == 'CLOSE_GRIPPERS':
                self.get_logger().info('Closing grippers...')
            elif self.state == 'DONE':
                self.get_logger().info('')
                self.get_logger().info('=' * 50)
                self.get_logger().info('Done! Arms forward, grippers closed.')
                self.get_logger().info('=' * 50)
            self.logged_state = self.state

        # State: Open grippers
        if self.state == 'OPEN_GRIPPERS':
            self.send_gripper_command(self.right_gripper_pub, 0.0)
            # self.send_gripper_command(self.left_gripper_pub, 0.0)
            if elapsed > 1.0:
                self.state = 'MOVE_ARMS'
                self.state_start_time = now

        # State: Move arms forward
        elif self.state == 'MOVE_ARMS':
            self.send_gripper_command(self.right_gripper_pub, 0.0)
            # self.send_gripper_command(self.left_gripper_pub, 0.0)
            if elapsed > MOVE_DURATION + 0.5:
                self.state = 'CLOSE_GRIPPERS'
                self.state_start_time = now

        # State: Close grippers
        elif self.state == 'CLOSE_GRIPPERS':
            self.send_gripper_command(self.right_gripper_pub, 1.0)
            # self.send_gripper_command(self.left_gripper_pub, 1.0)
            if elapsed > 1.0:
                self.state = 'DONE'
                self.state_start_time = now

        # State: Done
        elif self.state == 'DONE':
            self.send_gripper_command(self.right_gripper_pub, 1.0)
            # self.send_gripper_command(self.left_gripper_pub, 1.0)


def main(args=None):
    rclpy.init(args=args)
    node = TestArms()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
