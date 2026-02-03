#!/usr/bin/env python3
"""
Test TidyBot2 Grippers using gripper_wrapper_node.

This script tests the grippers by publishing to /right_gripper/cmd and
/left_gripper/cmd topics. The gripper_wrapper_node (launched with real.launch.py)
translates these to Interbotix SDK commands.

This approach allows the same gripper interface to work for both simulation
and real hardware.

Usage:
    # First, launch the real robot (includes gripper_wrapper_node):
    ros2 launch tidybot_bringup real.launch.py

    # Then run this test:
    ros2 run tidybot_bringup test_grippers_real.py

Topics used:
    /right_gripper/cmd (Float64MultiArray) - 0.0 = open, 1.0 = closed
    /left_gripper/cmd (Float64MultiArray) - 0.0 = open, 1.0 = closed
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class TestGrippers(Node):
    """Test grippers using wrapper node interface."""

    def __init__(self):
        super().__init__('test_grippers_real')

        # Gripper publishers (wrapper node translates to SDK commands)
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray, '/right_gripper/cmd', 10
        )
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray, '/left_gripper/cmd', 10
        )

        # Gripper state tracking
        self.right_connected = False
        self.left_connected = False
        self.right_gripper_pos = None
        self.left_gripper_pos = None

        # Subscribers for joint states (to monitor gripper position)
        self.right_joint_sub = self.create_subscription(
            JointState, '/right_arm/joint_states', self.right_joint_callback, 10
        )
        self.left_joint_sub = self.create_subscription(
            JointState, '/left_arm/joint_states', self.left_joint_callback, 10
        )

        self.get_logger().info('Waiting for joint states...')

    def right_joint_callback(self, msg):
        if not self.right_connected:
            self.right_connected = True
            self.get_logger().info('Connected to right_arm!')
        if 'right_gripper' in msg.name:
            idx = msg.name.index('right_gripper')
            self.right_gripper_pos = msg.position[idx]

    def left_joint_callback(self, msg):
        if not self.left_connected:
            self.left_connected = True
            self.get_logger().info('Connected to left_arm!')
        if 'left_gripper' in msg.name:
            idx = msg.name.index('left_gripper')
            self.left_gripper_pos = msg.position[idx]

    def set_gripper(self, side, position, duration=2.5):
        """
        Set gripper position.

        Args:
            side: 'right', 'left', or 'both'
            position: 0.0 (open) to 1.0 (closed)
            duration: Time to hold the command (seconds)
        """
        msg = Float64MultiArray()
        msg.data = [float(position)]

        # Publish for duration (reduced rate to avoid bus overload)
        start = time.time()
        while (time.time() - start) < duration:
            if side in ('right', 'both') and self.right_connected:
                self.right_gripper_pub.publish(msg)
            if side in ('left', 'both') and self.left_connected:
                self.left_gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.1)  # 10Hz instead of 20Hz

        # Send stop command (0.5 maps to PWM=0 in wrapper)
        stop_msg = Float64MultiArray()
        stop_msg.data = [0.5]
        if side in ('right', 'both') and self.right_connected:
            self.right_gripper_pub.publish(stop_msg)
        if side in ('left', 'both') and self.left_connected:
            self.left_gripper_pub.publish(stop_msg)
        rclpy.spin_once(self, timeout_sec=0.05)

        # Report final positions
        if side in ('right', 'both') and self.right_gripper_pos is not None:
            self.get_logger().info(f'  Right gripper position: {self.right_gripper_pos:.4f} rad')
        if side in ('left', 'both') and self.left_gripper_pos is not None:
            self.get_logger().info(f'  Left gripper position: {self.left_gripper_pos:.4f} rad')

    def run_test(self):
        # Wait for arms to connect
        timeout = 5.0
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.right_connected and self.left_connected:
                break

        if not self.right_connected and not self.left_connected:
            self.get_logger().error('No joint states received!')
            self.get_logger().error('Make sure real.launch.py is running:')
            self.get_logger().error('  ros2 launch tidybot_bringup real.launch.py')
            return False

        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('GRIPPER TEST (via gripper_wrapper_node)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Right arm: {"Connected" if self.right_connected else "Not found"}')
        self.get_logger().info(f'Left arm: {"Connected" if self.left_connected else "Not found"}')
        self.get_logger().info('')
        self.get_logger().info('Using /right_gripper/cmd and /left_gripper/cmd topics')
        self.get_logger().info('Command: 0.0 = open, 1.0 = closed')
        self.get_logger().info('')

        # Step 1: Open both grippers
        self.get_logger().info('[Step 1/3] Opening both grippers...')
        self.set_gripper('both', 0.0, duration=2.5)
        input('\n>>> Grippers should be OPEN. Press Enter to CLOSE...\n')

        # Step 2: Close both grippers
        self.get_logger().info('[Step 2/3] Closing both grippers...')
        self.set_gripper('both', 1.0, duration=2.5)
        input('\n>>> Grippers should be CLOSED. Press Enter to OPEN again...\n')

        # Step 3: Open both grippers
        self.get_logger().info('[Step 3/3] Opening both grippers...')
        self.set_gripper('both', 0.0, duration=2.5)

        self.get_logger().info('')
        self.get_logger().info('Gripper test complete!')
        self.get_logger().info('Grippers should now be OPEN.')

        return True


def main():
    rclpy.init()
    node = TestGrippers()
    try:
        node.run_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
