#!/usr/bin/env python3
"""
Comprehensive Real Hardware Test for TidyBot2.

This script tests all major robot subsystems:
1. Mobile base - moves forward 0.1 meters
2. Arms - moves both arms to a forward reaching position
3. Pan-tilt camera - sweeps through pan and tilt range

Can be run from a remote client machine or locally on the robot.

Usage:
    # From remote client (after setting ROS_DOMAIN_ID=42)
    ros2 run tidybot_bringup test_real_hardware.py

    # Or run directly
    python3 test_real_hardware.py

Prerequisites:
    - Robot running: ros2 launch tidybot_bringup real.launch.py
    - Same ROS_DOMAIN_ID on client and robot
    - interbotix_xs_msgs package available
"""

import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

# Import interbotix messages for arm control
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand


class RealHardwareTest(Node):
    """Test node for TidyBot2 real hardware."""

    def __init__(self):
        super().__init__('real_hardware_test')

        self.get_logger().info('=' * 60)
        self.get_logger().info('TidyBot2 Real Hardware Test')
        self.get_logger().info('=' * 60)

        # State tracking
        self.joint_states_received = False
        self.odom_received = False
        self.goal_reached = False
        self.current_joints = {}

        # QoS for reliable delivery
        qos = QoSProfile(depth=10)

        # ========== Publishers ==========

        # Base control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_pose_pub = self.create_publisher(Pose2D, '/base/target_pose', 10)

        # Arm control using interbotix messages
        self.right_arm_pub = self.create_publisher(
            JointGroupCommand, '/right_arm/commands/joint_group', 10
        )
        self.left_arm_pub = self.create_publisher(
            JointGroupCommand, '/left_arm/commands/joint_group', 10
        )

        # Gripper control using interbotix messages
        self.right_gripper_pub = self.create_publisher(
            JointSingleCommand, '/right_arm/commands/joint_single', 10
        )
        self.left_gripper_pub = self.create_publisher(
            JointSingleCommand, '/left_arm/commands/joint_single', 10
        )

        # Pan-tilt control (part of right_arm namespace)
        self.pan_tilt_pub = self.create_publisher(
            JointGroupCommand, '/right_arm/commands/joint_group', 10
        )

        # ========== Subscribers ==========

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.goal_reached_sub = self.create_subscription(
            Bool, '/base/goal_reached', self.goal_reached_callback, 10
        )

        self.get_logger().info('Waiting for robot connection...')

    def joint_state_callback(self, msg: JointState):
        """Track joint states."""
        if not self.joint_states_received:
            self.joint_states_received = True
            self.get_logger().info(f'Connected! Receiving joint states ({len(msg.name)} joints)')

        for i, name in enumerate(msg.name):
            self.current_joints[name] = msg.position[i]

    def goal_reached_callback(self, msg: Bool):
        """Track when base reaches target pose."""
        if msg.data:
            self.goal_reached = True
            self.get_logger().info('Base goal reached!')

    def wait_for_connection(self, timeout=10.0):
        """Wait for joint states to confirm connection."""
        start = time.time()
        while not self.joint_states_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.joint_states_received:
            self.get_logger().error('Timeout waiting for robot connection!')
            self.get_logger().error('Make sure the robot is running:')
            self.get_logger().error('  ros2 launch tidybot_bringup real.launch.py')
            return False

        return True

    def test_base_velocity(self):
        """Test base velocity control - move forward briefly."""
        self.get_logger().info('')
        self.get_logger().info('-' * 40)
        self.get_logger().info('TEST 1: Base Velocity Control')
        self.get_logger().info('-' * 40)
        self.get_logger().info('Moving base forward at 0.1 m/s for 1 second...')

        vel = Twist()
        vel.linear.x = 0.1  # Forward 0.1 m/s

        # Send velocity commands for 1 second
        start = time.time()
        while (time.time() - start) < 1.0:
            self.cmd_vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Stop
        vel.linear.x = 0.0
        self.cmd_vel_pub.publish(vel)
        self.get_logger().info('Base velocity test complete (moved ~0.1m forward)')

    def test_base_position(self):
        """Test base position control - move to target pose."""
        self.get_logger().info('')
        self.get_logger().info('-' * 40)
        self.get_logger().info('TEST 2: Base Position Control')
        self.get_logger().info('-' * 40)
        self.get_logger().info('Sending target pose: forward 0.1m...')

        self.goal_reached = False

        target = Pose2D()
        target.x = 0.1   # 0.1m forward
        target.y = 0.0   # No lateral movement
        target.theta = 0.0  # No rotation

        self.target_pose_pub.publish(target)

        # Wait for goal reached (with timeout)
        start = time.time()
        timeout = 10.0
        while not self.goal_reached and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.goal_reached:
            self.get_logger().info('Base position test complete!')
        else:
            self.get_logger().warn('Base position test timed out (may still be moving)')

    def test_arms(self):
        """Test arm control - move to forward reaching position."""
        self.get_logger().info('')
        self.get_logger().info('-' * 40)
        self.get_logger().info('TEST 3: Arm Control')
        self.get_logger().info('-' * 40)

        # Forward reaching position (safe, visible motion)
        # Joint order for WX250s (6-DOF): [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
        # Home position matches sleep_positions from config (arms folded up)
        home_position = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]
        forward_position = [0.0, -0.3, 0.4, 0.0, 0.0, 0.0]  # Reaching forward

        self.get_logger().info('Moving arms to forward reaching position...')

        # Create interbotix JointGroupCommand messages
        right_msg = JointGroupCommand()
        right_msg.name = 'right_arm'
        right_msg.cmd = forward_position

        left_msg = JointGroupCommand()
        left_msg.name = 'arm'  # Left arm group is named 'arm' in config
        left_msg.cmd = forward_position

        # Publish multiple times to ensure delivery
        for _ in range(20):
            self.right_arm_pub.publish(right_msg)
            self.left_arm_pub.publish(left_msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info('Waiting for arms to reach position...')
        time.sleep(2.0)

        self.get_logger().info('Moving arms back to home position...')
        # Create new messages for home position (don't reuse)
        right_home = JointGroupCommand()
        right_home.name = 'right_arm'
        right_home.cmd = home_position

        left_home = JointGroupCommand()
        left_home.name = 'arm'
        left_home.cmd = home_position

        for _ in range(20):
            self.right_arm_pub.publish(right_home)
            self.left_arm_pub.publish(left_home)
            rclpy.spin_once(self, timeout_sec=0.05)

        time.sleep(2.0)
        self.get_logger().info('Arm test complete!')

    def test_grippers(self):
        """Test gripper control - open and close."""
        self.get_logger().info('')
        self.get_logger().info('-' * 40)
        self.get_logger().info('TEST 4: Gripper Control')
        self.get_logger().info('-' * 40)

        # Gripper uses PWM mode: positive = close, negative = open
        # Typical values: -350 (open) to 350 (close)

        self.get_logger().info('Opening grippers...')
        right_open = JointSingleCommand()
        right_open.name = 'right_gripper'
        right_open.cmd = -350.0  # PWM to open

        left_open = JointSingleCommand()
        left_open.name = 'gripper'  # Left gripper is named 'gripper' in config
        left_open.cmd = -350.0

        for _ in range(10):
            self.right_gripper_pub.publish(right_open)
            self.left_gripper_pub.publish(left_open)
            rclpy.spin_once(self, timeout_sec=0.05)

        time.sleep(1.0)

        self.get_logger().info('Closing grippers...')
        right_close = JointSingleCommand()
        right_close.name = 'right_gripper'
        right_close.cmd = 350.0  # PWM to close

        left_close = JointSingleCommand()
        left_close.name = 'gripper'
        left_close.cmd = 350.0

        for _ in range(10):
            self.right_gripper_pub.publish(right_close)
            self.left_gripper_pub.publish(left_close)
            rclpy.spin_once(self, timeout_sec=0.05)

        time.sleep(1.0)

        self.get_logger().info('Re-opening grippers...')
        for _ in range(10):
            self.right_gripper_pub.publish(right_open)
            self.left_gripper_pub.publish(left_open)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info('Gripper test complete!')

    def test_pan_tilt(self):
        """Test pan-tilt camera control."""
        self.get_logger().info('')
        self.get_logger().info('-' * 40)
        self.get_logger().info('TEST 5: Pan-Tilt Camera Control')
        self.get_logger().info('-' * 40)

        # Pan-tilt positions to test [pan, tilt] in radians
        positions = [
            ([0.0, 0.0], 'Center'),
            ([0.5, 0.0], 'Pan left'),
            ([-0.5, 0.0], 'Pan right'),
            ([0.0, 0.3], 'Tilt up'),
            ([0.0, -0.3], 'Tilt down'),
            ([0.0, 0.0], 'Center'),
        ]

        for cmd, description in positions:
            self.get_logger().info(f'Moving camera: {description} (pan={cmd[0]:.1f}, tilt={cmd[1]:.1f})')

            msg = JointGroupCommand()
            msg.name = 'pan_tilt'
            msg.cmd = cmd

            for _ in range(10):
                self.pan_tilt_pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.05)

            time.sleep(1.0)

        self.get_logger().info('Pan-tilt test complete!')

    def run_all_tests(self):
        """Run all hardware tests."""
        # Wait for connection
        if not self.wait_for_connection():
            return False

        self.get_logger().info('')
        self.get_logger().info('Starting hardware tests...')
        self.get_logger().info('Press Ctrl+C at any time to stop.')
        self.get_logger().info('')

        try:
            # Test 1: Base velocity
            self.test_base_velocity()
            time.sleep(1.0)

            # Test 2: Base position (commented out - can be slow)
            # self.test_base_position()
            # time.sleep(1.0)

            # Test 3: Arms
            self.test_arms()
            time.sleep(1.0)

            # Test 4: Grippers
            self.test_grippers()
            time.sleep(1.0)

            # Test 5: Pan-tilt
            self.test_pan_tilt()

            self.get_logger().info('')
            self.get_logger().info('=' * 60)
            self.get_logger().info('All tests complete!')
            self.get_logger().info('=' * 60)

            return True

        except KeyboardInterrupt:
            self.get_logger().info('Tests interrupted by user')
            # Stop base
            stop_vel = Twist()
            self.cmd_vel_pub.publish(stop_vel)
            return False


def main(args=None):
    rclpy.init(args=args)

    test_node = RealHardwareTest()

    try:
        success = test_node.run_all_tests()
        sys.exit(0 if success else 1)
    except Exception as e:
        test_node.get_logger().error(f'Test failed with error: {e}')
        sys.exit(1)
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
