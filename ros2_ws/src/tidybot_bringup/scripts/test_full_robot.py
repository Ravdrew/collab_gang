#!/usr/bin/env python3
"""
TidyBot2 Full Robot Test Script.

Sequentially tests all hardware components:
1. Phoenix 6 mobile base (forward/backward, rotation)
2. Left arm (home, forward, waist rotation, sleep)
3. Right arm (home, forward, waist rotation, sleep)
4. Grippers (via gripper_wrapper_node)
5. Pan-tilt camera system (pan left/right, tilt up/down)

Usage:
    # First, launch the full robot (includes gripper_wrapper_node):
    ros2 launch tidybot_bringup real.launch.py

    # Then run this test:
    ros2 run tidybot_bringup test_full_robot.py

Gripper control uses /right_gripper/cmd and /left_gripper/cmd topics
(Float64MultiArray, 0.0 = open, 1.0 = closed). The gripper_wrapper_node
translates these to Interbotix SDK commands.
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
from interbotix_xs_msgs.srv import TorqueEnable
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# Arm poses (6 joints: waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate)
HOME_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]
FORWARD_POSE = [0.0, -0.5, 0.5, 0.0, 0.0, 0.0]

# Pan-tilt poses [pan, tilt]
PAN_TILT_CENTER = [0.0, 0.0]
PAN_TILT_LEFT = [0.5, 0.0]
PAN_TILT_RIGHT = [-0.5, 0.0]
PAN_TILT_UP = [0.0, -0.3]


class FullRobotTester(Node):
    def __init__(self):
        super().__init__('full_robot_tester')

        # ===== BASE =====
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # ===== RIGHT ARM (namespace: /right_arm) =====
        self.right_group_pub = self.create_publisher(
            JointGroupCommand, '/right_arm/commands/joint_group', 10
        )
        self.right_single_pub = self.create_publisher(
            JointSingleCommand, '/right_arm/commands/joint_single', 10
        )
        self.right_torque_client = self.create_client(
            TorqueEnable, '/right_arm/torque_enable'
        )
        self.right_joint_states = None
        self.right_js_sub = self.create_subscription(
            JointState, '/right_arm/joint_states', self._right_js_cb, 10
        )

        # ===== LEFT ARM (namespace: /left_arm) =====
        self.left_group_pub = self.create_publisher(
            JointGroupCommand, '/left_arm/commands/joint_group', 10
        )
        self.left_single_pub = self.create_publisher(
            JointSingleCommand, '/left_arm/commands/joint_single', 10
        )
        self.left_torque_client = self.create_client(
            TorqueEnable, '/left_arm/torque_enable'
        )
        self.left_joint_states = None
        self.left_js_sub = self.create_subscription(
            JointState, '/left_arm/joint_states', self._left_js_cb, 10
        )

        # ===== PAN-TILT (separate topic after xs_sdk modification) =====
        self.pan_tilt_joint_states = None
        self.pan_tilt_js_sub = self.create_subscription(
            JointState, '/pan_tilt/joint_states', self._pan_tilt_js_cb, 10
        )

        # ===== GRIPPERS (via gripper_wrapper_node) =====
        # Publishes to /right_gripper/cmd and /left_gripper/cmd
        # The gripper_wrapper_node translates these to Interbotix SDK commands
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray, '/right_gripper/cmd', 10
        )
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray, '/left_gripper/cmd', 10
        )

        self.get_logger().info('Full Robot Tester initialized')

    def _odom_cb(self, msg):
        self.odom = msg

    def _right_js_cb(self, msg):
        self.right_joint_states = msg

    def _left_js_cb(self, msg):
        self.left_joint_states = msg

    def _pan_tilt_js_cb(self, msg):
        self.pan_tilt_joint_states = msg

    def spin_for(self, duration):
        """Spin for a duration while processing callbacks."""
        start = time.time()
        while (time.time() - start) < duration:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_components(self, timeout=10.0):
        """Wait for hardware components to be available."""
        start = time.time()
        has_base = False
        has_right_arm = False
        has_left_arm = False
        has_pan_tilt = False

        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.odom is not None:
                has_base = True
            if self.right_joint_states is not None:
                has_right_arm = True
            if self.left_joint_states is not None:
                has_left_arm = True
            if self.pan_tilt_joint_states is not None:
                has_pan_tilt = True

            # Return early if we have all components
            if has_base and has_right_arm and has_left_arm and has_pan_tilt:
                break

        return has_base, has_right_arm, has_left_arm, has_pan_tilt

    # ===== BASE CONTROL =====
    def move_base(self, linear_x, angular_z, duration):
        """Move the base with given velocities for a duration."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.get_logger().info(f'Base: linear={linear_x:.2f} m/s, angular={angular_z:.2f} rad/s')

        start = time.time()
        while (time.time() - start) < duration:
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Stop
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.spin_for(0.2)

    def test_base(self):
        """Test the mobile base."""
        print()
        print("=" * 50)
        print("TESTING: Phoenix 6 Mobile Base")
        print("=" * 50)

        print("[1/4] Moving forward...")
        self.move_base(0.1, 0.0, 1.5)

        print("[2/4] Moving backward...")
        self.move_base(-0.1, 0.0, 1.5)

        print("[3/4] Rotating left...")
        self.move_base(0.0, 0.3, 1.5)

        print("[4/4] Rotating right...")
        self.move_base(0.0, -0.3, 1.5)

        print("Base test complete!")

    # ===== ARM CONTROL =====
    def set_torque(self, namespace, group_name, enable):
        """Enable or disable torque on a group."""
        req = TorqueEnable.Request()
        req.cmd_type = 'group'
        req.name = group_name
        req.enable = enable

        if namespace == 'right_arm':
            client = self.right_torque_client
        else:
            client = self.left_torque_client

        if not client.service_is_ready():
            self.get_logger().warn(f'{namespace} torque service not ready')
            return None

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()

    def move_arm_group(self, namespace, group_name, positions, move_time=2.0):
        """Move an arm group to specified positions."""
        msg = JointGroupCommand()
        msg.name = group_name
        msg.cmd = positions

        self.get_logger().info(f'{namespace}/{group_name} -> {[f"{p:.2f}" for p in positions]}')

        if namespace == 'right_arm':
            self.right_group_pub.publish(msg)
        else:
            self.left_group_pub.publish(msg)

        self.spin_for(move_time)

    def move_arm_single(self, namespace, joint_name, position, move_time=1.5):
        """Move a single joint to specified position."""
        msg = JointSingleCommand()
        msg.name = joint_name
        msg.cmd = position

        self.get_logger().info(f'{namespace}/{joint_name} -> {position:.2f}')

        if namespace == 'right_arm':
            self.right_single_pub.publish(msg)
        else:
            self.left_single_pub.publish(msg)

        self.spin_for(move_time)

    def test_arm(self, side):
        """Test one arm."""
        if side == 'right':
            namespace = 'right_arm'
            group_name = 'right_arm'
            waist_joint = 'right_waist'
        else:
            namespace = 'left_arm'
            group_name = 'left_arm'
            waist_joint = 'left_waist'

        print()
        print("=" * 50)
        print(f"TESTING: {side.upper()} Arm")
        print("=" * 50)

        # Enable torque
        self.get_logger().info(f'Enabling torque on {group_name}...')
        self.set_torque(namespace, group_name, True)
        self.spin_for(0.5)

        print("[1/5] Moving to HOME position...")
        self.move_arm_group(namespace, group_name, HOME_POSE, move_time=2.5)

        print("[2/5] Moving to FORWARD position...")
        self.move_arm_group(namespace, group_name, FORWARD_POSE, move_time=2.0)

        print("[3/5] Rotating waist 45 degrees...")
        self.move_arm_single(namespace, waist_joint, math.pi / 4.0, move_time=1.5)

        print("[4/5] Returning waist to center...")
        self.move_arm_single(namespace, waist_joint, 0.0, move_time=1.5)

        print("[5/5] Moving to SLEEP position...")
        self.move_arm_group(namespace, group_name, SLEEP_POSE, move_time=2.5)

        print(f"{side.upper()} arm test complete!")

    # ===== GRIPPER CONTROL =====
    def set_grippers(self, position, duration=2.0):
        """
        Set both grippers to a position using wrapper node.

        Args:
            position: 0.0 (open) to 1.0 (closed)
            duration: Time to hold the command (seconds)
        """
        msg = Float64MultiArray()
        msg.data = [float(position)]

        # Publish for duration (reduced rate to avoid bus overload)
        start = time.time()
        while (time.time() - start) < duration:
            self.right_gripper_pub.publish(msg)
            self.left_gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.1)  # 10Hz instead of 20Hz

        # Send stop command (0.5 maps to PWM=0 in wrapper)
        stop_msg = Float64MultiArray()
        stop_msg.data = [0.5]
        self.right_gripper_pub.publish(stop_msg)
        self.left_gripper_pub.publish(stop_msg)
        rclpy.spin_once(self, timeout_sec=0.05)

    def test_grippers(self):
        """Test both grippers using gripper_wrapper_node interface."""
        print()
        print("=" * 50)
        print("TESTING: Grippers (via gripper_wrapper_node)")
        print("=" * 50)

        print("[1/3] Opening both grippers...")
        self.set_grippers(0.0, duration=2.0)

        print("[2/3] Closing both grippers...")
        self.set_grippers(1.0, duration=2.0)

        print("[3/3] Opening both grippers again...")
        self.set_grippers(0.0, duration=2.0)

        print("Gripper test complete!")

    # ===== PAN-TILT CONTROL =====
    def test_pan_tilt(self):
        """Test the pan-tilt camera system."""
        print()
        print("=" * 50)
        print("TESTING: Pan-Tilt Camera System")
        print("=" * 50)

        namespace = 'right_arm'  # Pan-tilt is on right_arm namespace

        # Enable torque
        self.get_logger().info('Enabling torque on pan_tilt...')
        self.set_torque(namespace, 'pan_tilt', True)
        self.spin_for(0.5)

        print("[1/5] Moving to CENTER...")
        self.move_arm_group(namespace, 'pan_tilt', PAN_TILT_CENTER, move_time=1.5)

        print("[2/5] Panning LEFT...")
        self.move_arm_group(namespace, 'pan_tilt', PAN_TILT_LEFT, move_time=1.5)

        print("[3/5] Panning RIGHT...")
        self.move_arm_group(namespace, 'pan_tilt', PAN_TILT_RIGHT, move_time=1.5)

        print("[4/5] Tilting UP...")
        self.move_arm_group(namespace, 'pan_tilt', PAN_TILT_UP, move_time=1.5)

        print("[5/5] Returning to CENTER...")
        self.move_arm_group(namespace, 'pan_tilt', PAN_TILT_CENTER, move_time=1.5)

        print("Pan-tilt test complete!")


def main():
    print()
    print("#" * 60)
    print("#" + " " * 58 + "#")
    print("#" + "       TidyBot2 Full Robot Hardware Test".center(58) + "#")
    print("#" + " " * 58 + "#")
    print("#" * 60)
    print()
    print("This test will sequentially verify all hardware components:")
    print("  1. Phoenix 6 mobile base")
    print("  2. Left arm (Interbotix WX250s)")
    print("  3. Right arm (Interbotix WX250s)")
    print("  4. Grippers (both arms)")
    print("  5. Pan-tilt camera system")
    print()

    rclpy.init()
    node = FullRobotTester()

    try:
        # Wait for components
        print("Detecting hardware components...")
        has_base, has_right_arm, has_left_arm, has_pan_tilt = node.wait_for_components(timeout=10.0)

        print()
        print("Detected components:")
        print(f"  Phoenix 6 Base:  {'YES' if has_base else 'NO'}")
        print(f"  Left Arm:        {'YES' if has_left_arm else 'NO'}")
        print(f"  Right Arm:       {'YES' if has_right_arm else 'NO'}")
        print(f"  Pan-Tilt:        {'YES' if has_pan_tilt else 'NO'}")
        print()

        if not any([has_base, has_right_arm, has_left_arm]):
            print("ERROR: No hardware components detected!")
            print("Make sure to launch the robot first:")
            print("  ros2 launch tidybot_bringup real.launch.py")
            return 1

        input("Press ENTER to start the test sequence (Ctrl+C to cancel)...")

        # Test sequence
        tests_passed = 0
        tests_total = 0

        # 1. Test base
        if has_base:
            tests_total += 1
            try:
                node.test_base()
                tests_passed += 1
            except Exception as e:
                print(f"Base test failed: {e}")
        else:
            print("\nSkipping base test (not detected)")

        # 2. Test left arm
        if has_left_arm:
            tests_total += 1
            try:
                node.test_arm('left')
                tests_passed += 1
            except Exception as e:
                print(f"Left arm test failed: {e}")
        else:
            print("\nSkipping left arm test (not detected)")

        # 3. Test right arm
        if has_right_arm:
            tests_total += 1
            try:
                node.test_arm('right')
                tests_passed += 1
            except Exception as e:
                print(f"Right arm test failed: {e}")
        else:
            print("\nSkipping right arm test (not detected)")

        # 4. Test grippers (using GripperController with SDK mode)
        if has_left_arm or has_right_arm:
            tests_total += 1
            try:
                node.test_grippers()
                tests_passed += 1
            except Exception as e:
                print(f"Gripper test failed: {e}")
        else:
            print("\nSkipping gripper test (no arms detected)")

        # 5. Test pan-tilt
        if has_pan_tilt:
            tests_total += 1
            try:
                node.test_pan_tilt()
                tests_passed += 1
            except Exception as e:
                print(f"Pan-tilt test failed: {e}")
        else:
            print("\nSkipping pan-tilt test (not detected)")

        # Summary
        print()
        print("#" * 60)
        print("#" + " " * 58 + "#")
        print("#" + f"Test Results: {tests_passed}/{tests_total} passed".center(58) + "#")
        print("#" + " " * 58 + "#")
        print("#" * 60)
        print()

        if tests_passed == tests_total:
            print("All hardware tests PASSED!")
        else:
            print(f"WARNING: {tests_total - tests_passed} test(s) failed")

        return 0 if tests_passed == tests_total else 1

    except KeyboardInterrupt:
        print("\n\nTest cancelled by user")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
