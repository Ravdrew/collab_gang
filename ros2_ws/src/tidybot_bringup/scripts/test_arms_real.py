#!/usr/bin/env python3
"""
Test TidyBot2 Arms, Pan-Tilt, and Grippers using simulation-compatible topics.

This script uses the same topics as the MuJoCo simulation, allowing the same
code to work on both simulation and real hardware (via arm_wrapper_node).

Topics used (same as simulation):
    - /right_arm/joint_cmd (Float64MultiArray) - right arm 6 joint positions
    - /left_arm/joint_cmd (Float64MultiArray) - left arm 6 joint positions
    - /right_gripper/cmd (Float64MultiArray) - right gripper (0=open, 1=closed)
    - /left_gripper/cmd (Float64MultiArray) - left gripper (0=open, 1=closed)

Hardware Setup (Dual U2D2):
    - U2D2 #1 (/dev/ttyUSB_RIGHT): Right arm (IDs 1-9) + Pan-tilt (IDs 21-22)
    - U2D2 #2 (/dev/ttyUSB_LEFT): Left arm (IDs 11-19)

Usage:
    # First, launch real.launch.py (includes arm_wrapper_node and gripper_wrapper_node):
    ros2 launch tidybot_bringup real.launch.py

    # Then run this test:
    ros2 run tidybot_bringup test_arms_real.py
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import TorqueEnable
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# Predefined poses (6 joints: waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate)
HOME_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]
FORWARD_POSE = [0.0, -0.5, 0.5, 0.0, 0.0, 0.0]

# Pan-tilt poses [pan, tilt]
PAN_TILT_CENTER = [0.0, 0.0]
PAN_TILT_LEFT = [0.5, 0.0]
PAN_TILT_RIGHT = [-0.5, 0.0]
PAN_TILT_UP = [0.0, -0.3]
PAN_TILT_DOWN = [0.0, 0.3]


class RobotTester(Node):
    def __init__(self):
        super().__init__('robot_tester')

        # Arm publishers - simulation-compatible topics (Float64MultiArray)
        # These are translated by arm_wrapper_node to xs_sdk commands
        self.right_arm_pub = self.create_publisher(
            Float64MultiArray, '/right_arm/joint_cmd', 10
        )
        self.left_arm_pub = self.create_publisher(
            Float64MultiArray, '/left_arm/joint_cmd', 10
        )

        # Pan-tilt publisher - directly to xs_sdk (no wrapper needed)
        self.pan_tilt_pub = self.create_publisher(
            JointGroupCommand, '/right_arm/commands/joint_group', 10
        )

        # Torque clients (still needed for enabling motors)
        self.right_torque_client = self.create_client(
            TorqueEnable, '/right_arm/torque_enable'
        )
        self.left_torque_client = self.create_client(
            TorqueEnable, '/left_arm/torque_enable'
        )

        # Subscribe to joint states from both namespaces
        self.right_joint_states = None
        self.left_joint_states = None
        self.pan_tilt_joint_states = None
        self.right_js_sub = self.create_subscription(
            JointState, '/right_arm/joint_states', self._right_joint_state_cb, 10
        )
        self.left_js_sub = self.create_subscription(
            JointState, '/left_arm/joint_states', self._left_joint_state_cb, 10
        )
        self.pan_tilt_js_sub = self.create_subscription(
            JointState, '/pan_tilt/joint_states', self._pan_tilt_joint_state_cb, 10
        )

        # Gripper publishers (simulation-compatible, translated by gripper_wrapper_node)
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray, '/right_gripper/cmd', 10
        )
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray, '/left_gripper/cmd', 10
        )

        self.get_logger().info('Waiting for xs_sdk services...')
        # Wait for at least one service to be available
        right_ready = self.right_torque_client.wait_for_service(timeout_sec=5.0)
        left_ready = self.left_torque_client.wait_for_service(timeout_sec=5.0)

        if right_ready:
            self.get_logger().info('Connected to right_arm xs_sdk!')
        if left_ready:
            self.get_logger().info('Connected to left_arm xs_sdk!')
        if not right_ready and not left_ready:
            self.get_logger().warn('No xs_sdk services found!')

    def _right_joint_state_cb(self, msg):
        self.right_joint_states = msg

    def _left_joint_state_cb(self, msg):
        self.left_joint_states = msg

    def _pan_tilt_joint_state_cb(self, msg):
        self.pan_tilt_joint_states = msg

    def wait_for_joint_states(self, timeout=5.0):
        """Wait until we receive joint states from arms and pan-tilt (or timeout)."""
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            # Keep spinning until we have arms and pan-tilt, or timeout
            has_arms = (self.right_joint_states is not None or
                       self.left_joint_states is not None)
            has_pan_tilt = self.pan_tilt_joint_states is not None
            if has_arms and has_pan_tilt:
                return True
        # Return True if we got at least one arm
        return self.right_joint_states is not None or self.left_joint_states is not None

    def move_arm(self, arm_side, positions, move_time=2.0):
        """Move arm to specified positions using simulation-compatible topic.

        Args:
            arm_side: 'left' or 'right'
            positions: list of 6 joint positions in radians
            move_time: time to wait for motion to complete
        """
        msg = Float64MultiArray()
        msg.data = positions

        self.get_logger().info(f'Moving {arm_side} arm to {[f"{p:.2f}" for p in positions]}')

        if arm_side == 'right':
            self.right_arm_pub.publish(msg)
        else:
            self.left_arm_pub.publish(msg)

        # Wait for motion to complete
        time.sleep(move_time)
        # Spin to process callbacks
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

    def move_single_joint(self, arm_side, joint_index, position, move_time=1.5):
        """Move a single joint while keeping others at current position.

        Args:
            arm_side: 'left' or 'right'
            joint_index: 0-5 for waist through wrist_rotate
            position: target position in radians
            move_time: time to wait for motion
        """
        # Get current joint states
        if arm_side == 'right':
            js = self.right_joint_states
        else:
            js = self.left_joint_states

        if js is None:
            self.get_logger().warn(f'No joint states for {arm_side} arm')
            return

        # Build position array from current state
        positions = list(js.position[:6]) if len(js.position) >= 6 else [0.0] * 6
        positions[joint_index] = position

        joint_names = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
        self.get_logger().info(f'Moving {arm_side} {joint_names[joint_index]} to {position:.2f}')

        self.move_arm(arm_side, positions, move_time)

    def move_pan_tilt(self, positions, move_time=1.5):
        """Move pan-tilt to specified positions (directly to xs_sdk)."""
        msg = JointGroupCommand()
        msg.name = 'pan_tilt'
        msg.cmd = positions

        self.get_logger().info(f'Moving pan_tilt to {[f"{p:.2f}" for p in positions]}')
        self.pan_tilt_pub.publish(msg)

        time.sleep(move_time)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

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

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()

    def test_arm(self, arm_side):
        """Run test routine on one arm.

        Args:
            arm_side: 'left' or 'right'
        """
        namespace = f'{arm_side}_arm'
        group_name = f'{arm_side}_arm'

        print()
        print(f"Testing {arm_side.upper()} ARM...")
        print("-" * 40)

        # Enable torque
        self.get_logger().info(f'Enabling torque on {namespace}/{group_name}...')
        self.set_torque(namespace, group_name, True)
        time.sleep(0.5)

        # Test 1: Go to home position
        print(f"[1/5] Moving to HOME position...")
        self.move_arm(arm_side, HOME_POSE, move_time=2.5)

        # Test 2: Move forward
        print(f"[2/5] Moving to FORWARD position...")
        self.move_arm(arm_side, FORWARD_POSE, move_time=2.0)

        # Test 3: Rotate waist
        print(f"[3/5] Rotating waist 45 degrees...")
        self.move_single_joint(arm_side, 0, math.pi / 4.0, move_time=1.5)

        # Test 4: Return waist
        print(f"[4/5] Returning waist to center...")
        self.move_single_joint(arm_side, 0, 0.0, move_time=1.5)

        # Test 5: Go to sleep position
        print(f"[5/5] Moving to SLEEP position...")
        self.move_arm(arm_side, SLEEP_POSE, move_time=2.5)

        print(f"{arm_side.upper()} ARM test complete!")

    def test_pan_tilt(self):
        """Run test routine on pan-tilt (on right_arm namespace/U2D2)."""
        print()
        print("Testing PAN-TILT...")
        print("-" * 40)

        # Enable torque
        self.get_logger().info('Enabling torque on pan_tilt...')
        self.set_torque('right_arm', 'pan_tilt', True)
        time.sleep(0.5)

        # Test sequence
        print("[1/5] Moving to CENTER...")
        self.move_pan_tilt(PAN_TILT_CENTER, move_time=1.5)

        print("[2/5] Panning LEFT...")
        self.move_pan_tilt(PAN_TILT_LEFT, move_time=1.5)

        print("[3/5] Panning RIGHT...")
        self.move_pan_tilt(PAN_TILT_RIGHT, move_time=1.5)

        print("[4/5] Tilting UP...")
        self.move_pan_tilt(PAN_TILT_UP, move_time=1.5)

        print("[5/5] Returning to CENTER...")
        self.move_pan_tilt(PAN_TILT_CENTER, move_time=1.5)

        print("PAN-TILT test complete!")

    def set_gripper(self, side, position, duration=2.0):
        """
        Set gripper position using wrapper node.

        Args:
            side: 'right' or 'left'
            position: 0.0 (open) to 1.0 (closed)
            duration: Time to hold the command (seconds)
        """
        msg = Float64MultiArray()
        msg.data = [float(position)]

        pub = self.right_gripper_pub if side == 'right' else self.left_gripper_pub
        state_desc = 'OPEN' if position < 0.5 else 'CLOSED'
        self.get_logger().info(f'{side.capitalize()} gripper -> {state_desc} ({position:.1f})')

        # Publish for duration (reduced rate to avoid bus overload)
        start = time.time()
        while (time.time() - start) < duration:
            pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)  # 10Hz instead of 20Hz

        # Send stop command (0.5 maps to PWM=0 in wrapper)
        stop_msg = Float64MultiArray()
        stop_msg.data = [0.5]
        pub.publish(stop_msg)
        rclpy.spin_once(self, timeout_sec=0.05)

    def test_grippers(self):
        """Test both grippers using wrapper node interface."""
        print()
        print("Testing GRIPPERS (via gripper_wrapper_node)...")
        print("-" * 40)

        # Allow bus to settle after pan-tilt commands
        print("Waiting for bus to settle...")
        time.sleep(1.0)

        print("[1/4] Opening RIGHT gripper...")
        self.set_gripper('right', 0.0, duration=2.0)

        print("[2/4] Closing RIGHT gripper...")
        self.set_gripper('right', 1.0, duration=2.0)

        # Small delay before switching to left arm U2D2
        time.sleep(1.0)

        print("[3/4] Opening LEFT gripper...")
        self.set_gripper('left', 0.0, duration=2.0)

        print("[4/4] Closing LEFT gripper...")
        self.set_gripper('left', 1.0, duration=2.0)

        # Leave grippers open
        print("[5/5] Opening both grippers...")
        msg = Float64MultiArray()
        msg.data = [0.0]
        start = time.time()
        while (time.time() - start) < 2.0:
            self.right_gripper_pub.publish(msg)
            self.left_gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

        # Send stop command
        stop_msg = Float64MultiArray()
        stop_msg.data = [0.5]
        self.right_gripper_pub.publish(stop_msg)
        self.left_gripper_pub.publish(stop_msg)
        rclpy.spin_once(self, timeout_sec=0.05)

        print("GRIPPER test complete!")


def main():
    print("=" * 60)
    print("TidyBot2 Full Robot Test (Simulation-Compatible Topics)")
    print("=" * 60)
    print()
    print("This test uses the same topics as simulation:")
    print("  - /right_arm/joint_cmd (Float64MultiArray)")
    print("  - /left_arm/joint_cmd (Float64MultiArray)")
    print("  - /right_gripper/cmd, /left_gripper/cmd")
    print()
    print("Hardware configuration:")
    print("  U2D2 #1 (/dev/ttyUSB_RIGHT): Right arm + Pan-tilt")
    print("  U2D2 #2 (/dev/ttyUSB_LEFT): Left arm")
    print()

    rclpy.init()
    node = RobotTester()

    try:
        # Wait for joint states from both arms
        print("Waiting for joint states from both arms...")
        if not node.wait_for_joint_states(timeout=10.0):
            print("ERROR: No joint states received!")
            print("Make sure to launch the arm drivers first:")
            print("  ros2 launch tidybot_bringup real.launch.py")
            return 1

        # Check which components are available
        right_joints = node.right_joint_states.name if node.right_joint_states else []
        left_joints = node.left_joint_states.name if node.left_joint_states else []
        pan_tilt_joints = node.pan_tilt_joint_states.name if node.pan_tilt_joint_states else []

        has_right = len(right_joints) > 0
        has_left = len(left_joints) > 0
        has_pan_tilt = 'camera_pan' in pan_tilt_joints and 'camera_tilt' in pan_tilt_joints

        print()
        print("Detected components:")
        print(f"  Right arm (/right_arm): {'Yes' if has_right else 'No'}")
        if has_right:
            print(f"    Joints: {right_joints}")
        print(f"  Left arm (/left_arm):   {'Yes' if has_left else 'No'}")
        if has_left:
            print(f"    Joints: {left_joints}")
        print(f"  Pan-tilt (/pan_tilt):   {'Yes' if has_pan_tilt else 'No'}")
        if has_pan_tilt:
            print(f"    Joints: {pan_tilt_joints}")

        # Test left arm first (per user request)
        if has_left:
            node.test_arm('left')
        else:
            print("\nSkipping left arm (not found)")

        # Then test right arm
        if has_right:
            node.test_arm('right')
        else:
            print("\nSkipping right arm (not found)")

        # Then test pan-tilt
        if has_pan_tilt:
            node.test_pan_tilt()
        else:
            print("\nSkipping pan-tilt (not found)")

        # Test grippers (requires gripper_wrapper_node from real.launch.py)
        if has_right or has_left:
            node.test_grippers()
        else:
            print("\nSkipping grippers (no arms found)")

        print()
        print("=" * 60)
        print("All tests complete!")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
