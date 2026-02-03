#!/usr/bin/env python3
"""
TidyBot2 Motion Planner Test for Real Hardware

Tests the IK motion planning service on real hardware.
Uses conservative poses suitable for real robot testing.

Usage:
    # Terminal 1: Start real hardware with motion planner
    ros2 launch tidybot_bringup real.launch.py use_planner:=true

    # Terminal 2: Run this test
    ros2 run tidybot_bringup test_planner_real.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tidybot_msgs.srv import PlanToTarget
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointGroupCommand
import numpy as np
import time


class TestPlannerReal(Node):
    """Test node for motion planning on real hardware."""

    # Sleep pose for arms (same as test_arms_real.py)
    SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]

    def __init__(self):
        super().__init__('test_planner_real')

        # Create client for planning service
        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

        # Publishers for direct arm commands (for returning to sleep pose)
        self.arm_cmd_pubs = {
            'right': self.create_publisher(JointGroupCommand, '/right_arm/commands/joint_group', 10),
            'left': self.create_publisher(JointGroupCommand, '/left_arm/commands/joint_group', 10),
        }

        # Subscribe to joint states to verify we're connected and track positions
        self.joint_states_received = False
        self.current_joint_positions = {}
        self.create_subscription(JointState, '/joint_states', self._js_callback, 10)

        self.get_logger().info('=' * 50)
        self.get_logger().info('TidyBot2 IK Planner Test (Real Hardware)')
        self.get_logger().info('=' * 50)

        # Wait for service
        self.get_logger().info('Waiting for /plan_to_target service...')
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service not available! Make sure motion_planner_real_node is running.')
            self.get_logger().error('Launch with: ros2 launch tidybot_bringup real.launch.py use_planner:=true')
            raise RuntimeError('Planning service not available')

        self.get_logger().info('Service connected!')

        # Wait for joint states
        self.get_logger().info('Waiting for joint states...')
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_states_received:
                break

        if not self.joint_states_received:
            self.get_logger().warn('No joint states received - proceeding anyway')

        self.get_logger().info('')

    def _js_callback(self, msg):
        self.joint_states_received = True
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def create_pose(self, x: float, y: float, z: float,
                    qw: float = 1.0, qx: float = 0.0,
                    qy: float = 0.0, qz: float = 0.0) -> Pose:
        """Create a Pose message."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = qw
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        return pose

    def call_service_sync(self, request, timeout_sec=15.0):
        """Call service synchronously using spin_until_future_complete."""
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            self.get_logger().error('Service call timed out!')
            return None

        if future.exception() is not None:
            self.get_logger().error(f'Service call exception: {future.exception()}')
            return None

        return future.result()

    def plan_only(self, arm_name: str, pose: Pose,
                  use_orientation: bool = True) -> bool:
        """Plan without executing - for safety check."""
        request = PlanToTarget.Request()
        request.arm_name = arm_name
        request.target_pose = pose
        request.use_orientation = use_orientation
        request.execute = False  # Plan only
        request.duration = 2.0
        request.max_condition_number = 100.0

        self.get_logger().info(
            f'Planning {arm_name} arm to: '
            f'({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})'
        )

        result = self.call_service_sync(request)
        if result is None:
            return False

        if result.success:
            self.get_logger().info(f'  SUCCESS: {result.message}')
            self.get_logger().info(f'  Solution: {[f"{j:.3f}" for j in result.joint_positions]}')
            return True
        else:
            self.get_logger().warn(f'  FAILED: {result.message}')
            return False

    def plan_and_execute(self, arm_name: str, pose: Pose,
                         use_orientation: bool = True,
                         duration: float = 3.0) -> bool:
        """Send planning request and execute."""
        request = PlanToTarget.Request()
        request.arm_name = arm_name
        request.target_pose = pose
        request.use_orientation = use_orientation
        request.execute = True
        request.duration = duration
        request.max_condition_number = 100.0

        self.get_logger().info(
            f'Planning and executing {arm_name} arm to: '
            f'({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})'
        )

        result = self.call_service_sync(request)
        if result is None:
            return False

        if result.success:
            self.get_logger().info(f'  SUCCESS: {result.message}')
            if result.executed:
                self.get_logger().info(f'  Executing over {duration}s...')
            return True
        else:
            self.get_logger().warn(f'  FAILED: {result.message}')
            return False

    def get_arm_positions(self, arm_name: str) -> np.ndarray:
        """Get current joint positions for an arm."""
        joint_names = [f'{arm_name}_waist', f'{arm_name}_shoulder', f'{arm_name}_elbow',
                       f'{arm_name}_forearm_roll', f'{arm_name}_wrist_angle', f'{arm_name}_wrist_rotate']
        positions = np.array([self.current_joint_positions.get(jname, 0.0) for jname in joint_names])
        return positions

    def go_to_sleep_pose(self, arm_name: str, max_joint_speed: float = 0.5):
        """Send arm to sleep pose using smooth interpolated trajectory.

        Args:
            arm_name: 'right' or 'left'
            max_joint_speed: Maximum joint velocity in rad/s (default 0.5 rad/s ~ 30 deg/s)
        """
        # Spin to get latest joint states
        rclpy.spin_once(self, timeout_sec=0.1)

        # Get current and target positions
        current = self.get_arm_positions(arm_name)
        target = np.array(self.SLEEP_POSE)

        # Calculate required duration based on max joint difference
        max_diff = np.max(np.abs(target - current))
        duration = max(max_diff / max_joint_speed, 1.0)  # At least 1 second

        self.get_logger().info(f'Moving {arm_name} arm to sleep pose over {duration:.1f}s (max joint diff: {max_diff:.2f} rad)')

        # Interpolate trajectory with smooth cosine profile
        rate_hz = 50.0
        dt = 1.0 / rate_hz
        num_steps = max(int(duration * rate_hz), 1)

        for i in range(num_steps + 1):
            t = i / num_steps
            # Cosine interpolation for smooth acceleration/deceleration
            alpha = 0.5 * (1 - np.cos(np.pi * t))

            # Interpolate
            q = current + alpha * (target - current)

            cmd = JointGroupCommand()
            cmd.name = f'{arm_name}_arm'
            cmd.cmd = q.tolist()
            self.arm_cmd_pubs[arm_name].publish(cmd)

            if i < num_steps:
                time.sleep(dt)

    def run_tests(self):
        """Run the test sequence."""
        # Target positions must be in the arm's workspace!
        # Right arm base is at x=-0.15, y=-0.12 (roughly)
        # Left arm base is at x=0.15, y=-0.12 (roughly)
        #
        # Swapped x positions to avoid collision:
        # - Right arm reaches to negative x (its own right side)
        # - Left arm reaches to positive x (its own left side)

        # Test 1: Plan only (no execution) - safe test
        self.get_logger().info('-' * 40)
        self.get_logger().info('Test 1: Right arm - PLAN ONLY (no execution)')
        # Right arm reaches to positive x (away from left arm)
        pose1 = self.create_pose(0.05, -0.35, 0.55)
        success1 = self.plan_only('right', pose1, use_orientation=False)
        time.sleep(1.0)

        # Test 2: Plan only for left arm
        self.get_logger().info('-' * 40)
        self.get_logger().info('Test 2: Left arm - PLAN ONLY (no execution)')
        # Left arm reaches to negative x (away from right arm)
        pose2 = self.create_pose(-0.05, -0.35, 0.55)
        success2 = self.plan_only('left', pose2, use_orientation=False)
        time.sleep(1.0)

        if success1 and success2:
            self.get_logger().info('')
            self.get_logger().info('Planning tests passed!')
            self.get_logger().info('')

            # Ask before executing
            self.get_logger().info('=' * 50)
            self.get_logger().info('Ready to test execution.')
            self.get_logger().info('The robot will move! Make sure the workspace is clear.')
            self.get_logger().info('Press Ctrl+C now if you want to abort.')
            self.get_logger().info('Executing in 5 seconds...')
            self.get_logger().info('=' * 50)
            time.sleep(5.0)

            # Test 3: Execute right arm movement (3 sec trajectory)
            self.get_logger().info('-' * 40)
            self.get_logger().info('Test 3: Right arm - EXECUTE')
            pose3 = self.create_pose(-0.1, -0.35, 0.55)
            self.plan_and_execute('right', pose3, use_orientation=False, duration=3.0)
            time.sleep(1.0)  # Small pause between movements

            # Test 4: Execute left arm movement (3 sec trajectory)
            self.get_logger().info('-' * 40)
            self.get_logger().info('Test 4: Left arm - EXECUTE')
            pose4 = self.create_pose(0.1, -0.35, 0.55)
            self.plan_and_execute('left', pose4, use_orientation=False, duration=3.0)
            time.sleep(1.0)

            # Test 5: Move right arm to different position (3 sec trajectory)
            self.get_logger().info('-' * 40)
            self.get_logger().info('Test 5: Right arm - different position')
            pose5 = self.create_pose(-0.0, -0.60, 0.30)
            self.plan_and_execute('right', pose5, use_orientation=False, duration=3.0)
            time.sleep(1.0)

        else:
            self.get_logger().error('Planning tests failed - skipping execution tests')

        # Return arms to sleep pose (duration calculated automatically based on joint diff)
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Returning arms to sleep pose...')
        self.get_logger().info('=' * 50)
        self.go_to_sleep_pose('right')
        self.go_to_sleep_pose('left')

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Test complete!')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = TestPlannerReal()

    try:
        # Run tests directly instead of using timer callback
        # This avoids nested executor issues with spin_until_future_complete
        node.run_tests()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
