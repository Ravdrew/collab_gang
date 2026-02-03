#!/usr/bin/env python3
"""
TidyBot2 Motion Planner Demo

Demonstrates the motion planning service with collision and singularity checking.
Plans arm motions to various target poses and executes them.

Usage:
    # Terminal 1: Start simulation
    ros2 launch tidybot_bringup sim.launch.py

    # Terminal 2: Run this demo
    ros2 run tidybot_bringup test_planner_sim.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tidybot_msgs.srv import PlanToTarget
import time
import numpy as np


class TestPlanner(Node):
    """Demo node for testing motion planning service."""

    def __init__(self):
        super().__init__('test_planner')

        # Create client for planning service
        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

        self.get_logger().info('=' * 50)
        self.get_logger().info('TidyBot2 Motion Planner Demo')
        self.get_logger().info('=' * 50)

        # Wait for service
        self.get_logger().info('Waiting for /plan_to_target service...')
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Service connected!')
        self.get_logger().info('')

        # Run demo after short delay
        self.create_timer(1.0, self.run_demo_once)
        self.demo_run = False

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

    def plan_and_execute(self, arm_name: str, pose: Pose,
                        use_orientation: bool = True,
                        duration: float = 2.0) -> bool:
        """Send planning request and wait for result."""
        request = PlanToTarget.Request()
        request.arm_name = arm_name
        request.target_pose = pose
        request.use_orientation = use_orientation
        request.execute = True
        request.duration = duration
        request.max_condition_number = 100.0

        self.get_logger().info(
            f'Planning {arm_name} arm to: '
            f'({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})'
        )

        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('Service call failed!')
            return False

        result = future.result()
        if result.success:
            self.get_logger().info(f'  SUCCESS: {result.message}')
            if result.executed:
                self.get_logger().info(f'  Executing over {duration}s...')
            return True
        else:
            self.get_logger().warn(f'  FAILED: {result.message}')
            return False

    def run_demo_once(self):
        """Run the demo sequence once."""
        if self.demo_run:
            return
        self.demo_run = True

        # Top-down grasp orientation: gripper pointing down
        # Quaternion for 180 deg rotation around X-axis
        top_down = {'qw': 0.0, 'qx': 1.0, 'qy': 0.0, 'qz': 0.0}

        # Demo 1: Move right arm to position in front of robot
        self.get_logger().info('-' * 40)
        self.get_logger().info('Demo 1: Right arm forward reach')
        pose1 = self.create_pose(0.35, -0.15, 0.45, **top_down)
        self.plan_and_execute('right', pose1, use_orientation=True, duration=2.0)
        time.sleep(2.5)

        # Demo 2: Move right arm to side (position only)
        self.get_logger().info('-' * 40)
        self.get_logger().info('Demo 2: Right arm side reach (position only)')
        pose2 = self.create_pose(0.25, -0.30, 0.50)
        self.plan_and_execute('right', pose2, use_orientation=False, duration=2.0)
        time.sleep(2.5)

        # Demo 3: Try a pose that might cause arm collision (intentional test)
        self.get_logger().info('-' * 40)
        self.get_logger().info('Demo 3: Left arm forward (should succeed)')
        pose3 = self.create_pose(0.35, 0.15, 0.45, **top_down)
        self.plan_and_execute('left', pose3, use_orientation=True, duration=2.0)
        time.sleep(2.5)

        # Demo 4: Try to reach very far (might fail due to limits/singularity)
        self.get_logger().info('-' * 40)
        self.get_logger().info('Demo 4: Right arm far reach (may fail - testing limits)')
        pose4 = self.create_pose(0.60, -0.15, 0.40, **top_down)
        self.plan_and_execute('right', pose4, use_orientation=True, duration=2.0)
        time.sleep(2.5)

        # Demo 5: Move both arms close (testing collision check)
        self.get_logger().info('-' * 40)
        self.get_logger().info('Demo 5: Right arm center (testing collision check)')
        pose5 = self.create_pose(0.35, -0.05, 0.45, **top_down)  # Close to center
        self.plan_and_execute('right', pose5, use_orientation=True, duration=2.0)
        time.sleep(2.5)

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Demo complete!')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = TestPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
