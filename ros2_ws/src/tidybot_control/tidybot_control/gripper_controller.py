#!/usr/bin/env python3
"""
Gripper Controller - Importable gripper control for TidyBot2.

This module provides a GripperController class that can be imported by any
test script or node to control the grippers.

Two modes are supported:
    - 'sim': Publishes to /right_gripper/cmd and /left_gripper/cmd
             (Float64MultiArray, 0-1 normalized). Requires gripper_wrapper_node
             for real hardware, or works directly with MuJoCo simulation.
    - 'sdk': Publishes directly to Interbotix SDK topics using PWM control.
             Works with real hardware without gripper_wrapper_node.

Usage:
    from tidybot_control.gripper_controller import GripperController

    # For simulation or when gripper_wrapper_node is running:
    gripper = GripperController(self, mode='sim')

    # For direct real hardware control:
    gripper = GripperController(self, mode='sdk')

    # Control grippers:
    gripper.open('right')
    gripper.close('left')
    gripper.open_both()
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from interbotix_xs_msgs.msg import JointSingleCommand


# PWM pressure limits (from Interbotix SDK)
GRIPPER_PRESSURE_LOWER = 150   # Minimum PWM for movement
GRIPPER_PRESSURE_UPPER = 350   # Maximum PWM (avoid motor overload)


class GripperController:
    """
    Gripper controller that can be used by any ROS2 node.

    Supports both simulation-compatible mode and direct SDK mode.
    """

    def __init__(self, node: Node, mode: str = 'sdk', pressure: float = 1.0):
        """
        Initialize the gripper controller.

        Args:
            node: The ROS2 node instance to create publishers on.
            mode: 'sim' for simulation-compatible topics, 'sdk' for direct Interbotix SDK.
            pressure: Gripper pressure from 0.0 (gentle) to 1.0 (strong). Only used in SDK mode.
        """
        self.node = node
        self.mode = mode
        self.pressure = pressure

        # Calculate PWM value for SDK mode
        self.pwm_value = GRIPPER_PRESSURE_LOWER + pressure * (
            GRIPPER_PRESSURE_UPPER - GRIPPER_PRESSURE_LOWER
        )

        if mode == 'sim':
            # Simulation-compatible publishers
            self.right_pub = node.create_publisher(
                Float64MultiArray, '/right_gripper/cmd', 10
            )
            self.left_pub = node.create_publisher(
                Float64MultiArray, '/left_gripper/cmd', 10
            )
        else:
            # Direct Interbotix SDK publishers
            self.right_pub = node.create_publisher(
                JointSingleCommand, '/right_arm/commands/joint_single', 10
            )
            self.left_pub = node.create_publisher(
                JointSingleCommand, '/left_arm/commands/joint_single', 10
            )

        self.node.get_logger().debug(f'GripperController initialized (mode={mode})')

    def _publish_sim(self, side: str, position: float):
        """Publish to simulation-compatible topic."""
        msg = Float64MultiArray()
        msg.data = [float(position)]

        if side == 'right':
            self.right_pub.publish(msg)
        else:
            self.left_pub.publish(msg)

    def _publish_sdk(self, side: str, pwm: float):
        """Publish to Interbotix SDK topic."""
        cmd = JointSingleCommand()
        cmd.cmd = float(pwm)

        if side == 'right':
            cmd.name = 'right_gripper'
            self.right_pub.publish(cmd)
        else:
            cmd.name = 'left_gripper'
            self.left_pub.publish(cmd)

    def set_position(self, side: str, position: float, duration: float = 2.0):
        """
        Set gripper to a specific position.

        Args:
            side: 'right' or 'left'
            position: 0.0 (fully open) to 1.0 (fully closed)
            duration: Time to publish the command (seconds)
        """
        position = max(0.0, min(1.0, position))

        if self.mode == 'sim':
            # Publish normalized position
            start_time = time.time()
            while (time.time() - start_time) < duration:
                self._publish_sim(side, position)
                rclpy.spin_once(self.node, timeout_sec=0.01)
                time.sleep(0.05)
        else:
            # Convert to PWM: 0.0 -> +pwm (open), 1.0 -> -pwm (close)
            pwm = self.pwm_value - position * (2 * self.pwm_value)
            start_time = time.time()
            while (time.time() - start_time) < duration:
                self._publish_sdk(side, pwm)
                rclpy.spin_once(self.node, timeout_sec=0.01)
                time.sleep(0.05)

            # Stop gripper
            self._publish_sdk(side, 0.0)

    def open(self, side: str, duration: float = 2.0):
        """
        Open the gripper.

        Args:
            side: 'right' or 'left'
            duration: Time to publish the command (seconds)
        """
        self.set_position(side, 0.0, duration)

    def close(self, side: str, duration: float = 2.0):
        """
        Close the gripper.

        Args:
            side: 'right' or 'left'
            duration: Time to publish the command (seconds)
        """
        self.set_position(side, 1.0, duration)

    def open_both(self, duration: float = 2.0):
        """Open both grippers simultaneously."""
        if self.mode == 'sim':
            msg = Float64MultiArray()
            msg.data = [0.0]
            start_time = time.time()
            while (time.time() - start_time) < duration:
                self.right_pub.publish(msg)
                self.left_pub.publish(msg)
                rclpy.spin_once(self.node, timeout_sec=0.01)
                time.sleep(0.05)
        else:
            pwm = self.pwm_value  # Positive = open
            start_time = time.time()
            while (time.time() - start_time) < duration:
                self._publish_sdk('right', pwm)
                self._publish_sdk('left', pwm)
                rclpy.spin_once(self.node, timeout_sec=0.01)
                time.sleep(0.05)
            # Stop grippers
            self._publish_sdk('right', 0.0)
            self._publish_sdk('left', 0.0)

    def close_both(self, duration: float = 2.0):
        """Close both grippers simultaneously."""
        if self.mode == 'sim':
            msg = Float64MultiArray()
            msg.data = [1.0]
            start_time = time.time()
            while (time.time() - start_time) < duration:
                self.right_pub.publish(msg)
                self.left_pub.publish(msg)
                rclpy.spin_once(self.node, timeout_sec=0.01)
                time.sleep(0.05)
        else:
            pwm = -self.pwm_value  # Negative = close
            start_time = time.time()
            while (time.time() - start_time) < duration:
                self._publish_sdk('right', pwm)
                self._publish_sdk('left', pwm)
                rclpy.spin_once(self.node, timeout_sec=0.01)
                time.sleep(0.05)
            # Stop grippers
            self._publish_sdk('right', 0.0)
            self._publish_sdk('left', 0.0)
