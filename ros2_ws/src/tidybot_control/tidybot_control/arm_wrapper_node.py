#!/usr/bin/env python3
"""
Arm Command Wrapper Node for TidyBot2.

Provides simulation-compatible arm interface for real hardware.
Translates from:
    /right_arm/joint_cmd (Float64MultiArray) - 6 joint positions
    /left_arm/joint_cmd (Float64MultiArray) - 6 joint positions
To Interbotix SDK:
    /right_arm/commands/joint_group (JointGroupCommand)
    /left_arm/commands/joint_group (JointGroupCommand)

This allows the same user code to work for both simulation and real hardware.
The actual motor control is handled by xs_sdk nodes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from interbotix_xs_msgs.msg import JointGroupCommand


class ArmWrapperNode(Node):
    """Wrapper node to translate sim arm commands to Interbotix SDK."""

    def __init__(self):
        super().__init__('arm_wrapper')

        # Publishers to Interbotix SDK
        self.right_arm_pub = self.create_publisher(
            JointGroupCommand, '/right_arm/commands/joint_group', 10
        )
        self.left_arm_pub = self.create_publisher(
            JointGroupCommand, '/left_arm/commands/joint_group', 10
        )

        # Subscribers - same topics as MuJoCo simulation
        self.right_arm_sub = self.create_subscription(
            Float64MultiArray, '/right_arm/joint_cmd',
            lambda msg: self._arm_callback(msg, 'right'), 10
        )
        self.left_arm_sub = self.create_subscription(
            Float64MultiArray, '/left_arm/joint_cmd',
            lambda msg: self._arm_callback(msg, 'left'), 10
        )

        self.get_logger().info('Arm wrapper node started')
        self.get_logger().info('  Listening on /right_arm/joint_cmd and /left_arm/joint_cmd')
        self.get_logger().info('  Publishing to Interbotix SDK joint_group topics')

    def _arm_callback(self, msg: Float64MultiArray, side: str):
        """
        Handle arm command from simulation-compatible topic.

        Args:
            msg: Float64MultiArray with 6 joint positions in radians
            side: 'right' or 'left'
        """
        if len(msg.data) < 6:
            self.get_logger().warn(f'{side} arm: expected 6 joints, got {len(msg.data)}')
            return

        # Create Interbotix JointGroupCommand
        cmd = JointGroupCommand()
        cmd.name = f'{side}_arm'  # Group name defined in xs_sdk config (right_arm or left_arm)
        cmd.cmd = list(msg.data[:6])  # First 6 joint positions

        if side == 'right':
            self.right_arm_pub.publish(cmd)
        else:
            self.left_arm_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ArmWrapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
