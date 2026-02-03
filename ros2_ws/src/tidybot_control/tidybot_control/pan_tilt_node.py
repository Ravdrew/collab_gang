#!/usr/bin/env python3
"""
Pan-Tilt Camera Control Node for TidyBot2.

This node controls the pan-tilt camera mechanism using Dynamixel XL330 servos.
It provides the SAME interface as the MuJoCo bridge for seamless sim-to-real transfer.

Topics (SAME as MuJoCo bridge):
- Subscribe: /camera/pan_tilt_cmd (Float64MultiArray) - [pan, tilt] in radians
- Publish: /camera/pan_tilt_state (Float64MultiArray) - current [pan, tilt] positions

Motor Configuration (on shared U2D2 bus):
- Pan motor: ID 21 (XL330)
- Tilt motor: ID 22 (XL330)

Motor IDs can be remapped using Dynamixel Wizard if needed for shared bus operation.

Prerequisites:
- Dynamixel SDK: pip install dynamixel-sdk
- U2D2 adapter connected to USB

Usage:
    ros2 run tidybot_control pan_tilt_node
    ros2 run tidybot_control pan_tilt_node --ros-args -p port_name:=/dev/ttyUSB0
"""

import math
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Try to import Dynamixel SDK
try:
    from dynamixel_sdk import (
        PortHandler,
        PacketHandler,
        GroupSyncWrite,
        GroupSyncRead,
        COMM_SUCCESS,
        DXL_LOBYTE,
        DXL_LOWORD,
        DXL_HIBYTE,
        DXL_HIWORD,
    )
    HAS_DYNAMIXEL = True
except ImportError:
    HAS_DYNAMIXEL = False

# Dynamixel XL330 Control Table
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128
ADDR_OPERATING_MODE = 11
ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCELERATION = 108

# Protocol version
PROTOCOL_VERSION = 2.0

# Default values
DEFAULT_BAUDRATE = 1000000
DEFAULT_PORT = '/dev/ttyUSB0'

# XL330 position units: 4096 counts per revolution
POSITION_UNIT = 2 * math.pi / 4096  # radians per count
VELOCITY_UNIT = 0.229 * 2 * math.pi / 60  # radians per second (0.229 rpm per unit)


class PanTiltNode(Node):
    """
    ROS2 node for pan-tilt camera control via Dynamixel servos.

    Provides the SAME interface as MuJoCo bridge for seamless sim-to-real transfer.
    """

    def __init__(self):
        super().__init__('pan_tilt')

        # Check for Dynamixel SDK
        if not HAS_DYNAMIXEL:
            self.get_logger().error(
                'Dynamixel SDK not installed! Install with:\n'
                '  pip install dynamixel-sdk'
            )
            raise RuntimeError('Dynamixel SDK not available')

        # Declare parameters
        self.declare_parameter('port_name', DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('pan_motor_id', 21)
        self.declare_parameter('tilt_motor_id', 22)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('profile_velocity', 100)  # Dynamixel units
        self.declare_parameter('profile_acceleration', 50)  # Dynamixel units

        # Get parameters
        port_name = self.get_parameter('port_name').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.pan_id = self.get_parameter('pan_motor_id').get_parameter_value().integer_value
        self.tilt_id = self.get_parameter('tilt_motor_id').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        profile_velocity = self.get_parameter('profile_velocity').get_parameter_value().integer_value
        profile_acceleration = self.get_parameter('profile_acceleration').get_parameter_value().integer_value

        # Thread safety
        self.lock = threading.Lock()

        # Initialize port handler
        self.port_handler = PortHandler(port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # Open port
        if not self.port_handler.openPort():
            self.get_logger().error(f'Failed to open port {port_name}')
            raise RuntimeError(f'Failed to open port {port_name}')

        # Set baudrate
        if not self.port_handler.setBaudRate(baudrate):
            self.get_logger().error(f'Failed to set baudrate to {baudrate}')
            raise RuntimeError(f'Failed to set baudrate to {baudrate}')

        self.get_logger().info(f'Connected to Dynamixel bus on {port_name} at {baudrate} baud')

        # Configure motors
        self._configure_motor(self.pan_id, 'pan', profile_velocity, profile_acceleration)
        self._configure_motor(self.tilt_id, 'tilt', profile_velocity, profile_acceleration)

        # Enable torque
        self._set_torque(self.pan_id, True)
        self._set_torque(self.tilt_id, True)

        # Initialize sync read for positions
        self.sync_read = GroupSyncRead(
            self.port_handler, self.packet_handler,
            ADDR_PRESENT_POSITION, 4
        )
        self.sync_read.addParam(self.pan_id)
        self.sync_read.addParam(self.tilt_id)

        # Initialize sync write for positions
        self.sync_write = GroupSyncWrite(
            self.port_handler, self.packet_handler,
            ADDR_GOAL_POSITION, 4
        )

        # Current state
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.target_pan = 0.0
        self.target_tilt = 0.0

        # Publishers - SAME as MuJoCo bridge
        self.state_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_state', 10
        )

        # Subscribers - SAME as MuJoCo bridge
        self.cmd_sub = self.create_subscription(
            Float64MultiArray, '/camera/pan_tilt_cmd', self.cmd_callback, 10
        )

        # Timer for publishing state and sending commands
        self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info('Pan-tilt node ready')
        self.get_logger().info(f'  Pan motor ID: {self.pan_id}')
        self.get_logger().info(f'  Tilt motor ID: {self.tilt_id}')

    def _configure_motor(self, motor_id, name, profile_velocity, profile_acceleration):
        """Configure a Dynamixel motor for position control."""
        # Disable torque before configuring
        self._set_torque(motor_id, False)

        # Set operating mode to Position Control (value 3)
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ADDR_OPERATING_MODE, 3
        )
        if result != COMM_SUCCESS:
            self.get_logger().error(
                f'Failed to set operating mode for {name} motor: '
                f'{self.packet_handler.getTxRxResult(result)}'
            )
        elif error != 0:
            self.get_logger().error(
                f'Error setting operating mode for {name} motor: '
                f'{self.packet_handler.getRxPacketError(error)}'
            )

        # Set profile velocity
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, ADDR_PROFILE_VELOCITY, profile_velocity
        )
        if result != COMM_SUCCESS:
            self.get_logger().warn(
                f'Failed to set profile velocity for {name} motor: '
                f'{self.packet_handler.getTxRxResult(result)}'
            )

        # Set profile acceleration
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, ADDR_PROFILE_ACCELERATION, profile_acceleration
        )
        if result != COMM_SUCCESS:
            self.get_logger().warn(
                f'Failed to set profile acceleration for {name} motor: '
                f'{self.packet_handler.getTxRxResult(result)}'
            )

        self.get_logger().info(f'Configured {name} motor (ID {motor_id})')

    def _set_torque(self, motor_id, enable):
        """Enable or disable torque for a motor."""
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 1 if enable else 0
        )
        if result != COMM_SUCCESS:
            self.get_logger().error(
                f'Failed to {"enable" if enable else "disable"} torque for motor {motor_id}: '
                f'{self.packet_handler.getTxRxResult(result)}'
            )
        elif error != 0:
            self.get_logger().error(
                f'Error {"enabling" if enable else "disabling"} torque for motor {motor_id}: '
                f'{self.packet_handler.getRxPacketError(error)}'
            )

    def _radians_to_counts(self, radians):
        """Convert radians to Dynamixel position counts (centered at 2048)."""
        counts = int(radians / POSITION_UNIT) + 2048
        return max(0, min(4095, counts))

    def _counts_to_radians(self, counts):
        """Convert Dynamixel position counts to radians (centered at 2048)."""
        return (counts - 2048) * POSITION_UNIT

    def cmd_callback(self, msg: Float64MultiArray):
        """Handle pan-tilt commands."""
        if len(msg.data) < 2:
            self.get_logger().warn(f'Expected 2 values [pan, tilt], got {len(msg.data)}')
            return

        with self.lock:
            self.target_pan = msg.data[0]
            self.target_tilt = msg.data[1]

    def timer_callback(self):
        """Read state and send commands."""
        with self.lock:
            # Read current positions
            result = self.sync_read.txRxPacket()
            if result == COMM_SUCCESS:
                if self.sync_read.isAvailable(self.pan_id, ADDR_PRESENT_POSITION, 4):
                    pan_counts = self.sync_read.getData(self.pan_id, ADDR_PRESENT_POSITION, 4)
                    self.current_pan = self._counts_to_radians(pan_counts)

                if self.sync_read.isAvailable(self.tilt_id, ADDR_PRESENT_POSITION, 4):
                    tilt_counts = self.sync_read.getData(self.tilt_id, ADDR_PRESENT_POSITION, 4)
                    self.current_tilt = self._counts_to_radians(tilt_counts)

            # Send target positions
            pan_counts = self._radians_to_counts(self.target_pan)
            tilt_counts = self._radians_to_counts(self.target_tilt)

            # Clear previous sync write data
            self.sync_write.clearParam()

            # Add pan position
            pan_data = [
                DXL_LOBYTE(DXL_LOWORD(pan_counts)),
                DXL_HIBYTE(DXL_LOWORD(pan_counts)),
                DXL_LOBYTE(DXL_HIWORD(pan_counts)),
                DXL_HIBYTE(DXL_HIWORD(pan_counts)),
            ]
            self.sync_write.addParam(self.pan_id, pan_data)

            # Add tilt position
            tilt_data = [
                DXL_LOBYTE(DXL_LOWORD(tilt_counts)),
                DXL_HIBYTE(DXL_LOWORD(tilt_counts)),
                DXL_LOBYTE(DXL_HIWORD(tilt_counts)),
                DXL_HIBYTE(DXL_HIWORD(tilt_counts)),
            ]
            self.sync_write.addParam(self.tilt_id, tilt_data)

            # Transmit
            self.sync_write.txPacket()

        # Publish state
        state_msg = Float64MultiArray()
        state_msg.data = [self.current_pan, self.current_tilt]
        self.state_pub.publish(state_msg)

    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info('Shutting down pan-tilt node...')
        try:
            # Disable torque
            self._set_torque(self.pan_id, False)
            self._set_torque(self.tilt_id, False)
            # Close port
            self.port_handler.closePort()
        except Exception as e:
            self.get_logger().warn(f'Error during cleanup: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = PanTiltNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
