#!/usr/bin/env python3
"""
Interbotix WX250s Hardware Interface Node (Direct Dynamixel SDK).

This node provides the SAME topic interface as the MuJoCo bridge,
but sends commands to real hardware via the Dynamixel SDK directly.

This is a self-contained implementation that does NOT require the
Interbotix xs_sdk or InterbotixManipulatorXS class.

Swap between sim and real by changing which node is launched:
- Simulation: mujoco_bridge_node (from tidybot_mujoco_bridge)
- Real hardware: interbotix_arm_node (this file)

Topics are IDENTICAL - student code doesn't change.

Motor Configuration (WX250s with default IDs):
- Right arm: IDs 1-6 (arm joints), ID 7 (gripper)
- Left arm (on shared bus): IDs 11-16 (arm joints), ID 17 (gripper)

Prerequisites:
- Dynamixel SDK: pip install dynamixel-sdk (or uv add dynamixel-sdk)
- U2D2 adapter connected to USB
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
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


# Dynamixel X Series Control Table (XM430, XL430, etc.)
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128
ADDR_OPERATING_MODE = 11
ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCELERATION = 108
ADDR_MOVING = 122
ADDR_GOAL_PWM = 100

# Protocol version
PROTOCOL_VERSION = 2.0

# Default values
DEFAULT_BAUDRATE = 1000000
DEFAULT_PORT = '/dev/ttyUSB0'

# Position conversion: 4096 counts per revolution for X series
POSITION_UNIT = 2 * math.pi / 4096  # radians per count
VELOCITY_UNIT = 0.229 * 2 * math.pi / 60  # radians per second (0.229 rpm per unit)


class InterbotixArmNode(Node):
    """
    Real hardware interface for Interbotix WX250s arm (6-DOF) using Dynamixel SDK.

    Provides the SAME interface as MuJoCo bridge:
    - Subscribes to: /right_arm/joint_cmd, /left_arm/joint_cmd, etc.
    - Publishes to: /joint_states

    This allows student code to work identically on sim and real.
    """

    # WX250s has 6 DOF
    JOINT_NAMES = [
        'waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate'
    ]

    # Default motor IDs for right arm (1-6 arm, 7 gripper)
    # For left arm on shared bus: use 11-16 arm, 17 gripper
    DEFAULT_RIGHT_IDS = [1, 2, 3, 4, 5, 6]
    DEFAULT_LEFT_IDS = [11, 12, 13, 14, 15, 16]
    DEFAULT_RIGHT_GRIPPER_ID = 7
    DEFAULT_LEFT_GRIPPER_ID = 17

    # Joint limits for WX250s (in radians)
    JOINT_LIMITS = {
        'waist': (-3.14, 3.14),
        'shoulder': (-1.88, 1.99),
        'elbow': (-2.15, 1.61),
        'forearm_roll': (-3.14, 3.14),
        'wrist_angle': (-1.75, 2.15),
        'wrist_rotate': (-3.14, 3.14),
    }

    # Home position (centered, safe)
    HOME_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def __init__(self):
        super().__init__('interbotix_arm')

        # Check for Dynamixel SDK
        if not HAS_DYNAMIXEL:
            self.get_logger().error(
                'Dynamixel SDK not installed! Install with:\n'
                '  uv add dynamixel-sdk\n'
                '  or: pip install dynamixel-sdk'
            )
            raise RuntimeError('Dynamixel SDK not available')

        # Parameters
        self.declare_parameter('arm_name', 'right')
        self.declare_parameter('robot_model', 'wx250s')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('port_name', DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('profile_velocity', 200)  # Dynamixel units
        self.declare_parameter('profile_acceleration', 100)  # Dynamixel units

        self.arm_name = self.get_parameter('arm_name').get_parameter_value().string_value
        self.robot_model = self.get_parameter('robot_model').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        port_name = self.get_parameter('port_name').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        profile_velocity = self.get_parameter('profile_velocity').get_parameter_value().integer_value
        profile_acceleration = self.get_parameter('profile_acceleration').get_parameter_value().integer_value

        self.num_joints = 6  # WX250s always has 6 joints
        self.joint_names = self.JOINT_NAMES

        # Set motor IDs based on arm name
        if self.arm_name == 'left':
            self.motor_ids = self.DEFAULT_LEFT_IDS
            self.gripper_id = self.DEFAULT_LEFT_GRIPPER_ID
        else:
            self.motor_ids = self.DEFAULT_RIGHT_IDS
            self.gripper_id = self.DEFAULT_RIGHT_GRIPPER_ID

        # Full joint names with arm prefix (matches simulation)
        self.full_joint_names = [f'{self.arm_name}_{j}' for j in self.joint_names]
        self.full_joint_names += [f'{self.arm_name}_left_finger', f'{self.arm_name}_right_finger']

        # Thread safety
        self.lock = threading.Lock()

        # Current state
        self.current_positions = [0.0] * self.num_joints
        self.current_velocities = [0.0] * self.num_joints
        self.target_positions = list(self.HOME_POSITION)
        self.gripper_position = 0.0  # 0 = open, 1 = closed

        # Initialize Dynamixel communication
        self.get_logger().info(f'Connecting to {self.robot_model} arm "{self.arm_name}" ({self.num_joints} DOF)...')
        self.get_logger().info(f'  Port: {port_name}, Baudrate: {baudrate}')
        self.get_logger().info(f'  Motor IDs: {self.motor_ids}, Gripper ID: {self.gripper_id}')

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

        self.get_logger().info(f'Connected to Dynamixel bus on {port_name}')

        # Ping all motors first to verify communication
        all_motors = self.motor_ids + [self.gripper_id]
        for motor_id in all_motors:
            model_num, result, error = self.packet_handler.ping(self.port_handler, motor_id)
            if result != COMM_SUCCESS:
                self.get_logger().error(f'Cannot ping motor ID {motor_id} - check connections!')
            else:
                self.get_logger().info(f'Motor ID {motor_id} responded (model: {model_num})')

        # Configure motors
        for i, motor_id in enumerate(self.motor_ids):
            self._configure_motor(motor_id, self.joint_names[i], profile_velocity, profile_acceleration)
            self._set_torque(motor_id, True)

        # Configure gripper
        self._configure_motor(self.gripper_id, 'gripper', profile_velocity, profile_acceleration)
        self._set_torque(self.gripper_id, True)

        # Brief delay to let motors stabilize after torque enable
        time.sleep(0.1)

        # Use individual reads instead of sync read for better reliability
        # Sync read can be finicky with some USB-serial adapters
        self.use_sync_read = False  # Set to True to try sync read

        if self.use_sync_read:
            # Initialize sync read for all arm positions
            self.sync_read = GroupSyncRead(
                self.port_handler, self.packet_handler,
                ADDR_PRESENT_POSITION, 4
            )
            for motor_id in self.motor_ids:
                self.sync_read.addParam(motor_id)
            self.sync_read.addParam(self.gripper_id)

        # Initialize sync write for all arm positions (sync write is more reliable)
        self.sync_write = GroupSyncWrite(
            self.port_handler, self.packet_handler,
            ADDR_GOAL_POSITION, 4
        )

        # Read initial positions
        self._read_positions()

        # Publishers - SAME as MuJoCo bridge
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribers - SAME as MuJoCo bridge
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            f'/{self.arm_name}_arm/joint_cmd',
            self.joint_cmd_callback,
            10
        )
        self.gripper_cmd_sub = self.create_subscription(
            Float64MultiArray,
            f'/{self.arm_name}_gripper/cmd',
            self.gripper_cmd_callback,
            10
        )

        # Timer to publish joint states and send commands
        self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(f'{self.arm_name} arm hardware interface ready')

    def _configure_motor(self, motor_id, name, profile_velocity, profile_acceleration):
        """Configure a Dynamixel motor for position control."""
        # Disable torque before configuring
        self._set_torque(motor_id, False)

        # Set operating mode to Position Control (value 3)
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ADDR_OPERATING_MODE, 3
        )
        if result != COMM_SUCCESS:
            self.get_logger().warn(
                f'Failed to set operating mode for {name} (ID {motor_id}): '
                f'{self.packet_handler.getTxRxResult(result)}'
            )
        elif error != 0:
            self.get_logger().warn(
                f'Error setting operating mode for {name} (ID {motor_id}): '
                f'{self.packet_handler.getRxPacketError(error)}'
            )

        # Set profile velocity
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, ADDR_PROFILE_VELOCITY, profile_velocity
        )
        if result != COMM_SUCCESS:
            self.get_logger().warn(
                f'Failed to set profile velocity for {name} (ID {motor_id})'
            )

        # Set profile acceleration
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, ADDR_PROFILE_ACCELERATION, profile_acceleration
        )
        if result != COMM_SUCCESS:
            self.get_logger().warn(
                f'Failed to set profile acceleration for {name} (ID {motor_id})'
            )

        self.get_logger().info(f'Configured {name} motor (ID {motor_id})')

    def _set_torque(self, motor_id, enable):
        """Enable or disable torque for a motor."""
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 1 if enable else 0
        )
        if result != COMM_SUCCESS:
            self.get_logger().warn(
                f'Failed to {"enable" if enable else "disable"} torque for motor {motor_id}'
            )

    def _radians_to_counts(self, radians):
        """Convert radians to Dynamixel position counts (centered at 2048)."""
        counts = int(radians / POSITION_UNIT) + 2048
        return max(0, min(4095, counts))

    def _counts_to_radians(self, counts):
        """Convert Dynamixel position counts to radians (centered at 2048)."""
        return (counts - 2048) * POSITION_UNIT

    def _read_positions(self):
        """Read current positions from all motors."""
        with self.lock:
            if self.use_sync_read:
                # Use sync read (faster but can be unreliable)
                result = self.sync_read.txRxPacket()
                if result == COMM_SUCCESS:
                    for i, motor_id in enumerate(self.motor_ids):
                        if self.sync_read.isAvailable(motor_id, ADDR_PRESENT_POSITION, 4):
                            counts = self.sync_read.getData(motor_id, ADDR_PRESENT_POSITION, 4)
                            self.current_positions[i] = self._counts_to_radians(counts)

                    # Read gripper position
                    if self.sync_read.isAvailable(self.gripper_id, ADDR_PRESENT_POSITION, 4):
                        counts = self.sync_read.getData(self.gripper_id, ADDR_PRESENT_POSITION, 4)
                        gripper_rad = self._counts_to_radians(counts)
                        self.gripper_position = max(0.0, min(1.0, gripper_rad / 0.037))
                else:
                    # Log which motors didn't respond
                    missing = []
                    for motor_id in self.motor_ids + [self.gripper_id]:
                        if not self.sync_read.isAvailable(motor_id, ADDR_PRESENT_POSITION, 4):
                            missing.append(motor_id)
                    if missing:
                        self.get_logger().warn(f'Sync read failed for motors: {missing}')
                    else:
                        self.get_logger().warn(
                            f'Sync read failed: {self.packet_handler.getTxRxResult(result)}'
                        )
            else:
                # Use individual reads (slower but more reliable)
                for i, motor_id in enumerate(self.motor_ids):
                    counts, result, error = self.packet_handler.read4ByteTxRx(
                        self.port_handler, motor_id, ADDR_PRESENT_POSITION
                    )
                    if result == COMM_SUCCESS:
                        self.current_positions[i] = self._counts_to_radians(counts)

                # Read gripper
                counts, result, error = self.packet_handler.read4ByteTxRx(
                    self.port_handler, self.gripper_id, ADDR_PRESENT_POSITION
                )
                if result == COMM_SUCCESS:
                    gripper_rad = self._counts_to_radians(counts)
                    self.gripper_position = max(0.0, min(1.0, gripper_rad / 0.037))

    def _write_positions(self):
        """Write target positions to all motors using individual writes."""
        # Use individual writes for reliability (sync write can be unreliable)
        for i, motor_id in enumerate(self.motor_ids):
            counts = self._radians_to_counts(self.target_positions[i])
            result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, ADDR_GOAL_POSITION, counts
            )
            if result != COMM_SUCCESS:
                self.get_logger().warn(f'Failed to write position to motor {motor_id}')

    def joint_cmd_callback(self, msg: Float64MultiArray):
        """
        Handle joint commands - send to real hardware.

        Same interface as MuJoCo bridge subscribes to.
        """
        if len(msg.data) != self.num_joints:
            self.get_logger().warn(f'Expected {self.num_joints} joint values, got {len(msg.data)}')
            return

        with self.lock:
            # Clip to joint limits
            for i, (name, pos) in enumerate(zip(self.joint_names, msg.data)):
                limits = self.JOINT_LIMITS[name]
                self.target_positions[i] = max(limits[0], min(limits[1], pos))

        # Log first command received (debug)
        if not hasattr(self, '_first_cmd_logged'):
            self._first_cmd_logged = True
            self.get_logger().info(f'Received arm command: {list(msg.data)}')

    def gripper_cmd_callback(self, msg: Float64MultiArray):
        """Handle gripper commands (normalized 0-1: 0.0 = open, 1.0 = closed)."""
        if len(msg.data) < 1:
            return

        # Convert normalized position to gripper angle
        gripper_target = max(0.0, min(1.0, msg.data[0]))

        # WX250s gripper motor: 0 rad = open, ~0.037 rad = closed
        gripper_rad = gripper_target * 0.037
        counts = self._radians_to_counts(gripper_rad)

        # Write directly to gripper (not in sync group)
        data = [
            DXL_LOBYTE(DXL_LOWORD(counts)),
            DXL_HIBYTE(DXL_LOWORD(counts)),
            DXL_LOBYTE(DXL_HIWORD(counts)),
            DXL_HIBYTE(DXL_HIWORD(counts)),
        ]

        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.gripper_id, ADDR_GOAL_POSITION, counts
        )
        if result != COMM_SUCCESS:
            self.get_logger().warn(f'Failed to send gripper command')

    def timer_callback(self):
        """Read state and send commands at fixed rate."""
        # Read current positions
        self._read_positions()

        # Write target positions
        with self.lock:
            self._write_positions()

        # Publish joint states
        self.publish_joint_states()

    def publish_joint_states(self):
        """
        Publish joint states from real hardware.

        Same format as MuJoCo bridge publishes.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        with self.lock:
            # Arm joints
            for i, name in enumerate(self.full_joint_names[:self.num_joints]):
                msg.name.append(name)
                msg.position.append(self.current_positions[i])
                msg.velocity.append(self.current_velocities[i])
                msg.effort.append(0.0)

            # Gripper joints - map normalized [0=open, 1=closed] to URDF slide joint [0.022=open, -0.014=closed]
            finger_pos = 0.022 - self.gripper_position * 0.036
            msg.name.append(f'{self.arm_name}_left_finger')
            msg.position.append(finger_pos)
            msg.velocity.append(0.0)
            msg.effort.append(0.0)

            msg.name.append(f'{self.arm_name}_right_finger')
            msg.position.append(finger_pos)
            msg.velocity.append(0.0)
            msg.effort.append(0.0)

        self.joint_state_pub.publish(msg)

    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info(f'Shutting down {self.arm_name} arm node...')
        try:
            # Disable torque on all motors
            for motor_id in self.motor_ids:
                self._set_torque(motor_id, False)
            self._set_torque(self.gripper_id, False)
            # Close port
            self.port_handler.closePort()
        except Exception as e:
            self.get_logger().warn(f'Error during cleanup: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = InterbotixArmNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
