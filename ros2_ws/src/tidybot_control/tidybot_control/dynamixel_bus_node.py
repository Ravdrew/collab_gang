#!/usr/bin/env python3
"""
Unified Dynamixel Bus Manager for TidyBot2.

This node manages ALL Dynamixel motors on a shared U2D2 bus:
- Right arm: Motor IDs 1-6 (arm joints) + ID 7 (gripper)
- Left arm: Motor IDs 11-16 (arm joints) + ID 17 (gripper) [optional]
- Pan-tilt: Motor IDs 21-22

Having a single node manage the bus prevents port conflicts that occur
when multiple nodes try to access the same serial port.

Topics (same interface as separate nodes):
- /right_arm/joint_cmd (Float64MultiArray) - right arm joint positions
- /left_arm/joint_cmd (Float64MultiArray) - left arm joint positions
- /right_gripper/cmd (Float64MultiArray) - right gripper command
- /left_gripper/cmd (Float64MultiArray) - left gripper command
- /camera/pan_tilt_cmd (Float64MultiArray) - pan/tilt angles [pan, tilt]
- /joint_states (JointState) - all joint positions
- /camera/pan_tilt_state (Float64MultiArray) - pan/tilt state

Prerequisites:
- Dynamixel SDK: uv add dynamixel-sdk
- U2D2 adapter connected to USB
- Motors configured with correct IDs
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
        COMM_SUCCESS,
    )
    HAS_DYNAMIXEL = True
except ImportError:
    HAS_DYNAMIXEL = False


# Dynamixel X Series Control Table (EEPROM - requires torque off)
ADDR_DRIVE_MODE = 10
ADDR_OPERATING_MODE = 11
ADDR_SECONDARY_ID = 12
ADDR_HOMING_OFFSET = 20

# Dynamixel X Series Control Table (RAM)
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_MOVING = 122
ADDR_PRESENT_POSITION = 132

# Protocol version
PROTOCOL_VERSION = 2.0

# Default values
DEFAULT_BAUDRATE = 1000000
DEFAULT_PORT = '/dev/ttyUSB0'

# Position conversion: 4096 counts per revolution for X series
POSITION_UNIT = 2 * math.pi / 4096  # radians per count


class DynamixelBusNode(Node):
    """
    Unified Dynamixel bus manager for all motors on shared U2D2.

    WX250s has 9 motors per arm:
    - waist (ID 1), shoulder (ID 2), shoulder_shadow (ID 3),
    - elbow (ID 4), elbow_shadow (ID 5), forearm_roll (ID 6),
    - wrist_angle (ID 7), wrist_rotate (ID 8), gripper (ID 9)

    Shadow motors mirror their primary joints for extra torque.
    """

    # WX250s joint configuration (6 DOF - shadow motors mirror primary)
    ARM_JOINT_NAMES = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']

    # Motor ID mappings for WX250s (9 motors per arm)
    # Right arm: IDs 1-9
    RIGHT_ARM_IDS = [1, 2, 4, 6, 7, 8]  # Primary joint motors only (for reading)
    RIGHT_SHADOW_IDS = {2: 3, 4: 5}     # Primary ID -> Shadow ID mapping
    RIGHT_GRIPPER_ID = 9

    # Left arm: IDs 11-19 (requires remapping from default)
    LEFT_ARM_IDS = [11, 12, 14, 16, 17, 18]  # Primary joint motors only
    LEFT_SHADOW_IDS = {12: 13, 14: 15}       # Primary ID -> Shadow ID mapping
    LEFT_GRIPPER_ID = 19

    # Pan-tilt camera
    PAN_MOTOR_ID = 21
    TILT_MOTOR_ID = 22

    # Joint limits for WX250s (radians)
    JOINT_LIMITS = {
        'waist': (-3.14, 3.14),
        'shoulder': (-1.88, 1.99),
        'elbow': (-2.15, 1.61),
        'forearm_roll': (-3.14, 3.14),
        'wrist_angle': (-1.75, 2.15),
        'wrist_rotate': (-3.14, 3.14),
    }

    def __init__(self):
        super().__init__('dynamixel_bus')

        if not HAS_DYNAMIXEL:
            self.get_logger().error('Dynamixel SDK not installed! Run: uv add dynamixel-sdk')
            raise RuntimeError('Dynamixel SDK not available')

        # Parameters
        self.declare_parameter('port_name', DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('use_right_arm', True)
        self.declare_parameter('use_left_arm', False)
        self.declare_parameter('use_pan_tilt', True)
        self.declare_parameter('profile_velocity', 200)
        self.declare_parameter('profile_acceleration', 100)
        self.declare_parameter('debug_mode', False)  # Enable verbose logging

        port_name = self.get_parameter('port_name').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.use_right_arm = self.get_parameter('use_right_arm').get_parameter_value().bool_value
        self.use_left_arm = self.get_parameter('use_left_arm').get_parameter_value().bool_value
        self.use_pan_tilt = self.get_parameter('use_pan_tilt').get_parameter_value().bool_value
        profile_velocity = self.get_parameter('profile_velocity').get_parameter_value().integer_value
        profile_acceleration = self.get_parameter('profile_acceleration').get_parameter_value().integer_value
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value

        # Track when torque was last verified OK for debugging
        self._torque_last_ok = {}

        # Thread safety
        self.lock = threading.Lock()

        # State storage
        self.right_arm_positions = [0.0] * 6
        self.right_arm_targets = [0.0] * 6
        self.right_gripper_pos = 0.0
        self.right_gripper_target = 0.0

        self.left_arm_positions = [0.0] * 6
        self.left_arm_targets = [0.0] * 6
        self.left_gripper_pos = 0.0
        self.left_gripper_target = 0.0

        self.pan_position = 0.0
        self.tilt_position = 0.0
        self.pan_target = 0.0
        self.tilt_target = 0.0

        # Open serial port
        self.get_logger().info(f'Opening Dynamixel bus on {port_name} at {baudrate} baud...')
        self.port_handler = PortHandler(port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error(f'Failed to open port {port_name}')
            raise RuntimeError(f'Failed to open port {port_name}')

        if not self.port_handler.setBaudRate(baudrate):
            self.get_logger().error(f'Failed to set baudrate to {baudrate}')
            raise RuntimeError(f'Failed to set baudrate to {baudrate}')

        self.get_logger().info('Dynamixel bus opened successfully')

        # Discover and configure motors
        self.active_motors = {}  # motor_id -> name mapping
        self.shadow_offsets = {}  # shadow_id -> offset (shadow_pos - primary_pos)

        if self.use_right_arm:
            self._setup_arm('right', self.RIGHT_ARM_IDS, self.RIGHT_GRIPPER_ID,
                           self.RIGHT_SHADOW_IDS, profile_velocity, profile_acceleration)

        if self.use_left_arm:
            self._setup_arm('left', self.LEFT_ARM_IDS, self.LEFT_GRIPPER_ID,
                           self.LEFT_SHADOW_IDS, profile_velocity, profile_acceleration)

        if self.use_pan_tilt:
            self._setup_pan_tilt(profile_velocity, profile_acceleration)

        # Brief delay after configuration
        time.sleep(0.1)

        # Initialize torque tracking for all active motors
        init_time = time.time()
        for motor_id in self.active_motors:
            self._torque_last_ok[motor_id] = init_time

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        if self.use_pan_tilt:
            self.pan_tilt_state_pub = self.create_publisher(
                Float64MultiArray, '/camera/pan_tilt_state', 10
            )

        # Subscribers
        if self.use_right_arm:
            self.right_arm_sub = self.create_subscription(
                Float64MultiArray, '/right_arm/joint_cmd',
                lambda msg: self._arm_cmd_callback(msg, 'right'), 10
            )
            self.right_gripper_sub = self.create_subscription(
                Float64MultiArray, '/right_gripper/cmd',
                lambda msg: self._gripper_cmd_callback(msg, 'right'), 10
            )

        if self.use_left_arm:
            self.left_arm_sub = self.create_subscription(
                Float64MultiArray, '/left_arm/joint_cmd',
                lambda msg: self._arm_cmd_callback(msg, 'left'), 10
            )
            self.left_gripper_sub = self.create_subscription(
                Float64MultiArray, '/left_gripper/cmd',
                lambda msg: self._gripper_cmd_callback(msg, 'left'), 10
            )

        if self.use_pan_tilt:
            self.pan_tilt_sub = self.create_subscription(
                Float64MultiArray, '/camera/pan_tilt_cmd',
                self._pan_tilt_cmd_callback, 10
            )

        # Timer for read/write cycle
        self.create_timer(1.0 / publish_rate, self._timer_callback)

        self.get_logger().info('Dynamixel bus node ready')
        self.get_logger().info(f'  Active motors: {list(self.active_motors.keys())}')

    def _ping_motor(self, motor_id, name):
        """Ping a motor and return True if it responds."""
        model_num, result, error = self.packet_handler.ping(self.port_handler, motor_id)
        if result == COMM_SUCCESS:
            self.get_logger().info(f'  Motor ID {motor_id} ({name}) responded (model: {model_num})')
            return True
        else:
            self.get_logger().warn(f'  Motor ID {motor_id} ({name}) not found')
            return False

    def _configure_motor(self, motor_id, name, profile_velocity, profile_acceleration):
        """Configure a motor for position control."""
        # Disable torque first (required before changing operating mode)
        self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 0)
        time.sleep(0.02)

        # Set position control mode (mode 3)
        self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_OPERATING_MODE, 3)
        time.sleep(0.02)

        # Verify operating mode
        mode_val, result, error = self.packet_handler.read1ByteTxRx(self.port_handler, motor_id, ADDR_OPERATING_MODE)
        if mode_val != 3:
            self.get_logger().error(f'Failed to set position mode on motor {motor_id}! Mode={mode_val}')

        # Set profile velocity and acceleration
        self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, ADDR_PROFILE_VELOCITY, profile_velocity)
        time.sleep(0.01)
        self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, ADDR_PROFILE_ACCELERATION, profile_acceleration)
        time.sleep(0.01)

        # Verify profile velocity was set
        vel_val, result, error = self.packet_handler.read4ByteTxRx(self.port_handler, motor_id, ADDR_PROFILE_VELOCITY)
        if vel_val == 0:
            self.get_logger().warn(f'Profile velocity is 0 on motor {motor_id} - motor may not move!')

        # Enable torque - this is critical!
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 1)
        time.sleep(0.02)

        # Verify torque is enabled
        torque_val, result, error = self.packet_handler.read1ByteTxRx(self.port_handler, motor_id, ADDR_TORQUE_ENABLE)
        if torque_val != 1:
            self.get_logger().error(f'Failed to enable torque on motor {motor_id} ({name})! Torque={torque_val}')
        else:
            self.get_logger().info(f'  Torque enabled on motor {motor_id} ({name})')

        # Check for hardware errors
        hw_error, result, error = self.packet_handler.read1ByteTxRx(self.port_handler, motor_id, ADDR_HARDWARE_ERROR)
        if hw_error != 0:
            self.get_logger().error(f'  HARDWARE ERROR on motor {motor_id}: {hw_error} (clear by power cycling)')

        # Read and log current position and goal position
        curr_pos, _, _ = self.packet_handler.read4ByteTxRx(self.port_handler, motor_id, ADDR_PRESENT_POSITION)
        goal_pos, _, _ = self.packet_handler.read4ByteTxRx(self.port_handler, motor_id, ADDR_GOAL_POSITION)
        self.get_logger().info(f'  Motor {motor_id}: present={curr_pos} counts, goal={goal_pos} counts')

    def _setup_arm(self, arm_name, motor_ids, gripper_id, shadow_ids, profile_velocity, profile_acceleration):
        """Set up an arm (6 joints + 2 shadow motors + gripper = 9 motors).

        Following Interbotix SDK approach:
        1. First disable torque on ALL motors (primaries and shadows)
        2. Configure EEPROM settings (Drive_Mode, Secondary_ID, Homing_Offset) on shadows
        3. Configure operating mode and profiles on ALL motors
        4. Enable torque on ALL motors together
        """
        self.get_logger().info(f'Setting up {arm_name} arm (9 motors)...')

        # Step 1: Discover all motors and disable torque
        discovered_primaries = []
        discovered_shadows = []  # List of (shadow_id, primary_id, shadow_name)

        # Check primary motors
        for i, motor_id in enumerate(motor_ids):
            joint_name = f'{arm_name}_{self.ARM_JOINT_NAMES[i]}'
            if self._ping_motor(motor_id, joint_name):
                discovered_primaries.append((motor_id, joint_name))
                # Disable torque immediately
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 0)
                time.sleep(0.01)

        # Check shadow motors
        for primary_id, shadow_id in shadow_ids.items():
            if primary_id in motor_ids:
                idx = motor_ids.index(primary_id)
                shadow_name = f'{arm_name}_{self.ARM_JOINT_NAMES[idx]}_shadow'
                if self._ping_motor(shadow_id, shadow_name):
                    discovered_shadows.append((shadow_id, primary_id, shadow_name))
                    # Disable torque immediately
                    self.packet_handler.write1ByteTxRx(
                        self.port_handler, shadow_id, ADDR_TORQUE_ENABLE, 0)
                    time.sleep(0.01)

        # Step 2: Configure EEPROM settings on ALL motors (primaries and shadows)
        # Primaries need Drive_Mode = 0 (normal), no Secondary_ID
        for motor_id, joint_name in discovered_primaries:
            # Read current Drive_Mode
            drive_mode, _, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, motor_id, ADDR_DRIVE_MODE)

            # Ensure bit 0 is 0 (normal direction) while preserving other bits (profile type)
            if drive_mode is not None:
                new_drive_mode = drive_mode & 0xFE  # Clear bit 0
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, motor_id, ADDR_DRIVE_MODE, new_drive_mode)
                time.sleep(0.01)

            # Set Operating Mode = 3 (position control)
            self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, ADDR_OPERATING_MODE, 3)
            time.sleep(0.01)

            # Set profile parameters
            self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, ADDR_PROFILE_VELOCITY, profile_velocity)
            self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, ADDR_PROFILE_ACCELERATION, profile_acceleration)
            time.sleep(0.01)

            self.get_logger().info(f'  Primary {motor_id} ({joint_name}): Drive_Mode={drive_mode}&0xFE, OpMode=3')

        # Shadow motors: DISABLED for now (debugging) - just log their presence
        # Not using Secondary_ID approach as it seems to cause torque issues
        for shadow_id, primary_id, shadow_name in discovered_shadows:
            # Keep torque disabled on shadow motors (they'll be passive followers via gearbox)
            self.packet_handler.write1ByteTxRx(
                self.port_handler, shadow_id, ADDR_TORQUE_ENABLE, 0)
            time.sleep(0.01)

            # Read current positions for logging
            primary_pos, _, _ = self.packet_handler.read4ByteTxRx(
                self.port_handler, primary_id, ADDR_PRESENT_POSITION)
            shadow_pos, _, _ = self.packet_handler.read4ByteTxRx(
                self.port_handler, shadow_id, ADDR_PRESENT_POSITION)

            self.get_logger().info(
                f'  Shadow {shadow_id} ({shadow_name}): TORQUE DISABLED (passive follower) '
                f'(primary_pos={primary_pos}, shadow_pos={shadow_pos})')

        # Step 3: Enable torque on primaries
        # CRITICAL: Set goal position to current position BEFORE enabling torque
        # to prevent sudden movements
        for motor_id, joint_name in discovered_primaries:
            # Read current position
            curr_pos, _, _ = self.packet_handler.read4ByteTxRx(
                self.port_handler, motor_id, ADDR_PRESENT_POSITION)
            time.sleep(0.005)

            # Set goal to current position BEFORE enabling torque
            self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, ADDR_GOAL_POSITION, curr_pos)
            time.sleep(0.01)

            # Now enable torque
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 1)
            time.sleep(0.02)

            # Verify torque enabled
            torque_val, _, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, motor_id, ADDR_TORQUE_ENABLE)
            if torque_val != 1:
                self.get_logger().error(f'  FAILED to enable torque on primary {motor_id}!')
            else:
                self.get_logger().info(f'  Torque enabled on primary {motor_id} ({joint_name})')

            # Check hardware error
            hw_err, _, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, motor_id, ADDR_HARDWARE_ERROR)
            if hw_err and hw_err != 0:
                self.get_logger().error(f'  HARDWARE ERROR on {motor_id}: 0x{hw_err:02X}')

            # Log position
            goal_pos, _, _ = self.packet_handler.read4ByteTxRx(
                self.port_handler, motor_id, ADDR_GOAL_POSITION)
            self.get_logger().info(f'  Motor {motor_id}: pos={curr_pos}, goal={goal_pos}')

            self.active_motors[motor_id] = joint_name

        # Shadow motors remain with torque disabled
        for shadow_id, primary_id, shadow_name in discovered_shadows:
            self.get_logger().info(f'  Shadow {shadow_id} ({shadow_name}): keeping torque disabled')
            # Don't add shadows to active_motors since they're passive

        # Gripper
        gripper_name = f'{arm_name}_gripper'
        if self._ping_motor(gripper_id, gripper_name):
            self._configure_motor(gripper_id, gripper_name, profile_velocity, profile_acceleration)
            self.active_motors[gripper_id] = gripper_name

    def _setup_pan_tilt(self, profile_velocity, profile_acceleration):
        """Set up pan-tilt motors."""
        self.get_logger().info('Setting up pan-tilt...')

        if self._ping_motor(self.PAN_MOTOR_ID, 'pan'):
            self._configure_motor(self.PAN_MOTOR_ID, 'pan', profile_velocity, profile_acceleration)
            self.active_motors[self.PAN_MOTOR_ID] = 'pan'

        if self._ping_motor(self.TILT_MOTOR_ID, 'tilt'):
            self._configure_motor(self.TILT_MOTOR_ID, 'tilt', profile_velocity, profile_acceleration)
            self.active_motors[self.TILT_MOTOR_ID] = 'tilt'

    def _radians_to_counts(self, radians):
        """Convert radians to Dynamixel counts (centered at 2048)."""
        counts = int(radians / POSITION_UNIT) + 2048
        return max(0, min(4095, counts))

    def _counts_to_radians(self, counts):
        """Convert Dynamixel counts to radians."""
        return (counts - 2048) * POSITION_UNIT

    def _read_motor(self, motor_id):
        """Read position from a single motor."""
        counts, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, motor_id, ADDR_PRESENT_POSITION
        )
        time.sleep(0.001)  # Small delay to prevent bus congestion
        if result == COMM_SUCCESS:
            return self._counts_to_radians(counts)
        return None

    def _write_motor(self, motor_id, radians):
        """Write position to a single motor with verification."""
        counts = self._radians_to_counts(radians)

        # Debug: log first non-zero write for each motor
        if not hasattr(self, '_write_logged'):
            self._write_logged = set()
        if motor_id not in self._write_logged and abs(radians) > 0.01:
            self._write_logged.add(motor_id)
            self.get_logger().info(f'Writing to motor {motor_id}: {radians:.3f} rad = {counts} counts')

        # Use TxRx to get confirmation the write was received
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, ADDR_GOAL_POSITION, counts
        )
        time.sleep(0.002)  # Small delay to prevent bus congestion

        if result != COMM_SUCCESS:
            self.get_logger().warn(f'Motor {motor_id} write failed: {self.packet_handler.getTxRxResult(result)}')
        elif error != 0:
            self.get_logger().warn(f'Motor {motor_id} error: {self.packet_handler.getRxPacketError(error)}')

    def _arm_cmd_callback(self, msg, arm_name):
        """Handle arm joint commands - write to primary motors only.

        Shadow motors automatically follow via Secondary_ID feature.
        """
        if len(msg.data) != 6:
            self.get_logger().warn(f'Expected 6 joint values for {arm_name} arm, got {len(msg.data)}')
            return

        motor_ids = self.RIGHT_ARM_IDS if arm_name == 'right' else self.LEFT_ARM_IDS

        with self.lock:
            targets = self.right_arm_targets if arm_name == 'right' else self.left_arm_targets
            for i, (joint_name, pos) in enumerate(zip(self.ARM_JOINT_NAMES, msg.data)):
                limits = self.JOINT_LIMITS[joint_name]
                targets[i] = max(limits[0], min(limits[1], pos))

            # Write to primary motors only
            # Shadow motors follow automatically via Secondary_ID
            for i, motor_id in enumerate(motor_ids):
                if motor_id in self.active_motors:
                    self._write_motor(motor_id, targets[i])

        # Debug: log command changes
        if not hasattr(self, '_last_cmd'):
            self._last_cmd = {}
        cmd_tuple = tuple(msg.data)
        if self._last_cmd.get(arm_name) != cmd_tuple:
            self._last_cmd[arm_name] = cmd_tuple
            self.get_logger().info(f'Received {arm_name} arm command: {list(msg.data)}')

    def _gripper_cmd_callback(self, msg, arm_name):
        """Handle gripper commands (0-1 normalized: 0.0 = open, 1.0 = closed)."""
        if len(msg.data) < 1:
            return

        gripper_val = max(0.0, min(1.0, msg.data[0]))
        with self.lock:
            if arm_name == 'right':
                self.right_gripper_target = gripper_val * 0.037  # 0=open(0rad), 1=closed(0.037rad)
                if self.RIGHT_GRIPPER_ID in self.active_motors:
                    self._write_motor(self.RIGHT_GRIPPER_ID, self.right_gripper_target)
            else:
                self.left_gripper_target = gripper_val * 0.037  # 0=open(0rad), 1=closed(0.037rad)
                if self.LEFT_GRIPPER_ID in self.active_motors:
                    self._write_motor(self.LEFT_GRIPPER_ID, self.left_gripper_target)

    def _pan_tilt_cmd_callback(self, msg):
        """Handle pan-tilt commands [pan, tilt] in radians."""
        if len(msg.data) < 2:
            return

        with self.lock:
            self.pan_target = max(-1.5, min(1.5, msg.data[0]))
            self.tilt_target = max(-0.5, min(0.5, msg.data[1]))

            # Write directly to motors NOW
            if self.PAN_MOTOR_ID in self.active_motors:
                self._write_motor(self.PAN_MOTOR_ID, self.pan_target)
            if self.TILT_MOTOR_ID in self.active_motors:
                self._write_motor(self.TILT_MOTOR_ID, self.tilt_target)

        self.get_logger().info(f'Pan-tilt command: pan={self.pan_target:.3f}, tilt={self.tilt_target:.3f}')

    def _check_motor_health(self, motor_id, name, auto_recover=True):
        """Check if a motor has hardware errors or lost torque.

        If auto_recover is True and torque is lost without hardware error,
        attempt to re-enable torque.
        """
        now = time.time()

        # Check if torque is still enabled
        torque_val, result, _ = self.packet_handler.read1ByteTxRx(
            self.port_handler, motor_id, ADDR_TORQUE_ENABLE)

        if torque_val == 1:
            # Torque is OK - update last OK time
            self._torque_last_ok[motor_id] = now
            return True

        # Torque is off - calculate how long since it was last OK
        last_ok = self._torque_last_ok.get(motor_id, now)
        time_since_ok = now - last_ok

        # Check WHY torque is off by reading hardware error
        hw_err, _, _ = self.packet_handler.read1ByteTxRx(
            self.port_handler, motor_id, ADDR_HARDWARE_ERROR)

        error_bits = []
        if hw_err:
            if hw_err & 0x01: error_bits.append("InputVoltage")
            if hw_err & 0x04: error_bits.append("Overheating")
            if hw_err & 0x08: error_bits.append("MotorEncoder")
            if hw_err & 0x10: error_bits.append("ElecShock")
            if hw_err & 0x20: error_bits.append("Overload")

        if error_bits:
            self.get_logger().error(
                f'Motor {motor_id} ({name}) HARDWARE ERROR: {", ".join(error_bits)} (0x{hw_err:02X})')
            self.get_logger().error(f'  -> Time since torque was OK: {time_since_ok:.2f}s')
            self.get_logger().error(f'  -> Power cycle required to clear hardware error!')
            return False
        else:
            self.get_logger().warn(
                f'Motor {motor_id} ({name}) lost torque (no hw error, time_since_ok={time_since_ok:.2f}s)')

            # Also read and log the current position and goal for debugging
            curr_pos, _, _ = self.packet_handler.read4ByteTxRx(
                self.port_handler, motor_id, ADDR_PRESENT_POSITION)
            goal_pos, _, _ = self.packet_handler.read4ByteTxRx(
                self.port_handler, motor_id, ADDR_GOAL_POSITION)
            self.get_logger().info(f'  -> current_pos={curr_pos}, goal_pos={goal_pos}, diff={abs(goal_pos - curr_pos) if goal_pos and curr_pos else "N/A"}')

            if auto_recover:
                # Attempt to re-enable torque
                self.get_logger().info(f'  -> Attempting to re-enable torque on motor {motor_id}...')

                # Set goal to current position to prevent jump
                if curr_pos is not None:
                    self.packet_handler.write4ByteTxRx(
                        self.port_handler, motor_id, ADDR_GOAL_POSITION, curr_pos)
                    time.sleep(0.01)

                # Re-enable torque
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 1)
                time.sleep(0.02)

                # Verify
                new_torque, _, _ = self.packet_handler.read1ByteTxRx(
                    self.port_handler, motor_id, ADDR_TORQUE_ENABLE)
                if new_torque == 1:
                    self.get_logger().info(f'  -> Torque re-enabled successfully on motor {motor_id}')
                    self._torque_last_ok[motor_id] = time.time()
                    return True
                else:
                    self.get_logger().error(f'  -> Failed to re-enable torque on motor {motor_id}!')
                    return False

            return False

    def _timer_callback(self):
        """Read motor positions and publish joint states."""
        # Periodic health check - more frequent during first 10 seconds, then every ~1 second
        if not hasattr(self, '_health_check_counter'):
            self._health_check_counter = 0
            self._startup_time = time.time()
        self._health_check_counter += 1

        # More aggressive health checks during startup (every 0.5s for first 30s, then every 1s)
        elapsed = time.time() - self._startup_time
        if elapsed < 30:
            health_check_interval = 25  # Every 0.5s at 50Hz
        else:
            health_check_interval = 50  # Every 1s at 50Hz

        do_health_check = (self._health_check_counter % health_check_interval) == 0
        if do_health_check:
            with self.lock:
                # Check health of primary arm motors with auto-recovery
                if self.use_right_arm:
                    for motor_id in self.RIGHT_ARM_IDS:
                        if motor_id in self.active_motors:
                            self._check_motor_health(motor_id, self.active_motors[motor_id], auto_recover=True)

        with self.lock:
            # Read arm positions
            if self.use_right_arm:
                for i, motor_id in enumerate(self.RIGHT_ARM_IDS):
                    if motor_id in self.active_motors:
                        pos = self._read_motor(motor_id)
                        if pos is not None:
                            self.right_arm_positions[i] = pos

                if self.RIGHT_GRIPPER_ID in self.active_motors:
                    pos = self._read_motor(self.RIGHT_GRIPPER_ID)
                    if pos is not None:
                        self.right_gripper_pos = pos

            if self.use_left_arm:
                for i, motor_id in enumerate(self.LEFT_ARM_IDS):
                    if motor_id in self.active_motors:
                        pos = self._read_motor(motor_id)
                        if pos is not None:
                            self.left_arm_positions[i] = pos

                if self.LEFT_GRIPPER_ID in self.active_motors:
                    pos = self._read_motor(self.LEFT_GRIPPER_ID)
                    if pos is not None:
                        self.left_gripper_pos = pos

            if self.use_pan_tilt:
                if self.PAN_MOTOR_ID in self.active_motors:
                    pos = self._read_motor(self.PAN_MOTOR_ID)
                    if pos is not None:
                        self.pan_position = pos

                if self.TILT_MOTOR_ID in self.active_motors:
                    pos = self._read_motor(self.TILT_MOTOR_ID)
                    if pos is not None:
                        self.tilt_position = pos

        # Publish joint states
        self._publish_joint_states()

        if self.use_pan_tilt:
            self._publish_pan_tilt_state()

    def _publish_joint_states(self):
        """Publish joint states for all active motors."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        with self.lock:
            if self.use_right_arm:
                for i, joint_name in enumerate(self.ARM_JOINT_NAMES):
                    msg.name.append(f'right_{joint_name}')
                    msg.position.append(self.right_arm_positions[i])
                    msg.velocity.append(0.0)
                    msg.effort.append(0.0)

                # Gripper fingers - convert motor radians (0=open, 0.037=closed)
                # to URDF slide joint meters (0.022=open, -0.014=closed)
                finger_pos = 0.022 - (self.right_gripper_pos / 0.037) * 0.036
                msg.name.append('right_left_finger')
                msg.position.append(finger_pos)
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
                msg.name.append('right_right_finger')
                msg.position.append(finger_pos)
                msg.velocity.append(0.0)
                msg.effort.append(0.0)

            if self.use_left_arm:
                for i, joint_name in enumerate(self.ARM_JOINT_NAMES):
                    msg.name.append(f'left_{joint_name}')
                    msg.position.append(self.left_arm_positions[i])
                    msg.velocity.append(0.0)
                    msg.effort.append(0.0)

                # Convert motor radians to URDF slide joint meters (0.022=open, -0.014=closed)
                finger_pos = 0.022 - (self.left_gripper_pos / 0.037) * 0.036
                msg.name.append('left_left_finger')
                msg.position.append(finger_pos)
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
                msg.name.append('left_right_finger')
                msg.position.append(finger_pos)
                msg.velocity.append(0.0)
                msg.effort.append(0.0)

            if self.use_pan_tilt:
                msg.name.append('pan')
                msg.position.append(self.pan_position)
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
                msg.name.append('tilt')
                msg.position.append(self.tilt_position)
                msg.velocity.append(0.0)
                msg.effort.append(0.0)

        self.joint_state_pub.publish(msg)

    def _publish_pan_tilt_state(self):
        """Publish pan-tilt state."""
        msg = Float64MultiArray()
        with self.lock:
            msg.data = [self.pan_position, self.tilt_position]
        self.pan_tilt_state_pub.publish(msg)

    def destroy_node(self):
        """Clean up."""
        self.get_logger().info('Shutting down Dynamixel bus...')
        try:
            # Disable torque on all motors
            for motor_id in self.active_motors:
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 0
                )
            self.port_handler.closePort()
        except Exception as e:
            self.get_logger().warn(f'Error during cleanup: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelBusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
