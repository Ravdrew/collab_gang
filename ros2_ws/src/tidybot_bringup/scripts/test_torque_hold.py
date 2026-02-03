#!/usr/bin/env python3
"""
Test script to monitor if arm motors hold torque.

This script connects directly to the Dynamixel bus (without going through ROS2)
to test if the motors can hold torque for an extended period.

Usage:
    cd ~/collaborative-robotics-2026/ros2_ws
    source .venv/bin/activate  # or: source ~/collaborative-robotics-2026/.venv/bin/activate
    python3 src/tidybot_bringup/scripts/test_torque_hold.py

This will:
1. Connect to the U2D2 on /dev/ttyUSB0
2. Ping all expected motors
3. Enable torque on all motors (setting goal = current position)
4. Monitor torque status every 0.5 seconds
5. Report if/when torque is lost
"""

import sys
import time

try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    print("ERROR: Dynamixel SDK not found!")
    print("Install with: pip install dynamixel-sdk")
    print("Or activate the project venv: source ~/collaborative-robotics-2026/.venv/bin/activate")
    sys.exit(1)

# Dynamixel X Series Control Table
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCELERATION = 108
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

# Protocol version
PROTOCOL_VERSION = 2.0

# Default settings
PORT = '/dev/ttyUSB0'
BAUDRATE = 1000000

# WX250s motor IDs (right arm only for this test)
# Primary motors only - shadows will be passive
RIGHT_ARM_PRIMARY_IDS = [1, 2, 4, 6, 7, 8]
RIGHT_ARM_SHADOW_IDS = [3, 5]  # Shadows for shoulder and elbow
RIGHT_GRIPPER_ID = 9
PAN_TILT_IDS = [21, 22]

MOTOR_NAMES = {
    1: 'waist',
    2: 'shoulder',
    3: 'shoulder_shadow',
    4: 'elbow',
    5: 'elbow_shadow',
    6: 'forearm_roll',
    7: 'wrist_angle',
    8: 'wrist_rotate',
    9: 'gripper',
    21: 'pan',
    22: 'tilt',
}


def main():
    print("=" * 60)
    print("Dynamixel Torque Hold Test")
    print("=" * 60)
    print(f"Port: {PORT}, Baudrate: {BAUDRATE}")
    print()

    # Initialize port and packet handler
    port_handler = PortHandler(PORT)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort():
        print(f"ERROR: Failed to open port {PORT}")
        return 1

    if not port_handler.setBaudRate(BAUDRATE):
        print(f"ERROR: Failed to set baudrate to {BAUDRATE}")
        return 1

    print(f"Port opened successfully")
    print()

    # Discover motors
    print("Discovering motors...")
    discovered_motors = []
    all_ids = RIGHT_ARM_PRIMARY_IDS + RIGHT_ARM_SHADOW_IDS + [RIGHT_GRIPPER_ID] + PAN_TILT_IDS

    for motor_id in all_ids:
        model_num, result, error = packet_handler.ping(port_handler, motor_id)
        if result == COMM_SUCCESS:
            name = MOTOR_NAMES.get(motor_id, f'motor_{motor_id}')
            print(f"  Found motor {motor_id} ({name}), model: {model_num}")
            discovered_motors.append(motor_id)
        time.sleep(0.01)

    print(f"\nDiscovered {len(discovered_motors)} motors")
    print()

    if not discovered_motors:
        print("ERROR: No motors found!")
        port_handler.closePort()
        return 1

    # Enable torque on primary motors only
    print("Configuring and enabling torque on primary motors...")
    primary_motors = [m for m in discovered_motors if m not in RIGHT_ARM_SHADOW_IDS]
    shadow_motors = [m for m in discovered_motors if m in RIGHT_ARM_SHADOW_IDS]

    for motor_id in primary_motors:
        name = MOTOR_NAMES.get(motor_id, f'motor_{motor_id}')

        # Disable torque first
        packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE, 0)
        time.sleep(0.02)

        # Set position control mode
        packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_OPERATING_MODE, 3)
        time.sleep(0.01)

        # Set profile velocity and acceleration
        packet_handler.write4ByteTxRx(port_handler, motor_id, ADDR_PROFILE_VELOCITY, 200)
        packet_handler.write4ByteTxRx(port_handler, motor_id, ADDR_PROFILE_ACCELERATION, 100)
        time.sleep(0.01)

        # Read current position
        curr_pos, result, error = packet_handler.read4ByteTxRx(port_handler, motor_id, ADDR_PRESENT_POSITION)
        time.sleep(0.01)

        # Set goal to current position
        packet_handler.write4ByteTxRx(port_handler, motor_id, ADDR_GOAL_POSITION, curr_pos)
        time.sleep(0.01)

        # Enable torque
        packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE, 1)
        time.sleep(0.02)

        # Verify
        torque_val, result, error = packet_handler.read1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE)
        if torque_val == 1:
            print(f"  {motor_id} ({name}): Torque ENABLED, pos={curr_pos}")
        else:
            print(f"  {motor_id} ({name}): FAILED to enable torque!")

    # Keep shadow motors with torque disabled
    for motor_id in shadow_motors:
        name = MOTOR_NAMES.get(motor_id, f'motor_{motor_id}')
        packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE, 0)
        print(f"  {motor_id} ({name}): Torque DISABLED (shadow motor)")

    print()
    print("=" * 60)
    print("MONITORING TORQUE STATUS")
    print("Press Ctrl+C to stop")
    print("=" * 60)
    print()

    # Monitor loop
    start_time = time.time()
    check_count = 0
    torque_lost_motors = set()

    try:
        while True:
            check_count += 1
            elapsed = time.time() - start_time

            all_ok = True
            status_line = f"[{elapsed:6.1f}s] "

            for motor_id in primary_motors:
                torque_val, result, error = packet_handler.read1ByteTxRx(
                    port_handler, motor_id, ADDR_TORQUE_ENABLE)

                if torque_val != 1:
                    all_ok = False
                    name = MOTOR_NAMES.get(motor_id, f'motor_{motor_id}')

                    # Read hardware error
                    hw_err, _, _ = packet_handler.read1ByteTxRx(
                        port_handler, motor_id, ADDR_HARDWARE_ERROR)

                    error_bits = []
                    if hw_err:
                        if hw_err & 0x01: error_bits.append("InputVoltage")
                        if hw_err & 0x04: error_bits.append("Overheating")
                        if hw_err & 0x08: error_bits.append("MotorEncoder")
                        if hw_err & 0x10: error_bits.append("ElecShock")
                        if hw_err & 0x20: error_bits.append("Overload")

                    if motor_id not in torque_lost_motors:
                        torque_lost_motors.add(motor_id)
                        if error_bits:
                            print(f"\n*** TORQUE LOST on {motor_id} ({name}) at {elapsed:.1f}s - HW ERROR: {', '.join(error_bits)} ***")
                        else:
                            print(f"\n*** TORQUE LOST on {motor_id} ({name}) at {elapsed:.1f}s - NO HW ERROR (torque={torque_val}) ***")

                time.sleep(0.005)  # Small delay between reads

            if all_ok and check_count % 4 == 0:
                print(f"{status_line}All motors OK (torque enabled)")

            time.sleep(0.5)  # Check every 0.5 seconds

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")

    # Summary
    print()
    print("=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    elapsed = time.time() - start_time
    print(f"Test duration: {elapsed:.1f} seconds")
    print(f"Checks performed: {check_count}")

    if torque_lost_motors:
        print(f"Motors that lost torque: {list(torque_lost_motors)}")
    else:
        print("SUCCESS: All motors maintained torque throughout the test!")

    # Cleanup - disable torque
    print("\nDisabling torque on all motors...")
    for motor_id in discovered_motors:
        packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE, 0)
        time.sleep(0.01)

    port_handler.closePort()
    print("Done.")

    return 0 if not torque_lost_motors else 1


if __name__ == '__main__':
    sys.exit(main())
