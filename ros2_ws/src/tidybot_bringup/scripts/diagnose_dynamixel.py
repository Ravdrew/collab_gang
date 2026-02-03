#!/usr/bin/env python3
"""
Diagnostic script to scan for Dynamixel motors on U2D2 ports.
This bypasses ROS2 and tests direct communication.
"""

import sys

try:
    from dynamixel_sdk import PortHandler, PacketHandler
except ImportError:
    print("ERROR: dynamixel_sdk not found. Install with: pip install dynamixel-sdk")
    sys.exit(1)

# Protocol version
PROTOCOL_VERSION = 2.0

# Common baud rates to try
BAUD_RATES = [1000000, 57600, 115200, 2000000, 3000000, 4000000]

# Motor ID range to scan
ID_RANGE = range(1, 30)  # Scan IDs 1-29


def scan_port(port_name, baud_rate):
    """Scan a port at a specific baud rate for motors."""
    port_handler = PortHandler(port_name)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort():
        print(f"  Failed to open {port_name}")
        return []

    if not port_handler.setBaudRate(baud_rate):
        print(f"  Failed to set baud rate {baud_rate}")
        port_handler.closePort()
        return []

    found_motors = []

    for motor_id in ID_RANGE:
        # Try to ping the motor
        model_number, result, error = packet_handler.ping(port_handler, motor_id)

        if result == 0:  # COMM_SUCCESS
            found_motors.append((motor_id, model_number))

    port_handler.closePort()
    return found_motors


def main():
    print("=" * 60)
    print("Dynamixel Motor Diagnostic Tool")
    print("=" * 60)
    print()

    # Check available ports
    ports = ['/dev/ttyUSB0', '/dev/ttyUSB1']

    import os
    available_ports = [p for p in ports if os.path.exists(p)]

    if not available_ports:
        print("ERROR: No USB serial ports found!")
        print("Please check that U2D2 adapters are connected.")
        return 1

    print(f"Found ports: {available_ports}")
    print()

    # Scan each port
    for port in available_ports:
        print(f"Scanning {port}...")
        print("-" * 40)

        found_any = False

        for baud in BAUD_RATES:
            motors = scan_port(port, baud)
            if motors:
                found_any = True
                print(f"  Baud {baud}: Found {len(motors)} motor(s)")
                for motor_id, model in motors:
                    print(f"    - ID {motor_id}: Model {model}")

        if not found_any:
            print("  No motors found at any baud rate!")
            print("  Possible causes:")
            print("    1. Arm is not powered on (check power supply)")
            print("    2. U2D2 not connected to arm (check 3-pin cable)")
            print("    3. U2D2 cable issue")

        print()

    print("=" * 60)
    print("Diagnostic complete")
    print()
    print("Expected configuration:")
    print("  /dev/ttyUSB0: Left arm motors (IDs 11-19)")
    print("  /dev/ttyUSB1: Right arm (IDs 1-9) + Pan-tilt (IDs 21-22)")
    print()
    print("If no motors were found, check:")
    print("  1. Are both arms POWERED ON?")
    print("  2. Are the 3-pin Dynamixel cables connected from U2D2 to arm?")
    print("  3. Is the power supply voltage correct (12V)?")

    return 0


if __name__ == '__main__':
    sys.exit(main())
