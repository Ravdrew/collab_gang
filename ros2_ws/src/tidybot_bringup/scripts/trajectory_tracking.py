#!/usr/bin/env python3
"""
TidyBot2 Circular Trajectory Tracking - Student Template

This script demonstrates proportional control for tracking a circular trajectory.
The robot tracks: r(t) = (0.5*cos(t*2π/10), 0.5*sin(t*2π/10)) for t ∈ [0, 20s]

This is a circle with:
- Radius: 0.5 meters
- Period: 10 seconds (completes circle every 10s)
- Duration: 20 seconds (two full revolutions)

Topics used:
- /cmd_vel (geometry_msgs/Twist) - velocity commands [v, omega]
- /odom (nav_msgs/Odometry) - robot pose feedback

Usage:
    # Terminal 1: Start simulation
    ros2 launch tidybot_bringup sim.launch.py

    # Terminal 2: Run trajectory tracking with default gain (Kp=1.0)
    ros2 run tidybot_bringup trajectory_tracking.py

    # Or specify a custom gain:
    ros2 run tidybot_bringup trajectory_tracking.py --ros-args -p kp:=0.5
    ros2 run tidybot_bringup trajectory_tracking.py --ros-args -p kp:=2.0

Control Law Reference:
    The proportional control law with feedforward is:
        [vx_des, vy_des] = Kp * [error_x, error_y] + [ref_vx, ref_vy]

    For a differential drive robot, convert world velocities to robot commands:
        v = vx_des * cos(theta) + vy_des * sin(theta)
        omega = Kp * angle_to_desired_heading

STUDENT TODO: Implement the TrajectoryTracker class below.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time
import csv
import os
from datetime import datetime


class TrajectoryTracker(Node):
    """
    ROS2 node for tracking a circular trajectory.

    Students must implement:
    1. __init__: Set up publishers, subscribers, and timers
    2. odom_callback: Process odometry messages to get robot pose
    3. get_reference_trajectory: Compute desired position and velocity at time t
    4. control_loop: Implement the proportional controller
    """

    def __init__(self):
        super().__init__('trajectory_tracker')

        # Declare and get parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('save_data', True)
        self.declare_parameter('duration', 20.0)

        self.kp = self.get_parameter('kp').value
        self.save_data = self.get_parameter('save_data').value
        self.duration = self.get_parameter('duration').value

        # Trajectory parameters
        self.radius = 0.5   # meters
        self.period = 10.0  # seconds

        # Robot state (to be updated from odometry)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False

        # Data storage for plotting
        self.data = {
            'time': [], 'ref_x': [], 'ref_y': [],
            'actual_x': [], 'actual_y': [],
            'error_x': [], 'error_y': []
        }

        # Timing
        self.start_time = None
        self.running = True

        # =====================================================================
        # TODO: Create publisher for velocity commands
        # - Topic: '/cmd_vel'
        # - Message type: Twist
        # - Use: self.create_publisher(MessageType, 'topic_name', queue_size)
        # =====================================================================
        self.cmd_vel_pub = None  # TODO

        # =====================================================================
        # TODO: Create subscriber for odometry
        # - Topic: '/odom'
        # - Message type: Odometry
        # - Callback: self.odom_callback
        # - Use: self.create_subscription(MessageType, 'topic', callback, queue_size)
        # =====================================================================
        # TODO: Create odometry subscriber

        # =====================================================================
        # TODO: Create a timer for the control loop
        # - Period: 0.02 seconds (50 Hz)
        # - Callback: self.control_loop
        # - Use: self.create_timer(period, callback)
        # =====================================================================
        # TODO: Create control loop timer

        self.get_logger().info(f'Trajectory Tracker initialized with Kp={self.kp}')

    def odom_callback(self, msg: Odometry):
        """
        Callback for odometry messages.

        TODO: Extract the robot's current pose from the odometry message.
        - Position: msg.pose.pose.position.x, msg.pose.pose.position.y
        - Orientation: msg.pose.pose.orientation (quaternion: x, y, z, w)

        Convert quaternion to yaw angle using: odom_theta = 2 * atan2(qz, qw)

        IMPORTANT: The MuJoCo simulation has a coordinate offset. The odometry
        theta is π/2 ahead of the actual robot heading used for velocity control.
        When odom reports theta=π/2, the robot is actually facing +X (heading=0).
        So you need: actual_heading = odom_theta - π/2

        Update: self.current_x, self.current_y, self.current_theta
        Set self.odom_received = True after first message
        """
        # TODO: Implement this method
        pass

    def get_reference_trajectory(self, t):
        """
        Compute the reference trajectory at time t.

        The circular trajectory is:
            x(t) = radius * cos(omega * t)
            y(t) = radius * sin(omega * t)

        where omega = 2*pi / period

        The velocity (time derivative) is:
            vx(t) = -radius * omega * sin(omega * t)
            vy(t) =  radius * omega * cos(omega * t)

        Args:
            t: Time in seconds

        Returns:
            tuple: (ref_x, ref_y, ref_vx, ref_vy)
        """
        # TODO: Implement this method
        # Hint: omega = 2.0 * np.pi / self.period
        return 0.0, 0.0, 0.0, 0.0

    def control_loop(self):
        """
        Main control loop - called at 50 Hz.

        TODO: Implement proportional control with feedforward:

        1. Check if odometry has been received and if still running
        2. Initialize start_time on first call
        3. Compute elapsed time t
        4. Check if duration exceeded (stop if so)
        5. Get reference trajectory: ref_x, ref_y, ref_vx, ref_vy
        6. Compute position errors: error_x, error_y
        7. Compute desired world-frame velocities using control law:
           vx_des = Kp * error_x + ref_vx
           vy_des = Kp * error_y + ref_vy
        8. Convert to robot commands (v, omega):
           - v = vx_des * cos(theta) + vy_des * sin(theta)
           - Compute desired heading: atan2(vy_des, vx_des)
           - Compute heading error (normalize to [-pi, pi])
           - omega = 2 * Kp * heading_error
        9. Apply velocity limits (max_v=1.0, max_omega=2.0)
        10. Publish Twist message to cmd_vel_pub
        11. Store data for plotting
        """
        # TODO: Implement this method
        pass

    def stop_robot(self):
        """Send zero velocity to stop the robot."""
        if self.cmd_vel_pub is not None:
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Robot stopped.')

    def save_results(self):
        """Save trajectory data to CSV file for plotting."""
        if not self.save_data or len(self.data['time']) == 0:
            return

        data_dir = os.path.expanduser('~/tidybot_trajectory_data')
        os.makedirs(data_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'trajectory_kp{self.kp}_{timestamp}.csv'
        filepath = os.path.join(data_dir, filename)

        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'ref_x', 'ref_y', 'actual_x', 'actual_y',
                           'error_x', 'error_y'])
            for i in range(len(self.data['time'])):
                writer.writerow([
                    self.data['time'][i], self.data['ref_x'][i],
                    self.data['ref_y'][i], self.data['actual_x'][i],
                    self.data['actual_y'][i], self.data['error_x'][i],
                    self.data['error_y'][i]
                ])

        self.get_logger().info(f'Data saved to: {filepath}')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
