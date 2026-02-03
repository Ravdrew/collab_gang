#!/usr/bin/env python3
"""
TidyBot2 Trajectory Tracking

This script demonstrates proportional control for navigating to a specific x,y,yaw location.

This script should be housed in ros2_ws/install/tidybot_bringup/lib/tidybot_bringup


Topics used:
- /cmd_vel (geometry_msgs/Twist) - velocity commands [v, omega]
- /odom (nav_msgs/Odometry) - robot pose feedback
- /nav_goal (geometry_msgs/Pose2D) - position of goal object in world frame, given by perception team
- /end_nav_pose (geometry_msgs/Pose2D) - position of robot state in world frame after reaching goal pose, given to manipulator team

Usage:
    # Terminal 1: Start simulation
    ros2 launch tidybot_bringup sim.launch.py

    # Terminal 2: Run trajectory tracking with default gain (Kp=1.0)
    ros2 run tidybot_bringup our_trajectory_tracking.py

    # Or specify a custom gain:
    ros2 run tidybot_bringup our_trajectory_tracking.py --ros-args -p kp:=0.5
    ros2 run tidybot_bringup our_trajectory_tracking.py --ros-args -p kp:=2.0

    # Terminal 3: SPECIFY A GOAL POSE:
    ros2 topic pub /nav_goal geometry_msgs/Pose2D \
    "{x: 1.0, y: 0.0, theta: 0.0}"

this control law is outdated for now:
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
from geometry_msgs.msg import Pose2D # for (x, y, orientation)
from nav_msgs.msg import Odometry
import numpy as np
import time
import csv
import os
from datetime import datetime
import math
from rclpy.time import Time


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
        super().__init__('our_trajectory_tracker')

        # Declare and get parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('save_data', True)
        # self.declare_parameter('duration', 20.0)

        self.kp = self.get_parameter('kp').value
        self.save_data = self.get_parameter('save_data').value
        # self.duration = self.get_parameter('duration').value

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
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # TODO

        # =====================================================================
        # TODO: Create subscriber for odometry
        # - Topic: '/odom'
        # - Message type: Odometry
        # - Callback: self.odom_callback
        # - Use: self.create_subscription(MessageType, 'topic', callback, queue_size)
        # =====================================================================
        # TODO: Create odometry subscriber
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # ===========================
        # SUBSCRIBER TO NAVIGATION GOAL
        self.goal_sub = self.create_subscription(Pose2D, '/nav_goal', self.goal_callback, 10)

        # ===========================
        # PUBLISHER FOR FINAL ROBOT POSE
        self.end_pose_pub = self.create_publisher(Pose2D, '/end_nav_pose', 10)

        # ===========================
        # GOAL AND TOLERANCE INITIALIZATION
        self.goal = None
        self.goal_active = False
        self.goal_tolerance = 0.05  # meters
        self.yaw_tolerance = 0.05 # radians
        
        # =====================================================================
        # TODO: Create a timer for the control loop
        # - Period: 0.02 seconds (50 Hz)
        # - Callback: self.control_loop
        # - Use: self.create_timer(period, callback)
        # =====================================================================
        # TODO: Create control loop timer
        self.create_timer(0.02, self.control_loop)
        self.get_logger().info(f'Trajectory Tracker initialized with Kp={self.kp}')

    def goal_callback(self, msg: Pose2D):
        """
        Callback for robot goal pose. Shows user what goal the robot is moving to.
        """
        self.goal = msg 
        self.goal_activ = True
        self.get_logger().info(f"Received goal: x={msg.x:.2f}, y={msg.y:.2f}, yaw={msg.theta:.2f}")

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
        # get positional information
        self.current_x = msg.pose.pose.position.x 
        self.current_y = msg.pose.pose.position.y 
        q = msg.pose.pose.orientation 
        odom_theta = 2.0 * math.atan2(q.z, q.w)
        # correct for mujoco heading offset
        self.current_theta = odom_theta - math.pi/2
        self.odom_received = True

    def control_loop(self):
        """
        Main control loop - called at 50 Hz.
        """
        if not self.odom_received or not self.goal_active:
            return

        dx = self.goal.x - self.current_x
        dy = self.goal.y - self.current_y
        distance = np.hypot(dx, dy)

        # Desired heading to goal
        desired_theta = np.arctan2(dy, dx)
        heading_error = desired_theta - self.current_theta
        heading_error = (heading_error + np.pi) % (2*np.pi) - np.pi

        # Check arrival
        yaw_error = (self.goal.theta - self.current_theta + np.pi) % (2*np.pi) - np.pi

        if distance < self.goal_tolerance and abs(yaw_error) < self.yaw_tolerance:
            self.stop_robot()
            self.publish_final_pose()
            self.goal_active = False
            self.get_logger().info("Goal reached.")
            return

        # Proportional control
        v = self.kp * distance
        omega = 2.0 * self.kp * heading_error

        # Limits
        v = np.clip(v, 0.0, 0.5)
        omega = np.clip(omega, -2.0, 2.0)

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_vel_pub.publish(cmd)

    def publish_final_pose(self):
        pose = Pose2D()
        pose.x = self.current_x
        pose.y = self.current_y
        pose.theta = self.current_theta
        self.end_pose_pub.publish(pose)

    def stop_robot(self):
        """Send zero velocity to stop the robot."""
        if self.cmd_vel_pub is not None:
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Robot stopped.')


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
